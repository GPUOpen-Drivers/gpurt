/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2025 Advanced Micro Devices, Inc. All Rights Reserved.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 **********************************************************************************************************************/
#ifndef _REFIT_ORIENTED_BOUNDSTOPLEVEL3_1_HLSL
#define _REFIT_ORIENTED_BOUNDSTOPLEVEL3_1_HLSL

#if NO_SHADER_ENTRYPOINT == 0

#define BUILD_THREADGROUP_SIZE 32

#include "../shadersClean/common/ShaderDefs.hlsli"

#define GC_DSTBUFFER
#define GC_DSTMETADATA
#define GC_SCRATCHBUFFER
#include "../shadersClean/build/BuildRootSignature.hlsli"

template<typename T>
T LoadInstanceDescBuffer(uint offset)
{
    return InstanceDescBuffer.Load<T>(offset);
}
#include "IndirectArgBufferUtils.hlsl"
#include "IntersectCommon.hlsl"
#include "BuildCommonScratch.hlsl"
#include "CompactCommon.hlsl"

#endif

#if GPURT_BUILD_RTIP3_1

#include "rtip3_1.hlsli"
#include "OrientedBoundingBoxes.hlsl"
#include "QBVH8Common.hlsl"
#include "../shadersClean/common/Extensions.hlsli"
#include "../shadersClean/common/LaneGroup.hlsli"

#define OBB_SA_FACTOR_THRESHOLD 1.12
#define LANE_GROUP_SIZE 8

//=====================================================================================================================
// Calculate a K-DOP and an OBB for a BVH8 primitive node.
BoundingBox ComputeInstanceBoundsFromKdop(
    in  uint              nodeAddr,
    in  uint              obbMatrixIndex,
    in AccelStructOffsets offsets)
{
    const float3x3 obbMatrix = OBBIndexToMatrix(obbMatrixIndex);

    // Get current child node instance desc:
    const uint sidebandOffset     = ComputeInstanceSidebandOffset(nodeAddr, offsets.leafNodes, offsets.geometryInfo);
    InstanceSidebandData sideband = DstBuffer.Load<InstanceSidebandData>(sidebandOffset);
    const uint64_t       blasBaseAddr =
        ExtractInstanceAddr(DstBuffer.Load<uint64_t>(nodeAddr + RTIP3_INSTANCE_NODE_CHILD_BASE_PTR_OFFSET));
    const uint64_t kdopAddress = blasBaseAddr + ACCEL_STRUCT_METADATA_KDOP_OFFSET;

    /**
    * Our OBB Matrix has 6 facets that its basis vectors map to.
    * We must find those 6 facets and each of their K-DOP extents.
    */
    BoundingBox obbBbox;
    float2      boundsX = ComputeOBBFromKdopAddr(kdopAddress, obbMatrix[0]);
    float2      boundsY = ComputeOBBFromKdopAddr(kdopAddress, obbMatrix[1]);
    float2      boundsZ = ComputeOBBFromKdopAddr(kdopAddress, obbMatrix[2]);
    float3      p0      = float3(boundsX.x, boundsY.x, boundsZ.x);
    float3      p1      = float3(boundsX.y, boundsY.y, boundsZ.y);

    obbBbox.min = min(p0, p1);
    obbBbox.max = max(p0, p1);

    // Recompose the instance transform applying scaling based on difference between rotation and descrete rotation.
    const float3x3 rotatedInstanceTransform = float3x3(sideband.objectToWorld[0][0],
                                                       sideband.objectToWorld[0][1],
                                                       sideband.objectToWorld[0][2],
                                                       sideband.objectToWorld[1][0],
                                                       sideband.objectToWorld[1][1],
                                                       sideband.objectToWorld[1][2],
                                                       sideband.objectToWorld[2][0],
                                                       sideband.objectToWorld[2][1],
                                                       sideband.objectToWorld[2][2]) * transpose(obbMatrix);

    // Variant of TransformBoundingBox that avoids multiple transform copies.
    float3 center  = (obbBbox.max + obbBbox.min) * 0.5;
    float3 extents = (obbBbox.max - center);

    float3 translation = mul(obbMatrix, float3(sideband.objectToWorld[0][3], sideband.objectToWorld[2][3], sideband.objectToWorld[3][3]));
    float4x4 transform;
    transform[0] = float4(rotatedInstanceTransform[0].xyz, translation.x);
    transform[1] = float4(rotatedInstanceTransform[1].xyz, translation.y);
    transform[2] = float4(rotatedInstanceTransform[2].xyz, translation.z);
    transform[3] = float4(0, 0, 0, 1);

    // transform center
    float3 transformedCenter = mul(transform, float4(center, 1.0)).xyz;

    // transform extents (take absolute)
    float3x3 absMat             = float3x3(abs(transform[0].xyz),
                                           abs(transform[1].xyz),
                                           abs(transform[2].xyz));
    float3   transformedExtents = mul(absMat, extents);

    // convert back to min/max box representation
    float3 min = transformedCenter - transformedExtents;
    float3 max = transformedCenter + transformedExtents;

    BoundingBox transformedBbox;

    transformedBbox.min = min;
    transformedBbox.max = max;

    return transformedBbox;
}

//=====================================================================================================================
// This only computes the best fit OBB for nodes at the leaf node level.
void RefitOrientedBoundsTopLevelImpl(
    uint globalId,
    uint localId,
    uint totalNodes)
{
    if (Settings.topLevelBuild == true)
    {
        return;
    }

    LaneGroup laneGroup;
    laneGroup.Alloc(LANE_GROUP_SIZE);

    // We start with box nodes whose leaves are primitive structures.
    // Every 8 threads increments our current node index and
    // every 8 threads is assigned one of the children of that node.
    uint       obbStackIndex  = globalId / LANE_GROUP_SIZE;
    const uint laneGroupIndex = localId / LANE_GROUP_SIZE;

    if (obbStackIndex >= totalNodes)
    {
        return;
    }

    // This algorithm begins by computing Kdops from the leaves starting from internal nodes with all leaf references.
    // These Kdops are then propagated upwards towards the root.
    uint curNodeIndex = ScratchGlobal.Load(ShaderConstants.offsets.obbRefitStack + (obbStackIndex * sizeof(uint)));

    // Load acceleration structure header
    const AccelStructOffsets offsets        = ShaderConstants.header.offsets;

    GroupMemoryBarrier();

    const uint curNodeKdopOfs = ShaderConstants.offsets.qbvhKdops + (curNodeIndex * sizeof(ObbKdop));
    const uint nodeOffset     = offsets.internalNodes + (curNodeIndex * QBVH8::NodeSizeInBytes);

    // Fetch internal node data on first lane in lane group
    uint baseInternalNodeOffset = 0;
    uint baseLeafNodeOffset = 0;
    uint exponentsChildIndexAndChildCount = 0;
    uint parentIndex = 0;
    uint obbFlags = 0;
    float aabbSASum = 0.0f;

    if (laneGroup.IsFirstLane())
    {
        baseInternalNodeOffset = DstBuffer.Load(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_INTERNAL_NODE_BASE_OFFSET);
        baseLeafNodeOffset = DstBuffer.Load(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_LEAF_NODE_BASE_OFFSET);
        const uint parentAddr = ExtractNodePointerOffset3_1(DstBuffer.Load(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_PARENT_POINTER));
        parentIndex = (parentAddr - offsets.internalNodes) / QBVH8::NodeSizeInBytes;
        exponentsChildIndexAndChildCount = DstBuffer.Load(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT);

        const uint flagsOffset = ShaderConstants.offsets.qbvhObbFlags + (curNodeIndex * OBB_TABLE_ENTRY_SIZE);
        obbFlags = ScratchGlobal.Load<uint>(flagsOffset);
        aabbSASum = ScratchGlobal.Load<float>(flagsOffset + OBB_TABLE_SA_OFFSET);
    }

    baseInternalNodeOffset = laneGroup.ReadFirstLane(baseInternalNodeOffset) << 3u;
    baseLeafNodeOffset = laneGroup.ReadFirstLane(baseLeafNodeOffset) << 3u;
    parentIndex = laneGroup.ReadFirstLane(parentIndex);
    exponentsChildIndexAndChildCount = laneGroup.ReadFirstLane(exponentsChildIndexAndChildCount);
    obbFlags = laneGroup.ReadFirstLane(obbFlags);
    aabbSASum = laneGroup.ReadFirstLane(aabbSASum);

    // Fetch child info on each lane in the group
    ChildInfo childInfo = DstBuffer.Load<ChildInfo>(nodeOffset + GetChildInfoOffset(laneGroup.laneIndex));
    const bool isChildValid = childInfo.Valid();

    // Pack packet offset in upper bits for leaf nodes and lower bits for internal nodes.
    const uint shift = 16;
    const uint packedLength = (childInfo.NodeRangeLength() << shift);

    // The following can be done more efficiently using cluster_prefix_sum
    //
    // const uint packedOffset = AmdExtD3DShaderIntrinsics_WaveClusterPrefixSum(packedLength, SG_CLUSTER_SIZE_8);
    //

    // Compute the per-lane prefix sum and subtract the lead lane sum from all to obtain the lane group offsets
    const uint packedOffset = laneGroup.ExclusivePrefixSum(packedLength);

    // Compute absolute offset in memory from packed offset per lane
    uint childNodeOffset = baseLeafNodeOffset;
    childNodeOffset += (((packedOffset >> shift) & 0xffff) * 128u);

    const uint childIndex = (childNodeOffset - offsets.internalNodes) / QBVH8::NodeSizeInBytes;
    const uint bestObbLaneIndex = (obbFlags >> 16u) & 0xFu;
    const uint bestObbMatrixIdxLeaf = (obbFlags >> 24u);

    uint bestObbMatrixIndex = bestObbMatrixIdxLeaf;
    bestObbMatrixIndex = laneGroup.Broadcast(bestObbMatrixIndex, bestObbLaneIndex);

    // Compute candidate OBB for valid children.
    BoundingBox candidateObb = InvalidBoundingBox;
    if (isChildValid)
    {
        candidateObb = ComputeInstanceBoundsFromKdop(childNodeOffset, bestObbMatrixIndex, offsets);

        // Add 6 ULP scaled to the longest extent of padding to OBB extents for watertightness:
        const float3 extents = candidateObb.max - candidateObb.min;
        const float longestExtent = abs(laneGroup.Max(max(extents.x, max(extents.y, extents.z))));
        const float3 epsilonPadding = 7.15254e-7 * float3(longestExtent, longestExtent, longestExtent);
        candidateObb.min -= epsilonPadding;
        candidateObb.max += epsilonPadding;
    }

    // Verify if OBB fits better than original AABBs.
    if (bestObbMatrixIndex != INVALID_OBB)
    {
        const float obbSA = ComputeBoxSurfaceArea(candidateObb);
        const float obbSASum = laneGroup.Sum(obbSA);

        if (obbSASum >= (aabbSASum * OBB_SA_FACTOR_THRESHOLD))
        {
            bestObbMatrixIndex = INVALID_OBB;
        }
    }

    const bool shouldWriteObbs = (Settings.enableOrientedBoundingBoxes & bit(TOP_LEVEL));
    if ((bestObbMatrixIndex != INVALID_OBB) && shouldWriteObbs)
    {
        const float3 minOfMins = laneGroup.Min(candidateObb.min);
        const float3 maxOfMaxs = laneGroup.Max(candidateObb.max);

        const uint3  exponents    = ComputeCommonExponent(minOfMins, maxOfMaxs, 12);
        const float3 rcpExponents = ComputeFastExpReciprocal(exponents, 12);

        // Update OBB matrix index, origin and exponents in parent box node.
        if (laneGroup.IsFirstLane())
        {
            DstBuffer.Store<uint16_t>(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_OBB_MATRIX_INDEX,
                                        uint16_t(bestObbMatrixIndex));
            DstBuffer.Store<float3>(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_ORIGIN,
                                    minOfMins);

            exponentsChildIndexAndChildCount = bitFieldInsert(exponentsChildIndexAndChildCount, 0, 8, exponents.x);
            exponentsChildIndexAndChildCount = bitFieldInsert(exponentsChildIndexAndChildCount, 8, 8, exponents.y);
            exponentsChildIndexAndChildCount = bitFieldInsert(exponentsChildIndexAndChildCount, 16, 8, exponents.z);

            DstBuffer.Store<uint>(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT,
                                    exponentsChildIndexAndChildCount);
        }

        // Update child bounds for valid children.
        if (isChildValid)
        {
            const UintBoundingBox quantBounds = ComputeQuantizedBounds(candidateObb, minOfMins, rcpExponents, 12);
            childInfo.SetMin(quantBounds.min);
            childInfo.SetMax(quantBounds.max);

            DstBuffer.Store<ChildInfo>(nodeOffset + GetChildInfoOffset(laneGroup.laneIndex), childInfo);
        }
    }
}
#endif

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
// Main Function : RefitOrientedBoundsTopLevel
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void RefitOrientedBoundsTopLevel(
    in uint globalId : SV_DispatchThreadID,
    in uint localId  : SV_GroupThreadID)
{
#if GPURT_BUILD_RTIP3_1
    const uint obbRefitBatchCount = ScratchGlobal.Load(
        ShaderConstants.offsets.obbRefitStackPtrs + STACK_PTRS_OBB_REFIT_BATCH_COUNT);

    RefitOrientedBoundsTopLevelImpl(globalId, localId, obbRefitBatchCount);
#endif
}
#endif

#endif
