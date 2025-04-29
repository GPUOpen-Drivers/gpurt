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
#ifndef _REFIT_ORIENTED_BOUNDS3_1_HLSL
#define _REFIT_ORIENTED_BOUNDS3_1_HLSL

#if NO_SHADER_ENTRYPOINT == 0

#define BUILD_THREADGROUP_SIZE 32

#include "../shadersClean/common/ShaderDefs.hlsli"

#define GC_DSTBUFFER
#define GC_DSTMETADATA
#define GC_SCRATCHBUFFER
#include "../shadersClean/build/BuildRootSignature.hlsli"

#include "../shadersClean/common/Common.hlsli"
#include "IntersectCommon.hlsl"
#include "../shadersClean/build/BuildCommonScratch.hlsli"
#include "CompactCommon.hlsl"

// 64-bytes per thread. Each lane group of 8 threads shares a group shared block of 512-bytes
#define MAX_ELEMENT_PER_THREAD 16
groupshared uint SharedMem[BUILD_THREADGROUP_SIZE * MAX_ELEMENT_PER_THREAD];
uint GetSharedMem(uint index)
{
    return SharedMem[index];
}
void SetSharedMem(uint index, uint value)
{
    SharedMem[index] = value;
}

#endif

#if GPURT_BUILD_RTIP3_1
#include "rtip3_1.hlsli"
#include "OrientedBoundingBoxes.hlsl"
#include "QBVH8Common.hlsl"
#include "../shadersClean/common/Extensions.hlsli"
#include "../shadersClean/common/LaneGroup.hlsli"

#define OBB_SA_FACTOR_THRESHOLD 1.12
#define LANE_GROUP_SIZE 8
#define LANE_GROUP_STRIDE (LANE_GROUP_SIZE * MAX_ELEMENT_PER_THREAD)
#define LANE_GROUP_ELEMENT_INDEX_KDOP_MIN 0
#define LANE_GROUP_ELEMENT_INDEX_KDOP_MAX 1

//=====================================================================================================================
// Group shared memory layout
//
// element_stride: 128 DW
//
// min0 : g0, g1, g2, g3, g4, g5, g6, g7 min1: g0, g1, g2, g3, g4, g5, g6, g7,
// min2 : g0, g1, g2, g3, g4, g5, g6, g7 min3: g0, g1, g2, g3, g4, g5, g6, g7,
// ...
// min12: g0, g1, g2, g3, g4, g5, g6, g7 unused, ....
// ...
// max0 : g0, g1, g2, g3, g4, g5, g6, g7 max1: g0, g1, g2, g3, g4, g5, g6, g7,
// max2 : g0, g1, g2, g3, g4, g5, g6, g7 max3: g0, g1, g2, g3, g4, g5, g6, g7,
// ...
// max12: g0, g1, g2, g3, g4, g5, g6, g7 unused, ....
// ...
//
//=====================================================================================================================
uint GetLdsIndexKDopMin(uint laneGroupIndex, uint planeIndex)
{
    const uint elementOffset = (LANE_GROUP_ELEMENT_INDEX_KDOP_MIN * LANE_GROUP_STRIDE);
    return elementOffset + (planeIndex * LANE_GROUP_SIZE) + laneGroupIndex;
}

//=====================================================================================================================
uint GetLdsIndexKDopMax(uint laneGroupIndex, uint planeIndex)
{
    const uint elementOffset = (LANE_GROUP_ELEMENT_INDEX_KDOP_MAX * LANE_GROUP_STRIDE);
    return elementOffset + (planeIndex * LANE_GROUP_SIZE) + laneGroupIndex;
}

//=====================================================================================================================
void UpdateKDopMin(uint laneGroupIndex, uint planeIndex, float value)
{
    InterlockedMin(SharedMem[GetLdsIndexKDopMin(laneGroupIndex, planeIndex)], FloatToUint(value));
}

//=====================================================================================================================
void UpdateKDopMax(uint laneGroupIndex, uint planeIndex, float value)
{
    InterlockedMax(SharedMem[GetLdsIndexKDopMax(laneGroupIndex, planeIndex)], FloatToUint(value));
}

//=====================================================================================================================
void WriteKDopMin(uint laneGroupIndex, uint planeIndex, float value)
{
    SharedMem[GetLdsIndexKDopMin(laneGroupIndex, planeIndex)] = FloatToUint(value);
}

//=====================================================================================================================
void WriteKDopMax(uint laneGroupIndex, uint planeIndex, float value)
{
    SharedMem[GetLdsIndexKDopMax(laneGroupIndex, planeIndex)] = FloatToUint(value);
}

//=====================================================================================================================
float ReadKDopMin(uint laneGroupIndex, uint planeIndex)
{
    return UintToFloat(SharedMem[GetLdsIndexKDopMin(laneGroupIndex, planeIndex)]);
}

//=====================================================================================================================
float ReadKDopMax(uint laneGroupIndex, uint planeIndex)
{
    return UintToFloat(SharedMem[GetLdsIndexKDopMax(laneGroupIndex, planeIndex)]);
}

//=====================================================================================================================
BoundingBox ComputeTriangleKDop(
    const uint         laneGroupIndex,
    const TriangleData tri,
    const float3x3     obbMatrix)
{
    for (uint i = 0; i < KDOP_PLANE_COUNT; i++)
    {
        const float2 extents = ComputeTriangleKdopExtents(i, tri.v0, tri.v1, tri.v2);
        UpdateKDopMin(laneGroupIndex, i, extents.x);
        UpdateKDopMax(laneGroupIndex, i, extents.y);
    }

    return ComputeOBBFromTri(tri.v0, tri.v1, tri.v2, obbMatrix);
}

//=====================================================================================================================
inline uint ReadPackedBitsFromNode(
    uint nodeOffset,
    uint startBitOffset,
    uint length)
{
    // DWORD aligned 64 bit loads are technically not allowed, but it works on our hardware.
    const uint startByteOffset = sizeof(uint) * (startBitOffset / 32);
    const uint64_t packedData = DstBuffer.Load<uint64_t>(nodeOffset + startByteOffset);

    const uint shiftBitsInDW = (startBitOffset % 32u);
    const uint64_t packedDataRSH = packedData >> shiftBitsInDW;

    const uint64_t dataOut64 = (packedDataRSH & ((uint64_t(1) << length) - uint64_t(1)));
    return uint(dataOut64 & 0xFFFFFFFF);
}

//=====================================================================================================================
inline TrianglePairDesc ReadTrianglePairDescFromNode(
    uint nodeOffset,
    uint triPairIndex)
{
    const uint startOffset = 1024 - TRI_PAIR_DESC_SIZE * (triPairIndex + 1);
    const uint packedData = ReadPackedBitsFromNode(nodeOffset, startOffset, TRI_PAIR_DESC_SIZE);

    TrianglePairDesc triDesc;
    // There is dxc bug that the generated spv use OpAccessChain point as OpFunctionCall Parameter,
    // which is illegal per spv spec.
    triDesc.triDesc = packedData;

    return triDesc;
}

//=====================================================================================================================
// A local version of the primitive structure header using bitfields
struct PrimStructHeader
{
    uint PayloadXLength()     { return payloadXLengthMinusOne + 1; }
    uint PayloadYLength()     { return payloadYLengthMinusOne + 1; }
    uint PayloadZLength()     { return payloadZLengthMinusOne + 1; }
    uint TrailingZeroLength() { return trailingZeroLength;         }
    uint TrianglePairCount()  { return triPairCountMinusOne + 1;   }

    uint PrefixXLength() { return 32 - TrailingZeroLength() - PayloadXLength(); };
    uint PrefixYLength() { return 32 - TrailingZeroLength() - PayloadYLength(); };
    uint PrefixZLength() { return 32 - TrailingZeroLength() - PayloadZLength(); };

    uint VertexPrefixesLength()
    {
        return PrefixXLength() + PrefixYLength() + PrefixZLength();
    }

    uint VertexPayloadLength()
    {
        return PayloadXLength() + PayloadYLength() + PayloadZLength();
    }

    uint32_t payloadXLengthMinusOne : 5;
    uint32_t payloadYLengthMinusOne : 5;
    uint32_t payloadZLengthMinusOne : 5;
    uint32_t trailingZeroLength     : 5;
    uint32_t geoIdAnchorSizeDivTwo  : 4;
    uint32_t geoIdPayloadSizeDivTwo : 4;
    uint32_t triPairCountMinusOne   : 3;
    uint32_t floatOrUnorm16         : 1;

    // Remaining bits of the header are not required for vertex fetch
    // uint32_t primIdAnchorSize     : 5;
    // uint32_t primIdPayloadSize    : 5;
    // uint32_t indexSectionMidpoint : 10;
    // uint32_t reserved             : 12;
};

//=====================================================================================================================
inline float3 ReadVertexFromNode(
    uint             nodeOffset,
    PrimStructHeader header,
    uint3            prefixes,
    uint             index)
{
    uint x = 0;
    uint y = 0;
    uint z = 0;

    const uint sectionOffset = TRI_STRUCT_HEADER_SIZE_BITS + header.VertexPrefixesLength();
    uint offset = sectionOffset + (index)*header.VertexPayloadLength();

    uint payloadXLength = header.PayloadXLength();
    uint payloadYLength = header.PayloadYLength();
    uint payloadZLength = header.PayloadZLength();
    uint trailingZeroLength = header.TrailingZeroLength();

    x = prefixes.x | (ReadPackedBitsFromNode(nodeOffset, offset, payloadXLength) << trailingZeroLength);
    offset += payloadXLength;
    y = prefixes.y | (ReadPackedBitsFromNode(nodeOffset, offset, payloadYLength) << trailingZeroLength);
    offset += payloadYLength;
    z = prefixes.z | (ReadPackedBitsFromNode(nodeOffset, offset, payloadZLength) << trailingZeroLength);

    float xf = asfloat(x);
    float yf = asfloat(y);
    float zf = asfloat(z);

    return float3(xf, yf, zf);
}

//=====================================================================================================================
inline TriangleData ReadTriangleVerticesFromNode(
    uint             nodeOffset,
    PrimStructHeader header,
    uint3            prefixes,
    TrianglePairDesc pairDesc,
    uint             triIndex)
{
    TriangleData tri = (TriangleData)0;
    if (triIndex == 0)
    {
        tri.v0 = ReadVertexFromNode(nodeOffset, header, prefixes, pairDesc.Tri0V0());
        tri.v1 = ReadVertexFromNode(nodeOffset, header, prefixes, pairDesc.Tri0V1());
        tri.v2 = ReadVertexFromNode(nodeOffset, header, prefixes, pairDesc.Tri0V2());
    }
    else
    {
        tri.v0 = ReadVertexFromNode(nodeOffset, header, prefixes, pairDesc.Tri1V0());
        tri.v1 = ReadVertexFromNode(nodeOffset, header, prefixes, pairDesc.Tri1V1());
        tri.v2 = ReadVertexFromNode(nodeOffset, header, prefixes, pairDesc.Tri1V2());
    }

    return tri;
}

//=====================================================================================================================
inline uint3 ReadVertexPrefixesFromNode(
    uint             nodeOffset,
    PrimStructHeader header)
{
    uint prefixXLength = header.PrefixXLength();
    uint prefixYLength = header.PrefixYLength();
    uint prefixZLength = header.PrefixZLength();

    const uint bitOffsetX = TRI_STRUCT_HEADER_SIZE_BITS;
    const uint bitOffsetY = bitOffsetX + prefixXLength;
    const uint bitOffsetZ = bitOffsetY + prefixYLength;

    const uint prefixX = (prefixXLength > 0) ? ReadPackedBitsFromNode(nodeOffset, bitOffsetX, prefixXLength) : 0;
    const uint prefixY = (prefixYLength > 0) ? ReadPackedBitsFromNode(nodeOffset, bitOffsetY, prefixYLength) : 0;
    const uint prefixZ = (prefixZLength > 0) ? ReadPackedBitsFromNode(nodeOffset, bitOffsetZ, prefixZLength) : 0;

    uint3 prefixes = { prefixX, prefixY, prefixZ };

    prefixes.x <<= 32 - prefixXLength;
    prefixes.y <<= 32 - prefixYLength;
    prefixes.z <<= 32 - prefixZLength;

    return prefixes;
}

//=====================================================================================================================
// Calculate a K-DOP and an OBB for a BVH8 primitive node.
BoundingBox ComputePrimNodeKDop(
    in  uint        laneGroupIndex,
    in  uint        nodeType,
    in  uint        nodeAddr,
    in  uint        obbMatrixIndex)
{
    const float3x3 obbMatrix = OBBIndexToMatrix(obbMatrixIndex);

    BoundingBox obb = (BoundingBox)0;
    obb.min = float3(+FLT_MAX, +FLT_MAX, +FLT_MAX);
    obb.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    uint currPairIdx = GetPairIndexFromNodeType(nodeType);
    uint currNodeAddr = nodeAddr;
    bool endOfRange = true;

    do
    {
        // Prefetch header data and triangle pair descriptor for reading packed data below
        const PrimStructHeader header = DstBuffer.Load<PrimStructHeader>(currNodeAddr);
        const TrianglePairDesc pairDesc = ReadTrianglePairDescFromNode(currNodeAddr, currPairIdx);

        // Prefetch vertex prefixes for unpacking vertex data
        const uint3 prefixes = ReadVertexPrefixesFromNode(currNodeAddr, header);

        const TriangleData tri0          = ReadTriangleVerticesFromNode(currNodeAddr, header, prefixes, pairDesc, 0);
        const BoundingBox  tri0ObbBounds = ComputeTriangleKDop(laneGroupIndex, tri0, obbMatrix);

        obb.min = min(obb.min, tri0ObbBounds.min);
        obb.max = max(obb.max, tri0ObbBounds.max);

        if (pairDesc.Tri1Valid())
        {
            const TriangleData tri1 = ReadTriangleVerticesFromNode(currNodeAddr, header, prefixes, pairDesc, 1);
            const BoundingBox  tri1ObbBounds = ComputeTriangleKDop(laneGroupIndex, tri1, obbMatrix);

            obb.min = min(obb.min, tri1ObbBounds.min);
            obb.max = max(obb.max, tri1ObbBounds.max);
        }

        if (Settings.maxPrimRangeSize > 2)
        {
            // Advance to the next triangle pair
            endOfRange = (pairDesc.PrimRangeStopBit() != 0);

            if (endOfRange == false)
            {
                // Primitive range continues
                const uint trianglePairCount = header.TrianglePairCount();
                if (currPairIdx == (trianglePairCount - 1))
                {
                    // this is the last triangle pair in node, continue in next node
                    currNodeAddr += PRIMITIVE_STRUCT_SIZE_IN_BYTE;
                    currPairIdx = 0;
                }
                else
                {
                    // Continue to next triangle pair in node
                    currPairIdx++;
                }
            }
        }
    } while (endOfRange == false);

    return obb;
}

//=====================================================================================================================
// Convert a given K-DOP to an OBB for a BVH8 box node. This is done by computing 6
// apex points in run-time and projecting them to the best aligned OBB.
BoundingBox ComputeBoxNodeOBB(
    in uint currentObbKdopOffset,
    in uint obbMatrixIndex)
{
    BoundingBox obbBbox;
    // Get 3 closest directions to each OBB extent.
    const float3x3 obbMatrix = OBBIndexToMatrix(obbMatrixIndex);

    /**
    * Our OBB Matrix has 6 facets that its basis vectors map to.
    * We must find those 6 facets and each of their K-DOP extents.
    */
    float2 boundsX = ComputeOBBBounds(currentObbKdopOffset, obbMatrix[0]);
    float2 boundsY = ComputeOBBBounds(currentObbKdopOffset, obbMatrix[1]);
    float2 boundsZ = ComputeOBBBounds(currentObbKdopOffset, obbMatrix[2]);
    float3 p0 = float3(boundsX.x, boundsY.x, boundsZ.x);
    float3 p1 = float3(boundsX.y, boundsY.y, boundsZ.y);

    obbBbox.min = min(p0, p1);
    obbBbox.max = max(p0, p1);

    return obbBbox;
}

//=====================================================================================================================
// This version of Post Proc Obbs assigns 8 threads per node and uses LDS to store the current node's
// Kdop, 8 OBBs and the best obbMatrixIndex.
void RefitOrientedBoundsImpl(
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

    const uint rootIndex = 0;

    // This algorithm begins by computing Kdops from the leaves starting from internal nodes with all leaf references.
    // These Kdops are then propagated upwards towards the root.
    uint curNodeIndex = ScratchGlobal.Load(ShaderConstants.offsets.obbRefitStack + (obbStackIndex * sizeof(uint)));

    // Load acceleration structure header
    const AccelStructOffsets offsets        = ShaderConstants.header.offsets;

    // Initialise KDOP in group shared memory for first iteration. For the remaining iterations, the KDOP persists
    // as one child node thread always traverses up the tree.
    if (laneGroup.IsFirstLane())
    {
        for (uint i = 0; i < KDOP_PLANE_COUNT; i++)
        {
            WriteKDopMin(laneGroupIndex, i, +FLT_MAX);
            WriteKDopMax(laneGroupIndex, i, -FLT_MAX);
        }
    }

    GroupMemoryBarrier();

    bool done = false;
    uint iterations = 0;
    while ((done == false) && (iterations < Settings.obbNumLevels))
    {
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

        const uint childNodeType = childInfo.NodeType();
        const bool childIsLeaf = (childNodeType != NODE_TYPE_BOX_QUANTIZED_BVH8);
        const bool isChildValid = childInfo.Valid();

        // Pack packet offset in upper bits for leaf nodes and lower bits for internal nodes.
        const uint shift = childIsLeaf ? 16 : 0;
        const uint packedLength = (childInfo.NodeRangeLength() << shift);

        // The following can be done more efficiently using cluster_prefix_sum
        //
        // const uint packedOffset = AmdExtD3DShaderIntrinsics_WaveClusterPrefixSum(packedLength, SG_CLUSTER_SIZE_8);
        //

        // Compute the per-lane prefix sum and subtract the lead lane sum from all to obtain the lane group offsets
        const uint packedOffset = laneGroup.ExclusivePrefixSum(packedLength);

        // Compute absolute offset in memory from packed offset per lane
        uint childNodeOffset = childIsLeaf ? baseLeafNodeOffset : baseInternalNodeOffset;
        childNodeOffset += (((packedOffset >> shift) & 0xffff) * 128u);

        const uint childIndex = (childNodeOffset - offsets.internalNodes) / QBVH8::NodeSizeInBytes;
        const uint childKdopOfs = ShaderConstants.offsets.qbvhKdops + (childIndex * sizeof(ObbKdop));

        const uint bestObbLaneIndex = (obbFlags >> 16u) & 0xFu;
        const uint bestObbMatrixIdxLeaf = (obbFlags >> 24u);

        uint bestObbMatrixIndex = bestObbMatrixIdxLeaf;
        if (laneGroup.laneIndex == bestObbLaneIndex)
        {
            if (childIsLeaf == false)
            {
                bestObbMatrixIndex = ScratchGlobal.Load<uint>(childKdopOfs + OBBKDOP_OBBMATRIX_INDEX_OFFSET);
            }
        }

        bestObbMatrixIndex = laneGroup.Broadcast(bestObbMatrixIndex, bestObbLaneIndex);

        // Compute candidate OBB for valid children.
        BoundingBox candidateObb = InvalidBoundingBox;
        if (isChildValid)
        {
            if (childIsLeaf)
            {
                candidateObb = ComputePrimNodeKDop(laneGroupIndex, childNodeType, childNodeOffset, bestObbMatrixIndex);
            }
            else
            {
                candidateObb = ComputeBoxNodeOBB(childKdopOfs, bestObbMatrixIndex);

                // Update KDOP in group shared memory mapped to the current lane group.
                for (uint i = 0; i < KDOP_PLANE_COUNT; i++)
                {
                    float2 extents = ScratchGlobal.Load<float2>(childKdopOfs + (i * sizeof(float2)));
                    UpdateKDopMin(laneGroupIndex, i, extents.x);
                    UpdateKDopMax(laneGroupIndex, i, extents.y);
                }
            }

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

        const bool shouldWriteObbs = (Settings.enableOrientedBoundingBoxes & bit(BOTTOM_LEVEL));
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

        // Write K-DOPs for nodes up the tree
        if (laneGroup.IsFirstLane())
        {
            for (uint i = 0; i < KDOP_PLANE_COUNT; i++)
            {
                ScratchGlobal.Store<float2>(curNodeKdopOfs + (i * sizeof(float2)),
                                            float2(ReadKDopMin(laneGroupIndex, i), ReadKDopMax(laneGroupIndex, i)));
            }

            ScratchGlobal.Store(curNodeKdopOfs + (KDOP_PLANE_COUNT * sizeof(float2)), bestObbMatrixIndex);
        }

        // Continue up the tree
        iterations++;
        DeviceMemoryBarrier();

        if (curNodeIndex != rootIndex)
        {
            uint oldCount;
            // Atomically decrement the numBoxChildren for the parent of this node
            if (laneGroup.IsFirstLane())
            {
                const uint parentFlagsOffset =
                    ShaderConstants.offsets.qbvhObbFlags + parentIndex * OBB_TABLE_ENTRY_SIZE;

                // numBoxChildren is already encoded as two's complement, so adding 1 actually
                // decrements it:
                ScratchGlobal.InterlockedAdd(parentFlagsOffset, 1, oldCount);
            }
            oldCount = laneGroup.ReadFirstLane(oldCount);

            // Apply two's complement and retrieve bits 0-3 to get the original numBoxChildren:
            const int numBoxChildren = ((oldCount ^ 0x0Fu) + 1) & 0x0Fu;

            // InterlockedAdd puts the previous value in numBoxChildren, so if it's 1, that
            // means this counter is currently 0 and this thread can continue up the tree.
            // If not, then the parent of this node has more inner nodes that need to be
            // processed first.
            if (numBoxChildren == 1)
            {
                curNodeIndex = parentIndex;
            }
            else
            {
                done = true;
            }
        }
        else
        {
            done = true;
        }
    }
}
#endif

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
// Main Function : RefitOrientedBounds
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void RefitOrientedBounds(
    in uint globalId : SV_DispatchThreadID,
    in uint localId  : SV_GroupThreadID)
{
#if GPURT_BUILD_RTIP3_1
    const uint obbRefitBatchCount = ScratchGlobal.Load(
        ShaderConstants.offsets.obbRefitStackPtrs + STACK_PTRS_OBB_REFIT_BATCH_COUNT);

    RefitOrientedBoundsImpl(globalId, localId, obbRefitBatchCount);
#endif
}
#endif

#endif
