/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
#include "../shadersClean/common/ShaderDefs.hlsli"

#include "../shadersClean/build/BuildRootSignature.hlsli"
#endif

//=====================================================================================================================
#include "MortonCodes.hlsl"

#if NO_SHADER_ENTRYPOINT == 0
template<typename T>
T LoadInstanceDescBuffer(uint offset)
{
    return InstanceDescBuffer.Load<T> (offset);
}
#include "IndirectArgBufferUtils.hlsl"
#endif

#include "../shadersClean/build/BuildCommonScratch.hlsli"
#if GPURT_BUILD_RTIP3_1
#include "OrientedBoundingBoxes.hlsl"
#endif

//=====================================================================================================================
void GenerateMortonCodesImpl(
    float3  boundsMin,
    float3  boundsExtent,
    float2  sizeMinMax,
    uint    primitiveIndex,
    uint    leafNodesOffset,
    uint    mortonCodesOffset,
    uint    useMortonCode30,
    uint    numSizeBits)
{
    ScratchNode node = FetchScratchNode(leafNodesOffset, primitiveIndex);

    if (useMortonCode30)
    {
        uint mortonCode = UINT32_MAX;
        if (IsNodeActive(node))
        {
            const BoundingBox primitiveBounds = GetScratchNodeBoundingBox(node,
                                                                          true,
                                                                          Settings.enableEarlyPairCompression,
                                                                          leafNodesOffset);

            const float3 position = 0.5f * (primitiveBounds.max + primitiveBounds.min);

            mortonCode = CalculateMortonCode32(
                boundsMin,
                boundsExtent,
                position,
                IsInvalidBoundingBox(primitiveBounds)
            );

            // If we generated an invalid morton, make it "valid" so we don't lose the primitive
            if (mortonCode == UINT32_MAX)
            {
                mortonCode = UINT32_MAX - 1U;
            }

            IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET, 1);
        }
        WriteMortonCode(mortonCodesOffset, primitiveIndex, mortonCode);
    }
    else
    {
        uint64_t mortonCode = 0xFFFFFFFFFFFFFFFEULL;
        if (IsNodeActive(node))
        {
            const BoundingBox primitiveBounds = GetScratchNodeBoundingBox(node,
                                                                          true,
                                                                          Settings.enableEarlyPairCompression,
                                                                          leafNodesOffset);
            const float surfaceArea = ComputeBoxSurfaceArea(primitiveBounds);
            const float3 position = 0.5f * (primitiveBounds.max + primitiveBounds.min);

            mortonCode = CalculateMortonCode64(
                boundsMin,
                boundsExtent,
                sizeMinMax,
                position,
                surfaceArea,
                IsInvalidBoundingBox(primitiveBounds),
                numSizeBits
            );

            // If we generated an invalid morton, make it "valid" so we don't lose the primitive
            if (mortonCode > 0xFFFFFFFFFFFFFFFEULL)
            {
                mortonCode = 0xFFFFFFFFFFFFFFFEULL;
            }

            IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET, 1);
        }
        WriteMortonCode64(mortonCodesOffset, primitiveIndex, mortonCode);
    }

#if GPURT_BUILD_RTIP3_1
    // Note, FetchTriKdopObbMatrixIndex current implementation is quite expensive in terms of both VGPR usage and
    //       ALU. We pre-compute those in the morton code shader to avoid being occupancy limited due to VGPRs in
    //       the EncodeHwBvh3_1 phase. An alternative is to compute this in EncodeNodes, but the code changes
    //       there affect Updates as well and really need the unified bindings to implement this in a clean way.
    if (IsObbEnabled() && IsNodeActive(node))
    {
        uint obbMatrixIdx = INVALID_OBB;
        if ((Settings.topLevelBuild == false))
        {
            float3 v0 = node.bbox_min_or_v0;
            float3 v1 = node.bbox_max_or_v1;
            float3 v2 = node.sah_or_v2_or_instBasePtr;
            obbMatrixIdx = FetchTriKdopObbMatrixIndex(v0, v1, v2);
        }
        else
        {
            // Note: LTD builder uses entirety of flag offset, thus can't be used with TLAS OBBs
            const uint instanceDescOffsetInBytes = LoadNumPrimAndOffset().primitiveOffset;
            const InstanceDesc desc = LoadInstanceDesc(node.left_or_primIndex_or_instIndex, instanceDescOffsetInBytes);
            const float3x3 rotationScale = float3x3(desc.Transform[0].xyz,
                                                    desc.Transform[1].xyz,
                                                    desc.Transform[2].xyz);

            const float3 scale = float3(length(rotationScale[0]),
                                        length(rotationScale[1]),
                                        length(rotationScale[2]));

            const float3x3 rotation = float3x3(rotationScale[0] * rcp(scale.x),
                                               rotationScale[1] * rcp(scale.y),
                                               rotationScale[2] * rcp(scale.z));
            obbMatrixIdx = MatrixToBestOBBIndex(rotation);
        }
        const uint scratchNodeOffset = leafNodesOffset + (primitiveIndex * sizeof(ScratchNode));
        const uint packedFlags = (node.packedFlags & bits(24)) | (obbMatrixIdx << 24);
        ScratchGlobal.Store(scratchNodeOffset + SCRATCH_NODE_FLAGS_OFFSET, packedFlags);
    }
#endif

    // Clear refit propagation flags for each leaf node in BVH2.
    const uint initValue = (UsesFastLbvhLayout() ? 0xffffffffu : 0);
    const uint flagOffset = ShaderConstants.offsets.propagationFlags + (primitiveIndex * sizeof(uint));
    ScratchBuffer.Store(flagOffset, initValue);
}

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
// Main Function : GenerateMortonCodes
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void GenerateMortonCodes(
    uint globalThreadId : SV_DispatchThreadID)
{
    const uint primitiveIndex = globalThreadId;

    const uint numPrimitives = FetchTaskCounter(
        ShaderConstants.offsets.encodeTaskCounter + ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET);

    if (primitiveIndex < numPrimitives)
    {
        MortonBoundingBox mortonBounds = FetchMortonBounds(ShaderConstants.offsets.sceneBounds, ShaderConstants.numMortonSizeBits);

        GenerateMortonCodesImpl(
            mortonBounds.min,
            mortonBounds.extent,
            mortonBounds.sizeMinMax,
            primitiveIndex,
            ShaderConstants.offsets.bvhLeafNodeData,
            ShaderConstants.offsets.mortonCodes,
            Settings.useMortonCode30,
            mortonBounds.numSizeBits);
    }
}
#endif
