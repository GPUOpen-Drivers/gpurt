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

template<typename T>
T LoadInstanceDescBuffer(uint offset)
{
    return InstanceDescBuffer.Load<T> (offset);
}
#include "IndirectArgBufferUtils.hlsl"
#endif

#include "BuildCommonScratch.hlsl"

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

            float3 normalizedPos = clamp((position - boundsMin) / boundsExtent, 0.0f, 1.0f);

            // TODO: optimize degenerates based on instance transform or primitive centroid
            if (IsInvalidBoundingBox(primitiveBounds))
            {
                normalizedPos = float3(0.5f, 0.5f, 0.5f);
            }

            mortonCode = CalculateMortonCode32(normalizedPos, boundsExtent);

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
