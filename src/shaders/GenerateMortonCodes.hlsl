/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

#include "BuildCommonScratch.hlsl"

//=====================================================================================================================
void GenerateMortonCodesImpl(
    float3  sceneMin,
    float3  sceneExtents,
    uint    primitiveIndex,
    uint    leafNodesOffset,
    uint    mortonCodesOffset,
    uint    doTriangleSplitting,
    uint    splitBoxesOffset,
    uint    enableVariableBits,
    uint    useMortonCode30,
    uint    numSizeBits,
    float2  sizeMinMax)
{
    ScratchNode node = FetchScratchNode(leafNodesOffset, primitiveIndex);

    if (IsNodeActive(node))
    {
        // Calculate the scene-normalized position of the center of the current bounding box.
        // This is required for the morton code generation.
        const BoundingBox bounds = GetScratchNodeBoundingBox(node,
                                                             true,
                                                             doTriangleSplitting,
                                                             splitBoxesOffset,
                                                             Settings.enableEarlyPairCompression,
                                                             leafNodesOffset);

        const bool isDegenerate = IsInvalidBoundingBox(bounds);

        const float3 center = (0.5 * (bounds.max + bounds.min));
        float3 normalizedPos = ((center - sceneMin) / sceneExtents);

        // TODO: optimize this based on instance transform or primitive centroid
        if (isDegenerate)
        {
            normalizedPos = float3(0.5, 0.5, 0.5);
        }

        if (useMortonCode30)
        {
            const uint mortonCode = CalculateMortonCode32(normalizedPos, sceneExtents, enableVariableBits);
            WriteMortonCode(mortonCodesOffset, primitiveIndex, mortonCode);
        }
        else
        {
            const float surfaceArea = ComputeBoxSurfaceArea(bounds);

            uint64_t mortonCode = CalculateMortonCode64(
                normalizedPos, sceneExtents, surfaceArea, sizeMinMax, numSizeBits, enableVariableBits);

            // To address the bug in MergeSort().
            if (mortonCode == 0x7FFFFFFFFFFFFFFF)
            {
                mortonCode = 0x7FFFFFFFFFFFFFFE;
            }

            WriteMortonCode64(mortonCodesOffset, primitiveIndex, mortonCode);
        }

        IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET, 1);
    }
    else
    {
        if (useMortonCode30)
        {
            WriteMortonCode(mortonCodesOffset, primitiveIndex, INT_MAX);
        }
        else
        {
            WriteMortonCode64(mortonCodesOffset, primitiveIndex, 0x7FFFFFFFFFFFFFFE);
        }
    }

    // Clear refit propagation flags for each leaf node in BVH2.
    const uint initValue = (Settings.enableFastLBVH ? 0xffffffffu : 0);
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
        const BoundingBox sceneBounds = FetchSceneBounds(ShaderConstants.offsets.sceneBounds);

        const float3 sceneExtent = sceneBounds.max - sceneBounds.min;
        const float3 sceneMin    = sceneBounds.min;

        GenerateMortonCodesImpl(
            sceneMin,
            sceneExtent,
            primitiveIndex,
            ShaderConstants.offsets.bvhLeafNodeData,
            ShaderConstants.offsets.mortonCodes,
            Settings.doTriangleSplitting,
            ShaderConstants.offsets.triangleSplitBoxes,
            Settings.enableVariableBitsMortonCode,
            Settings.useMortonCode30,
            0,
            float2(0, 0));
    }
}
#endif
