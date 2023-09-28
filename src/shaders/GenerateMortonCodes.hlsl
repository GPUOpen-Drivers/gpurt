/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
// Root signature
#define RootSig "CBV(b0), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "CBV(b255),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL)"

#include "../shared/rayTracingDefs.h"

[[vk::binding(1, 0)]] ConstantBuffer<BuildShaderConstants> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] RWByteAddressBuffer DstBuffer     : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer DstMetadata   : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer ScratchBuffer : register(u2);

// unused buffer
[[vk::binding(3, 0)]] RWByteAddressBuffer SrcBuffer     : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer EmitBuffer    : register(u4);
#endif

#include "BuildCommonScratch.hlsl"

//=====================================================================================================================
uint CalculateVariableBitsMortonCode(float3 sceneExtent, float3 normalizedPos)
{
    const uint numMortonBits = 30;
    int3 numBits = uint3(0, 0, 0);

    int i;

    int3 numPrebits;
    int3 startAxis;

    // find the largest start axis
    // and how many prebits are needed between largest and two other axes
    if (sceneExtent.x < sceneExtent.y)
    {
        if (sceneExtent.x < sceneExtent.z)
        {
            if (sceneExtent.y < sceneExtent.z)
            {
                // z, y, x
                startAxis[0] = 2;
                numPrebits[0] = log2(sceneExtent.z / sceneExtent.y);

                startAxis[1] = 1;
                numPrebits[1] = log2(sceneExtent.y / sceneExtent.x);

                startAxis[2] = 0;
                numPrebits[2] = log2(sceneExtent.z / sceneExtent.x);
            }
            else
            {
                // y, z, x
                startAxis[0] = 1;
                numPrebits[0] = log2(sceneExtent.y / sceneExtent.z);

                startAxis[1] = 2;
                numPrebits[1] = log2(sceneExtent.z / sceneExtent.x);

                startAxis[2] = 0;
                numPrebits[2] = log2(sceneExtent.y / sceneExtent.x);
            }
        }
        else
        {
            // y, x, z
            startAxis[0] = 1;
            numPrebits[0] = log2(sceneExtent.y / sceneExtent.x);

            startAxis[1] = 0;
            numPrebits[1] = log2(sceneExtent.x / sceneExtent.z);

            startAxis[2] = 2;
            numPrebits[2] = log2(sceneExtent.y / sceneExtent.z);
        }
    }
    else
    {
        if (sceneExtent.y < sceneExtent.z)
        {
            if (sceneExtent.x < sceneExtent.z)
            {
                // z, x, y
                startAxis[0] = 2;
                numPrebits[0] = log2(sceneExtent.z / sceneExtent.x);

                startAxis[1] = 0;
                numPrebits[1] = log2(sceneExtent.x / sceneExtent.y);

                startAxis[2] = 1;
                numPrebits[2] = log2(sceneExtent.z / sceneExtent.y);
            }
            else
            {
                // x, z, y
                startAxis[0] = 0;
                numPrebits[0] = log2(sceneExtent.x / sceneExtent.z);

                startAxis[1] = 2;
                numPrebits[1] = log2(sceneExtent.z / sceneExtent.y);

                startAxis[2] = 1;
                numPrebits[2] = log2(sceneExtent.x / sceneExtent.y);
            }
        }
        else
        {
            // x, y, z
            startAxis[0] = 0;
            numPrebits[0] = log2(sceneExtent.x / sceneExtent.y);

            startAxis[1] = 1;
            numPrebits[1] = log2(sceneExtent.y / sceneExtent.z);

            startAxis[2] = 2;
            numPrebits[2] = log2(sceneExtent.x / sceneExtent.z);
        }
    }

    // say x > y > z
    // prebits[0] = 3
    // prebits[1] = 2
    // if swap == 1
    // xxx xy xy x yxz yxz ...
    // if swap == 0
    // xxx xy xy xyz xyz ...
    int swap = numPrebits[2] - (numPrebits[0] + numPrebits[1]);

    numPrebits[0] = min(numPrebits[0], numMortonBits);
    numPrebits[1] = min(numPrebits[1] * 2, numMortonBits - numPrebits[0]) / 2;

    int numPrebitsSum = numPrebits[0] + numPrebits[1] * 2;

    if (numPrebitsSum != numMortonBits)
    {
        numPrebitsSum += swap;
    }
    else
    {
        swap = 0;
    }

    // the scene might be 2D so check for the smallest axis
    numBits[2] = (sceneExtent[startAxis[2]] != 0) ? max(0, (numMortonBits - numPrebitsSum) / 3) : 0;

    if (swap > 0)
    {
        numBits[0] = max(0, (numMortonBits - numBits[2] - numPrebitsSum) / 2 + numPrebits[1] + numPrebits[0] + 1);
        numBits[1] = numMortonBits - numBits[0] - numBits[2];
    }
    else
    {
        numBits[1] = max(0, (numMortonBits - numBits[2] - numPrebitsSum) / 2 + numPrebits[1]);
        numBits[0] = numMortonBits - numBits[1] - numBits[2];
    }

    uint mortonCode = 0;
    uint3 axisCode;

    // based on the number of bits, calculate each code per axis
    [unroll]
    for (uint a = 0; a < 3; a++)
    {
        axisCode[a] = min(max(normalizedPos[startAxis[a]] * (1U << numBits[a]), 0.0), (1U << numBits[a]) - 1);
    }

    uint delta0 = 0;
    uint delta1 = 0;

    // if there are prebits, set them in the morton code:
    // if swap == 1
    // [xxx xy xy x] yxz yxz ...
    // if swap == 0
    // [xxx xy xy xyz] xyz ...
    if (numPrebitsSum > 0)
    {
        numBits[0] -= numPrebits[0];
        mortonCode = axisCode[0] & (((1U << numPrebits[0]) - 1) << numBits[0]);
        mortonCode >>= numBits[0];

        mortonCode <<= numPrebits[1] * 2;
        numBits[0] -= numPrebits[1];
        numBits[1] -= numPrebits[1];
        uint temp0 = axisCode[0] & (((1U << numPrebits[1]) - 1) << numBits[0]);
        temp0 >>= numBits[0];
        temp0 = ExpandBits2D(temp0);

        uint temp1 = axisCode[1] & (((1U << numPrebits[1]) - 1) << numBits[1]);
        temp1 >>= numBits[1];
        temp1 = ExpandBits2D(temp1);

        mortonCode |= temp0 * 2 + temp1;

        if (swap > 0)
        {
            mortonCode <<= 1;
            numBits[0] -= 1;
            uint temp = axisCode[0] & (1U << numBits[0]);
            temp >>= numBits[0];
            mortonCode |= temp;
        }

        mortonCode <<= numBits[0] + numBits[1] + numBits[2];

        axisCode[0] &= ((1U << numBits[0]) - 1);
        axisCode[1] &= ((1U << numBits[1]) - 1);

        if (swap > 0)
        {
            delta0 = (numBits[1] - numBits[0]);
            axisCode[0] <<= delta0;

            delta1 = (numBits[1] - numBits[2]);
            axisCode[2] <<= delta1;
        }
        else
        {
            delta0 = (numBits[0] - numBits[1]);
            axisCode[1] <<= delta0;

            delta1 = (numBits[0] - numBits[2]);
            axisCode[2] <<= delta1;
        }
    }

    // 2D case, just use xy xy xy...
    if (numBits[2] == 0)
    {
        [unroll]
        for (int r = 0; r < 2; r++)
        {
            axisCode[r] = ExpandBits2D(axisCode[r]);
        }

        mortonCode |= axisCode[0] * 2 + axisCode[1];
    }
    else // 3D case, just use if swap == 0 xyz xyz xyz..., if swap == 1 yxz yxz yxz...
    {
        [unroll]
        for (int i = 0; i < 3; i++)
        {
            axisCode[i] = (axisCode[i] > 0) ? ExpandBits(axisCode[i]) : 0;
        }

        if (swap > 0)
        {
            mortonCode |= (axisCode[1] * 4 + axisCode[0] * 2 + axisCode[2]) >> (delta0 + delta1);
        }
        else
        {
            mortonCode |= (axisCode[0] * 4 + axisCode[1] * 2 + axisCode[2]) >> (delta0 + delta1);
        }
    }

    return mortonCode;
}

//=====================================================================================================================
uint ExpandForSizeBits(uint value)
{
    // input is 24 bits
    value &= ((1U << 24) - 1);

    uint mask = ((1U << 12) - 1);

    // 12[4]12
    value = ((value & ~mask) << 4) | (value & mask);

    mask = (((1U << 6) - 1) << 6) | (((1U << 6) - 1) << 22);

    // 6[2]6[2]6[2]6
    value = (value & ~mask) | ((value & mask) << 2);

    mask = (((1U << 3) - 1) << 3) +
           (((1U << 3) - 1) << 11) +
           (((1U << 3) - 1) << 19) +
           (((1U << 3) - 1) << 27);

    // 3[1]3 [1] 3[1]3 [1] 3[1]3 [1] 3[1]3
    value = ((value & mask) << 1) | (value & ~mask);

    return value;
}

//=====================================================================================================================
void GenerateMortonCodesImpl(
    float3  sceneMin,
    float3  sceneExtent,
    uint    primitiveIndex,
    uint    leafNodesOffset,
    uint    mortonCodesOffset,
    uint    doTriangleSplitting,
    uint    splitBoxesOffset,
    uint    enableVariableBits,
    uint    useMortonCode30,
    uint    numSizeBits,
    float2  sizeMinMax,
    bool    enableGridPos,
    uint    gridPosOffset)
{
    ScratchNode node = FetchScratchNode(leafNodesOffset, primitiveIndex);

    if (IsNodeActive(node) && (IsNodeLinkedOnly(node) == false))
    {
        // Calculate the scene-normalized position of the center of the current bounding box.
        // This is required for the morton code generation.
        BoundingBox bounds;

        if (doTriangleSplitting)
        {
            bounds = FetchSplitBoxAtIndex(splitBoxesOffset, node.splitBox_or_nodePointer);
        }
        else
        {
            bounds = GetScratchNodeBoundingBox(node);
        }

        const float3 center = (0.5 * (bounds.max + bounds.min));
        const float3 normalizedPos = ((center - sceneMin) / sceneExtent);

        if (useMortonCode30)
        {
            const uint mortonCode = (enableVariableBits) ? CalculateVariableBitsMortonCode(sceneExtent, normalizedPos) :
                                                           CalculateMortonCode(normalizedPos);

            WriteMortonCode(mortonCodesOffset, primitiveIndex, mortonCode);
        }
        else
        {
            if (sizeMinMax.y == sizeMinMax.x)
            {
                numSizeBits = 0;
            }

            uint numAxisBits;
            uint3 valuesWithNoSize;
            uint4 numBits;
            uint64_t mortonCode = (enableVariableBits) ? CalculateVariableBitsMortonCode64(sceneExtent,
                                                                                           normalizedPos,
                                                                                           numSizeBits,
                                                                                           valuesWithNoSize,
                                                                                           numAxisBits,
                                                                                           numBits) :
                                                         CalculateMortonCode64(normalizedPos);

            uint w = 0;

            if (numSizeBits > 0)
            {
                const uint64_t mortonCodeTemp = mortonCode;

                const float surfaceArea = ComputeBoxSurfaceArea(bounds);

                const float size = (surfaceArea - sizeMinMax.x) / (sizeMinMax.y - sizeMinMax.x);

                w =  min(max(size * (1UL << numSizeBits), 0.0), (1UL << numSizeBits) - 1);

                const uint64_t sizeValue = ExpandBits4D(w);

                const uint numAxisBitsWithSize = numSizeBits * 3;

                const uint numAxisBitsWithNoSize = numAxisBits - numAxisBitsWithSize;

                const uint bitsToExpand = uint(mortonCodeTemp >> numAxisBitsWithNoSize);

                mortonCode = (ExpandForSizeBits(bitsToExpand) << 1) | sizeValue;

                mortonCode = (mortonCode << numAxisBitsWithNoSize) | (mortonCodeTemp & ((1ULL << numAxisBitsWithNoSize) - 1));
            }

            if (enableGridPos)
            {
                uint4 gridPos;
                gridPos[0] = valuesWithNoSize[0];
                gridPos[1] = valuesWithNoSize[1];
                gridPos[2] = valuesWithNoSize[2];
                gridPos[3] = w;

                WriteGridPosAtIndex(gridPosOffset, primitiveIndex, gridPos);
            }

            // To address the bug in MergeSort().
            if (mortonCode == 0x7FFFFFFFFFFFFFFF)
            {
                mortonCode = 0x7FFFFFFFFFFFFFFE;
            }
            WriteMortonCode64(mortonCodesOffset, primitiveIndex, mortonCode);
        }

        IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET, 1);
    }
    else if (IsNodeLinkedOnly(node))
    {
        if (useMortonCode30)
        {
            WriteMortonCode(mortonCodesOffset, primitiveIndex, 0x7FFFFFFD);
        }
        else
        {
            WriteMortonCode64(mortonCodesOffset, primitiveIndex, 0x7FFFFFFFFFFFFFFD);
        }
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
    const uint numPrimitives =
        (Settings.topLevelBuild && Settings.rebraidType == RebraidType::V2) || Settings.doTriangleSplitting ?
            ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET) :
            ShaderConstants.numLeafNodes;

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
            float2(0, 0),
            false,
            0);
    }
}
#endif
