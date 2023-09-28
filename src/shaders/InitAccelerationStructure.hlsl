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
#define RootSig "RootConstants(num32BitConstants=1, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "DescriptorTable(CBV(b0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 2)),"\
                "CBV(b255)"

//=====================================================================================================================

struct RootConstants
{
    uint numBuilders;
};

[[vk::push_constant]] ConstantBuffer<RootConstants> ShaderRootConstants : register(b0);

#include "Common.hlsl"

struct Constants
{
    uint rebraidType;
    uint triangleCompressionMode;
    uint debugCountersScratchOffset;
    uint sceneBoundsScratchOffset;
    uint numBatchesScratchOffset;
    uint numLeafNodes;
    // Add padding for 16-byte alignment rules
    uint pad0;
    uint pad1;

    AccelStructHeader header;
    AccelStructMetadataHeader metadataHeader;
};

[[vk::binding(0, 1)]] ConstantBuffer<Constants> BatchBuilderConstants[] : register(b0, space1);
[[vk::binding(0, 0)]] RWByteAddressBuffer       BatchHeaderBuffers[]    : register(u0, space1);
[[vk::binding(1, 0)]] RWByteAddressBuffer       BatchScratchBuffers[]   : register(u0, space2);

[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void InitAccelerationStructure(
    uint globalId : SV_DispatchThreadID)
{
    if (globalId >= ShaderRootConstants.numBuilders)
    {
        return;
    }

    while (true)
    {
        if (WaveIsFirstLane() == false)
        {
            continue;
        }

        const uint builderIndex = WaveReadLaneFirst(globalId);
        const ConstantBuffer<Constants> BuilderConstants = BatchBuilderConstants[builderIndex];
        const RWByteAddressBuffer       HeaderBuffer     = BatchHeaderBuffers[builderIndex];
        const RWByteAddressBuffer       ScratchBuffer    = BatchScratchBuffers[builderIndex];

        if (Settings.enableBVHBuildDebugCounters)
        {
            // Clear debug counters
            const uint countersOffset = BuilderConstants.debugCountersScratchOffset;
            const uint RayTracingBuildDebugCounters = 11;
            for (uint i = 0; i < RayTracingBuildDebugCounters; ++i)
            {
                ScratchBuffer.Store<uint>(countersOffset + (i * sizeof(uint)), 0u);
            }
        }

        const uint InitialMax = FloatToUint(-FLT_MAX);
        const uint InitialMin = FloatToUint(FLT_MAX);

        if (BuilderConstants.numLeafNodes != 0)
        {
            if (BuilderConstants.rebraidType == RebraidType::V2)
            {
                const uint sceneBounds[] =
                {
                    InitialMin, InitialMin, InitialMin,
                    InitialMax, InitialMax, InitialMax,
                    InitialMin, InitialMax, // size

                    InitialMin, InitialMin, InitialMin, //used for rebraid
                    InitialMax, InitialMax, InitialMax,
                };
                ScratchBuffer.Store(BuilderConstants.sceneBoundsScratchOffset, sceneBounds);
            }
            else
            {
                const uint sceneBounds[] =
                {
                    InitialMin, InitialMin, InitialMin,
                    InitialMax, InitialMax, InitialMax,
                    InitialMin, InitialMax, // size
                };

                ScratchBuffer.Store(BuilderConstants.sceneBoundsScratchOffset, sceneBounds);
            }

            if (BuilderConstants.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION)
            {
                ScratchBuffer.Store(BuilderConstants.numBatchesScratchOffset, 0);
            }
        }

        HeaderBuffer.Store<AccelStructMetadataHeader>(0, BuilderConstants.metadataHeader);
        HeaderBuffer.Store<AccelStructHeader>(BuilderConstants.header.metadataSizeInBytes, BuilderConstants.header);
        break;
    }
}
