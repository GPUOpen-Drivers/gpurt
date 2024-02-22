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
#ifdef IS_UPDATE
#define RootSig "RootConstants(num32BitConstants=1, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 2)),"
#else
#define RootSig "RootConstants(num32BitConstants=1, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "DescriptorTable(CBV(b0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 2)),"\
                "CBV(b255)"
#endif

//=====================================================================================================================

struct RootConstants
{
    uint numBuilders;
};

[[vk::push_constant]] ConstantBuffer<RootConstants> ShaderRootConstants : register(b0);

#include "Common.hlsl"

#ifndef IS_UPDATE
struct Constants
{
    uint numLeafNodes;
    uint debugCountersScratchOffset;
    uint sceneBoundsScratchOffset;
    uint numBatchesScratchOffset;

    uint rebraidTaskQueueCounterScratchOffset;
    uint tdTaskQueueCounterScratchOffset;
    uint plocTaskQueueCounterScratchOffset;
    uint encodeTaskCounterScratchOffset;

    uint taskLoopCountersOffset;
    uint padding0;
    uint padding1;
    uint padding2;

    AccelStructHeader header;
    AccelStructMetadataHeader metadataHeader;
};

[[vk::binding(0, 1)]] ConstantBuffer<Constants> BatchBuilderConstants[] : register(b0, space1);
[[vk::binding(0, 0)]] RWByteAddressBuffer       BatchHeaderBuffers[]    : register(u0, space1);
#endif
[[vk::binding(1, 0)]] RWByteAddressBuffer       BatchScratchBuffers[]   : register(u0, space2);

// =====================================================================================================================
// Reset the taskQueue build counters
void ResetTaskQueueCounters(
    RWByteAddressBuffer scratchBuffer,
    uint                counterOffset)
{
    const uint RayTracingTaskQueueCounters = 5;
    for (uint i = 0; i < RayTracingTaskQueueCounters; ++i)
    {
        scratchBuffer.Store(counterOffset + (i * sizeof(uint)), 0u);
    }
}

static const uint INVALID_SCRATCH_OFFSET = 0xFFFFFFFF;

// =====================================================================================================================
[RootSignature(RootSig)]
[numthreads(1, 1, 1)]
void InitAccelerationStructure(
    uint globalId : SV_DispatchThreadID,
    uint groupId  : SV_GroupId)
{
    const RWByteAddressBuffer ScratchBuffer = BatchScratchBuffers[groupId];

#ifdef IS_UPDATE

    // Reset update stack pointer and update task counters.
    ScratchBuffer.Store<uint4>(0, uint4(0u, 0u, 0u, 0u));

#else

    const RWByteAddressBuffer       HeaderBuffer     = BatchHeaderBuffers[groupId];
    const ConstantBuffer<Constants> BuilderConstants = BatchBuilderConstants[groupId];

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
        const bool isRebraidEnabled = BuilderConstants.rebraidTaskQueueCounterScratchOffset != INVALID_SCRATCH_OFFSET;
        if (isRebraidEnabled)
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
            ResetTaskQueueCounters(ScratchBuffer, BuilderConstants.rebraidTaskQueueCounterScratchOffset);
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

        const bool isPairCompressionEnabled = BuilderConstants.numBatchesScratchOffset != INVALID_SCRATCH_OFFSET;
        if (isPairCompressionEnabled)
        {
            ScratchBuffer.Store(BuilderConstants.numBatchesScratchOffset, 0);
        }

        const bool isTopDownBuild = BuilderConstants.tdTaskQueueCounterScratchOffset != INVALID_SCRATCH_OFFSET;
        if (isTopDownBuild)
        {
            ResetTaskQueueCounters(ScratchBuffer, BuilderConstants.tdTaskQueueCounterScratchOffset);
        }
        else if (BuilderConstants.plocTaskQueueCounterScratchOffset != INVALID_SCRATCH_OFFSET)
        {
            ResetTaskQueueCounters(ScratchBuffer, BuilderConstants.plocTaskQueueCounterScratchOffset);
        }

        // Initialise encode counters
        ScratchBuffer.Store(
            BuilderConstants.encodeTaskCounterScratchOffset + ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET, 0);

        // Early triangle pairing and triangle splitting dynamically increment primitive reference counter. Initialise
        // counters to 0 when these features are enabled
        const uint primRefInitCount =
            (Settings.enableEarlyPairCompression || Settings.doTriangleSplitting) ?
                0 : BuilderConstants.header.numPrimitives;

        ScratchBuffer.Store(
            BuilderConstants.encodeTaskCounterScratchOffset + ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET, primRefInitCount);

        for (uint i = 0; i < TASK_LOOP_COUNTERS_NUM_DWORDS; ++i)
        {
            ScratchBuffer.Store(BuilderConstants.taskLoopCountersOffset + (i * sizeof(uint)), 0);
        }
    }

    HeaderBuffer.Store<AccelStructMetadataHeader>(0, BuilderConstants.metadataHeader);
    HeaderBuffer.Store<AccelStructHeader>(BuilderConstants.header.metadataSizeInBytes, BuilderConstants.header);

#endif
}
