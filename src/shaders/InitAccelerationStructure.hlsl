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
#define RootSig "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 1)),"\
                "CBV(b255)"
#else
#define RootSig "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(CBV(b0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 2)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 3)),"\
                "CBV(b255)"
#endif

#include "Common.hlsl"

[[vk::binding(0, 3)]] RWByteAddressBuffer       BatchScratchGlobals[]   : register(u0, space1);
#ifndef IS_UPDATE
struct Constants
{
    uint maxNumPrimitives;
    uint debugCountersScratchOffset;
    uint sceneBoundsScratchOffset;
    uint numBatchesScratchOffset;

    uint rebraidTaskQueueCounterScratchOffset;
    uint tdTaskQueueCounterScratchOffset;
    uint plocTaskQueueCounterScratchOffset;
    uint encodeTaskCounterScratchOffset;

    uint taskLoopCountersOffset;
    uint isIndirectBuild;
    uint dynamicallyIncrementsPrimRefCount;
    uint padding1;

    AccelStructHeader header;
    AccelStructMetadataHeader metadataHeader;
};

[[vk::binding(0, 4)]] ConstantBuffer<Constants> BatchBuilderConstants[] : register(b0, space1);
[[vk::binding(0, 5)]] RWByteAddressBuffer       BatchHeaderBuffers[]    : register(u0, space2);
[[vk::binding(0, 6)]] RWByteAddressBuffer       BatchScratchBuffers[]   : register(u0, space3);
#endif

static const uint INVALID_SCRATCH_OFFSET = 0xFFFFFFFF;
// =====================================================================================================================
// Reset counters
void ResetCounters(
    RWByteAddressBuffer scratchBuffer,
    uint                counterOffset,
    uint                numDwords)
{
    if (counterOffset != INVALID_SCRATCH_OFFSET)
    {
        for (uint i = 0; i < numDwords; ++i)
        {
            scratchBuffer.Store(counterOffset + (i * sizeof(uint)), 0u);
        }
    }
}

// =====================================================================================================================
[RootSignature(RootSig)]
[numthreads(1, 1, 1)]
void InitAccelerationStructure(
    uint groupId  : SV_GroupId)
{
#ifdef IS_UPDATE
    const RWByteAddressBuffer ScratchGlobal = BatchScratchGlobals[groupId];

    // Reset update stack pointer and update task counters.
    ScratchGlobal.Store<uint4>(0, uint4(0u, 0u, 0u, 0u));

#else
    const ConstantBuffer<Constants> BuilderConstants = BatchBuilderConstants[groupId];
    const RWByteAddressBuffer       HeaderBuffer     = BatchHeaderBuffers[groupId];
    const RWByteAddressBuffer       ScratchBuffer    = BatchScratchBuffers[groupId];
    const RWByteAddressBuffer       ScratchGlobal    = BatchScratchGlobals[groupId];

    const uint RayTracingBuildDebugCounters = 11;
    const uint RayTracingTaskQueueCounters = 5;

    if (Settings.enableBVHBuildDebugCounters)
    {
        // Clear debug counters
        ResetCounters(ScratchGlobal, BuilderConstants.debugCountersScratchOffset, RayTracingBuildDebugCounters);
    }

    const uint InitialMax = FloatToUint(-FLT_MAX);
    const uint InitialMin = FloatToUint(FLT_MAX);

    if (BuilderConstants.maxNumPrimitives != 0)
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

            ResetCounters(ScratchGlobal, BuilderConstants.rebraidTaskQueueCounterScratchOffset, RayTracingTaskQueueCounters);
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
            ScratchGlobal.Store(BuilderConstants.numBatchesScratchOffset, 0);
        }

        ResetCounters(ScratchGlobal, BuilderConstants.tdTaskQueueCounterScratchOffset, RayTracingTaskQueueCounters);
        ResetCounters(ScratchGlobal, BuilderConstants.plocTaskQueueCounterScratchOffset, RayTracingTaskQueueCounters);

        // Initialise encode counters
        ScratchGlobal.Store(
            BuilderConstants.encodeTaskCounterScratchOffset + ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET, 0);

        // Early triangle pairing, triangle splitting and indirect BLAS builds dynamically increment
        // primitive reference counter. Initialise counters to 0 when these features are enabled.
        const uint primRefInitCount = (BuilderConstants.dynamicallyIncrementsPrimRefCount == 0) ?
            BuilderConstants.header.numPrimitives : 0;

        ScratchGlobal.Store(
            BuilderConstants.encodeTaskCounterScratchOffset + ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET, primRefInitCount);

        ScratchGlobal.Store3(
            BuilderConstants.encodeTaskCounterScratchOffset + ENCODE_TASK_COUNTER_INDIRECT_ARGS, uint3(0, 1, 1));

        ResetCounters(ScratchGlobal, BuilderConstants.taskLoopCountersOffset, TASK_LOOP_COUNTERS_NUM_DWORDS);
    }

    HeaderBuffer.Store<AccelStructMetadataHeader>(0, BuilderConstants.metadataHeader);
    HeaderBuffer.Store<AccelStructHeader>(BuilderConstants.header.metadataSizeInBytes, BuilderConstants.header);

#endif
}
