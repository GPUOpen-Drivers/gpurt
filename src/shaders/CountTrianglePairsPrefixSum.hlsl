/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#define RootSig "RootConstants(num32BitConstants=3, b0),"\
                "UAV(u0),"\
                "UAV(u1)"

#include "../shared/rayTracingDefs.h"

//=====================================================================================================================
// This is a duplicate from shared/math.h. Just including math.h requires many other dependencies. We should
// probably refactor the non-extension specific functions elsewhere.
//
static uint32_t Pow2Align(
    uint32_t value,
    uint32_t alignment)
{
    return ((value + alignment - 1) & ~(alignment - 1));
}

struct RootConstants
{
    uint numBlocks;
    uint basePrimRefCountOffset;
    uint encodeTaskCounterScratchOffset;
};

[[vk::push_constant]] ConstantBuffer<RootConstants> ShaderRootConstants : register(b0);
[[vk::binding(0, 0)]] RWByteAddressBuffer           DstBuffer           : register(u0);
[[vk::binding(3, 0)]] RWByteAddressBuffer           ScratchGlobal       : register(u1);

#define PREFIX_SUM_THREADGROUP_SIZE 1024
groupshared uint SharedMem[PREFIX_SUM_THREADGROUP_SIZE];

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(PREFIX_SUM_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void CountTrianglePairsPrefixSum(
    in uint globalID : SV_DispatchThreadID,
    in uint localID  : SV_GroupThreadID)
{
    uint globalCount = 0;

    // Compute wave size aligned block count
    const uint alignedBlockCount = Pow2Align(ShaderRootConstants.numBlocks, WaveGetLaneCount());

    // Compute maximum number of active waves within this workgroup. This is used to clamp out of bounds read from
    // LDS below in case where numBlocks < PREFIX_SUM_THREADGROUP_COUNT
    const uint maxActiveWavesPerThreadGroup =
        min(alignedBlockCount, PREFIX_SUM_THREADGROUP_SIZE) / WaveGetLaneCount();

    // Each lane represents a wave worth of primitives for source geometries.
    //
    // geom_0_wave_0
    // geom_0_wave_1
    // ...
    // geom_1_wave_0
    // geom_1_wave_1
    // ...
    for (uint blockID = localID; blockID < ShaderRootConstants.numBlocks; blockID += PREFIX_SUM_THREADGROUP_SIZE)
    {
        const uint primRefCountOffset = ShaderRootConstants.basePrimRefCountOffset + blockID * sizeof(uint);

        const uint primRefCount = DstBuffer.Load(primRefCountOffset);

        // Prefix sum primitive reference count across geometries within this wave
        const uint laneOffset = WavePrefixSum(primRefCount);

        // Store partial result for each wave in group shared memory indexed by the global wave ID
        const uint waveID = localID / WaveGetLaneCount();
        SharedMem[waveID] = WaveActiveSum(primRefCount);

        GroupMemoryBarrierWithGroupSync();

        // Remap lanes to waves in this thread group with partial sum
        const uint laneID = WaveGetLaneIndex();

        const uint waveCount = (laneID < maxActiveWavesPerThreadGroup) ? SharedMem[laneID] : 0;
        const uint waveOffset = WavePrefixSum(waveCount);

        const uint threadGroupOffset = laneOffset + WaveReadLaneAt(waveOffset, waveID);
        const uint threadGroupCount = WaveActiveSum(waveCount);

        const uint blockOffset = globalCount + threadGroupOffset;
        DstBuffer.Store(primRefCountOffset, blockOffset);

        globalCount += threadGroupCount;
    }

    // Update primitive reference count
    if (globalID == 0)
    {
        ScratchGlobal.Store(
            ShaderRootConstants.encodeTaskCounterScratchOffset + ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET, globalCount);
    }
}
