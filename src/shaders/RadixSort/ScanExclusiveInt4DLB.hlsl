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
#define RootSig "RootConstants(num32BitConstants=2, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "CBV(b1),"\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u6, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u7, visibility=SHADER_VISIBILITY_ALL)"

//=====================================================================================================================
struct RootConstants
{
    uint NumElements;
    uint passIndex;
};
#include "../../shared/rayTracingDefs.h"

[[vk::push_constant]] ConstantBuffer<RootConstants> ShaderRootConstants    : register(b0);
[[vk::binding(1, 1)]] ConstantBuffer<BuildShaderConstants> ShaderConstants : register(b1);

[[vk::binding(0, 0)]] RWByteAddressBuffer                  SrcBuffer           : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer                  DstBuffer           : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer                  DstMetadata         : register(u2);
[[vk::binding(3, 0)]] globallycoherent RWByteAddressBuffer ScratchBuffer       : register(u3);
[[vk::binding(4, 0)]] globallycoherent RWByteAddressBuffer ScratchGlobal       : register(u4);
[[vk::binding(5, 0)]] RWByteAddressBuffer                  InstanceDescBuffer  : register(u5);
[[vk::binding(6, 0)]] RWByteAddressBuffer                  EmitBuffer          : register(u6);
[[vk::binding(7, 0)]] RWByteAddressBuffer                  IndirectArgBuffer   : register(u7);

#include "ScanCommon.hlsli"
#include "../BuildCommon.hlsl"

//=====================================================================================================================
groupshared int SharedMem[GROUP_SIZE];

#include "ScanExclusiveInt4DLBCommon.hlsl"
#endif

//=====================================================================================================================
void InitScanExclusiveInt4DLBImpl(
    uint globalId,
    uint numElements,
    uint dynamicBlockIndexScratchOffset,
    uint atomicFlagsScratchOffset)
{
    if (globalId == 0)
    {
        ScratchBuffer.Store(dynamicBlockIndexScratchOffset, 0);
    }

    const uint numBlocks = RoundUpQuotient(numElements, DLB_KEYS_PER_GROUP);

    if (globalId < numBlocks)
    {
        for (uint type = 0; type < NUM_DLB_VALID_TYPES; type++)
        {
            Flags flags;
            flags.dataValid = 0;
            flags.prefixSum = 0;

            WriteFlagsDLB(atomicFlagsScratchOffset, globalId, type, flags);
        }
    }
}

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(GROUP_SIZE, 1, 1)]
void InitScanExclusiveInt4DLB(
    in uint3 globalThreadID : SV_DispatchThreadID,
    in uint3 localThreadID  : SV_GroupThreadID,
    in uint3 groupID        : SV_GroupID)
{
    InitScanExclusiveInt4DLBImpl(globalThreadID.x,
                                 ShaderRootConstants.NumElements,
                                 ShaderConstants.offsets.dynamicBlockIndex,
                                 ShaderConstants.offsets.prefixSumAtomicFlags);
}
#endif

//=====================================================================================================================
void ScanExclusiveInt4DLBImpl(
    uint globalId,
    uint localId,
    uint numElements,
    uint iterationIndex,
    uint dynamicBlockIndexScratchOffset,
    uint atomicFlagsScratchOffset,
    uint inOutArrayScratchOffset)
{
    const uint currIterationValidValue = iterationIndex + 1;

    uint blockId;
    if (localId == 0)
    {
        ScratchBuffer.InterlockedAdd(dynamicBlockIndexScratchOffset, 1, blockId);

        const uint blockSize = GROUP_SIZE;
        const uint elementsPerBlock = blockSize * DLB_KEYS_PER_THREAD;
        const uint numBlocks = RoundUpQuotient(numElements, elementsPerBlock);
        const uint prevBlocksCompleted = iterationIndex * numBlocks;

        SharedMem[0] = blockId - prevBlocksCompleted;
    }

    GroupMemoryBarrierWithGroupSync();

    blockId = SharedMem[0];

    int localKeys[DLB_KEYS_PER_THREAD];

    // Perform coalesced load into LDS
    uint rangeBegin = blockId * GROUP_SIZE * DLB_KEYS_PER_THREAD;

    for (uint i = 0; i < DLB_KEYS_PER_THREAD; i += 4)
    {
        uint loadIndex = rangeBegin + (localId * DLB_KEYS_PER_THREAD) + i;

        uint loadIndex4 = loadIndex / 4;

        uint4 value = safe_load_int4(inOutArrayScratchOffset,
                                     loadIndex4,
                                     numElements);

        localKeys[i]     = value.x;
        localKeys[i + 1] = value.y;
        localKeys[i + 2] = value.z;
        localKeys[i + 3] = value.w;
    }

    int threadSum = 0;

    // Calculate scan on this thread's elements
    for (uint i = 0; i < DLB_KEYS_PER_THREAD; ++i)
    {
        int tmp = localKeys[i];
        localKeys[i] = threadSum;
        threadSum += tmp;
    }

    // Scan partial sums
    int threadSumScanned = BlockScanExclusiveAdd(threadSum, localId);

    // Add partial sums back
    for (uint i = 0; i < DLB_KEYS_PER_THREAD; ++i)
    {
        localKeys[i] += threadSumScanned;
    }

    // Wait until previous dynamic block finished and acquire its sum.
    int prevSum = 0;
    if (blockId > 0)
    {
        if (localId == (GROUP_SIZE - 1))
        {
            // write out the sum
            Flags sum;
            sum.prefixSum = threadSumScanned + threadSum;
            sum.dataValid = currIterationValidValue;

            WriteFlagsDLB(atomicFlagsScratchOffset, blockId, DLB_VALID_SUM, sum);

            int readBackIndex = blockId - 1;

            // checks previous blocks for prefix sum
            // if the prefix sum is valid then it breaks out of loop
            // if the prefix sum is NOT valid, then it fetches the SUM and continues to move backwards
            while (readBackIndex >= 0)
            {
                if (ReadValidDLB(atomicFlagsScratchOffset, readBackIndex, DLB_VALID_PREFIX_SUM) == currIterationValidValue)
                {
                    prevSum += ReadPrefixSumDLB(atomicFlagsScratchOffset, readBackIndex, DLB_VALID_PREFIX_SUM);

                    Flags flags;
                    flags.prefixSum = prevSum + threadSumScanned + threadSum;
                    flags.dataValid = currIterationValidValue;

                    WriteFlagsDLB(atomicFlagsScratchOffset, blockId, DLB_VALID_PREFIX_SUM, flags);
                    break;
                }
                else if (ReadValidDLB(atomicFlagsScratchOffset, readBackIndex, DLB_VALID_SUM) == currIterationValidValue)
                {
                    prevSum += ReadPrefixSumDLB(atomicFlagsScratchOffset, readBackIndex, DLB_VALID_SUM);
                    readBackIndex--;
                }

                DeviceMemoryBarrier();
            }

            SharedMem[0] = prevSum;
        }

        GroupMemoryBarrierWithGroupSync();

        prevSum = SharedMem[0];
    }     // Write our sum and enable successive blocks.
    else if (localId == (GROUP_SIZE - 1))
    {
        Flags flags;
        flags.prefixSum = threadSumScanned + threadSum;
        flags.dataValid = currIterationValidValue;

        WriteFlagsDLB(atomicFlagsScratchOffset, blockId, DLB_VALID_PREFIX_SUM, flags);
    }

    GroupMemoryBarrierWithGroupSync();

    // Perform coalesced writes back to global memory
    for (uint i = 0; i < DLB_KEYS_PER_THREAD; i += 4)
    {
        uint storeIndex = rangeBegin + localId * DLB_KEYS_PER_THREAD + i;

        int4 value = int4(localKeys[i], localKeys[i + 1], localKeys[i + 2], localKeys[i + 3]);

        if (storeIndex < numElements)
        {
            const uint storeIndex4 = storeIndex / 4;
            const uint storeOffset = inOutArrayScratchOffset + (storeIndex4 * sizeof(int4));

            ScratchBuffer.Store<int4>(storeOffset, value + int4(prevSum, prevSum, prevSum, prevSum));
        }
    }
}

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(GROUP_SIZE, 1, 1)]
void ScanExclusiveInt4DLB(
    in uint3 globalThreadID : SV_DispatchThreadID,
    in uint3 localThreadID  : SV_GroupThreadID,
    in uint3 groupID        : SV_GroupID)
{
    ScanExclusiveInt4DLBImpl(globalThreadID.x,
                             localThreadID.x,
                             ShaderRootConstants.NumElements,
                             ShaderRootConstants.passIndex,
                             ShaderConstants.offsets.dynamicBlockIndex,
                             ShaderConstants.offsets.prefixSumAtomicFlags,
                             ShaderConstants.offsets.histogram);
}
#endif
