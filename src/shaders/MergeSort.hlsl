/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2022 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "Common.hlsl"
#include "BuildCommon.hlsl"

//=====================================================================================================================
// Root signature
#define RootSig "RootConstants(num32BitConstants=6, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL)"

//=====================================================================================================================
// 32 bit constants
struct Constants
{
    uint NumPrimitives;
    uint InputKeysScratchOffset;
    uint OutputKeysScratchOffset;
    uint OutputValuesScratchOffset;
    uint OutputValuesSwapScratchOffset;
    uint UseMortonCode30;
};

[[vk::push_constant]] ConstantBuffer<Constants> ShaderConstants           : register(b0);

//=====================================================================================================================
[[vk::binding(0, 0)]] RWByteAddressBuffer ResultBuffer                    : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer ResultMetadata : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer ScratchBuffer  : register(u2);

#define MAX_LDS_ELEMENTS (16 * BUILD_THREADGROUP_SIZE)
groupshared int SharedMem[MAX_LDS_ELEMENTS];
groupshared uint SharedIndex;
#endif

//=====================================================================================================================
uint NumElemsLessThan(uint val, uint offset, uint offsetNext, uint keysOffset)
{
    uint l = offset;
    uint r = offsetNext;

    while (l < r)
    {
        const uint index = (l + r) / 2;
        if (FetchMortonCode(ScratchBuffer, keysOffset, index) < val)
        {
            l = index + 1;
        }
        else
        {
            r = index;
        }
    }
    return l - offset;
}

//=====================================================================================================================
uint NumElemsLessThan64(uint64_t val, uint offset, uint offsetNext, uint keysOffset)
{
    uint l = offset;
    uint r = offsetNext;

    while (l < r)
    {
        const uint index = (l + r) / 2;
        if (FetchMortonCode64(ScratchBuffer, keysOffset, index) < val)
        {
            l = index + 1;
        }
        else
        {
            r = index;
        }
    }
    return l - offset;
}

//=====================================================================================================================
uint NumElemsLessThanOrEqualTo(uint val, uint offset, uint offsetNext, uint keysOffset)
{
    uint l = offset;
    uint r = offsetNext;

    while (l < r)
    {
        const uint index = (l + r) / 2;
        if (FetchMortonCode(ScratchBuffer, keysOffset, index) <= val)
        {
            l = index + 1;
        }
        else
        {
            r = index;
        }
    }
    return l - offset;
}

//=====================================================================================================================
uint NumElemsLessThanOrEqualTo64(uint64_t val, uint offset, uint offsetNext, uint keysOffset)
{
    uint l = offset;
    uint r = offsetNext;

    while (l < r)
    {
        const uint index = (l + r) / 2;
        if (FetchMortonCode64(ScratchBuffer, keysOffset, index) <= val)
        {
            l = index + 1;
        }
        else
        {
            r = index;
        }
    }
    return l - offset;
}

//=====================================================================================================================
void GlobalMerge(
    inout uint numTasksWait,
    inout uint waveId,
    uint localId,
    uint groupId,
    uint numPrimitives,
    uint groupCapacity,
    uint activeGroups,
    uint groupSize,
    uint outputKeysOffset,
    uint outputValuesOffset,
    uint keysOffsetSwap,
    uint valuesOffsetSwap,
    uint useMortonCode30)
{
    const uint numLevelsOfMergeTree = ceil(log2(activeGroups));
    activeGroups  = RoundUpQuotient(numPrimitives, groupSize);
    uint cmpGap   = 1;
    uint splitGap = 2;

    // Level 0 is the sorted partitions
    for (uint level = 1; level <= numLevelsOfMergeTree; level++)
    {
        BEGIN_TASK(activeGroups);
        const uint groupIdNew = groupId / 2;
        const uint capacity   = cmpGap * groupCapacity;
        bool leftSubtree      = true;
        uint subtreeOffset;
        uint subtreeEnd;

        // Left Subtree
        if (groupIdNew % splitGap < cmpGap)
        {
            subtreeOffset = capacity * (groupIdNew / cmpGap) + capacity;
            subtreeEnd    = (subtreeOffset + capacity > numPrimitives) ? numPrimitives : subtreeOffset + capacity;
        }
        // Right Subtree
        else
        {
            subtreeEnd    = (groupIdNew / cmpGap) * capacity;
            subtreeOffset = subtreeEnd - capacity;
            leftSubtree   = false;
        }

        if (globalId < numPrimitives)
        {
            if (useMortonCode30)
            {
                const uint mortonCode = FetchMortonCode(ScratchBuffer, outputKeysOffset, globalId);
                const uint index      = FetchSortedPrimIndex(ScratchBuffer, outputValuesOffset, globalId);

                uint posInMergedList  =  localId + (groupId % splitGap) * groupSize + (groupIdNew / splitGap) * capacity * 2;

                posInMergedList += (leftSubtree) ? NumElemsLessThan(mortonCode, subtreeOffset, subtreeEnd, outputKeysOffset) :
                                                   NumElemsLessThanOrEqualTo(mortonCode, subtreeOffset, subtreeEnd, outputKeysOffset);

                WriteMortonCode(ScratchBuffer, keysOffsetSwap, posInMergedList, mortonCode);
                WriteSortedPrimIndex(ScratchBuffer, valuesOffsetSwap, posInMergedList, index);
            }
            else
            {
                const uint64_t mortonCode = FetchMortonCode64(ScratchBuffer, outputKeysOffset, globalId);
                const uint index          = FetchSortedPrimIndex(ScratchBuffer, outputValuesOffset, globalId);

                uint posInMergedList      =  localId + (groupId % splitGap) * groupSize + (groupIdNew / splitGap) * capacity * 2;

                posInMergedList += (leftSubtree) ? NumElemsLessThan64(mortonCode, subtreeOffset, subtreeEnd, outputKeysOffset) :
                                                   NumElemsLessThanOrEqualTo64(mortonCode, subtreeOffset, subtreeEnd, outputKeysOffset);

                WriteMortonCode64(ScratchBuffer, keysOffsetSwap, posInMergedList, mortonCode);
                WriteSortedPrimIndex(ScratchBuffer, valuesOffsetSwap, posInMergedList, index);
            }
        }
        END_TASK(activeGroups);

        splitGap <<= 1;
        cmpGap   <<= 1;

        BEGIN_TASK(activeGroups);
        if (globalId < numPrimitives)
        {
            if (useMortonCode30)
            {
                const uint mortonCode = FetchMortonCode(ScratchBuffer, keysOffsetSwap, globalId);
                WriteMortonCode(ScratchBuffer, outputKeysOffset, globalId, mortonCode);
            }
            else
            {
                const uint64_t mortonCode = FetchMortonCode64(ScratchBuffer, keysOffsetSwap, globalId);
                WriteMortonCode64(ScratchBuffer, outputKeysOffset, globalId, mortonCode);
            }

            const uint index = FetchSortedPrimIndex(ScratchBuffer, valuesOffsetSwap, globalId);
            WriteSortedPrimIndex(ScratchBuffer, outputValuesOffset, globalId, index);
        }
        END_TASK(activeGroups);
    }
}

//=====================================================================================================================
uint64_t FetchMortonCode64LDS(uint i)
{
    return (((uint64_t) SharedMem[(i << 1) + 1]) << 32) | (((uint64_t) SharedMem[i << 1]) & 0x00000000FFFFFFFF);
}

//=====================================================================================================================
void WriteMortonCode64LDS(uint i, uint64_t mortonCode)
{
    SharedMem[i << 1]       = uint(mortonCode);
    SharedMem[(i << 1) + 1] = uint(mortonCode >> 32);
}

//=====================================================================================================================
uint2 Fetch2MortonCodes(uint mortonCodesOffset, uint2 span, uint gId, uint numPrimitives)
{
    // Morton Codes can be 0x3fffffff maximum in 32-bit so INT_MAX is safe.
    uint2 res = uint2(INT_MAX, INT_MAX);

    if (span.y <= numPrimitives)
    {
        res = ScratchBuffer.Load<uint2>(mortonCodesOffset + gId * sizeof(uint2));
    }
    else if (span.x < numPrimitives)
    {
        res.x = ScratchBuffer.Load(mortonCodesOffset + gId * sizeof(uint2));
    }
    return res;
}

//=====================================================================================================================
uint64_t2 Fetch2MortonCodes64(uint mortonCodesOffset, uint2 span, uint gId, uint numPrimitives)
{
    // Morton Codes can be 0x7fffffffffffffff maximum in 64-bit.
    // Bug if that is the case. We handle that in GenerateMortonCodesImpl()
    uint64_t2 res = uint64_t2(0x7FFFFFFFFFFFFFFF, 0x7FFFFFFFFFFFFFFF);

    if (span.y <= numPrimitives)
    {
        res = ScratchBuffer.Load<uint64_t2>(mortonCodesOffset + gId * sizeof(uint64_t2));
    }
    else if (span.x < numPrimitives)
    {
        res.x = ScratchBuffer.Load<uint64_t>(mortonCodesOffset + gId * sizeof(uint64_t2));
    }
    return res;
}

//=====================================================================================================================
void Store2MortonCodes(uint mortonCodesOffset, uint2 span, uint2 mortonCodeIndex, uint gId, uint ldsOffset, uint numPrimitives)
{
    if (span.y <= numPrimitives)
    {
        const int2 val = int2(SharedMem[ldsOffset + mortonCodeIndex.x], SharedMem[ldsOffset + mortonCodeIndex.y]);
        ScratchBuffer.Store<int2>(mortonCodesOffset + gId * sizeof(int2), val);
    }
    else if (span.x < numPrimitives)
    {
        const int val = SharedMem[ldsOffset + mortonCodeIndex.x];
        ScratchBuffer.Store(mortonCodesOffset + gId * sizeof(int2), val);
    }
}

//=====================================================================================================================
void Store2MortonCodes64(uint mortonCodesOffset, uint2 span, uint2 mortonCodeIndex, uint gId, uint numPrimitives)
{
    if (span.y <= numPrimitives)
    {
        const uint64_t2 val = uint64_t2(FetchMortonCode64LDS(mortonCodeIndex.x), FetchMortonCode64LDS(mortonCodeIndex.y));
        ScratchBuffer.Store<uint64_t2>(mortonCodesOffset + gId * sizeof(uint64_t2), val);
    }
    else if (span.x < numPrimitives)
    {
        const uint64_t val = FetchMortonCode64LDS(mortonCodeIndex.x);
        ScratchBuffer.Store<uint64_t>(mortonCodesOffset + gId * sizeof(uint64_t2), val);
    }
}

//=====================================================================================================================
void SwapKeyValue(uint i, uint j, uint mortonCode, uint mortonCodeCmp, uint indicesOffset)
{
    SharedMem[i] = mortonCodeCmp;
    SharedMem[j] = mortonCode;

    const uint index = SharedMem[indicesOffset + i];
    SharedMem[indicesOffset + i] = SharedMem[indicesOffset + j];
    SharedMem[indicesOffset + j] = index;
}

//=====================================================================================================================
void SwapKeyValue64(uint i, uint j, uint64_t mortonCode, uint64_t mortonCodeCmp, uint indicesOffset)
{
    WriteMortonCode64LDS(i, mortonCodeCmp);
    WriteMortonCode64LDS(j, mortonCode);

    const uint index = SharedMem[indicesOffset + i];
    SharedMem[indicesOffset + i] = SharedMem[indicesOffset + j];
    SharedMem[indicesOffset + j] = index;
}

//=====================================================================================================================
void Ascend(uint i, uint j, uint indicesOffset, uint useMortonCode30)
{
    if (useMortonCode30)
    {
        const uint mortonCode    = SharedMem[i];
        const uint mortonCodeCmp = SharedMem[j];

        if (mortonCode > mortonCodeCmp)
        {
            SwapKeyValue(i, j, mortonCode, mortonCodeCmp, indicesOffset);
        }
    }
    else
    {
        const uint64_t mortonCode    = FetchMortonCode64LDS(i);
        const uint64_t mortonCodeCmp = FetchMortonCode64LDS(j);

        if (mortonCode > mortonCodeCmp)
        {
            SwapKeyValue64(i, j, mortonCode, mortonCodeCmp, indicesOffset);
        }
    }
}

//=====================================================================================================================
void OddEvenMergeSort(uint localId, uint groupSize, uint partitionSize, uint indicesOffset, uint useMortonCode30)
{
    const uint numSteps = log2(partitionSize);

    for (uint i = 0; i < numSteps; i++)
    {
        uint cmpGap   = 1U << i;
        uint splitGap = 1U << (i + 1);
        const uint splitGapFixed = splitGap;

        for (uint j = 0; j < i + 1; j++)
        {
            uint localIndex = localId % splitGap < cmpGap ? localId : localId + groupSize - cmpGap;

            if (localIndex < partitionSize)
            {
                if (j == 0)
                {
                    Ascend(localIndex, localIndex + cmpGap, indicesOffset, useMortonCode30);
                }
                else
                {
                    localIndex += cmpGap;
                    if (localIndex + cmpGap < (localIndex / splitGapFixed + 1) * splitGapFixed)
                    {
                        Ascend(localIndex, localIndex + cmpGap, indicesOffset, useMortonCode30);
                    }
                }
            }
            GroupMemoryBarrierWithGroupSync();

            cmpGap   >>= 1;
            splitGap >>= 1;
        }
    }
}

//=====================================================================================================================
void FetchAndLocalSortAndWriteBack(
    uint localId,
    uint groupId,
    uint globalId,
    uint numPrimitives,
    uint groupCapacity,
    uint groupSize,
    uint inputKeysOffset,
    uint outputKeysOffset,
    uint outputValuesOffset,
    uint useMortonCode30)
{
    // Step 1) Fetch the keys (Morton Codes) to LDS for local sort.
    const uint2 globalFetchSpan = uint2(globalId << 1, (globalId + 1) << 1);
    const uint2 mortonCodeIndex = uint2(localId << 1, (localId << 1) + 1);
    if (useMortonCode30)
    {
        // Assign uint to int. Might break if a Morton Codes's MSB is 1.
        // But it is okay because Morton Codes' MSB is always 0: 00XYZXYZ.. in 32-bit and 0XYZXYZ.. in 64-bit.
        const uint2 res = Fetch2MortonCodes(inputKeysOffset, globalFetchSpan, globalId, numPrimitives);
        SharedMem[mortonCodeIndex.x] = res.x;
        SharedMem[mortonCodeIndex.y] = res.y;
    }
    else
    {
        const uint64_t2 res = Fetch2MortonCodes64(inputKeysOffset, globalFetchSpan, globalId, numPrimitives);
        WriteMortonCode64LDS(mortonCodeIndex.x, res.x);
        WriteMortonCode64LDS(mortonCodeIndex.y, res.y);
    }

    // Step 2) Initialize the values (Morton Code indices).
    const uint pSize = numPrimitives / ((groupId + 1) * groupCapacity) ? groupCapacity : numPrimitives % groupCapacity;
    const uint partitionSize = firstbithigh(pSize) == firstbitlow(pSize) ? pSize : 1U << (firstbithigh(pSize) + 1);
    // Extend the partition size to the nearest n^2 for OddEvenMerge Sort

    const uint indicesOffset = (useMortonCode30) ? partitionSize : partitionSize * 2;
    const uint groupOffset   = groupId * groupCapacity;

    SharedMem[indicesOffset + mortonCodeIndex.x] = groupOffset + mortonCodeIndex.x;
    SharedMem[indicesOffset + mortonCodeIndex.y] = groupOffset + mortonCodeIndex.y;
    GroupMemoryBarrierWithGroupSync();

    // Step 3) Sort LDS-local Morton Code partitions.
    OddEvenMergeSort(localId, groupSize, partitionSize, indicesOffset, useMortonCode30);
    // Implicit sync at the end of the OddEvenMerge Sort.

    // Step 4) Write back LDS-local sorted Morton Code partitions to global memory.
    if (useMortonCode30)
    {
        Store2MortonCodes(outputKeysOffset, globalFetchSpan, mortonCodeIndex, globalId, 0, numPrimitives);
    }
    else
    {
        Store2MortonCodes64(outputKeysOffset, globalFetchSpan, mortonCodeIndex, globalId, numPrimitives);
    }

    Store2MortonCodes(outputValuesOffset, globalFetchSpan, mortonCodeIndex, globalId, indicesOffset, numPrimitives);
}

//=====================================================================================================================
void MergeSortImpl(
    inout uint numTasksWait,
    inout uint waveId,
    uint localId,
    uint groupId,
    uint numPrimitives,
    uint inputKeysOffset,
    uint outputKeysOffset,
    uint outputValuesOffset,
    uint valuesOffsetSwap,
    uint useMortonCode30)
{
    const uint groupCapacity = BUILD_THREADGROUP_SIZE * 2;
    const uint activeGroups  = RoundUpQuotient(numPrimitives, groupCapacity);

    BEGIN_TASK(activeGroups);
    FetchAndLocalSortAndWriteBack(localId,
                                  groupId,
                                  globalId,
                                  numPrimitives,
                                  groupCapacity,
                                  BUILD_THREADGROUP_SIZE,
                                  inputKeysOffset,
                                  outputKeysOffset,
                                  outputValuesOffset,
                                  useMortonCode30);
    END_TASK(activeGroups);

    GlobalMerge(numTasksWait,
                waveId,
                localId,
                groupId,
                numPrimitives,
                groupCapacity,
                activeGroups,
                BUILD_THREADGROUP_SIZE,
                outputKeysOffset,
                outputValuesOffset,
                inputKeysOffset,
                valuesOffsetSwap,
                useMortonCode30);
    // Implicit Global Sync at the end of GlobalMerge();
}

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
// Main Function : MergeSort
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void MergeSort(
    uint localId      : SV_GroupThreadID,
    uint groupId      : SV_GroupID)
{
    uint numTasksWait = 0;
    uint waveId       = 0;
    INIT_TASK;

    MergeSortImpl(numTasksWait,
                  waveId,
                  localId,
                  groupId,
                  ShaderConstants.NumPrimitives,
                  ShaderConstants.InputKeysScratchOffset,
                  ShaderConstants.OutputKeysScratchOffset,
                  ShaderConstants.OutputValuesScratchOffset,
                  ShaderConstants.OutputValuesSwapScratchOffset,
                  ShaderConstants.UseMortonCode30);
}
#endif