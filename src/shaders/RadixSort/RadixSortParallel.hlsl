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
#include "ScanCommon.hlsli"
#include "BitHistogram.hlsl"
#include "ScanExclusiveInt4.hlsl"
#include "ScanExclusivePartInt4.hlsl"
#include "DistributePartSumInt4.hlsl"
#include "ScatterKeysAndValues.hlsl"
#include "ScanExclusiveInt4DLB.hlsl"

//======================================================================================================================
// Radix sort configuration constants
static const uint WorkGroupSize            = GROUP_SIZE;
static const uint BitsPerPass              = 4;
static const uint NumBins                  = 1u << BitsPerPass;
static const uint NumPasses30              = 32 / BitsPerPass;
static const uint NumPasses                = 64 / BitsPerPass;
static const uint BlocksPerGroup           = 1;
static const uint NumScanElemsPerWorkItem  = 8;
static const uint NumScanElemsPerWorkGroup = (WorkGroupSize * NumScanElemsPerWorkItem);
static const uint GroupBlockSize           = (WorkGroupSize * BitsPerPass * BlocksPerGroup);
static const uint ScanThresholdOneLevel    = NumScanElemsPerWorkGroup;
static const uint ScanThresholdTwoLevel    = ScanThresholdOneLevel * NumScanElemsPerWorkGroup;
static const uint ScanThresholdThreeLevel  = ScanThresholdTwoLevel * NumScanElemsPerWorkGroup;
static const uint GroupBlockSizeScan       = (WorkGroupSize << 3);
static const uint GroupBlockSizeDistribute = (WorkGroupSize << 2);

//======================================================================================================================
// Swap two uint variables
void SwapUint(
    inout uint x,
    inout uint y)
{
    const uint temp = x;
    x = y;
    y = temp;
}

//======================================================================================================================
void BitHistogram(
    uint localId,
    uint groupId,
    uint totalGroups,
    uint bitShiftCount,
    uint inputOffset,
    uint numPrimitives,
    uint useMortonCode30)
{
    for (; groupId < totalGroups; groupId += ShaderRootConstants.numThreadGroups)
    {
        BitHistogramImpl(
            localId,
            groupId,
            totalGroups,
            numPrimitives,
            bitShiftCount,
            inputOffset,
            ShaderConstants.offsets.histogram,
            useMortonCode30);
        GroupMemoryBarrierWithGroupSync();
    }
}

//======================================================================================================================
// Compute exclusive prefix sum of the histogram data
void ScanExclusiveAdd(
    inout uint  numTasksWait,
    inout uint  waveId,
    uint        globalId,
    uint        localId,
    uint        groupId,
    uint        numElements,
    uint        radixSortScanLevel)
{
    if (radixSortScanLevel == 0) // use DLB scan
    {
        const uint blockSize = WorkGroupSize;
        const uint elementsPerBlock = blockSize * DLB_KEYS_PER_THREAD;
        const uint numBlocks = RoundUpQuotient(numElements, elementsPerBlock);
        const uint numInitGroups = RoundUpQuotient(numBlocks, WorkGroupSize);

        BEGIN_TASK(numInitGroups);

        InitScanExclusiveInt4DLBImpl(globalId,
                                     numElements,
                                     ShaderConstants.offsets.dynamicBlockIndex,
                                     ShaderConstants.offsets.prefixSumAtomicFlags);

        END_TASK(numInitGroups);

        BEGIN_TASK(numBlocks);

        ScanExclusiveInt4DLBImpl(globalId,
                                 localId,
                                 numElements,
                                 ShaderConstants.offsets.dynamicBlockIndex,
                                 ShaderConstants.offsets.prefixSumAtomicFlags,
                                 ShaderConstants.offsets.histogram);

        END_TASK(numBlocks);
    }
    else if (radixSortScanLevel == 1)
    {
        BEGIN_TASK(ShaderRootConstants.numThreadGroups);

        ScanExclusiveInt4Impl(globalId, localId, ShaderConstants.offsets.histogram, numElements);

        END_TASK(ShaderRootConstants.numThreadGroups);
    }
    else if (radixSortScanLevel == 2)
    {
        const uint numGroupsBottomLevelScan       = RoundUpQuotient(numElements, GroupBlockSizeScan);
        const uint numGroupsTopLevelScan          = RoundUpQuotient(numGroupsBottomLevelScan, GroupBlockSizeScan);
        const uint numGroupsBottomLevelDistribute = RoundUpQuotient(numElements, GroupBlockSizeDistribute);

        const uint partialSumsOffset = ShaderConstants.offsets.distributedPartialSums;
        const uint histogramOffset   = ShaderConstants.offsets.histogram;

        BEGIN_TASK(numGroupsBottomLevelScan);

        // Bottom level partial sum
        ScanExclusivePartialInt4Impl(
            groupId * WorkGroupSize + localId,
            localId,
            groupId,
            histogramOffset,
            partialSumsOffset,
            numElements);
        GroupMemoryBarrierWithGroupSync();

        END_TASK(numGroupsBottomLevelScan);

        // Top level sum of partial sums
        BEGIN_TASK(numGroupsTopLevelScan);

        ScanExclusiveInt4Impl(
            groupId * WorkGroupSize + localId,
            localId,
            partialSumsOffset,
            numGroupsBottomLevelScan);
        GroupMemoryBarrierWithGroupSync();

        END_TASK(numGroupsTopLevelScan);

        // Distribute top level sums to bottom level
        BEGIN_TASK(numGroupsBottomLevelDistribute);

        DistributePartSumInt4Impl(
            groupId * WorkGroupSize + localId,
            groupId,
            histogramOffset,
            partialSumsOffset,
            numElements);

        END_TASK(numGroupsBottomLevelDistribute);
    }
    else if (radixSortScanLevel == 3)
    {
        const uint numGroupsBottomLevelScan       = RoundUpQuotient(numElements, GroupBlockSizeScan);
        const uint numGroupsMidLevelScan          = RoundUpQuotient(numGroupsBottomLevelScan, GroupBlockSizeScan);
        const uint numGroupsTopLevelScan          = RoundUpQuotient(numGroupsMidLevelScan, GroupBlockSizeScan);
        const uint numGroupsBottomLevelDistribute = RoundUpQuotient(numElements, GroupBlockSizeDistribute);
        const uint numGroupsMidLevelDistribute    = RoundUpQuotient(numGroupsBottomLevelDistribute, GroupBlockSizeDistribute);

        const uint partSumsBottomLevelOffset = ShaderConstants.offsets.distributedPartialSums;
        const uint partSumsMidLevelOffset    = partSumsBottomLevelOffset + (numGroupsBottomLevelScan * sizeof(uint));
        const uint histogramOffset           = ShaderConstants.offsets.histogram;

        uint groupId;

        // Bottom level partial sum
        BEGIN_TASK(numGroupsBottomLevelScan);

        ScanExclusivePartialInt4Impl(
            groupId * WorkGroupSize + localId,
            localId,
            groupId,
            histogramOffset,
            partSumsBottomLevelOffset,
            numElements);
        GroupMemoryBarrierWithGroupSync();

        END_TASK(numGroupsBottomLevelScan);

        // Middle level partial sum
        BEGIN_TASK(numGroupsMidLevelScan);

        ScanExclusivePartialInt4Impl(
            groupId * WorkGroupSize + localId,
            localId,
            groupId,
            partSumsBottomLevelOffset,
            partSumsMidLevelOffset,
            numGroupsBottomLevelScan);
        GroupMemoryBarrierWithGroupSync();

        END_TASK(numGroupsMidLevelScan);

        // Top level sum of partial sums
        BEGIN_TASK(numGroupsTopLevelScan);

        ScanExclusiveInt4Impl(
            groupId * WorkGroupSize + localId,
            localId,
            partSumsMidLevelOffset,
            numGroupsMidLevelScan);
        GroupMemoryBarrierWithGroupSync();

        END_TASK(numGroupsTopLevelScan);

        // Distribute top level sums to middle level
        BEGIN_TASK(numGroupsMidLevelDistribute);

        DistributePartSumInt4Impl(
            groupId * WorkGroupSize + localId,
            groupId,
            partSumsBottomLevelOffset,
            partSumsMidLevelOffset,
            numGroupsBottomLevelDistribute);

        END_TASK(numGroupsMidLevelDistribute);

        // Distribute middle level sums to bottom level
        BEGIN_TASK(numGroupsBottomLevelDistribute);

        DistributePartSumInt4Impl(
            groupId * WorkGroupSize + localId,
            groupId,
            histogramOffset,
            partSumsBottomLevelOffset,
            numElements);

        END_TASK(numGroupsBottomLevelDistribute);
    }
}

//======================================================================================================================
void ScatterKeysAndValues(
    uint groupId,
    uint localId,
    uint bitShiftSize,
    uint totalNumGroups,
    uint inputKeysOffset,
    uint inputValuesOffset,
    uint outputKeysOffset,
    uint outputValuesOffset,
    uint numPrimitives,
    uint useMortonCode30)
{
    for (; groupId < totalNumGroups; groupId += ShaderRootConstants.numThreadGroups)
    {
        ScatterKeysAndValuesImpl(
            groupId,
            localId,
            bitShiftSize,
            numPrimitives,
            totalNumGroups,
            inputKeysOffset,
            inputValuesOffset,
            ShaderConstants.offsets.histogram,
            outputKeysOffset,
            outputValuesOffset,
            useMortonCode30);
    }
}

//======================================================================================================================
void RadixSort(
    inout uint numTasksWait,
    inout uint waveId,
    uint       globalId,
    uint       localId,
    uint       groupId,
    uint       numPrimitives,
    uint       radixSortScanLevel,
    uint       useMortonCode30)
{
    const uint inputKeysOffset    = ShaderConstants.offsets.mortonCodes;
    const uint outputKeysOffset   = ShaderConstants.offsets.mortonCodesSorted;
    // Passing in 0 for initial primitive indices since we auto-generate them for the first pass
    const uint inputValuesOffset  = 0;
    const uint outputValuesOffset = ShaderConstants.offsets.primIndicesSorted;

    const uint scratchKeysOffset   = ShaderConstants.offsets.tempKeys;
    const uint scratchValuesOffset = ShaderConstants.offsets.tempVals;

    const uint numElements = numPrimitives;

    const uint totalNumGroups       = RoundUpQuotient(numElements, GroupBlockSize);
    const uint numHistogramElements = totalNumGroups * NumBins;

    uint currentFromKeys = inputKeysOffset;
    uint currentFromVals = inputValuesOffset;
    uint currentToKeys   = scratchKeysOffset;
    uint currentToVals   = scratchValuesOffset;

    uint bitShiftCount = 0;

    const uint passes = (useMortonCode30) ? NumPasses30 : NumPasses;

    for (uint passIdx = 0; passIdx < passes; passIdx++)
    {
        BEGIN_TASK(ShaderRootConstants.numThreadGroups);

        BitHistogram(localId, groupId, totalNumGroups, bitShiftCount, currentFromKeys, numPrimitives, useMortonCode30);

        END_TASK(ShaderRootConstants.numThreadGroups);

        ScanExclusiveAdd(numTasksWait, waveId, globalId, localId, groupId, numHistogramElements, radixSortScanLevel);

        BEGIN_TASK(ShaderRootConstants.numThreadGroups);

        ScatterKeysAndValues(
            groupId,
            localId,
            bitShiftCount,
            totalNumGroups,
            currentFromKeys,
            currentFromVals,
            currentToKeys,
            currentToVals,
            numPrimitives,
            useMortonCode30);

        END_TASK(ShaderRootConstants.numThreadGroups);

        // The first pass uses the input buffer. Swap between output and temp resources for later passes.
        if (bitShiftCount == 0)
        {
            currentFromKeys = outputKeysOffset;
            currentFromVals = outputValuesOffset;
        }

        SwapUint(currentFromKeys, currentToKeys);
        SwapUint(currentFromVals, currentToVals);

        bitShiftCount += BitsPerPass;
    }
}
