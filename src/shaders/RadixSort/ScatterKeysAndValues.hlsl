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
#define RootSig "RootConstants(num32BitConstants=2, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "CBV(b1),"\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "CBV(b255)"/*Build Settings binding*/

//=====================================================================================================================
// 32 bit constants
struct RootConstants
{
    uint BitShiftSize; // Number of bits to shift
    uint NumGroups;    // Number of work groups dispatched
};
#include "../../shared/rayTracingDefs.h"

[[vk::push_constant]] ConstantBuffer<RootConstants> ShaderRootConstants    : register(b0);
[[vk::binding(1, 1)]] ConstantBuffer<BuildShaderConstants> ShaderConstants : register(b1);

[[vk::binding(0, 0)]] RWByteAddressBuffer DstBuffer     : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer DstMetadata   : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer ScratchBuffer : register(u2);

// unused buffer
[[vk::binding(3, 0)]] RWByteAddressBuffer SrcBuffer     : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer EmitBuffer    : register(u4);

#include "ScanCommon.hlsli"
#include "../BuildCommon.hlsl"

//=====================================================================================================================
// Local memory for offsets counting
groupshared int SharedMem[SHARED_KEYS_OFFSET + GROUP_SIZE * 8];
#endif

static const uint SharedScannedHistogramOffset = GROUP_SIZE * 4;

//=====================================================================================================================
// Generate fixed input value indices
int4 safe_load_int4_intmax_fixed(uint idx, uint sizeInInts)
{
    const int val = idx << 2;
    int4 res = int4(INT_MAX, INT_MAX, INT_MAX, INT_MAX);
    if (((idx + 1) << 2) <= sizeInInts)
    {
        res.x = val;
        res.y = val + 1;
        res.z = val + 2;
        res.w = val + 3;
    }
    else
    {
        if ((idx << 2) < sizeInInts)
        {
            res.x = val;
        }
        if ((idx << 2) + 1 < sizeInInts)
        {
            res.y = val + 1;
        }
        if ((idx << 2) + 2 < sizeInInts)
        {
            res.z = val + 2;
        }
    }

    return res;
}

//=====================================================================================================================
void group_scan_exclusive_uint(int localId, int groupSize)
{
    int stride;
    for (stride = 1; stride <= (groupSize >> 1); stride <<= 1)
    {
        if (localId < groupSize/(2*stride))
        {
            SharedMem[2*(localId + 1)*stride-1] =
                SharedMem[2*(localId + 1)*stride-1] + SharedMem[(2*localId + 1)*stride-1];
        }

        GroupMemoryBarrierWithGroupSync();
    }

    if (localId == 0)
    {
        SharedMem[groupSize - 1] = 0;
    }

    GroupMemoryBarrierWithGroupSync();

    for (stride = (groupSize >> 1); stride > 0; stride >>= 1)
    {
        if (localId < groupSize/(2*stride))
        {
            uint temp = SharedMem[(2*localId + 1)*stride-1];
            SharedMem[(2*localId + 1)*stride-1] = SharedMem[2*(localId + 1)*stride-1];
            SharedMem[2*(localId + 1)*stride-1] = SharedMem[2*(localId + 1)*stride-1] + temp;
        }

        GroupMemoryBarrierWithGroupSync();
    }
}

//=====================================================================================================================
void group_scan_exclusive_sum_uint(int localId, int groupSize, out uint sum)
{
    int stride;
    for (stride = 1; stride <= (groupSize >> 1); stride <<= 1)
    {
        if (localId < groupSize / (2 * stride))
        {
            SharedMem[2 * (localId + 1)*stride - 1] =
                SharedMem[2 * (localId + 1)*stride - 1] + SharedMem[(2 * localId + 1)*stride - 1];
        }

        GroupMemoryBarrierWithGroupSync();
    }

    sum = SharedMem[groupSize - 1];

    GroupMemoryBarrierWithGroupSync();

    if (localId == 0)
    {
        SharedMem[groupSize - 1] = 0;
    }

    GroupMemoryBarrierWithGroupSync();

    for (stride = (groupSize >> 1); stride > 0; stride >>= 1)
    {
        if (localId < groupSize/(2*stride))
        {
            uint temp = SharedMem[(2*localId + 1)*stride-1];
            SharedMem[(2*localId + 1)*stride-1] = SharedMem[2*(localId + 1)*stride-1];
            SharedMem[2*(localId + 1)*stride-1] = SharedMem[2*(localId + 1)*stride-1] + temp;
        }

        GroupMemoryBarrierWithGroupSync();
    }
}

//=====================================================================================================================
uint64_t FetchKey64LDS(uint i)
{
    return (((uint64_t) SharedMem[SHARED_KEYS_OFFSET + (i << 1) + 1]) << 32) |
           (((uint64_t) SharedMem[SHARED_KEYS_OFFSET + (i << 1)]) & 0x00000000FFFFFFFF);
}

//=====================================================================================================================
void WriteKey64LDS(uint i, uint64_t mortonCode)
{
    SharedMem[SHARED_KEYS_OFFSET + (i << 1)] = uint(mortonCode);
    SharedMem[SHARED_KEYS_OFFSET + (i << 1) + 1] = uint(mortonCode >> 32);
}

//=====================================================================================================================
void ScatterKeysAndValuesImpl(
    uint groupId,
    uint localId,
    uint bitShiftSize,
    uint numElements,
    uint totalNumGroups,
    uint inputKeysOffset,
    uint inputValuesOffset,
    uint histogramOffset,
    uint outputKeysOffset,
    uint outputValuesOffset,
    uint useMortonCode30)
{
    const int numBlocksPerGroup = NUMBER_OF_BLOCKS_PER_GROUP;
    const int numElemsPerGroup = numBlocksPerGroup * GROUP_SIZE;
    const int numBlocksTotal = (numElements + GROUP_SIZE * 4 - 1) / (GROUP_SIZE * 4);
    const int maxBlocks = numBlocksTotal - groupId * numBlocksPerGroup;
    const int blockCount = min(numBlocksPerGroup, maxBlocks);

    // Copy scanned histogram for the group to local memory for fast indexing
    if (localId < NUM_BINS)
    {
        const uint inputHistogramByteOffset =
            histogramOffset + ((groupId + localId * totalNumGroups) * sizeof(int));

        SharedMem[SharedScannedHistogramOffset + localId] = ScratchBuffer.Load(inputHistogramByteOffset);
    }

    // Make sure everything is up to date
    GroupMemoryBarrierWithGroupSync();

    if (useMortonCode30)
    {
        int loadIdx = groupId * numElemsPerGroup + localId;
        for (int block = 0; block < blockCount; ++block, loadIdx += GROUP_SIZE)
        {
            // Load single int4 value
            int4 localKeys = safe_load_int4_intmax(inputKeysOffset, loadIdx, numElements);
            int4 localVals;

            // Generate fixed indices for the first pass to avoid initialising the primitive indices
            // buffer to 0:N unnecessarily.
            if (bitShiftSize == 0)
            {
                localVals = safe_load_int4_intmax_fixed(loadIdx, numElements);
            }
            else
            {
                localVals = safe_load_int4_intmax(inputValuesOffset, loadIdx, numElements);
            }

            // Clear the histogram
            SharedMem[localId] = 0;

            // Make sure everything is up to date
            GroupMemoryBarrierWithGroupSync();

            // Do 2 bits per pass
            for (int bit = 0; bit <= 2; bit += 2)
            {
                // Count histogram
                int4 b = (int4)(((localKeys >> bitShiftSize) >> bit) & 0x3);

                int4 p;
                p.x = 1u << (8 * b.x);
                p.y = 1u << (8 * b.y);
                p.z = 1u << (8 * b.z);
                p.w = 1u << (8 * b.w);

                // Pack the histogram
                uint packedKey = (uint)(p.x + p.y + p.z + p.w);

                // Put into LDS
                SharedMem[localId] = packedKey;

                // Make sure everything is up to date
                GroupMemoryBarrierWithGroupSync();

                // Scan the histogram in LDS with 4-way plus scan
                uint total = 0;
                group_scan_exclusive_sum_uint(localId, GROUP_SIZE, total);

                // Load value back
                packedKey = SharedMem[localId];

                // Make sure everything is up to date
                GroupMemoryBarrierWithGroupSync();

                // Scan total histogram (4 chars)
                total = (total << 8) + (total << 16) + (total << 24);
                uint offset = total + packedKey;

                int4 newOffset;

                int t = p.y + p.x;
                p.w = p.z + t;
                p.z = t;
                p.y = p.x;
                p.x = 0;

                p += (int) offset;
                newOffset = (p >> (b * 8)) & 0xFF;

                SharedMem[SHARED_KEYS_OFFSET + newOffset.x] = localKeys.x;
                SharedMem[SHARED_KEYS_OFFSET + newOffset.y] = localKeys.y;
                SharedMem[SHARED_KEYS_OFFSET + newOffset.z] = localKeys.z;
                SharedMem[SHARED_KEYS_OFFSET + newOffset.w] = localKeys.w;

                SharedMem[newOffset.x] = localVals.x;
                SharedMem[newOffset.y] = localVals.y;
                SharedMem[newOffset.z] = localVals.z;
                SharedMem[newOffset.w] = localVals.w;

                // Make sure everything is up to date
                GroupMemoryBarrierWithGroupSync();

                // Reload values back to registers for the second bit pass
                localKeys.x = SharedMem[SHARED_KEYS_OFFSET + (localId << 2)];
                localKeys.y = SharedMem[SHARED_KEYS_OFFSET + (localId << 2) + 1];
                localKeys.z = SharedMem[SHARED_KEYS_OFFSET + (localId << 2) + 2];
                localKeys.w = SharedMem[SHARED_KEYS_OFFSET + (localId << 2) + 3];

                // Reload values back to registers for the second bit pass
                localVals.x = SharedMem[localId << 2];
                localVals.y = SharedMem[(localId << 2) + 1];
                localVals.z = SharedMem[(localId << 2) + 2];
                localVals.w = SharedMem[(localId << 2) + 3];

                // Make sure everything is up to date
                GroupMemoryBarrierWithGroupSync();
            }

            // Clear LDS
            SharedMem[localId] = 0;

            // Make sure everything is up to date
            GroupMemoryBarrierWithGroupSync();

            // Reconstruct 16 bins histogram
            int4 bin = (int4)((localKeys >> bitShiftSize) & 0xF);
            InterlockedAdd(SharedMem[bin.x], 1);
            InterlockedAdd(SharedMem[bin.y], 1);
            InterlockedAdd(SharedMem[bin.z], 1);
            InterlockedAdd(SharedMem[bin.w], 1);

            GroupMemoryBarrierWithGroupSync();

            int sum = 0;
            if (localId < NUM_BINS)
            {
                sum = SharedMem[localId];
            }

            // Make sure everything is up to date
            GroupMemoryBarrierWithGroupSync();

            // Scan reconstructed histogram
            group_scan_exclusive_uint(localId, 16);

            const uint keySize = sizeof(int);

            // Store LDS data to scratch
            [unroll]
            for (uint i = 0; i < 4; i++)
            {
                const int offset = i + SharedMem[SharedScannedHistogramOffset + bin[i]] + (localId << 2) - SharedMem[bin[i]];

                if (offset < numElements)
                {
                    ScratchBuffer.Store(outputKeysOffset + (offset * keySize), localKeys[i]);
                    ScratchBuffer.Store(outputValuesOffset + (offset * sizeof(int)), localVals[i]);
                }
            }

            GroupMemoryBarrierWithGroupSync();

            if ((localId < NUM_BINS) && (block < (blockCount - 1)))
            {
                SharedMem[SharedScannedHistogramOffset + localId] += sum;
            }
        }
    }
    else
    {
        int loadIdx = groupId * numElemsPerGroup + localId;
        for (int block = 0; block < blockCount; ++block, loadIdx += GROUP_SIZE)
        {
            // Load single int4 value
            uint64_t4 localKeys = safe_load_int64_4_intmax(inputKeysOffset, loadIdx, numElements);
            int4 localVals;

            // Generate fixed indices for the first pass to avoid initialising the primitive indices
            // buffer to 0:N unnecessarily.
            if (bitShiftSize == 0)
            {
                localVals = safe_load_int4_intmax_fixed(loadIdx, numElements);
            }
            else
            {
                localVals = safe_load_int4_intmax(inputValuesOffset, loadIdx, numElements);
            }

            // Clear the histogram
            SharedMem[localId] = 0;

            // Make sure everything is up to date
            GroupMemoryBarrierWithGroupSync();

            // Do 2 bits per pass
            for (int bit = 0; bit <= 2; bit += 2)
            {
                // Count histogram
                int4 b = (int4)(((localKeys >> bitShiftSize) >> bit) & 0x3);

                int4 p;
                p.x = 1u << (8 * b.x);
                p.y = 1u << (8 * b.y);
                p.z = 1u << (8 * b.z);
                p.w = 1u << (8 * b.w);

                // Pack the histogram
                uint packedKey = (uint)(p.x + p.y + p.z + p.w);

                // Put into LDS
                SharedMem[localId] = packedKey;

                // Make sure everything is up to date
                GroupMemoryBarrierWithGroupSync();

                // Scan the histogram in LDS with 4-way plus scan
                uint total = 0;
                group_scan_exclusive_sum_uint(localId, GROUP_SIZE, total);

                // Load value back
                packedKey = SharedMem[localId];

                // Make sure everything is up to date
                GroupMemoryBarrierWithGroupSync();

                // Scan total histogram (4 chars)
                total = (total << 8) + (total << 16) + (total << 24);
                uint offset = total + packedKey;

                int4 newOffset;

                int t = p.y + p.x;
                p.w = p.z + t;
                p.z = t;
                p.y = p.x;
                p.x = 0;

                p += (int)offset;
                newOffset = (p >> (b * 8)) & 0xFF;

                WriteKey64LDS(newOffset.x, localKeys.x);
                WriteKey64LDS(newOffset.y, localKeys.y);
                WriteKey64LDS(newOffset.z, localKeys.z);
                WriteKey64LDS(newOffset.w, localKeys.w);

                SharedMem[newOffset.x] = localVals.x;
                SharedMem[newOffset.y] = localVals.y;
                SharedMem[newOffset.z] = localVals.z;
                SharedMem[newOffset.w] = localVals.w;

                // Make sure everything is up to date
                GroupMemoryBarrierWithGroupSync();

                localKeys.x = FetchKey64LDS(localId << 2);
                localKeys.y = FetchKey64LDS((localId << 2) + 1);
                localKeys.z = FetchKey64LDS((localId << 2) + 2);
                localKeys.w = FetchKey64LDS((localId << 2) + 3);

                // Reload values back to registers for the second bit pass
                localVals.x = SharedMem[localId << 2];
                localVals.y = SharedMem[(localId << 2) + 1];
                localVals.z = SharedMem[(localId << 2) + 2];
                localVals.w = SharedMem[(localId << 2) + 3];

                // Make sure everything is up to date
                GroupMemoryBarrierWithGroupSync();
            }

            // Clear LDS
            SharedMem[localId] = 0;

            // Make sure everything is up to date
            GroupMemoryBarrierWithGroupSync();

            // Reconstruct 16 bins histogram
            int4 bin = (int4)((localKeys >> bitShiftSize) & 0xF);
            InterlockedAdd(SharedMem[bin.x], 1);
            InterlockedAdd(SharedMem[bin.y], 1);
            InterlockedAdd(SharedMem[bin.z], 1);
            InterlockedAdd(SharedMem[bin.w], 1);

            GroupMemoryBarrierWithGroupSync();

            int sum = 0;
            if (localId < NUM_BINS)
            {
                sum = SharedMem[localId];
            }

            // Make sure everything is up to date
            GroupMemoryBarrierWithGroupSync();

            // Scan reconstructed histogram
            group_scan_exclusive_uint(localId, 16);

            const uint keySize = sizeof(uint64_t);

            // Store LDS data to scratch
            [unroll]
            for (uint i = 0; i < 4; i++)
            {
                const int offset = i + SharedMem[SharedScannedHistogramOffset + bin[i]] + (localId << 2) - SharedMem[bin[i]];

                if (offset < numElements)
                {
                    ScratchBuffer.Store(outputKeysOffset + (offset * keySize), localKeys[i]);
                    ScratchBuffer.Store(outputValuesOffset + (offset * sizeof(int)), localVals[i]);
                }
            }

            GroupMemoryBarrierWithGroupSync();

            if ((localId < NUM_BINS) && (block < (blockCount - 1)))
            {
                SharedMem[SharedScannedHistogramOffset + localId] += sum;
            }
        }
    }
}

#if NO_SHADER_ENTRYPOINT == 0
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

//=====================================================================================================================
// Main Function        : ScatterKeysAndValues
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(GROUP_SIZE, 1, 1)]
void ScatterKeysAndValues(
    uint localId : SV_GroupThreadID,
    uint groupId : SV_GroupId)
{
    const uint numPrimitives =
        (Settings.topLevelBuild && Settings.rebraidType == RebraidType::V2) || Settings.doTriangleSplitting ?
            ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET) :
            ShaderConstants.numLeafNodes;

    const uint inputKeysOffset    = ShaderConstants.offsets.mortonCodes;
    const uint outputKeysOffset   = ShaderConstants.offsets.mortonCodesSorted;
    // Passing in 0 for initial primitive indices since we auto-generate them for the first pass
    const uint inputValuesOffset  = 0;
    const uint outputValuesOffset = ShaderConstants.offsets.primIndicesSorted;

    const uint scratchKeysOffset   = ShaderConstants.offsets.tempKeys;
    const uint scratchValuesOffset = ShaderConstants.offsets.tempVals;

    uint currentFromKeys = inputKeysOffset;
    uint currentFromVals = inputValuesOffset;
    uint currentToKeys   = scratchKeysOffset;
    uint currentToVals   = scratchValuesOffset;

    if (ShaderRootConstants.BitShiftSize > 0)
    {
        currentFromKeys = outputKeysOffset;
        currentFromVals = outputValuesOffset;
    }
    const uint BitsPerPass = 4;
    const uint pass = ShaderRootConstants.BitShiftSize / BitsPerPass;
    if (pass % 2 == 1)
    {
        SwapUint(currentFromKeys, currentToKeys);
        SwapUint(currentFromVals, currentToVals);
    }

    ScatterKeysAndValuesImpl(
        groupId,
        localId,
        ShaderRootConstants.BitShiftSize,
        numPrimitives,
        ShaderRootConstants.NumGroups,
        currentFromKeys,
        currentFromVals,
        ShaderConstants.offsets.histogram,
        currentToKeys,
        currentToVals,
        Settings.useMortonCode30);
}
#endif
