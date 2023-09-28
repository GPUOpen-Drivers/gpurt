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

#include "..\BuildCommon.hlsl"
#include "ScanCommon.hlsli"

//=====================================================================================================================
// Shared memory used to store an intermediate histogram.
groupshared int SharedMem[NUM_BINS * GROUP_SIZE];
#endif

//=====================================================================================================================
void BitHistogramImpl(
    uint localId,
    uint groupId,
    uint totalGroups,
    uint numElements,
    uint bitShiftCount,
    uint inputOffset,
    uint outputOffset,
    uint useMortonCode30)
{
    // Clear local histogram
    for (int i = 0; i < NUM_BINS; ++i)
    {
        SharedMem[i * GROUP_SIZE + localId] = 0;
    }

    // Make sure everything is up to date
    GroupMemoryBarrierWithGroupSync();

    const int numblocks_per_group = NUMBER_OF_BLOCKS_PER_GROUP;
    const int numelems_per_group = numblocks_per_group * GROUP_SIZE;

    int numblocks_total = RoundUpQuotient(numElements, GROUP_SIZE * 4);
    int maxblocks = numblocks_total - groupId * numblocks_per_group;

    int loadidx = groupId * numelems_per_group + localId;
    for (int block = 0; block < min(numblocks_per_group, maxblocks); ++block, loadidx += GROUP_SIZE)
    {
        int4 bin;
        if (useMortonCode30)
        {
            // Load single int4 value
            int4 value = safe_load_int4_intmax(inputOffset,
                                               loadidx,
                                               numElements);

            // Handle value adding histogram bins
            // for all 4 elements
            bin = ((value >> bitShiftCount) & 0xF);
        }
        else
        {
            // Load single uint64_t4 value
            uint64_t4 value = safe_load_int64_4_intmax(inputOffset,
                                                       loadidx,
                                                       numElements);

            // Handle value adding histogram bins
            // for all 4 elements
            bin = (int4) ((value >> bitShiftCount) & 0xF);
        }

        InterlockedAdd(SharedMem[bin.x * GROUP_SIZE + localId], 1);
        InterlockedAdd(SharedMem[bin.y * GROUP_SIZE + localId], 1);
        InterlockedAdd(SharedMem[bin.z * GROUP_SIZE + localId], 1);
        InterlockedAdd(SharedMem[bin.w * GROUP_SIZE + localId], 1);
    }

    // Wait for all threads to finish
    GroupMemoryBarrierWithGroupSync();

    int sum = 0;
    if (localId < NUM_BINS)
    {
        for (int i = 0; i < GROUP_SIZE; ++i)
        {
            sum += SharedMem[localId * GROUP_SIZE + i];
        }

        const uint outputIndex = totalGroups * localId + groupId;
        ScratchBuffer.Store(outputOffset + (outputIndex * sizeof(uint)), sum);
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
// Main Function                                                                                                      //
// The kernel computes 16 bins histogram of the 256 input elements.                                                   //
// The bin is determined by (InputArray[tid] >> BitShiftSize) & 0xF                                                   //
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(GROUP_SIZE, 1, 1)]
void BitHistogram(
    uint localThreadId : SV_GroupThreadID,
    uint groupId       : SV_groupId)
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

    BitHistogramImpl(
        localThreadId,
        groupId,
        ShaderRootConstants.NumGroups,
        numPrimitives,
        ShaderRootConstants.BitShiftSize,
        currentFromKeys,
        ShaderConstants.offsets.histogram,
        Settings.useMortonCode30);
}
#endif
