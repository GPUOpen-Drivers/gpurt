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
#ifndef _LANE_GROUP_HLSL
#define _LANE_GROUP_HLSL

#include "Extensions.hlsl"

//=====================================================================================================================
// The following is a wrapper structure for wave lanes groups. A lanes group is a set of lanes within a wave that
// cooperate on a piece of work.
struct LaneGroup
{
    uint laneIndex;   // Lane index within this group
    uint groupIndex;  // Group index within the parent wave
    uint groupSize;   // Number of lanes in this group
    uint groupMask;   // Lane mask for the group

    // Allocate a lane group within a wave. Note, groupSize must be a compile time constant
    // and must be a power of 2 (i.e. 1, 2, 4, 8, 16, 32, 64)
    void Alloc(uint size)
    {
        groupSize  = size;
        laneIndex  = (WaveGetLaneIndex() % groupSize);
        groupIndex = (WaveGetLaneIndex() / groupSize);
        groupMask  = (1u << groupSize) - 1;
    }

    // Returns the index of the global (wave wide) lane index of the first lane in this group
    uint GetFirstLaneIndex()
    {
        return (groupIndex * groupSize);
    }

    // Checks if the source lane index matches the current lane group index
    bool IsLaneIndex(uint idx)
    {
        return (laneIndex == idx);
    }

    // Returns true if this is the first lane in the group
    bool IsFirstLane()
    {
        return IsLaneIndex(0);
    }

    // Counts the number of group lanes where the expression evaluates to true
    uint Ballot(bool expression)
    {
        uint32_t mask = uint32_t(WaveActiveBallot64(expression) >> GetFirstLaneIndex());

        return mask & groupMask;
    }

    // Returns the index of the first active lane within this group that matches the source expression
    uint GetFirstActiveLaneIndex(bool expression)
    {
        // Equivalent to uint4 mask = WaveMultiPrefixCountBits(value); but this generates better code
        // than the HLSL intrinsic.
        //
        return firstbitlow(Ballot(expression));
    }

    template<typename T>
    T ReadFirstLane(T val)
    {
        return WaveReadLaneAt(val, GetFirstLaneIndex());
    }

    float Sum(float val)
    {
        const uint clusterSize = log2(groupSize) + 1;

        return AmdExtD3DShaderIntrinsics_WaveClusterSum(val, clusterSize);
    }

    float Max(float val)
    {
        const uint clusterSize = log2(groupSize) + 1;

        return AmdExtD3DShaderIntrinsics_WaveClusterMax(val, clusterSize);
    }
    template<typename T>
    T Broadcast(T val, uint targetLane)
    {
        // Ensure the target lane is within the bounds of the group
        uint groupFirstLaneIndex = GetFirstLaneIndex();
        uint adjustedTargetLane = groupFirstLaneIndex + (targetLane % groupSize);

        // Use WaveReadLaneAt to read the value from the target lane to all lanes
        return WaveReadLaneAt(val, adjustedTargetLane);
    }

    uint BallotIndex(bool condition)
    {
        // Ballot collects the bits from all lanes where the condition is true.
        uint mask = Ballot(condition);

        // Compute the prefix sum to find the index within the group.
        // For each lane, this gives the number of true conditions before it.
        uint prefixSum = WavePrefixCountBits(condition);

        // If the condition is true for the current lane, its index is the prefix sum;
        // otherwise, it's an invalid index.
        return condition ? prefixSum : INVALID_IDX;
    }

    uint ExclusivePrefixSum(bool condition)
    {
        //  Ballot the condition across the group to get a bitmask.
        uint mask = Ballot(condition);

        // Compute prefix sum only within the group.
        // Shift the mask so that bits before the current lane are considered.
        uint shiftedMask = mask & ((1u << laneIndex) - 1);

        // Count the number of set bits before this lane in the group.
        // This becomes the exclusive prefix sum.
        uint exclusiveSum = countbits(shiftedMask);

        return exclusiveSum;
    }
};
#endif
