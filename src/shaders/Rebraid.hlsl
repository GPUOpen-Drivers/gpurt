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
//
// New rebraid algorithm supports up to a rebraidFactor < 4.  It's optimized for this case.
// It prioritizes the largest sized Instances for a single opening.

#if NO_SHADER_ENTRYPOINT == 0
#define RootSig "RootConstants(num32BitConstants=1, b0),"\
                "CBV(b1),"\
                "UAV(u0),"\
                "UAV(u1),"\
                "UAV(u2),"\
                "UAV(u3),"\
                "CBV(b255),"\
                "UAV(u4)"

#include "../shared/rayTracingDefs.h"

struct RootConstants
{
    uint numThreadGroups;
};

[[vk::push_constant]] ConstantBuffer<RootConstants> ShaderRootConstants    : register(b0);
[[vk::binding(1, 1)]] ConstantBuffer<BuildShaderConstants> ShaderConstants : register(b1);

[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer DstBuffer          : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer DstMetadata        : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer ScratchBuffer      : register(u2);
[[vk::binding(3, 0)]]                  RWByteAddressBuffer InstanceDescBuffer : register(u3);

// unused buffer
[[vk::binding(4, 0)]] RWByteAddressBuffer                  SrcBuffer          : register(u4);

#include "BuildCommonScratch.hlsl"

#define MAX_LDS_ELEMENTS (16 * BUILD_THREADGROUP_SIZE)
groupshared uint SharedMem[MAX_LDS_ELEMENTS];

#include "TaskQueueCounter.hlsl"

#include "RadixSort/ScanExclusiveInt4DLBCommon.hlsl"
#endif

#define REBRAID_KEYS_PER_THREAD         4
#define REBRAID_KEYS_PER_GROUP          (BUILD_THREADGROUP_SIZE * REBRAID_KEYS_PER_THREAD)

#define ENABLE_HIGHER_QUALITY           1

//=====================================================================================================================
struct RebraidArgs
{
    uint numPrimitives;
    uint numThreadGroups;
    uint maxNumPrims;
    uint bvhLeafNodeDataScratchOffset;
    uint sceneBoundsOffset;
    uint stateScratchOffset;
    uint taskQueueCounterScratchOffset;
    uint atomicFlagsScratchOffset;
    uint encodeArrayOfPointers;
    uint enableCentroidSceneBoundsWithSize;
    uint enableSAHCost;
};

//=====================================================================================================================
void WriteFlags(RebraidArgs args, uint index, Flags flags)
{
    const uint offset = args.atomicFlagsScratchOffset + (index * sizeof(Flags));

    ScratchBuffer.Store(offset + FLAGS_PREFIX_SUM_OFFSET, flags.prefixSum);

    DeviceMemoryBarrier();

    ScratchBuffer.Store(offset + FLAGS_DATA_VALID_OFFSET, flags.dataValid);
}

//=====================================================================================================================
uint InterlockedCmpxDataValid(
    RebraidArgs   args,
    uint          index,
    uint          compare,
    uint          value)
{
    const uint offset = args.atomicFlagsScratchOffset + (index * sizeof(Flags)) + FLAGS_DATA_VALID_OFFSET;

    uint original;
    ScratchBuffer.InterlockedCompareExchange(offset, compare, value, original);

    return original;
}

//=====================================================================================================================
uint ReadPrefixSum(
    RebraidArgs args,
    uint        index)
{
    const uint offset = index * sizeof(Flags);

    return ScratchBuffer.Load(args.atomicFlagsScratchOffset + offset + FLAGS_PREFIX_SUM_OFFSET);
}

//=====================================================================================================================
// Fetch bottom-level acceleration structure root child information.
void FetchBlasRootChildInfo(
    uint64_t               blasBaseAddr,
    uint32_t               rootNodePtr,
    out_param(BoundingBox) bbox[4],
    out_param(uint4)       child)
{
    {
        const Float32BoxNode node = FetchFloat32BoxNode(blasBaseAddr,
                                                        rootNodePtr);
        bbox[0].min = node.bbox0_min;
        bbox[0].max = node.bbox0_max;
        bbox[1].min = node.bbox1_min;
        bbox[1].max = node.bbox1_max;
        bbox[2].min = node.bbox2_min;
        bbox[2].max = node.bbox2_max;
        bbox[3].min = node.bbox3_min;
        bbox[3].max = node.bbox3_max;
        child[0] = node.child0;
        child[1] = node.child1;
        child[2] = node.child2;
        child[3] = node.child3;
    }
}

//=====================================================================================================================
float CalculatePriorityUsingSAHComparison(RebraidArgs args, ScratchNode leaf)
{
    const BoundingBox aabb = GetScratchNodeBoundingBox(leaf);

    const uint rootNodePointer = CreateRootNodePointer();

    InstanceDesc desc;

    if (args.encodeArrayOfPointers != 0)
    {
        const GpuVirtualAddress addr = InstanceDescBuffer.Load<GpuVirtualAddress>(leaf.left_or_primIndex_or_instIndex *
                                                                                  GPU_VIRTUAL_ADDRESS_SIZE);
        desc = FetchInstanceDescAddr(addr);
    }
    else
    {
        desc = InstanceDescBuffer.Load<InstanceDesc>(leaf.left_or_primIndex_or_instIndex * INSTANCE_DESC_SIZE);
    }

    const GpuVirtualAddress address = GetInstanceAddr(asuint(leaf.sah_or_v2_or_instBasePtr.x),
                                                      asuint(leaf.sah_or_v2_or_instBasePtr.y));

    uint4 child = uint4(INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX);

    BoundingBox bbox[4];
    FetchBlasRootChildInfo(address, rootNodePointer, bbox, child);

    BoundingBox temp = TransformBoundingBox(bbox[0], desc.Transform);
    float surfaceArea = ComputeBoxSurfaceArea(temp);

    if (child[1] != INVALID_IDX)
    {
        temp = TransformBoundingBox(bbox[1], desc.Transform);
        surfaceArea += ComputeBoxSurfaceArea(temp);
    }

    if (child[2] != INVALID_IDX)
    {
        temp = TransformBoundingBox(bbox[2], desc.Transform);
        surfaceArea += ComputeBoxSurfaceArea(temp);
    }

    if (child[3] != INVALID_IDX)
    {
        temp = TransformBoundingBox(bbox[3], desc.Transform);
        surfaceArea += ComputeBoxSurfaceArea(temp);
    }

    return max(0, ComputeBoxSurfaceArea(aabb) - surfaceArea);
}

//=====================================================================================================================
// Main Function : RebraidImpl
//=====================================================================================================================
void RebraidImpl(
    uint            globalId,
    uint            localId,
    uint            groupId,
    RebraidArgs     args)
{
    const uint taskQueueOffset           = args.taskQueueCounterScratchOffset;
    const uint sumValueOffset            = args.stateScratchOffset + STATE_REBRAID_SUM_VALUE_OFFSET;
    const uint mutexOffset               = args.stateScratchOffset + STATE_REBRAID_MUTEX_OFFSET;

    if (args.numPrimitives == 0)
    {
        return;
    }

    ///////// init /////////
    const uint numGroups = args.numThreadGroups;

    const uint numThreads = numGroups * BUILD_THREADGROUP_SIZE;

    if (globalId == 0)
    {
        AllocTasks(numGroups, REBRAID_PHASE_INIT, taskQueueOffset);
    }

    const uint rootNodePointer = CreateRootNodePointer();

    const float openFactor = 0.2; // this is the percentage of extension allowed to the scene bounds

    while (1)
    {
        const uint2 task = BeginTask(localId, taskQueueOffset);

        const uint taskIndex = task.x;
        const uint phase = task.y;

        groupId = taskIndex;
        globalId = taskIndex * BUILD_THREADGROUP_SIZE + localId;

        switch (phase)
        {
            case REBRAID_PHASE_INIT:
            {
                const uint numStages = RoundUpQuotient(args.numPrimitives, REBRAID_KEYS_PER_GROUP);

                for (uint stage = globalId; stage < numStages; stage += numThreads)
                {
                    Flags flags;
                    flags.dataValid = 0;
                    flags.prefixSum = 0;

                    WriteFlags(args, stage, flags);
                }

                if (globalId == 0)
                {
                    ScratchBuffer.Store<float>(sumValueOffset, 0);
                    ScratchBuffer.Store<uint>(mutexOffset, 0);
                }

                if (EndTask(localId, taskQueueOffset))
                {
                    AllocTasks(numGroups, REBRAID_PHASE_CALC_SUM, taskQueueOffset);
                }
                break;
            }

            // calculates the priorities based on surface of all instances and also the sum of all priorities
            case REBRAID_PHASE_CALC_SUM:
            {
                BoundingBox initialSceneBounds;
                float3 initSceneExtent;

                if (args.enableCentroidSceneBoundsWithSize)
                {
                    const uint rebraidSceneOffset = sizeof(BoundingBox) + 2 * sizeof(float);
                    initialSceneBounds = FetchSceneBounds(args.sceneBoundsOffset + rebraidSceneOffset);

                    initSceneExtent = initialSceneBounds.max - initialSceneBounds.min;
                }

                for (uint i = globalId; i < args.numPrimitives; i += numThreads)
                {
                    const ScratchNode leaf = ScratchBuffer.Load<ScratchNode>(args.bvhLeafNodeDataScratchOffset +
                                                                             i * sizeof(ScratchNode));

                    if (IsNodeActive(leaf))
                    {
                        const BoundingBox aabb = GetScratchNodeBoundingBox(leaf);

                        // check to skip if too large
                        if (args.enableCentroidSceneBoundsWithSize)
                        {
                            bool dontOpen = false;

                            [unroll]
                            for (uint a = 0; a < 3; a++)
                            {
                                dontOpen |= aabb.min[a] < (initialSceneBounds.min[a] - (initSceneExtent[a] * openFactor));
                                dontOpen |= aabb.max[a] > (initialSceneBounds.max[a] + (initSceneExtent[a] * openFactor));
                            }

                            if (dontOpen)
                            {
                                continue;
                            }
                        }

#if ENABLE_HIGHER_QUALITY
                        const float surfaceArea = CalculatePriorityUsingSAHComparison(args, leaf);
#else
                        const float surfaceArea = ComputeBoxSurfaceArea(aabb);
#endif
                        const float waveSum = WaveActiveSum(log(surfaceArea + 1));

                        if (WaveIsFirstLane())
                        {
                            // mutex lock
                            uint orig = 1;

                            do {
                                DeviceMemoryBarrier();
                                ScratchBuffer.InterlockedCompareExchange(mutexOffset, 0, 1, orig);
                            } while (orig);

                            const float globalSum = ScratchBuffer.Load<float>(sumValueOffset);

                            ScratchBuffer.Store<float>(sumValueOffset, globalSum + waveSum);

                            DeviceMemoryBarrier();

                            ScratchBuffer.Store(mutexOffset, 0); //unlock
                        }
                    }
                }

                if (EndTask(localId, taskQueueOffset))
                {
                    AllocTasks(RoundUpQuotient(args.numPrimitives, REBRAID_KEYS_PER_GROUP),
                                                 REBRAID_PHASE_OPEN, taskQueueOffset);
                }
                break;
            }

            // allows a single opening to an instance based on if the numOpenings >= 1
            case REBRAID_PHASE_OPEN:
            {
                const float globalSum = ScratchBuffer.Load<float>(sumValueOffset);

                const uint maxOpenings = (args.maxNumPrims - args.numPrimitives) / 3;

                const uint start = globalId * REBRAID_KEYS_PER_THREAD;
                const uint end = min(start + REBRAID_KEYS_PER_THREAD, args.numPrimitives);

                uint localKeys[REBRAID_KEYS_PER_THREAD];
                bool open[REBRAID_KEYS_PER_THREAD];

                uint keyIndex = 0;

                BoundingBox initialSceneBounds;
                float3 initSceneExtent;

                if (args.enableCentroidSceneBoundsWithSize)
                {
                    const uint rebraidSceneOffset = sizeof(BoundingBox) + 2 * sizeof(float);
                    initialSceneBounds = FetchSceneBounds(args.sceneBoundsOffset + rebraidSceneOffset);

                    initSceneExtent = initialSceneBounds.max - initialSceneBounds.min;
                }

                for (uint i = start; i < end; i++)
                {
                    const ScratchNode leaf = ScratchBuffer.Load<ScratchNode>(args.bvhLeafNodeDataScratchOffset +
                                                                             i * sizeof(ScratchNode));

                    if (IsNodeActive(leaf))
                    {
                        const BoundingBox aabb = GetScratchNodeBoundingBox(leaf);

                        bool dontOpen = false;

                         // check to skip if too large
                        if (args.enableCentroidSceneBoundsWithSize)
                        {
                            [unroll]
                            for (uint a = 0; a < 3; a++)
                            {
                                dontOpen |= aabb.min[a] < (initialSceneBounds.min[a] - (initSceneExtent[a] * openFactor));
                                dontOpen |= aabb.max[a] > (initialSceneBounds.max[a] + (initSceneExtent[a] * openFactor));
                            }
                        }

                        uint numOpenings = 0;

                        if (!dontOpen)
                        {
#if ENABLE_HIGHER_QUALITY
                            const float surfaceArea = CalculatePriorityUsingSAHComparison(args, leaf);
#else
                            const float surfaceArea = ComputeBoxSurfaceArea(aabb);
#endif
                            const float value = log(surfaceArea + 1);

                            numOpenings = maxOpenings * (value / globalSum);
                        }

                        if (numOpenings > 0)
                        {
                            const uint64_t instanceBasePointer =
                                PackUint64(asuint(leaf.sah_or_v2_or_instBasePtr.x), asuint(leaf.sah_or_v2_or_instBasePtr.y));

                            const uint childCount =
                                GetBlasInternalNodeChildCount(instanceBasePointer, rootNodePointer);

                            localKeys[keyIndex] = childCount - 1; // Additional children
                            open[keyIndex] = true;
                        }
                        else
                        {
                            if (args.enableCentroidSceneBoundsWithSize)
                            {
                                UpdateCentroidSceneBoundsWithSize(args.sceneBoundsOffset, aabb);
                            }

                            localKeys[keyIndex] = 0;
                            open[keyIndex] = false;
                        }

                        keyIndex++;
                    }
                    else
                    {
                        localKeys[keyIndex] = 0;
                        open[keyIndex] = false;

                        keyIndex++;
                    }
                }

                uint threadSum = 0;

                // Calculate scan on this thread's elements
                for (uint i = 0; i < keyIndex; ++i)
                {
                    const uint tmp = localKeys[i];

                    localKeys[i] = threadSum;
                    threadSum += tmp;
                }

                const uint threadSumScanned = BlockScanExclusiveAdd(threadSum, localId);

                // Add partial sums back
                for (uint i = 0; i < keyIndex; ++i)
                {
                    localKeys[i] += threadSumScanned;
                }

                // Wait until previous dynamic block finished and acquire its sum.

                int prevSum = 0;

                if (taskIndex > 0)
                {
                    if (localId == (BUILD_THREADGROUP_SIZE - 1))
                    {
                        while (1)
                        {
                            uint orig = InterlockedCmpxDataValid(args, taskIndex - 1, 1, 1);

                            if (orig != 0)
                            {
                                break;
                            }
                        }

                        prevSum = ReadPrefixSum(args, taskIndex - 1);
                        SharedMem[0] = prevSum;
                    }

                    GroupMemoryBarrierWithGroupSync();

                    prevSum = SharedMem[0];
                }

                GroupMemoryBarrierWithGroupSync();

                // Write our sum and enable successive blocks.
                if (localId == (BUILD_THREADGROUP_SIZE - 1))
                {
                    Flags flags;
                    flags.prefixSum = prevSum + threadSumScanned + threadSum;
                    flags.dataValid = 1;

                    WriteFlags(args, taskIndex, flags);
                }

                GroupMemoryBarrierWithGroupSync();

                keyIndex = 0;

                for (uint i = start; i < end; i++)
                {
                    const uint scratchNodeOffset = args.bvhLeafNodeDataScratchOffset + i * sizeof(ScratchNode);

                    ScratchNode leaf = ScratchBuffer.Load<ScratchNode>(scratchNodeOffset);

                    if (IsNodeActive(leaf))
                    {
                        const GpuVirtualAddress address = GetInstanceAddr(asuint(leaf.sah_or_v2_or_instBasePtr.x),
                                                                          asuint(leaf.sah_or_v2_or_instBasePtr.y));

                        InstanceDesc desc;
                        if (args.encodeArrayOfPointers != 0)
                        {
                            const GpuVirtualAddress addr = InstanceDescBuffer.Load<GpuVirtualAddress>(leaf.left_or_primIndex_or_instIndex *
                                                                                                      GPU_VIRTUAL_ADDRESS_SIZE);
                            desc = FetchInstanceDescAddr(addr);
                        }
                        else
                        {
                            desc = InstanceDescBuffer.Load<InstanceDesc>(leaf.left_or_primIndex_or_instIndex * INSTANCE_DESC_SIZE);
                        }

                        const uint4 costOrNumChildPrims = FetchHeaderCostOrNumChildPrims(address);

                        float4 cost = float4(0,0,0,0);
                        if (args.enableSAHCost)
                        {
                            const float transformFactor = FetchScratchLeafNodeCost(leaf);
                            cost = transformFactor * asfloat(costOrNumChildPrims);
                        }

                        if (open[keyIndex])
                        {
                            uint4 child = uint4(INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX);

                            BoundingBox bbox[4];
                            FetchBlasRootChildInfo(address, rootNodePointer, bbox, child);

                            BoundingBox temp = TransformBoundingBox(bbox[0], desc.Transform);

                            // update a new leaf
                            ScratchBuffer.Store<float3>(scratchNodeOffset + SCRATCH_NODE_BBOX_MIN_OFFSET, temp.min);
                            ScratchBuffer.Store<float3>(scratchNodeOffset + SCRATCH_NODE_BBOX_MAX_OFFSET, temp.max);
                            ScratchBuffer.Store<uint>(scratchNodeOffset + SCRATCH_NODE_NODE_POINTER_OFFSET, child[0]);

                            if (args.enableSAHCost == false)
                            {
                                ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_INSTANCE_NUM_PRIMS_OFFSET,
                                                    costOrNumChildPrims[0]);
                            }
                            else
                            {
                                WriteScratchNodeCost(args.bvhLeafNodeDataScratchOffset, i, cost[0], true);
                            }

                            if (args.enableCentroidSceneBoundsWithSize)
                            {
                                UpdateCentroidSceneBoundsWithSize(args.sceneBoundsOffset, temp);
                            }

                            // update leaf and siblings
                            uint numSiblings = 0;

                            if (child[1] != INVALID_IDX)
                            {
                                temp = TransformBoundingBox(bbox[1], desc.Transform);

                                leaf.bbox_min_or_v0 = temp.min;
                                leaf.bbox_max_or_v1 = temp.max;

                                const uint writeIndex = args.numPrimitives + prevSum + localKeys[keyIndex] + numSiblings;

                                leaf.splitBox_or_nodePointer = child[1];

                                if (args.enableSAHCost == false)
                                {
                                    leaf.sah_or_v2_or_instBasePtr.z = asfloat(costOrNumChildPrims[1]);
                                }
                                else
                                {
                                    leaf.numPrimitivesAndDoCollapse = asuint(cost[1]);
                                }

                                ScratchBuffer.Store<ScratchNode>(
                                    args.bvhLeafNodeDataScratchOffset + writeIndex * sizeof(ScratchNode), leaf);

                                if (args.enableCentroidSceneBoundsWithSize)
                                {
                                    UpdateCentroidSceneBoundsWithSize(args.sceneBoundsOffset, temp);
                                }

                                numSiblings++;
                            }

                            if (child[2] != INVALID_IDX)
                            {
                                temp = TransformBoundingBox(bbox[2], desc.Transform);

                                leaf.bbox_min_or_v0 = temp.min;
                                leaf.bbox_max_or_v1 = temp.max;

                                const uint writeIndex = args.numPrimitives + prevSum + localKeys[keyIndex] + numSiblings;

                                leaf.splitBox_or_nodePointer = child[2];

                                if (args.enableSAHCost == false)
                                {
                                    leaf.sah_or_v2_or_instBasePtr.z = asfloat(costOrNumChildPrims[2]);
                                }
                                else
                                {
                                    leaf.numPrimitivesAndDoCollapse = asuint(cost[2]);
                                }

                                ScratchBuffer.Store<ScratchNode>(args.bvhLeafNodeDataScratchOffset
                                                                 + writeIndex * sizeof(ScratchNode), leaf);

                                if (args.enableCentroidSceneBoundsWithSize)
                                {
                                    UpdateCentroidSceneBoundsWithSize(args.sceneBoundsOffset, temp);
                                }

                                numSiblings++;
                            }

                            if (child[3] != INVALID_IDX)
                            {
                                temp = TransformBoundingBox(bbox[3], desc.Transform);

                                leaf.bbox_min_or_v0 = temp.min;
                                leaf.bbox_max_or_v1 = temp.max;

                                const uint writeIndex = args.numPrimitives + prevSum + localKeys[keyIndex] + numSiblings;

                                leaf.splitBox_or_nodePointer = child[3];

                                if (args.enableSAHCost == false)
                                {
                                    leaf.sah_or_v2_or_instBasePtr.z = asfloat(costOrNumChildPrims[3]);
                                }
                                else
                                {
                                    leaf.numPrimitivesAndDoCollapse = asuint(cost[3]);
                                }

                                ScratchBuffer.Store<ScratchNode>(args.bvhLeafNodeDataScratchOffset
                                                                 + writeIndex * sizeof(ScratchNode), leaf);

                                if (args.enableCentroidSceneBoundsWithSize)
                                {
                                    UpdateCentroidSceneBoundsWithSize(args.sceneBoundsOffset, temp);
                                }

                                numSiblings++;
                            }
                        }
                        else // no openings
                        {
                            // point instance to the root
                            ScratchBuffer.Store<uint>(scratchNodeOffset + SCRATCH_NODE_NODE_POINTER_OFFSET,
                                                      CreateRootNodePointer());

                            if (args.enableSAHCost)
                            {
                                float costSum = 0;

                                for (uint c = 0; c < 4; c++)
                                {
                                    costSum += cost[c];
                                }

                                const BoundingBox aabb = GetScratchNodeBoundingBox(leaf);

                                const float surfaceArea = ComputeBoxSurfaceArea(aabb);

                                costSum += SAH_COST_AABBB_INTERSECTION * surfaceArea;

                                WriteScratchNodeCost(args.bvhLeafNodeDataScratchOffset, i, costSum, true);
                            }
                        }
                    }

                    keyIndex++;

                    if (i == (args.numPrimitives - 1))
                    {
                        WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET,
                                                    args.numPrimitives + prevSum + threadSumScanned + threadSum);
                    }
                }

                if (EndTask(localId, taskQueueOffset))
                {
                    AllocTasks(numGroups, REBRAID_PHASE_DONE, taskQueueOffset);
                }

                break;
            }

            case REBRAID_PHASE_DONE:
                return;
        }
    }
}

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
// Entry point when this shader is called in its own Dispatch() call
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void Rebraid(
    uint globalId : SV_DispatchThreadID,
    uint localId  : SV_GroupThreadID,
    uint groupId  : SV_GroupID)
{

    RebraidArgs args;

    args.numPrimitives                      = ShaderConstants.numPrimitives;
    args.numThreadGroups                    = ShaderRootConstants.numThreadGroups;
    args.maxNumPrims                        = ShaderConstants.rebraidFactor * ShaderConstants.numPrimitives;
    args.bvhLeafNodeDataScratchOffset       = ShaderConstants.offsets.bvhLeafNodeData;
    args.sceneBoundsOffset                  = ShaderConstants.offsets.sceneBounds;
    args.stateScratchOffset                 = ShaderConstants.offsets.rebraidState;
    args.taskQueueCounterScratchOffset      = ShaderConstants.offsets.rebraidTaskQueueCounter;
    args.atomicFlagsScratchOffset           = ShaderConstants.offsets.splitAtomicFlags;
    args.encodeArrayOfPointers              = ShaderConstants.encodeArrayOfPointers;
    args.enableSAHCost                      = Settings.enableSAHCost;

    RebraidImpl(globalId, localId, groupId, args);
}
#endif
