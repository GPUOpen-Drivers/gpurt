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

struct RebraidArgs
{
    uint numPrimitives;
    uint numThreadGroups;
    uint maxNumPrims;
    uint bvhLeafNodeDataScratchOffset;
    uint sceneBoundsOffset;
    uint stateScratchOffset;
    uint atomicFlagsScratchOffset;
    uint encodeArrayOfPointers;
    uint enableCentroidSceneBoundsWithSize;
    uint enableSAHCost;
};

#define REBRAID_KEYS_PER_THREAD         4
#define REBRAID_KEYS_PER_GROUP          (BUILD_THREADGROUP_SIZE * REBRAID_KEYS_PER_THREAD)

#define ENABLE_HIGHER_QUALITY           1

//=====================================================================================================================
void AllocTasksRB(const uint numTasks, const uint phase, uint taskQueueOffset)
{
    // start = end
    const uint end = ScratchBuffer.Load(taskQueueOffset + STATE_TASK_QUEUE_END_PHASE_INDEX_OFFSET);

    ScratchBuffer.Store(taskQueueOffset + STATE_TASK_QUEUE_START_PHASE_INDEX_OFFSET, end);

    ScratchBuffer.Store(taskQueueOffset + STATE_TASK_QUEUE_PHASE_OFFSET, phase);

    DeviceMemoryBarrier();

    ScratchBuffer.Store(taskQueueOffset + STATE_TASK_QUEUE_END_PHASE_INDEX_OFFSET, end + numTasks);
}

//=====================================================================================================================
uint2 BeginTaskRB(const uint localId, uint taskQueueOffset)
{
    if (localId == 0)
    {
        ScratchBuffer.InterlockedAdd(taskQueueOffset + STATE_TASK_QUEUE_TASK_COUNTER_OFFSET, 1, SharedMem[0]);
    }

    GroupMemoryBarrierWithGroupSync();

    const uint index = SharedMem[0];

    // wait till there are valid tasks to do
    do
    {
        DeviceMemoryBarrier();
    } while (index >= ScratchBuffer.Load(taskQueueOffset + STATE_TASK_QUEUE_END_PHASE_INDEX_OFFSET));

    const uint phase = ScratchBuffer.Load(taskQueueOffset + STATE_TASK_QUEUE_PHASE_OFFSET);

    const uint startPhaseIndex = ScratchBuffer.Load(taskQueueOffset + STATE_TASK_QUEUE_START_PHASE_INDEX_OFFSET);

    return uint2(index - startPhaseIndex, phase);
}

//=====================================================================================================================
bool EndTaskRB(const uint localId, uint taskQueueOffset)
{
    bool returnValue = false;

    DeviceMemoryBarrier();

    if (localId == 0)
    {
        const uint endPhaseIndex = ScratchBuffer.Load(taskQueueOffset + STATE_TASK_QUEUE_END_PHASE_INDEX_OFFSET);

        uint orig;
        ScratchBuffer.InterlockedAdd(taskQueueOffset + STATE_TASK_QUEUE_NUM_TASKS_DONE_OFFSET, 1, orig);

        // current phase is done
        if (orig == (endPhaseIndex - 1))
        {
            returnValue = true;
        }
    }

    return returnValue;
}

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

    const Float32BoxNode node = FetchFloat32BoxNode(address,
                                                    rootNodePointer);

    BoundingBox temp;
    temp.min = node.bbox0_min;
    temp.max = node.bbox0_max;

    temp = TransformBoundingBox(temp, desc.Transform);

    float surfaceArea = ComputeBoxSurfaceArea(temp);

    if (node.child1 != INVALID_IDX)
    {
        temp.min = node.bbox1_min;
        temp.max = node.bbox1_max;

        temp = TransformBoundingBox(temp, desc.Transform);

        surfaceArea += ComputeBoxSurfaceArea(temp);
    }

    if (node.child2 != INVALID_IDX)
    {
        temp.min = node.bbox2_min;
        temp.max = node.bbox2_max;

        temp = TransformBoundingBox(temp, desc.Transform);

        surfaceArea += ComputeBoxSurfaceArea(temp);
    }

    if (node.child3 != INVALID_IDX)
    {
        temp.min = node.bbox3_min;
        temp.max = node.bbox3_max;

        temp = TransformBoundingBox(temp, desc.Transform);

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
    const uint taskQueueOffset           = args.stateScratchOffset;
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
        AllocTasksRB(numGroups, REBRAID_PHASE_INIT, taskQueueOffset);
    }

    const uint rootNodePointer = CreateRootNodePointer();

    const float openFactor = 0.2; // this is the percentage of extension allowed to the scene bounds

    while (1)
    {
        const uint2 task = BeginTaskRB(localId, taskQueueOffset);

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

                if (EndTaskRB(localId, taskQueueOffset))
                {
                    AllocTasksRB(numGroups, REBRAID_PHASE_CALC_SUM, taskQueueOffset);
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
                    initialSceneBounds = FetchSceneBounds(ShaderConstants.offsets.sceneBounds
                                                          + rebraidSceneOffset);

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

                if (EndTaskRB(localId, taskQueueOffset))
                {
                    AllocTasksRB(RoundUpQuotient(args.numPrimitives, REBRAID_KEYS_PER_GROUP),
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

                uint i;

                uint keyIndex = 0;

                BoundingBox initialSceneBounds;
                float3 initSceneExtent;

                if (args.enableCentroidSceneBoundsWithSize)
                {
                    const uint rebraidSceneOffset = sizeof(BoundingBox) + 2 * sizeof(float);
                    initialSceneBounds = FetchSceneBounds(ShaderConstants.offsets.sceneBounds
                                                          + rebraidSceneOffset);

                    initSceneExtent = initialSceneBounds.max - initialSceneBounds.min;
                }

                for (i = start; i < end; i++)
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
                            const GpuVirtualAddress address = GetInstanceAddr(
                                                asuint(leaf.sah_or_v2_or_instBasePtr.x),
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
                                desc = InstanceDescBuffer.Load<InstanceDesc>(leaf.left_or_primIndex_or_instIndex
                                                                             * INSTANCE_DESC_SIZE);
                            }

                            const uint childCount = GetChildCount(address,
                                                                  rootNodePointer);

                            localKeys[keyIndex] = childCount - 1; // additional children
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
                for (i = 0; i < keyIndex; ++i)
                {
                    const uint tmp = localKeys[i];

                    localKeys[i] = threadSum;
                    threadSum += tmp;
                }

                const uint threadSumScanned = BlockScanExclusiveAdd(threadSum, localId);

                // Add partial sums back
                for (i = 0; i < keyIndex; ++i)
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

                for (i = start; i < end; i++)
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

                        float4 cost;

                        if (args.enableSAHCost)
                        {
                            const float transformFactor = FetchScratchLeafNodeCost(leaf);

                            cost[0] = transformFactor * asfloat(FetchHeaderField(address, ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET));
                            cost[1] = transformFactor * asfloat(FetchHeaderField(address, ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 4));
                            cost[2] = transformFactor * asfloat(FetchHeaderField(address, ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 8));
                            cost[3] = transformFactor * asfloat(FetchHeaderField(address, ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 12));
                        }

                        if (open[keyIndex])
                        {
                            uint4 numPrims;

                            if (args.enableSAHCost == false)
                            {
                                numPrims[0] = FetchHeaderField(address, ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET);
                                numPrims[1] = FetchHeaderField(address, ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 4);
                                numPrims[2] = FetchHeaderField(address, ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 8);
                                numPrims[3] = FetchHeaderField(address, ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 12);
                            }

                            const Float32BoxNode node = FetchFloat32BoxNode(address,
                                                                            rootNodePointer);

                            BoundingBox temp;
                            temp.min = node.bbox0_min;
                            temp.max = node.bbox0_max;
                            temp = TransformBoundingBox(temp, desc.Transform);

                            // update a new leaf
                            ScratchBuffer.Store<float3>(scratchNodeOffset + SCRATCH_NODE_BBOX_MIN_OFFSET, temp.min);
                            ScratchBuffer.Store<float3>(scratchNodeOffset + SCRATCH_NODE_BBOX_MAX_OFFSET, temp.max);
                            ScratchBuffer.Store<uint>(scratchNodeOffset + SCRATCH_NODE_NODE_POINTER_OFFSET, node.child0);

                            if (args.enableSAHCost == false)
                            {
                                ScratchBuffer.Store<uint>(scratchNodeOffset + SCRATCH_NODE_INSTANCE_NUM_PRIMS_OFFSET, numPrims[0]);
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

                            if (node.child1 != INVALID_IDX)
                            {
                                temp.min = node.bbox1_min;
                                temp.max = node.bbox1_max;
                                temp = TransformBoundingBox(temp, desc.Transform);

                                leaf.bbox_min_or_v0 = temp.min;
                                leaf.bbox_max_or_v1 = temp.max;

                                const uint writeIndex = args.numPrimitives + prevSum + localKeys[keyIndex] + numSiblings;

                                leaf.splitBox_or_nodePointer = node.child1;

                                if (args.enableSAHCost == false)
                                {
                                    leaf.sah_or_v2_or_instBasePtr.z = asfloat(numPrims[1]);
                                }
                                else
                                {
                                    leaf.numPrimitivesAndDoCollapse = asuint(cost[1]);
                                }

                                ScratchBuffer.Store<ScratchNode>(args.bvhLeafNodeDataScratchOffset
                                                                 + writeIndex * sizeof(ScratchNode), leaf);

                                if (args.enableCentroidSceneBoundsWithSize)
                                {
                                    UpdateCentroidSceneBoundsWithSize(args.sceneBoundsOffset, temp);
                                }

                                numSiblings++;
                            }

                            if (node.child2 != INVALID_IDX)
                            {
                                temp.min = node.bbox2_min;
                                temp.max = node.bbox2_max;
                                temp = TransformBoundingBox(temp, desc.Transform);

                                leaf.bbox_min_or_v0 = temp.min;
                                leaf.bbox_max_or_v1 = temp.max;

                                const uint writeIndex = args.numPrimitives + prevSum + localKeys[keyIndex] + numSiblings;

                                leaf.splitBox_or_nodePointer = node.child2;

                                if (args.enableSAHCost == false)
                                {
                                    leaf.sah_or_v2_or_instBasePtr.z = asfloat(numPrims[2]);
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

                            if (node.child3 != INVALID_IDX)
                            {
                                temp.min = node.bbox3_min;
                                temp.max = node.bbox3_max;
                                temp = TransformBoundingBox(temp, desc.Transform);

                                leaf.bbox_min_or_v0 = temp.min;
                                leaf.bbox_max_or_v1 = temp.max;

                                const uint writeIndex = args.numPrimitives + prevSum + localKeys[keyIndex] + numSiblings;

                                leaf.splitBox_or_nodePointer = node.child3;

                                if (args.enableSAHCost == false)
                                {
                                    leaf.sah_or_v2_or_instBasePtr.z = asfloat(numPrims[3]);
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
                            ScratchBuffer.Store<uint>(scratchNodeOffset +
                                                      SCRATCH_NODE_NODE_POINTER_OFFSET,
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
                        DstBuffer.Store(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET,
                                        args.numPrimitives + prevSum + threadSumScanned + threadSum);
                    }
                }

                if (EndTaskRB(localId, taskQueueOffset))
                {
                    AllocTasksRB(numGroups, REBRAID_PHASE_DONE, taskQueueOffset);
                }

                break;
            }

            case REBRAID_PHASE_DONE:
                return;
        }
    }
}
