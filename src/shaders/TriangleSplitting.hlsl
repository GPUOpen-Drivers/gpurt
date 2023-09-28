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
// 32 bit constants
struct TriangleSplittingArgs
{
    uint numThreadGroups;
    uint numPrimitives;
    uint maxNumPrimitives;
    uint numSizeBits;
    uint scratchLeafNodesScratchOffset;
    uint splitBoxesScratchOffset;
    uint refList0ScratchOffset;
    uint refList1ScratchOffset;
    uint splitPrioritiesScratchOffset;
    uint currentStateScratchOffset;
    uint taskQueueCounterScratchOffset;
    uint atomicFlagsScratchOffset;
    uint dynamicBlockIndexScratchOffset;
    uint sceneBoundsByteOffset;
    uint tsBudgetPerTriangle;
};

#define TS_KEYS_PER_THREAD         4
#define TS_KEYS_PER_GROUP          (BUILD_THREADGROUP_SIZE * TS_KEYS_PER_THREAD)

//=====================================================================================================================
void WriteFlags(TriangleSplittingArgs args, uint index, Flags flags)
{
    const uint offset = args.atomicFlagsScratchOffset + (index * sizeof(Flags));

    ScratchBuffer.Store(offset + FLAGS_PREFIX_SUM_OFFSET, flags.prefixSum);

    DeviceMemoryBarrier();

    ScratchBuffer.Store(offset + FLAGS_DATA_VALID_OFFSET, flags.dataValid);
}

//=====================================================================================================================
uint InterlockedCmpxDataValid(
    TriangleSplittingArgs   args,
    uint                    index,
    uint                    compare,
    uint                    value)
{
    const uint offset = args.atomicFlagsScratchOffset + (index * sizeof(Flags)) + FLAGS_DATA_VALID_OFFSET;

    uint original;
    ScratchBuffer.InterlockedCompareExchange(offset, compare, value, original);

    return original;
}

//=====================================================================================================================
uint ReadPrefixSum(
    TriangleSplittingArgs   args,
    uint                    index)
{
    const uint offset = index * sizeof(Flags);

    return ScratchBuffer.Load(args.atomicFlagsScratchOffset + offset + FLAGS_PREFIX_SUM_OFFSET);
}

//=====================================================================================================================
void InitPrefixSum(TriangleSplittingArgs args, uint globalThreadID, uint count)
{
    const uint numBlocks = (count + (TS_KEYS_PER_GROUP - 1)) / TS_KEYS_PER_GROUP;

    const uint numThreads = args.numThreadGroups * BUILD_THREADGROUP_SIZE;

    for (uint blockIndex = globalThreadID; blockIndex < numBlocks; blockIndex += numThreads)
    {
        Flags flags;
        flags.dataValid = 0;
        flags.prefixSum = 0;

        WriteFlags(args, blockIndex, flags);
    }

    if (globalThreadID == 0)
    {
        ScratchBuffer.Store(args.dynamicBlockIndexScratchOffset, 0);
    }
}

//=====================================================================================================================
ScratchTSRef ReadRefList(TriangleSplittingArgs args, uint listIndex, uint index)
{
    uint offset;

    if (listIndex == 0)
    {
        offset = args.refList0ScratchOffset;
    }
    else
    {
        offset = args.refList1ScratchOffset;
    }

    return ScratchBuffer.Load<ScratchTSRef>(offset + (index * sizeof(ScratchTSRef)));
}

//=====================================================================================================================
void WriteRefList(TriangleSplittingArgs args, uint listIndex, uint index, ScratchTSRef ref)
{
    uint offset;

    if (listIndex == 0)
    {
        offset = args.refList0ScratchOffset;
    }
    else
    {
        offset = args.refList1ScratchOffset;
    }

    ScratchBuffer.Store<ScratchTSRef>(offset + (index * sizeof(ScratchTSRef)), ref);
}

//=====================================================================================================================
void AllocRefs(TriangleSplittingArgs args,
               uint                  localId,
               float                 globalSum)
{
    const uint numBlocks = (args.numPrimitives + (TS_KEYS_PER_GROUP - 1)) / TS_KEYS_PER_GROUP;

    while(1)
    {
        uint blockId;

        if (WaveIsFirstLane())
        {
            ScratchBuffer.InterlockedAdd(args.dynamicBlockIndexScratchOffset, 1, blockId);
        }

        blockId = WaveReadLaneFirst(blockId);

        if (blockId >= numBlocks)
        {
            break;
        }

        int localKeys[TS_KEYS_PER_THREAD];

        const uint rangeBegin = blockId * TS_KEYS_PER_GROUP;

        const uint smax = args.maxNumPrimitives - args.numPrimitives;

        for (uint i = 0; i < TS_KEYS_PER_THREAD; i++)
        {
            const uint readIndex = rangeBegin + (localId * TS_KEYS_PER_THREAD) + i;
            uint count = 0;

            if (readIndex < args.numPrimitives)
            {
                const float priority = ScratchBuffer.Load<float>(args.splitPrioritiesScratchOffset
                                                                 + sizeof(float) * readIndex);

                uint numSplits = (priority / globalSum) * smax;
                if (args.tsBudgetPerTriangle > 0)
                {
                    numSplits = min(numSplits, args.tsBudgetPerTriangle);
                }

                count = numSplits;
            }

            localKeys[i] = count;
        }

        int threadSumLeft = 0;

        // Calculate scan on this thread's elements
        for (uint i = 0; i < TS_KEYS_PER_THREAD; i++)
        {
            int tmp = localKeys[i];
            localKeys[i] = threadSumLeft;
            threadSumLeft += tmp;
        }

        // Scan partial sums
        int threadSumScannedLeft = WavePrefixSum(threadSumLeft);

        // Add partial sums back
        for (uint i = 0; i < TS_KEYS_PER_THREAD; i++)
        {
            localKeys[i] += threadSumScannedLeft;
        }

        // Wait until previous dynamic block finished and acquire its sum.
        int prevSumLeft = 0;

        if (blockId > 0)
        {
            if (localId == (BUILD_THREADGROUP_SIZE - 1))
            {
                while (1)
                {
                    uint orig = InterlockedCmpxDataValid(args, blockId - 1, 1, 1);

                    if (orig != 0)
                    {
                        break;
                    }
                }

                prevSumLeft = ReadPrefixSum(args, blockId - 1);
            }

            prevSumLeft = WaveReadLaneAt(prevSumLeft, BUILD_THREADGROUP_SIZE - 1);
        }

        GroupMemoryBarrierWithGroupSync();

        // Write our sum and enable successive blocks.
        if (localId == (BUILD_THREADGROUP_SIZE - 1))
        {
            Flags flags;
            flags.prefixSum = prevSumLeft + threadSumScannedLeft + threadSumLeft;
            flags.dataValid = 1;

            WriteFlags(args, blockId, flags);
        }

        GroupMemoryBarrierWithGroupSync();

        // Perform coalesced writes back to global memory
        for (uint i = 0; i < TS_KEYS_PER_THREAD; i++)
        {
            const uint storeIndex = rangeBegin + (localId * TS_KEYS_PER_THREAD) + i;
            const uint valueLeft = localKeys[i];

            if (storeIndex < args.numPrimitives)
            {
                ScratchNode node = FetchScratchNode(args.scratchLeafNodesScratchOffset, storeIndex);

                const BoundingBox bbox = GetScratchNodeBoundingBox(node);

                const float priority = ScratchBuffer.Load<float>(args.splitPrioritiesScratchOffset
                                                                 + sizeof(float) * storeIndex);

                uint numSplits = (priority / globalSum) * smax;
                if (args.tsBudgetPerTriangle > 0)
                {
                    numSplits = min(numSplits, args.tsBudgetPerTriangle);
                }

                // write ref
                ScratchTSRef ref;

                ref.leafIndex = storeIndex;
                ref.bbox = bbox;
                ref.numSplits = numSplits;
                ref.splitLeafBaseIndex = ((args.numPrimitives + valueLeft + prevSumLeft) << 1) | 0x1;

                WriteRefList(args, 0, storeIndex, ref);
            }
        }
    }
}

//=====================================================================================================================
uint3 CalculateVariableAxisBitCount64(float3 sceneExtent,
                                      uint numSizeBits)
{
    int numMortonBits = 62;

    if (numSizeBits > 0)
    {
        numMortonBits -= numSizeBits;
    }

    int3 numBits = uint3(0, 0, 0);

    int i;

    int3 numPrebits;
    int3 startAxis;

    // find the largest start axis
    // and how many prebits are needed between largest and two other axes
    if (sceneExtent.x < sceneExtent.y)
    {
        if (sceneExtent.x < sceneExtent.z)
        {
            if (sceneExtent.y < sceneExtent.z)
            {
                // z, y, x
                startAxis[0] = 2;
                numPrebits[0] = log2(sceneExtent.z / sceneExtent.y);

                startAxis[1] = 1;
                numPrebits[1] = log2(sceneExtent.y / sceneExtent.x);

                startAxis[2] = 0;
                numPrebits[2] = log2(sceneExtent.z / sceneExtent.x);
            }
            else
            {
                // y, z, x
                startAxis[0] = 1;
                numPrebits[0] = log2(sceneExtent.y / sceneExtent.z);

                startAxis[1] = 2;
                numPrebits[1] = log2(sceneExtent.z / sceneExtent.x);

                startAxis[2] = 0;
                numPrebits[2] = log2(sceneExtent.y / sceneExtent.x);
            }
        }
        else
        {
            // y, x, z
            startAxis[0] = 1;
            numPrebits[0] = log2(sceneExtent.y / sceneExtent.x);

            startAxis[1] = 0;
            numPrebits[1] = log2(sceneExtent.x / sceneExtent.z);

            startAxis[2] = 2;
            numPrebits[2] = log2(sceneExtent.y / sceneExtent.z);
        }
    }
    else
    {
        if (sceneExtent.y < sceneExtent.z)
        {
            if (sceneExtent.x < sceneExtent.z)
            {
                // z, x, y
                startAxis[0] = 2;
                numPrebits[0] = log2(sceneExtent.z / sceneExtent.x);

                startAxis[1] = 0;
                numPrebits[1] = log2(sceneExtent.x / sceneExtent.y);

                startAxis[2] = 1;
                numPrebits[2] = log2(sceneExtent.z / sceneExtent.y);
            }
            else
            {
                // x, z, y
                startAxis[0] = 0;
                numPrebits[0] = log2(sceneExtent.x / sceneExtent.z);

                startAxis[1] = 2;
                numPrebits[1] = log2(sceneExtent.z / sceneExtent.y);

                startAxis[2] = 1;
                numPrebits[2] = log2(sceneExtent.x / sceneExtent.y);
            }
        }
        else
        {
            // x, y, z
            startAxis[0] = 0;
            numPrebits[0] = log2(sceneExtent.x / sceneExtent.y);

            startAxis[1] = 1;
            numPrebits[1] = log2(sceneExtent.y / sceneExtent.z);

            startAxis[2] = 2;
            numPrebits[2] = log2(sceneExtent.x / sceneExtent.z);
        }
    }

    if (sceneExtent[startAxis[2]] == 0)
    {
        numPrebits[1] = 0;
        numPrebits[2] = 0;
    }

    // say x > y > z
    // prebits[0] = 3
    // prebits[1] = 2
    // if swap == 1
    // xxx xy xy x yxz yxz ...
    // if swap == 0
    // xxx xy xy xyz xyz ...
    int swap = numPrebits[2] > (numPrebits[0] + numPrebits[1]) ? 1 : 0;

    numPrebits[0] = min(numPrebits[0], numMortonBits);
    numPrebits[1] = min(numPrebits[1] * 2, numMortonBits - numPrebits[0]) / 2;

    int numPrebitsSum = numPrebits[0] + numPrebits[1] * 2;

    if (numPrebitsSum != numMortonBits)
    {
        numPrebitsSum += swap;
    }
    else
    {
        swap = 0;
    }

    // the scene might be 2D so check for the smallest axis
    numBits[2] = (sceneExtent[startAxis[2]] != 0) ? max(0, (numMortonBits - numPrebitsSum) / 3) : 0;

    if (swap > 0)
    {
        numBits[0] = max(0, (numMortonBits - numBits[2] - numPrebitsSum) / 2 + numPrebits[1] + numPrebits[0] + 1);
        numBits[1] = numMortonBits - numBits[0] - numBits[2];
    }
    else
    {
        numBits[1] = max(0, (numMortonBits - numBits[2] - numPrebitsSum) / 2 + numPrebits[1]);
        numBits[0] = numMortonBits - numBits[1] - numBits[2];
    }

    const int delta = numBits[0] - 31; // clamp axis values to avoid overflow of a uint

    if (delta > 0)
    {
        numBits[0] -= delta;

        numPrebits[0] = min(numPrebits[0], numBits[0]);

        if (numBits[0] == numPrebits[0])
            swap = 0;

        numBits[1] = max(0, numBits[1] - delta);

        numPrebits[1] = min(numPrebits[1], numBits[1]);

        numBits[2] = max(0, numBits[2] - delta);

        numPrebitsSum = numPrebits[0] + numPrebits[1] * 2 + swap;
    }

    uint3 outputNumBits;

    outputNumBits[startAxis[0]] = numBits[0];
    outputNumBits[startAxis[1]] = numBits[1];
    outputNumBits[startAxis[2]] = numBits[2];

    return outputNumBits;
}

//=====================================================================================================================
void WriteScratchNodeSplitBoxIndex(uint baseScratchNodesOffset,
                                   uint nodeIndex,
                                   uint splitBoxIndex)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    ScratchBuffer.Store(nodeOffset + SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET, splitBoxIndex);
}

//=====================================================================================================================
// Main Function : TriangleSplittingImpl
//=====================================================================================================================
void TriangleSplittingImpl(
    uint                    globalId,
    uint                    localId,
    uint                    groupId,
    TriangleSplittingArgs   args)
{
    const uint taskQueueOffset      = args.taskQueueCounterScratchOffset;
    const uint refListIndexOffset   = args.currentStateScratchOffset + STATE_TS_REF_LIST_INDEX_OFFSET;
    const uint numRefsOffset        = args.currentStateScratchOffset + STATE_TS_NUM_REFS_OFFSET;
    const uint numRefsAllocOffset   = args.currentStateScratchOffset + STATE_TS_NUM_REFS_ALLOC_OFFSET;
    const uint sumValueOffset       = args.currentStateScratchOffset + STATE_TS_SUM_OFFSET;
    const uint mutexOffset          = args.currentStateScratchOffset + STATE_TS_MUTEX_OFFSET;

    if (args.numPrimitives == 0)
    {
        return;
    }

    const uint numGroups = args.numThreadGroups;

    const uint numThreads = numGroups * BUILD_THREADGROUP_SIZE;

    if (globalId == 0)
    {
        AllocTasks(numGroups, TS_PHASE_INIT, taskQueueOffset);
    }

    while (1)
    {
        const uint2 task = BeginTask(localId, taskQueueOffset);

        const uint taskIndex = task.x;
        const uint phase = task.y;

        groupId = taskIndex;
        globalId = taskIndex * BUILD_THREADGROUP_SIZE + localId;

        uint i, a;

        switch (phase)
        {
            case TS_PHASE_INIT:
            {
                InitPrefixSum(args, globalId, args.numPrimitives);

                if (globalId == 0)
                {
                    ScratchBuffer.Store<uint>(refListIndexOffset, 0);
                    ScratchBuffer.Store<uint>(numRefsOffset, args.numPrimitives);
                    ScratchBuffer.Store<uint>(numRefsAllocOffset, 0);
                    ScratchBuffer.Store<float>(sumValueOffset, 0);
                    ScratchBuffer.Store<uint>(mutexOffset, 0);
                }

                if (EndTask(localId, taskQueueOffset))
                {
                    AllocTasks(numGroups, TS_PHASE_CALC_SUM, taskQueueOffset);
                }
                break;
            }

            case TS_PHASE_CALC_SUM:
            {
                for (uint i = globalId; i < args.numPrimitives; i += numThreads)
                {
                    float priority;

                    ScratchNode node = FetchScratchNode(args.scratchLeafNodesScratchOffset, i);

                    if ((node.flags_and_instanceMask & SCRATCH_NODE_FLAGS_DISABLE_TRIANGLE_SPLIT_MASK) ||
                        (IsNodeActive(node) == false) ||
                        (IsScratchTriangleNode(node) == false))
                    {
                        priority = 0;
                    }
                    else
                    {
                        // calculate priority
                        const float3 dir1 = node.bbox_max_or_v1 - node.bbox_min_or_v0;
                        const float3 dir2 = node.sah_or_v2_or_instBasePtr - node.bbox_min_or_v0;

                        const float3 dir3 = cross(dir1, dir2);

                        // ideal surface area of the primitive
                        const float Aideal = abs(dir3.x) + abs(dir3.y) + abs(dir3.z);

                        const BoundingBox bbox = GenerateTriangleBoundingBox(node.bbox_min_or_v0,
                                                                             node.bbox_max_or_v1,
                                                                             node.sah_or_v2_or_instBasePtr);

                        const float surfaceArea = ComputeBoxSurfaceArea(bbox);

                        // priority = surfaceArea - Aideal;
                        priority = pow(surfaceArea - Aideal, Settings.tsPriority);

                        const float waveSum = WaveActiveSum(priority);

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

                    ScratchBuffer.Store<float>(args.splitPrioritiesScratchOffset + sizeof(float) * i, priority);
                }

                if (EndTask(localId, taskQueueOffset))
                {
                    AllocTasks(RoundUpQuotient(args.numPrimitives, TS_KEYS_PER_GROUP),
                                 TS_PHASE_ALLOC_REFS, taskQueueOffset);
                }
                break;
            }

            case TS_PHASE_ALLOC_REFS:
            {
                const float globalSum = ScratchBuffer.Load<float>(sumValueOffset);

                AllocRefs(args, localId, globalSum);

                if (EndTask(localId, taskQueueOffset))
                {
                    AllocTasks(numGroups, TS_PHASE_SPLIT, taskQueueOffset);
                }
                break;
            }

            case TS_PHASE_SPLIT:
            {
                const uint refListIndex = ScratchBuffer.Load(refListIndexOffset);
                const uint numRefs = ScratchBuffer.Load(numRefsOffset);

                const BoundingBox sceneAABB = FetchSceneBounds(args.sceneBoundsByteOffset);
                const float3 sceneExtents = sceneAABB.max - sceneAABB.min;

                const uint3 numAxisBits = CalculateVariableAxisBitCount64(sceneExtents, args.numSizeBits);

                const uint basePrimNodePtrsOffset =
                    ReadAccelStructHeaderField(
                        ACCEL_STRUCT_HEADER_OFFSETS_OFFSET + ACCEL_STRUCT_OFFSETS_PRIM_NODE_PTRS_OFFSET);

                for (uint i = globalId; i < numRefs; i += numThreads)
                {
                    const uint index = i;

                    ScratchTSRef ref = ReadRefList(args, refListIndex, index);

                    uint largestAxis = 0;
                    float largestLength = ref.bbox.max[0] - ref.bbox.min[0];

                    [unroll]
                    for (uint a = 1; a < 3; a++)
                    {
                        const float length = ref.bbox.max[a] - ref.bbox.min[a];

                        if (length > largestLength)
                        {
                            largestAxis = a;
                            largestLength = length;
                        }
                    }

                    const float cellSize = sceneExtents[largestAxis] / (1UL << numAxisBits[largestAxis]);

                    const uint gridLength = largestLength / cellSize;

                    if ((ref.numSplits > 0) && (ref.leafIndex != INVALID_IDX))
                    {
                        float splitPlane;

                        if (gridLength > 1)
                        {
                            const float center = (ref.bbox.min[largestAxis] + ref.bbox.max[largestAxis]) / 2.0;

                            const float length = ref.bbox.max[largestAxis] - ref.bbox.min[largestAxis];

                            const uint exp = log2((length/2.0) / cellSize);

                            uint stride = 1UL << exp;

                            float centerWithStride;

                            bool isDone;

                            do
                            {
                                const float newCellSize = cellSize * stride;

                                centerWithStride = uint((center - sceneAABB.min[largestAxis]) / newCellSize) * newCellSize + sceneAABB.min[largestAxis];

                                float leftLength = centerWithStride - ref.bbox.min[largestAxis];
                                float rightLength = ref.bbox.max[largestAxis] - centerWithStride;

                                // invalid split
                                if (leftLength < cellSize || rightLength < cellSize)
                                {
                                    isDone = false;
                                }
                                else
                                {
                                    if (leftLength < rightLength)
                                    {
                                        isDone = rightLength < (2 * leftLength);
                                    }
                                    else
                                    {
                                        isDone = leftLength < (2 * rightLength);
                                    }
                                }

                                stride >>= 1;
                            } while (!isDone);

                            splitPlane = centerWithStride;
                        }
                        else
                        {
                            splitPlane = (ref.bbox.min[largestAxis] + ref.bbox.max[largestAxis]) / 2.0;
                        }

                        BoundingBox left = ref.bbox;
                        BoundingBox right = ref.bbox;

                        left.max[largestAxis] = min(splitPlane, left.max[largestAxis]);
                        right.min[largestAxis] = max(splitPlane, right.min[largestAxis]);

                        // generate bboxes around split triangle
                        BoundingBox leftTri;
                        BoundingBox rightTri;

                        leftTri.min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
                        leftTri.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

                        rightTri.min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
                        rightTri.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

                        ScratchNode leafNode = FetchScratchNode(args.scratchLeafNodesScratchOffset,
                                                                ref.leafIndex);

                        // v0->v1
                        float3 dir = leafNode.bbox_max_or_v1 - leafNode.bbox_min_or_v0;

                        float dist = (splitPlane - leafNode.bbox_min_or_v0[largestAxis]) / dir[largestAxis];

                        if (leafNode.bbox_min_or_v0[largestAxis] <= splitPlane)
                        {
                            leftTri.min = min(leafNode.bbox_min_or_v0, leftTri.min);
                            leftTri.max = max(leafNode.bbox_min_or_v0, leftTri.max);
                        }

                        if (leafNode.bbox_min_or_v0[largestAxis] >= splitPlane)
                        {
                            rightTri.min = min(leafNode.bbox_min_or_v0, rightTri.min);
                            rightTri.max = max(leafNode.bbox_min_or_v0, rightTri.max);
                        }

                        if ((dist > 0) && (dist <= 1))
                        {
                            float3 intersection = dir * dist + leafNode.bbox_min_or_v0;

                            leftTri.min = min(intersection, leftTri.min);
                            leftTri.max = max(intersection, leftTri.max);

                            rightTri.min = min(intersection, rightTri.min);
                            rightTri.max = max(intersection, rightTri.max);
                        }

                        // v1->v2
                        dir = leafNode.sah_or_v2_or_instBasePtr - leafNode.bbox_max_or_v1;

                        dist = (splitPlane - leafNode.bbox_max_or_v1[largestAxis]) / dir[largestAxis];

                        if (leafNode.bbox_max_or_v1[largestAxis] <= splitPlane)
                        {
                            leftTri.min = min(leafNode.bbox_max_or_v1, leftTri.min);
                            leftTri.max = max(leafNode.bbox_max_or_v1, leftTri.max);
                        }

                        if (leafNode.bbox_max_or_v1[largestAxis] >= splitPlane)
                        {
                            rightTri.min = min(leafNode.bbox_max_or_v1, rightTri.min);
                            rightTri.max = max(leafNode.bbox_max_or_v1, rightTri.max);
                        }

                        if ((dist > 0) && (dist <= 1))
                        {
                            float3 intersection = dir * dist + leafNode.bbox_max_or_v1;

                            leftTri.min = min(intersection, leftTri.min);
                            leftTri.max = max(intersection, leftTri.max);

                            rightTri.min = min(intersection, rightTri.min);
                            rightTri.max = max(intersection, rightTri.max);
                        }

                        // v2->v0
                        dir = leafNode.bbox_min_or_v0 - leafNode.sah_or_v2_or_instBasePtr;

                        dist = (splitPlane - leafNode.sah_or_v2_or_instBasePtr[largestAxis]) / dir[largestAxis];

                        if (leafNode.sah_or_v2_or_instBasePtr[largestAxis] <= splitPlane)
                        {
                            leftTri.min = min(leafNode.sah_or_v2_or_instBasePtr, leftTri.min);
                            leftTri.max = max(leafNode.sah_or_v2_or_instBasePtr, leftTri.max);
                        }

                        if (leafNode.sah_or_v2_or_instBasePtr[largestAxis] >= splitPlane)
                        {
                            rightTri.min = min(leafNode.sah_or_v2_or_instBasePtr, rightTri.min);
                            rightTri.max = max(leafNode.sah_or_v2_or_instBasePtr, rightTri.max);
                        }

                        if ((dist > 0) && (dist <= 1))
                        {
                            float3 intersection = dir * dist + leafNode.sah_or_v2_or_instBasePtr;

                            leftTri.min = min(intersection, leftTri.min);
                            leftTri.max = max(intersection, leftTri.max);

                            rightTri.min = min(intersection, rightTri.min);
                            rightTri.max = max(intersection, rightTri.max);
                        }

                        left.min = max(left.min, leftTri.min);
                        left.max = min(left.max, leftTri.max);

                        right.min = max(right.min, rightTri.min);
                        right.max = min(right.max, rightTri.max);

                        // calc how to partition num splits
                        float wa = -FLT_MAX;
                        float wb = -FLT_MAX;

                        [unroll]
                        for (uint a = 0; a < 3; a++)
                        {
                            float length = left.max[a] - left.min[a];

                            if (length > wa)
                            {
                                wa = length;
                            }

                            length = right.max[a] - right.min[a];

                            if (length > wb)
                            {
                                wb = length;
                            }
                        }

                        const uint sa = (ref.numSplits - 1) * wa / (wa + wb) + 0.5;
                        const uint sb = ref.numSplits - 1 - sa;

                        const uint waveSum = WaveActiveSum(2);
                        const int wavePrefixSum = WavePrefixSum(2);

                        uint waveOffset = 0;

                        if (WaveIsFirstLane())
                        {
                            ScratchBuffer.InterlockedAdd(numRefsAllocOffset, waveSum, waveOffset);
                        }

                        waveOffset = WaveReadLaneFirst(waveOffset);

                        const uint index = waveOffset + wavePrefixSum;

                        ScratchTSRef refLeft;

                        refLeft.leafIndex = ref.leafIndex;
                        refLeft.numSplits = sa;
                        refLeft.splitLeafBaseIndex = ref.splitLeafBaseIndex;
                        refLeft.bbox = left;

                        uint leftLeafUsage = sa;

                        // check if NOT using the original leaf
                        if ((ref.splitLeafBaseIndex & 0x1) == 0)
                        {
                            leftLeafUsage++;
                        }

                        WriteRefList(args, !refListIndex, index, refLeft);

                        ScratchTSRef refRight;

                        refRight.leafIndex = ref.leafIndex;
                        refRight.numSplits = sb;
                        refRight.splitLeafBaseIndex = ((refLeft.splitLeafBaseIndex >> 1) + leftLeafUsage) << 1;
                        refRight.bbox = right;

                        WriteRefList(args, !refListIndex, index + 1, refRight);
                    }
                    else // can't split
                    {
                        // can't find a split plane
                        if(ref.numSplits > 0)
                        {
                            const uint waveSum = WaveActiveSum(2);
                            const int wavePrefixSum = WavePrefixSum(2);

                            uint waveOffset = 0;

                            if (WaveIsFirstLane())
                            {
                                ScratchBuffer.InterlockedAdd(numRefsAllocOffset, waveSum, waveOffset);
                            }

                            waveOffset = WaveReadLaneFirst(waveOffset);

                            const uint index = waveOffset + wavePrefixSum;

                            ScratchTSRef refLeft;

                            refLeft.leafIndex = ref.leafIndex;
                            refLeft.numSplits = 0;
                            refLeft.splitLeafBaseIndex = ref.splitLeafBaseIndex;
                            refLeft.bbox = ref.bbox;

                            uint leftLeafUsage = 0;

                            // check if NOT using the original leaf
                            if ((ref.splitLeafBaseIndex & 0x1) == 0)
                            {
                                leftLeafUsage++;
                            }

                            WriteRefList(args, !refListIndex, index, refLeft);

                            const float3 nan3 = float3(NaN, NaN, NaN);

                            ScratchTSRef refRight;

                            refRight.leafIndex = INVALID_IDX;
                            refRight.numSplits = ref.numSplits - 1;
                            refRight.splitLeafBaseIndex = ((refLeft.splitLeafBaseIndex >> 1) + leftLeafUsage) << 1;
                            refRight.bbox.min = nan3;
                            refRight.bbox.max = nan3;

                            WriteRefList(args, !refListIndex, index + 1, refRight);
                        }
                        else // no more splits...done (alloc leaf node and point to its split box)
                        {
                            if (ref.leafIndex == INVALID_IDX) // deactivate
                            {
                                const uint writeIndex = ref.splitLeafBaseIndex >> 1;

                                WriteSplitBoxAtIndex(args.splitBoxesScratchOffset, writeIndex, ref.bbox);

                                WriteScratchNodeSplitBoxIndex(args.scratchLeafNodesScratchOffset,
                                                              writeIndex,
                                                              writeIndex);

                                const uint extraPrimNodePtrOffset = basePrimNodePtrsOffset + (writeIndex * sizeof(uint));
                                DstBuffer.Store(extraPrimNodePtrOffset, INVALID_IDX);

                            }
                            else if (ref.splitLeafBaseIndex & 0x1) // update original leaf
                            {
                                const float surfaceArea = ComputeBoxSurfaceArea(ref.bbox);

                                WriteSplitBoxAtIndex(args.splitBoxesScratchOffset, ref.leafIndex, ref.bbox);

                                const float sahCost = surfaceArea * SAH_COST_TRIANGLE_INTERSECTION;

                                WriteScratchNodeCost(args.scratchLeafNodesScratchOffset,
                                                     ref.leafIndex,
                                                     sahCost,
                                                     true);

                                WriteScratchNodeSplitBoxIndex(args.scratchLeafNodesScratchOffset,
                                                              ref.leafIndex,
                                                              ref.leafIndex);

                                UpdateSceneSize(args.sceneBoundsByteOffset + 24, surfaceArea);
                            }
                            else // copy original box to split boxes
                            {
                                const float surfaceArea = ComputeBoxSurfaceArea(ref.bbox);

                                ScratchNode leafNode = FetchScratchNode(args.scratchLeafNodesScratchOffset,
                                                                        ref.leafIndex);

                                const uint writeIndex = ref.splitLeafBaseIndex >> 1;

                                leafNode.splitBox_or_nodePointer = writeIndex;

                                leafNode.numPrimitivesAndDoCollapse = asuint(surfaceArea * SAH_COST_TRIANGLE_INTERSECTION);

                                WriteScratchNode(args.scratchLeafNodesScratchOffset, writeIndex, leafNode);

                                WriteSplitBoxAtIndex(args.splitBoxesScratchOffset, writeIndex, ref.bbox);

                                const uint extraPrimNodePtrOffset = basePrimNodePtrsOffset + (writeIndex * sizeof(uint));
                                DstBuffer.Store(extraPrimNodePtrOffset, INVALID_IDX);

                                UpdateSceneSize(args.sceneBoundsByteOffset + 24, surfaceArea);
                            }
                        }
                    }
                }

                if (EndTask(localId, taskQueueOffset))
                {
                    ScratchBuffer.Store(refListIndexOffset, !refListIndex);

                    const uint numRefsAlloc = ScratchBuffer.Load(numRefsAllocOffset);

                    if (numRefsAlloc == 0)
                    {
                        AllocTasks(numGroups, TS_PHASE_DONE, taskQueueOffset);
                    }
                    else
                    {
                        IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET, numRefsAlloc/2);

                        ScratchBuffer.Store(numRefsAllocOffset, 0);

                        ScratchBuffer.Store(numRefsOffset, numRefsAlloc);

                        AllocTasks(numGroups, TS_PHASE_SPLIT, taskQueueOffset);
                    }
                }
                break;
            }

            case TS_PHASE_DONE:
                return;
        }
    }
}
