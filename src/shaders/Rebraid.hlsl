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
//
// Newer Rebraid algorithm:
//
// Uses 2 types of heuristics for prioritizing openings (ENABLE_HIGHER_QUALITY)
// Can control number of iterations vis build settings (args.numIterations)

#if NO_SHADER_ENTRYPOINT == 0
#define RootSig "RootConstants(num32BitConstants=1, b0),"\
                "CBV(b1),"\
                "UAV(u0),"\
                "UAV(u1),"\
                "UAV(u2),"\
                "UAV(u3),"\
                "UAV(u4),"\
                "UAV(u5),"\
                "UAV(u6),"\
                "UAV(u7),"\
                "CBV(b255)"

#include "../shared/rayTracingDefs.h"

struct RootConstants
{
    uint numThreadGroups;
};

[[vk::push_constant]] ConstantBuffer<RootConstants> ShaderRootConstants    : register(b0);
[[vk::binding(1, 1)]] ConstantBuffer<BuildShaderConstants> ShaderConstants : register(b1);

[[vk::binding(0, 0)]] RWByteAddressBuffer                          SrcBuffer           : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer         DstBuffer           : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer         DstMetadata         : register(u2);
[[vk::binding(3, 0)]] globallycoherent RWByteAddressBuffer         ScratchBuffer       : register(u3);
[[vk::binding(4, 0)]] globallycoherent RWByteAddressBuffer         ScratchGlobal       : register(u4);
[[vk::binding(5, 0)]] RWByteAddressBuffer                          InstanceDescBuffer  : register(u5);
[[vk::binding(6, 0)]] RWByteAddressBuffer                          EmitBuffer          : register(u6);
[[vk::binding(7, 0)]] RWByteAddressBuffer                          IndirectArgBuffer   : register(u7);

template<typename T>
T LoadInstanceDescBuffer(uint offset)
{
    return InstanceDescBuffer.Load<T>(offset);
}
#include "IndirectArgBufferUtils.hlsl"
#include "BuildCommonScratch.hlsl"
#include "EncodeTopLevelCommon.hlsl"

#define MAX_LDS_ELEMENTS (16 * BUILD_THREADGROUP_SIZE)
groupshared uint SharedMem[MAX_LDS_ELEMENTS];

#include "TaskQueueCounter.hlsl"

#include "RadixSort/ScanExclusiveInt4DLBCommon.hlsl"
#endif

#define LOG_BASE_N                          10
#define LIMIT_OPENINGS_BASED_ON_ITERATION   1

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
    uint enableMortonSize;
    uint numIterations;
    uint qualityHeuristic;
};

//=====================================================================================================================
// Fetch bottom-level acceleration structure root child information.
void FetchBlasRootChildInfo(
    uint64_t               blasBaseAddr,
    uint32_t               nodePtr,
    out_param(BoundingBox) bbox[4],
    out_param(uint4)       child)
{
    {
        Float32BoxNode node;

        if (GetNodeType(nodePtr) == NODE_TYPE_BOX_FLOAT32)
        {
            node = FetchFloat32BoxNode(blasBaseAddr,
                                       nodePtr);
        }
        else
        {
            node = FetchFloat16BoxNodeAsFp32(blasBaseAddr,
                                             nodePtr);
        }
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
float CalculateInstanceChildrenSA(RebraidArgs args, uint nodePtr, ScratchNode leaf, uint instanceDescOffsetInBytes)
{
    const InstanceDesc desc = LoadInstanceDesc(leaf.left_or_primIndex_or_instIndex, instanceDescOffsetInBytes);

    const GpuVirtualAddress address = GetInstanceAddr(asuint(leaf.sah_or_v2_or_instBasePtr.x),
                                                      asuint(leaf.sah_or_v2_or_instBasePtr.y));

    uint4 child = uint4(INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX);

    BoundingBox bbox[4];
    FetchBlasRootChildInfo(address, nodePtr, bbox, child);

    float surfaceArea = 0;
    for (uint i = 0; i < 4; i++)
    {
        if (child[i] != INVALID_IDX)
        {
            BoundingBox temp = TransformBoundingBox(bbox[i], desc.Transform);
            if ((IsInvalidBoundingBox(bbox[i]) == false) && (IsCorruptBox(temp) == false))
            {
                float currentSurfaceArea = ComputeBoxSurfaceArea(temp);
                surfaceArea += currentSurfaceArea;
            }
        }
    }

    return surfaceArea;
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

    const uint numLeafIndicesOffset      = args.stateScratchOffset + STATE_REBRAID_NUM_LEAF_INDICES_OFFSET;
    const uint iterationCountOffset      = args.stateScratchOffset + STATE_REBRAID_ITERATION_COUNT_OFFSET;

    if (args.numPrimitives == 0)
    {
        return;
    }

    ///////// init /////////
    const uint instanceDescOffsetInBytes = LoadNumPrimAndOffset().primitiveOffset;

    const uint numGroups = args.numThreadGroups;

    const uint numThreads = numGroups * BUILD_THREADGROUP_SIZE;

    if (globalId == 0)
    {
        ScratchBuffer.Store<float2>(sumValueOffset, float2(0, 0));
        ScratchBuffer.Store<uint>(mutexOffset, 0);

        ScratchBuffer.Store<uint>(numLeafIndicesOffset, args.numPrimitives);
        ScratchBuffer.Store<uint>(iterationCountOffset, 0);

        DeviceMemoryBarrier();

        AllocTasks(numGroups, REBRAID_PHASE_CALC_SUM, taskQueueOffset);
    }

    const uint rootNodePointer = CreateRootNodePointer();

    // more than 1 iteration is not yet supported
    // since only the first level is up to 4 children

    while (1)
    {
        const uint2 task = BeginTask(localId, taskQueueOffset);

        const uint taskIndex = task.x;
        const uint phase = task.y;

        groupId = taskIndex;
        globalId = taskIndex * BUILD_THREADGROUP_SIZE + localId;

        switch (phase)
        {
            // calculates the priorities based on surface of all instances and also the sum of all priorities
            case REBRAID_PHASE_CALC_SUM:
            {
                const uint iterationCount = ScratchBuffer.Load(iterationCountOffset);
                const uint numPrims       = ScratchBuffer.Load(numLeafIndicesOffset);

                // init for next phase
                const uint numStages = RoundUpQuotient(numPrims, REBRAID_KEYS_PER_GROUP);
                for (uint stage = globalId; stage < numStages; stage += numThreads)
                {
                    for (uint type = 0; type < NUM_DLB_VALID_TYPES; type++)
                    {
                        Flags flags;
                        flags.dataValid = 0;
                        flags.prefixSum = 0;

                        WriteFlagsDLB(args.atomicFlagsScratchOffset, stage, type, flags);
                    }
                }

                if (iterationCount == 0)
                {
                    float localSum = 0;

                    const uint currentSumValueOffset = sumValueOffset + ((iterationCount & 0x1) * sizeof(float));

                    for (uint i = globalId; i < numPrims; i += numThreads)
                    {
                        const ScratchNode leaf = FetchScratchNode(args.bvhLeafNodeDataScratchOffset, i);

                        if (IsNodeActive(leaf))
                        {
                            const BoundingBox aabb = GetScratchNodeInstanceBounds(leaf);

                            const uint nodePtr = leaf.splitBox_or_nodePointer == 0 ?
                                                 rootNodePointer : leaf.splitBox_or_nodePointer;

                            if (IsBoxNode(nodePtr))
                            {
                                float instanceSA;

                                if (args.qualityHeuristic == 0)
                                {
                                    const float childSA = CalculateInstanceChildrenSA(args, nodePtr, leaf, instanceDescOffsetInBytes);
                                    instanceSA = log(max(0, ComputeBoxSurfaceArea(aabb) - childSA) + 1)
                                                    / log(LOG_BASE_N);
                                }
                                else
                                {
                                    instanceSA = log(ComputeBoxSurfaceArea(aabb) + 1)
                                                    / log(LOG_BASE_N);
                                }

                                localSum += instanceSA;
                            }
                        }
                    }

                    const float waveSum = WaveActiveSum(localSum);
                    if (WaveIsFirstLane())
                    {
                        // mutex lock
                        uint orig = 1;

                        do {
                            DeviceMemoryBarrier();
                            ScratchBuffer.InterlockedCompareExchange(mutexOffset, 0, 1, orig);
                        } while (orig);

                        const float globalSum = ScratchBuffer.Load<float>(currentSumValueOffset);

                        ScratchBuffer.Store<float>(currentSumValueOffset, globalSum + waveSum);

                        DeviceMemoryBarrier();

                        ScratchBuffer.Store(mutexOffset, 0); //unlock
                    }
                }

                if (EndTask(localId, taskQueueOffset))
                {
                    AllocTasks(RoundUpQuotient(numPrims, REBRAID_KEYS_PER_GROUP),
                               REBRAID_PHASE_OPEN, taskQueueOffset);
                }
                break;
            }

            // allows a single opening to an instance based on if the numOpenings >= 1
            case REBRAID_PHASE_OPEN:
            {
                const uint iterationCount = ScratchBuffer.Load(iterationCountOffset);
                const uint numPrims = ScratchBuffer.Load(numLeafIndicesOffset);

                const uint readSumValueOffset = sumValueOffset + ((iterationCount & 0x1) * sizeof(float));
                const uint writeSumValueOffset = sumValueOffset + (((iterationCount + 1) & 0x1) * sizeof(float));

                const float globalSum = ScratchBuffer.Load<float>(readSumValueOffset);

#if LIMIT_OPENINGS_BASED_ON_ITERATION
                const uint allocSpace = (args.maxNumPrims - numPrims) / (args.numIterations - iterationCount);
                uint maxOpenings = allocSpace / 3;
#else
                uint maxOpenings = (args.maxNumPrims - numPrims) / 3;
#endif
                // makes sure there's no out of bounds access due to float precision
                maxOpenings = (maxOpenings > 0) ? maxOpenings - 1 : 0;

                const uint start = globalId * REBRAID_KEYS_PER_THREAD;
                const uint end = min(start + REBRAID_KEYS_PER_THREAD, numPrims);

                uint localKeys[REBRAID_KEYS_PER_THREAD];
                bool open[REBRAID_KEYS_PER_THREAD];

                uint keyIndex = 0;

                for (uint i = start; i < end; i++)
                {
                    const ScratchNode leaf = FetchScratchNode(args.bvhLeafNodeDataScratchOffset, i);

                    bool isOpen = false;

                    if (IsNodeActive(leaf) && (globalSum != 0))
                    {
                        const BoundingBox aabb = GetScratchNodeInstanceBounds(leaf);

                        const uint nodePtr = leaf.splitBox_or_nodePointer == 0 ?
                                             rootNodePointer : leaf.splitBox_or_nodePointer;

                        if (IsBoxNode(nodePtr))
                        {
                            float instanceSA;

                            if (args.qualityHeuristic == 0)
                            {
                                const float childSA = CalculateInstanceChildrenSA(args, nodePtr, leaf, instanceDescOffsetInBytes);
                                instanceSA = log(max(0, ComputeBoxSurfaceArea(aabb) - childSA) + 1)
                                                / log(LOG_BASE_N);
                            }
                            else
                            {
                                instanceSA = log(ComputeBoxSurfaceArea(aabb) + 1) / log(LOG_BASE_N);
                            }

                            const float value = instanceSA;

                            const uint numOpenings = maxOpenings * (value / globalSum);

                            if (numOpenings > 0)
                            {
                                const uint64_t instanceBasePointer =
                                    PackUint64(asuint(leaf.sah_or_v2_or_instBasePtr.x),
                                                asuint(leaf.sah_or_v2_or_instBasePtr.y));

                                const uint childCount =
                                    GetBlasRebraidChildCount(instanceBasePointer, nodePtr);

                                localKeys[keyIndex] = childCount - 1; // Additional children
                                open[keyIndex] = true;

                                isOpen = true;
                            }
                        }
                    }

                    if (isOpen == false)
                    {
                        localKeys[keyIndex] = 0;
                        open[keyIndex] = false;
                    }

                    keyIndex++;
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
                        Flags sum;
                        sum.prefixSum = threadSumScanned + threadSum;
                        sum.dataValid = 1;

                        WriteFlagsDLB(args.atomicFlagsScratchOffset, taskIndex, DLB_VALID_SUM, sum);

                        int readBackIndex = taskIndex - 1;

                        // checks previous blocks for prefix sum
                        // if the prefix sum is valid then it breaks out of loop
                        // if the prefix sum is NOT valid, then it fetches the SUM and continues to move backwards
                        while (readBackIndex >= 0)
                        {
                            if (ReadValidDLB(args.atomicFlagsScratchOffset, readBackIndex, DLB_VALID_PREFIX_SUM) != 0)
                            {
                                prevSum += ReadPrefixSumDLB(args.atomicFlagsScratchOffset, readBackIndex, DLB_VALID_PREFIX_SUM);

                                Flags flags;
                                flags.prefixSum = prevSum + threadSumScanned + threadSum;
                                flags.dataValid = 1;

                                WriteFlagsDLB(args.atomicFlagsScratchOffset, taskIndex, DLB_VALID_PREFIX_SUM, flags);
                                break;
                            }
                            else if (ReadValidDLB(args.atomicFlagsScratchOffset, readBackIndex, DLB_VALID_SUM) != 0)
                            {
                                prevSum += ReadPrefixSumDLB(args.atomicFlagsScratchOffset, readBackIndex, DLB_VALID_SUM);
                                readBackIndex--;
                            }

                            DeviceMemoryBarrier();
                        }

                        SharedMem[0] = prevSum;
                    }

                    GroupMemoryBarrierWithGroupSync();

                    prevSum = SharedMem[0];
                }     // Write our sum and enable successive blocks.
                else if (localId == (BUILD_THREADGROUP_SIZE - 1))
                {
                    Flags flags;
                    flags.prefixSum = threadSumScanned + threadSum;
                    flags.dataValid = 1;

                    WriteFlagsDLB(args.atomicFlagsScratchOffset, taskIndex, DLB_VALID_PREFIX_SUM, flags);
                }

                GroupMemoryBarrierWithGroupSync();

                keyIndex = 0;

                float localSum = 0;

                for (uint i = start; i < end; i++)
                {
                    const uint scratchNodeOffset = CalcScratchNodeOffset(args.bvhLeafNodeDataScratchOffset, i);

                    ScratchNode leaf = FetchScratchNodeAtOffset(scratchNodeOffset);

                    if (IsNodeActive(leaf))
                    {
                        if (open[keyIndex])
                        {
                            const GpuVirtualAddress address = GetInstanceAddr(asuint(leaf.sah_or_v2_or_instBasePtr.x),
                                                                              asuint(leaf.sah_or_v2_or_instBasePtr.y));

                            const InstanceDesc desc = LoadInstanceDesc(leaf.left_or_primIndex_or_instIndex, instanceDescOffsetInBytes);

                            uint4 child = uint4(INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX);

                            BoundingBox bbox[4];

                            const uint nodePtr = leaf.splitBox_or_nodePointer == 0 ?
                                                 rootNodePointer : leaf.splitBox_or_nodePointer;

                            FetchBlasRootChildInfo(address, nodePtr, bbox, child);
                            // update leaf and siblings
                            uint numSiblings = 0;

                            for (uint j = 0; j < 4; j++)
                            {
                                if (child[j] != INVALID_IDX)
                                {
                                    BoundingBox temp = TransformBoundingBox(bbox[j], desc.Transform);
                                    uint writeIndex;

                                    // update current leaf
                                    if (j == 0)
                                    {
                                        writeIndex = i;
                                    }
                                    else // alloc new leaf
                                    {
                                        // "numPrims" number if leaves in leaf array
                                        // + "prevSum" prefix sum from previous waves
                                        // + "localKeys[keyIndex]" prefix sum from threads in the wave
                                        // + "numSiblings" prefix sum of leaves belonging to this thread
                                        writeIndex = numPrims + prevSum + localKeys[keyIndex] + numSiblings;
                                        numSiblings++;
                                    }

                                    // TODO: remove from list for next iteration
                                    if (IsInvalidBoundingBox(bbox[j]) || IsCorruptBox(temp))
                                    {
                                        temp.min.x = NaN; // set inactive
                                    }
                                    else
                                    {
                                        if (iterationCount == (args.numIterations - 1))
                                        {
                                            if (args.enableMortonSize)
                                            {
                                                UpdateSceneBoundsWithSize(args.sceneBoundsOffset, temp);
                                            }
                                            else
                                            {
                                                UpdateSceneBounds(args.sceneBoundsOffset, temp);
                                            }
                                        }
                                        else
                                        {
                                            if(IsBoxNode(child[j]))
                                            {
                                                float instanceSA;
                                                if (args.qualityHeuristic == 0)
                                                {
                                                    const float childSA = CalculateInstanceChildrenSA(args, child[j], leaf, instanceDescOffsetInBytes);
                                                    instanceSA = log(max(0, ComputeBoxSurfaceArea(temp) - childSA) + 1) / log(LOG_BASE_N);
                                                }
                                                else
                                                {
                                                    instanceSA = log(ComputeBoxSurfaceArea(temp) + 1) / log(LOG_BASE_N);
                                                }

                                                localSum += instanceSA;
                                            }
                                        }
                                    }

                                    leaf.bbox_min_or_v0 = temp.min;
                                    leaf.bbox_max_or_v1 = temp.max;

                                    leaf.splitBox_or_nodePointer = child[j];

                                    WriteScratchNode(args.bvhLeafNodeDataScratchOffset, writeIndex, leaf);
                                }
                            }
                        }
                        else // no openings
                        {
                            // last iteration
                            if (iterationCount == (args.numIterations - 1))
                            {
                                if (leaf.splitBox_or_nodePointer == 0)
                                {
                                    // point instance to the root
                                    WriteScratchNodeDataAtOffset(
                                        scratchNodeOffset,
                                        SCRATCH_NODE_NODE_POINTER_OFFSET,
                                        CreateRootNodePointer());
                                }

                                const BoundingBox aabb = GetScratchNodeInstanceBounds(leaf);

                                if (args.enableMortonSize)
                                {
                                    UpdateSceneBoundsWithSize(args.sceneBoundsOffset, aabb);
                                }
                                else
                                {
                                    UpdateSceneBounds(args.sceneBoundsOffset, aabb);
                                }
                            }
                            else
                            {
                                const BoundingBox aabb = GetScratchNodeInstanceBounds(leaf);

                                // TODO: maybe avoid this logic by subtracting the SA when opening
                                const uint nodePtr = leaf.splitBox_or_nodePointer == 0 ?
                                                     rootNodePointer : leaf.splitBox_or_nodePointer;

                                if (IsBoxNode(nodePtr))
                                {
                                    float instanceSA;
                                    if (args.qualityHeuristic == 0)
                                    {
                                        const float childSA = CalculateInstanceChildrenSA(args, nodePtr, leaf, instanceDescOffsetInBytes);
                                        instanceSA = log(max(0, ComputeBoxSurfaceArea(aabb) - childSA) + 1)
                                                        / log(LOG_BASE_N);
                                    }
                                    else
                                    {
                                        instanceSA = log(ComputeBoxSurfaceArea(aabb) + 1) / log(LOG_BASE_N);
                                    }

                                    localSum += instanceSA;
                                }
                            }
                        }
                    }

                    keyIndex++;

                    if (i == (numPrims - 1))
                    {
                        DeviceMemoryBarrier();

                        if (iterationCount == (args.numIterations - 1))
                        {
                            WriteTaskCounterData(
                                ShaderConstants.offsets.encodeTaskCounter,
                                ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET,
                                numPrims + prevSum + threadSumScanned + threadSum);
                        }
                        else
                        {
                            // no race condition as the prefix sum wait makes sure the
                            // numLeafIndices are read already by previous waves
                            ScratchBuffer.Store<float>(readSumValueOffset, 0);

                            ScratchBuffer.Store(numLeafIndicesOffset, numPrims + prevSum + threadSumScanned + threadSum);
                            ScratchBuffer.Store(iterationCountOffset, iterationCount + 1);
                        }
                    }
                }

                // create sum for next iteration
                if (iterationCount != (args.numIterations - 1))
                {
                    const float waveSum = WaveActiveSum(localSum);
                    if (WaveIsFirstLane())
                    {
                        // mutex lock
                        uint orig = 1;

                        do {
                            DeviceMemoryBarrier();
                            ScratchBuffer.InterlockedCompareExchange(mutexOffset, 0, 1, orig);
                        } while (orig);

                        const float globalSum = ScratchBuffer.Load<float>(writeSumValueOffset);

                        ScratchBuffer.Store<float>(writeSumValueOffset, globalSum + waveSum);

                        DeviceMemoryBarrier();

                        ScratchBuffer.Store(mutexOffset, 0); //unlock
                    }
                }

                if (EndTask(localId, taskQueueOffset))
                {
                    if (iterationCount == (args.numIterations - 1))
                    {
                        AllocTasks(numGroups, REBRAID_PHASE_DONE, taskQueueOffset);
                    }
                    else
                    {
                        AllocTasks(numGroups, REBRAID_PHASE_CALC_SUM, taskQueueOffset);
                    }
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

    args.numIterations                      = Settings.numRebraidIterations;
    args.qualityHeuristic                   = Settings.rebraidQualityHeuristic;

    RebraidImpl(globalId, localId, groupId, args);
}
#endif
