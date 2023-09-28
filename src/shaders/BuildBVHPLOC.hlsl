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
// For parts based on https://github.com/meistdan/ploc/blob/master/gpu-ray-traversal/src/rt/ploc/:
// Copyright (c) 2017, Czech Technical University in Prague
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met :
// *Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and / or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED.IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

struct BuildPlocArgs
{
    uint  numThreads;
    uint  scratchNodesScratchOffset;
    uint  clusterList0ScratchOffset;
    uint  clusterList1ScratchOffset;
    uint  neighbourIndicesScratchOffset;
    uint  currentStateScratchOffset;
    uint  taskQueueCounterScratchOffset;
    uint  atomicFlagsScratchOffset;
    uint  offsetsScratchOffset;
    uint  dynamicBlockIndexScratchOffset;
    uint  numBatchesScratchOffset;
    uint  baseBatchIndicesScratchOffset;
    uint  fp16BoxNodesInBlasMode;                   // Mode used for which BLAS interior nodes are FP16
    float fp16BoxModeMixedSaThresh;                 // For fp16 mode "mixed", surface area threshold
    uint  splitBoxesByteOffset;
    uint  plocRadius;
    uint  primIndicesSortedScratchOffset;
    uint  numLeafNodes;
    uint  unsortedBvhLeafNodesOffset;
    uint reserved0;
    uint reserved1;
};

#define PLOC_RADIUS_MAX         10
#define PLOC_KEYS_PER_THREAD    32
#define PLOC_KEYS_PER_GROUP     (BUILD_THREADGROUP_SIZE * PLOC_KEYS_PER_THREAD)
#define LDS_AABB_STRIDE         6

#define VALID_NEIGHBOURS         0
#define VALID_CLUSTER_LIST       1
#define VALID_CLUSTER_COUNT      2
#define VALID_PREFIX_SUM         3
#define NUM_VALID_TYPES          4

#define INIT_CLUSTER_LIST_LDS    0xfffffffe

#if NO_SHADER_ENTRYPOINT == 0
#include "Common.hlsl"

#define RootSig "RootConstants(num32BitConstants=1, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "CBV(b1),"\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b255),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL)"

struct RootConstants
{
    uint numThreads;
};
[[vk::push_constant]] ConstantBuffer<RootConstants> ShaderRootConstants    : register(b0);
[[vk::binding(1, 1)]] ConstantBuffer<BuildShaderConstants> ShaderConstants : register(b1);

[[vk::binding(0, 0)]] RWByteAddressBuffer                  DstBuffer     : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer DstMetadata   : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer ScratchBuffer : register(u2);

// unused buffer
[[vk::binding(3, 0)]] RWByteAddressBuffer                  SrcBuffer     : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer                  EmitBuffer    : register(u4);

#include "BuildCommonScratch.hlsl"

groupshared int SharedMem[(2 * PLOC_RADIUS_MAX + BUILD_THREADGROUP_SIZE) * LDS_AABB_STRIDE];

#include "TaskQueueCounter.hlsl"

#include "RadixSort/ScanExclusiveInt4DLBCommon.hlsl"

// https://dcgi.fel.cvut.cz/home/meistdan/dissertation/publications/ploc/paper.pdf
// https://github.com/meistdan/ploc/blob/master/gpu-ray-traversal/src/rt/ploc/PLOCBuilderKernels.cu

//=====================================================================================================================
static uint CalculateBvhNodesOffset(
    uint numActivePrims)
{
    return GetBvhNodesOffset(
               numActivePrims,
               ShaderConstants.numLeafNodes,
               ShaderConstants.offsets.bvhNodeData,
               ShaderConstants.offsets.bvhLeafNodeData,
               ShaderConstants.offsets.primIndicesSorted);
}
#endif

//=====================================================================================================================
uint ReadClusterList(
    BuildPlocArgs args,
    uint          listIndex,
    uint          index)
{
    const uint baseOffset = (listIndex == 0) ? args.clusterList0ScratchOffset : args.clusterList1ScratchOffset;

    return ScratchBuffer.Load(baseOffset + (index * sizeof(uint)));
}

//=====================================================================================================================
void WriteClusterList(
    BuildPlocArgs args,
    uint          listIndex,
    uint          index,
    uint          value)
{
    const uint baseOffset = (listIndex == 0) ? args.clusterList0ScratchOffset : args.clusterList1ScratchOffset;

    ScratchBuffer.Store(baseOffset + (index * sizeof(uint)), value);
}

//=====================================================================================================================
void WriteFlags(
    BuildPlocArgs args,
    uint          index,
    uint          type,
    Flags     flags)
{
    const uint idx = (index * NUM_VALID_TYPES) + type;
    const uint offset = idx * sizeof(Flags);

    ScratchBuffer.Store(args.atomicFlagsScratchOffset + offset + FLAGS_PREFIX_SUM_OFFSET, flags.prefixSum);

    DeviceMemoryBarrier();

    ScratchBuffer.Store(args.atomicFlagsScratchOffset + offset + FLAGS_DATA_VALID_OFFSET, flags.dataValid);
}

//=====================================================================================================================
uint ReadPrefixSum(
    BuildPlocArgs args,
    uint          index,
    uint          type)
{
    const uint idx = (index * NUM_VALID_TYPES) + type;
    const uint offset = idx * sizeof(Flags);

    return ScratchBuffer.Load(args.atomicFlagsScratchOffset + offset + FLAGS_PREFIX_SUM_OFFSET);
}

//=====================================================================================================================
uint ReadValid(
    BuildPlocArgs args,
    uint          index,
    uint          type)
{
    const uint idx = (index * NUM_VALID_TYPES) + type;
    const uint offset = idx * sizeof(Flags);

    return ScratchBuffer.Load(args.atomicFlagsScratchOffset + offset + FLAGS_DATA_VALID_OFFSET);
}

//=====================================================================================================================
uint InterlockedCmpxDataValid(
    BuildPlocArgs args,
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
uint ReadNeighbourIndex(
    BuildPlocArgs args,
    uint          index)
{
    return ScratchBuffer.Load(args.neighbourIndicesScratchOffset + (index * sizeof(uint)));
}

//=====================================================================================================================
void WriteNeighbourIndex(
    BuildPlocArgs args,
    uint          index,
    uint          neighbour)
{
    ScratchBuffer.Store(args.neighbourIndicesScratchOffset + (index * sizeof(uint)), neighbour);
}

//=====================================================================================================================
void StoreLdsBoundingBox(
    uint        index,
    BoundingBox bbox)
{
    uint offset = index * LDS_AABB_STRIDE;

    for (uint i = 0; i < 3; i++)
    {
        SharedMem[offset++] = asint(bbox.min[i]);
    }

    for (uint i = 0; i < 3; i++)
    {
        SharedMem[offset++] = asint(bbox.max[i]);
    }
}

//=====================================================================================================================
BoundingBox LoadLdsBoundingBox(
    uint index)
{
    uint offset = index * LDS_AABB_STRIDE;
    BoundingBox bbox;

    for (uint i = 0; i < 3; i++)
    {
        bbox.min[i] = asfloat(SharedMem[offset++]);
    }

    for (uint i = 0; i < 3; i++)
    {
        bbox.max[i] = asfloat(SharedMem[offset++]);
    }

    return bbox;
}

//=====================================================================================================================
void StoreLdsNeighbour(
    uint index,
    uint neighbour)
{
    SharedMem[index] = neighbour;
}

//=====================================================================================================================
uint LoadLdsNeighbour(
    uint index)
{
    return SharedMem[index];
}

//=====================================================================================================================
void StoreLdsClusterIndex(
    uint index,
    uint clusterIndex)
{
    SharedMem[BUILD_THREADGROUP_SIZE + index] = clusterIndex;
}

//=====================================================================================================================
uint LoadLdsClusterIndex(
    uint index)
{
    return SharedMem[BUILD_THREADGROUP_SIZE + index];
}

//=====================================================================================================================
void InitPLOC(
    uint            globalId,
    uint            numActivePrims,
    BuildPlocArgs   args)
{
    const uint numClustersOffset        = args.currentStateScratchOffset + STATE_PLOC_NUM_CLUSTERS_OFFSET;
    const uint internalNodesIndexOffset = args.currentStateScratchOffset + STATE_PLOC_INTERNAL_NODES_INDEX_OFFSET;
    const uint clusterListIndexOffset   = args.currentStateScratchOffset + STATE_PLOC_CLUSTER_LIST_INDEX_OFFSET;
    const uint numClustersAllocOffset   = args.currentStateScratchOffset + STATE_PLOC_NUM_CLUSTERS_ALLOC_OFFSET;
    const uint numInternalNodes         = numActivePrims - 1;

    if (globalId == 0)
    {
        ScratchBuffer.Store(numClustersOffset, numActivePrims);
        ScratchBuffer.Store(internalNodesIndexOffset, numActivePrims - 2);
        ScratchBuffer.Store(clusterListIndexOffset, 0);
        ScratchBuffer.Store(numClustersAllocOffset, 0);

        DeviceMemoryBarrier();
    }

    for (uint i = globalId; i < numActivePrims; i += args.numThreads)
    {
        const uint primIndex = Settings.noCopySortedNodes ?
            FetchSortedPrimIndex(args.primIndicesSortedScratchOffset, i) : i;
        WriteClusterList(args, 0, i, numInternalNodes + primIndex);
    }

    const uint numStages = RoundUpQuotient(numActivePrims + args.plocRadius, BUILD_THREADGROUP_SIZE);

    for (uint stage = globalId; stage < numStages; stage += args.numThreads)
    {
        for (uint type = 0; type < NUM_VALID_TYPES; type++)
        {
            Flags flags;
            flags.dataValid = 0;
            flags.prefixSum = 0;

            WriteFlags(args, stage, type, flags);
        }
    }
}

//=====================================================================================================================
void FindNearestNeighbour(
    uint            globalId,
    uint            localId,
    uint            numActivePrims,
    BuildPlocArgs   args)
{
    const uint numClustersOffset        = args.currentStateScratchOffset + STATE_PLOC_NUM_CLUSTERS_OFFSET;
    const uint clusterListIndexOffset   = args.currentStateScratchOffset + STATE_PLOC_CLUSTER_LIST_INDEX_OFFSET;
    const uint internalNodesIndexOffset = args.currentStateScratchOffset + STATE_PLOC_INTERNAL_NODES_INDEX_OFFSET;
    const uint numClustersAllocOffset   = args.currentStateScratchOffset + STATE_PLOC_NUM_CLUSTERS_ALLOC_OFFSET;

    const int  clusterIndex = globalId;
    const int  stageIndex   = (clusterIndex / BUILD_THREADGROUP_SIZE);
    const int  stageOffset  = stageIndex * BUILD_THREADGROUP_SIZE;
    const uint numClusters  = ScratchBuffer.Load(numClustersOffset);
    uint clusterListIndex   = ScratchBuffer.Load(clusterListIndexOffset);
    const bool isCluster    = clusterIndex < numClusters; // the padding threads are not real clusters

    // load LDS
    int plocRadius = (int)args.plocRadius;
    for (int t = int(localId) - plocRadius; t < (BUILD_THREADGROUP_SIZE + plocRadius); t += BUILD_THREADGROUP_SIZE)
    {
        const int clusterIndex2 = stageOffset + t;

        if ((clusterIndex2 >= 0) && (clusterIndex2 < numClusters))
        {
            const uint nodeIndex = ReadClusterList(args, clusterListIndex, clusterIndex2);
            const ScratchNode scratchNode = FetchScratchNode(args.scratchNodesScratchOffset, nodeIndex);

            // Would need to fetch paired triangle BBox here, if "enableEarlyPairCompression" is ON
            BoundingBox aabb = FetchScratchNodeBoundingBox(scratchNode,
                                                           IsLeafNode(nodeIndex, numActivePrims),
                                                           Settings.doTriangleSplitting,
                                                           args.splitBoxesByteOffset,
                                                           Settings.enableEarlyPairCompression,
                                                           args.unsortedBvhLeafNodesOffset);

            StoreLdsBoundingBox(t + plocRadius, aabb);
        }
    }

    const int minLdsIndex = plocRadius - min(stageOffset, plocRadius);
    const int maxLdsIndex = plocRadius + min(stageOffset + BUILD_THREADGROUP_SIZE + plocRadius, numClusters) - stageOffset;

    GroupMemoryBarrierWithGroupSync();

    uint minIndex = 0xFFFFFFFF;
    float minDistance = FLT_MAX;

    BoundingBox aabb;
    BoundingBox bestNeighbourAabb;

    uint nodeIndex;

    if (isCluster)
    {
        const uint clusterIndex2 = localId + args.plocRadius; // index in LDS

        aabb = LoadLdsBoundingBox(clusterIndex2);

        nodeIndex = ReadClusterList(args, clusterListIndex, clusterIndex);

        // search left
        for (int neighbourIndex = clusterIndex2 - plocRadius; neighbourIndex < clusterIndex2; ++neighbourIndex)
        {
            const BoundingBox neighbourAabb = LoadLdsBoundingBox(neighbourIndex);

            if (neighbourIndex >= minLdsIndex)
            {
                const float distance = ComputeBoxSurfaceArea(CombineAABB(aabb, neighbourAabb));

                if (minIndex == 0xFFFFFFFF || minDistance > distance)
                {
                    minDistance = distance;
                    minIndex = neighbourIndex;
                    bestNeighbourAabb = neighbourAabb;
                }
                else if (minDistance == distance)
                {
                    if ((clusterIndex & 0x1) && (neighbourIndex == (clusterIndex2 - 1)))
                    {
                        minIndex = neighbourIndex;
                        bestNeighbourAabb = neighbourAabb;
                    }
                }
            }
        }

        // search right
        for (int neighbourIndex = clusterIndex2 + 1; neighbourIndex < clusterIndex2 + plocRadius + 1; ++neighbourIndex)
        {
            const BoundingBox neighbourAabb = LoadLdsBoundingBox(neighbourIndex);

            if (neighbourIndex < maxLdsIndex)
            {
                const float distance = ComputeBoxSurfaceArea(CombineAABB(aabb, neighbourAabb));

                if (minIndex == 0xFFFFFFFF || minDistance > distance)
                {
                    minDistance = distance;
                    minIndex = neighbourIndex;
                    bestNeighbourAabb = neighbourAabb;
                }
                else if (minDistance == distance)
                {
                    if (((clusterIndex & 0x1) == 0) && (neighbourIndex == (clusterIndex2 + 1)))
                    {
                        minIndex = neighbourIndex;
                        bestNeighbourAabb = neighbourAabb;
                    }
                }
            }
        }
    }

    GroupMemoryBarrierWithGroupSync();

    if (isCluster)
    {
        const uint clusterIndex2 = localId + args.plocRadius; // index in LDS
        const uint neighbour = min(numClusters - 1, stageOffset + (minIndex - args.plocRadius));

        StoreLdsNeighbour(localId, neighbour);

        // save proposal globally only if it's within the radius of the right side
        if (clusterIndex2 >= (args.plocRadius + BUILD_THREADGROUP_SIZE - args.plocRadius))
        {
            WriteNeighbourIndex(args, clusterIndex, neighbour);
        }
    }

    GroupMemoryBarrierWithGroupSync();

    // notify right side that neighbours is done
    if (localId == 0)
    {
        Flags flags;
        flags.prefixSum = 0;
        flags.dataValid = 1;

        WriteFlags(args, stageIndex, VALID_NEIGHBOURS, flags);
    }

    GroupMemoryBarrierWithGroupSync();

    // fill in with INIT_CLUSTER_LIST_LDS
    // ClusterIndices represents the cluster list of the "left radius + (block - right side's radius)"
    // localId = 0 is ClusterIndices[0] and so forth
    StoreLdsClusterIndex(localId, INIT_CLUSTER_LIST_LDS);

    GroupMemoryBarrierWithGroupSync();

    // merge
    if (isCluster)
    {
        const uint clusterIndex2 = localId + args.plocRadius; // index in LDS

        const uint neighbour = min(numClusters - 1, stageOffset + (minIndex - args.plocRadius));

        bool merging = false;
        bool leftOfMerge = false;

        //if it's hitting the right radius then skip as the wave to the right will handle the merge
        if (minIndex < (args.plocRadius + BUILD_THREADGROUP_SIZE))
        {
            uint neighboursBest;

            //only a local load
            if (minIndex >= args.plocRadius)
            {
                neighboursBest = LoadLdsNeighbour(minIndex - args.plocRadius);
            }
            else //requires global load from left side
            {
                if (stageIndex > 0)
                {
                    do
                    {
                        DeviceMemoryBarrier();
                    } while (ReadValid(args, stageIndex - 1, VALID_NEIGHBOURS) == 0 );
                }

                neighboursBest = ReadNeighbourIndex(args, neighbour);
            }

            if (neighboursBest == clusterIndex)
            {
                if (clusterIndex > neighbour)
                {
                    merging = true;
                }
                else
                {
                    leftOfMerge = true;
                }
            }
        }

        // alloc internal nodes
        int waveCount = -WaveActiveCountBits(merging);
        uint waveIndex = WavePrefixCountBits(merging);

        uint internalNodeOffset = 0;

        if (WaveIsFirstLane())
        {
            ScratchBuffer.InterlockedAdd(internalNodesIndexOffset, waveCount, internalNodeOffset);
        }

        internalNodeOffset = WaveReadLaneFirst(internalNodeOffset);

        if (merging)
        {
            const int mergedNodeIndex = internalNodeOffset - waveIndex;

            const uint leftNodeIndex = ReadClusterList(args, clusterListIndex, neighbour);
            const uint rightNodeIndex = nodeIndex;

            BoundingBox aabbLeft = bestNeighbourAabb;
            BoundingBox aabbRight = aabb;

            BoundingBox mergedBox = CombineAABB(aabbLeft, aabbRight);
            const float mergedBoxSurfaceArea = ComputeBoxSurfaceArea(mergedBox);

            const uint mergedNodeOffset = CalcScratchNodeOffset(args.scratchNodesScratchOffset, mergedNodeIndex);
            const uint leftNodeOffset   = CalcScratchNodeOffset(args.scratchNodesScratchOffset, leftNodeIndex);
            const uint rightNodeOffset  = CalcScratchNodeOffset(args.scratchNodesScratchOffset, rightNodeIndex);

            // Decide on what type of interior box node the parent should be
            // and write the type into scratch
            const ScratchNode leftNode  = ScratchBuffer.Load<ScratchNode>(leftNodeOffset);
            const ScratchNode rightNode = ScratchBuffer.Load<ScratchNode>(rightNodeOffset);

            ScratchBuffer.Store(mergedNodeOffset + SCRATCH_NODE_BBOX_MIN_OFFSET, mergedBox.min);
            ScratchBuffer.Store(mergedNodeOffset + SCRATCH_NODE_BBOX_MAX_OFFSET, mergedBox.max);
            ScratchBuffer.Store(mergedNodeOffset + SCRATCH_NODE_LEFT_OFFSET, leftNodeIndex);
            ScratchBuffer.Store(mergedNodeOffset + SCRATCH_NODE_RIGHT_OFFSET, rightNodeIndex);

            MergeScratchNodes(
                args.scratchNodesScratchOffset,
                args.numBatchesScratchOffset,
                args.baseBatchIndicesScratchOffset,
                numActivePrims,
                mergedNodeIndex,
                leftNodeIndex,
                leftNode,
                rightNodeIndex,
                rightNode,
                mergedBoxSurfaceArea);

            ScratchBuffer.Store(leftNodeOffset + SCRATCH_NODE_PARENT_OFFSET, mergedNodeIndex);
            ScratchBuffer.Store(rightNodeOffset + SCRATCH_NODE_PARENT_OFFSET, mergedNodeIndex);

            // local write if needed
            // write locally for indices that include the left radius to (thread group size - radius)
            // else write to global memory
            if (clusterIndex2 < (args.plocRadius + BUILD_THREADGROUP_SIZE - args.plocRadius))
            {
                StoreLdsClusterIndex(clusterIndex2, INVALID_IDX);
            }
            else
            {
                //write to global memory
                WriteClusterList(args, !clusterListIndex, clusterIndex, INVALID_IDX);
            }

            // local write if needed
            // write locally for indices that include the left radius to (thread group size - radius)
            // else write to global memory
            if (minIndex < (args.plocRadius + BUILD_THREADGROUP_SIZE - args.plocRadius))
            {
                StoreLdsClusterIndex(minIndex, mergedNodeIndex);
            }
            else
            {
                //write to global memory
                WriteClusterList(args, !clusterListIndex, neighbour, mergedNodeIndex);
            }
        }
        else if (leftOfMerge == false) // if not part of a merge (known for now... could be merged with right side)
        {
            // local write if needed
            if (clusterIndex2 < (args.plocRadius + BUILD_THREADGROUP_SIZE - args.plocRadius))
            {
                StoreLdsClusterIndex(clusterIndex2, nodeIndex);
            }
            else
            {
                //write to global memory
                WriteClusterList(args, !clusterListIndex, clusterIndex, nodeIndex);
            }
        }
    }

    GroupMemoryBarrierWithGroupSync();

    // notify the right side that we wrote out the cluster list
    if (localId == 0)
    {
        Flags flags;
        flags.prefixSum = 0;
        flags.dataValid = 1;

        WriteFlags(args, stageIndex, VALID_CLUSTER_LIST, flags);
    }

    // wait for left side to finish its cluster list write
    if (stageIndex > 0)
    {
        // fill in the left radius to the LDS ClusterIndices
        // if it's still INIT_CLUSTER_LIST_LDS after the merge, then read from the left side
        // it wouldn't be INIT_CLUSTER_LIST_LDS if it was merged from this stage
        if ((localId < args.plocRadius) && (LoadLdsClusterIndex(localId) == INIT_CLUSTER_LIST_LDS))
        {
            do
            {
                DeviceMemoryBarrier();
            } while (ReadValid(args, stageIndex - 1, VALID_CLUSTER_LIST) == 0);

            StoreLdsClusterIndex(localId, ReadClusterList(args, !clusterListIndex, stageOffset - args.plocRadius + localId));
        }
    }

    GroupMemoryBarrierWithGroupSync();

    // calculate prefix sum
    const uint compactIndex = LoadLdsClusterIndex(localId);
    const uint value = (compactIndex != INVALID_IDX) && (compactIndex != INIT_CLUSTER_LIST_LDS) ? 1 : 0;
    const int threadSumScanned = BlockScanExclusiveAdd(value, localId);

    // prefix sum including the radius of the left side and excluding the (end - radius) of the right side
    uint prevSum = 0;

    // wait for prefix sum
    if (stageIndex > 0)
    {
        if (localId == (BUILD_THREADGROUP_SIZE - 1))
        {
            Flags flags;
            flags.prefixSum = threadSumScanned + value;
            flags.dataValid = 1;

            WriteFlags(args, stageIndex, VALID_CLUSTER_COUNT, flags);

            int readBackIndex = stageIndex - 1;

            while (readBackIndex >= 0)
            {
                if (ReadValid(args, readBackIndex, VALID_PREFIX_SUM) != 0)
                {
                    prevSum += ReadPrefixSum(args, readBackIndex, VALID_PREFIX_SUM);

                    Flags flags;
                    flags.prefixSum = prevSum + threadSumScanned + value;
                    flags.dataValid = 1;

                    WriteFlags(args, stageIndex, VALID_PREFIX_SUM, flags);
                    break;
                }
                else if (ReadValid(args, readBackIndex, VALID_CLUSTER_COUNT) != 0)
                {
                    prevSum += ReadPrefixSum(args, readBackIndex, VALID_CLUSTER_COUNT);
                    readBackIndex--;
                }

                DeviceMemoryBarrier();
            }

            SharedMem[0] = prevSum;
        }

        GroupMemoryBarrierWithGroupSync();

        prevSum = SharedMem[0];
    }  // write out for the right side
    else if (localId == (BUILD_THREADGROUP_SIZE - 1))
    {
        Flags flags;
        flags.prefixSum = threadSumScanned + value;
        flags.dataValid = 1;

        WriteFlags(args, stageIndex, VALID_PREFIX_SUM, flags);
    }

    // compact clusters to global memory
    if ((compactIndex != INVALID_IDX) && (compactIndex != INIT_CLUSTER_LIST_LDS))
    {
        const uint offset = prevSum + threadSumScanned;

        WriteClusterList(args, !clusterListIndex, offset, compactIndex);
    }

    const int shiftedClusterIndex = clusterIndex - plocRadius;

    // update the new cluster count
    if (shiftedClusterIndex == (numClusters - 1))
    {
        const uint lastCluster = (compactIndex != INVALID_IDX) && (compactIndex != INIT_CLUSTER_LIST_LDS) ? 1 : 0;

        ScratchBuffer.Store(numClustersAllocOffset, prevSum + threadSumScanned + lastCluster);
    }
}

//=====================================================================================================================
void UpdateClusterCount(
    uint            globalId,
    BuildPlocArgs   args)
{
    const uint numClustersOffset        = args.currentStateScratchOffset + STATE_PLOC_NUM_CLUSTERS_OFFSET;
    const uint clusterListIndexOffset   = args.currentStateScratchOffset + STATE_PLOC_CLUSTER_LIST_INDEX_OFFSET;
    const uint numClustersAllocOffset   = args.currentStateScratchOffset + STATE_PLOC_NUM_CLUSTERS_ALLOC_OFFSET;

    const uint clusterListIndex   = ScratchBuffer.Load(clusterListIndexOffset);
    const uint numClusters        = ScratchBuffer.Load(numClustersAllocOffset);
    const uint numStages          = RoundUpQuotient(numClusters + args.plocRadius, BUILD_THREADGROUP_SIZE);

    for (uint s = globalId; s < numStages; s += BUILD_THREADGROUP_SIZE)
    {
        // unroll improved perf from BLAS time 235->228 ms for ArchViz
        [unroll]
        for (uint type = 0; type < NUM_VALID_TYPES; type++)
        {
            Flags flags;
            flags.dataValid = 0;
            flags.prefixSum = 0;

            WriteFlags(args, s, type, flags);
        }
    }
}

//=====================================================================================================================
void BuildBvhPlocImpl(
    uint            globalId,
    uint            localId,
    uint            groupId,
    uint            numActivePrims,
    BuildPlocArgs   args)
{
    const uint taskQueueOffset          = args.taskQueueCounterScratchOffset;
    const uint numClustersOffset        = args.currentStateScratchOffset + STATE_PLOC_NUM_CLUSTERS_OFFSET;
    const uint internalNodesIndexOffset = args.currentStateScratchOffset + STATE_PLOC_INTERNAL_NODES_INDEX_OFFSET;
    const uint clusterListIndexOffset   = args.currentStateScratchOffset + STATE_PLOC_CLUSTER_LIST_INDEX_OFFSET;
    const uint numClustersAllocOffset   = args.currentStateScratchOffset + STATE_PLOC_NUM_CLUSTERS_ALLOC_OFFSET;

    const uint numInternalNodes = numActivePrims - 1;

    uint clusterListIndex = 0;
    uint numClusters      = numActivePrims;

    const uint numGroups = args.numThreads / BUILD_THREADGROUP_SIZE;

    if (numActivePrims <= 1)
    {
        if ((EnableLatePairCompression()) && (globalId == 0) && (numActivePrims == 1))
        {
            // Ensure that a batch index is written out for single-primitive acceleration structures.
            WriteScratchBatchIndex(args.numBatchesScratchOffset, args.baseBatchIndicesScratchOffset, 0);
        }

        return;
    }

    if (globalId == 0)
    {
        AllocTasks(numGroups, PLOC_PHASE_INIT, taskQueueOffset);
    }

    while (1)
    {
        const uint2 task = BeginTask(localId, taskQueueOffset);

        const uint taskIndex = task.x;
        const uint phase = task.y;

        globalId = taskIndex * BUILD_THREADGROUP_SIZE + localId;
        clusterListIndex = ScratchBuffer.Load(clusterListIndexOffset);
        numClusters = ScratchBuffer.Load(numClustersOffset);

        switch (phase)
        {
            case PLOC_PHASE_INIT:
            {
                InitPLOC(globalId, numActivePrims, args);

                if (EndTask(localId, taskQueueOffset))
                {
                    // + PLOC_RADIUS is needed to finish off the last stage
                    const uint numStages = RoundUpQuotient(numActivePrims + args.plocRadius, BUILD_THREADGROUP_SIZE);

                    AllocTasks(numStages, PLOC_PHASE_FIND_NEAREST_NEIGHBOUR, taskQueueOffset);
                }
                break;
            }
            // TODO: if wave doesnt find work then exit
            case PLOC_PHASE_FIND_NEAREST_NEIGHBOUR:
            {
                FindNearestNeighbour(globalId, localId, numActivePrims, args);

                if (EndTask(localId, taskQueueOffset))
                {
                    const uint numClusters = ScratchBuffer.Load(numClustersAllocOffset);

                    if (numClusters > 1)
                    {
                        AllocTasks(numGroups, PLOC_PHASE_UPDATE_CLUSTER_COUNT, taskQueueOffset);
                    }
                    else
                    {
                        AllocTasks(numGroups, PLOC_PHASE_DONE, taskQueueOffset);
                    }
                }
            }
            break;

            case PLOC_PHASE_UPDATE_CLUSTER_COUNT:
            {
                UpdateClusterCount(globalId, args);
                if (EndTask(localId, taskQueueOffset))
                {
                    const uint numClustersAlloc = ScratchBuffer.Load(numClustersAllocOffset);
                    const uint numStages   = RoundUpQuotient(numClustersAlloc + args.plocRadius, BUILD_THREADGROUP_SIZE);

                    ScratchBuffer.Store(numClustersOffset, numClustersAlloc);
                    ScratchBuffer.Store(numClustersAllocOffset, 0);

                    ScratchBuffer.Store(clusterListIndexOffset, !clusterListIndex);
                    AllocTasks(numStages, PLOC_PHASE_FIND_NEAREST_NEIGHBOUR, taskQueueOffset);
                }
            }
                break;

            case PLOC_PHASE_DONE:
                return;
        }
    }
}

#if NO_SHADER_ENTRYPOINT == 0
//====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void BuildBVHPLOC(
    uint globalIdIn : SV_DispatchThreadID,
    uint groupIdIn  : SV_GroupID,
    uint localIdIn  : SV_GroupThreadID)
{
    uint globalId = globalIdIn;
    uint localId = localIdIn;
    uint groupId = groupIdIn;

    const uint numActivePrims = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
    BuildPlocArgs plocArgs;

    plocArgs.numThreads                     = ShaderRootConstants.numThreads;
    plocArgs.clusterList0ScratchOffset      = ShaderConstants.offsets.clusterList0;
    plocArgs.clusterList1ScratchOffset      = ShaderConstants.offsets.clusterList1;
    plocArgs.neighbourIndicesScratchOffset  = ShaderConstants.offsets.neighborIndices;
    plocArgs.currentStateScratchOffset      = ShaderConstants.offsets.currentState;
    plocArgs.taskQueueCounterScratchOffset  = ShaderConstants.offsets.plocTaskQueueCounter;
    plocArgs.atomicFlagsScratchOffset       = ShaderConstants.offsets.atomicFlagsPloc;
    plocArgs.offsetsScratchOffset           = ShaderConstants.offsets.clusterOffsets;
    plocArgs.dynamicBlockIndexScratchOffset = ShaderConstants.offsets.dynamicBlockIndex;
    plocArgs.numBatchesScratchOffset        = ShaderConstants.offsets.numBatches;
    plocArgs.baseBatchIndicesScratchOffset  = ShaderConstants.offsets.batchIndices;
    plocArgs.fp16BoxNodesInBlasMode         = Settings.fp16BoxNodesMode;
    plocArgs.fp16BoxModeMixedSaThresh       = Settings.fp16BoxModeMixedSaThreshold;
    plocArgs.plocRadius                     = Settings.plocRadius;
    plocArgs.splitBoxesByteOffset           = ShaderConstants.offsets.triangleSplitBoxes;
    plocArgs.primIndicesSortedScratchOffset = ShaderConstants.offsets.primIndicesSorted;
    plocArgs.numLeafNodes                   = ShaderConstants.numLeafNodes;
    plocArgs.unsortedBvhLeafNodesOffset     = ShaderConstants.offsets.bvhLeafNodeData;

    plocArgs.scratchNodesScratchOffset = CalculateBvhNodesOffset(numActivePrims);

    if (numActivePrims > 0)
    {
        BuildBvhPlocImpl(globalId, localId, groupId, numActivePrims, plocArgs);
    }
}
#endif
