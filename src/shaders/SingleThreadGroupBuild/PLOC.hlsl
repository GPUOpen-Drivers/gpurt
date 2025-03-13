/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
void WriteClusterId(uint threadId, uint nodeId)
{
    const uint offset = ShaderConstants.header.offsets.internalNodes + (threadId * sizeof(uint));
    DstBuffer.Store(offset, nodeId);
}

//======================================================================================================================
uint ReadClusterId(uint threadId)
{
    const uint offset = ShaderConstants.header.offsets.internalNodes + (threadId * sizeof(uint));
    return DstBuffer.Load(offset);
}

//======================================================================================================================
// Performs a block exclusive scan/reduce operation on all elements in a thread group.
uint2 BlockExclusiveAddScanReduce(
    uint threadId, uint valid, uint clusterSize)
{
    // Compute the per-lane offset and the wave size for this wave
    uint dstClusterIdx = WavePrefixCountBits(valid);
    const uint waveSize = WaveActiveCountBits(valid);

    // Cache results in group shared memory for prefix sum
    const uint waveId = threadId / WaveGetLaneCount();
    LDS.Write(waveId, 7, waveSize);

    GroupMemoryBarrierWithGroupSync();

    // Compute prefix sum over all active waves
    const uint activeWaveCount = Pow2Align(clusterSize, WaveGetLaneCount()) / WaveGetLaneCount();
    if (waveId == 0)
    {
        const uint waveLaneId = WaveGetLaneIndex();
        const uint waveSize = (waveLaneId < activeWaveCount) ? LDS.Read(waveLaneId, 7) : 0;

        // and store result back into group shared memory
        LDS.Write(waveLaneId, 7, WavePrefixSum(waveSize));

        // Compute cluster size for next iteration;
        LDS.Write(activeWaveCount, 7, WaveActiveSum(waveSize));
    }

    GroupMemoryBarrierWithGroupSync();

    // Broadcast cluster size to all active threads
    clusterSize = LDS.Read(activeWaveCount, 7);

    // Distribute per-thread offsets to individual threads in the group
    dstClusterIdx += LDS.Read(waveId, 7);

    return uint2(dstClusterIdx, clusterSize);
}

//======================================================================================================================
void BuildPLOC(uint threadId, uint numPrimRefs)
{
    // Read sorted primitive index from group shared memory
    uint primRefIndex = 0;
    if (threadId < numPrimRefs)
    {
        primRefIndex = LDS.ReadPrimRefIdx(threadId);
    }

    // Wait for LDS.ReadPrimRefIdx() since we're going to overwrite the groupshared memory with initial cluster list
    GroupMemoryBarrierWithGroupSync();

    if (threadId < numPrimRefs)
    {
        // Cache primitive reference bounds in group shared memory for fast look-up. The maximum cluster size is the
        // primitive count
        const uint primRefNodeIndex = GetBvh2PrimRefNodeIndex(primRefIndex, numPrimRefs);
        const BoundingBox bbox = ReadBvh2NodeBounds(primRefNodeIndex);
        LDS.WriteCachedBounds(threadId, bbox);

        // Initialise cluster list with primitive references (leaf node indices for each primitive)
        WriteClusterId(threadId, MakePrimRefChildPtr(primRefIndex, numPrimRefs));
    }

    uint clusterSize = numPrimRefs;

    const uint radius = Settings.nnSearchRadius;

    // Iteratively merge clusters and create internal nodes until we hit a cluster size of 1
    while (clusterSize > 1)
    {
        // Wait for previous iteration
        GroupMemoryBarrierWithGroupSync();

        uint neighbor = 0xffffffff;

        if (threadId < clusterSize)
        {
            // Fetch cluster bounds at current index in the cluster list
            BoundingBox bbox = LDS.ReadCachedBounds(threadId);

            float minDistance = FLT_MAX;

            // Search left of the current cluster index within the specified radius and cache the index of
            // the closest neighbor in the cluster list
            //
            // 0, 1, 2, 3, [4], 5, 6, 7
            //    <---------+
            //
            const int rangeBegin = max(0, int(threadId - radius));
            const int rangeEnd = min(clusterSize, threadId + radius + 1);

            for (int i = rangeBegin; i < threadId; ++i)
            {
                const BoundingBox candidateBbox = LDS.ReadCachedBounds(i);
                const float distance = ComputeBoxSurfaceArea(CombineAABB(bbox, candidateBbox));

                if ((neighbor == 0xffffffff) || (minDistance > distance))
                {
                    minDistance = distance;
                    neighbor = i;
                }
                else if (minDistance == distance)
                {
                    // Tie breaker when all candidates have the same surface area, pick the immediate neighbor
                    // to the left for odd clusters
                    if ((threadId & 0x1) && (i == (threadId - 1)))
                    {
                        neighbor = i;
                    }
                }
            }

            // Search right of the current cluster index within the specified radius and cache the index of
            // the closest neighbor in the cluster list
            //
            // 0, 1, 2, 3, [4], 5, 6, 7
            //              +------>
            //
            for (int i = threadId + 1; i < rangeEnd; ++i)
            {
                const BoundingBox candidateBbox = LDS.ReadCachedBounds(i);
                const float distance = ComputeBoxSurfaceArea(CombineAABB(bbox, candidateBbox));

                if ((neighbor == 0xffffffff) || (minDistance > distance))
                {
                    minDistance = distance;
                    neighbor = i;
                }
                else if (minDistance == distance)
                {
                    // Tie breaker when all candidates have the same surface area, pick the immediate neighbor
                    // to the right for even clusters
                    if (((threadId & 0x1) == 0) && (i == (threadId + 1)))
                    {
                        neighbor = i;
                    }
                }
            }

            // Update neighbor index for the merge stage
            LDS.WriteNeighborIndex(threadId, neighbor);
        }

        // Wait for all threads to write their closest neighbor index in group shared memory
        GroupMemoryBarrierWithGroupSync();

        bool valid = false;
        bool alloc = false;

        if (neighbor != 0xffffffff)
        {
            // Count lanes with valid clusters for next iteration
            const bool merge = (threadId == LDS.ReadNeighborIndex(neighbor));
            // Use larger of the two merging lanes for allocating internal nodes
            alloc = merge && (threadId == max(threadId, neighbor));
            // Clusters that do not allocate or merge carry over into next iteration
            valid = !merge || !alloc;
        }

        // Perform a block scan and reduction operation on valid clusters
        uint2 result = BlockExclusiveAddScanReduce(threadId, valid, clusterSize);

        const uint dstClusterIdx = result.x;
        clusterSize = result.y;

        const uint waveAllocSize = WaveActiveCountBits(alloc);
        const uint waveAllocIndex = WavePrefixCountBits(alloc);

        // Allocate internal nodes for the current wave
        uint internalNodeOffset = 0;
        if (WaveIsFirstLane())
        {
            ScratchBuffer.InterlockedAdd(
                GetBaseOffsetMiscPlocNodeCounter(), waveAllocSize, internalNodeOffset);
        }

        internalNodeOffset = WaveReadLaneFirst(internalNodeOffset);

        // Merge clusters that agree with each other into an internal node using the larger of the two cluster
        // indices (a.k.a allocating thread)
        if (alloc)
        {
            const uint c0 = ReadClusterId(neighbor);
            const uint c1 = ReadClusterId(threadId);

            // Distribute internal node indices to lanes actively merging
            uint nodeId = ((numPrimRefs - 2) - internalNodeOffset) - waveAllocIndex;

            WriteBvh2ChildPtr(nodeId, c0, false);
            WriteBvh2ChildPtr(nodeId, c1, true);

            const BoundingBox bbox0 = LDS.ReadCachedBounds(neighbor);
            const BoundingBox bbox1 = LDS.ReadCachedBounds(threadId);
            GroupMemoryBarrier();

            const BoundingBox bbox = MergeBvh2Nodes(nodeId, c0, c1, bbox0, bbox1);

            WriteClusterId(neighbor, nodeId);
            LDS.WriteCachedBounds(neighbor, bbox);
        }

        // Wait for cluster updates from allocating threads in group before compaction
        AllMemoryBarrierWithGroupSync();

        uint nodeId = 0;
        BoundingBox bbox;

        if (valid)
        {
            // Read valid thread data into registers
            nodeId = ReadClusterId(threadId);
            bbox = LDS.ReadCachedBounds(threadId);
        }

        // Wait for all threads in group before overwriting data for next iteration
        AllMemoryBarrierWithGroupSync();

        if (valid)
        {
            // Write compacted list data for next iteration
            WriteClusterId(dstClusterIdx, nodeId);
            LDS.WriteCachedBounds(dstClusterIdx, bbox);
        }
    }
}
