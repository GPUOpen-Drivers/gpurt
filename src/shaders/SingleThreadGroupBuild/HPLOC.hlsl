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
uint EncodeRelativeOffset(uint ID, uint neighbor)
{
    const uint uOffset = neighbor - ID - 1;
    return uOffset << 1;
}

//=====================================================================================================================
int DecodeRelativeOffset(const int localID, const uint offset, const uint ID)
{
    const uint off = (offset >> 1) + 1;
    return localID + (((offset ^ ID) % 2 == 0) ? (int)off : -(int)off);
}

//=====================================================================================================================
// Fetches cluster list from global memory into LDS and returns size of list
uint FetchClusterList(
    in uint dstBaseIdx,
    in uint rangeStart,
    in uint rangeEnd,
    in uint halfWaveSize)
{
    const uint waveLaneId = WaveGetLaneIndex();

    uint numElements = min(rangeEnd - rangeStart, halfWaveSize);

    uint clusterId = -1;
    if (waveLaneId < numElements)
    {
        clusterId = ReadClusterId(rangeStart + waveLaneId);
    }

    numElements = min(numElements, WaveActiveCountBits(clusterId != -1));

    // Load all cluster bounds into group shared memory
    if (waveLaneId < numElements)
    {
        // Cache primitive reference bounds in group shared memory for fast look-up. The maximum cluster size is the
        // primitive count
        const BoundingBox bbox = ReadBvh2NodeBounds(GetBvh2NodeIdx(clusterId));

        LDS.WriteCachedBounds(dstBaseIdx + waveLaneId, bbox);
        LDS.WriteClusterId(dstBaseIdx + waveLaneId, clusterId);
    }

    return numElements;
}

//=====================================================================================================================
// Initialise cluster data in group shared memory and returns the cluster list size
uint InitClusterList(
    in uint baseWaveLdsIndex,
    in uint currentLeftStart,
    in uint currentLeftEnd,
    in uint currentRightStart,
    in uint currentRightEnd,
    in uint halfWaveSize)
{
    const uint numLeft = FetchClusterList(baseWaveLdsIndex,
                                          currentLeftStart,
                                          currentLeftEnd,
                                          halfWaveSize);

    const uint numRight = FetchClusterList(baseWaveLdsIndex + numLeft,
                                           currentRightStart,
                                           currentRightEnd,
                                           halfWaveSize);

    const uint clusterSize = numLeft + numRight;
    return clusterSize;
}

//======================================================================================================================
void BuildHPLOC(
    uint threadId, uint numPrimRefs)
{
    // Leaf nodes cover exactly one range of keys indexed by the primitive index

    uint left = threadId;
    uint right = threadId;

    const uint numInternalNodes = (numPrimRefs - 1);

    bool laneActive = (threadId < numPrimRefs);

    if (laneActive)
    {
        // Initialise global cluster list
        const uint primRefIndex = LDS.ReadPrimRefIdx(threadId);
        const uint currNodePtr = MakePrimRefChildPtr(primRefIndex, numPrimRefs);

        WriteClusterId(threadId, currNodePtr);
    }

    const uint waveSize = WaveGetLaneCount();
    const uint halfWaveSize = waveSize / 2;
    const uint baseWaveLdsIndex = WaveReadLaneFirst(threadId);

    while (WaveActiveBallot64(laneActive))
    {
        uint split = -1;

        if (laneActive)
        {
            const bool useRightParent = ((left == 0) || ((right != numInternalNodes) && IsSplitRight(left, right)));

            const uint parentNodeIndex = useRightParent ? right : left - 1;
            const uint rangeLimit = useRightParent ? left : right;

            const uint flagsOffset = ShaderConstants.header.offsets.leafNodes + (parentNodeIndex * sizeof(uint));

            uint previous = 0;
            DstBuffer.InterlockedExchange(flagsOffset, rangeLimit, previous);

            if (previous != 0xffffffff)
            {
                if (useRightParent)
                {
                    split = right + 1;
                    right = previous;
                }
                else
                {
                    split = left;
                    left = previous;
                }
            }
            else
            {
                laneActive = false;
            }
        }

        // PLOC merging phase
        const uint size = right - left + 1;
        const bool final = laneActive && (size == numPrimRefs);

        uint64_t mergeMask = WaveActiveBallot64((laneActive && (size > halfWaveSize)) || final);

        while (mergeMask)
        {
            const uint subtreeLaneId = FIRSTBITLOW_U64(mergeMask);
            mergeMask &= mergeMask - 1;

            const uint currentSplit      = WaveReadLaneAt(split, subtreeLaneId);
            const uint currentLeftStart  = WaveReadLaneAt(left, subtreeLaneId);
            const uint currentLeftEnd    = currentSplit;
            const uint currentRightStart = currentSplit;
            const uint currentRightEnd   = WaveReadLaneAt(right, subtreeLaneId) + 1;
            const bool currentFinal      = WaveReadLaneAt(final, subtreeLaneId);

            uint clusterSize = InitClusterList(baseWaveLdsIndex,
                                               currentLeftStart,
                                               currentLeftEnd,
                                               currentRightStart,
                                               currentRightEnd,
                                               halfWaveSize);

            // Iteratively merge clusters and create internal nodes until we hit a cluster size of threshold
            const uint threshold   = currentFinal ? 1 : halfWaveSize;
            const uint clusterIndex = WaveGetLaneIndex();

            GroupMemoryBarrier();

            while (clusterSize > threshold)
            {
                // (1 << searchRadiusShift) represents the maximum supported nearest neighbor search radius
                // Settings.nnSearchRadius must not exceed this value. The clamping in handled in GpuRt device.
                const uint searchRadiusShift = 4;
                const uint searchRadius = Settings.nnSearchRadius;
                const uint decodeMask = ((1u << (searchRadiusShift + 1u)) - 1u);
                const uint encodeMask = ~decodeMask;

                // initialise neighbor indices in groupshared memory to -1
                LDS.WriteNeighborIndex(threadId, -1);

                BoundingBox bbox = (BoundingBox)0;

                if (clusterIndex < clusterSize)
                {
                    // Fetch cluster bounds at current index in the cluster list
                    bbox = LDS.ReadCachedBounds(threadId);

                    uint minAreaIndex = -1;
                    for (uint r = 1; (r <= searchRadius) && ((clusterIndex + r) < clusterSize); ++r)
                    {
                        int neighborThreadId = r + threadId;

                        const BoundingBox candidateBbox = LDS.ReadCachedBounds(neighborThreadId);
                        const float distance = ComputeBoxSurfaceArea(CombineAABB(bbox, candidateBbox));

                        const uint newAreaInt = (asuint(distance) << 1) & encodeMask;
                        const uint encode0 = EncodeRelativeOffset(clusterIndex, clusterIndex + r);
                        const uint newAreaIndex0 = newAreaInt | encode0 | (clusterIndex & 1);
                        const uint newAreaIndex1 = newAreaInt | encode0 | (((clusterIndex + r) & 1) ^ 1);
                        minAreaIndex = min(minAreaIndex, newAreaIndex0);

                        LDS.UpdateNeighborIndex(neighborThreadId, newAreaIndex1);
                    }

                    LDS.UpdateNeighborIndex(threadId, minAreaIndex);
                }

                uint nodeId = -1;

                if (clusterIndex < clusterSize)
                {
                    const uint neighbor =
                        DecodeRelativeOffset(clusterIndex, LDS.ReadNeighborIndex(threadId) & decodeMask, clusterIndex);

                    const bool merge = (clusterIndex == WaveReadLaneAt(neighbor, neighbor));
                    const bool alloc = merge && (clusterIndex < neighbor);

                    const uint waveAllocSize  = WaveActiveCountBits(alloc);
                    const uint waveAllocIndex = WavePrefixCountBits(alloc);

                    // Allocate internal nodes for the current wave
                    uint internalNodeOffset = 0;
                    if (WaveIsFirstLane())
                    {
                        ScratchBuffer.InterlockedAdd(
                            GetBaseOffsetMiscPlocNodeCounter(), waveAllocSize, internalNodeOffset);
                    }

                    // Distribute internal node indices to lanes actively merging
                    internalNodeOffset = WaveReadLaneFirst(internalNodeOffset);
                    const uint mergedNodeIndex = ((numInternalNodes - 1) - internalNodeOffset) - waveAllocIndex;

                    nodeId = LDS.ReadClusterId(threadId);

                    if (merge)
                    {
                        if (alloc)
                        {
                            uint c0 = LDS.ReadClusterId(baseWaveLdsIndex + neighbor);
                            uint c1 = LDS.ReadClusterId(threadId);

                            WriteBvh2ChildPtr(mergedNodeIndex, c0, false);
                            WriteBvh2ChildPtr(mergedNodeIndex, c1, true);

                            const BoundingBox bbox0 = LDS.ReadCachedBounds(baseWaveLdsIndex + neighbor);
                            const BoundingBox bbox1 = LDS.ReadCachedBounds(threadId);
                            GroupMemoryBarrier();

                            nodeId = mergedNodeIndex;

                            bbox = MergeBvh2Nodes(nodeId, c0, c1, bbox0, bbox1);
                        }
                        else
                        {
                            nodeId = -1;
                        }
                    }
                }

                clusterSize = WaveActiveCountBits(nodeId != -1);
                const uint dstClusterIdx = baseWaveLdsIndex + WavePrefixCountBits(nodeId != -1);

                if (nodeId != -1)
                {
                    LDS.WriteClusterId(dstClusterIdx, nodeId);
                    LDS.WriteCachedBounds(dstClusterIdx, bbox);
                }
            }

            // The cluster list will be merged by another wave/thread, write cluster list to global memory
            const uint waveLaneId = WaveGetLaneIndex();
            if (waveLaneId < halfWaveSize)
            {
                const uint dstIdx = currentLeftStart + waveLaneId;
                if (dstIdx < numPrimRefs)
                {
                    const uint clusterId =
                        (waveLaneId < clusterSize) ? LDS.ReadClusterId(baseWaveLdsIndex + waveLaneId) : INVALID_IDX;

                    WriteClusterId(dstIdx, clusterId);
                }
            }

            // Ensure all global memory writes are visible to all threads
            DeviceMemoryBarrier();
        }

        if (final)
        {
            laneActive = false;
        }
    }
}
