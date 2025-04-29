/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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

//=====================================================================================================================
#include "../shadersClean/common/ShaderDefs.hlsli"

#define GC_SCRATCHBUFFER
#include "../shadersClean/build/BuildRootSignature.hlsli"

#include "../shadersClean/common/Common.hlsli"
#include "../shadersClean/build/BuildCommonScratch.hlsli"
#endif

//=====================================================================================================================
// This function indicates a distance metric between the two keys where each internal node splits the hierarchy
// Optionally, we can use the squared distance to compute the distance between two centroids
uint32_t Delta30(
    uint id)
{
    const uint left = id;
    const uint right = id + 1;

    // Special handling of duplicated codes: use their indices as a fallback
    const int leftCode  = ScratchBuffer.Load(ShaderConstants.offsets.mortonCodesSorted + (left * sizeof(int)));
    const int rightCode = ScratchBuffer.Load(ShaderConstants.offsets.mortonCodesSorted + (right * sizeof(int)));

    // returns number of matching bits starting from MSB
    return (leftCode != rightCode) ? clz(leftCode ^ rightCode) : (32 + clz(left ^ right));
}

//=====================================================================================================================
// This function indicates a distance metric between the two keys where each internal node splits the hierarchy
// Optionally, we can use the squared distance to compute the distance between two centroids
uint32_t Delta64(
    uint id)
{
    const uint left = id;
    const uint right = id + 1;

    // Special handling of duplicated codes: use their indices as a fallback
    const uint64_t leftCode  =
        ScratchBuffer.Load<uint64_t>(ShaderConstants.offsets.mortonCodesSorted + (left * sizeof(uint64_t)));
    const uint64_t rightCode =
        ScratchBuffer.Load<uint64_t>(ShaderConstants.offsets.mortonCodesSorted + (right * sizeof(uint64_t)));

    // returns number of matching bits starting from MSB
    return (leftCode != rightCode) ? clz64(leftCode ^ rightCode) : (64 + clz64(left ^ right));
}

//=====================================================================================================================
bool IsSplitRight(
    uint left,
    uint right)
{
    if (Settings.useMortonCode30)
    {
        return (Delta30(right) > Delta30(left - 1));
    }
    else
    {
        return (Delta64(right) > Delta64(left - 1));
    }
}

#define HPLOC_BUILD_THREADGROUP_SIZE 32

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
// Note, the LDS data is organized as a structure of arrays below to minimize bank conflicts.
struct CacheEntry
{
    float boundsMinX[HPLOC_BUILD_THREADGROUP_SIZE];
    float boundsMinY[HPLOC_BUILD_THREADGROUP_SIZE];
    float boundsMinZ[HPLOC_BUILD_THREADGROUP_SIZE];
    float boundsMaxX[HPLOC_BUILD_THREADGROUP_SIZE];
    float boundsMaxY[HPLOC_BUILD_THREADGROUP_SIZE];
    float boundsMaxZ[HPLOC_BUILD_THREADGROUP_SIZE];
    uint  clusterId[HPLOC_BUILD_THREADGROUP_SIZE];
    uint  neighbor[HPLOC_BUILD_THREADGROUP_SIZE];
};

groupshared CacheEntry ThreadGroupCache;
#endif

//======================================================================================================================
void WriteCachedBounds(uint threadId, BoundingBox bbox)
{
    ThreadGroupCache.boundsMinX[threadId] = bbox.min.x;
    ThreadGroupCache.boundsMinY[threadId] = bbox.min.y;
    ThreadGroupCache.boundsMinZ[threadId] = bbox.min.z;
    ThreadGroupCache.boundsMaxX[threadId] = bbox.max.x;
    ThreadGroupCache.boundsMaxY[threadId] = bbox.max.y;
    ThreadGroupCache.boundsMaxZ[threadId] = bbox.max.z;
}

//======================================================================================================================
BoundingBox ReadCachedBounds(uint threadId)
{
    BoundingBox bbox;
    bbox.min.x = ThreadGroupCache.boundsMinX[threadId];
    bbox.min.y = ThreadGroupCache.boundsMinY[threadId];
    bbox.min.z = ThreadGroupCache.boundsMinZ[threadId];
    bbox.max.x = ThreadGroupCache.boundsMaxX[threadId];
    bbox.max.y = ThreadGroupCache.boundsMaxY[threadId];
    bbox.max.z = ThreadGroupCache.boundsMaxZ[threadId];

    return bbox;
}
//======================================================================================================================
void WriteClusterId(uint threadId, uint id)
{
    ThreadGroupCache.clusterId[threadId] = id;
}

//======================================================================================================================
uint ReadClusterId(uint threadId)
{
    return ThreadGroupCache.clusterId[threadId];
}

//======================================================================================================================
void WriteNeighborIndex(uint threadId, uint neighbor)
{
    ThreadGroupCache.neighbor[threadId] = neighbor;
}

//======================================================================================================================
uint ReadNeighborIndex(uint threadId)
{
    return ThreadGroupCache.neighbor[threadId];
}

//======================================================================================================================
void UpdateNeighborIndex(uint threadId, uint newIndex)
{
    InterlockedMin(ThreadGroupCache.neighbor[threadId], newIndex);
}

//=====================================================================================================================
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
    in uint localId,
    in uint baseScratchNodesOffset,
    in uint numInternalNodes,
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
        clusterId = FetchSortedPrimIndex(ShaderConstants.offsets.primIndicesSorted, (rangeStart + waveLaneId));
    }

    numElements = min(numElements, WaveActiveCountBits(clusterId != -1));

    // Load all cluster bounds into group shared memory
    if (waveLaneId < numElements)
    {
        // Cache primitive reference bounds in group shared memory for fast look-up. The maximum cluster size is the
        // primitive count
        const bool isLeafNode = ((clusterId >> 31) == 0);
        const uint baseScratchNodeIdx = isLeafNode ? numInternalNodes : 0;
        const uint scratchNodeIdx = baseScratchNodeIdx + (clusterId & bits(31));

        const ScratchNode scratchNode = FetchScratchNode(baseScratchNodesOffset, scratchNodeIdx);
        BoundingBox bbox = GetScratchNodeBoundingBox(scratchNode,
                                                     isLeafNode,
                                                     Settings.enableEarlyPairCompression,
                                                     ShaderConstants.offsets.bvhLeafNodeData);

        WriteCachedBounds(dstBaseIdx + localId, bbox);
        WriteClusterId(dstBaseIdx + localId, clusterId);
    }

    return numElements;
}

//=====================================================================================================================
// Initialise cluster data in group shared memory and returns the cluster list size
uint InitClusterList(
    in uint localId,
    in uint baseScratchNodesOffset,
    in uint numInternalNodes,
    in uint currentLeftStart,
    in uint currentLeftEnd,
    in uint currentRightStart,
    in uint currentRightEnd,
    in uint halfWaveSize)
{
    const uint numLeft = FetchClusterList(localId,
                                          baseScratchNodesOffset,
                                          numInternalNodes,
                                          0,
                                          currentLeftStart,
                                          currentLeftEnd,
                                          halfWaveSize);

    const uint numRight = FetchClusterList(localId,
                                           baseScratchNodesOffset,
                                           numInternalNodes,
                                           numLeft,
                                           currentRightStart,
                                           currentRightEnd,
                                           halfWaveSize);

    const uint clusterSize = numLeft + numRight;
    return clusterSize;
}

//=====================================================================================================================
void BuildHPLOCImpl(
    const uint globalId,
    const uint localId,
    const uint numActivePrims)
{
    const uint baseScratchNodesOffset = CalculateBvhNodesOffset(ShaderConstants, numActivePrims);

    // Total number of internal nodes is N - 1
    const uint numInternalNodes = numActivePrims - 1;

    // The root of the tree will be stored in the left child of the n-th internal node, where n represents the size of
    // the key array

    // Generate hierarchy recursively. The construction starts from leaf nodes and walks towards the root by finding
    // the parent at each step. We process an internal node only after it has both it's children set. In order to find
    // the parent at each node we have to look at the nodes that split the heirarchy at the left and right ends of the
    // keys covered by the respective node.

    // Leaf nodes cover exactly one range of keys indexed by the primitive index
    uint left  = globalId;
    uint right = globalId;

    bool laneActive = (globalId < numActivePrims);

    const uint waveSize = WaveGetLaneCount();
    const uint halfWaveSize = waveSize / 2;

    while (WaveActiveBallot64(laneActive))
    {
        uint split = -1;

        if (laneActive)
        {
            uint previous = 0;

            const bool useRightParent = ((left == 0) || ((right != numInternalNodes) && IsSplitRight(left, right)));

            const uint parentNodeIndex = useRightParent ? right : left - 1;
            const uint rangeLimit = useRightParent ? left : right;

            const uint flagOffset = ShaderConstants.offsets.propagationFlags + (parentNodeIndex * sizeof(uint));
            ScratchBuffer.InterlockedExchange(flagOffset, rangeLimit, previous);

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
        const bool final = laneActive && (size == numActivePrims);

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

            uint clusterSize = InitClusterList(localId,
                                               baseScratchNodesOffset,
                                               numInternalNodes,
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
                WriteNeighborIndex(clusterIndex, -1);

                BoundingBox bbox = (BoundingBox)0;

                if (clusterIndex < clusterSize)
                {
                    // Fetch cluster bounds at current index in the cluster list
                    bbox = ReadCachedBounds(clusterIndex);

                    uint minAreaIndex = -1;
                    for (uint r = 1; (r <= searchRadius) && ((clusterIndex + r) < clusterSize); ++r)
                    {
                        int neighborIndex = r + clusterIndex;
                        const BoundingBox candidateBbox = ReadCachedBounds(neighborIndex);
                        const float distance = ComputeBoxSurfaceArea(CombineAABB(bbox, candidateBbox));

                        const uint newAreaInt = (asuint(distance) << 1) & encodeMask;
                        const uint encode0 = EncodeRelativeOffset(clusterIndex, clusterIndex + r);
                        const uint newAreaIndex0 = newAreaInt | encode0 | (clusterIndex & 1);
                        const uint newAreaIndex1 = newAreaInt | encode0 | (((clusterIndex + r) & 1) ^ 1);
                        minAreaIndex = min(minAreaIndex, newAreaIndex0);

                        UpdateNeighborIndex(neighborIndex, newAreaIndex1);
                    }

                    UpdateNeighborIndex(clusterIndex, minAreaIndex);
                }

                uint newClusterId = -1;

                if (clusterIndex < clusterSize)
                {
                    const uint neighbor =
                        DecodeRelativeOffset(clusterIndex, ReadNeighborIndex(clusterIndex) & decodeMask, clusterIndex);

                    const bool merge = (clusterIndex == WaveReadLaneAt(neighbor, neighbor));
                    const bool alloc = merge && (clusterIndex < neighbor);

                    const uint waveAllocSize  = WaveActiveCountBits(alloc);
                    const uint waveAllocIndex = WavePrefixCountBits(alloc);

                    // Allocate internal nodes for the current wave
                    uint internalNodeOffset = 0;
                    if (WaveIsFirstLane())
                    {
                        internalNodeOffset = IncrementScratchCounter(ShaderConstants.offsets.plocTaskQueueCounter, waveAllocSize);
                    }

                    // Distribute internal node indices to lanes actively merging
                    internalNodeOffset = WaveReadLaneFirst(internalNodeOffset);
                    const uint mergedNodeIndex = ((numInternalNodes - 1) - internalNodeOffset) - waveAllocIndex;

                    newClusterId = ReadClusterId(clusterIndex);

                    if (merge)
                    {
                        if (alloc)
                        {
                            uint clusterId0 = ReadClusterId(neighbor);
                            uint clusterId1 = ReadClusterId(clusterIndex);

                            const uint offset0 = (clusterId0 >> 31) == 0 ? numInternalNodes : 0;
                            const uint offset1 = (clusterId1 >> 31) == 0 ? numInternalNodes : 0;

                            const uint c0 = offset0 + (clusterId0 & bits(31));
                            const uint c1 = offset1 + (clusterId1 & bits(31));

                            const BoundingBox bboxN = ReadCachedBounds(neighbor);

                            const uint mergedNodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, mergedNodeIndex);
                            const uint leftNodeOffset   = CalcScratchNodeOffset(baseScratchNodesOffset, c0);
                            const uint rightNodeOffset  = CalcScratchNodeOffset(baseScratchNodesOffset, c1);

                            const ScratchNode leftNode = FetchScratchNodeAtOffset(leftNodeOffset);
                            const ScratchNode rightNode = FetchScratchNodeAtOffset(rightNodeOffset);

                            WriteScratchNodeDataAtOffset(mergedNodeOffset, SCRATCH_NODE_LEFT_OFFSET, c0);
                            WriteScratchNodeDataAtOffset(mergedNodeOffset, SCRATCH_NODE_RIGHT_OFFSET, c1);

#if GPURT_BUILD_RTIP3_1
                            // Note, for RTIP3.1 primitive ranges, we reuse the scratch node parent field to store the
                            // pointer to the next leaf node in the primitive range. The parent field is only required for RTIP2.0
                            // pair compression logic and should be removed once that code is updated.
                            if (EnableLatePairCompression())
#endif
                            {
                                WriteScratchNodeDataAtOffset(leftNodeOffset, SCRATCH_NODE_PARENT_OFFSET, mergedNodeIndex);
                                WriteScratchNodeDataAtOffset(rightNodeOffset, SCRATCH_NODE_PARENT_OFFSET, mergedNodeIndex);
                            }

                            MergeScratchNodes(baseScratchNodesOffset,
                                              ShaderConstants.offsets.numBatches,
                                              ShaderConstants.offsets.batchIndices,
                                              numActivePrims,
                                              mergedNodeIndex,
                                              c0,
                                              leftNode,
                                              bboxN,
                                              c1,
                                              rightNode,
                                              bbox,
                                              0);

                            bbox = CombineAABB(bbox, bboxN);

                            newClusterId = mergedNodeIndex | bit(31);
                        }
                        else
                        {
                            newClusterId = -1;
                        }
                    }
                }

                clusterSize = WaveActiveCountBits(newClusterId != -1);
                const uint dstClusterIdx = WavePrefixCountBits(newClusterId != -1);

                if (newClusterId != -1)
                {
                    WriteClusterId(dstClusterIdx, newClusterId);
                    WriteCachedBounds(dstClusterIdx, bbox);
                }

                // Make writes from MergeScratchNodes() visible to all threads
                DeviceMemoryBarrier();
            }

            // The cluster list will be merged by another wave/thread, write cluster list to global memory
            const uint waveLaneId = WaveGetLaneIndex();
            if (waveLaneId < halfWaveSize)
            {
                const uint dstIdx = currentLeftStart + waveLaneId;
                if (dstIdx < numActivePrims)
                {
                    const uint offset = ShaderConstants.offsets.primIndicesSorted + (dstIdx * sizeof(uint));
                    const uint clusterId = (waveLaneId < clusterSize) ? ReadClusterId(waveLaneId) : INVALID_IDX;
                    ScratchBuffer.Store(offset, clusterId);
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

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
// Main Function : BuildHPLOC
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(HPLOC_BUILD_THREADGROUP_SIZE, 1, 1)]
void BuildHPLOC(
    uint localId : SV_GroupThreadID,
    uint globalId : SV_DispatchThreadID)
{
    const uint numActivePrims = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
    uint rootNodeIndex = 0;

    if (numActivePrims <= 1)
    {
        if (globalId == 0)
        {
            if (numActivePrims == 1)
            {
                if (EnableLatePairCompression())
                {
                    // Ensure that a batch index is written out for single-primitive acceleration structures.
                    WriteScratchBatchIndex(ShaderConstants.offsets.numBatches, ShaderConstants.offsets.batchIndices, 0);
                }

                rootNodeIndex = FetchSortedPrimIndex(ShaderConstants.offsets.primIndicesSorted, globalId);
            }

            WriteRootNodeIndex(ShaderConstants.offsets.fastLBVHRootNodeIndex, rootNodeIndex);
        }
    }
    else
    {
        if (globalId == 0)
        {
            WriteRootNodeIndex(ShaderConstants.offsets.fastLBVHRootNodeIndex, rootNodeIndex);
        }
        BuildHPLOCImpl(globalId, localId, numActivePrims);
    }
}
#endif
