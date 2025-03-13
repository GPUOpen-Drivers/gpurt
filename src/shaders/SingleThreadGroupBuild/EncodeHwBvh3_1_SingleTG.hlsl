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
void WriteStackIndex(
    uint threadId,
    uint2 value)
{
    const uint offset = ShaderConstants.offsets.qbvhGlobalStack + (threadId * sizeof(uint2));
    ScratchBuffer.Store<uint2>(offset, uint2(value));
}

//=====================================================================================================================
uint2 ReadStackIndex(
    uint threadId)
{
    const uint offset = ShaderConstants.offsets.qbvhGlobalStack + (threadId * sizeof(uint2));
    return ScratchBuffer.Load<uint2>(offset);
}

//=====================================================================================================================
static uint GetNodeArrayBaseInLds(
    in uint localId, in uint maxNumChildren)
{
    return (localId * maxNumChildren);
}

//=====================================================================================================================
// Insert candidaate child in node array
static void InsertChildInNodeArray(
    in uint           baseLdsOffset,
    in uint           childNodeIdx,
    inout_param(uint) nextLeafIdx,
    inout_param(uint) nextBoxIdx)
{
    uint childSlotIdx = 0;

    if (IsBvh2InternalNode(childNodeIdx) == false)
    {
        childSlotIdx = --nextLeafIdx;
    }
    else
    {
        childSlotIdx = nextBoxIdx++;
    }

    SharedMem[baseLdsOffset + childSlotIdx] = childNodeIdx;
}

//=====================================================================================================================
// Build child node array from BVH2 treelet root by using surface area heuristics
static void BuildChildNodeArray(
    in    uint      threadId,
    in    uint      subtreeRootNodeIdx,
    in    uint      maxNumChildren,
    out_param(uint) numBoxChildren,
    out_param(uint) numLeafChildren)
{
    const uint baseLdsOffset  = GetNodeArrayBaseInLds(threadId, maxNumChildren);

    uint nextBoxIdx = 0;
    uint nextLeafIdx = maxNumChildren;

    if (IsBvh2InternalNode(subtreeRootNodeIdx) == false)
    {
        // Insert leaf child into node array
        InsertChildInNodeArray(baseLdsOffset, subtreeRootNodeIdx, nextLeafIdx, nextBoxIdx);
    }
    else
    {
        // Fetch bvh2 internal node
        const uint2 bvh2 = ReadBvh2ChildPtrs(subtreeRootNodeIdx);

        // Insert left child into node array
        InsertChildInNodeArray(baseLdsOffset, bvh2.x, nextLeafIdx, nextBoxIdx);

        // Insert right child into node array
        InsertChildInNodeArray(baseLdsOffset, bvh2.y, nextLeafIdx, nextBoxIdx);
    }

    // The following loop places box nodes at the front of the node array while all leaf children are placed
    // starting from the back of the node array with empty nodes in the middle. The two arrays are tracked
    // via two pointers, nextBoxIdx and nextLeafIdx
    //
    // [Box, Box, ..................., Leaf, Leaf]
    //           ^                   ^
    //       nextBoxIdx         nextLeafIdx
    //
    uint validChildCount = nextBoxIdx + (maxNumChildren - nextLeafIdx);

    // Cached candidate node data (left and right child indices)
    uint2 cachedCandidateData = uint2(0, 0);

    while (validChildCount < maxNumChildren)
    {
        float surfaceAreaMax = 0.0;
        uint candidateSlotIdx = INVALID_IDX;

        // Pick the largest surface area node from current node array of internal nodes.
        for (uint j = 0; j < nextBoxIdx; j++)
        {
            const uint candidateNodeIdx = SharedMem[baseLdsOffset + j];
            const BoundingBox bbox = ReadBvh2NodeBounds(candidateNodeIdx);

            const float surfaceArea = ComputeBoxSurfaceArea(bbox);

            // Pick the node with larger surface area. If the nodes have identical surface area, pick the
            // last node for opening.
            if (surfaceArea >= surfaceAreaMax)
            {
                surfaceAreaMax = surfaceArea;
                candidateSlotIdx = j;
                cachedCandidateData = ReadBvh2ChildPtrs(candidateNodeIdx);
            }
        }

        // If there are no valid nodes to open, exit loop
        if (candidateSlotIdx == INVALID_IDX)
        {
            break;
        }

        // Swap in the last box node in candidate slot and free last slot for opening. Below, C represents
        // candidate box node with the largest surface area.
        //
        // [B, C, B..................., L, L]
        //         ^                   ^
        //      nextBoxIdx         nextLeafIdx
        //
        // Last box node in node array gets swapped with candidate node and the box node pointer moves back
        // to allow insertion at the end of the list
        //
        // [B, B, C..................., L, L]
        //      ^                      ^
        //   nextBoxIdx            nextLeafIdx
        //
        nextBoxIdx--;
        SharedMem[baseLdsOffset + candidateSlotIdx] = SharedMem[baseLdsOffset + nextBoxIdx];

        InsertChildInNodeArray(baseLdsOffset, cachedCandidateData.x, nextLeafIdx, nextBoxIdx);

        InsertChildInNodeArray(baseLdsOffset, cachedCandidateData.y, nextLeafIdx, nextBoxIdx);

        // Update valid child count
        validChildCount = nextBoxIdx + (maxNumChildren - nextLeafIdx);
    }

    // Assign box and leaf node count in node array to output variables
    numBoxChildren  = nextBoxIdx;
    numLeafChildren = maxNumChildren - nextLeafIdx;

    // Shift leaf nodes left to cover up empty spaces if any in the child node array
    if (nextBoxIdx != nextLeafIdx)
    {
        for (uint i = 0; i < numLeafChildren; ++i)
        {
            SharedMem[baseLdsOffset + nextBoxIdx + i] = SharedMem[baseLdsOffset + nextLeafIdx + i];
        }
    }
}

//=====================================================================================================================
// Write intersectable instance node as a filter node with 4 distinct child bounds sharing the same root
// node
static void WriteBlasInstanceFilterNode(
    in uint   rootNodeIndex,
    in float3 origin,
    in uint3  exponents,
    in float3 rcpExponents)
{
    // For single primitive case, skip building node array and just push the lone leaf node to
    // the child node array
    uint numBoxChildren  = 0;
    uint numLeafChildren = 0;

    // For filter nodes, we just build 4 bounding boxes from the root node
    const uint baseLdsOffset = GetNodeArrayBaseInLds(0, 4);
    BuildChildNodeArray(0,
                        rootNodeIndex,
                        4,
                        numBoxChildren,
                        numLeafChildren);

    // The root of the BVH2 treelet contains the combined bounds of the child nodes in the node array.
    // Compute the common exponent from the root node bounds
    const uint validChildCount = numBoxChildren + numLeafChildren;

    // Origin
    const uint baseOffset = ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET;
    DstMetadata.Store3(baseOffset, asuint(origin));

    // Packed exponents, valid child count and child index in parent node
    const uint packedExpChildIdxAndCount =
        QuantizedBVH8BoxNode::PackExpChildIdxAndCount(exponents, 1, 0, 4);

    DstMetadata.Store(baseOffset + QUANTIZED_BVH4_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT,
                      packedExpChildIdxAndCount);

    for (uint i = 0; i < validChildCount; ++i)
    {
        const uint nodePtr = SharedMem[baseLdsOffset + i];

        const BoundingBox childBounds = ReadBvh2NodeBounds(GetBvh2NodeIdx(nodePtr));

        const UintBoundingBox quantBounds = ComputeQuantizedBounds(childBounds, origin, rcpExponents, 12);

        const uint3 childInfo = GetInstanceChildInfo(i, CreateRootNodePointer3_1(), quantBounds.min, quantBounds.max);

        DstMetadata.Store3(baseOffset + GetBvh4ChildInfoOffset(i), childInfo);
    }

    // Encode remaining child slots as invalid
    for (uint i = validChildCount; i < 4; ++i)
    {
        const uint3 childInfo = GetInstanceChildInfo(i, CreateRootNodePointer3_1(), 0xFFFFFFFF.xxx, 0.xxx);
        DstMetadata.Store3(baseOffset + GetBvh4ChildInfoOffset(i), childInfo);
    }
}

//=====================================================================================================================
// Pack destination node index (128-byte chunk index) and child index in parent (maximum value of 7) into a single
// DWORD in global stack
uint PackStackDstNodeIdx(
    uint dstNodeIdx,
    uint indexInParent)
{
    return (dstNodeIdx << 3) | indexInParent;
}

//=====================================================================================================================
uint GetStackDstIdx(
    uint packedEntry)
{
    return (packedEntry >> 3);
}

//=====================================================================================================================
uint GetStackDstIdxInParent(
    uint packedEntry)
{
    return (packedEntry & bits(3));
}

//=====================================================================================================================
// Allocation of compressed leaf nodes is deferred to the compression pass. Leaf nodes in the root of a rebraided BLAS
// are an exception. They must be allocated near the beginning of the tree to support rebraid. Some small unused space
// will be left between the end of the compressed data and the next node.
uint GetNumInlinePrimStructs(
    LaneGroup laneGroup,
    uint      laneNodePtr,
    uint      numLeafChildren,
    bool      isRebraidRootNode,
    bool      isLeaf)
{
    uint numInlinePrimStructs = 0;

    if (EnablePrimitiveCompressionPass() && (isRebraidRootNode == false))
    {
        numInlinePrimStructs = 0;
    }
    else if (Settings.maxPrimRangeSize <= 2)
    {
        numInlinePrimStructs = numLeafChildren;
    }
    else
    {
        if (isLeaf)
        {
            const uint rangeNumPrims = GetBvh2PrimitiveCount(laneNodePtr);

            // Minimum two triangles per prim struct in each range
            numInlinePrimStructs = RoundUpQuotient(rangeNumPrims, 2);
        }

        numInlinePrimStructs = laneGroup.Sum(numInlinePrimStructs);
    }

    return numInlinePrimStructs;
}

//======================================================================================================================
void EncodeHwBvh3_1(
    uint threadId,
    uint rootNodeIndex,
    uint numPrimRefs)
{
    const AccelStructOffsets offsets = ShaderConstants.header.offsets;

    const uint maxWorkItems = GetNumInternalNodeCount(numPrimRefs);

    if (threadId == 0)
    {
        EncodeHwBvh::WriteCounter(STGB_COUNTER_IDX_PRIM_REF, 0);
        EncodeHwBvh::WriteCounter(STGB_COUNTER_IDX_BOX_REF, 1);
        EncodeHwBvh::WriteCounter(STGB_COUNTER_IDX_NODE_ALLOC, 1);

        if (EnablePrimitiveCompressionPass())
        {
            // Initialise primitive compression batch counters
            EncodeHwBvh::WriteCounter(STGB_COUNTER_IDX_PRIM_COMP_BATCH, 0);
            EncodeHwBvh::WriteCounter(STGB_COUNTER_IDX_PRIM_COMP_BATCH_SINGLE, 0);
        }

        // Initialise leaf node counter to 0 for BLAS primitive compression
        WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET, 0);

        const uint flags = ReadBvh2NodeFlags(GetBvh2NodeIdx(rootNodeIndex));
        WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_PACKED_FLAGS_OFFSET, flags);

        // Write root node parent pointer
        QBVH8::WriteParentPointer(offsets.internalNodes, INVALID_IDX);

        // Generate intersectable instance node data in BLAS metadata
        const BoundingBox bbox = ReadBvh2NodeBounds(rootNodeIndex);

        if ((Settings.instanceMode == InstanceMode::Passthrough) || (numPrimRefs == 1))
        {
            WriteInstancePassthrough(CreateRootNodePointer(),
                                     bbox,
                                     ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET);
        }
        else if (Settings.instanceMode == InstanceMode::FilterNode)
        {
            // Generate filter node data in BLAS metadata
            const uint3 exponents = ComputeCommonExponent(bbox.min, bbox.max, 12);
            const float3 rcpExponents = ComputeFastExpReciprocal(exponents, 12);
            WriteBlasInstanceFilterNode(rootNodeIndex, bbox.min, exponents, rcpExponents);
        }
    }

    if (threadId < maxWorkItems)
    {
        // Enqueue root node to be constructed at offset 0
        const uint nodeIdx = (threadId == 0) ? rootNodeIndex : -1;
        WriteStackIndex(threadId, uint2(nodeIdx, 0));
    }

    DeviceMemoryBarrierWithGroupSync();

    LaneGroup laneGroup;
    laneGroup.Alloc(8);

    // 1:1 mapping of a 8-wide lane group to global stack index
    uint globalStackIndex = threadId / 8;

    // Rebraid currently only supports 4x. Create the root internal node as BVH4 when rebraid is enabled.
    // Note, this creates a partial BVH8 internal node at the root. We can either use variable-arity BLAS
    // which requires additional ALU during traversal or fill in the empty slots with box splitting
    //
    const bool isRebraidRootNode = Settings.enableInstanceRebraid && (globalStackIndex == 0);
    const uint maxNumChildren = isRebraidRootNode ? 4 : 8;

    while (globalStackIndex < maxWorkItems)
    {
#if USE_LDS_HW_ENCODE_COUNTERS
        AllMemoryBarrier();
#else
        DeviceMemoryBarrier();
#endif

        uint numPrimRefsDone = 0;
        if (WaveIsFirstLane())
        {
            numPrimRefsDone = EncodeHwBvh::ReadCounter(STGB_COUNTER_IDX_PRIM_REF);
        }

        numPrimRefsDone = WaveReadLaneFirst(numPrimRefsDone);

        if (numPrimRefsDone >= numPrimRefs)
        {
            break;
        }

        // Pop stack entry from LDS stack. If this thread has a valid node, the LDS memory will be repurposed for constructing
        // the temporary child node array in BuildChildNodeArray()
        uint2 stackEntry = uint2(0, 0);
        if (laneGroup.IsFirstLane())
        {
            stackEntry = ReadStackIndex(globalStackIndex);
        }

        stackEntry = laneGroup.ReadFirstLane(stackEntry);

        // If we have a valid node on the stack, process node
        if (stackEntry.x != INVALID_IDX)
        {
            const uint bvhNodeSrcIdx = stackEntry.x;
            const uint bvhNodeDstIdx = GetStackDstIdx(stackEntry.y);
            const uint indexInParent = GetStackDstIdxInParent(stackEntry.y);

            const uint currNodeOffset = offsets.internalNodes + (bvhNodeDstIdx * QBVH8::NodeSizeInBytes);

            uint srcNodePtr = INVALID_IDX;
            if (laneGroup.IsFirstLane())
            {
                srcNodePtr = bvhNodeSrcIdx;
            }

            bool loadSrcData = laneGroup.IsFirstLane();

            uint numBoxChildren = 0;
            float surfaceArea = -1.0f;
            uint2 childPtrs = uint2(0, 0);

            // This bitfield is an array of 2 bit indices indicating which of the first 4 boxes was split to produce
            // a given box. It is initialized to [0, 1, 2, 3] for the first 4 boxes. After the first 4 boxes are
            // generated, each additional box takes the index of the box it was split from.
            uint packedSourceBoxes = 0b11100100;

            // BuildChildNodeArray
            while (1)
            {
                const uint firstInvalidLane = laneGroup.GetFirstActiveLaneIndex(srcNodePtr == INVALID_IDX);

                // Load node data if needed
                if (loadSrcData)
                {
                    if (IsBvh2InternalNode(srcNodePtr))
                    {
                        const BoundingBox bbox = ReadBvh2NodeBounds(srcNodePtr);
                        childPtrs = ReadBvh2ChildPtrs(srcNodePtr);
                        surfaceArea = ComputeBoxSurfaceArea(bbox);
                    }
                    else
                    {
                        surfaceArea = -1.0f;
                    }

                    loadSrcData = false;
                }

                // Note, invalid node pointers are not counted as internal nodes by IsBvh2InternalNode()
                numBoxChildren = countbits(laneGroup.Ballot(IsBvh2InternalNode(srcNodePtr)));

                if ((numBoxChildren == 0) || (firstInvalidLane >= maxNumChildren))
                {
                    // Break the loop if there are no more box nodes needing to be processed,
                    // or if there's no remaining space in the structure.
                    break;
                }

                // Get the lane index of the node with the largest surface area
                const float maxSurfaceArea = laneGroup.Max(surfaceArea);
                const uint largestNodeLaneIdx = laneGroup.GetFirstActiveLaneIndex((surfaceArea == maxSurfaceArea));

                uint rightChildPtr = INVALID_IDX;

                if (laneGroup.laneIndex == largestNodeLaneIdx)
                {
                    // Assign left child for next processing
                    srcNodePtr = childPtrs.x;
                    rightChildPtr = childPtrs.y;
                    loadSrcData = true;
                }

                // Distribute the right child to the first thread with an invalid index
                rightChildPtr = laneGroup.Broadcast(rightChildPtr, largestNodeLaneIdx);

                if ((laneGroup.laneIndex == firstInvalidLane) && (rightChildPtr != INVALID_IDX))
                {
                    // This thread takes the right child
                    srcNodePtr = rightChildPtr;
                    loadSrcData = true;

                    if (laneGroup.laneIndex >= 4)
                    {
                        const uint sourceBoxIndex = (packedSourceBoxes >> (largestNodeLaneIdx * 2)) & 0b11;
                        packedSourceBoxes |= sourceBoxIndex << (laneGroup.laneIndex * 2);
                    }
                }
            }

            const bool isBox = IsBvh2InternalNode(srcNodePtr);
            const bool isLeaf = (srcNodePtr != INVALID_IDX) && (isBox == false);
            const uint numLeafChildren = countbits(laneGroup.Ballot(isLeaf));

            // Calculate exclusive prefix sum to get index (0,1,2... )
            uint boxOrLeafChildIndex = 0;
            if (isBox)
            {
                boxOrLeafChildIndex = laneGroup.ExclusivePrefixSumBool(true);
            }
            else if (isLeaf)
            {
                boxOrLeafChildIndex = laneGroup.ExclusivePrefixSumBool(true);
            }

            const uint numInlinePrimStructs =
                GetNumInlinePrimStructs(laneGroup, srcNodePtr, numLeafChildren, isRebraidRootNode, isLeaf);

            uint baseDstNodeIdx = 0;
            uint baseDstStackIdx = 0;

            if (laneGroup.IsFirstLane())
            {
                const uint numNodesAlloc = numBoxChildren + numInlinePrimStructs;
                if (numNodesAlloc > 0)
                {
                    baseDstNodeIdx = EncodeHwBvh::IncrementCounter(STGB_COUNTER_IDX_NODE_ALLOC, numNodesAlloc);
                }

                if (numBoxChildren > 0)
                {
                    baseDstStackIdx = EncodeHwBvh::IncrementCounter(STGB_COUNTER_IDX_BOX_REF, numBoxChildren);
                }
            }

            baseDstStackIdx = laneGroup.ReadFirstLane(baseDstStackIdx);
            baseDstNodeIdx = laneGroup.ReadFirstLane(baseDstNodeIdx);

            const uint baseBoxNodeOffset = offsets.internalNodes + baseDstNodeIdx * QBVH8::NodeSizeInBytes;
            const uint baseLeafNodeOffset =
                EnablePrimitiveCompressionPass() ?
                    0u : baseBoxNodeOffset + (numBoxChildren * QBVH8::NodeSizeInBytes);

            if (isBox)
            {
                const uint packedStackIndex = PackStackDstNodeIdx(baseDstNodeIdx + boxOrLeafChildIndex, boxOrLeafChildIndex);
                WriteStackIndex(baseDstStackIdx + boxOrLeafChildIndex, uint2(srcNodePtr, packedStackIndex));
            }

            BoundingBox bbox = (BoundingBox)0;

            if (laneGroup.IsFirstLane())
            {
                // TODO: Support OBBs
                bbox = ReadBvh2NodeBounds(bvhNodeSrcIdx);

                // The root of the BVH2 treelet contains the combined bounds of the child nodes in the node array. Compute the
                // common exponent from the root node bounds
                const uint3 exponents = ComputeCommonExponent(bbox.min, bbox.max, 12);

                const uint validChildCount = numBoxChildren + numLeafChildren;

                QBVH8::WriteInternalNodeBaseOffset(currNodeOffset, baseBoxNodeOffset);
                QBVH8::WriteLeafNodeBaseOffset(currNodeOffset, baseLeafNodeOffset);
                QBVH8::WriteOrigin(currNodeOffset, bbox.min);
                QBVH8::WritePackedExpChildIdxAndCount(currNodeOffset, exponents, indexInParent, validChildCount);
                QBVH8::WriteObbMatrixIndex(currNodeOffset, PackObbMatrixIndexBits(INVALID_OBB, packedSourceBoxes));
            }

            bbox.min = laneGroup.ReadFirstLane(bbox.min);
            bbox.max = laneGroup.ReadFirstLane(bbox.max);

            // Pre-compute exponent reciprocal
            const uint3 exponents = ComputeCommonExponent(bbox.min, bbox.max, 12);
            const float3 rcpExponents = ComputeFastExpReciprocal(exponents, 12);

            uint lanePrimRefCount = 0;
            float aabbSa = 0.0f;
            uint bestObbMatrixIdx = INVALID_OBB;

            const uint currNodePtr = PackNodePointer(NODE_TYPE_BOX_QUANTIZED_BVH8, currNodeOffset);

            // Process and compress box children
            if (srcNodePtr != INVALID_IDX)
            {
                // Initialise primitive range length to 0 for BLAS. It is updated later by CompressPrims pass.
                const uint primRangeLength = (isLeaf && EnablePrimitiveCompressionPass()) ? 0 : 1;
                const uint nodeType = isLeaf ? NODE_TYPE_TRIANGLE_0 : NODE_TYPE_BOX_QUANTIZED_BVH8;

                const BoundingBox childBounds = ReadBvh2NodeBounds(GetBvh2NodeIdx(srcNodePtr));
                const uint flags = ReadBvh2NodeFlags(GetBvh2NodeIdx(srcNodePtr));

                const uint3 childInfo = QBVH8::ComputePackedChildInfo(childBounds,
                                                                      ExtractScratchNodeInstanceMask(flags),
                                                                      ExtractScratchNodeBoxFlags(flags),
                                                                      nodeType,
                                                                      primRangeLength,
                                                                      rcpExponents,
                                                                      bbox.min);

                const uint childIdx = isLeaf ? boxOrLeafChildIndex + numBoxChildren : boxOrLeafChildIndex;
                QBVH8::WritePackedChildInfo(currNodeOffset, childIdx, childInfo);

                if (isLeaf)
                {
                    lanePrimRefCount = GetBvh2PrimitiveCount(srcNodePtr);
                }
                else
                {
                    const uint childNodeOffset = baseBoxNodeOffset + (boxOrLeafChildIndex * QBVH8::NodeSizeInBytes);
                    QBVH8::WriteParentPointer(childNodeOffset, currNodePtr);
                }

                if ((Settings.enableOrientedBoundingBoxes & bit(BOTTOM_LEVEL)) != 0)
                {
                    aabbSa = ComputeBoxSurfaceArea(childBounds);

                    if (isLeaf)
                    {
                        if (IsBvh2PrimRefNode(srcNodePtr))
                        {
                            // Note, OBB matrix index is stored in the quad swizzle field
                            bestObbMatrixIdx = ExtractScratchNodeQuadSwizzle(flags);
                        }
                        else
                        {
                            // Triangles in a range are colocated and tend to share a similar orientation, so
                            // choosing one at random from the range is about as effective as choosing one with
                            // a more sophisticated heuristic.
                            //
                            // Pick the first triangle in the range as an approximation.

                            // TODO: Support primRange > 2
                            const uint2 child = ReadBvh2ChildPtrs(GetBvh2NodeIdx(srcNodePtr));
                            const uint triNodePtr = child.x;

                            // Note, OBB matrix index is stored in the quadSwizzle field
                            bestObbMatrixIdx =
                                ExtractScratchNodeQuadSwizzle(ReadBvh2NodeFlags(GetBvh2NodeIdx(triNodePtr)));
                        }
                    }
                }
            }
            else
            {
                QBVH8::WriteInvalidChildInfo(currNodeOffset, laneGroup.laneIndex);
            }

            if ((Settings.enableOrientedBoundingBoxes & bit(BOTTOM_LEVEL)) != 0)
            {
                const float aabbSum = laneGroup.Sum(aabbSa);
                const float aabbMax = laneGroup.Max(aabbSa);

                const uint aabbMaxChildIdx = laneGroup.GetFirstActiveLaneIndex(aabbSa == aabbMax);
                bestObbMatrixIdx = laneGroup.Broadcast(bestObbMatrixIdx, aabbMaxChildIdx);

                if (laneGroup.IsFirstLane())
                {
                    if (numBoxChildren == 0)
                    {
                        uint obbRefitStackIndex = 0;

                        ScratchBuffer.InterlockedAdd(
                            ShaderConstants.offsets.obbRefitStackPtrs + STACK_PTRS_OBB_REFIT_BATCH_COUNT, 1, obbRefitStackIndex);

                        ScratchBuffer.Store(
                            ShaderConstants.offsets.obbRefitStack + (obbRefitStackIndex * sizeof(uint)), bvhNodeDstIdx);
                    }

                    // OBB refit flags are packed as follows:
                    // b[0:3] : 2's complement of box child count. The maximum value can be 8. That requires 3 bits plus the sign
                    //          bit --> 4 bits. The maximum carry bits (for when box count is incremented by refit kernel)
                    //          is 1, but we have plenty more room.
                    // b[4:15] : 2's complement increment overflow bits.
                    // b[16:23]: Child index with maximum AABB surface area for this node.
                    // b[24:31]: OBB matrix index of the best leaf node amongst the children. This is precomputed and stored in
                    //           scratch node during morton code generation phase to absorb the VGPR impact of that computation.
                    const uint encodedBits =
                        (bestObbMatrixIdx << 24u) | (aabbMaxChildIdx << 16u) | ((numBoxChildren ^ 0xFu) + 1);

                    const uint obbFlagsOffset =
                        ShaderConstants.offsets.qbvhObbFlags + (bvhNodeDstIdx * OBB_TABLE_ENTRY_SIZE);

                    ScratchBuffer.Store<uint2>(obbFlagsOffset, uint2(encodedBits, asuint(aabbSum)));
                }
            }

            // Update primitive reference counter for BLAS
            const uint wavePrimRefCount = WaveActiveSum(lanePrimRefCount);
            if (WaveIsFirstLane())
            {
                EncodeHwBvh::IncrementCounter(STGB_COUNTER_IDX_PRIM_REF, wavePrimRefCount);
            }

            if (EnablePrimitiveCompressionPass() == false)
            {
                uint basePrimStructIdx = 0;
                if (laneGroup.IsFirstLane())
                {
                    basePrimStructIdx =
                        IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET, numLeafChildren);
                }
                basePrimStructIdx = laneGroup.ReadFirstLane(basePrimStructIdx);
                const uint primStructIdx = basePrimStructIdx + boxOrLeafChildIndex;

                if (isLeaf)
                {
                    uint triNodePtr0 = INVALID_IDX;
                    uint triNodePtr1 = INVALID_IDX;

                    if (IsBvh2PrimRefNode(srcNodePtr))
                    {
                        triNodePtr0 = srcNodePtr;
                        triNodePtr1 = INVALID_IDX;
                    }
                    else
                    {
                        // Only maxPrimRangeSize=2 is supported in this path.
                        const uint2 childPtrs = ReadBvh2ChildPtrs(GetBvh2NodeIdx(srcNodePtr));
                        triNodePtr0 = childPtrs.x;
                        triNodePtr1 = childPtrs.y;
                    }

                    const uint2 id0 = ReadBvh2ChildPtrs(GetBvh2NodeIdx(triNodePtr0));
                    const uint flags0 = ReadBvh2NodeFlags(GetBvh2NodeIdx(triNodePtr0));
                    const TriangleData tri0 = FetchTrianglePrimitiveData(id0.x, id0.y);

                    const bool isTri1Valid = (triNodePtr1 != INVALID_IDX);

                    uint2 id1 = uint2(0, 0);
                    uint flags1 = 0;
                    TriangleData tri1 = (TriangleData)0;

                    if (isTri1Valid)
                    {
                        id1 = ReadBvh2ChildPtrs(GetBvh2NodeIdx(triNodePtr1));
                        flags1 = ReadBvh2NodeFlags(GetBvh2NodeIdx(triNodePtr1));
                        tri1 = FetchTrianglePrimitiveData(id1.x, id1.y);
                    }

                    const uint primStructOffset = baseLeafNodeOffset + (boxOrLeafChildIndex * QBVH8::NodeSizeInBytes);
                    DstBuffer.Store(offsets.primNodePtrs + (primStructIdx * sizeof(uint)), primStructOffset);

                    PrimStruct3_1::WritePairPrimStruct(tri0,
                                                       IsOpaqueNode(flags0),
                                                       id0.y,
                                                       id0.x,
                                                       isTri1Valid,
                                                       tri1,
                                                       IsOpaqueNode(flags1),
                                                       id1.y,
                                                       id1.x,
                                                       primStructOffset);
                }
            }
            else if (numLeafChildren > 0)
            {
                const uint nodePrimRefCount = laneGroup.Sum(lanePrimRefCount);
                const uint packedParentInfo = PackCompBatchParentInfo(currNodeOffset, numBoxChildren, numLeafChildren);

                const uint baseLdsOffset = GetNodeArrayBaseInLds(threadId, maxNumChildren) + numBoxChildren;
                if (nodePrimRefCount == 1)
                {
                    // The srcNodePtr in a lane group with exactly one primitive reference resides in the one and only
                    // leaf lane in the lane group
                    if (isLeaf)
                    {
                        // Repurpose the OBB matrix index slot for writing the BVH2 parent slot index
                        // Note, packedSourceBoxes is a 8-bit value that needs to be preserved. srcNodePtr can
                        // have a maximum value of 2046, and requires only 12-bits for the index. Bits 30:31
                        // are reserved for indicating a leaf node vs primitive reference and needs to be preserved.
                        //
                        // index            : 16
                        // packedSourceBoxes: 8
                        // unused           : 6
                        // leaf_node        : 1
                        // primitive_ref    : 1
                        //
                        QBVH8::WriteObbMatrixIndex(currNodeOffset, PackObbMatrixIndexBits(srcNodePtr, packedSourceBoxes));

                        // Enqueue single triangle batches to the end of the buffer
                        const uint batchIdx = EncodeHwBvh::IncrementCounter(STGB_COUNTER_IDX_PRIM_COMP_BATCH_SINGLE, 1) + 1;

                        ScratchBuffer.Store(
                            ShaderConstants.offsets.primCompNodeIndicesEnd - (batchIdx * sizeof(uint)), packedParentInfo);
                    }
                }
                else
                {
                    if (laneGroup.IsFirstLane())
                    {
                        // Repurpose the OBB matrix index slot for writing the BVH2 parent slot index
                        // Note, packedSourceBoxes is a 8-bit value that needs to be preserved. bvhNodeSrcIdx can
                        // have a maximum value of 2046, and requires only 12-bits for the index. Bits 30:31
                        // are reserved for indicating a leaf node vs primitive reference and needs to be preserved.
                        //
                        // index            : 16
                        // packedSourceBoxes: 8
                        // unused           : 6
                        // leaf_node        : 1
                        // primitive_ref    : 1
                        //
                        QBVH8::WriteObbMatrixIndex(currNodeOffset, PackObbMatrixIndexBits(bvhNodeSrcIdx, packedSourceBoxes));

                        // Multiple triangle batches are enqueued to the start of the buffer
                        const uint batchIdx = EncodeHwBvh::IncrementCounter(STGB_COUNTER_IDX_PRIM_COMP_BATCH, 1);

                        ScratchBuffer.Store(
                            ShaderConstants.offsets.primCompNodeIndices + (batchIdx * sizeof(uint)), packedParentInfo);
                    }

                    const uint bvh2NodeOffset = GetBvh2NodeOffset(bvhNodeSrcIdx);
                    if (isLeaf)
                    {
                        ScratchBuffer.Store(bvh2NodeOffset + (boxOrLeafChildIndex * sizeof(uint)), srcNodePtr);
                    }
                }
            }

            // Write fat leaf offsets.
            if (IsUpdateAllowed() && (numBoxChildren == 0) && laneGroup.IsFirstLane())
            {
                const uint index = IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP16_NODES_OFFSET, 1);
                const uint baseUpdatePtrOffset = offsets.primNodePtrs + (ShaderConstants.maxNumPrimitives * sizeof(uint));
                DstBuffer.Store(baseUpdatePtrOffset + (index * sizeof(uint)), currNodeOffset);
            }

            // Move to next global stack chunk
            globalStackIndex += (STGB_THREADGROUP_SIZE / 8);
        }
    }

    AllMemoryBarrierWithGroupSync();

    WriteAccelStructHeaderField(
        ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET, EncodeHwBvh::ReadCounter(STGB_COUNTER_IDX_BOX_REF));
}
