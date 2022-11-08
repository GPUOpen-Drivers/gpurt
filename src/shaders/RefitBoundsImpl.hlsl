/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2022 Advanced Micro Devices, Inc. All Rights Reserved.
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
//=====================================================================================================================
// The functions defined below require the following resources defined before including this header
//
// Must include BuildCommon.hlsl
//

//=====================================================================================================================
void RefitBoundsImpl(
    uint                primIndex,
    uint                numActivePrims,
    uint                baseFlagsOffset,
    uint                baseScratchNodesOffset,
    uint                sortedPrimIndicesOffset,
    uint                doCollapse,
    uint                doTriangleSplitting,
    uint                noCopySortedNodes,
    uint                enablePairCompression,
    uint                enablePairCostCheck,
    uint                splitBoxesOffset,
    uint                numBatchesOffset,
    uint                baseBatchIndicesOffset,
    uint                fp16BoxNodeMode,
    float               fp16BoxModeMixedSaThreshold,
    uint                reservedUint1,
    uint                reservedUint2,
    uint                reservedUint3,
    int4                reservedUint4,
    uint                enableInstancePrimCount)
{
    if (enablePairCompression && (primIndex == 0) && (numActivePrims == 1))
    {
        // Ensure that a batch index is written out for single-primitive acceleration structures.
        WriteBatchIndex(ScratchBuffer, numBatchesOffset, baseBatchIndicesOffset, 0);
    }

    // Start from leaf node referenced by this thread
    uint nodeIndex = noCopySortedNodes ?
        LEAFIDX(FetchSortedPrimIndex(ScratchBuffer, sortedPrimIndicesOffset, primIndex)) :
        LEAFIDX(primIndex);

    uint numTris = 1;

    uint instancePrimCount = 0;

    // Ct is the cost of intersecting triangle
    // Ci is the cost of interecting bbox
    const float Ct = SAH_COST_TRIANGLE_INTERSECTION;
    const float Ci = SAH_COST_AABBB_INTERSECTION;

    // Build tree up until we hit root node
    while (nodeIndex != 0)
    {
        // Move to parent node
        const uint parentNodeIndex = FetchScratchNode(ScratchBuffer, baseScratchNodesOffset, nodeIndex).parent;

        // Check parent node's flag
        const uint flagOffset = baseFlagsOffset + (parentNodeIndex * sizeof(uint));
        uint originalFlagValue = 0;

        ScratchBuffer.InterlockedAdd(flagOffset, numTris, originalFlagValue);

        if (originalFlagValue == 0)
        {
            // If the flag was 0 set it to 1 and bail out. The thread handling the second child will handle this node.
            return;
        }
        else
        {
            numTris += originalFlagValue;

            // If the flag was 1 the second child is ready and this thread calculates and writes
            // bbox data for the parent node
            const ScratchNode parentNode = FetchScratchNode(ScratchBuffer, baseScratchNodesOffset, parentNodeIndex);

            // Fetch child indices
            const uint lc = parentNode.left_or_primIndex_or_instIndex;
            const uint rc = parentNode.right_or_geometryIndex;

            const ScratchNode leftNode  = FetchScratchNode(ScratchBuffer, baseScratchNodesOffset, lc);
            const ScratchNode rightNode = FetchScratchNode(ScratchBuffer, baseScratchNodesOffset, rc);

            // Fetch bounding children bounding boxes
            BoundingBox bboxRightChild;
            BoundingBox bboxLeftChild;

            float rightCost;
            float leftCost;

            uint numMortonCells = 0;

            // Right child
            {
                const bool isLeafNode = IsLeafNode(rc, numActivePrims);

                if (doTriangleSplitting && isLeafNode)
                {
                    bboxRightChild =
                        ScratchBuffer.Load<BoundingBox>(splitBoxesOffset +
                                                        sizeof(BoundingBox) * rightNode.splitBox_or_nodePointer);
                }
                else
                {
                    bboxRightChild = GetScratchNodeBoundingBox(rightNode);
                }

                rightCost = isLeafNode ? FetchScratchLeafNodeCost(rightNode) : FetchScratchInternalNodeCost(rightNode);
                numMortonCells += isLeafNode ? 1 : rightNode.splitBox_or_nodePointer;
            }

            // Left child
            {
                const bool isLeafNode = IsLeafNode(lc, numActivePrims);

                if (doTriangleSplitting && isLeafNode)
                {
                    bboxLeftChild =
                        ScratchBuffer.Load<BoundingBox>(splitBoxesOffset +
                                                        sizeof(BoundingBox) * leftNode.splitBox_or_nodePointer);
                }
                else
                {
                    bboxLeftChild = GetScratchNodeBoundingBox(leftNode);
                }

                leftCost = isLeafNode ? FetchScratchLeafNodeCost(leftNode) : FetchScratchInternalNodeCost(leftNode);
                numMortonCells += isLeafNode ? 1 : leftNode.splitBox_or_nodePointer;
            }

            // Merge bounding boxes up to parent
            BoundingBox mergedBox = (BoundingBox)0;
            mergedBox.min = min(bboxLeftChild.min, bboxRightChild.min);
            mergedBox.max = max(bboxLeftChild.max, bboxRightChild.max);

            // Compute surface area of parent box
            const float mergedBoxSurfaceArea = ComputeBoxSurfaceArea(mergedBox);

            WriteScratchNodeBoundingBox(ScratchBuffer,
                                        baseScratchNodesOffset,
                                        parentNodeIndex,
                                        mergedBox.min,
                                        mergedBox.max);

            WriteScratchNodeSurfaceArea(ScratchBuffer,
                                        baseScratchNodesOffset,
                                        parentNodeIndex,
                                        mergedBoxSurfaceArea);

            float bestCost = leftCost + rightCost + Ci * mergedBoxSurfaceArea;
            bool isCollapsed = false;

            if (doCollapse || enablePairCompression)
            {
                const float splitCost    = Ci + leftCost / mergedBoxSurfaceArea + rightCost / mergedBoxSurfaceArea;
                const float collapseCost = Ct * numTris;
                const bool  useCostCheck = doCollapse || enablePairCostCheck;

                const bool leftCollapse      = (leftNode.numPrimitivesAndDoCollapse & 0x1) ||
                                               IsLeafNode(lc, numActivePrims);
                const bool rightCollapse     = (rightNode.numPrimitivesAndDoCollapse & 0x1) ||
                                               IsLeafNode(rc, numActivePrims);
                const bool collapseBothSides = leftCollapse && rightCollapse;

                // Limit number of triangles collapsed in a single bounding box to MAX_COLLAPSED_TRIANGLES
                if ((useCostCheck && (collapseCost > splitCost)) ||
                    (numTris > MAX_COLLAPSED_TRIANGLES) ||
                    (enablePairCompression && ((collapseBothSides == false) || (parentNodeIndex == 0))))
                {
                    if (enablePairCompression)
                    {
                        if (leftCollapse)
                        {
                            WriteBatchIndex(ScratchBuffer, numBatchesOffset, baseBatchIndicesOffset, lc);
                        }

                        if (rightCollapse)
                        {
                            WriteBatchIndex(ScratchBuffer, numBatchesOffset, baseBatchIndicesOffset, rc);
                        }
                    }
                }
                else
                {
                    bestCost = collapseCost * mergedBoxSurfaceArea;
                    isCollapsed = true;
                }
            }

            WriteScratchNodeCost(ScratchBuffer, baseScratchNodesOffset, parentNodeIndex, bestCost, false);
            WriteScratchNodeNumPrimitives(ScratchBuffer, baseScratchNodesOffset, parentNodeIndex, numTris, isCollapsed);

            // Decide on what type of interior box node the parent should be
            // and write the type into scratch
            WriteScratchNodeType(ScratchBuffer,
                                 baseScratchNodesOffset,
                                 fp16BoxNodeMode,
                                 fp16BoxModeMixedSaThreshold,
                                 parentNodeIndex,
                                 leftNode.type,
                                 rightNode.type,
                                 mergedBox.min,
                                 mergedBox.max);

            WriteScratchNodeFlagsFromNodes(ScratchBuffer,
                                           baseScratchNodesOffset,
                                           parentNodeIndex,
                                           leftNode,
                                           rightNode);

            if (enableInstancePrimCount)
            {
                const uint instancePrimCount = asuint(rightNode.sah_or_v2_or_instBasePtr[2]) +
                                               asuint(leftNode.sah_or_v2_or_instBasePtr[2]);

                WriteScratchNodeInstanceNumPrims(ScratchBuffer,
                                                 baseScratchNodesOffset,
                                                 parentNodeIndex,
                                                 instancePrimCount);
            }
        }

        DeviceMemoryBarrier();

        // Set parent node as next node index
        nodeIndex = parentNodeIndex;
    }
}
