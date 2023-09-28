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
#include "BuildCommonScratch.hlsl"

//=====================================================================================================================
void RefitBoundsImpl(
    uint                primIndex,
    uint                numActivePrims,
    uint                baseFlagsOffset,
    uint                baseScratchNodesOffset,
    uint                unsortedNodesBaseOffset,
    uint                sortedPrimIndicesOffset,
    uint                doCollapse,
    uint                doTriangleSplitting,
    uint                noCopySortedNodes,
    uint                enableEarlyPairCompression,
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
    uint                enableInstancePrimCount,
    uint                reservedUint5,
    uint                reservedUint6,
    uint                reservedUint7
)
{
    const bool doLatePairCompression = enablePairCompression && (!enableEarlyPairCompression);
    if (doLatePairCompression && (primIndex == 0) && (numActivePrims == 1))
    {
        // Ensure that a batch index is written out for single-primitive acceleration structures.
        WriteScratchBatchIndex(numBatchesOffset, baseBatchIndicesOffset, 0);
    }

    // Start from leaf node referenced by this thread
    uint nodeIndex = noCopySortedNodes ?
        LEAFIDX(FetchSortedPrimIndex(sortedPrimIndicesOffset, primIndex)) :
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
        const uint parentNodeIndex = FetchScratchNode(baseScratchNodesOffset, nodeIndex).parent;

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
            const ScratchNode parentNode = FetchScratchNode(baseScratchNodesOffset, parentNodeIndex);

            // Fetch child indices
            const uint lc = parentNode.left_or_primIndex_or_instIndex;
            const uint rc = parentNode.right_or_geometryIndex;

            const ScratchNode leftNode  = FetchScratchNode(baseScratchNodesOffset, lc);
            const ScratchNode rightNode = FetchScratchNode(baseScratchNodesOffset, rc);

            // Fetch bounding children bounding boxes
            BoundingBox bboxRightChild;
            BoundingBox bboxLeftChild;

            uint numMortonCells = 0;

            // Right child
            {
                const bool isLeafNode = IsLeafNode(rc, numActivePrims);

                bboxRightChild = FetchScratchNodeBoundingBox(rightNode,
                                                             isLeafNode,
                                                             doTriangleSplitting,
                                                             splitBoxesOffset,
                                                             enableEarlyPairCompression,
                                                             unsortedNodesBaseOffset);

                numMortonCells += isLeafNode ? 1 : rightNode.splitBox_or_nodePointer;
            }

            // Left child
            {
                const bool isLeafNode = IsLeafNode(lc, numActivePrims);

                bboxLeftChild = FetchScratchNodeBoundingBox(leftNode,
                                                            isLeafNode,
                                                            doTriangleSplitting,
                                                            splitBoxesOffset,
                                                            enableEarlyPairCompression,
                                                            unsortedNodesBaseOffset);

                numMortonCells += isLeafNode ? 1 : leftNode.splitBox_or_nodePointer;
            }

            // Merge bounding boxes up to parent
            BoundingBox mergedBox = (BoundingBox)0;
            mergedBox.min = min(bboxLeftChild.min, bboxRightChild.min);
            mergedBox.max = max(bboxLeftChild.max, bboxRightChild.max);

            // Compute surface area of parent box
            const float mergedBoxSurfaceArea = ComputeBoxSurfaceArea(mergedBox);

            WriteScratchNodeBoundingBox(baseScratchNodesOffset,
                                        parentNodeIndex,
                                        mergedBox.min,
                                        mergedBox.max);

            MergeScratchNodes(
                baseScratchNodesOffset,
                numBatchesOffset,
                baseBatchIndicesOffset,
                numActivePrims,
                parentNodeIndex,
                lc,
                leftNode,
                rc,
                rightNode,
                mergedBoxSurfaceArea);

            if (enableInstancePrimCount)
            {
                const uint instancePrimCount = asuint(rightNode.sah_or_v2_or_instBasePtr[2]) +
                                               asuint(leftNode.sah_or_v2_or_instBasePtr[2]);

                WriteScratchNodeInstanceNumPrims(baseScratchNodesOffset,
                                                 parentNodeIndex,
                                                 instancePrimCount);
            }
        }

        DeviceMemoryBarrier();

        // Set parent node as next node index
        nodeIndex = parentNodeIndex;
    }
}
