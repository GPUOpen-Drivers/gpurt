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
void WriteScratchNodeCollapseInfo(
    uint  baseScratchNodesOffset,
    uint  nodeIndex,
    float cost,
    uint  numPrimitivesAndDoCollapse)
{
    const uint nodeOffset = baseScratchNodesOffset + (nodeIndex * SCRATCH_NODE_SIZE);

    ScratchBuffer.Store<float>(nodeOffset + SCRATCH_NODE_RANGE_OFFSET, cost);
    ScratchBuffer.Store(nodeOffset + SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET, numPrimitivesAndDoCollapse);
}

//=====================================================================================================================
void RefitBoundsImpl(
    uint                primIndex,
    uint                numActivePrims,
    uint                baseFlagsOffset,
    uint                baseScratchNodesOffset,
    uint                doCollapse,
    uint                doTriangleSplitting,
    uint                enablePairCompression,
    uint                enablePairCostCheck,
    uint                splitBoxesOffset,
    uint                numBatchesOffset,
    uint                baseBatchIndicesOffset,
    uint                fp16BoxNodeMode,
    float               fp16BoxModeMixedSaThreshold,
    uint                reservedUint1,
    uint                reservedUint2,
    uint                enableInstancePrimCount)
{
    if (enablePairCompression && (primIndex == 0) && (numActivePrims == 1))
    {
        // Ensure that a batch index is written out for single-primitive acceleration structures.
        WriteBatchIndex(ScratchBuffer, numBatchesOffset, baseBatchIndicesOffset, 0);
    }

    // Start from leaf node referenced by this thread
    uint nodeIndex = LEAFIDX(primIndex);

    uint numTris = 1;

    uint instancePrimCount = 0;

    //Ct is the cost of intersecting triangle
    //Ci is the cost of interecting bbox
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

            if (doTriangleSplitting)
            {
                if (IsLeafNode(rc, numActivePrims))
                {
                    bboxRightChild =
                        ScratchBuffer.Load<BoundingBox>(splitBoxesOffset +
                                                        sizeof(BoundingBox) * rightNode.splitBox_or_nodePointer);

                    rightCost = ComputeBoxSurfaceArea(bboxRightChild) * Ct;

                    numMortonCells++;
                }
                else
                {
                    bboxRightChild = GetScratchNodeBoundingBox(rightNode);

                    rightCost = ComputeBoxSurfaceArea(bboxRightChild) * rightNode.range_or_v2_or_instBasePtr.x;

                    numMortonCells += rightNode.splitBox_or_nodePointer;
                }

                if (IsLeafNode(lc, numActivePrims))
                {
                    bboxLeftChild =
                        ScratchBuffer.Load<BoundingBox>(splitBoxesOffset +
                                                        sizeof(BoundingBox) * leftNode.splitBox_or_nodePointer);

                    leftCost = ComputeBoxSurfaceArea(bboxLeftChild) * Ct;

                    numMortonCells++;
                }
                else
                {
                    bboxLeftChild = GetScratchNodeBoundingBox(leftNode);

                    leftCost = ComputeBoxSurfaceArea(bboxLeftChild) * leftNode.range_or_v2_or_instBasePtr.x;

                    numMortonCells += leftNode.splitBox_or_nodePointer;
                }
            }
            else
            {
                if (IsLeafNode(rc, numActivePrims))
                {
                    bboxRightChild = GetScratchNodeBoundingBox(rightNode);

                    rightCost = ComputeBoxSurfaceArea(bboxRightChild) * Ct;

                    numMortonCells++;
                }
                else
                {
                    bboxRightChild = GetScratchNodeBoundingBox(rightNode);
                    rightCost = ComputeBoxSurfaceArea(bboxRightChild) * rightNode.range_or_v2_or_instBasePtr.x;

                    numMortonCells += rightNode.splitBox_or_nodePointer;
                }

                if (IsLeafNode(lc, numActivePrims))
                {
                    bboxLeftChild = GetScratchNodeBoundingBox(leftNode);

                    leftCost = ComputeBoxSurfaceArea(bboxLeftChild) * Ct;

                    numMortonCells++;
                }
                else
                {
                    bboxLeftChild = GetScratchNodeBoundingBox(leftNode);
                    leftCost = ComputeBoxSurfaceArea(bboxLeftChild) * leftNode.range_or_v2_or_instBasePtr.x;

                    numMortonCells += leftNode.splitBox_or_nodePointer;
                }
            }

            // Merge bounding boxes up to parent
            const float3 bboxMinParent = min(bboxLeftChild.min, bboxRightChild.min);
            const float3 bboxMaxParent = max(bboxLeftChild.max, bboxRightChild.max);
            WriteScratchNodeBoundingBox(ScratchBuffer,
                                        baseScratchNodesOffset,
                                        parentNodeIndex,
                                        bboxMinParent,
                                        bboxMaxParent);
            if (doCollapse || enablePairCompression)
            {
                const BoundingBox mergedBbox = { bboxMinParent, bboxMaxParent };

                const float surfaceArea  = ComputeBoxSurfaceArea(mergedBbox);
                const float splitCost    = Ci + leftCost / surfaceArea + rightCost / surfaceArea;
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
                    WriteScratchNodeCollapseInfo(baseScratchNodesOffset,
                                                 parentNodeIndex,
                                                 splitCost,
                                                 numTris << 1);

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
                    WriteScratchNodeCollapseInfo(baseScratchNodesOffset,
                                                 parentNodeIndex,
                                                 collapseCost,
                                                 (numTris << 1) | 1);
                }
            }
            else
            {
                WriteScratchNodeNumPrimitives(ScratchBuffer,
                                              baseScratchNodesOffset,
                                              parentNodeIndex,
                                              numTris << 1);
            }

            // Decide on what type of interior box node the parent should be
            // and write the type into scratch
            WriteScratchNodeType(ScratchBuffer,
                                 baseScratchNodesOffset,
                                 fp16BoxNodeMode,
                                 fp16BoxModeMixedSaThreshold,
                                 parentNodeIndex,
                                 leftNode.type,
                                 rightNode.type,
                                 bboxMinParent,
                                 bboxMaxParent);

            WriteScratchNodeFlags(ScratchBuffer,
                                  baseScratchNodesOffset,
                                  parentNodeIndex,
                                  leftNode,
                                  rightNode);

            if (enableInstancePrimCount)
            {
                const uint instancePrimCount = asuint(rightNode.range_or_v2_or_instBasePtr[2]) +
                                               asuint(leftNode.range_or_v2_or_instBasePtr[2]);

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