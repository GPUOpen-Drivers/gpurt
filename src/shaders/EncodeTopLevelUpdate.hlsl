/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
void WriteInstanceDescriptor(
    in InstanceDesc       instanceDesc,
    in uint               geometryType,
    in uint               boxNodeFlags,
    in uint               instanceIndex,
    in uint               instNodePtr,
    in uint               blasRootNodePointer,
    in uint               blasMetadataSize,
    in uint               tlasMetadataSize)
{
    {
        WriteInstanceNode1_1(instanceDesc,
                             geometryType,
                             instanceIndex,
                             instNodePtr,
                             blasRootNodePointer,
                             blasMetadataSize,
                             tlasMetadataSize);
    }
}

//=====================================================================================================================
void EncodeInstancesUpdate(
    uint               index,
    InstanceDesc       desc,
    uint               tlasMetadataSize,
    uint               primNodePointerOffset,
    uint64_t           baseAddrAccelStructHeader,
    uint               numActivePrims,
    uint               blasMetadataSize)
{
    // If the BLAS address is NULL, the instance is inactive. Inactive instances cannot be activated during updates
    // so we always exclude them from the build.
    if (baseAddrAccelStructHeader != 0)
    {
        const uint geometryType =
            FetchHeaderField(baseAddrAccelStructHeader, ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET);

        // calc transformed AABB
        BoundingBox boundingBox;

        if (numActivePrims != 0)
        {
            // Fetch root bounds from BLAS header
            const BoundingBox rootBbox = FetchHeaderRootBoundingBox(baseAddrAccelStructHeader);

            boundingBox = GenerateInstanceBoundingBox(desc.Transform, rootBbox);
        }

        const uint packedFlags = FetchHeaderField(baseAddrAccelStructHeader, ACCEL_STRUCT_HEADER_PACKED_FLAGS_OFFSET);
        const uint boxNodeFlags = CalcTopLevelBoxNodeFlags(geometryType,
                                                           desc.InstanceContributionToHitGroupIndex_and_Flags >> 24,
                                                           ExtractScratchNodeBoxFlags(packedFlags));

        // Propagate the instance mask from the BLAS by fetching it from the header.
        // Note, the header contains the exlusion mask so we take its bitwise complement to get the inclusion mask.
        // Set the instance inclusion mask to 0 for degenerate instances so that they are culled out.
        const uint instanceMask = (boundingBox.min.x > boundingBox.max.x) ?
                                  0 :
                                  (GetInstanceMask(desc) & ExtractScratchNodeInstanceMask(packedFlags));

        if (numActivePrims != 0)
        {
            const uint nodePointer = SrcBuffer.Load(primNodePointerOffset);

            uint parentNodePointer;
            {
                parentNodePointer = ReadParentPointer(tlasMetadataSize, nodePointer);
            }

            // Update prim node pointer and parent pointer in out of place destination buffer
            if (IsUpdateInPlace() == false)
            {
                DstMetadata.Store(primNodePointerOffset, nodePointer);

                {
                    WriteParentPointer(tlasMetadataSize, nodePointer, parentNodePointer);
                }
            }

            // Compute box node count and child index in parent node
            uint boxNodeCount = 0;
            uint childIdx = ComputeChildIndexAndValidBoxCount(tlasMetadataSize,
                                                              parentNodePointer,
                                                              nodePointer,
                                                              boxNodeCount);

            // If even a single child node is a box node, this is a node higher up the tree. Skip queueing parent
            // node as another leaf at the bottom of the tree will queue its parent which will handle our parent
            // node.

            // B B B B --> 4 --> Not possible
            // B x B x --> 2 --> Not possible
            // B L B L --> 4 --> Skip queuing
            // L x B x --> 1 --> Skip queuing
            // L x x x --> 0 --> Queue parent node
            // L x L x --> 0 --> Queue parent node
            // L L L L --> 0 --> Queue parent node

            const bool pushNodeToUpdateStack = (childIdx == 0) && (boxNodeCount == 0);

            if (pushNodeToUpdateStack && (IsUpdateInPlace() == false))
            {
                // Copy child pointers and flags to destination buffer. Note, the new instance flags
                // get updated atomically below
                CopyChildPointersAndFlags(parentNodePointer, tlasMetadataSize);
            }

            uint baseBoxNodeOffset = tlasMetadataSize + ExtractNodePointerOffset(parentNodePointer);

            {

                const uint childBboxOffset = FLOAT32_BOX_NODE_BB0_MIN_OFFSET + (childIdx * FLOAT32_BBOX_STRIDE);
                DstMetadata.Store<BoundingBox>(baseBoxNodeOffset + childBboxOffset, boundingBox);

                const uint boxNodeFlagsClearBits = ~(0xFFu << (childIdx * BOX_NODE_FLAGS_BIT_STRIDE));
                const uint boxNodeFlagsSetBits = (boxNodeFlags << (childIdx * BOX_NODE_FLAGS_BIT_STRIDE));

                // The instance flags can be changed in an update, so the node flags need to be updated.
                DstMetadata.InterlockedAnd(baseBoxNodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET,
                                           boxNodeFlagsClearBits);
                DstMetadata.InterlockedOr(baseBoxNodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET,
                                          boxNodeFlagsSetBits);
            }

            // If this is the first child in the parent node with all leaf children, queue parent pointer to
            // stack in scratch memory
            if (pushNodeToUpdateStack)
            {
                PushNodeToUpdateStack(ShaderConstants.offsets.updateStack, parentNodePointer);
            }

            WriteInstanceDescriptor(desc,
                                    geometryType,
                                    boxNodeFlags,
                                    index,
                                    nodePointer,
                                    CreateRootNodePointer(),
                                    blasMetadataSize,
                                    tlasMetadataSize);
        }
    }
}
