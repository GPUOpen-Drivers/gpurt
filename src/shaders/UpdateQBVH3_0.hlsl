/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef UPDATEQBVH3_0_HLSL
#define UPDATEQBVH3_0_HLSL

//=====================================================================================================================
// Generate FP32 box node from update scratch memory (bounds) and compressed HPB64 node (flags and child pointers)
Float32BoxNode ReadHighPrecisionBoxNodeForUpdate(
    uint currentNodeOffset,
    uint currentNodePointer,
    uint baseChildIdx)
{
    // Read child pointers from source buffer
    const uint4 childPointers = DecodeHighPrecisionBoxNodeChildPointers(currentNodeOffset);

    Float32BoxNode tempNode = (Float32BoxNode)0;
    tempNode.child0 = childPointers.x;
    tempNode.child1 = childPointers.y;
    tempNode.child2 = childPointers.z;
    tempNode.child3 = childPointers.w;

    // Read box node flags from destination buffer. Note, these have already been copied and modified by the Encode
    // phase
    tempNode.flags = UnpackHighPrecisionBoxNodeFlags(LoadBits_gc(DstMetadata,
                                                              currentNodeOffset,
                                                              HPB64_BOX_NODE_FLAGS_BIT_OFFSET,
                                                              HPB64_BOX_NODE_FLAGS_BIT_COUNT));

    tempNode.bbox0_min = float3(+FLT_MAX, +FLT_MAX, +FLT_MAX);
    tempNode.bbox0_max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    tempNode.bbox1_min = float3(+FLT_MAX, +FLT_MAX, +FLT_MAX);
    tempNode.bbox1_max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    tempNode.bbox2_min = float3(+FLT_MAX, +FLT_MAX, +FLT_MAX);
    tempNode.bbox2_max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    tempNode.bbox3_min = float3(+FLT_MAX, +FLT_MAX, +FLT_MAX);
    tempNode.bbox3_max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

#if GPURT_BUILD_RTIP3_1
    tempNode.obbMatrixIndex = INVALID_OBB;
#endif

    // Since sibling nodes are allocated in contiguous memory. The following series of loads
    // should result in contiguous indices and would fetch a single cache line for all 4 bounds
    // 24 * 4 = 96 bytes.
    if (tempNode.child0 != INVALID_NODE)
    {
        const BoundingBox bbox =
            ReadUpdateScratchBoundingBox(currentNodePointer, baseChildIdx + 0);

        tempNode.bbox0_min = bbox.min;
        tempNode.bbox0_max = bbox.max;
    }

    if (tempNode.child1 != INVALID_NODE)
    {
        const BoundingBox bbox =
            ReadUpdateScratchBoundingBox(currentNodePointer, baseChildIdx + 1);

        tempNode.bbox1_min = bbox.min;
        tempNode.bbox1_max = bbox.max;
    }

    if (tempNode.child2 != INVALID_NODE)
    {
        const BoundingBox bbox =
            ReadUpdateScratchBoundingBox(currentNodePointer, baseChildIdx + 2);

        tempNode.bbox2_min = bbox.min;
        tempNode.bbox2_max = bbox.max;
    }

    if (tempNode.child3 != INVALID_NODE)
    {
        const BoundingBox bbox =
            ReadUpdateScratchBoundingBox(currentNodePointer, baseChildIdx + 3);

        tempNode.bbox3_min = bbox.min;
        tempNode.bbox3_max = bbox.max;
    }

    return tempNode;
}

//=====================================================================================================================
// Handle root node update
void UpdateHighPrecisionBoxNode(
    uint                     boxNodePointer,
    uint                     boxNodeOffset,
    inout_param(BoundingBox) bbox,
    inout_param(uint)        boxNodeFlags)
{
    const Float32BoxNode boxNode = ReadHighPrecisionBoxNodeForUpdate(boxNodeOffset, boxNodePointer, 0);
    HighPrecisionBoxNode hpBoxNode = EncodeHighPrecisionBoxNode(boxNode);
    DstMetadata.Store<HighPrecisionBoxNode>(boxNodeOffset, hpBoxNode);

    // Initialise bounds and node flags from fp32 node
    bbox = GenerateBoxNode32BoundingBox(boxNode);
    boxNodeFlags = ComputeMergedFloat32BoxNodeFlags(boxNode.flags);

    if (Settings.bvh8Enable)
    {
        // Merge second half of the BVH8 node
        const uint boxNodeOffset1 = boxNodeOffset + sizeof(HighPrecisionBoxNode);

        const Float32BoxNode boxNode1 = ReadHighPrecisionBoxNodeForUpdate(boxNodeOffset1, boxNodePointer, 4);
        HighPrecisionBoxNode hpBoxNode1 = EncodeHighPrecisionBoxNode(boxNode1);
        DstMetadata.Store<HighPrecisionBoxNode>(boxNodeOffset1, hpBoxNode1);

        const BoundingBox bbox1 = GenerateBoxNode32BoundingBox(boxNode1);
        bbox = CombineAABB(bbox, bbox1);

        boxNodeFlags &= ComputeMergedFloat32BoxNodeFlags(boxNode1.flags);
    }
}

//=====================================================================================================================
void UpdateFloat32BoxNode3_0(
    uint                     boxNodeOffset,
    inout_param(BoundingBox) bbox,
    inout_param(uint)        boxNodeFlags)
{
    const Float32BoxNode boxNode = FetchFloat32BoxNode(false, boxNodeOffset);

    // Initialise bounds and node flags from fp32 node
    bbox = GenerateBoxNode32BoundingBox(boxNode);
    boxNodeFlags = ComputeMergedFloat32BoxNodeFlags(boxNode.flags);

    if (Settings.bvh8Enable)
    {
        // Merge second half of the BVH8 node
        const uint boxNodeOffset1 = boxNodeOffset + sizeof(Float32BoxNode);
        const Float32BoxNode boxNode1 = FetchFloat32BoxNode(false, boxNodeOffset1);

        const BoundingBox bbox1 = GenerateBoxNode32BoundingBox(boxNode1);
        bbox = CombineAABB(bbox, bbox1);

        boxNodeFlags &= ComputeMergedFloat32BoxNodeFlags(boxNode1.flags);
    }
}

//=====================================================================================================================
// Generates the merged child bounding box and updates in parent node. This function assumes that the child node is
// up to date when this function is called.
//
void UpdateChildBoundingBox3_0(
    uint metadataSize,
    uint parentNodePointer,
    uint nodePointer,
    uint childIdx)
{
    BoundingBox bbox = (BoundingBox)0;
    uint        boxNodeFlags = 0;

    uint childNodeOffset  = metadataSize + ExtractNodePointerOffset(nodePointer);
    uint parentNodeOffset = metadataSize + ExtractNodePointerOffset(parentNodePointer);

    if (Settings.highPrecisionBoxNodeEnable)
    {
        UpdateHighPrecisionBoxNode(nodePointer, childNodeOffset, bbox, boxNodeFlags);

        WriteUpdateScratchBoundingBox(parentNodePointer, childIdx, bbox);

        // The instance flags can be changed in an update, so the node flags need to be updated.
        if (Settings.topLevelBuild != 0)
        {
            // Update parent node offset based on child index
            parentNodeOffset += (childIdx > 3) ? sizeof(HighPrecisionBoxNode) : 0;

            // Update child index in adjacent HighPrecisionBoxNode
            childIdx = childIdx % 4;

            UpdateHighPrecisionBoxNodeFlags(parentNodeOffset, childIdx, boxNodeFlags);
        }
    }
    else
    {
        UpdateFloat32BoxNode3_0(childNodeOffset, bbox, boxNodeFlags);

        if (Settings.bvh8Enable)
        {
            // Update parent node offset based on child index
            parentNodeOffset += (childIdx > 3) ? sizeof(Float32BoxNode) : 0;

            // Update child index in adjacent Float32BoxNode
            childIdx = childIdx % 4;

            const uint childBoundsOffset = FLOAT32_BOX_NODE_BB0_MIN_OFFSET + (childIdx * FLOAT32_BBOX_STRIDE);
            DstMetadata.Store<BoundingBox>(parentNodeOffset + childBoundsOffset, bbox);
        }
        else
        {
            if (IsBoxNode32(parentNodePointer))
            {
                const uint childBoundsOffset = FLOAT32_BOX_NODE_BB0_MIN_OFFSET + (childIdx * FLOAT32_BBOX_STRIDE);
                DstMetadata.Store<BoundingBox>(parentNodeOffset + childBoundsOffset, bbox);
            }
            else // Float16BoxNode
            {
                const uint  bbox16Offset = FLOAT16_BOX_NODE_BB0_OFFSET + (childIdx * FLOAT16_BBOX_STRIDE);
                const uint3 bbox16 = CompressBBoxToUint3(bbox);

                DstMetadata.Store<uint3>(parentNodeOffset + bbox16Offset, bbox16);
            }
        }
    }
}

//=====================================================================================================================
// Handle root node update
void UpdateRootNode3_0(
    uint metadataSize,
    uint rootNodePointer,
    uint numActivePrims)
{
    BoundingBox bbox         = (BoundingBox)0;
    uint        boxNodeFlags = 0;
    uint        instanceMask = 0;

    const uint boxNodeOffset = metadataSize + sizeof(AccelStructHeader);

    if (Settings.highPrecisionBoxNodeEnable)
    {
        UpdateHighPrecisionBoxNode(rootNodePointer, boxNodeOffset, bbox, boxNodeFlags);
    }
    else
    {
        UpdateFloat32BoxNode3_0(boxNodeOffset, bbox, boxNodeFlags);
    }

    DstMetadata.Store<BoundingBox>(metadataSize + ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET, bbox);

    const uint packedFlags = PackInstanceMaskAndNodeFlags(instanceMask, boxNodeFlags);
    DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_PACKED_FLAGS_OFFSET, packedFlags);
}

//=====================================================================================================================
void UpdateQBVHImpl3_0(
    uint                globalID,
    uint                numWorkItems,
    uint                numThreads)
{
    const AccelStructMetadataHeader metadata = SrcBuffer.Load<AccelStructMetadataHeader>(0);
    const uint metadataSize = metadata.sizeInBytes;
    const AccelStructHeader header = SrcBuffer.Load<AccelStructHeader>(metadataSize);

    // Root node is always fp32 regardless of mode for fp16 box nodes
    const uint rootNodePointer = CreateRootNodePointer();

    // Handle the header for not-in-place updates
    if (globalID == 0)
    {
        if (Settings.isUpdateInPlace == false)
        {
            AccelStructMetadataHeader resultMetadata = metadata;

            if ((Settings.topLevelBuild == 0) || (header.numActivePrims > 0))
            {
                uint64_t bvhVa =
                    PackUint64(ShaderConstants.resultBufferAddrLo, ShaderConstants.resultBufferAddrHi);

                bvhVa += metadataSize;

                resultMetadata.addressLo = LowPart(bvhVa);
                resultMetadata.addressHi = HighPart(bvhVa);
            }
            else
            {
                resultMetadata.addressLo = 0;
                resultMetadata.addressHi = 0;
            }

            DstMetadata.Store<AccelStructMetadataHeader>(0, resultMetadata);

            DstMetadata.Store<AccelStructHeader>(metadataSize, header);

            WriteParentPointer(metadataSize,
                               rootNodePointer,
                               INVALID_IDX);
        }
    }

    if (globalID >= numWorkItems)
    {
        return;
    }

    // initialise node pointer from stack
    uint offset = GetUpdateStackOffset(globalID);
    uint nodePointer = ScratchBuffer.Load(offset);

    // The last child of the root node updates the root bounding box in the header
    if (nodePointer == rootNodePointer)
    {
        UpdateRootNode3_0(metadataSize, rootNodePointer, header.numActivePrims);
    }

    // Choice to decode parent pointer into node index using fp16. Mixing interior node types
    // relies on fp16 node size chunks for decoding (only in BLAS)
    const bool decodeNodePtrAsFp16 = (Settings.topLevelBuild == 0) &&
                                     (Settings.fp16BoxNodesMode != NO_NODES_IN_BLAS_AS_FP16);

    // Build tree up until we hit root node
    while (1)
    {
        if (nodePointer == rootNodePointer)
        {
            // Pick the next node to process
            uint nextIndex;
            ScratchBuffer.InterlockedAdd(UPDATE_SCRATCH_REFIT_TASK_COUNT_OFFSET, 1, nextIndex);
            // Skip the initial set of nodes processed when the shader launches
            nextIndex += numThreads;

            if (nextIndex < numWorkItems)
            {
                offset = GetUpdateStackOffset(nextIndex);
                nodePointer = ScratchBuffer.Load(offset);
            }
            else
            {
                // Nothing left to do
                break;
            }
        }

        const uint parentNodePointer = ReadParentPointer(metadataSize, nodePointer);

        uint numValidChildren = 0;

        const uint childIdx =
            ComputeChildIndexAndValidBoxCount(metadataSize, parentNodePointer, nodePointer, numValidChildren);

        UpdateChildBoundingBox3_0(metadataSize, parentNodePointer, nodePointer, childIdx);

        if (Settings.isUpdateInPlace == false)
        {
            WriteParentPointer(metadataSize, nodePointer, parentNodePointer);
        }

        // Ensure the child bounding box write is done
        DeviceMemoryBarrier();

        uint originalFlagValue = SignalParentNode(ShaderConstants.offsets.propagationFlags,
                                                  parentNodePointer,
                                                  decodeNodePtrAsFp16);

        if (originalFlagValue < (numValidChildren - 1))
        {
            // Allocate new node on next iteration
            nodePointer = rootNodePointer;
            continue;
        }
        else
        {
            if (Settings.isUpdateInPlace == false)
            {
                CopyChildPointersAndFlags(parentNodePointer, metadataSize);
            }

            if ((Settings.topLevelBuild != 0) && (Settings.highPrecisionBoxNodeEnable == 0))
            {
                const uint parentNodeOffset = metadataSize + ExtractNodePointerOffset(parentNodePointer);

                UpdateNodeFlagsTopLevel(metadataSize, parentNodeOffset);

                if (Settings.bvh8Enable)
                {
                    // Update second half of Float32x2 box node for BVH8
                    UpdateNodeFlagsTopLevel(metadataSize, parentNodeOffset + sizeof(Float32BoxNode));
                }
            }

            // The last child of the root node updates the root bounding box in the header
            if (parentNodePointer == rootNodePointer)
            {
                UpdateRootNode3_0(metadataSize, rootNodePointer, header.numActivePrims);
            }
        }

        // Set parent node as next node index
        nodePointer = parentNodePointer;
    }
}
#endif
