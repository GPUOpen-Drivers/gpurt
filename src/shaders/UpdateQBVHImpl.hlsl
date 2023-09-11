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
// Signal parent node
uint SignalParentNode(
    uint baseFlagsOffset,
    uint parentNodePointer,
    uint decodeNodePtrAsFp16)
{
    const uint parentNodeOffset = ExtractNodePointerOffset(parentNodePointer);
    const uint parentNodeIndex = CalcQbvhInternalNodeIndex(parentNodeOffset, decodeNodePtrAsFp16);
    const uint flagOffset = baseFlagsOffset + (parentNodeIndex * sizeof(uint));

    uint originalFlagValue = 0;
    ScratchBuffer.InterlockedAdd(flagOffset, 1, originalFlagValue);

    return originalFlagValue;
}

//=====================================================================================================================
BoundingBox GenerateNodeBoundingBox(
    uint                metadataSize,
    uint                nodePointer)
{
    const uint nodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);

    BoundingBox bbox;

    {
        if (IsBoxNode16(nodePointer))
        {
            const Float16BoxNode node = DstMetadata.Load<Float16BoxNode>(nodeOffset);
            bbox = GenerateBoxNode16BoundingBox(node);
        }
        else
        {
            const Float32BoxNode node = DstMetadata.Load<Float32BoxNode>(nodeOffset);
            bbox = GenerateBoxNode32BoundingBox(node);
        }
    }

    return bbox;
}

//=====================================================================================================================
// Computes box node flags by merging individual child flags
uint ComputeMergedFloat32BoxNodeFlags(
    in uint flags)
{
    const uint flags0 = ExtractBoxNodeFlagsField(flags, 0);
    const uint flags1 = ExtractBoxNodeFlagsField(flags, 1);
    const uint flags2 = ExtractBoxNodeFlagsField(flags, 2);
    const uint flags3 = ExtractBoxNodeFlagsField(flags, 3);

    return (flags0 & flags1 & flags2 & flags3);
}

//=====================================================================================================================
void UpdateFloat32BoxNode(
    uint                     boxNodeOffset,
    inout_param(BoundingBox) bbox,
    inout_param(uint)        boxNodeFlags)
{
    const Float32BoxNode boxNode = FetchFloat32BoxNode(
                                                       boxNodeOffset);

    // Initialise bounds and node flags from fp32 node
    bbox = GenerateBoxNode32BoundingBox(boxNode);
    boxNodeFlags = ComputeMergedFloat32BoxNodeFlags(boxNode.flags);

}

//=====================================================================================================================
// Generates the merged child bounding box and updates in parent node. This function assumes that the child node is
// up to date when this function is called.
//
void UpdateChildBoundingBox(
    uint metadataSize,
    uint parentNodePointer,
    uint nodePointer,
    uint childIdx)
{
    BoundingBox bbox = (BoundingBox)0;
    uint        boxNodeFlags = 0;

    uint childNodeOffset  = metadataSize + ExtractNodePointerOffset(nodePointer);
    uint parentNodeOffset = metadataSize + ExtractNodePointerOffset(parentNodePointer);

    {
        UpdateFloat32BoxNode(childNodeOffset, bbox, boxNodeFlags);

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
void UpdateRootNode(
    uint metadataSize,
    uint rootNodePointer,
    uint numActivePrims)
{
    BoundingBox bbox         = (BoundingBox)0;
    uint        boxNodeFlags = 0;
    uint        instanceMask = 0;

    const uint boxNodeOffset = metadataSize + sizeof(AccelStructHeader);

    {
        UpdateFloat32BoxNode(boxNodeOffset, bbox, boxNodeFlags);
    }

    DstMetadata.Store<BoundingBox>(metadataSize + ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET, bbox);

    const uint packedFlags = PackInstanceMaskAndNodeFlags(instanceMask, boxNodeFlags);
    DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_PACKED_FLAGS_OFFSET, packedFlags);
}

//=====================================================================================================================
void UpdateNodeFlagsTopLevel(
    uint metadataSize,
    uint parentNodeOffset)
{
    // Read child pointers from source buffer
    const uint childPointers[4] = SrcBuffer.Load<uint[4]>(parentNodeOffset);

    // Read node flags from parent. Note, instance nodes update their relevant flag bits at EncodeTopLevel
    uint parentNodeFlags = DstMetadata.Load(parentNodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET);

    for (uint i = 0; i < 4; i++)
    {
        // Note this also returns false for INVALID_IDX
        if (IsBoxNode32(childPointers[i]))
        {
            const uint childOffset = metadataSize + ExtractNodePointerOffset(childPointers[i]);
            const uint childFlags = DstMetadata.Load(childOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET);

            const uint flags0 = ExtractBoxNodeFlagsField(childFlags, 0);
            const uint flags1 = ExtractBoxNodeFlagsField(childFlags, 1);
            const uint flags2 = ExtractBoxNodeFlagsField(childFlags, 2);
            const uint flags3 = ExtractBoxNodeFlagsField(childFlags, 3);

            const uint mergedNodeFlags = flags0 & flags1 & flags2 & flags3;

            parentNodeFlags = ClearBoxNodeFlagsField(parentNodeFlags, i);
            parentNodeFlags = SetBoxNodeFlagsField(parentNodeFlags, mergedNodeFlags, i);
        }
    }

    DstMetadata.Store(parentNodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET, parentNodeFlags);
}

//=====================================================================================================================
void UpdateQBVHImpl(
    uint                globalID,
    uint                baseFlagsOffset,
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
        if (ShaderConstants.isUpdateInPlace == false)
        {
            AccelStructMetadataHeader resultMetadata = metadata;

            if ((Settings.topLevelBuild == 0) || (header.numActivePrims > 0))
            {
                GpuVirtualAddress bvhVa = MakeGpuVirtualAddress(ShaderConstants.addressLo, ShaderConstants.addressHi);

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

            DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_VERSION_OFFSET,                 header.accelStructVersion);
            DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_INFO_OFFSET,                    header.info);
            DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_METADATA_SIZE_OFFSET,           header.metadataSizeInBytes);
            DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_BYTE_SIZE_OFFSET,               header.sizeInBytes);
            DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_NUM_PRIMS_OFFSET,               header.numPrimitives);
            DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET,        header.numActivePrims);
            DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_NUM_DESCS_OFFSET,               header.numDescs);
            DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET,           header.geometryType);
            DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET, header.numInternalNodesFp32);
            DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP16_NODES_OFFSET, header.numInternalNodesFp16);
            DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET,          header.numLeafNodes);
            DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_COMPACTED_BYTE_SIZE_OFFSET,     header.compactedSizeInBytes);

            DstMetadata.Store<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET, header.offsets);

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
    uint offset = GetUpdateStackOffset(ShaderConstants.baseUpdateStackScratchOffset, globalID);
    uint nodePointer = ScratchBuffer.Load(offset);

    // The last child of the root node updates the root bounding box in the header
    if (nodePointer == rootNodePointer)
    {
        UpdateRootNode(metadataSize, rootNodePointer, header.numActivePrims);
    }

    // Choice to decode parent pointer into node index using fp16. Mixing interior node types
    // relies on fp16 node size chunks for decoding (only in BLAS)
    const bool decodeNodePtrAsFp16 = (Settings.topLevelBuild == 0) &&
                                     (ShaderConstants.fp16BoxNodesInBlasMode != NO_NODES_IN_BLAS_AS_FP16);

    // Build tree up until we hit root node
    while (1)
    {
        if (nodePointer == rootNodePointer)
        {
            // Pick the next node to process
            uint nextIndex;
            ScratchBuffer.InterlockedAdd(UPDATE_SCRATCH_TASK_COUNT_OFFSET, 1, nextIndex);
            // Skip the initial set of nodes processed when the shader launches
            nextIndex += numThreads;

            if (nextIndex < numWorkItems)
            {
                offset = GetUpdateStackOffset(ShaderConstants.baseUpdateStackScratchOffset, nextIndex);
                nodePointer = ScratchBuffer.Load(offset);
            }
            else
            {
                // Nothing left to do
                break;
            }
        }

        const uint parentNodePointer = ReadParentPointer(metadataSize,
                                                         ExtractNodePointerCollapse(nodePointer));

        uint numValidChildren = 0;

        const uint childIdx =
            ComputeChildIndexAndValidBoxCount(metadataSize, parentNodePointer, nodePointer, numValidChildren);

        UpdateChildBoundingBox(metadataSize, parentNodePointer, nodePointer, childIdx);

        if (ShaderConstants.isUpdateInPlace == false)
        {
            WriteParentPointer(metadataSize,
                               ExtractNodePointerCollapse(nodePointer),
                               parentNodePointer);
        }

        // Ensure the child bounding box write is done
        DeviceMemoryBarrier();

        uint originalFlagValue = SignalParentNode(baseFlagsOffset,
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
            if (ShaderConstants.isUpdateInPlace == false)
            {
                CopyChildPointersAndFlags(parentNodePointer, metadataSize);
            }

            if (Settings.topLevelBuild != 0)
            {
                const uint parentNodeOffset = metadataSize + ExtractNodePointerOffset(parentNodePointer);

                UpdateNodeFlagsTopLevel(metadataSize, parentNodeOffset);

            }

            // The last child of the root node updates the root bounding box in the header
            if (parentNodePointer == rootNodePointer)
            {
                UpdateRootNode(metadataSize, rootNodePointer, header.numActivePrims);
            }
        }

        // Set parent node as next node index
        nodePointer = parentNodePointer;
    }
}
