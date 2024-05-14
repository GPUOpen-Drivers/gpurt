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
#ifndef UPDATEQBVH1_1_HLSL
#define UPDATEQBVH1_1_HLSL

//=====================================================================================================================
void UpdateFloat32BoxNode1_1(
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
void UpdateChildBoundingBox1_1(
    uint metadataSize,
    uint parentNodePointer,
    uint nodePointer,
    uint childIdx)
{
    BoundingBox bbox = (BoundingBox)0;
    uint        boxNodeFlags = 0;

    uint childNodeOffset  = metadataSize + ExtractNodePointerOffset(nodePointer);
    uint parentNodeOffset = metadataSize + ExtractNodePointerOffset(parentNodePointer);

    UpdateFloat32BoxNode1_1(childNodeOffset, bbox, boxNodeFlags);

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

//=====================================================================================================================
// Handle root node update
void UpdateRootNode1_1(
    uint metadataSize,
    uint rootNodePointer,
    uint numActivePrims)
{
    BoundingBox bbox         = (BoundingBox)0;
    uint        boxNodeFlags = 0;
    uint        instanceMask = 0;

    const uint boxNodeOffset = metadataSize + sizeof(AccelStructHeader);

    UpdateFloat32BoxNode1_1(boxNodeOffset, bbox, boxNodeFlags);

    DstMetadata.Store<BoundingBox>(metadataSize + ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET, bbox);

    const uint packedFlags = PackInstanceMaskAndNodeFlags(instanceMask, boxNodeFlags);
    DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_PACKED_FLAGS_OFFSET, packedFlags);
}

//=====================================================================================================================
void UpdateQBVHImpl1_1(
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
    uint offset = GetUpdateStackOffset(ShaderConstants.offsets.updateStack, globalID);
    uint nodePointer = ScratchBuffer.Load(offset);

    // The last child of the root node updates the root bounding box in the header
    if (nodePointer == rootNodePointer)
    {
        UpdateRootNode1_1(metadataSize, rootNodePointer, header.numActivePrims);
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
            ScratchBuffer.InterlockedAdd(UPDATE_SCRATCH_TASK_COUNT_OFFSET, 1, nextIndex);
            // Skip the initial set of nodes processed when the shader launches
            nextIndex += numThreads;

            if (nextIndex < numWorkItems)
            {
                offset = GetUpdateStackOffset(ShaderConstants.offsets.updateStack, nextIndex);
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

        UpdateChildBoundingBox1_1(metadataSize, parentNodePointer, nodePointer, childIdx);

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

            if (Settings.topLevelBuild != 0)
            {
                const uint parentNodeOffset = metadataSize + ExtractNodePointerOffset(parentNodePointer);

                UpdateNodeFlagsTopLevel(metadataSize, parentNodeOffset);
            }

            // The last child of the root node updates the root bounding box in the header
            if (parentNodePointer == rootNodePointer)
            {
                UpdateRootNode1_1(metadataSize, rootNodePointer, header.numActivePrims);
            }
        }

        // Set parent node as next node index
        nodePointer = parentNodePointer;
    }
}
#endif
