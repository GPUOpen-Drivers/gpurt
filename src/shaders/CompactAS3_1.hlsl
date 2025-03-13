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
// Copies Quantized BVH8 box node
void CopyQuantizedBVH8BoxNode(
    uint nodeOffset,
    uint srcOffsetInternalNodes,
    uint srcOffsetLeafNodes,
    uint srcMetadataSizeInBytes,
    uint dstOffsetInternalNodes,
    uint dstOffsetLeafNodes,
    uint dstMetadataSizeInBytes)
{
    const uint srcInternalNodeDataOffset = srcOffsetInternalNodes + nodeOffset;
    const uint dstInternalNodeDataOffset = dstOffsetInternalNodes + nodeOffset;

    QuantizedBVH8BoxNode node =
        SrcBuffer.Load<QuantizedBVH8BoxNode>(srcMetadataSizeInBytes + srcInternalNodeDataOffset);

    // Adjust the leaf child base offset
    node.leafNodeBaseOffset -= ((srcOffsetLeafNodes - dstOffsetLeafNodes) >> 3);

    DstMetadata.Store<QuantizedBVH8BoxNode>(dstMetadataSizeInBytes + dstInternalNodeDataOffset, node);
}

//=====================================================================================================================
// Copies fat leaf offsets
void CopyFatLeafOffsets(
    uint globalId,
    uint numFatLeafOffsets,
    uint numPrimitives,
    uint srcOffsetDataPrimNodePtrs,
    uint dstOffsetDataPrimNodePtrs)
{
    const uint srcFatLeafOffsetsBase = srcOffsetDataPrimNodePtrs + (numPrimitives * NODE_PTR_SIZE);
    const uint dstFatLeafOffsetsBase = dstOffsetDataPrimNodePtrs + (numPrimitives * NODE_PTR_SIZE);

    for (uint i = globalId; i < numFatLeafOffsets; i += ShaderConstants.numThreads)
    {
        const uint srcOffset = srcFatLeafOffsetsBase + (i * NODE_PTR_SIZE);
        const uint dstOffset = dstFatLeafOffsetsBase + (i * NODE_PTR_SIZE);

        const uint fatLeafOffset = SrcBuffer.Load(srcOffset);
        DstMetadata.Store(dstOffset, fatLeafOffset);
    }
}

//=====================================================================================================================
void CompactASImpl3_1(
    uint globalId)
{
    // The following code assumes the base address of SrcBuffer and DstMetadata includes the acceleration structure
    // metadata header

    // Fetch acceleration structure metadata size
    const uint32_t metadataSizeInBytes = SrcBuffer.Load<uint32_t>(ACCEL_STRUCT_METADATA_SIZE_OFFSET);
    const AccelStructHeader srcHeader = SrcBuffer.Load<AccelStructHeader>(metadataSizeInBytes);
    const AccelStructOffsets srcOffsets = srcHeader.offsets;

    uint srcMetadataSizeInBytes = srcHeader.metadataSizeInBytes;
    uint dstMetadataSizeInBytes = 0;

    const uint type = (srcHeader.info & ACCEL_STRUCT_HEADER_INFO_TYPE_MASK);

    INIT_VAR(AccelStructOffsets, dstOffsets);
    const uint dstSizeInBytes = CalcCompactedSize(srcHeader,
                                                  (type == TOP_LEVEL),
                                                  dstOffsets,
                                                  dstMetadataSizeInBytes);

    // Write the destination headers
    if (globalId == 0)
    {
        uint64_t dstAccelStructAddr = MakeGpuVirtualAddress(ShaderConstants.addressLo, ShaderConstants.addressHi);
        dstAccelStructAddr += dstMetadataSizeInBytes;

        DstMetadata.Store<uint64_t>(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, dstAccelStructAddr);
        DstMetadata.Store<uint32_t>(ACCEL_STRUCT_METADATA_SIZE_OFFSET, dstMetadataSizeInBytes);
        DstMetadata.Store<uint32_t>(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET, 0);
        DstMetadata.Store<uint32_t>(ACCEL_STRUCT_METADATA_NUM_TASKS_DONE_OFFSET, 0);

        // Copy intersectable instance node data
        const uint4 payload[4] = SrcBuffer.Load<uint4[4]>(ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET);
        DstMetadata.Store<uint4[4]>(ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET, payload);

        // Copy indirect dispatch group count for updates.
        const uint3 updateGroupCount = SrcBuffer.Load3(ACCEL_STRUCT_METADATA_UPDATE_GROUP_COUNT_OFFSET);
        DstMetadata.Store3(ACCEL_STRUCT_METADATA_UPDATE_GROUP_COUNT_OFFSET, updateGroupCount);

        // Acceleration structure header
        AccelStructHeader dstHeader   = srcHeader;
        dstHeader.metadataSizeInBytes = dstMetadataSizeInBytes;
        dstHeader.sizeInBytes         = srcHeader.compactedSizeInBytes;
        dstHeader.offsets             = dstOffsets;

        if (srcHeader.obbBlasMetadataOffset != 0)
        {
            dstHeader.obbBlasMetadataOffset = dstOffsets.primNodePtrs + (srcHeader.numPrimitives * NODE_PTR_SIZE);
        }

        // RayTracing structure build info
        dstHeader.info2 |= (1 << ACCEL_STRUCT_HEADER_INFO_2_BVH_COMPACTION_FLAGS_SHIFT);

        DstMetadata.Store<AccelStructHeader>(dstMetadataSizeInBytes, dstHeader);
    }

    // Add metadata size to get to absolute data offsets in source/destination memory
    const uint srcOffsetDataInternalNodes = srcOffsets.internalNodes + srcMetadataSizeInBytes;
    const uint srcOffsetDataLeafNodes     = srcOffsets.leafNodes     + srcMetadataSizeInBytes;
    const uint srcOffsetDataGeometryInfo  = srcOffsets.geometryInfo  + srcMetadataSizeInBytes;
    const uint srcOffsetDataPrimNodePtrs  = srcOffsets.primNodePtrs  + srcMetadataSizeInBytes;

    const uint dstOffsetDataInternalNodes = dstOffsets.internalNodes + dstMetadataSizeInBytes;
    const uint dstOffsetDataLeafNodes     = dstOffsets.leafNodes     + dstMetadataSizeInBytes;
    const uint dstOffsetDataGeometryInfo  = dstOffsets.geometryInfo  + dstMetadataSizeInBytes;
    const uint dstOffsetDataPrimNodePtrs  = dstOffsets.primNodePtrs  + dstMetadataSizeInBytes;

    if (type == TOP_LEVEL)
    {
        // Copy internal nodes
        for (uint nodeIndex = globalId; nodeIndex < srcHeader.numInternalNodesFp32; nodeIndex += ShaderConstants.numThreads)
        {
            const uint nodeOffset = nodeIndex * sizeof(QuantizedBVH8BoxNode);
            CopyQuantizedBVH8BoxNode(nodeOffset,
                                     srcOffsets.internalNodes,
                                     srcOffsets.leafNodes,
                                     srcMetadataSizeInBytes,
                                     dstOffsets.internalNodes,
                                     dstOffsets.leafNodes,
                                     dstMetadataSizeInBytes);
        }

        for (uint nodeIndex = globalId; nodeIndex < srcHeader.numLeafNodes; nodeIndex += ShaderConstants.numThreads)
        {
            const uint nodeOffset         = nodeIndex * sizeof(HwInstanceNode);
            const uint srcNodeDataOffset  = srcOffsetDataLeafNodes + nodeOffset;
            const uint dstNodeDataOffset  = dstOffsetDataLeafNodes + nodeOffset;

            // Copy instance node
            const HwInstanceNode node = SrcBuffer.Load<HwInstanceNode>(srcNodeDataOffset);
            DstMetadata.Store<HwInstanceNode>(dstNodeDataOffset, node);
        }

        // Copy instance sideband data
        for (uint nodeIndex = globalId; nodeIndex < srcHeader.numLeafNodes; nodeIndex += ShaderConstants.numThreads)
        {
            const uint srcInstanceSidebandOffset =
                srcOffsetDataGeometryInfo + (nodeIndex * sizeof(InstanceSidebandData));
            const uint dstInstanceSidebandOffset =
                dstOffsetDataGeometryInfo + (nodeIndex * sizeof(InstanceSidebandData));

            const InstanceSidebandData sideband = SrcBuffer.Load<InstanceSidebandData>(srcInstanceSidebandOffset);
            DstMetadata.Store<InstanceSidebandData>(dstInstanceSidebandOffset, sideband);
        }

        // Copy primitive node pointers
        for (uint primIndex = globalId; primIndex < srcHeader.numPrimitives; primIndex += ShaderConstants.numThreads)
        {
            const uint srcNodePtrOffset = srcOffsetDataPrimNodePtrs + (primIndex * NODE_PTR_SIZE);
            const uint dstNodePtrOffset = dstOffsetDataPrimNodePtrs + (primIndex * NODE_PTR_SIZE);

            const uint srcNodePointer = SrcBuffer.Load(srcNodePtrOffset);

            uint dstNodePointer = INVALID_IDX;
            if (srcNodePointer != INVALID_IDX)
            {
                const uint srcNodeType = GetNodeType(srcNodePointer);
                const uint srcNodeOffset = ExtractNodePointerOffset(srcNodePointer);
                const uint nodeOffset = (srcNodeOffset - srcOffsets.leafNodes);

                dstNodePointer = PackNodePointer(srcNodeType, dstOffsets.leafNodes + nodeOffset);
            }

            DstMetadata.Store(dstNodePtrOffset, dstNodePointer);
        }

        CopyFatLeafOffsets(globalId,
                           srcHeader.numInternalNodesFp16,
                           srcHeader.numPrimitives,
                           srcOffsetDataPrimNodePtrs,
                           dstOffsetDataPrimNodePtrs);
    }
    else
    {
        // Note, both source and destination metadata section starts right before the acceleration structure
        // header. The metadata size includes the metadata header (written by globalId=0 above) and is always 128-bytes
        // aligned. As a result the size to copy must exclude the header and not overwrite it
        const uint dstMetadataSize = dstMetadataSizeInBytes - sizeof(AccelStructMetadataHeader);

        // Note, we need to copy metadata chunks separate than the node data because the acceleration structure
        // header is sandwiched between them. We could possibly do a single loop copy of the entire data followed
        // by updating the header separately, but that requires synchronisation across workgroups. It is much
        // simpler to do this and probably no worse performance as the other solution.
        //

        // Compute compacted metadata chunks to copy
        const uint numValidMetadataChunks = dstMetadataSize / sizeof(uint);

        // Note, we cannot do larger than 4 byte chunks because the metadata size aligned to 128-byte includes
        // the header which has already been written and we have to avoid overwriting it here.

        // We copy in reverse order starting from the end of the metadata section to avoid copying the unused space
        for (uint chunkIdx = globalId; chunkIdx < numValidMetadataChunks; chunkIdx += ShaderConstants.numThreads)
        {
            // +1 because we are copying in reverse, so chunk offsets would be [-4, -8, -16, ...]
            const uint chunkOffset = ((chunkIdx + 1) * sizeof(uint));

            const uint payload = SrcBuffer.Load(srcMetadataSizeInBytes - chunkOffset);
            DstMetadata.Store(dstMetadataSizeInBytes - chunkOffset, payload);
        }

        // Copy KDOP metadata
        for (uint i = globalId; i < KDOP_PLANE_COUNT; i += ShaderConstants.numThreads)
        {
            uint offset = ACCEL_STRUCT_METADATA_KDOP_OFFSET + (i * sizeof(uint2));
            const uint2 data = SrcBuffer.Load2(offset);
            DstMetadata.Store2(offset, data);
        }

        // Internal nodes and leaf nodes are in contiguous memory. Just BLT the data section containing internal
        // nodes and leaf nodes

        // Each thread copies 64-byte chunks on each iteration. The total size to copy is guaranteed to be a multiple
        // of 64-bytes since the minimum node size is 64-bytes.

        const uint leafSize    = PRIMITIVE_STRUCT_SIZE_IN_BYTE;
        const uint boxNodeSize = sizeof(QuantizedBVH8BoxNode);

        uint numLeafNodes;
        if (srcHeader.UsesLegacyLeafCompression())
        {
            // In RTIP 3.1, legacy compression leaves gaps in the AS, so we take numActivePrims as worst case.
            numLeafNodes = srcHeader.numActivePrims;
        }
        else
        {
            numLeafNodes = srcHeader.numLeafNodes;
        }

        const uint sizeInBytes = (srcHeader.numInternalNodesFp32 * boxNodeSize) + (numLeafNodes * leafSize);

        // Copy acceleration structure internal and leaf nodes
        const uint numChunks = sizeInBytes  / sizeof(uint4[4]);
        for (uint chunkIdx = globalId; chunkIdx < numChunks; chunkIdx += ShaderConstants.numThreads)
        {
            const uint chunkOffset = (chunkIdx * sizeof(uint4[4]));

            const uint4 payload[4] = SrcBuffer.Load<uint4[4]>(srcOffsetDataInternalNodes + chunkOffset);
            DstMetadata.Store<uint4[4]>(dstOffsetDataInternalNodes + chunkOffset, payload);
        }

        // Copy the geometry info
        for (uint geometryIndex = globalId; geometryIndex < srcHeader.numDescs; geometryIndex += ShaderConstants.numThreads)
        {
            const uint srcGeometryInfoOffset = srcOffsetDataGeometryInfo + (geometryIndex * sizeof(GeometryInfo));
            const uint dstGeometryInfoOffset = dstOffsetDataGeometryInfo + (geometryIndex * sizeof(GeometryInfo));

            const GeometryInfo geometryInfo = SrcBuffer.Load<GeometryInfo>(srcGeometryInfoOffset);
            DstMetadata.Store<GeometryInfo>(dstGeometryInfoOffset, geometryInfo);
        }

        // Copy primitive node pointers
        for (uint primIndex = globalId; primIndex < srcHeader.numPrimitives; primIndex += ShaderConstants.numThreads)
        {
            const uint srcNodePtrOffset = srcOffsetDataPrimNodePtrs + (primIndex * NODE_PTR_SIZE);
            const uint dstNodePtrOffset = dstOffsetDataPrimNodePtrs + (primIndex * NODE_PTR_SIZE);

            const uint srcNodePointer = SrcBuffer.Load(srcNodePtrOffset);
            DstMetadata.Store(dstNodePtrOffset, srcNodePointer);
        }

        CopyFatLeafOffsets(globalId,
                           srcHeader.numInternalNodesFp16,
                           srcHeader.numPrimitives,
                           srcOffsetDataPrimNodePtrs,
                           dstOffsetDataPrimNodePtrs);

        // Copy the BLAS root node K-DOP if present
        if (srcHeader.obbBlasMetadataOffset != 0)
        {
            const uint obbSizeInDwords = BLAS_OBB_METADATA_SIZE / sizeof(uint);
            const uint srcBaseOffset   = srcOffsetDataPrimNodePtrs + (srcHeader.numPrimitives * NODE_PTR_SIZE);
            const uint dstBaseOffset   = dstOffsetDataPrimNodePtrs + (srcHeader.numPrimitives * NODE_PTR_SIZE);
            for (uint dataIndex = globalId; dataIndex < obbSizeInDwords; dataIndex += ShaderConstants.numThreads)
            {
                const uint srcOffset = srcBaseOffset + (dataIndex * sizeof(uint));
                const uint dstOffset = dstBaseOffset + (dataIndex * sizeof(uint));
                const uint data = SrcBuffer.Load<uint>(srcOffset);
                DstMetadata.Store<uint>(dstOffset, data);
            }
        }
    }
}
