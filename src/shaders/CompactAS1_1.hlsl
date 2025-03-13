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
uint UpdateParentPointer(
    uint srcMetadataSizeInBytes,
    uint srcNodePointer,
    uint dstMetadataSizeInBytes,
    uint dstNodePointer)
{
    uint parentNodePointer = ReadParentPointer(srcMetadataSizeInBytes,
                                               srcNodePointer);
    WriteParentPointer(dstMetadataSizeInBytes,
                       dstNodePointer,
                       parentNodePointer);

    return parentNodePointer;
}

//=====================================================================================================================
// Updates parent pointer in the metadata and the child pointer in the parent of a dst node.
void UpdateParentPointerAndChildPointer(
    uint srcMetadataSizeInBytes,
    uint srcNodePointer,
    uint dstMetadataSizeInBytes,
    uint dstNodePointer)
{
    // Update parent pointer
    uint parentNodePointer =
        UpdateParentPointer(srcMetadataSizeInBytes, srcNodePointer, dstMetadataSizeInBytes, dstNodePointer);

    const uint parentNodeOffset       = ExtractNodePointerOffset(parentNodePointer);
    const uint parentChildPointers[4] = SrcBuffer.Load<uint[4]>(parentNodeOffset + srcMetadataSizeInBytes);

    for (uint childIndex = 0; childIndex < 4; childIndex++)
    {
        if (srcNodePointer == parentChildPointers[childIndex])
        {
            DstMetadata.Store(parentNodeOffset + dstMetadataSizeInBytes + (childIndex * sizeof(uint)), dstNodePointer);
            break;
        }
    }

#if GPURT_BUILD_RTIP3
    const AccelStructHeader srcHeader = SrcBuffer.Load<AccelStructHeader>(srcMetadataSizeInBytes);
    const bool enableFloat32x2 = (srcHeader.UsesHighPrecisionBoxNode() == false) && srcHeader.UsesBVH8();

    if (enableFloat32x2)
    {
        const uint parentChildPointers2[4] =
            SrcBuffer.Load<uint[4]>(parentNodeOffset + sizeof(Float32BoxNode) + srcMetadataSizeInBytes);

        for (uint childIndex = 0; childIndex < 4; childIndex++)
        {
            if (srcNodePointer == parentChildPointers2[childIndex])
            {
                DstMetadata.Store(
                    parentNodeOffset + sizeof(Float32BoxNode) + dstMetadataSizeInBytes + (childIndex * sizeof(uint)),
                    dstNodePointer);
                break;
            }
        }
    }
#endif
}

//=====================================================================================================================
// Copies FP16 box node
void CopyFp16BoxNode(
    uint nodeOffset,
    uint srcOffsetDataInternalNodes,
    uint srcMetadataSizeInBytes,
    uint dstOffsetDataInternalNodes,
    uint dstMetadataSizeInBytes)
{
    const uint srcInternalNodeDataOffset = srcOffsetDataInternalNodes + nodeOffset;
    const uint dstInternalNodeDataOffset = dstOffsetDataInternalNodes + nodeOffset;

    const Float16BoxNode node = SrcBuffer.Load<Float16BoxNode>(srcInternalNodeDataOffset);

    // Skip leaf child pointers because they will be updated while copying leaf nodes
#if GPURT_BUILD_RTIP3_1
    if ((node.child0 == INVALID_IDX) || IsBoxNode(node.child0, false, false, false))
#elif GPURT_BUILD_RTIP3
    // HighPrecisionBoxNode can't be enabled the same time if FP16 is enabled.
    if ((node.child0 == INVALID_IDX) || IsBoxNode(node.child0, false, false))
#else
    if ((node.child0 == INVALID_IDX) || IsBoxNode(node.child0))
#endif
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT16_BOX_NODE_CHILD0_OFFSET, node.child0);
    }

#if GPURT_BUILD_RTIP3_1
    if ((node.child1 == INVALID_IDX) || IsBoxNode(node.child1, false, false, false))
#elif GPURT_BUILD_RTIP3
    // HighPrecisionBoxNode can't be enabled the same time if FP16 is enabled.
    if ((node.child1 == INVALID_IDX) || IsBoxNode(node.child1, false, false))
#else
    if ((node.child1 == INVALID_IDX) || IsBoxNode(node.child1))
#endif
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT16_BOX_NODE_CHILD1_OFFSET, node.child1);
    }

#if GPURT_BUILD_RTIP3_1
    if ((node.child2 == INVALID_IDX) || IsBoxNode(node.child2, false, false, false))
#elif GPURT_BUILD_RTIP3
    // HighPrecisionBoxNode can't be enabled the same time if FP16 is enabled.
    if ((node.child2 == INVALID_IDX) || IsBoxNode(node.child2, false, false))
#else
    if ((node.child2 == INVALID_IDX) || IsBoxNode(node.child2))
#endif
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT16_BOX_NODE_CHILD2_OFFSET, node.child2);
    }

#if GPURT_BUILD_RTIP3_1
    if ((node.child3 == INVALID_IDX) || IsBoxNode(node.child3, false, false, false))
#elif GPURT_BUILD_RTIP3
    // HighPrecisionBoxNode can't be enabled the same time if FP16 is enabled.
    if ((node.child3 == INVALID_IDX) || IsBoxNode(node.child3, false, false))
#else
    if ((node.child3 == INVALID_IDX) || IsBoxNode(node.child3))
#endif
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT16_BOX_NODE_CHILD3_OFFSET, node.child3);
    }

    DstMetadata.Store<uint3>(dstInternalNodeDataOffset + FLOAT16_BOX_NODE_BB0_OFFSET, node.bbox0);
    DstMetadata.Store<uint3>(dstInternalNodeDataOffset + FLOAT16_BOX_NODE_BB1_OFFSET, node.bbox1);
    DstMetadata.Store<uint3>(dstInternalNodeDataOffset + FLOAT16_BOX_NODE_BB2_OFFSET, node.bbox2);
    DstMetadata.Store<uint3>(dstInternalNodeDataOffset + FLOAT16_BOX_NODE_BB3_OFFSET, node.bbox3);

    const uint srcNodePointer =
        PackNodePointer(NODE_TYPE_BOX_FLOAT16, srcInternalNodeDataOffset - srcMetadataSizeInBytes);
    const uint dstNodePointer =
        PackNodePointer(NODE_TYPE_BOX_FLOAT16, dstInternalNodeDataOffset - dstMetadataSizeInBytes);

    // Update parent pointer
    UpdateParentPointer(srcMetadataSizeInBytes, srcNodePointer, dstMetadataSizeInBytes, dstNodePointer);
}

//=====================================================================================================================
// Copies FP32 box node
void CopyFp32BoxNode(
    uint nodeOffset,
    uint srcOffsetDataInternalNodes,
    uint srcMetadataSizeInBytes,
    uint dstOffsetDataInternalNodes,
    uint dstMetadataSizeInBytes)
{
    const uint srcInternalNodeDataOffset = srcOffsetDataInternalNodes + nodeOffset;
    const uint dstInternalNodeDataOffset = dstOffsetDataInternalNodes + nodeOffset;

    const Float32BoxNode node = SrcBuffer.Load<Float32BoxNode>(srcInternalNodeDataOffset);

    // Skip leaf child pointers because they will be updated while copying leaf nodes

#if GPURT_BUILD_RTIP3_1
    if ((node.child0 == INVALID_IDX) || IsBoxNode(node.child0, false, Settings.bvh8Enable, false))
#elif GPURT_BUILD_RTIP3
    // CopyFp32BoxNode is not called when HighPrecisionBoxNode is enabled.
    if ((node.child0 == INVALID_IDX) || IsBoxNode(node.child0, false, Settings.bvh8Enable))
#else
    if ((node.child0 == INVALID_IDX) || IsBoxNode(node.child0))
#endif
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_CHILD0_OFFSET, node.child0);
    }

#if GPURT_BUILD_RTIP3_1
    if ((node.child1 == INVALID_IDX) || IsBoxNode(node.child1, false, Settings.bvh8Enable, false))
#elif GPURT_BUILD_RTIP3
    if ((node.child1 == INVALID_IDX) || IsBoxNode(node.child1, false, Settings.bvh8Enable))
#else
    if ((node.child1 == INVALID_IDX) || IsBoxNode(node.child1))
#endif
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_CHILD1_OFFSET, node.child1);
    }

#if GPURT_BUILD_RTIP3_1
    if ((node.child2 == INVALID_IDX) || IsBoxNode(node.child2, false, Settings.bvh8Enable, false))
#elif GPURT_BUILD_RTIP3
    if ((node.child2 == INVALID_IDX) || IsBoxNode(node.child2, false, Settings.bvh8Enable))
#else
    if ((node.child2 == INVALID_IDX) || IsBoxNode(node.child2))
#endif
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_CHILD2_OFFSET, node.child2);
    }

#if GPURT_BUILD_RTIP3_1
    if ((node.child3 == INVALID_IDX) || IsBoxNode(node.child3, false, Settings.bvh8Enable, false))
#elif GPURT_BUILD_RTIP3
    if ((node.child3 == INVALID_IDX) || IsBoxNode(node.child3, false, Settings.bvh8Enable))
#else
    if ((node.child3 == INVALID_IDX) || IsBoxNode(node.child3))
#endif
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_CHILD3_OFFSET, node.child3);
    }

    DstMetadata.Store<float3>(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_BB0_MIN_OFFSET, node.bbox0_min);
    DstMetadata.Store<float3>(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_BB0_MAX_OFFSET, node.bbox0_max);
    DstMetadata.Store<float3>(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_BB1_MIN_OFFSET, node.bbox1_min);
    DstMetadata.Store<float3>(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_BB1_MAX_OFFSET, node.bbox1_max);
    DstMetadata.Store<float3>(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_BB2_MIN_OFFSET, node.bbox2_min);
    DstMetadata.Store<float3>(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_BB2_MAX_OFFSET, node.bbox2_max);
    DstMetadata.Store<float3>(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_BB3_MIN_OFFSET, node.bbox3_min);
    DstMetadata.Store<float3>(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_BB3_MAX_OFFSET, node.bbox3_max);
    DstMetadata.Store(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET, node.flags);
    DstMetadata.Store(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_NUM_PRIM_OFFSET, node.numPrimitives);

    const uint srcNodePointer = PackNodePointer(NODE_TYPE_BOX_FLOAT32, srcInternalNodeDataOffset - srcMetadataSizeInBytes);
    const uint dstNodePointer = PackNodePointer(NODE_TYPE_BOX_FLOAT32, dstInternalNodeDataOffset - dstMetadataSizeInBytes);

    // Update parent pointer.
#if GPURT_BUILD_RTIP3
    // This reads and writes garbage for the second half of Float32x2 nodes, but that's okay.
#endif
    UpdateParentPointer(srcMetadataSizeInBytes, srcNodePointer, dstMetadataSizeInBytes, dstNodePointer);
}

//=====================================================================================================================
// Traverses up the tree from a leaf node, copying internal nodes
void CopyInteriorNodesTraversingUpwards(
    uint srcNodePointer,
    uint srcOffsetDataInternalNodes,
    uint srcMetadataSizeInBytes,
    uint dstOffsetDataInternalNodes,
    uint dstMetadataSizeInBytes)
{
    // Load parent triangle/procedural node's parent
    uint parentNodePointer = ReadParentPointer(srcMetadataSizeInBytes,
                                               srcNodePointer);

    // Traverse up the tree.
    // Input srcNodePointer includes just the leaf node offset.
    uint nodePointer = srcNodePointer;
    while (parentNodePointer != INVALID_IDX)
    {
        // Load the children pointers for the internal node
        const uint parentNodeOffset       = ExtractNodePointerOffset(parentNodePointer);
        const uint parentChildPointers[4] = SrcBuffer.Load<uint[4]>(parentNodeOffset + srcMetadataSizeInBytes);

        // Are we the first child of this node?
        // Need to iterate until the first valid index
        bool isFirstChild = false;
        for (uint cId = 0; cId < 4; ++cId)
        {
            // fetch address of the child, excluding upper 3 bits and type
            if (parentChildPointers[cId] != INVALID_IDX)
            {
                const uint childPointer = ClearNodeType(parentChildPointers[cId]);
                isFirstChild = (nodePointer == childPointer);
                break;
            }
        }

        // Only the first child copies parent and proceeds up the tree
        if (isFirstChild)
        {
            if (IsBoxNode16(parentNodePointer))
            {
                CopyFp16BoxNode(parentNodeOffset - sizeof(AccelStructHeader),
                                srcOffsetDataInternalNodes,
                                srcMetadataSizeInBytes,
                                dstOffsetDataInternalNodes,
                                dstMetadataSizeInBytes);
            }
            else
            {
                CopyFp32BoxNode(parentNodeOffset - sizeof(AccelStructHeader),
                                srcOffsetDataInternalNodes,
                                srcMetadataSizeInBytes,
                                dstOffsetDataInternalNodes,
                                dstMetadataSizeInBytes);
            }

            // Load the next parent pointer
            nodePointer       = ClearNodeType(parentNodePointer);
            parentNodePointer = ReadParentPointer(srcMetadataSizeInBytes,
                                                  parentNodePointer);
        }
        else
        {
            parentNodePointer = INVALID_IDX;
        }
    }
}

//=====================================================================================================================
void CompactASImpl1_1(
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

    const uint triangleCompressionMode =
        (srcHeader.info >> ACCEL_STRUCT_HEADER_INFO_TRI_COMPRESS_SHIFT) & ACCEL_STRUCT_HEADER_INFO_TRI_COMPRESS_MASK;

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

        // Acceleration structure header
        AccelStructHeader dstHeader   = srcHeader;
        dstHeader.metadataSizeInBytes = dstMetadataSizeInBytes;
        dstHeader.sizeInBytes         = srcHeader.compactedSizeInBytes;
        dstHeader.offsets             = dstOffsets;

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

#if GPURT_BUILD_RTIP3
    if (Settings.highPrecisionBoxNodeEnable)
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

        // Internal nodes and leaf nodes are in contiguous memory. Just BLT the data section containing internal
        // nodes and leaf nodes

        // Each thread copies 64-byte chunks on each iteration. The total size to copy is guaranteed to be a multiple
        // of 64-bytes since the minimum node size is 64-bytes.

        const uint numLeafNodes = srcHeader.numLeafNodes;

        const uint leafSize =
            ((type == TOP_LEVEL) ? GetBvhNodeSizeLeaf(PrimitiveType::Instance, Settings.enableFusedInstanceNode) :
                ((srcHeader.geometryType == GEOMETRY_TYPE_TRIANGLES) ? sizeof(TriangleNode) : sizeof(ProceduralNode)));

        const uint boxNodeSize =
            Settings.highPrecisionBoxNodeEnable ? sizeof(HighPrecisionBoxNode) : sizeof(Float32BoxNode);

        const uint sizeInBytes = (srcHeader.numInternalNodesFp32 * boxNodeSize) +
                                 (srcHeader.numInternalNodesFp16 * sizeof(Float16BoxNode)) +
                                 (numLeafNodes * leafSize);

        // Copy acceleration structure internal and leaf nodes
        const uint numChunks = sizeInBytes  / sizeof(uint4[4]);
        for (uint chunkIdx = globalId; chunkIdx < numChunks; chunkIdx += ShaderConstants.numThreads)
        {
            const uint chunkOffset = (chunkIdx * sizeof(uint4[4]));

            const uint4 payload[4] = SrcBuffer.Load<uint4[4]>(srcOffsetDataInternalNodes + chunkOffset);
            DstMetadata.Store<uint4[4]>(dstOffsetDataInternalNodes + chunkOffset, payload);
        }
    }
    else
#endif
    {
        const uint fp16BoxNodesInBlasMode =
            (srcHeader.info >> ACCEL_STRUCT_HEADER_INFO_FP16_BOXNODE_IN_BLAS_MODE_SHIFT) & ACCEL_STRUCT_HEADER_INFO_FP16_BOXNODE_IN_BLAS_MODE_MASK;

        // Copy internal nodes
        // 16-bit internal nodes only apply to BLAS
        if (type == BOTTOM_LEVEL)
        {
            if ((fp16BoxNodesInBlasMode == LEAF_NODES_IN_BLAS_AS_FP16) ||
                (fp16BoxNodesInBlasMode == MIXED_NODES_IN_BLAS_AS_FP16))
            {
                // Because interior box nodes are mixed (fp16 and fp32), each thread traverses up the tree from leaf nodes
                // following parent pointers, copying each parent. The iteration traversing up the tree continues
                // only for the thread coming from the 1st child of the parent.

                // NOTE: in case this method is slow, we can try other alternatives which use some extra memory:
                // 1. add sideband data which is a bitfield. Each bit corresponds to an interior node and would store
                //    a flag whether the node is fp16 or fp32. Computing offset to a particular node that corresponds
                //    to a bit i requries counting bits [0...i-1] and multiplying by 64B.
                // 2. store atomic counters per node to count how many children were processed. The last thread
                //    continues up the tree. See UpdateQBVHImpl.hlsl
                // 3. traverse tree top-down and use a stack for the next node address to process. The stack passes
                //    indexes to threads. See BuildQBVH.hlsl

                // Iterate over number of leaf nodes. This is also required in the case of
                // tri-splitting since prim node pointer cannot point to multiple split triangle nodes.
                for (uint nodeIndex = globalId; nodeIndex < srcHeader.numLeafNodes; nodeIndex += ShaderConstants.numThreads)
                {
                    const uint primNodeSize      = (srcHeader.geometryType == GEOMETRY_TYPE_TRIANGLES) ?
                                                    sizeof(TriangleNode) :
                                                    sizeof(ProceduralNode);
                    const uint nodeOffset        = nodeIndex * primNodeSize;
                    const uint srcNodeDataOffset = srcOffsetDataLeafNodes + nodeOffset;

                    // Node type does not matter, so just use 0
                    const uint srcNodePointer    = PackNodePointer(0, srcOffsets.leafNodes + nodeOffset);

                    CopyInteriorNodesTraversingUpwards(srcNodePointer,
                                                       srcOffsetDataInternalNodes,
                                                       srcMetadataSizeInBytes,
                                                       dstOffsetDataInternalNodes,
                                                       dstMetadataSizeInBytes);
                }
            }
            else if (fp16BoxNodesInBlasMode == NO_NODES_IN_BLAS_AS_FP16)
            {
                for (uint nodeIndex = globalId; nodeIndex < srcHeader.numInternalNodesFp32; nodeIndex += ShaderConstants.numThreads)
                {
                    const uint nodeOffset = nodeIndex * sizeof(Float32BoxNode);

                    CopyFp32BoxNode(nodeOffset,
                                    srcOffsetDataInternalNodes,
                                    srcMetadataSizeInBytes,
                                    dstOffsetDataInternalNodes,
                                    dstMetadataSizeInBytes);
                }
            }
            else
            {
                // Write out the root node, which is fp32
                if (globalId == 0)
                {
                    CopyFp32BoxNode(0,
                                    srcOffsetDataInternalNodes,
                                    srcMetadataSizeInBytes,
                                    dstOffsetDataInternalNodes,
                                    dstMetadataSizeInBytes);
                }
                // Write out the rest as fp16
                for (uint nodeIndex = globalId; nodeIndex < srcHeader.numInternalNodesFp16; nodeIndex += ShaderConstants.numThreads)
                {
                    const uint nodeOffset = nodeIndex * sizeof(Float16BoxNode) + sizeof(Float32BoxNode);
                    CopyFp16BoxNode(nodeOffset,
                                    srcOffsetDataInternalNodes,
                                    srcMetadataSizeInBytes,
                                    dstOffsetDataInternalNodes,
                                    dstMetadataSizeInBytes);
                }
            }
        }
        else // TOP_LEVEL
        {
            for (uint nodeIndex = globalId; nodeIndex < srcHeader.numInternalNodesFp32; nodeIndex += ShaderConstants.numThreads)
            {
                const uint nodeOffset = nodeIndex * sizeof(Float32BoxNode);

                CopyFp32BoxNode(nodeOffset,
                                srcOffsetDataInternalNodes,
                                srcMetadataSizeInBytes,
                                dstOffsetDataInternalNodes,
                                dstMetadataSizeInBytes);
            }
        }

        // Copy leaf nodes
        if (type == TOP_LEVEL)
        {
            // Need to loop over all the prims, not just numLeafNodes.
            for (uint nodeIndex = globalId; nodeIndex < srcHeader.numPrimitives; nodeIndex += ShaderConstants.numThreads)
            {
                // Since there could be invalid instance nodes, we need to skip over them. Invalid instance nodes
                // will have corresponding prim node pointers as -1. So check for this and skip the node if invalid.
                // Note: We don't need to skip invalid nodes for BLASs because their leaf nodes will be packed one
                // after another, ie: no holes -> no invalid nodes.
                const uint primNodePtrOffset = srcOffsetDataPrimNodePtrs + (nodeIndex * NODE_PTR_SIZE);

                if (SrcBuffer.Load(primNodePtrOffset) != INVALID_IDX)
                {
                    const uint nodeOffset
                        = nodeIndex * GetBvhNodeSizeLeaf(PrimitiveType::Instance, Settings.enableFusedInstanceNode);
                    const uint srcNodeDataOffset = srcOffsetDataLeafNodes + nodeOffset;
                    const uint dstNodeDataOffset = dstOffsetDataLeafNodes + nodeOffset;

                    // Copy instance node
                    // Note, fused instance nodes are twice the size of normal instance nodes. We need to copy it correspondingly.
                    if (Settings.enableFusedInstanceNode)
                    {
                        const FusedInstanceNode node = SrcBuffer.Load<FusedInstanceNode>(srcNodeDataOffset);
                        DstMetadata.Store<FusedInstanceNode>(dstNodeDataOffset, node);
                    }
                    else
                    {
                        const InstanceNode node = SrcBuffer.Load<InstanceNode>(srcNodeDataOffset);
                        DstMetadata.Store<InstanceNode>(dstNodeDataOffset, node);
                    }

                    // Top level acceleration structures do not have geometry info.

                    const uint srcNodePointer = PackNodePointer(NODE_TYPE_USER_NODE_INSTANCE, srcOffsets.leafNodes + nodeOffset);
                    const uint dstNodePointer = PackNodePointer(NODE_TYPE_USER_NODE_INSTANCE, dstOffsets.leafNodes + nodeOffset);

                    // Update the parent pointer and fix up the child pointer in the parent node
                    UpdateParentPointerAndChildPointer(srcMetadataSizeInBytes,
                                                       srcNodePointer,
                                                       dstMetadataSizeInBytes,
                                                       dstNodePointer);
                }
            }
        }
        else if (srcHeader.geometryType == GEOMETRY_TYPE_TRIANGLES)
        {
            // Unlike TOP_LEVEL, this assumes that all leaf nodes are packed contiguously without any holes in between.
            for (uint nodeIndex = globalId; nodeIndex < srcHeader.numLeafNodes; nodeIndex += ShaderConstants.numThreads)
            {
                const uint nodeOffset         = (nodeIndex * sizeof(TriangleNode));
                const uint srcNodeDataOffset  = srcOffsetDataLeafNodes + nodeOffset;
                const uint dstNodeDataOffset  = dstOffsetDataLeafNodes + nodeOffset;

                // Copy triangle node data
                const TriangleNode node = SrcBuffer.Load<TriangleNode>(srcNodeDataOffset);
                DstMetadata.Store<TriangleNode>(dstNodeDataOffset, node);

                // Handle per-primitive data
                if (triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION)
                {
                    uint nodeType = NODE_TYPE_TRIANGLE_0;
                    if (((node.triangleId >> (NODE_TYPE_TRIANGLE_1 * TRIANGLE_ID_BIT_STRIDE)) & 0xf) != 0)
                    {
                        nodeType = NODE_TYPE_TRIANGLE_1;
                    }

                    const uint srcNodePointer = PackNodePointer(nodeType, srcOffsets.leafNodes + nodeOffset);
                    const uint dstNodePointer = PackNodePointer(nodeType, dstOffsets.leafNodes + nodeOffset);

                    // Update the parent pointer and fix up the child pointer in the parent node
                    UpdateParentPointerAndChildPointer(srcMetadataSizeInBytes,
                                                       srcNodePointer,
                                                       dstMetadataSizeInBytes,
                                                       dstNodePointer);
                }
                else
                {
                    for (uint nodeType = 0; nodeType < 4; nodeType++)
                    {
                        if (((node.triangleId >> (nodeType * TRIANGLE_ID_BIT_STRIDE)) & 0xf) != 0)
                        {
                            const uint srcNodePointer = PackNodePointer(nodeType, srcOffsets.leafNodes + nodeOffset);
                            const uint dstNodePointer = PackNodePointer(nodeType, dstOffsets.leafNodes + nodeOffset);

                            // Update the parent pointer and fix up the child pointer in the parent node
                            UpdateParentPointerAndChildPointer(srcMetadataSizeInBytes,
                                                               srcNodePointer,
                                                               dstMetadataSizeInBytes,
                                                               dstNodePointer);
                        }
                    }
                }
            }
        }
        else // GEOMETRY_TYPE_AABBS
        {
            for (uint nodeIndex = globalId; nodeIndex < srcHeader.numLeafNodes; nodeIndex += ShaderConstants.numThreads)
            {
                const uint nodeOffset        = nodeIndex * sizeof(ProceduralNode);
                const uint srcNodeDataOffset = srcOffsetDataLeafNodes + nodeOffset;
                const uint dstNodeDataOffset = dstOffsetDataLeafNodes + nodeOffset;

                // Copy procedural node
                const ProceduralNode node = SrcBuffer.Load<ProceduralNode>(srcNodeDataOffset);
                DstMetadata.Store<ProceduralNode>(dstNodeDataOffset, node);

                const uint srcNodePointer = PackNodePointer(NODE_TYPE_USER_NODE_PROCEDURAL, srcOffsets.leafNodes + nodeOffset);
                const uint dstNodePointer = PackNodePointer(NODE_TYPE_USER_NODE_PROCEDURAL, dstOffsets.leafNodes + nodeOffset);

                // Update the parent pointer and fix up the child pointer in the parent node
                UpdateParentPointerAndChildPointer(srcMetadataSizeInBytes,
                                                   srcNodePointer,
                                                   dstMetadataSizeInBytes,
                                                   dstNodePointer);
            }
        }
    }

    if (type == BOTTOM_LEVEL)
    {
        // Copy the geometry info
        for (uint geometryIndex = globalId; geometryIndex < srcHeader.numDescs; geometryIndex += ShaderConstants.numThreads)
        {
            const uint srcGeometryInfoOffset = srcOffsetDataGeometryInfo + (geometryIndex * sizeof(GeometryInfo));
            const uint dstGeometryInfoOffset = dstOffsetDataGeometryInfo + (geometryIndex * sizeof(GeometryInfo));

            const GeometryInfo geometryInfo = SrcBuffer.Load<GeometryInfo>(srcGeometryInfoOffset);
            DstMetadata.Store<GeometryInfo>(dstGeometryInfoOffset, geometryInfo);
        }

#if GPURT_BUILD_RTIP3_1
        // Copy KDOP metadata
        for (uint i = globalId; i < KDOP_PLANE_COUNT; i += ShaderConstants.numThreads)
        {
            uint offset = ACCEL_STRUCT_METADATA_KDOP_OFFSET + (i * sizeof(uint2));
            const uint2 data = SrcBuffer.Load2(offset);
            DstMetadata.Store2(offset, data);
        }
#endif
    }

    // Copy primitive node pointers
#if GPURT_BUILD_RTIP3
    if (Settings.highPrecisionBoxNodeEnable)
    {
        for (uint primIndex = globalId; primIndex < srcHeader.numPrimitives; primIndex += ShaderConstants.numThreads)
        {
            const uint srcNodePtrOffset = srcOffsetDataPrimNodePtrs + (primIndex * NODE_PTR_SIZE);
            const uint dstNodePtrOffset = dstOffsetDataPrimNodePtrs + (primIndex * NODE_PTR_SIZE);

            const uint srcNodePointer = SrcBuffer.Load(srcNodePtrOffset);
            DstMetadata.Store(dstNodePtrOffset, srcNodePointer);
        }
    }
    else
#endif
    {
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
    }
}
