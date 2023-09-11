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
#define RootSig "RootConstants(num32BitConstants=3, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL)"

//=====================================================================================================================
// 32 bit constants
struct InputArgs
{
    uint numThreads;
    uint addressLo; // top only
    uint addressHi;
};

[[vk::push_constant]] ConstantBuffer<InputArgs> ShaderConstants : register(b0);

//=====================================================================================================================
[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer    DstMetadata   : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer                     SrcBuffer     : register(u1);

// unused buffer
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer    DstBuffer     : register(u2);
[[vk::binding(3, 0)]] globallycoherent RWByteAddressBuffer    ScratchBuffer : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer                     EmitBuffer    : register(u4);

#include "Common.hlsl"
#include "BuildCommon.hlsl"
#include "CompactCommon.hlsl"

//=====================================================================================================================
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
        if (srcNodePointer == ExtractNodePointerCollapse(parentChildPointers[childIndex]))
        {
            // Increment primitive count to offset the decrement in PackLeafNodePointer
            const uint collapsePrimCount = ExtractPrimitiveCount(parentChildPointers[childIndex]) + 1;

            const uint dstPackedNodePointer = PackLeafNodePointer(GetNodeType(dstNodePointer),
                                                                  ExtractNodePointerOffset(dstNodePointer),
                                                                  collapsePrimCount);

            DstMetadata.Store(parentNodeOffset + dstMetadataSizeInBytes + (childIndex * sizeof(uint)),
                              dstPackedNodePointer);
            break;
        }
    }

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
    if ((node.child0 == INVALID_IDX) || IsBoxNode(node.child0))
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT16_BOX_NODE_CHILD0_OFFSET, node.child0);
    }

    if ((node.child1 == INVALID_IDX) || IsBoxNode(node.child1))
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT16_BOX_NODE_CHILD1_OFFSET, node.child1);
    }

    if ((node.child2 == INVALID_IDX) || IsBoxNode(node.child2))
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT16_BOX_NODE_CHILD2_OFFSET, node.child2);
    }

    if ((node.child3 == INVALID_IDX) || IsBoxNode(node.child3))
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

    if ((node.child0 == INVALID_IDX) || IsBoxNode(node.child0))
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_CHILD0_OFFSET, node.child0);
    }

    if ((node.child1 == INVALID_IDX) || IsBoxNode(node.child1))
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_CHILD1_OFFSET, node.child1);
    }

    if ((node.child2 == INVALID_IDX) || IsBoxNode(node.child2))
    {
        DstMetadata.Store(dstInternalNodeDataOffset + FLOAT32_BOX_NODE_CHILD2_OFFSET, node.child2);
    }

    if ((node.child3 == INVALID_IDX) || IsBoxNode(node.child3))
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
                uint childPointer = ExtractNodePointerCollapse(parentChildPointers[cId]);
                childPointer = ClearNodeType(childPointer);

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
// CompactAS
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void CompactAS(in uint3 globalThreadId : SV_DispatchThreadID)
{
    const uint globalId = globalThreadId.x;

    // The following code assumes the base address of SrcBuffer and DstMetadata includes the acceleration structure
    // metadata header

    // Fetch acceleration structure metadata size
    const uint32_t metadataSizeInBytes = SrcBuffer.Load<uint32_t>(ACCEL_STRUCT_METADATA_SIZE_OFFSET);

    const AccelStructHeader  srcHeader  = SrcBuffer.Load<AccelStructHeader>(metadataSizeInBytes);
    const AccelStructOffsets srcOffsets = srcHeader.offsets;

    uint srcMetadataSizeInBytes = srcHeader.metadataSizeInBytes;
    uint dstMetadataSizeInBytes = 0;

    const uint type = (srcHeader.info & ACCEL_STRUCT_HEADER_INFO_TYPE_MASK);

    const uint triangleCompressionMode =
        (srcHeader.info >> ACCEL_STRUCT_HEADER_INFO_TRI_COMPRESS_SHIFT) & ACCEL_STRUCT_HEADER_INFO_TRI_COMPRESS_MASK;

    const uint enableFusedInstanceNode =
            (srcHeader.info >> ACCEL_STRUCT_HEADER_INFO_FUSED_INSTANCE_NODE_FLAGS_SHIFT) &
                ACCEL_STRUCT_HEADER_INFO_FUSED_INSTANCE_NODE_FLAGS_MASK;

    AccelStructOffsets dstOffsets;
    const uint dstSizeInBytes = CalcCompactedSize(srcHeader,
                                                  type,
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
        dstHeader.sizeInBytes         = dstSizeInBytes;
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
            for (uint nodeIndex = globalId; nodeIndex < srcHeader.numLeafNodes; nodeIndex += ShaderConstants.numThreads)
            {
                const uint nodeOffset         = nodeIndex * GetBvhNodeSizeLeaf(PrimitiveType::Instance, enableFusedInstanceNode);
                const uint srcNodeDataOffset  = srcOffsetDataLeafNodes + nodeOffset;
                const uint dstNodeDataOffset  = dstOffsetDataLeafNodes + nodeOffset;

                // Copy instance node
                const InstanceNode node = SrcBuffer.Load<InstanceNode>(srcNodeDataOffset);
                DstMetadata.Store<InstanceNode>(dstNodeDataOffset, node);

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
        else if (srcHeader.geometryType == GEOMETRY_TYPE_TRIANGLES)
        {
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
    }

    // Copy primitive node pointers
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
                // Increment primitive count to offset the decrement in PackLeafNodePointer
                const uint collapsePrimCount = ExtractPrimitiveCount(srcNodePointer) + 1;
                const uint nodeOffset = (srcNodeOffset - srcOffsets.leafNodes);

                dstNodePointer = PackLeafNodePointer(srcNodeType, dstOffsets.leafNodes + nodeOffset, collapsePrimCount);
            }

            DstMetadata.Store(dstNodePtrOffset, dstNodePointer);
        }
    }
}
