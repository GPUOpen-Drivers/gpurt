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
// The functions defined below require the following resources defined before including this header
//
// RWByteAddressBuffer DstBuffer
// RWByteAddressBuffer DstMetadata
// RWByteAddressBuffer ScratchBuffer
// RWByteAddressBuffer InstanceDescBuffer
//

//=====================================================================================================================
void SetQBVHStackCollapse(uint index, Task task, uint baseQbvhStackOffset)
{
    const uint stackOffset = baseQbvhStackOffset + (index * TASK_SIZE);

    ScratchBuffer.Store(stackOffset + TASK_LEAF_INDEX_OFFSET,                    task.leafIndex);
    ScratchBuffer.Store(stackOffset + TASK_NUM_PRIMS_OFFSET,                     task.numPrimitives);
    ScratchBuffer.Store(stackOffset + TASK_LAST_NODE_INDEX_OFFSET,               task.lastNodeIndex);
    ScratchBuffer.Store(stackOffset + TASK_PARENT_OF_COLLAPSE_NODE_INDEX_OFFSET, task.parentOfCollapseNodeIndex);
    ScratchBuffer.Store(stackOffset + TASK_NODE_DEST_INDEX,                      task.nodeDestIndex);
    DeviceMemoryBarrier();
    ScratchBuffer.Store(stackOffset + TASK_NODE_INDEX_OFFSET, task.nodeIndex);
}

//=====================================================================================================================
Task FetchStackTask(uint index, uint baseQbvhStackOffset)
{
    const uint stackOffset = baseQbvhStackOffset + (index * TASK_SIZE);

    return ScratchBuffer.Load<Task>(stackOffset);
}

//=====================================================================================================================
uint FetchStackTaskNodeIndex(uint index, uint baseQbvhStackOffset)
{
    const uint stackOffset = baseQbvhStackOffset + (index * TASK_SIZE);

    return ScratchBuffer.Load(stackOffset + TASK_NODE_INDEX_OFFSET);
}

//=====================================================================================================================
uint WritePrimitiveNodeCollapse(
    in BuildQbvhArgs      args,
    in ScratchNode        node,
    in uint               parentPtr,
    in AccelStructOffsets offsets,
    in Task               task)
{
    // Constants when interior FP16 box nodes are enabled
    const uint boxUsedNodeType     = GetNodeType(parentPtr);
    bool usedNodeTypeIsFp16 = false;
    {
        usedNodeTypeIsFp16 = (boxUsedNodeType == NODE_TYPE_BOX_FLOAT16);
    }
    const uint boxUsedChild0Offset = (usedNodeTypeIsFp16 ? FLOAT16_BOX_NODE_CHILD0_OFFSET
                                                         : FLOAT32_BOX_NODE_CHILD0_OFFSET);
    uint nodeType = GetNodeType(node.type);

    // Load geometry info
    const uint geometryInfoOffset = offsets.geometryInfo + (node.right_or_geometryIndex * sizeof(GeometryInfo));
    const GeometryInfo geometryInfo = DstBuffer.Load<GeometryInfo>(geometryInfoOffset);
    const uint geometryFlags = ExtractGeometryInfoFlags(geometryInfo.geometryFlagsAndNumPrimitives);
    const uint geometryIndexAndFlags = PackGeometryIndexAndFlags(node.right_or_geometryIndex, geometryFlags);
    const uint geometryPrimNodePtrsOffset = offsets.primNodePtrs + geometryInfo.primNodePtrsOffset;

    uint numLeafsDone;
    ScratchBuffer.InterlockedAdd(args.stackPtrsScratchOffset + STACK_PTRS_NUM_LEAFS_DONE_OFFSET, 1, numLeafsDone);

    const uint destLeafIndex = task.leafIndex;

    uint nodeOffset;

    const uint triangleId = node.type >> 3;

    if (nodeType == NODE_TYPE_USER_NODE_PROCEDURAL)
    {
        nodeOffset = offsets.leafNodes + (destLeafIndex * USER_NODE_PROCEDURAL_SIZE);

        DstBuffer.Store3(nodeOffset + USER_NODE_PROCEDURAL_MIN_OFFSET, asuint(node.bbox_min_or_v0));
        DstBuffer.Store3(nodeOffset + USER_NODE_PROCEDURAL_MAX_OFFSET, asuint(node.bbox_max_or_v1));
        {
            DstBuffer.Store(nodeOffset + USER_NODE_PROCEDURAL_PRIMITIVE_INDEX_OFFSET, node.left_or_primIndex_or_instIndex);
            DstBuffer.Store(nodeOffset + USER_NODE_PROCEDURAL_GEOMETRY_INDEX_AND_FLAGS_OFFSET, geometryIndexAndFlags);
        }
    }
    else
    {
        nodeOffset = offsets.leafNodes + (destLeafIndex * TRIANGLE_NODE_SIZE);

        DstBuffer.Store(nodeOffset + TRIANGLE_NODE_ID_OFFSET, triangleId);

        {
            DstBuffer.Store(nodeOffset + TRIANGLE_NODE_GEOMETRY_INDEX_AND_FLAGS_OFFSET, geometryIndexAndFlags);
            DstBuffer.Store(nodeOffset + TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET + (nodeType * 4),
                            node.left_or_primIndex_or_instIndex);
        }

        uint3 vertexOffsets;
        if (args.triangleCompressionMode != NO_TRIANGLE_COMPRESSION)
        {
            vertexOffsets = CalcTriangleCompressionVertexOffsets(nodeType, triangleId);
        }
        else
        {
            vertexOffsets = CalcTriangleVertexOffsets(nodeType);
        }

        DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V0_OFFSET + vertexOffsets.x, node.bbox_min_or_v0);
        DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V0_OFFSET + vertexOffsets.y, node.bbox_max_or_v1);
        DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V0_OFFSET + vertexOffsets.z, node.sah_or_v2_or_instBasePtr);

        // For PAIR_TRIANGLE_COMPRESSION, the other node is not linked in the BVH tree, so we need to find it and
        // store it as well if it exists.
        if ((args.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
            (node.splitBox_or_nodePointer != INVALID_IDX))
        {
            const ScratchNode otherNode = FetchScratchNode(args.scratchNodesScratchOffset,
                                                           node.splitBox_or_nodePointer);
            const uint otherNodeType = GetNodeType(otherNode.type);

            const float3 otherVerts[3] = { otherNode.bbox_min_or_v0,
                                           otherNode.bbox_max_or_v1,
                                           otherNode.sah_or_v2_or_instBasePtr };
            const uint3 otherVertexOffsets = CalcTriangleCompressionVertexOffsets(otherNodeType, triangleId);
            for (uint i = 0; i < 3; ++i)
            {
                // Since the other node will always be of type NODE_TYPE_TRIANGLE_0, it is sufficient to store only
                // the vertex that goes into v0. v1, v2, and v3 were already stored by NODE_TYPE_TRIANGLE_1 above.
                if (otherVertexOffsets[i] == 0)
                {
                    DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V0_OFFSET, otherVerts[i]);
                }
            }

            {
                DstBuffer.Store(nodeOffset + TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET + (otherNodeType * 4),
                                otherNode.left_or_primIndex_or_instIndex);
            }

            const uint otherPrimNodePointer = PackNodePointer(otherNodeType, nodeOffset);
            DstBuffer.Store(geometryPrimNodePtrsOffset + (otherNode.left_or_primIndex_or_instIndex * sizeof(uint)),
                            otherPrimNodePointer);
        }
    }

    const uint nodePointer = PackNodePointer(nodeType, nodeOffset);

    // The primitive node pointers need to include the number of primitives for updates to be able to properly
    // handle the collapsed triangle lists.
    const uint numPrims = (task.numPrimitives > 0) ? task.numPrimitives : 1;
    const uint primNodePointer = PackLeafNodePointer(nodeType, nodeOffset, numPrims);
    DstBuffer.Store(geometryPrimNodePtrsOffset + (node.left_or_primIndex_or_instIndex * sizeof(uint)),
                    primNodePointer);

    uint collapseParentNodeAddress = CalcQbvhInternalNodeOffset((task.parentOfCollapseNodeIndex >> 2), true);

    // if first in the collapse list
    if (task.numPrimitives > 0)
    {
        const uint offset = task.parentOfCollapseNodeIndex & 0x3;

        // update ptr to list from internal node
        DstBuffer.Store(collapseParentNodeAddress + boxUsedChild0Offset + offset * NODE_PTR_SIZE, primNodePointer);

        // parent pointer
        WriteParentPointer(args.metadataSizeInBytes,
                           nodePointer,
                           PackNodePointer(boxUsedNodeType, collapseParentNodeAddress));
    }
    else if (task.parentOfCollapseNodeIndex != INVALID_IDX) // not first in the collapse list
    {
        // parent pointer
        WriteParentPointer(args.metadataSizeInBytes,
                           nodePointer,
                           PackNodePointer(boxUsedNodeType, collapseParentNodeAddress));
    }
    else // no collapse
    {
        // parent pointer
        WriteParentPointer(args.metadataSizeInBytes,
                           nodePointer,
                           parentPtr);
    }

    return nodePointer;
}

//=====================================================================================================================
// Writes out child node bbox into child's parent node given the child index
void WriteQbvhInternalNodeBbox(
    in BuildQbvhArgs args,
    in ScratchNode   scratchNode,         ///< Node whose bbox to write out
    in uint          childIndex,          ///< Child index of this node relative to parent (0, 1, 2, 3)
    in uint          qbvhNodeAddr,        ///< Base address of the (parent) node being written
    in bool          writeAsFp16BoxNode)  ///< Flag whether to write this interior node as fp16
{
    const BoundingBox bbox = GetScratchNodeBoundingBoxTS(args, false, scratchNode);

    if (writeAsFp16BoxNode)
    {
        const uint3 bboxC         = CompressBBoxToUint3(bbox);
        const uint  f16BoundsSize = sizeof(uint3);
        const uint  boxOffset     = FLOAT16_BOX_NODE_BB0_OFFSET + (childIndex * f16BoundsSize);
        DstBuffer.Store3(qbvhNodeAddr + boxOffset, bboxC);
    }
    else
    {
        const uint f32BoundsSize = sizeof(float3) * 2;
        const uint minOffset     = FLOAT32_BOX_NODE_BB0_MIN_OFFSET + (childIndex * f32BoundsSize);
        const uint maxOffset     = minOffset + sizeof(float3);
        DstBuffer.Store3(qbvhNodeAddr + minOffset, asuint(bbox.min));
        DstBuffer.Store3(qbvhNodeAddr + maxOffset, asuint(bbox.max));
    }
}

//=====================================================================================================================
void WriteQbvhInternalNodeNumPrimitives(
    in ScratchNode scratchNode,             ///< Node whose bbox to write out
    in uint        qbvhNodeAddr,            ///< Base address of the (parent) node being written
    in bool        writeAsFp16BoxNode,      ///< Flag whether to write this interior node as fp16
    in bool        isLeaf)
{
    if (writeAsFp16BoxNode == false)
    {
        const uint numPrims = FetchScratchNodeNumPrimitives(scratchNode, isLeaf);
        DstBuffer.Store(qbvhNodeAddr + FLOAT32_BOX_NODE_NUM_PRIM_OFFSET, numPrims);
    }
}

//=====================================================================================================================
void WriteQbvhInternalNodeFlags(
    in uint nodeFlags,
    in uint qbvhNodeAddr,
    in bool writeAsFp16BoxNode)
{
    if (writeAsFp16BoxNode == false)
    {
        DstBuffer.Store(qbvhNodeAddr + FLOAT32_BOX_NODE_FLAGS_OFFSET, nodeFlags);
    }
}

//=====================================================================================================================
void BuildQbvhCollapseImpl(
    uint          globalId,
    uint          numActivePrims,
    BuildQbvhArgs args)
{
    // Each stack item stores data for writes to linear QBVH memory, indexed by stack index
    uint stackIndex = globalId;

    // Load acceleration structure header
    const AccelStructHeader header = DstBuffer.Load<AccelStructHeader>(0);

    const AccelStructOffsets offsets = header.offsets;

    const uint baseQbvhStackOffset = args.qbvhStackScratchOffset;

    bool isDone = false;

    const uint32_t rootNodeIndex = args.enableFastLBVH ? args.fastLBVHRootNodeIndex : 0;

    while (1)
    {
        DeviceMemoryBarrier();

        const uint numLeafsDone = ScratchBuffer.Load(args.stackPtrsScratchOffset + STACK_PTRS_NUM_LEAFS_DONE_OFFSET);

        // Check if we've processed all leaves or have internal nodes on the stack
        if ((numLeafsDone >= args.numPrimitives) ||
            (stackIndex >= CalcNumQBVHInternalNodes(args.numPrimitives)) ||
            isDone)
        {
            break;
        }

        uint4 intChildStackIdx = { 0, 0, 0, 0 };
        uint bvhNodeSrcIdx;

        bvhNodeSrcIdx = FetchStackTaskNodeIndex(stackIndex, baseQbvhStackOffset);

        // If we have a valid node on the stack, process node
        if (bvhNodeSrcIdx != INVALID_IDX)
        {
            // Fetch BVH2 node - it knows whether to be fp16 or fp32 interior node
            const ScratchNode node = FetchScratchNode(args.scratchNodesScratchOffset, bvhNodeSrcIdx);

            // Declare child nodes so they can be fetched at most once in this code block and reused later.
            // This can be done because the logic checking which grandchildren are valid is the same.
            // TODO: storing all 4 grandchildren seems to use 152 VGPRs, which TDRs on prototype
            ScratchNode c0;
            ScratchNode c1;

            // Data to write into scratch memory
            uint4 intChildNodeIdx = { INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX };     // node index of each child as stored by the parent
            uint4 intChildDstIdx  = { 0, 0, 0, 0 };                                             // pre-allocated locations for our children using their sizes
            uint  compChildInfo   = 0;

            // Fetch and count children, keeping track of their types
            if (IsLeafNode(bvhNodeSrcIdx, numActivePrims) == false)
            {
                // If this is an internal node, we need to fetch its child nodes
                c0 = FetchScratchNode(args.scratchNodesScratchOffset, node.left_or_primIndex_or_instIndex);
                c1 = FetchScratchNode(args.scratchNodesScratchOffset, node.right_or_geometryIndex);

                uint numIntChildren   = 0;
                uint childDstCount64B = 0;
                if (IsLeafNode(node.left_or_primIndex_or_instIndex, numActivePrims) == false)
                {
                    const ScratchNode c00 = FetchScratchNode(args.scratchNodesScratchOffset, c0.left_or_primIndex_or_instIndex);
                    const ScratchNode c01 = FetchScratchNode(args.scratchNodesScratchOffset, c0.right_or_geometryIndex);

                    const uint c00Type = GetNodeType(c00.type);
                    const uint c01Type = GetNodeType(c01.type);

                    intChildNodeIdx.x = c0.left_or_primIndex_or_instIndex;
                    intChildNodeIdx.y = c0.right_or_geometryIndex;

                    intChildDstIdx.x  = childDstCount64B;
                    compChildInfo |= (IsBoxNode(c00Type) ? 0x1 : 0);
                    {
                        childDstCount64B += (IsBoxNode32(c00Type) ? 2 : (IsBoxNode16(c00Type) ? 1 : 0));
                    }

                    intChildDstIdx.y  = childDstCount64B;
                    compChildInfo |= (IsBoxNode(c01Type) ? 0x2 : 0);
                    {
                        childDstCount64B += (IsBoxNode32(c01Type) ? 2 : (IsBoxNode16(c01Type) ? 1 : 0));
                    }
                }

                if (IsLeafNode(node.right_or_geometryIndex, numActivePrims) == false)
                {
                    const ScratchNode c10 = FetchScratchNode(args.scratchNodesScratchOffset, c1.left_or_primIndex_or_instIndex);
                    const ScratchNode c11 = FetchScratchNode(args.scratchNodesScratchOffset, c1.right_or_geometryIndex);

                    const uint c10Type = GetNodeType(c10.type);
                    const uint c11Type = GetNodeType(c11.type);

                    intChildNodeIdx.z = c1.left_or_primIndex_or_instIndex;
                    intChildNodeIdx.w = c1.right_or_geometryIndex;

                    intChildDstIdx.z  = childDstCount64B;
                    compChildInfo |= (IsBoxNode(c10Type) ? 0x4 : 0);
                    {
                        childDstCount64B += (IsBoxNode32(c10Type) ? 2 : (IsBoxNode16(c10Type) ? 1 : 0));
                    }

                    intChildDstIdx.w  = childDstCount64B;
                    compChildInfo |= (IsBoxNode(c11Type) ? 0x8 : 0);
                    {
                        childDstCount64B += (IsBoxNode32(c11Type) ? 2 : (IsBoxNode16(c11Type) ? 1 : 0));
                    }
                }

                // Fetch the stack index and destination for the 1st child, and then offset our indices
                const uint origStackIdx = AllocQBVHStackNumItems(args,
                                                                 false,
                                                                 compChildInfo,
                                                                 intChildNodeIdx,
                                                                 intChildDstIdx,
                                                                 childDstCount64B);

                intChildStackIdx = origStackIdx + uint4(0,
                                                        countbits(compChildInfo & 0x1),
                                                        countbits(compChildInfo & 0x3),
                                                        countbits(compChildInfo & 0x7));
            }

            Task task = FetchStackTask(stackIndex, baseQbvhStackOffset);

            // Fetch BVH2 node - it knows whether to be fp16 or fp32 interior node
            const uint qbvhNodeType = GetNodeType(node.type);
            bool writeAsFp16BoxNode = false;
            {
                writeAsFp16BoxNode = (qbvhNodeType == NODE_TYPE_BOX_FLOAT16);
            }

            // Fetch node destination offset
            const bool nodeTypesMixed = (args.fp16BoxNodesInBlasMode == LEAF_NODES_IN_BLAS_AS_FP16) ||
                                        (args.fp16BoxNodesInBlasMode == MIXED_NODES_IN_BLAS_AS_FP16);
            const uint bvhNodeDstIdx  = (nodeTypesMixed ? task.nodeDestIndex : GetDestIdx(args, false, stackIndex));

            // Each stack writes to linear QBVH memory indexed by stack index
            const uint qbvhNodeAddr = CalcQbvhInternalNodeOffset(bvhNodeDstIdx, true);
            const uint qbvhNodePtr  = PackNodePointer(qbvhNodeType, qbvhNodeAddr);

            // Box node child offsets based on whether to be written as fp16 and fp32 node type
            uint boxUsedChildOffset[4];
            boxUsedChildOffset[0] = (writeAsFp16BoxNode ? FLOAT16_BOX_NODE_CHILD0_OFFSET : FLOAT32_BOX_NODE_CHILD0_OFFSET);
            boxUsedChildOffset[1] = (writeAsFp16BoxNode ? FLOAT16_BOX_NODE_CHILD1_OFFSET : FLOAT32_BOX_NODE_CHILD1_OFFSET);
            boxUsedChildOffset[2] = (writeAsFp16BoxNode ? FLOAT16_BOX_NODE_CHILD2_OFFSET : FLOAT32_BOX_NODE_CHILD2_OFFSET);
            boxUsedChildOffset[3] = (writeAsFp16BoxNode ? FLOAT16_BOX_NODE_CHILD3_OFFSET : FLOAT32_BOX_NODE_CHILD3_OFFSET);

            // If it is a leaf node, copy its data into QBVH node
            if (IsLeafNode(bvhNodeSrcIdx, numActivePrims))
            {
                // This path is hit only when numActivePrims == 1 because that is the only case in which leaf node is
                // pushed onto the stack.
                // Generate fp32 box node with a leaf reference in child index 0 for single primitive acceleration structure
                // This code assumes that baseLeafOffset accounts for this interior node and the stackIndex is 0
                if (numActivePrims == 1)
                {
                    WriteQbvhInternalNodeBbox(args, node, 0, qbvhNodeAddr, false);

                    DstBuffer.Store(qbvhNodeAddr + FLOAT32_BOX_NODE_CHILD1_OFFSET, INVALID_IDX);
                    DstBuffer.Store(qbvhNodeAddr + FLOAT32_BOX_NODE_CHILD2_OFFSET, INVALID_IDX);
                    DstBuffer.Store(qbvhNodeAddr + FLOAT32_BOX_NODE_CHILD3_OFFSET, INVALID_IDX);
                    DstBuffer.Store(qbvhNodeAddr + FLOAT32_BOX_NODE_FLAGS_OFFSET, 0xFFFFFF00);
                    DstBuffer.Store(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET, 1);

                    task.leafIndex = 0;
                    task.numPrimitives = 0;
                    task.parentOfCollapseNodeIndex = INVALID_IDX;

                    const uint parentPtr = CreateRootNodePointer();
                    const uint ptr       = WritePrimitiveNodeCollapse(args, node, parentPtr, offsets, task);

                    DstBuffer.Store(qbvhNodeAddr + FLOAT32_BOX_NODE_CHILD0_OFFSET, ptr);

                    WriteQbvhInternalNodeNumPrimitives(node, qbvhNodeAddr, writeAsFp16BoxNode, true);

                    WriteQbvhInternalNodeFlags(ExtractScratchNodeFlags(node.flags_and_instanceMask),
                                               qbvhNodeAddr,
                                               writeAsFp16BoxNode);
                }

                isDone = true;
                continue;
            }
            else
            {
                const uint nodeTypeToAccum = (writeAsFp16BoxNode ? ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP16_NODES_OFFSET
                                                                 : ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET);
                DstBuffer.InterlockedAdd(nodeTypeToAccum, 1);
            }

            WriteQbvhInternalNodeNumPrimitives(node, qbvhNodeAddr, writeAsFp16BoxNode, false);

            if (args.flags & BUILD_FLAGS_COLLAPSE)
            {
                // decide whether to collapse node

                // hasn't collapsed yet
                if (task.parentOfCollapseNodeIndex == INVALID_IDX)
                {
                    // not the root
                    if (task.lastNodeIndex != INVALID_IDX)
                    {
                        const ScratchNode taskNode = FetchScratchNode(args.scratchNodesScratchOffset, task.nodeIndex);

                        // ScratchNode.numPrimitivesAndDoCollapse holds the numPrims << 1 | (doCollapse)
                        if (taskNode.numPrimitivesAndDoCollapse & 0x1)
                        {
                            task.numPrimitives             = taskNode.numPrimitivesAndDoCollapse >> 1;
                            task.parentOfCollapseNodeIndex = task.lastNodeIndex;
                        }
                    }
                }
            }
            // If this is an internal node, its child nodes (c0, c1) have been loaded
            uint nodeFlags = 0;

            // Fetch next level children if these are internal nodes and pull them into the QBVH
            if (IsLeafNode(node.left_or_primIndex_or_instIndex, numActivePrims) == false)
            {
                const ScratchNode c00 = FetchScratchNode(args.scratchNodesScratchOffset, c0.left_or_primIndex_or_instIndex);
                const ScratchNode c01 = FetchScratchNode(args.scratchNodesScratchOffset, c0.right_or_geometryIndex);

                uint leafIndex00 = task.leafIndex;
                uint leafIndex01 = task.leafIndex + (c00.numPrimitivesAndDoCollapse >> 1);

                uint numPrimitives = 0;
                uint parentOfCollapseNodeIndex = INVALID_IDX;

                if (task.parentOfCollapseNodeIndex == INVALID_IDX) //if not full node collapsing
                {
                    if (c0.numPrimitivesAndDoCollapse & 0x1) //partial node collapse
                    {
                        WriteQbvhInternalNodeBbox(args, c0, 0, qbvhNodeAddr, writeAsFp16BoxNode);

                        //child0 will point to the starting of the prim list
                        DstBuffer.Store(qbvhNodeAddr + boxUsedChildOffset[1], INVALID_IDX);
                        nodeFlags = ExtractScratchNodeFlags(node.flags_and_instanceMask);

                        numPrimitives = c0.numPrimitivesAndDoCollapse >> 1;

                        //bit-packing is for the index into the 4 child pointers
                        parentOfCollapseNodeIndex = (CalcQbvhInternalNodeIndex(qbvhNodeAddr, true) << 2) | 0;
                    }
                    else //no collapse
                    {
                        WriteQbvhInternalNodeBbox(args, c00, 0, qbvhNodeAddr, writeAsFp16BoxNode);
                        WriteQbvhInternalNodeBbox(args, c01, 1, qbvhNodeAddr, writeAsFp16BoxNode);

                        nodeFlags = ExtractScratchNodeFlags(node.flags_and_instanceMask);
                    }
                }
                else //full node collapse
                {
                    numPrimitives             = task.numPrimitives;
                    parentOfCollapseNodeIndex = task.parentOfCollapseNodeIndex;
                }

                Task task00;
                task00.numPrimitives             = numPrimitives;
                task00.parentOfCollapseNodeIndex = parentOfCollapseNodeIndex;
                //bit-packing is for the index into the 4 child pointers
                task00.lastNodeIndex             = (CalcQbvhInternalNodeIndex(qbvhNodeAddr, true) << 2) | 0;
                task00.leafIndex                 = leafIndex00;
                task00.nodeIndex                 = uint(c0.left_or_primIndex_or_instIndex);
                task00.nodeDestIndex             = intChildDstIdx.x;

                uint ptr0;
                if (IsLeafNode(c0.left_or_primIndex_or_instIndex, numActivePrims))
                {
                    ptr0 = WritePrimitiveNodeCollapse(args, c00, qbvhNodePtr, offsets, task00);
                }
                else
                {
                    const uint nodeType   = GetNodeType(c00.type);
                    const uint nodeOffset = CalcQbvhInternalNodeOffset(intChildDstIdx.x, true);
                    ptr0 = PackNodePointer(nodeType, nodeOffset);

                    if (parentOfCollapseNodeIndex == INVALID_IDX)
                    {
                        // parent
                        WriteParentPointer(args.metadataSizeInBytes,
                                           ptr0,
                                           qbvhNodePtr);
                    }

                    SetQBVHStackCollapse(intChildStackIdx.x, task00, baseQbvhStackOffset);
                }

                if (parentOfCollapseNodeIndex == INVALID_IDX)
                {
                    DstBuffer.Store(qbvhNodeAddr + boxUsedChildOffset[0], ptr0);
                }

                Task task01;
                task01.numPrimitives                = 0;
                task01.parentOfCollapseNodeIndex    = parentOfCollapseNodeIndex;
                //bit-packing is for the index into the 4 child pointers
                task01.lastNodeIndex                = (CalcQbvhInternalNodeIndex(qbvhNodeAddr, true) << 2) | 1;
                task01.leafIndex                    = leafIndex01;
                task01.nodeIndex                    = uint(c0.right_or_geometryIndex);
                task01.nodeDestIndex                = intChildDstIdx.y;

                uint ptr1;
                if (IsLeafNode(c0.right_or_geometryIndex, numActivePrims))
                {
                    ptr1 = WritePrimitiveNodeCollapse(args, c01, qbvhNodePtr, offsets, task01);
                }
                else
                {
                    const uint nodeType   = GetNodeType(c01.type);
                    const uint nodeOffset = CalcQbvhInternalNodeOffset(intChildDstIdx.y, true);
                    ptr1 = PackNodePointer(nodeType, nodeOffset);

                    if (parentOfCollapseNodeIndex == INVALID_IDX)
                    {
                        // parent
                        WriteParentPointer(args.metadataSizeInBytes,
                                           ptr1,
                                           qbvhNodePtr);
                    }

                    SetQBVHStackCollapse(intChildStackIdx.y, task01, baseQbvhStackOffset);
                }

                if (parentOfCollapseNodeIndex == INVALID_IDX)
                {
                    DstBuffer.Store(qbvhNodeAddr + boxUsedChildOffset[1], ptr1);
                }
            }
            else
            {
                uint ptr0;
                Task task00;
                task00.numPrimitives             = task.numPrimitives;
                task00.parentOfCollapseNodeIndex = task.parentOfCollapseNodeIndex;
                //bit-packing is for the index into the 4 child pointers
                task00.lastNodeIndex             = (CalcQbvhInternalNodeIndex(qbvhNodeAddr, true) << 2) | 0;
                task00.leafIndex                 = task.leafIndex;
                task00.nodeIndex                 = uint(node.left_or_primIndex_or_instIndex);

                ptr0 = WritePrimitiveNodeCollapse(args, c0, qbvhNodePtr, offsets, task00);

                if (task.parentOfCollapseNodeIndex == INVALID_IDX)
                {
                    WriteQbvhInternalNodeBbox(args, c0, 0, qbvhNodeAddr, writeAsFp16BoxNode);

                    DstBuffer.Store(qbvhNodeAddr + boxUsedChildOffset[0], ptr0);
                    DstBuffer.Store(qbvhNodeAddr + boxUsedChildOffset[1], INVALID_IDX);

                    nodeFlags = ExtractScratchNodeFlags(node.flags_and_instanceMask);
                }
            }

            // Fetch next level children if these are internal nodes and pull them into the QBVH
            if (IsLeafNode(node.right_or_geometryIndex, numActivePrims) == false)
            {
                const ScratchNode c10 = FetchScratchNode(args.scratchNodesScratchOffset, c1.left_or_primIndex_or_instIndex);
                const ScratchNode c11 = FetchScratchNode(args.scratchNodesScratchOffset, c1.right_or_geometryIndex);

                uint leafIndex10 = task.leafIndex + (c0.numPrimitivesAndDoCollapse >> 1);
                uint leafIndex11 = task.leafIndex + (c0.numPrimitivesAndDoCollapse >> 1) +
                                    (c10.numPrimitivesAndDoCollapse >> 1);

                uint numPrimitives = 0;
                uint parentOfCollapseNodeIndex = INVALID_IDX;

                if (task.parentOfCollapseNodeIndex == INVALID_IDX) //if not full node collapsing
                {
                    if (c1.numPrimitivesAndDoCollapse & 0x1) //partial node collapse
                    {
                        WriteQbvhInternalNodeBbox(args, c1, 2, qbvhNodeAddr, writeAsFp16BoxNode);

                        //child2 will point to the starting of the prim list
                        DstBuffer.Store(qbvhNodeAddr + boxUsedChildOffset[3], INVALID_IDX);
                        nodeFlags = SetBoxNodeFlagsField(nodeFlags, 0xFF, 3);

                        const uint rightScratchNodeFlags = ExtractScratchNodeFlags(node.flags_and_instanceMask);
                        nodeFlags = SetBoxNodeFlagsField(nodeFlags, rightScratchNodeFlags, 2);

                        numPrimitives = c1.numPrimitivesAndDoCollapse >> 1;
                        //bit-packing is for the index into the 4 child pointers
                        parentOfCollapseNodeIndex = (CalcQbvhInternalNodeIndex(qbvhNodeAddr, true) << 2) | 2;
                    }
                    else //no collapse
                    {
                        WriteQbvhInternalNodeBbox(args, c10, 2, qbvhNodeAddr, writeAsFp16BoxNode);
                        WriteQbvhInternalNodeBbox(args, c11, 3, qbvhNodeAddr, writeAsFp16BoxNode);

                        nodeFlags = SetBoxNodeFlagsField(nodeFlags, c1.flags_and_instanceMask, 2);
                    }
                }
                else //full node collapse
                {
                    numPrimitives = 0;
                    parentOfCollapseNodeIndex = task.parentOfCollapseNodeIndex;
                }

                Task task10;
                task10.numPrimitives             = numPrimitives;
                task10.parentOfCollapseNodeIndex = parentOfCollapseNodeIndex;
                //bit-packing is for the index into the 4 child pointers
                task10.lastNodeIndex             = (CalcQbvhInternalNodeIndex(qbvhNodeAddr, true) << 2) | 2;
                task10.leafIndex                 = leafIndex10;
                task10.nodeIndex                 = uint(c1.left_or_primIndex_or_instIndex);
                task10.nodeDestIndex             = intChildDstIdx.z;

                uint ptr2;
                if (IsLeafNode(c1.left_or_primIndex_or_instIndex, numActivePrims))
                {
                    ptr2 = WritePrimitiveNodeCollapse(args, c10, qbvhNodePtr, offsets, task10);
                }
                else
                {
                    const uint nodeType   = GetNodeType(c10.type);
                    const uint nodeOffset = CalcQbvhInternalNodeOffset(intChildDstIdx.z, true);
                    ptr2 = PackNodePointer(nodeType, nodeOffset);

                    if (parentOfCollapseNodeIndex == INVALID_IDX)
                    {
                        // parent
                        WriteParentPointer(args.metadataSizeInBytes,
                                           ptr2,
                                           qbvhNodePtr);
                    }

                    SetQBVHStackCollapse(intChildStackIdx.z, task10, baseQbvhStackOffset);
                }

                if (parentOfCollapseNodeIndex == INVALID_IDX)
                {
                    DstBuffer.Store(qbvhNodeAddr + boxUsedChildOffset[2], ptr2);
                }

                Task task11;
                task11.numPrimitives             = 0;
                task11.parentOfCollapseNodeIndex = parentOfCollapseNodeIndex;
                //bit-packing is for the index into the 4 child pointers
                task11.lastNodeIndex             = (CalcQbvhInternalNodeIndex(qbvhNodeAddr, true) << 2) | 3;
                task11.leafIndex                 = leafIndex11;
                task11.nodeIndex                 = uint(c1.right_or_geometryIndex);
                task11.nodeDestIndex             = intChildDstIdx.w;

                uint ptr3;
                if (IsLeafNode(c1.right_or_geometryIndex, numActivePrims))
                {
                    ptr3 = WritePrimitiveNodeCollapse(args, c11, qbvhNodePtr, offsets, task11);
                }
                else
                {
                    const uint nodeType   = GetNodeType(c11.type);
                    const uint nodeOffset = CalcQbvhInternalNodeOffset(intChildDstIdx.w, true);
                    ptr3 = PackNodePointer(nodeType, nodeOffset);

                    if (parentOfCollapseNodeIndex == INVALID_IDX)
                    {
                        // parent
                        WriteParentPointer(args.metadataSizeInBytes,
                                           ptr3,
                                           qbvhNodePtr);
                    }

                    SetQBVHStackCollapse(intChildStackIdx.w, task11, baseQbvhStackOffset);
                }

                if (parentOfCollapseNodeIndex == INVALID_IDX)
                {
                    DstBuffer.Store(qbvhNodeAddr + boxUsedChildOffset[3], ptr3);
                }
            }
            else
            {
                uint ptr2;
                Task task10;
                task10.numPrimitives             = 0;
                task10.parentOfCollapseNodeIndex = task.parentOfCollapseNodeIndex;
                //bit-packing is for the index into the 4 child pointers
                task10.lastNodeIndex             = (CalcQbvhInternalNodeIndex(qbvhNodeAddr, true) << 2) | 2;
                task10.leafIndex                 = task.leafIndex + (c0.numPrimitivesAndDoCollapse >> 1);
                task10.nodeIndex                 = uint(node.right_or_geometryIndex);

                ptr2 = WritePrimitiveNodeCollapse(args, c1, qbvhNodePtr, offsets, task10);

                if (task.parentOfCollapseNodeIndex == INVALID_IDX)
                {
                    WriteQbvhInternalNodeBbox(args, c1, 2, qbvhNodeAddr, writeAsFp16BoxNode);

                    DstBuffer.Store(qbvhNodeAddr + boxUsedChildOffset[2], ptr2);
                    DstBuffer.Store(qbvhNodeAddr + boxUsedChildOffset[3], INVALID_IDX);
                    nodeFlags = SetBoxNodeFlagsField(nodeFlags, 0xFF, 3);

                    const uint rightScratchNodeFlags = ExtractScratchNodeFlags(node.flags_and_instanceMask);
                    nodeFlags = SetBoxNodeFlagsField(nodeFlags, rightScratchNodeFlags, 2);
                }
            }

            if (task.parentOfCollapseNodeIndex == INVALID_IDX)
            {
                WriteQbvhInternalNodeFlags(nodeFlags, qbvhNodeAddr, writeAsFp16BoxNode);
            }

            isDone = true;
        }
    }

    if (globalId == 0)
    {
        // Root node begins after the acceleration structure header
        // Is always of type fp32 regardless of mode for fp16 box nodes
        uint rootNodePtr = CreateRootNodePointer();

        const ScratchNode rootScratchNode = FetchScratchNode(args.scratchNodesScratchOffset, rootNodeIndex);
        const BoundingBox bbox            = GetScratchNodeBoundingBoxTS(args, false, rootScratchNode);

        DstBuffer.Store3(ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET, asuint(bbox.min));
        DstBuffer.Store3(ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET + 12, asuint(bbox.max));
        DstBuffer.Store(ACCEL_STRUCT_HEADER_NODE_FLAGS_OFFSET, ExtractScratchNodeFlags(rootScratchNode.flags_and_instanceMask));

        WriteParentPointer(args.metadataSizeInBytes,
                           rootNodePtr,
                           INVALID_IDX);
    }
}
