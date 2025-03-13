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
// Builder terminology
//
// BVH2                : A binary tree hierarchy with exactly N-1 internal nodes and N leaf nodes
// Primitive Reference : A bvh2 node corresponding directly to a source primitive (instance, triangle or AABB)
// Leaf Node           : A bvh2 leaf node that contains 1 or more primitive references
// Internal Node       : A bvh2 internal node with 2 child pointers
//

//======================================================================================================================
uint GetBvh2PrimRefNodeIndex(
    uint index,
    uint numPrimRefs)
{
    return ((numPrimRefs - 1) + index);
}

//======================================================================================================================
// BVH2 child pointer encoding
// index               : 29
// unused              : 1
// leaf_node           : 1
// primitive_reference : 1
//
uint MakePrimRefChildPtr(
    uint index,
    uint numPrimRefs)
{
    // Note, a primitive reference node is always a leaf node with 1 primitive
    return GetBvh2PrimRefNodeIndex(index, numPrimRefs) | (1u << 31u);
}

//======================================================================================================================
uint MakeLeafChildPtr(
    uint index)
{
    // Note, a primitive reference node is always a leaf node with 1 primitive
    return (index) | (1u << 30u);
}

//======================================================================================================================
bool IsBvh2InternalNode(
    uint nodePtr)
{
    return ((nodePtr >> 29) == 0);
}

//======================================================================================================================
bool IsBvh2PrimRefNode(
    uint nodePtr)
{
    return ((nodePtr >> 31) != 0);
}

//======================================================================================================================
uint GetBvh2NodeIdx(
    uint nodePtr)
{
    return (nodePtr & bits(29));
}

#define BVH2_NODE_STRIDE      32

//=====================================================================================================================
// Scratch buffer layout by phases
//
// 'x' denotes unused slots (4 bytes)
//
// BVH2 Internal Nodes : [bbox_min.xyz, bbox_max.xyz, child0, child1] x numPrimitives - 1
// BVH2 Leaf Nodes     : [bbox_min.xyz, bbox_max.xyz, geomId, primId] x numPrimitives
// BVH2 root node index: [root_idx, ploc_node_counter, x, x, x, x, x, x             ] x 1
// BVH2 node flags     : [flags] x 2 * numPrimitives
// BVH2 node prim count: [count] x 1 * numPrimitives
//
// Total: (32 x 2N) + (4 * 2N) + (4 * N) = 76 bytes per primitive
//
//======================================================================================================================
uint GetBaseOffsetBvh2Node()
{
    return ShaderConstants.offsets.bvhNodeData;
}

//=====================================================================================================================
uint GetBvh2NodeOffset(uint index)
{
    return GetBaseOffsetBvh2Node() + (index * BVH2_NODE_STRIDE);
}

//======================================================================================================================
uint GetBaseOffsetMiscRootNodeIndex()
{
    // Occupies the last unused slot in the BVH2 node data section
    return GetBvh2NodeOffset((2 * ShaderConstants.numPrimitives) - 1);
}

//======================================================================================================================
uint GetBaseOffsetMiscPlocNodeCounter()
{
    // Occupies the last unused slot in the BVH2 node data section
    return GetBvh2NodeOffset((2 * ShaderConstants.numPrimitives) - 1) + sizeof(uint);
}

//======================================================================================================================
uint GetBaseOffsetBvh2NodeFlags()
{
    return GetBvh2NodeOffset(2 * ShaderConstants.numPrimitives);
}

//======================================================================================================================
uint GetBaseOffsetBvh2PrimitiveCount()
{
    return GetBaseOffsetBvh2NodeFlags() + ((2 * ShaderConstants.numPrimitives) * sizeof(uint));
}

//=====================================================================================================================
void WriteBvh2Bounds(
    uint index,
    BoundingBox bbox)
{
    ScratchBuffer.Store<BoundingBox>(GetBvh2NodeOffset(index), bbox);
}

//=====================================================================================================================
void WriteBvh2ChildPtr(
    uint index,
    uint childPtr,
    bool isRight)
{
    const uint childOffset = isRight ? 28 : 24;
    ScratchBuffer.Store(GetBvh2NodeOffset(index) + childOffset, childPtr);
}

//======================================================================================================================
void WriteRootNodeIndex(
    uint rootNodeIndex)
{
    ScratchBuffer.Store(GetBaseOffsetMiscRootNodeIndex(), rootNodeIndex);
}

//=====================================================================================================================
void WriteBvh2NodeFlags(
    uint index,
    uint flags)
{
    ScratchBuffer.Store<uint>(GetBaseOffsetBvh2NodeFlags() + (index * sizeof(uint)), flags);
}

//=====================================================================================================================
void WriteBvh2PrimitiveCount(
    uint index,
    uint flags)
{
    ScratchBuffer.Store<uint>(GetBaseOffsetBvh2PrimitiveCount() + (index * sizeof(uint)), flags);
}

//======================================================================================================================
uint ReadRootNodeIndex()
{
    return ScratchBuffer.Load(GetBaseOffsetMiscRootNodeIndex());
}

//=====================================================================================================================
BoundingBox ReadBvh2NodeBounds(
    uint index)
{
    return ScratchBuffer.Load<BoundingBox>(GetBvh2NodeOffset(index));
}

//=====================================================================================================================
uint2 ReadBvh2ChildPtrs(
    uint index)
{
    return ScratchBuffer.Load<uint2>(GetBvh2NodeOffset(index) + 24);
}

//=====================================================================================================================
uint ReadBvh2NodeFlags(
    uint index)
{
    return ScratchBuffer.Load<uint>(GetBaseOffsetBvh2NodeFlags() + (index * sizeof(uint)));
}

//=====================================================================================================================
uint ReadBvh2PrimitiveCount(
    uint index)
{
    return ScratchBuffer.Load<uint>(GetBaseOffsetBvh2PrimitiveCount() + (index * sizeof(uint)));
}

//=====================================================================================================================
uint GetBvh2PrimitiveCount(
    uint nodePtr)
{
    uint primRefCount = 1;
    if (IsBvh2PrimRefNode(nodePtr) == false)
    {
        primRefCount = ReadBvh2PrimitiveCount(GetBvh2NodeIdx(nodePtr));
    }

    return primRefCount;
}

//=====================================================================================================================
// Merges two nodes, updates parent node in memory and returns merged bounds.
BoundingBox MergeBvh2Nodes(
    inout_param(uint) parentNodeIndex,// Node index of parent node to merge into
    uint              c0,             // Child 0 node pointer
    uint              c1,             // Child 1 node pointer
    BoundingBox       bbox0,          // Child 0 bounds
    BoundingBox       bbox1)          // Child 1 bounds
{
    const BoundingBox bbox = CombineAABB(bbox0, bbox1);

    WriteBvh2Bounds(parentNodeIndex, bbox);

    const uint flags0 = ReadBvh2NodeFlags(GetBvh2NodeIdx(c0));
    const uint flags1 = ReadBvh2NodeFlags(GetBvh2NodeIdx(c1));

    WriteBvh2NodeFlags(parentNodeIndex, flags0 & flags1);

    const uint primRefCount0 = GetBvh2PrimitiveCount(c0);
    const uint primRefCount1 = GetBvh2PrimitiveCount(c1);
    const uint primRefCount  = primRefCount0 + primRefCount1;

    WriteBvh2PrimitiveCount(parentNodeIndex, primRefCount);

    if (Settings.maxPrimRangeSize > 1)
    {
        // TODO: Apply cost heuristic during collapse

        // Create fat leaf nodes for maxPrimRangeSize of 2
        if ((primRefCount <= Settings.maxPrimRangeSize) && (primRefCount == 2))
        {
            parentNodeIndex = MakeLeafChildPtr(parentNodeIndex);
        }
    }

    // TODO:
    // intersection cost for BLAS
    //

    return bbox;
}
