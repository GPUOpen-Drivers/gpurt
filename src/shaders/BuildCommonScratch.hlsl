/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
//
// For morton code functions based on libmorton:
// Copyright(c) 2016 Jeroen Baert
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef _BUILDCOMMONSCRATCH_HLSL
#define _BUILDCOMMONSCRATCH_HLSL

#include "BuildCommon.hlsl"

//=====================================================================================================================
void WriteScratchBatchIndex(
    uint numBatchesOffset,
    uint baseBatchIndicesOffset,
    uint index)
{
    uint numBatches;
    ScratchBuffer.InterlockedAdd(numBatchesOffset, 1, numBatches);

    ScratchBuffer.Store(baseBatchIndicesOffset + (numBatches * sizeof(uint)), index);
}

//=====================================================================================================================
ScratchNode FetchScratchNode(
    uint baseScratchNodesOffset,
    uint nodeIndex)
{
    const uint offset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    return ScratchBuffer.Load<ScratchNode>(offset);
}

//=====================================================================================================================
BoundingBox FetchScratchNodeBoundingBox(
    uint baseScratchNodesOffset,
    uint nodeIndex)
{
    const ScratchNode scratchNode = FetchScratchNode(baseScratchNodesOffset, nodeIndex);

    return GetScratchNodeBoundingBox(scratchNode);
}

//=====================================================================================================================
BoundingBox FetchScratchNodeBoundingBoxTS(
    uint baseScratchNodesOffset,
    uint baseSplitBoxesOffset,
    uint nodeIndex)
{
    const ScratchNode scratchNode = FetchScratchNode(baseScratchNodesOffset, nodeIndex);

    BoundingBox bbox;

    // For triangle geometry we need to generate bounding box from split boxes
    if (IsTriangleNode(scratchNode.type))
    {
        bbox = ScratchBuffer.Load<BoundingBox>(baseSplitBoxesOffset +
                                               sizeof(BoundingBox) * scratchNode.splitBox_or_nodePointer);
    }
    else
    {
        // Internal nodes and AABB geometry encodes bounding box in scratch node
        bbox.min = scratchNode.bbox_min_or_v0;
        bbox.max = scratchNode.bbox_max_or_v1;
    }

    return bbox;
}

//=====================================================================================================================
uint FetchScratchNodeFlags(
    uint baseScratchNodesOffset,
    uint nodeIndex)
{
    const ScratchNode scratchNode = FetchScratchNode(baseScratchNodesOffset, nodeIndex);

    return scratchNode.flags;
}

//=====================================================================================================================
void WriteScratchNodeFlagsFromNodes(
    uint        baseScratchNodesOffset,
    uint        nodeIndex,
    ScratchNode leftNode,
    ScratchNode rightNode)
{
    const uint leftNodeFlags = CalcNodeFlags(leftNode);
    const uint rightNodeFlags = CalcNodeFlags(rightNode);

    uint nodeFlags = 0;
    nodeFlags = SetNodeFlagsField(nodeFlags, leftNodeFlags, 0);
    nodeFlags = SetNodeFlagsField(nodeFlags, rightNodeFlags, 1);

    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);
    ScratchBuffer.Store<uint>(nodeOffset + SCRATCH_NODE_FLAGS_OFFSET, nodeFlags);
}

//=====================================================================================================================
void WriteScratchNodeFlags(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint nodeFlags)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);
    ScratchBuffer.Store<uint>(nodeOffset + SCRATCH_NODE_FLAGS_OFFSET, nodeFlags);
}

//=====================================================================================================================
void WriteParentScratchNodeFlags(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint flags,
    bool isLeft)
{
    uint nodeFlags = 0;
    nodeFlags = SetNodeFlagsField(nodeFlags, flags, !isLeft);

    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);
    ScratchBuffer.InterlockedOr(nodeOffset + SCRATCH_NODE_FLAGS_OFFSET, nodeFlags);
}

//=====================================================================================================================
uint FetchScratchNodeNumPrimitives(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    bool isLeaf)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    if (isLeaf)
    {
        return 1;
    }
    else
    {
        return ScratchBuffer.Load(nodeOffset + SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET) >> 1;
    }
}

//=====================================================================================================================
uint FetchScratchNodeNumPrimitives(
    ScratchNode node,
    bool        isLeaf)
{
    if (isLeaf)
    {
        return 1;
    }
    else
    {
        return node.numPrimitivesAndDoCollapse >> 1;
    }
}

//=====================================================================================================================
void WriteScratchNodeNumPrimitives(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint numPrimitives,
    bool doCollapse)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    ScratchBuffer.Store(nodeOffset + SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET, numPrimitives << 1 | doCollapse);
}

//=====================================================================================================================
void WriteScratchNodeCost(
    uint  baseScratchNodesOffset,
    uint  nodeIndex,
    float cost,
    bool  isLeaf)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    const uint offset = nodeOffset + (isLeaf ? SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET
                                             : SCRATCH_NODE_COST_OFFSET);

    ScratchBuffer.Store<float>(offset, cost);
}

//=====================================================================================================================
float FetchScratchNodeCost(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    bool isLeaf)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    const uint offset = nodeOffset + (isLeaf ? SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET
                                             : SCRATCH_NODE_COST_OFFSET);

    return ScratchBuffer.Load<float>(offset);
}

//=====================================================================================================================
uint FetchSortedPrimIndex(
    uint baseSortedPrimIndicesOffset,
    uint index)
{
    const uint indexOffset = baseSortedPrimIndicesOffset + (index * sizeof(uint));

    return ScratchBuffer.Load(indexOffset);
}

//=====================================================================================================================
void WriteScratchNode(
    uint        baseScratchNodesOffset,
    uint        nodeIndex,
    ScratchNode node)
{
    const uint offset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    ScratchBuffer.Store<ScratchNode>(offset, node);
}

//=====================================================================================================================
void WriteScratchNodeBoundingBox(
    uint   baseScratchNodesOffset,
    uint   nodeIndex,
    float3 bboxMin,
    float3 bboxMax)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    ScratchBuffer.Store<float3>(nodeOffset + SCRATCH_NODE_BBOX_MIN_OFFSET, bboxMin);
    ScratchBuffer.Store<float3>(nodeOffset + SCRATCH_NODE_BBOX_MAX_OFFSET, bboxMax);
}

//=====================================================================================================================
void WriteScratchNodeInstanceNumPrims(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint value)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    ScratchBuffer.Store(nodeOffset + SCRATCH_NODE_INSTANCE_NUM_PRIMS_OFFSET, value);
}

//=====================================================================================================================
void WriteScratchNodeNumMortonCells(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint value)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    ScratchBuffer.Store(nodeOffset + SCRATCH_NODE_NUM_MORTON_CELLS_OFFSET, value);
}

//=====================================================================================================================
void WriteScratchNodeType(
    uint   baseScratchNodesOffset,
    uint   fp16BoxNodesInBlasMode,
    float  fp16BoxModeMixedSaThresh,
    uint   nodeIndex,
    uint   rootIndex,
    uint   leftNodeType,
    uint   rightNodeType,
    float3 nodeBboxMin,
    float3 nodeBboxMax)
{
    uint nodeType = 0;
    {
        // Parents of leaves are marked to avoid propagating FP16 throughout the entire tree
        const uint nodeTypeLeafParentFp16 = (NODE_TYPE_BOX_FLOAT16 | NODE_POINTER_MASK_MSB);

        // Should this node be fp16 or fp32 box node?
        bool isParentOfLeaf  = false;
        bool writeNodeAsFp16 = false;
        if (fp16BoxNodesInBlasMode == ALL_INTERIOR_NODES_IN_BLAS_AS_FP16)
        {
            writeNodeAsFp16 = true;
        }
        else if (fp16BoxNodesInBlasMode == LEAF_NODES_IN_BLAS_AS_FP16)
        {
            // Mark node as fp16 IFF:
            // 1. directly attached to leaf
            // 2. child is directly attached to leaf
            const bool isParentOfFp16 = ( leftNodeType == nodeTypeLeafParentFp16) ||
                                        (rightNodeType == nodeTypeLeafParentFp16);
            isParentOfLeaf =  IsTriangleNode(leftNodeType)  ||
                              IsTriangleNode(rightNodeType) ||
                              IsUserNodeProcedural( leftNodeType) ||
                              IsUserNodeProcedural(rightNodeType);
            writeNodeAsFp16 = (isParentOfLeaf || isParentOfFp16);
        }
        else if (fp16BoxNodesInBlasMode == MIXED_NODES_IN_BLAS_AS_FP16)
        {
            // NOTE: it may help to skip nodes that would never be written out (maybe besed on depth from root?)

            // Mark node as fp16 if its bounds compress well:
            // increase in surface area should be < threshold
            const BoundingBox parentBboxFp32 = { nodeBboxMin, nodeBboxMax };
            const uint3       parentBboxFp16 = CompressBBoxToUint3(parentBboxFp32);

            const float saAsFp32 = ComputeBoxSurfaceArea(parentBboxFp32);
            const float saAsFp16 = ComputeBoxSurfaceArea(parentBboxFp16);

            const float saAsFp32Scaled = saAsFp32 * fp16BoxModeMixedSaThresh;
            writeNodeAsFp16 = (saAsFp16 < saAsFp32Scaled);
        }

        // Root node is always fp32, regardless of mode for fp16 box nodes
        if (nodeIndex == rootIndex)
        {
            isParentOfLeaf  = false;
            writeNodeAsFp16 = false;
        }

        const uint nodeTypeAsFp16 = (isParentOfLeaf  ? nodeTypeLeafParentFp16 : NODE_TYPE_BOX_FLOAT16);
        nodeType = (writeNodeAsFp16 ? nodeTypeAsFp16 : NODE_TYPE_BOX_FLOAT32);
    }

    const uint nodeOffset     = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);
    ScratchBuffer.Store<uint>(nodeOffset + SCRATCH_NODE_TYPE_OFFSET, nodeType);
}

//=====================================================================================================================
float FetchScratchInternalNodeCost(ScratchNode node)
{
    return node.sah_or_v2_or_instBasePtr.x;
}

//=====================================================================================================================
float FetchScratchLeafNodeCost(ScratchNode node)
{
    return asfloat(node.numPrimitivesAndDoCollapse);
}

//=====================================================================================================================
void WriteScratchNodeSurfaceArea(
    uint  baseScratchNodesOffset,
    uint  nodeIndex,
    float surfaceArea)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    ScratchBuffer.Store<float>(nodeOffset + SCRATCH_NODE_SA_OFFSET, surfaceArea);
}

//=====================================================================================================================
float FetchScratchNodeSurfaceArea(
    uint baseScratchNodesOffset,
    uint nodeIndex)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    return ScratchBuffer.Load<float>(nodeOffset + SCRATCH_NODE_SA_OFFSET);
}

//=====================================================================================================================
void WriteScratchNodeChild(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint child,
    bool isLeft)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    const uint direction = isLeft ? SCRATCH_NODE_LEFT_OFFSET : SCRATCH_NODE_RIGHT_OFFSET;

    ScratchBuffer.Store(nodeOffset + direction, child);
}

//=====================================================================================================================
void WriteScratchNodeParent(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint parent)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    ScratchBuffer.Store(nodeOffset + SCRATCH_NODE_PARENT_OFFSET, parent);
}

//=====================================================================================================================
void WriteSortedPrimIndex(
    uint baseSortedPrimIndicesOffset,
    uint index,
    uint primIndex)
{
    const uint indexOffset = baseSortedPrimIndicesOffset + (index * sizeof(uint));

    ScratchBuffer.Store(indexOffset, primIndex);
}

//=====================================================================================================================
uint FetchMortonCode(uint mortonCodesOffset, uint primitiveIndex)
{
    const uint offset = mortonCodesOffset + (primitiveIndex << 2);
    return ScratchBuffer.Load(offset);
}

//=====================================================================================================================
uint64_t FetchMortonCode64(uint mortonCodesOffset, uint primitiveIndex)
{
    const uint offset = mortonCodesOffset + (primitiveIndex << 3);
    return ScratchBuffer.Load<uint64_t>(offset);
}

//=====================================================================================================================
void WriteMortonCode(uint mortonCodesOffset, uint primitiveIndex, uint code)
{
    const uint offset = mortonCodesOffset + (primitiveIndex << 2);
    ScratchBuffer.Store(offset, code);
}

//=====================================================================================================================
void WriteMortonCode64(uint mortonCodesOffset, uint primitiveIndex, uint64_t code)
{
    const uint offset = mortonCodesOffset + (primitiveIndex << 3);
    ScratchBuffer.Store<uint64_t>(offset, code);
}

//=====================================================================================================================
UintBoundingBox4 FetchCentroidBox(
    uint  baseOffset,
    uint  nodeIndex,
    uint4 numBits,
    bool  pack,
    uint  numActivePrims)
{
    UintBoundingBox4 box;

    if (pack)
    {
        if (IS_LEAF(nodeIndex))
        {
            // Leaf nodes don't need to have box.max stored, because it will always be
            // box.min + 1. See WriteCentroidBox() for details.
            const uint baseLeafOffset = (numActivePrims - 1) * sizeof(PackedUintBoundingBox4);
            const uint leafOffset = sizeof(uint64_t) * LEAF_OFFSET(nodeIndex);
            uint64_t boxMin = ScratchBuffer.Load<uint64_t>(baseOffset + baseLeafOffset + leafOffset);
            box.min = UnpackUint64ToUint32x4(boxMin, numBits);
            box.max = box.min + 1;
        }
        else
        {
            PackedUintBoundingBox4 packedBox;

            packedBox = ScratchBuffer.Load<PackedUintBoundingBox4>(baseOffset + nodeIndex * sizeof(PackedUintBoundingBox4));
            box.min = UnpackUint64ToUint32x4(packedBox.min, numBits);
            box.max = UnpackUint64ToUint32x4(packedBox.max, numBits) + 1;
        }
    }
    else
    {
        box = ScratchBuffer.Load<UintBoundingBox4>(baseOffset + nodeIndex * sizeof(UintBoundingBox4));
    }
    return box;
}

//=====================================================================================================================
void WriteCentroidBox(
    uint             baseOffset,
    uint             nodeIndex,
    uint4            numBits,
    bool             pack,
    uint             numActivePrims,
    UintBoundingBox4 box)
{
    if (pack)
    {
        if (IS_LEAF(nodeIndex))
        {
            // Leaf nodes don't need to have box.max stored, because it will always be
            // box.min + 1, so we only store box.min in this case. But the inner nodes
            // still need to store both min and max.
            const uint baseLeafOffset = (numActivePrims - 1) * sizeof(PackedUintBoundingBox4);
            const uint leafOffset = sizeof(uint64_t) * LEAF_OFFSET(nodeIndex);
            uint64_t boxMin = PackUint32x4ToUint64(box.min, numBits);
            ScratchBuffer.Store<uint64_t>(baseOffset + baseLeafOffset + leafOffset, boxMin);
        }
        else
        {
            PackedUintBoundingBox4 packedBox;

            packedBox.min = PackUint32x4ToUint64(box.min, numBits);
            packedBox.max = PackUint32x4ToUint64(box.max - 1, numBits);
            ScratchBuffer.Store<PackedUintBoundingBox4>(baseOffset + nodeIndex * sizeof(PackedUintBoundingBox4), packedBox);
        }
    }
    else
    {
        ScratchBuffer.Store<UintBoundingBox4>(baseOffset + nodeIndex * sizeof(UintBoundingBox4), box);
    }
}

//=====================================================================================================================
void UpdateSceneSize(uint byteOffset, float size)
{
    // Calculate the combined AABB for the entire wave.
    const float waveSizeMin = WaveActiveMin(size);
    const float waveSizeMax = WaveActiveMax(size);

    //TODO: can just use centroids rather than boxes

    // Calculate the AABB for the entire scene using memory atomics.
    // Scalarize the atomic min/max writes by only using the first lane.
    if (WaveIsFirstLane())
    {
        // Convert the wave bounds to uints so we can atomically min/max them against the scene bounds in memory.
        const uint waveMinUint = FloatToUint(waveSizeMin);
        const uint waveMaxUint = FloatToUint(waveSizeMax);

        uint outValue;
        ScratchBuffer.InterlockedMin(byteOffset,     waveMinUint, outValue);
        ScratchBuffer.InterlockedMax(byteOffset + 4, waveMaxUint, outValue);
    }
}

//=====================================================================================================================
void UpdateSceneBounds(uint byteOffset, BoundingBox boundingBox)
{
    // Calculate the combined AABB for the entire wave.
    const float3 waveBoundsMin = WaveActiveMin(boundingBox.min);
    const float3 waveBoundsMax = WaveActiveMax(boundingBox.max);

    //TODO: can just use centroids rather than boxes

    // Calculate the AABB for the entire scene using memory atomics.
    // Scalarize the atomic min/max writes by only using the first lane.
    if (WaveIsFirstLane())
    {
        // Convert the wave bounds to uints so we can atomically min/max them against the scene bounds in memory.
        const uint3 waveMinUint = Float3ToUint3(waveBoundsMin);
        const uint3 waveMaxUint = Float3ToUint3(waveBoundsMax);

        uint outValue;
        ScratchBuffer.InterlockedMin(byteOffset,     waveMinUint.x, outValue);
        ScratchBuffer.InterlockedMin(byteOffset + 4, waveMinUint.y, outValue);
        ScratchBuffer.InterlockedMin(byteOffset + 8, waveMinUint.z, outValue);

        ScratchBuffer.InterlockedMax(byteOffset + 12, waveMaxUint.x, outValue);
        ScratchBuffer.InterlockedMax(byteOffset + 16, waveMaxUint.y, outValue);
        ScratchBuffer.InterlockedMax(byteOffset + 20, waveMaxUint.z, outValue);
    }
}

//=====================================================================================================================
void UpdateSceneBoundsUsingCentroid(uint byteOffset, float3 centroidPoint)
{
    // Calculate the combined AABB for the entire wave.
    const float3 waveBoundsMin = WaveActiveMin(centroidPoint);
    const float3 waveBoundsMax = WaveActiveMax(centroidPoint);

    //TODO: can just use centroids rather than boxes

    // Calculate the AABB for the entire scene using memory atomics.
    // Scalarize the atomic min/max writes by only using the first lane.
    if (WaveIsFirstLane())
    {
        // Convert the wave bounds to uints so we can atomically min/max them against the scene bounds in memory.
        const uint3 waveMinUint = Float3ToUint3(waveBoundsMin);
        const uint3 waveMaxUint = Float3ToUint3(waveBoundsMax);

        uint outValue;
        ScratchBuffer.InterlockedMin(byteOffset,     waveMinUint.x, outValue);
        ScratchBuffer.InterlockedMin(byteOffset + 4, waveMinUint.y, outValue);
        ScratchBuffer.InterlockedMin(byteOffset + 8, waveMinUint.z, outValue);

        ScratchBuffer.InterlockedMax(byteOffset + 12, waveMaxUint.x, outValue);
        ScratchBuffer.InterlockedMax(byteOffset + 16, waveMaxUint.y, outValue);
        ScratchBuffer.InterlockedMax(byteOffset + 20, waveMaxUint.z, outValue);
    }
}

//=====================================================================================================================
void UpdateCentroidSceneBoundsWithSize(uint byteOffset, BoundingBox boundingBox)
{
    const float3 centroidPoint = (0.5 * (boundingBox.max + boundingBox.min));

    UpdateSceneBoundsUsingCentroid( byteOffset, centroidPoint);

    UpdateSceneSize( byteOffset + 24, ComputeBoxSurfaceArea(boundingBox));
}

//=====================================================================================================================
void UpdateSceneBoundsWithSize(uint byteOffset, BoundingBox boundingBox)
{
    UpdateSceneBounds( byteOffset, boundingBox);

    UpdateSceneSize( byteOffset + 24, ComputeBoxSurfaceArea(boundingBox));
}

//=====================================================================================================================
BoundingBox FetchSceneBounds(uint sceneBoundsOffset)
{
    UintBoundingBox sceneBounds;

    uint4 data;
    data            = ScratchBuffer.Load4(sceneBoundsOffset);
    sceneBounds.min = data.xyz;
    data.xy         = ScratchBuffer.Load2(sceneBoundsOffset + 0x10);
    sceneBounds.max = data.wxy;

    BoundingBox bbox;
    bbox.min = Uint3ToFloat3(sceneBounds.min);
    bbox.max = Uint3ToFloat3(sceneBounds.max);

    return bbox;
}

//=====================================================================================================================
float2 FetchSceneSize(uint sceneBoundsOffset)
{
    uint2 data;
    data.x          = ScratchBuffer.Load(sceneBoundsOffset + 24);
    data.y          = ScratchBuffer.Load(sceneBoundsOffset + 28);

    float2 minMax;
    minMax.x = UintToFloat(data.x);
    minMax.y = UintToFloat(data.y);

    return minMax;
}

//=====================================================================================================================
void RefitNode(
    uint                rootIndex,
    uint                nodeIndex,
    uint                numTris,
    const RefitArgs     args)
{
    if (args.enablePairCompression && (nodeIndex == 0) && (args.numActivePrims == 1))
    {
        // Ensure that a batch index is written out for single-primitive acceleration structures.
        WriteScratchBatchIndex(args.numBatchesOffset, args.baseBatchIndicesOffset, 0);
    }

    uint instancePrimCount = 0;

    // Ct is the cost of intersecting triangle
    // Ci is the cost of interecting bbox
    const float Ct = SAH_COST_TRIANGLE_INTERSECTION;
    const float Ci = SAH_COST_AABBB_INTERSECTION;

    const ScratchNode node = FetchScratchNode(args.baseScratchNodesOffset, nodeIndex);

    // Fetch child indices
    const uint lc = node.left_or_primIndex_or_instIndex;
    const uint rc = node.right_or_geometryIndex;

    const ScratchNode leftNode  = FetchScratchNode(args.baseScratchNodesOffset, lc);
    const ScratchNode rightNode = FetchScratchNode(args.baseScratchNodesOffset, rc);

    // Fetch bounding children bounding boxes
    BoundingBox bboxRightChild;
    BoundingBox bboxLeftChild;

    float rightCost;
    float leftCost;

    uint numMortonCells = 0;

    // Right child
    {
        const bool isLeafNode = IsLeafNode(rc, args.numActivePrims);

        if (args.doTriangleSplitting && isLeafNode)
        {
            bboxRightChild =
                ScratchBuffer.Load<BoundingBox>(args.splitBoxesOffset +
                                                sizeof(BoundingBox) * rightNode.splitBox_or_nodePointer);
        }
        else
        {
            bboxRightChild = GetScratchNodeBoundingBox(rightNode);
        }

        rightCost = isLeafNode ? FetchScratchLeafNodeCost(rightNode) : FetchScratchInternalNodeCost(rightNode);
        numMortonCells += isLeafNode ? 1 : rightNode.splitBox_or_nodePointer;
    }

    // Left child
    {
        const bool isLeafNode = IsLeafNode(lc, args.numActivePrims);

        if (args.doTriangleSplitting && isLeafNode)
        {
            bboxLeftChild =
                ScratchBuffer.Load<BoundingBox>(args.splitBoxesOffset +
                                                sizeof(BoundingBox) * leftNode.splitBox_or_nodePointer);
        }
        else
        {
            bboxLeftChild = GetScratchNodeBoundingBox(leftNode);
        }

        leftCost = isLeafNode ? FetchScratchLeafNodeCost(leftNode) : FetchScratchInternalNodeCost(leftNode);
        numMortonCells += isLeafNode ? 1 : leftNode.splitBox_or_nodePointer;
    }

    // Merge bounding boxes up to parent
    BoundingBox mergedBox = (BoundingBox)0;
    mergedBox.min = min(bboxLeftChild.min, bboxRightChild.min);
    mergedBox.max = max(bboxLeftChild.max, bboxRightChild.max);

    // Compute surface area of parent box
    const float mergedBoxSurfaceArea = ComputeBoxSurfaceArea(mergedBox);

    WriteScratchNodeBoundingBox(args.baseScratchNodesOffset,
                                nodeIndex,
                                mergedBox.min,
                                mergedBox.max);

    WriteScratchNodeSurfaceArea(args.baseScratchNodesOffset,
                                nodeIndex,
                                mergedBoxSurfaceArea);

    float bestCost = leftCost + rightCost + Ci * mergedBoxSurfaceArea;
    bool isCollapsed = false;

    if (args.doCollapse || args.enablePairCompression)
    {
        const float splitCost    = Ci + leftCost / mergedBoxSurfaceArea + rightCost / mergedBoxSurfaceArea;
        const float collapseCost = Ct * numTris;
        const bool  useCostCheck = args.doCollapse || args.enablePairCostCheck;

        const bool leftCollapse      = (leftNode.numPrimitivesAndDoCollapse & 0x1) ||
                                       IsLeafNode(lc, args.numActivePrims);
        const bool rightCollapse     = (rightNode.numPrimitivesAndDoCollapse & 0x1) ||
                                       IsLeafNode(rc, args.numActivePrims);
        const bool collapseBothSides = leftCollapse && rightCollapse;

        // Limit number of triangles collapsed in a single bounding box to MAX_COLLAPSED_TRIANGLES
        if ((useCostCheck && (collapseCost > splitCost)) ||
            (numTris > MAX_COLLAPSED_TRIANGLES) ||
            (args.enablePairCompression && ((collapseBothSides == false) || (nodeIndex == rootIndex))))
        {
            if (args.enablePairCompression)
            {
                if (leftCollapse)
                {
                    WriteScratchBatchIndex(args.numBatchesOffset, args.baseBatchIndicesOffset, lc);
                }

                if (rightCollapse)
                {
                    WriteScratchBatchIndex(args.numBatchesOffset, args.baseBatchIndicesOffset, rc);
                }
            }
        }
        else
        {
            bestCost = collapseCost * mergedBoxSurfaceArea;
            isCollapsed = true;
        }
    }

    WriteScratchNodeCost(args.baseScratchNodesOffset, nodeIndex, bestCost, false);
    WriteScratchNodeNumPrimitives(args.baseScratchNodesOffset, nodeIndex, numTris, isCollapsed);

    // Decide on what type of interior box node the parent should be
    // and write the type into scratch
    WriteScratchNodeType(args.baseScratchNodesOffset,
                         args.fp16BoxNodesMode,
                         args.fp16BoxModeMixedSaThreshold,
                         nodeIndex,
                         rootIndex,
                         leftNode.type,
                         rightNode.type,
                         mergedBox.min,
                         mergedBox.max);

    WriteScratchNodeFlagsFromNodes(args.baseScratchNodesOffset,
                                   nodeIndex,
                                   leftNode,
                                   rightNode);

    if (args.enableInstancePrimCount)
    {
        const uint instancePrimCount = asuint(rightNode.sah_or_v2_or_instBasePtr[2]) +
                                       asuint(leftNode.sah_or_v2_or_instBasePtr[2]);

        WriteScratchNodeInstanceNumPrims(args.baseScratchNodesOffset,
                                         nodeIndex,
                                         instancePrimCount);
    }
}
#endif
