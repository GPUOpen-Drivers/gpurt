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

#include "../shared/scratchNode.h"
#include "BuildCommon.hlsl"

//=====================================================================================================================
void WriteGridPosAtIndex(uint offset, uint index, uint4 gridPos)
{
    ScratchBuffer.Store<uint4>(offset + index * sizeof(uint4), gridPos);
}

//=====================================================================================================================
uint4 FetchGridPosAtIndex(uint offset, uint index)
{
    return ScratchBuffer.Load<uint4>(offset + index * sizeof(uint4));
}

//=====================================================================================================================
void WriteSplitBoxAtIndex(uint offset, uint index, BoundingBox box)
{
    ScratchBuffer.Store<BoundingBox>(offset + index * sizeof(BoundingBox), box);
}

//=====================================================================================================================
BoundingBox FetchSplitBoxAtIndex(uint offset, uint index)
{
    return ScratchBuffer.Load<BoundingBox>(offset + index * sizeof(BoundingBox));
}

//=====================================================================================================================
void WriteRootNodeIndex(uint offset, uint index)
{
    ScratchBuffer.Store(offset, index);
}

//=====================================================================================================================
uint FetchRootNodeIndex(uint enableFastLBVH, uint offset)
{
    return enableFastLBVH ? ScratchBuffer.Load(offset) : 0;
}

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
static bool IsScratchTriangleNode(
    in ScratchNode node)
{
    return (GetNodeType(node.type) <= NODE_TYPE_TRIANGLE_1);
}

//=====================================================================================================================
// Calculate BBox for scratch node for BVH2 builder
static BoundingBox FetchScratchNodeBoundingBox(
    in ScratchNode node,
    in bool        isLeafNode,
    in bool        doTriangleSplit,
    in uint        splitBoxesByteOffset,
    in bool        doTriangleCompression,
    in uint        baseScratchNodesOffset)
{
    BoundingBox bbox;

    // For triangle geometry we need to generate bounding box from split boxes
    if (IsScratchTriangleNode(node))
    {
        if (doTriangleSplit && isLeafNode)
        {
            bbox = FetchSplitBoxAtIndex(splitBoxesByteOffset, node.splitBox_or_nodePointer);
        }
        else
        {
            // Generate the BoundingBox of given triangle node
            bbox = GenerateTriangleBoundingBox(node.bbox_min_or_v0,
                                               node.bbox_max_or_v1,
                                               node.sah_or_v2_or_instBasePtr);

            // if TriangleCompression is on, and this is a leaf node
            // need to fetch the "pairedNode" (if any) and update boundingbox
            if (doTriangleCompression && isLeafNode && (node.splitBox_or_nodePointer != INVALID_IDX))
            {
                BoundingBox nodeBbox = bbox;
                ScratchNode pairedNode = FetchScratchNode(baseScratchNodesOffset,
                                                          node.splitBox_or_nodePointer);

                BoundingBox pairedNodeBbox = GenerateTriangleBoundingBox(pairedNode.bbox_min_or_v0,
                                                                         pairedNode.bbox_max_or_v1,
                                                                         pairedNode.sah_or_v2_or_instBasePtr);

                bbox.min = min(nodeBbox.min, pairedNodeBbox.min);
                bbox.max = max(nodeBbox.max, pairedNodeBbox.max);
            }
        }
    }
    else
    {
        // Internal nodes and AABB geometry encodes bounding box in scratch node
        bbox.min = node.bbox_min_or_v0;
        bbox.max = node.bbox_max_or_v1;
    }

    return bbox;
}

//=====================================================================================================================
static BoundingBox GetScratchNodeBoundingBox(
    in ScratchNode node)
{
    BoundingBox bbox;

    // For triangle geometry we need to generate bounding box from triangle vertices
    if (IsScratchTriangleNode(node))
    {
        bbox = GenerateTriangleBoundingBox(node.bbox_min_or_v0,
                                           node.bbox_max_or_v1,
                                           node.sah_or_v2_or_instBasePtr);
    }
    else
    {
        // Internal nodes and AABB geometry encodes bounding box in scratch node
        bbox.min = node.bbox_min_or_v0;
        bbox.max = node.bbox_max_or_v1;
    }

    return bbox;
}

//=====================================================================================================================
void WriteScratchNodeFlags(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint nodeFlags)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);
    ScratchBuffer.Store<uint>(nodeOffset + SCRATCH_NODE_FLAGS_AND_INSTANCE_MASK_OFFSET, nodeFlags);
}

//=====================================================================================================================
void WriteScratchNodeFlagsFromNodes(
    uint           baseScratchNodesOffset,
    uint           nodeIndex,
    in ScratchNode leftNode,
    in ScratchNode rightNode)
{
    // Combine box node flags from child nodes and write to parent scratch node
    const uint boxNodeFlags = leftNode.flags_and_instanceMask & rightNode.flags_and_instanceMask;
    WriteScratchNodeFlags(baseScratchNodesOffset, nodeIndex, boxNodeFlags);
}

//=====================================================================================================================
// Update node flags from child in parent scratch node
void UpdateParentScratchNodeFlags(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint flagsAndInstanceMask)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);
    ScratchBuffer.InterlockedAnd(nodeOffset + SCRATCH_NODE_FLAGS_AND_INSTANCE_MASK_OFFSET, flagsAndInstanceMask);
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
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint nodeType)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);
    ScratchBuffer.Store<uint>(nodeOffset + SCRATCH_NODE_TYPE_OFFSET, nodeType);
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

    const uint childOffset = isLeft ? SCRATCH_NODE_LEFT_OFFSET : SCRATCH_NODE_RIGHT_OFFSET;

    ScratchBuffer.Store(nodeOffset + childOffset, child);
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
uint FetchScratchNodeParent(
    uint baseScratchNodesOffset,
    uint nodeIndex)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    return ScratchBuffer.Load<uint>(nodeOffset + SCRATCH_NODE_PARENT_OFFSET);
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

//======================================================================================================================
uint GetBvhNodesOffset(
    uint numActivePrims,
    uint numLeafNodes,
    uint bvhNodeDataOffset,
    uint bvhLeafNodeDataOffset,
    uint primIndicesSortedOffset)
{
    if ((Settings.noCopySortedNodes == true) && (Settings.enableFastLBVH == false) && (numActivePrims == 1))
    {
        // If there is one active primitive among a number of inactive primitives and
        // NoCopySortedNodes is enabled, then we need to move the root to match the
        // offset of the single active primitive.
        // The FastLBVH does not need to do this, because it provides the root's index
        // in ShaderConstants.offsets.fastLBVHRootNodeIndex.
        const uint nodeIndex = FetchSortedPrimIndex(primIndicesSortedOffset, 0);
        return bvhLeafNodeDataOffset + nodeIndex * SCRATCH_NODE_SIZE;
    }
    else
    {
        return CalculateScratchBvhNodesOffset(
                   numActivePrims,
                   numLeafNodes,
                   bvhNodeDataOffset,
                   Settings.noCopySortedNodes);
    }
}

//=====================================================================================================================
void MergeScratchNodes(
    uint        scratchNodesOffset,
    uint        numBatchesScratchOffset,
    uint        batchIndicesScratchOffset,
    uint        numActivePrims,
    uint        mergedNodeIndex,
    uint        leftNodeIndex,
    ScratchNode leftNode,
    uint        rightNodeIndex,
    ScratchNode rightNode,
    float       mergedBoxSurfaceArea)
{
    const uint numLeft  = FetchScratchNodeNumPrimitives(leftNode, IsLeafNode(leftNodeIndex, numActivePrims));
    const uint numRight = FetchScratchNodeNumPrimitives(rightNode, IsLeafNode(rightNodeIndex, numActivePrims));
    const uint numTris  = numLeft + numRight;

    const float Ct =
        SAH_COST_TRIANGLE_INTERSECTION;

    const float Ci = SAH_COST_AABBB_INTERSECTION;

    const float leftCost = FetchScratchNodeCost(scratchNodesOffset, leftNodeIndex,
                                                IsLeafNode(leftNodeIndex, numActivePrims));

    const float rightCost = FetchScratchNodeCost(scratchNodesOffset, rightNodeIndex,
                                                 IsLeafNode(rightNodeIndex, numActivePrims));

    const bool leftCollapse      = (leftNode.numPrimitivesAndDoCollapse & 0x1) ||
                                    IsLeafNode(leftNodeIndex, numActivePrims);
    const bool rightCollapse     = (rightNode.numPrimitivesAndDoCollapse & 0x1) ||
                                    IsLeafNode(rightNodeIndex, numActivePrims);
    const bool collapseBothSides = leftCollapse && rightCollapse;

    float bestCost = leftCost + rightCost + Ci * mergedBoxSurfaceArea;

    const float collapseCost = Ct * numTris;

    const float splitCost    = Ci + leftCost / mergedBoxSurfaceArea + rightCost / mergedBoxSurfaceArea;

    bool isCollapsed = false;

    if (EnableLatePairCompression()
    )
    {
        const bool  useCostCheck = Settings.enablePairCostCheck;

        // Limit number of triangles collapsed in a single bounding box to MAX_COLLAPSED_TRIANGLES
        if ((useCostCheck && (collapseCost > splitCost)) ||
            (numTris > MAX_COLLAPSED_TRIANGLES) ||
            (collapseBothSides == false) ||
            (mergedNodeIndex == 0))
        {
            if (leftCollapse)
            {
                WriteScratchBatchIndex(numBatchesScratchOffset,
                                       batchIndicesScratchOffset,
                                       leftNodeIndex);
            }

            if (rightCollapse)
            {
                WriteScratchBatchIndex(numBatchesScratchOffset,
                                       batchIndicesScratchOffset,
                                       rightNodeIndex);
            }
        }
        else // do collapse
        {
            bestCost = collapseCost * mergedBoxSurfaceArea;
            isCollapsed = true;
        }
    }

    WriteScratchNodeType(scratchNodesOffset,
                         mergedNodeIndex,
                         GetInternalNodeType());
    WriteScratchNodeFlagsFromNodes(scratchNodesOffset,
                                   mergedNodeIndex,
                                   leftNode,
                                   rightNode);
    WriteScratchNodeSurfaceArea(scratchNodesOffset,
                                mergedNodeIndex,
                                mergedBoxSurfaceArea);
    WriteScratchNodeCost(scratchNodesOffset, mergedNodeIndex, bestCost, false);
    WriteScratchNodeNumPrimitives(scratchNodesOffset, mergedNodeIndex, numTris, isCollapsed);

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
    // need to fetch "other paired node" if earlyPairCompression is enabled
    {
        const bool isLeafNode = IsLeafNode(rc, args.numActivePrims);
        bboxRightChild = FetchScratchNodeBoundingBox(rightNode,
                                                     isLeafNode,
                                                     args.doTriangleSplitting,
                                                     args.splitBoxesOffset,
                                                     args.enableEarlyPairCompression,
                                                     args.unsortedNodesBaseOffset);
        numMortonCells += isLeafNode ? 1 : rightNode.splitBox_or_nodePointer;
    }

    // Left child
    // need to fetch "other paired node" if earlyPairCompression is enabled
    {
        const bool isLeafNode = IsLeafNode(lc, args.numActivePrims);
        bboxLeftChild = FetchScratchNodeBoundingBox(leftNode,
                                                    isLeafNode,
                                                    args.doTriangleSplitting,
                                                    args.splitBoxesOffset,
                                                    args.enableEarlyPairCompression,
                                                    args.unsortedNodesBaseOffset);
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

    MergeScratchNodes(
        args.baseScratchNodesOffset,
        args.numBatchesOffset,
        args.baseBatchIndicesOffset,
        args.numActivePrims,
        nodeIndex,
        lc,
        leftNode,
        rc,
        rightNode,
        mergedBoxSurfaceArea);

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
