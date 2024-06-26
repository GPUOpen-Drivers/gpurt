/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "BuildCommonScratchGlobal.hlsl"
#include "TaskCounter.hlsl"

//=====================================================================================================================
void WriteScratchBatchIndex(
    uint numBatchesOffset,
    uint baseBatchIndicesOffset,
    uint index)
{
    uint numBatches = AllocateBatchIndex(numBatchesOffset);

    ScratchBuffer.Store(baseBatchIndicesOffset + (numBatches * sizeof(uint)), index);
}

//=====================================================================================================================
static bool IsTrianglePrimitiveBuild()
{
    return (Settings.topLevelBuild == 0) && (Settings.geometryType == GEOMETRY_TYPE_TRIANGLES);
}

//=====================================================================================================================
// Compute bounding box for a BVH2 scratch node
static BoundingBox GetScratchNodeBoundingBox(
    in ScratchNode node,
    in bool        isLeafNode,
    in bool        triangleSplittingEnabled,
    in uint        splitBoxesByteOffset,
    in bool        trianglePairingEnabled,
    in uint        baseScratchNodesOffset)
{
    BoundingBox bbox;

    if (isLeafNode && IsTrianglePrimitiveBuild())
    {
        // For triangle geometry we need to generate bounding box from split boxes
        if (triangleSplittingEnabled && (node.splitBox_or_nodePointer != INVALID_IDX))
        {
            bbox = FetchSplitBoxAtIndex(splitBoxesByteOffset, node.splitBox_or_nodePointer);
        }
        else
        {
            // Generate the BoundingBox of given triangle node
            bbox = GenerateTriangleBoundingBox(node.bbox_min_or_v0,
                                               node.bbox_max_or_v1,
                                               node.sah_or_v2_or_instBasePtr);

            if (trianglePairingEnabled)
            {
                if (Settings.enableEarlyPairCompression)
                {
                    // Early triangle pairing encodes the quad in a single scratch node
                    if (IsScratchNodeQuadPrimitive(node))
                    {
                        const float3 v3 = float3(asfloat(node.splitBox_or_nodePointer),
                                                 asfloat(node.numPrimitivesAndDoCollapse),
                                                 asfloat(node.sortedPrimIndex));

                        bbox.min = min(v3, bbox.min);
                        bbox.max = max(v3, bbox.max);
                    }
                }
                else if (node.splitBox_or_nodePointer != INVALID_IDX)
                {
                    // Late triangle pairing uses multiple scratch nodes to encode paired triangle data
                    BoundingBox nodeBbox = bbox;
                    ScratchNode pairedNode = FetchScratchNode(baseScratchNodesOffset, node.splitBox_or_nodePointer);

                    BoundingBox pairedNodeBbox = GenerateTriangleBoundingBox(pairedNode.bbox_min_or_v0,
                                                                             pairedNode.bbox_max_or_v1,
                                                                             pairedNode.sah_or_v2_or_instBasePtr);

                    bbox.min = min(nodeBbox.min, pairedNodeBbox.min);
                    bbox.max = max(nodeBbox.max, pairedNodeBbox.max);
                }
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
// Get instance node bound from scratch node. Caller must ensure the scratch node passed in is a TLAS leaf node
static BoundingBox GetScratchNodeInstanceBounds(
    in ScratchNode node)
{
    BoundingBox bbox;
    bbox.min = node.bbox_min_or_v0;
    bbox.max = node.bbox_max_or_v1;
    return bbox;
}

//=====================================================================================================================
void WriteScratchNodeFlags(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint nodeFlags)
{
    WriteScratchNodeData(
        baseScratchNodesOffset,
        nodeIndex,
        SCRATCH_NODE_FLAGS_OFFSET,
        nodeFlags);
}

//=====================================================================================================================
void WriteScratchNodeFlagsFromNodes(
    uint           baseScratchNodesOffset,
    uint           nodeIndex,
    in ScratchNode leftNode,
    in ScratchNode rightNode)
{
    // Combine box node flags from child nodes and write to parent scratch node
    const uint boxNodeFlags = leftNode.packedFlags & rightNode.packedFlags;
    WriteScratchNodeFlags(baseScratchNodesOffset, nodeIndex, boxNodeFlags);
}

//=====================================================================================================================
uint FetchScratchNodeNumPrimitives(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    bool isLeaf)
{
    if (isLeaf)
    {
        if (Settings.enableEarlyPairCompression)
        {
            const uint packedGeometryIndex =
                FETCH_SCRATCH_NODE_DATA(uint, baseScratchNodesOffset, nodeIndex, SCRATCH_NODE_GEOMETRY_INDEX_OFFSET);
            return IsQuadPrimitive(packedGeometryIndex) ? 2 : 1;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return (FETCH_SCRATCH_NODE_DATA(
            uint,
            baseScratchNodesOffset,
            nodeIndex,
            SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET) >> 1);
    }
}

//=====================================================================================================================
static uint FetchScratchNodeNumPrimitives(
    ScratchNode node,
    bool        isLeaf)
{
    if (isLeaf)
    {
        if (Settings.enableEarlyPairCompression)
        {
            return IsScratchNodeQuadPrimitive(node) ? 2 : 1;
        }
        else
        {
            return 1;
        }
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
    WriteScratchNodeData(
        baseScratchNodesOffset,
        nodeIndex,
        SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET,
        (numPrimitives << 1 | doCollapse));
}

//=====================================================================================================================
void WriteScratchNodeCost(
    uint  baseScratchNodesOffset,
    uint  nodeIndex,
    float cost)
{
    WriteScratchNodeData(
        baseScratchNodesOffset,
        nodeIndex,
        SCRATCH_NODE_COST_OFFSET,
        cost);
}

//=====================================================================================================================
float FetchScratchNodeCost(
    uint baseScratchNodesOffset,
    uint nodeIndex)
{
    return FETCH_SCRATCH_NODE_DATA(float, baseScratchNodesOffset, nodeIndex, SCRATCH_NODE_COST_OFFSET);
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
void WriteScratchNodeBoundingBox(
    uint   baseScratchNodesOffset,
    uint   nodeIndex,
    float3 bboxMin,
    float3 bboxMax)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    WriteScratchNodeDataAtOffset(nodeOffset, SCRATCH_NODE_BBOX_MIN_OFFSET, bboxMin);
    WriteScratchNodeDataAtOffset(nodeOffset, SCRATCH_NODE_BBOX_MAX_OFFSET, bboxMax);
}

//=====================================================================================================================
void WriteScratchNodeLargestLength(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    float value)
{
    WriteScratchNodeData(baseScratchNodesOffset, nodeIndex, SCRATCH_NODE_NUM_LARGEST_LENGTH_OFFSET, value);
}

//=====================================================================================================================
void WriteScratchNodeSurfaceArea(
    uint  baseScratchNodesOffset,
    uint  nodeIndex,
    float surfaceArea)
{
    WriteScratchNodeData(baseScratchNodesOffset, nodeIndex, SCRATCH_NODE_SA_OFFSET, surfaceArea);
}

//=====================================================================================================================
float FetchScratchNodeSurfaceArea(
    uint baseScratchNodesOffset,
    uint nodeIndex)
{
    return FETCH_SCRATCH_NODE_DATA(float, baseScratchNodesOffset, nodeIndex, SCRATCH_NODE_SA_OFFSET);
}

//=====================================================================================================================
void WriteScratchNodeChild(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint child,
    bool isLeft)
{
    const uint dataOffset = isLeft ? SCRATCH_NODE_LEFT_OFFSET : SCRATCH_NODE_RIGHT_OFFSET;

    WriteScratchNodeData(baseScratchNodesOffset, nodeIndex, dataOffset, child);
}

//=====================================================================================================================
void WriteScratchNodeParent(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint parent)
{
    {
        WriteScratchNodeData(baseScratchNodesOffset, nodeIndex, SCRATCH_NODE_PARENT_OFFSET, parent);
    }
}

//=====================================================================================================================
uint FetchScratchNodeParent(
    uint baseScratchNodesOffset,
    uint nodeIndex)
{
    return FETCH_SCRATCH_NODE_DATA(uint, baseScratchNodesOffset, nodeIndex, SCRATCH_NODE_PARENT_OFFSET);
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
float FetchScratchNodeLargestLength(
    uint baseScratchNodesOffset,
    uint nodeIndex)
{
    return FETCH_SCRATCH_NODE_DATA(float, baseScratchNodesOffset, nodeIndex, SCRATCH_NODE_NUM_LARGEST_LENGTH_OFFSET);
}

//=====================================================================================================================
float FetchScratchNodeLargestLength(ScratchNode node)
{
    return asfloat(node.splitBox_or_nodePointer);
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
    uint maxNumPrimitives,
    uint bvhNodeDataOffset,
    uint bvhLeafNodeDataOffset,
    uint primIndicesSortedOffset)
{
    if ((Settings.enableTopDownBuild == false) &&
        (Settings.enableFastLBVH == false) &&
        (numActivePrims == 1) &&
        (maxNumPrimitives > 1))
    {
        // If there is one active primitive among a number of inactive primitives and
        // the top down builder is disabled, then we need to move the root to match the
        // offset of the single active primitive.
        // The FastLBVH does not need to do this, because it provides the root's index
        // in ShaderConstants.offsets.fastLBVHRootNodeIndex.
        const uint nodeIndex = FetchSortedPrimIndex(primIndicesSortedOffset, 0);
        return CalcScratchNodeOffset(bvhLeafNodeDataOffset, nodeIndex);
    }
    else
    {
        return CalculateScratchBvhNodesOffset(
                   numActivePrims,
                   maxNumPrimitives,
                   bvhNodeDataOffset,
                   Settings.enableTopDownBuild);
    }
}

//=====================================================================================================================
static uint CalculateBvhNodesOffset(
    BuildShaderConstants shaderConstants,
    uint                 numActivePrims)
{
    return GetBvhNodesOffset(
               numActivePrims,
               shaderConstants.maxNumPrimitives,
               shaderConstants.offsets.bvhNodeData,
               shaderConstants.offsets.bvhLeafNodeData,
               shaderConstants.offsets.primIndicesSorted);
}

//=====================================================================================================================
bool IsLeafOrIsCollapsed(
    in uint nodeIndex,
    in uint baseScratchNodesOffset,
    in uint numActivePrims)
{
    bool result = false;

    if (IsLeafNode(nodeIndex, numActivePrims))
    {
        result = true;
    }
    else if (Settings.topLevelBuild == false)
    {
        const ScratchNode node = FetchScratchNode(baseScratchNodesOffset, nodeIndex);

        if (node.numPrimitivesAndDoCollapse & 1)
        {
            result = true;
        }
    }

    return result;
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
    BoundingBox leftBounds,
    uint        rightNodeIndex,
    ScratchNode rightNode,
    BoundingBox rightBounds,
    uint        rootIndex)
{
    BoundingBox mergedBounds = CombineAABB(leftBounds, rightBounds);
    const float mergedBoxSurfaceArea = ComputeBoxSurfaceArea(mergedBounds);

    WriteScratchNodeBoundingBox(scratchNodesOffset,
                                mergedNodeIndex,
                                mergedBounds.min,
                                mergedBounds.max);

    WriteScratchNodeFlagsFromNodes(scratchNodesOffset,
                                   mergedNodeIndex,
                                   leftNode,
                                   rightNode);

    WriteScratchNodeSurfaceArea(scratchNodesOffset, mergedNodeIndex, mergedBoxSurfaceArea);

    if (Settings.topLevelBuild)
    {
        uint numPrims = 0;

        WriteScratchNodeNumPrimitives(scratchNodesOffset, mergedNodeIndex, numPrims, false);
    }
    else
    {
        const uint numLeft  = FetchScratchNodeNumPrimitives(leftNode, IsLeafNode(leftNodeIndex, numActivePrims));
        const uint numRight = FetchScratchNodeNumPrimitives(rightNode, IsLeafNode(rightNodeIndex, numActivePrims));
        const uint numTris = numLeft + numRight;

        const float Ct =
            SAH_COST_TRIANGLE_INTERSECTION;

        const float Ci = SAH_COST_AABBB_INTERSECTION;

        const float leftCost =
            IsLeafNode(leftNodeIndex, numActivePrims) ?
                (Ct * ComputeBoxSurfaceArea(leftBounds)) : FetchScratchNodeCost(scratchNodesOffset, leftNodeIndex);

        const float rightCost =
            IsLeafNode(rightNodeIndex, numActivePrims) ?
                (Ct * ComputeBoxSurfaceArea(rightBounds)) : FetchScratchNodeCost(scratchNodesOffset, rightNodeIndex);

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
            const bool useCostCheck = Settings.enablePairCostCheck;

            if ((useCostCheck && (collapseCost > splitCost)) ||
                (numTris > LATE_PAIR_COMP_BATCH_SIZE) ||
                (collapseBothSides == false) ||
                (mergedNodeIndex == rootIndex))
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

        WriteScratchNodeCost(scratchNodesOffset, mergedNodeIndex, bestCost);
        WriteScratchNodeNumPrimitives(scratchNodesOffset, mergedNodeIndex, numTris, isCollapsed);
    }
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

    const ScratchNode node = FetchScratchNode(args.baseScratchNodesOffset, nodeIndex);

    // Fetch child indices
    const uint lc = node.left_or_primIndex_or_instIndex;
    const uint rc = node.right_or_geometryIndex;

    const ScratchNode leftNode  = FetchScratchNode(args.baseScratchNodesOffset, lc);
    const ScratchNode rightNode = FetchScratchNode(args.baseScratchNodesOffset, rc);

    // Fetch bounding children bounding boxes
    BoundingBox bboxRightChild;
    BoundingBox bboxLeftChild;

    // Right child
    // need to fetch "other paired node" if earlyPairCompression is enabled
    {
        const bool isLeafNode = IsLeafNode(rc, args.numActivePrims);
        bboxRightChild = GetScratchNodeBoundingBox(rightNode,
                                                   isLeafNode,
                                                   args.doTriangleSplitting,
                                                   args.splitBoxesOffset,
                                                   args.enableEarlyPairCompression,
                                                   args.unsortedNodesBaseOffset);
    }

    // Left child
    // need to fetch "other paired node" if earlyPairCompression is enabled
    {
        const bool isLeafNode = IsLeafNode(lc, args.numActivePrims);
        bboxLeftChild = GetScratchNodeBoundingBox(leftNode,
                                                  isLeafNode,
                                                  args.doTriangleSplitting,
                                                  args.splitBoxesOffset,
                                                  args.enableEarlyPairCompression,
                                                  args.unsortedNodesBaseOffset);
    }

    MergeScratchNodes(
        args.baseScratchNodesOffset,
        args.numBatchesOffset,
        args.baseBatchIndicesOffset,
        args.numActivePrims,
        nodeIndex,
        lc,
        leftNode,
        bboxLeftChild,
        rc,
        rightNode,
        bboxRightChild,
        rootIndex);
}

//=====================================================================================================================
static TriangleData GetScratchNodeTrianglePairVertices(
    in uint scratchNodesOffset,
    in uint nodeIndex,
    in uint triangleIndex)
{
    const uint nodeType = (triangleIndex == 0) ? NODE_TYPE_TRIANGLE_0 : NODE_TYPE_TRIANGLE_1;

    const uint packedFlags = FETCH_SCRATCH_NODE_DATA(uint, scratchNodesOffset, nodeIndex, SCRATCH_NODE_FLAGS_OFFSET);

    uint3 indices = CalcTriangleCompressionVertexIndices(nodeType, ExtractScratchNodeTriangleId(packedFlags));

    TriangleData tri;

    tri.v0 = FETCH_SCRATCH_NODE_DATA(
        float3, scratchNodesOffset, nodeIndex, SCRATCH_NODE_V0_OFFSET + (indices.x * SCRATCH_NODE_TRIANGLE_VERTEX_STRIDE));
    tri.v1 = FETCH_SCRATCH_NODE_DATA(
        float3, scratchNodesOffset, nodeIndex, SCRATCH_NODE_V0_OFFSET + (indices.y * SCRATCH_NODE_TRIANGLE_VERTEX_STRIDE));
    tri.v2 = FETCH_SCRATCH_NODE_DATA(
        float3, scratchNodesOffset, nodeIndex, SCRATCH_NODE_V0_OFFSET + (indices.z * SCRATCH_NODE_TRIANGLE_VERTEX_STRIDE));

    return tri;
}

#endif
