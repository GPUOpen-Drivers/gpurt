/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _ENCODE_HW_BVH_3_1_HLSL
#define _ENCODE_HW_BVH_3_1_HLSL

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
#include "../shadersClean/common/ShaderDefs.hlsli"

#define GC_DSTBUFFER
#define GC_DSTMETADATA
#define GC_SCRATCHBUFFER
#include "../shadersClean/build/BuildRootSignature.hlsli"

#define TASK_COUNTER_BUFFER   ScratchGlobal
#define TASK_COUNTER_OFFSET   (ShaderConstants.offsets.taskLoopCounters + TASK_LOOP_QBVH_COUNTER_OFFSET)
#define NUM_TASKS_DONE_OFFSET (ShaderConstants.offsets.taskLoopCounters + TASK_LOOP_QBVH_TASKS_DONE_OFFSET)
#include "TaskMacros.hlsl"

template<typename T>
T LoadInstanceDescBuffer(uint offset)
{
    return InstanceDescBuffer.Load<T>(offset);
}
#include "IndirectArgBufferUtils.hlsl"

#define MAX_ELEMENTS_PER_THREAD 8
#define MAX_LDS_ELEMENTS_PER_THREADGROUP (MAX_ELEMENTS_PER_THREAD * BUILD_THREADGROUP_SIZE)
groupshared uint SharedMem[MAX_LDS_ELEMENTS_PER_THREADGROUP];
uint GetSharedMem(uint index)
{
    return SharedMem[index];
}
void SetSharedMem(uint index, uint value)
{
    SharedMem[index] = value;
}

#include "BuildCommonScratch.hlsl"
#endif

#if GPURT_BUILD_RTIP3_1

#include "rtip3_1.hlsli"
#include "EncodeHwBvhCommon.hlsl"
#include "QBVH8Common.hlsl"
#include "../shadersClean/common/LaneGroup.hlsli"
#include "PrimitiveStructureEncoder3_1.hlsl"
#include "OrientedBoundingBoxes.hlsl"

namespace Gfx12
{

//=====================================================================================================================
// Returns true to enable a separate leaf compression pass over all children of one internal node.
bool EnableLeafCompressionPass()
{
    return (Settings.topLevelBuild == false) &&
           (Settings.geometryType == GEOMETRY_TYPE_TRIANGLES) &&
           ((Settings.primCompressionFlags & PrimCompFlags::MultiPrim) ||
            (Settings.maxPrimRangeSize > 2));
}

//=====================================================================================================================
// Returns true to enable the optimized wave parallel compression pass (only supports wave32 and maxPrimRangeSize<=4)
bool EnableWaveLeafCompressionPass()
{
#if BUILD_PARALLEL
    return false;
#else
    return EnableLeafCompressionPass();
#endif
}

//=====================================================================================================================
// Returns true to enable the legacy leaf compression pass (supports wave64 and larger prim range sizes)
bool EnableLegacyLeafCompressionPass()
{
#if BUILD_PARALLEL
    return EnableLeafCompressionPass();
#else
    return false;
#endif
}

//=====================================================================================================================
// Pack destination node index (128-byte chunk index) and child index in parent (maximum value of 7) into a single
// DWORD in global stack
uint PackStackDstNodeIdx(
    uint dstNodeIdx,
    uint indexInParent)
{
    return (dstNodeIdx << 3) | indexInParent;
}

//=====================================================================================================================
uint GetStackDstIdx(
    uint packedEntry)
{
    return (packedEntry >> 3);
}

//=====================================================================================================================
uint GetStackDstIdxInParent(
    uint packedEntry)
{
    return (packedEntry & bits(3));
}

//=====================================================================================================================
static uint GetNodeArrayBaseInLds(
    in uint localId, in uint maxNumChildren)
{
    return (localId * maxNumChildren);
}

//=====================================================================================================================
// Insert candidaate child in node array
static void InsertChildInNodeArray(
    in uint           baseLdsOffset,
    in uint           baseScratchNodesOffset,
    in uint           numPrimRefs,
    in uint           childScratchNodeIdx,
    inout_param(uint) nextLeafIdx,
    inout_param(uint) nextBoxIdx)
{
    uint childSlotIdx = 0;

    if (IsLeafOrIsCollapsed(childScratchNodeIdx, baseScratchNodesOffset, numPrimRefs))
    {
        childSlotIdx = --nextLeafIdx;
    }
    else
    {
        childSlotIdx = nextBoxIdx++;
    }

    SharedMem[baseLdsOffset + childSlotIdx] = childScratchNodeIdx;
}

//=====================================================================================================================
// Build child node array from BVH2 treelet root by using surface area heuristics
static void BuildChildNodeArray(
    in    uint                    localId,
    in    uint                    baseScratchNodesOffset,
    in    uint                    numPrimRefs,
    in    uint                    subtreeRootScratchNodeIdx,
    in    uint                    maxNumChildren,
    out_param(uint)               numBoxChildren,
    out_param(uint)               numLeafChildren)
{
    const uint baseLdsOffset  = GetNodeArrayBaseInLds(localId, maxNumChildren);

    const ScratchNode subtreeRootScratchNode = FetchScratchNode(baseScratchNodesOffset, subtreeRootScratchNodeIdx);

    uint nextBoxIdx = 0;
    uint nextLeafIdx = maxNumChildren;

    if (IsLeafOrIsCollapsed(subtreeRootScratchNodeIdx, baseScratchNodesOffset, numPrimRefs))
    {
        InsertChildInNodeArray(baseLdsOffset,
                               baseScratchNodesOffset,
                               numPrimRefs,
                               subtreeRootScratchNodeIdx,
                               nextLeafIdx,
                               nextBoxIdx);
    }
    else
    {
        // Insert left child into node array
        InsertChildInNodeArray(baseLdsOffset,
                               baseScratchNodesOffset,
                               numPrimRefs,
                               subtreeRootScratchNode.left_or_primIndex_or_instIndex,
                               nextLeafIdx,
                               nextBoxIdx);

        // Insert right child into node array
        InsertChildInNodeArray(baseLdsOffset,
                               baseScratchNodesOffset,
                               numPrimRefs,
                               subtreeRootScratchNode.right_or_geometryIndex,
                               nextLeafIdx,
                               nextBoxIdx);
    }

    // The following loop places box nodes at the front of the node array while all leaf children are placed
    // starting from the back of the node array with empty nodes in the middle. The two arrays are tracked
    // via two pointers, nextBoxIdx and nextLeafIdx
    //
    // [Box, Box, ..................., Leaf, Leaf]
    //           ^                   ^
    //       nextBoxIdx         nextLeafIdx
    //
    uint validChildCount = nextBoxIdx + (maxNumChildren - nextLeafIdx);

    // Cached candidate node data (left and right child indices)
    uint2 cachedCandidateData = uint2(0, 0);

    while (validChildCount < maxNumChildren)
    {
        float surfaceAreaMax = 0.0;
        uint candidateSlotIdx = INVALID_IDX;

        // Pick the largest surface area node from current node array of internal nodes.
        for (uint j = 0; j < nextBoxIdx; j++)
        {
            const uint candidateNodeIdx = SharedMem[baseLdsOffset + j];

            const ScratchNode candidateNode = FetchScratchNode(baseScratchNodesOffset, candidateNodeIdx);

            const float surfaceArea = candidateNode.sah_or_v2_or_instBasePtr.y;

            // Pick the node with larger surface area. If the nodes have identical surface area, pick the
            // last node for opening.
            if (surfaceArea >= surfaceAreaMax)
            {
                surfaceAreaMax = surfaceArea;
                candidateSlotIdx = j;

                cachedCandidateData.x = candidateNode.left_or_primIndex_or_instIndex;
                cachedCandidateData.y = candidateNode.right_or_geometryIndex;
            }
        }

        // If there are no valid nodes to open, exit loop
        if (candidateSlotIdx == INVALID_IDX)
        {
            break;
        }

        // Swap in the last box node in candidate slot and free last slot for opening. Below, C represents
        // candidate box node with the largest surface area.
        //
        // [B, C, B..................., L, L]
        //         ^                   ^
        //      nextBoxIdx         nextLeafIdx
        //
        // Last box node in node array gets swapped with candidate node and the box node pointer moves back
        // to allow insertion at the end of the list
        //
        // [B, B, C..................., L, L]
        //      ^                      ^
        //   nextBoxIdx            nextLeafIdx
        //
        nextBoxIdx--;
        SharedMem[baseLdsOffset + candidateSlotIdx] = SharedMem[baseLdsOffset + nextBoxIdx];

        InsertChildInNodeArray(
            baseLdsOffset, baseScratchNodesOffset, numPrimRefs,
            cachedCandidateData.x, nextLeafIdx, nextBoxIdx);

        InsertChildInNodeArray(
            baseLdsOffset, baseScratchNodesOffset, numPrimRefs,
            cachedCandidateData.y, nextLeafIdx, nextBoxIdx);

        // Update valid child count
        validChildCount = nextBoxIdx + (maxNumChildren - nextLeafIdx);
    }

    // Assign box and leaf node count in node array to output variables
    numBoxChildren  = nextBoxIdx;
    numLeafChildren = maxNumChildren - nextLeafIdx;

    // Shift leaf nodes left to cover up empty spaces if any in the child node array
    if (nextBoxIdx != nextLeafIdx)
    {
        for (uint i = 0; i < numLeafChildren; ++i)
        {
            SharedMem[baseLdsOffset + nextBoxIdx + i] = SharedMem[baseLdsOffset + nextLeafIdx + i];
        }
    }
}

//=====================================================================================================================
void WriteInstanceNode(
    in AccelStructOffsets offsets,
    in uint               numPrimRefs,
    in ScratchNode        scratchNode,
    in uint               scratchNodeIndex,
    in uint               nodeOffset,
    in uint               parentNodePointer)
{
    const uint instanceIndex = scratchNode.left_or_primIndex_or_instIndex;
    const uint nodePointer = PackNodePointer(NODE_TYPE_USER_NODE_INSTANCE, nodeOffset);
    InstanceDesc instanceDesc = LoadInstanceDesc(instanceIndex, LoadNumPrimAndOffset().primitiveOffset);

    const uint blasRootNodePointer = scratchNode.splitBox_or_nodePointer;
    const uint blasMetadataSize = scratchNode.numPrimitivesAndDoCollapse;
    const uint geometryType = ExtractScratchNodeGeometryType(scratchNode.packedFlags);

    CullIllegalInstances(blasMetadataSize, scratchNode, instanceDesc);

    WriteInstanceNode3_1(instanceDesc,
                         geometryType,
                         instanceIndex,
                         nodePointer,
                         blasRootNodePointer,
                         blasMetadataSize,
                         offsets,
                         parentNodePointer);

    // When rebraiding is disabled the destination index is just the instance index.
    const uint destIndex = (Settings.enableRebraid) ? (scratchNodeIndex - numPrimRefs + 1) : instanceIndex;

    DstBuffer.Store(offsets.primNodePtrs + (destIndex * sizeof(uint)), nodePointer);
}

//=====================================================================================================================
static void WriteLeafNode(
    in uint               scratchNodesScratchOffset,
    in AccelStructOffsets offsets,
    in uint               numPrimRefs,
    in ScratchNode        scratchNode,
    in uint               scratchNodeIdx,
    in uint               primStructIdx,
    in uint               nodeOffset,
    in uint               parentNodePointer)
{
    if (Settings.topLevelBuild)
    {
        Gfx12::WriteInstanceNode(offsets,
                                 numPrimRefs,
                                 scratchNode,
                                 scratchNodeIdx,
                                 nodeOffset,
                                 parentNodePointer);
    }
    else
    {
        bool isTri1Valid = false;
        bool isTri0Opaque = false;
        bool isTri1Opaque = false;
        uint primId0 = 0;
        uint primId1 = 0;
        uint geomId0 = 0;
        uint geomId1 = 0;
        TriangleData tri0 = (TriangleData) 0;
        TriangleData tri1 = (TriangleData) 0;

        const bool isQuadPrim =
            Settings.enableEarlyPairCompression && IsScratchNodeQuadPrimitive(scratchNode);

        if (isQuadPrim)
        {
            isTri1Valid = true;
            isTri0Opaque = IsOpaqueNode(scratchNode.packedFlags);
            isTri1Opaque = isTri0Opaque;
            primId0 = GetScratchNodePrimitiveIndex(scratchNode, 0);
            primId1 = GetScratchNodePrimitiveIndex(scratchNode, 1);
            geomId0 = ExtractScratchNodeGeometryIndex(scratchNode);
            geomId1 = geomId0;
            tri0 = GetScratchNodeQuadVertices(scratchNodesScratchOffset, scratchNodeIdx, 0);
            tri1 = GetScratchNodeQuadVertices(scratchNodesScratchOffset, scratchNodeIdx, 1);
        }
        else if (IsLeafNode(scratchNodeIdx, numPrimRefs) == false)
        {
            uint firstNodeIndex;
            ScratchNode scratchNodeTri0;

            uint secondNodeIndex;
            ScratchNode scratchNodeTri1;

            if (UsePrimIndicesArray())
            {
                firstNodeIndex = GetPrimRefIdx(FetchSortedPrimIndex(ShaderConstants.offsets.primIndicesSorted,
                                                                    scratchNode.sortedPrimIndex),
                                               numPrimRefs);
                scratchNodeTri0 = FetchScratchNode(scratchNodesScratchOffset, firstNodeIndex);

                secondNodeIndex = GetPrimRefIdx(FetchSortedPrimIndex(ShaderConstants.offsets.primIndicesSorted,
                                                                     scratchNode.sortedPrimIndex + 1),
                                                numPrimRefs);
                scratchNodeTri1 = FetchScratchNode(scratchNodesScratchOffset, secondNodeIndex);
            }
            else if (CollapseAnyPairs())
            {
                firstNodeIndex = scratchNode.left_or_primIndex_or_instIndex;
                scratchNodeTri0 = FetchScratchNode(scratchNodesScratchOffset, firstNodeIndex);

                secondNodeIndex = scratchNode.right_or_geometryIndex;
                scratchNodeTri1 = FetchScratchNode(scratchNodesScratchOffset, secondNodeIndex);
            }
            else
            {
                firstNodeIndex = scratchNode.left_or_primIndex_or_instIndex;
                scratchNodeTri0 = FetchScratchNode(scratchNodesScratchOffset, firstNodeIndex);

                secondNodeIndex = scratchNodeTri0.parent;
                scratchNodeTri1 = FetchScratchNode(scratchNodesScratchOffset, secondNodeIndex);
            }

            isTri1Valid = true;
            isTri0Opaque = IsOpaqueNode(scratchNodeTri0.packedFlags);
            isTri1Opaque = IsOpaqueNode(scratchNodeTri1.packedFlags);
            primId0 = scratchNodeTri0.left_or_primIndex_or_instIndex;
            primId1 = scratchNodeTri1.left_or_primIndex_or_instIndex;
            geomId0 = ExtractScratchNodeGeometryIndex(scratchNodeTri0);
            geomId1 = ExtractScratchNodeGeometryIndex(scratchNodeTri1);
            tri0.v0 = scratchNodeTri0.bbox_min_or_v0;
            tri0.v1 = scratchNodeTri0.bbox_max_or_v1;
            tri0.v2 = scratchNodeTri0.sah_or_v2_or_instBasePtr;
            tri1.v0 = scratchNodeTri1.bbox_min_or_v0;
            tri1.v1 = scratchNodeTri1.bbox_max_or_v1;
            tri1.v2 = scratchNodeTri1.sah_or_v2_or_instBasePtr;
        }
        else
        {
            isTri0Opaque = IsOpaqueNode(scratchNode.packedFlags);
            primId0 = scratchNode.left_or_primIndex_or_instIndex;
            geomId0 = ExtractScratchNodeGeometryIndex(scratchNode);
            tri0.v0 = scratchNode.bbox_min_or_v0;
            tri0.v1 = scratchNode.bbox_max_or_v1;
            tri0.v2 = scratchNode.sah_or_v2_or_instBasePtr;
        }

        PrimStruct3_1::WritePairPrimStruct(tri0,
                                           isTri0Opaque,
                                           primId0,
                                           geomId0,
                                           isTri1Valid,
                                           tri1,
                                           isTri1Opaque,
                                           primId1,
                                           geomId1,
                                           nodeOffset);

        DstBuffer.Store(offsets.primNodePtrs + (primStructIdx * sizeof(uint)), nodeOffset);
    }
}

//=====================================================================================================================
static void CompressLeafChildren(
    uint               scratchNodesScratchOffset,
    AccelStructOffsets offsets,
    uint               parentBoxOffset,
    uint               primStructOffset,
    uint               numLeafChildren,
    uint               numBoxChildren,
    uint               numPrimRefs,
    uint               ldsOffset,
    bool               doRebraid)
{
    // Filter node support assumes rebraid root children are not compressed
    const bool isRebraidRootNode = doRebraid;
    const bool compressMultiple =
        (Settings.primCompressionFlags & PrimCompFlags::MultiPrim) && (isRebraidRootNode == false);
    const bool enableTrailingZeroCompression = Settings.primCompressionFlags & PrimCompFlags::EnableTrailingZeroAndPrefix;

    // TODO: Allow more pairs. Update can only handle one:
    // - Parent pointer logic cannot handle multiple prims per slot in various places
    // - Vertex data update does not yet support multiple prims
    // - ExtractNodePointerOffset doesn't work with 4 bit node types
    const uint maxPairs = compressMultiple ? 8 : 1;

    TrianglePairDesc pairDescs[9];
    float3 verts[16];

    uint3 prefixSource = uint3(0, 0, 0);
    uint3 planeDiffs = uint3(0, 0, 0);
    uint  planeUnion = 0;

    uint primIdAnchor = 0;
    uint primIdDiff = 0;
    uint geoIdAnchor = 0;
    uint geoIdDiff = 0;

    uint uniqueVertexCount = 0;
    uint triPairDescCount = 0;

    PrimitiveStructure primStruct;

    uint geometryInfoOffset = 0;

    uint pairCount = 0;

    const uint parentBoxPtr = PackNodePointer(NODE_TYPE_BOX_QUANTIZED_BVH8, parentBoxOffset);
    QBVH8::WriteLeafNodeBaseOffset(parentBoxOffset, primStructOffset);

    uint3 planeDiffsBackup;
    uint uniqueVertexCountBackup;
    uint primIdDiffBackup;
    uint geoIdDiffBackup;
    uint triPairDescCountBackup;
    uint planeUnionBackup;

    uint nodeIndex = INVALID_IDX;
    uint leafChildIdx = 0;
    uint rangeNextNode = INVALID_IDX;
    uint firstNodeInPair = INVALID_IDX;
    bool isTri1 = false;

    uint rangePacketCount = 0;
    uint rangeStartPair = 0;
    uint lastValidChildInfoOffset = 0;

    bool firstPairInRange = true;

    const bool usePrimListArray = UsePrimIndicesArray();
    uint currentIndexInArray;
    uint numPrimsInArray;
    uint primArrayStartIndex;
    uint primIndexFirstInPair;

    while ((leafChildIdx < numLeafChildren) || (rangeNextNode != INVALID_IDX))
    {
        if (rangeNextNode != INVALID_IDX)
        {
            nodeIndex = rangeNextNode;
        }
        else
        {
            nodeIndex = SharedMem[ldsOffset + leafChildIdx];

            currentIndexInArray = 0;
            numPrimsInArray = 1;

            // Check for collapsed primitive range
            if (IsLeafNode(nodeIndex, numPrimRefs) == false)
            {
                const ScratchNode scratchNode = FetchScratchNode(scratchNodesScratchOffset, nodeIndex);

                GPU_ASSERT(scratchNode.numPrimitivesAndDoCollapse & 1);

                if (usePrimListArray)
                {
                    numPrimsInArray = scratchNode.numPrimitivesAndDoCollapse >> 1;
                    primArrayStartIndex = scratchNode.sortedPrimIndex;

                    nodeIndex = GetPrimRefIdx(FetchSortedPrimIndex(ShaderConstants.offsets.primIndicesSorted,
                                                                   primArrayStartIndex),
                                                                   numPrimRefs);
                }
                else
                {
                    // Set the next node index to the head pointer of the range
                    nodeIndex = scratchNode.left_or_primIndex_or_instIndex;
                }
            }

            isTri1 = false;
            firstPairInRange = true;
        }

        GPU_ASSERT(IsLeafNode(nodeIndex, numPrimRefs));

        const ScratchNode scratchNode = FetchScratchNode(scratchNodesScratchOffset, nodeIndex);
        float3 v0 = scratchNode.bbox_min_or_v0;
        float3 v1 = scratchNode.bbox_max_or_v1;
        float3 v2 = scratchNode.sah_or_v2_or_instBasePtr;

        uint primId = scratchNode.left_or_primIndex_or_instIndex;
        uint geomId = ExtractScratchNodeGeometryIndex(scratchNode);
        const bool isOpaque = IsOpaqueNode(scratchNode.packedFlags);

        if (Settings.enableEarlyPairCompression)
        {
            if (IsScratchNodeQuadPrimitive(scratchNode))
            {
                const uint triangleIndex = isTri1 ? 1 : 0;

                const TriangleData tri =
                    GetScratchNodeQuadVertices(scratchNodesScratchOffset, nodeIndex, triangleIndex);

                v0 = tri.v0;
                v1 = tri.v1;
                v2 = tri.v2;

                primId = GetScratchNodePrimitiveIndex(scratchNode, triangleIndex);

                // Re-queue current node for extracting triangle 1.
                rangeNextNode = isTri1 ? scratchNode.parent : nodeIndex;
            }
            else
            {
                rangeNextNode = scratchNode.parent;
            }
        }
        else
        {
            if (usePrimListArray)
            {
                if ((currentIndexInArray + 1) == numPrimsInArray)
                {
                    rangeNextNode = INVALID_IDX;
                }
                else
                {
                    currentIndexInArray++;

                    rangeNextNode = GetPrimRefIdx(FetchSortedPrimIndex(ShaderConstants.offsets.primIndicesSorted,
                                                                       primArrayStartIndex + currentIndexInArray),
                                                                       numPrimRefs);
                }
            }
            else
            {
                rangeNextNode = scratchNode.parent;
            }
        }

        if (triPairDescCount == 0)
        {
            prefixSource = asuint(v0);
            primIdAnchor = primId;
            primIdDiff = 0;
            geoIdAnchor = geomId;
            geoIdDiff = 0;
            planeDiffs = 0;
            planeUnion = 0;
            uniqueVertexCount = 0;
        }

        uint index0 = 0xFF;
        uint index1 = 0xFF;
        uint index2 = 0xFF;
        uint3 diff0 = asuint(v0) ^ prefixSource;
        uint3 diff1 = asuint(v1) ^ prefixSource;
        uint3 diff2 = asuint(v2) ^ prefixSource;

        // State used to generate the meshlet if triangle *cannot* be added
        if (isTri1 == false)
        {
            planeDiffsBackup        = planeDiffs;
            uniqueVertexCountBackup = uniqueVertexCount;
            primIdDiffBackup        = primIdDiff;
            geoIdDiffBackup         = geoIdDiff;
            triPairDescCountBackup  = triPairDescCount;
            planeUnionBackup        = planeUnion;
            firstNodeInPair         = nodeIndex;
            primIndexFirstInPair    = currentIndexInArray - 1;
        }

        for (uint i = 0; i < uniqueVertexCount; i++)
        {
            index0 = (all(v0 == verts[i])) ? i : index0;
            index1 = (all(v1 == verts[i])) ? i : index1;
            index2 = (all(v2 == verts[i])) ? i : index2;
        }

        if (index0 == 0xFF)
        {
            index0 = uniqueVertexCount;
            verts[index0] = v0;
            planeDiffs = planeDiffs | diff0;
            planeUnion = planeUnion | asuint(v0.x) | asuint(v0.y) | asuint(v0.z);
            uniqueVertexCount++;
        }

        if (index1 == 0xFF)
        {
            index1 = uniqueVertexCount;
            verts[index1] = v1;
            planeDiffs = planeDiffs | diff1;
            planeUnion = planeUnion | asuint(v1.x) | asuint(v1.y) | asuint(v1.z);
            uniqueVertexCount++;
        }

        if (index2 == 0xFF)
        {
            index2 = uniqueVertexCount;
            verts[index2] = v2;
            planeDiffs = planeDiffs | diff2;
            planeUnion = planeUnion | asuint(v2.x) | asuint(v2.y) | asuint(v2.z);
            uniqueVertexCount++;
        }

        geoIdDiff  |= (geoIdAnchor ^ geomId);
        primIdDiff |= (primIdAnchor ^ primId);

        bool pairDone = false;

        if (isTri1)
        {
            GPU_ASSERT(triPairDescCount > 0);

            pairDescs[triPairDescCount-1].InitTri1(
                uint3(index0, index1, index2), isOpaque, primId, geomId);

            // Last pair in the range
            if (rangeNextNode == INVALID_IDX)
            {
                pairDescs[triPairDescCount-1].SetPrimRangeStopBit(true);
            }

            pairDone = true;
        }
        else
        {
            pairDescs[triPairDescCount].Init();
            pairDescs[triPairDescCount].InitTri0(
                uint3(index0, index1, index2), isOpaque, primId, geomId);

            if (rangeNextNode == INVALID_IDX)
            {
                // No additional triangle to pair
                pairDescs[triPairDescCount].InvalidateTri1();
                pairDescs[triPairDescCount].SetPrimRangeStopBit(true);
                pairDone = true;
            }

            triPairDescCount++;
        }

        const uint triPairIdx = triPairDescCount - 1;
        const uint nodeType   = (triPairIdx >= 4) ? (triPairIdx + 4) : triPairIdx;

        if (pairDone == false)
        {
            // Wait until the full pair is setup before encoding
            isTri1 = true;
            continue;
        }

        isTri1 = false;

        INIT_VAR(UnpackedPrimStructHeader, triStructHeader);

        bool valid = TryInitTriStructHeader(asfloat(prefixSource),
                                            planeDiffs,
                                            planeUnion,
                                            primIdDiff,
                                            geoIdDiff,
                                            primIdAnchor,
                                            geoIdAnchor,
                                            uniqueVertexCount,
                                            triPairDescCount,
                                            false,
                                            enableTrailingZeroCompression,
                                            triStructHeader);

        if (triPairDescCount > maxPairs)
        {
            valid = false;
        }

        if (valid)
        {
            if (firstPairInRange)
            {
                // Update range of the last child
                if (rangePacketCount > 0)
                {
                    GPU_ASSERT(rangePacketCount < 16);
                    DstBuffer.InterlockedOr(lastValidChildInfoOffset, rangePacketCount << 28);
                    rangePacketCount = 0;
                }

                const uint childIdx = numBoxChildren + leafChildIdx;

                // Update child type in the box node
                const uint childInfoOffset =
                    parentBoxOffset +
                    QUANTIZED_BVH8_NODE_OFFSET_CHILD_INFO_0 +
                    (childIdx * QUANTIZED_NODE_CHILD_INFO_STRIDE) +
                    QUANTIZED_NODE_CHILD_INFO_OFFSET_MAXY_MAXZ_NODE_TYPE_AND_RANGE;

                lastValidChildInfoOffset = childInfoOffset;

                DstBuffer.InterlockedOr(childInfoOffset, nodeType << 24);
            }

            if (rangeNextNode == INVALID_NODE)
            {
                leafChildIdx++;
            }

            firstPairInRange = false;
        }
        else
        {
            valid = TryInitTriStructHeader(asfloat(prefixSource),
                                           planeDiffsBackup,
                                           planeUnionBackup,
                                           primIdDiffBackup,
                                           geoIdDiffBackup,
                                           primIdAnchor,
                                           geoIdAnchor,
                                           uniqueVertexCountBackup,
                                           triPairDescCountBackup,
                                           false,
                                           enableTrailingZeroCompression,
                                           triStructHeader);

            GPU_ASSERT(valid);

            primStruct.Encode16(triStructHeader,
                                verts,
                                uniqueVertexCountBackup,
                                pairDescs,
                                triPairDescCountBackup,
                                primStructOffset);

            const uint primStructIdx = IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET, 1);
            DstBuffer.Store(offsets.primNodePtrs + (primStructIdx * sizeof(uint)), primStructOffset);

            rangePacketCount++;

            triPairDescCount = 0;

            // Reset back to the start of the pair that failed
            rangeNextNode = firstNodeInPair;

            if (usePrimListArray)
            {
                currentIndexInArray = primIndexFirstInPair;
            }

            primStructOffset += QBVH8::NodeSizeInBytes;
        }
    }

    if (triPairDescCount > 0)
    {
        INIT_VAR(UnpackedPrimStructHeader, triStructHeader);

        bool valid = TryInitTriStructHeader(asfloat(prefixSource),
                                            planeDiffs,
                                            planeUnion,
                                            primIdDiff,
                                            geoIdDiff,
                                            primIdAnchor,
                                            geoIdAnchor,
                                            uniqueVertexCount,
                                            triPairDescCount,
                                            false,
                                            enableTrailingZeroCompression,
                                            triStructHeader);

        GPU_ASSERT(valid);

        primStruct.Encode16(triStructHeader,
                            verts,
                            uniqueVertexCount,
                            pairDescs,
                            triPairDescCount,
                            primStructOffset);

        const uint primStructIdx = IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET, 1);
        DstBuffer.Store(offsets.primNodePtrs + (primStructIdx * sizeof(uint)), primStructOffset);
    }
}

//=====================================================================================================================
static uint3 ComputePackedChildInfo(
    in uint          scratchNodesScratchOffset,
    in ScratchNode   childScratchNode,
    in uint          childScratchNodeIdx,
    in uint          nodeType,
    in uint          nodeRangeLength,
    in float3        rcpExponents,
    in float3        origin,
    in uint          numPrimRefs)
{
    const bool isLeafNode = IsLeafNode(childScratchNodeIdx, numPrimRefs);
    BoundingBox childBounds = GetScratchNodeBoundingBoxTS(scratchNodesScratchOffset, isLeafNode, childScratchNode);

    const uint instanceMask =
        ExtractScratchNodeInstanceMask(childScratchNode.packedFlags);

    const uint boxNodeFlags =
        ExtractScratchNodeBoxFlags(childScratchNode.packedFlags);

    const uint3 childInfo = QBVH8::ComputePackedChildInfo(childBounds,
                                                          instanceMask,
                                                          boxNodeFlags,
                                                          nodeType,
                                                          nodeRangeLength,
                                                          rcpExponents,
                                                          origin);
    return childInfo;
}

//=====================================================================================================================
// Write intersectable instance node as a filter node with 4 distinct child bounds sharing the same root node.
// This function prebuilds the filter node and stores it in the BLAS metadata. It is copied directly to the TLAS when
// an instance is not rebraided.
static void WriteBlasInstanceFilterNode(
    in uint   scratchNodesScratchOffset,
    in uint   numPrimRefs,
    in uint   rootNodeIndex,
    in float3 origin,
    in uint3  exponents,
    in float3 rcpExponents)
{
    // For single primitive case, skip building node array and just push the lone leaf node to
    // the child node array
    uint numBoxChildren  = 0;
    uint numLeafChildren = 0;

    // For filter nodes, we just build 4 bounding boxes from the root node
    const uint baseLdsOffset = GetNodeArrayBaseInLds(0, 4);
    BuildChildNodeArray(0,
                        scratchNodesScratchOffset,
                        numPrimRefs,
                        rootNodeIndex,
                        4,
                        numBoxChildren,
                        numLeafChildren);

    // The root of the BVH2 treelet contains the combined bounds of the child nodes in the node array.
    // Compute the common exponent from the root node bounds
    const uint validChildCount = numBoxChildren + numLeafChildren;

    // Origin
    const uint baseOffset = ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET;
    DstMetadata.Store3(baseOffset, asuint(origin));

    // Packed exponents, valid child count and child index in parent node
    const uint packedExpChildIdxAndCount =
        QuantizedBVH8BoxNode::PackExpChildIdxAndCount(exponents, 0, 0, 4);

    DstMetadata.Store(baseOffset + QUANTIZED_BVH4_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT,
                      packedExpChildIdxAndCount);

    for (uint i = 0; i < validChildCount; ++i)
    {
        const uint childScratchNodeIdx = SharedMem[baseLdsOffset + i];

        const ScratchNode childScratchNode =
            FetchScratchNode(scratchNodesScratchOffset, childScratchNodeIdx);

        const bool isLeafNode = IsLeafNode(childScratchNodeIdx, numPrimRefs);

        const BoundingBox childBounds =
            GetScratchNodeBoundingBoxTS(scratchNodesScratchOffset, isLeafNode, childScratchNode);

        const UintBoundingBox quantBounds = ComputeQuantizedBounds(childBounds, origin, rcpExponents, 12);

        const uint3 childInfo = GetInstanceChildInfo(i, CreateRootNodePointer3_1(), quantBounds.min, quantBounds.max);

        DstMetadata.Store3(baseOffset + GetBvh4ChildInfoOffset(i), childInfo);
    }

    // Encode remaining child slots as invalid
    for (uint i = validChildCount; i < 4; ++i)
    {
        const uint3 childInfo = GetInstanceChildInfo(i, CreateRootNodePointer3_1(), 0xFFFFFFFF.xxx, 0.xxx);
        DstMetadata.Store3(baseOffset + GetBvh4ChildInfoOffset(i), childInfo);
    }
}

//=====================================================================================================================
// Box splitting / Child reuse
static void BoxSplitting(
    LaneGroup                laneGroup,
    in    uint               scratchNodesScratchOffset,
    in    uint               currNodeOffset,
    in    uint               indexInParent,
    in    AccelStructOffsets offsets,
    in    uint               baseLeafNodeOffset,
    in    uint               maxNumChildren,
    in    uint               numLeafChildren,
    in    uint               numBoxChildren,
    in    uint               boxOrLeafChildIndex,
    in    ScratchNode        childScratchNode,
    in    uint               numEmptySlots,
    in    float3             rcpExponents,
    in    uint3              exponents,
    in    BoundingBox        parentBbox,
    in    bool               isLeaf)
{
    uint maxNodeLane = 0;

    if (Settings.topLevelBuild && (numEmptySlots > 0) && (Settings.boxSplittingFlags & BoxSplittingFlags::Instance))
    {
        const NumPrimAndOffset numPrimAndOffset = LoadNumPrimAndOffset();

        // Make sure the compressed box node is encoded
        DeviceMemoryBarrier();

        // Pick the instances in decreasing SAH order
        if (isLeaf)
        {
            // Flag to indicate if the instance node in the current lane has been processed.
            bool processed = false;

            // Note, boxOrLeafChildIndex is only the index among the leaf children here.
            uint childInfoIndex = numBoxChildren + boxOrLeafChildIndex;

            BoundingBox instanceBounds[4];

            const uint intersectableInstanceOffset =
                baseLeafNodeOffset + (boxOrLeafChildIndex * QBVH8::NodeSizeInBytes) + sizeof(HwInstanceTransformNode);
            const uint4 dw = DstBuffer.Load4(intersectableInstanceOffset);

            // Fetch origin from intersectable instance node
            const float3 instanceOrigin = float3(asfloat(dw.xyz));

            // Decode valid child count from intersectable instance node
            const uint numValidInstanceBounds = ExtractBvh4ValidChildCount(dw.w);

            // Extract packed exponents from intersectable instance node
            uint3 instanceExponents;
            instanceExponents.x = bitFieldExtract(dw.w, 0, 8);
            instanceExponents.y = bitFieldExtract(dw.w, 8, 8);
            instanceExponents.z = bitFieldExtract(dw.w, 16, 8);

            // Decode intersectable instance bounds
            for (uint j = 0; j < 4; ++j)
            {
                const uint childInfoOffset = intersectableInstanceOffset + GetBvh4ChildInfoOffset(j);
                const ChildInfo childInfo = DstBuffer.Load<ChildInfo>(childInfoOffset);
                if (childInfo.Valid())
                {
                    instanceBounds[j] = childInfo.DecodeBounds(instanceOrigin, instanceExponents);
                }
                else
                {
                    instanceBounds[j] = InvalidBoundingBox;
                }
            }

            uint numFinalBounds = numValidInstanceBounds;

            const float area =
                ComputeBoxSurfaceArea(GetScratchNodeBoundingBoxTS(scratchNodesScratchOffset, true, childScratchNode));

            while ((numEmptySlots > 0) && (processed == false))
            {
                const float maxArea = laneGroup.Max(area);
                const bool isMaxAreaNode = (area == maxArea);
                maxNodeLane = laneGroup.GetFirstActiveLaneIndex(isMaxAreaNode); // Get the lane index of the max node

                // Do box splitting on the instance node with max area.
                if (laneGroup.laneIndex == maxNodeLane)
                {
                    // Mark this instance node as processed.
                    processed = true;

                    // Merge intersectable instance bounds if there are not enough empty slots available.
                    // TODO: Intersectable instance bounds sorted in increasing order of SAH would be ideal for
                    //       merging here

                    if ((numEmptySlots == 1) && (numValidInstanceBounds == 4))
                    {
                        // Merge [2:3]
                        instanceBounds[2] = CombineAABB(instanceBounds[2], instanceBounds[3]);

                        numFinalBounds--;
                    }

                    if (((numEmptySlots == 1) && (numValidInstanceBounds >= 3)) ||
                        ((numEmptySlots == 2) && (numValidInstanceBounds == 4)))
                    {
                        // Merge [0:1]
                        instanceBounds[0] = CombineAABB(instanceBounds[0], instanceBounds[1]);

                        numFinalBounds--;

                        // Copy [2:3] to [1:2] to remove gaps
                        instanceBounds[1] = instanceBounds[2];
                        instanceBounds[2] = instanceBounds[3];
                    }

                    // Make space for the box splitting child infos
                    for (uint j = maxNumChildren - numEmptySlots - 1; j > childInfoIndex; --j)
                    {
                        const uint srcChildInfoOffset = currNodeOffset + GetChildInfoOffset(j);
                        const uint dstChildInfoOffset = currNodeOffset + GetChildInfoOffset(j + numFinalBounds - 1);
                        const uint3 childInfo = DstBuffer.Load3(srcChildInfoOffset);
                        DstBuffer.Store3(dstChildInfoOffset, childInfo);
                    }

                    const uint instanceIndex = childScratchNode.left_or_primIndex_or_instIndex;
                    const uint instanceMask = ExtractScratchNodeInstanceMask(childScratchNode.packedFlags);
                    const uint boxNodeFlags = ExtractScratchNodeBoxFlags(childScratchNode.packedFlags);

                    const InstanceDesc instanceDesc = LoadInstanceDesc(instanceIndex, numPrimAndOffset.primitiveOffset);

                    // Apply objectToWorld transform to intersectable instance bounds and write to child infos
                    for (uint j = 0; j < numFinalBounds; ++j)
                    {
                        instanceBounds[j] = TransformBoundingBox(instanceBounds[j], instanceDesc.Transform);

                        // Child reuse has node range length 0
                        const uint nodeRangeLength = (j < (numFinalBounds - 1)) ? 0 : 1;

                        const uint3 childInfo = QBVH8::ComputePackedChildInfo(instanceBounds[j],
                                                                              instanceMask,
                                                                              boxNodeFlags,
                                                                              NODE_TYPE_USER_NODE_INSTANCE,
                                                                              nodeRangeLength,
                                                                              rcpExponents,
                                                                              parentBbox.min);

                        QBVH8::WritePackedChildInfo(currNodeOffset, childInfoIndex + j, childInfo);
                    }

                    numEmptySlots -= numFinalBounds - 1;
                }

                const uint maxNodeChildInfoIndex = laneGroup.Broadcast(childInfoIndex, maxNodeLane);
                const uint maxNodeNumFinalBounds = laneGroup.Broadcast(numFinalBounds, maxNodeLane);

                // Update any child index that were shifted over to make space above.
                if (childInfoIndex > maxNodeChildInfoIndex)
                {
                    childInfoIndex += maxNodeNumFinalBounds - 1;
                }

                numEmptySlots = laneGroup.Broadcast(numEmptySlots, maxNodeLane);

                // Update the child count on the last iteration.
                if (((numEmptySlots == 0) || (laneGroup.BallotCount(processed == false) == 0)) &&
                    laneGroup.IsFirstActiveLane())
                {
                    QBVH8::WritePackedExpChildIdxAndCount(currNodeOffset,
                                                          exponents,
                                                          indexInParent,
                                                          maxNumChildren - numEmptySlots);
                }
            }
        }
    }
}

//=====================================================================================================================
// Write the child scratch indices for processing in the primitive compression pass
static void WritePrimCompBatches3_1(
    LaneGroup   laneGroup,
    uint        rootNodeIndex,
    uint        scratchNodesScratchOffset,
    uint        parentScratchNodeIdx,
    uint        currNodeAddr,
    uint        numBoxChildren,
    uint        numLeafChildren,
    uint        numChildPrims,
    uint        numPrimRefs,
    uint        nodeIndex,
    uint        childIdx,
    bool        isLeaf)
{
    uint offset = CalcScratchNodeOffset(scratchNodesScratchOffset, parentScratchNodeIdx);

    if (laneGroup.IsFirstLane())
    {
        // numPrimRefs represents the number of active triangles in the AS and not the number of
        // primitive (leaf) references in the BVH2. In case of a primitive range that spans all triangles
        // in the AS, numPrimRefs is > 1. The code here assumes that in that scenario, a true BVH2 with N
        // leaf references still exists. The assert below when triggered indicates that this assumption has changed.
        if (numPrimRefs == 1)
        {
           GPU_ASSERT(numChildPrims == 1);
           GPU_ASSERT(rootNodeIndex == nodeIndex);
        }

        // For single primitive AS there is no parent scratch node to use, hence we always queue the root scratch node
        // itself to the compression stack and handle it in the compression pass
        if (numPrimRefs == 1)
        {
            // Add single triangles at the end of the buffer
            ScratchGlobal.Store(ShaderConstants.offsets.qbvhGlobalStackPtrs + STACK_PTRS_PRIM_COMP_SINGLE_COUNT, 1);
            const uint primCompBatchOffset = ShaderConstants.offsets.primCompNodeIndicesEnd - sizeof(uint);
            ScratchGlobal.Store(primCompBatchOffset, rootNodeIndex);
        }
        else
        {
            // Pack numBoxChildren and numLeafChildren bits in the 128-byte aligned currNodeAddr
            // numBoxChildren    :  3 // Maximum value is 7 since we only queue internal nodes with at least one leaf node
            // numLeafChildren   :  4 // Maximum valid is 8 as an BVH8 internal node can have maximum of 8 leaf children
            // currentNodeOffset : 25 // 128-byte aligned node offset. Lower 7 bits are guaranteed to be 0
            //
            const uint packedParentInfo = PackCompBatchParentInfo(currNodeAddr, numBoxChildren, numLeafChildren);

            // Store packedParentInfo followed by leaf node indices in parent scratch node
            ScratchGlobal.Store(offset, packedParentInfo);

            const uint stackPtrOffset = numChildPrims > 1 ? STACK_PTRS_PRIM_COMP_BATCH_COUNT : STACK_PTRS_PRIM_COMP_SINGLE_COUNT;
            uint stackIndex;
            ScratchGlobal.InterlockedAdd(ShaderConstants.offsets.qbvhGlobalStackPtrs + stackPtrOffset, 1, stackIndex);

            const uint primCompBatchOffset = (numChildPrims > 1) ?
                      ShaderConstants.offsets.primCompNodeIndices + (stackIndex * sizeof(uint)) :
                      ShaderConstants.offsets.primCompNodeIndicesEnd - ((stackIndex + 1) * sizeof(uint));

            ScratchGlobal.Store(primCompBatchOffset, parentScratchNodeIdx);
        }
    }

    if ((numPrimRefs != 1) && isLeaf)
    {
        offset += sizeof(uint) * (childIdx + 1);
        ScratchGlobal.Store(offset, nodeIndex);
    }
}

//=====================================================================================================================
// Allocation of compressed leaf nodes is deferred to the compression pass. Leaf nodes in the root of a rebraided BLAS
// are an exception. They must be allocated near the beginning of the tree to support rebraid. Some small unused space
// will be left between the end of the compressed data and the next node.
uint GetNumInlinePrimStructs(
    LaneGroup   laneGroup,
    uint        scratchNodesScratchOffset,
    uint        numPrimRefs,
    uint        numLeafChildren,
    uint        childScratchNodeIdx,
    bool        isRebraidRootNode,
    bool        isLeaf)
{
    uint numInlinePrimStructs = 0;

    if (EnableWaveLeafCompressionPass() && (isRebraidRootNode == false))
    {
        numInlinePrimStructs = 0;
    }
    else if (Settings.maxPrimRangeSize <= 2)
    {
        numInlinePrimStructs = numLeafChildren;
    }
    else
    {
        uint numPrimStructs = 0;
        if (isLeaf)
        {
            const uint rangeNumPrims = FetchScratchNodeNumPrimitives(
                scratchNodesScratchOffset, childScratchNodeIdx, IsLeafNode(childScratchNodeIdx, numPrimRefs));

            // Minimum two triangles per prim struct in each range
            numPrimStructs = RoundUpQuotient(rangeNumPrims, 2);
        }
        numInlinePrimStructs = laneGroup.Sum(numPrimStructs);
    }

    return numInlinePrimStructs;
}

//======================================================================================================================
void InitObbRefitStackPtrs()
{
    if (IsObbEnabled())
    {
        ObbRefitStackPtrs obbRefitStackPtrs = (ObbRefitStackPtrs)0;
        obbRefitStackPtrs.groupCountY = 1;
        obbRefitStackPtrs.groupCountZ = 1;

        ScratchGlobal.Store<ObbRefitStackPtrs>(ShaderConstants.offsets.obbRefitStackPtrs, obbRefitStackPtrs);
    }
}

//=====================================================================================================================
static void EncodeCompressedBoxNode(
    LaneGroup                laneGroup,
    in    uint               rootNodeIndex,
    in    uint               scratchNodesScratchOffset,
    in    uint               localId,
    in    uint               parentScratchNodeIdx,
    in    uint               bvhNodeDstIdx,
    in    uint               currNodePtr,
    in    uint               currNodeOffset,
    in    uint               indexInParent,
    in    AccelStructOffsets offsets,
    in    uint               maxNumChildren,
    in    uint               numPrimRefs,
    in    bool               isRebraidRootNode)
{

    uint baseDstBoxIdx      = 0;
    uint numLeafChildren    = 0;
    uint numBoxChildren     = 0;
    uint baseLeafNodeOffset = 0;
    uint baseBoxNodeOffset  = 0;
    float3 rcpExponents     = 0;
    uint3 exponents;
    BoundingBox parentBbox;

    bool isBox      = false;
    bool needToLoad = false;
    bool isLeaf     = false;
    uint boxOrLeafChildIndex;
    uint scratchNodeIdx  = INVALID_IDX; // Start with an invalid index for each thread.
    float scratchNodeArea    = -1.0f;   // Mark area as invalid initially.
    ScratchNode scratchNode;

    float rootSA;

    // Initialization for the first lane
    if (laneGroup.IsFirstLane())
    {
        scratchNodeIdx = parentScratchNodeIdx;
        needToLoad = true;

        if (isRebraidRootNode)
        {
            scratchNode = FetchScratchNode(scratchNodesScratchOffset, scratchNodeIdx);
            scratchNodeArea = scratchNode.sah_or_v2_or_instBasePtr.y;
            rootSA = scratchNodeArea;
        }
    }

    // This bitfield is an array of 2 bit indices indicating which of the first 4 boxes was split to produce a given
    // box. It is initialized to [0, 1, 2, 3] for the first 4 boxes. After the first 4 boxes are generated, each
    // additional box takes the index of the box it was split from.
    uint packedSourceBoxes = 0b11100100;

    const uint maxNumRebraidChildren = 4;
    bool doRebraid = false;

    while (1)
    {
        const uint firstInvalidLane = laneGroup.GetFirstActiveLaneIndex(scratchNodeIdx == INVALID_IDX);

        // Load node data if needed
        if (needToLoad)
        {
            scratchNode = FetchScratchNode(scratchNodesScratchOffset, scratchNodeIdx);

            bool isCollapse = scratchNode.numPrimitivesAndDoCollapse & 1;

            if (CollapseAnyPairs())
            {
                isCollapse = (scratchNode.numPrimitivesAndDoCollapse >> 1) == 2;
            }

            isLeaf = (IsLeafNode(scratchNodeIdx, numPrimRefs) ||
                      ((Settings.topLevelBuild == false) && isCollapse));
            isBox  = !isLeaf;
            scratchNodeArea = isLeaf ? -1.0f : scratchNode.sah_or_v2_or_instBasePtr.y;

            needToLoad = false;
        }

        numBoxChildren = laneGroup.BallotCount(isBox);

        // decide to rebraid
        if (isRebraidRootNode && (firstInvalidLane >= 2) && (firstInvalidLane <= 4) && (doRebraid == false))
        {
            float surfaceArea = 0;

            if (scratchNodeIdx != INVALID_IDX)
            {
                const bool isLeafNode = IsLeafNode(scratchNodeIdx, numPrimRefs);
                const BoundingBox box = GetScratchNodeBoundingBoxTS(scratchNodesScratchOffset, isLeafNode, scratchNode);
                surfaceArea = ComputeBoxSurfaceArea(box);
            }

            const float childSASum = laneGroup.Sum(surfaceArea);

            const float openFactor = Settings.rebraidOpenSAFactor;
            rootSA = laneGroup.ReadFirstLane(rootSA);
            const float factor = rootSA > 0 ? (childSASum / rootSA) : 1.0;

            if (factor <= openFactor)
            {
                doRebraid = true;
                maxNumChildren = 4;
            }
        }

        if ((numBoxChildren == 0) || (firstInvalidLane >= maxNumChildren))
        {
            // Break the loop if there are no more box nodes needing to be processed,
            // or if there's no remaining space in the structure.
            break;
        }

        const float maxArea      = laneGroup.Max(scratchNodeArea);
        const bool isMaxAreaNode = (scratchNodeArea == maxArea);
        // Get the lane index of the max node
        const uint maxNodeLane   = laneGroup.GetFirstActiveLaneIndex(isMaxAreaNode);

        uint rightNode = INVALID_IDX;

        if (laneGroup.laneIndex == maxNodeLane)
        {
            // Assign left child for next processing
            scratchNodeIdx = scratchNode.left_or_primIndex_or_instIndex;
            rightNode = scratchNode.right_or_geometryIndex;
            needToLoad = true;
        }

        // Distribute the right child to the first thread with an invalid index
        rightNode = laneGroup.Broadcast(rightNode, maxNodeLane);

        if ((laneGroup.laneIndex == firstInvalidLane) && (rightNode != INVALID_IDX))
        {
            // This thread takes the right child
            scratchNodeIdx = rightNode;
            needToLoad = true;

            if (laneGroup.laneIndex >= 4)
            {
                const uint sourceBoxIndex = (packedSourceBoxes >> (maxNodeLane * 2)) & 0b11;
                packedSourceBoxes |= sourceBoxIndex << (laneGroup.laneIndex * 2);
            }
        }
    }
    numLeafChildren = laneGroup.BallotCount(isLeaf);

    if (laneGroup.IsFirstLane() && doRebraid)
    {
        const uint infoTemp = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_INFO_OFFSET) | (1 << ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_SHIFT);
        WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_INFO_OFFSET, infoTemp);
    }

    // Calculate exclusive prefix sum to get index (0,1,2... )
    if (isBox)
    {
        boxOrLeafChildIndex = laneGroup.ExclusivePrefixSumBool(true);
    }
    else if (isLeaf)
    {
        boxOrLeafChildIndex = laneGroup.ExclusivePrefixSumBool(true);
    }

    uint dstStackIdx = 0;

    if (Settings.topLevelBuild == 0)
    {
        const uint numInlinePrimStructs = GetNumInlinePrimStructs(laneGroup,
                                                                  scratchNodesScratchOffset,
                                                                  numPrimRefs,
                                                                  numLeafChildren,
                                                                  scratchNodeIdx,
                                                                  doRebraid,
                                                                  isLeaf);

        if (laneGroup.IsFirstLane())
        {
            IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET, 1);

            // This code assumes STACK_PTRS_SRC_PTR_OFFSET and STACK_PTRS_DST_PTR_OFFSET are in consecutive
            // DWORDs.
            const uint stackPtrOffset = ShaderConstants.offsets.qbvhGlobalStackPtrs + STACK_PTRS_SRC_PTR_OFFSET;

            uint64_t returnVal;
            ScratchGlobal.InterlockedAdd(stackPtrOffset, numBoxChildren, dstStackIdx);
            ScratchGlobal.InterlockedAdd(stackPtrOffset + 4, numBoxChildren + numInlinePrimStructs, baseDstBoxIdx);

            baseBoxNodeOffset = offsets.internalNodes + (baseDstBoxIdx * QBVH8::NodeSizeInBytes);

            // BLAS leaf nodes are allocated in a unified data section unlike TLAS which requires the instance nodes
            // to be in a separate data section for sideband addressing. See ComputeInstanceSidebandOffset()
            baseLeafNodeOffset = EnableWaveLeafCompressionPass() ?
                0 : (baseBoxNodeOffset + (numBoxChildren * QBVH8::NodeSizeInBytes));
        }
    }
    else
    {
        if (laneGroup.IsFirstLane())
        {
            // TODO: This atomic increment can be removed if we place fp32 node count and leaf node count
            // adjacent in memory and use a single 64-bit atomic to increment both box and leaf node counts
            IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET, 1);

            // This code assumes STACK_PTRS_SRC_PTR_OFFSET and STACK_PTRS_DST_PTR_OFFSET are in consecutive
            // DWORDs.
            const uint stackPtrOffset = ShaderConstants.offsets.qbvhGlobalStackPtrs + STACK_PTRS_SRC_PTR_OFFSET;

            uint baseDstLeafIdx = 0;
            uint64_t returnVal;
            ScratchGlobal.InterlockedAdd(stackPtrOffset, numBoxChildren, baseDstBoxIdx);
            ScratchGlobal.InterlockedAdd(stackPtrOffset + 4, numLeafChildren, baseDstLeafIdx);
            dstStackIdx = baseDstBoxIdx;

            baseBoxNodeOffset = offsets.internalNodes + (baseDstBoxIdx * QBVH8::NodeSizeInBytes);
            baseLeafNodeOffset = offsets.leafNodes + (baseDstLeafIdx * QBVH8::NodeSizeInBytes);
        }
    }

    baseDstBoxIdx = laneGroup.ReadFirstLane(baseDstBoxIdx);
    dstStackIdx   = laneGroup.ReadFirstLane(dstStackIdx);

    // Write box nodes to the global memory to be process by another thread
    if (isBox)
    {
        uint childStackIdx = dstStackIdx + boxOrLeafChildIndex;
        uint packedDstNodeIdx = PackStackDstNodeIdx(baseDstBoxIdx + boxOrLeafChildIndex, boxOrLeafChildIndex);

        // Regular operation for other threads with isBox set
        ScratchGlobal.Store2(ShaderConstants.offsets.qbvhGlobalStack + (childStackIdx * sizeof(uint2)),
                         uint2(scratchNodeIdx, packedDstNodeIdx));
    }

    baseLeafNodeOffset = laneGroup.ReadFirstLane(baseLeafNodeOffset);
    baseBoxNodeOffset  = laneGroup.ReadFirstLane(baseBoxNodeOffset);

    uint basePrimStructIdx = 0;
    if (laneGroup.IsFirstLane())
    {
        const ScratchNode parentScratchNode = FetchScratchNode(scratchNodesScratchOffset, parentScratchNodeIdx);

        // Note, the current subtree being collapsed can be just a single primitive
        const bool isLeafNode = IsLeafNode(parentScratchNodeIdx, numPrimRefs);
        parentBbox = GetScratchNodeBoundingBoxTS(scratchNodesScratchOffset, isLeafNode, parentScratchNode);

        // The root of the BVH2 treelet contains the combined bounds of the child nodes in the node array. Compute the
        // common exponent from the root node bounds
        exponents = ComputeCommonExponent(parentBbox.min, parentBbox.max, 12);

        // Pre-compute exponent reciprocal
        rcpExponents = ComputeFastExpReciprocal(exponents, 12);

        QBVH8::WriteInternalNodeBaseOffset(currNodeOffset, baseBoxNodeOffset);
        if (EnableWaveLeafCompressionPass() == false)
        {
            QBVH8::WriteLeafNodeBaseOffset(currNodeOffset, baseLeafNodeOffset);
        }
        else if (Settings.topLevelBuild == 0)
        {
            QBVH8::WriteLeafNodeBaseOffset(currNodeOffset, INVALID_NODE);
        }

        QBVH8::WriteOrigin(currNodeOffset, parentBbox.min);

        const uint validChildCount = numBoxChildren + numLeafChildren;
        QBVH8::WritePackedExpChildIdxAndCount(currNodeOffset, exponents, indexInParent, validChildCount);

        // Store the source box bitfield in the upper bits of the OBB DWORD (unused by HW). This is used to construct
        // four filter boxes for instances.
        QBVH8::WriteObbMatrixIndex(currNodeOffset, PackObbMatrixIndexBits(INVALID_OBB, packedSourceBoxes));

        if ((Settings.topLevelBuild == false) && (EnableLeafCompressionPass() == false))
        {
            basePrimStructIdx = IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET, numLeafChildren);
        }
    }

    parentBbox.min    = laneGroup.ReadFirstLane(parentBbox.min);
    parentBbox.max    = laneGroup.ReadFirstLane(parentBbox.max);
    exponents         = laneGroup.ReadFirstLane(exponents);
    rcpExponents      = laneGroup.ReadFirstLane(rcpExponents);
    basePrimStructIdx = laneGroup.ReadFirstLane(basePrimStructIdx);

    if (scratchNodeIdx != INVALID_IDX)
    {
        // TLAS builds initialize these fields immediately. They should be left 0 for BLAS builds if we are compressing
        // leaf nodes. If we are not compressing, a single triangle primitive structure is produced immediately.
        // Either way the triangle node type field is 0.
        const uint nodeType        = isLeaf ? (Settings.topLevelBuild ?
                                               NODE_TYPE_USER_NODE_INSTANCE : NODE_TYPE_TRIANGLE_0) :
                                               NODE_TYPE_BOX_QUANTIZED_BVH8;
        const uint primRangeLength = isLeaf ? (EnableLeafCompressionPass() ? 0 : 1) : 1;
        const uint baseNodeOffset  = isLeaf ? baseLeafNodeOffset : baseBoxNodeOffset;
        const uint offset          = isLeaf ? boxOrLeafChildIndex + numBoxChildren : boxOrLeafChildIndex;

        const uint childNodeOffset = baseNodeOffset + (boxOrLeafChildIndex * QBVH8::NodeSizeInBytes);

        const uint3 childInfo = ComputePackedChildInfo(scratchNodesScratchOffset,
                                                       scratchNode,
                                                       scratchNodeIdx,
                                                       nodeType,
                                                       primRangeLength,
                                                       rcpExponents,
                                                       parentBbox.min,
                                                       numPrimRefs);

        QBVH8::WritePackedChildInfo(currNodeOffset, offset, childInfo);

        if (isBox)
        {
            QBVH8::WriteParentPointer(childNodeOffset, currNodePtr);
        }
        else //isLeaf
        {
            // Process and compress leaf children
            if (EnableLeafCompressionPass() == false)
            {
                WriteLeafNode(scratchNodesScratchOffset,
                              offsets,
                              numPrimRefs,
                              scratchNode,
                              scratchNodeIdx,
                              basePrimStructIdx + boxOrLeafChildIndex,
                              childNodeOffset,
                              currNodePtr);
            }
        }
    }
    else
    {
        QBVH8::WriteInvalidChildInfo(currNodeOffset, laneGroup.laneIndex);
    }

    if (IsObbEnabled())
    {
        float aabbSa = 0.0f;
        if (scratchNodeIdx != INVALID_IDX)
        {
            aabbSa = GetScratchNodeSurfaceArea(
                scratchNodesScratchOffset, scratchNodeIdx, scratchNode, numPrimRefs);
        }

        const float aabbSum = laneGroup.Sum(aabbSa);
        const float aabbMax = laneGroup.Max(aabbSa);
        const uint aabbMaxChildIdx = laneGroup.GetFirstActiveLaneIndex(aabbSa == aabbMax);

        uint bestObbMatrixIdx = INVALID_OBB;
        if ((aabbMaxChildIdx == laneGroup.laneIndex) && (isLeaf))
        {
            uint packedFlags = scratchNode.packedFlags;

            bool isCollapse = scratchNode.numPrimitivesAndDoCollapse & 1;

            if (CollapseAnyPairs())
            {
                isCollapse = (scratchNode.numPrimitivesAndDoCollapse >> 1) == 2;
            }

            if (isCollapse)
            {
                // Triangles in a range are colocated and tend to share a similar orientation, so
                // choosing one at random from the range is about as effective as choosing one with
                // a more sophisticated heuristic.
                //
                // Pick the first triangle in the range as an approximation.
                uint triNodeIdx = scratchNode.left_or_primIndex_or_instIndex;

                if (UsePrimIndicesArray())
                {
                    triNodeIdx = GetPrimRefIdx(FetchSortedPrimIndex(ShaderConstants.offsets.primIndicesSorted,
                                                                    scratchNode.sortedPrimIndex), numPrimRefs);
                }

                packedFlags =
                    FETCH_SCRATCH_NODE_DATA(uint, scratchNodesScratchOffset, triNodeIdx, SCRATCH_NODE_FLAGS_OFFSET);
            }

            bestObbMatrixIdx = ExtractScratchNodeObbMatrixIdx(packedFlags);
        }

        bestObbMatrixIdx = laneGroup.Broadcast(bestObbMatrixIdx, aabbMaxChildIdx);

        if (laneGroup.IsFirstLane())
        {
            if (numBoxChildren == 0)
            {
                uint obbRefitStackIndex = 0;

                ScratchGlobal.InterlockedAdd(
                    ShaderConstants.offsets.obbRefitStackPtrs + STACK_PTRS_OBB_REFIT_BATCH_COUNT, 1, obbRefitStackIndex);

                ScratchGlobal.Store(
                    ShaderConstants.offsets.obbRefitStack + (obbRefitStackIndex * sizeof(uint)), bvhNodeDstIdx);
            }

            // OBB refit flags are packed as follows:
            // b[0:3] : 2's complement of box child count. The maximum value can be 8. That requires 3 bits plus the sign
            //          bit --> 4 bits. The maximum carry bits (for when box count is incremented by refit kernel)
            //          is 1, but we have plenty more room.
            // b[4:15] : 2's complement increment overflow bits.
            // b[16:23]: Child index with maximum AABB surface area for this node.
            // b[24:31]: OBB matrix index of the best leaf node amongst the children. This is precomputed and stored in
            //           scratch node during morton code generation phase to absorb the VGPR impact of that computation.
            const uint encodedBits = (bestObbMatrixIdx << 24u) |
                                     (aabbMaxChildIdx  << 16u) |
                                     ((numBoxChildren ^ 0xFu) + 1);

            const uint obbFlagsOffset = ShaderConstants.offsets.qbvhObbFlags + (bvhNodeDstIdx * OBB_TABLE_ENTRY_SIZE);
            ScratchGlobal.Store<uint2>(obbFlagsOffset, uint2(encodedBits, asuint(aabbSum)));
        }
    }

    uint localPrimCount = 0;
    if (scratchNodeIdx != INVALID_IDX)
    {
        if (isBox == false)
        {
            localPrimCount = FetchScratchNodeNumPrimitives(scratchNode, IsLeafNode(scratchNodeIdx, numPrimRefs));
        }
    }

    const uint primCount = WaveActiveSum(localPrimCount);
    if (WaveIsFirstLane())
    {
        if ((Settings.topLevelBuild == false) && (primCount > 0))
        {
            ScratchGlobal.InterlockedAdd(ShaderConstants.offsets.qbvhGlobalStackPtrs +
                                         NumLeafsDoneOffset(),
                                         primCount);
        }
    }

    if (EnableLegacyLeafCompressionPass() && (numLeafChildren != 0))
    {
        const uint baseLdsOffset = GetNodeArrayBaseInLds(laneGroup.ReadFirstLane(localId), maxNumChildren);

        if (isLeaf)
        {
            const uint index = baseLdsOffset + boxOrLeafChildIndex;
            SharedMem[index] = scratchNodeIdx;
        }

        const uint leafLdsOffset = baseLdsOffset;
        if (laneGroup.IsFirstLane())
        {
            CompressLeafChildren(scratchNodesScratchOffset,
                                 offsets,
                                 currNodeOffset,
                                 baseLeafNodeOffset,
                                 numLeafChildren,
                                 numBoxChildren,
                                 numPrimRefs,
                                 leafLdsOffset,
                                 doRebraid);
        }
    }
    const uint numEmptySlots = maxNumChildren - (numBoxChildren + numLeafChildren);

    BoxSplitting(laneGroup,
                 scratchNodesScratchOffset,
                 currNodeOffset,
                 indexInParent,
                 offsets,
                 baseLeafNodeOffset,
                 maxNumChildren,
                 numLeafChildren,
                 numBoxChildren,
                 boxOrLeafChildIndex,
                 scratchNode,
                 numEmptySlots,
                 rcpExponents,
                 exponents,
                 parentBbox,
                 isLeaf);

    if (EnableWaveLeafCompressionPass() && (numLeafChildren > 0))
    {
        WritePrimCompBatches3_1(laneGroup,
                                rootNodeIndex,
                                scratchNodesScratchOffset,
                                parentScratchNodeIdx,
                                currNodeOffset,
                                numBoxChildren,
                                numLeafChildren,
                                primCount,
                                numPrimRefs,
                                scratchNodeIdx,
                                boxOrLeafChildIndex,
                                isLeaf);
    }

    // Write fat leaf offsets.
    if (IsUpdateAllowed() && (numBoxChildren == 0) && laneGroup.IsFirstLane())
    {
        const uint index = IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP16_NODES_OFFSET, 1);
        const uint baseUpdatePtrOffset = offsets.primNodePtrs + (ShaderConstants.maxNumPrimitives * sizeof(uint));
        DstBuffer.Store(baseUpdatePtrOffset + (index * sizeof(uint)), currNodeOffset);
    }
}

//=====================================================================================================================
// Convert source BVH2 (in scratch memory) into RTIP3.1 compatible hardware format
void EncodeHwBvhImpl(
    uint globalId,
    uint localId,
    uint numPrimRefs)
{
    const AccelStructOffsets offsets = ShaderConstants.header.offsets;

    const uint rootNodeIndex = GetBvh2RootNodeIndex();

    const uint scratchNodesScratchOffset = CalculateBvhNodesOffset(ShaderConstants, numPrimRefs);

    const bool isSinglePrimBvh = (numPrimRefs == 1);

    const ScratchNode rootScratchNode = FetchScratchNode(scratchNodesScratchOffset, rootNodeIndex);
    const uint numActivePrims =
        Settings.topLevelBuild ? numPrimRefs : FetchScratchNodeNumPrimitives(rootScratchNode, isSinglePrimBvh);

    const uint maxNumInternalNodes = GetNumInternalNodeCount(numActivePrims);

    if (IsObbEnabled())
    {
        if (globalId < maxNumInternalNodes)
        {
            ScratchGlobal.Store<uint2>(ShaderConstants.offsets.qbvhObbFlags + globalId * OBB_TABLE_ENTRY_SIZE, 0);
        }
    }

    if (globalId == 0)
    {
        const BoundingBox bbox =
            GetScratchNodeBoundingBoxTS(scratchNodesScratchOffset, isSinglePrimBvh, rootScratchNode);

        WriteAccelStructHeaderRootBoundingBox(bbox);

        const uint3 exponents = ComputeCommonExponent(bbox.min, bbox.max, 12);

        // Pre-compute exponent reciprocal
        const float3 rcpExponents = ComputeFastExpReciprocal(exponents, 12);

        if (Settings.topLevelBuild == false)
        {
            // Generate intersectable instance node data in BLAS metadata
            if ((Settings.instanceMode == InstanceMode::Passthrough) || isSinglePrimBvh)
            {
                const uint instanceMask =
                    ExtractScratchNodeInstanceMask(rootScratchNode.packedFlags);

                WriteInstancePassthrough(CreateRootNodePointer(),
                                         bbox,
                                         ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET);
            }
            else if (Settings.instanceMode == InstanceMode::FilterNode)
            {
                // Generate filter node data in BLAS metadata
                WriteBlasInstanceFilterNode(scratchNodesScratchOffset,
                                            numPrimRefs,
                                            rootNodeIndex,
                                            bbox.min,
                                            exponents,
                                            rcpExponents);
            }

            if (EnableLegacyLeafCompressionPass())
            {
                // Load acceleration structure header
                AccelStructHeader header = DstBuffer.Load<AccelStructHeader>(0);

                header.info2 =
                    bitFieldInsert(header.info2, ACCEL_STRUCT_HEADER_INFO_2_LEGACY_LEAF_COMPRESSION_FLAGS_SHIFT, 1, 1);
                DstBuffer.Store(ACCEL_STRUCT_HEADER_INFO_2_OFFSET, header.info2);
            }

            DstBuffer.Store(ACCEL_STRUCT_HEADER_PACKED_FLAGS_OFFSET, rootScratchNode.packedFlags);
        }

        // Write parent pointer for root node.
        QBVH8::WriteParentPointer(offsets.internalNodes, INVALID_IDX);
    }

    LaneGroup laneGroup;
    laneGroup.Alloc(8);

    // Each stack item stores data for writes to linear QBVH memory, indexed by stack index
    const uint globalStackIndex = globalId / 8;

    // With early pairing enabled, numPrimRefs represents the active primitive references. However,
    // we need to use the active primitive count to estimate the hardware internal node count.
    bool isGlobalStackEntryDone = (globalStackIndex >= maxNumInternalNodes);

    // Rebraid currently only supports 4x. Create the root internal node as BVH4 when rebraid is enabled.
    // Note, this creates a partial BVH8 internal node at the root. We can either use variable-arity BLAS
    // which requires additional ALU during traversal or fill in the empty slots with box splitting
    //

    // Note, thread 0 always handles root node
    const bool isRebraidRootNode = Settings.enableInstanceRebraid &&
                                   (globalStackIndex == 0) &&
                                   (numActivePrims > Settings.rebraidOpenMinPrims);

    const uint maxNumChildren = 8;

    while (isGlobalStackEntryDone == false)
    {
        uint numLeafsDone = 0;
        if (WaveIsFirstLane())
        {
            numLeafsDone = ScratchGlobal.Load(ShaderConstants.offsets.qbvhGlobalStackPtrs +
                                              NumLeafsDoneOffset());
        }

        numLeafsDone = WaveReadLaneFirst(numLeafsDone);

        if (numLeafsDone >= numActivePrims)
        {
            break;
        }

        uint2 stackEntry = uint2(0, 0);
        if (laneGroup.IsFirstLane())
        {
            stackEntry = StackPop(globalStackIndex);
        }

        stackEntry = laneGroup.ReadFirstLane(stackEntry);

        // If we have a valid node on the stack, process node
        if (stackEntry.x != INVALID_IDX)
        {
            const uint bvhNodeSrcIdx = stackEntry.x;
            const uint bvhNodeDstIdx = GetStackDstIdx(stackEntry.y);
            const uint indexInParent = GetStackDstIdxInParent(stackEntry.y);

            const uint currNodeAddr = offsets.internalNodes + (bvhNodeDstIdx * QBVH8::NodeSizeInBytes);
            const uint currNodePtr = CreateBoxNodePointer(currNodeAddr, false);

            // Pull up the nodes chosen by SAH to be the new children in BVH8
            EncodeCompressedBoxNode(laneGroup,
                                    rootNodeIndex,
                                    scratchNodesScratchOffset,
                                    localId,
                                    bvhNodeSrcIdx,
                                    bvhNodeDstIdx,
                                    currNodePtr,
                                    currNodeAddr,
                                    indexInParent,
                                    offsets,
                                    maxNumChildren,
                                    numPrimRefs,
                                    isRebraidRootNode);

            isGlobalStackEntryDone = true;
        }
    }
}

} // namespace Gfx12

#endif

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
// Main Function : EncodeHwBvh3_1
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void EncodeHwBvh3_1(
    uint globalIdIn : SV_DispatchThreadID,
    uint groupIdIn  : SV_GroupID,
    uint localIdIn  : SV_GroupThreadID)
{
#if GPURT_BUILD_RTIP3_1

    uint globalId = globalIdIn;
    uint localId  = localIdIn;
    uint groupId  = groupIdIn;

    const uint numActivePrims = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
    const uint maxInternalNodeCount = (numActivePrims == 0) ? 0 : GetMaxInternalNodeCount(numActivePrims);

    uint numTasksWait = 0;
    uint waveId       = 0;

    INIT_TASK;

    BEGIN_TASK(1)
    if (globalId == 0)
    {
        InitStackPtrs();
        Gfx12::InitObbRefitStackPtrs();
    }
    END_TASK(1);

    BEGIN_TASK(RoundUpQuotient(maxInternalNodeCount, BUILD_THREADGROUP_SIZE));

    ClearStackPtrs(globalId, maxInternalNodeCount, ShaderRootConstants.NumThreads());

    END_TASK(RoundUpQuotient(maxInternalNodeCount, BUILD_THREADGROUP_SIZE));

    const uint numThreads = maxInternalNodeCount * 8;
    BEGIN_TASK(RoundUpQuotient(numThreads, BUILD_THREADGROUP_SIZE));

    Gfx12::EncodeHwBvhImpl(globalId, localId, numActivePrims);

    END_TASK(RoundUpQuotient(numThreads, BUILD_THREADGROUP_SIZE));

    BEGIN_TASK(1);

    PostHwBvhBuild(localId, numActivePrims);

    if (Settings.enableInstanceRebraid)
    {
        const uint tempInfo = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_INFO_OFFSET);
        const uint doRebraid = (tempInfo >> ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_SHIFT) &
                                ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_MASK;
        const bool blasWithOBB = (Settings.topLevelBuild == false) && (Settings.enableOrientedBoundingBoxes != 0);
        if (doRebraid && blasWithOBB)
        {
            const uint numInternalNodes = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET);
            InitObbBlasMetadata(globalId, numInternalNodes, BUILD_THREADGROUP_SIZE);
        }
    }

    if (globalId == 0)
    {
        if (Gfx12::EnableWaveLeafCompressionPass())
        {
            // The CompressPrims dispatch requires one group per compression batch plus enough groups to process the
            // entire single prims list
            const uint compressBatches =
                RoundUpQuotient(FetchCompressPrimBatchCount(ShaderConstants.offsets.qbvhGlobalStackPtrs),
                                COMPRESSION_BATCHES_PER_GROUP);

            const uint singlePrimCount  = FetchCompressPrimSingleCount(ShaderConstants.offsets.qbvhGlobalStackPtrs);
            const uint singlePrimGroups = RoundUpQuotient(RoundUpQuotient(singlePrimCount, 2), PRIM_COMP_GROUP_SIZE);

            ScratchGlobal.Store(ShaderConstants.offsets.qbvhGlobalStackPtrs + STACK_PTRS_PRIM_COMP_GROUP_COUNT_X,
                                compressBatches + singlePrimGroups);
        }

        if (IsObbEnabled())
        {
            const uint obbRefitBatchCount =
                ScratchGlobal.Load(ShaderConstants.offsets.obbRefitStackPtrs + STACK_PTRS_OBB_REFIT_BATCH_COUNT);

            // Note, a group of 8 lanes works on a single internal node.
            const uint obbRefitGroupCount = RoundUpQuotient(obbRefitBatchCount,
                                                            REFIT_ORIENTED_BOUNDS3_1_THREADGROUP_SIZE / 8u);
            ScratchGlobal.Store(ShaderConstants.offsets.obbRefitStackPtrs + STACK_PTRS_OBB_REFIT_GROUP_COUNT_X,
                                obbRefitGroupCount);
        }
    }

    END_TASK_NO_WAIT(1);

#endif
}
#endif

#endif
