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
#ifndef _PRIMITIVE_STRUCTURE3_1_HLSL
#define _PRIMITIVE_STRUCTURE3_1_HLSL

// Max LDS for full occupancy (2 units of 256DW)
#define PRIM_COMP_LDS_SIZE 512

#if NO_SHADER_ENTRYPOINT == 0
#include "../shadersClean/build/BuildRootSignature.hlsli"
groupshared uint SharedMem[PRIM_COMP_LDS_SIZE];
uint GetSharedMem(uint index)
{
    return SharedMem[index];
}
void SetSharedMem(uint index, uint value)
{
    SharedMem[index] = value;
}
#endif

#if GPURT_BUILD_RTIP3_1

#include "../shadersClean/common/ShaderDefs.hlsli"
#include "../shadersClean/common/Common.hlsli"
#include "IntersectCommon.hlsl"
#include "../shadersClean/build/BuildCommonScratch.hlsli"
#include "rtip3_1.hlsli"
#include "PrimitiveStructureEncoder3_1.hlsl"

#define LDS_OFFSET_PRIM_SCRATCH_LIST 0  // 32 DW - one node per lane
#define LDS_OFFSET_PRIM_STRUCT       0  // 64 DW - two prim structs
#define LDS_OFFSET_PRIM_STRUCT_VERTS (LDS_OFFSET_PRIM_STRUCT + PRIMITIVE_STRUCT_SIZE_IN_DW * COMPRESSION_BATCHES_PER_GROUP)
#define LDS_OFFSET_END               (LDS_OFFSET_PRIM_STRUCT_VERTS + (COMPRESSION_BATCH_SIZE * 9) * COMPRESSION_BATCHES_PER_GROUP)

GPURT_STATIC_ASSERT(LDS_OFFSET_END <= PRIM_COMP_LDS_SIZE, "Not enough LDS allocated.");

namespace PrimStruct3_1
{
//=====================================================================================================================
// Determine the offset of the next primitive structure and increment the header count
void AllocPrimStructs(
    AccelStructOffsets offsets,
    uint primStructCount,
    uint numBoxChildren,
    bool isRebraidRootChild,
    out uint primStructIdx,
    out uint primStructOffset)
{
    if (isRebraidRootChild)
    {
        // Children of the root node must be allocated at the start of the AS to support rebraid.
        // Space for the leaf children is preallocated after the box children of the root.
        primStructOffset = sizeof(AccelStructHeader) + (numBoxChildren + 1) * QUANTIZED_BVH8_NODE_SIZE;
    }
    else
    {
        uint baseNodeIndex;
        ScratchGlobal.InterlockedAdd(ShaderConstants.offsets.qbvhGlobalStackPtrs + STACK_PTRS_DST_PTR_OFFSET,
                                     primStructCount, baseNodeIndex);

        // Primitive structures are interleaved with internal nodes
        primStructOffset = offsets.internalNodes + baseNodeIndex * PRIMITIVE_STRUCT_SIZE_IN_BYTE;
    }

    primStructIdx = IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET, primStructCount);

    // BuildParallel will update the compacted size and emit buffer after the CompressPrims pass
#if BUILD_PARALLEL == 0
    if (Settings.disableCompaction == false)
    {
        const uint primStructSize = primStructCount * PRIMITIVE_STRUCT_SIZE_IN_BYTE;
        IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_COMPACTED_BYTE_SIZE_OFFSET, primStructSize);

        if (Settings.emitCompactSize)
        {
            EmitBuffer.InterlockedAdd(0, primStructSize);
        }
    }
#endif
}

//======================================================================================================================
static uint2 WaveAllocPrimStructs(
    AccelStructOffsets offsets,
    uint parentBoxOffset,
    uint numBoxChildren,
    uint primStructCount)
{
    // Allocate prim structs
    uint primStructOffset;
    uint primStructIdx = 0;
    if (g_laneGroup.IsFirstLane())
    {
        uint doRebraid = 0;

        if (Settings.enableInstanceRebraid)
        {
            const uint tempInfo = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_INFO_OFFSET);
            doRebraid = (tempInfo >> ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_SHIFT) &
                        ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_MASK;
        }

        const bool isRebraidRootChild =
            (doRebraid && (parentBoxOffset == sizeof(AccelStructHeader)));

        AllocPrimStructs(offsets, primStructCount, numBoxChildren, isRebraidRootChild, primStructIdx, primStructOffset);

        DstBuffer.Store(parentBoxOffset + QUANTIZED_BVH8_NODE_OFFSET_LEAF_NODE_BASE_OFFSET,
            PackNodePointer(0, primStructOffset));
    }
    primStructOffset = g_laneGroup.ReadFirstLane(primStructOffset);

    return uint2(primStructIdx, primStructOffset);
}

//======================================================================================================================
static uint FetchScratchPackedParentInfo(
    uint baseScratchNodeOffset)
{
    return ScratchGlobal.Load(baseScratchNodeOffset);
}

//======================================================================================================================
static uint FetchScratchPrimCompBatchNodeIdx(
    uint baseScratchNodeOffset,
    uint index)
{
    // Primitive compression node indices follow the packed parent info
    return ScratchGlobal.Load(baseScratchNodeOffset + sizeof(uint) + (index * sizeof(uint)));
}

//======================================================================================================================
// Add a scratch node index to the LDS list and increment the current index
void AppendToLdsScratchList(
    inout uint index,
    uint       nodeIndex)
{
    SharedMem[LDS_OFFSET_PRIM_SCRATCH_LIST + g_laneGroup.GetFirstLaneIndex() + index] = nodeIndex;
    index++;
}

//======================================================================================================================
// Load all the leaf nodes into LDS. Expands prim ranges in the process.
static void LoadLeafNodes(
    in    uint     scratchNodesScratchOffset,
    in    uint     primIndicesSortedOffset,
    in    uint     compBatchOffset,
    in    uint     numActivePrims,
    inout uint     primCount,
    inout uint     parentBoxOffset,
    inout uint     numBoxChildren,
    inout uint64_t primRangeStartMask,
    inout uint64_t primRangeEndMask)
{
    uint leafPrimCount = 0;

    uint baseNodeOffset = INVALID_IDX;
    uint packedParentInfo = 0;

    const uint batchOffset = ShaderConstants.offsets.primCompNodeIndices + compBatchOffset;

    if (g_laneGroup.IsFirstLane())
    {
        const uint batchNodeIdx = ScratchGlobal.Load(batchOffset);

        baseNodeOffset = CalcScratchNodeOffset(scratchNodesScratchOffset, batchNodeIdx);
        packedParentInfo = FetchScratchPackedParentInfo(baseNodeOffset);
    }

    packedParentInfo = g_laneGroup.ReadFirstLane(packedParentInfo);
    baseNodeOffset = g_laneGroup.ReadFirstLane(baseNodeOffset);

    const CompBatchParentInfo parentInfo = UnpackCompBatchParentInfo(packedParentInfo);
    parentBoxOffset = parentInfo.boxNodeOffset;
    numBoxChildren = parentInfo.numBoxChildren;

    if (g_laneGroup.laneIndex < parentInfo.numLeafChildren)
    {
        // Load leaf child node indices from scratch memory following the packedParentInfo
        uint nodeIndex = FetchScratchPrimCompBatchNodeIdx(baseNodeOffset, g_laneGroup.laneIndex);

        // Each lane reads the prim count for a child then walks the linked list to find the nodes
        leafPrimCount = FetchScratchNodeNumPrimitives(
            scratchNodesScratchOffset, nodeIndex, IsLeafNode(nodeIndex, numActivePrims));

        uint ldsDstOffset = g_laneGroup.PrefixSum(leafPrimCount);

        primRangeStartMask |= bit64(ldsDstOffset);

        if (IsLeafNode(nodeIndex, numActivePrims))
        {
            bool isQuadPrim = false;

            if (Settings.enableEarlyPairCompression)
            {
                const ScratchNode scratchNode = FetchScratchNode(scratchNodesScratchOffset, nodeIndex);
                isQuadPrim = IsScratchNodeQuadPrimitive(scratchNode);
            }

            if (isQuadPrim)
            {
                AppendToLdsScratchList(ldsDstOffset, nodeIndex);
                AppendToLdsScratchList(ldsDstOffset, nodeIndex);
            }
            else
            {
                AppendToLdsScratchList(ldsDstOffset, nodeIndex);
            }
        }
        else
        {
            ScratchNode scratchNode = FetchScratchNode(scratchNodesScratchOffset, nodeIndex);
            GPU_ASSERT(scratchNode.numPrimitivesAndDoCollapse & 1);

            if (UsePrimIndicesArray())
            {
                for (uint i = 0; i < leafPrimCount; i++)
                {
                    nodeIndex = LEAFIDX(FetchSortedPrimIndex(primIndicesSortedOffset, scratchNode.sortedPrimIndex + i));
                    AppendToLdsScratchList(ldsDstOffset, nodeIndex);
                }
            }
            else if (CollapseAnyPairs())
            {
                AppendToLdsScratchList(ldsDstOffset, scratchNode.left_or_primIndex_or_instIndex);
                AppendToLdsScratchList(ldsDstOffset, scratchNode.right_or_geometryIndex);
            }
            else
            {
                 // Set the next node index to the head pointer of the range
                nodeIndex = scratchNode.left_or_primIndex_or_instIndex;

                uint primsFound = 0;

                // TODO: Does it help to put a fixed upper bound here for unrolling?
                while (nodeIndex != INVALID_IDX)
                {
                    bool isQuadPrim = false;

                    if (Settings.enableEarlyPairCompression)
                    {
                        const ScratchNode scratchNode = FetchScratchNode(scratchNodesScratchOffset, nodeIndex);
                        isQuadPrim = IsScratchNodeQuadPrimitive(scratchNode);
                    }

                    if (isQuadPrim)
                    {
                        AppendToLdsScratchList(ldsDstOffset, nodeIndex);
                        AppendToLdsScratchList(ldsDstOffset, nodeIndex);
                        primsFound += 2;
                    }
                    else
                    {
                        AppendToLdsScratchList(ldsDstOffset, nodeIndex);
                        primsFound++;
                    }

                    scratchNode = FetchScratchNode(scratchNodesScratchOffset, nodeIndex);
                    nodeIndex = scratchNode.parent;
                }

                GPU_ASSERT(primsFound == leafPrimCount);
            }
        }

        primRangeEndMask |= bit64(ldsDstOffset - 1);
    }

    GroupMemoryBarrierWithGroupSync();

    primCount = g_laneGroup.Sum(leafPrimCount);

    primRangeStartMask = g_laneGroup.BitOr(uint(primRangeStartMask));
    primRangeEndMask   = g_laneGroup.BitOr(uint(primRangeEndMask));
}

//======================================================================================================================
static void WaveCompressLeafChildren(
    AccelStructOffsets offsets,
    uint               numActivePrims,
    uint               compBatchOffset)
{
    const uint scratchNodesScratchOffset = CalculateBvhNodesOffset(ShaderConstants, numActivePrims);

    uint primCount = 0;
    uint parentBoxOffset = 0;
    uint numBoxChildren = 0;

    uint64_t primRangeStartMask = 0;
    uint64_t primRangeEndMask = 0;

    LoadLeafNodes(
        scratchNodesScratchOffset,
        ShaderConstants.offsets.primIndicesSorted,
        compBatchOffset,
        numActivePrims,
        primCount,
        parentBoxOffset,
        numBoxChildren,
        primRangeStartMask,
        primRangeEndMask);

    TriCompressState triState;
    triState.emitPrimStruct = false;

    if (g_laneGroup.laneIndex < primCount)
    {
        bool compressMultiplePairs = Settings.primCompressionFlags & PrimCompFlags::MultiPrim;

        uint doRebraid = 0;
        if (Settings.enableInstanceRebraid)
        {
            const uint tempInfo = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_INFO_OFFSET);
            doRebraid = (tempInfo >> ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_SHIFT) &
                        ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_MASK;
        }

        if (doRebraid && (parentBoxOffset == sizeof(AccelStructHeader)))
        {
            // The primitive structure space for the children of a rebraided root node is pre-reserved in the BVH8 phase
            // after the box children. Disable compression across pairs to avoid leaving unused space after the root
            // node. This simplifies compaction.
            compressMultiplePairs = false;
        }

        const uint primListIndex = g_laneGroup.laneIndex;
        const uint nodeIndex = SharedMem[LDS_OFFSET_PRIM_SCRATCH_LIST + primListIndex + g_laneGroup.GetFirstLaneIndex()];
        GPU_ASSERT(IsLeafNode(nodeIndex, numActivePrims));

        const ScratchNode scratchNode = FetchScratchNode(scratchNodesScratchOffset, nodeIndex);

        bool isQuadPrim = false;

        if (Settings.enableEarlyPairCompression)
        {
            isQuadPrim = IsScratchNodeQuadPrimitive(scratchNode);
        }

        const bool isTri0 = IsTriangle0(primRangeStartMask, primListIndex);

        TriangleData tri;
        tri.v0 = scratchNode.bbox_min_or_v0;
        tri.v1 = scratchNode.bbox_max_or_v1;
        tri.v2 = scratchNode.sah_or_v2_or_instBasePtr;

        uint primId = scratchNode.left_or_primIndex_or_instIndex;
        uint geomId = ExtractScratchNodeGeometryIndex(scratchNode);

        if (isQuadPrim)
        {
            const uint triangleIdx = isTri0 ? 0 : 1;
            tri = GetScratchNodeQuadVertices(scratchNodesScratchOffset, nodeIndex, triangleIdx);
            primId = GetScratchNodePrimitiveIndex(scratchNode, triangleIdx);
        }

        triState = ComputeCompressedRanges(
            offsets,
            LDS_OFFSET_PRIM_STRUCT,
            primListIndex,
            geomId,
            primId,
            tri.v0,
            tri.v1,
            tri.v2,
            primRangeStartMask,
            primRangeEndMask,
            numBoxChildren,
            IsOpaqueNode(scratchNode.packedFlags),
            isTri0,
            compressMultiplePairs);
    }

    uint64_t emitPrimStructMask = g_laneGroup.Ballot(triState.emitPrimStruct);
    const uint primStructCount = countbits64(emitPrimStructMask);

    // Allocate prim structs
    const uint2 allocData = WaveAllocPrimStructs(offsets, parentBoxOffset, numBoxChildren, primStructCount);
    uint primStructIdx = allocData.x;
    uint primStructOffset = allocData.y;

    EmitPrimitiveStructures(offsets,
                            LDS_OFFSET_PRIM_STRUCT,
                            triState,
                            parentBoxOffset,
                            numBoxChildren,
                            primRangeEndMask,
                            emitPrimStructMask,
                            primStructIdx,
                            primStructOffset);
}

//=====================================================================================================================
// Create a Primitive Structure containing two single triangles
static void WriteTwoTriPrimStruct(
    AccelStructOffsets offsets,
    uint  scratchNodesScratchOffset,
    uint2 batchNodeIdx,
    uint2 packedParentInfo)
{
    // Store full vertex with no compression
    //
    // Each vertex is stored as 3 DW (32bit) without prefix
    //
    // Note there is another unused primId and geoId not specified here (for the second invalid triangle in the second
    // triangle pair). The offsets of these do not matter because they will never be read and do not influence the
    // position of other fields.
    //
    // Header ----------- 52 bits
    // Vertex Data ------ 576 bits (96 * 3 * 2)
    // ----- Unused ----- 12 bits
    // Parent pointer 0 - 32 bits
    // Parent pointer 1 - 32 bits
    // ----- Unused ----- 79 bits
    // geometryId1Payload 30 bits
    // Unused geo ID ---- 30 bits
    // geometryIdAnchor-- 30 bits
    //------------------- indexSectionMidpoint
    // primIdAnchor ----- 31 bits
    // Unused prim ID --- 31 bits
    // primId1Payload --- 31 bits
    // triDesc 0 -------- 29 bits
    // triDesc 1 -------- 29 bits
    //------------------- 1024 bits

    // Note the second primitive ID of the second pair overlaps a pair descriptor.
    // This primitive is never valid, so it's not an issue.
    const uint indexSectionMidpoint = 1024 - (2 * TRI_PAIR_DESC_SIZE) - (3 * 31);

    uint d[PRIMITIVE_STRUCT_SIZE_IN_DW];

    const uint pairCount = (batchNodeIdx[1] != INVALID_IDX) ? 2 : 1;

    // Metadata header
    d[0] = PackMetadataHeaderBitsLo(31, 31, 31, 0, 15, 15, pairCount - 1, false);
    d[1] = PackMetadataHeaderBitsHi(31, 31, indexSectionMidpoint);

    uint triDesc0 = 0;
    uint triDesc1 = 0;

    const ScratchNode scratchNodeTri0 = FetchScratchNode(scratchNodesScratchOffset, batchNodeIdx[0]);

    const CompBatchParentInfo parentInfo0 = UnpackCompBatchParentInfo(packedParentInfo[0]);
    const uint parentBoxOffset0 = parentInfo0.boxNodeOffset;
    const uint numBoxChildren0  = parentInfo0.numBoxChildren;

    const uint tri0Opaque = IsOpaqueNode(scratchNodeTri0.packedFlags);

    TriangleData tri0;

    tri0.v0.x = scratchNodeTri0.bbox_min_or_v0.x;
    tri0.v0.y = scratchNodeTri0.bbox_min_or_v0.y;
    tri0.v0.z = scratchNodeTri0.bbox_min_or_v0.z;

    tri0.v1.x = scratchNodeTri0.bbox_max_or_v1.x;
    tri0.v1.y = scratchNodeTri0.bbox_max_or_v1.y;
    tri0.v1.z = scratchNodeTri0.bbox_max_or_v1.z;

    tri0.v2.x = scratchNodeTri0.sah_or_v2_or_instBasePtr.x;
    tri0.v2.y = scratchNodeTri0.sah_or_v2_or_instBasePtr.y;
    tri0.v2.z = scratchNodeTri0.sah_or_v2_or_instBasePtr.z;

    const uint primId0 = scratchNodeTri0.left_or_primIndex_or_instIndex;
    const uint geomId0 = ExtractScratchNodeGeometryIndex(scratchNodeTri0);

    uint parentBoxOffset1 = 0;
    uint numBoxChildren1 = 0;

    TriangleData tri1;
    uint tri1Opaque;
    uint primId1;
    uint geomId1;

    if (batchNodeIdx[1] != INVALID_IDX)
    {
        const ScratchNode scratchNodeTri1 = FetchScratchNode(scratchNodesScratchOffset, batchNodeIdx[1]);

        tri1Opaque = IsOpaqueNode(scratchNodeTri1.packedFlags);

        tri1.v0.x = scratchNodeTri1.bbox_min_or_v0.x;
        tri1.v0.y = scratchNodeTri1.bbox_min_or_v0.y;
        tri1.v0.z = scratchNodeTri1.bbox_min_or_v0.z;

        tri1.v1.x = scratchNodeTri1.bbox_max_or_v1.x;
        tri1.v1.y = scratchNodeTri1.bbox_max_or_v1.y;
        tri1.v1.z = scratchNodeTri1.bbox_max_or_v1.z;

        tri1.v2.x = scratchNodeTri1.sah_or_v2_or_instBasePtr.x;
        tri1.v2.y = scratchNodeTri1.sah_or_v2_or_instBasePtr.y;
        tri1.v2.z = scratchNodeTri1.sah_or_v2_or_instBasePtr.z;

        primId1 = scratchNodeTri1.left_or_primIndex_or_instIndex;
        geomId1 = ExtractScratchNodeGeometryIndex(scratchNodeTri1);

        const CompBatchParentInfo parentInfo1 = UnpackCompBatchParentInfo(packedParentInfo[1]);
        parentBoxOffset1 = parentInfo1.boxNodeOffset;
        numBoxChildren1  = parentInfo1.numBoxChildren;
    }

    // tri0 v0
    d[1] = bitFieldInsert(d[1], 20, 12, asuint(tri0.v0.x));
    d[2] = bitFieldInsert(0,     0, 20, asuint(tri0.v0.x) >> 12);
    d[2] = bitFieldInsert(d[2], 20, 12, asuint(tri0.v0.y));
    d[3] = bitFieldInsert(0,     0, 20, asuint(tri0.v0.y) >> 12);
    d[3] = bitFieldInsert(d[3], 20, 12, asuint(tri0.v0.z));
    d[4] = bitFieldInsert(0,     0, 20, asuint(tri0.v0.z) >> 12);
    // tri0 v1
    d[4] = bitFieldInsert(d[4], 20, 12, asuint(tri0.v1.x));
    d[5] = bitFieldInsert(0,     0, 20, asuint(tri0.v1.x) >> 12);
    d[5] = bitFieldInsert(d[5], 20, 12, asuint(tri0.v1.y));
    d[6] = bitFieldInsert(0,     0, 20, asuint(tri0.v1.y) >> 12);
    d[6] = bitFieldInsert(d[6], 20, 12, asuint(tri0.v1.z));
    d[7] = bitFieldInsert(0,     0, 20, asuint(tri0.v1.z) >> 12);
    // tri0 v2
    d[7]  = bitFieldInsert(d[7], 20, 12, asuint(tri0.v2.x));
    d[8]  = bitFieldInsert(0,     0, 20, asuint(tri0.v2.x) >> 12);
    d[8]  = bitFieldInsert(d[8], 20, 12, asuint(tri0.v2.y));
    d[9]  = bitFieldInsert(0,     0, 20, asuint(tri0.v2.y) >> 12);
    d[9]  = bitFieldInsert(d[9], 20, 12, asuint(tri0.v2.z));
    d[10] = bitFieldInsert(0,     0, 20, asuint(tri0.v2.z) >> 12);

    triDesc0 |= 1 << TRI_DESC_PRIM_RANGE_STOP_BIT_SHIFT;
    PackedTriDescSetTriangle(0, 0, 1, 2, tri0Opaque, triDesc0);

    if (batchNodeIdx[1] != INVALID_IDX)
    {
        // tri1 v0
        d[10] = bitFieldInsert(d[10], 20, 12, asuint(tri1.v0.x));
        d[11] = bitFieldInsert(0,      0, 20, asuint(tri1.v0.x) >> 12);
        d[11] = bitFieldInsert(d[11], 20, 12, asuint(tri1.v0.y));
        d[12] = bitFieldInsert(0,      0, 20, asuint(tri1.v0.y) >> 12);
        d[12] = bitFieldInsert(d[12], 20, 12, asuint(tri1.v0.z));
        d[13] = bitFieldInsert(0,      0, 20, asuint(tri1.v0.z) >> 12);
        // tri1 v1
        d[13] = bitFieldInsert(d[13], 20, 12, asuint(tri1.v1.x));
        d[14] = bitFieldInsert(0,      0, 20, asuint(tri1.v1.x) >> 12);
        d[14] = bitFieldInsert(d[14], 20, 12, asuint(tri1.v1.y));
        d[15] = bitFieldInsert(0,      0, 20, asuint(tri1.v1.y) >> 12);
        d[15] = bitFieldInsert(d[15], 20, 12, asuint(tri1.v1.z));
        d[16] = bitFieldInsert(0,      0, 20, asuint(tri1.v1.z) >> 12);
        // tri1 v2
        d[16] = bitFieldInsert(d[16], 20, 12, asuint(tri1.v2.x));
        d[17] = bitFieldInsert(0,      0, 20, asuint(tri1.v2.x) >> 12);
        d[17] = bitFieldInsert(d[17], 20, 12, asuint(tri1.v2.y));
        d[18] = bitFieldInsert(0,      0, 20, asuint(tri1.v2.y) >> 12);
        d[18] = bitFieldInsert(d[18], 20, 12, asuint(tri1.v2.z));
        d[19] = bitFieldInsert(0,      0, 20, asuint(tri1.v2.z) >> 12);

        triDesc1 |= 1 << TRI_DESC_PRIM_RANGE_STOP_BIT_SHIFT;
        PackedTriDescSetTriangle(0, 3, 4, 5, tri1Opaque, triDesc1);

        // Geometry ID 1
        d[24] = bitFieldInsert(0, 15, 17, geomId1);
        d[25] = bitFieldInsert(0, 0,  13, geomId1 >> 17);

        // Primitive ID 1
        d[29] = bitFieldInsert(0, 7, 25, primId1);
        d[30] = bitFieldInsert(0, 0,  6, primId1 >> 25);

        // Triangle descriptor 1
        d[30] = bitFieldInsert(d[30], 6, 26, triDesc1);
        d[31] = bitFieldInsert(d[30], 0, 3, triDesc1 >> 26);
    }
    else
    {
        d[11] = 0;
        d[12] = 0;
        d[13] = 0;
        d[14] = 0;
        d[15] = 0;
        d[16] = 0;
        d[17] = 0;
        d[18] = 0;
        d[19] = 0;

        d[21] = 0;

        d[24] = 0;
        d[25] = 0;

        d[29] = 0;
        d[30] = 0;
        d[31] = 0;
    }

    d[20] = 0;
    d[21] = 0;
    d[22] = 0;
    d[23] = 0;

    // Geometry ID 0
    d[26] = bitFieldInsert(0, 11, 21, geomId0);
    d[27] = bitFieldInsert(0, 0,  9, geomId0 >> 21);

    // Index section midpoint

    // Primitive ID 0
    d[27] = bitFieldInsert(d[27], 9, 23, primId0);
    d[28] = bitFieldInsert(0,     0,  8, primId0 >> 23);

    // Triangle descriptor 0
    d[31] = bitFieldInsert(d[31], 3, 29, triDesc0);

    // Allocate the prim struct below the root if a triangle is a child of the root node
    bool isRebraidRootChild = false;
    uint rebraidRootNumBoxChildren = 0;

    uint doRebraid = 0;
    if (Settings.enableInstanceRebraid)
    {
        const uint tempInfo = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_INFO_OFFSET);
        doRebraid = (tempInfo >> ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_SHIFT) &
                    ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_MASK;
    }

    if (doRebraid)
    {
        if (parentBoxOffset0 == sizeof(AccelStructHeader))
        {
            rebraidRootNumBoxChildren = numBoxChildren0;
            isRebraidRootChild = true;
        }
        else if (parentBoxOffset1 == sizeof(AccelStructHeader))
        {
            rebraidRootNumBoxChildren = numBoxChildren1;
            isRebraidRootChild = true;
        }
    }

    uint primStructIdx = 0;
    uint primStructOffset = 0;
    AllocPrimStructs(offsets, 1, rebraidRootNumBoxChildren, isRebraidRootChild, primStructIdx, primStructOffset);

    DstBuffer.Store(offsets.primNodePtrs + (primStructIdx * sizeof(uint)), primStructOffset);

    DstBuffer.Store(parentBoxOffset0 + QUANTIZED_BVH8_NODE_OFFSET_LEAF_NODE_BASE_OFFSET,
                    PackNodePointer(0, primStructOffset));

    if (batchNodeIdx[1] != INVALID_IDX)
    {
        DstBuffer.Store(parentBoxOffset1 + QUANTIZED_BVH8_NODE_OFFSET_LEAF_NODE_BASE_OFFSET,
                        PackNodePointer(0, primStructOffset));

        const uint childInfoOffset =
            parentBoxOffset1 +
            QUANTIZED_BVH8_NODE_OFFSET_CHILD_INFO_0 +
            (numBoxChildren1 * QUANTIZED_NODE_CHILD_INFO_STRIDE) +
            QUANTIZED_NODE_CHILD_INFO_OFFSET_MAXY_MAXZ_NODE_TYPE_AND_RANGE;

        DstBuffer.InterlockedOr(childInfoOffset, NODE_TYPE_TRIANGLE_1 << 24);
    }

    [unroll]
    for (uint i = 0; i < PRIMITIVE_STRUCT_SIZE_IN_DW; i += 4)
    {
        DstBuffer.Store4(primStructOffset + (i*4), uint4(d[i], d[i+1], d[i+2], d[i+3]));
    }
}

//======================================================================================================================
// Compress list of single triangles into primitive structures
void CompressSinglePrims(
    uint globalId)
{
    const uint singlePrimCount = FetchCompressPrimSingleCount(ShaderConstants.offsets.qbvhGlobalStackPtrs);

    if (globalId < RoundUpQuotient(singlePrimCount, 2))
    {
        // The single prim list starts at the end of the data section and grows down
        const uint nodeIndicesOffset =
            ShaderConstants.offsets.primCompNodeIndicesEnd -
            singlePrimCount * sizeof(uint);

        const uint offset = nodeIndicesOffset + globalId * 2 * sizeof(uint);

        uint2 batchNodeIdx;

        if ((globalId * 2 + 1) < singlePrimCount)
        {
            batchNodeIdx = ScratchGlobal.Load2(offset);
        }
        else
        {
            // Only one node is valid at the end of the buffer
            batchNodeIdx[0] = ScratchGlobal.Load(offset);
            batchNodeIdx[1] = INVALID_IDX;
        }

        const uint numActivePrims = DstBuffer.Load(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
        const uint scratchNodesScratchOffset = CalculateBvhNodesOffset(ShaderConstants, numActivePrims);

        const AccelStructOffsets offsets = DstBuffer.Load<AccelStructOffsets>(ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

        // Initialise packed parent info for the single primitive case
        uint2 packedParentInfo = uint2(PackCompBatchParentInfo(offsets.internalNodes, 0, 1), 0);
        if (numActivePrims > 1)
        {
            // Each entry in the single primitives section is structured as a batch with one entry.
            // Each thread reads a pair of batches and produces one primitive structure.
            if (batchNodeIdx[0] != INVALID_IDX)
            {
                const uint scratchNodeOffset = CalcScratchNodeOffset(scratchNodesScratchOffset, batchNodeIdx[0]);
                packedParentInfo[0]          = FetchScratchPackedParentInfo(scratchNodeOffset);
                batchNodeIdx[0]              = FetchScratchPrimCompBatchNodeIdx(scratchNodeOffset, 0);
            }
            if (batchNodeIdx[1] != INVALID_IDX)
            {
                const uint scratchNodeOffset = CalcScratchNodeOffset(scratchNodesScratchOffset, batchNodeIdx[1]);
                packedParentInfo[1]          = FetchScratchPackedParentInfo(scratchNodeOffset);
                batchNodeIdx[1]              = FetchScratchPrimCompBatchNodeIdx(scratchNodeOffset, 0);
            }
        }

        WriteTwoTriPrimStruct(offsets, scratchNodesScratchOffset, batchNodeIdx, packedParentInfo);
    }
}

//======================================================================================================================
void WriteSingleQuadPrimitive(
    uint localId)
{
    if (localId == 0)
    {
        const uint singlePrimCount = FetchCompressPrimSingleCount(ShaderConstants.offsets.qbvhGlobalStackPtrs);
        GPU_ASSERT(singlePrimCount == 1);

        const AccelStructHeader header = DstBuffer.Load<AccelStructHeader>(0);
        GPU_ASSERT(header.numActivePrims == 1);

        // The single prim list starts at the end of the data section and grows down
        const uint nodeIndicesOffset = ShaderConstants.offsets.primCompNodeIndicesEnd - singlePrimCount * sizeof(uint);
        const uint scratchNodeIdx = ScratchGlobal.Load(nodeIndicesOffset);

        bool isTri1Valid = false;
        bool isTri0Opaque = false;
        bool isTri1Opaque = false;
        uint primId0 = 0;
        uint primId1 = 0;
        uint geomId0 = 0;
        uint geomId1 = 0;
        TriangleData tri0 = (TriangleData)0;
        TriangleData tri1 = (TriangleData)0;

        const uint scratchNodesScratchOffset = CalculateBvhNodesOffset(ShaderConstants, header.numActivePrims);

        const ScratchNode scratchNode = FetchScratchNode(scratchNodesScratchOffset, scratchNodeIdx);

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
        else if (IsLeafNode(scratchNodeIdx, header.numActivePrims) == false)
        {
            uint firstNodeIndex;
            ScratchNode scratchNodeTri0;

            uint secondNodeIndex;
            ScratchNode scratchNodeTri1;

            if (UsePrimIndicesArray())
            {
                firstNodeIndex = GetPrimRefIdx(FetchSortedPrimIndex(ShaderConstants.offsets.primIndicesSorted,
                                                                    scratchNode.sortedPrimIndex),
                                               header.numActivePrims);
                scratchNodeTri0 = FetchScratchNode(scratchNodesScratchOffset, firstNodeIndex);

                secondNodeIndex = GetPrimRefIdx(FetchSortedPrimIndex(ShaderConstants.offsets.primIndicesSorted,
                                                                     scratchNode.sortedPrimIndex + 1),
                                                header.numActivePrims);
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

        const uint nodeOffset = ShaderConstants.header.offsets.internalNodes + QUANTIZED_BVH8_NODE_SIZE;
        WritePairPrimStruct(tri0,
                            isTri0Opaque,
                            primId0,
                            geomId0,
                            isTri1Valid,
                            tri1,
                            isTri1Opaque,
                            primId1,
                            geomId1,
                            nodeOffset);

        DstBuffer.Store(ShaderConstants.header.offsets.primNodePtrs, nodeOffset);

        const uint boxNodeLeafBaseOffset =
            ShaderConstants.header.offsets.internalNodes + QUANTIZED_BVH8_NODE_OFFSET_LEAF_NODE_BASE_OFFSET;

        DstBuffer.Store(boxNodeLeafBaseOffset, PackNodePointer(0, nodeOffset));
    }
}

} // namespace PrimStruct3_1

#endif

#if NO_SHADER_ENTRYPOINT == 0
//======================================================================================================================
// RTIP3.1 primitive compression multi-dispatch entrypoint
[RootSignature(RootSig)]
#if GPURT_BUILD_RTIP3_1
[numthreads(PRIM_COMP_GROUP_SIZE, 1, 1)]
#else
[numthreads(1, 1, 1)]
#endif
void CompressPrims(
    uint groupId : SV_GroupID,
    uint localId : SV_GroupThreadID)
{
#if GPURT_BUILD_RTIP3_1
    g_laneGroup.Alloc(PRIM_COMP_GROUP_SIZE / COMPRESSION_BATCHES_PER_GROUP);

    const uint compressBatches = FetchCompressPrimBatchCount(ShaderConstants.offsets.qbvhGlobalStackPtrs);
    const uint compressGroups  = RoundUpQuotient(compressBatches, COMPRESSION_BATCHES_PER_GROUP);

    const AccelStructHeader header = DstBuffer.Load<AccelStructHeader>(0);

    // Note, header.numActivePrims represents number of primitive references (triangles or quads).
    const bool isSingleQuadBvh =
        (Settings.topLevelBuild == 0) && Settings.enableEarlyPairCompression && (header.numActivePrims == 1);

    if (isSingleQuadBvh)
    {
        PrimStruct3_1::WriteSingleQuadPrimitive(localId);
    }
    else if (groupId < compressGroups)
    {
        const uint batchId = groupId * COMPRESSION_BATCHES_PER_GROUP + g_laneGroup.groupIndex;
        const uint compBatchOffset = batchId * sizeof(uint);

        if (batchId < compressBatches)
        {
            PrimStruct3_1::WaveCompressLeafChildren(ShaderConstants.header.offsets,
                                                    header.numActivePrims,
                                                    compBatchOffset);
        }
    }
    else
    {
        const uint globalId = (groupId - compressGroups) * PRIM_COMP_GROUP_SIZE + localId;
        PrimStruct3_1::CompressSinglePrims(globalId);
    }
#endif
}
#endif

#endif
