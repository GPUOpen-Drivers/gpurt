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
uint2 AllocPrimStructs(
    uint packedParentInfo,
    uint primStructCount)
{
    const uint boxNodeOffset = packedParentInfo & ~bits(7);

    const bool isRebraidRootChild =
        (Settings.enableInstanceRebraid && (boxNodeOffset == sizeof(AccelStructHeader)));

    uint primStructOffset = 0;

    if (isRebraidRootChild)
    {
        // Children of the root node must be allocated at the start of the AS to support rebraid.
        // Space for the leaf children is preallocated after the box children of the root.
        const uint numBoxChildren = packedParentInfo & bits(3);
        primStructOffset = sizeof(AccelStructHeader) + (numBoxChildren + 1) * QUANTIZED_BVH8_NODE_SIZE;
    }
    else
    {
        // Primitive structures are interleaved with internal nodes
        const uint baseNodeIndex = EncodeHwBvh::IncrementCounter(STGB_COUNTER_IDX_BOX_REF, primStructCount);
        primStructOffset = ShaderConstants.header.offsets.internalNodes + baseNodeIndex * PRIMITIVE_STRUCT_SIZE_IN_BYTE;
    }

    const uint primStructIdx =
        IncrementAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET, primStructCount);

    return uint2(primStructIdx, primStructOffset);
}

//=====================================================================================================================
// Create a Primitive Structure containing two single triangles
static void WriteTwoTriPrimStruct(
    AccelStructOffsets offsets,
    uint pairCount,
    TriangleData tri0,
    TriangleData tri1,
    uint geomId0,
    uint geomId1,
    uint primId0,
    uint primId1,
    uint flags0,
    uint flags1,
    uint packedParentInfo0,
    uint packedParentInfo1)
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

    const uint indexSectionMidpoint = ComputeIndexSectionMidpoint(2, 31, 31);

    uint d[PRIMITIVE_STRUCT_SIZE_IN_DW];

    // Metadata header
    d[0] = PackMetadataHeaderBitsLo(31, 31, 31, 0, 15, 15, pairCount - 1, false);
    d[1] = PackMetadataHeaderBitsHi(31, 31, indexSectionMidpoint);

    uint triDesc0 = 0;
    uint triDesc1 = 0;

    const uint tri0Opaque = flags0 & (1 << BOX_NODE_FLAGS_ONLY_OPAQUE_SHIFT);
    const uint tri1Opaque = flags1 & (1 << BOX_NODE_FLAGS_ONLY_OPAQUE_SHIFT);

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

    if (pairCount > 1)
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

    // Pick the minimum of the two packed parents infos to determine rebraid root node primitives
    const uint packedParentInfo = min(packedParentInfo0, packedParentInfo1);

    const uint2 allocData = AllocPrimStructs(packedParentInfo, 1);
    const uint primStructIdx = allocData.x;
    const uint primStructOffset = allocData.y;

    DstBuffer.Store(offsets.primNodePtrs + (primStructIdx * sizeof(uint)), primStructOffset);

    const uint boxNodeOffset0 = packedParentInfo0 & ~bits(7);
    DstBuffer.Store(boxNodeOffset0 + QUANTIZED_BVH8_NODE_OFFSET_LEAF_NODE_BASE_OFFSET,
                    PackNodePointer(0, primStructOffset));

    if (pairCount > 1)
    {
        const uint boxNodeOffset1 = packedParentInfo1 & ~bits(7);
        const uint numBoxChildren = packedParentInfo1 &  bits(7);

        DstBuffer.Store(boxNodeOffset1 + QUANTIZED_BVH8_NODE_OFFSET_LEAF_NODE_BASE_OFFSET,
                        PackNodePointer(0, primStructOffset));

        const uint childInfoOffset =
            boxNodeOffset1 +
            QUANTIZED_BVH8_NODE_OFFSET_CHILD_INFO_0 +
            (numBoxChildren * QUANTIZED_NODE_CHILD_INFO_STRIDE) +
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
uint32_t UnpackSrcNodeIdx(
    uint boxNodeOffset)
{
    const uint32_t packedNodeIdx = DstBuffer.Load(boxNodeOffset + QUANTIZED_BVH8_NODE_OFFSET_OBB_MATRIX_INDEX);
    const uint32_t packedSourceBoxes = ExtractPackedSourceBoxBits(packedNodeIdx);
    const uint32_t bvh2SrcNodeIdx = packedNodeIdx & ~(PACKED_SOURCE_BOX_BIT_MASK);

    // Restore OBB matrix index to invalid
    DstBuffer.Store(boxNodeOffset + QUANTIZED_BVH8_NODE_OFFSET_OBB_MATRIX_INDEX,
                    PackObbMatrixIndexBits(INVALID_OBB, packedSourceBoxes));

    return bvh2SrcNodeIdx;
}

//======================================================================================================================
void CompressPrims3_1(
    uint threadId)
{
    const uint multiPrimBatchCount  = EncodeHwBvh::ReadCounter(STGB_COUNTER_IDX_PRIM_COMP_BATCH);
    const uint singlePrimBatchCount = EncodeHwBvh::ReadCounter(STGB_COUNTER_IDX_PRIM_COMP_BATCH_SINGLE);

    const uint maxNumWaves = STGB_THREADGROUP_SIZE / WaveGetLaneCount();
    const uint waveId = threadId / WaveGetLaneCount();

    // Encode multiple triangle batches
    for (uint batchId = waveId; batchId < multiPrimBatchCount; batchId += maxNumWaves)
    {
        const uint packedParentInfo = WaveReadLaneFirst(
            ScratchBuffer.Load(ShaderConstants.offsets.primCompNodeIndices + (batchId * sizeof(uint))));

        const CompBatchParentInfo parentInfo = UnpackCompBatchParentInfo(packedParentInfo);

        const uint boxNodeOffset = parentInfo.boxNodeOffset;
        const uint numBoxChildren = parentInfo.numBoxChildren;
        const uint numLeafChildren = parentInfo.numLeafChildren;

        uint64_t primRangeStartMask = 0;
        uint64_t primRangeEndMask = 0;
        uint32_t primRefCount = 0;

        const uint32_t bvh2SrcNodeIdx = UnpackSrcNodeIdx(boxNodeOffset);

        // Load child info on independent threads
        const uint waveLdsOffset = waveId * (WaveGetLaneCount() * MAX_ELEMENT_PER_THREAD);
        if (WaveGetLaneIndex() < numLeafChildren)
        {
            // Load leaf child node indices from parent scratch memory
            uint nodePtr = ScratchBuffer.Load(GetBvh2NodeOffset(bvh2SrcNodeIdx) + (WaveGetLaneIndex() * sizeof(uint)));

            // Each lane reads the primitive reference count for a child then walks the linked list to find the nodes
            primRefCount = GetBvh2PrimitiveCount(nodePtr);

            uint ldsDstOffset = WavePrefixSum(primRefCount);
            primRangeStartMask |= bit64(ldsDstOffset);

            if (IsBvh2PrimRefNode(nodePtr))
            {
                SharedMem[waveLdsOffset + ldsDstOffset] = nodePtr;
                primRangeEndMask |= bit64(ldsDstOffset);
            }
            else
            {
                // TODO: Support primRange > 2
                uint2 child = ReadBvh2ChildPtrs(GetBvh2NodeIdx(nodePtr));

                SharedMem[waveLdsOffset + ldsDstOffset] = child.x;
                ldsDstOffset++;

                SharedMem[waveLdsOffset + ldsDstOffset] = child.y;
                primRangeEndMask |= bit64(ldsDstOffset);
            }
        }

        const uint batchPrimRefCount = WaveActiveSum(primRefCount);

        primRangeStartMask = WaveActiveBitOr(primRangeStartMask);
        primRangeEndMask = WaveActiveBitOr(primRangeEndMask);

        PrimStruct3_1::TriCompressState triState;
        triState.emitPrimStruct = false;

        if (WaveGetLaneIndex() < batchPrimRefCount)
        {
            bool compressMultiplePairs = Settings.primCompressionFlags & PrimCompFlags::MultiPrim;

            if (Settings.enableInstanceRebraid && (boxNodeOffset == sizeof(AccelStructHeader)))
            {
                // The primitive structure space for the children of a rebraided root node is pre-reserved in the BVH8 phase
                // after the box children. Disable compression across pairs to avoid leaving unused space after the root
                // node. This simplifies compaction.
                compressMultiplePairs = false;
            }

            const uint primRefIdx = WaveGetLaneIndex();

            const uint nodePtr = SharedMem[waveLdsOffset + primRefIdx];

            const uint2 id = ReadBvh2ChildPtrs(GetBvh2NodeIdx(nodePtr));
            const uint packedFlags = ReadBvh2NodeFlags(GetBvh2NodeIdx(nodePtr));
            const bool isTri0 = PrimStruct3_1::IsTriangle0(primRangeStartMask, primRefIdx);

            const TriangleData tri = FetchTrianglePrimitiveData(id.x, id.y);

            triState = PrimStruct3_1::ComputeCompressedRanges(ShaderConstants.header.offsets,
                                                              waveLdsOffset,
                                                              primRefIdx,
                                                              id.x,
                                                              id.y,
                                                              tri.v0,
                                                              tri.v1,
                                                              tri.v2,
                                                              primRangeStartMask,
                                                              primRangeEndMask,
                                                              numBoxChildren,
                                                              IsOpaqueNode(packedFlags),
                                                              isTri0,
                                                              compressMultiplePairs);
        }

        uint64_t emitPrimStructMask = WaveActiveBallot64(triState.emitPrimStruct);
        const uint primStructCount = countbits64(emitPrimStructMask);

        // Allocate prim structs
        uint primStructOffset = 0;
        uint primStructIdx = 0;

        if (WaveIsFirstLane())
        {
            const uint2 allocData = AllocPrimStructs(packedParentInfo, primStructCount);

            primStructIdx = allocData.x;
            primStructOffset = allocData.y;

            DstBuffer.Store(boxNodeOffset + QUANTIZED_BVH8_NODE_OFFSET_LEAF_NODE_BASE_OFFSET,
                            PackNodePointer(0, primStructOffset));
        }

        primStructOffset = WaveReadLaneFirst(primStructOffset);

        PrimStruct3_1::EmitPrimitiveStructures(ShaderConstants.header.offsets,
                                               waveLdsOffset,
                                               triState,
                                               boxNodeOffset,
                                               numBoxChildren,
                                               primRangeEndMask,
                                               emitPrimStructMask,
                                               primStructIdx,
                                               primStructOffset);
    }

    // Encode single primitive batches. Note, we compress two triangles per thread into a single primitive packet
    const uint batchThreadCount = RoundUpQuotient(singlePrimBatchCount, 2);
    if (threadId < batchThreadCount)
    {
        // The single primitive list starts at the end of the data section and grows down
        const uint basePrimListOffset =
            ShaderConstants.offsets.primCompNodeIndicesEnd - (singlePrimBatchCount * sizeof(uint));

        const uint offset = basePrimListOffset + (threadId * sizeof(uint2));

        const uint2 packedParentInfo = ScratchBuffer.Load2(offset);

        // Only one node is valid at the end of the buffer
        const bool tri1Valid = ((threadId * 2 + 1) < singlePrimBatchCount);

        const CompBatchParentInfo parentInfo0 = UnpackCompBatchParentInfo(packedParentInfo.x);
        const uint32_t bvh2SrcNodeIdx0 = UnpackSrcNodeIdx(parentInfo0.boxNodeOffset);

        uint flags0 = ReadBvh2NodeFlags(GetBvh2NodeIdx(bvh2SrcNodeIdx0));
        uint2 id0 = ReadBvh2ChildPtrs(GetBvh2NodeIdx(bvh2SrcNodeIdx0));
        const TriangleData tri0 = FetchTrianglePrimitiveData(id0.x, id0.y);

        uint flags1 = 0;
        uint2 id1 = uint2(0, 0);
        TriangleData tri1 = (TriangleData)0;

        if (tri1Valid)
        {
            const CompBatchParentInfo parentInfo1 = UnpackCompBatchParentInfo(packedParentInfo.y);
            const uint32_t bvh2SrcNodeIdx1 = UnpackSrcNodeIdx(parentInfo1.boxNodeOffset);

            flags1 = ReadBvh2NodeFlags(GetBvh2NodeIdx(bvh2SrcNodeIdx1));
            id1 = ReadBvh2ChildPtrs(GetBvh2NodeIdx(bvh2SrcNodeIdx1));
            tri1 = FetchTrianglePrimitiveData(id1.x, id1.y);
        }

        WriteTwoTriPrimStruct(ShaderConstants.header.offsets,
                              tri1Valid ? 2 : 1,
                              tri0,
                              tri1,
                              id0.x,
                              id1.x,
                              id0.y,
                              id1.y,
                              flags0,
                              flags1,
                              packedParentInfo.x,
                              packedParentInfo.y);
    }
}
