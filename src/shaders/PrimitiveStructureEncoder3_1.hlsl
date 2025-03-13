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
#ifndef _PRIMITIVE_STRUCTURE_ENCODER_3_1_HLSL
#define _PRIMITIVE_STRUCTURE_ENCODER_3_1_HLSL
#include "../shadersClean/common/PrimitiveStructure.hlsli"

// A batch is the immediate triangle children of one internal node (max one prim range * 8 children)
#define COMPRESSION_BATCH_SIZE (PRIM_COMP_MAX_RANGE_SIZE_3_1 * 8)

// Cluster scan ops are required to increase this to 2
#define COMPRESSION_BATCHES_PER_GROUP 1

//======================================================================================================================
struct CompBatchParentInfo
{
    uint boxNodeOffset;
    uint numBoxChildren;
    uint numLeafChildren;
};

//======================================================================================================================
uint PackCompBatchParentInfo(
    uint boxNodeOffset,
    uint numBoxChildren,
    uint numLeafChildren)
{
    // Pack numBoxChildren and numLeafChildren bits in the 128-byte aligned boxNodeOffset
    // numBoxChildren    :  3 // Maximum value is 7 since we only queue internal nodes with at least one leaf node
    // numLeafChildren   :  4 // Maximum valid is 8 as an BVH8 internal node can have maximum of 8 leaf children
    // currentNodeOffset : 25 // 128-byte aligned node offset. Lower 7 bits are guaranteed to be 0
    //
    const uint packedParentInfo = boxNodeOffset | (numLeafChildren << 3) | numBoxChildren;
    return packedParentInfo;
}

//======================================================================================================================
CompBatchParentInfo UnpackCompBatchParentInfo(
    uint packedParentInfo)
{
    const CompBatchParentInfo info =
    {
        packedParentInfo & ~bits(7),
        packedParentInfo & bits(3),
        (packedParentInfo >> 3) & bits(4),
    };

    return info;
}

//======================================================================================================================
uint GetLdsOffsetPrimStruct(uint baseLdsOffset)
{
    return baseLdsOffset + g_laneGroup.groupIndex * PRIMITIVE_STRUCT_SIZE_IN_DW;
}

//======================================================================================================================
uint GetLdsOffsetVertex(uint baseLdsOffset)
{
    // Vertex data follows the 128-byte primitive packet data in LDS
    return baseLdsOffset +
           (PRIMITIVE_STRUCT_SIZE_IN_DW * COMPRESSION_BATCHES_PER_GROUP) +
           (g_laneGroup.groupIndex * (COMPRESSION_BATCH_SIZE * 9));
}

namespace PrimStruct3_1 {
//======================================================================================================================
// Primitive structure encoder object. Mangages the data layout and writing of primitive structures.
struct Encoder
{
    uint   baseLdsOffset; // Offset in group shared memory where the primitive packet is constructed

    float3 prefixSource;

    uint triPairCount;

    uint payloadXLength;
    uint payloadYLength;
    uint payloadZLength;

    uint trailingZeroLength;

    uint geoIdAnchor;
    uint geoIdAnchorSize;
    uint geoIdPayloadSize;

    uint primIdAnchor;
    uint primIdAnchorSize;
    uint primIdPayloadSize;

    uint indexSectionMidpoint;

    Encoder ReadFromLane(uint index)
    {
        Encoder newEncoder;

        newEncoder.baseLdsOffset = g_laneGroup.Broadcast(baseLdsOffset, index);

        newEncoder.prefixSource = g_laneGroup.Broadcast(prefixSource, index);

        newEncoder.triPairCount = g_laneGroup.Broadcast(triPairCount, index);

        newEncoder.payloadXLength = g_laneGroup.Broadcast(payloadXLength, index);
        newEncoder.payloadYLength = g_laneGroup.Broadcast(payloadYLength, index);
        newEncoder.payloadZLength = g_laneGroup.Broadcast(payloadZLength, index);

        newEncoder.trailingZeroLength = g_laneGroup.Broadcast(trailingZeroLength, index);

        newEncoder.geoIdAnchor      = g_laneGroup.Broadcast(geoIdAnchor, index);
        newEncoder.geoIdAnchorSize  = g_laneGroup.Broadcast(geoIdAnchorSize, index);
        newEncoder.geoIdPayloadSize = g_laneGroup.Broadcast(geoIdPayloadSize, index);

        newEncoder.primIdAnchor      = g_laneGroup.Broadcast(primIdAnchor, index);
        newEncoder.primIdAnchorSize  = g_laneGroup.Broadcast(primIdAnchorSize, index);
        newEncoder.primIdPayloadSize = g_laneGroup.Broadcast(primIdPayloadSize, index);

        newEncoder.indexSectionMidpoint = g_laneGroup.Broadcast(indexSectionMidpoint, index);

        return newEncoder;
    }

    uint PrefixXLength()
    {
        return 32 - trailingZeroLength - payloadXLength;
    }

    uint PrefixYLength()
    {
        return 32 - trailingZeroLength - payloadYLength;
    }

    uint PrefixZLength()
    {
        return 32 - trailingZeroLength - payloadZLength;
    }

    uint VertexPayloadLength()
    {
        return payloadXLength + payloadYLength + payloadZLength;
    }

    uint VertexPrefixesLength()
    {
        return PrefixXLength() + PrefixYLength() + PrefixZLength();
    }

    void WriteDw(uint offset, uint data)
    {
        SharedMem[GetLdsOffsetPrimStruct(baseLdsOffset) + offset] = data;
    }

    void OrSharedMemBits(uint offset, uint data)
    {
        InterlockedOr(SharedMem[GetLdsOffsetPrimStruct(baseLdsOffset) + offset], data);
    }

    // Write value at the specified bit offset in the LDS primitive structure
    void WritePackedBits(
        uint startBitOffset, // Offset of the bit from the beginning of the structure
        uint length,         // Length of the bits to write
        uint value)          // Value to store in memory
    {
        if (length > 0)
        {
            const uint startDwOffset = startBitOffset / 32u;
            const uint shiftBitsInDw = startBitOffset % 32u;

            const uint64_t value64 = uint64_t(value) << shiftBitsInDw;

            OrSharedMemBits(startDwOffset, LowPart(value64));
            if ((length + shiftBitsInDw) >= 32)
            {
                OrSharedMemBits(startDwOffset + 1, HighPart(value64));
            }
        }
    }

    void WriteVertexPrefixes(float3 prefixes)
    {
        const uint x = (asuint(prefixes.x) >> (32 - PrefixXLength()));
        const uint y = (asuint(prefixes.y) >> (32 - PrefixYLength()));
        const uint z = (asuint(prefixes.z) >> (32 - PrefixZLength()));

        WritePackedBits(TRI_STRUCT_HEADER_SIZE_BITS, PrefixXLength(), x);
        WritePackedBits(TRI_STRUCT_HEADER_SIZE_BITS + PrefixXLength(), PrefixYLength(), y);
        WritePackedBits(TRI_STRUCT_HEADER_SIZE_BITS + PrefixXLength() + PrefixYLength(), PrefixZLength(), z);
    }

    void WritePrimIdAnchor(uint anchor)
    {
        WritePackedBits(indexSectionMidpoint, primIdAnchorSize, anchor);
    }

    void WriteGeoIdAnchor(uint anchor)
    {
        WritePackedBits((indexSectionMidpoint - geoIdAnchorSize), geoIdAnchorSize, anchor);
    }

    void WriteGeoIdPayload(uint triIndex, uint geoId)
    {
        if (triIndex > 0)
        {
            const uint startOffset = indexSectionMidpoint - geoIdAnchorSize - triIndex * geoIdPayloadSize;
            WritePackedBits(startOffset, geoIdPayloadSize, geoId & bits(geoIdPayloadSize));
        }
    }

    void WritePrimIdPayload(uint triIndex, uint primId)
    {
        if (triIndex > 0)
        {
            const uint startOffset = indexSectionMidpoint + primIdAnchorSize + (triIndex - 1) * primIdPayloadSize;
            WritePackedBits(startOffset, primIdPayloadSize, primId & bits(primIdPayloadSize));
        }
    }

    void WriteVertexComponent(const uint startBitOffset, const uint payloadLength, float f)
    {
        const uint component = asuint(f) >> trailingZeroLength;
        WritePackedBits(startBitOffset, payloadLength, component & bits(payloadLength));
    }

    void WriteVertex(uint vertexIndex, float3 vertex)
    {
        const uint sectionOffset = TRI_STRUCT_HEADER_SIZE_BITS + VertexPrefixesLength();

        if (Settings.geometryType == GEOMETRY_TYPE_AABBS)
        {
            // store full vertex for Procedural nodes
            const uint32 dwBitLength = 0x20;
            uint32_t offset = TRI_STRUCT_HEADER_SIZE_BITS + vertexIndex * dwBitLength * 3;
            WriteVertexComponent(offset, dwBitLength, vertex.x);
            offset += dwBitLength;
            WriteVertexComponent(offset, dwBitLength, vertex.y);
            offset += dwBitLength;
            WriteVertexComponent(offset, dwBitLength, vertex.z);
        }
        else
        {
            uint startBitOffset = sectionOffset + vertexIndex * VertexPayloadLength();
            WriteVertexComponent(startBitOffset, payloadXLength, vertex.x);

            startBitOffset += payloadXLength;
            WriteVertexComponent(startBitOffset, payloadYLength, vertex.y);

            startBitOffset += payloadYLength;
            WriteVertexComponent(startBitOffset, payloadZLength, vertex.z);
        }
    }

    void WriteTriPairDesc(uint triPairIndex, uint desc)
    {
        const uint startOffset = 1024 - TRI_PAIR_DESC_SIZE * (triPairIndex + 1);
        WritePackedBits(startOffset, TRI_PAIR_DESC_SIZE, desc);
    }

    void WriteMetadata()
    {
        const uint headerLo = PackMetadataHeaderBitsLo(
            payloadXLength - 1,
            payloadYLength - 1,
            payloadZLength - 1,
            trailingZeroLength,
            geoIdAnchorSize / 2,
            geoIdPayloadSize / 2,
            triPairCount - 1,
            false /* unorm16 */);

        const uint headerHi = PackMetadataHeaderBitsHi(
            primIdAnchorSize,
            primIdPayloadSize,
            indexSectionMidpoint);

        WriteDw(0, headerLo);
        WriteDw(1, headerHi);

        WriteVertexPrefixes(prefixSource);
        WritePrimIdAnchor(primIdAnchor);
        WriteGeoIdAnchor(geoIdAnchor);
    }

    bool TryInit(
        float3 prefixSource_,
        uint3  planeDiffs,
        uint   planeUnion,
        uint   primIdDiff,
        uint   geoIdDiff,
        uint   primIdAnchor_,
        uint   geoIdAnchor_,
        uint   uniqueVertexCount,
        uint   triPairCount_)
    {
        prefixSource = prefixSource_;
        primIdAnchor = primIdAnchor_;
        geoIdAnchor  = geoIdAnchor_;
        triPairCount = triPairCount_;

        const bool enableTrailingZeroCompression =
            Settings.primCompressionFlags & PrimCompFlags::EnableTrailingZeroAndPrefix;

        trailingZeroLength = enableTrailingZeroCompression ? CommonTrailingZeroBits(planeUnion) : 0;

        uint3 prefixLengths = enableTrailingZeroCompression ? CommonPrefixBits(planeDiffs) : uint3(0, 0, 0);

        prefixLengths = min(prefixLengths, (31 - trailingZeroLength).xxx);

        const uint vertexSize =	96 - (prefixLengths.x + prefixLengths.y + prefixLengths.z + 3 * trailingZeroLength);

        primIdPayloadSize = 32 - LeadingZeroBits(primIdDiff);
        primIdAnchorSize  = 32 - LeadingZeroBits(primIdAnchor);

        // Note, geometry ID payload and anchor lengths are encoded as multiple of 2. We need to align the lengths here
        // to account for that
        geoIdPayloadSize = Pow2Align(32 - LeadingZeroBits(geoIdDiff), 2);
        geoIdAnchorSize  = Pow2Align(32 - LeadingZeroBits(geoIdAnchor), 2);

        const uint pairDescDataSize  = triPairCount * TRI_PAIR_DESC_SIZE;
        const uint idPayloadDataSize = ((2 * triPairCount) - 1) * (primIdPayloadSize + geoIdPayloadSize);
        const uint vertexDataSize    = uniqueVertexCount * vertexSize;
        const uint anchorSizes
            = prefixLengths.x + prefixLengths.y + prefixLengths.z + primIdAnchorSize + geoIdAnchorSize;
        const uint requisiteBits
            = pairDescDataSize + idPayloadDataSize + vertexDataSize + anchorSizes;

        const bool valid =
            ((requisiteBits <= TRI_STRUCT_DATA_SECTION_SIZE_BITS) && (uniqueVertexCount <= 16) && (triPairCount <= 8));

        if (valid)
        {
            GPU_ASSERT((prefixLengths.x + trailingZeroLength) <= 31);
            GPU_ASSERT((prefixLengths.y + trailingZeroLength) <= 31);
            GPU_ASSERT((prefixLengths.z + trailingZeroLength) <= 31);

            payloadXLength = (32 - prefixLengths.x - trailingZeroLength);
            payloadYLength = (32 - prefixLengths.y - trailingZeroLength);
            payloadZLength = (32 - prefixLengths.z - trailingZeroLength);

            indexSectionMidpoint = ComputeIndexSectionMidpoint(triPairCount, primIdPayloadSize, primIdAnchorSize);
        }

        // One pair should always fit
        GPU_ASSERT(valid || (triPairCount > 1));

        return valid;
    }
};

struct TriCompressState
{
    // State valid for a lane with a complete prim struct
    VertexMask vertexMask;
    Encoder encoder;

    // Per-lane/primitive state
    uint basePrimIndex;
    uint triPairIdx;
    uint pairDesc;
    uint geometryId;
    uint primId;
    uint childIdx;
    bool isTri1;
    bool completePair;
    bool primRangeStart;
    bool emitPrimStruct;
};

//======================================================================================================================
bool IsTriangle0(
    uint64_t primRangeStartMask,
    uint32_t primListIndex)
{
    // Mask off start bits above this lane's index
    const uint64_t lowerPrimStartMask = primRangeStartMask & bits64(primListIndex + 1);
    // Search the start mask from the end down to find the last start point
    const uint lastPrimStart = FIRSTBITHIGH_U64(lowerPrimStartMask);
    const uint triIndexInRange = primListIndex - lastPrimStart;

    const bool isTri0 = (triIndexInRange & 1) == 0;
    return isTri0;
}

//======================================================================================================================
// Determine which primitives to compress together. Each primitive is assigned to a lane. On each iteration of the
// main loop, each lane attempts to compress with all active preceeding lanes. The highest successful lane determines
// a final primitive structure.
static TriCompressState ComputeCompressedRanges(
    AccelStructOffsets offsets,
    uint     baseLdsOffset,
    uint     primListIndex,
    uint     geomId,
    uint     primId,
    float3   v0,
    float3   v1,
    float3   v2,
    uint64_t primRangeStartMask,
    uint64_t primRangeEndMask,
    uint     numBoxChildren,
    bool     isOpaque,
    bool     isTri0,
    bool     compressMultiplePairs)
{
    const uint maxPairs = compressMultiplePairs ? 8 : 1;

    const uint childIdx       = countbits64(primRangeStartMask & bits64(primListIndex)) + numBoxChildren;
    const bool primRangeStart = (primRangeStartMask & bit64(primListIndex));
    const bool primRangeEnd   = (primRangeEndMask & bit64(primListIndex));

    const bool isTri1 = (isTri0 == false);

    // This lane is the end of a pair if it's tri1 or tri0 at the end of a range
    const bool completePair = primRangeEnd || isTri1;

    uint triPairIdx = 0;

    uint primsDone = 0;

    TrianglePairDesc pairDesc;

    StoreVert(baseLdsOffset, primListIndex * 3 + 0, v0);
    StoreVert(baseLdsOffset, primListIndex * 3 + 1, v1);
    StoreVert(baseLdsOffset, primListIndex * 3 + 2, v2);

    uint numPrimsDone  = 0;
    uint basePrimIndex = 0;

    Encoder encoder;
    encoder.baseLdsOffset = baseLdsOffset;

    VertexMask vertexMask;
    uint uniqueVertexCount = 0;

    bool emitPrimStruct = false;
    bool triDone = false;

    while (triDone == false)
    {
        basePrimIndex = numPrimsDone;

        const uint3 prefixSource = g_laneGroup.ReadFirstActiveLane(asuint(v0));

        const uint primIdAnchor = g_laneGroup.ReadFirstActiveLane(primId);
        const uint geomIdAnchor = g_laneGroup.ReadFirstActiveLane(geomId);

        if (g_laneGroup.IsFirstActiveLane())
        {
            GPU_ASSERT(numPrimsDone == g_laneGroup.laneIndex);
        }

        const uint3 index =
            FindVerts(baseLdsOffset, basePrimIndex, primListIndex, v0, v1, v2);

        uint3 planeDiffs =
            (asuint(v0) ^ prefixSource) |
            (asuint(v1) ^ prefixSource) |
            (asuint(v2) ^ prefixSource);

        planeDiffs = g_laneGroup.PostfixOr(planeDiffs);

        uint planeUnion =
            asuint(v0.x) | asuint(v0.y) | asuint(v0.z) |
            asuint(v1.x) | asuint(v1.y) | asuint(v1.z) |
            asuint(v2.x) | asuint(v2.y) | asuint(v2.z);

        planeUnion = g_laneGroup.PostfixOr(planeUnion);

        uint geoIdDiff  = g_laneGroup.PostfixOr(geomIdAnchor ^ geomId);
        uint primIdDiff = g_laneGroup.PostfixOr(primIdAnchor ^ primId);

        vertexMask.Init();
        vertexMask.SetBit(index[0]);
        vertexMask.SetBit(index[1]);
        vertexMask.SetBit(index[2]);
        vertexMask.PostfixOr();

        uniqueVertexCount = vertexMask.CountBits();

        const uint triPairDescCount = g_laneGroup.ExclusivePrefixSumBool(isTri0) + isTri0;

        bool valid = false;

        if ((triPairDescCount <= maxPairs) && completePair)
        {
            valid = encoder.TryInit(asfloat(prefixSource),
                                    planeDiffs,
                                    planeUnion,
                                    primIdDiff,
                                    geoIdDiff,
                                    primIdAnchor,
                                    geomIdAnchor,
                                    uniqueVertexCount,
                                    triPairDescCount);
        }

        const uint64_t validMask = g_laneGroup.Ballot(valid);
        GPU_ASSERT(validMask != 0);
        const uint lastValidTri = FIRSTBITHIGH_U64(validMask);

        if (primListIndex <= lastValidTri)
        {
            emitPrimStruct = (primListIndex == lastValidTri);

            uint3 vertIndicesInStruct = vertexMask.CompactIndices(index);

            pairDesc.Init();
            pairDesc.InitTri0(vertIndicesInStruct, isOpaque, primId, geomId);

            const uint prevLanePairDesc = g_laneGroup.Broadcast(pairDesc.triDesc, g_laneGroup.laneIndex - 1);

            if (completePair)
            {
                if (isTri1)
                {
                    // This lane is tri1, shift the tri0 data into the tri1 position
                    pairDesc.triDesc >>= (TRI_DESC_TRI0_DOUBLE_SIDED_SHIFT - 1);
                    // Or in the tri0 data from the previous lane
                    pairDesc.triDesc |= prevLanePairDesc;
                }

                pairDesc.SetPrimRangeStopBit(primRangeEnd);
            }

            triPairIdx = triPairDescCount - 1;

            triDone = true;
        }

        numPrimsDone = lastValidTri + 1;
    }

    TriCompressState outputState;

    outputState.encoder = encoder;
    outputState.vertexMask = vertexMask;
    outputState.basePrimIndex = basePrimIndex;
    outputState.triPairIdx = triPairIdx;
    outputState.pairDesc = pairDesc.GetData();
    outputState.geometryId = geomId;
    outputState.primId = primId;
    outputState.childIdx = childIdx;
    outputState.isTri1 = isTri1;
    outputState.completePair = completePair;
    outputState.primRangeStart = primRangeStart;
    outputState.emitPrimStruct = emitPrimStruct;

    return outputState;
}

//======================================================================================================================
static void EmitPrimitiveStructures(
    AccelStructOffsets offsets,
    uint baseLdsOffset,
    TriCompressState triState,
    uint parentBoxOffset,
    uint numBoxChildren,
    uint64_t primRangeEndMask,
    uint64_t emitPrimStructMask,
    uint primStructIdx,
    uint primStructOffset)
{
    const uint primStructCount = countbits64(emitPrimStructMask);

    const uint primListIndex = g_laneGroup.laneIndex;

    if (primListIndex < primStructCount)
    {
        // Write prim struct offsets
        const uint lanePrimStructIdx = g_laneGroup.ReadFirstLane(primStructIdx) + primListIndex;
        const uint lanePrimStructOffset = primStructOffset + PRIMITIVE_STRUCT_SIZE_IN_BYTE * primListIndex;
        DstBuffer.Store(offsets.primNodePtrs + (lanePrimStructIdx * sizeof(uint)), lanePrimStructOffset);
    }

    // For each prim struct, each lane writes to LDS, then data is copied to memory
    uint firstPrimListIndex = 0;
    while (emitPrimStructMask != 0)
    {
        const uint emitIndex = FIRSTBITLOW_U64(emitPrimStructMask);

        for (uint i = g_laneGroup.laneIndex; i < PRIMITIVE_STRUCT_SIZE_IN_DW; i += g_laneGroup.groupSize)
        {
            SharedMem[GetLdsOffsetPrimStruct(baseLdsOffset) + i] = 0;
        }

        GroupMemoryBarrierWithGroupSync();

        Encoder emitEncoder = triState.encoder.ReadFromLane(emitIndex);

        if (g_laneGroup.IsFirstActiveLane())
        {
            emitEncoder.WriteMetadata();
        }

        GroupMemoryBarrierWithGroupSync();

        VertexMask emitVertexMask = triState.vertexMask.ReadFromLane(emitIndex);

        const uint basePrimIndex = g_laneGroup.Broadcast(triState.basePrimIndex, emitIndex);

        for (uint vert = g_laneGroup.laneIndex; vert < COMPRESSION_BATCH_SIZE * 3; vert += g_laneGroup.groupSize)
        {
            if (emitVertexMask.TestBit(vert))
            {
                const uint vertIndexInStruct = emitVertexMask.CompactIndex(vert);
                const uint absoluteVertexIndex = vert + basePrimIndex * 3;
                emitEncoder.WriteVertex(vertIndexInStruct, ReadVert(baseLdsOffset, absoluteVertexIndex));
            }
        }

        if ((primListIndex >= firstPrimListIndex) && (primListIndex <= emitIndex))
        {
            if (triState.completePair)
            {
                emitEncoder.WriteTriPairDesc(triState.triPairIdx, triState.pairDesc);
            }

            const uint triIdx = (triState.triPairIdx * 2) + triState.isTri1;

            emitEncoder.WriteGeoIdPayload(triIdx, triState.geometryId);
            emitEncoder.WritePrimIdPayload(triIdx, triState.primId);

            const uint nodeType = (triState.triPairIdx >= 4) ? (triState.triPairIdx + 4) : triState.triPairIdx;

            if (triState.primRangeStart)
            {
                // Find the first range end bit at or above the current position
                const uint rangeEnd = FIRSTBITLOW_U64(primRangeEndMask & ~bits64(primListIndex));
                // Mask of primitives in the range
                const uint64_t rangeMask = bits64(rangeEnd + 1) - bits64(primListIndex);
                // Count the packets that will be emitted in the range
                const uint rangePacketCount = countbits64(rangeMask & emitPrimStructMask);
                GPU_ASSERT(rangePacketCount < 16);

                // Update child type in the box node
                const uint childInfoOffset =
                    parentBoxOffset +
                    QUANTIZED_BVH8_NODE_OFFSET_CHILD_INFO_0 +
                    (triState.childIdx * QUANTIZED_NODE_CHILD_INFO_STRIDE) +
                    QUANTIZED_NODE_CHILD_INFO_OFFSET_MAXY_MAXZ_NODE_TYPE_AND_RANGE;

                DstBuffer.InterlockedOr(childInfoOffset, (nodeType << 24) | (rangePacketCount << 28));
            }
        }

        GroupMemoryBarrierWithGroupSync();

        for (uint i = g_laneGroup.laneIndex; i < PRIMITIVE_STRUCT_SIZE_IN_DW; i += g_laneGroup.groupSize)
        {
            DstBuffer.Store(primStructOffset + i * 4,
                            SharedMem[GetLdsOffsetPrimStruct(baseLdsOffset) + i]);
        }

        primStructOffset += PRIMITIVE_STRUCT_SIZE_IN_BYTE;

        emitPrimStructMask &= emitPrimStructMask - 1;
        firstPrimListIndex = emitIndex + 1;
    }
}

//=====================================================================================================================
// Create a Primitive Structure containing a single triangle pair
// NOTE: This function is used by the Trivial Builder, so it cannot use scratch memory or LDS.
static void WritePairPrimStruct(
    TriangleData tri0,
    bool         tri0Opaque,
    uint         tri0PrimitiveIndex,
    uint         tri0GeometryIndex,
    bool         isTri1Valid,
    TriangleData tri1,
    bool         tri1Opaque,
    uint         tri1PrimitiveIndex,
    uint         tri1GeometryIndex,
    uint         nodeOffset)
{
    uint d[PRIMITIVE_STRUCT_SIZE_IN_DW];

    for (uint i = 0; i < PRIMITIVE_STRUCT_SIZE_IN_DW; ++i)
    {
        d[i] = 0;
    }

    const uint indexSectionMidpoint = ComputeIndexSectionMidpoint(1, 31, 31);

    if (Settings.geometryType == GEOMETRY_TYPE_AABBS)
    {
        // HW would detect "malformed_vtx" based on index and payloadlength
        // Since procedural node index are 0xF
        // here we manually pack payloadLength to be 1 (payloadLengthMinusOne = 0)
        // in order to prevent HW decode proceduralNodes as "malformed_vtx"
        //
        // i.e., procedural node needs to be packed as:
        // payloadXLengthMinusOne = 0;      // decode as 1
        // payloadYLengthMinusOne = 0;      // decode as 1
        // payloadZLengthMinusOne = 0;      // decode as 1
        // trailingZeroLength     = 0;

        d[0] = PackMetadataHeaderBitsLo(0, 0, 0, 0, 15, 15, 0, false);
        d[1] = PackMetadataHeaderBitsHi(31, 31, indexSectionMidpoint);

        // tri0 v0 is encoded as float3(0, 0, 0) to report invalid triangle vertices as origin.

        // AABB min/max are written to unused section of the primitive packet.
        d[5]  = asuint(tri0.v0.x);
        d[6]  = asuint(tri0.v0.y);
        d[7]  = asuint(tri0.v0.z);
        d[8]  = asuint(tri0.v1.x);
        d[9]  = asuint(tri0.v1.y);
        d[10] = asuint(tri0.v1.z);

        // Geometry IDs
        d[28] = bitFieldInsert(d[28], 7, 25, tri0GeometryIndex);
        d[29] = bitFieldInsert(0,     0,  5, tri0GeometryIndex >> 25);

        // Index section midpoint

        // Primitive IDs
        d[29] = bitFieldInsert(d[29], 5, 27, tri0PrimitiveIndex);
        d[30] = bitFieldInsert(d[30], 0,  4, tri0PrimitiveIndex >> 27);

        // Triangle descriptor
        uint triDesc = 1 << TRI_DESC_PRIM_RANGE_STOP_BIT_SHIFT;
        // Mark this node as procedural by setting triangle 0 indices to 0xff, 0xff, x
        PackedTriDescSetTriangle(0, 0xff, 0xff, 0xff, (uint) tri0Opaque, triDesc);
        // Encode triangle 1 as degenerate triangle at vertex0.
        PackedTriDescSetTriangle(1, 0, 0, 0, 0, triDesc);

        d[31] = bitFieldInsert(d[31], 3, 29, triDesc);
    }
    else
    {
        // Store full vertex with no compression
        //
        // Each vertex is stored as 3 DW (32bit) without prefix
        //
        // Header ----------- 52 bits
        // Vertex Data ------ 576 bits (96 * 3 * 2)
        // ----- Unused ----- 245 bits
        // geometryId1Payload 30 bits
        // geometryIdAnchor-- 30 bits
        //------------------- indexSectionMidpoint
        // primIdAnchor ----- 31 bits
        // primId1Payload---- 31 bits
        // triDesc----------- 29 bits
        //------------------- 1024 bits
        d[0] = PackMetadataHeaderBitsLo(31, 31, 31, 0, 15, 15, 0, false);

        d[1] = PackMetadataHeaderBitsHi(31, 31, indexSectionMidpoint);

        uint triDesc = 1 << TRI_DESC_PRIM_RANGE_STOP_BIT_SHIFT;

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

        PackedTriDescSetTriangle(0, 0, 1, 2, (uint) tri0Opaque, triDesc);

        if (isTri1Valid)
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

            PackedTriDescSetTriangle(1, 3, 4, 5, (uint) tri1Opaque, triDesc);

            // Geometry IDs
            d[27] = bitFieldInsert(0, 9, 23, tri1GeometryIndex);
            d[28] = bitFieldInsert(0, 0,  7, tri1GeometryIndex >> 23);

            // Primitive IDs
            d[30] = bitFieldInsert(0, 4, 28, tri1PrimitiveIndex);
            d[31] = bitFieldInsert(0, 0,  3, tri1PrimitiveIndex >> 28);
        }
        else
        {
            // Set tri1 to invalid
            PackedTriDescSetInvalidTriangle(1, triDesc);
        }

        // Geometry IDs
        d[28] = bitFieldInsert(d[28], 7, 25, tri0GeometryIndex);
        d[29] = bitFieldInsert(0,     0,  5, tri0GeometryIndex >> 25);

        // Index section midpoint

        // Primitive IDs
        d[29] = bitFieldInsert(d[29], 5, 27, tri0PrimitiveIndex);
        d[30] = bitFieldInsert(d[30], 0,  4, tri0PrimitiveIndex >> 27);

        // Triangle descriptor
        d[31] = bitFieldInsert(d[31], 3, 29, triDesc);
    }

    [unroll]
    for (uint i = 0; i < PRIMITIVE_STRUCT_SIZE_IN_DW; i += 4)
    {
        DstBuffer.Store4(nodeOffset + (i*4), uint4(d[i], d[i+1], d[i+2], d[i+3]));
    }
}

} // namespace PrimStruct3_1
#endif
