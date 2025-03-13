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
// This header file contains shared definitions between the HLSL raytracing shaders and the prototype c++ code
//
#ifndef _GFX12_PRIMITIVE_NODE_H
#define _GFX12_PRIMITIVE_NODE_H

#include "../Math.hlsli"
#include "../ShaderDefs.hlsli"
#include "../Bits.hlsli"
#include "../../debug/Debug.hlsli"

//=====================================================================================================================
// Primitive Structure triangle size: 128 Byte / 32 Dwords
#define PRIMITIVE_STRUCT_SIZE_IN_BYTE   128
#define PRIMITIVE_STRUCT_SIZE_IN_DW     32
#define TRIANGLE_PAIR_COUNT_OFFSET      28
#define TRI_PAIR_DESC_SIZE              29

#define PRIM_RANGE_UPDATE 0xfffffff8
#define END_OF_PAIR       0 // Increment node type
#define END_OF_NODE       1 // Increment node offset and clear node type
#define END_OF_RANGE      3 // Return invalid node

#define TRI_VERTEX_IDX_NUM_BITS                 4
#define OPAQUE_NUM_BITS                         1
#define DOUBLE_SIDED_NUM_BITS                   1
#define PRIM_RANGE_STOP_NUM_BITS                1

#define TRI_DESC_PRIM_RANGE_STOP_BIT_SHIFT      0
#define TRI_DESC_TRI1_DOUBLE_SIDED_SHIFT        1
#define TRI_DESC_TRI1_OPAQUE_SHIFT              2
#define TRI_DESC_TRI1_V0_IDX_SHIFT              3
#define TRI_DESC_TRI1_V1_IDX_SHIFT              7
#define TRI_DESC_TRI1_V2_IDX_SHIFT              11
#define TRI_DESC_TRI0_DOUBLE_SIDED_SHIFT        15
#define TRI_DESC_TRI0_OPAQUE_SHIFT              16
#define TRI_DESC_TRI0_V0_IDX_SHIFT              17
#define TRI_DESC_TRI0_V1_IDX_SHIFT              21
#define TRI_DESC_TRI0_V2_IDX_SHIFT              25

//=====================================================================================================================
static void PackedTriDescSetTriangle(
    uint triangleIdx,
    uint i0,
    uint i1,
    uint i2,
    uint opaque,
    inout_param(uint) triDesc)
{
    if (triangleIdx == 0)
    {
        triDesc = bitFieldInsert(triDesc, TRI_DESC_TRI0_V0_IDX_SHIFT, TRI_VERTEX_IDX_NUM_BITS, i0);
        triDesc = bitFieldInsert(triDesc, TRI_DESC_TRI0_V1_IDX_SHIFT, TRI_VERTEX_IDX_NUM_BITS, i1);
        triDesc = bitFieldInsert(triDesc, TRI_DESC_TRI0_V2_IDX_SHIFT, TRI_VERTEX_IDX_NUM_BITS, i2);
        triDesc = bitFieldInsert(triDesc, TRI_DESC_TRI0_OPAQUE_SHIFT, OPAQUE_NUM_BITS, opaque);
    }
    else
    {
        triDesc = bitFieldInsert(triDesc, TRI_DESC_TRI1_V0_IDX_SHIFT, TRI_VERTEX_IDX_NUM_BITS, i0);
        triDesc = bitFieldInsert(triDesc, TRI_DESC_TRI1_V1_IDX_SHIFT, TRI_VERTEX_IDX_NUM_BITS, i1);
        triDesc = bitFieldInsert(triDesc, TRI_DESC_TRI1_V2_IDX_SHIFT, TRI_VERTEX_IDX_NUM_BITS, i2);
        triDesc = bitFieldInsert(triDesc, TRI_DESC_TRI1_OPAQUE_SHIFT, OPAQUE_NUM_BITS, opaque);
    }
}

//=====================================================================================================================
static void PackedTriDescSetInvalidTriangle(
    uint triangleIdx,
    inout_param(uint) triDesc)
{
    PackedTriDescSetTriangle(triangleIdx, 0, 0, 0, 0, triDesc);
}

//=====================================================================================================================
static uint ComputeIndexSectionMidpoint(
    uint triPairCount,
    uint primIdPayloadLength,
    uint primIdAnchorLength)
{
    return 1024u -
           (triPairCount * TRI_PAIR_DESC_SIZE) -
           (((2 * triPairCount) - 1) * primIdPayloadLength) -
           primIdAnchorLength;
}

//=====================================================================================================================
static uint PackMetadataHeaderBitsLo(
    uint payloadXLengthMinusOne,
    uint payloadYLengthMinusOne,
    uint payloadZLengthMinusOne,
    uint trailingZeroLength,
    uint geoIdAnchorSizeDivTwo,
    uint geoIdPayloadSizeDivTwo,
    uint triPairCountMinusOne,
    uint floatOrUnorm16)
{
    return (((payloadXLengthMinusOne & 0x1F) |
            ((payloadYLengthMinusOne & 0x1F) << 5) |
            ((payloadZLengthMinusOne & 0x1F) << 10) |
            ((trailingZeroLength & 0x1F) << 15) |
            ((geoIdAnchorSizeDivTwo & 0xF) << 20) |
            ((geoIdPayloadSizeDivTwo & 0xF) << 24) |
            ((triPairCountMinusOne & 0x7) << 28) |
            ((floatOrUnorm16 & 0x1) << 31)));
}

//=====================================================================================================================
static uint PackMetadataHeaderBitsHi(
    uint primIdAnchorSize,
    uint primIdPayloadSize,
    uint indexSectionMidpoint)
{
    return ((primIdAnchorSize & 0x1F) |
            ((primIdPayloadSize & 0x1F) << 5) |
            ((indexSectionMidpoint & 0x3FF) << 10));
}

//=====================================================================================================================
// Packed 32-bit hardware triangle pair descriptor
struct HwTrianglePairDesc
{
    uint triDesc; // 32-bit packed triangle pair descriptor

    void Init()
    {
        triDesc = 0;
    }

    uint GetData()
    {
        return triDesc;
    }

    void SetData(uint packedData)
    {
        triDesc = packedData;
    }

    uint Tri1V0()
    {
        return bitFieldExtract(triDesc, TRI_DESC_TRI1_V0_IDX_SHIFT, TRI_VERTEX_IDX_NUM_BITS);
    }

    uint Tri1V1()
    {
        return bitFieldExtract(triDesc, TRI_DESC_TRI1_V1_IDX_SHIFT, TRI_VERTEX_IDX_NUM_BITS);
    }

    uint Tri1V2()
    {
        return bitFieldExtract(triDesc, TRI_DESC_TRI1_V2_IDX_SHIFT, TRI_VERTEX_IDX_NUM_BITS);
    }

    bool Tri1Opaque()
    {
        return bitFieldExtract(triDesc, TRI_DESC_TRI1_OPAQUE_SHIFT, 1);
    }

    bool Tri1DoubleSided()
    {
        return bitFieldExtract(triDesc, TRI_DESC_TRI1_DOUBLE_SIDED_SHIFT, 1);
    }

    bool Tri1Valid()
    {
        return Tri1V0() != 0 || Tri1V1() != 0 || Tri1V2() != 0;
    }

    uint Tri0V0()
    {
        return bitFieldExtract(triDesc, TRI_DESC_TRI0_V0_IDX_SHIFT, TRI_VERTEX_IDX_NUM_BITS);
    }

    uint Tri0V1()
    {
        return bitFieldExtract(triDesc, TRI_DESC_TRI0_V1_IDX_SHIFT, TRI_VERTEX_IDX_NUM_BITS);
    }

    uint Tri0V2()
    {
        return bitFieldExtract(triDesc, TRI_DESC_TRI0_V2_IDX_SHIFT, TRI_VERTEX_IDX_NUM_BITS);
    }

    bool Tri0Opaque()
    {
        return bitFieldExtract(triDesc, TRI_DESC_TRI0_OPAQUE_SHIFT, OPAQUE_NUM_BITS);
    }

    bool Tri0DoubleSided()
    {
        return bitFieldExtract(triDesc, TRI_DESC_TRI0_DOUBLE_SIDED_SHIFT, DOUBLE_SIDED_NUM_BITS);
    }

    uint PrimRangeStopBit()
    {
        return bitFieldExtract(triDesc, TRI_DESC_PRIM_RANGE_STOP_BIT_SHIFT, PRIM_RANGE_STOP_NUM_BITS);
    }

    void SetPrimRangeStopBit(uint value)
    {
        triDesc = bitFieldInsert(triDesc, TRI_DESC_PRIM_RANGE_STOP_BIT_SHIFT, PRIM_RANGE_STOP_NUM_BITS, value);
    }

    void SetTri0(uint i0, uint i1, uint i2, bool opaque, uint primId, uint geoId)
    {
        PackedTriDescSetTriangle(0, i0, i1, i2, opaque, triDesc);
    }

    void SetTri1(uint i0, uint i1, uint i2, bool opaque, uint primId, uint geoId)
    {
        PackedTriDescSetTriangle(1, i0, i1, i2, opaque, triDesc);
    }
};

//=====================================================================================================================
// Unpacked triangle pair descriptor
struct TrianglePairDesc : HwTrianglePairDesc
{
    uint tri0PrimId;
    uint tri0GeoId;
    uint tri1PrimId;
    uint tri1GeoId;

    void InitTri0(uint3 index, bool opaque, uint primId, uint geoId)
    {
        SetTri0(index[0], index[1], index[2], opaque, primId, geoId);

        tri0PrimId = primId;
        tri0GeoId = geoId;
    }

    void InitTri1(uint3 index, bool opaque, uint primId, uint geoId)
    {
        SetTri1(index[0], index[1], index[2], opaque, primId, geoId);

        tri1PrimId = primId;
        tri1GeoId = geoId;
    }

    void InvalidateTri1()
    {
        SetTri1(0, 0, 0, false, 0, 0);
    }
};

//=====================================================================================================================
// TODO - combine TryInitTriStructHeader & Encode
//        Simplify "UnpackedPrimStructHeader" to reduce from 16DW
struct UnpackedPrimStructHeader
{
    float prefixSourceX;
    float prefixSourceY;
    float prefixSourceZ;
    uint anchorPrimId;
    uint anchorGeoId;
    uint indexSectionMidpoint;
    uint primIdAnchorSize;
    uint primIdPayloadSize;
    uint geoIdAnchorSizeDivTwo;
    uint geoIdPayloadSizeDivTwo;
    uint payloadXLengthMinusOne;
    uint payloadYLengthMinusOne;
    uint payloadZLengthMinusOne;
    uint trailingZeroLength;
    uint triPairCountMinusOne;
    bool floatOrUnorm16;
};

//=====================================================================================================================
#define TRI_STRUCT_HEADER_SIZE_BITS         52
#define TRI_STRUCT_DATA_SECTION_SIZE_BITS   (1024 - TRI_STRUCT_HEADER_SIZE_BITS)

#define PAYLOAD_NUM_BITS                            5
#define TRIALING_ZERO_NUM_BITS                      5
#define GEO_ID_ANCHOR_NUM_BITS                      4
#define GEO_ID_PAYLOAD_NUM_BITS                     4
#define TRIPAIR_COUNT_NUM_BITS                      3
#define VERTEX_TYPE_NUM_BITS                        1
#define PRIM_ID_ANCHOR_NUM_BITS                     5
#define PRIM_ID_PAYLOAD_NUM_BITS                    5
#define INDEX_SECTION_MID_POINT_NUM_BITS            10

#define PAYLOAD_X_BIT_SHIFT_IN_DW0                  0
#define PAYLOAD_Y_BIT_SHIFT_IN_DW0                  5
#define PAYLOAD_Z_BIT_SHIFT_IN_DW0                  10
#define TRAILING_ZERO_BIT_SHIFT_IN_DW0              15
#define GEO_ID_ANCHOR_SIZE_BIT_SHIFT_IN_DW0         20
#define GEO_ID_PAYLOAD_SIZE_BIT_SHIFT_IN_DW0        24
#define TRIANGLE_PAIR_COUNT_BIT_SHIFT_IN_DW0	    28
#define VERTEX_TYPE_BIT_SHIFT_IN_DW0                31

#define PRIM_ID_ANCOR_SIZE_BIT_SHIFT_IN_DW1         0
#define PRIM_ID_PAYLOAD_SIZE_BIT_SHIFT_IN_DW1       5
#define INDEX_SECTION_MIDPOINT_SHIFT_IN_DW1         10

//=====================================================================================================================
// 128-byte primitive structure
struct PrimitiveStructure
{
    // PrimitiveStructure static header data section layout:
    //    uint payloadXLength;        // 5 bit
    //    uint payloadYLength;        // 5 bit
    //    uint payloadZLength;        // 5 bit
    //    uint trailingZeroLength;    // 5 bit
    //    uint geoIndexAnchorSize;    // 4 bit
    //    uint geoIndexPayloadSize;   // 4 bit
    //    uint trianglePairCount;     // 3 bit
    //    uint vertexType;            // 1 bit
    //    uint primIndexAnchorSize;   // 5 bit
    //    uint primIndexPayloadSize;  // 5 bit
    //    uint indexSectionMidPoint;  // 10 bit

    uint primData[PRIMITIVE_STRUCT_SIZE_IN_DW];

    void Init()
    {
        uint i = 0;
        for (i = 0; i < PRIMITIVE_STRUCT_SIZE_IN_DW; i++)
        {
            primData[i] = 0;
        }
    }

    void Init(uint d[PRIMITIVE_STRUCT_SIZE_IN_DW])
    {
        uint i = 0;
        for (i = 0; i < PRIMITIVE_STRUCT_SIZE_IN_DW; i++)
        {
            primData[i] = d[i];
        }
    }

    uint PayloadXLength()
    {
        return 1 + bitFieldExtract(primData[0], PAYLOAD_X_BIT_SHIFT_IN_DW0, PAYLOAD_NUM_BITS);
    }

    uint PayloadYLength()
    {
        return 1 + bitFieldExtract(primData[0], PAYLOAD_Y_BIT_SHIFT_IN_DW0, PAYLOAD_NUM_BITS);
    }

    uint PayloadZLength()
    {
        return 1 + bitFieldExtract(primData[0], PAYLOAD_Z_BIT_SHIFT_IN_DW0, PAYLOAD_NUM_BITS);
    }

    uint TrailingZeroLength()
    {
        return bitFieldExtract(primData[0], TRAILING_ZERO_BIT_SHIFT_IN_DW0, TRIALING_ZERO_NUM_BITS);
    }

    uint GeoIdAnchorSize()
    {
        return 2 * bitFieldExtract(primData[0], GEO_ID_ANCHOR_SIZE_BIT_SHIFT_IN_DW0, GEO_ID_ANCHOR_NUM_BITS);
    }

    uint GeoIdPayloadSize()
    {
        return 2 * bitFieldExtract(primData[0], GEO_ID_PAYLOAD_SIZE_BIT_SHIFT_IN_DW0, GEO_ID_PAYLOAD_NUM_BITS);
    }

    uint TrianglePairCount()
    {
        return 1 + bitFieldExtract(primData[0], TRIANGLE_PAIR_COUNT_BIT_SHIFT_IN_DW0, TRIPAIR_COUNT_NUM_BITS);
    }

    uint VertexType()
    {
        return bitFieldExtract(primData[0], VERTEX_TYPE_BIT_SHIFT_IN_DW0, VERTEX_TYPE_NUM_BITS);
    }

    uint PrimIdAnchorSize()
    {
        return bitFieldExtract(primData[1], PRIM_ID_ANCOR_SIZE_BIT_SHIFT_IN_DW1, PRIM_ID_ANCHOR_NUM_BITS);
    }

    uint PrimIdPayloadSize()
    {
        return bitFieldExtract(primData[1], PRIM_ID_PAYLOAD_SIZE_BIT_SHIFT_IN_DW1, PRIM_ID_PAYLOAD_NUM_BITS);
    }

    uint IndexSectionMidpoint()
    {
        return bitFieldExtract(primData[1], INDEX_SECTION_MIDPOINT_SHIFT_IN_DW1, INDEX_SECTION_MID_POINT_NUM_BITS);
    }

    uint PrefixXLength()
    {
        return 32 - TrailingZeroLength() - PayloadXLength();
    }

    uint PrefixYLength()
    {
        return 32 - TrailingZeroLength() - PayloadYLength();
    }

    uint PrefixZLength()
    {
        return 32 - TrailingZeroLength() - PayloadZLength();
    }

    uint VertexPayloadLength()
    {
        return PayloadXLength() + PayloadYLength() + PayloadZLength();
    }

    uint VertexPrefixesLength()
    {
        return PrefixXLength() + PrefixYLength() + PrefixZLength();
    }

    uint TriangleCount()
    {
        uint triCount = 0;
        uint i = 0;
        for (i = 0; i < TrianglePairCount(); i++)
        {
            TrianglePairDesc desc = ReadTrianglePairDesc(i);
            triCount += 1 + desc.Tri1Valid();
        }
        return triCount;
    }

    // Write PackedBits (up to 32 bits) from metadata
    void WritePackedBits32(
        uint   startBitOffset, // offset of the bit from the beginning of the node
        uint   length,         // length of the bits to write
        uint   value)          // value to store in memory
    {
        uint startDwordIdx = startBitOffset / 32;

        uint data32      = primData[startDwordIdx];
        uint dwordOffset = startBitOffset & 31;
        data32 = bitFieldInsert(data32, dwordOffset, length, value);

        primData[startDwordIdx] = data32;
    }

    // Clear bits first, then update with the new data
    void WritePackedBits64(
        uint     startBitOffset, // offset of the bit from the beginning of the node
        uint     length,         // length of the bits to write
        uint64_t value)          // value to store in memory
    {
        if (length > 0)
        {
            uint startDwordIdx = startBitOffset / 32;
            if (startDwordIdx == 0x1f)
            {
                // only can read in one DW from primData, switch to 32bit mode
                uint value32 = uint(value & 0xFFFFFFFF);
                WritePackedBits32(startBitOffset, length, value32);
            }
            else
            {
                uint64_t dataLo = primData[startDwordIdx];
                uint64_t dataHi = primData[startDwordIdx + 1];
                uint64_t data64 = (dataHi << 32) | dataLo;

                uint dwordOffset   = startBitOffset & 31;
                data64 = bitFieldInsert64(data64, dwordOffset, length, value);

                // Update data packed in DW#
                uint data64Lo = uint(data64 & 0xFFFFFFFF);
                uint data64Hi = uint((data64 & 0xFFFFFFFF00000000) >> 32);
                primData[startDwordIdx]     = data64Lo;
                primData[startDwordIdx + 1] = data64Hi;
            }
        }
    }

    uint PackPrimitiveStructureHeaderLo(UnpackedPrimStructHeader header)
    {
        uint dataLo = PackMetadataHeaderBitsLo(header.payloadXLengthMinusOne,
                                               header.payloadYLengthMinusOne,
                                               header.payloadZLengthMinusOne,
                                               header.trailingZeroLength,
                                               header.geoIdAnchorSizeDivTwo,
                                               header.geoIdPayloadSizeDivTwo,
                                               header.triPairCountMinusOne,
                                               header.floatOrUnorm16);
        return dataLo;
    }

    // Pack the High 32bit of PrimitiveStructure Header
    uint PackPrimitiveStructureHeaderHi(UnpackedPrimStructHeader header)
    {
        uint dataHi = PackMetadataHeaderBitsHi(header.primIdAnchorSize,
                                               header.primIdPayloadSize,
                                               header.indexSectionMidpoint);
        return dataHi;
    }

    void WriteVertexPrefixes(float3 prefixes)
    {
        const uint trailingZeroLength = TrailingZeroLength();

        const uint xLength		 = PayloadXLength();
        const uint yLength		 = PayloadYLength();
        const uint zLength		 = PayloadZLength();

        const uint prefixXLength = PrefixXLength();
        const uint prefixYLength = PrefixYLength();
        const uint prefixZLength = PrefixZLength();

        const uint x = (asuint(prefixes.x) >> (32 - prefixXLength));
        const uint y = (asuint(prefixes.y) >> (32 - prefixYLength));
        const uint z = (asuint(prefixes.z) >> (32 - prefixZLength));

        WritePackedBits64(TRI_STRUCT_HEADER_SIZE_BITS, prefixXLength, uint64_t(x));
        WritePackedBits64(TRI_STRUCT_HEADER_SIZE_BITS + prefixXLength, prefixYLength, uint64_t(y));
        WritePackedBits64(TRI_STRUCT_HEADER_SIZE_BITS + prefixXLength + prefixYLength, prefixZLength, uint64_t(z));
    }

    void WritePrimIdAnchor(uint anchor)
    {
        const uint primIdAnchorSize = PrimIdAnchorSize();
        const uint64_t mask         = (1ull << primIdAnchorSize) - 1ull;
        const uint midPoint         = IndexSectionMidpoint();
        const uint64_t anchor64     = uint64_t(anchor);

        WritePackedBits64(midPoint, primIdAnchorSize, (anchor64 & mask));
    }

    void WriteGeoIdAnchor(uint anchor)
    {
        const uint geoIdAnchorSize = GeoIdAnchorSize();
        const uint64_t mask        = (1ull << geoIdAnchorSize) - 1ull;
        const uint midPoint        = IndexSectionMidpoint();
        const uint64_t anchor64    = uint64_t(anchor);

        WritePackedBits64((midPoint - geoIdAnchorSize), geoIdAnchorSize, (anchor64 & mask));
    }

    void WriteGeoIdPayload(uint triIndex, uint geoId)
    {
        if (triIndex > 0)
        {
            uint geoIdAnchorSize = GeoIdAnchorSize();
            uint geoIdPayloadSize = GeoIdPayloadSize();

            const uint startOffset = IndexSectionMidpoint() - geoIdAnchorSize - (triIndex)*geoIdPayloadSize;

            uint64_t geoId64 = uint64_t(geoId);
            if (geoIdPayloadSize < geoIdAnchorSize)
            {
                geoId64 = geoId64 & ((1ull << geoIdPayloadSize) - 1ull);
            }

            WritePackedBits64(startOffset, geoIdPayloadSize, geoId64);
        }
    }

    void WritePrimIdPayload(uint triIndex, uint primId)
    {
        if (triIndex > 0)
        {
            const uint primIdAnchorSize = PrimIdAnchorSize();
            const uint primIdPayloadSize = PrimIdPayloadSize();

            const uint startOffset = IndexSectionMidpoint() + primIdAnchorSize + (triIndex - 1) * primIdPayloadSize;

            uint64_t primId64 = uint64_t(primId);
            if (primIdPayloadSize < primIdAnchorSize)
            {
                primId64 = primId64 & ((1ull << primIdPayloadSize) - 1ull);
            }  // else no compression
            WritePackedBits64(startOffset, primIdPayloadSize, primId64);
        }
    }

    void WriteFloat(const uint startBitOffset, const uint payloadLength, float f)
    {
        uint component = asuint(f) >> TrailingZeroLength();
        uint64_t component64 = (uint64_t(component)) & ((1ull << payloadLength) - 1ull);
        WritePackedBits64(startBitOffset, payloadLength, component64);
    }

    void WriteVertex(uint verticeIndex, float3 vertex, bool isProcedural)
    {
        const uint sectionOffset = TRI_STRUCT_HEADER_SIZE_BITS + VertexPrefixesLength();
        const uint payloadXLength = PayloadXLength();
        const uint payloadYLength = PayloadYLength();
        const uint payloadZLength = PayloadZLength();

        if (isProcedural)
        {
            // store full vertex for Procedural nodes
            const uint32 dwBitLength = 0x20;
            uint32_t offset = TRI_STRUCT_HEADER_SIZE_BITS + (verticeIndex) * dwBitLength * 3;
            WriteFloat(offset, dwBitLength, vertex.x);
            offset += dwBitLength;
            WriteFloat(offset, dwBitLength, vertex.y);
            offset += dwBitLength;
            WriteFloat(offset, dwBitLength, vertex.z);
        }
        else
        {
            uint startBitOffset = sectionOffset + (verticeIndex)*VertexPayloadLength();
            WriteFloat(startBitOffset, payloadXLength, vertex.x);

            startBitOffset += payloadXLength;
            WriteFloat(startBitOffset, payloadYLength, vertex.y);

            startBitOffset += payloadYLength;
            WriteFloat(startBitOffset, payloadZLength, vertex.z);
        }
    }

    void WriteTriPairDesc(uint triPairIndex, TrianglePairDesc  desc)
    {
        const uint startOffset = 1024 - TRI_PAIR_DESC_SIZE * (triPairIndex + 1);
        WritePackedBits64(startOffset, TRI_PAIR_DESC_SIZE, uint64_t(desc.GetData()));
    }

    // Pack Metadata
    void SetMetadata(UnpackedPrimStructHeader header)
    {
        primData[0] = PackPrimitiveStructureHeaderLo(header);
        primData[1] = PackPrimitiveStructureHeaderHi(header);

        float3 perfixSource = float3(header.prefixSourceX, header.prefixSourceY, header.prefixSourceZ);
        WriteVertexPrefixes(perfixSource);
        WritePrimIdAnchor(header.anchorPrimId);
        WriteGeoIdAnchor(header.anchorGeoId);
    }

#if GPURT_BVH_BUILD_SHADER
    void Encode16(
        UnpackedPrimStructHeader header,
        float3                   verts[16],
        uint                     vertCount,
        TrianglePairDesc         triDescs[9],
        uint                     triPairDescCount,
        uint                     dstNodeOffset)
    {
        // clear out memory
        Init();

        // Setup header Metadata
        SetMetadata(header);

        // Pack Verts
        uint startBitOffset = 0;
        uint vertIdx = 0;
        for (vertIdx = 0; vertIdx < vertCount; vertIdx++)
        {
            WriteVertex(vertIdx, verts[vertIdx], Settings.geometryType == GEOMETRY_TYPE_AABBS);
        }

        // Write triangle descriptors
        uint triDescIdx = 0;
        for (triDescIdx = 0; triDescIdx < triPairDescCount; triDescIdx++)
        {
            TrianglePairDesc desc = triDescs[triDescIdx];
            WriteTriPairDesc(triDescIdx, desc);
            const uint triIndex = triDescIdx * 2;
            if (triIndex > 0)
            {
                WriteGeoIdPayload(triIndex, desc.tri0GeoId);
                WritePrimIdPayload(triIndex, desc.tri0PrimId);
            }
            if (desc.Tri1Valid())
            {
                WriteGeoIdPayload(triIndex + 1, desc.tri1GeoId);
                WritePrimIdPayload(triIndex + 1, desc.tri1PrimId);
            }
        }

        uint idx = 0;
        for (idx = 0; idx < PRIMITIVE_STRUCT_SIZE_IN_DW; idx++)
        {
            DstBuffer.Store(dstNodeOffset + 4 * idx, primData[idx]);
        }
    }

    void EncodeSinglePrimitive(
        UnpackedPrimStructHeader header,
        float3                   verts[3],
        uint                     vertCount,
        TrianglePairDesc         triDesc,
        uint                     dstNodeOffset,
        bool                     isProcedural)
    {
        // clear out memory
        Init();

        // Setup header Metadata
        SetMetadata(header);

        // Pack Verts
        uint vertIdx = 0;
        for (vertIdx = 0; vertIdx < vertCount; vertIdx++)
        {
            WriteVertex(vertIdx, verts[vertIdx], isProcedural);
        }

        // Write triangle descriptors
        WriteTriPairDesc(0, triDesc);

        uint idx = 0;
        for (idx = 0; idx < PRIMITIVE_STRUCT_SIZE_IN_DW; idx++)
        {
            DstBuffer.Store(dstNodeOffset + 4 * idx, primData[idx]);
        }
    }
#endif

    uint ReadPackedBits(uint startBitOffset, uint length)
    {
        uint startByteOffset = startBitOffset / 8;
        uint startDWOffset = startBitOffset / 32;

        // Reading in the data and the one Byte after it
        uint64_t currData = uint64_t(primData[startDWOffset]);
        uint64_t nextData = 0;
        if (startDWOffset < 0x1f)
        {
            nextData = uint64_t(primData[startDWOffset + 1]);
        }
        uint64_t packedData = (nextData << 32) | currData;

        uint shiftBit = (startBitOffset % 8);
        uint shiftBitsInDW = (startBitOffset % 32u);

        uint64_t packedDataRSH = packedData >> shiftBitsInDW;

        uint64_t dataOut64 = (packedDataRSH & ((uint64_t(1) << length) - uint64_t(1)));
        uint dataOut = uint(dataOut64 & 0xFFFFFFFF);

        return dataOut;
    }

    uint3 VertexPrefixes()
    {
        uint prefixXLength = PrefixXLength();
        uint prefixYLength = PrefixYLength();
        uint prefixZLength = PrefixZLength();

        uint prefixX = (prefixXLength > 0) ? ReadPackedBits(TRI_STRUCT_HEADER_SIZE_BITS, prefixXLength) : 0;
        uint prefixY = (prefixYLength > 0) ? ReadPackedBits(TRI_STRUCT_HEADER_SIZE_BITS + prefixXLength, prefixYLength) : 0;
        uint prefixZ = (prefixZLength > 0) ? ReadPackedBits(TRI_STRUCT_HEADER_SIZE_BITS + prefixXLength + prefixYLength, prefixZLength) : 0;

        uint3 prefixes = { prefixX, prefixY, prefixZ };

        prefixes.x <<= 32 - prefixXLength;
        prefixes.y <<= 32 - prefixYLength;
        prefixes.z <<= 32 - prefixZLength;

        return prefixes;
    }

    float3 ReadVertex(uint index, bool isProcedural)
    {
        uint x = 0;
        uint y = 0;
        uint z = 0;

        if (isProcedural)
        {
            const uint32 dwBitLength = 0x20;
            uint32_t offset = TRI_STRUCT_HEADER_SIZE_BITS + (index) * dwBitLength * 3;

            x = ReadPackedBits(offset, dwBitLength);
            offset += dwBitLength;
            y = ReadPackedBits(offset, dwBitLength);
            offset += dwBitLength;
            z = ReadPackedBits(offset, dwBitLength);
        }
        else
        {
            uint3 prefixes = VertexPrefixes();
            const uint sectionOffset = TRI_STRUCT_HEADER_SIZE_BITS + VertexPrefixesLength();
            uint offset = sectionOffset + (index)*VertexPayloadLength();

            uint payloadXLength = PayloadXLength();
            uint payloadYLength = PayloadYLength();
            uint payloadZLength = PayloadZLength();
            uint trailingZeroLength = TrailingZeroLength();

            x = prefixes.x | (ReadPackedBits(offset, payloadXLength) << trailingZeroLength);
            offset += payloadXLength;
            y = prefixes.y | (ReadPackedBits(offset, payloadYLength) << trailingZeroLength);
            offset += payloadYLength;
            z = prefixes.z | (ReadPackedBits(offset, payloadZLength) << trailingZeroLength);
        }
        float xf = asfloat(x);
        float yf = asfloat(y);
        float zf = asfloat(z);

        return float3(xf, yf, zf);
    }

    TrianglePairDesc ReadTrianglePairDesc(uint triPairIndex)
    {
        const uint startOffset = 1024 - TRI_PAIR_DESC_SIZE * (triPairIndex + 1);
        uint packedData = ReadPackedBits(startOffset, TRI_PAIR_DESC_SIZE);

        TrianglePairDesc triDesc;
        // There is dxc bug that the generated spv use OpAccessChain point as OpFunctionCall Parameter,
        // which is illeagel per spv spec.
        triDesc.triDesc = packedData;

        return triDesc;
    }

    uint PrimIdAnchor()
    {
        uint indexSectionMidpoint = IndexSectionMidpoint();
        uint primIdAnchorSize = PrimIdAnchorSize();
        return ReadPackedBits(indexSectionMidpoint, primIdAnchorSize);
    }

    uint ReadPrimIdPayload(uint triIndex)
    {
        uint anchor = PrimIdAnchor();
        uint indexSectionMidpoint = IndexSectionMidpoint();
        uint ret = 0;

        if (triIndex == 0)
        {
            ret = anchor;
        }
        else
        {
            uint primIdAnchorSize = PrimIdAnchorSize();
            uint primIdPayloadSize = PrimIdPayloadSize();
            const uint startOffset =
                indexSectionMidpoint + primIdAnchorSize + (triIndex - 1) * primIdPayloadSize;
            uint packedData = ReadPackedBits(startOffset, primIdPayloadSize);
            if (primIdPayloadSize >= primIdAnchorSize)
            {
                ret = packedData;
            }
            else
            {
                const uint removeMask = ~((1u << primIdPayloadSize) - 1u);
                ret = (anchor & removeMask) | packedData;
            }
        }

        return ret;
    }

    uint GeoIdAnchor()
    {
        uint indexSectionMidpoint = IndexSectionMidpoint();
        uint geoIdAnchorSize = GeoIdAnchorSize();

        return ReadPackedBits(indexSectionMidpoint - geoIdAnchorSize, geoIdAnchorSize);
    }

    uint ReadGeoIdPayload(uint triIndex)
    {
        uint anchor = GeoIdAnchor();
        uint indexSectionMidpoint = IndexSectionMidpoint();
        uint geoIdAnchorSize = GeoIdAnchorSize();
        uint geoIdPayloadSize = GeoIdPayloadSize();

        uint ret = 0;

        if (triIndex == 0)
        {
            ret = anchor;
        }
        else
        {
            const uint startOffset =
                indexSectionMidpoint - geoIdAnchorSize - (triIndex)*GeoIdPayloadSize();
            uint packedData = ReadPackedBits(startOffset, geoIdPayloadSize);
            if (geoIdPayloadSize >= geoIdAnchorSize)
            {
                ret = packedData;
            }
            else
            {
                const uint removeMask = ~((1u << geoIdPayloadSize) - 1u);
                ret = (anchor & removeMask) | packedData;
            }
        }

        return ret;
    }

    TriangleData ReadTriangleVertices(TrianglePairDesc pairDesc, uint triIndex)
    {
        TriangleData tri = (TriangleData)0;
        if (triIndex == 0)
        {
            tri.v0 = ReadVertex(pairDesc.Tri0V0(), false);
            tri.v1 = ReadVertex(pairDesc.Tri0V1(), false);
            tri.v2 = ReadVertex(pairDesc.Tri0V2(), false);
        }
        else
        {
            tri.v0 = ReadVertex(pairDesc.Tri1V0(), false);
            tri.v1 = ReadVertex(pairDesc.Tri1V1(), false);
            tri.v2 = ReadVertex(pairDesc.Tri1V2(), false);
        }

        return tri;
    }

    TriangleData ReadProceduralData()
    {
        TriangleData tri = (TriangleData)0;
        tri.v0 = ReadVertex(0, true);
        tri.v1 = ReadVertex(1, true);
        tri.v2 = ReadVertex(2, true);
        return tri;
    }

    TriangleData UnpackTriangleVertices(uint pair, uint triIndex)
    {
#if defined(GPURT_DEVELOPER) && defined(__cplusplus)
        // only 2 triangles per pair
        assert(triIndex <= 1);
#endif
        TrianglePairDesc pairDesc = ReadTrianglePairDesc(pair);

        bool isProceduralNode = (pairDesc.Tri0V0() == 15) && (pairDesc.Tri0V1() == 15);

        TriangleData tri;
        if (isProceduralNode)
        {
            tri = ReadProceduralData();
        }
        else
        {
            tri = ReadTriangleVertices(pairDesc, triIndex);
        }

        return tri;
    }

    bool IsOpaque(uint pair, uint triIndex)
    {
#if defined(GPURT_DEVELOPER) && defined(__cplusplus)
        // only 2 triangles per pair
        assert(triIndex <= 1);
#endif

        TrianglePairDesc pairDesc = ReadTrianglePairDesc(pair);
        return (triIndex == 0) ? pairDesc.Tri0Opaque() : pairDesc.Tri1Opaque();
    }

    bool IsProcedural(uint pair, uint triIndex)
    {
#if defined(GPURT_DEVELOPER) && defined(__cplusplus)
        // only 2 triangles per pair
        assert(triIndex <= 1);
#endif

        TrianglePairDesc pairDesc = ReadTrianglePairDesc(pair);

        const uint i0 = (triIndex == 0) ? pairDesc.Tri0V0() : pairDesc.Tri1V0();
        const uint i1 = (triIndex == 0) ? pairDesc.Tri0V1() : pairDesc.Tri1V1();

        return ((i0 == 0xF) && (i1 == 0xF));
    }

    uint UnpackGeometryIndex(uint pair, uint triIndex)
    {
#if defined(GPURT_DEVELOPER) && defined(__cplusplus)
        // only 2 triangles per pair
        assert(triIndex <= 1);
#endif
        return ReadGeoIdPayload(pair * 2 + triIndex);
    }

    uint UnpackPrimitiveIndex(uint pair, uint triIndex)
    {
#if defined(GPURT_DEVELOPER) && defined(__cplusplus)
        // only 2 triangles per pair
        assert(triIndex <= 1);
#endif
        return ReadPrimIdPayload(pair * 2 + triIndex);
    }

    uint CalcNavigationBits(uint pair)
    {
        TrianglePairDesc pairDesc = ReadTrianglePairDesc(pair);
        const uint trianglePairCount = TrianglePairCount();

        uint navigationBits = END_OF_RANGE;

        if (!pairDesc.PrimRangeStopBit())
        {
            // prim range continues
            if (pair == (trianglePairCount - 1))
            {
                // this is the last triangle pair in node, continue in next node
                navigationBits = END_OF_NODE;
            }
            else
            {
                // continue to next pair in node
                navigationBits = END_OF_PAIR;
            }
        }

        return navigationBits;
    }
};

//=====================================================================================================================
static bool TryInitTriStructHeader(
    in float3                             vertex0,
    in uint3                              planeDiffs,
    in uint                               planeUnion,
    in uint                               primIdDiff,
    in uint                               geoIdDiff,
    in uint                               primIdAnchor,
    in uint                               geoIdAnchor,
    in uint                               uniqueVertexCount,
    in uint                               triPairCount,
    in bool                               isProcedural,
    in bool                               enableTrailingZeroCompression,
    inout_param(UnpackedPrimStructHeader) header)
{
    const uint trailingZeroLength = (isProcedural || !enableTrailingZeroCompression) ?
                                    0 : CommonTrailingZeroBits(planeUnion);
    uint3 prefixLengths = (isProcedural || !enableTrailingZeroCompression) ?
                           uint3(0, 0, 0) : CommonPrefixBits(planeDiffs);
    prefixLengths = min(prefixLengths, uint3(31 - trailingZeroLength, 31 - trailingZeroLength, 31 - trailingZeroLength));
    const uint vertexSize =	96 - (prefixLengths.x + prefixLengths.y + prefixLengths.z + 3 * trailingZeroLength);
    const uint primPayloadLength = 32 - LeadingZeroBits(primIdDiff);
    const uint primAnchorLength  = 32 - LeadingZeroBits(primIdAnchor);

    // Note, geometry ID payload and anchor lengths are encoded as multiple of 2. We need to align the lengths here
    // to account for that
    const uint geoPayloadLength = Pow2Align(32 - LeadingZeroBits(geoIdDiff), 2);
    const uint geoAnchorLength = Pow2Align(32 - LeadingZeroBits(geoIdAnchor), 2);

    const uint pairDescDataSize = triPairCount * TRI_PAIR_DESC_SIZE;
    const uint idPayloadDataSize = ((2 * triPairCount) - 1) * (primPayloadLength + geoPayloadLength);
    const uint vertexDataSize = uniqueVertexCount * vertexSize;
    const uint anchorSizes = prefixLengths.x + prefixLengths.y + prefixLengths.z + primAnchorLength + geoAnchorLength;
    const uint requisiteBits = pairDescDataSize + idPayloadDataSize + vertexDataSize + anchorSizes;
    if (requisiteBits > TRI_STRUCT_DATA_SECTION_SIZE_BITS || uniqueVertexCount > 16 || triPairCount > 8)
    {
        // One pair should always fit
        GPU_ASSERT(triPairCount > 1);
        return false;
    }

    GPU_ASSERT(prefixLengths.x + trailingZeroLength <= 31);
    GPU_ASSERT(prefixLengths.y + trailingZeroLength <= 31);
    GPU_ASSERT(prefixLengths.z + trailingZeroLength <= 31);

    header.anchorGeoId          = geoIdAnchor;
    header.anchorPrimId         = primIdAnchor;
    header.floatOrUnorm16       = false;
    header.prefixSourceX        = vertex0.x;
    header.prefixSourceY        = vertex0.y;
    header.prefixSourceZ        = vertex0.z;
    header.triPairCountMinusOne = (triPairCount - 1);

    if (isProcedural)
    {
        // store full vertex instead of compressed format
        //
        // each vertex is stored as 1 DW (32bit)
        // min.x
        // min.y
        // min.z
        // max.x
        // max.y
        // max.z
        //
        // geometryPayload---  0 bit
        // geometryIdAncor--- 15 bit
        //------------------- index_section_midpoint
        // primIdAnchor ----- 32 bit
        // primIdPayload-----  0 bit
        // triDesc----------- 29 bit
        //------------------- 1024bit

        header.payloadXLengthMinusOne = 0;      // decode as 1
        header.payloadYLengthMinusOne = 0;      // decode as 1
        header.payloadZLengthMinusOne = 0;      // decode as 1
        header.trailingZeroLength     = 0;

        header.geoIdPayloadSizeDivTwo = 0x0;
        header.geoIdAnchorSizeDivTwo  = 0xF;    // decode as 30
        header.indexSectionMidpoint   = ComputeIndexSectionMidpoint(1, 0, 32);
        header.primIdAnchorSize       = 0x1F;
        header.primIdPayloadSize      = 0;
    }
    else
    {
        header.payloadXLengthMinusOne = (32 - prefixLengths.x - trailingZeroLength) - 1;
        header.payloadYLengthMinusOne = (32 - prefixLengths.y - trailingZeroLength) - 1;
        header.payloadZLengthMinusOne = (32 - prefixLengths.z - trailingZeroLength) - 1;
        header.trailingZeroLength     = trailingZeroLength;
        header.geoIdAnchorSizeDivTwo  = geoAnchorLength / 2;
        header.geoIdPayloadSizeDivTwo = geoPayloadLength / 2;
        header.indexSectionMidpoint   = ComputeIndexSectionMidpoint(triPairCount, primPayloadLength, primAnchorLength);
        header.primIdAnchorSize       = primAnchorLength;
        header.primIdPayloadSize      = primPayloadLength;
    }

    return true;
}

//=====================================================================================================================
static uint GetPairIndex(
    in uint nodePointer)
{
    return (nodePointer & 0x3) | ((nodePointer & 8) >> 1);
}

//=====================================================================================================================
static uint GetPairIndexFromNodeType(
    in uint nodeType)
{
    return GetPairIndex(nodeType);
}

#endif
