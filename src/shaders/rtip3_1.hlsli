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
#ifndef RTIP_3_1_HLSLI
#define RTIP_3_1_HLSLI

#include "../shadersClean/common/Common.hlsli"
#include "../shadersClean/common/gfx12/primitiveNode.hlsli"

#if GPURT_BVH_BUILD_SHADER
//=====================================================================================================================
// Update a Primitive Structure containing a single triangle pair
void UpdatePairPrimStruct(
    TriangleData tri0,
    TriangleData tri1,
    bool         isTri1Valid,
    uint4        geomIdPrimIdAndPairDescDwords,
    uint         geomId1Dword,
    uint         nodeOffset,
    bool         isProcedural)
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

    const uint indexSectionMidpoint = ComputeIndexSectionMidpoint(1, 31, 31);

    uint d[PRIMITIVE_STRUCT_SIZE_IN_DW];

    if (Settings.isUpdateInPlace == false)
    {
        // Metadata header
        if (isProcedural)
        {
            d[0] = PackMetadataHeaderBitsLo(0, 0, 0, 0, 15, 15, 0, false);
        }
        else
        {
            d[0] = PackMetadataHeaderBitsLo(31, 31, 31, 0, 15, 15, 0, false);
        }
        DstMetadata.Store(nodeOffset, d[0]);
    }

    // Rest of Metadata header
    d[1] = PackMetadataHeaderBitsHi(31, 31, indexSectionMidpoint);

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

        [unroll]
        for (uint i = 1; i < 17; i += 4)
        {
            DstMetadata.Store4(nodeOffset + (i * 4), uint4(d[i], d[i + 1], d[i + 2], d[i + 3]));
        }
        DstMetadata.Store3(nodeOffset + (17 * 4), uint3(d[17], d[18], d[19]));
    }
    else
    {
        DstMetadata.Store4(nodeOffset + (1 * 4), uint4(d[1], d[2], d[3], d[4]));
        DstMetadata.Store4(nodeOffset + (5 * 4), uint4(d[5], d[6], d[7], d[8]));
        DstMetadata.Store2(nodeOffset + (9 * 4), uint2(d[9], d[10]));
    }

    if (Settings.isUpdateInPlace == false)
    {
        d[20] = 0;
        d[21] = 0;
        d[22] = 0;
        d[23] = 0;
        d[24] = 0;
        d[25] = 0;
        d[26] = 0;

        // Do not have to fetch geom ids, prim ids and triangle pair descriptor again, we can reuse
        d[27] = geomId1Dword;
        d[28] = geomIdPrimIdAndPairDescDwords[0];
        d[29] = geomIdPrimIdAndPairDescDwords[1];
        d[30] = geomIdPrimIdAndPairDescDwords[2];
        d[31] = geomIdPrimIdAndPairDescDwords[3];

        [unroll]
        for (uint i = 20; i < PRIMITIVE_STRUCT_SIZE_IN_DW; i += 4)
        {
            DstMetadata.Store4(nodeOffset + (i*4), uint4(d[i], d[i+1], d[i+2], d[i+3]));
        }
    }
}
#endif

//=====================================================================================================================
static void FetchPrimitiveStructFromNodeAddr(
    in GpuVirtualAddress            nodeAddr,
    inout_param(PrimitiveStructure) primStruct)
{
    // cleanup memory before assign data
    primStruct.Init();

    // Primitive Struecture has 32 DW
    uint i = 0;
    for (i = 0; i < PRIMITIVE_STRUCT_SIZE_IN_DW; i++)
    {
        primStruct.primData[i] = LoadDwordAtAddr(nodeAddr + i * sizeof(uint));
    }
}

//=====================================================================================================================
static void FetchPrimitiveStructFromBvh(
    in uint64_t                     bvhAddress,
    in uint                         nodePointer,
    inout_param(PrimitiveStructure) primStruct)
{
    const uint64_t nodeAddress = bvhAddress + ((nodePointer & ~0xF) << 3);
    FetchPrimitiveStructFromNodeAddr(nodeAddress, primStruct);
}

#endif
