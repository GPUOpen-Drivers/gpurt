/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2021-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _HPB64_HLSL
#define _HPB64_HLSL

#include "../ShaderDefs.hlsli"
#include "../gfx10/BoxNode1_0.hlsli"

#define HPB64_BOX_NODE_FLAGS_BIT_OFFSET 41
#define HPB64_BOX_NODE_FLAGS_BIT_COUNT  15
#define INVALID_TYPE_OF_HP              NODE_TYPE_BOX_FLOAT32x2

//=====================================================================================================================
struct HighPrecisionBoxNode
{
#ifdef __cplusplus
    HighPrecisionBoxNode(uint val)
    {
        memset(this, val, sizeof(HighPrecisionBoxNode));
    }

    HighPrecisionBoxNode() : HighPrecisionBoxNode(0)
    {}
#endif
    uint32_t dwords[16];
};

//=====================================================================================================================
static uint GetHighPrecisionBoxChildFlagsBitMask(in uint childIdx);

//=====================================================================================================================
static uint DecodeExponent(float fp32);

//=====================================================================================================================
// Encode fp32 into 18-bit block
static uint EncodeBlock(
    in float fp32, in uint commonExp);

//=====================================================================================================================
// Unpack high precision box node flags into fp32 box node flags
static uint UnpackHighPrecisionBoxNodeFlags(
    in uint packedFlags);

//=====================================================================================================================
static HighPrecisionBoxNode EncodeHighPrecisionBoxNode(
    in Float32BoxNode node);

//=====================================================================================================================
static float DecodeBlock(
    in uint encodedBlock, in uint commonExp, in bool roundToPositive);

//=====================================================================================================================
static Float32BoxNode DecodeHighPrecisionBoxNode(
    in HighPrecisionBoxNode node);

//=====================================================================================================================
static uint LoadBits(
    RWByteAddressBuffer DataBuffer, uint nodeOffset, uint bitOffset, uint size);

//=====================================================================================================================
// globallycoherent version
static uint LoadBits_gc(
    globallycoherent RWByteAddressBuffer DataBuffer, uint nodeOffset, uint bitOffset, uint size);

//=====================================================================================================================
static void StoreBits(
    RWByteAddressBuffer DataBuffer, uint nodeOffset, uint bitOffset, uint size, uint data);

// ====================================================================================================================
static int GetChildPlaneMinBitOffset(
    in int idx);

// ====================================================================================================================
static int GetChildPlaneMaxBitOffset(
    in int idx);

#if GPURT_BVH_BUILD_SHADER

//=====================================================================================================================
// Count valid children in high precision box node
static uint HighPrecisionBoxNodeCountValidChildren(
    RWByteAddressBuffer DataBuffer, uint nodeOffset);
#endif

#if TEST
//=====================================================================================================================
// Local root signature
#define RootSig "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL)"

//=====================================================================================================================
RWByteAddressBuffer DstBuffer : register(u0);
RWByteAddressBuffer SrcBuffer : register(u1);

//=====================================================================================================================
// Main Function : Encode
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(32, 1, 1)]
void Encode(
    in uint3 globalThreadId : SV_DispatchThreadID);

#endif

#ifndef LIBRARY_COMPILATION
#include "HighPrecisionBoxNode.hlsl"
#endif

#endif
