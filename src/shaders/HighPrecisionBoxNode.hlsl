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

#include "../shadersClean/common/Common.hlsli"

#define HPB64_BOX_NODE_FLAGS_BIT_OFFSET 41
#define HPB64_BOX_NODE_FLAGS_BIT_COUNT  15
#define INVALID_TYPE_OF_HP              NODE_TYPE_BOX_FLOAT32x2
//=====================================================================================================================
static uint GetHighPrecisionBoxChildFlagsBitMask(in uint childIdx)
{
    const uint bitMask = (childIdx == 3) ? 0x7 : 0xF;
    return bitMask;
}

//=====================================================================================================================
static uint DecodeExponent(float fp32)
{
    const uint fp32Bits = asuint(fp32);
    return (fp32Bits >> 23) & 0xfful;
}

//=====================================================================================================================
// Encode fp32 into 18-bit block
static uint EncodeBlock(
    in float fp32, in uint commonExp)
{
    // Each sign_mantissa block is formatted as 1-bit sign and 17-bits of mantissa
    // [00:16] Mantissa
    // [17:17] Sign
    //
    const uint fp32Bits = asuint(fp32);

    // Extract sign, exponent, and mantissa bits from fp32
    const uint exp = (fp32Bits >> 23) & 0xfful;
    const uint sign = (fp32Bits >> 31) & 1;

    // Extract mantissa bits from fp32
    uint mantissa = fp32Bits & bits(23);

    // To freely change a floating point exponent value, it needs to be de-normalized. The implicit 1 bit is shifted
    // down to mantissa. However, if the floating point value was already de-normalized, this step is skipped
    if (exp > 0)
    {
        mantissa = (1 << 22) | (mantissa >> 1);
    }

    // Now, exponent is right shifted by the exponent difference. This can be interpreted as a pow-2 division in the
    // mantissa to compensate for the pow-2 multiplication due to the exponent change. The right shift must be zero
    // extended
    const uint shift = commonExp - exp;
    if (shift < 23)
    {
        mantissa >>= shift; // Must be zero-extended
    }
    else
    {
        mantissa = 0;
    }

    // Finally, truncate the 23-bit mantissa into 17-bit and then concatenate the sign bit
    const uint encodedBlock = (sign << 17) | (mantissa >> 6);

#if DEBUG_ENCODE_DECODE
    std::printf("\nfloat: 0x%08x -> exp: 0x%08x, sign: 0x%08x, mantissa: 0x%08x", fp32Bits, exp, sign, mantissa);
#endif

    return encodedBlock;
};

//=====================================================================================================================
// Unpack high precision box node flags into fp32 box node flags
static uint UnpackHighPrecisionBoxNodeFlags(
    in uint packedFlags)
{
    uint boxNodeFlags = packedFlags & 0xF;
    boxNodeFlags |= ((packedFlags >> 0x4) & 0xF) << 0x8;
    boxNodeFlags |= ((packedFlags >> 0x8) & 0xF) << 0x10;
    boxNodeFlags |= ((packedFlags >> 0xc) & 0x7) << 0x18;
    return boxNodeFlags;
}

//=====================================================================================================================
static HighPrecisionBoxNode EncodeHighPrecisionBoxNode(
    in Float32BoxNode node)
{
    // Repack Float32BoxNode flags into compressed node flags
    uint packedFlags;
    packedFlags = (((node.flags >> 0x00) & 0xF) << 0x0);
    packedFlags |= (((node.flags >> 0x08) & 0xF) << 0x4);
    packedFlags |= (((node.flags >> 0x10) & 0xF) << 0x8);
    packedFlags |= (((node.flags >> 0x18) & 0x7) << 0xc);

    HighPrecisionBoxNode result = (HighPrecisionBoxNode)0;

    bool isChild0Valid = IsValidNode(node.child0);
    bool isChild1Valid = IsValidNode(node.child1);
    bool isChild2Valid = IsValidNode(node.child2);
    bool isChild3Valid = IsValidNode(node.child3);

    // Note, High precision box nodes nodes are indicated by NODE_TYPE_BOX_HP64 type. Extract child node type and
    // fix up node type as necessary.
    uint4 childPtr;
    childPtr.x = isChild0Valid ? (node.child0 & bits(3)) : INVALID_TYPE_OF_HP;
    childPtr.y = isChild1Valid ? (node.child1 & bits(3)) : INVALID_TYPE_OF_HP;
    childPtr.z = isChild2Valid ? (node.child2 & bits(3)) : INVALID_TYPE_OF_HP;
    childPtr.w = isChild3Valid ? (node.child3 & bits(3)) : INVALID_TYPE_OF_HP;

    // Encode 29-bits of child base pointer. Note, child node pointers are expected to be contiguous in memory
    //
    // [00:28] childBasePtr, [29:31] childType0
    result.dword[0] = (node.child0 >> 3) | (childPtr.x << 29);

    // Exclude children with inverted bounding boxes (degenerate triangles) while calculating max exponent
    node.child0 = (node.bbox0_min.x > node.bbox0_max.x) ? INVALID_NODE : node.child0;
    node.child1 = (node.bbox1_min.x > node.bbox1_max.x) ? INVALID_NODE : node.child1;
    node.child2 = (node.bbox2_min.x > node.bbox2_max.x) ? INVALID_NODE : node.child2;
    node.child3 = (node.bbox3_min.x > node.bbox3_max.x) ? INVALID_NODE : node.child3;

    // Search for common exponents along each axis (direction)
    uint3 commonExp = uint3(0, 0, 0);

    if (node.child0 != INVALID_NODE)
    {
        commonExp.x = max(commonExp.x, DecodeExponent(node.bbox0_min.x));
        commonExp.y = max(commonExp.y, DecodeExponent(node.bbox0_min.y));
        commonExp.z = max(commonExp.z, DecodeExponent(node.bbox0_min.z));
        commonExp.x = max(commonExp.x, DecodeExponent(node.bbox0_max.x));
        commonExp.y = max(commonExp.y, DecodeExponent(node.bbox0_max.y));
        commonExp.z = max(commonExp.z, DecodeExponent(node.bbox0_max.z));
    }
    if (node.child1 != INVALID_NODE)
    {
        commonExp.x = max(commonExp.x, DecodeExponent(node.bbox1_min.x));
        commonExp.y = max(commonExp.y, DecodeExponent(node.bbox1_min.y));
        commonExp.z = max(commonExp.z, DecodeExponent(node.bbox1_min.z));
        commonExp.x = max(commonExp.x, DecodeExponent(node.bbox1_max.x));
        commonExp.y = max(commonExp.y, DecodeExponent(node.bbox1_max.y));
        commonExp.z = max(commonExp.z, DecodeExponent(node.bbox1_max.z));
    }
    if (node.child2 != INVALID_NODE)
    {
        commonExp.x = max(commonExp.x, DecodeExponent(node.bbox2_min.x));
        commonExp.y = max(commonExp.y, DecodeExponent(node.bbox2_min.y));
        commonExp.z = max(commonExp.z, DecodeExponent(node.bbox2_min.z));
        commonExp.x = max(commonExp.x, DecodeExponent(node.bbox2_max.x));
        commonExp.y = max(commonExp.y, DecodeExponent(node.bbox2_max.y));
        commonExp.z = max(commonExp.z, DecodeExponent(node.bbox2_max.z));
    }
    if (node.child3 != INVALID_NODE)
    {
        commonExp.x = max(commonExp.x, DecodeExponent(node.bbox3_min.x));
        commonExp.y = max(commonExp.y, DecodeExponent(node.bbox3_min.y));
        commonExp.z = max(commonExp.z, DecodeExponent(node.bbox3_min.z));
        commonExp.x = max(commonExp.x, DecodeExponent(node.bbox3_max.x));
        commonExp.y = max(commonExp.y, DecodeExponent(node.bbox3_max.y));
        commonExp.z = max(commonExp.z, DecodeExponent(node.bbox3_max.z));
    }

    // Encode bounds into 18-bit blocks containing sign and mantissa
    uint encodedBlocks[24];

    // Initialise blocks
    for (int boxIdx = 0; boxIdx < 4; ++boxIdx)
    {
        // Full mantissa, positive sign
        const uint boxOffset = boxIdx * 6;
        encodedBlocks[boxOffset + 0] = bits(17);
        encodedBlocks[boxOffset + 1] = bits(17);
        encodedBlocks[boxOffset + 2] = bits(17);

        // Full mantissa, negative sign
        encodedBlocks[boxOffset + 3] = bits(18);
        encodedBlocks[boxOffset + 4] = bits(18);
        encodedBlocks[boxOffset + 5] = bits(18);
    }

    if (node.child0 != INVALID_NODE)
    {
        encodedBlocks[0] = EncodeBlock(node.bbox0_min.x, commonExp.x);
        encodedBlocks[1] = EncodeBlock(node.bbox0_min.y, commonExp.y);
        encodedBlocks[2] = EncodeBlock(node.bbox0_min.z, commonExp.z);
        encodedBlocks[3] = EncodeBlock(node.bbox0_max.x, commonExp.x);
        encodedBlocks[4] = EncodeBlock(node.bbox0_max.y, commonExp.y);
        encodedBlocks[5] = EncodeBlock(node.bbox0_max.z, commonExp.z);
    }
    if (node.child1 != INVALID_NODE)
    {
        encodedBlocks[6] = EncodeBlock(node.bbox1_min.x, commonExp.x);
        encodedBlocks[7] = EncodeBlock(node.bbox1_min.y, commonExp.y);
        encodedBlocks[8] = EncodeBlock(node.bbox1_min.z, commonExp.z);
        encodedBlocks[9] = EncodeBlock(node.bbox1_max.x, commonExp.x);
        encodedBlocks[10] = EncodeBlock(node.bbox1_max.y, commonExp.y);
        encodedBlocks[11] = EncodeBlock(node.bbox1_max.z, commonExp.z);
    }
    if (node.child2 != INVALID_NODE)
    {
        encodedBlocks[12] = EncodeBlock(node.bbox2_min.x, commonExp.x);
        encodedBlocks[13] = EncodeBlock(node.bbox2_min.y, commonExp.y);
        encodedBlocks[14] = EncodeBlock(node.bbox2_min.z, commonExp.z);
        encodedBlocks[15] = EncodeBlock(node.bbox2_max.x, commonExp.x);
        encodedBlocks[16] = EncodeBlock(node.bbox2_max.y, commonExp.y);
        encodedBlocks[17] = EncodeBlock(node.bbox2_max.z, commonExp.z);
    }
    if (node.child3 != INVALID_NODE)
    {
        encodedBlocks[18] = EncodeBlock(node.bbox3_min.x, commonExp.x);
        encodedBlocks[19] = EncodeBlock(node.bbox3_min.y, commonExp.y);
        encodedBlocks[20] = EncodeBlock(node.bbox3_min.z, commonExp.z);
        encodedBlocks[21] = EncodeBlock(node.bbox3_max.x, commonExp.x);
        encodedBlocks[22] = EncodeBlock(node.bbox3_max.y, commonExp.y);
        encodedBlocks[23] = EncodeBlock(node.bbox3_max.z, commonExp.z);
    }

    // [32:34] childType1, [35:37] childType2, [38:40] childType3
    // [41:55] boxFlags
    // [56:63] commonExp.x
    result.dword[1] = (childPtr.y) | (childPtr.z << 3) | (childPtr.w << 6) | (packedFlags << 9) | (commonExp.x << 24);

    // [64:71] commonExp.y
    // [72:79] commonExp.z
    // b0[00:15]
    result.dword[2] = (commonExp.y) | (commonExp.z << 8) | ((encodedBlocks[0] & bits(16)) << 16);
    // b0[16:17],  b1[00:17], b2[00:11]
    result.dword[3] = (encodedBlocks[0] >> 16)  | (encodedBlocks[1] << 2)   | (encodedBlocks[2] << 20);
    // b2[12:17],  b3[00:17], b4[00:07]
    result.dword[4] = (encodedBlocks[2] >> 12)  | (encodedBlocks[3] << 6)   | (encodedBlocks[4] << 24);
    // b4[08:17],  b5[00:17], b6[00:03]
    result.dword[5] = (encodedBlocks[4] >> 8)   | (encodedBlocks[5] << 10)  | (encodedBlocks[6] << 28);
    // b6[04:17],  b7[00:17]
    result.dword[6] = (encodedBlocks[6] >> 4)   | (encodedBlocks[7] << 14);
    // b8[04:17],  b9[00:13]
    result.dword[7] = (encodedBlocks[8] >> 0)   | (encodedBlocks[9] << 18);
    // b9[14:17],  b10[00:17], b11[00,09]
    result.dword[8] = (encodedBlocks[9] >> 14)  | (encodedBlocks[10] << 4)  | (encodedBlocks[11] << 22);
    // b11[10:17], b12[00:17], b13[00:05]
    result.dword[9] = (encodedBlocks[11] >> 10) | (encodedBlocks[12] << 8)  | (encodedBlocks[13] << 26);
    // b13[06:17], b14[00:17], b15[00:01]
    result.dword[10] = (encodedBlocks[13] >> 6)  | (encodedBlocks[14] << 12) | (encodedBlocks[15] << 30);
    // b15[02:17], b16[00:15]
    result.dword[11] = (encodedBlocks[15] >> 2)  | (encodedBlocks[16] << 16);
    // b16[16:17], b17[00:17], b18[00:11]
    result.dword[12] = (encodedBlocks[16] >> 16) | (encodedBlocks[17] << 2)  | (encodedBlocks[18] << 20);
    // b18[12:17], b19[00:17], b20[00:07]
    result.dword[13] = (encodedBlocks[18] >> 12) | (encodedBlocks[19] << 6)  | (encodedBlocks[20] << 24);
    // b20[08:17], b21[00:17], b22[00:03]
    result.dword[14] = (encodedBlocks[20] >> 8)  | (encodedBlocks[21] << 10) | (encodedBlocks[22] << 28);
    // b22[04:17], b23[00:17]
    result.dword[15] = (encodedBlocks[22] >> 4)  | (encodedBlocks[23] << 14);

    return result;
}

//=====================================================================================================================
static float DecodeBlock(
    in uint encodedBlock, in uint commonExp, in bool roundToPositive)
{
    // Signs, mantissas, and exponents are recombined into IEEE floats.The crucial step is to decode the original
    // exponent value.This information is encoded in the position of implicit leading 1 in mantissa bits.The number
    // of leading zero bits is the difference between source exponent and the common exponent.
    //

    // First, extract sign bit and the mantissa bits :
    const uint sign = (encodedBlock >> 17) & 1ul;
    const uint mantissa = (encodedBlock & bits(17));

    // Scan right to left for implicit one bit's position used to find source exponent.
    int firstBitHigh = firstbithigh(mantissa);

    uint srcExp = 0;
    uint srcMantissa = 0;

    // Determine whether we need to round up to infinity or not
    const bool roundToInf = (roundToPositive && (sign == 0)) || (!roundToPositive && (sign == 1));

    // Since the number of leading zero bits is equal to the difference between source exponent and common exponent,
    // we can use it recover source exponent
    if (firstBitHigh != -1)
    {
        uint expDiff = (16 - firstBitHigh);

        // Clamp exponent difference to common exponent for denormal cases
        expDiff = min(expDiff, commonExp);

        srcExp = commonExp - expDiff;

        // Conservative box grow
        uint trail = roundToInf ? bits(7u + expDiff) : 0u;

        // Handle infinity
        if (srcExp == 0xff)
        {
            trail = 0;
        }

        // 17-bit to 23-bit & remove leading zeros
        srcMantissa = (mantissa << (7u + expDiff)) | trail;
        srcMantissa &= bits(23);
    }
    else
    {
        if (roundToInf)
        {
            srcExp = (commonExp > 17) ? (commonExp - 17u) : 0u;
            srcMantissa = bits(23);
        }
        else
        {
            srcExp = 0;
            srcMantissa = 0;
        }
    }

    const uint decodedBits = srcMantissa | (srcExp << 23) | (sign << 31);

#if DEBUG_ENCODE_DECODE
    std::printf("\nfloat: 0x%08x -> exp: 0x%08x, sign: 0x%08x, mantissa: 0x%08x", decodedBits, srcExp, sign, srcMantissa);
#endif

    return asfloat(decodedBits);
}

//=====================================================================================================================
static Float32BoxNode DecodeHighPrecisionBoxNode(
    in HighPrecisionBoxNode node)
{
    uint childBase  = (node.dword[0] & bits(29));
    uint childType0 = (node.dword[0] >> 29);
    uint childType1 = (node.dword[1] >> 0) & bits(3);
    uint childType2 = (node.dword[1] >> 3) & bits(3);
    uint childType3 = (node.dword[1] >> 6) & bits(3);

    uint flags = (node.dword[1] >> 9) & bits(15);

    uint3 commonExp;
    commonExp.x = (node.dword[1] >> 24);
    commonExp.y = (node.dword[2] >> 0) & bits(8);
    commonExp.z = (node.dword[2] >> 8) & bits(8);

    Float32BoxNode box32 = (Float32BoxNode)0;
    // Decode child node pointers. Note, only instance node is 128 bytes. All other nodes are 64-bytes
    uint offset = 0;
    box32.child0 = (childType0 == INVALID_TYPE_OF_HP) ? INVALID_NODE : ((childBase << 3) | childType0);
    offset += ((childType0 == NODE_TYPE_USER_NODE_INSTANCE) || (childType0 == NODE_TYPE_BOX_HP64x2)) ? 2 : 1;
    box32.child1 = (childType1 == INVALID_TYPE_OF_HP) ? INVALID_NODE : (((childBase + offset) << 3) | childType1);
    offset += ((childType1 == NODE_TYPE_USER_NODE_INSTANCE) || (childType1 == NODE_TYPE_BOX_HP64x2)) ? 2 : 1;
    box32.child2 = (childType2 == INVALID_TYPE_OF_HP) ? INVALID_NODE : (((childBase + offset) << 3) | childType2);
    offset += ((childType2 == NODE_TYPE_USER_NODE_INSTANCE) || (childType2 == NODE_TYPE_BOX_HP64x2)) ? 2 : 1;
    box32.child3 = (childType3 == INVALID_TYPE_OF_HP) ? INVALID_NODE : (((childBase + offset) << 3) | childType3);

    // Decode node flags
    box32.flags = 0;
    box32.flags |= (flags & bits(4));
    box32.flags |= ((flags >> 4) & bits(4)) << 8;
    box32.flags |= ((flags >> 8) & bits(4)) << 16;
    box32.flags |= ((flags >> 12) & bits(3)) << 24;

    uint bbox0_min_x_sign_mantissa = (node.dword[2] >> 16) | ((node.dword[3] & 0x3) << 16);
    uint bbox0_min_y_sign_mantissa = (node.dword[3] >> 2) & 0x3FFFF;
    uint bbox0_min_z_sign_mantissa = (node.dword[3] >> 20) | ((node.dword[4] & 0x3F) << 12);
    box32.bbox0_min.x = DecodeBlock(bbox0_min_x_sign_mantissa, commonExp.x, false);
    box32.bbox0_min.y = DecodeBlock(bbox0_min_y_sign_mantissa, commonExp.y, false);
    box32.bbox0_min.z = DecodeBlock(bbox0_min_z_sign_mantissa, commonExp.z, false);

    uint bbox0_max_x_sign_mantissa = (node.dword[4] >> 6) & 0x3FFFF;
    uint bbox0_max_y_sign_mantissa = (node.dword[4] >> 24) | ((node.dword[5] & 0x3FF) << 8);
    uint bbox0_max_z_sign_mantissa = (node.dword[5] >> 10) & 0x3FFFF;
    box32.bbox0_max.x = DecodeBlock(bbox0_max_x_sign_mantissa, commonExp.x, true);
    box32.bbox0_max.y = DecodeBlock(bbox0_max_y_sign_mantissa, commonExp.y, true);
    box32.bbox0_max.z = DecodeBlock(bbox0_max_z_sign_mantissa, commonExp.z, true);

    uint bbox1_min_x_sign_mantissa = (node.dword[5] >> 28) | ((node.dword[6] & 0x3FFF) << 4);
    uint bbox1_min_y_sign_mantissa = (node.dword[6] >> 14);
    uint bbox1_min_z_sign_mantissa = (node.dword[7] & 0x3FFFF);
    box32.bbox1_min.x = DecodeBlock(bbox1_min_x_sign_mantissa, commonExp.x, false);
    box32.bbox1_min.y = DecodeBlock(bbox1_min_y_sign_mantissa, commonExp.y, false);
    box32.bbox1_min.z = DecodeBlock(bbox1_min_z_sign_mantissa, commonExp.z, false);

    uint bbox1_max_x_sign_mantissa = (node.dword[7] >> 18) | ((node.dword[8] & 0xF) << 14);
    uint bbox1_max_y_sign_mantissa = (node.dword[8] >> 4) & 0x3FFFF;
    uint bbox1_max_z_sign_mantissa = (node.dword[8] >> 22) | ((node.dword[9] & 0xFF) << 10);
    box32.bbox1_max.x = DecodeBlock(bbox1_max_x_sign_mantissa, commonExp.x, true);
    box32.bbox1_max.y = DecodeBlock(bbox1_max_y_sign_mantissa, commonExp.y, true);
    box32.bbox1_max.z = DecodeBlock(bbox1_max_z_sign_mantissa, commonExp.z, true);

    uint bbox2_min_x_sign_mantissa = (node.dword[9] >> 8) & 0x3FFFF;
    uint bbox2_min_y_sign_mantissa = (node.dword[9] >> 26) | ((node.dword[10] & 0xFFF) << 6);
    uint bbox2_min_z_sign_mantissa = (node.dword[10] >> 12) & 0x3FFFF;
    box32.bbox2_min.x = DecodeBlock(bbox2_min_x_sign_mantissa, commonExp.x, false);
    box32.bbox2_min.y = DecodeBlock(bbox2_min_y_sign_mantissa, commonExp.y, false);
    box32.bbox2_min.z = DecodeBlock(bbox2_min_z_sign_mantissa, commonExp.z, false);

    uint bbox2_max_x_sign_mantissa = (node.dword[10] >> 30) | ((node.dword[11] & 0xFFFF) << 2);
    uint bbox2_max_y_sign_mantissa = (node.dword[11] >> 16) | ((node.dword[12] & 0x3) << 16);
    uint bbox2_max_z_sign_mantissa = (node.dword[12] >> 2) & 0x3FFFF;
    box32.bbox2_max.x = DecodeBlock(bbox2_max_x_sign_mantissa, commonExp.x, true);
    box32.bbox2_max.y = DecodeBlock(bbox2_max_y_sign_mantissa, commonExp.y, true);
    box32.bbox2_max.z = DecodeBlock(bbox2_max_z_sign_mantissa, commonExp.z, true);

    uint bbox3_min_x_sign_mantissa = (node.dword[12] >> 20) | ((node.dword[13] & 0x3F) << 12);
    uint bbox3_min_y_sign_mantissa = (node.dword[13] >> 6) & 0x3FFFF;
    uint bbox3_min_z_sign_mantissa = (node.dword[13] >> 24) | ((node.dword[14] & 0x3FF) << 8);
    box32.bbox3_min.x = DecodeBlock(bbox3_min_x_sign_mantissa, commonExp.x, false);
    box32.bbox3_min.y = DecodeBlock(bbox3_min_y_sign_mantissa, commonExp.y, false);
    box32.bbox3_min.z = DecodeBlock(bbox3_min_z_sign_mantissa, commonExp.z, false);

    uint bbox3_max_x_sign_mantissa = (node.dword[14] >> 10) & 0x3FFFF;
    uint bbox3_max_y_sign_mantissa = (node.dword[14] >> 28) | ((node.dword[15] & 0x3FFF) << 4);
    uint bbox3_max_z_sign_mantissa = (node.dword[15] >> 14);
    box32.bbox3_max.x = DecodeBlock(bbox3_max_x_sign_mantissa, commonExp.x, true);
    box32.bbox3_max.y = DecodeBlock(bbox3_max_y_sign_mantissa, commonExp.y, true);
    box32.bbox3_max.z = DecodeBlock(bbox3_max_z_sign_mantissa, commonExp.z, true);

#if GPURT_BUILD_RTIP3_1
    box32.obbMatrixIndex = INVALID_OBB;
#endif

    return box32;
}

//=====================================================================================================================
static uint LoadBits(
    RWByteAddressBuffer DataBuffer, uint nodeOffset, uint bitOffset, uint size)
{
    uint dword0_address = nodeOffset + ((bitOffset / 32) * sizeof(uint32_t));
    uint dword0_offset  = bitOffset & 31;
    uint dword0_size    = min(32 - dword0_offset, size);

    // Fetch packed bits as DWORD (Note, could further optimize by using global_load_ubyte?)
    const uint dword0 = DataBuffer.Load(dword0_address);

    uint result = (dword0 >> dword0_offset) & bits(dword0_size);

    // For straddled bit fields, fetch next set of bits from data and extract remaining bits
    if (dword0_size != size)
    {
        uint dword1_offset = 0;
        uint dword1_size   = (size + dword0_offset) - 32;

        const uint dword1 = DataBuffer.Load(dword0_address + sizeof(uint32_t));

        const uint remainingBits = (dword1 & bits(dword1_size)) << dword0_size;
        result |= remainingBits;
    }

    return result;
}

//=====================================================================================================================
// globallycoherent version
static uint LoadBits_gc(
    globallycoherent RWByteAddressBuffer DataBuffer, uint nodeOffset, uint bitOffset, uint size)
{
    uint dword0_address = nodeOffset + ((bitOffset / 32) * sizeof(uint32_t));
    uint dword0_offset  = bitOffset & 31;
    uint dword0_size    = min(32 - dword0_offset, size);

    // Fetch packed bits as DWORD (Note, could further optimize by using global_load_ubyte?)
    const uint dword0 = DataBuffer.Load(dword0_address);

    uint result = (dword0 >> dword0_offset) & bits(dword0_size);

    // For straddled bit fields, fetch next set of bits from data and extract remaining bits
    if (dword0_size != size)
    {
        uint dword1_offset = 0;
        uint dword1_size   = (size + dword0_offset) - 32;

        const uint dword1 = DataBuffer.Load(dword0_address + sizeof(uint32_t));

        const uint remainingBits = (dword1 & bits(dword1_size)) << dword0_size;
        result |= remainingBits;
    }

    return result;
}

//=====================================================================================================================
static void StoreBits(
    RWByteAddressBuffer DataBuffer, uint nodeOffset, uint bitOffset, uint size, uint data)
{
    uint dword0_address = nodeOffset + ((bitOffset / 32) * sizeof(uint32_t));
    uint dword0_offset  = bitOffset & 31;
    uint dword0_size    = min(32 - dword0_offset, size);
    uint dword0_mask    = bits(dword0_size) << dword0_offset;

    uint dword0 = DataBuffer.Load(dword0_address) & ~dword0_mask;
    dword0 |= ((data << dword0_offset) & dword0_mask);
    DataBuffer.Store(dword0_address, dword0);

    if (dword0_size != size)
    {
        uint dword1_size = size - dword0_size;
        uint dword1_mask = bits(dword1_size);

        uint dword1 = DataBuffer.Load(dword0_address + sizeof(uint32_t)) & ~dword1_mask;
        dword1 |= ((data >> dword0_size) & dword1_mask);
        DataBuffer.Store(dword0_address + sizeof(uint32_t), dword1);
    }
}

// ====================================================================================================================
static int GetChildPlaneMinBitOffset(
    in int idx)
{
    // Each component of the box plane is 18-bits starting from the 80th bit in the 64-byte node
    return 80 + (18 * idx * 6);
}

// ====================================================================================================================
static int GetChildPlaneMaxBitOffset(
    in int idx)
{
    // Each component of the box plane is 18-bits starting from the 80th bit in the 64-byte node and max follows min
    return 80 + (18 * ((idx * 6) + 3));
}

#if GPURT_BVH_BUILD_SHADER
//=====================================================================================================================
static void UpdateHighPrecisionBoxNodeFlags(
    in uint             nodeOffset,
    in uint             childIdx,
    in uint             boxNodeFlags)
{
    const uint bitMask = GetHighPrecisionBoxChildFlagsBitMask(childIdx);
    const uint fieldMask = bitMask << (childIdx * HPB64_BOX_NODE_FLAGS_BIT_STRIDE);
    const uint fieldData = (boxNodeFlags & bitMask) << (childIdx * HPB64_BOX_NODE_FLAGS_BIT_STRIDE);

    // Compute DWORD aligned byte offset and bit offset within compressed node
    const uint boxNodeFlagsOffset = (HPB64_BOX_NODE_FLAGS_BIT_OFFSET / 32) * sizeof(uint32_t);
    const uint packedFlagsBitOffset = (HPB64_BOX_NODE_FLAGS_BIT_OFFSET % 32);

    const uint boxNodeFlagsClearBits = ~(fieldMask << packedFlagsBitOffset);
    const uint boxNodeFlagsSetBits = (fieldData << packedFlagsBitOffset);

    DstMetadata.InterlockedAnd(nodeOffset + boxNodeFlagsOffset, boxNodeFlagsClearBits);
    DstMetadata.InterlockedOr(nodeOffset + boxNodeFlagsOffset, boxNodeFlagsSetBits);
}

//=====================================================================================================================
static uint4 DecodeHighPrecisionBoxNodeChildPointers(
    uint                nodeOffset)
{
    uint childBasePtr = SrcBuffer.Load(nodeOffset);
    const uint childTypeAndBoxFlags = LoadBits(SrcBuffer, nodeOffset, 32, 24);

    // Decode child node pointer types
    uint4 type;
    type[0] = (childBasePtr >> 29);
    type[1] = (childTypeAndBoxFlags) & bits(3);
    type[2] = (childTypeAndBoxFlags >> 3) & bits(3);
    type[3] = (childTypeAndBoxFlags >> 6) & bits(3);

    // Decode child base pointer
    childBasePtr = childBasePtr & bits(29);

    // Decode 64-byte aligned child offsets. Note, only instance node is 128 bytes. All other nodes are 64-bytes

    uint4 offset;
    offset[0] = childBasePtr;
    offset[1] = offset[0] +
        (((type[0] == NODE_TYPE_USER_NODE_INSTANCE) || (type[0] == NODE_TYPE_BOX_HP64x2)) ? 2 : 1);
    offset[2] = offset[1] +
        (((type[1] == NODE_TYPE_USER_NODE_INSTANCE) || (type[1] == NODE_TYPE_BOX_HP64x2)) ? 2 : 1);
    offset[3] = offset[2] +
        (((type[2] == NODE_TYPE_USER_NODE_INSTANCE) || (type[2] == NODE_TYPE_BOX_HP64x2)) ? 2 : 1);

    // Convert to absolute offset
    offset[0] = offset[0] << 6;
    offset[1] = offset[1] << 6;
    offset[2] = offset[2] << 6;
    offset[3] = offset[3] << 6;

    // Exclude children with inverted bounding boxes while calculating max exponent
    uint4 childPointers;

    childPointers.x =
        (type[0] == INVALID_TYPE_OF_HP) ? INVALID_NODE : PackNodePointer(type[0], offset[0]);
    childPointers.y =
        (type[1] == INVALID_TYPE_OF_HP) ? INVALID_NODE : PackNodePointer(type[1], offset[1]);
    childPointers.z =
        (type[2] == INVALID_TYPE_OF_HP) ? INVALID_NODE : PackNodePointer(type[2], offset[2]);
    childPointers.w =
        (type[3] == INVALID_TYPE_OF_HP) ? INVALID_NODE : PackNodePointer(type[3], offset[3]);

    return childPointers;
}

//=====================================================================================================================
// Count valid children in high precision box node
static uint HighPrecisionBoxNodeCountValidChildren(
    RWByteAddressBuffer DataBuffer, uint nodeOffset)
{
    uint numValidChildren = 0;

    const uint child0_bbox_min_x_sign = LoadBits(DataBuffer, nodeOffset, GetChildPlaneMinBitOffset(0) + 17, 1);
    const uint child1_bbox_min_x_sign = LoadBits(DataBuffer, nodeOffset, GetChildPlaneMinBitOffset(1) + 17, 1);
    const uint child2_bbox_min_x_sign = LoadBits(DataBuffer, nodeOffset, GetChildPlaneMinBitOffset(2) + 17, 1);
    const uint child3_bbox_min_x_sign = LoadBits(DataBuffer, nodeOffset, GetChildPlaneMinBitOffset(3) + 17, 1);

    const uint child0_bbox_max_x_sign = LoadBits(DataBuffer, nodeOffset, GetChildPlaneMaxBitOffset(0) + 17, 1);
    const uint child1_bbox_max_x_sign = LoadBits(DataBuffer, nodeOffset, GetChildPlaneMaxBitOffset(1) + 17, 1);
    const uint child2_bbox_max_x_sign = LoadBits(DataBuffer, nodeOffset, GetChildPlaneMaxBitOffset(2) + 17, 1);
    const uint child3_bbox_max_x_sign = LoadBits(DataBuffer, nodeOffset, GetChildPlaneMaxBitOffset(3) + 17, 1);

    numValidChildren += (child0_bbox_min_x_sign < child0_bbox_max_x_sign) ? 0 : 1;
    numValidChildren += (child1_bbox_min_x_sign < child1_bbox_max_x_sign) ? 0 : 1;
    numValidChildren += (child2_bbox_min_x_sign < child2_bbox_max_x_sign) ? 0 : 1;
    numValidChildren += (child3_bbox_min_x_sign < child3_bbox_max_x_sign) ? 0 : 1;

    return numValidChildren;
}
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
    in uint3 globalThreadId : SV_DispatchThreadID)
{
    const Float32BoxNode f32BoxNode = SrcBuffer.Load<Float32BoxNode>(globalThreadId.x * sizeof(Float32BoxNode));

    HighPrecisionBoxNode hpBoxNode = EncodeHighPrecisionBoxNode(f32BoxNode);
    DstBuffer.Store<HighPrecisionBoxNode>(globalThreadId.x, hpBoxNode);
}

//=====================================================================================================================
// Main Function : Decode
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(32, 1, 1)]
void Decode(
    in uint3 globalThreadId : SV_DispatchThreadID)
{
    const HighPrecisionBoxNode hpBoxNode = SrcBuffer.Load<HighPrecisionBoxNode>(globalThreadId.x * sizeof(Float32BoxNode));

    Float32BoxNode f32BoxNode = DecodeHighPrecisionBoxNode(hpBoxNode);
    DstBuffer.Store<Float32BoxNode>(globalThreadId.x, f32BoxNode);
}
#endif

#endif
