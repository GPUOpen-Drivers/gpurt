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

#ifndef _OBB_COMMON_HLSL
#define _OBB_COMMON_HLSL

#define INVALID_OBB 0x7f

static const uint64_t ObbStage1Lut[104] = {
    0x162c0ad6000019, 0x16ac02d6000019, 0x11440c51000019, 0x11c40451000019, 0xb580d8b000019,
    0xbd8058b000019,  0x640e40000019,   0xe40640000019,   0x1602b01900b016, 0x1600b01902b016,
    0x11031019011011, 0x11011019031011, 0xb03601901600b,  0xb01601903600b,  0x39019019000,
    0x19019039000,    0x190000162c0ad6, 0x19000016ac02d6, 0x19000011440c51, 0x19000011c40451,
    0x1900000b580d8b, 0x1900000bd8058b, 0x19000000640e40, 0x19000000e40640, 0x18066058186996,
    0x180460589a6196, 0x140ac09430cb11, 0x1408c094b2c311, 0x101ef1d03cfbcb, 0x101cf1d0bef3cb,
    0xc33130c451c40,  0xc31130cc71440,  0x188468581a6996, 0x18866858986196, 0x1488c89432cb11,
    0x148ac894b0c311, 0x109cf9d03efbcb, 0x109ef9d0bcf3cb, 0xcb11b0c471c40,  0xcb31b0cc51440,
    0x18181996181998, 0x18981196981198, 0x14302b11302b14, 0x14b02311b02314, 0x103c7bcb3c7bd0,
    0x10bc73cbbc73d0, 0xc44cc4044cc4c,  0xcc4c440c4c44c,  0x181a19969a1198, 0x189a11961a1998,
    0x14322b11b22314, 0x14b22311322b14, 0x103e7bcbbe73d0, 0x10be73cb3e7bd0, 0xc46cc40c6c44c,
    0xcc6c44046cc4c,  0x161a6998046058, 0x16986198066058, 0x1132cb1408c094, 0x11b0c3140ac094,
    0xb3efbd01cf1d0,  0xbbcf3d01ef1d0,  0x471c4c31130c,   0xc5144c33130c,   0x169a6198846858,
    0x16186998866858, 0x11b2c31488c894, 0x1130cb148ac894, 0xbbef3d09cf9d0,  0xb3cfbd09ef9d0,
    0xc7144cb11b0c,   0x451c4cb31b0c,   0x171638d71458d7, 0x178c51578e3157, 0x13368a1334da13,
    0x13a0d353a28353, 0xe4a9a4e492a4e,  0xea5248ea6948e,  0xa56490a55590a,  0xa91554a92454a,
    0x178e51570c3957, 0x171438d79650d7, 0x13a2d353208b53, 0x13348a13b6d213, 0xea7248e249c8e,
    0xe489a4ecb224e,  0xa93554a104d4a,  0xa54490ad7510a,  0x170c59570e3957, 0x179630d79450d7,
    0x1320db53228b53, 0x13b68213b4d213, 0xe252c8e269c8e,  0xeca924ec9224e,  0xa115d4a124d4a,
    0xad6410ad5510a,  0x179430d71658d7, 0x170e59578c3157, 0x13b4821336da13, 0x1322db53a08353,
    0xec8924e4b2a4e,  0xe272c8ea4948e,  0xad4410a57590a,  0xa135d4a90454a };

static const uint ObbStage2Lut[32] = {
    0x00000000, 0x3d1be50c, 0x3e15f61a, 0x3e484336, 0x3e79df93, 0x3e7c3a3a, 0x3e8a8bd4, 0x3e9e0875,
    0x3e9f0938, 0x3ea7bf1b, 0x3eaaaaab, 0x3ec3ef15, 0x3f000000, 0x3f01814f, 0x3f16a507, 0x3f273d75,
    0x3f30fbc5, 0x3f3504f3, 0x3f3d3a87, 0x3f4e034d, 0x3f5a827a, 0x3f692290, 0x3f6c835e, 0x3f73023f,
    0x3f7641af, 0x3f800000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 };

//=====================================================================================================================
// Map LUT matrix indices to locations in a 3x3 matrix.
static float MatrixIndexToFloat(uint index, uint row, uint col)
{
    // Each entry in the stage-1 lookup table contains 9 packed 6-bit indices corresponding
    // to the elements of the matrix.
    uint shift = (18 * row) + (6 * col);
    uint stage2Index = uint((ObbStage1Lut[index] >> shift) & 0x3f);

    // Bits [29:0] of the float are encoded in the stage-2 lookup table.
    // Bits [4:0] of the stage-2 index are used to access the stage-2 lookup table.
    // The sign bit is encoded in bit 5 of the stage2Index.
    // Bit 30 of the float will always be zero as all exponents are less than 128.
    uint outValue = ObbStage2Lut[stage2Index & 0x1f] | ((stage2Index >> 5) << 31);
    return asfloat(outValue);
}

//=====================================================================================================================
// Convert a discretized OBB index to a float3x3 matrix.
static float3x3 OBBIndexToMatrix(uint index)
{
    // If this box node is an OBB, decode the matrix and transform the ray origin and
    // direction before proceeding to the box test.
    float3x3 m = { MatrixIndexToFloat(index, 0, 0),
                   MatrixIndexToFloat(index, 0, 1),
                   MatrixIndexToFloat(index, 0, 2),
                   MatrixIndexToFloat(index, 1, 0),
                   MatrixIndexToFloat(index, 1, 1),
                   MatrixIndexToFloat(index, 1, 2),
                   MatrixIndexToFloat(index, 2, 0),
                   MatrixIndexToFloat(index, 2, 1),
                   MatrixIndexToFloat(index, 2, 2) };

    return m;
}
#endif
