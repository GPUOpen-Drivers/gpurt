/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
// MIT License
//
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

// For 32 bit morton code functions based on inkblot-sdnbhd
//https://github.com/inkblot-sdnbhd/Morton-Z-Code-C-library/blob/master/MZC2D32.h
// The MIT License(MIT)
//
// Copyright(c) 2015 Inkblot Sdn.Bhd.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files(the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef _MORTONCODES_HLSL
#define _MORTONCODES_HLSL

#include "../shadersClean/common/Common.hlsli"

//=====================================================================================================================
uint Expand2D32(uint x)
{
    // 32 bit function to expand an axiscode to 2D bits
    // For an input : xxxxxxxxxxxxxxx...
    // The output is: _x_x_x_x_x_x_x_...

    // Use 16 bits
    x = x >> 16U;

    // [8]8[8]8
    x = (x | (x << 8U)) & 0x00FF00FFU;

    // [4]4[4]4[4]4[4]4
    x = (x | (x << 4U)) & 0x0F0F0F0FU;

    // [2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2
    x = (x | (x << 2U)) & 0x33333333U;

    // [1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1
    x = (x | (x << 1U)) & 0x55555555U;

    // Note: Initial bit is empty
    return x;
}

//=====================================================================================================================
uint64_t Expand2D64(uint code)
{
    // 64 bit function to expand an axiscode to 2D bits
    // For an input : xxxxxxxxxxxxxxx...
    // The output is: _x_x_x_x_x_x_x_...

    uint64_t x = (uint64_t) code;

    // [16]16[16]16
    x = (x | (x << 16ULL)) & 0x0000FFFF0000FFFFULL;

    // [8]8[8]8[8]8[8]8
    x = (x | (x << 8ULL)) & 0x00FF00FF00FF00FFULL;

    // [4]4[4]4[4]4[4]4[4]4[4]4[4]4[4]4
    x = (x | (x << 4ULL)) & 0x0F0F0F0F0F0F0F0FULL;

    // [2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2
    x = (x | (x << 2ULL)) & 0x3333333333333333ULL;

    // [1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1....
    x = (x | (x << 1ULL)) & 0x5555555555555555ULL;

    // Note: Initial bit is empty
    return x;
}

//=====================================================================================================================
uint ExpandFor2DBits32(uint x)
{
    // 32 bit function to expand an existing code for insertion of 2D bits
    // For an input : xxxxxxxxxxxxxxx...
    // The output is: x_x_x_x_x_x_x_x..

    // input is 16 bits, at the top
    x &= 0xFFFF0000U;

    // 8[8]8[8]
    x = (x | (x >> 8U)) & 0xFF00FF00U;

    // 4[4]4[4]4[4]4[4]
    x = (x | (x >> 4U)) & 0xF0F0F0F0U;

    // 2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]
    x = (x | (x >> 2U)) & 0xCCCCCCCCU;

    // 1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]
    x = (x | (x >> 1U)) & 0xAAAAAAAAU;
    return x;
}

//=====================================================================================================================
uint64_t ExpandFor2DBits64(uint64_t value)
{
    // 64 bit function to expand an existing code for insertion of 2D bits
    // For an input : xxxxxxxxxxxxxxx...
    // The output is: x_x_x_x_x_x_x_x...

    // input is 32 bits, at the top
    value &= 0xFFFFFFFF00000000ULL;

    // 16[16]16[16]
    value = (value | (value >> 16ULL)) & 0xFFFF0000FFFF0000ULL;

    // 8[8]8[8]8[8]8[8]
    value = (value | (value >> 8ULL)) & 0xFF00FF00FF00FF00ULL;

    // 4[4]4[4]4[4]4[4]4[4]4[4]4[4]4[4]
    value = (value | (value >> 4ULL)) & 0xF0F0F0F0F0F0F0F0ULL;

    // 2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]2[2]
    value = (value | (value >> 2ULL)) & 0xCCCCCCCCCCCCCCCCULL;

    // 1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]1[1]....
    value = (value | (value >> 1ULL)) & 0xAAAAAAAAAAAAAAAAULL;
    return value;
}

//=====================================================================================================================
uint Expand3D32(uint code)
{
    // 32 bit function to expand an axiscode to 3D bits
    // For an input : xxxxxxxxxxxxxxx...
    // The output is: x__x__x__x__x__....

    // we only look at the first 11 bits
    uint x = code & 0xFFE00000U;

    // 6[12]5[9]
    x = (x | x >> 12U) & 0xFC003E00U;

    // 3[6]3[6]3[6]2[3]
    x = (x | x >> 6U) & 0xE0703818U;

    // 1[2]2[4]1[2]2[4]1[2]2[4]2[3]
    x = (x | x >> 2U) & 0x984C2618U;

    // 1[2]1[2]1[2]1[2]1[2]1[2]1[2]1[2]1[2]1[2]1[1]
    x = (x | x >> 2U) & 0x92492492U;

    return x;
}

//=====================================================================================================================
uint64_t Expand3D64(uint code)
{
    // 64 bit function to expand an axiscode to 3D bits
    // For an input : xxxxxxxxxxxxxxx...
    // The output is: x__x__x__x__x__....

    // we only look at the first 22 bits
    uint64_t x = (uint64_t) (code) << 32ULL;

    // 11[22]11[20]
    x = (x | x >> 22ULL) & 0xFFE000007FF00000ULL;

    // 6[12]5 [10]6[12]5[8]
    x = (x | x >> 12ULL) & 0xFC003E007E001F00ULL;

    // 3[6]3[6]3[6]2[4] 3[6]3[6]3[6]2[2]
    x = (x | x >> 6ULL) & 0xE070381870381C0CULL;

    // 1[2]2[4]1[2]2[4]1[2]2[4]2[4] 1[2]2[4]1[2]2[4]1[2]2[4]2[2]
    x = (x | x >> 2ULL) & 0x984C26184C26130CULL;

    // 1[2]1[2]1[2]1[2]1[2]1[2]1[2]1[2]1[2]1[2]1[2]1[2]1[2]1[2]1[2]1
    x = (x | x >> 2ULL) & 0x9249249249249249ULL;

    return x;
}

//=====================================================================================================================
uint ExpandFor3DBits32(uint x)
{
    // 32 bit function to expand an existing code for insertion of 3D bits
    // For an input : xyxyxyxyxyxyxy...
    // The output is: xy_xy_xy_xy_xy_....

    // input is 22 bits, at the top
    x &= 0xFFFFFC00U;

    // 12[6]10[4]
    uint mask = 0x000FFC00U;
    x = ((x & mask) >> 6U) | (x & ~mask);

    // 6[3]6[3]6[3]4[1]
    mask = 0x03F000F0U;
    x = ((x & mask) >> 3U) | (x & ~mask);

    // 4[2]2[1]4[2]2[1]4[2]2[1]4[1]
    mask = 0x0C060300U;
    x = ((x & mask) >> 2U) | (x & ~mask);

    // 2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2
    mask = 0x30180C06U;
    x = ((x & mask) >> 1U) | (x & ~mask);

    return x;
}

//=====================================================================================================================
uint64_t ExpandFor3DBits64(uint64_t value)
{
    // 64 bit function to expand an existing code for insertion of 3D bits
    // For an input : xyxyxyxyxyxyxy...
    // The output is: xy_xy_xy_xy_xy_....

    // input is 43 bits, at the top
    value &= 0xFFFFFFFFFFE00000ULL;

    // 22[11]21[10]
    uint64_t mask = 0x000003FFFFE00000ULL;
    value = ((value & mask) >> 11ULL) | (value & ~mask);

    // 12[6]10[5] 12[6]9[4]
    mask = 0x000FFC000007FC00ULL;
    value = ((value & mask) >> 6ULL) | (value & ~mask);

    // 6[3]6[3]6[3]4[2] 6[3]6[3]6[3]3[1]
    mask = 0x03F000F001F80070ULL;
    value = ((value & mask) >> 3ULL) | (value & ~mask);

    // 4[2]2[1]4[2]2[1]4[2]2[1]4[2] 4[2]2[1]4[2]2[1]4[2]2[1]3[1]
    mask = 0x0C06030006030180ULL;
    value = ((value & mask) >> 2ULL) | (value & ~mask);

    // 2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]2[1]1
    mask = 0x30180C06180C0602ULL;
    value = ((value & mask) >> 1ULL) | (value & ~mask);

    return value;
}

//=====================================================================================================================
uint MortonCode3D32(uint x, uint y, uint z)
{
    // Generates a 32 bit regular morton code in xyzxyz...

    // x__x__x__x__x__x__x...
    const uint xx = Expand3D32(x);

    // y__y__y__y__y__y__y...
    const uint yy = Expand3D32(y);

    // z__z__z__z__z__z__z...
    const uint zz = Expand3D32(z);

    // Shift and or to obtain xyzxyzxyzxyz....
    return (xx) | (yy >> 1U) | (zz >> 2U);
}

//=====================================================================================================================
uint64_t MortonCode3D64(uint x, uint y, uint z)
{
    // Generates a 64 bit regular morton code in xyzxyz..

    // x__x__x__x__x__x__x...
    const uint64_t xx = Expand3D64(x);

    // y__y__y__y__y__y__y...
    const uint64_t yy = Expand3D64(y);

    // z__z__z__z__z__z__z...
    const uint64_t zz = Expand3D64(z);

    // Shift and or to obtain xyzxyzxyzxyz....
    return (xx) | (yy >> 1ULL) | (zz >> 2ULL);
}

//=====================================================================================================================
uint CreatePrebitMask32(uint prebits)
{
    // Generates a 32 bit mask for the given number of prebits
    // E.G. Input = 4
    //     Output = 111100000....
    return prebits > 0U ? 0xFFFFFFFFU << (32U - prebits) : 0U;
}

//=====================================================================================================================
uint64_t CreatePrebitMask64(uint prebits)
{
    // Generates a 64 bit mask for the given number of prebits
    // E.G. Input = 6
    //     Output = 111111000....
    return prebits > 0U ? 0xFFFFFFFFFFFFFFFFULL << (uint64_t) (64U - prebits) : 0U;
}

//=====================================================================================================================
uint FastVariableBitMorton32(uint3 codes, float3 extents)
{
    // Generates a 32-bit variable bit morton code

    // Sort codes and extents
    if (extents.x < extents.y)
    {
        if (extents.y < extents.z)
        {
            // z y x
            extents = extents.zyx;
            codes = codes.zyx;
        }
        else if (extents.x < extents.z)
        {
            // y z x
            extents = extents.yzx;
            codes = codes.yzx;
        }
        else
        {
            // y x z
            extents = extents.yxz;
            codes = codes.yxz;
        }
    }
    else
    {
        if (extents.x < extents.z)
        {
            // z x y
            extents = extents.zxy;
            codes = codes.zxy;
        }
        else if (extents.y < extents.z)
        {
            // x z y
            extents = extents.xzy;
            codes = codes.xzy;
        }
        // other case is xyz, but extents is already in that order
    }

    // NOTE: all comments here will assume the order is xyz of largest to smallest axis

    uint mortonCode;

    // if there are no prebits between the largest and smallest axis, there will also not
    // be any prebits between x and y, and y and z
    const uint prebitsXZ = log2(extents.x / extents.z);
    if (prebitsXZ == 0U)
    {
        // There are no prebits in this code, so early exit preventing 1 expansion
        mortonCode = MortonCode3D32(codes.x, codes.y, codes.z);
    }
    else
    {
        // This code has prebits

        // Start with only x bits
        mortonCode = codes.x;

        // Fetch the prebits from different axes
        // This calculation computes the number of bits required BEFORE x becomes smaller than z to prevent resorting
        const uint prebitsXY = log2(extents.x / extents.y);
        const uint prebitsYZ = log2(extents.y / extents.z);
        const uint prebits1D = prebitsXY;

        // < 31, as we'd first insert x again, but that is already present in the current code
        if (prebits1D < 31U)
        {
            const uint prebitMask1D = CreatePrebitMask32(prebits1D);
            const uint prebits2D = prebitsXZ + prebitsYZ;

            // Check if there are no 2D prebits
            if (prebits1D == prebits2D)
            {
                // Short circuit from 1D to 3D bits, prevents 1 expansion

                // regular tail: xyzxyzxyz
                uint tail = MortonCode3D32(codes.x << prebits1D, codes.y, codes.z);

                // E.G. 1D prebits = 4:
                //      1D mask = xxxx________...
                //      tail    = ____xyzxyzxy... (shifted by prebits1D)
                //      Or op   = xxxxxyzxyzxy...
                mortonCode = (mortonCode & prebitMask1D) | (tail >> prebits1D);
            }
            else
            {
                // Default case with 1D and 2D prebits

                // Mask out 1D prebits and 'weave' in 2D tail
                // ExpandFor2DBits = x_x_x_x_x_x_...
                // Expand2D        = _y_y_y_y_y_y...
                // Or op           = xyxyxyxyxyxy...
                uint tail2D = ExpandFor2DBits32(mortonCode << (prebits1D)) | (Expand2D32(codes.y));

                // E.G. 1D prebits = 6:
                //      1D mask = xxxxxx______...
                //      tail2D  = ______xyxyxy... (shifted by prebits1D)
                //      Or op   = xxxxxxxyxyxy...
                mortonCode = (mortonCode & prebitMask1D) | (tail2D >> prebits1D);

                // Check to prevent overshifting, 30 as we'd first insert xy again, but that already is currently in the code
                if (prebits2D < 30U)
                {
                    // Contains all 1D and 2D prebits
                    const uint prebitMask2D = CreatePrebitMask32(prebits2D);

                    // Mask out 1D and 2D prebits and 'weave' in 3D tail
                    // ExpandFor3DBits = xy_xy_xy_xy_...
                    // Expand3D        = __z__z__z__z... (shifted by 2U)
                    // Or op           = xyzxyzxyzxyz...
                    uint tail3D = ExpandFor3DBits32(mortonCode << (prebits2D)) | (Expand3D32(codes.z) >> 2U);

                    // E.G. 2D prebits = 6 (Includes 1D prebits)
                    //      2D mask = xxxyxy______... (2 1D bits + 4 2D bits)
                    //      tail3D  = ______xyzxyz... (shifted by prebits2D)
                    //      Or op   = xxxyxyxyzxyz...
                    // OR (in case of uneven 2D prebits)
                    //      2D prebits = 5 (Includes 1D prebits)
                    //      2D mask = xxxyx_______... (2 1D bits + 3 2D bits)
                    //      tail3D  = _____yxzyxzy... (shifted by prebits2D)
                    //      Or op   = xxxyxyxzyxzy...
                    mortonCode = (mortonCode & prebitMask2D) | (tail3D >> prebits2D);
                }
            }
        }
    }

    return mortonCode;
}

//=====================================================================================================================
uint64_t FastVariableBitMorton64(uint3 codes, float3 extents)
{
    // Generates a 64-bit variable bit morton code

    // Sort codes and extents
    if (extents.x < extents.y)
    {
        if (extents.y < extents.z)
        {
            // z y x
            extents = extents.zyx;
            codes = codes.zyx;
        }
        else if (extents.x < extents.z)
        {
            // y z x
            extents = extents.yzx;
            codes = codes.yzx;
        }
        else
        {
            // y x z
            extents = extents.yxz;
            codes = codes.yxz;
        }
    }
    else
    {
        if (extents.x < extents.z)
        {
            // z x y
            extents = extents.zxy;
            codes = codes.zxy;
        }
        else if (extents.y < extents.z)
        {
            // x z y
            extents = extents.xzy;
            codes = codes.xzy;
        }
        // other case is xyz, but extents is already in that order
    }

    // NOTE: all comments here will assume the order is xyz of largest to smallest axis

    uint64_t mortonCode;

    // if there are no prebits between the largest and smallest axis, there will also not
    // be any prebits between x and y, and y and z
    const uint prebitsXZ = log2(extents.x / extents.z);
    if (prebitsXZ == 0U)
    {
        // There are no prebits in this code, so early exit preventing 1 expansion
        mortonCode = MortonCode3D64(codes.x, codes.y, codes.z);
    }
    else
    {
        // This code has prebits

        // Start with only x bits
        mortonCode = ((uint64_t)(codes.x) << 32ULL);

        // Fetch the prebits from different axes
        // This calculation computes the number of bits required BEFORE x becomes smaller than z to prevent resorting
        const uint prebitsXY = log2(extents.x / extents.y);
        const uint prebitsYZ = log2(extents.y / extents.z);
        const uint prebits1D = prebitsXY;

        // < 31 because if we have 31 prebits, the 32nd bit would be an x as well
        if (prebits1D < 31U)
        {
            const uint64_t prebitMask1D = CreatePrebitMask64(prebits1D);
            const uint prebits2D = prebitsXZ + prebitsYZ;

            // Check if there are no 2D prebits
            if (prebits1D == prebits2D)
            {
                // Short circuit, 1D to 3D, prevents 1 expansion

                // regular tail: xyzxyzxyz
                const uint64_t tail3D = MortonCode3D64(codes.x << prebits1D, codes.y, codes.z);

                // E.G. 1D prebits = 4:
                //      1D mask = xxxx________...
                //      tail    = ____xyzxyzxy... (shifted by prebits1D)
                //      Or op   = xxxxxyzxyzxy...
                mortonCode = (mortonCode & prebitMask1D) | (tail3D >> prebits1D);
            }
            else
            {
                // Default case of 1D and 2D prebits

                // Mask out 1D prebits and 'weave' in 2D tail
                // ExpandFor2DBits = x_x_x_x_x_x_...
                // Expand2D        = _y_y_y_y_y_y...
                // Or op           = xyxyxyxyxyxy...
                const uint64_t tail2D = ExpandFor2DBits64(mortonCode << ((uint64_t)prebits1D)) | (Expand2D64(codes.y));

                // E.G. 1D prebits = 6:
                //      1D mask = xxxxxx______...
                //      tail2D  = ______xyxyxy... (shifted by prebits1D)
                //      Or op   = xxxxxxxyxyxy...
                mortonCode = (mortonCode & prebitMask1D) | (tail2D >> prebits1D);

                // Check to prevent overshifting, 62 as we'd first insert xy again, but that already is currently in the code
                if (prebits2D < 62U)
                {
                    // Contains all 1D and 2D prebits
                    const uint64_t prebitMask2D = CreatePrebitMask64(prebits2D);

                    // Mask out 1D and 2D prebits and 'weave' in 3D tail

                    // ExpandFor3DBits = xy_xy_xy_xy_...
                    // Expand3D        = __z__z__z__z... (shifted by 2U)
                    // Or op           = xyzxyzxyzxyz...
                    const uint64_t tail3D = ExpandFor3DBits64(mortonCode << ((uint64_t)prebits2D)) | (Expand3D64(codes.z) >> 2ULL);

                    // E.G. 2D prebits = 6 (Includes 1D prebits)
                    //      2D mask = xxxyxy______... (2 1D bits + 4 2D bits)
                    //      tail3D  = ______xyzxyz... (shifted by prebits2D)
                    //      Or op   = xxxyxyxyzxyz...
                    // OR (in case of uneven 2D prebits)
                    //      2D prebits = 5 (Includes 1D prebits)
                    //      2D mask = xxxyx_______... (2 1D bits + 3 2D bits)
                    //      tail3D  = _____yxzyxzy... (shifted by prebits2D)
                    //      Or op   = xxxyxyxzyxzy...
                    mortonCode = (mortonCode & prebitMask2D) | (tail3D >> prebits2D);
                }
            }
        }
        else if (prebitsYZ < 31U)
        {
            // Edge case where more 1D bits are required than possible
            // Prevents 4 64 bit expansions for 2 32 bit expansions

            // 1D part of tail
            // E.G. prebitsYZ = 4
            //           tail = yyyy______
            uint tail = codes.y & CreatePrebitMask32(prebitsYZ);

            // ExpandFor2DBits = y_y_y_y_y_y_...
            // Expand2D        = _z_z_z_z_z_z...
            // Or op           = yzyzyzyzyzyz...
            const uint tail2D = ExpandFor2DBits32(codes.y << prebitsYZ) | Expand2D32(codes.z);

            // E.G. prebitsYZ = 3:
            //      1D mask = yyy_________...
            //      tail2D  = ___yzyzyzyzy... (shifted by prebitsYZ)
            //      Or op   = yyyyzyzyzyzy...
            tail = tail | (tail2D >> prebitsYZ);

            // Combines the 32 bit tail with the existing 1D prebits
            // E.G.  tail = ________________________________yyyyyzyzyzy...
            // mortonCode = xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx___________...
            // Or op      = xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxyyyyyzyzyzy...
            mortonCode = mortonCode | ((uint64_t)tail);
        }
        else
        {
            // All bits in the tail are y's
            //       tail = ________________________________yyyyyyyyyyy...
            // mortonCode = xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx___________...
            // Or op      = xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxyyyyyyyyyyy...
            mortonCode = mortonCode | ((uint64_t) codes.y);
        }
    }
    return mortonCode;
}

//=====================================================================================================================
static uint64_t ExpandBits4D(uint a)
{
    uint64_t w = a & 0x7fff; // we only look at the first 15 bits

    uint64_t mask = (((1ULL << 7) - 1) << 32) | ((1ULL << 8) - 1);

    w = (w | (w << 24)) & mask;

    mask = (((1ULL << 3) - 1) << 48) |
           (((1ULL << 4) - 1) << 32) |
           (((1ULL << 4) - 1) << 16) |
           ((1ULL << 4) - 1);

    w = (w | (w << 12)) & mask;

    mask = (((1ULL << 1) - 1) << 56) |
           (((1ULL << 2) - 1) << 48) |
           (((1ULL << 2) - 1) << 40) |
           (((1ULL << 2) - 1) << 32) |
           (((1ULL << 2) - 1) << 24) |
           (((1ULL << 2) - 1) << 16) |
           (((1ULL << 2) - 1) << 8) |
           ((1ULL << 2) - 1);

    w = (w | (w << 6)) & mask;

    mask = 0;

#if !__cplusplus
    [unroll]
#endif
    for (uint b = 0; b < 15; b++)
    {
        mask |= 1ULL << (b * 4);
    }

    w = (w | (w << 3)) & mask;

    return w;
}

//=====================================================================================================================
uint ExpandForSizeBits(uint value)
{
    // input is 24 bits
    value &= ((1U << 24) - 1);

    uint mask = ((1U << 12) - 1);

    // 12[4]12
    value = ((value & ~mask) << 4) | (value & mask);

    mask = (((1U << 6) - 1) << 6) | (((1U << 6) - 1) << 22);

    // 6[2]6[2]6[2]6
    value = (value & ~mask) | ((value & mask) << 2);

    mask = (((1U << 3) - 1) << 3) +
           (((1U << 3) - 1) << 11) +
           (((1U << 3) - 1) << 19) +
           (((1U << 3) - 1) << 27);

    // 3[1]3 [1] 3[1]3 [1] 3[1]3 [1] 3[1]3
    value = ((value & mask) << 1) | (value & ~mask);

    return value;
}

//=====================================================================================================================
uint3 CalculateAxisCodes(
    float3 boundsMin,
    float3 boundsExtent,
    float3 position,
    uint isDegenerate
)
{
    float3 normalizedPos = isDegenerate ? float3(0.5f, 0.5f, 0.5f) : clamp((position - boundsMin) / boundsExtent, 0.0f, 0.99999994f);

    return uint3(normalizedPos * float(0xFFFFFFFFU));
}

//=====================================================================================================================
uint32_t CalculateMortonCode32(
    float3 boundsMin,
    float3 boundsExtent,
    float3 position,
    uint isDegenerate)
{
    const uint3 axisCodes = CalculateAxisCodes(
        boundsMin,
        boundsExtent,
        position,
        isDegenerate
    );

    bool regularCodes = IsRegularMortonCodeEnabled();

    const uint32_t mortonCode = regularCodes ? MortonCode3D32(axisCodes.x, axisCodes.y, axisCodes.z) :
                                               FastVariableBitMorton32(axisCodes, boundsExtent);

    // TODO: make 32 bit morton codes actually 32 bit, they are now used as ints everywhere
    //       which disables the msb
    return mortonCode >> 1U;
}

//=====================================================================================================================
uint64_t CalculateMortonCode64(
    float3 boundsMin,
    float3 boundsExtent,
    float2 sizeMinMax,
    float3 position,
    float surfaceArea,
    uint isDegenerate,
    uint numSizeBits
)
{
    const uint3 axisCodes = CalculateAxisCodes(
        boundsMin,
        boundsExtent,
        position,
        isDegenerate
    );

    uint64_t mortonCode = 0;

    if (IsRegularMortonCodeEnabled())
    {
        mortonCode = MortonCode3D64(axisCodes.x, axisCodes.y, axisCodes.z);
    }
    else
    {
        mortonCode = FastVariableBitMorton64(axisCodes, boundsExtent);
    }

    if (numSizeBits > 0)
    {
        const uint64_t mortonCodeTemp = mortonCode;

        const float size = (surfaceArea - sizeMinMax.x) / (sizeMinMax.y - sizeMinMax.x);

        uint w = min(max(size * (1UL << numSizeBits), 0.0), (1UL << numSizeBits) - 1);

        const uint64_t sizeValue = ExpandBits4D(w);

        const uint numAxisBitsWithSize = numSizeBits * 3;

        const uint numAxisBitsWithNoSize = 64U - numAxisBitsWithSize;

        const uint bitsToExpand = uint(mortonCodeTemp >> numAxisBitsWithNoSize);

        mortonCode = (ExpandForSizeBits(bitsToExpand) << 1) | sizeValue;

        mortonCode = (mortonCode << numAxisBitsWithNoSize) | (mortonCodeTemp & ((1ULL << numAxisBitsWithNoSize) - 1ULL));
    }

    return mortonCode;
}

#endif
