/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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

#ifndef _MORTONCODES_HLSL
#define _MORTONCODES_HLSL

//=====================================================================================================================
// Expands 10 bits unsigned into into 30 bits unsigned int
static uint ExpandBits(in uint ui)
{
    const uint factors[4] = { 0x00010001u, 0x00000101u, 0x00000011u, 0x00000005u };
    const uint bitMasks[4] = { 0xFF0000FFu , 0x0F00F00Fu, 0xC30C30C3u, 0x49249249u };

    ui = (ui * factors[0]) & bitMasks[0];
    ui = (ui * factors[1]) & bitMasks[1];
    ui = (ui * factors[2]) & bitMasks[2];
    ui = (ui * factors[3]) & bitMasks[3];

    return ui;
}

//=====================================================================================================================
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
static uint ExpandBits2D(uint w)
{
    w &= 0x0000ffff;                  /* w = ---- ---- ---- ---- fedc ba98 7654 3210 */
    w = (w ^ (w << 8)) & 0x00ff00ff;  /* w = ---- ---- fedc ba98 ---- ---- 7654 3210 */
    w = (w ^ (w << 4)) & 0x0f0f0f0f;  /* w = ---- fedc ---- ba98 ---- 7654 ---- 3210 */
    w = (w ^ (w << 2)) & 0x33333333;  /* w = --fe --dc --ba --98 --76 --54 --32 --10 */
    w = (w ^ (w << 1)) & 0x55555555;  /* w = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0 */
    return w;
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
// Calculates a 30-bit Morton code for the
// given 3D point located within the unit cube [0,1].
static uint CalculateMortonCode(in float3 p)
{
    const float x = min(max(p.x * 1024.0, 0.0), 1023.0);
    const float y = min(max(p.y * 1024.0, 0.0), 1023.0);
    const float z = min(max(p.z * 1024.0, 0.0), 1023.0);
    const uint xx = ExpandBits(uint(x));
    const uint yy = ExpandBits(uint(y));
    const uint zz = ExpandBits(uint(z));
    return xx * 4 + yy * 2 + zz;
}

//=====================================================================================================================
// Expands 21 bits into 63 bits
// The following two functions are based on functions from
// https://github.com/Forceflow/libmorton/blob/main/libmorton/morton3D.h
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
static uint64_t ExpandBits64(in uint64_t a)
{
    uint64_t x = a & 0x1fffff; // we only look at the first 21 bits

    uint64_t temp = 0x1f00000000ffffull;
    x = (x | x << 32) & temp;

    temp = 0x1f0000ff0000ffull;
    x = (x | x << 16) & temp;

    temp = 0x100f00f00f00f00full;
    x = (x | x << 8) & temp;

    temp = 0x10c30c30c30c30c3ull;
    x = (x | x << 4) & temp;

    temp = 0x1249249249249249ull;
    x = (x | x << 2) & temp;

    return x;
}

//=====================================================================================================================
static uint64_t ExpandBits2D64(uint64_t w)
{
    w &= 0x00000000ffffffff;
    w = (w ^ (w << 16)) & 0x0000ffff0000ffff;
    w = (w ^ (w << 8)) & 0x00ff00ff00ff00ff;
    w = (w ^ (w << 4)) & 0x0f0f0f0f0f0f0f0f;
    w = (w ^ (w << 2)) & 0x3333333333333333;
    w = (w ^ (w << 1)) & 0x5555555555555555;
    return w;
}

//=====================================================================================================================
// Calculates a 63-bit Morton code for the
// given 3D point located within the unit cube [0,1].
static uint64_t CalculateMortonCode64(in float3 p)
{
    const float x = min(max(p.x * 2097152.0, 0.0), 2097151.0);
    const float y = min(max(p.y * 2097152.0, 0.0), 2097151.0);
    const float z = min(max(p.z * 2097152.0, 0.0), 2097151.0);
    const uint64_t xx = ExpandBits64(uint(x));
    const uint64_t yy = ExpandBits64(uint(y));
    const uint64_t zz = ExpandBits64(uint(z));

    return  xx * 4 + yy * 2 + zz;
}

//=====================================================================================================================
static uint64_t CalculateVariableBitsMortonCode64(
    float3 sceneExtent,
    float3 normalizedPos,
    uint numSizeBits,
    out_param(uint3) values,
    out_param(uint) numAxisBits,
    out_param(uint4) numMortonBitsPerAxis)
{
    int numMortonBits = 62;

    if (numSizeBits > 0)
    {
        numMortonBits -= numSizeBits;
    }

    numMortonBitsPerAxis = uint4(0, 0, 0, numSizeBits);
    int3 numBits = int3(0,0,0);
    int3 numPrebits;
    int3 startAxis;

    // find the largest start axis
    // and how many prebits are needed between largest and two other axes
    if (sceneExtent.x < sceneExtent.y)
    {
        if (sceneExtent.x < sceneExtent.z)
        {
            if (sceneExtent.y < sceneExtent.z)
            {
                // z, y, x
                startAxis[0] = 2;
                numPrebits[0] = log2(sceneExtent.z / sceneExtent.y);

                startAxis[1] = 1;
                numPrebits[1] = log2(sceneExtent.y / sceneExtent.x);

                startAxis[2] = 0;
                numPrebits[2] = log2(sceneExtent.z / sceneExtent.x);
            }
            else
            {
                // y, z, x
                startAxis[0] = 1;
                numPrebits[0] = log2(sceneExtent.y / sceneExtent.z);

                startAxis[1] = 2;
                numPrebits[1] = log2(sceneExtent.z / sceneExtent.x);

                startAxis[2] = 0;
                numPrebits[2] = log2(sceneExtent.y / sceneExtent.x);
            }
        }
        else
        {
            // y, x, z
            startAxis[0] = 1;
            numPrebits[0] = log2(sceneExtent.y / sceneExtent.x);

            startAxis[1] = 0;
            numPrebits[1] = log2(sceneExtent.x / sceneExtent.z);

            startAxis[2] = 2;
            numPrebits[2] = log2(sceneExtent.y / sceneExtent.z);
        }
    }
    else
    {
        if (sceneExtent.y < sceneExtent.z)
        {
            if (sceneExtent.x < sceneExtent.z)
            {
                // z, x, y
                startAxis[0] = 2;
                numPrebits[0] = log2(sceneExtent.z / sceneExtent.x);

                startAxis[1] = 0;
                numPrebits[1] = log2(sceneExtent.x / sceneExtent.y);

                startAxis[2] = 1;
                numPrebits[2] = log2(sceneExtent.z / sceneExtent.y);
            }
            else
            {
                // x, z, y
                startAxis[0] = 0;
                numPrebits[0] = log2(sceneExtent.x / sceneExtent.z);

                startAxis[1] = 2;
                numPrebits[1] = log2(sceneExtent.z / sceneExtent.y);

                startAxis[2] = 1;
                numPrebits[2] = log2(sceneExtent.x / sceneExtent.y);
            }
        }
        else
        {
            // x, y, z
            startAxis[0] = 0;
            numPrebits[0] = log2(sceneExtent.x / sceneExtent.y);

            startAxis[1] = 1;
            numPrebits[1] = log2(sceneExtent.y / sceneExtent.z);

            startAxis[2] = 2;
            numPrebits[2] = log2(sceneExtent.x / sceneExtent.z);
        }
    }

    if (sceneExtent[startAxis[2]] == 0)
    {
        numPrebits[1] = 0;
        numPrebits[2] = 0;
    }

    // say x > y > z
    // prebits[0] = 3
    // prebits[1] = 2
    // if swap == 1
    // xxx xy xy x yxz yxz ...
    // if swap == 0
    // xxx xy xy xyz xyz ...
    int swap = numPrebits[2] > (numPrebits[0] + numPrebits[1]) ? 1 : 0;

    numPrebits[0] = min(numPrebits[0], numMortonBits);
    numPrebits[1] = min(numPrebits[1] * 2, numMortonBits - numPrebits[0]) / 2;

    int numPrebitsSum = numPrebits[0] + numPrebits[1] * 2;

    if (numPrebitsSum != numMortonBits)
    {
        numPrebitsSum += swap;
    }
    else
    {
        swap = 0;
    }

    // the scene might be 2D so check for the smallest axis
    numBits[2] = (sceneExtent[startAxis[2]] != 0) ? max(0, (numMortonBits - numPrebitsSum) / 3) : 0;

    if (swap > 0)
    {
        numBits[0] = max(0, (numMortonBits - numBits[2] - numPrebitsSum) / 2 + numPrebits[1] + numPrebits[0] + 1);
        numBits[1] = numMortonBits - numBits[0] - numBits[2];
    }
    else
    {
        numBits[1] = max(0, (numMortonBits - numBits[2] - numPrebitsSum) / 2 + numPrebits[1]);
        numBits[0] = numMortonBits - numBits[1] - numBits[2];
    }

    const int delta = numBits[0] - 31; // clamp axis values to avoid overflow of a uint

    if (delta > 0)
    {
        numBits[0] -= delta;

        numPrebits[0] = min(numPrebits[0], numBits[0]);

        if (numBits[0] == numPrebits[0])
            swap = 0;

        numBits[1] = max(0, numBits[1] - delta);

        numPrebits[1] = min(numPrebits[1], numBits[1]);

        numBits[2] = max(0, numBits[2] - delta);

        numPrebitsSum = numPrebits[0] + numPrebits[1] * 2 + swap;
    }

    numAxisBits = numBits[2] + numBits[1] + numBits[0];

    uint64_t mortonCode = 0;
    uint64_t3 axisCode;

    // based on the number of bits, calculate each code per axis
#if !__cplusplus
    [unroll]
#endif
    for (uint a = 0; a < 3; a++)
    {
        axisCode[a] = min(max(uint(normalizedPos[startAxis[a]] * (1UL << numBits[a])), 0u), uint(1UL << numBits[a]) - 1u);
        numMortonBitsPerAxis[startAxis[a]] = numBits[a];
    }

    values[startAxis[0]] = uint(axisCode[0]);
    values[startAxis[1]] = uint(axisCode[1]);
    values[startAxis[2]] = uint(axisCode[2]);

    uint delta0 = 0;
    uint delta1 = 0;

    // if there are prebits, set them in the morton code:
    // if swap == 1
    // [xxx xy xy x] yxz yxz ...
    // if swap == 0
    // [xxx xy xy xyz] xyz ...
    if (numPrebitsSum > 0)
    {
        numBits[0] -= numPrebits[0];
        mortonCode = axisCode[0] & (((1ULL << numPrebits[0]) - 1) << numBits[0]);
        mortonCode >>= numBits[0];

        mortonCode <<= numPrebits[1] * 2;
        numBits[0] -= numPrebits[1];
        numBits[1] -= numPrebits[1];
        uint64_t temp0 = axisCode[0] & (((1ULL << numPrebits[1]) - 1) << numBits[0]);
        temp0 >>= numBits[0];
        temp0 = ExpandBits2D64(temp0);

        uint64_t temp1 = axisCode[1] & (((1ULL << numPrebits[1]) - 1) << numBits[1]);
        temp1 >>= numBits[1];
        temp1 = ExpandBits2D64(temp1);

        mortonCode |= temp0 * 2 + temp1;

        if (swap > 0)
        {
            mortonCode <<= 1;
            numBits[0] -= 1;
            uint64_t temp = axisCode[0] & (1ULL << numBits[0]);
            temp >>= numBits[0];
            mortonCode |= temp;
        }

        mortonCode <<= numBits[0] + numBits[1] + numBits[2];

        axisCode[0] &= ((1ULL << numBits[0]) - 1);
        axisCode[1] &= ((1ULL << numBits[1]) - 1);

        if (swap > 0)
        {
            uint64_t temp = axisCode[0];
            axisCode[0] = axisCode[1];
            axisCode[1] = temp;

            uint temp2 = numBits[0];
            numBits[0] = numBits[1];
            numBits[1] = temp2;
        }
    }

    // 2D case, just use xy xy xy...
    if (numBits[2] == 0)
    {
#if !__cplusplus
        [unroll]
#endif
        for (int r = 0; r < 2; r++)
        {
            axisCode[r] = ExpandBits2D64(axisCode[r]);
        }

        uint delta = numBits[0] - numBits[1];

        mortonCode |= (axisCode[0] << (1 - delta)) + (axisCode[1] << delta);
    }
    else // 3D case, just use if swap == 0 xyz xyz xyz..., if swap == 1 yxz yxz yxz...
    {
#if !__cplusplus
        [unroll]
#endif
        for (int i = 0; i < 3; i++)
        {
            axisCode[i] = (axisCode[i] > 0) ? ExpandBits64(axisCode[i]) : 0;
        }

        uint delta = numBits[0] - numBits[1];
        uint delta2 = numBits[0] - numBits[2];

        mortonCode |= (((axisCode[0] << (1 - delta)) + (axisCode[1] << (2 * delta))) << (1 - delta2)) +
                      (axisCode[2] << ((1 + (1 - delta)) * delta2));
    }

    return mortonCode;
}

#endif
