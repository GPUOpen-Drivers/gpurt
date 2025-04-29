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
#include "Math.hlsli"

#include "Bits.hlsli"
#include "Extensions.hlsli"

//=====================================================================================================================
// Divide uints and round up
static uint RoundUpQuotient(
    uint dividend,
    uint divisor)
{
    return (dividend + divisor - 1) / divisor;
}

//=====================================================================================================================
// Divide ints and round up
static int RoundUpQuotient(
    int dividend,
    int divisor)
{
    return (dividend + divisor - 1) / divisor;
}

//=====================================================================================================================
// Divide ints and round up
static uint64_t RoundUpQuotient(
    uint64_t dividend,
    uint64_t divisor)
{
    return (dividend + divisor - 1) / divisor;
}

//=====================================================================================================================
static uint CeilLog2(uint value)
{
    return firstbithigh(value) == firstbitlow(value) ?
           firstbithigh(value) : firstbithigh(value) + 1;
}

//=====================================================================================================================
//https://github.com/Microsoft/DirectX-Graphics-Samples/blob/master/Libraries/D3D12RaytracingFallback/src/RayTracingHelper.hlsli
// The MIT License (MIT)
//
// Copyright(c) 2015 Microsoft
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
static float Determinant(in float3x4 transform)
{
    return transform[0][0] * transform[1][1] * transform[2][2] -
        transform[0][0] * transform[2][1] * transform[1][2] -
        transform[1][0] * transform[0][1] * transform[2][2] +
        transform[1][0] * transform[2][1] * transform[0][2] +
        transform[2][0] * transform[0][1] * transform[1][2] -
        transform[2][0] * transform[1][1] * transform[0][2];
}

//=====================================================================================================================
//https://github.com/Microsoft/DirectX-Graphics-Samples/blob/master/Libraries/D3D12RaytracingFallback/src/RayTracingHelper.hlsli
// The MIT License (MIT)
//
// Copyright(c) 2015 Microsoft
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
static float3x4 Inverse3x4(in float3x4 transform)
{
    const float invDet = rcp(Determinant(transform));

    float3x4 invertedTransform;
    invertedTransform[0][0] = invDet * (transform[1][1] * (transform[2][2] * 1.0f - 0.0f * transform[2][3]) + transform[2][1] * (0.0f * transform[1][3] - transform[1][2] * 1.0f) + 0.0f * (transform[1][2] * transform[2][3] - transform[2][2] * transform[1][3]));
    invertedTransform[1][0] = invDet * (transform[1][2] * (transform[2][0] * 1.0f - 0.0f * transform[2][3]) + transform[2][2] * (0.0f * transform[1][3] - transform[1][0] * 1.0f) + 0.0f * (transform[1][0] * transform[2][3] - transform[2][0] * transform[1][3]));
    invertedTransform[2][0] = invDet * (transform[1][3] * (transform[2][0] * 0.0f - 0.0f * transform[2][1]) + transform[2][3] * (0.0f * transform[1][1] - transform[1][0] * 0.0f) + 1.0f * (transform[1][0] * transform[2][1] - transform[2][0] * transform[1][1]));
    invertedTransform[0][1] = invDet * (transform[2][1] * (transform[0][2] * 1.0f - 0.0f * transform[0][3]) + 0.0f * (transform[2][2] * transform[0][3] - transform[0][2] * transform[2][3]) + transform[0][1] * (0.0f * transform[2][3] - transform[2][2] * 1.0f));
    invertedTransform[1][1] = invDet * (transform[2][2] * (transform[0][0] * 1.0f - 0.0f * transform[0][3]) + 0.0f * (transform[2][0] * transform[0][3] - transform[0][0] * transform[2][3]) + transform[0][2] * (0.0f * transform[2][3] - transform[2][0] * 1.0f));
    invertedTransform[2][1] = invDet * (transform[2][3] * (transform[0][0] * 0.0f - 0.0f * transform[0][1]) + 1.0f * (transform[2][0] * transform[0][1] - transform[0][0] * transform[2][1]) + transform[0][3] * (0.0f * transform[2][1] - transform[2][0] * 0.0f));
    invertedTransform[0][2] = invDet * (0.0f * (transform[0][2] * transform[1][3] - transform[1][2] * transform[0][3]) + transform[0][1] * (transform[1][2] * 1.0f - 0.0f * transform[1][3]) + transform[1][1] * (0.0f * transform[0][3] - transform[0][2] * 1.0f));
    invertedTransform[1][2] = invDet * (0.0f * (transform[0][0] * transform[1][3] - transform[1][0] * transform[0][3]) + transform[0][2] * (transform[1][0] * 1.0f - 0.0f * transform[1][3]) + transform[1][2] * (0.0f * transform[0][3] - transform[0][0] * 1.0f));
    invertedTransform[2][2] = invDet * (1.0f * (transform[0][0] * transform[1][1] - transform[1][0] * transform[0][1]) + transform[0][3] * (transform[1][0] * 0.0f - 0.0f * transform[1][1]) + transform[1][3] * (0.0f * transform[0][1] - transform[0][0] * 0.0f));
    invertedTransform[0][3] = invDet * (transform[0][1] * (transform[2][2] * transform[1][3] - transform[1][2] * transform[2][3]) + transform[1][1] * (transform[0][2] * transform[2][3] - transform[2][2] * transform[0][3]) + transform[2][1] * (transform[1][2] * transform[0][3] - transform[0][2] * transform[1][3]));
    invertedTransform[1][3] = invDet * (transform[0][2] * (transform[2][0] * transform[1][3] - transform[1][0] * transform[2][3]) + transform[1][2] * (transform[0][0] * transform[2][3] - transform[2][0] * transform[0][3]) + transform[2][2] * (transform[1][0] * transform[0][3] - transform[0][0] * transform[1][3]));
    invertedTransform[2][3] = invDet * (transform[0][3] * (transform[2][0] * transform[1][1] - transform[1][0] * transform[2][1]) + transform[1][3] * (transform[0][0] * transform[2][1] - transform[2][0] * transform[0][1]) + transform[2][3] * (transform[1][0] * transform[0][1] - transform[0][0] * transform[1][1]));

    return invertedTransform;
}

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
// Search the mask data from least significant bit (LSB) / Lowest Order bit
// to the most significant bit (MSB) for a set bit (1).
//  firstbitlow   <--> _BitScanForward   LSB -> MSB  (lowBit -> HighBit)
static uint32_t ScanForward(uint32_t value)
{
    // Gets the location of the first set bit starting from the highest order bit and working downward, per component.
    return firstbitlow(value);
}

//=====================================================================================================================
// Search the mask data from most significant bit (MSB) / Highest Order bit
// to least significant bit (LSB) for a set bit (1)
//  firstbithigh  <--> _BitScanReverse   MSB -> LSB  (HighBit -> LowBit)
static uint32_t ScanReverse(uint32_t value)
{
    // This function is different than "clz"
    // if the input value is 0, this function needs to return 32 instaed.
    uint32_t ret = (value > 0) ? (31u - firstbithigh(value)) : 32u;

    return ret;
}

//=====================================================================================================================
static uint32_t ScanReverse64(uint64_t value)
{
    // This function is different than "clz"
    // if the input value is 0, this function needs to return 64 instaed.
    uint32_t ret = (value > 0) ? (63u - firstbithigh(value)) : 64u;

    return ret;
}

//=====================================================================================================================
static uint32_t CommonTrailingZeroBits(uint32_t unions)
{
    uint32_t scanSuffix = ScanForward(unions);
    return scanSuffix;
}

//=====================================================================================================================
static uint32_t CommonPrefixBit(uint32_t diff)
{
    uint32_t prefixLength = ScanReverse(diff);
    return prefixLength == 32 ? 31u : prefixLength;
}

//=====================================================================================================================
static uint3 CommonPrefixBits(uint3 diffs)
{
    uint32_t xPrefixLength = CommonPrefixBit(diffs.x);
    uint32_t yPrefixLength = CommonPrefixBit(diffs.y);
    uint32_t zPrefixLength = CommonPrefixBit(diffs.z);

    uint3 retData = uint3(xPrefixLength, yPrefixLength, zPrefixLength);
    return retData;
}

//=====================================================================================================================
static uint32_t LeadingZeroBits(uint32_t u)
{
    return ScanReverse(u);
}

//=====================================================================================================================
static uint32_t LeadingZeroBits64(uint64_t u)
{
    return ScanReverse64(u);
}

//=====================================================================================================================
// Compute common exponent
static uint3 ComputeCommonExponent(
    float3 minOfMins,
    float3 maxOfMaxs,
    uint   numQuantBits)
{
    // Rounding mode here is necessary for cases where floating point precision
    // loses the MSB of min in the subtract. For example, 33554432 - (-0.1) = 33554432.
    // The correct exponent for this operation should be log2(2 * 33554432) not log2(33554432)
    float3 extent = FloatOpWithRoundMode(AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_TowardPositive,
                                         AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Subtract,
                                         maxOfMaxs,
                                         minOfMins);

    uint3 exponents = (((asuint(extent) + 0x7FFFFF) >> 23) & 0xFF);

    // Due to a HW-savings measure in the dequantize function, boxes using very small exponents (<= 12)
    // cannot make effective use of all plane values. This is because the dequantization occurs
    // separately from the origin so it's possible to generate denormal numbers.
    // For example, if numQuantBits=12, exponent is 2 and the encoded value is 1 then decoding would produce:
    // (1 / 4096) * 2 ** (2 - 127) = 2 ** (-12) * 2 ** (-127) = 2 ** (-139) = 2 ** (-12 - 127)
    // negative exponents result in denormal numbers in ieee float32

    // | Exponent   | Valid Plane Encodings |
    // | 0          | 0                     |
    // | 1          | 1                     |
    // | 2          | 2 (0, 2048)           |
    // | 3          | 4 (0,1024,2048,3192)  |
    // | 4          | 8                     |
    // ...
    // | 12         | 2048                  |
    // | 13         | 4096                  |

    // Given this restriction, it is equivalent to use exponent 13 instead of most precise exponent when exponent <= 12.
    // For example, if the exponent is 3 and a plane is encoded with 1024 it would decode to
    // (1024 / 4096) * 2 ** (3 - 127) = (1/4) * 2 ** (-124) = [2 ** (-2)] * [2 ** (-124)] = 2 ** -126
    // Now the same using an exponent value of 13
    // (1 / 4096) * 2 ** (13 - 127) = 2 ** (-12) * 2 ** (-114) = 2 ** -126

    // Forcing the minimum exponent saves multiplications in the quantizaiton loop
    // with no loss of precision (beyond the existing hardware limitation).
    // 0 is a special case of infinitly-thin geometry that is slightly more precise
    // than using minimum exponent.
    const uint minExponent = numQuantBits + 1;
    exponents.x = exponents.x == 0 ? 0 : max(minExponent, exponents.x);
    exponents.y = exponents.y == 0 ? 0 : max(minExponent, exponents.y);
    exponents.z = exponents.z == 0 ? 0 : max(minExponent, exponents.z);

    return exponents;
}

//=====================================================================================================================
// Compute 8-bit integer reciprocal.
static float3 ComputeFastExpReciprocal(
    uint3 exponents,
    uint  numQuantBits)
{
    // Computing rcpExponents guarentees that the compiler will not emit
    // transcendental ops for the plane quantization.
    // When numQuantBits=12 for example, the + 12 comes from the fact that 2 ** 12 = 4096 which is the
    // encoding granularity.
    const float3 rcpExponents = asfloat((uint3(254, 254, 254) - exponents + numQuantBits) << 23);

    // Note that this optimization results in this function being ill-defined for inputs with exponent == 254.
    // It is unlikely any app will use geometry with enourmous bounds like this. This could be handled with a special
    // case or by falling back to infinitely large boxes and accepting all hits and pushing the problem down the BVH.
    return rcpExponents;
}

//=====================================================================================================================
// Compute N-bit quantized max
static uint3 ComputeQuantizedMax(
    float3 maxValue,
    float3 origin,
    float3 rcpExponents,
    uint   numQuantBits)
{
    float3 diff = FloatOpWithRoundMode(AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_TowardPositive,
                                       AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Subtract,
                                       maxValue,
                                       origin);

    float3 fquantMax = FloatOpWithRoundMode(AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_TowardPositive,
                                            AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Multiply,
                                            diff,
                                            rcpExponents);

    fquantMax = ceil(fquantMax);

    const float maxFloat = (1u << numQuantBits) * 1.0f;

    // Map the max to a value in [1,2^n].
    // Clamp is necessary to map 2^n + 1 back to a valid number (possible when using upward rounding mode.)
    const uint3 quantMax = clamp(fquantMax, float3(1., 1., 1.), float3(maxFloat, maxFloat, maxFloat));

    // Subtract 1 to map value to a N-bit number
    return quantMax - uint3(1, 1, 1);
}

//=====================================================================================================================
// Compute N-bit quantized min
static uint3 ComputeQuantizedMin(
    float3 minValue,
    float3 origin,
    float3 rcpExponents,
    uint   numQuantBits)
{
    float3 diff = FloatOpWithRoundMode(AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_TowardNegative,
                                       AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Subtract,
                                       minValue,
                                       origin);

    float3 fquantMin = FloatOpWithRoundMode(AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_TowardNegative,
                                            AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Multiply,
                                            diff,
                                            rcpExponents);

    fquantMin = floor(fquantMin);

    const float maxFloat = ((1u << numQuantBits) - 1) * 1.0f;

    const uint3 quantMin = clamp(fquantMin, float3(0., 0., 0.), float3(maxFloat, maxFloat, maxFloat));
    return quantMin;
}

#endif

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
static float Dequantize(
    float origin,
    uint  exponent,
    uint  plane,
    uint  numQuantBits)
{
    uint result = 0;

    if (plane)
    {
        int signedPos = firstbithigh(plane) - numQuantBits; // 31 - numQuantBits - count_leading_zeros(plane)
        int signedExponent = signedPos + exponent;

        if (signedExponent > 0)
        {
            result = (plane << (23 - numQuantBits - signedPos)) & bits(23);  // set mantissa
            result |= (signedExponent << 23) & bits(31);      // set exponent
        }
    }

    return asfloat(result) + origin;
}

#endif

