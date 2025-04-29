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
#ifndef MATH_HLSLI
#define MATH_HLSLI

//=====================================================================================================================
// Divide uints and round up
static uint RoundUpQuotient(
    uint dividend,
    uint divisor);

//=====================================================================================================================
// Divide ints and round up
static int RoundUpQuotient(
    int dividend,
    int divisor);

//=====================================================================================================================
// Divide ints and round up
static uint64_t RoundUpQuotient(
    uint64_t dividend,
    uint64_t divisor);

static float Determinant(in float3x4 transform);

static float3x4 Inverse3x4(in float3x4 transform);

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
// Search the mask data from least significant bit (LSB) / Lowest Order bit
// to the most significant bit (MSB) for a set bit (1).
//  firstbitlow   <--> _BitScanForward   LSB -> MSB  (lowBit -> HighBit)
static uint32_t ScanForward(uint32_t value);

//=====================================================================================================================
// Search the mask data from most significant bit (MSB) / Highest Order bit
// to least significant bit (LSB) for a set bit (1)
//  firstbithigh  <--> _BitScanReverse   MSB -> LSB  (HighBit -> LowBit)
static uint32_t ScanReverse(uint32_t value);

//=====================================================================================================================
static uint32_t ScanReverse64(uint64_t value);

//=====================================================================================================================
static uint32_t CommonTrailingZeroBits(uint32_t unions);

//=====================================================================================================================
static uint32_t CommonPrefixBit(uint32_t diff);

//=====================================================================================================================
static uint3 CommonPrefixBits(uint3 diffs);

//=====================================================================================================================
static uint32_t LeadingZeroBits(uint32_t u);

//=====================================================================================================================
static uint32_t LeadingZeroBits64(uint64_t u);

//=====================================================================================================================
// Compute common exponent
static uint3 ComputeCommonExponent(
    float3 minOfMins,
    float3 maxOfMaxs,
    uint   numQuantBits);

//=====================================================================================================================
// Compute 8-bit integer reciprocal.
static float3 ComputeFastExpReciprocal(
    uint3 exponents,
    uint  numQuantBits);

//=====================================================================================================================
// Compute N-bit quantized max
static uint3 ComputeQuantizedMax(
    float3 maxValue,
    float3 origin,
    float3 rcpExponents,
    uint   numQuantBits);

//=====================================================================================================================
// Compute N-bit quantized min
static uint3 ComputeQuantizedMin(
    float3 minValue,
    float3 origin,
    float3 rcpExponents,
    uint   numQuantBits);

//=====================================================================================================================
// Decode a quantized integer into float
static float Dequantize(
    float origin,
    uint  exponent,
    uint  plane,
    uint  numQuantBits);

#endif

#ifndef LIBRARY_COMPILATION
#include "Math.hlsl"
#endif

#endif
