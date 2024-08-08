/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef EXTENSIONS_HLSLI
#define EXTENSIONS_HLSLI

#if !defined(__cplusplus)

#define __decl [noinline]

#endif

#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_TiesToEven     0x0
#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_TowardPositive 0x1
#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_TowardNegative 0x2
#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_TowardZero     0x3

#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Add      0x0
#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Subtract 0x1
#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Multiply 0x2

//=====================================================================================================================
static float FloatOpWithRoundMode(uint roundMode, uint operation, float src0, float src1);

//=====================================================================================================================
static float2 FloatOpWithRoundMode(uint roundMode, uint operation, float2 src0, float2 src1);

//=====================================================================================================================
static float3 FloatOpWithRoundMode(uint roundMode, uint operation, float3 src0, float3 src1);

#ifndef LIBRARY_COMPILATION
#include "Extensions.hlsl"
#endif

#endif
