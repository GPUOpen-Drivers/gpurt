/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
// This header file contains definitions for common builtin HLSL types to allow compiling shared HLSL code in C++
//
#ifndef _HLSL_TYPES_H
#define _HLSL_TYPES_H

// Added as cpp doesn't allow for the `StructType s = (StructType)0;` pattern
#ifdef __cplusplus
#define INIT_VAR(type, name) type name = {}
#else
#define INIT_VAR(type, name) type name = (type)0
#endif

#ifdef __cplusplus

#include <cstdint>

using uint = uint32_t;

struct uint2
{
    uint2() = default;
    uint2(uint val0, uint val1) : x(val0), y(val1) {}

    uint x;
    uint y;
};
struct uint3
{
    uint3() = default;
    uint3(uint val0, uint val1, uint val2) : x(val0), y(val1), z(val2) {}

    uint x;
    uint y;
    uint z;
};
struct uint4
{
    uint4() = default;
    uint4(uint val0, uint val1, uint val2, uint val3) : x(val0), y(val1), z(val2), w(val3) {}

    uint x;
    uint y;
    uint z;
    uint w;
};

struct float2
{
    float2() = default;
    float2(float val0, float val1) : x(val0), y(val1) {}

    float x;
    float y;
};
struct float3
{
    float3() = default;
    float3(float val0, float val1, float val2) : x(val0), y(val1), z(val2) {}

    float x;
    float y;
    float z;
};
struct float4
{
    float4() = default;
    float4(float val0, float val1, float val2, float val3) : x(val0), y(val1), z(val2), w(val3) {}

    float x;
    float y;
    float z;
    float w;
};

#endif
#endif
