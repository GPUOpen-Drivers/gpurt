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
#ifndef MATH_HLSLI
#define MATH_HLSLI

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

#ifndef LIBRARY_COMPILATION
#include "Math.hlsl"
#endif

#endif
