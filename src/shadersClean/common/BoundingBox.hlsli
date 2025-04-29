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

#ifndef BOUNDING_BOX_HLSLI
#define BOUNDING_BOX_HLSLI

//=====================================================================================================================
struct BoundingBox // matches D3D12_RAYTRACING_AABB
{
    float3 min;
    float3 max;
};

//=====================================================================================================================
// FLT_MAX defined here again due to include issues with common.hlsli
#ifndef FLT_MAX
#define FLT_MAX 3.402823466e+38F
#endif

static const BoundingBox InvalidBoundingBox =
{
    float3(FLT_MAX, FLT_MAX, FLT_MAX),
    float3(-FLT_MAX, -FLT_MAX, -FLT_MAX)
};

//=====================================================================================================================
struct MortonBoundingBox
{
    float3 min;
    float3 extent;
    float2 sizeMinMax;
    uint numSizeBits;
};

//=====================================================================================================================
struct BoundingBox4
{
    float4 min;
    float4 max;
};

//=====================================================================================================================
// Internal bounding box type for scene bounds.
struct UintBoundingBox
{
    uint3 min;
    uint3 max;
};

struct UintBoundingBox4
{
    uint4 min;
    uint4 max;
};

struct PackedUintBoundingBox4
{
    uint64_t min;
    uint64_t max;
};

//=====================================================================================================================
static BoundingBox CombineAABB(
    BoundingBox b0,
    BoundingBox b1);

//=====================================================================================================================
// Calculate an AABB from the 3 points of the triangle.
static BoundingBox GenerateTriangleBoundingBox(float3 v0, float3 v1, float3 v2);

//=====================================================================================================================
// Compresses a single bounding box into a uint3 representing each bounding plane using half floats
static uint3 CompressBBoxToUint3(in BoundingBox box);

//=====================================================================================================================
// Uncompresses a single bounding box from uint3
// Intended for software path only
static BoundingBox UncompressBBoxFromUint3(in uint3 box);

//=====================================================================================================================
static bool IsInvalidBoundingBox(BoundingBox box);

//=====================================================================================================================
static bool IsCorruptBox(BoundingBox box);

//=====================================================================================================================
static float ComputeBoxSurfaceArea(BoundingBox aabb);

//=====================================================================================================================
static float ComputeBoxSurfaceArea(const uint3 aabb);

#ifndef LIBRARY_COMPILATION
#include "BoundingBox.hlsl"
#endif

#endif
