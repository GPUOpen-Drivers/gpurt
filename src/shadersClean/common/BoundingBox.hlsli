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

#ifndef BOUNDING_BOX_HLSLI
#define BOUNDING_BOX_HLSLI

//=====================================================================================================================
struct BoundingBox // matches D3D12_RAYTRACING_AABB
{
    float3 min;
    float3 max;
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
    BoundingBox b1)
{
    BoundingBox bbox;
    bbox.min = min(b0.min, b1.min);
    bbox.max = max(b0.max, b1.max);
    return bbox;
}

#endif
