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

#include "BoundingBox.hlsli"
#include "Common.hlsli"

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

#if GPURT_BVH_BUILD_SHADER
#include "../build/BuildSettings.hlsli"
//=====================================================================================================================
// Calculate an AABB from the 3 points of the triangle.
static BoundingBox GenerateTriangleBoundingBox(float3 v0, float3 v1, float3 v2)
{
    BoundingBox bbox =
    {
        min(min(v0, v1), v2),
        max(max(v0, v1), v2),
    };

    if (Settings.disableDegenPrims == 0)
    {
        // Generate degenerate bounding box for triangles that degenerate into line or point
        if (all(v0 == v1) || all(v0 == v2) || all(v1 == v2))
        {
            bbox = InvalidBoundingBox;
        }
    }

    return bbox;
};
#endif

//=====================================================================================================================
// Compresses a single bounding box into a uint3 representing each bounding plane using half floats
static uint3 CompressBBoxToUint3(in BoundingBox box)
{
    // Note: Converted bounds must be conservative and, assuming infinite precision,
    // resulting 16-bit bound representation must contain original 32-bit bounds
    // Thus: round down for min and round up for max

    // HLSL spec forces rounding modes towards 0, rather than -inf or +inf
    // So use AMDIL extension to call appropriate rounding function

    // For prototype/testing, can help to force default HLSL conversion mode
#if FORCE_FP16_CONV_MODE_TO_DEF
    const uint3 boxMin = f32tof16(box.min);
    const uint3 boxMax = f32tof16(box.max);
#else
    const uint3 boxMin = AmdExtD3DShaderIntrinsics_ConvertF32toF16NegInf(box.min);
    const uint3 boxMax = AmdExtD3DShaderIntrinsics_ConvertF32toF16PosInf(box.max);
#endif

    // Interleave into the following format:
    //         low,   high
    // word 0: min.x, min.y
    // word 1: min.z, max.x
    // word 2: max.y, max.z
    const uint3 loRet = { boxMin.x, boxMin.z, boxMax.y };
    const uint3 hiRet = { boxMin.y, boxMax.x, boxMax.z };

    // Combine component-wise to return
    // f32tof16 documentation specifies (loRet & 0xFFFF) is unnecessary
    return ((hiRet << 16) | loRet);
}

//=====================================================================================================================
// Uncompresses a single bounding box from uint3
// Intended for software path only
static BoundingBox UncompressBBoxFromUint3(in uint3 box)
{
    // See notes above about rounding modes

    // Swizzle into individual min,max bounds
    const uint3  loU = (box & 0x0000FFFF);
    const uint3  hiU = (box >> 16);
    const uint3 minU = { loU.x, hiU.x, loU.y };
    const uint3 maxU = { hiU.y, loU.z, hiU.z };

    // We should convert fp16->fp32 conservatively (which would likely expand the bbox)
    // but seems like op codes for f16->f32 conversion with +-inf rounding are not exposed and
    // this path is for SW only, so use default
    BoundingBox toRet;
    toRet.min = f16tof32(minU);
    toRet.max = f16tof32(maxU);

    return toRet;
}

//=====================================================================================================================
static bool IsInvalidBoundingBox(BoundingBox box)
{
    return box.min.x > box.max.x;
}

//=====================================================================================================================
static bool IsCorruptBox(BoundingBox box)
{
    if (any(isinf(box.min)) || any(isinf(box.max)) || any(isnan(box.min)) || any(isnan(box.max)))
    {
        return true;
    }

    return false;
}

//=====================================================================================================================
static float ComputeBoxSurfaceArea(BoundingBox aabb)
{
    // check for degenerate
    float3 dim = aabb.max - aabb.min;
    return IsInvalidBoundingBox(aabb) ? 0 : 2.0f * (dim.x * dim.y + dim.x * dim.z + dim.y * dim.z);
}

//=====================================================================================================================
static float ComputeBoxSurfaceArea(const uint3 aabb)
{
    const BoundingBox aabbFp32 = UncompressBBoxFromUint3(aabb);
    return ComputeBoxSurfaceArea(aabbFp32);
}
