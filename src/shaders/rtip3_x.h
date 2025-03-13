/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2020-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef RTIP3_X_H
#define RTIP3_X_H

//=====================================================================================================================
// RTIP3.x dual intersection result data structure
struct DualIntersectResult
{
    uint4 first;
    uint4 second;
    uint2 geometryId;
};

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
// RTIP3.1 bvh8 intersection result data structure
struct Bvh8IntersectResult
{
#ifdef __cplusplus
    // parameterised constructor for HLSL compatibility
    Bvh8IntersectResult(int val)
    {
        memset(this, val, sizeof(Bvh8IntersectResult));
    }
#endif

    uint4 slot0;
    uint4 slot1;
    uint2 ext;

    uint InstanceHitContribution()
    {
        // only the low 24-bits of this data are valid.
        return slot1.z;
    }

    uint64_t InstanceChildBasePtr()
    {
        return PackUint64(slot0.z, slot0.w);
    }

    float T(bool tri0)
    {
        return asfloat(tri0 ? slot0.x : slot1.x);
    }

    float BaryI(bool tri0)
    {
        return abs(asfloat(tri0 ? slot0.y : slot1.y));
    }

    bool IsProcedural(bool tri0)
    {
        return asint((tri0 ? slot0.y : slot1.y)) < 0;
    }

    float BaryJ(bool tri0)
    {
        return abs(asfloat(tri0 ? slot0.z : slot1.z));
    }

    bool IsNonOpaque(bool tri0)
    {
        return asint((tri0 ? slot0.z : slot1.z)) < 0;
    }

    uint PrimitiveIndexAndHitKind(bool tri0)
    {
        return tri0 ? slot0.w : slot1.w;
    }

    uint PrimitiveIndex(bool tri0)
    {
        return ((tri0 ? slot0.w : slot1.w) >> 1u);
    }

    uint HitKind(bool tri0)
    {
        // The decoded hit kind value 1 corresponds to back-face and 0 corresponds to front-face.
        return ((tri0 ? slot0.w : slot1.w) & 0x1) | HIT_KIND_TRIANGLE_FRONT_FACE;
    }

    uint FrontFace(bool tri0)
    {
        // The decoded hit kind value 1 corresponds to back-face and 0 corresponds to front-face.
        return ((tri0 ? slot0.w : slot1.w) & 0x1) == 0;
    }

    uint GeometryIndexAndNavigationBits(bool tri0)
    {
        return tri0 ? ext.x : ext.y;
    }

    uint GeometryIndex(bool tri0)
    {
        return (tri0 ? ext.x : ext.y) >> 2;
    }

    uint NavigationBits(bool tri0)
    {
        return (tri0 ? ext.x : ext.y) & 0x3;
    }
};

#endif

//=====================================================================================================================
struct IntersectionState
{
    float  t;
    float2 barycentrics;
    uint   hitKind;
    uint   geometryIndex;
    uint   primitiveIndex;
    uint   instanceContribution;
};

//=====================================================================================================================
// RTIP3.x common instructions
__decl DualIntersectResult AmdTraceRayIntersectRayBvh8(
    in uint64_t         baseNodePtr,
    inout_param(float3) rayOrigin,
    inout_param(float3) rayDir,
    in float            rayExtent,
    in uint             instanceMask,
    in uint             boxSortHeuristic,
    in uint             node) DUMMY_WIDE_INTERSECT_FUNC;

//=====================================================================================================================
__decl DualIntersectResult AmdTraceRayDualIntersectRay(
    in uint64_t         baseNodePtr,
    inout_param(float3) rayOrigin,
    inout_param(float3) rayDir,
    in float            rayExtent,
    in uint             instanceMask,
    in uint             boxSortHeuristic,
    in uint             node0,
    in uint             node1) DUMMY_WIDE_INTERSECT_FUNC;

//=====================================================================================================================
__decl uint AmdTraceRayDsStackPush8Pop1(
    inout_param(uint) packedStackAddr,
    in uint           lastNodePtr,
    in uint4          data0,
    in uint4          data1) DUMMY_UINT_FUNC;

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
__decl uint AmdTraceRayDsStackPush8Pop1PrimRangeEnabled(
    inout_param(uint) packedStackAddr,
    in uint           lastNodePtr,
    in uint4          data0,
    in uint4          data1) DUMMY_UINT_FUNC;
#endif

//=====================================================================================================================
__decl uint2 AmdTraceRayDsStackPush8Pop2(
    inout_param(uint) packedStackAddr,
    in uint           lastNodePtr,
    in uint4          data0,
    in uint4          data1) DUMMY_UINT2_FUNC;

// Bit 31 of the stack address indicates a BLAS to TLAS transition
#define STACK_ADDR_BLAS_TO_TLAS_MASK 0x80000000

#endif
