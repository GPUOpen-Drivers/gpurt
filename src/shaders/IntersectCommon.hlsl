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
#ifndef _INTERSECTCOMMON_HLSL
#define _INTERSECTCOMMON_HLSL

#include "Common.hlsl"

#define SORT(childA,childB,distA,distB) if((childB!=INVALID_NODE&&distB<distA)||childA==INVALID_NODE){  float t0 = distA; uint t1 = childA;  childA = childB; distA = distB;  childB=t1; distB=t0; }

#define INTERSECT_RAY_VERSION_1 1

#if GPURT_BUILD_RTIP2
#define INTERSECT_RAY_VERSION_2 2
#endif

//=====================================================================================================================
// Avoid tracing NaN rays or null acceleration structures.
// - Null acceleration structure bindings should result in a miss.
// - Empty top levels should result in a miss. The address in the struct header is set to 0 for this case.
// - NaNs are illegal, but we check here for robustness to avoid traversal hangs in bad apps (should only affect
//   the software intersection path).
static bool IsValidTrace(
    RayDesc           ray,
    GpuVirtualAddress accelStruct,
    uint              instanceInclusionMask,
    uint              rayFlags,
    uint              pipelineFlags)
{
    bool valid = true;
    const bool skipProceduralFlag = (pipelineFlags & PIPELINE_FLAG_SKIP_PROCEDURAL_PRIMITIVES) ? true : false;
    const bool raySkipProcedural = (rayFlags & RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES) || (skipProceduralFlag == true);

    if ((accelStruct == 0) || (instanceInclusionMask == 0))
    {
        valid = false;
    }
#if USE_HW_INTRINSIC == 0
    if (any(isnan(ray.Origin)) || any(isnan(ray.Direction)))
    {
        valid = false;
    }
#endif
    if ((ray.TMin < 0) || (ray.TMin > ray.TMax) || isinf(ray.TMin))
    {
        valid = false;
    }
    if ((ray.TMin == ray.TMax) && (raySkipProcedural == true))
    {
        valid = false;
    }

    return valid;
}

//=====================================================================================================================
static bool IsBoxNode1_1(
    uint nodePtr)
{
    return IsBoxNode(nodePtr
                    );
}

//=====================================================================================================================
static uint CreateRootNodePointer1_1()
{
    return CreateRootNodePointer(
        );
}

//=====================================================================================================================
static GpuVirtualAddress FetchAccelStructBaseAddr(GpuVirtualAddress bvhAddress)
{
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION  >= 34
    if (bvhAddress)
    {
        // Fetch acceleration structure base address. Note that an empty BVH will have 0 as the address in the metadata.
        bvhAddress = MakeGpuVirtualAddress(LoadDwordAtAddr(bvhAddress + ACCEL_STRUCT_METADATA_VA_LO_OFFSET),
                                           LoadDwordAtAddr(bvhAddress + ACCEL_STRUCT_METADATA_VA_HI_OFFSET));
    }
#endif
    return bvhAddress;
}

//=====================================================================================================================
static GpuVirtualAddress FetchAccelStructBaseAddr(in uint lowBits, in uint highBits)
{
    return FetchAccelStructBaseAddr(MakeGpuVirtualAddress(lowBits, highBits));
}

//=====================================================================================================================
static InstanceDesc FetchInstanceDescAddr(in GpuVirtualAddress instanceAddr)
{
    uint4 d0, d1, d2, d3;

    d0 = LoadDwordAtAddrx4(instanceAddr);
    d1 = LoadDwordAtAddrx4(instanceAddr + 0x10);
    d2 = LoadDwordAtAddrx4(instanceAddr + 0x20);
    d3 = LoadDwordAtAddrx4(instanceAddr + 0x30);

    InstanceDesc desc;

    desc.Transform[0] = asfloat(d0);
    desc.Transform[1] = asfloat(d1);
    desc.Transform[2] = asfloat(d2);

    desc.InstanceID_and_Mask                           = d3.x;
    desc.InstanceContributionToHitGroupIndex_and_Flags = d3.y;
    desc.accelStructureAddressLo                       = d3.z;
    desc.accelStructureAddressHiAndFlags               = d3.w;

    return desc;
}

//=====================================================================================================================
static InstanceDesc FetchInstanceDesc(in GpuVirtualAddress bvhAddress, uint nodePointer)
{
    const GpuVirtualAddress instanceAddr = bvhAddress + ExtractNodePointerOffset(nodePointer);
    return FetchInstanceDescAddr(instanceAddr);
}

//=====================================================================================================================
static uint FetchTriangleId(in GpuVirtualAddress bvhAddress, in uint nodePointer)
{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    const GpuVirtualAddress nodeAddr = bvhAddress + byteOffset;

    return LoadDwordAtAddr(nodeAddr + TRIANGLE_NODE_ID_OFFSET);
}

//=====================================================================================================================
static bool CheckInstanceCulling(in InstanceDesc desc, const uint rayFlags, const uint pipelineRayFlags)
{
    // If the instance base pointer 'Skip Triangles' bit is 1, then the geometry is procedural.
    const uint geomType = (desc.accelStructureAddressHiAndFlags >> (NODE_POINTER_SKIP_TRIANGLES_SHIFT - 32)) & 0x1;

    bool isTriangleInstanceCulled =
        ((geomType == GEOMETRY_TYPE_TRIANGLES) &&
           ((rayFlags         & RAY_FLAG_SKIP_TRIANGLES) ||
            (pipelineRayFlags & PIPELINE_FLAG_SKIP_TRIANGLES)));

    bool isProceduralInstanceCulled =
        ((geomType == GEOMETRY_TYPE_AABBS) &&
           ((rayFlags         & RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES) ||
            (pipelineRayFlags & PIPELINE_FLAG_SKIP_PROCEDURAL_PRIMITIVES)));

    return (isTriangleInstanceCulled || isProceduralInstanceCulled);
}

//=====================================================================================================================
static uint FetchInstanceNodePointerAddr(in GpuVirtualAddress nodeAddr)
{
    return LoadDwordAtAddr(nodeAddr + INSTANCE_NODE_EXTRA_OFFSET + RTIP1_1_INSTANCE_SIDEBAND_CHILD_POINTER_OFFSET);
}

//=====================================================================================================================
static uint64_t CalculateNodeAddr64(in GpuVirtualAddress bvhAddress, in uint nodePointer)
{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    return (bvhAddress + byteOffset);
}

//=====================================================================================================================
static TriangleData FetchTriangleFromNode(in GpuVirtualAddress bvhAddress, in uint nodePointer)
{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    const GpuVirtualAddress nodeAddr = bvhAddress + byteOffset;
    const uint  nodeType = GetNodeType(nodePointer);
    const uint3 offsets = CalcTriangleVertexOffsets(nodeType);

    uint4 d0, d1, d2;

    d0 = LoadDwordAtAddrx4(nodeAddr + offsets.x);
    d1 = LoadDwordAtAddrx4(nodeAddr + offsets.y);
    d2 = LoadDwordAtAddrx4(nodeAddr + offsets.z);

    TriangleData tri;

    tri.v0 = asfloat(d0.xyz);
    tri.v1 = asfloat(d1.xyz);
    tri.v2 = asfloat(d2.xyz);

    return tri;
}

//=====================================================================================================================
static uint FetchFloat32BoxNodeNumPrimitives(in GpuVirtualAddress bvhAddress, in uint nodePointer)
{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    const GpuVirtualAddress nodeAddr = bvhAddress + byteOffset + FLOAT32_BOX_NODE_NUM_PRIM_OFFSET;

    return LoadDwordAtAddr(nodeAddr);
}

//=====================================================================================================================
static Float32BoxNode FetchFloat32BoxNode(in GpuVirtualAddress bvhAddress,
                                          in uint              nodePointer)

{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    const GpuVirtualAddress nodeAddr = bvhAddress + byteOffset;

    uint4 d0, d1, d2, d3, d4, d5, d6;
    d0 = LoadDwordAtAddrx4(nodeAddr);
    d1 = LoadDwordAtAddrx4(nodeAddr + 0x10);
    d2 = LoadDwordAtAddrx4(nodeAddr + 0x20);
    d3 = LoadDwordAtAddrx4(nodeAddr + 0x30);

    d4 = LoadDwordAtAddrx4(nodeAddr + 0x40);
    d5 = LoadDwordAtAddrx4(nodeAddr + 0x50);
    d6 = LoadDwordAtAddrx4(nodeAddr + 0x60);

    Float32BoxNode node;

    node.child0 = d0.x;
    node.child1 = d0.y;
    node.child2 = d0.z;
    node.child3 = d0.w;

    node.bbox0_min = asfloat(d1.xyz);
    node.bbox0_max = float3(asfloat(d1.w), asfloat(d2.xy));
    node.bbox1_min = float3(asfloat(d2.zw), asfloat(d3.x));
    node.bbox1_max = asfloat(d3.yzw);
    node.bbox2_min = asfloat(d4.xyz);
    node.bbox2_max = float3(asfloat(d4.w), asfloat(d5.xy));
    node.bbox3_min = float3(asfloat(d5.zw), asfloat(d6.x));
    node.bbox3_max = asfloat(d6.yzw);

    node.flags = LoadDwordAtAddr(nodeAddr + FLOAT32_BOX_NODE_FLAGS_OFFSET);

    return node;
}

//=====================================================================================================================
static Float32BoxNode FetchFloat16BoxNodeAsFp32(in GpuVirtualAddress bvhAddress, in uint nodePointer)
{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    const GpuVirtualAddress nodeAddr = bvhAddress + byteOffset;

    uint4 d0, d1, d2, d3;
    d0 = LoadDwordAtAddrx4(nodeAddr);
    d1 = LoadDwordAtAddrx4(nodeAddr + 0x10);
    d2 = LoadDwordAtAddrx4(nodeAddr + 0x20);
    d3 = LoadDwordAtAddrx4(nodeAddr + 0x30);

    Float32BoxNode node;

    node.child0 = d0.x;
    node.child1 = d0.y;
    node.child2 = d0.z;
    node.child3 = d0.w;

    const BoundingBox b0 = UncompressBBoxFromUint3(d1.xyz);
    const BoundingBox b1 = UncompressBBoxFromUint3(uint3(d1.w,  d2.xy));
    const BoundingBox b2 = UncompressBBoxFromUint3(uint3(d2.zw, d3.x ));
    const BoundingBox b3 = UncompressBBoxFromUint3(d3.yzw);

    node.bbox0_min = b0.min;
    node.bbox0_max = b0.max;
    node.bbox1_min = b1.min;
    node.bbox1_max = b1.max;
    node.bbox2_min = b2.min;
    node.bbox2_max = b2.max;
    node.bbox3_min = b3.min;
    node.bbox3_max = b3.max;

    // fp16 node does not have space to store flags,
    // initialize the field to 0.
    node.flags = 0;

    return node;
}

//=====================================================================================================================
static Float16BoxNode FetchFloat16BoxNode(in GpuVirtualAddress bvhAddress, in uint nodePointer)
{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    const GpuVirtualAddress nodeAddr = bvhAddress + byteOffset;

    uint4 d0, d1, d2, d3;
    d0 = LoadDwordAtAddrx4(nodeAddr);
    d1 = LoadDwordAtAddrx4(nodeAddr + 0x10);
    d2 = LoadDwordAtAddrx4(nodeAddr + 0x20);
    d3 = LoadDwordAtAddrx4(nodeAddr + 0x30);

    Float16BoxNode node;

    node.child0 = d0.x;
    node.child1 = d0.y;
    node.child2 = d0.z;
    node.child3 = d0.w;

    node.bbox0 = d1.xyz;
    node.bbox1 = uint3(d1.w,  d2.xy);
    node.bbox2 = uint3(d2.zw, d3.x );
    node.bbox3 = d3.yzw;

    return node;
}

//=====================================================================================================================
static ProceduralNode FetchProceduralNode(in GpuVirtualAddress bvhAddress, in uint nodePointer)
{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    const GpuVirtualAddress nodeAddr = bvhAddress + byteOffset;

    uint4 d0, d1;

    d0 = LoadDwordAtAddrx4(nodeAddr);
    d1 = LoadDwordAtAddrx4(nodeAddr + 0x10);

    ProceduralNode node;

    node.bbox_min = asfloat(d0.xyz);
    node.bbox_max = float3(asfloat(d0.w), asfloat(d1.xy));

    return node;
}

//=====================================================================================================================
static bool IsOpaque(uint geometryFlags, uint instanceFlags, uint rayFlags)
{
    const bool isOpaqueGeometry    = (geometryFlags & D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE);
    const bool isOpaqueInstance    = (instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_FORCE_OPAQUE);
    const bool isNonOpaqueInstance = (instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_FORCE_NON_OPAQUE);
    bool isOpaque                  = isOpaqueInstance || (isOpaqueGeometry && (isNonOpaqueInstance == false));

    if (rayFlags & RAY_FLAG_FORCE_OPAQUE)
    {
        isOpaque = true;
    }
    else if (rayFlags & RAY_FLAG_FORCE_NON_OPAQUE)
    {
        isOpaque = false;
    }

    return isOpaque;
}

//=====================================================================================================================
static uint2 CalculateRawBvh64NodePointer(
    GpuVirtualAddress bvhAddress, uint nodePointer)
{
    // Node pointers are 64-byte aligned with the node type in the bottom 3 bits.
    GpuVirtualAddress nodeAddr = (bvhAddress >> 3) + nodePointer;

    uint2 retVal;
    retVal.x = LowPart(nodeAddr);
    retVal.y = HighPart(nodeAddr);

    return retVal;
}

//=====================================================================================================================
static uint GetBoxSortingHeuristicFromRayFlags(
    in uint rayFlags,
    in uint mode)
{
    uint heuristic = BoxSortHeuristic::Closest;

#ifdef __cplusplus
    if (rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)
#else
    // Note, wave traversal is always bound by the longest running thread/ray. Because of that we want to pick a
    // a heuristic that makes the longest ray run faster. Select largest first heuristic only if all rays in a wave
    // benefit from it.

    // When largest first heuristic is enabled one of the values used to calculate the sort key gets clamped to
    // ray's tMax. On a commit the ray's tMax value is updated. As a result if after committing the hit a ray visits
    // nodes via stackless walkback, the sort keys are different than the original intersection and hence the child
    // nodes may be sorted in a different order. This breaks stackless walkback as it strictly requires the intersected
    // child nodes to be in the original intersection order. This heuristic is only intended to be used for
    // acceptFirstHitAndEndSearch scenarios where stackless walkback after a committed hit is not triggered.
    //
    if (WaveActiveAllTrue(rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH))
#endif
    {
        if (mode == BoxSortHeuristic::DisabledOnAcceptFirstHit)
        {
            // intentionally choose to disable BoxSort
            // if rayFlag is set to "RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH"
            // The intention is to benefit ShadowRay with BvhNodeSort
            heuristic = BoxSortHeuristic::Disabled;
        }
#if GPURT_BUILD_RTIP2
        else
        {
            // good for shadow rays with terminate on first hit
            heuristic = BoxSortHeuristic::Largest;
        }
#endif
    }
#if GPURT_BUILD_RTIP2
    else if (mode == BoxSortHeuristic::LargestFirstOrClosestMidPoint)
    {
        heuristic = BoxSortHeuristic::MidPoint;
    }
#endif
    else
    {
        heuristic = BoxSortHeuristic::Closest;
    }

    return heuristic;
}

#if USE_HW_INTRINSIC == 0
//=====================================================================================================================
// Intersect rays vs bbox and return intersection span.
static float4 fast_intersect_bbox(
    float3 ray_origin, float3 ray_inv_dir, float3 box_min, float3 box_max, float t_max)
{
    const float3 box_min_rel = box_min - ray_origin;
    const float3 box_max_rel = box_max - ray_origin;

    const float3 t_plane_min = box_min_rel * ray_inv_dir;
    const float3 t_plane_max = box_max_rel * ray_inv_dir;

    float3 min_interval, max_interval;

    min_interval.x = ray_inv_dir.x >= 0.0f ? t_plane_min.x : t_plane_max.x;
    max_interval.x = ray_inv_dir.x >= 0.0f ? t_plane_max.x : t_plane_min.x;

    min_interval.y = ray_inv_dir.y >= 0.0f ? t_plane_min.y : t_plane_max.y;
    max_interval.y = ray_inv_dir.y >= 0.0f ? t_plane_max.y : t_plane_min.y;

    min_interval.z = ray_inv_dir.z >= 0.0f ? t_plane_min.z : t_plane_max.z;
    max_interval.z = ray_inv_dir.z >= 0.0f ? t_plane_max.z : t_plane_min.z;

    // intersection interval before clamping
    float min_of_intervals_t = max3(min_interval);
    float max_of_intervals_t = min3(max_interval);

    // intersection interval after clamping
    float min_t = max(min_of_intervals_t, 0.0f);
    float max_t = min(max_of_intervals_t, t_max);

    if (isnan(min_of_intervals_t) || isnan(max_of_intervals_t))
    {
        min_t = INFINITY;
        max_t = -INFINITY;
    }

    // NaNs for values used in the closest midpoint sort algorithm are overridden to
    // maintain consistency with the other sorting heuristic.
    if (isnan(min_of_intervals_t))
    {
        min_of_intervals_t = 0;
    }

    if (isnan(max_of_intervals_t))
    {
        max_of_intervals_t = INFINITY;
    }

    return float4(min_t, max_t, min_of_intervals_t, max_of_intervals_t);
}

//=====================================================================================================================
static uint4 IntersectNodeBvh4(
    const Float32BoxNode node, float ray_extent, float3 ray_origin, float3 ray_inv_dir, uint box_grow_ulp, uint box_sort_heuristic)
{
    // Box nodes consist of the bounds of four different axis aligned
    // bounding boxes
    uint child0, child1, child2, child3;
    float3 pmin0, pmin1, pmin2, pmin3, pmax0, pmax1, pmax2, pmax3;
    bool box_sorting_enabled = (box_sort_heuristic != BoxSortHeuristic::Disabled);
    child0 = node.child0;
    child1 = node.child1;
    child2 = node.child2;
    child3 = node.child3;

    pmin0 = node.bbox0_min;
    pmax0 = node.bbox0_max;
    pmin1 = node.bbox1_min;
    pmax1 = node.bbox1_max;
    pmin2 = node.bbox2_min;
    pmax2 = node.bbox2_max;
    pmin3 = node.bbox3_min;
    pmax3 = node.bbox3_max;

    // Perform ray box intersection for all 4 children
    float4 s0 = fast_intersect_bbox(ray_origin, ray_inv_dir, pmin0, pmax0, ray_extent);
    float4 s1 = fast_intersect_bbox(ray_origin, ray_inv_dir, pmin1, pmax1, ray_extent);
    float4 s2 = fast_intersect_bbox(ray_origin, ray_inv_dir, pmin2, pmax2, ray_extent);
    float4 s3 = fast_intersect_bbox(ray_origin, ray_inv_dir, pmin3, pmax3, ray_extent);

    // Setup default sorting key to handle box_sort_heuristic == 0(closest child)
    // and box_sort_heuristic == 3 (undefined sorting order))
    float sort_key0 = s0.x;
    float sort_key1 = s1.x;
    float sort_key2 = s2.x;
    float sort_key3 = s3.x;

#if GPURT_BUILD_RTIP2
    // Setup box sort keys for largest interval first traversal.
    // Note: this computation causes the sort keys to be the negative size of the
    // interval. This is because the sorting network sorts from smallest to largest key
    // and we want the boxes to be sorted from largest
    // interval to the smallest sized interval in this mode.
    if (box_sort_heuristic == BoxSortHeuristic::Largest) // largest first
    {
        sort_key0 = s0.x-s0.y; //min_t0-max_t0
        sort_key1 = s1.x-s1.y; //min_t1-max_t1
        sort_key2 = s2.x-s2.y; //min_t2-max_t2
        sort_key3 = s3.x-s3.y; //min_t3-max_t3
    }
    // Setup the box sort keys for the closest midpoint.
    // Note: the full computation of midpoint is (min_of_intervals+max_of_intervals)/2,
    // however the division by 2 is removed to save HW cost as it does not change the results of the sorting network.
    // Also, sort keys may be negative if the midpoint occurs behind the ray origin.
    else if (box_sort_heuristic == BoxSortHeuristic::MidPoint) // closest mid-point
    {
        sort_key0 = s0.z+s0.w; //min_of_intervals_t0+max_of_intervals_t0
        sort_key1 = s1.z+s1.w; //min_of_intervals_t1+max_of_intervals_t1
        sort_key2 = s2.z+s2.w; //min_of_intervals_t2+max_of_intervals_t2
        sort_key3 = s3.z+s3.w; //min_of_intervals_t3+max_of_intervals_t3
    }
#endif

    //Mark out the nodes that didn't hit by setting them to the
    //INVALID_NODE.

    //The check for equality here is important since errors from the
    //floating point rounding will cause small intervals to snap to identical
    //times. Under this situation you can not determine if the box intersected
    //So, you must conservatively treat equality as a hit.
    //
    //Also in order to address the inconsistant error between ray-box and
    //ray-triangle test, we make the ray-box test more conservative by growing the
    //interval in post-projection space by several ULPs.

    const float eps = 5.960464478e-8f; // 2^-24;
    uint traverse0 = (s0.x <= (s0.y * (1 + box_grow_ulp * eps))) ? child0 : INVALID_NODE;
    uint traverse1 = (s1.x <= (s1.y * (1 + box_grow_ulp * eps))) ? child1 : INVALID_NODE;
    uint traverse2 = (s2.x <= (s2.y * (1 + box_grow_ulp * eps))) ? child2 : INVALID_NODE;
    uint traverse3 = (s3.x <= (s3.y * (1 + box_grow_ulp * eps))) ? child3 : INVALID_NODE;

    //4-item sorting network to optimize BVH traversal using the
    //distance heuristic
    //Traverse0 --*------*------- Traverse0
    //            |      |
    //Traverse1 -(---*---*---*--- Traverse1
    //            |  |       |
    //Traverse2 --*-(----*---*--- Traverse2
    //               |   |
    //Traverse3 -----*---*------- Traverse3

    if (box_sorting_enabled)
    {
        SORT(traverse0, traverse2, sort_key0, sort_key2)
        SORT(traverse1, traverse3, sort_key1, sort_key3)
        SORT(traverse0, traverse1, sort_key0, sort_key1)
        SORT(traverse2, traverse3, sort_key2, sort_key3)
        SORT(traverse1, traverse2, sort_key1, sort_key2)
    }

    return uint4(traverse0, traverse1, traverse2, traverse3);
}

//=====================================================================================================================
// Intersect ray against a triangle and return whether the triangle hit is accepted or not. If hit is accepted
// hit attributes (closest distance, barycentrics and hit kind) are updated
static uint4 fast_intersect_triangle(
    float3      origin,
    float3      direction,
    float3      v1,
    float3      v2,
    float3      v3)
{
    // Determine edge vectors for clockwise triangle vertices
    float3 e1 = v2 - v1;
    float3 e2 = v3 - v1;
    float3 e3 = origin - v1;

    float4 result;

    const float3 s1 = cross(direction, e2);
    const float3 s2 = cross(e3, e1);

    result.x = dot(e2, s2);
    result.y = dot(s1, e1);
    result.z = dot(e3, s1);
    result.w = dot(direction, s2);

    float t = result.x / result.y;
    float u = result.z / result.y;
    float v = result.w / result.y;

    // Barycentric coordinate U is outside range
    bool triangle_missed = ((u < 0.f) || (u > 1.f));
    triangle_missed |= ((v < 0.f) || (u + v > 1.f));
    triangle_missed |= (t < 0.f);

    const float inf = INFINITY;
    result.x = triangle_missed ? inf  : result.x;
    result.y = triangle_missed ? 1.0f : result.y;

    return asuint(result);
}

//=====================================================================================================================
// Swizzle barycentrics to undo the triangle rotation that was applied during compression.
static void SwizzleBarycentrics(
    inout_param(uint4) result,
    in uint            nodePointer,
    in uint            triangleId)
{
    // The triangle barycentric coordinates are:
    // barycentrics.x = asfloat(result.z) / asfloat(result.y)
    // barycentrics.y = asfloat(result.w) / asfloat(result.y)
    // barycentrics.w = (1 - barycentrics.x - barycentrics.y)
    //
    // The following is derived by multiplying all the barycentric coordinates by the tDenom, i.e., result.y, to avoid
    // additional ALU ops since we have to multiply by result.y again anyway when modifying result.z and result.w.
    float baryc[3] = { asfloat(result.y) - asfloat(result.z) - asfloat(result.w),
                       asfloat(result.z),
                       asfloat(result.w) };

    const uint triangleShift = TRIANGLE_ID_BIT_STRIDE * GetNodeType(nodePointer);

    result.z = asuint(baryc[(triangleId >> (triangleShift + TRIANGLE_ID_I_SRC_SHIFT)) % 4]);
    result.w = asuint(baryc[(triangleId >> (triangleShift + TRIANGLE_ID_J_SRC_SHIFT)) % 4]);
}

#if GPURT_BUILD_RTIP2
//=====================================================================================================================
static void PerformTriangleCulling(
    in uint32_t        intersectRayVersion,
    in uint64_t        hwNodePtr,
    in bool            isOpaque,
    inout_param(uint4) result)
{
    bool triangleCulled = false;

    {
        // If determinant is positive and front face winding is counterclockwise or vice versa, the triangle is
        // back facing.
        bool frontFacingTriangle = (asfloat(result.y) >= 0.0f);
        if (FLAG_IS_SET(hwNodePtr, NODE_POINTER_FLIP_FACEDNESS))
        {
            frontFacingTriangle = !frontFacingTriangle;

            // The signs of t_num and t_denom are flipped when winding is flipped.
            // The sign of t_denom indicates front vs back face.
            result.x ^= 0x80000000;
            result.y ^= 0x80000000;
        }

        const bool faceCulled = frontFacingTriangle ?
            FLAG_IS_SET(hwNodePtr, NODE_POINTER_CULL_FRONT_FACING) :
            FLAG_IS_SET(hwNodePtr, NODE_POINTER_CULL_BACK_FACING);

        triangleCulled = faceCulled || FLAG_IS_SET(hwNodePtr, NODE_POINTER_SKIP_TRIANGLES);
    }

    const bool opaque =
        FLAG_IS_SET(hwNodePtr, NODE_POINTER_FORCE_OPAQUE) ||
        (FLAG_IS_CLEAR(hwNodePtr, NODE_POINTER_FORCE_NON_OPAQUE) &&
            isOpaque);

    const bool opaqueCulled = opaque && FLAG_IS_SET(hwNodePtr, NODE_POINTER_CULL_OPAQUE);

    if (triangleCulled || opaqueCulled)
    {
        // Set tNum and tDenom when primitive is culled
        result = uint4(asuint(INFINITY), asuint(1.0f), 0, 0);
    }
}

//=====================================================================================================================
static bool IsChildEarlyCulled(uint childIndex, uint allBoxFlags, uint64_t hwNodePtr)
{
    const uint boxFlags = (allBoxFlags >> (childIndex * 8)) & 0xFF;

    const bool subtree_only_contains_opaque =
        (FLAG_IS_SET(boxFlags, BOX_NODE_FLAGS_ONLY_OPAQUE) ||
         FLAG_IS_SET(hwNodePtr, NODE_POINTER_FORCE_OPAQUE)) &&
        FLAG_IS_CLEAR(hwNodePtr, NODE_POINTER_FORCE_NON_OPAQUE);

    const bool subtree_only_contains_non_opaque =
        (FLAG_IS_SET(boxFlags, BOX_NODE_FLAGS_ONLY_NON_OPAQUE) ||
         FLAG_IS_SET(hwNodePtr, NODE_POINTER_FORCE_NON_OPAQUE)) &&
        FLAG_IS_CLEAR(hwNodePtr, NODE_POINTER_FORCE_OPAQUE);

    const bool should_cull_subtree =
        (subtree_only_contains_opaque     && FLAG_IS_SET(hwNodePtr, NODE_POINTER_CULL_OPAQUE)) ||
        (subtree_only_contains_non_opaque && FLAG_IS_SET(hwNodePtr, NODE_POINTER_CULL_NON_OPAQUE)) ||
        (FLAG_IS_SET(boxFlags, BOX_NODE_FLAGS_ONLY_TRIANGLES)  && FLAG_IS_SET(hwNodePtr, NODE_POINTER_SKIP_TRIANGLES)) ||
        (FLAG_IS_SET(boxFlags, BOX_NODE_FLAGS_ONLY_PROCEDURAL) && FLAG_IS_SET(hwNodePtr, NODE_POINTER_SKIP_PROCEDURAL));

    return should_cull_subtree;
}

//=====================================================================================================================
static void PerformEarlyBoxCulling(inout_param(Float32BoxNode) node, uint64_t hwNodePtr)
{
    const uint boxFlags = node.flags;

    if (IsChildEarlyCulled(0, boxFlags, hwNodePtr))
    {
        node.child0 = INVALID_NODE;
    }

    if (IsChildEarlyCulled(1, boxFlags, hwNodePtr))
    {
        node.child1 = INVALID_NODE;
    }

    if (IsChildEarlyCulled(2, boxFlags, hwNodePtr))
    {
        node.child2 = INVALID_NODE;
    }

    if (IsChildEarlyCulled(3, boxFlags, hwNodePtr))
    {
        node.child3 = INVALID_NODE;
    }
}
#endif
#endif

//=====================================================================================================================
static uint4 image_bvh64_intersect_ray_base(
    GpuVirtualAddress bvhAddress,
    uint              nodePointer,
    uint              pointerFlags,
    uint              boxSortHeuristic,
    float             rayExtent,
    float3            rayOrigin,
    float3            rayDirection,
    float3            rayDirectionInverse,
    uint              intersectRayVersion)
{
    uint4 result;

#if USE_HW_INTRINSIC
    uint2 address = CalculateRawBvh64NodePointer(bvhAddress, nodePointer);

#if GPURT_BUILD_RTIP2
    if (intersectRayVersion >= INTERSECT_RAY_VERSION_2)
    {
        address.y |= pointerFlags;
    }
#endif

    result = AmdExtD3DShaderIntrinsics_IntersectInternal(address,
                                                         rayExtent,
                                                         rayOrigin,
                                                         rayDirection,
                                                         rayDirectionInverse,
                                                         boxSortHeuristic,
                                                         BOX_EXPANSION_DEFAULT_AMOUNT);

#else
    uint64_t hwNodePtr = (bvhAddress >> 3) + nodePointer;

#if GPURT_BUILD_RTIP2
    if (intersectRayVersion >= INTERSECT_RAY_VERSION_2)
    {
        const uint64_t pointerFlagsU64 = pointerFlags;
        hwNodePtr |= pointerFlagsU64 << 32;
    }
#endif

    if (IsBoxNode1_1(nodePointer))
    {
        // When using 16bit bboxes in BLAS, convert to a full 32bit box on load for simplicity
        Float32BoxNode node;

        if (IsBoxNode16(nodePointer)
            )
        {
            node = FetchFloat16BoxNodeAsFp32(bvhAddress, nodePointer);
        }
        else
        {
            node = FetchFloat32BoxNode(bvhAddress,
                                       nodePointer);
#if GPURT_BUILD_RTIP2
            if (intersectRayVersion >= INTERSECT_RAY_VERSION_2)
            {
                PerformEarlyBoxCulling(node, hwNodePtr);
            }
#endif
        }

        // Intersect ray with qbvh node
        result = IntersectNodeBvh4(node,
                                   rayExtent,
                                   rayOrigin,
                                   rayDirectionInverse,
                                   BOX_EXPANSION_DEFAULT_AMOUNT,
                                   boxSortHeuristic);
    }
    else if (IsTriangleNode1_1(nodePointer))
    {
        bool procedural = false;
        uint hwTriFlags = 0;

        uint triangleId = FetchTriangleId(bvhAddress, nodePointer);

#if GPURT_BUILD_RTIP2
        if (intersectRayVersion >= INTERSECT_RAY_VERSION_2)
        {
            hwTriFlags = triangleId >> (GetNodeType(nodePointer) * TRIANGLE_ID_BIT_STRIDE);

        }
#endif

        {
            TriangleData tri = FetchTriangleFromNode(bvhAddress, nodePointer);
            result = fast_intersect_triangle(rayOrigin,
                                             rayDirection,
                                             tri.v0,
                                             tri.v1,
                                             tri.v2);

            SwizzleBarycentrics(result, nodePointer, triangleId);
        }

#if GPURT_BUILD_RTIP2
        if (intersectRayVersion >= INTERSECT_RAY_VERSION_2)
        {

            const bool opaque = FLAG_IS_SET(hwTriFlags, TRIANGLE_ID_OPAQUE);
            PerformTriangleCulling(intersectRayVersion,
                                   hwNodePtr,
                                   opaque,
                                   result);
        }
#endif
    }
    else
    {
        // User defined node. HW returns -1
        return uint4(-1, -1, -1, -1);
    }
#endif

    return result;
}

//=====================================================================================================================
static uint4 image_bvh64_intersect_ray(
    GpuVirtualAddress bvhAddress,
    uint              nodePointer,
    uint              boxSortHeuristic,
    float             rayExtent,
    float3            rayOrigin,
    float3            rayDirection,
    float3            rayDirectionInverse)
{
    return image_bvh64_intersect_ray_base(
        bvhAddress, nodePointer, 0, boxSortHeuristic, rayExtent, rayOrigin, rayDirection, rayDirectionInverse,
        INTERSECT_RAY_VERSION_1);
}

#if GPURT_BUILD_RTIP2
//=====================================================================================================================
static uint4 image_bvh64_intersect_ray_2_0(
    GpuVirtualAddress bvhAddress,
    uint              nodePointer,
    uint              pointerFlags,
    uint              boxSortHeuristic,
    float             rayExtent,
    float3            rayOrigin,
    float3            rayDirection,
    float3            rayDirectionInverse)
{
    return image_bvh64_intersect_ray_base(
        bvhAddress, nodePointer, pointerFlags, boxSortHeuristic, rayExtent, rayOrigin, rayDirection, rayDirectionInverse,
        INTERSECT_RAY_VERSION_2);
}
#endif

#endif
