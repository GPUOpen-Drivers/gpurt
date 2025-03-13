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

#ifndef RTIP3_OPCODE_EMULATION_HLSL
#define RTIP3_OPCODE_EMULATION_HLSL

#if GPURT_BUILD_RTIP3_1
#include "rtip3_1.hlsli"
#include "ObbCommon.hlsl"
#endif

//=====================================================================================================================
// Fetch hardware instance transform node data. This is the first 64-bytes of the 128-byte hardware instance
// node data.
//
static HwInstanceTransformNode FetchHwInstanceTransformNode(
    in uint64_t baseAddr,
    in uint32_t nodePtr)
{
    const uint64_t instanceAddr = baseAddr + ExtractNodePointerOffset(nodePtr);

    uint4 d0, d1, d2, d3;

    d0 = LoadDwordAtAddrx4(instanceAddr);
    d1 = LoadDwordAtAddrx4(instanceAddr + 0x10);
    d2 = LoadDwordAtAddrx4(instanceAddr + 0x20);
    d3 = LoadDwordAtAddrx4(instanceAddr + 0x30);

    HwInstanceTransformNode node;

    node.worldToObject[0][0] = asfloat(d0.x);
    node.worldToObject[0][1] = asfloat(d0.y);
    node.worldToObject[0][2] = asfloat(d0.z);
    node.worldToObject[0][3] = asfloat(d0.w);
    node.worldToObject[1][0] = asfloat(d1.x);
    node.worldToObject[1][1] = asfloat(d1.y);
    node.worldToObject[1][2] = asfloat(d1.z);
    node.worldToObject[1][3] = asfloat(d1.w);
    node.worldToObject[2][0] = asfloat(d2.x);
    node.worldToObject[2][1] = asfloat(d2.y);
    node.worldToObject[2][2] = asfloat(d2.z);
    node.worldToObject[2][3] = asfloat(d2.w);

    node.childBasePtr               = PackUint64(d3.x, d3.y);
    node.childRootNodeOrParentPtr   = d3.z;
    node.userDataAndInstanceMask    = d3.w;

    return node;
}

//=====================================================================================================================
// Generate 64-bit bottom level acceleration structure base address from hardware instance node
//
static uint64_t GetBlasBaseNodePtr(
    in uint64_t tlasBasePtr,
    in uint64_t blasBasePtr)
{
    const uint baseFlags = uint(tlasBasePtr >> NODE_POINTER_FLAGS_SHIFT);
    const uint instanceFlags = uint(blasBasePtr >> NODE_POINTER_FLAGS_SHIFT);

    // If the ray flags (pointer flags in the passed in base) force opaque on or off, the opaque
    // flag in the instance is masked out. Other flags are ORed together.
    const uint instanceFlagsPreserveMask =
        ((baseFlags & RAY_FLAG_OVERRIDE_MASK) ? RAY_FLAG_PRESERVE_MASK : RAY_FLAG_VALID_MASK);

    const uint64_t newFlags = (instanceFlags & instanceFlagsPreserveMask) | baseFlags;

    return (blasBasePtr & INSTANCE_BASE_POINTER_ADDRESS_MASK) | (newFlags << NODE_POINTER_FLAGS_SHIFT);
}

//=====================================================================================================================
static uint FetchTriangleGeometryIndex3_0(in GpuVirtualAddress bvhAddress, in uint nodePointer)
{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    const GpuVirtualAddress nodeAddr = bvhAddress + byteOffset;

    return LoadDwordAtAddr(nodeAddr + RTIP3_TRIANGLE_NODE_GEOMETRY_INDEX_OFFSET);
}

//=====================================================================================================================
static void Sort(
    inout_param(uint) ext_sort_key0,
    inout_param(uint) ext_sort_key1,
    inout_param(uint) pointer0,
    inout_param(uint) pointer1,
    inout_param(float) sort_key0,
    inout_param(float) sort_key1)
{
    bool swap = false;
    if (ext_sort_key0 != ext_sort_key1)
    {
        swap = (ext_sort_key1 < ext_sort_key0);
    }
    else
    {
        swap = (sort_key1 < sort_key0);
    }

    if (swap)
    {
        uint t0 = ext_sort_key0;
        ext_sort_key0 = ext_sort_key1;
        ext_sort_key1 = t0;

        float t1 = sort_key0;
        sort_key0 = sort_key1;
        sort_key1 = t1;

        uint t2 = pointer0;
        pointer0 = pointer1;
        pointer1 = t2;
    }
}

#define NODE_TYPE(ptr) (ptr & 0x7)

//=====================================================================================================================
// Dual-intersection ray-box intersection emulation
static void IntersectNodeBvh8(
    in Float32BoxNode                node0,
    in Float32BoxNode                node1,
    in float                         ray_extent,
    in float3                        ray_origin,
    in float3                        ray_inv_dir,
    in uint                          box_grow_ulp,
    in uint                          box_sort_heuristic,
    in bool                          sort_triangles_first,
    out_param(uint4)                 result0,
    out_param(uint4)                 result1)
{
    // Box nodes consist of the bounds of eight different axis aligned bounding boxes
    uint child0, child1, child2, child3, child4, child5, child6, child7;
    child0 = node0.child0;
    child1 = node0.child1;
    child2 = node0.child2;
    child3 = node0.child3;
    child4 = node1.child0;
    child5 = node1.child1;
    child6 = node1.child2;
    child7 = node1.child3;

    float3 pmin0, pmin1, pmin2, pmin3, pmin4, pmin5, pmin6, pmin7, pmax0, pmax1, pmax2, pmax3, pmax4, pmax5, pmax6, pmax7;
    pmin0 = node0.bbox0_min;
    pmax0 = node0.bbox0_max;
    pmin1 = node0.bbox1_min;
    pmax1 = node0.bbox1_max;
    pmin2 = node0.bbox2_min;
    pmax2 = node0.bbox2_max;
    pmin3 = node0.bbox3_min;
    pmax3 = node0.bbox3_max;
    pmin4 = node1.bbox0_min;
    pmax4 = node1.bbox0_max;
    pmin5 = node1.bbox1_min;
    pmax5 = node1.bbox1_max;
    pmin6 = node1.bbox2_min;
    pmax6 = node1.bbox2_max;
    pmin7 = node1.bbox3_min;
    pmax7 = node1.bbox3_max;

    // Perform ray box intersection for all 8 children
    float4 s0 = fast_intersect_bbox(ray_origin, ray_inv_dir, pmin0, pmax0, ray_extent);
    float4 s1 = fast_intersect_bbox(ray_origin, ray_inv_dir, pmin1, pmax1, ray_extent);
    float4 s2 = fast_intersect_bbox(ray_origin, ray_inv_dir, pmin2, pmax2, ray_extent);
    float4 s3 = fast_intersect_bbox(ray_origin, ray_inv_dir, pmin3, pmax3, ray_extent);
    float4 s4 = fast_intersect_bbox(ray_origin, ray_inv_dir, pmin4, pmax4, ray_extent);
    float4 s5 = fast_intersect_bbox(ray_origin, ray_inv_dir, pmin5, pmax5, ray_extent);
    float4 s6 = fast_intersect_bbox(ray_origin, ray_inv_dir, pmin6, pmax6, ray_extent);
    float4 s7 = fast_intersect_bbox(ray_origin, ray_inv_dir, pmin7, pmax7, ray_extent);

    // Setup default sorting key to handle box_sort_heuristic == 0(closest child)
    // and box_sort_heuristic == 3 (undefined sorting order))
    float sort_key0 = s0.x;
    float sort_key1 = s1.x;
    float sort_key2 = s2.x;
    float sort_key3 = s3.x;
    float sort_key4 = s4.x;
    float sort_key5 = s5.x;
    float sort_key6 = s6.x;
    float sort_key7 = s7.x;

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
        sort_key4 = s4.x-s4.y; //min_t4-max_t4
        sort_key5 = s5.x-s5.y; //min_t5-max_t5
        sort_key6 = s6.x-s6.y; //min_t6-max_t6
        sort_key7 = s7.x-s7.y; //min_t7-max_t7
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
        sort_key4 = s4.z+s4.w; //min_of_intervals_t4+max_of_intervals_t4
        sort_key5 = s5.z+s5.w; //min_of_intervals_t5+max_of_intervals_t5
        sort_key6 = s6.z+s6.w; //min_of_intervals_t6+max_of_intervals_t6
        sort_key7 = s7.z+s7.w; //min_of_intervals_t7+max_of_intervals_t7
    }

    // Mark out the nodes that didn't hit by setting them to the
    // INVALID_NODE.

    // The check for equality here is important since errors from the
    // floating point rounding will cause small intervals to snap to identical
    // times. Under this situation you can not determine if the box intersected
    // So, you must conservatively treat equality as a hit.
    //
    // Also in order to address the inconsistant error between ray-box and
    // ray-triangle test, we make the ray-box test more conservative by growing the
    // interval in post-projection space by several ULPs.

    const float eps = 5.960464478e-8f; // 2^-24;

    uint traverse[8];
    traverse[0] = (s0.x <= (s0.y * (1 + box_grow_ulp * eps))) ? child0 : INVALID_NODE;
    traverse[1] = (s1.x <= (s1.y * (1 + box_grow_ulp * eps))) ? child1 : INVALID_NODE;
    traverse[2] = (s2.x <= (s2.y * (1 + box_grow_ulp * eps))) ? child2 : INVALID_NODE;
    traverse[3] = (s3.x <= (s3.y * (1 + box_grow_ulp * eps))) ? child3 : INVALID_NODE;
    traverse[4] = (s4.x <= (s4.y * (1 + box_grow_ulp * eps))) ? child4 : INVALID_NODE;
    traverse[5] = (s5.x <= (s5.y * (1 + box_grow_ulp * eps))) ? child5 : INVALID_NODE;
    traverse[6] = (s6.x <= (s6.y * (1 + box_grow_ulp * eps))) ? child6 : INVALID_NODE;
    traverse[7] = (s7.x <= (s7.y * (1 + box_grow_ulp * eps))) ? child7 : INVALID_NODE;

    // De-duplicate child reuse nodes. All child reuse nodes are expected to be in contiguous slots
    uint previous = traverse[0];
    for (uint i = 1; i < 8; ++i)
    {
        if (traverse[i] == previous)
        {
            traverse[i] = INVALID_NODE;
        }
        else
        {
            previous = traverse[i];
        }
    }

    // 8-item sorting network to optimize BVH traversal using the
    // distance heuristic

    const uint max_leaf_node_type = 3; // BVH8 addressing only

    // Generate 4-bit extended sort key with following bits
    // uint ext_sort_key0[3:0] = {dest_slot_id, is_not_valid0, is_not_leaf0, is_not_pair0}
    //

    // is_not_valid
    uint ext_sort_key0 = (traverse[0] == INVALID_NODE) ? 4 : 0;
    uint ext_sort_key1 = (traverse[1] == INVALID_NODE) ? 4 : 0;
    uint ext_sort_key2 = (traverse[2] == INVALID_NODE) ? 4 : 0;
    uint ext_sort_key3 = (traverse[3] == INVALID_NODE) ? 4 : 0;
    uint ext_sort_key4 = (traverse[4] == INVALID_NODE) ? 4 : 0;
    uint ext_sort_key5 = (traverse[5] == INVALID_NODE) ? 4 : 0;
    uint ext_sort_key6 = (traverse[6] == INVALID_NODE) ? 4 : 0;
    uint ext_sort_key7 = (traverse[7] == INVALID_NODE) ? 4 : 0;

    // is_node_leaf
    ext_sort_key0 |= ((NODE_TYPE(traverse[0]) > max_leaf_node_type) && sort_triangles_first) ? 2 : 0;
    ext_sort_key1 |= ((NODE_TYPE(traverse[1]) > max_leaf_node_type) && sort_triangles_first) ? 2 : 0;
    ext_sort_key2 |= ((NODE_TYPE(traverse[2]) > max_leaf_node_type) && sort_triangles_first) ? 2 : 0;
    ext_sort_key3 |= ((NODE_TYPE(traverse[3]) > max_leaf_node_type) && sort_triangles_first) ? 2 : 0;
    ext_sort_key4 |= ((NODE_TYPE(traverse[4]) > max_leaf_node_type) && sort_triangles_first) ? 2 : 0;
    ext_sort_key5 |= ((NODE_TYPE(traverse[5]) > max_leaf_node_type) && sort_triangles_first) ? 2 : 0;
    ext_sort_key6 |= ((NODE_TYPE(traverse[6]) > max_leaf_node_type) && sort_triangles_first) ? 2 : 0;
    ext_sort_key7 |= ((NODE_TYPE(traverse[7]) > max_leaf_node_type) && sort_triangles_first) ? 2 : 0;

    // is_not_pair
    ext_sort_key0 |= ((NODE_TYPE(traverse[0]) != 1) && sort_triangles_first) ? 1 : 0;
    ext_sort_key1 |= ((NODE_TYPE(traverse[1]) != 1) && sort_triangles_first) ? 1 : 0;
    ext_sort_key2 |= ((NODE_TYPE(traverse[2]) != 1) && sort_triangles_first) ? 1 : 0;
    ext_sort_key3 |= ((NODE_TYPE(traverse[3]) != 1) && sort_triangles_first) ? 1 : 0;
    ext_sort_key4 |= ((NODE_TYPE(traverse[4]) != 1) && sort_triangles_first) ? 1 : 0;
    ext_sort_key5 |= ((NODE_TYPE(traverse[5]) != 1) && sort_triangles_first) ? 1 : 0;
    ext_sort_key6 |= ((NODE_TYPE(traverse[6]) != 1) && sort_triangles_first) ? 1 : 0;
    ext_sort_key7 |= ((NODE_TYPE(traverse[7]) != 1) && sort_triangles_first) ? 1 : 0;

    const bool box_sorting_enabled = (box_sort_heuristic != BoxSortHeuristic::Disabled);
    if (box_sorting_enabled)
    {
        // Stage 1
        Sort(ext_sort_key0, ext_sort_key2, traverse[0], traverse[2], sort_key0, sort_key2);
        Sort(ext_sort_key1, ext_sort_key3, traverse[1], traverse[3], sort_key1, sort_key3);
        Sort(ext_sort_key4, ext_sort_key6, traverse[4], traverse[6], sort_key4, sort_key6);
        Sort(ext_sort_key5, ext_sort_key7, traverse[5], traverse[7], sort_key5, sort_key7);

        // Stage 2
        Sort(ext_sort_key0, ext_sort_key1, traverse[0], traverse[1], sort_key0, sort_key1);
        Sort(ext_sort_key2, ext_sort_key3, traverse[2], traverse[3], sort_key2, sort_key3);
        Sort(ext_sort_key4, ext_sort_key5, traverse[4], traverse[5], sort_key4, sort_key5);
        Sort(ext_sort_key6, ext_sort_key7, traverse[6], traverse[7], sort_key6, sort_key7);

        // Stage 3
        Sort(ext_sort_key1, ext_sort_key2, traverse[1], traverse[2], sort_key1, sort_key2);
        Sort(ext_sort_key5, ext_sort_key6, traverse[5], traverse[6], sort_key5, sort_key6);

        // Stage 4
        Sort(ext_sort_key0, ext_sort_key4, traverse[0], traverse[4], sort_key0, sort_key4);
        Sort(ext_sort_key1, ext_sort_key5, traverse[1], traverse[5], sort_key1, sort_key5);
        Sort(ext_sort_key2, ext_sort_key6, traverse[2], traverse[6], sort_key2, sort_key6);
        Sort(ext_sort_key3, ext_sort_key7, traverse[3], traverse[7], sort_key3, sort_key7);

        // Stage 5
        Sort(ext_sort_key2, ext_sort_key4, traverse[2], traverse[4], sort_key2, sort_key4);
        Sort(ext_sort_key3, ext_sort_key5, traverse[3], traverse[5], sort_key3, sort_key5);

        // Stage 6
        Sort(ext_sort_key1, ext_sort_key2, traverse[1], traverse[2], sort_key1, sort_key2);
        Sort(ext_sort_key3, ext_sort_key4, traverse[3], traverse[4], sort_key3, sort_key4);
        Sort(ext_sort_key5, ext_sort_key6, traverse[5], traverse[6], sort_key5, sort_key6);
    }

    result0 = uint4(traverse[0], traverse[1], traverse[2], traverse[3]);
    result1 = uint4(traverse[4], traverse[5], traverse[6], traverse[7]);
}

//=====================================================================================================================
// Hardware ray-instance intersection emulation
static void IntersectHardwareInstanceNode(
    in uint32_t                    rtIpLevel,
    in uint64_t                    baseNodePtr,
    in uint32_t                    nodePtr,
    inout_param(float3)            rayOrigin,
    inout_param(float3)            rayDir,
    in float                       rayExtent,
    in uint32_t                    instanceMask,
    in uint                        boxSortHeuristic,
    out_param(uint4)               result0,
    out_param(uint4)               result1)
{
    const uint pointerFlags = uint(baseNodePtr >> NODE_POINTER_FLAGS_SHIFT);

    const uint64_t baseAddr = ExtractInstanceAddr(baseNodePtr);

    HwInstanceTransformNode instanceNode = FetchHwInstanceTransformNode(baseAddr, nodePtr);

    /// @note: Instance mask is 0 for null BLAS and it would be ignored
    bool isInstanceCulled = true;

    if ((instanceNode.userDataAndInstanceMask >> 24) & instanceMask)
    {
        //TODO: need consulting on whether to include pipelineRayFlags
        //            and if this encoding of geomType is the same with spec
        const uint geomType = uint(instanceNode.childBasePtr >> 62) & 0x1;

        const bool isTriangleInstanceCulled =
            ((geomType == GEOMETRY_TYPE_TRIANGLES) && (pointerFlags & RAY_FLAG_SKIP_TRIANGLES));

        const bool isProceduralInstanceCulled =
            ((geomType == GEOMETRY_TYPE_AABBS) && (pointerFlags & RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES));

        isInstanceCulled = (isTriangleInstanceCulled || isProceduralInstanceCulled);
    }

    // grab next node offset
    uint nextNodeOffset = INVALID_NODE;
    if (isInstanceCulled == false)
    {
        // transform ray
        {
            /// Note, there is a precision issue using GLMs native matrix-vector multiplication. Using the
            /// unrolled InstanceTransform() function from IntersectCommon.hlsl for consistency. Note, this
            /// would be the path used for any emulation on SW anyway for consistency between implementations
            float4 t0 = float4(instanceNode.worldToObject[0][0],
                               instanceNode.worldToObject[0][1],
                               instanceNode.worldToObject[0][2],
                               instanceNode.worldToObject[0][3]);

            float4 t1 = float4(instanceNode.worldToObject[1][0],
                               instanceNode.worldToObject[1][1],
                               instanceNode.worldToObject[1][2],
                               instanceNode.worldToObject[1][3]);

            float4 t2 = float4(instanceNode.worldToObject[2][0],
                               instanceNode.worldToObject[2][1],
                               instanceNode.worldToObject[2][2],
                               instanceNode.worldToObject[2][3]);

            float r0x = mad(rayOrigin.z, t0.z, t0.w);
            float r0y = mad(rayOrigin.z, t1.z, t1.w);
            float r0z = mad(rayOrigin.z, t2.z, t2.w);

            float r1x = mul(rayDir.z, t0.z);
            float r1y = mul(rayDir.z, t1.z);
            float r1z = mul(rayDir.z, t2.z);

            r0x = mad(rayOrigin.y, t0.y, r0x);
            r0y = mad(rayOrigin.y, t1.y, r0y);
            r0z = mad(rayOrigin.y, t2.y, r0z);

            r1x = mad(rayDir.y, t0.y, r1x);
            r1y = mad(rayDir.y, t1.y, r1y);
            r1z = mad(rayDir.y, t2.y, r1z);

            r0x = mad(rayOrigin.x, t0.x, r0x);
            r0y = mad(rayOrigin.x, t1.x, r0y);
            r0z = mad(rayOrigin.x, t2.x, r0z);

            r1x = mad(rayDir.x, t0.x, r1x);
            r1y = mad(rayDir.x, t1.x, r1y);
            r1z = mad(rayDir.x, t2.x, r1z);

            rayOrigin = float3(r0x, r0y, r0z);
            rayDir = float3(r1x, r1y, r1z);
        }

        nextNodeOffset = instanceNode.childRootNodeOrParentPtr;
    }

#if GPURT_BUILD_RTIP3_1
    // Handle intersectable instance node
    if (rtIpLevel >= GPURT_RTIP3_1)
    {
        const uint64_t quantizedBoxNodeAddr = baseAddr + ExtractNodePointerOffset(nodePtr) + sizeof(HwInstanceTransformNode);

        // Start by initializing base offsets. These are updated as children are decoded.
        // Lower 4 bits of internal offset will always be 5 as this is the only valid internal
        // node type.
        uint32_t internalOffset = 0;
        uint32_t primitiveOffset = 0;

        // Fetch origin
        const float3 origin = float3(asfloat(LoadDwordAtAddr(quantizedBoxNodeAddr)),
                                     asfloat(LoadDwordAtAddr(quantizedBoxNodeAddr + 4)),
                                     asfloat(LoadDwordAtAddr(quantizedBoxNodeAddr + 8)));

        // Decode valid child count from quantized box node
        const uint exponentsChildIndexAndChildCount = LoadDwordAtAddr(quantizedBoxNodeAddr + 12);

        // 'validChildCount' has range 1-4 but encoded as 0-3 in node.
        const uint validChildCount = ExtractBvh4ValidChildCount(exponentsChildIndexAndChildCount);

        Float32BoxNode node = (Float32BoxNode)0;
        node.child0 = INVALID_NODE;
        node.child1 = INVALID_NODE;
        node.child2 = INVALID_NODE;
        node.child3 = INVALID_NODE;

        // Extract packed exponents
        uint3 exponents;
        exponents.x = bitFieldExtract(exponentsChildIndexAndChildCount, 0, 8);
        exponents.y = bitFieldExtract(exponentsChildIndexAndChildCount, 8, 8);
        exponents.z = bitFieldExtract(exponentsChildIndexAndChildCount, 16, 8);

        // Decode child pointers and bounds from quantized node
        for (uint32_t i = 0; i < validChildCount; ++i)
        {
            const uint64_t childInfoAddr = quantizedBoxNodeAddr + GetBvh4ChildInfoOffset(i);

            uint3 childInfo;
            childInfo.x = LoadDwordAtAddr(childInfoAddr);
            childInfo.y = LoadDwordAtAddr(childInfoAddr + 4);
            childInfo.z = LoadDwordAtAddr(childInfoAddr + 8);

            const uint32_t nodeType = bitFieldExtract(childInfo.z, 24, 4);
            const uint32_t nodeRangeLength = bitFieldExtract(childInfo.z, 28, 4);

            const uint3 quantMin = uint3(bitFieldExtract(childInfo.x, 0, 12),
                                         bitFieldExtract(childInfo.x, 12, 12),
                                         bitFieldExtract(childInfo.y, 0, 12));
            const uint3 quantMax = uint3(bitFieldExtract(childInfo.y, 12, 12),
                                         bitFieldExtract(childInfo.z, 0, 12),
                                         bitFieldExtract(childInfo.z, 12, 12));

            const float3 bbox_min = float3(Dequantize(origin.x, exponents.x, quantMin.x, 12),
                                           Dequantize(origin.y, exponents.y, quantMin.y, 12),
                                           Dequantize(origin.z, exponents.z, quantMin.z, 12));

            const float3 bbox_max = float3(Dequantize(origin.x, exponents.x, quantMax.x + 1, 12),
                                           Dequantize(origin.y, exponents.y, quantMax.y + 1, 12),
                                           Dequantize(origin.z, exponents.z, quantMax.z + 1, 12));

            const uint nodeOffset =
                (nodeType == NODE_TYPE_BOX_QUANTIZED_BVH8) ? internalOffset : primitiveOffset;

            const uint nodePtr = nodeOffset | nodeType;

            // Decode bounds
            switch (i)
            {
            case 0:
                node.child0 = nodePtr;
                node.bbox0_min = bbox_min;
                node.bbox0_max = bbox_max;
                break;
            case 1:
                node.child1 = nodePtr;
                node.bbox1_min = bbox_min;
                node.bbox1_max = bbox_max;
                break;
            case 2:
                node.child2 = nodePtr;
                node.bbox2_min = bbox_min;
                node.bbox2_max = bbox_max;
                break;
            case 3:
                node.child3 = nodePtr;
                node.bbox3_min = bbox_min;
                node.bbox3_max = bbox_max;
                break;
            default:
                break;
            }

            if (nodeType == NODE_TYPE_BOX_QUANTIZED_BVH8)
            {
                internalOffset += (nodeRangeLength << 4);
            }
            else
            {
                primitiveOffset += (nodeRangeLength << 4);
            }
        }

        Float32BoxNode node1 = (Float32BoxNode)0;
        node1.child0 = INVALID_NODE;
        node1.child1 = INVALID_NODE;
        node1.child2 = INVALID_NODE;
        node1.child3 = INVALID_NODE;

        IntersectNodeBvh8(node,
                          node1,
                          rayExtent,
                          rayOrigin,
                          rcp(rayDir),
                          BOX_EXPANSION_DEFAULT_AMOUNT,
                          boxSortHeuristic,
                          true,
                          result0,
                          result1);

        // Pack the 8-bit child pointers in the return DWORD from slot 0-3
        nextNodeOffset = (bitFieldExtract(result0.x, 0, 8) << 0)  |
                         (bitFieldExtract(result0.y, 0, 8) << 8)  |
                         (bitFieldExtract(result0.z, 0, 8) << 16) |
                         (bitFieldExtract(result0.w, 0, 8) << 24);

        // Note, this code deliberately omits processing the per-child instance masks since for GPURT BLAS' all the
        // child nodes will have the same instance mask as provided in the HwInstanceTransformNode.
        //
    }
#endif

    const uint64_t blasPtr = GetBlasBaseNodePtr(baseNodePtr, instanceNode.childBasePtr);

    result0.x = 0xDEADBEEF;
    result0.y = 0xDEADBEEF;
    result0.z = LowPart(blasPtr);
    result0.w = HighPart(blasPtr);

    result1.x = 0xDEADBEEF;
    result1.y = 0xDEADBEEF;
    result1.z = instanceNode.userDataAndInstanceMask & 0x00FFFFFF;
    result1.w = nextNodeOffset;
}

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
static uint4 IntersectTriangle3_1(
    GpuVirtualAddress  bvhAddress,
    uint               nodePointer,
    uint               pointerFlags,
    uint               boxSortHeuristic,
    float              rayExtent,
    float3             rayOrigin,
    float3             rayDirection,
    float3             rayDirectionInverse,
    uint               triIndex,
    PrimitiveStructure primStruct)
{
    uint4 result = uint4(-1, -1, -1, -1);

    uint64_t hwNodePtr = (bvhAddress >> 3) + nodePointer;
    const uint64_t pointerFlagsU64 = pointerFlags;
    hwNodePtr |= pointerFlagsU64 << 32;

    const uint pair       = GetPairIndex(nodePointer);
    const bool procedural = primStruct.IsProcedural(pair, triIndex);

    if (procedural == false)
    {
        TriangleData tri = primStruct.UnpackTriangleVertices(pair, triIndex);
        result = fast_intersect_triangle(rayOrigin,
                                         rayDirection,
                                         tri.v0,
                                         tri.v1,
                                         tri.v2);

        result.z = asuint(asfloat(result.z) / asfloat(result.y));
        result.w = asuint(asfloat(result.w) / asfloat(result.y));
    }

    const bool opaque = primStruct.IsOpaque(pair, triIndex);
    PerformTriangleCulling(INTERSECT_RAY_VERSION_3_1,
                           hwNodePtr,
                           opaque,
                           procedural,
                           result);

    const uint primitiveIndex = primStruct.UnpackPrimitiveIndex(pair, triIndex);

    if (procedural)
    {
        result.y = result.z;
        result.z = result.w;
        result.w = (primitiveIndex << 1u) | 0u;
    }
    else
    {
        uint hitkind = result.y >> 31u;
        result.x = asuint(asfloat(result.x) / asfloat(result.y));
        result.y = result.z;
        result.z = result.w;
        result.w = (primitiveIndex << 1u) | hitkind;
    }

    return result;
}
#endif

//=====================================================================================================================
// BVH4x2 dual intersection emulation
static DualIntersectResult image_bvh_dual_intersect_ray(
    in uint64_t         baseNodePtr,
    in uint             node0,
    in uint             node1,
    inout_param(float3) rayOrigin,
    inout_param(float3) rayDir,
    in float            rayExtent,
    in uint             instanceMask,
    in uint             boxSortHeuristic)
{
    DualIntersectResult result;

    uint node0_type = node0 & 7;
    uint node1_type = node1 & 7;

    const uint pointerFlags = uint(baseNodePtr >> NODE_POINTER_FLAGS_SHIFT);

    const uint64_t bvh = ExtractInstanceAddr(baseNodePtr);

    if (node0_type == NODE_TYPE_USER_NODE_INSTANCE) // instance transform
    {
        IntersectHardwareInstanceNode(GPURT_RTIP3_0,
                                      baseNodePtr,
                                      node0,
                                      rayOrigin,
                                      rayDir,
                                      rayExtent,
                                      instanceMask,
                                      boxSortHeuristic,
                                      result.first,
                                      result.second);

        // Do not expect two instance intersections at the same time
        if (node1_type == NODE_TYPE_USER_NODE_INSTANCE)
        {
            result.first  = uint4(-1, -1, -1, -1);
            result.second = uint4(-1, -1, -1, -1);
        }
    }
    else
    {
        result.first  = image_bvh64_intersect_ray_3_0(
            bvh, node0, pointerFlags << 22, boxSortHeuristic, rayExtent, rayOrigin, rayDir, rcp(rayDir));

        result.second = image_bvh64_intersect_ray_3_0(
            bvh, node1, pointerFlags << 22, boxSortHeuristic, rayExtent, rayOrigin, rayDir, rcp(rayDir));

        if (node0_type == 7)
        {
            result.first = uint4(-1, -1, -1, -1);
        }
        if (node1_type == 7)
        {
            result.second = uint4(-1, -1, -1, -1);
        }
    }

    result.geometryId[0] = 0xFFFFFFFF;
    result.geometryId[1] = 0xFFFFFFFF;

    if (IsTriangleNode1_1(node0))
    {
        result.geometryId[0] = FetchTriangleGeometryIndex3_0(bvh, node0);
    }
    if (IsTriangleNode1_1(node1))
    {
        result.geometryId[1] = FetchTriangleGeometryIndex3_0(bvh, node1);
    }

    return result;
}

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
static QuantizedBVH8BoxNode FetchQuantizedBVH8BoxNode(
    in GpuVirtualAddress bvhAddress,
    in uint              nodePointer)

{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    const GpuVirtualAddress nodeAddr = bvhAddress + byteOffset;

    uint4 d0, d1, d2, d3, d4, d5, d6, d7;
    d0 = LoadDwordAtAddrx4(nodeAddr);
    d1 = LoadDwordAtAddrx4(nodeAddr + 0x10);
    d2 = LoadDwordAtAddrx4(nodeAddr + 0x20);
    d3 = LoadDwordAtAddrx4(nodeAddr + 0x30);
    d4 = LoadDwordAtAddrx4(nodeAddr + 0x40);
    d5 = LoadDwordAtAddrx4(nodeAddr + 0x50);
    d6 = LoadDwordAtAddrx4(nodeAddr + 0x60);
    d7 = LoadDwordAtAddrx4(nodeAddr + 0x70);

    QuantizedBVH8BoxNode result;

    result.internalNodeBaseOffset                     = d0.x;
    result.leafNodeBaseOffset                         = d0.y;
    result.parentPointer                              = d0.z;
    result.origin                                     = float3(asfloat(d0.w), asfloat(d1.xy));
    result.exponentsChildIndexAndChildCount           = d1.z;
    result.obbMatrixIndex                             = d1.w & 0x7F;
    result.childInfos[0].minXMinYAndCullingFlags      = d2.x;
    result.childInfos[0].minZMaxXAndInstanceMask      = d2.y;
    result.childInfos[0].maxYMaxZNodeTypeAndNodeRange = d2.z;
    result.childInfos[1].minXMinYAndCullingFlags      = d2.w;
    result.childInfos[1].minZMaxXAndInstanceMask      = d3.x;
    result.childInfos[1].maxYMaxZNodeTypeAndNodeRange = d3.y;
    result.childInfos[2].minXMinYAndCullingFlags      = d3.z;
    result.childInfos[2].minZMaxXAndInstanceMask      = d3.w;
    result.childInfos[2].maxYMaxZNodeTypeAndNodeRange = d4.x;
    result.childInfos[3].minXMinYAndCullingFlags      = d4.y;
    result.childInfos[3].minZMaxXAndInstanceMask      = d4.z;
    result.childInfos[3].maxYMaxZNodeTypeAndNodeRange = d4.w;
    result.childInfos[4].minXMinYAndCullingFlags      = d5.x;
    result.childInfos[4].minZMaxXAndInstanceMask      = d5.y;
    result.childInfos[4].maxYMaxZNodeTypeAndNodeRange = d5.z;
    result.childInfos[5].minXMinYAndCullingFlags      = d5.w;
    result.childInfos[5].minZMaxXAndInstanceMask      = d6.x;
    result.childInfos[5].maxYMaxZNodeTypeAndNodeRange = d6.y;
    result.childInfos[6].minXMinYAndCullingFlags      = d6.z;
    result.childInfos[6].minZMaxXAndInstanceMask      = d6.w;
    result.childInfos[6].maxYMaxZNodeTypeAndNodeRange = d7.x;
    result.childInfos[7].minXMinYAndCullingFlags      = d7.y;
    result.childInfos[7].minZMaxXAndInstanceMask      = d7.z;
    result.childInfos[7].maxYMaxZNodeTypeAndNodeRange = d7.w;

    return result;
}

//=====================================================================================================================
static void PerformInstanceMaskCulling(
    inout_param(Float32BoxNode) node,
    in uint                     instanceMask)
{
    uint packedInstanceMask = node.instanceMask;
    if ((packedInstanceMask & instanceMask) == 0)
    {
        node.child0 = INVALID_NODE;
    }
    packedInstanceMask >>= 8;
    if ((packedInstanceMask & instanceMask) == 0)
    {
        node.child1 = INVALID_NODE;
    }
    packedInstanceMask >>= 8;
    if ((packedInstanceMask & instanceMask) == 0)
    {
        node.child2 = INVALID_NODE;
    }
    packedInstanceMask >>= 8;
    if ((packedInstanceMask & instanceMask) == 0)
    {
        node.child3 = INVALID_NODE;
    }
}
#endif

//=====================================================================================================================
static DualIntersectResult image_bvh8_intersect_ray(
    in uint64_t         baseNodePtr,
    in uint32_t         nodePtr,
    inout_param(float3) rayOrigin,
    inout_param(float3) rayDir,
    in float            rayExtent,
    in uint             instanceMask,
    in uint             boxSortHeuristic)
{
    DualIntersectResult result;
    result.first  = uint4(-1, -1, -1, -1);
    result.second = uint4(-1, -1, -1, -1);
    result.geometryId = uint2(-1, -1);

    const uint nodeOffset = nodePtr >> 3;
    const uint nodeType = nodePtr & 7;

    if (nodeType == 2)// FP32x2
    {
        const uint nodePtr0 = nodePtr;
        const uint nodePtr1 = nodePtr0 + 0x10; // +128 byte offset

        Float32BoxNode node0 = FetchFloat32BoxNode(ExtractInstanceAddr(baseNodePtr), false, nodePtr0);
        Float32BoxNode node1 = FetchFloat32BoxNode(ExtractInstanceAddr(baseNodePtr), false, nodePtr1);

        PerformEarlyBoxCulling(node0, baseNodePtr);
        PerformEarlyBoxCulling(node1, baseNodePtr);

        IntersectNodeBvh8(node0,
                          node1,
                          rayExtent,
                          rayOrigin,
                          rcp(rayDir),
                          BOX_EXPANSION_DEFAULT_AMOUNT,
                          boxSortHeuristic,
                          true,
                          result.first,
                          result.second);
    }
    else if (nodeType == 3)// HPB64x2
    {
        const uint nodePtr0 = nodePtr;
        const uint nodePtr1 = nodePtr0 + 0x8; // +64 byte offset

        Float32BoxNode node0 = FetchFloat32BoxNode(ExtractInstanceAddr(baseNodePtr), true, nodePtr0);
        Float32BoxNode node1 = FetchFloat32BoxNode(ExtractInstanceAddr(baseNodePtr), true, nodePtr1);

        PerformEarlyBoxCulling(node0, baseNodePtr);
        PerformEarlyBoxCulling(node1, baseNodePtr);

        IntersectNodeBvh8(node0,
                          node1,
                          rayExtent,
                          rayOrigin,
                          rcp(rayDir),
                          BOX_EXPANSION_DEFAULT_AMOUNT,
                          boxSortHeuristic,
                          true,
                          result.first,
                          result.second);
    }
    else
    {
        // nodeType: 0, 1, 4, 5, 6
        const uint nodePtr0 = (nodeType == 1) ? nodePtr - 1 : nodePtr;
        const uint nodePtr1 = (nodeType == 1) ? nodePtr : INVALID_NODE;

        result = image_bvh_dual_intersect_ray(baseNodePtr,
                                              nodePtr0,
                                              nodePtr1,
                                              rayOrigin,
                                              rayDir,
                                              rayExtent,
                                              instanceMask,
                                              boxSortHeuristic);
    }

    return result;
}

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
// RTIP3.1 equivalent emulation of image_bvh8_intersect_ray (T#.compressed_format_enable = 1)
//
static Bvh8IntersectResult image_bvh8_intersect_ray_3_1(
    in uint64_t         baseNodePtr,
    in uint32_t         nodePtr,
    inout_param(float3) rayOrigin,
    inout_param(float3) rayDir,
    in float            rayExtent,
    in uint             instanceMask,
    in uint             boxSortHeuristic)
{
    Bvh8IntersectResult result = (Bvh8IntersectResult) -1;
    // The node type field is 4-bits, but triangle pair type enumeration is such that the lower 3 bits between
    // pair0-3 and pair4-7 are identical.
    const uint nodeType = (nodePtr & 7);

    if (nodeType == 5) // bvh8_box_128
    {
        Float32BoxNode node0 = (Float32BoxNode)0;
        Float32BoxNode node1 = (Float32BoxNode)0;
        uint parentPointer;

        QuantizedBVH8BoxNode node = FetchQuantizedBVH8BoxNode(ExtractInstanceAddr(baseNodePtr), nodePtr);
        node.Decode(node0, node1, parentPointer);

        PerformEarlyBoxCulling(node0, baseNodePtr);
        PerformEarlyBoxCulling(node1, baseNodePtr);

        PerformInstanceMaskCulling(node0, instanceMask);
        PerformInstanceMaskCulling(node1, instanceMask);

        // OBB Transform
        float3 localRayOrigin = rayOrigin;
        float3 localRayDir = rayDir;
        if (node.obbMatrixIndex != INVALID_OBB && IsOrientedBoundingBoxesEnabled())
        {
            float3x3 obbMatrix = OBBIndexToMatrix(node.obbMatrixIndex);

            OBBTransform(obbMatrix,
                         rayOrigin,
                         rayDir,
                         localRayOrigin,
                         localRayDir);
        }

        IntersectNodeBvh8(node0,
                          node1,
                          rayExtent,
                          localRayOrigin,
                          rcp(localRayDir),
                          BOX_EXPANSION_DEFAULT_AMOUNT,
                          boxSortHeuristic,
                          true,
                          result.slot0,
                          result.slot1);
    }
    else if (nodeType == 6) // Instance node
    {
        IntersectHardwareInstanceNode(GPURT_RTIP3_1,
                                      baseNodePtr,
                                      nodePtr,
                                      rayOrigin,
                                      rayDir,
                                      rayExtent,
                                      instanceMask,
                                      boxSortHeuristic,
                                      result.slot0,
                                      result.slot1);
    }
    // primitive structure nodes types are 0, 1, 2, 3
    else if (IsTriangleNode3_1(nodePtr))
    {
        const uint64_t bvh = ExtractInstanceAddr(baseNodePtr);
        const uint pointerFlags = uint(baseNodePtr >> NODE_POINTER_FLAGS_SHIFT);
        INIT_VAR(PrimitiveStructure, primStruct);
        FetchPrimitiveStructFromBvh(bvh, nodePtr, primStruct);

        result.slot0 = IntersectTriangle3_1(bvh,
                                            nodePtr,
                                            pointerFlags << 22,
                                            boxSortHeuristic,
                                            rayExtent,
                                            rayOrigin,
                                            rayDir,
                                            rcp(rayDir),
                                            0,
                                            primStruct);

        const uint pair = GetPairIndex(nodePtr);
        TrianglePairDesc pairDesc = primStruct.ReadTrianglePairDesc(pair);
        if (pairDesc.Tri1Valid())
        {
            result.slot1 = IntersectTriangle3_1(bvh,
                                                nodePtr,
                                                pointerFlags << 22,
                                                boxSortHeuristic,
                                                rayExtent,
                                                rayOrigin,
                                                rayDir,
                                                rcp(rayDir),
                                                1,
                                                primStruct);
        }
        else
        {
            result.slot1.x = asuint(INFINITY);
            result.slot1.y = asuint(1.0f);
            result.slot1.z = asuint(1.0f);
            result.slot1.w = 0;
        }

        const uint navigationBits = primStruct.CalcNavigationBits(pair);
        const uint2 geometryIndex = { primStruct.UnpackGeometryIndex(pair, 0),
                                      primStruct.UnpackGeometryIndex(pair, 1) };
        result.ext.x = (geometryIndex.x << 2u) | navigationBits;
        result.ext.y = (geometryIndex.y << 2u) | navigationBits;
    }

    return result;
}
#endif

#endif
