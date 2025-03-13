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

#ifndef RTIP3_OPS_HLSL
#define RTIP3_OPS_HLSL

#include "rtip3_x.h"

#if USE_HW_INTRINSIC
//=====================================================================================================================
static DualIntersectResult image_bvh8_intersect_ray(
    in uint64_t         baseNodePtr,
    in uint             node,
    inout_param(float3) rayOrigin,
    inout_param(float3) rayDir,
    in float            rayExtent,
    in uint             instanceMask,
    in uint             boxSortHeuristic)
{
    return AmdTraceRayIntersectRayBvh8(
        baseNodePtr, rayOrigin, rayDir, rayExtent, instanceMask, boxSortHeuristic, node);
}
#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
static Bvh8IntersectResult image_bvh8_intersect_ray_3_1(
    in uint64_t         baseNodePtr,
    in uint             node,
    inout_param(float3) rayOrigin,
    inout_param(float3) rayDir,
    in float            rayExtent,
    in uint             instanceMask,
    in uint             boxSortHeuristic)
{
    return Bvh8IntersectResult(AmdTraceRayIntersectRayBvh8(
        baseNodePtr, rayOrigin, rayDir, rayExtent, instanceMask, boxSortHeuristic, node));
}
#endif
//=====================================================================================================================
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
    return AmdTraceRayDualIntersectRay(
        baseNodePtr, rayOrigin, rayDir, rayExtent, instanceMask, boxSortHeuristic, node0, node1);
}

#else
#include "rtip3_opcode_emulation.hlsl"
#endif

#if USE_HW_INTRINSIC
//=====================================================================================================================
static uint ds_stack_push8_pop1(
    inout_param(uint) packedStackAddr,
    in uint           lastNodePtr,
    in uint4          nextNodes0,
    in uint4          nextNodes1,
    in uint           rtipLevel)
{
#if GPURT_BUILD_RTIP3_1
    if (rtipLevel >= GPURT_RTIP3_1)
    {
        // TODO: Add handling of AmdTraceRayDsStackPush8Pop1PrimRangeEnabled to dxcp
        return AmdTraceRayDsStackPush8Pop1PrimRangeEnabled(packedStackAddr,
                                                           lastNodePtr,
                                                           nextNodes0,
                                                           nextNodes1);
    }
    else
#endif
    {
        return AmdTraceRayDsStackPush8Pop1(packedStackAddr, lastNodePtr, nextNodes0, nextNodes1);
    }
}

//=====================================================================================================================
static uint2 ds_stack_push8_pop2(
    inout_param(uint) packedStackAddr,
    in uint           lastNodePtr,
    in uint4          nextNodes0,
    in uint4          nextNodes1)
{
    return AmdTraceRayDsStackPush8Pop2(packedStackAddr, lastNodePtr, nextNodes0, nextNodes1);
}

//=====================================================================================================================
static uint RtIp3LdsStackInit()
{
    return AmdTraceRayLdsStackInit();
}

#else
#include "rtip3_ds_stack_emulation.hlsl"
#define ds_stack_push8_pop2 ds_stack_push8_pop2_emulation
#define ds_stack_push8_pop1 ds_stack_push8_pop1_emulation
#define RtIp3LdsStackInit   RtIp3LdsStackInit_emulation
#endif

#endif
