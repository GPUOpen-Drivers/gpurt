/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2021-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
//
#ifndef _GPURT_LIBRARY_HLSL
#define _GPURT_LIBRARY_HLSL

// Following order matters as AccelStructTracker relies on defines from TraceRayCommon.hlsl
#include "TraceRayCommon.hlsl"
#include "AccelStructTracker.hlsl"

#include "RayQuery.hlsl"
#include "TraceRay.hlsl"

//=====================================================================================================================
// TraceRay() entry point for ray tracing IP 1.1
export void TraceRay1_1(
    uint  accelStructLo,
    uint  accelStructHi,
    uint  rayFlags,
    uint  instanceInclusionMask,
    uint  rayContributionToHitGroupIndex,
    uint  multiplierForGeometryContributionToShaderIndex,
    uint  missShaderIndex,
    float originX,
    float originY,
    float originZ,
    float tMin,
    float dirX,
    float dirY,
    float dirZ,
    float tMax)
{
    TraceRayCommon(
        accelStructLo,
        accelStructHi,
        rayFlags,
        instanceInclusionMask,
        rayContributionToHitGroupIndex,
        multiplierForGeometryContributionToShaderIndex,
        missShaderIndex,
        originX,
        originY,
        originZ,
        tMin,
        dirX,
        dirY,
        dirZ,
        tMax,
        0,
        0,
        true,
        GPURT_RTIP1_1
    );
}

//=====================================================================================================================
// TraceRay() hit-token extension entry point for ray tracing IP 1.1
export void TraceRayUsingHitToken1_1(
    uint  accelStructLo,
    uint  accelStructHi,
    uint  rayFlags,
    uint  instanceInclusionMask,
    uint  rayContributionToHitGroupIndex,
    uint  multiplierForGeometryContributionToShaderIndex,
    uint  missShaderIndex,
    float originX,
    float originY,
    float originZ,
    float tMin,
    float dirX,
    float dirY,
    float dirZ,
    float tMax,
    uint  blasPointer,
    uint  tlasPointer)
{
    TraceRayCommon(
        accelStructLo,
        accelStructHi,
        rayFlags,
        instanceInclusionMask,
        rayContributionToHitGroupIndex,
        multiplierForGeometryContributionToShaderIndex,
        missShaderIndex,
        originX,
        originY,
        originZ,
        tMin,
        dirX,
        dirY,
        dirZ,
        tMax,
        blasPointer,
        tlasPointer,
        false,
        GPURT_RTIP1_1
    );
}

#if GPURT_BUILD_RTIP2
//=====================================================================================================================
// TraceRay() entry point for ray tracing IP 2.0
export void TraceRay2_0(
    uint  accelStructLo,
    uint  accelStructHi,
    uint  rayFlags,
    uint  instanceInclusionMask,
    uint  rayContributionToHitGroupIndex,
    uint  multiplierForGeometryContributionToShaderIndex,
    uint  missShaderIndex,
    float originX,
    float originY,
    float originZ,
    float tMin,
    float dirX,
    float dirY,
    float dirZ,
    float tMax)
{
    TraceRayCommon(
        accelStructLo,
        accelStructHi,
        rayFlags,
        instanceInclusionMask,
        rayContributionToHitGroupIndex,
        multiplierForGeometryContributionToShaderIndex,
        missShaderIndex,
        originX,
        originY,
        originZ,
        tMin,
        dirX,
        dirY,
        dirZ,
        tMax,
        0,
        0,
        true,
        GPURT_RTIP2_0
    );
}

//=====================================================================================================================
// TraceRay() hit-token extension entry point for ray tracing IP 2.0
export void TraceRayUsingHitToken2_0(
    uint  accelStructLo,
    uint  accelStructHi,
    uint  rayFlags,
    uint  instanceInclusionMask,
    uint  rayContributionToHitGroupIndex,
    uint  multiplierForGeometryContributionToShaderIndex,
    uint  missShaderIndex,
    float originX,
    float originY,
    float originZ,
    float tMin,
    float dirX,
    float dirY,
    float dirZ,
    float tMax,
    uint  blasPointer,
    uint  tlasPointer)
{
    TraceRayCommon(
        accelStructLo,
        accelStructHi,
        rayFlags,
        instanceInclusionMask,
        rayContributionToHitGroupIndex,
        multiplierForGeometryContributionToShaderIndex,
        missShaderIndex,
        originX,
        originY,
        originZ,
        tMin,
        dirX,
        dirY,
        dirZ,
        tMax,
        blasPointer,
        tlasPointer,
        false,
        GPURT_RTIP2_0
    );
}
#endif

//=====================================================================================================================
// RayQuery::Proceed() entry point for ray tracing IP 1.1
export bool RayQueryProceed1_1(
    inout_param(RayQueryInternal) rayQuery,
    in    uint                    constRayFlags,
    in    uint3                   dispatchThreadId)
{
    return RayQueryProceedCommon(
        rayQuery,
        constRayFlags,
        dispatchThreadId,
        GPURT_RTIP1_1
    );
}

#if GPURT_BUILD_RTIP2
//=====================================================================================================================
// RayQuery::Proceed() entry point for ray tracing IP 2.0
export bool RayQueryProceed2_0(
    inout_param(RayQueryInternal) rayQuery,
    in    uint                    constRayFlags,
    in    uint3                   dispatchThreadId)
{
    return RayQueryProceedCommon(
        rayQuery,
        constRayFlags,
        dispatchThreadId,
        GPURT_RTIP2_0
    );
}
#endif

//=====================================================================================================================
// TraceRayInline() entry point for ray tracing IP 1.1
export void TraceRayInline1_1(
    inout_param(RayQueryInternal) rayQuery,
    in    uint                    accelStructLo,
    in    uint                    accelStructHi,
    in    uint                    constRayFlags,
    in    uint                    rayFlags,
    in    uint                    instanceMask,
    in    RayDesc                 rayDesc,
    in    uint3                   dispatchThreadId)
{
    TraceRayInlineCommon(rayQuery,
                         accelStructLo,
                         accelStructHi,
                         constRayFlags,
                         rayFlags,
                         instanceMask,
                         rayDesc,
                         dispatchThreadId,
                         GPURT_RTIP1_1);
}

#if GPURT_BUILD_RTIP2
//=====================================================================================================================
// TraceRayInline() entry point for ray tracing IP 2.0
export void TraceRayInline2_0(
    inout_param(RayQueryInternal) rayQuery,
    in    uint                    accelStructLo,
    in    uint                    accelStructHi,
    in    uint                    constRayFlags,
    in    uint                    rayFlags,
    in    uint                    instanceMask,
    in    RayDesc                 rayDesc,
    in    uint3                   dispatchThreadId)
{
    TraceRayInlineCommon(rayQuery,
                         accelStructLo,
                         accelStructHi,
                         constRayFlags,
                         rayFlags,
                         instanceMask,
                         rayDesc,
                         dispatchThreadId,
                         GPURT_RTIP2_0);
}
#endif

//=====================================================================================================================
// GPURT intrinsic for fetching instance ID from instance node
export uint GetInstanceID(
    in uint64_t instanceNodePtr) // 64-bit instance node address
{
    const uint instanceIdAndMask = LoadDwordAtAddr(instanceNodePtr + INSTANCE_DESC_ID_AND_MASK_OFFSET);
    return (instanceIdAndMask & 0x00ffffff);
}

//=====================================================================================================================
// GPURT intrinsic for fetching instance index from instance node
export uint GetInstanceIndex(
    in uint64_t instanceNodePtr) // 64-bit instance node address
{
    return LoadDwordAtAddr(instanceNodePtr + sizeof(InstanceDesc) + RTIP1_1_INSTANCE_SIDEBAND_INSTANCE_INDEX_OFFSET);
}

//=====================================================================================================================
// GPURT intrinsic for fetching object to world transform matrix from instance node
export float GetObjectToWorldTransform(
    in uint64_t instanceNodePtr, // 64-bit instance node address
    in uint32_t row,             // row index
    in uint32_t col)             // column index
{
    const uint32_t elementOffset = (row * sizeof(float4)) + (col * sizeof(float));
    return asfloat(LoadDwordAtAddr(instanceNodePtr +
                                   sizeof(InstanceDesc) +
                                   RTIP1_1_INSTANCE_SIDEBAND_OBJECT2WORLD_OFFSET +
                                   elementOffset));
}

//=====================================================================================================================
// GPURT intrinsic for fetching world to object transform matrix (float3x4) from instance node
export float GetWorldToObjectTransform(
    in uint64_t instanceNodePtr, // 64-bit instance node address
    in uint32_t row,             // row index
    in uint32_t col)             // column index
{
    const uint32_t elementOffset = (row * sizeof(float4)) + (col * sizeof(float));
    return asfloat(LoadDwordAtAddr(instanceNodePtr + INSTANCE_DESC_WORLD_TO_OBJECT_XFORM_OFFSET + elementOffset));
}

//=====================================================================================================================
// GPURT function for fetching 64-bit instance node address used in RayQuery
export uint64_t GetRayQuery64BitInstanceNodePtr(
    in uint64_t tlasBaseAddr,     // 64-bit TLAS base address
    in uint32_t instanceNodePtr)  // Instance node pointer
{
    return CalculateNodeAddr64(tlasBaseAddr, instanceNodePtr);
}

//=====================================================================================================================
// GPURT intrinsic for fetching triangle position from given BVH address and node pointer
export TriangleData FetchTrianglePositionFromNodePointer(
    in GpuVirtualAddress bvhAddress,  // BVH address
    in uint              nodePointer) // Node pointer
{
    return FetchTriangleFromNode(bvhAddress, nodePointer);
}

//=====================================================================================================================
// GPURT intrinsic for fetching triangle position from given ray query object
export TriangleData FetchTrianglePositionFromRayQuery(
    inout_param(RayQueryInternal) rayQuery,  // BVH address
    in bool                       committed) // Node pointer
{
    GpuVirtualAddress bvhAddress =
        MakeGpuVirtualAddress(rayQuery.bvhLo, (rayQuery.bvhHi & POINTER_FLAGS_EXCLUDED_MASK));
    uint nodePointer = committed ? rayQuery.committed.currNodePtr : rayQuery.candidate.currNodePtr;

    return FetchTriangleFromNode(bvhAddress, nodePointer);
}

#endif
