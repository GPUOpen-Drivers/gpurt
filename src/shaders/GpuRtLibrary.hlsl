/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2021-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

#include "../shadersClean/traversal/TraversalDefs.hlsli"
#include "../shadersClean/common/InstanceDesc.hlsli"

// Following order matters as AccelStructTracker relies on defines from TraceRayCommon.hlsl
#include "TraceRayCommon.hlsl"
#include "AccelStructTracker.hlsl"

#ifdef __cplusplus
extern uint g_rtIpLevel;          // defined in cputraversal
void _AmdSetRtip(uint rtIpLevel); // defined in cputraversal
#endif

// Only the default path (Continuation) provides _AmdGetRtip().
static RayTracingIpLevel GetRtIpLevel()
{
#ifdef __cplusplus
    switch (g_rtIpLevel)
    {
    case GPURT_RTIP1_1:
        return RayTracingIpLevel::RtIp1_1;
    case GPURT_RTIP2_0:
        return RayTracingIpLevel::RtIp2_0;
    default:
        // Should never be called
        GPU_ASSERT(false);
        return RayTracingIpLevel::_None;
    }
#else // __cplusplus
#if GPURT_DEBUG_CONTINUATION_TRAVERSAL
    if (GPURT_RTIP_LEVEL == (uint)RayTracingIpLevel::_None)
    {
        return RayTracingIpLevel::_None;
    }
    return RayTracingIpLevel::RtIp2_0; //default to ip 2.0
#else // GPURT_DEBUG_CONTINUATION_TRAVERSAL
    return _AmdGetRtip(); // Continuation path
#endif
#endif
}

#if GPURT_BUILD_CONTINUATION && LLPC_CLIENT_INTERFACE_MAJOR_VERSION
// Include the continuations library
#include "GpuRtLibraryCont.hlsl"
#endif

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

//=====================================================================================================================
// GPURT intrinsic for fetching instance ID from instance node for ray tracing IP 1.1/2.0
export uint GetInstanceID(
    in uint64_t instanceNodePtr) // 64-bit instance node address
{
    uint instanceId = 0;
    if (instanceNodePtr != 0)
    {
        const uint instanceIdAndMask = LoadDwordAtAddr(instanceNodePtr + INSTANCE_DESC_ID_AND_MASK_OFFSET);
        instanceId = (instanceIdAndMask & 0x00ffffff);
    }
    return instanceId;
}

//=====================================================================================================================
// GPURT intrinsic for fetching instance index from instance node for ray tracing IP 1.1/2.0
export uint GetInstanceIndex(
    in uint64_t instanceNodePtr) // 64-bit instance node address
{
    uint instanceIndex = 0;
    if (instanceNodePtr != 0)
    {
        instanceIndex = LoadDwordAtAddr(instanceNodePtr + sizeof(InstanceDesc) +
                                        RTIP1_1_INSTANCE_SIDEBAND_INSTANCE_INDEX_OFFSET);
    }
    return instanceIndex;
}

//=====================================================================================================================
// GPURT intrinsic for fetching object to world transform matrix from instance node
export float GetObjectToWorldTransform(
    in uint64_t instanceNodePtr, // 64-bit instance node address
    in uint32_t row,             // row index
    in uint32_t col)             // column index
{
    float transform = 0;
    if (instanceNodePtr != 0)
    {
        const uint32_t elementOffset = (row * sizeof(float4)) + (col * sizeof(float));
        transform = asfloat(LoadDwordAtAddr(instanceNodePtr +
                                            sizeof(InstanceDesc) +
                                            RTIP1_1_INSTANCE_SIDEBAND_OBJECT2WORLD_OFFSET +
                                            elementOffset));
    }
    return transform;
}

//=====================================================================================================================
// GPURT intrinsic for fetching world to object transform matrix (float3x4) from instance node
export float GetWorldToObjectTransform(
    in uint64_t instanceNodePtr, // 64-bit instance node address
    in uint32_t row,             // row index
    in uint32_t col)             // column index
{
    float transform = 0;
    if (instanceNodePtr != 0)
    {
        const uint32_t elementOffset = (row * sizeof(float4)) + (col * sizeof(float));
        transform = asfloat(LoadDwordAtAddr(instanceNodePtr + INSTANCE_DESC_WORLD_TO_OBJECT_XFORM_OFFSET +
                                            elementOffset));
    }
    return transform;
}

//=====================================================================================================================
// Return a ObjectToWorld transform matrix (float3x4) from instance node
static float3x4 GetObjectToWorld3x4(
    in uint64_t instanceNodePtr)
{
    float3x4 transform = (float3x4)0;

    if (instanceNodePtr != 0)
    {
        switch (GetRtIpLevel())
        {
        default:
        {
            const uint offset = RTIP1_1_INSTANCE_SIDEBAND_OBJECT2WORLD_OFFSET;
            transform[0] = asfloat(ConstantLoadDwordAtAddrx4(instanceNodePtr + sizeof(InstanceDesc) + offset + 0));
            transform[1] = asfloat(ConstantLoadDwordAtAddrx4(instanceNodePtr + sizeof(InstanceDesc) + offset + 16));
            transform[2] = asfloat(ConstantLoadDwordAtAddrx4(instanceNodePtr + sizeof(InstanceDesc) + offset + 32));
            break;
        }
        }
    }

    return transform;
}

//=====================================================================================================================
// Return a WorldToObject transform matrix (float3x4) from instance node
static float3x4 GetWorldToObject3x4(
    in uint64_t instanceNodePtr)
{
    float3x4 transform = (float3x4)0;

    if (instanceNodePtr != 0)
    {
        switch (GetRtIpLevel())
        {
        default:
        {
            const uint offset = INSTANCE_DESC_WORLD_TO_OBJECT_XFORM_OFFSET;

            transform[0] = asfloat(ConstantLoadDwordAtAddrx4(instanceNodePtr + offset + 0));
            transform[1] = asfloat(ConstantLoadDwordAtAddrx4(instanceNodePtr + offset + 16));
            transform[2] = asfloat(ConstantLoadDwordAtAddrx4(instanceNodePtr + offset + 32));

            break;
        }
        }
    }

    return transform;
}

//=====================================================================================================================
// Return a ObjectToWorld transform matrix (float4x3) from instance node
static float4x3 GetObjectToWorld4x3(
    in uint64_t instanceNodePtr)
{
    return transpose(GetObjectToWorld3x4(instanceNodePtr));
}

//=====================================================================================================================
// Return a WorldToObject transform matrix (float4x3) from instance node
static float4x3 GetWorldToObject4x3(
    in uint64_t instanceNodePtr)
{
    return transpose(GetWorldToObject3x4(instanceNodePtr));
}

//=====================================================================================================================
// GPURT function for fetching 64-bit instance node address used in RayQuery
export uint64_t GetRayQuery64BitInstanceNodePtr(
    in uint64_t tlasBaseAddr,     // 64-bit TLAS base address
    in uint32_t instanceNodePtr)  // Instance node pointer
{
    uint64_t nodeAddr = 0;
    if (instanceNodePtr != 0)
    {
        nodeAddr = CalculateNodeAddr64(tlasBaseAddr, instanceNodePtr);
    }
    return nodeAddr;
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

//=====================================================================================================================
// Internal General function for fetching instance ID from instance node for different ray tracing IP
static uint GetGeneralInstanceID(
    in uint64_t instNodeAddr) // 64-bit instance node address
{
    uint id = 0;
    switch (GetRtIpLevel())
    {
    default:
    {
        // For RtIp2.0 and Rtip1.1
        id = GetInstanceID(instNodeAddr);
        break;
    }
    }
    return id;
}

//=====================================================================================================================
// Internal General function for fetching instance Index from instance node for different ray tracing IP
static uint GetGeneralInstanceIndex(
    in uint64_t instNodeAddr) // 64-bit instance node address
{
    uint index = 0;
    RayTracingIpLevel rtip = GetRtIpLevel();
    switch (rtip)
    {
    default:
    {
        // For RtIp2.0 and Rtip1.1
        index = GetInstanceIndex(instNodeAddr);
        break;
    }
    }
    return index;
}

//=====================================================================================================================
// Internal GPURT function for fetching 64-bit instance node pointer used in RayQuery for different ray tracing IP
static uint64_t GetRayQueryInstanceNodePtr(
    in uint64_t tlasBaseAddr,     // 64-bit TLAS base address
    in uint32_t instanceNodePtr)  // Instance node pointer
{
    uint64_t instNodePtr = 0;
    RayTracingIpLevel rtip = GetRtIpLevel();
    switch (rtip)
    {
    default:
    {
        instNodePtr = GetRayQuery64BitInstanceNodePtr(tlasBaseAddr, instanceNodePtr);
        break;
    }
    }
    return instNodePtr;
}
//=====================================================================================================================
// Allocate ray query object. This is a notification for client drivers for querying ray query object size.
export RayQueryInternal _RayQuery_Allocate()
{
    return (RayQueryInternal)0;
}

//=====================================================================================================================
// Abort traversal on next call to Proceed()
export void _RayQuery_Abort(
    inout_param(RayQueryInternal) rayQuery)
{
    uint rtIp = (uint)GetRtIpLevel();
    if (rtIp >= (uint)RayTracingIpLevel::RtIp2_0)
    {
        rayQuery.currNodePtr = TERMINAL_NODE;
    }
    else
    {
        rayQuery.stackPtr        = AmdTraceRayGetStackBase();
        rayQuery.stackNumEntries = 0;
        rayQuery.currNodePtr     = INVALID_NODE;
    }
}

//=====================================================================================================================
// Interleaved to call proceed()
export void _RayQuery_EndInterleavedProceed(
    inout_param(RayQueryInternal) rayQuery)
{
    rayQuery.stackNumEntries = 0;
}

//=====================================================================================================================
// Commit a non-opaque triangle hit
export void _RayQuery_CommitNonOpaqueTriangleHit(
    inout_param(RayQueryInternal) rayQuery)
{
    if (rayQuery.candidateType == CANDIDATE_NON_OPAQUE_TRIANGLE)
    {
        rayQuery.committedStatus = COMMITTED_TRIANGLE_HIT;
        rayQuery.committed       = rayQuery.candidate;
    }
}

//=====================================================================================================================
// Commit procedural primitive hit
export void _RayQuery_CommitProceduralPrimitiveHit(
    inout_param(RayQueryInternal) rayQuery,
    float                         tHit)
{
    if (rayQuery.candidateType > CANDIDATE_NON_OPAQUE_TRIANGLE)
    {
        rayQuery.committedStatus       = COMMITTED_PROCEDURAL_PRIMITIVE_HIT;
        rayQuery.committed             = rayQuery.candidate;
        rayQuery.committed.rayTCurrent = tHit - rayQuery.rayTMin;
    }
}

//=====================================================================================================================
// Return committed status
export uint _RayQuery_CommittedStatus(
    in RayQueryInternal rayQuery)
{
    return rayQuery.committedStatus;
}

//=====================================================================================================================
// Return candidate type
export uint _RayQuery_CandidateType(
    inout_param(RayQueryInternal) rayQuery,
    in uint                       constRayFlags)
{
    if ((constRayFlags & RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES) != 0)
    {
        rayQuery.candidateType = CANDIDATE_NON_OPAQUE_TRIANGLE;
    }

    return min(rayQuery.candidateType, uint(CANDIDATE_PROCEDURAL_PRIMITIVE));
}

//=====================================================================================================================
// Return candidate ObjectToWorld transform
export float3x4 _RayQuery_CandidateObjectToWorld3x4(
    in RayQueryInternal rayQuery)
{
    const uint64_t topLevelAddr = PackUint64(rayQuery.topLevelBvhLo, rayQuery.topLevelBvhHi);
    const uint64_t instNodeAddr = GetRayQueryInstanceNodePtr(topLevelAddr, rayQuery.candidate.instNodePtr);
    return GetObjectToWorld3x4(instNodeAddr);
}

//=====================================================================================================================
// Return candidate WorldToObject transform
export float3x4 _RayQuery_CandidateWorldToObject3x4(
    in RayQueryInternal rayQuery)
{
    const uint64_t topLevelAddr = PackUint64(rayQuery.topLevelBvhLo, rayQuery.topLevelBvhHi);
    const uint64_t instNodeAddr = GetRayQueryInstanceNodePtr(topLevelAddr, rayQuery.candidate.instNodePtr);
    return GetWorldToObject3x4(instNodeAddr);
}

//=====================================================================================================================
// Return committed ObjectToWorld transform
export float3x4 _RayQuery_CommittedObjectToWorld3x4(
    in RayQueryInternal rayQuery)
{
    const uint64_t topLevelAddr = PackUint64(rayQuery.topLevelBvhLo, rayQuery.topLevelBvhHi);
    const uint64_t instNodeAddr = GetRayQueryInstanceNodePtr(topLevelAddr, rayQuery.committed.instNodePtr);
    return GetObjectToWorld3x4(instNodeAddr);
}

//=====================================================================================================================
// Return committed WorldToObject transform
export float3x4 _RayQuery_CommittedWorldToObject3x4(
    in RayQueryInternal rayQuery)
{
    const uint64_t topLevelAddr = PackUint64(rayQuery.topLevelBvhLo, rayQuery.topLevelBvhHi);
    const uint64_t instNodeAddr = GetRayQueryInstanceNodePtr(topLevelAddr, rayQuery.committed.instNodePtr);
    return GetWorldToObject3x4(instNodeAddr);
}

//=====================================================================================================================
// Return whether candidate procedural primitive is opaque or non-opaque
export bool _RayQuery_CandidateProceduralPrimitiveNonOpaque(
    in RayQueryInternal rayQuery)
{
    return (rayQuery.candidateType == CANDIDATE_NON_OPAQUE_PROCEDURAL_PRIMITIVE);
}

//=====================================================================================================================
// Return whether candidate triangle node is front facing
export bool _RayQuery_CandidateTriangleFrontFace(
    in RayQueryInternal rayQuery)
{
    return rayQuery.candidate.frontFace;
}

//=====================================================================================================================
// Return whether committed triangle node is front facing
export bool _RayQuery_CommittedTriangleFrontFace(
    in RayQueryInternal rayQuery)
{
    return rayQuery.committed.frontFace;
}

//=====================================================================================================================
// Return candidate triangle node barycentrics
export float2 _RayQuery_CandidateTriangleBarycentrics(
    in RayQueryInternal rayQuery)
{
    return rayQuery.candidate.barycentrics;
}

//=====================================================================================================================
// Return committed triangle node barycentrics
export float2 _RayQuery_CommittedTriangleBarycentrics(
    in RayQueryInternal rayQuery)
{
    return rayQuery.committed.barycentrics;
}

//=====================================================================================================================
// Return ray flags
export uint _RayQuery_RayFlags(
    in RayQueryInternal rayQuery)
{
    return rayQuery.rayFlags;
}

//=====================================================================================================================
// Return world space ray origin
export float3 _RayQuery_WorldRayOrigin(
    in RayQueryInternal rayQuery)
{
    return rayQuery.rayDesc.Origin;
}

//=====================================================================================================================
// Return world space ray direction
export float3 _RayQuery_WorldRayDirection(
    in RayQueryInternal rayQuery)
{
    return rayQuery.rayDesc.Direction;
}

//=====================================================================================================================
// Return ray tMin value
export float _RayQuery_RayTMin(
    in RayQueryInternal rayQuery)
{
    return rayQuery.rayTMin;
}

//=====================================================================================================================
// Return candidate triangle hit T
export float _RayQuery_CandidateTriangleRayT(
    inout_param(RayQueryInternal) rayQuery)
{
    rayQuery.candidate.rayTCurrent += rayQuery.rayTMin;
    return rayQuery.candidate.rayTCurrent;
}

//=====================================================================================================================
// Return committed hit T
export float _RayQuery_CommittedRayT(
    inout_param(RayQueryInternal) rayQuery)
{
    rayQuery.committed.rayTCurrent += rayQuery.rayTMin;
    return rayQuery.committed.rayTCurrent;
}

//=====================================================================================================================
// Return candidate instance index
export uint _RayQuery_CandidateInstanceIndex(
    in RayQueryInternal rayQuery)
{
    const uint64_t topLevelAddr = PackUint64(rayQuery.topLevelBvhLo, rayQuery.topLevelBvhHi);
    const uint64_t instNodeAddr = GetRayQueryInstanceNodePtr(topLevelAddr, rayQuery.candidate.instNodePtr);
    return GetGeneralInstanceIndex(instNodeAddr);
}

//=====================================================================================================================
// Return committed instance index
export uint _RayQuery_CommittedInstanceIndex(
    in RayQueryInternal rayQuery)
{
    const uint64_t topLevelAddr = PackUint64(rayQuery.topLevelBvhLo, rayQuery.topLevelBvhHi);
    const uint64_t instNodeAddr = GetRayQueryInstanceNodePtr(topLevelAddr, rayQuery.committed.instNodePtr);
    return GetGeneralInstanceIndex(instNodeAddr);
}

//=====================================================================================================================
// Return candidate instance ID
export uint _RayQuery_CandidateInstanceID(
    in RayQueryInternal rayQuery)
{
    const uint64_t topLevelAddr = PackUint64(rayQuery.topLevelBvhLo, rayQuery.topLevelBvhHi);
    const uint64_t instNodeAddr = GetRayQueryInstanceNodePtr(topLevelAddr, rayQuery.candidate.instNodePtr);
    return GetGeneralInstanceID(instNodeAddr);
}

//=====================================================================================================================
// Return committed instance ID
export uint _RayQuery_CommittedInstanceID(
    in RayQueryInternal rayQuery)
{
    const uint64_t topLevelAddr = PackUint64(rayQuery.topLevelBvhLo, rayQuery.topLevelBvhHi);
    const uint64_t instNodeAddr = GetRayQueryInstanceNodePtr(topLevelAddr, rayQuery.committed.instNodePtr);
    return GetGeneralInstanceID(instNodeAddr);
}

//=====================================================================================================================
// Return candidate geometry index
export uint _RayQuery_CandidateGeometryIndex(
    in RayQueryInternal rayQuery)
{
    return rayQuery.candidate.geometryIndex;
}

//=====================================================================================================================
// Return committed geometry index
export uint _RayQuery_CommittedGeometryIndex(
    in RayQueryInternal rayQuery)
{
    return rayQuery.committed.geometryIndex;
}

//=====================================================================================================================
// Return candidate primitive index
export uint _RayQuery_CandidatePrimitiveIndex(
    in RayQueryInternal rayQuery)
{
    return rayQuery.candidate.primitiveIndex;
}

//=====================================================================================================================
// Return committed primitive index
export uint _RayQuery_CommittedPrimitiveIndex(
    in RayQueryInternal rayQuery)
{
    return rayQuery.committed.primitiveIndex;
}

//=====================================================================================================================
// Return candidate object space ray origin
export float3 _RayQuery_CandidateObjectRayOrigin(
    in RayQueryInternal rayQuery)
{
    return rayQuery.candidate.origin;
}

//=====================================================================================================================
// Return candidate object space ray direction
export float3 _RayQuery_CandidateObjectRayDirection(
    in RayQueryInternal rayQuery)
{
    return rayQuery.candidate.direction;
}

//=====================================================================================================================
// Return committed object space ray origin
export float3 _RayQuery_CommittedObjectRayOrigin(
    in RayQueryInternal rayQuery)
{
    return rayQuery.committed.origin;
}

//=====================================================================================================================
// Return committed object space ray direction
export float3 _RayQuery_CommittedObjectRayDirection(
    in RayQueryInternal rayQuery)
{
    return rayQuery.committed.direction;
}

//=====================================================================================================================
// Return candidate instance contribution to hit group index
export uint _RayQuery_CandidateInstanceContributionToHitGroupIndex(
    in RayQueryInternal rayQuery)
{
    return rayQuery.candidate.instanceContribution;
}

//=====================================================================================================================
// Return committed instance contribution to hit group index
export uint _RayQuery_CommittedInstanceContributionToHitGroupIndex(
    in RayQueryInternal rayQuery)
{
    return rayQuery.committed.instanceContribution;
}

//=====================================================================================================================
// Return triangle node barycentrics
export float2 _RayQuery_TriangleBarycentrics(
    in RayQueryInternal rayQuery,
    bool                committed)
{
    if (committed)
    {
        return rayQuery.committed.barycentrics;
    }
    else
    {
        return rayQuery.candidate.barycentrics;
    }
}

//=====================================================================================================================
// Return whether committed triangle node is front facing
export bool _RayQuery_TriangleFrontFace(
    in RayQueryInternal rayQuery,
    bool                committed)
{
    if (committed)
    {
        return rayQuery.committed.frontFace;
    }
    else
    {
        return rayQuery.candidate.frontFace;
    }
}

//=====================================================================================================================
// Candidate procedural primitive hit
export bool _RayQuery_CandidateAabbOpaque(
    inout_param(RayQueryInternal) rayQuery)
{
    return (rayQuery.candidateType == CANDIDATE_PROCEDURAL_PRIMITIVE);
}

//=====================================================================================================================
// Rayquery intersection type for commit/candidate
export uint _RayQuery_IntersectionType(
    in RayQueryInternal rayQuery,
    bool                committed)
{
    if (committed)
    {
        return rayQuery.committedStatus;
    }
    else
    {
        return (uint)(rayQuery.candidateType > CANDIDATE_NON_OPAQUE_TRIANGLE);
    }
}

export float4x3 _RayQuery_WorldToObject4x3(
    in RayQueryInternal rayQuery,
    bool                committed)
{
    uint64_t topLevelAddr = PackUint64(rayQuery.topLevelBvhLo, rayQuery.topLevelBvhHi);
    uint64_t instNodeAddr;
    if (committed)
    {
        instNodeAddr = GetRayQueryInstanceNodePtr(topLevelAddr, rayQuery.committed.instNodePtr);
    }
    else
    {
        instNodeAddr = GetRayQueryInstanceNodePtr(topLevelAddr, rayQuery.candidate.instNodePtr);
    }

    return GetWorldToObject4x3(instNodeAddr);
}

export float4x3 _RayQuery_ObjectToWorld4x3(
    in RayQueryInternal rayQuery,
    bool                committed)
{
    uint64_t topLevelAddr = PackUint64(rayQuery.topLevelBvhLo, rayQuery.topLevelBvhHi);
    uint64_t instNodeAddr;
    if (committed)
    {
        instNodeAddr = GetRayQueryInstanceNodePtr(topLevelAddr, rayQuery.committed.instNodePtr);
    }
    else
    {
        instNodeAddr = GetRayQueryInstanceNodePtr(topLevelAddr, rayQuery.candidate.instNodePtr);
    }

    return  GetObjectToWorld4x3(instNodeAddr);
}

//=====================================================================================================================
// Return committed hit T
export float _RayQuery_RayT(in RayQueryInternal rayQuery, bool committed)
{
    float rayTCurrent = (committed) ? rayQuery.committed.rayTCurrent :
        rayQuery.candidate.rayTCurrent;

    rayTCurrent += rayQuery.rayTMin;
    return rayTCurrent;
}

//=====================================================================================================================
// Return committed instance ID
export uint _RayQuery_InstanceID(in RayQueryInternal rayQuery, bool committed)
{
    if (committed)
    {
        return _RayQuery_CommittedInstanceID(rayQuery);
    }
    else
    {
        return _RayQuery_CandidateInstanceID(rayQuery);
    }
}

//=====================================================================================================================
// Return committed geometry index
export uint _RayQuery_GeometryIndex(in RayQueryInternal rayQuery, bool committed)
{
    if (committed)
    {
        return rayQuery.committed.geometryIndex;
    }
    else
    {
        return rayQuery.candidate.geometryIndex;
    }
}

//=====================================================================================================================
// Return candidate primitive index
export uint _RayQuery_PrimitiveIndex(in RayQueryInternal rayQuery, bool committed)
{
    if (committed)
    {
        return rayQuery.committed.primitiveIndex;
    }
    else
    {
        return rayQuery.candidate.primitiveIndex;
    }
}

export float3 _RayQuery_ObjectRayDirection(in RayQueryInternal rayQuery, bool committed)
{
    if (committed)
    {
        return rayQuery.committed.direction;
    }
    else
    {
        return rayQuery.candidate.direction;
    }
}

export float3 _RayQuery_ObjectRayOrigin(in RayQueryInternal rayQuery, bool committed)
{
    if (committed)
    {
        return rayQuery.committed.origin;
    }
    else
    {
        return rayQuery.candidate.origin;
    }
}

export uint _RayQuery_InstanceContributionToHitGroupIndex(in RayQueryInternal rayQuery, bool committed)
{
    if (committed)
    {
        return rayQuery.committed.instanceContribution;
    }
    else
    {
        return rayQuery.candidate.instanceContribution;
    }
}

//=====================================================================================================================
// Return committed instance ID
export uint _RayQuery_InstanceIndex(in RayQueryInternal rayQuery, bool committed)
{
    if (committed)
    {
        return _RayQuery_CommittedInstanceIndex(rayQuery);
    }
    else
    {
        return _RayQuery_CandidateInstanceIndex(rayQuery);
    }
}

//=====================================================================================================================
// Fetch triangle position
export TriangleData _RayQuery_FetchTrianglePosition(
    inout_param(RayQueryInternal) rayQuery,  // BVH address
    in bool                       committed) // Node pointer
{
    TriangleData tdata;
    RayTracingIpLevel rtip = GetRtIpLevel();
    switch (rtip)
    {
    default:
    {
        tdata = FetchTrianglePositionFromRayQuery(rayQuery, committed);
        break;
    }
    }
    return tdata;
}

//=====================================================================================================================
// RayQuery::Proceed() entry point
export bool _RayQuery_Proceed(
    inout_param(RayQueryInternal) rayQuery,
    in    uint                    constRayFlags,
    in    uint3                   dispatchThreadId)
{
    uint rtIpLevel = ConvertRtIpLevel(GetRtIpLevel());
    return RayQueryProceedCommon(
        rayQuery,
        constRayFlags,
        dispatchThreadId,
        rtIpLevel
    );
}

//=====================================================================================================================
// TraceRayInline() entry point
export void _RayQuery_TraceRayInline(
    inout_param(RayQueryInternal) rayQuery,
    in    uint                    accelStructLo,
    in    uint                    accelStructHi,
    in    uint                    constRayFlags,
    in    uint                    rayFlags,
    in    uint                    instanceMask,
    in    RayDesc                 rayDesc,
    in    uint3                   dispatchThreadId)
{
    uint rtIpLevel = ConvertRtIpLevel(GetRtIpLevel());
    TraceRayInlineCommon(rayQuery,
                         accelStructLo,
                         accelStructHi,
                         constRayFlags,
                         rayFlags,
                         instanceMask,
                         rayDesc,
                         dispatchThreadId,
                         rtIpLevel);
}

export void _RayQuery_SetObjId(in RayQueryInternal rayQuery, int objId)
{
    rayQuery.rayQueryObjId = objId;
}

export uint _RayQuery_GetObjId(in RayQueryInternal rayQuery)
{
    return rayQuery.rayQueryObjId;
}

#endif
