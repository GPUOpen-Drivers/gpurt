/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2022 Advanced Micro Devices, Inc. All Rights Reserved.
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

#include "TraceRay1_1.hlsl"

//=====================================================================================================================
static IntersectionResult TraceRay(
    in GpuVirtualAddress topLevelBvh,               ///< Top-level acceleration structure to use
    in uint              rayFlags,                  ///< Ray flags
    in uint              traceRayParameters,        ///< Packed trace ray parameters
    in RayDesc           ray,                       ///< Ray to be traced
    in uint              rayId,                     ///< Ray ID for profiling
    in uint              rtHwVersion,               ///< HW version to determine TraceRay implementation
    in float             earlyTerminateThreshold)   ///< Early Ray Terminate Threshold
{
    switch (rtHwVersion)
    {
        case RTIP1_1:  return TraceRayImpl1_1(topLevelBvh,
                                              rayFlags,
                                              traceRayParameters,
                                              ray,
                                              rayId,
                                              earlyTerminateThreshold);

        default: return (IntersectionResult) 0;
    }
}

//=====================================================================================================================
static IntersectionResult IntersectRay(
    in GpuVirtualAddress topLevelBvh,   ///< Top-level acceleration structure to use
    in RayDesc           ray,           ///< Ray to be traced
    in uint              blasPointer,   ///< bottom level node pointer
    in uint              tlasPointer,   ///< top level node pointer
    in uint              rayId,         ///< Ray ID for profiling
    in uint              rtHwVersion)   ///< HW version to determine IntersectRay implementation
{
    switch (rtHwVersion)
    {
        case RTIP1_1:  return IntersectRayImpl1_1(topLevelBvh, ray, blasPointer, tlasPointer, rayId);
        default: return (IntersectionResult)0;
    }
}

//=====================================================================================================================
static bool TraceRayCommon(
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
    uint  tlasPointer,
    bool  traverse,
    uint  rtHwVersion,
    float earlyTerminateThreshold)
{
#if DEVELOPER
    rayFlags |= DispatchRaysInfo.ProfileRayFlags;
    TraversalCounter counter;
    uint64_t timerBegin = SampleGpuTimer();
#endif

    // Capture parameters so they can be used to implement HLSL intrinsic functions.
    AmdTraceRaySetTraceParams(rayFlags,
#ifdef AMD_VULKAN
                              instanceInclusionMask,
#endif
                              originX, originY, originZ,
                              tMin,
                              dirX, dirY, dirZ);

    RayDesc ray;
    ray.Origin    = float3(originX, originY, originZ);
    ray.Direction = float3(dirX, dirY, dirZ);
    ray.TMax      = tMax;
    ray.TMin      = tMin;

    IntersectionResult result = (IntersectionResult)0;

    const GpuVirtualAddress accelStruct = MakeGpuVirtualAddress(accelStructLo, accelStructHi);
    uint rayId = 0;

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        rayId = GetRayId(AmdTraceRayDispatchRaysIndex());
    }
#endif

    const uint packedTraceParams =
        (instanceInclusionMask & 0xFF) |
        ((rayContributionToHitGroupIndex & 0xF) << 8) |
        ((multiplierForGeometryContributionToShaderIndex & 0xF) << 12) |
        ((missShaderIndex & 0xFFFF) << 16);

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        WriteRayHistoryTokenBegin(rayId, AmdTraceRayDispatchRaysIndex(), accelStruct, rayFlags, packedTraceParams, ray);
        WriteRayHistoryTokenTimeStamp(rayId, timerBegin);
    }
#endif

    if (IsValidTrace(ray, accelStruct, instanceInclusionMask, rayFlags, AmdTraceRayGetStaticFlags()))
    {
        LogAccelStruct(accelStruct);

        if (traverse)
        {
            result = TraceRay(accelStruct,
                              rayFlags,
                              packedTraceParams,
                              ray,
                              rayId,
                              rtHwVersion,
                              earlyTerminateThreshold);
        }
        else
        {
            result = IntersectRay(accelStruct, ray, blasPointer, tlasPointer, rayId, rtHwVersion);
        }
    }
    else
    {
        // Report miss for invalid ray or BVH
        result.nodeIndex = INVALID_IDX;
    }

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        WriteDispatchCounters(result.numIterations);

        uint64_t timerEnd = SampleGpuTimer();
        WriteRayHistoryTokenTimeStamp(rayId, timerEnd);

        if (timerEnd > timerBegin)
        {
            counter.data[TCID_TIMING_DATA] = (uint)(timerEnd - timerBegin);
        }
        else
        {
            counter.data[TCID_TIMING_DATA] = 0;
        }

        if (result.nodeIndex != INVALID_NODE)
        {
            WriteRayHistoryTokenEnd(rayId, uint2(result.primitiveIndex, result.geometryIndex));
        }
        else
        {
            WriteRayHistoryTokenEnd(rayId, uint2(~0, ~0));
        }

        counter.data[TCID_NUM_RAY_BOX_TEST] = result.numRayBoxTest;
        counter.data[TCID_NUM_RAY_TRIANGLE_TEST] = result.numRayTriangleTest;
        counter.data[TCID_NUM_ITERATION] = result.numIterations;
        counter.data[TCID_MAX_TRAVERSAL_DEPTH] = result.maxStackDepth;
        counter.data[TCID_NUM_ANYHIT_INVOCATION] = result.numAnyHitInvocation;
        counter.data[TCID_WAVE_ID] = AmdTraceRayGetHwWaveId();
        counter.data[TCID_NUM_CANDIDATE_HITS] = result.numCandidateHits;
        counter.data[TCID_INSTANCE_INTERSECTIONS] = result.instanceIntersections;
    }
#endif

    if (result.nodeIndex != INVALID_IDX) // Hit
    {
        if ((rayFlags & RAY_FLAG_SKIP_CLOSEST_HIT_SHADER) == 0)
        {
            const uint instanceContribution = (result.instanceContribution & 0x00ffffff);
            const HitGroupInfo hitInfo = GetHitGroupInfo(rayContributionToHitGroupIndex,
                                                         multiplierForGeometryContributionToShaderIndex,
                                                         result.geometryIndex,
                                                         instanceContribution);

            const uint2 instNodeAddr64 = CalculateNodeAddr64(accelStruct, result.instNodePtr);

            // Set intersection attributes
            AmdTraceRaySetHitAttributes(result.t,
                                        result.hitkind,
                                        HIT_STATUS_ACCEPT,
                                        instNodeAddr64.x,
                                        instNodeAddr64.y,
                                        result.primitiveIndex,
                                        false,
                                        result.geometryIndex);

            AmdTraceRayCallClosestHitShader(hitInfo.closestHitId, hitInfo.tableIndex);
#if DEVELOPER
            if (EnableTraversalCounter())
            {
                WriteRayHistoryTokenFunctionCall(rayId,
                                                 hitInfo.closestHitId,
                                                 hitInfo.tableIndex,
                                                 RAY_HISTORY_FUNC_CALL_TYPE_CLOSEST);
                counter.data[TCID_SHADER_ID]           = hitInfo.closestHitId.x;
                counter.data[TCID_SHADER_RECORD_INDEX] = hitInfo.tableIndex;
            }
#endif
        }
    }
    else // Miss
    {
        const uint2 shaderId = GetShaderId(MakeGpuVirtualAddress(DispatchRaysInfo.MissShaderTableStartAddressLow,
                                                                 DispatchRaysInfo.MissShaderTableStartAddressHigh),
                                           missShaderIndex,
                                           DispatchRaysInfo.MissShaderTableStrideInBytes);

        // Only tCurrent/tMax is valid in the miss shader
        AmdTraceRaySetHitAttributes((tMax - tMin), 0, 0, 0, 0, 0, false, 0);

        // Driver patch point for Miss shader
        AmdTraceRayCallMissShader(shaderId.xy, missShaderIndex);

#if DEVELOPER
        if (EnableTraversalCounter())
        {
            WriteRayHistoryTokenFunctionCall(rayId, shaderId, missShaderIndex, RAY_HISTORY_FUNC_CALL_TYPE_MISS);
            counter.data[TCID_SHADER_ID] = shaderId.x;
            counter.data[TCID_SHADER_RECORD_INDEX] = missShaderIndex | TCID_SHADER_RECORD_INDEX_MISS;
        }
#endif
    }

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        WriteTraversalCounter(rayId, counter);
    }
#endif

    return (result.hitkind != HIT_KIND_EARLY_RAY_TERMINATE);
}

//=====================================================================================================================
// TraceRay() implemented on top of TraceRayInline
//
// Uses TraceRayInline RayQueryInternal structs to implement TraceRays
// Need to patch in arguments the same way TraceRayInlineAmdInternal and RayQueryProceedAmdInternal are done
//
static void TraceRayUsingRayQueryCommon(uint  accelStructLo,
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
                                        uint  rtHwVersion)
{
    // Capture parameters so they can be used to implement HLSL intrinsic functions.
    AmdTraceRaySetTraceParams(rayFlags,
#ifdef AMD_VULKAN
                              instanceInclusionMask,
#endif
                              originX,
                              originY,
                              originZ,
                              tMin,
                              dirX,
                              dirY,
                              dirZ);

    RayDesc ray;
    ray.Origin    = float3(originX, originY, originZ);
    ray.Direction = float3(dirX, dirY, dirZ);
    ray.TMax      = tMax;
    ray.TMin      = tMin;

    RayQueryInternal rayQuery = (RayQueryInternal)0;
    // Initialize to the static pipeline flags
    const uint constRayFlags = AmdTraceRayGetStaticFlags();
    uint rayId = 0;
#if DEVELOPER
    rayId = GetRayId(AmdTraceRayDispatchRaysIndex());
#endif

    // Initialise static ray query
    TraceRayInlineCommon(rayQuery,
                         accelStructLo,
                         accelStructHi,
                         constRayFlags,
                         rayFlags,
                         instanceInclusionMask,
                         ray,
                         AmdTraceRayDispatchRaysIndex(),
                         rtHwVersion);

    const GpuVirtualAddress accelStruct = MakeGpuVirtualAddress(accelStructLo, accelStructHi);
    const bool raySkipProcedural        = (rayQuery.rayFlags & RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES);
    const bool isValid                  = IsValidTrace(rayQuery.rayDesc, accelStruct, instanceInclusionMask, rayQuery.rayFlags, AmdTraceRayGetStaticFlags());
    bool  continueTraversal             = true;
    float currentT                      = rayQuery.rayDesc.TMax;
    uint  committedHitkind              = 0;

    if (isValid == true)
    {
        LogAccelStruct(accelStruct);

        while (RayQueryProceedCommon(rayQuery, constRayFlags, AmdTraceRayDispatchRaysIndex(), rtHwVersion, 0.0) == true)
        {
            const uint instanceContribution = rayQuery.candidate.instanceContribution;
            const HitGroupInfo hitInfo      = GetHitGroupInfo(rayContributionToHitGroupIndex,
                                                              multiplierForGeometryContributionToShaderIndex,
                                                              rayQuery.candidate.geometryIndex,
                                                              instanceContribution);

            uint  status  = HIT_STATUS_IGNORE;
            uint  hitkind = 0;

            const uint2 candidateInstNodeAddr64 = CalculateNodeAddr64(accelStruct, rayQuery.candidate.instNodePtr);

            // Handle candidate hits and calling intersection/anyhit shaders
            if ((raySkipProcedural == false) && (rayQuery.candidateType != CANDIDATE_NON_OPAQUE_TRIANGLE))
            {
                // The most significant bit contains whether or not the geometry is set to not have duplicate anyhit calls
                uint anyHitCallType = (rayQuery.candidateType == CANDIDATE_PROCEDURAL_PRIMITIVE) ?
                                      ANYHIT_CALLTYPE_SKIP :
                                      (rayQuery.candidateType == CANDIDATE_NO_DUPLICATE_ANYHIT_PROCEDURAL_PRIMITIVE) ?
                                      ANYHIT_CALLTYPE_NO_DUPLICATE :
                                      ANYHIT_CALLTYPE_DUPLICATE;

                currentT = rayQuery.committed.rayTCurrent;
                status   = HIT_STATUS_IGNORE;

                // Set intersection attributes for trace rays
                AmdTraceRaySetHitAttributes(currentT,
                                            0,
                                            status,
                                            candidateInstNodeAddr64.x,
                                            candidateInstNodeAddr64.y,
                                            rayQuery.candidate.primitiveIndex,
                                            anyHitCallType,
                                            rayQuery.candidate.geometryIndex);

                AmdTraceRayCallIntersectionShader(hitInfo.intersectionId, hitInfo.anyHitId, hitInfo.tableIndex);
#if DEVELOPER
                if (EnableTraversalCounter())
                {
                    WriteRayHistoryTokenFunctionCall(rayId,
                                                     hitInfo.intersectionId,
                                                     hitInfo.tableIndex,
                                                     RAY_HISTORY_FUNC_CALL_TYPE_INTERSECTION);
                }
#endif
            }
            else
            {
                currentT = rayQuery.candidate.rayTCurrent;
                status   = HIT_STATUS_ACCEPT;

                // Set intersection attributes for trace rays
                AmdTraceRaySetHitAttributes(rayQuery.candidate.rayTCurrent,
                                            rayQuery.candidate.frontFace ? HIT_KIND_TRIANGLE_FRONT_FACE :
                                                                           HIT_KIND_TRIANGLE_BACK_FACE,
                                            status,
                                            candidateInstNodeAddr64.x,
                                            candidateInstNodeAddr64.y,
                                            rayQuery.candidate.primitiveIndex,
                                            ANYHIT_CALLTYPE_NO_DUPLICATE,
                                            rayQuery.candidate.geometryIndex);

                BuiltInTriangleIntersectionAttributes attr = { rayQuery.candidate.barycentrics };

                AmdTraceRayCallTriangleAnyHitShader(hitInfo.anyHitId, hitInfo.tableIndex, attr);
#if DEVELOPER
                if (EnableTraversalCounter())
                {
                    WriteRayHistoryTokenFunctionCall(rayId,
                                                     hitInfo.anyHitId,
                                                     hitInfo.tableIndex,
                                                     RAY_HISTORY_FUNC_CALL_TYPE_ANY_HIT);
                }
#endif
            }

            AmdTraceRayGetHitAttributes(currentT, hitkind, status);

            // Accept hit
            if (status != HIT_STATUS_IGNORE)
            {
                if ((raySkipProcedural == false) && (rayQuery.candidateType != CANDIDATE_NON_OPAQUE_TRIANGLE))
                {
                    rayQuery.committedStatus = COMMITTED_PROCEDURAL_PRIMITIVE_HIT;
                    committedHitkind = hitkind;
                }
                else
                {
                    rayQuery.committedStatus = COMMITTED_TRIANGLE_HIT;
                }

                rayQuery.committed             = rayQuery.candidate;
                rayQuery.committed.rayTCurrent = currentT;

                if ((status == HIT_STATUS_ACCEPT_AND_END_SEARCH) ||
                    (rayQuery.rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH))
                {
                    break;
                }
            }
        }

        switch (rayQuery.committedStatus)
        {
            // Call closest hit shader
            case COMMITTED_TRIANGLE_HIT:
            case COMMITTED_PROCEDURAL_PRIMITIVE_HIT:
            {
                if ((rayQuery.rayFlags & RAY_FLAG_SKIP_CLOSEST_HIT_SHADER) == 0)
                {
                    const uint instanceContribution = rayQuery.committed.instanceContribution;
                    const HitGroupInfo hitInfo      = GetHitGroupInfo(rayContributionToHitGroupIndex,
                                                                      multiplierForGeometryContributionToShaderIndex,
                                                                      rayQuery.committed.geometryIndex,
                                                                      instanceContribution);

                    if (rayQuery.committedStatus == COMMITTED_TRIANGLE_HIT)
                    {
                        // Set barycentric coordinates
                        AmdTraceRaySetTriangleIntersectionAttributes(rayQuery.committed.barycentrics);

                        committedHitkind = rayQuery.committed.frontFace ? HIT_KIND_TRIANGLE_FRONT_FACE :
                                                                          HIT_KIND_TRIANGLE_BACK_FACE;
                    }

                    const uint2 committedInstNodeAddr64 = CalculateNodeAddr64(accelStruct,
                                                                              rayQuery.committed.instNodePtr);

                    // Set intersection attributes
                    AmdTraceRaySetHitAttributes(rayQuery.committed.rayTCurrent,
                                                committedHitkind,
                                                HIT_STATUS_ACCEPT,
                                                committedInstNodeAddr64.x,
                                                committedInstNodeAddr64.y,
                                                rayQuery.committed.primitiveIndex,
                                                0,
                                                rayQuery.committed.geometryIndex);

                    // Driver patch point for Closest hit shader
                    AmdTraceRayCallClosestHitShader(hitInfo.closestHitId, hitInfo.tableIndex);
#if DEVELOPER
                    if (EnableTraversalCounter())
                    {
                        WriteRayHistoryTokenFunctionCall(rayId,
                                                         hitInfo.closestHitId,
                                                         hitInfo.tableIndex,
                                                         RAY_HISTORY_FUNC_CALL_TYPE_CLOSEST);
                    }
#endif
                }
                break;
            }
            // Call miss shader
            case COMMITTED_NOTHING:
            {
                // Calculate miss shader record address
                const uint2 shaderId = GetShaderId(MakeGpuVirtualAddress(DispatchRaysInfo.MissShaderTableStartAddressLow,
                                                                         DispatchRaysInfo.MissShaderTableStartAddressHigh),
                                                   missShaderIndex,
                                                   DispatchRaysInfo.MissShaderTableStrideInBytes);

                // Only tCurrent/tMax is valid in the miss shader
                AmdTraceRaySetHitAttributes((rayQuery.rayDesc.TMax - rayQuery.rayDesc.TMin), 0, 0, 0, 0, 0, false, 0);

                // Driver patch point for Miss shader
                AmdTraceRayCallMissShader(shaderId.xy, missShaderIndex);
#if DEVELOPER
                if (EnableTraversalCounter())
                {
                    WriteRayHistoryTokenFunctionCall(rayId, shaderId, missShaderIndex, RAY_HISTORY_FUNC_CALL_TYPE_MISS);
                }
#endif
                break;
            }
        }
    }
}
