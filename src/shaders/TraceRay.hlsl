/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
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

#if GPURT_BUILD_RTIP2
#include "TraceRay2_0.hlsl"
#endif

//=====================================================================================================================
static IntersectionResult TraceRayInternal(
    in GpuVirtualAddress topLevelBvh,             // Top-level acceleration structure to use
    in uint              rayFlags,                // Ray flags
    in uint              traceRayParameters,      // Packed trace ray parameters
    in RayDesc           rayDesc,                 // Ray to be traced
    in uint              rayId,                   // Ray ID for profiling
    in uint              rtIpLevel                // HW version to determine TraceRay implementation
)
// Default path
{
    switch (rtIpLevel)
    {
        case GPURT_RTIP1_1:
        return TraceRayImpl1_1(
            topLevelBvh,
            rayFlags,
            traceRayParameters,
            rayDesc,
            rayId
        );
#if GPURT_BUILD_RTIP2
        case GPURT_RTIP2_0:
        return TraceRayImpl2_0(
            topLevelBvh,
            rayFlags,
            traceRayParameters,
            rayDesc,
            rayId
        );
#endif
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
    in uint              rtIpLevel)   ///< HW version to determine IntersectRay implementation
{
    switch (rtIpLevel)
    {
        case GPURT_RTIP1_1:  return IntersectRayImpl1_1(topLevelBvh, ray, blasPointer, tlasPointer, rayId);
#if GPURT_BUILD_RTIP2
        case GPURT_RTIP2_0:  return IntersectRayImpl1_1(topLevelBvh, ray, blasPointer, tlasPointer, rayId);
#endif
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
    uint  rtIpLevel
)
{
#if DEVELOPER
    rayFlags |= DispatchRaysConstBuf.profileRayFlags;
    TraversalCounter counter;
    uint64_t timerBegin   = SampleGpuTimer();
    uint parentId = AmdTraceRayGetParentId();
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

    const GpuVirtualAddress accelStruct = FetchAccelStructBaseAddr(accelStructLo, accelStructHi);
    uint rayId = 0;
    uint dynamicId  = 0;
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        rayId     = GetRayId(AmdTraceRayDispatchRaysIndex());
        dynamicId = AllocateRayHistoryDynamicId();
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
        WriteRayHistoryTokenBegin(rayId,
                                  AmdTraceRayDispatchRaysIndex(),
                                  accelStruct,
                                  rayFlags,
                                  packedTraceParams,
                                  ray,
                                  AmdTraceRayGetStaticId(),
                                  dynamicId,
                                  parentId);
        WriteRayHistoryTokenTimeStamp(rayId, timerBegin);
    }
#endif

    if (IsValidTrace(ray, accelStruct, instanceInclusionMask, rayFlags, AmdTraceRayGetStaticFlags()))
    {
        LogAccelStruct(accelStruct);

        if (traverse)
        {
            result = TraceRayInternal(
                accelStruct,
                rayFlags,
                packedTraceParams,
                ray,
                rayId,
                rtIpLevel
                );
        }
        else
        {
            result = IntersectRay(accelStruct, ray, blasPointer, tlasPointer, rayId, rtIpLevel);
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
            const GpuVirtualAddress nodeAddr64 = accelStruct + ExtractNodePointerOffset(result.instNodePtr);
            uint instNodeIndex = FetchInstanceIdx(nodeAddr64);

            WriteRayHistoryTokenEnd(rayId,
                                    uint2(result.primitiveIndex, result.geometryIndex),
                                    instNodeIndex,
                                    result.numIterations,
                                    result.instanceIntersections,
                                    result.hitkind,
                                    result.t);
        }
        else
        {
            WriteRayHistoryTokenEnd(rayId,
                                    uint2(~0, ~0),
                                    uint(~0),
                                    result.numIterations,
                                    result.instanceIntersections,
                                    uint(~0),
                                    (tMax - tMin));
        }
        AmdTraceRaySetParentId(dynamicId);

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

            const uint64_t instNodePtr64 = CalculateInstanceNodePtr64(rtIpLevel, accelStruct, result.instNodePtr);
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
            // Set intersection attributes
            AmdTraceRaySetHitAttributes(result.t,
                                        result.hitkind,
                                        HIT_STATUS_ACCEPT,
                                        LowPart(instNodePtr64),
                                        HighPart(instNodePtr64),
                                        result.primitiveIndex,
                                        false,
                                        result.geometryIndex);
            // Set hit triangle information
            const InstanceDesc      desc = FetchInstanceDesc(accelStruct, result.instNodePtr);
            const GpuVirtualAddress blas = GetInstanceAddr(desc);
            AmdTraceRaySetHitTriangleNodePointer(blas, result.nodeIndex);

            AmdTraceRayCallClosestHitShader(hitInfo.closestHitId, hitInfo.tableIndex);
        }
    }
    else // Miss
    {
        const uint64_t missTableBaseAddress =
            PackUint64(DispatchRaysConstBuf.missTableBaseAddressLo, DispatchRaysConstBuf.missTableBaseAddressHi);

        const uint2 shaderId = GetShaderId(missTableBaseAddress,
                                           missShaderIndex,
                                           DispatchRaysConstBuf.missTableStrideInBytes);
#if DEVELOPER
        if (EnableTraversalCounter())
        {
            WriteRayHistoryTokenFunctionCall(rayId, shaderId, missShaderIndex, RAY_HISTORY_FUNC_CALL_TYPE_MISS);
            counter.data[TCID_SHADER_ID] = shaderId.x;
            counter.data[TCID_SHADER_RECORD_INDEX] = missShaderIndex | TCID_SHADER_RECORD_INDEX_MISS;
        }
#endif
        // Only tCurrent/tMax is valid in the miss shader
        AmdTraceRaySetHitAttributes((tMax - tMin), 0, 0, 0, 0, 0, false, 0);

        // Driver patch point for Miss shader
        AmdTraceRayCallMissShader(shaderId.xy, missShaderIndex);
    }

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        WriteTraversalCounter(rayId, counter);
    }
#endif

    return (result.hitkind != HIT_KIND_EARLY_RAY_TERMINATE);
}
