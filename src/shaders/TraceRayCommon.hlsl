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
//

#include "IntersectCommon.hlsl"

#if DEVELOPER
#include "../../gpurt/gpurtCounter.h"
#endif

// Driver reserved space ID and resource bindings
#ifdef AMD_VULKAN

#define SPACEID space93

#define DispatchDynamicRaysInfoId b16
#define DispatchRaysInfoId        b17
#else

#define SPACEID space2147420893

#define DispatchDynamicRaysInfoId b0
#define DispatchRaysInfoId        b1

#endif

#ifndef __cplusplus
//=====================================================================================================================
ConstantBuffer<DispatchRaysDynamicInfoData> DispatchDynamicRaysInfo : register(DispatchDynamicRaysInfoId, SPACEID);
ConstantBuffer<DispatchRaysInfoData>        DispatchRaysInfo        : register(DispatchRaysInfoId, SPACEID);

#if DEVELOPER
globallycoherent RWByteAddressBuffer        Counters                : register(u0, SPACEID);
#endif
#endif

//=====================================================================================================================
static uint ExtractInstanceInclusionMask(in uint traceRayParameters)
{
    return (traceRayParameters & 0xFF);
}

//=====================================================================================================================
static uint ExtractRayContributionToHitIndex(in uint traceRayParameters)
{
    return ((traceRayParameters >> 8) & 0xF);
}

//=====================================================================================================================
static uint ExtractMultiplierForGeometryContributionToHitIndex(in uint traceRayParameters)
{
    return ((traceRayParameters >> 12) & 0xF);
}

//=====================================================================================================================
static uint ExtractMissShaderIndex(in uint traceRayParameters)
{
    return ((traceRayParameters >> 16) & 0xFFFF);
}

//=====================================================================================================================
static uint2 GetShaderId(GpuVirtualAddress tableAddress, uint index, uint stride)
{
    return LoadDwordAtAddrx2(tableAddress + stride * index);
}

//=====================================================================================================================
static uint CalculateHitGroupRecordAddress(
    uint RayContributionToHitGroupIndex,
    uint MultiplierForGeometryContributionToHitGroupIndex,
    uint GeometryContributionToHitGroupIndex,
    uint InstanceContributionToHitGroupIndex)
{
    return (RayContributionToHitGroupIndex +                   // from shader: TraceRay()
           (MultiplierForGeometryContributionToHitGroupIndex * // from shader: TraceRay()
            GeometryContributionToHitGroupIndex) +             // system generated index of geometry in
                                                               // bottom level acceleration structure (0,1,2,3..)
            InstanceContributionToHitGroupIndex                // from instance
           );
}

//=====================================================================================================================
static HitGroupInfo GetHitGroupInfo(
    uint RayContributionToHitGroupIndex,
    uint MultiplierForGeometryContributionToHitGroupIndex,
    uint GeometryContributionToHitGroupIndex,
    uint InstanceContributionToHitGroupIndex)
{
    const uint hitGroupRecordIndex = CalculateHitGroupRecordAddress(RayContributionToHitGroupIndex,
                                                                    MultiplierForGeometryContributionToHitGroupIndex,
                                                                    GeometryContributionToHitGroupIndex,
                                                                    InstanceContributionToHitGroupIndex);

#if __cplusplus
    HitGroupInfo hitInfo;
    hitInfo.tableIndex = hitGroupRecordIndex;
#else
    const uint offset = DispatchRaysInfo.HitGroupTableStrideInBytes * hitGroupRecordIndex;

    const GpuVirtualAddress tableVa = MakeGpuVirtualAddress(DispatchRaysInfo.HitGroupTableStartAddressLow,
                                                            DispatchRaysInfo.HitGroupTableStartAddressHigh);

    const uint4 d0 = LoadDwordAtAddrx4(tableVa + offset);
    const uint2 d1 = LoadDwordAtAddrx2(tableVa + offset + 0x10);

    HitGroupInfo hitInfo;
    hitInfo.closestHitId   = d0.xy;
    hitInfo.anyHitId       = d0.zw;
    hitInfo.intersectionId = d1.xy;
    hitInfo.tableIndex     = hitGroupRecordIndex;
#endif

    return hitInfo;
}

//=====================================================================================================================
static uint GetAnyHitIdLow(
    in uint traceParameters,
    in uint geometryContributionToHitGroupIndex,
    in uint instanceContributionToHitGroupIndex)
{
    const uint hitGroupRecordIndex =
        CalculateHitGroupRecordAddress(ExtractRayContributionToHitIndex(traceParameters),
                                       ExtractMultiplierForGeometryContributionToHitIndex(traceParameters),
                                       geometryContributionToHitGroupIndex,
                                       instanceContributionToHitGroupIndex);

    const uint offset = DispatchRaysInfo.HitGroupTableStrideInBytes * hitGroupRecordIndex;

    const GpuVirtualAddress tableVa = MakeGpuVirtualAddress(DispatchRaysInfo.HitGroupTableStartAddressLow,
                                                            DispatchRaysInfo.HitGroupTableStartAddressHigh);

    const uint anyHitIdLow = LoadDwordAtAddr(tableVa + offset + 8);

    return anyHitIdLow;
}

#if DEVELOPER
//=====================================================================================================================
static uint GetRayId(in uint3 dispatchRaysIndex)
{
    const uint flatRayIndex = dispatchRaysIndex.x +
                             (dispatchRaysIndex.y * DispatchRaysInfo.RayGridWidth) +
                             (dispatchRaysIndex.z * DispatchRaysInfo.RayGridWidth * DispatchRaysInfo.RayGridHeight);
    return flatRayIndex;
}

//=====================================================================================================================
// Reserve token space in counter buffer for logging
//
static uint ReserveTokenSpace(
    in uint numDwords)
{
    // Offset 0 is reserved for tracking token requests
    uint offset;
    Counters.InterlockedAdd(RAY_TRACING_COUNTER_REQUEST_BYTE_OFFSET, numDwords << 2, offset);

    // Account for the reserved bytes
    return offset + RAY_TRACING_COUNTER_RESERVED_BYTE_SIZE;
}

//=====================================================================================================================
// Write ray-history control token to counter buffer
//
// id           : Unique caller generated ray identifier
// type         : Must be one of RayHistoryTokenType
// dwordSize    : Size of payload data that follows the control DWORDs
// optionalData : Optional control data
//
static uint WriteRayHistoryControlToken(
    in uint id,
    in uint type,
    in uint dwordSize,
    in uint optionalData)
{
    // Write token to memory
    const uint rayId = id | RAYID_CONTROL_MASK;
    const uint cntrl = type | (dwordSize << 16) | (optionalData << 24);

    uint offset = ReserveTokenSpace(dwordSize + RAY_HISTORY_TOKEN_CONTROL_SIZE);
    Counters.Store2(offset, uint2(rayId, cntrl));

    return offset + (RAY_HISTORY_TOKEN_CONTROL_SIZE << 2);
}

//=====================================================================================================================
static bool LogCounters(in uint id, in uint mode)
{
    return ((DispatchRaysInfo.CounterMode == mode) &&
            (id >= DispatchRaysInfo.RayIdRangeBegin) &&
            (id < DispatchRaysInfo.RayIdRangeEnd));
}

//=====================================================================================================================
// Write 32-bit hardware node pointer to ray history buffer
//
static void WriteRayHistoryTokenNodePtr(
    in uint id,
    in uint nodePtr)
{
    if (LogCounters(id, TRACERAY_COUNTER_MODE_RAYHISTORY_FULL))
    {
        // Reserve token space and write data
        uint offset = ReserveTokenSpace(2);
        Counters.Store2(offset, uint2(id, nodePtr));
    }
}

//=====================================================================================================================
// Write 64-bit top-level acceleration structure base address to ray history buffer
//
static void WriteRayHistoryTokenTopLevel(
    in uint     id,
    in uint64_t baseAddr)
{
    if (LogCounters(id, TRACERAY_COUNTER_MODE_RAYHISTORY_FULL))
    {
        uint offset = WriteRayHistoryControlToken(id,
                                                  RAY_HISTORY_TOKEN_TYPE_TOP_LEVEL,
                                                  RAY_HISTORY_TOKEN_TOP_LEVEL_SIZE,
                                                  0);

        Counters.Store2(offset, uint2(LowPart(baseAddr), HighPart(baseAddr)));
    }
}

//=====================================================================================================================
// Write 64-bit bottom-level acceleration structure base address to ray history buffer.
//
static void WriteRayHistoryTokenBottomLevel(
    in uint     id,
    in uint64_t baseAddr)
{
    if (LogCounters(id, TRACERAY_COUNTER_MODE_RAYHISTORY_FULL))
    {
        uint offset = WriteRayHistoryControlToken(id,
                                                  RAY_HISTORY_TOKEN_TYPE_BOTTOM_LEVEL,
                                                  RAY_HISTORY_TOKEN_BOTTOM_LEVEL_SIZE,
                                                  0);

        Counters.Store2(offset, uint2(LowPart(baseAddr), HighPart(baseAddr)));
    }
}

//=====================================================================================================================
// Write per-TraceRay data to ray history buffer
//
// id                : Unique caller generated ray identifier
// dispatchRaysIndex : Dispatch grid coordinates for this ray
// topLevelBvh       : Base address of top-level acceleration structure
// rayFlags          : API ray flags
// traceRayParams    : TraceRay parameters packed into a 32-bit integer. See RayHistoryTokenBeginData.packedTraceRayParams
// ray               : API ray description
//
static void WriteRayHistoryTokenBegin(
    in uint     id,
    in uint3    dispatchRaysIndex,
    in uint64_t topLevelBvh,
    in uint     rayFlags,
    in uint     traceRayParams,
    in RayDesc  ray)
{
    if (LogCounters(id, TRACERAY_COUNTER_MODE_RAYHISTORY_FULL) ||
        LogCounters(id, TRACERAY_COUNTER_MODE_RAYHISTORY_LIGHT))
    {
        // Write ray history begin tokens
        uint offset = WriteRayHistoryControlToken(id,
                                                  RAY_HISTORY_TOKEN_TYPE_BEGIN,
                                                  RAY_HISTORY_TOKEN_BEGIN_SIZE,
                                                  0);

        Counters.Store4(offset,
            uint4(AmdTraceRayGetHwWaveId(), dispatchRaysIndex));

        Counters.Store4(offset + 0x10,
            uint4(LowPart(topLevelBvh), HighPart(topLevelBvh), rayFlags, traceRayParams));

        Counters.Store4(offset + 0x20,
            uint4(asuint(ray.Origin.x), asuint(ray.Origin.y), asuint(ray.Origin.z), asuint(ray.TMin)));

        Counters.Store4(offset + 0x30,
            uint4(asuint(ray.Direction.x), asuint(ray.Direction.y), asuint(ray.Direction.z), asuint(ray.TMax)));
    }
}

//=====================================================================================================================
// Write per-TraceRay end data
//
// id   : Unique caller generated ray identifier
// data : GeometryIndex and PrimitiveIndex as uint2
//
static void WriteRayHistoryTokenEnd(
    in uint  id,
    in uint2 data)
{
    if (LogCounters(id, TRACERAY_COUNTER_MODE_RAYHISTORY_FULL))
    {
        uint offset = WriteRayHistoryControlToken(id,
                                                  RAY_HISTORY_TOKEN_TYPE_END,
                                                  RAY_HISTORY_TOKEN_END_SIZE,
                                                  0);

        Counters.Store2(offset, data);
    }
}

//=====================================================================================================================
// Write per-ray function call data
//
// id             : Unique caller generated ray identifier
// shaderId       : 64-bit unique shader ID
// shaderTableIdx : Shader table index
// shaderType     : Type of the shader called (RAY_HISTORY_FUNC_CALL_TYPE_MISS, etc)
//
static void WriteRayHistoryTokenFunctionCall(
    in uint  id,
    in uint2 shaderId,
    in uint  shaderTableIdx,
    in uint  shaderType)
{
    if (LogCounters(id, TRACERAY_COUNTER_MODE_RAYHISTORY_FULL))
    {
        uint offset = WriteRayHistoryControlToken(id,
                                                  RAY_HISTORY_TOKEN_TYPE_FUNC_CALL_V2,
                                                  RAY_HISTORY_TOKEN_FUNC_CALL_V2_SIZE,
                                                  (shaderType & RAY_HISTORY_CONTROL_TOKEN_DATA_MASK));

        Counters.Store2(offset, shaderId);
        Counters.Store(offset + 8, shaderTableIdx);
    }
}

//=====================================================================================================================
// Write per-ray AnyHit call status
//
// id     : Unique caller generated ray identifier
// status : hit status post anyHit call. Could be 0: ignore, 1: accept. 2: acceptAndEndSearch
//
static void WriteRayHistoryTokenAnyHitStatus(
    in uint id,
    in uint status)
{
    if (LogCounters(id, TRACERAY_COUNTER_MODE_RAYHISTORY_LIGHT))
    {
        uint offset = WriteRayHistoryControlToken(id,
                                                  RAY_HISTORY_TOKEN_TYPE_ANYHIT_STATUS,
                                                  0,
                                                  status);
    }
}

//=====================================================================================================================
// Write per-ray Intersection shader call status
//
// id     : Unique caller generated ray identifier
// status : hit status post intersection shader call. Could be 0: ignore, 1: accept. 2: acceptAndEndSearch
// hitT   : hit distance as reported by the intersection shader
// hitKind: hit kind as reported by the intersection shader
//
static void WriteRayHistoryTokenProceduralIntersectionStatus(
    in uint  id,
    in uint  status,
    in float hitT,
    in uint  hitKind)
{
    if (LogCounters(id, TRACERAY_COUNTER_MODE_RAYHISTORY_LIGHT))
    {
        uint offset = WriteRayHistoryControlToken(id,
                                                  RAY_HISTORY_TOKEN_TYPE_PROC_ISECT_STATUS,
                                                  RAY_HISTORY_TOKEN_PROC_ISECT_DATA_SIZE,
                                                  status);
        Counters.Store2(offset, uint2(asuint(hitT), hitKind));
    }
}

//=====================================================================================================================
// Write per-ray gpu timestamp
//
// id        : Unique caller generated ray identifier
// timeStamp : 64-bit timestamp provided by SampleGpuTimer()
static void WriteRayHistoryTokenTimeStamp(
    in uint     id,
    in uint64_t timeStamp)
{
    if (LogCounters(id, TRACERAY_COUNTER_MODE_RAYHISTORY_FULL))
    {
        uint offset = WriteRayHistoryControlToken(id,
                                                  RAY_HISTORY_TOKEN_TYPE_GPU_TIME,
                                                  RAY_HISTORY_TOKEN_GPU_TIME_SIZE,
                                                  0);

        Counters.Store2(offset, uint2(LowPart(timeStamp), HighPart(timeStamp)));
    }
}

//=====================================================================================================================
static void WriteTraversalCounter(
    in uint             rayId,
    in TraversalCounter counter)
{
    if (LogCounters(rayId, TRACERAY_COUNTER_MODE_TRAVERSAL))
    {
        // Reserve token space and write data. (+1 for rayID)
        uint offset = ReserveTokenSpace(TCID_COUNT + 1);

        // Write rayID
        Counters.Store(offset, rayId);
        offset += TCID_STRIDE;

        // Followed by counter data
        for (uint i = 0; i < TCID_COUNT; ++i)
        {
            Counters.Store(offset, counter.data[i]);
            offset += TCID_STRIDE;
        }
    }
}

//=====================================================================================================================
static void WriteTraversalCounter(inout_param(RayQueryInternal) rayQuery, in uint rayId)
{
    TraversalCounter counter;
    counter.data[TCID_NUM_RAY_BOX_TEST]       = rayQuery.numRayBoxTest;
    counter.data[TCID_NUM_RAY_TRIANGLE_TEST]  = rayQuery.numRayTriangleTest;
    counter.data[TCID_NUM_ITERATION]          = rayQuery.numIterations;
    counter.data[TCID_MAX_TRAVERSAL_DEPTH]    = rayQuery.maxStackDepth;
    counter.data[TCID_NUM_ANYHIT_INVOCATION]  = 0;
    counter.data[TCID_SHADER_ID]              = 0;
    counter.data[TCID_SHADER_RECORD_INDEX]    = 0;
    counter.data[TCID_TIMING_DATA]            = rayQuery.clocks;
    counter.data[TCID_WAVE_ID]                = AmdTraceRayGetHwWaveId();
    counter.data[TCID_NUM_CANDIDATE_HITS]     = rayQuery.numCandidateHits;
    counter.data[TCID_INSTANCE_INTERSECTIONS] = rayQuery.instanceIntersections;

    WriteTraversalCounter(rayId, counter);
}

//=====================================================================================================================
static uint64_t SampleGpuTimer()
{
    uint clocksHi = 0;
    uint clocksLo = 0;
    AmdTraceRaySampleGpuTimer(clocksHi, clocksLo);
    uint64_t clocks = clocksHi;
    clocks = (clocks << 32) | clocksLo;

    return clocks;
}

//=====================================================================================================================
static void WriteDispatchCounters(
    in uint numIterations)
{
    if (LogCounters(0, TRACERAY_COUNTER_MODE_DISPATCH))
    {
        const uint cnt = WaveActiveCountBits(true);
        const uint sum = WaveActiveSum(numIterations);
        const uint min = WaveActiveMin(numIterations);
        const uint max = WaveActiveMax(numIterations);

        if (WaveIsFirstLane())
        {
            Counters.InterlockedAdd(0, cnt);
            Counters.InterlockedAdd(4, sum);
            Counters.InterlockedMin(8, min);
            Counters.InterlockedMax(12, max);
        }
    }
}

//=====================================================================================================================
static void UpdateWaveTraversalStatistics(
    in uint nodePtr
)
{
    if (LogCounters(0, TRACERAY_COUNTER_MODE_DISPATCH))
    {
        const uint activeLaneCount = WaveActiveCountBits(true);
        uint4 laneCnt;
        laneCnt.x = WaveActiveCountBits(IsBoxNode(nodePtr));
        laneCnt.y = WaveActiveCountBits(IsTriangleNode(nodePtr));
        laneCnt.z = WaveActiveCountBits(IsUserNodeInstance(nodePtr));
        laneCnt.w = WaveActiveCountBits(IsUserNodeProcedural(nodePtr));

        if (WaveIsFirstLane())
        {
            // activeLaneCount per iteration
            Counters.InterlockedAdd(16, activeLaneCount);

            // wave iterations
            Counters.InterlockedAdd(20, 1);

            // Max active lane count with common node type
            const uint maxActiveLaneCnt = max(laneCnt.x, max(laneCnt.y, max(laneCnt.z, laneCnt.w)));
            Counters.InterlockedAdd(24, maxActiveLaneCnt);
        }
    }
}
#endif
