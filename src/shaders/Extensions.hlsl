/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _EXTENSIONS_HLSL
#define _EXTENSIONS_HLSL

#if !defined(__cplusplus)

#define __decl [noinline]

// Dummy implementation for Vulkan build only
__decl uint AmdExtLaneCount() DUMMY_UINT_FUNC

__decl uint AmdExtD3DShaderIntrinsics_LoadDwordAtAddr(
    uint gpuVaLoBits, uint gpuVaHiBits, uint offset) DUMMY_UINT_FUNC

__decl uint2 AmdExtD3DShaderIntrinsics_LoadDwordAtAddrx2(
    uint gpuVaLoBits, uint gpuVaHiBits, uint offset) DUMMY_UINT2_FUNC

__decl uint3 AmdExtD3DShaderIntrinsics_LoadDwordAtAddrx3(
    uint gpuVaLoBits, uint gpuVaHiBits, uint offset) DUMMY_UINT3_FUNC

__decl uint4 AmdExtD3DShaderIntrinsics_LoadDwordAtAddrx4(
    uint gpuVaLoBits, uint gpuVaHiBits, uint offset) DUMMY_UINT4_FUNC

__decl uint AmdExtD3DShaderIntrinsics_ConstantLoadDwordAtAddr(
    uint gpuVaLoBits, uint gpuVaHiBits, uint offset) DUMMY_UINT_FUNC

__decl uint2 AmdExtD3DShaderIntrinsics_ConstantLoadDwordAtAddrx2(
    uint gpuVaLoBits, uint gpuVaHiBits, uint offset) DUMMY_UINT2_FUNC

__decl uint4 AmdExtD3DShaderIntrinsics_ConstantLoadDwordAtAddrx4(
    uint gpuVaLoBits, uint gpuVaHiBits, uint offset) DUMMY_UINT4_FUNC

__decl uint2 AmdExtD3DShaderIntrinsics_AtomicMinU64(
    RWByteAddressBuffer uav, uint address, uint2 value) DUMMY_UINT2_FUNC

__decl uint2 AmdExtD3DShaderIntrinsics_ShaderClock() DUMMY_UINT2_FUNC
__decl uint2 AmdExtD3DShaderIntrinsics_ShaderRealtimeClock() DUMMY_UINT2_FUNC

#define AmdExtD3DShaderIntrinsicsWaveOp_MinF        0x07
#define AmdExtD3DShaderIntrinsicsWaveOp_MaxF        0x0a
#define AmdExtD3DShaderIntrinsicsWaveOp_Inclusive   0x01

__decl float3 AmdExtD3DShaderIntrinsics_WaveScan(
    uint waveOp, uint flags, float3 src) DUMMY_FLOAT3_FUNC

// globallycoherent version
__decl uint2 AmdExtD3DShaderIntrinsics_AtomicMinU64_gc(
    globallycoherent RWByteAddressBuffer uav, uint address, uint2 value) DUMMY_UINT2_FUNC

__decl void AmdExtD3DShaderIntrinsics_Halt() DUMMY_VOID_FUNC

__decl void AmdExtD3DShaderIntrinsics_ShaderMarker(in uint data) DUMMY_VOID_FUNC

#ifdef AMD_VULKAN // To fix Vulkan compilation
/**
***********************************************************************************************************************
*   AmdExtIntersectBvhNodeFlag defines for supported flags for BVH srd.
*   To be used as an input AmdExtD3DShaderIntrinsicsOpcode_IntersectBvhNode instruction
***********************************************************************************************************************
*/
#define AmdExtIntersectBvhNodeFlag_None 0x0
#define AmdExtIntersectBvhNodeFlag_BoxSortEnable 0x1
#endif

__decl uint4 AmdExtD3DShaderIntrinsics_IntersectInternal(
    in uint2  nodePointer,
    in float  rayExtent,
    in float3 rayOrigin,
    in float3 rayDir,
    in float3 rayInvDir,
    in uint   flags,
    in uint   expansion)
{
    uint4 dummy;

    return dummy;
}

//=====================================================================================================================
// Floating point conversions
__decl uint3 AmdExtD3DShaderIntrinsics_ConvertF32toF16NegInf(in float3 inVec) DUMMY_UINT3_FUNC
__decl uint3 AmdExtD3DShaderIntrinsics_ConvertF32toF16PosInf(in float3 inVec) DUMMY_UINT3_FUNC

//=====================================================================================================================
// Floating point operations that set the round mode for the result
__decl float AmdExtD3DShaderIntrinsics_FloatOpWithRoundMode(
    uint roundMode, uint operation, float src0, float src1) DUMMY_FLOAT_FUNC
__decl float2 AmdExtD3DShaderIntrinsics_FloatOpWithRoundMode(
    uint roundMode, uint operation, float2 src0, float2 src1) DUMMY_FLOAT2_FUNC
__decl float3 AmdExtD3DShaderIntrinsics_FloatOpWithRoundMode(
    uint roundMode, uint operation, float3 src0, float3 src1) DUMMY_FLOAT3_FUNC

//=====================================================================================================================
// Sub-group wave reductions

[[vk::ext_capability(/* GroupNonUniform */ 61)]]
[[vk::ext_capability(/* GroupNonUniformArithmetic */ 63)]]
[[vk::ext_capability(/* GroupNonUniformClustered */ 67)]]

[[vk::ext_instruction(350)]]
float spirv_OpGroupNonUniformFAdd_clustered(uint scope, [[vk::ext_literal]] uint op, float value, uint clusterSize);

float AmdExtD3DShaderIntrinsics_WaveClusterSum(float x, uint dxClusterSize)
{
    const uint clusterSize = (1u << (dxClusterSize - 1));
    return spirv_OpGroupNonUniformFAdd_clustered(/* Subgroup */ 3, /* ClusteredReduce */ 3, x, clusterSize);
}

[[vk::ext_instruction(355)]]
float spirv_OpGroupNonUniformFMin_clustered(uint scope, [[vk::ext_literal]] uint op, float value, uint clusterSize);

float AmdExtD3DShaderIntrinsics_WaveClusterMin(float x, uint dxClusterSize)
{
    const uint clusterSize = (1u << (dxClusterSize - 1));
    return spirv_OpGroupNonUniformFMin_clustered(/* Subgroup */ 3, /* ClusteredReduce */ 3, x, clusterSize);
}

[[vk::ext_instruction(358)]]
float spirv_OpGroupNonUniformFMax_clustered(uint scope, [[vk::ext_literal]] uint op, float value, uint clusterSize);

float AmdExtD3DShaderIntrinsics_WaveClusterMax(float x, uint dxClusterSize)
{
    const uint clusterSize = (1u << (dxClusterSize - 1));
    return spirv_OpGroupNonUniformFMax_clustered(/* Subgroup */ 3, /* ClusteredReduce */ 3, x, clusterSize);
}

#endif

//=====================================================================================================================
// The following extension functions are driver intrinsic functions
//

// Driver intrinsic that returns the dispatch rays index for the current thread
__decl uint3 AmdTraceRayDispatchRaysIndex()
#if defined(AMD_VULKAN) || defined(__cplusplus)
    DUMMY_UINT3_FUNC
#else
{
    return DispatchRaysIndex();
}
#endif

// Driver closest hit shader inlining patch function. Driver will possibly static none or all closest hit shaders.
// Returns true if the call was inlined.
__decl bool AmdTraceRayCallClosestHitShader(
    uint2 shaderId, ///< Closest hit shader identifier
    uint tableIndex ///< Hit group shader record table index
) DUMMY_BOOL_FUNC

// Driver miss shader inlining patch function. Driver will possibly static none or all miss shaders. Returns true if
// the call was inlined.
__decl bool AmdTraceRayCallMissShader(
    uint2 shaderId, ///< Shader identifier
    uint tableIndex ///< Miss shader record table index
) DUMMY_BOOL_FUNC

// Driver anyhit shader inlining patch function. This function calls all anyhit shaders associated with
// triangle intersection (i.e. intersection shader is not specified in the hit group)
__decl void AmdTraceRayCallTriangleAnyHitShader(
    uint2 shaderId,     ///< AnyHit shader identifier
    uint  tableIndex,   ///< Hit group shader record index
    in BuiltInTriangleIntersectionAttributes attr   ///< Triangle barycentrics
) DUMMY_VOID_FUNC

// Driver intersection shader inlining patch function. This function inlines all intersection-anyhit shader groups
__decl void AmdTraceRayCallIntersectionShader(
    uint2 shaderId,         ///< Programmable intersection shader identifier
    uint2 anyHitShaderId,   ///< AnyHit shader identifier
    uint  tableIndex        ///< Hit group shader record index
) DUMMY_VOID_FUNC

__decl void AmdTraceRaySetHitTokenData(
    in uint blasPointer,    ///< blas pointer of closest hit
    in uint tlasPointer     ///< tlas pointer of closest hit
) DUMMY_VOID_FUNC

__decl void AmdTraceRaySetHitAttributes(
    in float  tCurrent,          ///< Current parametric hit distance relative to TMin
    in uint   kind,              ///< Intersection hit kind
    in uint   status,            ///< Hit status
    in uint   instNodeAddrLo,    ///< Current instance node address lo bits
    in uint   instNodeAddrHi,    ///< Current instance node address hi bits
    in uint   primitiveIndex,    ///< Current primitive index
    in uint   anyHitCallType,    ///< Indication of calling behavior on any hit shader
    in uint   geometryIndex      ///< Current geometry index
) DUMMY_VOID_FUNC

#pragma dxc diagnostic push
#pragma dxc diagnostic ignored "-Wparameter-usage"
__decl void AmdTraceRayGetHitAttributes(
    inout_param(float) tCurrent,    ///< Current parametric hit distance relative to TMin
    inout_param(uint)  kind,        ///< Intersection hit kind
    out_param(uint)    status       ///< Hit status
) DUMMY_VOID_FUNC
#pragma dxc diagnostic pop

// Driver notification of trace input parameters.
__decl void AmdTraceRaySetTraceParams(
    in    uint  rayFlags,      ///< Ray flags
#ifdef AMD_VULKAN
    in    uint  instanceInclusionMask,
#endif
    in    float originX,       ///< Ray origin X
    in    float originY,       ///< Ray origin Y
    in    float originZ,       ///< Ray origin Z
    in    float tMin,          ///< T min
    in    float dirX,          ///< World ray direction X
    in    float dirY,          ///< World ray direction Y
    in    float dirZ           ///< World ray direction Z
) DUMMY_VOID_FUNC

// Update the current hit attributes for a triangle hit
__decl void AmdTraceRaySetTriangleIntersectionAttributes(
    float2 barycentrics ///< Triangle barycentrics
) DUMMY_VOID_FUNC

// Driver notification of hit triangle node pointer

#ifdef AMD_VULKAN
__decl void AmdTraceRaySetHitTriangleNodePointer(
#else
static void AmdTraceRaySetHitTriangleNodePointer(
#endif
    in GpuVirtualAddress bvhAddress, // The BVH address
    in uint              nodePointer // Node pointer of hit triangle
)
{
    // For DXR,    it is really an empty function.
    // For Vulkan, it means the implementation is done somewhere. Glslang has an issue where the compiler can't deal
    //             with calls to functions that don't have bodies, so WA by using {} as the function body.
}

__decl uint AmdTraceRayLdsRead(uint offset) DUMMY_UINT_FUNC
__decl uint AmdTraceRayLdsWrite(uint offset, uint data) DUMMY_UINT_FUNC
__decl uint AmdTraceRayLdsStackStore(inout_param(uint) stackAddr, uint lastVisited, uint4 data) DUMMY_UINT_FUNC
__decl uint AmdTraceRayLdsStackInit() DUMMY_UINT_FUNC
#pragma dxc diagnostic push
#pragma dxc diagnostic ignored "-Wparameter-usage"
__decl void     AmdTraceRaySampleGpuTimer(out_param(uint) timerHi, out_param(uint) timerLo) DUMMY_VOID_FUNC
__decl uint64_t AmdTraceRaySampleGpuTimer() DUMMY_UINT_FUNC
#pragma dxc diagnostic pop
__decl uint AmdExtLaneIndex() DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetHwCuId() DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetHwWaveId() DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetHwSimdId() DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetHwSeId() DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetTriangleCompressionMode() DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetBoxSortHeuristicMode() DUMMY_UINT_FUNC
__decl uint2 AmdTraceRayMakePC(uint pcVaLow) DUMMY_UINT2_FUNC
__decl uint AmdTraceRayGetKnownSetRayFlags() DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetKnownUnsetRayFlags() DUMMY_UINT_FUNC
__decl void AmdTraceRayInitStaticId() DUMMY_VOID_FUNC

//=====================================================================================================================
// Ref: GpuRt::Device::GetStaticPipelineFlags
__decl uint AmdTraceRayGetStaticFlags() DUMMY_UINT_FUNC

//=====================================================================================================================
// Query flags statically known to be set, combining AmdTraceRayGetStaticFlags() (from pipeline flags)
// and AmdTraceRayGetKnownSetRayFlags() (from known bits in TraceRay arguments).
static uint AmdTraceRayGetAllKnownStaticallySetFlags()
{
    return AmdTraceRayGetStaticFlags() | AmdTraceRayGetKnownSetRayFlags();
}

//=====================================================================================================================
// fetch data from LDS and return parent TraceRay rayId
__decl uint AmdTraceRayGetParentId() DUMMY_UINT_FUNC

//=====================================================================================================================
// store the TraceRay rayId into LDS
__decl void AmdTraceRaySetParentId(uint rayId) DUMMY_VOID_FUNC

//=====================================================================================================================
// Client drivers must generate a unique 32-bit identifier for unique TraceRay/TraceRayInline call site in the API shaders.
__decl uint AmdTraceRayGetStaticId() DUMMY_UINT_FUNC;

//=====================================================================================================================
// constant load dword at address
__decl uint     AmdExtConstantLoadDwordAtAddr(GpuVirtualAddress addr, uint offset) DUMMY_UINT_FUNC
__decl uint64_t AmdExtConstantLoad64AtAddr(GpuVirtualAddress addr, uint offset) DUMMY_UINT_FUNC

//=====================================================================================================================
// Loads from reserved register the dispath thread id as a single value computed from the dispatch dimensions.
__decl uint AmdExtDispatchThreadIdFlat() DUMMY_UINT_FUNC;

//=====================================================================================================================
__decl uint64_t AmdExtAtomic64AddAtAddr(uint64_t gpuVa, uint offset, uint64_t value) DUMMY_UINT_FUNC
__decl uint64_t AmdExtAtomic64CmpXchgAtAddr(uint64_t gpuVa, uint offset, uint64_t compare_value, uint64_t value) DUMMY_UINT_FUNC
__decl uint64_t AmdExtLoad64AtAddrUncached(uint64_t gpuVa, uint offset) DUMMY_UINT_FUNC
__decl uint  AmdExtLoadDwordAtAddrUncached(uint64_t addr, uint offset) DUMMY_UINT_FUNC
__decl void  AmdExtStoreDwordAtAddrUncached(uint64_t addr, uint offset, uint value) DUMMY_VOID_FUNC
__decl uint3 AmdExtGroupIdCompute() DUMMY_UINT3_FUNC
__decl uint3 AmdExtGroupDimCompute() DUMMY_UINT3_FUNC
__decl void AmdExtSleep(uint value) DUMMY_VOID_FUNC

#if USE_TEMP_ARRAY_STACK
//=====================================================================================================================
// Register based stack (shared with __cplusplus path)
#define SHORT_STACK_SIZE  16

//=====================================================================================================================
static uint AmdTraceRayGetStackSize()
{
    return SHORT_STACK_SIZE;
}

//=====================================================================================================================
static uint AmdTraceRayGetStackBase()
{
    return 0;
}

//=====================================================================================================================
static uint AmdTraceRayGetStackStride()
{
    return 1;
}
#else
__decl uint AmdTraceRayGetStackBase()   DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetStackStride() DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetStackSize()   DUMMY_UINT_FUNC
#endif

#define ANYHIT_CALLTYPE_SKIP            0
#define ANYHIT_CALLTYPE_NO_DUPLICATE    1
#define ANYHIT_CALLTYPE_DUPLICATE       2

//=====================================================================================================================
static uint LowPart(GpuVirtualAddress addr)
{
    return uint(addr);
}

//=====================================================================================================================
static uint HighPart(GpuVirtualAddress addr)
{
    return uint(addr >> 32);
}

#ifdef __cplusplus
//=====================================================================================================================
static uint LoadDwordAtAddr(GpuVirtualAddress addr)
{
    return *reinterpret_cast<uint*>(addr);
}
#else
//=====================================================================================================================
static uint LoadDwordAtAddr(GpuVirtualAddress addr)
{
    return AmdExtD3DShaderIntrinsics_LoadDwordAtAddr(LowPart(addr), HighPart(addr), 0);
}
#endif

//=====================================================================================================================
static uint2 LoadDwordAtAddrx2(GpuVirtualAddress addr)
{
#if !defined(__cplusplus)
    return AmdExtD3DShaderIntrinsics_LoadDwordAtAddrx2(LowPart(addr), HighPart(addr), 0);
#else
    uint2 retVal;
    retVal.x = LoadDwordAtAddr(addr);
    retVal.y = LoadDwordAtAddr(addr + 4);

    return retVal;
#endif
}

//=====================================================================================================================
static uint3 LoadDwordAtAddrx3(GpuVirtualAddress addr)
{
#if !defined(__cplusplus)
    return AmdExtD3DShaderIntrinsics_LoadDwordAtAddrx3(LowPart(addr), HighPart(addr), 0);
#else
    uint3 retVal;
    retVal.x = LoadDwordAtAddr(addr);
    retVal.y = LoadDwordAtAddr(addr + 4);
    retVal.z = LoadDwordAtAddr(addr + 8);

    return retVal;
#endif
}

//=====================================================================================================================
static uint4 LoadDwordAtAddrx4(GpuVirtualAddress addr)
{
#if !defined(__cplusplus)
    return AmdExtD3DShaderIntrinsics_LoadDwordAtAddrx4(LowPart(addr), HighPart(addr), 0);
#else
    uint4 retVal;
    retVal.x = LoadDwordAtAddr(addr);
    retVal.y = LoadDwordAtAddr(addr + 4);
    retVal.z = LoadDwordAtAddr(addr + 8);
    retVal.w = LoadDwordAtAddr(addr + 12);

    return retVal;
#endif
}

static uint ConstantLoadDwordAtAddr(GpuVirtualAddress addr)
{
#if !defined(__cplusplus)
    return AmdExtD3DShaderIntrinsics_ConstantLoadDwordAtAddr(LowPart(addr), HighPart(addr), 0);
#else
    return AmdExtConstantLoadDwordAtAddr(addr, 0);
#endif
}

static uint64_t ConstantLoadDwordAtAddrx2(GpuVirtualAddress addr)
{
#if !defined(__cplusplus)
    uint2 retVal = AmdExtD3DShaderIntrinsics_ConstantLoadDwordAtAddrx2(LowPart(addr), HighPart(addr), 0);
    return PackUint64(retVal.x, retVal.y);
#else
    return AmdExtConstantLoad64AtAddr(addr, 0);
#endif
}

static uint4 ConstantLoadDwordAtAddrx4(GpuVirtualAddress addr)
{
#if !defined(__cplusplus)
    return AmdExtD3DShaderIntrinsics_ConstantLoadDwordAtAddrx4(LowPart(addr), HighPart(addr), 0);
#else
    uint4 retVal;
    retVal.xy = SplitUint64(AmdExtConstantLoad64AtAddr(addr, 0));
    retVal.zw = SplitUint64(AmdExtConstantLoad64AtAddr(addr + 8, 0));

    return retVal;
#endif
}

#if defined(AMD_VULKAN) || defined(__cplusplus)
#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_TiesToEven     0x0
#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_TowardPositive 0x1
#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_TowardNegative 0x2
#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_TowardZero     0x3

#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Add      0x0
#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Subtract 0x1
#define AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Multiply 0x2
#endif

#ifdef __cplusplus
#include <cfenv>
static constexpr uint RoundModeTable[] =
{
    FE_TONEAREST,
    FE_UPWARD,
    FE_DOWNWARD,
    FE_TOWARDZERO,
};

//=====================================================================================================================
static float FloatOpWithRoundMode(uint roundMode, uint operation, float src0, float src1)
{
    std::fesetround(RoundModeTable[roundMode]);

    float result;

    switch (operation)
    {
        case AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Add:
            result = src0 + src1;
            break;

        case AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Subtract:
            result = src0 - src1;
            break;

        case AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Multiply:
            result = src0 * src1;
            break;

        default:
            printf("Unknown operation for FloatOpWithRoundMode\n");
            assert(false);
            break;
    }

    std::fesetround(FE_TONEAREST);

    return result;
}

//=====================================================================================================================
static float2 FloatOpWithRoundMode(uint roundMode, uint operation, float2 src0, float2 src1)
{
    std::fesetround(RoundModeTable[roundMode]);

    float2 result;

    switch (operation)
    {
        case AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Add:
            result = src0 + src1;
            break;

        case AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Subtract:
            result = src0 - src1;
            break;

        case AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Multiply:
            result = src0 * src1;
            break;

        default:
            printf("Unknown operation for FloatOpWithRoundMode\n");
            assert(false);
            break;
    }

    std::fesetround(FE_TONEAREST);

    return result;
}

//=====================================================================================================================
static float3 FloatOpWithRoundMode(uint roundMode, uint operation, float3 src0, float3 src1)
{
    std::fesetround(RoundModeTable[roundMode]);

    float3 result;

    switch (operation)
    {
        case AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Add:
            result = src0 + src1;
            break;

        case AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Subtract:
            result = src0 - src1;
            break;

        case AmdExtD3DShaderIntrinsicsFloatOpWithRoundMode_Multiply:
            result = src0 * src1;
            break;

        default:
            printf("Unknown operation for FloatOpWithRoundMode\n");
            assert(false);
            break;
    }

    std::fesetround(FE_TONEAREST);

    return result;
}
#else
//=====================================================================================================================
static float FloatOpWithRoundMode(uint roundMode, uint operation, float src0, float src1)
{
    return AmdExtD3DShaderIntrinsics_FloatOpWithRoundMode(roundMode, operation, src0, src1);
}

//=====================================================================================================================
static float2 FloatOpWithRoundMode(uint roundMode, uint operation, float2 src0, float2 src1)
{
    return AmdExtD3DShaderIntrinsics_FloatOpWithRoundMode(roundMode, operation, src0, src1);
}

//=====================================================================================================================
static float3 FloatOpWithRoundMode(uint roundMode, uint operation, float3 src0, float3 src1)
{
    return AmdExtD3DShaderIntrinsics_FloatOpWithRoundMode(roundMode, operation, src0, src1);
}
#endif

#endif
