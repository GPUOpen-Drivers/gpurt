/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2022 Advanced Micro Devices, Inc. All Rights Reserved.
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
__decl uint AmdExtD3DShaderIntrinsics_LoadDwordAtAddr(
    uint gpuVaLoBits, uint gpuVaHiBits, uint offset) DUMMY_UINT_FUNC

__decl uint AmdExtLaneIndex() DUMMY_UINT_FUNC

__decl uint AmdExtLaneCount() DUMMY_UINT_FUNC

__decl uint2 AmdExtD3DShaderIntrinsics_LoadDwordAtAddrx2(
    uint gpuVaLoBits, uint gpuVaHiBits, uint offset) DUMMY_UINT2_FUNC

__decl uint4 AmdExtD3DShaderIntrinsics_LoadDwordAtAddrx4(
    uint gpuVaLoBits, uint gpuVaHiBits, uint offset) DUMMY_UINT4_FUNC

__decl uint2 AmdExtD3DShaderIntrinsics_AtomicMinU64(
    RWByteAddressBuffer uav, uint address, uint2 value) DUMMY_UINT2_FUNC

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

__decl uint4 AmdExtD3DShaderIntrinsics_IntersectBvhNode(
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

__decl void AmdTraceRayGetHitAttributes(
    inout_param(float) tCurrent,    ///< Current parametric hit distance relative to TMin
    inout_param(uint)  kind,        ///< Intersection hit kind
    out_param(uint)    status       ///< Hit status
) DUMMY_VOID_FUNC

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
__decl void AmdTraceRaySetHitTriangleNodePointer(
    in GpuVirtualAddress bvhAddress, // The BVH address
    in uint              nodePointer // Node pointer of hit triangle
) DUMMY_VOID_FUNC

__decl uint AmdTraceRayLdsRead(uint offset) DUMMY_UINT_FUNC
__decl uint AmdTraceRayLdsWrite(uint offset, uint data) DUMMY_UINT_FUNC
__decl uint AmdTraceRayLdsStackStore(inout_param(uint) stackAddr, uint lastVisited, uint4 data) DUMMY_UINT_FUNC
__decl uint AmdTraceRayLdsStackInit() DUMMY_UINT_FUNC
__decl void AmdTraceRaySampleGpuTimer(out_param(uint) timerHi, out_param(uint) timerLo) DUMMY_VOID_FUNC
__decl uint AmdTraceRayGetHwCuId() DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetHwWaveId() DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetHwSimdId() DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetHwSeId() DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetTriangleCompressionMode() DUMMY_UINT_FUNC
__decl uint AmdTraceRayGetBoxSortHeuristicMode() DUMMY_UINT_FUNC
__decl uint2 AmdTraceRayMakePC(uint pcVaLow) DUMMY_UINT2_FUNC

//=====================================================================================================================
// Ref: GpuRt::Device::GetStaticPipelineFlags
__decl uint AmdTraceRayGetStaticFlags() DUMMY_UINT_FUNC

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
#endif
