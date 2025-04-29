/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef VPC_HLSLI
#define VPC_HLSLI

#include "../common/Extensions.hlsli"
#include "../common/ShaderDefs.hlsli"

#include "DispatchRaysConstants.hlsli"

//=====================================================================================================================
// The VPC (vector program counter) is an opaque compiler-controlled 32-bit value.
// The low 6 bits are used for storing metadata, the high 26 bits store a function address.
struct Vpc32 {
    uint32_t vpc;

#if defined(__cplusplus)
    Vpc32(uint32_t value) : vpc(value) {}
#endif

    uint32_t GetU32()
    {
        return vpc;
    }

    uint32_t GetFunctionAddr()
    {
        return (uint32_t)(vpc & 0xFFFFFFC0);
    }

    bool IsValid()
    {
        return vpc != 0;
    }
};

//=====================================================================================================================
static Vpc32 GetVpcFromShaderIdAddr(GpuVirtualAddress addr)
{
#ifdef __cplusplus
    return 1;
#else
    return Vpc32(ConstantLoadDwordAtAddr(addr));
#endif
}

//=====================================================================================================================
static Vpc32 GetVpcFromShaderIdTable(
    GpuVirtualAddress tableAddress,
    uint index,
    uint stride)
{
    return GetVpcFromShaderIdAddr(tableAddress + stride * index);
}

//=====================================================================================================================
// Returns the 32-bit part of the hit group shader id containing the AHS shader id.
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
#endif
static Vpc32 GetAnyHit32BitShaderId(
    uint hitGroupRecordIndex)
{
    const uint offset = DispatchRaysConstBuf.hitGroupTableStrideInBytes * hitGroupRecordIndex;

    const GpuVirtualAddress tableVa =
        PackUint64(DispatchRaysConstBuf.hitGroupTableBaseAddressLo, DispatchRaysConstBuf.hitGroupTableBaseAddressHi);
    if (tableVa == 0)
    {
       return Vpc32(0);
    }
    return Vpc32(ConstantLoadDwordAtAddr(tableVa + offset + 8));
}

//=====================================================================================================================
static Vpc32 GetTraversalVpc32()
{
    // NOTE: DXCP uses a table for TraceRay, thus a load to traceRayGpuVa retrieves the actual traversal function
    // address. But Vulkan does not use the table so far, traceRayGpuVa is already the traversal function address.
    return Vpc32(DispatchRaysConstBuf.traceRayGpuVaLo);
}

//=====================================================================================================================
static Vpc32 GetRayGenVpc32()
{
    return GetVpcFromShaderIdAddr(PackUint64(DispatchRaysConstBuf.rayGenerationTableAddressLo,
                                             DispatchRaysConstBuf.rayGenerationTableAddressHi));
}

#endif
