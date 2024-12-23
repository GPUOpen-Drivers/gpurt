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
#include "Math.hlsli"
#include "Bits.hlsli"

#include "Extensions.hlsli"

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
// GpuRt WaveClusterSum Intrinsics
float AmdExtD3DShaderIntrinsics_WaveClusterSum(float x, uint dxClusterSize)
{
    const uint clusterSize = (1u << (dxClusterSize - 1));
    return spirv_OpGroupNonUniformFAdd_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, x, clusterSize);
}

//=====================================================================================================================
// GpuRt WaveClusterMin Intrinsics
float AmdExtD3DShaderIntrinsics_WaveClusterMin(float x, uint dxClusterSize)
{
    const uint clusterSize = (1u << (dxClusterSize - 1));
    return spirv_OpGroupNonUniformFMin_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, x, clusterSize);
}

float2 AmdExtD3DShaderIntrinsics_WaveClusterMin(float2 val, uint dxClusterSize)
{
    float2 result;
    const uint clusterSize = (1u << (dxClusterSize - 1));
    result.x = spirv_OpGroupNonUniformFMin_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, val.x, clusterSize);
    result.y = spirv_OpGroupNonUniformFMin_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, val.y, clusterSize);
    return result;
}

float3 AmdExtD3DShaderIntrinsics_WaveClusterMin(float3 val, uint dxClusterSize)
{
    float3 result;
    const uint clusterSize = (1u << (dxClusterSize - 1));
    result.x = spirv_OpGroupNonUniformFMin_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, val.x, clusterSize);
    result.y = spirv_OpGroupNonUniformFMin_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, val.y, clusterSize);
    result.z = spirv_OpGroupNonUniformFMin_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, val.z, clusterSize);
    return result;
}

//=====================================================================================================================
// GpuRt WaveClusterMax Intrinsics
float AmdExtD3DShaderIntrinsics_WaveClusterMax(float val, uint dxClusterSize)
{
    const uint clusterSize = (1u << (dxClusterSize - 1));
    return spirv_OpGroupNonUniformFMax_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, val, clusterSize);
}

float2 AmdExtD3DShaderIntrinsics_WaveClusterMax(float2 val, uint dxClusterSize)
{
    float2 result;
    const uint clusterSize = (1u << (dxClusterSize - 1));
    result.x = spirv_OpGroupNonUniformFMax_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, val.x, clusterSize);
    result.y = spirv_OpGroupNonUniformFMax_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, val.y, clusterSize);
    return result;
}

float3 AmdExtD3DShaderIntrinsics_WaveClusterMax(float3 val, uint dxClusterSize)
{
    float3 result;
    const uint clusterSize = (1u << (dxClusterSize - 1));
    result.x = spirv_OpGroupNonUniformFMax_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, val.x, clusterSize);
    result.y = spirv_OpGroupNonUniformFMax_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, val.y, clusterSize);
    result.z = spirv_OpGroupNonUniformFMax_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, val.z, clusterSize);
    return result;
}

//=====================================================================================================================
// GpuRt WaveClusterBitAnd Intrinsics
uint AmdExtD3DShaderIntrinsics_WaveClusterBitAnd(uint x, uint dxClusterSize)
{
    const uint clusterSize = (1u << (dxClusterSize - 1));
    return spirv_OpGroupNonUniformBitwiseAnd_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, x, clusterSize);
}

//=====================================================================================================================
// GpuRt WaveClusterBitOr Intrinsics
uint AmdExtD3DShaderIntrinsics_WaveClusterBitOr(uint x, uint dxClusterSize)
{
    const uint clusterSize = (1u << (dxClusterSize - 1));
    return spirv_OpGroupNonUniformBitwiseOr_clustered(AmdExtClusteredSubgroup, AmdExtClusteredReduce, x, clusterSize);
}

// Driver intrinsic that returns the dispatch rays index for the current thread
__decl uint3 AmdTraceRayDispatchRaysIndex()
    DUMMY_UINT3_FUNC

//=====================================================================================================================
static uint LoadDwordAtAddr(GpuVirtualAddress addr)
{
    return AmdExtD3DShaderIntrinsics_LoadDwordAtAddr(LowPart(addr), HighPart(addr), 0);
}

//=====================================================================================================================
static uint2 LoadDwordAtAddrx2(GpuVirtualAddress addr)
{
    return AmdExtD3DShaderIntrinsics_LoadDwordAtAddrx2(LowPart(addr), HighPart(addr), 0);
}

//=====================================================================================================================
static uint3 LoadDwordAtAddrx3(GpuVirtualAddress addr)
{
    return AmdExtD3DShaderIntrinsics_LoadDwordAtAddrx3(LowPart(addr), HighPart(addr), 0);
}

//=====================================================================================================================
static uint4 LoadDwordAtAddrx4(GpuVirtualAddress addr)
{
    return AmdExtD3DShaderIntrinsics_LoadDwordAtAddrx4(LowPart(addr), HighPart(addr), 0);
}

static uint ConstantLoadDwordAtAddr(GpuVirtualAddress addr)
{
    return AmdExtD3DShaderIntrinsics_ConstantLoadDwordAtAddr(LowPart(addr), HighPart(addr), 0);
}

static uint64_t ConstantLoadDwordAtAddrx2(GpuVirtualAddress addr)
{
    uint2 retVal = AmdExtD3DShaderIntrinsics_ConstantLoadDwordAtAddrx2(LowPart(addr), HighPart(addr), 0);
    return PackUint64(retVal.x, retVal.y);
}

static uint4 ConstantLoadDwordAtAddrx4(GpuVirtualAddress addr)
{
    return AmdExtD3DShaderIntrinsics_ConstantLoadDwordAtAddrx4(LowPart(addr), HighPart(addr), 0);
}

