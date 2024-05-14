/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#pragma once
#include "gpurt.h"
#include "gpurtBackend.h"

namespace GpuRt
{

//=====================================================================================================================
// Generate 64-bit PSO hash for internal build shader type
inline uint64 GetInternalPsoHash(
    InternalRayTracingCsType        type,
    const CompileTimeBuildSettings& buildSettings)
{
    // Identifies ray-tracing-related dispatches for tools
    constexpr uint64 RayTracingPsoHashPrefix = 0xEEE5FFF6;

    // Layout used for build shader PSO hashes.
    union BuildShaderPsoHash
    {
        struct
        {
            uint64 shaderType              : 18;
            uint64 topLevelBuild           : 1;
            uint64 buildMode               : 2;
            uint64 reserved                : 1;
            uint64 triangleCompressionMode : 2;
            uint64 doTriangleSplitting     : 1;
            uint64 unused                  : 1;
            uint64 fp16BoxNodesMode        : 2;
            uint64 radixSortScanLevel      : 2;
            uint64 rebraidType             : 2;
            uint64 hashPrefix              : 32;
        };

        uint64 u64All;
    };

    BuildShaderPsoHash hash{};
    hash.shaderType              = static_cast<uint64>(type);
    hash.topLevelBuild           = buildSettings.topLevelBuild;
    hash.buildMode               = buildSettings.buildMode;
    hash.triangleCompressionMode = buildSettings.triangleCompressionMode;
    hash.doTriangleSplitting     = buildSettings.doTriangleSplitting;
    hash.fp16BoxNodesMode        = buildSettings.fp16BoxNodesMode;
    hash.radixSortScanLevel      = buildSettings.radixSortScanLevel;
    hash.rebraidType             = buildSettings.rebraidType;
    hash.hashPrefix              = RayTracingPsoHashPrefix;

    return hash.u64All;
}

//=====================================================================================================================
// Returns sum of components sizes (bpp) divided by 8.
// i.e. R10G10B10A2_Unorm => (10+10+10+2)/8 = 4
inline uint8 GetBytesPerPixelForFormat(BufferViewFormat format)
{
    switch (format)
    {
    case BufferViewFormat::R32G32B32_Float:
        return 12;
    case BufferViewFormat::R32G32_Float:
    case BufferViewFormat::R16G16B16A16_Float:
    case BufferViewFormat::R16G16B16A16_Snorm:
    case BufferViewFormat::R16G16B16A16_Unorm:
        return 8;
    case BufferViewFormat::R32_Float:
    case BufferViewFormat::R16G16_Float:
    case BufferViewFormat::R16G16_Snorm:
    case BufferViewFormat::R16G16_Unorm:
    case BufferViewFormat::R10G10B10A2_Unorm:
    case BufferViewFormat::R8G8B8A8_Snorm:
    case BufferViewFormat::R8G8B8A8_Unorm:
        return 4;
    case BufferViewFormat::R16_Float:
    case BufferViewFormat::R16_Snorm:
    case BufferViewFormat::R16_Unorm:
    case BufferViewFormat::R8G8_Snorm:
    case BufferViewFormat::R8G8_Unorm:
        return 2;
    case BufferViewFormat::R8_Snorm:
    case BufferViewFormat::R8_Unorm:
    case BufferViewFormat::Undefined:
        return 1;
    default:
        PAL_ASSERT_ALWAYS();
        return 0;
    }
}

//=====================================================================================================================
// Returns the single-component version of a buffer format.
inline BufferViewFormat GetSingleComponentFormatForFormat(BufferViewFormat format)
{
    switch (format)
    {
    case BufferViewFormat::R32G32B32_Float:
    case BufferViewFormat::R32G32_Float:
    case BufferViewFormat::R32_Float:
        return BufferViewFormat::R32_Float;

    case BufferViewFormat::R16G16B16A16_Float:
    case BufferViewFormat::R16G16_Float:
    case BufferViewFormat::R16_Float:
        return BufferViewFormat::R16_Float;

    case BufferViewFormat::R16G16B16A16_Snorm:
    case BufferViewFormat::R16G16_Snorm:
    case BufferViewFormat::R16_Snorm:
        return BufferViewFormat::R16_Snorm;

    case BufferViewFormat::R16G16B16A16_Unorm:
    case BufferViewFormat::R16G16_Unorm:
    case BufferViewFormat::R16_Unorm:
        return BufferViewFormat::R16_Unorm;

    case BufferViewFormat::R10G10B10A2_Unorm:
        return BufferViewFormat::Undefined;

    case BufferViewFormat::R8G8B8A8_Snorm:
    case BufferViewFormat::R8G8_Snorm:
    case BufferViewFormat::R8_Snorm:
        return BufferViewFormat::R8_Snorm;

    case BufferViewFormat::R8G8B8A8_Unorm:
    case BufferViewFormat::R8G8_Unorm:
    case BufferViewFormat::R8_Unorm:
        return BufferViewFormat::R8_Unorm;

    default:
        PAL_ASSERT_ALWAYS();
        return BufferViewFormat::Undefined;
    }
}

//=====================================================================================================================
// Converts the value of a Pal::HwPipePoint into a GpuRt::HwPipePoint without undefined behavior.
inline HwPipePoint PalToGpuRtHwPipePoint(uint32 palHwPipePoint)
{
#define HWPIPEPOINTCASE(x) case static_cast<uint32>(HwPipePoint::x): return HwPipePoint::x
    switch (palHwPipePoint)
    {
        HWPIPEPOINTCASE(HwPipeTop);
        HWPIPEPOINTCASE(HwPipePreCs);
        HWPIPEPOINTCASE(HwPipeBottom);
        default:
            PAL_ASSERT_ALWAYS_MSG("Unhandled HwPipePoint value in conversion: %u\n", palHwPipePoint);
            return HwPipePoint::HwPipeTop;
    }
#undef HWPIPEPOINTCASE
}

//=====================================================================================================================
// Return the number of components for a buffer view format when it's used as a vertex format.
inline uint8 GetNumComponentsForVertexFormat(VertexFormat format)
{
    switch (format)
    {
    case VertexFormat::R32G32B32_Float:
    case VertexFormat::R16G16B16A16_Float:
    case VertexFormat::R16G16B16A16_Snorm:
    case VertexFormat::R16G16B16A16_Unorm:
    case VertexFormat::R10G10B10A2_Unorm:
    case VertexFormat::R8G8B8A8_Snorm:
    case VertexFormat::R8G8B8A8_Unorm:
        return 3;
    case VertexFormat::R32G32_Float:
    case VertexFormat::R16G16_Float:
    case VertexFormat::R16G16_Snorm:
    case VertexFormat::R16G16_Unorm:
    case VertexFormat::R8G8_Snorm:
    case VertexFormat::R8G8_Unorm:
        return 2;
    default:
        PAL_ASSERT_ALWAYS();
        return 0;
    }
}

//=====================================================================================================================
// Converts a GpuRt::VertexFormat into a GpuRt::BufferViewFormat.
inline BufferViewFormat VertexFormatToBufferViewFormat(VertexFormat format)
{
#define FORMAT_CASE(x) case VertexFormat::x: return BufferViewFormat::x;
    switch (format)
    {
    FORMAT_CASE(R32G32B32_Float);
    FORMAT_CASE(R32G32_Float);
    FORMAT_CASE(R16G16B16A16_Float);
    FORMAT_CASE(R16G16_Float);
    FORMAT_CASE(R16G16B16A16_Snorm);
    FORMAT_CASE(R16G16_Snorm);
    FORMAT_CASE(R16G16B16A16_Unorm);
    FORMAT_CASE(R16G16_Unorm);
    FORMAT_CASE(R10G10B10A2_Unorm);
    FORMAT_CASE(R8G8B8A8_Snorm);
    FORMAT_CASE(R8G8_Snorm);
    FORMAT_CASE(R8G8B8A8_Unorm);
    FORMAT_CASE(R8G8_Unorm);
    case VertexFormat::Invalid: return BufferViewFormat::Undefined;
    default:
        PAL_ASSERT_ALWAYS();
        return BufferViewFormat::Undefined;
    }
#undef FORMAT_CASE
}

//=====================================================================================================================
// A helper function for converting VertexFormat to size of its component in bytes.
inline uint8 GetBytesPerComponentForFormat(VertexFormat format)
{
    return GetBytesPerPixelForFormat(
        GetSingleComponentFormatForFormat(
            VertexFormatToBufferViewFormat(format)));
}

}
