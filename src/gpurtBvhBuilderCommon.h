/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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

#include <cstdint>

#if GPURT_DEVELOPER
#if defined(__GNUC__)
#define RGP_PUSH_MARKER(format, ...) PushRGPMarker(format, ##__VA_ARGS__)
#else
#define RGP_PUSH_MARKER(format, ...) PushRGPMarker(format, __VA_ARGS__)
#endif
#define RGP_POP_MARKER() PopRGPMarker()
#else
#define RGP_PUSH_MARKER(format, ...) (static_cast<void>(0))
#define RGP_POP_MARKER() (static_cast<void>(0))
#endif
namespace GpuRt
{

enum class BuildPhaseFlags : uint32_t
{
    Rebraid                       = 1 << 0,
    // Unused1                    = 1 << 1,
    EncodeHwBvh                   = 1 << 2,
    GenerateMortonCodes           = 1 << 3,
    MergeSort                     = 1 << 4,
    RadixSort                     = 1 << 5,
    // Unused6                    = 1 << 6,
    BuildPLOC                     = 1 << 7,
    // Unused8                    = 1 << 8,
    PairCompression               = 1 << 9,
#if GPURT_BUILD_RTIP3_1
    CompressPrims                 = 1 << 10,
    RefitOrientedBounds           = 1 << 11,
#endif
    SeparateEmitPostBuildInfoPass = 1 << 12,
    BuildParallel                 = 1 << 13,
    BuildFastAgglomerativeLbvh    = 1 << 15,
    EncodeQuadPrimitives          = 1 << 16,
    BuildHPLOC                    = 1 << 17,
    EncodePrimitives              = 1 << 18,
    BuildDumpEvents               = 1 << 19,
#if GPURT_BUILD_RTIP3_1
    RefitInstanceBounds           = 1 << 21,
#endif
};

static const char* BuildPhaseName(BuildPhaseFlags phase)
{
    switch (phase)
    {
    case GpuRt::BuildPhaseFlags::Rebraid:
        return "Rebraid";
    case GpuRt::BuildPhaseFlags::EncodeHwBvh:
        return "EncodeHwBvh";
    case GpuRt::BuildPhaseFlags::GenerateMortonCodes:
        return "GenerateMortonCodes";
    case GpuRt::BuildPhaseFlags::MergeSort:
        return "MergeSort";
    case GpuRt::BuildPhaseFlags::RadixSort:
        return "RadixSort";
    case GpuRt::BuildPhaseFlags::BuildPLOC:
        return "BuildPLOC";
    case GpuRt::BuildPhaseFlags::PairCompression:
        return "PairCompression";
#if GPURT_BUILD_RTIP3_1
    case GpuRt::BuildPhaseFlags::CompressPrims:
        return "CompressPrims";
    case GpuRt::BuildPhaseFlags::RefitOrientedBounds:
        return "RefitOrientedBounds";
#endif
    case GpuRt::BuildPhaseFlags::SeparateEmitPostBuildInfoPass:
        return "SeparateEmitPostBuildInfoPass";
    case GpuRt::BuildPhaseFlags::BuildParallel:
        return "BuildParallel";
    case GpuRt::BuildPhaseFlags::BuildFastAgglomerativeLbvh:
        return "BuildFastAgglomerativeLbvh";
    case GpuRt::BuildPhaseFlags::EncodeQuadPrimitives:
        return "EncodeQuadPrimitives";
    case GpuRt::BuildPhaseFlags::BuildHPLOC:
        return "BuildHPLOC";
    case GpuRt::BuildPhaseFlags::EncodePrimitives:
        return "EncodePrimitives";
    case GpuRt::BuildPhaseFlags::BuildDumpEvents:
        return "BuildDumpEvents";
    }
    return "";
}

constexpr BuildPhaseFlags& operator|=(BuildPhaseFlags& a, BuildPhaseFlags b) noexcept
{
    a = static_cast<BuildPhaseFlags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
    return a;
}

} // namespace GpuRt
