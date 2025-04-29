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
#ifndef RAYPIPELINEFLAGS_HLSLI
#define RAYPIPELINEFLAGS_HLSLI

#include "../common/Extensions.hlsli"
#include "../common/ShaderDefs.hlsli"

#if DEVELOPER
#include "DispatchRaysConstants.hlsli"
#endif

//=====================================================================================================================
// Apply the known set/unset bits
static uint ApplyKnownFlags(
    uint incomingFlags)
{
    uint flags = incomingFlags;

#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION  >= 41
    // Apply known bits common to all TraceRay calls
    flags = ((flags & ~AmdTraceRayGetKnownUnsetRayFlags()) | AmdTraceRayGetKnownSetRayFlags());
#endif

    // Apply options overrides
    flags &= ~Options::getRayFlagsOverrideForceDisableMask();
    flags |=  Options::getRayFlagsOverrideForceEnableMask();

    return flags;
}

//=====================================================================================================================
// Apply compile time pipeline config flags only, it does not apply known common flags from TraceRay call sites
static uint ApplyCompileTimePipelineConfigFlags(
    uint incomingFlags)
{
    uint flags = incomingFlags;

    flags |= (AmdTraceRayGetStaticFlags() & (PIPELINE_FLAG_SKIP_PROCEDURAL_PRIMITIVES | PIPELINE_FLAG_SKIP_TRIANGLES));
#if DEVELOPER
    flags |= DispatchRaysConstBuf.profileRayFlags;
#endif

    return flags;
}

//=====================================================================================================================
// Apply all static known flags, include both compile time pipeline config flags and known set/unset bits
static uint ApplyAllStaticallyKnownFlags(
    uint incomingFlags)     // The flags from TraceRay call sites,
                            // 0 means get Pipeline flags for all shaders in this pipeline
{
    return ApplyCompileTimePipelineConfigFlags(ApplyKnownFlags(incomingFlags));
}

#endif
