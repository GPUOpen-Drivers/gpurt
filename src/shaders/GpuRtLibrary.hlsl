/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2021-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _GPURT_LIBRARY_HLSL
#define _GPURT_LIBRARY_HLSL

#include "../shadersClean/traversal/TraversalDefs.hlsli"
#include "../shadersClean/traversal/Vpc.hlsli"
#include "../shadersClean/common/InstanceDesc.hlsli"

// Following order matters as AccelStructTracker relies on defines from TraceRayCommon.hlsl
#include "TraceRayCommon.hlsl"
#include "AccelStructTracker.hlsl"

#if GPURT_BUILD_CONTINUATION && LLPC_CLIENT_INTERFACE_MAJOR_VERSION
// Include the continuations library
#include "GpuRtLibraryCont.hlsl"
#endif

#include "RayQuery.hlsl"
#include "TraceRay.hlsl"

#include "GpuRtLibraryRayQuery.hlsl"

//=====================================================================================================================
// TraceRay() entry point for ray tracing IP 1.1
export void TraceRay1_1(
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
    float tMax)
{
    TraceRayCommon(
        accelStructLo,
        accelStructHi,
        rayFlags,
        instanceInclusionMask,
        rayContributionToHitGroupIndex,
        multiplierForGeometryContributionToShaderIndex,
        missShaderIndex,
        originX,
        originY,
        originZ,
        tMin,
        dirX,
        dirY,
        dirZ,
        tMax,
        0,
        0,
        true,
        GPURT_RTIP1_1
#if GPURT_BUILD_RTIP3
      , false
#endif
    );
}

//=====================================================================================================================
// TraceRay() hit-token extension entry point for ray tracing IP 1.1
export void TraceRayUsingHitToken1_1(
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
    uint  tlasPointer)
{
    TraceRayCommon(
        accelStructLo,
        accelStructHi,
        rayFlags,
        instanceInclusionMask,
        rayContributionToHitGroupIndex,
        multiplierForGeometryContributionToShaderIndex,
        missShaderIndex,
        originX,
        originY,
        originZ,
        tMin,
        dirX,
        dirY,
        dirZ,
        tMax,
        blasPointer,
        tlasPointer,
        false,
        GPURT_RTIP1_1
#if GPURT_BUILD_RTIP3
      , false
#endif
    );
}

//=====================================================================================================================
// TraceRay() entry point for ray tracing IP 2.0
export void TraceRay2_0(
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
    float tMax)
{
    TraceRayCommon(
        accelStructLo,
        accelStructHi,
        rayFlags,
        instanceInclusionMask,
        rayContributionToHitGroupIndex,
        multiplierForGeometryContributionToShaderIndex,
        missShaderIndex,
        originX,
        originY,
        originZ,
        tMin,
        dirX,
        dirY,
        dirZ,
        tMax,
        0,
        0,
        true,
        GPURT_RTIP2_0
#if GPURT_BUILD_RTIP3
      , false
#endif
    );
}

//=====================================================================================================================
// TraceRay() hit-token extension entry point for ray tracing IP 2.0
export void TraceRayUsingHitToken2_0(
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
    uint  tlasPointer)
{
    TraceRayCommon(
        accelStructLo,
        accelStructHi,
        rayFlags,
        instanceInclusionMask,
        rayContributionToHitGroupIndex,
        multiplierForGeometryContributionToShaderIndex,
        missShaderIndex,
        originX,
        originY,
        originZ,
        tMin,
        dirX,
        dirY,
        dirZ,
        tMax,
        blasPointer,
        tlasPointer,
        false,
        GPURT_RTIP2_0
#if GPURT_BUILD_RTIP3
      , false
#endif
    );
}

#if GPURT_BUILD_RTIP3
//=====================================================================================================================
// TraceRay() entry point for ray tracing IP 3.0
export void TraceRay3_0(
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
    float tMax)
{
    TraceRayCommon(
        accelStructLo,
        accelStructHi,
        rayFlags,
        instanceInclusionMask,
        rayContributionToHitGroupIndex,
        multiplierForGeometryContributionToShaderIndex,
        missShaderIndex,
        originX,
        originY,
        originZ,
        tMin,
        dirX,
        dirY,
        dirZ,
        tMax,
        0,
        0,
        true,
        GPURT_RTIP3_0
      , false
    );
}

//=====================================================================================================================
// TraceRay() entry point for ray tracing IP 3.0
export void TraceRay3_0BVH8(
    uint accelStructLo,
    uint accelStructHi,
    uint rayFlags,
    uint instanceInclusionMask,
    uint rayContributionToHitGroupIndex,
    uint multiplierForGeometryContributionToShaderIndex,
    uint missShaderIndex,
    float originX,
    float originY,
    float originZ,
    float tMin,
    float dirX,
    float dirY,
    float dirZ,
    float tMax)
{
    TraceRayCommon(
        accelStructLo,
        accelStructHi,
        rayFlags,
        instanceInclusionMask,
        rayContributionToHitGroupIndex,
        multiplierForGeometryContributionToShaderIndex,
        missShaderIndex,
        originX,
        originY,
        originZ,
        tMin,
        dirX,
        dirY,
        dirZ,
        tMax,
        0,
        0,
        true,
        GPURT_RTIP3_0
        , true
    );
}

//=====================================================================================================================
// TraceRay() hit-token extension entry point for ray tracing IP 3.0
export void TraceRayUsingHitToken3_0(
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
    uint  tlasPointer)
{
    TraceRayCommon(
        accelStructLo,
        accelStructHi,
        rayFlags,
        instanceInclusionMask,
        rayContributionToHitGroupIndex,
        multiplierForGeometryContributionToShaderIndex,
        missShaderIndex,
        originX,
        originY,
        originZ,
        tMin,
        dirX,
        dirY,
        dirZ,
        tMax,
        blasPointer,
        tlasPointer,
        false,
        GPURT_RTIP3_0
#if GPURT_BUILD_RTIP3
        , false
#endif
    );
}
#endif

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
// TraceRay() entry point for ray tracing IP 3.1
export void TraceRay3_1(
    uint accelStructLo,
    uint accelStructHi,
    uint rayFlags,
    uint instanceInclusionMask,
    uint rayContributionToHitGroupIndex,
    uint multiplierForGeometryContributionToShaderIndex,
    uint missShaderIndex,
    float originX,
    float originY,
    float originZ,
    float tMin,
    float dirX,
    float dirY,
    float dirZ,
    float tMax)
{
    TraceRayCommon(
        accelStructLo,
        accelStructHi,
        rayFlags,
        instanceInclusionMask,
        rayContributionToHitGroupIndex,
        multiplierForGeometryContributionToShaderIndex,
        missShaderIndex,
        originX,
        originY,
        originZ,
        tMin,
        dirX,
        dirY,
        dirZ,
        tMax,
        0,
        0,
        true,
        GPURT_RTIP3_1
        , true
    );
}

//=====================================================================================================================
// TraceRay() hit-token extension entry point for ray tracing IP 3.1
export void TraceRayUsingHitToken3_1(
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
    uint  tlasPointer)
{
    TraceRayCommon(
        accelStructLo,
        accelStructHi,
        rayFlags,
        instanceInclusionMask,
        rayContributionToHitGroupIndex,
        multiplierForGeometryContributionToShaderIndex,
        missShaderIndex,
        originX,
        originY,
        originZ,
        tMin,
        dirX,
        dirY,
        dirZ,
        tMax,
        blasPointer,
        tlasPointer,
        false,
        GPURT_RTIP3_1
        , false
    );
}
#endif

//=====================================================================================================================
// GPURT intrinsic for fetching triangle position from given BVH address and node pointer
export TriangleData FetchTrianglePositionFromNodePointer(
    in GpuVirtualAddress bvhAddress,  // BVH address
    in uint              nodePointer) // Node pointer
{
    return FetchTriangleFromNode(bvhAddress, nodePointer);
}

#endif
