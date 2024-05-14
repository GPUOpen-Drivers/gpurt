/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

#include "RayQuery1_1.hlsl"
#include "RayQuery2_0.hlsl"

//=====================================================================================================================
// TraceRayInline() internal
static void TraceRayInlineCommon(
    inout_param(RayQueryInternal) rayQuery,
    in    uint                    accelStructLo,
    in    uint                    accelStructHi,
    in    uint                    constRayFlags,
    in    uint                    rayFlags,
    in    uint                    instanceMask,
    in    RayDesc                 rayDesc,
    in    uint3                   dispatchThreadId,
    in    uint                    rtIpLevel)
{
    switch (rtIpLevel)
    {
        case GPURT_RTIP1_1:
            TraceRayInlineImpl1_1(rayQuery,
                                  accelStructLo,
                                  accelStructHi,
                                  constRayFlags,
                                  rayFlags,
                                  instanceMask,
                                  rayDesc,
                                  dispatchThreadId);
            break;
        case GPURT_RTIP2_0:
            TraceRayInlineImpl2_0(rayQuery,
                                  accelStructLo,
                                  accelStructHi,
                                  constRayFlags,
                                  rayFlags,
                                  instanceMask,
                                  rayDesc,
                                  dispatchThreadId);
            break;
        default:
            break;
    }
}

//=====================================================================================================================
// RayQuery::Proceed() internal
static bool RayQueryProceedCommon(
    inout_param(RayQueryInternal) rayQuery,
    in    uint                    constRayFlags,
    in    uint3                   dispatchThreadId,
    in    uint                    rtIpLevel
)
{
    bool continueTraversal = false;

    switch (rtIpLevel)
    {
        case GPURT_RTIP1_1:
            continueTraversal = RayQueryProceedImpl1_1(rayQuery,
                                                       constRayFlags,
                                                       dispatchThreadId);
            break;
        case GPURT_RTIP2_0:
            continueTraversal = RayQueryProceedImpl2_0(rayQuery,
                                                       constRayFlags,
                                                       dispatchThreadId);
            break;
        default:
            break;
    }

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        if (continueTraversal == false)
        {
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION >= 42
            const uint rayId = AmdExtDispatchThreadIdFlat();
#else
            const uint rayId = GetRayId(dispatchThreadId);
#endif
            WriteDispatchCounters(rayQuery.numIterations);
            WriteTraversalCounter(rayQuery, rayId);

            if (rayQuery.committed.currNodePtr != INVALID_NODE)
            {
                uint instNodeIndex = FetchInstanceIdx(rtIpLevel, GetRayQueryTopBvhAddress(rayQuery), rayQuery.lastInstanceNode);

                WriteRayHistoryTokenEnd(
                    rayId,
                    uint2(rayQuery.committed.primitiveIndex, rayQuery.committed.geometryIndex),
                    instNodeIndex,
                    rayQuery.numIterations,
                    rayQuery.instanceIntersections,
                    rayQuery.committed.frontFace ? HIT_KIND_TRIANGLE_FRONT_FACE : HIT_KIND_TRIANGLE_BACK_FACE,
                    rayQuery.committed.rayTCurrent);
            }
            else
            {
                WriteRayHistoryTokenEnd(rayId,
                                        uint2(~0, ~0),
                                        uint(~0),
                                        rayQuery.numIterations,
                                        rayQuery.instanceIntersections,
                                        uint(~0),
                                        (rayQuery.rayDesc.TMax - rayQuery.rayDesc.TMin));
            }
        }
    }
#endif

    return continueTraversal;
}
