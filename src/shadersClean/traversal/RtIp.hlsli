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
#ifndef RTIP_HLSLI
#define RTIP_HLSLI

#include "llpc/GpurtIntrinsics.h"
#include "../debug/Debug.hlsli"

#ifdef __cplusplus
extern uint g_rtIpLevel;          // defined in cputraversal
void _AmdSetRtip(uint rtIpLevel); // defined in cputraversal
#endif

// Only the default path (Continuation) provides _AmdGetRtip().
static RayTracingIpLevel GetRtIpLevel()
{
#ifdef __cplusplus
    switch (g_rtIpLevel)
    {
    case GPURT_RTIP1_1:
        return RayTracingIpLevel::RtIp1_1;
    case GPURT_RTIP2_0:
        return RayTracingIpLevel::RtIp2_0;
#if GPURT_BUILD_RTIP3_1
    case GPURT_RTIP3_1:
        return RayTracingIpLevel::RtIp3_1;
#endif
    default:
        // Should never be called
        GPU_ASSERT(false);
        return RayTracingIpLevel::_None;
    }
#else // __cplusplus
#if GPURT_DEBUG_CONTINUATION_TRAVERSAL
    if (GPURT_RTIP_LEVEL == (uint)RayTracingIpLevel::_None)
    {
        return RayTracingIpLevel::_None;
    }
#if GPURT_BUILD_RTIP3_1
    if (GPURT_RTIP_LEVEL == (uint)RayTracingIpLevel::RtIp3_1)
    {
        return RayTracingIpLevel::RtIp3_1;
    }
#endif
    return RayTracingIpLevel::RtIp2_0; //default to ip 2.0
#else // GPURT_DEBUG_CONTINUATION_TRAVERSAL
    return _AmdGetRtip(); // Continuation path
#endif
#endif
}

static bool RtIpIsAtLeast(RayTracingIpLevel level)
{
    return ((uint32_t)GetRtIpLevel()) >= ((uint32_t)level);
}

#endif
