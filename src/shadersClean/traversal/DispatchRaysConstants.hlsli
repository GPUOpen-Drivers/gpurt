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
#ifndef DISPATCHRAYSCONSTANTS_HLSLI
#define DISPATCHRAYSCONSTANTS_HLSLI

#include "../common/Extensions.hlsli"
#include "../common/Math.hlsli"

#include "../../../gpurt/gpurtDispatch.h"

// Driver reserved space ID and resource bindings

#define DISPATCH_RAYS_CONSTANTS_SPACE_ID space93

#define DispatchRaysConstantsId        b17

// The total w * h * d of the dispatch must be <= 2^30.
// For DX, this is explicitly stated in the documentation of ID3D12GraphicsCommandList4::DispatchRays.
// For Vulkan, it is set in VkPhysicalDeviceRayTracingPipelinePropertiesKHR, which itself is set
// to RayTraceRayGenShaderThreads = 0x40000000, defined in gpurt.h.
// We bit-pack the dispatch id into a 32 bit integer. It can be shown that all 32-bit integers with one or
// less not-set bits do not yield a valid dispatch id (i.e. they imply a dispatch size > 2^30).
// So ~0 and ~0 - (1 << n) are all free to use.
//
// Note: when adding/changing sentinel values, make sure to update _AmdDispatchSystemData::IsDead.
#define DISPATCHID_DEAD_STACKFUL  0xffffffff
#define DISPATCHID_DEAD_STACKLESS 0xfffffffe

//=====================================================================================================================
ConstantBuffer<DispatchRaysConstantData> DispatchRaysConstBuf : register(DispatchRaysConstantsId, DISPATCH_RAYS_CONSTANTS_SPACE_ID);

//=====================================================================================================================
// Load dispatch dimensions from constant buffer.
static uint3 GetDispatchRaysDimensions()
{
    const uint width  = DispatchRaysConstBuf.rayDispatchWidth;
    const uint height = DispatchRaysConstBuf.rayDispatchHeight;
    const uint depth  = DispatchRaysConstBuf.rayDispatchDepth;

    return uint3(width, height, depth);
}

//=====================================================================================================================
// Persistent dispatch size (1D).
static uint GetPersistentDispatchSize()
{
    // Groups needed to cover the dispatch if each thread only processes 1 ray
    const uint3 rayDispatch   = GetDispatchRaysDimensions();
    const uint  threadsNeeded = rayDispatch.x * rayDispatch.y * rayDispatch.z;
    const uint3 groupDim      = AmdExtGroupDimCompute();
    const uint  groupsNeeded  = RoundUpQuotient(threadsNeeded, groupDim.x * groupDim.y * groupDim.z);

    // Dispatch size is the lesser of rayDispatchMaxGroups and groupsNeeded
    // rayDispatchMaxGroups would mean threads handle >= 1 ray, groupsNeeded would mean threads handle <= 1 ray
    return min(DispatchRaysConstBuf.rayDispatchMaxGroups, groupsNeeded);
}

#endif
