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
#define RootSig "RootConstants(num32BitConstants=1, b0),"\
                "CBV(b1),"\
                "UAV(u0),"\
                "UAV(u1),"\
                "UAV(u2),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b255),"\
                "UAV(u3),"\
                "UAV(u4)"

//======================================================================================================================
// 32 bit constants
struct RootConstants
{
    uint numThreads;
};

#include "../shared/rayTracingDefs.h"

[[vk::push_constant]] ConstantBuffer<RootConstants>        ShaderRootConstants : register(b0);
[[vk::binding(1, 1)]] ConstantBuffer<BuildShaderConstants> ShaderConstants     : register(b1);

[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer DstMetadata   : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer ScratchBuffer : register(u1);
[[vk::binding(2, 0)]]                  RWByteAddressBuffer SrcBuffer     : register(u2);

// unused buffer
[[vk::binding(3, 0)]] globallycoherent RWByteAddressBuffer DstBuffer     : register(u3);
[[vk::binding(4, 0)]]                  RWByteAddressBuffer EmitBuffer    : register(u4);

//=====================================================================================================================
// Note, these headers must be included after all resource bindings have been defined. Also, there is a strict naming
// requirement for resources and variables. See BuildCommon.hlsl for details.
#include "IntersectCommon.hlsl"
#include "BuildCommon.hlsl"
#include "UpdateQBVHImpl.hlsl"

//=====================================================================================================================
void WaitForEncodeTasksToFinish(uint numTasksWait)
{
    // Wait until numTasksWait tasks are done
    do
    {
        DeviceMemoryBarrier();
    } while (ScratchBuffer.Load(UPDATE_SCRATCH_ENCODE_TASK_COUNT_OFFSET) < numTasksWait);
}

//=====================================================================================================================
// Main Function : UpdateParallel
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void UpdateParallel(
    in uint globalThreadId : SV_DispatchThreadID)
{
    // Waiting for EncodeNodes/EncodeTopLevel to finish encoding the leaves/primitives
    WaitForEncodeTasksToFinish(ShaderConstants.numPrimitives);

    // Fetch number of nodes to process
    uint numWorkItems = ScratchBuffer.Load(UPDATE_SCRATCH_STACK_NUM_ENTRIES_OFFSET);

    UpdateQBVHImpl(globalThreadId,
                   numWorkItems,
                   ShaderRootConstants.numThreads);
}
