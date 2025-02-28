/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
// Note, CBV(b255) must be the last used binding in the root signature.
#define RootSig "RootConstants(num32BitConstants=1, b0),"\
                "CBV(b1),"\
                "CBV(b2),"\
                "UAV(u0),"\
                "UAV(u1),"\
                "UAV(u2),"\
                "CBV(b255),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\

#define TASK_COUNTER_BUFFER   ScratchBuffer
#define TASK_COUNTER_OFFSET   UPDATE_SCRATCH_TASK_COUNT_OFFSET
#define NUM_TASKS_DONE_OFFSET UPDATE_SCRATCH_TASKS_DONE_OFFSET
groupshared uint SharedMem[1];
uint GetSharedMem(uint index)
{
    return SharedMem[index];
}
void SetSharedMem(uint index, uint value)
{
    SharedMem[index] = value;
}
#include "TaskMacros.hlsl"

//======================================================================================================================
// 32 bit constants
struct RootConstants
{
    uint numThreads;
};

#include "../shadersClean/common/ShaderDefs.hlsli"

[[vk::push_constant]] ConstantBuffer<RootConstants>        ShaderRootConstants : register(b0);
[[vk::binding(1, 1)]] ConstantBuffer<BuildShaderConstants> ShaderConstants     : register(b1);
[[vk::binding(2, 1)]] ConstantBuffer<LutData>              LutBuffer           : register(b2);

[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer DstMetadata     : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer ScratchBuffer   : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer                  SrcBuffer       : register(u2);

// unused buffer
[[vk::binding(3, 0)]] globallycoherent RWByteAddressBuffer DstBuffer       : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer                  EmitBuffer      : register(u4);

#include "IntersectCommon.hlsl"
#include "UpdateCommon.hlsl"
#include "UpdateQBVHImpl.hlsl"

//=====================================================================================================================
// Main Function : UpdateQBVH
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void UpdateQBVH(
    in uint globalThreadId : SV_DispatchThreadID,
    in uint localId : SV_GroupThreadID)
{
    uint waveId = 0;
    uint numTasksWait = 0;
    INIT_TASK;

    const uint numGroups = ShaderRootConstants.numThreads / BUILD_THREADGROUP_SIZE;

    BEGIN_TASK(numGroups);

    ClearUpdateFlags(globalId);

    END_TASK(numGroups);

    // Fetch number of nodes to process
    uint numWorkItems = ScratchBuffer.Load(UPDATE_SCRATCH_STACK_NUM_ENTRIES_OFFSET);

    if (globalThreadId.x >= numWorkItems)
    {
        return;
    }

    UpdateQBVHImpl(globalThreadId.x,
                   numWorkItems,
                   numWorkItems);
}
