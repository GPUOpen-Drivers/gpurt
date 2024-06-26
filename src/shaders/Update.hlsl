/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
                "DescriptorTable(CBV(b0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b255),"\
                "UAV(u3),"\
                "UAV(u4),"\
                "UAV(u5)"

#define TASK_COUNTER_BUFFER   ScratchBuffer
#define TASK_COUNTER_OFFSET   UPDATE_SCRATCH_ENCODE_TASK_COUNT_OFFSET
#define NUM_TASKS_DONE_OFFSET UPDATE_SCRATCH_ENCODE_TASKS_DONE_OFFSET
#include "TaskMacros.hlsl"

//======================================================================================================================
// 32 bit constants
struct RootConstants
{
    uint numThreads;
};

#include "../shared/rayTracingDefs.h"

[[vk::push_constant]] ConstantBuffer<RootConstants>                ShaderRootConstants  : register(b0);
[[vk::binding(1, 1)]] ConstantBuffer<BuildShaderConstants>         ShaderConstants      : register(b1);
[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer         DstMetadata          : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer         ScratchBuffer        : register(u1);
[[vk::binding(2, 0)]]                  RWByteAddressBuffer         SrcBuffer            : register(u2);
[[vk::binding(0, 3)]] ConstantBuffer<BuildShaderGeometryConstants> GeometryConstants[]  : register(b0, space1);
[[vk::binding(0, 4)]] RWBuffer<float3>                             GeometryBuffer[]     : register(u0, space1);
// unused buffer
[[vk::binding(3, 0)]] globallycoherent RWByteAddressBuffer         DstBuffer            : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer                          EmitBuffer           : register(u4);
[[vk::binding(5, 0)]] globallycoherent RWByteAddressBuffer         ScratchGlobal        : register(u5);

#include "EncodeCommon.hlsl"

groupshared uint SharedMem[1];

//======================================================================================================================
// Note, these headers must be included after all resource bindings have been defined. Also, there is a strict naming
// requirement for resources and variables. See BuildCommon.hlsl for details.
#include "IntersectCommon.hlsl"
#include "UpdateQBVHImpl.hlsl"

#if !AMD_VULKAN
//======================================================================================================================
void EncodePrimitives(
    uint globalId,
    uint geometryType)
{
    for (uint geomId = 0; geomId < ShaderConstants.numDescs; geomId++)
    {
        const GeometryArgs geometryArgs = InitGeometryArgs(geomId);

        if (globalId == 0)
        {
            const uint primitiveStride =
                (geometryType == GEOMETRY_TYPE_TRIANGLES) ? DECODE_PRIMITIVE_STRIDE_TRIANGLE :
                                                            DECODE_PRIMITIVE_STRIDE_AABB;

            WriteGeometryInfo(geometryArgs, geometryArgs.PrimitiveOffset, geometryArgs.NumPrimitives, primitiveStride);
        }

        for (uint primId = globalId; primId < geometryArgs.NumPrimitives; primId += ShaderRootConstants.numThreads)
        {
            if (geometryType == GEOMETRY_TYPE_TRIANGLES)
            {
                EncodeTriangleNode(GeometryBuffer[geomId],
                                   geometryArgs,
                                   primId,
                                   geometryArgs.PrimitiveOffset,
                                   0,
                                   0,
                                   0,
                                   true);
            }
            else
            {
                EncodeAabbNode(GeometryBuffer[geomId],
                               geometryArgs,
                               primId,
                               geometryArgs.PrimitiveOffset,
                               0,
                               true);
            }
        }
    }
}

#endif

//======================================================================================================================
// Main Function : UpdateTriangles
//======================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void UpdateTriangles(
    uint globalId : SV_DispatchThreadID,
    uint localId  : SV_GroupThreadID)
{
    uint waveId = 0;
    uint numTasksWait = 0;
    INIT_TASK;

    const uint numGroups = ShaderRootConstants.numThreads / BUILD_THREADGROUP_SIZE;

#if !AMD_VULKAN
    BEGIN_TASK(numGroups);

    {
        EncodePrimitives(globalId, GEOMETRY_TYPE_TRIANGLES);
    }

    END_TASK(numGroups);
#endif

    const uint numWorkItems = ScratchBuffer.Load(UPDATE_SCRATCH_STACK_NUM_ENTRIES_OFFSET);

    UpdateQBVHImpl(globalId,
                   numWorkItems,
                   ShaderRootConstants.numThreads);
}

//======================================================================================================================
// Main Function : UpdateAabbs
//======================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void UpdateAabbs(
    uint globalId : SV_DispatchThreadID,
    uint localId  : SV_GroupThreadID)
{
    uint waveId = 0;
    uint numTasksWait = 0;
    INIT_TASK;

    const uint numGroups = ShaderRootConstants.numThreads / BUILD_THREADGROUP_SIZE;

#if !AMD_VULKAN
    BEGIN_TASK(numGroups);

    {
        EncodePrimitives(globalId, GEOMETRY_TYPE_AABBS);
    }

    END_TASK(numGroups);
#endif

    const uint numWorkItems = ScratchBuffer.Load(UPDATE_SCRATCH_STACK_NUM_ENTRIES_OFFSET);

    UpdateQBVHImpl(globalId,
                   numWorkItems,
                   ShaderRootConstants.numThreads);
}
