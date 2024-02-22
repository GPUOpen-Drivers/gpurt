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
#define RootSig "RootConstants(num32BitConstants=9, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 2)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 3)),"\
                "DescriptorTable(CBV(b0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b255),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL)"

#define TASK_COUNTER_BUFFER   ScratchBuffer
#define TASK_COUNTER_OFFSET   UPDATE_SCRATCH_ENCODE_TASK_COUNT_OFFSET
#define NUM_TASKS_DONE_OFFSET UPDATE_SCRATCH_ENCODE_TASKS_DONE_OFFSET

//======================================================================================================================
// 32 bit constants
struct InputArgs
{
    uint addressLo;
    uint addressHi;
    uint propagationFlagsScratchOffset;
    uint baseUpdateStackScratchOffset;
    uint triangleCompressionMode;
    uint fp16BoxNodesInBlasMode;
    uint numThreads;
    uint numPrimitives;
    uint numDescs;
};

[[vk::push_constant]] ConstantBuffer<InputArgs> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer DstMetadata    : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer ScratchBuffer  : register(u1);
[[vk::binding(2, 0)]]                  RWByteAddressBuffer SrcBuffer      : register(u2);

// unused buffer
[[vk::binding(3, 0)]] globallycoherent RWByteAddressBuffer DstBuffer      : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer                  EmitBuffer     : register(u4);
[[vk::binding(5, 0)]] globallycoherent RWByteAddressBuffer ScratchGlobal  : register(u5);

#include "EncodeCommon.hlsl"

[[vk::binding(0, 2)]] ConstantBuffer<GeometryArgs> GeometryConstants[] : register(b0, space1);

[[vk::binding(0, 3)]] RWBuffer<float3>           GeometryBuffer[]  : register(u0, space1);
[[vk::binding(0, 4)]] RWByteAddressBuffer        IndexBuffer[]     : register(u0, space2);
#if !AMD_VULKAN
// DXC does not support arrays of structured buffers for SPIRV currently. See issue 3281 / PR 4663).
[[vk::binding(0, 5)]] RWStructuredBuffer<float4> TransformBuffer[] : register(u0, space3);
#endif

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
    for (uint geometryIndex = 0; geometryIndex < ShaderConstants.numDescs; geometryIndex++)
    {
        const uint numPrimitives = GeometryConstants[geometryIndex].NumPrimitives;
        const uint primitiveOffset = GeometryConstants[geometryIndex].PrimitiveOffset;

        GeometryArgs geometryArgs = GeometryConstants[geometryIndex];
        geometryArgs.BuildFlags = DDI_BUILD_FLAG_PERFORM_UPDATE;

        if (globalId == 0)
        {
            const uint primitiveStride = (geometryType == GEOMETRY_TYPE_TRIANGLES) ?
                                         DECODE_PRIMITIVE_STRIDE_TRIANGLE :
                                         DECODE_PRIMITIVE_STRIDE_AABB;
            WriteGeometryInfo(geometryArgs, primitiveOffset, geometryArgs.NumPrimitives, primitiveStride);
        }

        for (uint primitiveIndex = globalId; primitiveIndex < numPrimitives; primitiveIndex += ShaderConstants.numThreads)
        {
            if (geometryType == GEOMETRY_TYPE_TRIANGLES)
            {
                EncodeTriangleNode(GeometryBuffer[geometryIndex],
                                   IndexBuffer[geometryIndex],
                                   TransformBuffer[geometryIndex],
                                   geometryArgs,
                                   primitiveIndex,
                                   primitiveOffset,
                                   0,
                                   true);
            }
            else
            {
                EncodeAabbNode(GeometryBuffer[geometryIndex],
                               IndexBuffer[geometryIndex],
                               TransformBuffer[geometryIndex],
                               geometryArgs,
                               primitiveIndex,
                               primitiveOffset,
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

    const uint numGroups = ShaderConstants.numThreads / BUILD_THREADGROUP_SIZE;

#if !AMD_VULKAN
    BEGIN_TASK(numGroups);

    {
        EncodePrimitives(globalId, GEOMETRY_TYPE_TRIANGLES);
    }

    END_TASK(numGroups);
#endif

    const uint numWorkItems = ScratchBuffer.Load(UPDATE_SCRATCH_STACK_NUM_ENTRIES_OFFSET);

    UpdateQBVHImpl(globalId,
                   ShaderConstants.propagationFlagsScratchOffset,
                   numWorkItems,
                   ShaderConstants.numThreads);
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

    const uint numGroups = ShaderConstants.numThreads / BUILD_THREADGROUP_SIZE;

#if !AMD_VULKAN
    BEGIN_TASK(numGroups);

    {
        EncodePrimitives(globalId, GEOMETRY_TYPE_AABBS);
    }

    END_TASK(numGroups);
#endif

    const uint numWorkItems = ScratchBuffer.Load(UPDATE_SCRATCH_STACK_NUM_ENTRIES_OFFSET);

    UpdateQBVHImpl(globalId,
                   ShaderConstants.propagationFlagsScratchOffset,
                   numWorkItems,
                   ShaderConstants.numThreads);
}
