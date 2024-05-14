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
                "DescriptorTable(CBV(b0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 1)),"\
                "UAV(u0),"\
                "UAV(u1),"\
                "UAV(u2),"\
                "UAV(u3),"\
                "UAV(u4),"\
                "UAV(u5),"\
                "UAV(u6),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b255)"

#include "..\shared\rayTracingDefs.h"

//======================================================================================================================
struct RootConstants
{
    uint geometryIndex;
};

[[vk::push_constant]] ConstantBuffer<RootConstants>                 ShaderRootConstants : register(b0);
[[vk::binding(1, 1)]] ConstantBuffer<BuildShaderConstants>          ShaderConstants     : register(b1);
[[vk::binding(0, 2)]] ConstantBuffer<BuildShaderGeometryConstants>  GeometryConstants[] : register(b0, space1);
[[vk::binding(0, 3)]] RWBuffer<float3>                              GeometryBuffer[]    : register(u0, space1);
[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer          DstMetadata         : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer                           ScratchBuffer       : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer                           ScratchGlobal       : register(u2);
[[vk::binding(3, 0)]] RWByteAddressBuffer                           SrcBuffer           : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer                           IndirectArgBuffer   : register(u4);
// unused buffer
[[vk::binding(5, 0)]] globallycoherent RWByteAddressBuffer          DstBuffer           : register(u5);
[[vk::binding(6, 0)]] RWByteAddressBuffer                           EmitBuffer          : register(u6);

#include "Common.hlsl"
#include "BuildCommon.hlsl"
#include "BuildCommonScratch.hlsl"
#include "EncodeCommon.hlsl"
#include "EncodePairedTriangle.hlsl"

//======================================================================================================================
struct InputOffsetsAndNumPrim
{
    uint numPrimitives;
    uint primitiveOffset;
    uint vertexOffsetInComponents;
    uint indexOffsetInBytes;
    uint transformOffsetInBytes;
};

//======================================================================================================================
// Helper function that fetches data from IndirectArgBuffer when needed, uses sane defaults for direct path otherwise.
InputOffsetsAndNumPrim GetInputOffsetsAndNumPrim(GeometryArgs args)
{
    InputOffsetsAndNumPrim result = (InputOffsetsAndNumPrim)0;

    if (Settings.isIndirectBuild)
    {
        // Sourced from Indirect Buffers
        const IndirectBuildOffset buildOffsetInfo =
            IndirectArgBuffer.Load<IndirectBuildOffset>(ShaderConstants.indirectArgBufferStride * args.GeometryIndex);
        const uint firstVertexInComponents = buildOffsetInfo.firstVertex * args.VertexComponentCount;

        result.numPrimitives             = buildOffsetInfo.primitiveCount;
        result.primitiveOffset           = ComputePrimitiveOffset(args);
        if (Settings.geometryType == GEOMETRY_TYPE_TRIANGLES)
        {
            if (args.IndexBufferFormat != IndexFormatInvalid)
            {
                result.vertexOffsetInComponents = firstVertexInComponents;
                result.indexOffsetInBytes       = buildOffsetInfo.primitiveOffset;
            }
            else
            {
                const uint primitiveOffsetInComponents = buildOffsetInfo.primitiveOffset / args.VertexComponentSize;

                result.vertexOffsetInComponents = primitiveOffsetInComponents + firstVertexInComponents;
                result.indexOffsetInBytes       = 0;
            }
            result.transformOffsetInBytes = buildOffsetInfo.transformOffset;
        }
        else
        {
            GPU_ASSERT(Settings.geometryType == GEOMETRY_TYPE_AABBS);
            result.vertexOffsetInComponents = buildOffsetInfo.primitiveOffset / sizeof(float);

            result.indexOffsetInBytes       = 0;
            result.transformOffsetInBytes   = 0;
        }
    }
    else
    {
        result.numPrimitives            = args.NumPrimitives;
        result.primitiveOffset          = args.PrimitiveOffset;
        result.vertexOffsetInComponents = 0;
        result.indexOffsetInBytes       = 0;
        result.transformOffsetInBytes   = 0;
    }

    return result;
}

//======================================================================================================================
// Helper function that increments ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET
// and ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET for INDIRECT_BUILD.
// Direct builds have access to primitive count during dispatch cmd recording and set
// ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET to correct value beforehand.
void IncrementPrimitiveTaskCounters(
    in uint encodeTaskCounterScratchOffset,
    in uint primitiveIndex,
    in uint numPrimitives,
    in uint maxNumPrimitives)
{
    // 1st task counter is being used as primitive counter, it means how many triangles is there to build
    // in case of indirect build we dispatch maxPrimCount waves, but less that that can be encoded
    const uint encodedPrimCount = WaveActiveCountBits(primitiveIndex < numPrimitives);
    // 2nd task counter is being used as spin-lock that build step waits for,
    // counter=GeometryConstants.numPrimitives means encoding is done
    const uint dispatchedPrimCount = WaveActiveCountBits(primitiveIndex < maxNumPrimitives);
    if (WaveIsFirstLane())
    {
        // compression and splitting update primRefCounter on their own,
        // direct builds set ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET to correct value during cmd recording
        if (Settings.isIndirectBuild && !Settings.enableEarlyPairCompression && !Settings.doTriangleSplitting)
        {
            IncrementTaskCounter(encodeTaskCounterScratchOffset + ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET,
                                 encodedPrimCount);
        }

        IncrementTaskCounter(encodeTaskCounterScratchOffset + ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET,
                             dispatchedPrimCount);
    }
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeTriangleNodes(
    in uint3 globalThreadId : SV_DispatchThreadID,
    in uint localId : SV_GroupThreadID)
{
    const GeometryArgs args = InitGeometryArgs(ShaderRootConstants.geometryIndex);
    const InputOffsetsAndNumPrim inputOffsets = GetInputOffsetsAndNumPrim(args);

    const bool enableEarlyPairCompression =
        (Settings.enableEarlyPairCompression == true) && (IsUpdate() == false);

    if (globalThreadId.x == 0)
    {
        WriteGeometryInfo(
            args, inputOffsets.primitiveOffset, inputOffsets.numPrimitives, DECODE_PRIMITIVE_STRIDE_TRIANGLE);
    }

    uint primitiveIndex = globalThreadId.x;

    if (primitiveIndex < inputOffsets.numPrimitives)
    {
        if (enableEarlyPairCompression)
        {
            EncodePairedTriangleNode(GeometryBuffer[ShaderRootConstants.geometryIndex],
                                     args,
                                     primitiveIndex,
                                     globalThreadId.x,
                                     inputOffsets.primitiveOffset,
                                     inputOffsets.vertexOffsetInComponents,
                                     inputOffsets.indexOffsetInBytes,
                                     inputOffsets.transformOffsetInBytes);
        }
        else
        {
            EncodeTriangleNode(GeometryBuffer[ShaderRootConstants.geometryIndex],
                               args,
                               primitiveIndex,
                               inputOffsets.primitiveOffset,
                               inputOffsets.vertexOffsetInComponents,
                               inputOffsets.indexOffsetInBytes,
                               inputOffsets.transformOffsetInBytes,
                               true);
        }
    }

    IncrementPrimitiveTaskCounters(args.encodeTaskCounterScratchOffset,
                                   primitiveIndex,
                                   inputOffsets.numPrimitives,
                                   args.NumPrimitives);
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeAABBNodes(
    in uint3 globalThreadId : SV_DispatchThreadID)
{
    const GeometryArgs args = InitGeometryArgs(ShaderRootConstants.geometryIndex);
    const InputOffsetsAndNumPrim inputOffsets = GetInputOffsetsAndNumPrim(args);

    if (globalThreadId.x == 0)
    {
        WriteGeometryInfo(args, inputOffsets.primitiveOffset, inputOffsets.numPrimitives, DECODE_PRIMITIVE_STRIDE_AABB);
    }

    uint primitiveIndex = globalThreadId.x;
    if (primitiveIndex < inputOffsets.numPrimitives)
    {
        EncodeAabbNode(GeometryBuffer[ShaderRootConstants.geometryIndex],
                       args,
                       primitiveIndex,
                       inputOffsets.primitiveOffset,
                       inputOffsets.vertexOffsetInComponents,
                       true);
    }

    IncrementPrimitiveTaskCounters(args.encodeTaskCounterScratchOffset,
                                   primitiveIndex,
                                   inputOffsets.numPrimitives,
                                   args.NumPrimitives);
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void CountTrianglePairs(
    in uint globalId : SV_DispatchThreadID,
    in uint localId : SV_GroupThreadID)
{
    const GeometryArgs args = InitGeometryArgs(ShaderRootConstants.geometryIndex);
    const InputOffsetsAndNumPrim inputOffsets = GetInputOffsetsAndNumPrim(args);

    // Reset geometry primitive reference count in result buffer
    const uint basePrimRefCountOffset =
        args.metadataSizeInBytes + args.DestLeafByteOffset;

    const uint primRefCountOffset =
        basePrimRefCountOffset + ComputePrimRefCountBlockOffset(args.blockOffset, globalId);

    if (globalId < inputOffsets.numPrimitives)
    {
        const int pairInfo = TryPairTriangleImpl(GeometryBuffer[ShaderRootConstants.geometryIndex],
                                                 args,
                                                 globalId,
                                                 inputOffsets.vertexOffsetInComponents,
                                                 inputOffsets.indexOffsetInBytes,
                                                 inputOffsets.transformOffsetInBytes);

        // Count quads produced by the current wave. Note, this includes unpaired triangles as well
        // (marked with a value of -1).
        const bool isActivePrimRef = (pairInfo >= -1);
        const uint wavePrimRefCount = WaveActiveCountBits(isActivePrimRef);

        if (WaveIsFirstLane())
        {
            DstMetadata.Store(primRefCountOffset, wavePrimRefCount);
        }
    }
}
