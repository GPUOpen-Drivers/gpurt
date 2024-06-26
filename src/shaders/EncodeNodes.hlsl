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
                "UAV(u7),"\
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
[[vk::binding(0, 3)]] ConstantBuffer<BuildShaderGeometryConstants>  GeometryConstants[] : register(b0, space1);
[[vk::binding(0, 4)]] RWBuffer<float3>                              GeometryBuffer[]    : register(u0, space1);

[[vk::binding(0, 0)]] RWByteAddressBuffer                           SrcBuffer           : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer          DstBuffer           : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer          DstMetadata         : register(u2);
[[vk::binding(3, 0)]] RWByteAddressBuffer                           ScratchBuffer       : register(u3);
[[vk::binding(4, 0)]] globallycoherent RWByteAddressBuffer          ScratchGlobal       : register(u4);
[[vk::binding(5, 0)]] RWByteAddressBuffer                           InstanceDescBuffer  : register(u5);
[[vk::binding(6, 0)]] RWByteAddressBuffer                           EmitBuffer          : register(u6);
[[vk::binding(7, 0)]] RWByteAddressBuffer                           IndirectArgBuffer   : register(u7);

template<typename T>
T LoadInstanceDescBuffer(uint offset)
{
    return InstanceDescBuffer.Load<T>(offset);
}
#include "IndirectArgBufferUtils.hlsl"
#include "Common.hlsl"
#include "BuildCommon.hlsl"
#include "BuildCommonScratch.hlsl"
#include "EncodeCommon.hlsl"
#include "TaskMacros.hlsl"
#include "EncodePairedTriangleImpl.hlsl"

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeTriangleNodes(
    in uint3 globalThreadId : SV_DispatchThreadID,
    in uint localId : SV_GroupThreadID)
{
    const GeometryArgs args = InitGeometryArgs(ShaderRootConstants.geometryIndex);
    const NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrim(args);

    if (globalThreadId.x == 0)
    {
        WriteGeometryInfo(
            args, inputOffsets.primitiveOffset, inputOffsets.numPrimitives, DECODE_PRIMITIVE_STRIDE_TRIANGLE);
    }

    uint primitiveIndex = globalThreadId.x;

    if (primitiveIndex < inputOffsets.numPrimitives)
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
    const NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrim(args);

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

static const uint LOCAL_COUNTS_READY = ((uint)1 << 30);
static const uint LOCAL_PREFIX_SUM_READY = ((uint)1 << 31);
static const uint COUNTS_MASK = (~(LOCAL_COUNTS_READY | LOCAL_PREFIX_SUM_READY));

// Group shared memory for task counters used for InitBlockPrefixSum
groupshared uint SharedMem[1];

//=====================================================================================================================
void InitBlockPrefixSum(
    uint localId,
    uint numBlocks)
{
    uint numTasksWait = 0;
    uint waveId = 0;
    const uint numGroups = RoundUpQuotient(numBlocks, BUILD_THREADGROUP_SIZE);

    INIT_TASK;

    BEGIN_TASK(numGroups);

    if (globalId < numBlocks)
    {
        // Initialise block prefix sum to 0
        DstBuffer.Store(ShaderConstants.header.offsets.leafNodes + (globalId * sizeof(uint)), 0);
    }

    END_TASK(numGroups);
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeQuadNodes(
    in uint globalId : SV_DispatchThreadID,
    in uint localId : SV_GroupThreadID)
{
    InitBlockPrefixSum(localId, ShaderRootConstants.geometryIndex);

    // Figure out which geometry desc contains this blocks primitives.
    uint geomId = 0;

    const uint blockId = (globalId / WaveGetLaneCount());

    for (; geomId < ShaderConstants.numDescs; ++geomId)
    {
        const uint numBlocks =
            Pow2Align(GeometryConstants[geomId].numPrimitives, BUILD_THREADGROUP_SIZE) / BUILD_THREADGROUP_SIZE;

        const uint blockOffset = GeometryConstants[geomId].blockOffset;
        if (blockId < (blockOffset + numBlocks))
        {
            break;
        }
    }

    const GeometryArgs geometryArgs = InitGeometryArgs(geomId);
    const NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrim(geometryArgs);

    const uint startId = geometryArgs.blockOffset * WaveGetLaneCount();
    const uint endId = startId + geometryArgs.NumPrimitives;

    // Initialise lane as inactive triangle
    int pairInfo = -2;

    TriangleData tri = (TriangleData)0;

    const uint primId = globalId - startId;
    const uint flattenedPrimitiveIndex = geometryArgs.PrimitiveOffset + primId;

    if (primId == 0)
    {
        WriteGeometryInfo(
            geometryArgs, inputOffsets.primitiveOffset, inputOffsets.numPrimitives, DECODE_PRIMITIVE_STRIDE_TRIANGLE);
    }

    if (globalId < endId)
    {
        {
            const uint primNodePointerOffset =
                geometryArgs.BasePrimNodePtrOffset + (flattenedPrimitiveIndex * sizeof(uint));

            // Store invalid prim node pointer for now during first time builds.
            // If the triangle is active, EncodeHwBvh will write it in.
            DstBuffer.Store(primNodePointerOffset, INVALID_IDX);
        }

        const IndexBufferInfo indexBufferInfo =
        {
            geometryArgs.IndexBufferVaLo,
            geometryArgs.IndexBufferVaHi,
            geometryArgs.IndexBufferByteOffset + inputOffsets.indexOffsetInBytes,
            geometryArgs.IndexBufferFormat,
        };

        // Fetch face indices from index buffer
        uint3 faceIndices = FetchFaceIndices(primId, indexBufferInfo);

        const bool isIndexed = (geometryArgs.IndexBufferFormat != IndexFormatInvalid);

        // Check if vertex indices are within bounds, otherwise make the triangle inactive
        const uint maxIndex = max(faceIndices.x, max(faceIndices.y, faceIndices.z));
        if (maxIndex < geometryArgs.vertexCount)
        {
            const uint64_t transformBufferGpuVa =
                PackUint64(geometryArgs.TransformBufferGpuVaLo, geometryArgs.TransformBufferGpuVaHi);

            // Fetch triangle vertex data from vertex buffer
            tri = FetchTransformedTriangleData(GeometryBuffer[geomId],
                                               faceIndices,
                                               geometryArgs.GeometryStride,
                                               inputOffsets.vertexOffsetInComponents,
                                               geometryArgs.VertexComponentCount,
                                               transformBufferGpuVa,
                                               inputOffsets.transformOffsetInBytes);

            pairInfo = PairTriangles(isIndexed, faceIndices, tri);
        }

        // Generate triangle bounds and update scene bounding box
        const BoundingBox boundingBox = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);

        const bool isActive = IsActive(tri);
        if (isActive)
        {
            if (Settings.sceneBoundsCalculationType == SceneBoundsBasedOnGeometry)
            {
                UpdateSceneBounds(geometryArgs.SceneBoundsByteOffset, boundingBox);
            }
            else if (Settings.sceneBoundsCalculationType == SceneBoundsBasedOnGeometryWithSize)
            {
                // TODO: with tri splitting, need to not update "size" here
                UpdateSceneBoundsWithSize(geometryArgs.SceneBoundsByteOffset, boundingBox);
            }
        }
        else
        {
            // Override v0.x for inactive case
            tri.v0.x = NaN;
        }

        // Count quads produced by the current wave. Note, this includes unpaired triangles as well
        // (marked with a value of -1).
        const bool isActivePrimRef = (pairInfo >= -1) && isActive;
        const uint blockPrimCount = WaveActiveCountBits(isActivePrimRef);
        const uint laneActivePrimIdx = WavePrefixCountBits(isActivePrimRef);

        uint exclusivePrefixCount = 0;

        if (WaveIsFirstLane())
        {
            const uint primRefCountOffset = ShaderConstants.header.offsets.leafNodes + (blockId * sizeof(uint));

            const uint flags = LOCAL_COUNTS_READY | ((blockId == 0) ? LOCAL_PREFIX_SUM_READY : 0);
            DstBuffer.Store(primRefCountOffset, blockPrimCount | flags);

            DeviceMemoryBarrier();

            int prevId = blockId - 1;
            while (prevId >= 0)
            {
                uint count = 0;
                while (1)
                {
                    count = DstBuffer.Load(ShaderConstants.header.offsets.leafNodes + (prevId * sizeof(uint)));
                    if (count & LOCAL_COUNTS_READY)
                    {
                        break;
                    }

                    DeviceMemoryBarrier();
                }

                exclusivePrefixCount += (count & COUNTS_MASK);
                if (count & LOCAL_PREFIX_SUM_READY)
                {
                    break;
                }
                else
                {
                    prevId--;
                }
            }

            DstBuffer.Store(primRefCountOffset,
                (blockPrimCount + exclusivePrefixCount) | ((LOCAL_COUNTS_READY | LOCAL_PREFIX_SUM_READY)));

            DeviceMemoryBarrier();
        }

        exclusivePrefixCount = WaveReadLaneFirst(exclusivePrefixCount);

        // Allocate scratch nodes for active triangle primitives using the precomputed block offset for each
        // triangle geometry
        const uint dstScratchNodeIdx = exclusivePrefixCount + laneActivePrimIdx;

        const bool hasValidQuad = (pairInfo >= 0);
        const uint pairLaneId = hasValidQuad ? pairInfo >> 16 : 0;

        TriangleData tri1;
        tri1.v0 = WaveReadLaneAt(tri.v0, pairLaneId);
        tri1.v1 = WaveReadLaneAt(tri.v1, pairLaneId);
        tri1.v2 = WaveReadLaneAt(tri.v2, pairLaneId);

        const uint primId1 = WaveReadLaneAt(primId, pairLaneId);

        if (hasValidQuad)
        {
            const uint triT0Rotation = (pairInfo & 0xF);
            const uint triT1Rotation = (pairInfo >> 4) & 0xF;

            WriteScratchQuadNode(geometryArgs.LeafNodeDataByteOffset,
                                 dstScratchNodeIdx,
                                 geometryArgs.GeometryIndex,
                                 geometryArgs.GeometryFlags,
                                 tri1,
                                 primId1,
                                 triT1Rotation,
                                 tri,
                                 primId,
                                 triT0Rotation);
        }
        else if (pairInfo == -1)
        {
            // Write out unpaired triangle
            WriteScratchTriangleNode(geometryArgs.LeafNodeDataByteOffset,
                                     dstScratchNodeIdx,
                                     geometryArgs.GeometryIndex,
                                     geometryArgs.GeometryFlags,
                                     tri,
                                     primId);
        }
        else
        {
            // This triangle is a pair in a quad handled by the lead lane. Nothing to do here.
        }

        // ClearFlags for refit and update
        ClearFlagsForRefitAndUpdate(geometryArgs, flattenedPrimitiveIndex, false);

        // Update primitive reference count
        if (blockId == (ShaderRootConstants.geometryIndex - 1))
        {
            const uint globalCount = (blockPrimCount + exclusivePrefixCount);

            WriteTaskCounterData(ShaderConstants.offsets.encodeTaskCounter,
                                 ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET,
                                 globalCount);

            WriteTaskCounterData(ShaderConstants.offsets.encodeTaskCounter,
                                 ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET,
                                 ShaderConstants.numPrimitives);

            const uint threadGroupSize = 64u;
            const uint buildBlockCount = Pow2Align(globalCount, threadGroupSize) / threadGroupSize;

            WriteTaskCounterData(ShaderConstants.offsets.encodeTaskCounter,
                                 ENCODE_TASK_COUNTER_INDIRECT_ARGS,
                                 buildBlockCount);
        }
    }
}
