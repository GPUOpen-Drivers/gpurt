/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include ".\Common.hlsl"
#include ".\BuildCommon.hlsl"

#define RootSig "RootConstants(num32BitConstants=28, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 1)),"\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894))"

[[vk::binding(0, 2)]] RWBuffer<float3>            GeometryBuffer  : register(u0, space1);

// Triangle nodes only
[[vk::binding(0, 0)]] RWByteAddressBuffer         IndexBuffer     : register(u0);
[[vk::binding(1, 0)]] RWStructuredBuffer<float4>  TransformBuffer : register(u1);

[[vk::binding(2, 0)]] RWByteAddressBuffer DstMetadata       : register(u2);
[[vk::binding(3, 0)]] RWByteAddressBuffer ScratchBuffer     : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer SrcBuffer         : register(u4);
[[vk::binding(5, 0)]] RWByteAddressBuffer IndirectArgBuffer : register(u5);

#include "EncodeCommon.hlsl"

[[vk::binding(0, 1)]] ConstantBuffer<GeometryArgs> ShaderConstants : register(b0);

//=====================================================================================================================
// Increment task counter to mark a task / primitive as done
void IncrementTaskCounter()
{
    DeviceMemoryBarrier();
    DstMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET, 1);
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeTriangleNodes(
    in uint3 globalThreadId : SV_DispatchThreadID)
{
#if INDIRECT_BUILD
    // Sourced from Indirect Buffers
    const IndirectBuildOffset buildOffsetInfo = IndirectArgBuffer.Load<IndirectBuildOffset>(0);

    const uint numPrimitives      = buildOffsetInfo.primitiveCount;
    const uint primitiveOffset    = ComputePrimitiveOffset(ShaderConstants);
    const uint vertexOffset       = buildOffsetInfo.firstVertex;
#else
    const uint numPrimitives      = ShaderConstants.NumPrimitives;
    const uint primitiveOffset    = ShaderConstants.PrimitiveOffset;
    const uint vertexOffset       = 0;
#endif

    uint primitiveIndex = globalThreadId.x;
    if (primitiveIndex < numPrimitives)
    {
        EncodeTriangleNode(
            GeometryBuffer,
            IndexBuffer,
            TransformBuffer,
            ShaderConstants,
            primitiveIndex,
            primitiveOffset,
            vertexOffset,
            true);

        IncrementTaskCounter();
    }
}

//=====================================================================================================================
// Fetch API bounding box from source buffer which is a typed R32G32 buffer.
BoundingBox FetchBoundingBoxData(RWBuffer<float3> buffer, uint index, uint boxStrideInElements)
{
    const uint baseElementIndex = index * boxStrideInElements;

    float2 data[3];
    data[0] = buffer[baseElementIndex+0].xy;
    data[1] = buffer[baseElementIndex+1].xy;
    data[2] = buffer[baseElementIndex+2].xy;

    BoundingBox bbox;
    bbox.min = float3(data[0].xy, data[1].x);
    bbox.max = float3(data[1].y, data[2].xy);

    // Generate degenerate bounding box for zero area bounds
    if (ComputeBoxSurfaceArea(bbox) == 0)
    {
        bbox.min.x = +FLT_MAX;
        bbox.min.y = +FLT_MAX;
        bbox.min.z = +FLT_MAX;
        bbox.max.x = -FLT_MAX;
        bbox.max.y = -FLT_MAX;
        bbox.max.z = -FLT_MAX;
    }

    return bbox;
}

//=====================================================================================================================
void WriteScratchBoundingBoxNode(
    RWByteAddressBuffer buffer,
    uint                primitiveOffset,
    uint                primitiveIndex,
    uint                geometryIndex,
    uint                flags,
    in BoundingBox      bbox)
{
    uint offset = (primitiveIndex * ByteStrideScratchNode) +
                  (primitiveOffset * ByteStrideScratchNode)  +
                  ShaderConstants.LeafNodeDataByteOffset;
    uint4 data;

    // LeafNode.bbox_min_or_v0, primitiveIndex
    data = uint4(asuint(bbox.min), primitiveIndex);
    buffer.Store4(offset + SCRATCH_NODE_BBOX_MIN_OFFSET, data);

    // LeafNode.bbox_max_or_v1, geometryIndex
    data = uint4(asuint(bbox.max), geometryIndex);
    buffer.Store4(offset + SCRATCH_NODE_BBOX_MAX_OFFSET, data);

    // LeafNode.v2, parent
    data = uint4(0xffffffff, 0xffffffff, 0xffffffff, 0);
    buffer.Store4(offset + SCRATCH_NODE_V2_OFFSET, data);

    // type, flags, splitBox, numPrimitivesAndDoCollapse
    uint typeAndId = NODE_TYPE_USER_NODE_PROCEDURAL;
    const float cost = SAH_COST_AABBB_INTERSECTION * ComputeBoxSurfaceArea(bbox);

    data = uint4(typeAndId, flags, INVALID_IDX, asuint(cost));
    buffer.Store4(offset + SCRATCH_NODE_TYPE_OFFSET, data);
}

//=====================================================================================================================
void WriteProceduralNodeBoundingBox(
    uint        metadataSize,
    uint        nodePointer,
    BoundingBox bbox)
{
    const uint nodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);

    DstMetadata.Store<float3>(nodeOffset + USER_NODE_PROCEDURAL_MIN_OFFSET,bbox.min);
    DstMetadata.Store<float3>(nodeOffset + USER_NODE_PROCEDURAL_MAX_OFFSET,bbox.max);
}

//=====================================================================================================================
void WriteProceduralNodePrimitiveData(
    uint metadataSize,
    uint nodePointer,
    uint primitiveIndex)
{
    const uint nodeOffset            = metadataSize + ExtractNodePointerOffset(nodePointer);
    const uint geometryIndexAndFlags = PackGeometryIndexAndFlags(ShaderConstants.GeometryIndex,
                                                                 ShaderConstants.GeometryFlags);
    DstMetadata.Store(nodeOffset + USER_NODE_PROCEDURAL_GEOMETRY_INDEX_AND_FLAGS_OFFSET, geometryIndexAndFlags);
    DstMetadata.Store(nodeOffset + USER_NODE_PROCEDURAL_PRIMITIVE_INDEX_OFFSET, primitiveIndex);
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeAABBNodes(
    in uint3 globalThreadId : SV_DispatchThreadID)
{
#if INDIRECT_BUILD
    // Sourced from Indirect Buffers
    const IndirectBuildOffset buildOffsetInfo = IndirectArgBuffer.Load<IndirectBuildOffset>(0);

    const uint numPrimitives      = buildOffsetInfo.primitiveCount;
    const uint primitiveOffset    = ComputePrimitiveOffset(ShaderConstants);
    const uint destLeafByteOffset = primitiveOffset * USER_NODE_PROCEDURAL_SIZE;
#else
    const uint numPrimitives      = ShaderConstants.NumPrimitives;
    const uint primitiveOffset    = ShaderConstants.PrimitiveOffset;
    const uint destLeafByteOffset = ShaderConstants.DestLeafByteOffset;
#endif

    if (globalThreadId.x == 0)
    {
        WriteGeometryInfo(ShaderConstants, primitiveOffset, numPrimitives, DECODE_PRIMITIVE_STRIDE_AABB);
    }

    uint primitiveIndex = globalThreadId.x;
    if (primitiveIndex < ShaderConstants.NumPrimitives)
    {
        const uint metadataSize =
            IsUpdate(ShaderConstants.BuildFlags) ? SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : ShaderConstants.metadataSizeInBytes;

        // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
        const AccelStructOffsets offsets =
            SrcBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

        const uint basePrimNodePtr =
            IsUpdate(ShaderConstants.BuildFlags) ? offsets.primNodePtrs : ShaderConstants.BasePrimNodePtrOffset;

        // Get bounds for this thread
        const BoundingBox boundingBox = FetchBoundingBoxData(GeometryBuffer,
                                                             primitiveIndex,
                                                             ShaderConstants.GeometryStride);

        const uint primNodePointerOffset =
            metadataSize + basePrimNodePtr + ((primitiveOffset + primitiveIndex) * sizeof(uint));

        if (IsUpdate(ShaderConstants.BuildFlags))
        {
            const uint nodePointer = SrcBuffer.Load(primNodePointerOffset);

            // If the primitive was active during the initial build, it will have a valid primitive node pointer.
            if (nodePointer != INVALID_IDX)
            {
                WriteProceduralNodeBoundingBox(metadataSize, nodePointer, boundingBox);

                if (ShaderConstants.isUpdateInPlace == false)
                {
                    WriteProceduralNodePrimitiveData(metadataSize, nodePointer, primitiveIndex);

                    DstMetadata.Store(primNodePointerOffset, nodePointer);
                }

                PushNodeForUpdate(ShaderConstants,
                                  GeometryBuffer,
                                  IndexBuffer,
                                  TransformBuffer,
                                  metadataSize,
                                  ShaderConstants.BaseUpdateStackScratchOffset,
                                  ShaderConstants.TriangleCompressionMode,
                                  ShaderConstants.isUpdateInPlace,
                                  nodePointer,
                                  0,
                                  0,
                                  boundingBox,
                                  true);
            }
            else if (ShaderConstants.isUpdateInPlace == false)
            {
                // For inactive primitives, just copy over the primitive node pointer.
                DstMetadata.Store(primNodePointerOffset, nodePointer);
            }
        }
        else
        {
            if (ShaderConstants.SceneBoundsCalculationType == SceneBoundsBasedOnGeometry)
            {
                UpdateSceneBounds(ScratchBuffer, ShaderConstants.SceneBoundsByteOffset, boundingBox);
            }
            else if (ShaderConstants.SceneBoundsCalculationType == SceneBoundsBasedOnGeometryWithSize)
            {
                UpdateSceneBoundsWithSize(ScratchBuffer, ShaderConstants.SceneBoundsByteOffset, boundingBox);
            }
            else
            {
                UpdateCentroidSceneBoundsWithSize(ScratchBuffer, ShaderConstants.SceneBoundsByteOffset, boundingBox);
            }

            WriteScratchBoundingBoxNode(ScratchBuffer,
                                        primitiveOffset,
                                        primitiveIndex,
                                        ShaderConstants.GeometryIndex,
                                        ShaderConstants.GeometryFlags,
                                        boundingBox);

            // Store invalid prim node pointer for now during first time builds.
            // If the Procedural node is active, BuildQBVH will update it.
            DstMetadata.Store(primNodePointerOffset, INVALID_IDX);

            const uint numLeafNodesOffset = metadataSize + ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET;
            DstMetadata.InterlockedAdd(numLeafNodesOffset, 1);
        }

        // ClearFlags for refit and update
        const uint flattenedPrimitiveIndex = primitiveOffset + primitiveIndex;

        {
            const uint flagOffset = ShaderConstants.PropagationFlagsScratchOffset + (flattenedPrimitiveIndex * sizeof(uint));
            ScratchBuffer.Store(flagOffset, 0);
        }

        IncrementTaskCounter();
    }
}
