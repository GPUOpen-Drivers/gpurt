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
#include "BuildCommonScratch.hlsl"

struct IndirectBuildOffset
{
    uint primitiveCount;
    uint primitiveOffset;
    uint firstVertex;
    uint transformOffset;
};

struct GeometryArgs
{
    uint metadataSizeInBytes;           // Size of the metadata header
    uint NumPrimitives;                 // Number of primitives
    uint LeafNodeDataByteOffset;        // Leaf node data byte offset
    uint PrimitiveOffset;               // Primitive Offset
    uint SceneBoundsByteOffset;         // Scene bounds byte offset
    uint PropagationFlagsScratchOffset; // Flags byte offset
    uint BaseUpdateStackScratchOffset;  // Update stack offset
    uint IndexBufferByteOffset;         // Index buffer byte offset
    uint HasValidTransform;             // Has a valid transform
    uint IndexBufferFormat;             // Index buffer format
    uint GeometryStride;                // Geometry buffer stride in terms of components for vertices. 0 if the
                                        // stride is accounted for in the SRD. R32G32 elements for AABBs.
    uint GeometryIndex;                 // Index of the geometry description that owns this node
    uint BaseGeometryInfoOffset;        // Base offset for the geometry info
    uint BasePrimNodePtrOffset;         // Base offset for the prim nodes
    uint BuildFlags;                    // DDI acceleration structure build flags
    uint isUpdateInPlace;               // Is update in place
    uint GeometryFlags;                 // Geometry flags (D3D12_RAYTRACING_GEOMETRY_FLAGS)
    uint VertexComponentCount;          // Components in vertex buffer format
    uint vertexCount;                   // Vertex count
    uint DestLeafByteOffset;            // Offset to this geometry's location in the dest leaf node buffer
    uint LeafNodeExpansionFactor;       // Leaf node expansion factor (> 1 for triangle splitting)
    uint IndexBufferInfoScratchOffset;
    uint IndexBufferVaLo;
    uint IndexBufferVaHi;
    uint SceneBoundsCalculationType;
    uint trianglePairingSearchRadius;   // The search radius for paired triangle, used by EarlyCompression
};

//======================================================================================================================
// Get face indices from 16-bit index buffer
uint3 GetFaceIndices16(RWByteAddressBuffer buffer, uint faceIndex, uint indexBufferByteOffset)
{
    const uint stride = 6; // 3 vertices per triangle with 2-byte indices
    uint address = (faceIndex * stride) + indexBufferByteOffset;

    // Load address must be 4-byte aligned
    const bool unalignedRead = (address % 4 == 2);
    if (unalignedRead)
    {
        // Align down load address
        address -= 2;
    }

    // Load index buffer data
    const uint2 data = buffer.Load2(address);

    uint3 faceIndices;
    if (unalignedRead == false)
    {
        faceIndices.x = (data.x & 0xFFFF);
        faceIndices.y = (data.x >> 16);
        faceIndices.z = (data.y & 0xFFFF);
    }
    else
    {
        faceIndices.x = (data.x >> 16);
        faceIndices.y = (data.y & 0xFFFF);
        faceIndices.z = (data.y >> 16);
    }

    return faceIndices;
}

//=====================================================================================================================
// Get face indices from 16-bit index buffer
uint FetchIndex16(RWByteAddressBuffer buffer, uint faceIndex, uint indexOffset, uint indexBufferByteOffset)
{
    const uint indexStride = 2;
    const uint faceStride = 3 * indexStride; // 3 vertices per triangle with 2-byte indices
    uint address = (faceIndex * faceStride) + (indexOffset * indexStride) + indexBufferByteOffset;

    // Load address must be 4-byte aligned
    const bool unalignedRead = (address % 4 == 2);
    if (unalignedRead)
    {
        // Align down load address
        address -= 2;
    }

    // Load index buffer data
    const uint data = buffer.Load(address);

    uint index;
    if (unalignedRead == false)
    {
        index = (data & 0xFFFF);
    }
    else
    {
        index = (data >> 16);
    }

    return index;
}

//=====================================================================================================================
// Get face indices from 32-bit index buffer
uint FetchIndex32(RWByteAddressBuffer buffer, uint faceIndex, uint indexOffset, uint indexBufferByteOffset)
{
    const uint indexStride = 4;
    const uint faceStride = 3 * indexStride; // 3 vertices per triangle with 4-byte indices

    const uint baseOffset = (faceIndex * faceStride) + (indexOffset * indexStride) + indexBufferByteOffset;
    return buffer.Load(baseOffset);
}

//=====================================================================================================================
// Get face indices from 32-bit index buffer
uint3 GetFaceIndices32(RWByteAddressBuffer buffer, uint faceIndex, uint indexBufferByteOffset)
{
    const uint stride = 12; // 3 vertices per triangle with 4-byte indices

    const uint baseOffset = (faceIndex * 12) + indexBufferByteOffset;
    return buffer.Load3(baseOffset);
}

//=====================================================================================================================
// Vertex buffers only require an address and stride alignment of the format component size not the entire element size.
// If the input data is not naturally aligned, we cannot use a single typed fetch for the 2-3 components. In this case,
// we need to fetch each component separately.
float3 FetchVertexPerComponent(GeometryArgs geometryArgs, RWBuffer<float3> buffer, uint index, uint strideInComponents)
{
    const uint firstComponentIndex = index * strideInComponents;

    float3 vertex;
    vertex.x = buffer[firstComponentIndex+0].x;
    vertex.y = buffer[firstComponentIndex+1].x;
    if (geometryArgs.VertexComponentCount > 2)
    {
        vertex.z = buffer[firstComponentIndex+2].x;
    }
    else
    {
        vertex.z = 0;
    }

    return vertex;
}

//=====================================================================================================================
TriangleData FetchTriangleData(
    GeometryArgs     geometryArgs,
    RWBuffer<float3> buffer,
    uint3            index,
    uint             strideInComponents,
    uint             vertexOffset)
{
    TriangleData tri;

    const uint3 i = index + vertexOffset;

    if (strideInComponents == 0)
    {
        tri.v0 = buffer[i.x];
        tri.v1 = buffer[i.y];
        tri.v2 = buffer[i.z];
    }
    else
    {
        tri.v0 = FetchVertexPerComponent(geometryArgs, buffer, i.x, strideInComponents);
        tri.v1 = FetchVertexPerComponent(geometryArgs, buffer, i.y, strideInComponents);
        tri.v2 = FetchVertexPerComponent(geometryArgs, buffer, i.z, strideInComponents);
    }

    return tri;
}

//======================================================================================================================
uint CalcTriangleBoxNodeFlags(
    in uint geometryFlags)
{
    // Determine opacity from geometry flags
    uint nodeFlags =
        (geometryFlags & D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE) ? 1u << BOX_NODE_FLAGS_ONLY_OPAQUE_SHIFT :
                                                                  1u << BOX_NODE_FLAGS_ONLY_NON_OPAQUE_SHIFT;

    // Note, a bottom-level acceleration structure can only contain a single geometry type.
    nodeFlags |= 1u << BOX_NODE_FLAGS_ONLY_TRIANGLES_SHIFT;

    return nodeFlags;
}

//======================================================================================================================
void WriteScratchTriangleNode(
    GeometryArgs        geometryArgs,
    uint                primitiveOffset,
    uint                primitiveIndex,
    uint                geometryIndex,
    uint                geometryFlags,
    uint                instanceMask,
    in BoundingBox      bbox,
    in TriangleData     tri)
{
    uint offset = (primitiveIndex  * ByteStrideScratchNode) +
                  (primitiveOffset * ByteStrideScratchNode) +
                  geometryArgs.LeafNodeDataByteOffset;

    uint4 data;

    // LeafNode.bbox_min_or_v0, primitiveIndex
    data = uint4(asuint(tri.v0), primitiveIndex);
    ScratchBuffer.Store4(offset + SCRATCH_NODE_V0_OFFSET, data);

    // LeafNode.bbox_max_or_v1, geometryIndex
    data = uint4(asuint(tri.v1), geometryIndex);
    ScratchBuffer.Store4(offset + SCRATCH_NODE_V1_OFFSET, data);

    // LeafNode.v2, parent
    data = uint4(asuint(tri.v2), 0);
    ScratchBuffer.Store4(offset + SCRATCH_NODE_V2_OFFSET, data);

    const float cost = SAH_COST_TRIANGLE_INTERSECTION * ComputeBoxSurfaceArea(bbox);

    // type, flags, splitBox, numPrimitivesAndDoCollapse
    const uint triangleId        = CalcUncompressedTriangleId(geometryFlags);
    const uint triangleTypeAndId = (triangleId << 3) | NODE_TYPE_TRIANGLE_0;

    uint flags = CalcTriangleBoxNodeFlags(geometryFlags);

    // Disable triangle splitting if geometry descriptor allows duplicate anyHit invocation
    if ((geometryFlags & D3D12_RAYTRACING_GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION) != 0)
    {
        flags |= SCRATCH_NODE_FLAGS_DISABLE_TRIANGLE_SPLIT_MASK;
    }

    const uint packedFlags = PackInstanceMaskAndNodeFlags(instanceMask, flags);

    data = uint4(triangleTypeAndId, packedFlags, INVALID_IDX, asuint(cost));
    ScratchBuffer.Store4(offset + SCRATCH_NODE_TYPE_OFFSET, data);
}

//=====================================================================================================================
#if INDIRECT_BUILD
uint ComputePrimitiveOffset(
    GeometryArgs geometryArgs)
{
    uint primitiveOffset = 0;

    const uint metadataSize = IsUpdate() ?
                              SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : geometryArgs.metadataSizeInBytes;

    // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
    const AccelStructOffsets offsets =
        SrcBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

    const uint baseGeometryInfoOffset = IsUpdate() ? offsets.geometryInfo : geometryArgs.BaseGeometryInfoOffset;

    for (uint geomIdx = 0; geomIdx < geometryArgs.GeometryIndex; ++geomIdx)
    {
        const uint geometryInfoOffset =
            metadataSize + baseGeometryInfoOffset +
            (geomIdx * GEOMETRY_INFO_SIZE);

        GeometryInfo info;
        info = DstMetadata.Load<GeometryInfo>(geometryInfoOffset);
        uint primitiveCount = ExtractGeometryInfoNumPrimitives(info.geometryFlagsAndNumPrimitives);

        primitiveOffset += primitiveCount;
    }

    return primitiveOffset;
}
#endif

//======================================================================================================================
void WriteGeometryInfo(
    GeometryArgs geometryArgs,
    uint         primitiveOffset,
    uint         numPrimitives,
    uint         primitiveStride)
{
    // For builds and not-in-place updates, write the geometry info.
    if ((IsUpdate() == false) ||
        (IsUpdate() && (geometryArgs.isUpdateInPlace == false)))
    {
        const uint metadataSize = IsUpdate() ?
            SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : geometryArgs.metadataSizeInBytes;

        // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
        const AccelStructOffsets offsets =
            SrcBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

        const uint baseGeometryInfoOffset =
            IsUpdate() ? offsets.geometryInfo : geometryArgs.BaseGeometryInfoOffset;

        const uint geometryInfoOffset =
            metadataSize + baseGeometryInfoOffset +
            (geometryArgs.GeometryIndex * GEOMETRY_INFO_SIZE);

        GeometryInfo info;
        info.geometryBufferOffset = primitiveOffset * primitiveStride;
        info.primNodePtrsOffset   = primitiveOffset * sizeof(uint);
        info.geometryFlagsAndNumPrimitives  =
            PackGeometryFlagsAndNumPrimitives(geometryArgs.GeometryFlags, numPrimitives);

        DstMetadata.Store<GeometryInfo>(geometryInfoOffset, info);
    }

    if ((IsUpdate() == false) &&
        (Settings.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
        // when enableEarlyTriangleCompression, we don't allocate scratch space for indexbuffer info
        (Settings.enableEarlyPairCompression == false))
    {
        IndexBufferInfo indexBufferInfo;
        indexBufferInfo.gpuVaLo    = geometryArgs.IndexBufferVaLo;
        indexBufferInfo.gpuVaHi    = geometryArgs.IndexBufferVaHi;
        indexBufferInfo.byteOffset = geometryArgs.IndexBufferByteOffset;
        indexBufferInfo.format     = geometryArgs.IndexBufferFormat;

        const uint indexBufferInfoOffset = geometryArgs.IndexBufferInfoScratchOffset +
                                            (geometryArgs.GeometryIndex * sizeof(IndexBufferInfo));
        ScratchBuffer.Store<IndexBufferInfo>(indexBufferInfoOffset, indexBufferInfo);
    }
}

//======================================================================================================================
TriangleData FetchTransformedTriangleData(
    in GeometryArgs               geometryArgs,
    in RWBuffer<float3>           geometryBuffer,
    in uint3                      faceIndices,
    in uint                       geometryStride,
    in uint                       vertexOffset,
    in bool                       hasValidTransform,
    in RWStructuredBuffer<float4> transformBuffer)
{
    // Fetch triangle vertex data from vertex buffer
    TriangleData tri =
        FetchTriangleData(geometryArgs, geometryBuffer, faceIndices, geometryStride, vertexOffset);

    // If this geometry has a valid transformation matrix. Transform each vertex using this matrix.
    if (hasValidTransform)
    {
        float4x4 transform;
        transform[0] = transformBuffer[0];
        transform[1] = transformBuffer[1];
        transform[2] = transformBuffer[2];
        transform[3] = float4(0, 0, 0, 1);

        tri.v0 = mul(transform, float4(tri.v0, 1)).xyz;
        tri.v1 = mul(transform, float4(tri.v1, 1)).xyz;
        tri.v2 = mul(transform, float4(tri.v2, 1)).xyz;
    }

    if (any(isinf(tri.v0)) || any(isinf(tri.v1)) || any(isinf(tri.v2)))
    {
        tri.v0 = float3(0, 0, 0);
        tri.v1 = float3(0, 0, 0);
        tri.v2 = float3(0, 0, 0);
    }

    return tri;
}

//======================================================================================================================
uint3 FetchFaceIndices(
    in RWByteAddressBuffer buffer,
    in uint                index,
    in uint                indexBufferByteOffset,
    in uint                indexBufferFormat)
{
    // Fetch face indices from index buffer
    uint3 faceIndices;
    if (indexBufferFormat == IndexFormatU16)
    {
        faceIndices = GetFaceIndices16(buffer, index, indexBufferByteOffset);
    }
    else if (indexBufferFormat == IndexFormatU32)
    {
        faceIndices = GetFaceIndices32(buffer, index, indexBufferByteOffset);
    }
    else
    {
        const uint startIndex = (index * 3);
        faceIndices.x = startIndex;
        faceIndices.y = startIndex + 1;
        faceIndices.z = startIndex + 2;
    }
    return faceIndices;
}

//=====================================================================================================================
uint FetchIndex(
    in RWByteAddressBuffer buffer,
    in uint                primitiveIndex,
    in uint                indexOffset,
    in uint                indexBufferByteOffset,
    in uint                indexBufferFormat)
{
    // Fetch face indices from index buffer
    uint index;
    if (indexBufferFormat == IndexFormatU16)
    {
        index = FetchIndex16(buffer, primitiveIndex, indexOffset, indexBufferByteOffset);
    }
    else if (indexBufferFormat == IndexFormatU32)
    {
        index = FetchIndex32(buffer, primitiveIndex, indexOffset, indexBufferByteOffset);
    }
    else
    {
        index = (primitiveIndex * 3) + indexOffset;
    }

    return index;
}

//======================================================================================================================
bool IsActive(TriangleData tri)
{
    return ((isnan(tri.v0.x) == false) && (isnan(tri.v1.x) == false) && (isnan(tri.v2.x) == false));
}

//=====================================================================================================================
void PushNodeForUpdate(
    GeometryArgs               geometryArgs,
    RWBuffer<float3>           GeometryBuffer,
    RWByteAddressBuffer        IndexBuffer,
    RWStructuredBuffer<float4> TransformBuffer,
    uint                       metadataSize,
    uint                       baseUpdateStackScratchOffset,
    uint                       triangleCompressionMode,
    uint                       isUpdateInPlace,
    uint                       nodePointer,
    uint                       triangleId,
    uint                       vertexOffset,
    uint                       instanceMask,
    in BoundingBox             boundingBox,
    bool                       writeNodesToUpdateStack)
{
    // Handle two triangles sharing a bounding box when pair compression is enabled.
    uint childNodePointer = nodePointer;
    if ((triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) && (GetNodeType(nodePointer) == NODE_TYPE_TRIANGLE_0))
    {
        const uint triNodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);

        // Check whether or not there is a triangle 1 in this node.
        if (((triangleId >> (NODE_TYPE_TRIANGLE_1 * TRIANGLE_ID_BIT_STRIDE)) & 0xf) != 0)
        {
            // Triangle 0 is not a child pointer in the parent box node, so correct the node pointer used for the
            // comparison when finding the child index in the parent below.
            childNodePointer |= NODE_TYPE_TRIANGLE_1;

            uint otherPrimIndexOffset = TRIANGLE_NODE_PRIMITIVE_INDEX1_OFFSET;
            const uint otherPrimIndex = SrcBuffer.Load(triNodeOffset + otherPrimIndexOffset);

            // Fetch face indices from index buffer.
            const uint3 faceIndices = FetchFaceIndices(IndexBuffer,
                                                       otherPrimIndex,
                                                       geometryArgs.IndexBufferByteOffset,
                                                       geometryArgs.IndexBufferFormat);

            // Check if vertex indices are within bounds.
            if ((faceIndices.x < geometryArgs.vertexCount) &&
                (faceIndices.y < geometryArgs.vertexCount) &&
                (faceIndices.z < geometryArgs.vertexCount))
            {
                // Fetch triangle vertex data from vertex buffer.
                const TriangleData tri = FetchTransformedTriangleData(geometryArgs,
                                                                      GeometryBuffer,
                                                                      faceIndices,
                                                                      geometryArgs.GeometryStride,
                                                                      vertexOffset,
                                                                      geometryArgs.HasValidTransform,
                                                                      TransformBuffer);

                const BoundingBox otherBox = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);

                // Merge the bounding boxes of the two triangles.
                boundingBox.min = min(boundingBox.min, otherBox.min);
                boundingBox.max = max(boundingBox.max, otherBox.max);
            }
        }
    }

    // Fetch parent node pointer
    const uint parentNodePointer = ReadParentPointer(metadataSize,
                                                     nodePointer);

    // Update out of place destination buffer
    if (isUpdateInPlace == 0)
    {
        WriteParentPointer(metadataSize,
                           nodePointer,
                           parentNodePointer);
    }

    // Compute box node count and child index in parent node
    uint boxNodeCount = 0;
    const uint childIdx = ComputeChildIndexAndValidBoxCount(metadataSize,
                                                            parentNodePointer,
                                                            childNodePointer,
                                                            boxNodeCount);

    // If even a single child node is a box node, this is a node higher up the tree. Skip queueing parent node as another
    // leaf at the bottom of the tree will queue its parent which will handle our parent node.

    // B B B B --> 4 --> Not possible
    // B x B x --> 2 --> Not possible
    // B L B L --> 4 --> Skip queueing
    // L x B x --> 1 --> Skip queueing
    // L x x x --> 0 --> Queue parent node
    // L x L x --> 0 --> Queue parent node
    // L L L L --> 0 --> Queue parent node

    // Always perform update for out-of-place updates
    bool performUpdate = (isUpdateInPlace == false);

    uint boxOffset;

    const uint parentNodeOffset = metadataSize + ExtractNodePointerOffset(parentNodePointer);

    {
        if (IsBoxNode32(parentNodePointer))
        {
            BoundingBox originalBox;

            boxOffset = childIdx * FLOAT32_BBOX_STRIDE;
            originalBox.min = DstMetadata.Load<float3>(parentNodeOffset + FLOAT32_BOX_NODE_BB0_MIN_OFFSET + boxOffset);
            originalBox.max = DstMetadata.Load<float3>(parentNodeOffset + FLOAT32_BOX_NODE_BB0_MAX_OFFSET + boxOffset);

            if (any(originalBox.min != boundingBox.min) ||
                any(originalBox.max != boundingBox.max))
            {
                DstMetadata.Store<float3>(parentNodeOffset + FLOAT32_BOX_NODE_BB0_MIN_OFFSET + boxOffset, boundingBox.min);
                DstMetadata.Store<float3>(parentNodeOffset + FLOAT32_BOX_NODE_BB0_MAX_OFFSET + boxOffset, boundingBox.max);
                performUpdate = true;
            }
        }
        else
        {
            boxOffset = childIdx * FLOAT16_BBOX_STRIDE;
            const uint3 originalBox16 = DstMetadata.Load<uint3>(parentNodeOffset + FLOAT16_BOX_NODE_BB0_OFFSET + boxOffset);

            const uint3 boundingBox16 = CompressBBoxToUint3(boundingBox);

            if (any(originalBox16 != boundingBox16))
            {
                DstMetadata.Store<float3>(parentNodeOffset + FLOAT16_BOX_NODE_BB0_OFFSET + boxOffset, boundingBox16);
                performUpdate = true;
            }
        }
    }

    const bool canWriteNodeToUpdateStack = (childIdx == 0) && (boxNodeCount == 0);
    if (canWriteNodeToUpdateStack && performUpdate)
    {
        if (isUpdateInPlace == false)
        {
            CopyChildPointersAndFlags(parentNodePointer, metadataSize);
        }
    }

    // If this is the first child in the parent node with all leaf children, queue parent pointer to
    // stack in scratch memory.
    // @note Right now we queue the parent node for update regardless of whether the leaf nodes' bounding boxes change
    // or not. We could optimize this by queuing the parent node only if any of the leaf nodes' bounding boxes change.
    if (writeNodesToUpdateStack && canWriteNodeToUpdateStack)
    {
        PushNodeToUpdateStack(baseUpdateStackScratchOffset, parentNodePointer);
    }
}

//======================================================================================================================
void ClearFlagsForRefitAndUpdate(
    GeometryArgs    geometryArgs,
    const uint      flattenedPrimitiveIndex,
    const bool      isAABB)
{
    if (isAABB)
    {
        const uint flagOffset = geometryArgs.PropagationFlagsScratchOffset + (flattenedPrimitiveIndex * sizeof(uint));
        ScratchBuffer.Store(flagOffset, 0);
    }
    else // Triangle
    {
        const uint stride = geometryArgs.LeafNodeExpansionFactor * sizeof(uint);
        const uint flagOffset = geometryArgs.PropagationFlagsScratchOffset + (flattenedPrimitiveIndex * stride);
        const uint initValue = Settings.enableFastLBVH ? 0xffffffffu : 0;
        for (uint i = 0; i < geometryArgs.LeafNodeExpansionFactor; ++i)
        {
            ScratchBuffer.Store(flagOffset + (i * sizeof(uint)), initValue);
        }
    }
}

//======================================================================================================================
void EncodeTriangleNode(
    RWBuffer<float3>           GeometryBuffer,
    RWByteAddressBuffer        IndexBuffer,
    RWStructuredBuffer<float4> TransformBuffer,
    GeometryArgs               geometryArgs,
    uint                       primitiveIndex,
    uint                       geometryBasePrimOffset,
    uint                       vertexOffset,
    bool                       writeNodesToUpdateStack)
{
    const uint metadataSize = IsUpdate() ?
        SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : geometryArgs.metadataSizeInBytes;

    // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
    const AccelStructOffsets offsets =
        SrcBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

    const uint basePrimNodePtr =
        IsUpdate() ? offsets.primNodePtrs : geometryArgs.BasePrimNodePtrOffset;

    const uint flattenedPrimitiveIndex = geometryBasePrimOffset + primitiveIndex;
    const uint primNodePointerOffset =
        metadataSize + basePrimNodePtr + (flattenedPrimitiveIndex * sizeof(uint));

    // Fetch face indices from index buffer
    uint3 faceIndices = FetchFaceIndices(IndexBuffer,
                                         primitiveIndex,
                                         geometryArgs.IndexBufferByteOffset,
                                         geometryArgs.IndexBufferFormat);

    // Check if vertex indices are within bounds, otherwise make the triangle inactive
    if ((faceIndices.x < geometryArgs.vertexCount) &&
        (faceIndices.y < geometryArgs.vertexCount) &&
        (faceIndices.z < geometryArgs.vertexCount))
    {
        // Fetch triangle vertex data from vertex buffer
        TriangleData tri = FetchTransformedTriangleData(geometryArgs,
                                                        GeometryBuffer,
                                                        faceIndices,
                                                        geometryArgs.GeometryStride,
                                                        vertexOffset,
                                                        geometryArgs.HasValidTransform,
                                                        TransformBuffer);

        uint nodePointer = INVALID_IDX;
        uint triangleId  = 0;

        // Generate triangle bounds and update scene bounding box
        const BoundingBox boundingBox = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);

        // Set the instance inclusion mask to 0 for degenerate triangles so that they are culled out.
        const uint instanceMask = (boundingBox.min.x > boundingBox.max.x) ? 0 : 0xff;

        if (IsUpdate())
        {
            nodePointer = SrcBuffer.Load(primNodePointerOffset);

            // If the primitive was active during the initial build, it will have a valid primitive node pointer.
            if (nodePointer != INVALID_IDX)
            {
                const uint nodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);
                const uint nodeType   = GetNodeType(nodePointer);
                {
                    uint3 vertexOffsets;

                    if (Settings.triangleCompressionMode != NO_TRIANGLE_COMPRESSION)
                    {
                        triangleId = SrcBuffer.Load(nodeOffset + TRIANGLE_NODE_ID_OFFSET);
                        vertexOffsets = CalcTriangleCompressionVertexOffsets(nodeType, triangleId);
                    }
                    else
                    {
                        triangleId = CalcUncompressedTriangleId(geometryArgs.GeometryFlags);
                        vertexOffsets = CalcTriangleVertexOffsets(nodeType);
                    }

                    DstMetadata.Store3(nodeOffset + vertexOffsets.x, asuint(tri.v0));
                    DstMetadata.Store3(nodeOffset + vertexOffsets.y, asuint(tri.v1));
                    DstMetadata.Store3(nodeOffset + vertexOffsets.z, asuint(tri.v2));

                    if (geometryArgs.isUpdateInPlace == false)
                    {
                        const uint geometryIndexAndFlags = PackGeometryIndexAndFlags(geometryArgs.GeometryIndex,
                            geometryArgs.GeometryFlags);
                        {
                            DstMetadata.Store(
                                nodeOffset + TRIANGLE_NODE_GEOMETRY_INDEX_AND_FLAGS_OFFSET, geometryIndexAndFlags);
                        }

                        const uint primIndexOffset = CalcPrimitiveIndexOffset(nodePointer);

                        {
                            DstMetadata.Store(
                                nodeOffset + TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET + primIndexOffset, primitiveIndex);
                        }

                        DstMetadata.Store(nodeOffset + TRIANGLE_NODE_ID_OFFSET, triangleId);
                    }
                }
            }

            if (geometryArgs.isUpdateInPlace == false)
            {
                DstMetadata.Store(primNodePointerOffset, nodePointer);
            }

            // The shared bounding box for this pair of triangles will be updated by the thread handling triangle 0.
            const bool skipPairUpdatePush = (Settings.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
                                            (GetNodeType(nodePointer) == NODE_TYPE_TRIANGLE_1);

            if ((nodePointer != INVALID_IDX) && (skipPairUpdatePush == false))
            {
                PushNodeForUpdate(geometryArgs,
                                  GeometryBuffer,
                                  IndexBuffer,
                                  TransformBuffer,
                                  metadataSize,
                                  geometryArgs.BaseUpdateStackScratchOffset,
                                  Settings.triangleCompressionMode,
                                  geometryArgs.isUpdateInPlace,
                                  nodePointer,
                                  triangleId,
                                  vertexOffset,
                                  instanceMask,
                                  boundingBox,
                                  writeNodesToUpdateStack);
            }
        }
        else
        {
            if (Settings.triangleCompressionMode != PAIR_TRIANGLE_COMPRESSION)
            {
                const uint numLeafNodesOffset = metadataSize + ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET;
                DstMetadata.InterlockedAdd(numLeafNodesOffset, 1);
            }

            if (IsActive(tri))
            {
                if (geometryArgs.SceneBoundsCalculationType == SceneBoundsBasedOnGeometry)
                {
                    UpdateSceneBounds(geometryArgs.SceneBoundsByteOffset, boundingBox);
                }
                else if (geometryArgs.SceneBoundsCalculationType == SceneBoundsBasedOnGeometryWithSize)
                {
                    UpdateSceneBoundsWithSize(geometryArgs.SceneBoundsByteOffset, boundingBox);
                }
                else
                {
                    UpdateCentroidSceneBoundsWithSize(geometryArgs.SceneBoundsByteOffset, boundingBox);
                }
            }
            else
            {
                // Override v0.x for inactive case
                tri.v0.x = NaN;
            }

            WriteScratchTriangleNode(geometryArgs,
                                     geometryBasePrimOffset,
                                     primitiveIndex,
                                     geometryArgs.GeometryIndex,
                                     geometryArgs.GeometryFlags,
                                     instanceMask,
                                     boundingBox,
                                     tri);

            // Store invalid prim node pointer for now during first time builds.
            // If the triangle is active, BuildQBVH will write it in.
            DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
        }
    }
    else
    {
        if (IsUpdate() == false)
        {
            // Deactivate primitive by setting bbox_min_or_v0.x to NaN
            const uint scratchLeafNodeOffset =
                geometryArgs.LeafNodeDataByteOffset + (flattenedPrimitiveIndex * ByteStrideScratchNode);

            ScratchBuffer.Store(scratchLeafNodeOffset, NaN);

            DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
        }
        else if (geometryArgs.isUpdateInPlace == false)
        {
            DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
        }
    }

    // ClearFlags for refit and update
    ClearFlagsForRefitAndUpdate(geometryArgs, flattenedPrimitiveIndex, false);
}
