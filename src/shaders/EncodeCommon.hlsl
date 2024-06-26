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
#include "BuildCommonScratch.hlsl"

#include "TrianglePrimitive.hlsl"

//=====================================================================================================================
GeometryArgs GetGeometryArgsFromConstants(
    uint geometryIndex,
    BuildShaderGeometryConstants geomConstants)
{
    GeometryArgs args = (GeometryArgs)0;

    args.metadataSizeInBytes            = ShaderConstants.header.metadataSizeInBytes;
    args.NumPrimitives                  = geomConstants.numPrimitives;
    args.LeafNodeDataByteOffset         = ShaderConstants.offsets.bvhLeafNodeData;
    args.PrimitiveOffset                = geomConstants.primitiveOffset;
    args.SceneBoundsByteOffset          = ShaderConstants.offsets.sceneBounds;
    args.PropagationFlagsScratchOffset  = ShaderConstants.offsets.propagationFlags;
    args.BaseUpdateStackScratchOffset   = ShaderConstants.offsets.updateStack;
    args.IndexBufferByteOffset          = geomConstants.indexBufferByteOffset;
    args.IndexBufferFormat              = geomConstants.indexBufferFormat;
    args.GeometryStride                 = geomConstants.geometryStride;
    args.GeometryIndex                  = geometryIndex;
    args.BaseGeometryInfoOffset         = ShaderConstants.header.offsets.geometryInfo;
    args.BasePrimNodePtrOffset          = ShaderConstants.header.offsets.primNodePtrs;
    args.GeometryFlags                  = geomConstants.geometryFlags;
    args.VertexComponentCount           = geomConstants.vertexComponentCount;
    args.VertexComponentSize            = geomConstants.vertexComponentSize;
    args.vertexCount                    = geomConstants.vertexCount;
    args.DestLeafByteOffset             = ShaderConstants.header.offsets.leafNodes;
    args.LeafNodeExpansionFactor        = ShaderConstants.leafNodeExpansionFactor;
    args.IndexBufferVaLo                = geomConstants.indexBufferGpuVaLo;
    args.IndexBufferVaHi                = geomConstants.indexBufferGpuVaHi;
    args.TransformBufferGpuVaLo         = geomConstants.transformBufferGpuVaLo;
    args.TransformBufferGpuVaHi         = geomConstants.transformBufferGpuVaHi;
    args.blockOffset                    = geomConstants.blockOffset;
    args.encodeTaskCounterScratchOffset = ShaderConstants.offsets.encodeTaskCounter;

    return args;
}

//=====================================================================================================================
GeometryArgs InitGeometryArgs(
    in uint geometryIndex)
{
    return GetGeometryArgsFromConstants(geometryIndex, GeometryConstants[geometryIndex]);
}

//=====================================================================================================================
GeometryArgs InitGeometryArgsNonUniform(
    in uint geometryIndex)
{
    return GetGeometryArgsFromConstants(geometryIndex, GeometryConstants[NonUniformResourceIndex(geometryIndex)]);
}

//======================================================================================================================
void WriteScratchTriangleNode(
    GeometryArgs        geometryArgs,
    uint                dstScratchNodeIdx,
    uint                primitiveIndex,
    uint                geometryIndex,
    uint                geometryFlags,
    uint                instanceMask,
    in BoundingBox      bbox,
    in TriangleData     tri)
{
    uint offset = CalcScratchNodeOffset(geometryArgs.LeafNodeDataByteOffset, dstScratchNodeIdx);

    uint4 data;

    // LeafNode.bbox_min_or_v0, primitiveIndex
    data = uint4(asuint(tri.v0), primitiveIndex);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_V0_OFFSET, data);

    // LeafNode.bbox_max_or_v1, geometryIndex
    data = uint4(asuint(tri.v1), geometryIndex);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_V1_OFFSET, data);

    // LeafNode.v2, parent
    data = uint4(asuint(tri.v2), INVALID_IDX);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_V2_OFFSET, data);

    // type, flags, splitBox, numPrimitivesAndDoCollapse
    uint flags = CalcTriangleBoxNodeFlags(geometryFlags);

    // Disable triangle splitting if geometry descriptor allows duplicate anyHit invocation
    if ((geometryFlags & D3D12_RAYTRACING_GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION) != 0)
    {
        flags |= SCRATCH_NODE_FLAGS_DISABLE_TRIANGLE_SPLIT_MASK;
    }

    const uint triangleId = CalcUncompressedTriangleId(geometryFlags);
    const uint packedFlags = PackScratchNodeFlags(instanceMask, flags, triangleId);

    data = uint4(INVALID_IDX, 0, 0, packedFlags);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET, data);
}

//======================================================================================================================
// NOTE: Used by Trivial Builder, so this function should not use SrcBuffer or ScratchBuffer.
void WriteGeometryInfoForBuildsAndCopies(
    GeometryArgs       geometryArgs,
    uint               primitiveOffset,
    uint               numPrimitives,
    uint               primitiveStride,
    uint               metadataSize,
    uint               baseGeometryInfoOffset)
{
    const uint geometryInfoOffset =
        metadataSize + baseGeometryInfoOffset +
        (geometryArgs.GeometryIndex * GEOMETRY_INFO_SIZE);

    GeometryInfo info;
    info.geometryBufferOffset = primitiveOffset * primitiveStride;
    info.primNodePtrsOffset   = primitiveOffset * sizeof(uint);
    info.geometryFlagsAndNumPrimitives =
        PackGeometryFlagsAndNumPrimitives(geometryArgs.GeometryFlags, numPrimitives);

    DstMetadata.Store<GeometryInfo>(geometryInfoOffset, info);
}

//======================================================================================================================
void WriteGeometryInfo(
    GeometryArgs geometryArgs,
    uint         primitiveOffset,
    uint         numPrimitives,
    uint         primitiveStride)
{
    // For builds and not-in-place updates, write the geometry info.
    if ((IsUpdate() == false) ||
        (IsUpdate() && (Settings.isUpdateInPlace == false)))
    {
        const uint metadataSize = IsUpdate() ?
            SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : geometryArgs.metadataSizeInBytes;

        // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
        const AccelStructOffsets offsets =
            SrcBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

        const uint baseGeometryInfoOffset =
            IsUpdate() ? offsets.geometryInfo : geometryArgs.BaseGeometryInfoOffset;

        WriteGeometryInfoForBuildsAndCopies(geometryArgs,
                                            primitiveOffset,
                                            numPrimitives,
                                            primitiveStride,
                                            metadataSize,
                                            baseGeometryInfoOffset);
    }
}

//=====================================================================================================================
void PushNodeForUpdate(
    uint                       metadataSize,
    uint                       baseUpdateStackScratchOffset,
    uint                       isUpdateInPlace,
    uint                       childNodePointer,
    uint                       parentNodePointer,
    uint                       instanceMask,
    in BoundingBox             boundingBox,
    bool                       writeNodesToUpdateStack)
{
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
    const uint initValue = Settings.enableFastLBVH ? 0xffffffffu : 0;

    if (isAABB)
    {
        const uint flagOffset = geometryArgs.PropagationFlagsScratchOffset + (flattenedPrimitiveIndex * sizeof(uint));
        ScratchBuffer.Store(flagOffset, initValue);
    }
    else // Triangle
    {
        const uint stride = geometryArgs.LeafNodeExpansionFactor * sizeof(uint);
        const uint flagOffset = geometryArgs.PropagationFlagsScratchOffset + (flattenedPrimitiveIndex * stride);
        for (uint i = 0; i < geometryArgs.LeafNodeExpansionFactor; ++i)
        {
            ScratchBuffer.Store(flagOffset + (i * sizeof(uint)), initValue);
        }
    }
}

//======================================================================================================================
void EncodeTriangleNode(
    RWBuffer<float3>           GeometryBuffer,
    GeometryArgs               geometryArgs,
    uint                       primitiveIndex,
    uint                       primitiveOffset,
    uint                       vertexOffsetInComponents,
    uint                       indexOffsetInBytes,
    uint                       transformOffsetInBytes,
    bool                       writeNodesToUpdateStack)
{
    const uint metadataSize = IsUpdate() ?
        SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : geometryArgs.metadataSizeInBytes;

    // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
    const AccelStructOffsets offsets =
        SrcBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

    const uint basePrimNodePtr =
        IsUpdate() ? offsets.primNodePtrs : geometryArgs.BasePrimNodePtrOffset;

    const uint flattenedPrimitiveIndex = primitiveOffset + primitiveIndex;
    const uint primNodePointerOffset =
        metadataSize + basePrimNodePtr + (flattenedPrimitiveIndex * sizeof(uint));

    // Fetch face indices from index buffer
    const IndexBufferInfo indexBufferInfo =
    {
        geometryArgs.IndexBufferVaLo,
        geometryArgs.IndexBufferVaHi,
        geometryArgs.IndexBufferByteOffset + indexOffsetInBytes,
        geometryArgs.IndexBufferFormat,
    };

    uint3 faceIndices = FetchFaceIndices(primitiveIndex, indexBufferInfo);

    // Check if vertex indices are within bounds, otherwise make the triangle inactive
    const uint maxIndex = max(faceIndices.x, max(faceIndices.y, faceIndices.z));
    if (maxIndex < geometryArgs.vertexCount)
    {
        const uint64_t transformBufferGpuVa =
            PackUint64(geometryArgs.TransformBufferGpuVaLo, geometryArgs.TransformBufferGpuVaHi);

        // Fetch triangle vertex data from vertex buffer
        TriangleData tri = FetchTransformedTriangleData(GeometryBuffer,
                                                        faceIndices,
                                                        geometryArgs.GeometryStride,
                                                        vertexOffsetInComponents,
                                                        geometryArgs.VertexComponentCount,
                                                        transformBufferGpuVa,
                                                        transformOffsetInBytes);

        uint nodePointer = INVALID_IDX;
        uint triangleId  = 0;

        // Generate triangle bounds and update scene bounding box
        BoundingBox boundingBox = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);

        if (IsUpdate())
        {
            nodePointer = SrcBuffer.Load(primNodePointerOffset);

            // If the primitive was active during the initial build, it will have a valid primitive node pointer.
            if (nodePointer != INVALID_IDX)
            {
                const uint nodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);
                const uint nodeType   = GetNodeType(nodePointer);

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

                if (Settings.isUpdateInPlace == false)
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

            if (Settings.isUpdateInPlace == false)
            {
                DstMetadata.Store(primNodePointerOffset, nodePointer);
            }

            // The shared bounding box for this pair of triangles will be updated by the thread handling triangle 0.
            const bool skipPairUpdatePush = (Settings.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
                                            (GetNodeType(nodePointer) == NODE_TYPE_TRIANGLE_1);

            if ((nodePointer != INVALID_IDX) && (skipPairUpdatePush == false))
            {
                // Fetch parent node pointer
                const uint parentNodePointer = ReadParentPointer(metadataSize, nodePointer);

                // Update out of place destination buffer
                if (Settings.isUpdateInPlace == false)
                {
                    WriteParentPointer(metadataSize, nodePointer, parentNodePointer);
                }

                // Handle two triangles sharing a bounding box when pair compression is enabled.
                uint childNodePointer = nodePointer;
                if ((Settings.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
                    (GetNodeType(nodePointer) == NODE_TYPE_TRIANGLE_0))
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

                        const IndexBufferInfo indexBufferInfo =
                        {
                            geometryArgs.IndexBufferVaLo,
                            geometryArgs.IndexBufferVaHi,
                            geometryArgs.IndexBufferByteOffset,
                            geometryArgs.IndexBufferFormat,
                        };

                        // Fetch face indices from index buffer.
                        const uint3 faceIndices = FetchFaceIndices(otherPrimIndex, indexBufferInfo);

                        // Check if vertex indices are within bounds.
                        const uint maxIndex = max(faceIndices.x, max(faceIndices.y, faceIndices.z));
                        if (maxIndex < geometryArgs.vertexCount)
                        {
                            const uint64_t transformBufferGpuVa =
                                PackUint64(geometryArgs.TransformBufferGpuVaLo, geometryArgs.TransformBufferGpuVaHi);

                            // Fetch triangle vertex data from vertex buffer.
                            const TriangleData tri = FetchTransformedTriangleData(GeometryBuffer,
                                                                                  faceIndices,
                                                                                  geometryArgs.GeometryStride,
                                                                                  vertexOffsetInComponents,
                                                                                  geometryArgs.VertexComponentCount,
                                                                                  transformBufferGpuVa,
                                                                                  transformOffsetInBytes);

                            const BoundingBox otherBox = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);

                            // Merge the bounding boxes of the two triangles.
                            boundingBox = CombineAABB(boundingBox, otherBox);
                        }
                    }
                }

                // Set the instance inclusion mask to 0 for degenerate triangles so that they are culled out.
                const uint instanceMask = (boundingBox.min.x > boundingBox.max.x) ? 0 : 0xff;

                PushNodeForUpdate(metadataSize,
                                  geometryArgs.BaseUpdateStackScratchOffset,
                                  Settings.isUpdateInPlace,
                                  childNodePointer,
                                  parentNodePointer,
                                  instanceMask,
                                  boundingBox,
                                  writeNodesToUpdateStack);
            }
        }
        else
        {
            const bool isActiveTriangle = IsActive(tri);
            if (isActiveTriangle)
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

            // Set the instance inclusion mask to 0 for degenerate triangles so that they are culled out.
            const uint instanceMask = (boundingBox.min.x > boundingBox.max.x) ? 0 : 0xff;

            // Triangle scratch nodes have a 1:1 mapping with global primitive index when pairing is disabled
            WriteScratchTriangleNode(geometryArgs,
                                     flattenedPrimitiveIndex,
                                     primitiveIndex,
                                     geometryArgs.GeometryIndex,
                                     geometryArgs.GeometryFlags,
                                     instanceMask,
                                     boundingBox,
                                     tri);

            {
                // Store invalid prim node pointer for now during first time builds.
                // If the triangle is active, EncodeHwBvh will write it in.
                DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
            }
        }
    }
    else
    {
        if (IsUpdate() == false)
        {
            // Deactivate primitive by setting bbox_min_or_v0.x to NaN
            WriteScratchNodeData(
                geometryArgs.LeafNodeDataByteOffset,
                flattenedPrimitiveIndex,
                0,
                NaN);

            {
                DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
            }
        }
        else if (Settings.isUpdateInPlace == false)
        {
            DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
        }
    }

    // ClearFlags for refit and update
    ClearFlagsForRefitAndUpdate(geometryArgs, flattenedPrimitiveIndex, false);
}

//=====================================================================================================================
// Fetch API bounding box from source buffer which is a typed R32G32 buffer.
BoundingBox FetchBoundingBoxData(RWBuffer<float3> buffer, uint index, uint offsetInElements, uint boxStrideInElements)
{
    const uint baseElementIndex = index * boxStrideInElements + offsetInElements;

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
        bbox = InvalidBoundingBox;
    }

    return bbox;
}

//======================================================================================================================
uint CalcProceduralBoxNodeFlags(
    in uint geometryFlags)
{
    // Determine opacity from geometry flags
    uint nodeFlags =
        (geometryFlags & D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE) ? 1u << BOX_NODE_FLAGS_ONLY_OPAQUE_SHIFT :
                                                                  1u << BOX_NODE_FLAGS_ONLY_NON_OPAQUE_SHIFT;

    // Note, a bottom-level acceleration structure can only contain a single geometry type.
    nodeFlags |= 1u << BOX_NODE_FLAGS_ONLY_PROCEDURAL_SHIFT;

    return nodeFlags;
}

//=====================================================================================================================
void WriteScratchProceduralNode(
    GeometryArgs        geometryArgs,
    uint                primitiveOffset,
    uint                primitiveIndex,
    uint                geometryIndex,
    uint                geometryFlags,
    uint                instanceMask,
    in BoundingBox      bbox)
{
    uint offset = CalcScratchNodeOffset(geometryArgs.LeafNodeDataByteOffset, primitiveOffset + primitiveIndex);

    uint4 data;

    // LeafNode.bbox_min_or_v0, primitiveIndex
    data = uint4(asuint(bbox.min), primitiveIndex);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_BBOX_MIN_OFFSET, data);

    // LeafNode.bbox_max_or_v1, geometryIndex
    data = uint4(asuint(bbox.max), geometryIndex);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_BBOX_MAX_OFFSET, data);

    // LeafNode.v2, parent
    data = uint4(0xffffffff, 0xffffffff, 0xffffffff, 0);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_V2_OFFSET, data);

    // type, flags, splitBox, numPrimitivesAndDoCollapse
    uint triangleId = 0;

    // Instance mask is assumed 0 in bottom level acceleration structures
    const uint flags = CalcProceduralBoxNodeFlags(geometryFlags);

    const uint packedFlags = PackScratchNodeFlags(instanceMask, flags, triangleId);

    data = uint4(INVALID_IDX, 0, 0, packedFlags);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET, data);
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
    GeometryArgs geometryArgs,
    uint         metadataSize,
    uint         nodePointer,
    uint         primitiveIndex)
{
    const uint nodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);

    {
        const uint geometryIndexAndFlags = PackGeometryIndexAndFlags(geometryArgs.GeometryIndex,
                                                                     geometryArgs.GeometryFlags);

        DstMetadata.Store(nodeOffset + USER_NODE_PROCEDURAL_PRIMITIVE_INDEX_OFFSET, primitiveIndex);
        DstMetadata.Store(nodeOffset + USER_NODE_PROCEDURAL_GEOMETRY_INDEX_AND_FLAGS_OFFSET, geometryIndexAndFlags);
    }
}

//======================================================================================================================
void EncodeAabbNode(
    RWBuffer<float3>           GeometryBuffer,
    GeometryArgs               geometryArgs,
    uint                       primitiveIndex,
    uint                       primitiveOffset,
    uint                       geometryOffsetInComponents,
    bool                       writeNodesToUpdateStack)
{
    const uint metadataSize =
        IsUpdate() ? SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : geometryArgs.metadataSizeInBytes;

    // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
    const AccelStructOffsets offsets =
        SrcBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

    const uint basePrimNodePtr =
        IsUpdate() ? offsets.primNodePtrs : geometryArgs.BasePrimNodePtrOffset;

    // Typed buffer view for AABBs uses two channels (X32Y32) - each element has 2 components.
    const uint geometryOffsetInElements = geometryOffsetInComponents / 2;
    // Get bounds for this thread
    const BoundingBox boundingBox = FetchBoundingBoxData(GeometryBuffer,
                                                         primitiveIndex,
                                                         geometryOffsetInElements,
                                                         geometryArgs.GeometryStride);

    // Set the instance inclusion mask to 0 for degenerate procedural nodes so that they are culled out.
    const uint instanceMask = (boundingBox.min.x > boundingBox.max.x) ? 0 : 0xff;

    const uint primNodePointerOffset =
        metadataSize + basePrimNodePtr + ((primitiveOffset + primitiveIndex) * sizeof(uint));

    if (IsUpdate())
    {
        const uint nodePointer = SrcBuffer.Load(primNodePointerOffset);

        // If the primitive was active during the initial build, it will have a valid primitive node pointer.
        if (nodePointer != INVALID_IDX)
        {
            WriteProceduralNodeBoundingBox(metadataSize, nodePointer, boundingBox);

            // Fetch parent node pointer
            const uint parentNodePointer = ReadParentPointer(metadataSize, nodePointer);

            if (Settings.isUpdateInPlace == false)
            {
                WriteProceduralNodePrimitiveData(geometryArgs,
                                                 metadataSize,
                                                 nodePointer,
                                                 primitiveIndex);

                WriteParentPointer(metadataSize, nodePointer, parentNodePointer);

                DstMetadata.Store(primNodePointerOffset, nodePointer);
            }

            PushNodeForUpdate(metadataSize,
                              geometryArgs.BaseUpdateStackScratchOffset,
                              Settings.isUpdateInPlace,
                              nodePointer,
                              parentNodePointer,
                              instanceMask,
                              boundingBox,
                              writeNodesToUpdateStack);
        }
        else if (Settings.isUpdateInPlace == false)
        {
            // For inactive primitives, just copy over the primitive node pointer.
            DstMetadata.Store(primNodePointerOffset, nodePointer);
        }
    }
    else
    {
        if (Settings.sceneBoundsCalculationType == SceneBoundsBasedOnGeometry)
        {
            UpdateSceneBounds(geometryArgs.SceneBoundsByteOffset, boundingBox);
        }
        else if (Settings.sceneBoundsCalculationType == SceneBoundsBasedOnGeometryWithSize)
        {
            UpdateSceneBoundsWithSize(geometryArgs.SceneBoundsByteOffset, boundingBox);
        }

        WriteScratchProceduralNode(geometryArgs,
                                   primitiveOffset,
                                   primitiveIndex,
                                   geometryArgs.GeometryIndex,
                                   geometryArgs.GeometryFlags,
                                   instanceMask,
                                   boundingBox);

        {
            // Store invalid prim node pointer for now during first time builds.
            // If the Procedural node is active, EncodeHwBvh will update it.
            DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
        }
    }

    // ClearFlags for refit and update
    const uint flattenedPrimitiveIndex = primitiveOffset + primitiveIndex;
    ClearFlagsForRefitAndUpdate(geometryArgs, flattenedPrimitiveIndex, true);
}
