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
#include "UpdateCommon.hlsl"

//======================================================================================================================
void WriteScratchTriangleNode(
    in uint         dstScratchNodeIdx,
    in uint         primitiveIndex,
    in uint         geometryIndex,
    in uint         geometryFlags,
    in uint         instanceMask,
    in BoundingBox  bbox,
    in TriangleData tri)
{
    uint offset = CalcScratchNodeOffset(ShaderConstants.offsets.bvhLeafNodeData, dstScratchNodeIdx);

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
    in BuildShaderGeometryConstants geomConstants,
    in NumPrimAndInputOffset        inputOffsets,
    in uint                         geometryIndex,
    in uint                         primitiveStride,
    in uint                         metadataSize,
    in uint                         baseGeometryInfoOffset)
{
    const uint geometryInfoOffset =
        metadataSize + baseGeometryInfoOffset +
        (geometryIndex * GEOMETRY_INFO_SIZE);

    GeometryInfo info;
    info.geometryBufferOffset = inputOffsets.primitiveOffset * primitiveStride;
    info.primNodePtrsOffset   = inputOffsets.primitiveOffset * sizeof(uint);
    info.geometryFlagsAndNumPrimitives =
        PackGeometryFlagsAndNumPrimitives(geomConstants.geometryFlags, inputOffsets.numPrimitives);

    DstMetadata.Store<GeometryInfo>(geometryInfoOffset, info);
}

//======================================================================================================================
void WriteGeometryInfo(
    in BuildShaderGeometryConstants geomConstants,
    in NumPrimAndInputOffset        inputOffsets,
    in uint                         geometryIndex,
    in uint                         primitiveStride)
{
    // For builds and not-in-place updates, write the geometry info.
    if ((IsUpdate() == false) ||
        (IsUpdate() && (Settings.isUpdateInPlace == false)))
    {
        const uint metadataSize = IsUpdate() ?
            SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : ShaderConstants.header.metadataSizeInBytes;

        // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
        const AccelStructOffsets offsets =
            SrcBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

        const uint baseGeometryInfoOffset =
            IsUpdate() ? offsets.geometryInfo : ShaderConstants.header.offsets.geometryInfo;

        WriteGeometryInfoForBuildsAndCopies(geomConstants,
                                            inputOffsets,
                                            geometryIndex,
                                            primitiveStride,
                                            metadataSize,
                                            baseGeometryInfoOffset);
    }
}

//======================================================================================================================
void EncodeTriangleNode(
    BuildShaderGeometryConstants geomConstants,
    NumPrimAndInputOffset        inputOffsets,
    uint                         geometryIndex,
    uint                         primitiveIndex,
    bool                         writeNodesToUpdateStack)
{
    const uint metadataSize = IsUpdate() ?
        SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : ShaderConstants.header.metadataSizeInBytes;

    // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
    const AccelStructOffsets offsets =
        SrcBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

    const uint basePrimNodePtr =
        IsUpdate() ? offsets.primNodePtrs : ShaderConstants.header.offsets.primNodePtrs;

    const uint flattenedPrimitiveIndex = inputOffsets.primitiveOffset + primitiveIndex;
    const uint primNodePointerOffset =
        metadataSize + basePrimNodePtr + (flattenedPrimitiveIndex * sizeof(uint));

    TriangleData tri = (TriangleData)0;
    uint3 indices = uint3(0, 0, 0);
    const bool validIndices =
        FetchTrianglePrimitive(
            geomConstants,
            inputOffsets,
            GeometryBuffer[geometryIndex],
            geometryIndex,
            primitiveIndex,
            tri,
            indices);
    if (validIndices)
    {
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
                    triangleId = CalcUncompressedTriangleId(geomConstants.geometryFlags);
                    vertexOffsets = CalcTriangleVertexOffsets(nodeType);
                }

                DstMetadata.Store3(nodeOffset + vertexOffsets.x, asuint(tri.v0));
                DstMetadata.Store3(nodeOffset + vertexOffsets.y, asuint(tri.v1));
                DstMetadata.Store3(nodeOffset + vertexOffsets.z, asuint(tri.v2));

                if (Settings.isUpdateInPlace == false)
                {
                    const uint geometryIndexAndFlags = PackGeometryIndexAndFlags(geometryIndex,
                                                                                 geomConstants.geometryFlags);
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

                        TriangleData otherTri = (TriangleData)0;
                        uint3 otherIndices = uint3(0, 0, 0);
                        const bool validIndices =
                            FetchTrianglePrimitive(
                                geomConstants,
                                inputOffsets,
                                GeometryBuffer[geometryIndex],
                                geometryIndex,
                                otherPrimIndex,
                                otherTri,
                                otherIndices);
                        if (validIndices)
                        {
                            const BoundingBox otherBox = GenerateTriangleBoundingBox(otherTri.v0, otherTri.v1, otherTri.v2);

                            // Merge the bounding boxes of the two triangles.
                            boundingBox = CombineAABB(boundingBox, otherBox);
                        }
                    }
                }

                // Set the instance inclusion mask to 0 for degenerate triangles so that they are culled out.
                const uint instanceMask = (boundingBox.min.x > boundingBox.max.x) ? 0 : 0xff;

                PushNodeForUpdate(metadataSize,
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
                    UpdateSceneBounds(ShaderConstants.offsets.sceneBounds, boundingBox);
                }
                else if (Settings.sceneBoundsCalculationType == SceneBoundsBasedOnGeometryWithSize)
                {
                    // TODO: with tri splitting, need to not update "size" here
                    UpdateSceneBoundsWithSize(ShaderConstants.offsets.sceneBounds, boundingBox);
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
            WriteScratchTriangleNode(flattenedPrimitiveIndex,
                                     primitiveIndex,
                                     geometryIndex,
                                     geomConstants.geometryFlags,
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
                ShaderConstants.offsets.bvhLeafNodeData,
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
}

//=====================================================================================================================
// Fetch API bounding box from source buffer which is a typed R32G32 buffer.
template<typename Float3Buffer>
BoundingBox FetchBoundingBoxData(
    Float3Buffer buffer,
    uint         index,
    uint         offsetInElements,
    uint         boxStrideInElements)
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
    uint                dstScratchNodeIdx,
    uint                primitiveIndex,
    uint                geometryIndex,
    uint                geometryFlags,
    uint                instanceMask,
    in BoundingBox      bbox)
{
    uint offset = CalcScratchNodeOffset(ShaderConstants.offsets.bvhLeafNodeData, dstScratchNodeIdx);

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
    uint         geometryFlags,
    uint         metadataSize,
    uint         nodePointer,
    uint         geometryIndex,
    uint         primitiveIndex)
{
    const uint nodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);

    {
        const uint geometryIndexAndFlags = PackGeometryIndexAndFlags(geometryIndex, geometryFlags);

        DstMetadata.Store(nodeOffset + USER_NODE_PROCEDURAL_PRIMITIVE_INDEX_OFFSET, primitiveIndex);
        DstMetadata.Store(nodeOffset + USER_NODE_PROCEDURAL_GEOMETRY_INDEX_AND_FLAGS_OFFSET, geometryIndexAndFlags);
    }
}

//======================================================================================================================
void EncodeAabbNode(
    BuildShaderGeometryConstants geomConstants,
    NumPrimAndInputOffset        inputOffsets,
    uint                         geometryIndex,
    uint                         primitiveIndex,
    bool                         writeNodesToUpdateStack)
{
    const uint metadataSize =
        IsUpdate() ? SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : ShaderConstants.header.metadataSizeInBytes;

    // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
    const AccelStructOffsets offsets =
        SrcBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

    const uint basePrimNodePtr =
        IsUpdate() ? offsets.primNodePtrs : ShaderConstants.header.offsets.primNodePtrs;

    // Typed buffer view for AABBs uses two channels (X32Y32) - each element has 2 components.
    const uint geometryOffsetInElements = inputOffsets.vertexOffsetInComponents / 2;
    // Get bounds for this thread
    const BoundingBox boundingBox = FetchBoundingBoxData(GeometryBuffer[geometryIndex],
                                                         primitiveIndex,
                                                         geometryOffsetInElements,
                                                         geomConstants.geometryStride);

    // Set the instance inclusion mask to 0 for degenerate procedural nodes so that they are culled out.
    const uint instanceMask = (boundingBox.min.x > boundingBox.max.x) ? 0 : 0xff;

    const uint flattenedPrimitiveIndex = inputOffsets.primitiveOffset + primitiveIndex;
    const uint primNodePointerOffset =
        metadataSize + basePrimNodePtr + (flattenedPrimitiveIndex * sizeof(uint));

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
                WriteProceduralNodePrimitiveData(geomConstants.geometryFlags,
                                                 metadataSize,
                                                 nodePointer,
                                                 geometryIndex,
                                                 primitiveIndex);

                WriteParentPointer(metadataSize, nodePointer, parentNodePointer);

                DstMetadata.Store(primNodePointerOffset, nodePointer);
            }

            PushNodeForUpdate(metadataSize,
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
            UpdateSceneBounds(ShaderConstants.offsets.sceneBounds, boundingBox);
        }
        else if (Settings.sceneBoundsCalculationType == SceneBoundsBasedOnGeometryWithSize)
        {
            UpdateSceneBoundsWithSize(ShaderConstants.offsets.sceneBounds, boundingBox);
        }

        WriteScratchProceduralNode(flattenedPrimitiveIndex,
                                   primitiveIndex,
                                   geometryIndex,
                                   geomConstants.geometryFlags,
                                   instanceMask,
                                   boundingBox);

        {
            // Store invalid prim node pointer for now during first time builds.
            // If the Procedural node is active, EncodeHwBvh will update it.
            DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
        }
    }
}
