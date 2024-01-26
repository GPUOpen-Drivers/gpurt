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
void WriteScratchCompressedTriangleNode(
    GeometryArgs        geometryArgs,
    uint                primitiveIndex,
    in TriangleData     tri,
    uint                triangleId,
    uint                nodeType,
    uint                pairedTriPrimIdx)
{
    uint primitiveOffset = geometryArgs.PrimitiveOffset;
    uint geometryIndex   = geometryArgs.GeometryIndex;
    uint flags           = geometryArgs.GeometryFlags;

    uint offset = CalcScratchNodeOffset(geometryArgs.LeafNodeDataByteOffset, primitiveOffset + primitiveIndex);

    uint4 data;

    // LeafNode.bbox_min_or_v0, primitiveIndex (globalIndex)
    data = uint4(asuint(tri.v0), primitiveIndex);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_V0_OFFSET, data);

    // LeafNode.bbox_max_or_v1, geometryIndex
    data = uint4(asuint(tri.v1), geometryIndex);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_V1_OFFSET, data);

    // LeafNode.v2, parent
    data = uint4(asuint(tri.v2), INVALID_IDX);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_V2_OFFSET, data);

    const BoundingBox box = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);
    const float cost = SAH_COST_TRIANGLE_INTERSECTION * ComputeBoxSurfaceArea(box);

    // type, flags, splitBox_or_pairedNodeIdx, numPrimitivesAndDoCollapse
    uint pairedId = pairedTriPrimIdx;
    if ((pairedTriPrimIdx != INVALID_IDX) &&
        (pairedTriPrimIdx != PAIRED_TRI_LINKONLY))
    {
        // need to log the "flattened" globalIdx here
        // (not just the primitiveIndex within geometry)
        //
        // this will be used by BVH builder to access the paired node from ScratchBuffer
        pairedId = pairedTriPrimIdx + primitiveOffset;
    }

    // Set the instance inclusion mask to 0 for degenerate triangles so that they are culled out.
    const uint instanceMask = (box.min.x > box.max.x) ? 0 : 0xff;
    const uint packedFlags = PackScratchNodeFlags(instanceMask, flags, triangleId);

    data = uint4(packedFlags, pairedId, asuint(cost), nodeType);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_FLAGS_OFFSET, data);
}

//=====================================================================================================================
void WriteScratchInactiveTriangleNode(
    GeometryArgs        geometryArgs,
    uint                flattenedPrimitiveIndex, // primitiveOffset + primitiveIndex,
    uint                primNodePointerOffset)
{
    if (IsUpdate() == false)
    {
        WriteScratchNodeData(geometryArgs.LeafNodeDataByteOffset,
                             flattenedPrimitiveIndex,
                             0,
                             NaN);

        DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
    }
    else if (Settings.isUpdateInPlace == false)
    {
        DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
    }
}

//=====================================================================================================================
void WriteOutPairedTriangles(
    GeometryArgs geometryArgs,
    TriangleData tri1,
    uint         tri1PrimIdx,
    uint         triT1Rotation,
    TriangleData tri0,
    uint         tri0PrimIdx,
    uint         triT0Rotation)
{
    uint triangleId = 0;

    // triT0 - NODE_TYPE_TRIANGLE_0 (2nd to intersect)
    triangleId = WriteTriangleIdField(triangleId, NODE_TYPE_TRIANGLE_0, triT0Rotation, geometryArgs.GeometryFlags);

    // triT1 - NODE_TYPE_TRIANGLE_1 (1st to intersect)
    triangleId = WriteTriangleIdField(triangleId, NODE_TYPE_TRIANGLE_1, triT1Rotation, geometryArgs.GeometryFlags);

    WriteScratchCompressedTriangleNode(geometryArgs,
                                       tri1PrimIdx,
                                       tri1,
                                       triangleId,
                                       NODE_TYPE_TRIANGLE_1,
                                       tri0PrimIdx);

    WriteScratchCompressedTriangleNode(geometryArgs,
                                       tri0PrimIdx,
                                       tri0,
                                       triangleId,
                                       NODE_TYPE_TRIANGLE_0,
                                       PAIRED_TRI_LINKONLY);

}

//=====================================================================================================================
// Note, vulkan shader compilation path does not support HLSL 2021. Hence, we cannot use templated functions here.
uint TryPairTriangles(
    const float3x3 t0,
    const float3x3 t1)
{
    // Packed triangle vertex offsets
    //
    // 0:3 : triangle_0_vertex_offset
    // 4:7 : triangle_1_vertex_offset
    //
    uint packedOffset = -1;

    if (all_equal(t1[2], t0[0]) && all_equal(t1[1], t0[1]))
    {
        packedOffset = 0x11;
    }

    if (all_equal(t1[1], t0[0]) && all_equal(t1[0], t0[1]))
    {
        packedOffset = 0x21;
    }

    if (all_equal(t1[0], t0[0]) && all_equal(t1[2], t0[1]))
    {
        packedOffset = 0x01;
    }

    if (all_equal(t1[2], t0[1]) && all_equal(t1[1], t0[2]))
    {
        packedOffset = 0x10;
    }

    if (all_equal(t1[1], t0[1]) && all_equal(t1[0], t0[2]))
    {
        packedOffset = 0x20;
    }

    if (all_equal(t1[0], t0[1]) && all_equal(t1[2], t0[2]))
    {
        packedOffset = 0x00;
    }

    if (all_equal(t1[2], t0[2]) && all_equal(t1[1], t0[0]))
    {
        packedOffset = 0x12;
    }

    if (all_equal(t1[1], t0[2]) && all_equal(t1[0], t0[0]))
    {
        packedOffset = 0x22;
    }

    if (all_equal(t1[0], t0[2]) && all_equal(t1[2], t0[0]))
    {
        packedOffset = 0x02;
    }

    return packedOffset;
}

//======================================================================================================================
uint PairTriangles(
    float3x3 tri,
    bool     isActive)
{
    bool valid = isActive;

    // Initialise to unpaired triangle
    uint pairInfo = -1;

    while (valid)
    {
        const bool isBroadcastLane = WaveIsFirstLane();
        if (isBroadcastLane)
        {
            valid = false;
        }

        const float3x3 broadcastTriangle = WaveReadLaneFirst(tri);

        uint packedOffset = -1;
        if (valid)
        {
            packedOffset = TryPairTriangles(broadcastTriangle, tri);
        }

        const uint firstPairedLane = firstbitlow(WaveActiveBallot64((packedOffset != -1)));

        if (firstPairedLane < WaveGetLaneCount())
        {
            if (WaveGetLaneIndex() == firstPairedLane)
            {
                valid = false;

                // Mark as triangle 1 in the quad
                pairInfo = -2;
            }

            packedOffset = WaveReadLaneAt(packedOffset, firstPairedLane);

            if (isBroadcastLane)
            {
                // Mark as triangle 0 in the quad. firstPairedLane denotes lane index with triangle 1
                pairInfo = (firstPairedLane << 16) | packedOffset;
            }
        }
    }

    return pairInfo;
}

//======================================================================================================================
uint TryPairTrianglesIndexed(
    const uint3 t0,
    const uint3 t1)
{
    // Packed triangle vertex offsets
    //
    // 0:3 : triangle_0_vertex_offset
    // 4:7 : triangle_1_vertex_offset
    //
    uint packedOffset = -1;

    if ((t1[2] == t0[0]) && (t1[1] == t0[1]))
    {
        packedOffset = 0x11;
    }

    if ((t1[1] == t0[0]) && (t1[0] == t0[1]))
    {
        packedOffset = 0x21;
    }

    if ((t1[0] == t0[0]) && (t1[2] == t0[1]))
    {
        packedOffset = 0x01;
    }

    if ((t1[2] == t0[1]) && (t1[1] == t0[2]))
    {
        packedOffset = 0x10;
    }

    if ((t1[1] == t0[1]) && (t1[0] == t0[2]))
    {
        packedOffset = 0x20;
    }

    if ((t1[0] == t0[1]) && (t1[2] == t0[2]))
    {
        packedOffset = 0x00;
    }

    if ((t1[2] == t0[2]) && (t1[1] == t0[0]))
    {
        packedOffset = 0x12;
    }

    if ((t1[1] == t0[2]) && (t1[0] == t0[0]))
    {
        packedOffset = 0x22;
    }

    if ((t1[0] == t0[2]) && (t1[2] == t0[0]))
    {
        packedOffset = 0x02;
    }

    return packedOffset;
}

//======================================================================================================================
uint PairTrianglesIndexed(
    uint3 tri,
    bool  isActive)
{
    bool valid = isActive;

    // Initialise to unpaired triangle
    uint pairInfo = -1;

    while (valid)
    {
        const bool isBroadcastLane = WaveIsFirstLane();
        if (isBroadcastLane)
        {
            valid = false;
        }

        const uint3 broadcastTriangle = WaveReadLaneFirst(tri);

        uint packedOffset = -1;
        if (valid)
        {
            packedOffset = TryPairTrianglesIndexed(broadcastTriangle, tri);
        }

        const uint firstPairedLane = firstbitlow(WaveActiveBallot64((packedOffset != -1)));

        if (firstPairedLane < WaveGetLaneCount())
        {
            if (WaveGetLaneIndex() == firstPairedLane)
            {
                valid = false;

                // Mark as triangle 1 in the quad
                pairInfo = -2;
            }

            packedOffset = WaveReadLaneAt(packedOffset, firstPairedLane);

            if (isBroadcastLane)
            {
                // Mark as triangle 0 in the quad. firstPairedLane denotes lane index with triangle 1
                pairInfo = (firstPairedLane << 16) | packedOffset;
            }
        }
    }

    return pairInfo;
}

//======================================================================================================================
void EncodePairedTriangleNodeImpl(
    RWBuffer<float3>           GeometryBuffer,
    RWByteAddressBuffer        IndexBuffer,
    RWStructuredBuffer<float4> TransformBuffer,
    GeometryArgs               geometryArgs,
    uint                       primitiveIndex,
    uint                       localId,
    uint                       primitiveOffset,
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

    const uint flattenedPrimitiveIndex = primitiveOffset + primitiveIndex;

    const uint primNodePointerOffset =
        metadataSize + basePrimNodePtr + (flattenedPrimitiveIndex * sizeof(uint));

    const uint numLeafNodesOffset = metadataSize + ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET;

    // Fetch face indices from index buffer
    uint3 faceIndices = FetchFaceIndices(IndexBuffer,
                                         primitiveIndex,
                                         geometryArgs.IndexBufferByteOffset,
                                         geometryArgs.IndexBufferFormat);

    const bool isIndexed = (geometryArgs.IndexBufferFormat != IndexFormatInvalid);

    uint numQuads = 0;

    // Check if vertex indices are within bounds, otherwise make the triangle inactive
    const uint maxIndex = max(faceIndices.x, max(faceIndices.y, faceIndices.z));
    if (maxIndex < geometryArgs.vertexCount)
    {
        // Fetch triangle vertex data from vertex buffer
        TriangleData tri = FetchTransformedTriangleData(geometryArgs,
                                                        GeometryBuffer,
                                                        faceIndices,
                                                        geometryArgs.GeometryStride,
                                                        vertexOffset,
                                                        geometryArgs.HasValidTransform,
                                                        TransformBuffer);

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

        uint pairInfo = 0;

        if (isIndexed)
        {
            pairInfo = PairTrianglesIndexed(faceIndices, true);
        }
        else
        {
            float3x3 faceVertices;
            faceVertices[0] = tri.v0;
            faceVertices[1] = tri.v1;
            faceVertices[2] = tri.v2;

            pairInfo = PairTriangles(faceVertices, isActive);
        }

        // Count quads produced by the current wave. Note, this includes unpaired triangles as well
        // (marked with a value of -1).
        numQuads = WaveActiveCountBits(int(pairInfo) >= -1);

        // TODO: Allocate quads in contiguous memory in the scratch buffer
        // const uint quadIdx = WavePrefixSum(hasValidQuad ? 1 : 0);

        const bool hasValidQuad = (int(pairInfo) >= 0);
        const uint pairLaneId = hasValidQuad ? pairInfo >> 16 : 0;

        TriangleData tri1;
        tri1.v0 = WaveReadLaneAt(tri.v0, pairLaneId);
        tri1.v1 = WaveReadLaneAt(tri.v1, pairLaneId);
        tri1.v2 = WaveReadLaneAt(tri.v2, pairLaneId);

        const uint primitiveIndex1 = WaveReadLaneAt(primitiveIndex, pairLaneId);

        if (hasValidQuad)
        {
            const uint triT0Rotation = (pairInfo & 0xF);
            const uint triT1Rotation = (pairInfo >> 4) & 0xF;

            WriteOutPairedTriangles(geometryArgs,
                                    tri1,
                                    primitiveIndex1,
                                    triT1Rotation,
                                    tri,
                                    primitiveIndex,
                                    triT0Rotation);
        }
        else if (pairInfo == -1)
        {
            // Write out unpaired triangle
            const uint triangleId = WriteTriangleIdField(0, NODE_TYPE_TRIANGLE_0, 0, geometryArgs.GeometryFlags);
            WriteScratchCompressedTriangleNode(geometryArgs,
                                               primitiveIndex,
                                               tri,
                                               triangleId,
                                               NODE_TYPE_TRIANGLE_0,
                                               INVALID_IDX);
        }
        else
        {
            // This triangle is a pair in a quad handled by the lead lane. Nothing to do here.
        }

        // Store invalid prim node pointer for now during first time builds.
        // If the triangle is active, BuildQBVH will write it in.
        DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
    }
    else
    {
        // Immediately write out the inactive nodes to scratch
        WriteScratchInactiveTriangleNode(geometryArgs,
                                         flattenedPrimitiveIndex,
                                         primNodePointerOffset);
    }

    if (WaveIsFirstLane())
    {
        DstMetadata.InterlockedAdd(numLeafNodesOffset, numQuads);
    }

    // ClearFlags for refit and update
    ClearFlagsForRefitAndUpdate(geometryArgs, flattenedPrimitiveIndex, false);
}
