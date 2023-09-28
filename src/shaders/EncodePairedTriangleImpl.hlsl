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
// 3 uint for faceIndices, 9 uint for vertex (3 uint3 vertex),  1 uint for primitiveIdx
#define TRIANGLE_CACHE_STRIDE       16
#define FACE_DATA_LDS_OFFSET        0
#define V0_DATA_LDS_OFFSET          3
#define V1_DATA_LDS_OFFSET          6
#define V2_DATA_LDS_OFFSET          9
#define PRIMITIVE_IDX_LDS_OFFSET    12

//=====================================================================================================================
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

    uint offset = (primitiveIndex * ByteStrideScratchNode) +
        (primitiveOffset * ByteStrideScratchNode) +
        geometryArgs.LeafNodeDataByteOffset;

    uint4 data;

    // LeafNode.bbox_min_or_v0, primitiveIndex (globalIndex)
    data = uint4(asuint(tri.v0), primitiveIndex);
    ScratchBuffer.Store4(offset + SCRATCH_NODE_V0_OFFSET, data);

    // LeafNode.bbox_max_or_v1, geometryIndex
    data = uint4(asuint(tri.v1), geometryIndex);
    ScratchBuffer.Store4(offset + SCRATCH_NODE_V1_OFFSET, data);

    // LeafNode.v2, parent
    data = uint4(asuint(tri.v2), INVALID_IDX);
    ScratchBuffer.Store4(offset + SCRATCH_NODE_V2_OFFSET, data);

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
    const uint packedFlags = PackInstanceMaskAndNodeFlags(instanceMask, flags);

    const uint triangleTypeAndId = (triangleId << 3) | nodeType;
    data = uint4(triangleTypeAndId, packedFlags, pairedId, asuint(cost));
    ScratchBuffer.Store4(offset + SCRATCH_NODE_TYPE_OFFSET, data);
}

//=====================================================================================================================
void WriteScratchInactiveTriangleNode(
    GeometryArgs        geometryArgs,
    uint                flattenedPrimitiveIndex, //geometryBasePrimOffset + primitiveIndex,
    uint                primNodePointerOffset)
{
    if (IsUpdate() == false)
    {
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

//=====================================================================================================================
bool CompareVertices(
    GeometryArgs geometryArgs,
    const uint   faceIndex0,
    const uint   faceIndex1,
    const float3 vertex0,
    const float3 vertex1)
{
    bool ret = false;

    if (faceIndex0 == faceIndex1)
    {
        ret = true;
    }
    else if ((geometryArgs.BuildFlags & DDI_BUILD_FLAG_ALLOW_UPDATE) ||
             (geometryArgs.BuildFlags & DDI_BUILD_FLAG_PERFORM_UPDATE) ||
             (geometryArgs.BuildFlags & DDI_BUILD_FLAG_PREFER_FAST_BUILD))
    {
        // If we decide to not rerun triangle compression on BVH Updates, we can't assume two vertices
        // with the same position but different vertex indices are the same.
        //
        // Also, speedup triangle compression when FAST_BUILD is used by not comparing vertices of different indices.
        ret = false;
    }
    else if (all(vertex0 == vertex1))
    {
        // Compress two vertices that have different indices but the same positions together.
        ret = true;
    }

    return ret;
}

//=====================================================================================================================
void StoreInactiveTriangleNodeInLDS(
    uint         localId,
    uint         primitiveIdx)
{
    uint startingOffset = localId * TRIANGLE_CACHE_STRIDE;
    // mark face data as inactive_prim
    SharedMem[startingOffset + FACE_DATA_LDS_OFFSET] = INACTIVE_PRIM;

    // need to log primitiveIdx, so as to write the correct data in scratch
    SharedMem[startingOffset + PRIMITIVE_IDX_LDS_OFFSET] = primitiveIdx;
}

//=====================================================================================================================
void StoreTriangleNodeInLDS(
    uint         localId,
    TriangleData tri,
    uint3        faceIndices,
    uint         primitiveIdx)
{
    uint startingOffset = localId * TRIANGLE_CACHE_STRIDE;

    uint faceOffset = startingOffset + FACE_DATA_LDS_OFFSET;
    SharedMem[faceOffset] = faceIndices.x;
    SharedMem[faceOffset + 1] = faceIndices.y;
    SharedMem[faceOffset + 2] = faceIndices.z;

    uint v0Offset = startingOffset + V0_DATA_LDS_OFFSET;
    SharedMem[v0Offset] = asuint(tri.v0.x);
    SharedMem[v0Offset + 1] = asuint(tri.v0.y);
    SharedMem[v0Offset + 2] = asuint(tri.v0.z);

    uint v1Offset = startingOffset + V1_DATA_LDS_OFFSET;
    SharedMem[v1Offset] = asuint(tri.v1.x);
    SharedMem[v1Offset + 1] = asuint(tri.v1.y);
    SharedMem[v1Offset + 2] = asuint(tri.v1.z);

    uint v2Offset = startingOffset + V2_DATA_LDS_OFFSET;
    SharedMem[v2Offset] = asuint(tri.v2.x);
    SharedMem[v2Offset + 1] = asuint(tri.v2.y);
    SharedMem[v2Offset + 2] = asuint(tri.v2.z);

    SharedMem[startingOffset + PRIMITIVE_IDX_LDS_OFFSET] = primitiveIdx;
}

//=====================================================================================================================
void ReadTriangleNodeFromLDS(
    in  uint    localId,
    out uint3   faceIndices,
    out uint3   vertex0,
    out uint3   vertex1,
    out uint3   vertex2,
    out uint    primitiveIdx)
{
    uint startingOffset = localId * TRIANGLE_CACHE_STRIDE;

    uint faceOffset = startingOffset + FACE_DATA_LDS_OFFSET;
    faceIndices.x = SharedMem[faceOffset];
    faceIndices.y = SharedMem[faceOffset + 1];
    faceIndices.z = SharedMem[faceOffset + 2];

    uint v0Offset = startingOffset + V0_DATA_LDS_OFFSET;
    vertex0.x = SharedMem[v0Offset];
    vertex0.y = SharedMem[v0Offset + 1];
    vertex0.z = SharedMem[v0Offset + 2];

    uint v1Offset = startingOffset + V1_DATA_LDS_OFFSET;
    vertex1.x = SharedMem[v1Offset];
    vertex1.y = SharedMem[v1Offset + 1];
    vertex1.z = SharedMem[v1Offset + 2];

    uint v2Offset = startingOffset + V2_DATA_LDS_OFFSET;
    vertex2.x = SharedMem[v2Offset];
    vertex2.y = SharedMem[v2Offset + 1];
    vertex2.z = SharedMem[v2Offset + 2];

    primitiveIdx = SharedMem[startingOffset + PRIMITIVE_IDX_LDS_OFFSET];
}

//=====================================================================================================================
void WriteOutUnPairedNodes(
    GeometryArgs    geometryArgs,
    uint            batchStartIndex,
    uint            batchEndIndex,
    uint            primNodePointerOffset,
    uint64_t        pairFoundBitMask64)
{
    // now write out triangle nodes (with compressed info)
    const uint numFaces = 2;
    uint geometryFlags = geometryArgs.GeometryFlags;

    for (uint i = batchStartIndex; i < batchEndIndex; i++)
    {
        uint triT1PrimitiveIdx = i;
        bool bitSet = ((pairFoundBitMask64 & (1ULL << i)) > 0);
        if (bitSet)
        {
            // The "paired triangle" has already been handled in "WriteOutPairedTriangles"
            // skip writing out this triangle
            continue;
        }

        uint3 faceIndices;
        uint3 v0;
        uint3 v1;
        uint3 v2;
        uint triPrimIdx;

        TriangleData tri;

        ReadTriangleNodeFromLDS(triT1PrimitiveIdx, faceIndices, v0, v1, v2, triPrimIdx);
        tri.v0 = asfloat(v0);
        tri.v1 = asfloat(v1);
        tri.v2 = asfloat(v2);

        if (faceIndices.x == INACTIVE_PRIM)
        {
           continue;
        }

        // this is an unpaired triangle
        // just write out the single triangle as NODE_TYPE_0 now
        uint triangleId = WriteTriangleIdField(0, NODE_TYPE_TRIANGLE_0, 0, geometryFlags);
        WriteScratchCompressedTriangleNode(geometryArgs,
                                           triPrimIdx,
                                           tri,
                                           triangleId,
                                           NODE_TYPE_TRIANGLE_0,
                                           INVALID_IDX);
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
// Comparing tri with other tris in the batch
void CompressTriangleNodes(
    in GeometryArgs geometryArgs,
    in uint         primitiveIdx,
    in uint         localId,
    in uint         numLeafNodesOffset,
    in uint         primitiveOffset,
    in uint         primNodePointerOffset)
{
    uint batchStartIndex = localId;
    const uint numIndices = 3;
    uint leafCount = 0; // leaf count per thread

    uint batchEndIndex = batchStartIndex + geometryArgs.trianglePairingSearchRadius;

    // adjust batchEndIndex if this BATCH is not a complete BATCH
    if ((primitiveIdx + geometryArgs.trianglePairingSearchRadius) >= geometryArgs.NumPrimitives)
    {
        batchEndIndex = batchEndIndex - ((primitiveIdx + geometryArgs.trianglePairingSearchRadius) - geometryArgs.NumPrimitives);
    }

    uint quadCount = 0;
    uint bestQuad = 0;
    uint triT1PrimIdx = 0;
    uint triT0PrimIdx = 0;
    uint64_t cachedQuadBitMask64 = 0;
    uint64_t pairFoundBitMask64  = 0;

    // Could either start from the beginning of the list, or the end of the list here
    for (uint triT1BatchIdxRange = batchEndIndex; triT1BatchIdxRange > batchStartIndex; triT1BatchIdxRange--)
    {
        uint triT1LocalIdx = triT1BatchIdxRange - 1;
        bestQuad = quadCount;

        uint3 faceIndices;
        uint3 v0;
        uint3 v1;
        uint3 v2;

        ReadTriangleNodeFromLDS(triT1LocalIdx, faceIndices, v0, v1, v2, triT1PrimIdx);

        if (faceIndices.x == INACTIVE_PRIM)
        {
            continue;
        }

        float3 triT1Vertex[3] = { asfloat(v0), asfloat(v1), asfloat(v2) };
        uint triT1FaceInd[numIndices] = { faceIndices.x, faceIndices.y, faceIndices.z };

        TriangleData tri1;
        tri1.v0 = asfloat(v0);
        tri1.v1 = asfloat(v1);
        tri1.v2 = asfloat(v2);

        uint triT0VtxOffset = 0;
        uint triT1VtxOffset = 0;
        uint triT0LocalIdx = 0;
        TriangleData tri0;

        uint64_t cachedQuadBitMaskCopy = cachedQuadBitMask64;

        for (uint quadIdx = 0; quadIdx < quadCount; ++quadIdx)
        {
            // When comparing triangle T1 with any cached Trianges,
            // first fetch triangle T0 from "cachedQuadBitMask"
            // by the hightset "set" bit
            // then comparing with the incoming triangle (triT1)
            //
            // -- If T1 found match T0, logging the bits per triT1 / triT0 localId (laneId)
            //  in pairFoundBitMask64
            // -- If triangle (trT1) cannot be paired, set the bit in "CachedQuadBitMask"
            // per T1's localId (laneId)

            // fetch local index from the "highest set" bit of "cachedQuadBitMaskCopy"
            triT0LocalIdx = firstbithigh(cachedQuadBitMaskCopy);
            // clear out the mostHigh bit, so next loop can fetch the "next Highest" bit
            cachedQuadBitMaskCopy &= ~(1ULL << triT0LocalIdx);
            bool pairBitSet = ((pairFoundBitMask64 & (1ULL << triT0LocalIdx)) > 0);
            if (pairBitSet)
            {
                // this prim has already been paired, move on to the next
                continue;
            }

            ReadTriangleNodeFromLDS(triT0LocalIdx, faceIndices, v0, v1, v2, triT0PrimIdx);
            float3 triT0Vertex[3] = { asfloat(v0), asfloat(v1), asfloat(v2) };
            uint triT0FaceInd[numIndices] = { faceIndices.x, faceIndices.y, faceIndices.z };

            tri0.v0 = asfloat(v0);
            tri0.v1 = asfloat(v1);
            tri0.v2 = asfloat(v2);

            for (int i = 0; i < numIndices; i++)
            {
                const uint indexOffset0 = i % numIndices;
                const uint indexOffset1 = (i + 1) % numIndices;

                const uint index0 = triT0FaceInd[indexOffset0];
                const uint index1 = triT0FaceInd[indexOffset1];

                // Compare all three matching vertex conditions with triangle 0
                uint4 data;
                if (CompareVertices(geometryArgs, triT1FaceInd[2], index0, triT1Vertex[2], triT0Vertex[indexOffset0]))
                {
                    if (CompareVertices(geometryArgs, triT1FaceInd[1], index1, triT1Vertex[1], triT0Vertex[indexOffset1]))
                    {
                        bestQuad = quadIdx;
                        triT0VtxOffset = i;
                        triT1VtxOffset = 1;
                        break;
                    }
                }

                if (CompareVertices(geometryArgs, triT1FaceInd[1], index0, triT1Vertex[1], triT0Vertex[indexOffset0]))
                {
                    if (CompareVertices(geometryArgs, triT1FaceInd[0], index1, triT1Vertex[0], triT0Vertex[indexOffset1]))
                    {
                        bestQuad = quadIdx;
                        triT0VtxOffset = i;
                        triT1VtxOffset = 2;
                        break;
                    }
                }

                if (CompareVertices(geometryArgs, triT1FaceInd[0], index0, triT1Vertex[0], triT0Vertex[indexOffset0]))
                {
                    if (CompareVertices(geometryArgs, triT1FaceInd[2], index1, triT1Vertex[2], triT0Vertex[indexOffset1]))
                    {
                        bestQuad = quadIdx;
                        triT0VtxOffset = i;
                        triT1VtxOffset = 0;
                        break;
                    }
                }
            }
            if (bestQuad < quadCount)
            {
                break;
            }
        }

        if (bestQuad == quadCount)
        {
            // did not pair with any other triangle
            uint64_t unpairedBitShift64 = (1ULL << triT1LocalIdx);
            cachedQuadBitMask64 |= unpairedBitShift64;

            quadCount++;
            leafCount++;
        }
        else
        {
            if (triT0VtxOffset == 0)
            {
                // Rotate triangle 0 once
                triT0VtxOffset = 1;
            }
            else if (triT0VtxOffset == 2)
            {
                // Rotate triangle 0 twice
                triT0VtxOffset = 2;
            }
            else
            {
                triT0VtxOffset = 0;
            }

            pairFoundBitMask64 |= (1ULL << triT1LocalIdx);
            pairFoundBitMask64 |= (1ULL << triT0LocalIdx);

            // TODO: can we clear this bit from UnPairedTriBitMask to reduce loop search?
            //UnPairedTriBitMask &= ((1u << triT1LocalIdx) - 1);
            //UnPairedTriBitMask &= ((1u << triT0LocalIdx) - 1);

            WriteOutPairedTriangles(geometryArgs,
                                    tri1,
                                    triT1PrimIdx,
                                    triT1VtxOffset,
                                    tri0,
                                    triT0PrimIdx,
                                    triT0VtxOffset);
        }
    }

    // update leaf node counts
    DstMetadata.InterlockedAdd(numLeafNodesOffset, leafCount);

    // Still need to write those triangles that aren't being paired
    WriteOutUnPairedNodes(geometryArgs, batchStartIndex, batchEndIndex, primNodePointerOffset, pairFoundBitMask64);
}

//======================================================================================================================
void EncodePairedTriangleNodeImpl(
    RWBuffer<float3>           GeometryBuffer,
    RWByteAddressBuffer        IndexBuffer,
    RWStructuredBuffer<float4> TransformBuffer,
    GeometryArgs               geometryArgs,
    uint                       primitiveIndex,
    uint                       localId,
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
    const uint numLeafNodesOffset = metadataSize + ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET;

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

        StoreTriangleNodeInLDS(localId, tri, faceIndices, primitiveIndex);

        // Store invalid prim node pointer for now during first time builds.
        // If the triangle is active, BuildQBVH will write it in.
        DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
    }
    else
    {
        // Immediately write out the inactive nodes to scratch
        // Also mark them in LDS, so as to skip such nodes reading during compression phase
        WriteScratchInactiveTriangleNode(geometryArgs,
                                         flattenedPrimitiveIndex,
                                         primNodePointerOffset);

        StoreInactiveTriangleNodeInLDS(localId, primitiveIndex);
    }

    GroupMemoryBarrierWithGroupSync();
    if (primitiveIndex % geometryArgs.trianglePairingSearchRadius == 0)
    {
        CompressTriangleNodes(geometryArgs,
                              primitiveIndex,
                              localId,
                              numLeafNodesOffset,
                              geometryBasePrimOffset,
                              primNodePointerOffset);
    }

    // ClearFlags for refit and update
    ClearFlagsForRefitAndUpdate(geometryArgs, flattenedPrimitiveIndex, false);
}
