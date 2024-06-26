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
void WriteScratchTriangleNode(
    uint         leafNodeDataOffset,
    uint         dstScratchNodeIdx,
    uint         geometryIndex,
    uint         geometryFlags,
    TriangleData tri,
    uint         primitiveIndex)
{
    uint offset = CalcScratchNodeOffset(leafNodeDataOffset, dstScratchNodeIdx);

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

    const uint triangleId = WriteTriangleIdField(0, NODE_TYPE_TRIANGLE_0, 0, geometryFlags);

    // Set the instance inclusion mask to 0 for degenerate triangles so that they are culled out.
    const uint instanceMask = (box.min.x > box.max.x) ? 0 : 0xff;
    const uint packedFlags = PackScratchNodeFlags(instanceMask, CalcTriangleBoxNodeFlags(geometryFlags), triangleId);

    data = uint4(0, 0, 0, packedFlags);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET, data);
}

//=====================================================================================================================
void WriteScratchQuadNode(
    uint         leafNodeDataOffset,
    uint         dstScratchNodeIdx,
    uint         geometryIndex,
    uint         geometryFlags,
    TriangleData tri1,
    uint         tri1PrimIdx,
    uint         triT1Rotation,
    TriangleData tri0,
    uint         tri0PrimIdx,
    uint         triT0Rotation)
{
    // TODO: For Navi3, we can directly write the scratch node data to the result leaf node data section
    //
    uint triangleId = 0;

    // triT0 - NODE_TYPE_TRIANGLE_0 (2nd to intersect)
    triangleId = WriteTriangleIdField(triangleId, NODE_TYPE_TRIANGLE_0, triT0Rotation, geometryFlags);

    // triT1 - NODE_TYPE_TRIANGLE_1 (1st to intersect)
    triangleId = WriteTriangleIdField(triangleId, NODE_TYPE_TRIANGLE_1, triT1Rotation, geometryFlags);

    uint offset = CalcScratchNodeOffset(leafNodeDataOffset, dstScratchNodeIdx);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_PRIMITIVE_ID_OFFSET, tri1PrimIdx);

    // Pack triangle 0 primitive offset from triangle 1. Note, the pairing process always searches within a contiguous
    // group of primitives and finds a paired triangle that has a lower primitive index than itself. Maximum offset
    // currently is not expected to be more than 63
    const uint primIdOffset = tri1PrimIdx - tri0PrimIdx;
    const uint packedGeomId = geometryIndex | (primIdOffset << 24u);

    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_GEOMETRY_INDEX_OFFSET, packedGeomId);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_PARENT_OFFSET, INVALID_IDX);

    const uint3 t0VtxIndices = CalcTriangleCompressionVertexIndices(NODE_TYPE_TRIANGLE_0, triangleId);
    const uint3 t1VtxIndices = CalcTriangleCompressionVertexIndices(NODE_TYPE_TRIANGLE_1, triangleId);

    const uint3 t1VtxOffsets = SCRATCH_NODE_V0_OFFSET + (t1VtxIndices * SCRATCH_NODE_TRIANGLE_VERTEX_STRIDE);
    WriteScratchNodeDataAtOffset(offset, t1VtxOffsets.x, tri1.v0);
    WriteScratchNodeDataAtOffset(offset, t1VtxOffsets.y, tri1.v1);
    WriteScratchNodeDataAtOffset(offset, t1VtxOffsets.z, tri1.v2);

    float3 v0 = tri0.v2;
    if (t0VtxIndices.x == 0)
    {
        v0 = tri0.v0;
    }
    else if (t0VtxIndices.y == 0)
    {
        v0 = tri0.v1;
    }

    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_V0_OFFSET, v0);

    // Account for the unshared vertex from triangle 0
    BoundingBox box = GenerateTriangleBoundingBox(tri1.v0, tri1.v1, tri1.v2);
    box.min = min(box.min, v0);
    box.max = max(box.max, v0);

    // Set the instance inclusion mask to 0 for degenerate triangles so that they are culled out.
    const uint instanceMask = (box.min.x > box.max.x) ? 0 : 0xff;

    const uint packedFlags = PackScratchNodeFlags(instanceMask, CalcTriangleBoxNodeFlags(geometryFlags), triangleId);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_FLAGS_OFFSET, packedFlags);
}

//=====================================================================================================================
template<typename T>
uint TryPairTriangles(
    const T t0,
    const T t1)
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
float ComputePairAreaRatio(
    BoundingBox bbox0,
    BoundingBox bbox1)
{
    const float tri0Sa = ComputeBoxSurfaceArea(bbox0);
    const float tri1Sa = ComputeBoxSurfaceArea(bbox1);
    const float mergedSa = ComputeBoxSurfaceArea(CombineAABB(bbox0, bbox1));
    const float ratio = (mergedSa / (tri0Sa + tri1Sa));
    return ratio;
}

//======================================================================================================================
template<typename T>
int PairTrianglesOptimal(
    T tri,
    BoundingBox bbox,
    bool isActive)
{
    bool valid = isActive;

    // Initialise to unpaired triangle
    int pairInfo = -1;

    while (valid)
    {
        const bool isBroadcastLane = WaveIsFirstLane();
        if (isBroadcastLane)
        {
            valid = false;
        }

        const T broadcastTriangle = WaveReadLaneFirst(tri);

        uint packedOffset = -1;
        if (valid)
        {
            packedOffset = TryPairTriangles(broadcastTriangle, tri);
        }

        const BoundingBox broadcastTriBounds =
        {
            WaveReadLaneFirst(bbox.min),
            WaveReadLaneFirst(bbox.max),
        };

        const float ratio = (packedOffset == -1) ? FLT_MAX : ComputePairAreaRatio(broadcastTriBounds, bbox);

        const float waveMinRatio = WaveActiveMin(ratio);

        const float pairingAreaThreshold = 1.15f;
        const bool isOptimalPair = (waveMinRatio <= pairingAreaThreshold) && (ratio == waveMinRatio);

        const uint firstPairedLane = FIRSTBITLOW_U64(WaveActiveBallot64((packedOffset != -1) && isOptimalPair));

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
template<typename T>
int PairTrianglesImpl(
    T tri,
    bool isActive)
{
    bool valid = isActive;

    // Initialise to unpaired triangle
    int pairInfo = -1;

    while (valid)
    {
        const bool isBroadcastLane = WaveIsFirstLane();
        if (isBroadcastLane)
        {
            valid = false;
        }

        const T broadcastTriangle = WaveReadLaneFirst(tri);

        uint packedOffset = -1;
        if (valid)
        {
            packedOffset = TryPairTriangles(broadcastTriangle, tri);
        }

        const uint firstPairedLane = FIRSTBITLOW_U64(WaveActiveBallot64((packedOffset != -1)));

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

//=====================================================================================================================
int PairTriangles(
    bool isIndexed,
    uint3 faceIndices,
    TriangleData tri)
{
    // Initialise as unpaired triangle
    int pairInfo = -1;

    // Indexed triangles can always be paired as their connectivity cannot change on updates.
    if (isIndexed)
    {
        if (Settings.enablePairCostCheck)
        {
            const BoundingBox bbox = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);
            pairInfo = PairTrianglesOptimal(faceIndices, bbox, true);
        }
        else
        {
            pairInfo = PairTrianglesImpl(faceIndices, true);
        }
    }
    // Only pair non-indexed triangles for non-updateable as the triangle positions can change on updates
    else if (IsUpdateAllowed() == false)
    {
        float3x3 faceVertices;
        faceVertices[0] = tri.v0;
        faceVertices[1] = tri.v1;
        faceVertices[2] = tri.v2;

        if (Settings.enablePairCostCheck)
        {
            const BoundingBox bbox = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);
            pairInfo = PairTrianglesOptimal(faceVertices, bbox, true);
        }
        else
        {
            pairInfo = PairTrianglesImpl(faceVertices, IsActive(tri));
        }
    }

    return pairInfo;
}
