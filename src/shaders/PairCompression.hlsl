/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2021-2022 Advanced Micro Devices, Inc. All Rights Reserved.
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
struct PairCompressionArgs
{
    uint scratchNodesScratchOffset;
    uint numBatchesScratchOffset;
    uint batchIndicesScratchOffset;
    uint indexBufferInfoScratchOffset;
    uint flagsScratchOffset;
    uint buildFlags;
};

#if NO_SHADER_ENTRYPOINT == 0
#include "Common.hlsl"
#include "BuildCommon.hlsl"

#define RootSig "RootConstants(num32BitConstants=6, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894))"

//=====================================================================================================================
[[vk::push_constant]] ConstantBuffer<PairCompressionArgs> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer  ResultBuffer   : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer  ResultMetadata : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer  ScratchBuffer  : register(u2);

#define MAX_LDS_ELEMENTS (16 * BUILD_THREADGROUP_SIZE)
groupshared uint SharedMem[MAX_LDS_ELEMENTS];

#endif

//=====================================================================================================================
#define PAIR_BATCH_SIZE MAX_COLLAPSED_TRIANGLES

#define LDS_OFFSET_BATCH_INDICES 0
#define LDS_OFFSET_STACK         PAIR_BATCH_SIZE
#define LDS_STRIDE               (PAIR_BATCH_SIZE * 2)

//=====================================================================================================================
// Quad is a symbolic representation of a pair of triangles compressed in a single node that share an edge.
struct Quad
{
    uint scratchNodeIndexAndOffset[2];
};

//=====================================================================================================================
// This array is declared as a static global in order to avoid it being duplicated multiple times in scratch memory.
static Quad quadArray[PAIR_BATCH_SIZE];

//=====================================================================================================================
void WriteBatchIndex(uint localId, uint index, uint data)
{
    const uint offset = (localId * LDS_STRIDE) + LDS_OFFSET_BATCH_INDICES + index;
    SharedMem[offset] = data;
}
//=====================================================================================================================
uint ReadBatchIndex(uint localId, uint index)
{
    const uint offset = (localId * LDS_STRIDE) + LDS_OFFSET_BATCH_INDICES + index;
    return SharedMem[offset];
}

//=====================================================================================================================
void WriteStack(uint localId, uint index, uint data)
{
    const uint offset = (localId * LDS_STRIDE) + LDS_OFFSET_STACK + index;
    SharedMem[offset] = data;
}
//=====================================================================================================================
uint ReadStack(uint localId, uint index)
{
    const uint offset = (localId * LDS_STRIDE) + LDS_OFFSET_STACK + index;
    return SharedMem[offset];
}

//=====================================================================================================================
uint GetQuadScratchNodeIndex(in uint packedNodeIndex)
{
    return (packedNodeIndex & 0x3fffffff);
}

//=====================================================================================================================
uint GetQuadScratchNodeVertexOffset(in uint packedNodeIndex)
{
    return (packedNodeIndex >> 30);
}

//=====================================================================================================================
// Get face indices from 16-bit index buffer
uint3 GetFaceIndices16(GpuVirtualAddress bufferVa, uint faceIndex, uint indexBufferByteOffset)
{
    // 3 vertices per triangle with 2-byte indices
    uint baseOffset = (faceIndex * 6) + indexBufferByteOffset;

    // Load address must be 4-byte aligned
    const bool unalignedRead = (baseOffset % 4 == 2);
    if (unalignedRead)
    {
        // Align down load address
        baseOffset -= 2;
    }

    const GpuVirtualAddress baseAddr = bufferVa + baseOffset;

    // Load index buffer data
    uint2 data;
    data.x = LoadDwordAtAddr(baseAddr);
    data.y = LoadDwordAtAddr(baseAddr + 0x4);

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
// Get face indices from 32-bit index buffer
uint3 GetFaceIndices32(GpuVirtualAddress bufferVa, uint faceIndex, uint indexBufferByteOffset)
{
    // 3 vertices per triangle with 4-byte indices
    const uint baseOffset = (faceIndex * 12) + indexBufferByteOffset;

    const GpuVirtualAddress baseAddr = bufferVa + baseOffset;

    uint3 faceIndices;
    faceIndices.x = LoadDwordAtAddr(baseAddr);
    faceIndices.y = LoadDwordAtAddr(baseAddr + 0x4);
    faceIndices.z = LoadDwordAtAddr(baseAddr + 0x8);

    return faceIndices;
}

//=====================================================================================================================
uint3 FetchFaceIndices(
    uint            primitiveIndex,
    IndexBufferInfo indexBufferInfo)
{
    const GpuVirtualAddress indexBufferVa = MakeGpuVirtualAddress(indexBufferInfo.gpuVaLo, indexBufferInfo.gpuVaHi);

    // Fetch face indices from index buffer
    uint3 faceIndices;
    if (indexBufferInfo.format == IndexFormatU16)
    {
        faceIndices = GetFaceIndices16(indexBufferVa, primitiveIndex, indexBufferInfo.byteOffset);
    }
    else if (indexBufferInfo.format == IndexFormatU32)
    {
        faceIndices = GetFaceIndices32(indexBufferVa, primitiveIndex, indexBufferInfo.byteOffset);
    }
    else
    {
        const uint startIndex = (primitiveIndex * 3);
        faceIndices.x = startIndex;
        faceIndices.y = startIndex + 1;
        faceIndices.z = startIndex + 2;
    }

    return faceIndices;
}

//=====================================================================================================================
uint UpdateLeafNodeCounters(uint numLeafNodesOffset, uint numLeafNodes)
{
    uint currentNumNodes;
    ResultBuffer.InterlockedAdd(numLeafNodesOffset, numLeafNodes, currentNumNodes);

    return currentNumNodes;
}

//=====================================================================================================================
void WriteCompressedNodes(
    uint localId,
    uint scratchNodesScratchOffset,
    uint quadIndex)
{
    const AccelStructHeader header = ResultBuffer.Load<AccelStructHeader>(0);

    const uint numLeafNodesOffset = ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET;
    UpdateLeafNodeCounters(numLeafNodesOffset, quadIndex);

    uint numEliminated = 0;

    // This step iterates through all of the quads after compression and writes out the triangle nodes.
    for (int x = 0; x < quadIndex; ++x)
    {
        const Quad quad = quadArray[x];

        uint pairScratchIndices[2] = { INVALID_IDX, INVALID_IDX };
        uint keptIndex = GetQuadScratchNodeIndex(quad.scratchNodeIndexAndOffset[0]);

        if (quad.scratchNodeIndexAndOffset[1] != INVALID_IDX)
        {
            pairScratchIndices[0] = GetQuadScratchNodeIndex(quad.scratchNodeIndexAndOffset[1]);
            pairScratchIndices[1] = GetQuadScratchNodeIndex(quad.scratchNodeIndexAndOffset[0]);

            WriteBatchIndex(localId, numEliminated, GetQuadScratchNodeIndex(quad.scratchNodeIndexAndOffset[0]));
            numEliminated++;

            keptIndex = GetQuadScratchNodeIndex(quad.scratchNodeIndexAndOffset[1]);
        }

        WriteBatchIndex(localId, PAIR_BATCH_SIZE - 1 - x, keptIndex);

        const uint quadScratchIndex = GetQuadScratchNodeIndex(quad.scratchNodeIndexAndOffset[0]);
        const ScratchNode triangleNode = FetchScratchNode(ScratchBuffer, scratchNodesScratchOffset, quadScratchIndex);
        const uint geometryFlags = triangleNode.flags;

        const uint numFaces = 2;

        uint triangleId = 0;
        uint i;
        for (i = 0; i < numFaces; ++i)
        {
            if (quad.scratchNodeIndexAndOffset[i] != INVALID_IDX)
            {
                triangleId = WriteTriangleIdField(
                    triangleId, i, GetQuadScratchNodeVertexOffset(quad.scratchNodeIndexAndOffset[i]), geometryFlags);
            }
        }

        for (i = 0; i < numFaces; ++i)
        {
            if (quad.scratchNodeIndexAndOffset[i] != INVALID_IDX)
            {
                const uint scratchNodeOffset = scratchNodesScratchOffset +
                                               (GetQuadScratchNodeIndex(quad.scratchNodeIndexAndOffset[i]) * sizeof(ScratchNode));

                // Store triangle type and triangle Id
                const uint triangleTypeAndId = (triangleId << 3) | i;
                ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_TYPE_OFFSET, triangleTypeAndId);

                // Repurpose the num prims field for saving the index of the other node in the pair.
                ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET,
                                    pairScratchIndices[i]);
            }
        }
    }
}

//=====================================================================================================================
// Returns true if vertexIndex0 and vertexIndex1 are equivalent for compression purposes.
bool CompareVertices(
    uint                vertexIndex0,
    uint                vertexIndex1,
    uint                scratchIndex0,
    uint                scratchIndex1,
    uint                scratchVertex0,
    uint                scratchVertex1,
    PairCompressionArgs args)
{
    // Vertices match if they have the same vertex index.
    if (vertexIndex0 == vertexIndex1)
    {
        return true;
    }

    // If we decide to not rerun triangle compression on BVH Updates, we can't assume two vertices
    // with the same position but different vertex indices are the same.
    if ((args.buildFlags & DDI_BUILD_FLAG_ALLOW_UPDATE) ||
        (args.buildFlags & DDI_BUILD_FLAG_PERFORM_UPDATE))
    {
        return false;
    }

    // Speedup triangle compression when FAST_BUILD is used by not comparing vertices of different indices.
    if (args.buildFlags & DDI_BUILD_FLAG_PREFER_FAST_BUILD)
    {
        return false;
    }

    // Compress two vertices that have different indices but the same positions together.
    const uint scratchOffset0 = args.scratchNodesScratchOffset + (scratchIndex0 * sizeof(ScratchNode));
    const uint scratchOffset1 = args.scratchNodesScratchOffset + (scratchIndex1 * sizeof(ScratchNode));

    const uint scratchVertexStride = SCRATCH_NODE_V1_OFFSET;
    const float3 vertex0 = ScratchBuffer.Load<float3>(scratchOffset0 + (scratchVertex0 * scratchVertexStride));
    const float3 vertex1 = ScratchBuffer.Load<float3>(scratchOffset1 + (scratchVertex1 * scratchVertexStride));

    if (all(vertex0 == vertex1))
    {
        return true;
    }

    return false;
}

//=====================================================================================================================
uint EncodeTwoTrianglesPerNodeQBVHCompression(
    uint                localId,
    uint                batchSize,
    PairCompressionArgs args)
{
    const uint numIndices = 3;

    uint quadIndex = 0;

    // Loop over every triangle in the triangle group to compress into quads.
    for (uint batchIndex = 0; batchIndex < batchSize; batchIndex++)
    {
        const uint scratchIndex = ReadBatchIndex(localId, batchIndex);

        const ScratchNode node = FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, scratchIndex);

        const uint geometryIndex  = node.right_or_geometryIndex;
        const uint primitiveIndex = node.left_or_primIndex_or_instIndex;

        // Fetch primitive vertex indices
        const uint indexBufferInfoOffset = args.indexBufferInfoScratchOffset +
                                           (geometryIndex * sizeof(IndexBufferInfo));

        const IndexBufferInfo indexBufferInfo = ScratchBuffer.Load<IndexBufferInfo>(indexBufferInfoOffset);

        const uint3 faceIndices = FetchFaceIndices(primitiveIndex, indexBufferInfo);

        uint bestQuad        = quadIndex;
        uint indOff          = 0;
        uint ind[numIndices] = { faceIndices.x, faceIndices.y, faceIndices.z };
        uint vtxOff          = 0;

        // This loop is used to find the Quad in the quadArray that the current triangle compresses the best into.
        for (uint n = 0; n < quadIndex; ++n)
        {
            if (quadArray[n].scratchNodeIndexAndOffset[1] != INVALID_IDX)
            {
                continue;
            }

            const uint quadScratchIndex = GetQuadScratchNodeIndex(quadArray[n].scratchNodeIndexAndOffset[0]);
            const ScratchNode triangleNode = FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, quadScratchIndex);

            // Do not compress triangles from different geometries
            if (geometryIndex != triangleNode.right_or_geometryIndex)
            {
                continue;
            }

            const uint3 quadFaceIndices = FetchFaceIndices(triangleNode.left_or_primIndex_or_instIndex, indexBufferInfo);

            for (int i = 0; i < numIndices; i++)
            {
                const uint indexOffset0 = i % numIndices;
                const uint indexOffset1 = (i + 1) % numIndices;

                const uint index0 = quadFaceIndices[indexOffset0];
                const uint index1 = quadFaceIndices[indexOffset1];

                // Compare all three matching vertex conditions with triangle 0
                if (CompareVertices(ind[2], index0, scratchIndex, quadScratchIndex, 2, indexOffset0, args))
                {
                    if (CompareVertices(ind[1], index1, scratchIndex, quadScratchIndex, 1, indexOffset1, args))
                    {
                        bestQuad = n;
                        indOff   = i;
                        vtxOff   = 1;
                        break;
                    }
                }

                if (CompareVertices(ind[1], index0, scratchIndex, quadScratchIndex, 1, indexOffset0, args))
                {
                    if (CompareVertices(ind[0], index1, scratchIndex, quadScratchIndex, 0, indexOffset1, args))
                    {
                        bestQuad = n;
                        indOff   = i;
                        vtxOff   = 2;
                        break;
                    }
                }

                if (CompareVertices(ind[0], index0, scratchIndex, quadScratchIndex, 0, indexOffset0, args))
                {
                    if (CompareVertices(ind[2], index1, scratchIndex, quadScratchIndex, 2, indexOffset1, args))
                    {
                        bestQuad = n;
                        indOff   = i;
                        vtxOff   = 0;
                        break;
                    }
                }
            }

            if (bestQuad < quadIndex)
            {
                break;
            }
        }

        if (bestQuad == quadIndex)
        {
            // There wasn't a good quad, so allocate a new one and default initialize it.
            quadArray[bestQuad].scratchNodeIndexAndOffset[0] = scratchIndex;
            quadArray[bestQuad].scratchNodeIndexAndOffset[1] = INVALID_IDX;

            quadIndex++;
        }
        else
        {
            uint vtxOff0 = 0;

            if (indOff == 0)
            {
                // Rotate triangle 0 once
                vtxOff0 = 1;
            }
            else if (indOff == 2)
            {
                // Rotate triangle 0 twice
                vtxOff0 = 2;
            }

            quadArray[bestQuad].scratchNodeIndexAndOffset[0]  =
                quadArray[bestQuad].scratchNodeIndexAndOffset[0] | (vtxOff0 << 30);

            quadArray[bestQuad].scratchNodeIndexAndOffset[1]  =
                scratchIndex | (vtxOff << 30);
        }
    }

    WriteCompressedNodes(localId, args.scratchNodesScratchOffset, quadIndex);

    // Return the number of result nodes
    return quadIndex;
}

//=====================================================================================================================
BoundingBox FetchScratchNodeBoundingBoxPair(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                nodeIndex,
    uint                numActivePrims)
{
    const ScratchNode scratchNode = FetchScratchNode(buffer, baseScratchNodesOffset, nodeIndex);

    BoundingBox bbox = GetScratchNodeBoundingBox(scratchNode);

    // If this is a pair, get the other triangle's bounding box and merge.
    if (IsLeafNode(nodeIndex, numActivePrims) && (scratchNode.numPrimitivesAndDoCollapse != INVALID_IDX))
    {
        const ScratchNode otherNode = FetchScratchNode(buffer,
                                                       baseScratchNodesOffset,
                                                       scratchNode.numPrimitivesAndDoCollapse);
        const BoundingBox otherBbox = GetScratchNodeBoundingBox(otherNode);

        bbox.min = min(bbox.min, otherBbox.min);
        bbox.max = max(bbox.max, otherBbox.max);
    }

    return bbox;
}

//=====================================================================================================================
void PairCompressionImpl(
    uint                globalId,
    uint                localId,
    PairCompressionArgs args)
{
    const uint numBatches = ScratchBuffer.Load(args.numBatchesScratchOffset);

    if (globalId >= numBatches)
    {
        return;
    }

    const uint batchRootIndex = ScratchBuffer.Load(args.batchIndicesScratchOffset + (globalId * sizeof(uint)));
    const uint numActivePrims = ResultBuffer.Load(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);

    uint batchSize = 0;
    uint stackPtr = 1;

    WriteStack(localId, 0, batchRootIndex);

    // Gather the scratch node indices of the primitives in the batch.
    while (stackPtr > 0)
    {
        stackPtr--;
        const uint index = ReadStack(localId, stackPtr);

        // Handle leaf-only batch
        if (IsLeafNode(index, numActivePrims))
        {
            WriteBatchIndex(localId, batchSize, index);
            batchSize++;
        }
        else
        {
            const ScratchNode node = FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, index);

            // Clear flags for refit.
            ScratchBuffer.Store(args.flagsScratchOffset + (index * sizeof(uint)), 0);

            const uint left  = node.left_or_primIndex_or_instIndex;
            const uint right = node.right_or_geometryIndex;

            if (IsLeafNode(right, numActivePrims))
            {
                WriteBatchIndex(localId, batchSize, right);
                batchSize++;
            }
            else
            {
                WriteStack(localId, stackPtr, right);
                stackPtr++;
            }

            if (IsLeafNode(left, numActivePrims))
            {
                WriteBatchIndex(localId, batchSize, left);
                batchSize++;
            }
            else
            {
                WriteStack(localId, stackPtr, left);
                stackPtr++;
            }
        }
    }

    // Wait for batches to be written to groupshared memory
    GroupMemoryBarrierWithGroupSync();

    // Compress batch and write out BVH4 nodes
    const uint numNodes = EncodeTwoTrianglesPerNodeQBVHCompression(localId, batchSize, args);

    // Wait for batches to be updated in groupshared memory by WriteCompressedNodes()
    GroupMemoryBarrierWithGroupSync();

    uint currentBatchRootIndex = batchRootIndex;

    // Rearrange the subtree of the batch to reflect the compressed nodes.
    const uint numEliminated = batchSize - numNodes;
    for (uint i = 0; i < numEliminated; i++)
    {
        DeviceMemoryBarrier();

        // Batch Indices now contains the scratch node indices which will no longer be directly linked in the BVH.
        const uint eliminatedIndex = ReadBatchIndex(localId, i);

        const ScratchNode eliminatedNode = FetchScratchNode(ScratchBuffer,
                                                            args.scratchNodesScratchOffset,
                                                            eliminatedIndex);
        const ScratchNode parentNode     = FetchScratchNode(ScratchBuffer,
                                                            args.scratchNodesScratchOffset,
                                                            eliminatedNode.parent);

        const uint possiblyNotEliminatedIndex = (parentNode.left_or_primIndex_or_instIndex == eliminatedIndex) ?
                                                parentNode.right_or_geometryIndex :
                                                parentNode.left_or_primIndex_or_instIndex;

        const uint possiblyNotEliminatedOffset = args.scratchNodesScratchOffset +
                                                 (possiblyNotEliminatedIndex * sizeof(ScratchNode));

        const uint grandParentNodeOffset  = args.scratchNodesScratchOffset + (parentNode.parent * sizeof(ScratchNode));
        const ScratchNode grandParentNode = FetchScratchNode(ScratchBuffer,
                                                             args.scratchNodesScratchOffset,
                                                             parentNode.parent);

        if (grandParentNode.left_or_primIndex_or_instIndex == eliminatedNode.parent)
        {
            ScratchBuffer.Store(grandParentNodeOffset + SCRATCH_NODE_LEFT_OFFSET, possiblyNotEliminatedIndex);
        }
        else
        {
            ScratchBuffer.Store(grandParentNodeOffset + SCRATCH_NODE_RIGHT_OFFSET, possiblyNotEliminatedIndex);
        }

        ScratchBuffer.Store(possiblyNotEliminatedOffset + SCRATCH_NODE_PARENT_OFFSET, parentNode.parent);

        if (eliminatedNode.parent == currentBatchRootIndex)
        {
            // The batch root node has been removed, so update the current batch root.
            currentBatchRootIndex = possiblyNotEliminatedIndex;
        }
    }

    // Refit the batch subtree to reflect the new topology.
    uint keepIndex = PAIR_BATCH_SIZE - 1;
    uint nodeIndex = ReadBatchIndex(localId, keepIndex);
    keepIndex--;

    while (nodeIndex != currentBatchRootIndex)
    {
        DeviceMemoryBarrier();

        // Move to parent node
        const uint parentNodeIndex = FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, nodeIndex).parent;

        // Check parent node's flag
        const uint flagOffset = args.flagsScratchOffset + (parentNodeIndex * sizeof(uint));
        uint originalFlagValue = 0;

        ScratchBuffer.InterlockedAdd(flagOffset, 1, originalFlagValue);

        if (originalFlagValue == 0)
        {
            // If the flag was 0 set it to 1 and continue to the next leaf node. The iteration handling the second
            // child will handle this node.
            nodeIndex = ReadBatchIndex(localId, keepIndex);
            keepIndex--;

            continue;
        }
        else
        {
            // If the flag was 1 the second child is ready and this iteration calculates and writes
            // bbox data for the parent node.
            const ScratchNode parentNode = FetchScratchNode(ScratchBuffer,
                                                            args.scratchNodesScratchOffset,
                                                            parentNodeIndex);

            // Fetch child indices
            const uint lc = parentNode.left_or_primIndex_or_instIndex;
            const uint rc = parentNode.right_or_geometryIndex;

            const ScratchNode leftNode  = FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, lc);
            const ScratchNode rightNode = FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, rc);

            // Fetch bounding children bounding boxes
            BoundingBox bboxRightChild;
            BoundingBox bboxLeftChild;

            if (IsLeafNode(rc, numActivePrims))
            {
                bboxRightChild = FetchScratchNodeBoundingBoxPair(ScratchBuffer,
                                                                 args.scratchNodesScratchOffset,
                                                                 rc,
                                                                 numActivePrims);
            }
            else
            {
                bboxRightChild = GetScratchNodeBoundingBox(rightNode);
            }

            if (IsLeafNode(lc, numActivePrims))
            {
                bboxLeftChild = FetchScratchNodeBoundingBoxPair(ScratchBuffer,
                                                                args.scratchNodesScratchOffset,
                                                                lc,
                                                                numActivePrims);
            }
            else
            {
                bboxLeftChild = GetScratchNodeBoundingBox(leftNode);
            }

            // Merge bounding boxes up to parent
            const float3 bboxMinParent = min(bboxLeftChild.min, bboxRightChild.min);
            const float3 bboxMaxParent = max(bboxLeftChild.max, bboxRightChild.max);
            WriteScratchNodeBoundingBox(ScratchBuffer,
                                        args.scratchNodesScratchOffset,
                                        parentNodeIndex,
                                        bboxMinParent,
                                        bboxMaxParent);

            // Set parent node as next node index
            nodeIndex = parentNodeIndex;
        }
    }
}

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void PairCompression(
    uint globalThreadId : SV_DispatchThreadID,
    uint localThreadId : SV_GroupThreadID)
{
    PairCompressionImpl(globalThreadId, localThreadId, (PairCompressionArgs)ShaderConstants);
}
#endif
