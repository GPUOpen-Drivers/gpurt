/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2021-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#if NO_SHADER_ENTRYPOINT == 0
#define RootSig "CBV(b0), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u6, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u7, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(CBV(b0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b255)"
//=====================================================================================================================
#include "../shared/rayTracingDefs.h"

[[vk::binding(1, 0)]] ConstantBuffer<BuildShaderConstants> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] RWByteAddressBuffer                          SrcBuffer           : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer         DstBuffer           : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer         DstMetadata         : register(u2);
[[vk::binding(3, 0)]] globallycoherent RWByteAddressBuffer         ScratchBuffer       : register(u3);
[[vk::binding(4, 0)]] globallycoherent RWByteAddressBuffer         ScratchGlobal       : register(u4);
[[vk::binding(5, 0)]] RWByteAddressBuffer                          InstanceDescBuffer  : register(u5);
[[vk::binding(6, 0)]] RWByteAddressBuffer                          EmitBuffer          : register(u6);
[[vk::binding(7, 0)]] RWByteAddressBuffer                          IndirectArgBuffer   : register(u7);

[[vk::binding(0, 3)]] ConstantBuffer<BuildShaderGeometryConstants> GeometryConstants[] : register(b0, space1);
[[vk::binding(0, 4)]] RWBuffer<float3>                             GeometryBuffer[]    : register(u0, space1);

#include "Common.hlsl"
#include "BuildCommonScratch.hlsl"
#include "EncodeCommon.hlsl"

#define MAX_LDS_ELEMENTS (16 * BUILD_THREADGROUP_SIZE)
groupshared uint SharedMem[MAX_LDS_ELEMENTS];

#endif

//=====================================================================================================================
#define LDS_OFFSET_BATCH_INDICES 0
#define LDS_OFFSET_STACK         LATE_PAIR_COMP_BATCH_SIZE
#define LDS_STRIDE               (LATE_PAIR_COMP_BATCH_SIZE * 2)

//=====================================================================================================================
// Quad is a symbolic representation of a pair of triangles compressed in a single node that share an edge.
struct Quad
{
    uint scratchNodeIndexAndOffset[2];
};

//=====================================================================================================================
// This array is declared as a static global in order to avoid it being duplicated multiple times in scratch memory.
static Quad quadArray[LATE_PAIR_COMP_BATCH_SIZE];

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
uint UpdateLeafNodeCounters(uint numLeafNodesOffset, uint numLeafNodes)
{
    uint currentNumNodes;
    DstBuffer.InterlockedAdd(numLeafNodesOffset, numLeafNodes, currentNumNodes);

    return currentNumNodes;
}

//=====================================================================================================================
void WriteCompressedNodes(
    uint localId,
    uint scratchNodesScratchOffset,
    uint quadIndex)
{
    const AccelStructHeader header = DstBuffer.Load<AccelStructHeader>(0);

    const uint numLeafNodesOffset = ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET;
    UpdateLeafNodeCounters(numLeafNodesOffset, quadIndex);

    uint numEliminated = 0;

    // This step iterates through all of the quads after compression and writes out the triangle nodes.
    for (int x = 0; x < quadIndex; ++x)
    {
        const Quad quad = quadArray[x];

        uint keptIndex = GetQuadScratchNodeIndex(quad.scratchNodeIndexAndOffset[0]);

        if (quad.scratchNodeIndexAndOffset[1] != INVALID_IDX)
        {
            WriteBatchIndex(localId, numEliminated, GetQuadScratchNodeIndex(quad.scratchNodeIndexAndOffset[0]));
            numEliminated++;

            keptIndex = GetQuadScratchNodeIndex(quad.scratchNodeIndexAndOffset[1]);
        }

        WriteBatchIndex(localId, LATE_PAIR_COMP_BATCH_SIZE - 1 - x, keptIndex);

        // Initialise pair triangle ID from triangle 0
        const uint scratchIdxTri0 = GetQuadScratchNodeIndex(quad.scratchNodeIndexAndOffset[0]);
        const ScratchNode triangleNode = FetchScratchNode(scratchNodesScratchOffset, scratchIdxTri0);

        const uint geometryInfoOffset = header.offsets.geometryInfo + (triangleNode.right_or_geometryIndex * GEOMETRY_INFO_SIZE);
        const uint packedGeometryInfoData = DstBuffer.Load(geometryInfoOffset + GEOMETRY_INFO_FLAGS_AND_NUM_PRIMS_OFFSET);
        const uint geometryFlags = ExtractGeometryInfoFlags(packedGeometryInfoData);

        uint triangleId = WriteTriangleIdField(0,
                                               NODE_TYPE_TRIANGLE_0,
                                               GetQuadScratchNodeVertexOffset(quad.scratchNodeIndexAndOffset[0]),
                                               geometryFlags);

        // If this quad has another triangle, update triangle ID for the pair and update referenced scratch
        // triangle node
        if (quad.scratchNodeIndexAndOffset[1] != INVALID_IDX)
        {
            triangleId = WriteTriangleIdField(triangleId,
                                              NODE_TYPE_TRIANGLE_1,
                                              GetQuadScratchNodeVertexOffset(quad.scratchNodeIndexAndOffset[1]),
                                              geometryFlags);

            const uint scratchNodeOffset = CalcScratchNodeOffset(scratchNodesScratchOffset, keptIndex);

            // Update triangle ID field in scratch node
            const uint packedFlags = (triangleNode.packedFlags & 0x0000ffff) | (triangleId << 16);
            WriteScratchNodeDataAtOffset(scratchNodeOffset, SCRATCH_NODE_FLAGS_OFFSET, packedFlags);

            // Repurpose the node pointer for saving the index of the other node in the pair.
            WriteScratchNodeDataAtOffset(scratchNodeOffset, SCRATCH_NODE_NODE_POINTER_OFFSET, scratchIdxTri0);
        }
    }
}

//=====================================================================================================================
// Returns true if vertexIndex0 and vertexIndex1 are equivalent for compression purposes.
bool CompareVertices(
    uint vertexIndex0,
    uint vertexIndex1,
    uint scratchIndex0,
    uint scratchIndex1,
    uint scratchVertex0,
    uint scratchVertex1,
    uint scratchNodesScratchOffset,
    uint buildFlags)
{
    // Vertices match if they have the same vertex index.
    if (vertexIndex0 == vertexIndex1)
    {
        return true;
    }

    // If we decide to not rerun triangle compression on BVH Updates, we can't assume two vertices
    // with the same position but different vertex indices are the same.
    if ((buildFlags & DDI_BUILD_FLAG_ALLOW_UPDATE) ||
        (buildFlags & DDI_BUILD_FLAG_PERFORM_UPDATE))
    {
        return false;
    }

    // Speedup triangle compression when FAST_BUILD is used by not comparing vertices of different indices.
    if (buildFlags & DDI_BUILD_FLAG_PREFER_FAST_BUILD)
    {
        return false;
    }

    // Compress two vertices that have different indices but the same positions together.
    const uint scratchVertexStride = SCRATCH_NODE_V1_OFFSET;

    float3 vertex0 =
        FETCH_SCRATCH_NODE_DATA(
            float3,
            scratchNodesScratchOffset,
            scratchIndex0,
            scratchVertex0 * scratchVertexStride);

    float3 vertex1 =
        FETCH_SCRATCH_NODE_DATA(
            float3,
            scratchNodesScratchOffset,
            scratchIndex1,
            scratchVertex1 * scratchVertexStride);

    if (all(vertex0 == vertex1))
    {
        return true;
    }

    return false;
}

//=====================================================================================================================
uint EncodeTwoTrianglesPerNodeQBVHCompression(
    uint localId,
    uint batchSize,
    uint scratchNodesScratchOffset)
{
    const uint numIndices = 3;

    uint quadIndex = 0;

    const uint buildInfo = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_INFO_OFFSET);
    const uint buildFlags = (buildInfo >> ACCEL_STRUCT_HEADER_INFO_FLAGS_SHIFT) & ACCEL_STRUCT_HEADER_INFO_FLAGS_MASK;

    // Loop over every triangle in the triangle group to compress into quads.
    for (uint batchIndex = 0; batchIndex < batchSize; batchIndex++)
    {
        const uint scratchIndex = ReadBatchIndex(localId, batchIndex);

        const ScratchNode node = FetchScratchNode(scratchNodesScratchOffset, scratchIndex);

        const uint geometryIndex  = node.right_or_geometryIndex;
        const uint primitiveIndex = node.left_or_primIndex_or_instIndex;

        uint indexOffsetInBytes = 0;

        if (Settings.isIndirectBuild)
        {
            const IndirectBuildOffset buildOffsetInfo =
                IndirectArgBuffer.Load<IndirectBuildOffset>(ShaderConstants.indirectArgBufferStride * geometryIndex);

            indexOffsetInBytes = buildOffsetInfo.primitiveOffset;
        }

        const IndexBufferInfo indexBufferInfo =
        {
            GeometryConstants[NonUniformResourceIndex(geometryIndex)].indexBufferGpuVaLo,
            GeometryConstants[NonUniformResourceIndex(geometryIndex)].indexBufferGpuVaHi,
            GeometryConstants[NonUniformResourceIndex(geometryIndex)].indexBufferByteOffset + indexOffsetInBytes,
            GeometryConstants[NonUniformResourceIndex(geometryIndex)].indexBufferFormat,
        };

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
            const ScratchNode triangleNode = FetchScratchNode(scratchNodesScratchOffset, quadScratchIndex);

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
                if (CompareVertices(ind[2], index0, scratchIndex, quadScratchIndex, 2, indexOffset0, scratchNodesScratchOffset, buildFlags))
                {
                    if (CompareVertices(ind[1], index1, scratchIndex, quadScratchIndex, 1, indexOffset1, scratchNodesScratchOffset, buildFlags))
                    {
                        bestQuad = n;
                        indOff   = i;
                        vtxOff   = 1;
                        break;
                    }
                }

                if (CompareVertices(ind[1], index0, scratchIndex, quadScratchIndex, 1, indexOffset0, scratchNodesScratchOffset, buildFlags))
                {
                    if (CompareVertices(ind[0], index1, scratchIndex, quadScratchIndex, 0, indexOffset1, scratchNodesScratchOffset, buildFlags))
                    {
                        bestQuad = n;
                        indOff   = i;
                        vtxOff   = 2;
                        break;
                    }
                }

                if (CompareVertices(ind[0], index0, scratchIndex, quadScratchIndex, 0, indexOffset0, scratchNodesScratchOffset, buildFlags))
                {
                    if (CompareVertices(ind[2], index1, scratchIndex, quadScratchIndex, 2, indexOffset1, scratchNodesScratchOffset, buildFlags))
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

    WriteCompressedNodes(localId, scratchNodesScratchOffset, quadIndex);

    // Return the number of result nodes
    return quadIndex;
}

//=====================================================================================================================
void PairCompressionImpl(
    uint globalId,
    uint localId,
    uint numActivePrims)
{
    const uint scratchNodesScratchOffset = CalculateBvhNodesOffset(ShaderConstants, numActivePrims);

    const uint numBatches = FetchNumBatches(ShaderConstants.offsets.numBatches);

    if (globalId >= numBatches)
    {
        return;
    }

    const uint batchRootIndex = ScratchBuffer.Load(ShaderConstants.offsets.batchIndices + (globalId * sizeof(uint)));

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
            const ScratchNode node = FetchScratchNode(scratchNodesScratchOffset, index);

            // Clear flags for refit.
            ScratchBuffer.Store(ShaderConstants.offsets.propagationFlags + (index * sizeof(uint)), 0);

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
    const uint numNodes = EncodeTwoTrianglesPerNodeQBVHCompression(localId, batchSize, scratchNodesScratchOffset);

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

        const ScratchNode eliminatedNode = FetchScratchNode(scratchNodesScratchOffset,
                                                            eliminatedIndex);
        const ScratchNode parentNode     = FetchScratchNode(scratchNodesScratchOffset,
                                                            eliminatedNode.parent);

        const uint possiblyNotEliminatedIndex = (parentNode.left_or_primIndex_or_instIndex == eliminatedIndex) ?
                                                parentNode.right_or_geometryIndex :
                                                parentNode.left_or_primIndex_or_instIndex;

        const ScratchNode grandParentNode = FetchScratchNode(scratchNodesScratchOffset,
                                                             parentNode.parent);

        WriteScratchNodeData(
            scratchNodesScratchOffset,
            parentNode.parent,
            (grandParentNode.left_or_primIndex_or_instIndex == eliminatedNode.parent ?
                SCRATCH_NODE_LEFT_OFFSET : SCRATCH_NODE_RIGHT_OFFSET),
            possiblyNotEliminatedIndex);

        WriteScratchNodeData(
            scratchNodesScratchOffset,
            possiblyNotEliminatedIndex,
            SCRATCH_NODE_PARENT_OFFSET,
            parentNode.parent);

        if (eliminatedNode.parent == currentBatchRootIndex)
        {
            // The batch root node has been removed, so update the current batch root.
            currentBatchRootIndex = possiblyNotEliminatedIndex;
        }
    }

    // Refit the batch subtree to reflect the new topology.
    uint keepIndex = LATE_PAIR_COMP_BATCH_SIZE - 1;
    uint nodeIndex = ReadBatchIndex(localId, keepIndex);
    keepIndex--;

    while (nodeIndex != currentBatchRootIndex)
    {
        DeviceMemoryBarrier();

        // Move to parent node
        const uint parentNodeIndex = FetchScratchNode(scratchNodesScratchOffset, nodeIndex).parent;

        // Check parent node's flag
        const uint flagOffset = ShaderConstants.offsets.propagationFlags + (parentNodeIndex * sizeof(uint));
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
            const ScratchNode parentNode = FetchScratchNode(scratchNodesScratchOffset,
                                                            parentNodeIndex);

            // Fetch child indices
            const uint lc = parentNode.left_or_primIndex_or_instIndex;
            const uint rc = parentNode.right_or_geometryIndex;

            const ScratchNode leftNode  = FetchScratchNode(scratchNodesScratchOffset, lc);
            const ScratchNode rightNode = FetchScratchNode(scratchNodesScratchOffset, rc);

            // Fetch bounding children bounding boxes
            BoundingBox bboxRightChild = GetScratchNodeBoundingBox(rightNode,
                                                                   IsLeafNode(rc, numActivePrims),
                                                                   false,
                                                                   0,
                                                                   true,
                                                                   scratchNodesScratchOffset);

            BoundingBox bboxLeftChild = GetScratchNodeBoundingBox(leftNode,
                                                                  IsLeafNode(lc, numActivePrims),
                                                                  false,
                                                                  0,
                                                                  true,
                                                                  scratchNodesScratchOffset);

            // Merge bounding boxes up to parent
            const float3 bboxMinParent = min(bboxLeftChild.min, bboxRightChild.min);
            const float3 bboxMaxParent = max(bboxLeftChild.max, bboxRightChild.max);
            WriteScratchNodeBoundingBox(scratchNodesScratchOffset,
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
    const uint geometryType = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET);
    if (geometryType == GEOMETRY_TYPE_TRIANGLES)
    {
        const uint numActivePrims = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
        PairCompressionImpl(globalThreadId, localThreadId, numActivePrims);
    }
}
#endif
