/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#define BUILD_THREADGROUP_SIZE 32

#include "../shadersClean/common/ShaderDefs.hlsli"

#define GC_DSTBUFFER
#define GC_DSTMETADATA
#define GC_SCRATCHBUFFER
#include "../shadersClean/build/BuildRootSignature.hlsli"

#define TASK_COUNTER_BUFFER   ScratchBuffer
#define TASK_COUNTER_OFFSET   UPDATE_SCRATCH_TASK_COUNT_OFFSET
#define NUM_TASKS_DONE_OFFSET UPDATE_SCRATCH_TASKS_DONE_OFFSET
#include "TaskMacros.hlsl"

groupshared uint SharedMem[1];
uint GetSharedMem(uint index)
{
    return SharedMem[index];
}
void SetSharedMem(uint index, uint value)
{
    SharedMem[index] = value;
}

#if GPURT_BUILD_RTIP3_1
template<typename T>
T LoadInstanceDescBuffer(uint offset)
{
    return InstanceDescBuffer.Load<T>(offset);
}

//======================================================================================================================
// Note, these headers must be included after all resource bindings have been defined. Also, there is a strict naming
// requirement for resources and variables. See BuildCommon.hlsli for details.
#include "IndirectArgBufferUtils.hlsl"
#include "../shadersClean/build/BuildCommon.hlsli"
#include "EncodeCommon.hlsl"
#include "../shadersClean/build/EncodeTopLevelCommon.hlsli"
#include "../shadersClean/common/LaneGroup.hlsli"

//======================================================================================================================
void ClearRootNodeParentPtr(
    in uint              metadataSize,
    in AccelStructHeader header)
{
    // Temporarily clear root parent ptr, will restore to INVALID at the end.
    DstMetadata.Store(metadataSize + header.offsets.internalNodes + QUANTIZED_BVH8_NODE_OFFSET_PARENT_POINTER, 0);
}

//======================================================================================================================
void ClearAndInitDstBVH(
    in uint                      globalId,
    in uint                      metadataSize,
    in AccelStructMetadataHeader metadata,
    in AccelStructHeader         header,
    in uint                      numThreads)
{
    // Handle the header for not-in-place updates
    if (globalId == 0)
    {
        AccelStructMetadataHeader resultMetadata = metadata;

        uint64_t bvhVa = PackUint64(ShaderConstants.resultBufferAddrLo, ShaderConstants.resultBufferAddrHi);

        bvhVa += metadataSize;

        resultMetadata.addressLo = LowPart(bvhVa);
        resultMetadata.addressHi = HighPart(bvhVa);

        DstMetadata.Store<AccelStructMetadataHeader>(0, resultMetadata);

        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_VERSION_OFFSET,                 header.accelStructVersion);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_INFO_OFFSET,                    header.info);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_METADATA_SIZE_OFFSET,           header.metadataSizeInBytes);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_BYTE_SIZE_OFFSET,               header.sizeInBytes);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_NUM_PRIMS_OFFSET,               header.numPrimitives);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET,        header.numActivePrims);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_NUM_DESCS_OFFSET,               header.numDescs);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET,           header.geometryType);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET, header.numInternalNodesFp32);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP16_NODES_OFFSET, header.numInternalNodesFp16);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET,          header.numLeafNodes);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_COMPACTED_BYTE_SIZE_OFFSET,     header.compactedSizeInBytes);

        DstMetadata.Store<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET, header.offsets);

        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_UUID_LO_OFFSET,                 header.uuidLo);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_UUID_HI_OFFSET,                 header.uuidHi);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_RTIP_LEVEL_OFFSET,              header.rtIpLevel);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_INFO_2_OFFSET,                  header.info2);
        DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_PACKED_FLAGS_OFFSET,            header.packedFlags);
    }

    const uint dstOffsetDataInternalNodes = header.offsets.internalNodes + metadataSize;
    const uint numOfNodes = header.numInternalNodesFp32 + header.numLeafNodes;

    for (uint chunkIdx = globalId; chunkIdx < numOfNodes; chunkIdx += numThreads)
    {
        uint byteOffset = chunkIdx * sizeof(QuantizedBVH8BoxNode);
        DstMetadata.Store(dstOffsetDataInternalNodes + byteOffset + QUANTIZED_BVH8_NODE_OFFSET_PARENT_POINTER, 0);
    }

    for (uint i = globalId; i < header.numActivePrims; i += numThreads)
    {
        const uint currNodeAddr = SrcBuffer.Load(metadataSize + header.offsets.primNodePtrs + (i * sizeof(uint)));
        DstMetadata.Store(metadataSize + header.offsets.primNodePtrs + (i * sizeof(uint)), currNodeAddr);
    }
}

//======================================================================================================================
void EncodeUpdate3_1(
    in uint              globalId,
    in uint              metadataSize,
    in AccelStructHeader header)
{
    const uint globalIndex  = globalId / RTIP3_1_UPDATE_THREADS_PER_FAT_LEAF;
    const bool isProcedural = (Settings.geometryType == GEOMETRY_TYPE_AABBS);

    if ((globalId == 0) && (Settings.topLevelBuild == false))
    {
        const uint primitiveStride = isProcedural ? DECODE_PRIMITIVE_STRIDE_AABB : DECODE_PRIMITIVE_STRIDE_TRIANGLE;

        for (uint geometryIndex = 0; geometryIndex < ShaderConstants.numDescs; geometryIndex++)
        {
            const BuildShaderGeometryConstants geomConstants = GeometryConstants[geometryIndex];
            const NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrim(geometryIndex, true);

            WriteGeometryInfo(geomConstants, inputOffsets, geometryIndex, primitiveStride);
        }
    }

    const uint rootNodeAddr  = metadataSize + header.offsets.internalNodes;
    const uint numUpdatePtrs = header.numInternalNodesFp16;

    if (globalIndex >= numUpdatePtrs)
    {
        return;
    }

    // Map a lane group of 8 threads to a single fat leaf node (internal node with all leaf references)
    LaneGroup laneGroup;
    laneGroup.Alloc(RTIP3_1_UPDATE_THREADS_PER_FAT_LEAF);

    const uint baseUpdatePtrOffset = metadataSize + header.offsets.primNodePtrs +
                                     (ShaderConstants.numPrimitives * sizeof(uint));

    uint currNodeAddr = 0;

    // Fetch fat leaf offset to start refit operation
    if (laneGroup.IsFirstLane())
    {
        currNodeAddr = SrcBuffer.Load(baseUpdatePtrOffset + (globalIndex * sizeof(uint)));
        if (Settings.isUpdateInPlace == false)
        {
            DstMetadata.Store(baseUpdatePtrOffset + (globalIndex * sizeof(uint)), currNodeAddr);
        }
    }

    currNodeAddr = laneGroup.ReadFirstLane(currNodeAddr) + metadataSize;

    while (currNodeAddr != INVALID_IDX)
    {
        uint baseBoxNodeOffset   = 0;
        uint baseLeafNodeOffset  = 0;
        uint expChildIdxAndCount = 0;
        uint parentPtr           = 0;

        if (laneGroup.IsFirstLane())
        {
            baseBoxNodeOffset  = SrcBuffer.Load(currNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_INTERNAL_NODE_BASE_OFFSET);
            baseLeafNodeOffset = SrcBuffer.Load(currNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_LEAF_NODE_BASE_OFFSET);

            parentPtr = (SrcBuffer.Load(currNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_PARENT_POINTER) & bits(29));
            expChildIdxAndCount =
                SrcBuffer.Load(currNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT);

            if (Settings.isUpdateInPlace == false)
            {
                const uint obbMatrixIdx = SrcBuffer.Load(currNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_OBB_MATRIX_INDEX);
                DstMetadata.Store(currNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_INTERNAL_NODE_BASE_OFFSET,
                                  baseBoxNodeOffset);
                DstMetadata.Store(currNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_LEAF_NODE_BASE_OFFSET,
                                  baseLeafNodeOffset);
                if (currNodeAddr != rootNodeAddr)
                {
                    DstMetadata.Store(currNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_PARENT_POINTER, parentPtr);
                }

                DstMetadata.Store(currNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_OBB_MATRIX_INDEX, obbMatrixIdx);
            }
        }

        // And broadcast data to all helper lanes
        baseBoxNodeOffset   = laneGroup.ReadFirstLane(baseBoxNodeOffset) << 3;
        baseLeafNodeOffset  = laneGroup.ReadFirstLane(baseLeafNodeOffset) << 3;
        expChildIdxAndCount = laneGroup.ReadFirstLane(expChildIdxAndCount);

        const uint validChildCount = 1 + bitFieldExtract(expChildIdxAndCount, 28, 3);
        const uint childInfoAddr   = currNodeAddr + GetChildInfoOffset(laneGroup.laneIndex);

        // Write invalid child info for out-of-place updates
        if ((Settings.isUpdateInPlace == false) && (laneGroup.laneIndex >= validChildCount))
        {
            DstMetadata.Store3(childInfoAddr, ChildInfo::Invalid);
        }

        BoundingBox boundingBox;
        boundingBox.min = float3(+FLT_MAX, +FLT_MAX, +FLT_MAX);
        boundingBox.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

        uint16_t childCullFlags = 0xF;
        uint16_t childInstanceInclusionMask = 0;

        // The code below updates the childBoundingBox from curNodeAddr
        // by iterating for all the grandchildren of the children (if it bounding box) of curNodeAddr
        // or calculating new bbox for children (if they are primitives)
        uint16_t nodeTypeAndRangeLength = 0;
        if (laneGroup.laneIndex < validChildCount)
        {
            // Each lane is mapped to an internal node child.
            nodeTypeAndRangeLength =
                (SrcBuffer.Load<uint16_t>(childInfoAddr + QUANTIZED_NODE_CHILD_INFO_OFFSET_MAXY_MAXZ_NODE_TYPE_AND_RANGE + 2) >> 8);
            const uint nodeType = bitFieldExtract(nodeTypeAndRangeLength, 0, 4);
            const bool childIsLeaf = !(NodeTypeIsQuantizedBVH8BoxNode(nodeType));

            const uint numBoxChildren = laneGroup.BallotCount(NodeTypeIsQuantizedBVH8BoxNode(nodeType));

            // Pack packet offset in upper bits for leaf nodes and lower bits for internal nodes.
            const uint shift = childIsLeaf ? 16 : 0;
            const uint nodeRangeLength = bitFieldExtract(nodeTypeAndRangeLength, 4, 4);
            const uint packedLength = (nodeRangeLength << shift);

            // The following can be done more efficiently using cluster_prefix_sum
            //
            // const uint packedOffset = AmdExtD3DShaderIntrinsics_WaveClusterPrefixSum(packedLength, SG_CLUSTER_SIZE_8);
            //

            // Compute the per-lane prefix sum and subtract the lead lane sum from all to obtain the lane group offsets
            const uint waveLaneOffset = WavePrefixSum(packedLength);
            const uint laneGroupSum = laneGroup.ReadFirstLane(waveLaneOffset);
            const uint packedOffset = waveLaneOffset - laneGroupSum;

            // Compute absolute offset in memory from packed offset per lane
            uint childNodeOffset = childIsLeaf ? baseLeafNodeOffset : baseBoxNodeOffset;
            childNodeOffset += (((packedOffset >> shift) & bits(16)) * 128u);

            if (childIsLeaf == false)
            {
                const uint boxNodeOffset            = metadataSize + childNodeOffset;
                const uint expGrandChildIdxAndCount =
                    DstMetadata.Load(boxNodeOffset + QUANTIZED_BVH8_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT);
                const uint validGrandChildCount     = 1 + bitFieldExtract(expGrandChildIdxAndCount, 28, 3);

                uint3 quantMax = uint3(0, 0, 0);
                uint3 quantMin = uint3(0xfff, 0xfff, 0xfff);

                for (uint i = 0; i < validGrandChildCount; ++i)
                {
                    const ChildInfo grandChildInfo =
                        DstMetadata.Load<ChildInfo>(boxNodeOffset + GetChildInfoOffset(i));

                    quantMax = max(quantMax, grandChildInfo.Max());
                    quantMin = min(quantMin, grandChildInfo.Min());

                    if (Settings.topLevelBuild)
                    {
                        childCullFlags &= grandChildInfo.CullingFlags();
                        childInstanceInclusionMask |= grandChildInfo.InstanceMask();
                    }
                }

                uint3 exponents;
                exponents.x = bitFieldExtract(expGrandChildIdxAndCount, 0, 8);
                exponents.y = bitFieldExtract(expGrandChildIdxAndCount, 8, 8);
                exponents.z = bitFieldExtract(expGrandChildIdxAndCount, 16, 8);

                // Expand bounds to full precision
                if ((quantMin.y == 0xfff) && (quantMin.x == 0xfff) && (quantMax.y == 0) && (quantMax.z == 0))
                {
                    boundingBox.min = float3(+FLT_MAX, +FLT_MAX, +FLT_MAX);
                    boundingBox.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
                }
                else
                {
                    float3 origin = DstMetadata.Load<float3>(boxNodeOffset + QUANTIZED_BVH8_NODE_OFFSET_ORIGIN);

                    boundingBox.min = float3(Dequantize(origin.x, exponents.x, quantMin.x, 12),
                                             Dequantize(origin.y, exponents.y, quantMin.y, 12),
                                             Dequantize(origin.z, exponents.z, quantMin.z, 12));

                    boundingBox.max = float3(Dequantize(origin.x, exponents.x, quantMax.x + 1, 12),
                                             Dequantize(origin.y, exponents.y, quantMax.y + 1, 12),
                                             Dequantize(origin.z, exponents.z, quantMax.z + 1, 12));
                }
            }
            else if (Settings.topLevelBuild)
            {
                const uint sidebandOffset = ComputeInstanceSidebandOffset(childNodeOffset,
                                                                          header.offsets.leafNodes,
                                                                          header.offsets.geometryInfo);

                const InstanceSidebandData sidebandData =
                    SrcBuffer.Load<InstanceSidebandData>(metadataSize + sidebandOffset);

                const NumPrimAndOffset numPrimAndOffset = LoadNumPrimAndOffset();
                const InstanceDesc instanceDesc = LoadInstanceDesc(sidebandData.instanceIndex,
                                                                   numPrimAndOffset.primitiveOffset);

                const GpuVirtualAddress blasBaseAddr =
                    MakeGpuVirtualAddress(instanceDesc.accelStructureAddressLo,
                                          instanceDesc.accelStructureAddressHiAndFlags);
                const GpuVirtualAddress blasHeaderAddr = blasBaseAddr + sidebandData.blasMetadataSize;

                const uint numActivePrims =
                    FetchHeaderField(blasHeaderAddr, ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);

                if (numActivePrims != 0)
                {
                    // Fetch root bounds from BLAS header
                    const BoundingBox rootBbox = FetchHeaderRootBoundingBox(blasHeaderAddr);

                    boundingBox = GenerateInstanceBoundingBox(instanceDesc.Transform, rootBbox);
                }

                const uint blasGeometryType = FetchHeaderField(blasHeaderAddr, ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET);
                const uint packedFlags = FetchHeaderField(blasHeaderAddr, ACCEL_STRUCT_HEADER_PACKED_FLAGS_OFFSET);
                childCullFlags =
                    uint16_t(CalcTopLevelBoxNodeFlags(blasGeometryType,
                                             instanceDesc.InstanceContributionToHitGroupIndex_and_Flags >> 24,
                                             ExtractScratchNodeBoxFlags(packedFlags)));

                // Propagate the instance mask from the BLAS by fetching it from the header.
                // Note, the header contains the exlusion mask so we take its bitwise complement to get the inclusion
                // mask. Set the instance inclusion mask to 0 for degenerate instances so that they are culled out.
                childInstanceInclusionMask =
                    (boundingBox.min.x > boundingBox.max.x) ?
                    0 :
                    uint16_t(GetInstanceMask(instanceDesc) & ExtractScratchNodeInstanceMask(packedFlags));

                WriteInstanceNode3_1(instanceDesc,
                                     blasGeometryType,
                                     sidebandData.instanceIndex,
                                     PackNodePointer(NODE_TYPE_USER_NODE_INSTANCE, childNodeOffset),
                                     CreateRootNodePointer3_1(),
                                     sidebandData.blasMetadataSize,
                                     header.offsets,
                                     PackNodePointer(NODE_TYPE_BOX_QUANTIZED_BVH8, (currNodeAddr - metadataSize)));
            }
            else
            {
                // Read leaf information and update local child info
                const uint primStructOffset = metadataSize + childNodeOffset;

                // Fetch geometry index, primitive index and pair descriptor
                const uint4 geomIdPrimIdAndPairDescDwords = SrcBuffer.Load4(primStructOffset + (28 * 4));

                TriangleData tri0 = (TriangleData)0;
                TriangleData tri1 = (TriangleData)0;
                uint3 indices0    = uint3(0, 0, 0);
                uint3 indices1    = uint3(0, 0, 0);
                bool isTri1Valid  = false;
                uint geomId1Dword = 0;

                const uint primId0 = bitFieldExtract(geomIdPrimIdAndPairDescDwords[1], 5, 27) |
                                     (bitFieldExtract(geomIdPrimIdAndPairDescDwords[2], 0, 4) << 27);
                const uint geomId0 = bitFieldExtract(geomIdPrimIdAndPairDescDwords[0], 7, 25) |
                                     (bitFieldExtract(geomIdPrimIdAndPairDescDwords[1], 0, 5) << 25);

                GPU_ASSERT(geomId0 < ShaderConstants.numDescs);

                const BuildShaderGeometryConstants geomConstants0 = GeometryConstants[NonUniformResourceIndex(geomId0)];
                const NumPrimAndInputOffset inputOffsets0 = LoadInputOffsetsAndNumPrim(geomId0, false);
                const uint flattenedPrimitiveIndex0 = inputOffsets0.primitiveOffset + primId0;

                if (isProcedural == false)
                {
                    TrianglePairDesc pairDesc;
                    pairDesc.SetData(bitFieldExtract(geomIdPrimIdAndPairDescDwords[3], 3, 29));

                    isTri1Valid = pairDesc.Tri1Valid();

                    FetchTrianglePrimitive(geomConstants0,
                                           inputOffsets0,
                                           GeometryBuffer[NonUniformResourceIndex(geomId0)],
                                           geomId0,
                                           primId0,
                                           tri0,
                                           indices0);

                    // Generate triangle bounds
                    boundingBox = GenerateTriangleBoundingBox(tri0.v0, tri0.v1, tri0.v2);

                    if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
                    {
                        UpdateTriangleKdop(tri0.v0, tri0.v1, tri0.v2);
                    }

                    if (isTri1Valid)
                    {
                        geomId1Dword = SrcBuffer.Load(primStructOffset + (27 * 4));

                        const uint primId1 = bitFieldExtract(geomIdPrimIdAndPairDescDwords[2], 4, 28) |
                                             (bitFieldExtract(geomIdPrimIdAndPairDescDwords[3], 0, 3) << 28);
                        const uint geomId1 = bitFieldExtract(geomId1Dword, 9, 23) |
                                             (bitFieldExtract(geomIdPrimIdAndPairDescDwords[0], 0, 7) << 23);

                        GPU_ASSERT(geomId1 < ShaderConstants.numDescs);

                        const BuildShaderGeometryConstants geomConstants1 =
                            GeometryConstants[NonUniformResourceIndex(geomId1)];
                        const NumPrimAndInputOffset inputOffsets1 = LoadInputOffsetsAndNumPrim(geomId1, false);
                        const uint flattenedPrimitiveIndex1 = inputOffsets1.primitiveOffset + primId1;

                        FetchTrianglePrimitive(geomConstants1,
                                               inputOffsets1,
                                               GeometryBuffer[NonUniformResourceIndex(geomId1)],
                                               geomId1,
                                               primId1,
                                               tri1,
                                               indices1);

                        // Generate triangle bounds
                        const BoundingBox bbox1 = GenerateTriangleBoundingBox(tri1.v0, tri1.v1, tri1.v2);
                        boundingBox = CombineAABB(boundingBox, bbox1);

                        if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
                        {
                            UpdateTriangleKdop(tri1.v0, tri1.v1, tri1.v2);
                        }
                    }
                }
                else
                {
                    // Generate AABB bounds
                    boundingBox = FetchBoundingBoxData(GeometryBuffer[NonUniformResourceIndex(geomId0)],
                                                       primId0,
                                                       inputOffsets0.vertexOffsetInComponents / 2,
                                                       geomConstants0.geometryStride);

                    tri0.v0 = boundingBox.min;
                    tri0.v1 = boundingBox.max;
                }

                UpdatePairPrimStruct(tri0,
                                     tri1,
                                     isTri1Valid,
                                     geomIdPrimIdAndPairDescDwords,
                                     geomId1Dword,
                                     primStructOffset,
                                     isProcedural);
            }
        }

        // Compute updated origin
        const float3 boundsMin = laneGroup.Min(boundingBox.min);
        const float3 boundsMax = laneGroup.Max(boundingBox.max);

        // Pre-compute exponent and reciprocal
        const uint3  exponents    = ComputeCommonExponent(boundsMin, boundsMax, 12);
        const float3 rcpExponents = ComputeFastExpReciprocal(exponents, 12);

        if (laneGroup.IsFirstLane())
        {
            expChildIdxAndCount = bitFieldInsert(expChildIdxAndCount, 0, 8, exponents.x);
            expChildIdxAndCount = bitFieldInsert(expChildIdxAndCount, 8, 8, exponents.y);
            expChildIdxAndCount = bitFieldInsert(expChildIdxAndCount, 16, 8, exponents.z);

            DstMetadata.Store<float3>(currNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_ORIGIN, boundsMin);
            DstMetadata.Store(currNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT, expChildIdxAndCount);
        }

        bool isDegenerate = false;

        // Re-encode quantized bounds
        if (laneGroup.laneIndex < validChildCount)
        {
            if (Settings.topLevelBuild == false)
            {
                // Geometry flags cannot change during updates as per DXR spec, so cullingFlags cannot change for BLAS.
                childCullFlags =
                    (SrcBuffer.Load<uint16_t>(childInfoAddr + QUANTIZED_NODE_CHILD_INFO_OFFSET_MINX_MINY_CULLING_FLAGS + 2) >> 8);

                // Set the instance inclusion mask to 0 for degenerate children so that they are culled out.
                isDegenerate = (boundingBox.min.x > boundingBox.max.x);
                childInstanceInclusionMask = isDegenerate ? 0 : 0xff;
            }

            const UintBoundingBox quantBounds = ComputeQuantizedBounds(boundingBox, boundsMin, rcpExponents, 12);

            uint3 newChildInfo = uint3(0, 0, 0);
            newChildInfo.x = bitFieldInsert(newChildInfo.x,  0, 12, quantBounds.min.x);
            newChildInfo.x = bitFieldInsert(newChildInfo.x, 12, 12, quantBounds.min.y);
            newChildInfo.x = bitFieldInsert(newChildInfo.x, 24,  8, childCullFlags);
            newChildInfo.y = bitFieldInsert(newChildInfo.y,  0, 12, quantBounds.min.z);
            newChildInfo.y = bitFieldInsert(newChildInfo.y, 12, 12, quantBounds.max.x);
            newChildInfo.y = bitFieldInsert(newChildInfo.y, 24,  8, childInstanceInclusionMask);
            newChildInfo.z = bitFieldInsert(newChildInfo.z,  0, 12, quantBounds.max.y);
            newChildInfo.z = bitFieldInsert(newChildInfo.z, 12, 12, quantBounds.max.z);
            newChildInfo.z = bitFieldInsert(newChildInfo.z, 24,  8, nodeTypeAndRangeLength);

            DstMetadata.Store3(childInfoAddr, newChildInfo);
        }

        const uint numDegenChildren = laneGroup.BallotCount(isDegenerate);

        if (laneGroup.IsFirstLane())
        {
            if (currNodeAddr != rootNodeAddr)
            {
                const uint nextNodeAddr = metadataSize + ExtractNodePointerOffset3_1(parentPtr);
                uint parentBoxNodeCount = 0;

                ComputeChildIndexAndValidBoxCount(metadataSize,
                                                  parentPtr,
                                                  (currNodeAddr - metadataSize) >> 3,
                                                  parentBoxNodeCount);

                uint originalFlagValue = 0;
                DstMetadata.InterlockedAdd(nextNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_PARENT_POINTER,
                                           (1 << 29),
                                           originalFlagValue);
                originalFlagValue >>= 29;

                if (originalFlagValue < (parentBoxNodeCount - 1))
                {
                    currNodeAddr = INVALID_IDX;
                }
                else if (originalFlagValue == (parentBoxNodeCount - 1))
                {
                    const uint Top3BitsClearMask32 = ~(7 << 29);
                    DstMetadata.InterlockedAnd(nextNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_PARENT_POINTER,
                                               Top3BitsClearMask32);
                    currNodeAddr = nextNodeAddr;
                }
                else
                {
                    // Should not get here
                    GPU_ASSERT(false);
                }
            }
            else // Handle root node
            {
                DstMetadata.Store(rootNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_PARENT_POINTER, INVALID_IDX);

                BoundingBox bbox = (BoundingBox)0;
                bbox.min = boundsMin;
                bbox.max = boundsMax;
                DstMetadata.Store<BoundingBox>(metadataSize + ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET, bbox);

                if (Settings.topLevelBuild == false)
                {
                    if ((Settings.instanceMode == InstanceMode::Passthrough) || (header.numActivePrims == 1))
                    {
                        const uint rootNodePointer = CreateRootNodePointer3_1();
                        WriteInstancePassthrough(rootNodePointer,
                                                 bbox,
                                                 ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET);
                    }
                    else if (Settings.instanceMode == InstanceMode::FilterNode)
                    {
                        const uint boxNodeOffset = metadataSize + sizeof(AccelStructHeader);
                        UpdateInstanceFilterNode(boxNodeOffset);
                    }

                    // If all children are degenerate, set instance mask to 0 for root node
                    const uint rootInstanceMask = (numDegenChildren == validChildCount) ? 0 : 0xff;
                    uint packedFlags = SrcBuffer.Load(metadataSize + ACCEL_STRUCT_HEADER_PACKED_FLAGS_OFFSET);
                    const uint boxNodeFlags = packedFlags & 0xFF;
                    packedFlags = PackInstanceMaskAndNodeFlags(rootInstanceMask, boxNodeFlags);
                    DstMetadata.Store(metadataSize + ACCEL_STRUCT_HEADER_PACKED_FLAGS_OFFSET, packedFlags);
                }

                currNodeAddr = INVALID_IDX;
            }
        }
        currNodeAddr = laneGroup.ReadFirstLane(currNodeAddr);
        DeviceMemoryBarrier();
    }
}
#endif

//======================================================================================================================
// Main Function : Update3_1
//======================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void Update3_1(
    uint globalId : SV_DispatchThreadID,
    uint localId  : SV_GroupThreadID)
{
#if GPURT_BUILD_RTIP3_1
    if ((Settings.topLevelBuild == 0) && (Settings.tlasRefittingMode != TlasRefittingMode::Disabled))
    {
        InitLocalKdop(localId, BUILD_THREADGROUP_SIZE);
        GroupMemoryBarrierWithGroupSync();
    }

    const AccelStructMetadataHeader metadata = SrcBuffer.Load<AccelStructMetadataHeader>(0);
    const uint metadataSize = metadata.sizeInBytes;
    const AccelStructHeader header = SrcBuffer.Load<AccelStructHeader>(metadataSize);

    const uint numGroups  = metadata.updateGroupCount[0];
    const uint numThreads = numGroups * BUILD_THREADGROUP_SIZE;

    uint waveId = 0;
    uint numTasksWait = 0;
    INIT_TASK;

    if (Settings.isUpdateInPlace == false)
    {
        BEGIN_TASK(numGroups);
        ClearAndInitDstBVH(globalId, metadataSize, metadata, header, numThreads);
        END_TASK(numGroups);
    }
    else
    {
        BEGIN_TASK(1);
        if (globalId == 0)
        {
            ClearRootNodeParentPtr(metadataSize, header);
        }
        END_TASK(1);
    }

    EncodeUpdate3_1(globalId, metadataSize, header);

    if ((Settings.topLevelBuild == 0) && (Settings.tlasRefittingMode != TlasRefittingMode::Disabled))
    {
        GroupMemoryBarrierWithGroupSync();
        MergeLocalKdop(localId, BUILD_THREADGROUP_SIZE);
    }
#endif
}
