/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "../shadersClean/common/ShaderDefs.hlsli"

#define GC_DSTBUFFER
#define GC_DSTMETADATA
#define GC_SCRATCHBUFFER
#include "../shadersClean/build/BuildRootSignature.hlsli"

template<typename T>
T LoadInstanceDescBuffer(uint offset)
{
    return InstanceDescBuffer.Load<T>(offset);
}
#include "IndirectArgBufferUtils.hlsl"
#include "../shadersClean/common/Common.hlsli"
#include "../shadersClean/build/BuildCommon.hlsli"
#include "../shadersClean/build/BuildCommonScratch.hlsli"
#include "EncodeCommon.hlsl"
#include "TaskMacros.hlsl"
#include "EncodePairedTriangleImpl.hlsl"

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeTriangleNodes(
    in uint3 globalThreadId : SV_DispatchThreadID,
    in uint localId : SV_GroupThreadID)
{
#if GPURT_BUILD_RTIP3_1
    if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
    {
        InitLocalKdop(localId, BUILD_THREADGROUP_SIZE);
    }
#endif

    const uint geometryIndex = ShaderRootConstants.GeometryIndex();
    const BuildShaderGeometryConstants geomConstants = GeometryConstants[geometryIndex];
    const NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrim(geometryIndex, true);

    if (globalThreadId.x == 0)
    {
        WriteGeometryInfo(geomConstants, inputOffsets, geometryIndex, DECODE_PRIMITIVE_STRIDE_TRIANGLE);
    }

    uint primitiveIndex = globalThreadId.x;

    if (primitiveIndex < inputOffsets.numPrimitives)
    {
        EncodeTriangleNode(geomConstants,
                           inputOffsets,
                           geometryIndex,
                           primitiveIndex,
                           true);
    }

    IncrementPrimitiveTaskCounters(primitiveIndex,
                                   inputOffsets.numPrimitives,
                                   geomConstants.numPrimitives);

#if GPURT_BUILD_RTIP3_1
    if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
    {
        GroupMemoryBarrierWithGroupSync();
        MergeLocalKdop(localId, BUILD_THREADGROUP_SIZE);
    }
#endif
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeAABBNodes(
    in uint3 globalThreadId : SV_DispatchThreadID)
{
    const uint geometryIndex = ShaderRootConstants.GeometryIndex();
    const BuildShaderGeometryConstants geomConstants = GeometryConstants[geometryIndex];
    const NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrim(geometryIndex, true);

    if (globalThreadId.x == 0)
    {
        WriteGeometryInfo(geomConstants, inputOffsets, geometryIndex, DECODE_PRIMITIVE_STRIDE_AABB);
    }

    uint primitiveIndex = globalThreadId.x;
    if (primitiveIndex < inputOffsets.numPrimitives)
    {
        EncodeAabbNode(geomConstants,
                       inputOffsets,
                       geometryIndex,
                       primitiveIndex,
                       true);
    }

    IncrementPrimitiveTaskCounters(primitiveIndex,
                                   inputOffsets.numPrimitives,
                                   geomConstants.numPrimitives);
}

static const uint LOCAL_COUNTS_READY = ((uint)1 << 30);
static const uint LOCAL_PREFIX_SUM_READY = ((uint)1 << 31);
static const uint COUNTS_MASK = (~(LOCAL_COUNTS_READY | LOCAL_PREFIX_SUM_READY));

// Group shared memory for task counters used for InitBlockPrefixSum
groupshared uint SharedMem[1];
uint GetSharedMem(uint index)
{
    return SharedMem[index];
}
void SetSharedMem(uint index, uint value)
{
    SharedMem[index] = value;
}

//=====================================================================================================================
void WritePrimRefCount(uint id, uint data)
{
    ScratchBuffer.Store(ShaderConstants.offsets.primRefCount + (id * sizeof(uint)), data);
}

//=====================================================================================================================
uint FetchPrimRefCount(uint id)
{
    return ScratchBuffer.Load(ShaderConstants.offsets.primRefCount + (id * sizeof(uint)));
}

//=====================================================================================================================
void InitBlockPrefixSum(
    uint localId,
    uint numBlocks)
{
    uint numTasksWait = 0;
    uint waveId = 0;
    const uint numGroups = RoundUpQuotient(numBlocks, BUILD_THREADGROUP_SIZE);

    INIT_TASK;

    BEGIN_TASK(numGroups);

    if (globalId < numBlocks)
    {
        // Initialise block prefix sum to 0
        WritePrimRefCount(globalId, 0);
    }

    END_TASK(numGroups);
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeQuadNodes(
    in uint globalId : SV_DispatchThreadID,
    in uint localId : SV_GroupThreadID)
{
#if GPURT_BUILD_RTIP3_1
    if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
    {
        InitLocalKdop(localId, BUILD_THREADGROUP_SIZE);
    }
#endif

    // Note, ShaderRootConstants.GeometryIndex() is reused for number of blocks needed across all geometries.
    InitBlockPrefixSum(localId, ShaderRootConstants.GeometryIndex());

    // Figure out which geometry desc contains this blocks primitives.
    const uint blockId = (globalId / WaveGetLaneCount());
    NumPrimAndInputOffset inputOffsets = (NumPrimAndInputOffset)0;
    uint blockOffset = 0;
    uint geomId = 0;
    for (; geomId < ShaderConstants.numDescs; ++geomId)
    {
        inputOffsets = LoadInputOffsetsAndNumPrim(geomId, true);

        const uint numBlocks =
            Pow2Align(inputOffsets.numPrimitives, BUILD_THREADGROUP_SIZE) / BUILD_THREADGROUP_SIZE;

        if (blockId < (blockOffset + numBlocks))
        {
            break;
        }
        else
        {
            blockOffset += numBlocks;
        }
    }

    const BuildShaderGeometryConstants geomConstants = GeometryConstants[geomId];

    const uint startId = blockOffset * WaveGetLaneCount();
    const uint endId = startId + inputOffsets.numPrimitives;

    // Initialise lane as inactive triangle
    int pairInfo = -2;

    TriangleData tri = (TriangleData)0;
    bool isDegenTri0 = true; // We just initialized all vertices of 'tri' to 0. Hence, isDegenTri0 = true
    uint3 indices = uint3(0, 0, 0);

    const uint primId = globalId - startId;
    const uint flattenedPrimitiveIndex = inputOffsets.primitiveOffset + primId;

    if (primId == 0)
    {
        WriteGeometryInfo(geomConstants, inputOffsets, geomId, DECODE_PRIMITIVE_STRIDE_TRIANGLE);
    }

    if (globalId < endId)
    {
        {
            const uint primNodePointerOffset =
                ShaderConstants.header.offsets.primNodePtrs + (flattenedPrimitiveIndex * sizeof(uint));

            // Store invalid prim node pointer for now during first time builds.
            // If the triangle is active, EncodeHwBvh will write it in.
            DstBuffer.Store(primNodePointerOffset, INVALID_IDX);
        }

        const bool validIndices =
            FetchTrianglePrimitive(geomConstants, inputOffsets, GeometryBuffer[geomId], geomId, primId, tri, indices);
        if (validIndices)
        {
            const bool isIndexed = (geomConstants.indexBufferFormat != IndexFormatInvalid);
            pairInfo = PairTriangles(isIndexed, indices, tri);
        }

        // Generate triangle bounds and update scene bounding box
        const BoundingBox boundingBox = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);

        bool isActive = IsActive(tri);

        const bool hasValidQuad = (pairInfo >= 0);
        const uint pairLaneId = hasValidQuad ? pairInfo >> 16 : 0;

        TriangleData tri1;
        tri1.v0 = WaveReadLaneAt(tri.v0, pairLaneId);
        tri1.v1 = WaveReadLaneAt(tri.v1, pairLaneId);
        tri1.v2 = WaveReadLaneAt(tri.v2, pairLaneId);

        bool isDegenTri1 = false;
        bool isDegenQuad = false;
        if (Settings.disableDegenPrims)
        {
            isDegenTri0 = IsDegenerateTriangle(tri); // Compute (before setting to NaN) for later use
            isDegenTri1 = IsDegenerateTriangle(tri1);
            isDegenQuad = hasValidQuad && isDegenTri1 && isDegenTri0;
        }

        if (isActive)
        {
#if GPURT_BUILD_RTIP3_1
            if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
            {
                UpdateTriangleKdop(tri.v0, tri.v1, tri.v2);
            }
#endif
            // Note: Make sure to check for Settings.disableDegenPrims as the very first test in the 'if'
            if ((Settings.disableDegenPrims) && (IsUpdateAllowed() == false) && (isDegenQuad || isDegenTri0))
            {
                // Override v0.x for inactive case
                tri.v0.x = NaN;
                isActive = false;
            }
            else
            {
                // Always encode the scene bounds
                UpdateSceneBounds(ShaderConstants.offsets.sceneBounds, boundingBox);

                // Only when size bits are enabled, update the scene size
                if (IsMortonSizeBitsEnabled(ShaderConstants.numMortonSizeBits))
                {
                    // TODO: with tri splitting, need to not update "size" here
                    UpdateSceneSize(ShaderConstants.offsets.sceneBounds, boundingBox);
                }

                // Only if the centroid bounds are required, update the centroid bounds
                if (IsCentroidMortonBoundsEnabled() || IsConciseMortonBoundsEnabled())
                {
                    // TODO: with tri splitting, need to not update "centroids" here
                    UpdateCentroidBounds(ShaderConstants.offsets.sceneBounds, boundingBox);
                }
            }
        }
        else
        {
            // Override v0.x for inactive case
            tri.v0.x = NaN;
        }

        // Count quads produced by the current wave. Note, this includes unpaired triangles as well
        // (marked with a value of -1).
        const bool isActivePrimRef = (pairInfo >= -1) && isActive;
        const uint blockPrimCount = WaveActiveCountBits(isActivePrimRef);
        const uint laneActivePrimIdx = WavePrefixCountBits(isActivePrimRef);

        uint exclusivePrefixCount = 0;

        if (WaveIsFirstLane())
        {
            {
                const uint flags = LOCAL_COUNTS_READY | ((blockId == 0) ? LOCAL_PREFIX_SUM_READY : 0);
                WritePrimRefCount(blockId, blockPrimCount | flags);
            }

            DeviceMemoryBarrier();

            int prevId = blockId - 1;
            while (prevId >= 0)
            {
                uint count = 0;
                while (1)
                {
                    count = FetchPrimRefCount(prevId);
                    if (count & LOCAL_COUNTS_READY)
                    {
                        break;
                    }

                    DeviceMemoryBarrier();
                }

                exclusivePrefixCount += (count & COUNTS_MASK);
                if (count & LOCAL_PREFIX_SUM_READY)
                {
                    break;
                }
                else
                {
                    prevId--;
                }
            }

            {
                const uint flags = LOCAL_COUNTS_READY | LOCAL_PREFIX_SUM_READY;
                WritePrimRefCount(blockId, (blockPrimCount + exclusivePrefixCount) | flags);
            }
            DeviceMemoryBarrier();
        }

        exclusivePrefixCount = WaveReadLaneFirst(exclusivePrefixCount);

        // Allocate scratch nodes for active triangle primitives using the precomputed block offset for each
        // triangle geometry
        const uint dstScratchNodeIdx = exclusivePrefixCount + laneActivePrimIdx;

        if (isActive)
        {
            const uint primId1 = WaveReadLaneAt(primId, pairLaneId);

            if (hasValidQuad)
            {
                uint instanceMask = 0;
                if (Settings.disableDegenPrims)
                {
                    // Set the instance inclusion mask to 0 for degenerate triangles so that they are culled out.
                    instanceMask = (isDegenTri0 && isDegenTri1) ? 0 : 0xff;
                }

                WriteScratchQuadNode(dstScratchNodeIdx,
                                    geomId,
                                    geomConstants.geometryFlags,
                                    tri1,
                                    primId1,
                                    tri,
                                    instanceMask,
                                    primId,
                                    pairInfo & 0xFF);
            }
            else if (pairInfo == -1)
            {
                uint instanceMask = 0;
                if (Settings.disableDegenPrims)
                {
                    // Set the instance inclusion mask to 0 for degenerate triangles so that they are culled out.
                    instanceMask = isDegenTri0 ? 0 : 0xff;
                }

                // Write out unpaired triangle
                WriteScratchTriangleNode(dstScratchNodeIdx,
                                        geomId,
                                        geomConstants.geometryFlags,
                                        tri,
                                        instanceMask,
                                        primId);
            }
            else
            {
                // This triangle is a pair in a quad handled by the lead lane. Nothing to do here.
            }
        }
        // Update primitive reference count
        if (blockId == (ShaderRootConstants.GeometryIndex() - 1))
        {
            const uint globalCount = (blockPrimCount + exclusivePrefixCount);

            WriteTaskCounterData(ShaderConstants.offsets.encodeTaskCounter,
                                 ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET,
                                 globalCount);

            WriteTaskCounterData(ShaderConstants.offsets.encodeTaskCounter,
                                 ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET,
                                 ShaderConstants.numPrimitives);

            uint buildBlockCount = INVALID_IDX;

            // TODO: The non-HPLOC thread group size can vary. Pass in downstream group size via constant buffer
            if (Settings.buildMode == BUILD_MODE_HPLOC)
            {
                const uint threadGroupSize = 32u;
                buildBlockCount = Pow2Align(globalCount, threadGroupSize) / threadGroupSize;
            }
            if (buildBlockCount == INVALID_IDX)
            {
                const uint threadGroupSize = 64u;
                buildBlockCount = Pow2Align(globalCount, threadGroupSize) / threadGroupSize;
            }

            WriteTaskCounterData(ShaderConstants.offsets.encodeTaskCounter,
                                 ENCODE_TASK_COUNTER_INDIRECT_ARGS,
                                 buildBlockCount);
        }
    }

#if GPURT_BUILD_RTIP3_1
    if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
    {
        GroupMemoryBarrierWithGroupSync();
        MergeLocalKdop(localId, BUILD_THREADGROUP_SIZE);
    }
#endif
}
