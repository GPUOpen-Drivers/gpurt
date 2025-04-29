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
// Note, CBV(b255) must be the last used binding in the root signature.
#define RootSig "CBV(b0),"\
                "CBV(b1),"\
                "UAV(u0),"\
                "UAV(u1),"\
                "UAV(u2),"\
                "UAV(u3),"\
                "UAV(u4),"\
                "UAV(u5),"\
                "DescriptorTable(CBV(b0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 1)),"\
                "CBV(b255),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\

#define DISABLE_BUILD_ROOT_SIGNATURE

#if GPURT_BUILD_RTIP3_1

#include "../shared/gpurtBuildConstants.h"
#include "../shared/rayTracingDefs.h"

[[vk::binding(0, 1)]] ConstantBuffer<BuildShaderConstants>             ShaderConstants     : register(b0);
[[vk::binding(1, 1)]] ConstantBuffer<ObbData>                          LutBuffer           : register(b1);
[[vk::binding(0, 0)]] RWByteAddressBuffer                              DstMetadata         : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer                              DstBuffer           : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer                              ScratchBuffer       : register(u2);
[[vk::binding(3, 0)]] RWByteAddressBuffer                              InstanceDescBuffer  : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer                              EmitBuffer          : register(u4);
[[vk::binding(5, 0)]] RWByteAddressBuffer                              IndirectArgBuffer   : register(u5);
[[vk::binding(0, 3)]] ConstantBuffer<BuildShaderGeometryConstants>     GeometryConstants[] : register(b0, space1);
[[vk::binding(0, 4)]] RWBuffer<float3>                                 GeometryBuffer[]    : register(u0, space1);

// The following headers have a dependency on the bindings above
#define SrcBuffer DstBuffer

#include "../shadersClean/common/ShaderDefs.hlsli"
#include "../shadersClean/common/Common.hlsli"

#include "../shadersClean/build/BuildCommon.hlsli"
#include "CompactCommon.hlsl"
#include "MortonCodes.hlsl"
#include "TrianglePrimitive.hlsl"
#include "../shadersClean/common/LaneGroup.hlsli"
#include "IndirectArgBufferUtils.hlsl"

#include "SingleThreadGroupBuild/GroupShared.hlsl"
GroupSharedMemLayout LDS;

#include "SingleThreadGroupBuild/ScratchBuffer.hlsl"
#include "SingleThreadGroupBuild/MergeSortLocal.hlsl"
#include "SingleThreadGroupBuild/LBVH.hlsl"
#include "SingleThreadGroupBuild/PLOC.hlsl"
#include "SingleThreadGroupBuild/HPLOC.hlsl"

//=====================================================================================================================
TriangleData FetchTrianglePrimitiveData(
    uint geomId,
    uint primId)
{
    const BuildShaderGeometryConstants geomConst = GeometryConstants[NonUniformResourceIndex(geomId)];
    const NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrim(geomId, false);

    uint3 indices = uint3(0, 0, 0);

    TriangleData tri = (TriangleData)0;
    const bool validIndices = FetchTrianglePrimitive(geomConst,
                                                     inputOffsets,
                                                     GeometryBuffer[NonUniformResourceIndex(geomId)],
                                                     geomId,
                                                     primId,
                                                     tri,
                                                     indices);

    return tri;
}

#include "QBVH8Common.hlsl"
#include "PrimitiveStructureEncoder3_1.hlsl"
#include "SingleThreadGroupBuild/EncodeHwBvhCounters.hlsl"
#include "SingleThreadGroupBuild/PrimCompress3_1_SingleTG.hlsl"
#include "SingleThreadGroupBuild/EncodeHwBvh3_1_SingleTG.hlsl"

#define ScratchGlobal ScratchBuffer
#include "OrientedBoundingBoxes.hlsl"

#define NO_SHADER_ENTRYPOINT 1
#include "RefitOrientedBounds3_1.hlsl"

//=====================================================================================================================
bool EnableMortonSizeBits()
{
    return (ShaderConstants.numMortonSizeBits > 0);
}

//======================================================================================================================
// Binary search the geometry info to map a thread to a geometry and primitive within the geometry
uint FindGeometryIndex(
    uint index)
{
    uint left = 0;
    uint right = ShaderConstants.numDescs;

    do
    {
        uint mid = (left + right) / 2;

        const NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrim(mid, false);
        const uint begin = inputOffsets.primitiveOffset;
        const uint end = begin + inputOffsets.numPrimitives;

        if ((begin <= index) && (index < end))
        {
            left = mid;
            break;
        }
        else if (index > begin)
        {
            left = mid + 1;
        }
        else
        {
            right = mid;
        }
    } while (left + 1 < right);

    return left;
}

//======================================================================================================================
void UpdateSceneBounds(BoundingBox bbox)
{
    // Generate scene bounds in group shared memory
    float3 waveBoundsMin = WaveActiveMin(bbox.min);
    float3 waveBoundsMax = WaveActiveMax(bbox.max);

    if (WaveIsFirstLane())
    {
        LDS.UpdateSceneBounds(waveBoundsMin, waveBoundsMax);
    }

    if (EnableMortonSizeBits())
    {
        const float surfaceArea = ComputeBoxSurfaceArea(bbox);
        const float surfaceAreaMin = WaveActiveMin(surfaceArea);
        const float surfaceAreaMax = WaveActiveMax(surfaceArea);

        if (WaveIsFirstLane())
        {
            LDS.UpdateSceneSize(surfaceAreaMin, surfaceAreaMax);
        }
    }
}

//=====================================================================================================================
static void UpdateTriangleKdop(
    float3 v0,
    float3 v1,
    float3 v2)
{
    for (uint i = 0; i < KDOP_PLANE_COUNT; i++)
    {
        const float3 kdopDirection = GenerateKdopVertex(2 * i);
        const float dist0 = dot(kdopDirection, v0);
        const float dist1 = dot(kdopDirection, v1);
        const float dist2 = dot(kdopDirection, v2);
        const float minExtent = min(dist0, min(dist1, dist2));
        const float maxExtent = max(dist0, max(dist1, dist2));

        LDS.UpdateKDopMin(i, minExtent);
        LDS.UpdateKDopMax(i, maxExtent);
    }
}

//======================================================================================================================
// Initialise group shared memory data associated with encode phase
void InitBuild(
    uint threadId)
{
    if (threadId == 0)
    {
        // Global scene bounds
        LDS.InitSceneBounds();

        if (EnableMortonSizeBits())
        {
            // Global scene size min/max
            LDS.InitSceneSizeMinMax();
        }

        // Initalize headers
        DstMetadata.Store(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, ShaderConstants.resultBufferAddrLo);
        DstMetadata.Store(ACCEL_STRUCT_METADATA_VA_HI_OFFSET, ShaderConstants.resultBufferAddrHi);
        DstMetadata.Store(ACCEL_STRUCT_METADATA_SIZE_OFFSET, ShaderConstants.header.metadataSizeInBytes);
        DstBuffer.Store(0, ShaderConstants.header);

        // Initialise PLOC node counter to 0
        ScratchBuffer.Store(GetBaseOffsetMiscPlocNodeCounter(), 0);

        if (Settings.enableOrientedBoundingBoxes != 0)
        {
            const ObbRefitStackPtrs initRefitStackPtrs = {
                0, 1, 1, 0,
            };

            ScratchBuffer.Store<ObbRefitStackPtrs>(ShaderConstants.offsets.obbRefitStackPtrs, initRefitStackPtrs);
        }
    }

    if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
    {
        for (uint i = threadId; i < KDOP_PLANE_COUNT; i += STGB_THREADGROUP_SIZE)
        {
            LDS.WriteKDopMin(i, +FLT_MAX);
            LDS.WriteKDopMax(i, -FLT_MAX);
        }
    }
}

//======================================================================================================================
uint EncodeTrianglePrimitive(
    uint threadId)
{
    // Write geometry info data structures per-geometry
    if (threadId < ShaderConstants.numDescs)
    {
        const BuildShaderGeometryConstants geometryConstants = GeometryConstants[NonUniformResourceIndex(threadId)];
        const NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrim(threadId, false);

        GeometryInfo info;
        info.geometryBufferOffset = inputOffsets.primitiveOffset * DECODE_PRIMITIVE_STRIDE_TRIANGLE;
        info.primNodePtrsOffset   = inputOffsets.primitiveOffset * sizeof(uint);
        info.geometryFlagsAndNumPrimitives =
            PackGeometryFlagsAndNumPrimitives(geometryConstants.geometryFlags, inputOffsets.numPrimitives);

        const uint geometryInfoOffset = ShaderConstants.header.offsets.geometryInfo + (threadId * GEOMETRY_INFO_SIZE);
        DstBuffer.Store<GeometryInfo>(geometryInfoOffset, info);
    }

    const uint maxActiveWaveCount = RoundUpQuotient(ShaderConstants.numPrimitives, WaveGetLaneCount());
    const uint waveId = threadId / WaveGetLaneCount();

    BoundingBox bbox = InvalidBoundingBox;
    uint geomId = 0;
    uint primId = 0;
    uint packedFlags = 0;
    bool isActiveTriangle = false;

    if (threadId < ShaderConstants.numPrimitives)
    {
        // Map thread index to geometry index and primitive index
        geomId = FindGeometryIndex(threadId);

        const BuildShaderGeometryConstants geomConst = GeometryConstants[NonUniformResourceIndex(geomId)];
        const NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrim(geomId, false);
        primId = threadId - inputOffsets.primitiveOffset;

        TriangleData tri = (TriangleData)0;
        uint3 indices = uint3(0, 0, 0);
        const bool validIndices = FetchTrianglePrimitive(geomConst,
                                                         inputOffsets,
                                                         GeometryBuffer[NonUniformResourceIndex(geomId)],
                                                         geomId,
                                                         primId,
                                                         tri,
                                                         indices);
        isActiveTriangle = (validIndices && IsActive(tri));

        uint obbMatrixIdx = INVALID_OBB;
        if (isActiveTriangle)
        {
            if (Settings.enableOrientedBoundingBoxes != 0)
            {
                obbMatrixIdx = FetchTriKdopObbMatrixIndex(tri.v0, tri.v1, tri.v2);
            }

            if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
            {
                UpdateTriangleKdop(tri.v0, tri.v1, tri.v2);
            }

            if ((Settings.disableDegenPrims) && (IsUpdateAllowed() == false) && IsDegenerateTriangle(tri))
            {
                isActiveTriangle = false;
            }
            else
            {
                bbox = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);
                UpdateSceneBounds(bbox);
            }
        }

        // Set the instance inclusion mask to 0 for degenerate triangles so that they are culled out.
        uint instanceMask = 0;
        if (Settings.disableDegenPrims)
        {
            instanceMask = IsDegenerateTriangle(tri) ? 0 : 0xff;
        }
        else
        {
            instanceMask = (bbox.min.x > bbox.max.x) ? 0 : 0xff;
        }

        packedFlags = PackScratchNodeFlags(instanceMask, CalcTriangleBoxNodeFlags(geomConst.geometryFlags), obbMatrixIdx);
    }

    const uint waveActivePrimCount = WaveActiveCountBits(isActiveTriangle);
    const uint laneActivePrimIndex = WavePrefixCountBits(isActiveTriangle);

    if (WaveIsFirstLane())
    {
        LDS.WriteWaveActivePrimRefCount(waveId, waveActivePrimCount);
    }

    if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
    {
        GroupMemoryBarrierWithGroupSync();

        if (threadId < KDOP_PLANE_COUNT)
        {
            const uint dstOffset = ACCEL_STRUCT_METADATA_KDOP_OFFSET + (threadId * sizeof(float2));
            const uint2 kdopMinMax = uint2(LDS.ReadKDopMinUint(threadId), LDS.ReadKDopMaxUint(threadId));
            DstMetadata.Store2(dstOffset, kdopMinMax);
        }
    }

    GroupMemoryBarrierWithGroupSync();

    // Prefix sum active primitive counters
    if (waveId == 0)
    {
        const uint wavePrimCount = (threadId < maxActiveWaveCount) ? LDS.ReadWaveActivePrimRefCount(threadId) : 0u;
        GroupMemoryBarrier();

        const uint wavePrimOffset = WavePrefixSum(wavePrimCount);
        LDS.WriteWaveActivePrimRefCount(threadId, wavePrimOffset);
        LDS.WriteWaveActivePrimRefCount(maxActiveWaveCount, WaveActiveSum(wavePrimCount));
    }

    GroupMemoryBarrierWithGroupSync();

    const uint numPrimRefs = LDS.ReadWaveActivePrimRefCount(maxActiveWaveCount);
    if (isActiveTriangle)
    {
        const uint primRefIndex = laneActivePrimIndex + LDS.ReadWaveActivePrimRefCount(waveId);

        const uint dstNodeIndex = GetBvh2PrimRefNodeIndex(primRefIndex, numPrimRefs);
        WriteBvh2Bounds(dstNodeIndex, bbox);
        WriteBvh2ChildPtr(dstNodeIndex, geomId, false);
        WriteBvh2ChildPtr(dstNodeIndex, primId, true);
        WriteBvh2NodeFlags(dstNodeIndex, packedFlags);
    }

    return numPrimRefs;
}

//=====================================================================================================================
void PostHwBvhBuild(
    uint threadId)
{
    if (threadId == 0)
    {
        // TODO: Avoid loading header from memory. The header data is either a constant (part of BuildShaderConstants)
        //       or is computed by the builder elsewhere.
        const AccelStructHeader header = DstBuffer.Load<AccelStructHeader>(0);

        // Unused
        uint metadataSizeInBytes = 0;
        INIT_VAR(AccelStructOffsets, offsets);

        // Calculate compacted size
        uint compactedSize = CalcCompactedSize(header,
                                               false,
                                               offsets,
                                               metadataSizeInBytes);
        if (Settings.disableCompaction)
        {
            compactedSize = ShaderConstants.header.sizeInBytes;
        }
        WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_COMPACTED_BYTE_SIZE_OFFSET, compactedSize);

        if (Settings.emitCompactSize != 0)
        {
            EmitBuffer.Store2(0, uint2(compactedSize, 0));
        }

        WriteUpdateGroupCount(RTIP3_1_UPDATE_THREADS_PER_FAT_LEAF);
    }
}
#endif

//======================================================================================================================
[RootSignature(RootSig)]
[numthreads(STGB_THREADGROUP_SIZE, 1, 1)]
void BuildSingleThreadGroup(
    in uint threadId : SV_DispatchthreadId)
{
#if GPURT_BUILD_RTIP3_1
    InitBuild(threadId);

    GroupMemoryBarrierWithGroupSync();

    const uint numPrimRefs = EncodeTrianglePrimitive(threadId);

    // Cache transient data from group shared memory into registers. GenerateMortion/Sort will use up all group shared
    // memory slots. Note, we can potentially pack these in unused offsets in per-thread LDS, but it's simpler to just
    // cache them here since they are required for GenerateMorton phase anyway.

    // TODO: Store Centroid bounds in LDS to fetch MortonBounds
    BoundingBox sceneBounds = LDS.ReadSceneBounds();
    const float3 sceneExtents = sceneBounds.max - sceneBounds.min;
    const float2 sizeMinMax = LDS.ReadSceneSizeMinMax();

    // Ensure all threads have read the LDS data before continuing
    GroupMemoryBarrierWithGroupSync();

    if (threadId == 0)
    {
        // Update scene bounds and active primitive count in acceleration structure header
        DstBuffer.Store<BoundingBox>(ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET, sceneBounds);
        DstBuffer.Store<uint>(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET, numPrimRefs);
    }

    // Compute power2 numPrimitives
    const uint pow2NumElements = (firstbithigh(numPrimRefs) == firstbitlow(numPrimRefs))
        ? numPrimRefs : 1U << (firstbithigh(numPrimRefs) + 1);

    // Initialise morton code for valid threads to a large value such that they are sorted towards the end
    uint64_t mortonCode = Settings.useMortonCode30 ? 0x00000000fffffffe : 0xfffffffffffffffe;

    // Generate morton codes for valid primitives
    if (threadId < numPrimRefs)
    {
        const uint primRefNodeIndex = GetBvh2PrimRefNodeIndex(threadId, numPrimRefs);
        const BoundingBox bbox = ReadBvh2NodeBounds(primRefNodeIndex);

        if (IsInvalidBoundingBox(bbox) == false)
        {
            const float3 position = 0.5f * (bbox.max + bbox.min);
            if (Settings.useMortonCode30)
            {
                // TODO: Store centroid bounds in LDS
                const uint32_t mortonCode30 = CalculateMortonCode32(sceneBounds.min,
                                                                    sceneExtents,
                                                                    position,
                                                                    false);
                mortonCode = PackUint64(mortonCode30, 0);
            }
            else
            {
                const float surfaceArea = ComputeBoxSurfaceArea(bbox);

                // TODO: Store centroid bounds in LDS
                mortonCode = CalculateMortonCode64(sceneBounds.min,
                                                   sceneExtents,
                                                   sizeMinMax,
                                                   position,
                                                   surfaceArea,
                                                   false,
                                                   ShaderConstants.numMortonSizeBits);
            }
        }
    }

    // Write morton code and primitive indices to group shared memory for sorting
    if (threadId < pow2NumElements)
    {
        LDS.WriteMortonCode(threadId, mortonCode);
        LDS.WritePrimRefIdx(threadId, threadId);

        if (Settings.buildMode == BUILD_MODE_LINEAR)
        {
            LDS.WriteBvh2RangeFlag(threadId, 0xffffffffu);
        }
        else if (Settings.buildMode == BUILD_MODE_HPLOC)
        {
            // Note, LDS is not available to store the per-internal node flags in HPLOC builder.
            DstBuffer.Store(ShaderConstants.header.offsets.leafNodes + (threadId * sizeof(uint)), 0xffffffffu);
        }
    }

    // Wait for morton codes and associated data to be written in group shared memory
    GroupMemoryBarrierWithGroupSync();

    OddEvenMergeSort(threadId, STGB_THREADGROUP_SIZE, pow2NumElements);

    // implied GroupMemoryBarrierWithGroupSync() from last iteration of merge sort.
    if (Settings.buildMode == BUILD_MODE_LINEAR)
    {
        BuildLBVH(threadId, numPrimRefs);
    }
    else if (Settings.buildMode == BUILD_MODE_HPLOC)
    {
        BuildHPLOC(threadId, numPrimRefs);
    }
    else if (Settings.buildMode == BUILD_MODE_PLOC)
    {
        BuildPLOC(threadId, numPrimRefs);
    }

    // Wait for BVH2 hierarchy data to be written to global memory
    DeviceMemoryBarrierWithGroupSync();

    const uint fixedRootNodeIndex = (numPrimRefs == 1) ? MakePrimRefChildPtr(0, 1) : 0;

    const uint rootNodeIndex =
        (Settings.buildMode == BUILD_MODE_LINEAR) ? ReadRootNodeIndex() : fixedRootNodeIndex;

    EncodeHwBvh3_1(threadId, rootNodeIndex, numPrimRefs);

    // Wait for hardware BVH data to be written to global memory
    DeviceMemoryBarrierWithGroupSync();

    if (EnablePrimitiveCompressionPass())
    {
        CompressPrims3_1(threadId);
        DeviceMemoryBarrierWithGroupSync();
    }

    if (Settings.enableOrientedBoundingBoxes != 0)
    {
        if (Settings.enableInstanceRebraid)
        {
            const uint numInternalNodes =
                ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET);

            InitObbBlasMetadata(threadId, numInternalNodes, STGB_THREADGROUP_SIZE);
        }

        const uint obbRefitBatchCount =
            ScratchBuffer.Load(ShaderConstants.offsets.obbRefitStackPtrs + STACK_PTRS_OBB_REFIT_BATCH_COUNT);

        // Note, a group of 8 lanes works on a single internal node.
        const uint obbRefitGroupCount = RoundUpQuotient(obbRefitBatchCount,
                                                        REFIT_ORIENTED_BOUNDS3_1_THREADGROUP_SIZE / 8u);
        ScratchBuffer.Store(ShaderConstants.offsets.obbRefitStackPtrs + STACK_PTRS_OBB_REFIT_GROUP_COUNT_X,
                            obbRefitGroupCount);
    }

    PostHwBvhBuild(threadId);
#endif
}
