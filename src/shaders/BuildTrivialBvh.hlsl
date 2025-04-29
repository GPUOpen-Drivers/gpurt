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
#define NUM_THREAD_GROUPS 1
#define NUM_THREADS 16
#define TRIVIAL_BUILDER 1
#define MAIN_THREAD (NUM_THREADS - 1)

#if GPURT_ENABLE_GPU_DEBUG
#define str(a)  #a
#define xstr(a) str(a)

#define DEBUG_BUFFER_SLOT u0
#define DEBUG_BUFFER      "UAV(" xstr(DEBUG_BUFFER_SLOT) "),"
#else
#define DEBUG_BUFFER
#endif

// Note, CBV(b255) must be the last used binding in the root signature.
#define RootSig "RootConstants(num32BitConstants=1, b0),"\
                "CBV(b1),"\
                "SRV(t0),"\
                "SRV(t1),"\
                 DEBUG_BUFFER \
                "DescriptorTable(SRV(t0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 2)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 3)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 4)),"\
                "CBV(b255),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\

#define DISABLE_BUILD_ROOT_SIGNATURE

#if GPURT_BUILD_RTIP3_1

struct RootConstants
{
    uint maxGeometryCount;
};

#include "../shadersClean/common/ShaderDefs.hlsli"

[[vk::push_constant]] ConstantBuffer<RootConstants>                  ShaderRootConstants  : register(b0);
[[vk::binding(1, 1)]] ConstantBuffer<LutData>                        LutBuffer            : register(b1);
[[vk::binding(0, 2)]] StructuredBuffer<BuildShaderConstants>         BuildConstants       : register(t0);
[[vk::binding(1, 2)]] StructuredBuffer<BuildShaderGeometryConstants> GeometryConstants    : register(t1);
// u0 is the DebugBuffer
[[vk::binding(0, 3)]] Buffer<float3>                                 GeometryBuffer[]     : register(t0, space1);
[[vk::binding(0, 4)]] globallycoherent RWByteAddressBuffer           DstBuffers[]         : register(u0, space1);
[[vk::binding(0, 5)]] globallycoherent RWByteAddressBuffer           DstMetadatas[]       : register(u0, space2);
[[vk::binding(0, 6)]] RWByteAddressBuffer                            EmitBuffers[]        : register(u0, space3);
[[vk::binding(0, 7)]] RWByteAddressBuffer                            IndirectArgBuffers[] : register(u0, space4);
// Unmapped buffer
[[vk::binding(0, 0)]] RWByteAddressBuffer                            NullBuffer           : register(u1);
#define ScratchBuffer      NullBuffer
#define ScratchGlobal      NullBuffer
#define InstanceDescBuffer NullBuffer

static globallycoherent RWByteAddressBuffer DstBuffer;
static globallycoherent RWByteAddressBuffer DstMetadata;
static RWByteAddressBuffer EmitBuffer;
static RWByteAddressBuffer IndirectArgBuffer;
static BuildShaderConstants ShaderConstants;

#define SrcBuffer NullBuffer
#include "../shadersClean/build/BuildCommonScratch.hlsli"
#include "CompactCommon.hlsl"
#undef SrcBuffer

groupshared uint SharedMem[0];
uint GetSharedMem(uint index)
{
    return SharedMem[index];
}
void SetSharedMem(uint index, uint value)
{
    SharedMem[index] = value;
}

// The encode path uses SrcBuffer and DstBuffer as the true acceleration structure base.
#define SrcBuffer NullBuffer
#include "EncodeCommon.hlsl"
#undef SrcBuffer

#include "IndirectArgBufferUtils.hlsl"

// Forward declarations for code shared with BuildQBVH.hlsl
InstanceDesc LoadInstanceDesc(uint instanceId, uint offsetInBytes);

// Include implementations for each pass without shader entry points and resource declarations
#define NO_SHADER_ENTRYPOINT 1

#include "../shadersClean/build/EncodeTopLevelCommon.hlsli"
#include "BuildQBVH.hlsl"

#include "QBVH8Common.hlsl"
#include "PrimitiveStructure3_1.hlsl"

//======================================================================================================================
// Initialize headers
void InitAccelerationStructure()
{
    DstMetadata.Store(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, ShaderConstants.resultBufferAddrLo);
    DstMetadata.Store(ACCEL_STRUCT_METADATA_VA_HI_OFFSET, ShaderConstants.resultBufferAddrHi);
    DstMetadata.Store(ACCEL_STRUCT_METADATA_SIZE_OFFSET, ShaderConstants.header.metadataSizeInBytes);

    DstBuffer.Store(0, ShaderConstants.header);
}

//======================================================================================================================
// Calculates the size for compaction in the Trivial Builder
void CalcTrivialCompactedSize()
{
    // In order to rebuild an updateable Acceleration Structure thats compacted we need to report back the original size.
    if (Settings.emitCompactSize != 0)
    {
        EmitBuffer.Store2(0, uint2(ShaderConstants.header.compactedSizeInBytes, 0));
    }
}

//======================================================================================================================
bool LegacyIsDegenerate(TriangleData tri)
{
    float3 edge1 = tri.v0 - tri.v1;
    float3 edge2 = tri.v0 - tri.v2;
    return (any(cross(edge1, edge2)) == false);
}

//======================================================================================================================
// Use to fetch data from IndirectArgBuffer.
NumPrimAndInputOffset LoadInputOffsetsAndNumPrimTrivial(
    in uint geometryStart,
    in uint geometryIndex)
{
    IndirectBuildRangeInfo buildRangeInfo;
    if (Settings.isIndirectBuild)
    {
        buildRangeInfo =
            IndirectArgBuffer.Load<IndirectBuildRangeInfo>(ShaderConstants.indirectArgBufferStride * geometryIndex);
    }
    else
    {
        buildRangeInfo = (IndirectBuildRangeInfo)0;
    }

    BuildShaderGeometryConstants geomConstants = GeometryConstants.Load(geometryStart + geometryIndex);

    return GetInputOffsetsAndNumPrim(geomConstants, geometryIndex, buildRangeInfo);
}

//======================================================================================================================
void FetchTrianglePrimitive(
    uint             flattenedPrimitiveIndex,
    uint             geometryStart,
    out TriangleData tri,
    out BoundingBox  boundingBox,
    out uint         geometryIndex,
    out uint         primitiveIndexWithinGeometryDesc,
    out uint         packedFlags)
{
    // Figure out which geometry desc contains this thread's primitive.
    NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrimTrivial(geometryStart, 0);
    geometryIndex = 0;
    for (uint i = 1; i < ShaderConstants.numDescs; ++i)
    {
        NumPrimAndInputOffset nextInputOffsets = LoadInputOffsetsAndNumPrimTrivial(geometryStart, i);
        if (flattenedPrimitiveIndex < nextInputOffsets.primitiveOffset)
        {
            break;
        }
        geometryIndex = i;
        inputOffsets = nextInputOffsets;
    }

    const uint geometryOffset = geometryStart + geometryIndex;
    const BuildShaderGeometryConstants geometryConstants = GeometryConstants.Load(geometryOffset);

    GPU_ASSERT((inputOffsets.primitiveOffset <= flattenedPrimitiveIndex) &&
               (flattenedPrimitiveIndex < (inputOffsets.primitiveOffset + inputOffsets.numPrimitives)));

    primitiveIndexWithinGeometryDesc = flattenedPrimitiveIndex - inputOffsets.primitiveOffset;

    uint3 indices = uint3(0, 0, 0);
    const bool validIndices =
        FetchTrianglePrimitive(
            geometryConstants,
            inputOffsets,
            GeometryBuffer[NonUniformResourceIndex(geometryOffset)],
            geometryIndex,
            primitiveIndexWithinGeometryDesc,
            tri,
            indices);

    if (validIndices)
    {
        // Generate triangle bounds and update scene bounding box
        boundingBox = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);

        uint instanceMask = 0;
        if (Settings.disableDegenPrims)
        {
            // Set the instance inclusion mask to 0 for degenerate triangles so that they are culled out.
            // Do this before possibly setting tri's vertex to NaN.
            instanceMask = IsDegenerateTriangle(tri) ? 0 : 0xff;

            if (IsActive(tri))
            {
                if ((Settings.disableDegenPrims) && (IsUpdateAllowed() == false) && IsDegenerateTriangle(tri))
                {
                    // Override v0.x for inactive case
                    tri.v0.x = NaN;
                }
            }
            else
            {
                // Override v0.x for inactive case
                tri.v0.x = NaN;
            }
        }
        else
        {
            if (IsActive(tri) == false)
            {
                // Override v0.x for inactive case
                tri.v0.x = NaN;
            }

            // Set the instance inclusion mask to 0 for degenerate triangles so that they are culled out.
            instanceMask = LegacyIsDegenerate(tri) ? 0 : 0xff;
        }

        const uint flags = CalcTriangleBoxNodeFlags(geometryConstants.geometryFlags);

        packedFlags = PackInstanceMaskAndNodeFlags(instanceMask, flags);
    }
    else
    {
        // Override v0.x for inactive case
        tri.v0.x = NaN;
        boundingBox = (BoundingBox) 0;
        packedFlags = 0;
    }

    const uint metadataSize = ShaderConstants.header.metadataSizeInBytes;
    const uint basePrimNodePtr = ShaderConstants.header.offsets.primNodePtrs;
    const uint primNodePointerOffset = metadataSize + basePrimNodePtr + (flattenedPrimitiveIndex * sizeof(uint));
    DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
}
#endif

//======================================================================================================================
[RootSignature(RootSig)]
[numthreads(NUM_THREADS, 1, 1)]
void BuildTrivialBvh(
    uint buildIdx  : SV_GroupID,
    uint threadIdx : SV_GroupThreadID)
{
#if GPURT_BUILD_RTIP3_1
    // Assign global resources
    DstBuffer = DstBuffers[NonUniformResourceIndex(buildIdx)];
    DstMetadata = DstMetadatas[NonUniformResourceIndex(buildIdx)];
    EmitBuffer = EmitBuffers[NonUniformResourceIndex(buildIdx)];
    IndirectArgBuffer = IndirectArgBuffers[NonUniformResourceIndex(buildIdx)];
    ShaderConstants = BuildConstants.Load(buildIdx);

    const uint geometryStart = buildIdx * ShaderRootConstants.maxGeometryCount;
    const uint numPrimitives = ShaderConstants.maxNumPrimitives;
    const uint maxChildNodes = (Settings.enableRebraid) ? 4 : 8;

    BoundingBox rootNodeBbox = {
        float3(FLT_MAX, FLT_MAX, FLT_MAX),
        float3(-FLT_MAX, -FLT_MAX, -FLT_MAX)
    };

    if (threadIdx == MAIN_THREAD)
    {
        InitAccelerationStructure();
    }

    // Write geometry infos.
    if (threadIdx < ShaderConstants.numDescs)
    {
        const NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrimTrivial(geometryStart, threadIdx);
        WriteGeometryInfoForBuildsAndCopies(GeometryConstants.Load(geometryStart + threadIdx),
                                            inputOffsets,
                                            threadIdx,
                                            DECODE_PRIMITIVE_STRIDE_TRIANGLE,
                                            ShaderConstants.header.metadataSizeInBytes,
                                            ShaderConstants.header.offsets.geometryInfo);
    }

    TriangleData laneTri                         = (TriangleData) 0;
    BoundingBox laneTriBbox                      = (BoundingBox) 0;
    uint laneTriPackedFlags                      = 0;
    uint laneTriGeometryIndex                    = 0;
    uint laneTriPrimitiveIndexWithinGeometryDesc = 0;
    BoundingBox laneActiveBbox = {
        float3(FLT_MAX, FLT_MAX, FLT_MAX),
        float3(-FLT_MAX, -FLT_MAX, -FLT_MAX)
    };

    if (threadIdx < numPrimitives)
    {
        FetchTrianglePrimitive(threadIdx,
                               geometryStart,
                               laneTri,
                               laneTriBbox,
                               laneTriGeometryIndex,
                               laneTriPrimitiveIndexWithinGeometryDesc,
                               laneTriPackedFlags);

        if (isnan(laneTri.v0.x) == false)
        {
            // Update the bbox if it's active.
            laneActiveBbox = laneTriBbox;
        }
    }

    // Calculate min/max rootNodeBbox across all active prims.
    rootNodeBbox.min = WaveActiveMin(laneActiveBbox.min);
    rootNodeBbox.max = WaveActiveMax(laneActiveBbox.max);

    const bool isLaneActiveTri = (threadIdx < numPrimitives) && (isnan(laneTri.v0.x) == false);
    const uint waveActivePrimMask = WaveActiveBallot(isLaneActiveTri).x;
    const uint numActivePrims = countbits(waveActivePrimMask);
    GPU_ASSERT(numActivePrims <= (maxChildNodes * 2));

    if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
    {
        for (uint i = 0; i < KDOP_PLANE_COUNT; i++)
        {
            float2 extents = float2(+FLT_MAX, -FLT_MAX);
            if (isLaneActiveTri)
            {
                extents = ComputeTriangleKdopExtents(i, laneTri.v0, laneTri.v1, laneTri.v2);
            }

            const float waveMinExtent = WaveActiveMin(extents.x);
            const float waveMaxExtent = WaveActiveMax(extents.y);

            if (WaveIsFirstLane())
            {
                const uint dstOffset = ACCEL_STRUCT_METADATA_KDOP_OFFSET + (i * sizeof(float2));
                DstMetadata.Store2(dstOffset, uint2(FloatToUint(waveMinExtent), FloatToUint(waveMaxExtent)));
            }
        }
    }

    // numLeafChildren = ceil(numActivePrims / 2)
    const uint numLeafChildren = ((numActivePrims >> 1) + (numActivePrims & 1));

    const uint rootNodePtr = CreateRootNodePointer();

    const uint3 exponents = ComputeCommonExponent(rootNodeBbox.min, rootNodeBbox.max, 12);
    const float3 rcpExponents = ComputeFastExpReciprocal(exponents, 12);

    const AccelStructOffsets offsets = ShaderConstants.header.offsets;

    const uint rootNodeOffset = offsets.internalNodes;
    GPU_ASSERT(CreateBoxNodePointer(rootNodeOffset, false) == rootNodePtr);

    const uint baseBoxNodeOffset = offsets.internalNodes + QBVH8::NodeSizeInBytes;
    const uint baseLeafNodeOffset = baseBoxNodeOffset;

    const bool shouldWriteFilterNode =
        (numActivePrims > 1) &&  // 0,1 prim(s) should just be written as passthrough
        (numActivePrims <= 4) && // More than 4 is non-trivial to compute.
        (Settings.instanceMode == InstanceMode::FilterNode);

    // Write header fields
    if (threadIdx == MAIN_THREAD)
    {
        WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET, numLeafChildren);
        WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET, numActivePrims);
        WriteAccelStructHeaderRootBoundingBox(rootNodeBbox);

        if (shouldWriteFilterNode)
        {
            const uint baseOffset = ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET;
            const uint originOffset = baseOffset + QUANTIZED_BVH4_NODE_OFFSET_ORIGIN;
            DstMetadata.Store3(originOffset, asuint(rootNodeBbox.min));
            const uint packedExpChildIdxAndCount =
                QuantizedBVH8BoxNode::PackExpChildIdxAndCount(exponents, 1, 0, /* validChildCount */ numLeafChildren);
            DstMetadata.Store(baseOffset + QUANTIZED_BVH4_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT,
                              packedExpChildIdxAndCount);
        }
        else
        {
            const uint instanceMask = 0xff;

            WriteInstancePassthrough(rootNodePtr, rootNodeBbox, ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET);
        }

        WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET, 1);
        WriteParentPointer(ShaderConstants.header.metadataSizeInBytes, rootNodePtr, INVALID_IDX);
        QBVH8::WriteInternalNodeBaseOffset(rootNodeOffset, baseBoxNodeOffset);
        QBVH8::WriteLeafNodeBaseOffset(rootNodeOffset, baseLeafNodeOffset);
        QBVH8::WriteParentPointer(rootNodeOffset, INVALID_IDX);
        QBVH8::WriteOrigin(rootNodeOffset, rootNodeBbox.min);
        QBVH8::WritePackedExpChildIdxAndCount(rootNodeOffset, exponents, /* indexInParent */ 0, numLeafChildren);
        QBVH8::WriteObbMatrixIndex(rootNodeOffset, INVALID_OBB);
    }

    const uint activeTriIndex = WavePrefixCountBits(isLaneActiveTri);
    const bool isTri0Lane = (isLaneActiveTri == true) && ((activeTriIndex & 1) == 0);
    const bool hasTri1LaneAvailableToRead = ((activeTriIndex + 1) < numActivePrims);
    const uint tri1LaneIndex = firstbitlow(waveActivePrimMask & ~bits(threadIdx + 1));
    // Sanity check the tri1 lane index.
    GPU_ASSERT((hasTri1LaneAvailableToRead == false) ||
               (countbits(waveActivePrimMask & bits(tri1LaneIndex)) == (activeTriIndex + 1)));

    if (isLaneActiveTri)
    {
        const TriangleData tri0 = laneTri;
        const uint tri0PackedFlags = laneTriPackedFlags;
        const uint tri0PrimitiveIndexWithinGeometryDesc = laneTriPrimitiveIndexWithinGeometryDesc;
        const uint tri0GeometryIndex = laneTriGeometryIndex;
        const BoundingBox tri0Bbox = laneActiveBbox;

        // We have to run WaveReadLaneAt on every lane with an active triangle, even tri1 lanes, because if the tri1
        // lane is an inactive lane and we try to do a WaveReadLaneAt from the tri0 lane, then we end up having missing
        // triangles because the tri0 reads undefined data from the inactive tri1 lane.
        TriangleData tri1 = {
            WaveReadLaneAt(laneTri.v0, tri1LaneIndex),
            WaveReadLaneAt(laneTri.v1, tri1LaneIndex),
            WaveReadLaneAt(laneTri.v2, tri1LaneIndex)
        };
        uint tri1PackedFlags = WaveReadLaneAt(laneTriPackedFlags, tri1LaneIndex);
        uint tri1PrimitiveIndexWithinGeometryDesc =
            WaveReadLaneAt(laneTriPrimitiveIndexWithinGeometryDesc, tri1LaneIndex);
        uint tri1GeometryIndex = WaveReadLaneAt(laneTriGeometryIndex, tri1LaneIndex);

        BoundingBox tri1Bbox = (BoundingBox) 0;

        if (hasTri1LaneAvailableToRead)
        {
            tri1Bbox = GenerateTriangleBoundingBox(tri1.v0, tri1.v1, tri1.v2);
        }
        else
        {
            // If there's no paired tri1 lane, undefined data was returned, and we should immediately overwrite it here.
            tri1.v0 = 0;
            tri1.v1 = 0;
            tri1.v2 = 0;
            tri1PackedFlags = PackInstanceMaskAndNodeFlags(0, 0xff);
            tri1PrimitiveIndexWithinGeometryDesc = 0;

            tri1Bbox.min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
            tri1Bbox.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        }

        if (isTri0Lane)
        {
            const uint nodeType = NODE_TYPE_TRIANGLE_0;
            const uint primRangeLength = 1;

            const uint leafIndex = (activeTriIndex >> 1);
            const uint childNodeOffset = baseLeafNodeOffset + (leafIndex * QBVH8::NodeSizeInBytes);

            const BoundingBox childBounds = {
                min(tri0Bbox.min, tri1Bbox.min),
                max(tri0Bbox.max, tri1Bbox.max)
            };

            const uint instanceMask = ExtractScratchNodeInstanceMask(tri0PackedFlags) |
                                      ExtractScratchNodeInstanceMask(tri1PackedFlags);
            const uint boxNodeFlags = ExtractScratchNodeBoxFlags(tri0PackedFlags) &
                                      ExtractScratchNodeBoxFlags(tri1PackedFlags);

            // Compute packed child info.
            const UintBoundingBox quantBounds =
                ComputeQuantizedBounds(childBounds, rootNodeBbox.min, rcpExponents, 12);

            const uint3 childInfo = ChildInfo::BuildPacked(quantBounds.min,
                                                           quantBounds.max,
                                                           boxNodeFlags,
                                                           instanceMask,
                                                           nodeType,
                                                           primRangeLength);
            QBVH8::WritePackedChildInfo(rootNodeOffset, leafIndex, childInfo);

            const bool isTri0Opaque = (tri0PackedFlags & (1 << BOX_NODE_FLAGS_ONLY_OPAQUE_SHIFT)) != 0;
            const bool isTri1Opaque = (tri1PackedFlags & (1 << BOX_NODE_FLAGS_ONLY_OPAQUE_SHIFT)) != 0;

            PrimStruct3_1::WritePairPrimStruct(tri0,
                                               isTri0Opaque,
                                               tri0PrimitiveIndexWithinGeometryDesc,
                                               tri0GeometryIndex,
                                               hasTri1LaneAvailableToRead,
                                               tri1,
                                               isTri1Opaque,
                                               tri1PrimitiveIndexWithinGeometryDesc,
                                               tri1GeometryIndex,
                                               childNodeOffset);

            DstBuffer.Store(offsets.primNodePtrs + (leafIndex * sizeof(uint)), childNodeOffset);

            if (shouldWriteFilterNode)
            {
                const uint3 headerNodeChildInfo =
                    GetInstanceChildInfo(leafIndex, CreateRootNodePointer3_1(), quantBounds.min, quantBounds.max);

                GPU_ASSERT(leafIndex < 4);
                DstMetadata.Store3(ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET + GetBvh4ChildInfoOffset(leafIndex),
                                   headerNodeChildInfo);
            }
        }
    }

    if ((numLeafChildren <= threadIdx) && (threadIdx < 8))
    {
        // This BVH8 child doesn't have a primitive to write, so we write invalid data.
        QBVH8::WriteInvalidChildInfo(rootNodeOffset, threadIdx);
        if (shouldWriteFilterNode && (threadIdx < 4))
        {
            const uint3 childInfo = GetInstanceChildInfo(threadIdx, CreateRootNodePointer3_1(), 0xFFFFFFFF.xxx, 0.xxx);
            DstMetadata.Store3(ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET + GetBvh4ChildInfoOffset(threadIdx),
                               childInfo);
        }
    }

    if (threadIdx == MAIN_THREAD)
    {
        CalcTrivialCompactedSize();
    }
#endif
}
