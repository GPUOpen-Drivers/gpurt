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
//
//=====================================================================================================================
// The functions defined below require the following variables/resources defined before including this header
//
// uint numActivePrims

#ifndef _BUILDCOMMON_HLSL
#define _BUILDCOMMON_HLSL

#include "IntersectCommon.hlsl"
#include "MortonCodes.hlsl"
#include "../shadersClean/common/Math.hlsli"

//=====================================================================================================================
#define LEAFIDX(i) ((numActivePrims-1) + (i))
#define NODEIDX(i) (i)
#define IS_LEAF(i) ((i) >= (numActivePrims - 1))
#define LEAF_OFFSET(i) ((i) - (numActivePrims - 1))

#if GPURT_BUILD_RTIP3_1
#define RTIP3_1_UPDATE_THREADS_PER_FAT_LEAF 8
#define UPDATE3_1_THREADGROUP_SIZE          32
#endif

//=====================================================================================================================
struct RefitArgs
{
    uint topLevelBuild;
    uint numActivePrims;
    uint baseScratchNodesOffset;
    uint enablePairCompression;
    uint enablePairCostCheck;
    uint numBatchesOffset;
    uint baseBatchIndicesOffset;
    uint fp16BoxNodesMode;
    float fp16BoxModeMixedSaThreshold;
    uint unsortedNodesBaseOffset;
    uint enableEarlyPairCompression;
};

//=====================================================================================================================
static float3x4 CreateMatrix(float4 rows[3])
{
    float3x4 mat;
    mat[0] = rows[0];
    mat[1] = rows[1];
    mat[2] = rows[2];
    return mat;
}

//=====================================================================================================================
// Extract node flags from hardware box node flags
uint ExtractBoxNodeFlagsField(
    uint nodeFlags, uint childIndex)
{
    return (nodeFlags >> (childIndex * BOX_NODE_FLAGS_BIT_STRIDE)) & 0xFF;
}

//=====================================================================================================================
// Clear node flags field from hardware box node flags
uint ClearBoxNodeFlagsField(
    uint nodeFlags, uint childIndex)
{
    return nodeFlags & ~(0xFFu << (childIndex * BOX_NODE_FLAGS_BIT_STRIDE));
}

//=====================================================================================================================
// Set specific node flags field in hardware box node flags
uint SetBoxNodeFlagsField(
    uint nodeFlags, uint childFlags, uint childIndex)
{
    return nodeFlags | (childFlags << (childIndex * BOX_NODE_FLAGS_BIT_STRIDE));
}

//=====================================================================================================================
uint64_t PackUint32x4ToUint64(uint4 v, uint4 numBits)
{
    uint64_t r = uint64_t(v[3]);
    r = ((r << numBits[2]) | uint64_t(v[2]));
    r = ((r << numBits[1]) | uint64_t(v[1]));
    r = ((r << numBits[0]) | uint64_t(v[0]));
    return r;
}

//=====================================================================================================================
uint4 UnpackUint64ToUint32x4(uint64_t v, uint4 numBits)
{
    uint4 r;
    r[0] = uint(v & ((1ULL << numBits[0]) - 1));
    v >>= numBits[0];
    r[1] = uint(v & ((1ULL << numBits[1]) - 1));
    v >>= numBits[1];
    r[2] = uint(v & ((1ULL << numBits[2]) - 1));
    v >>= numBits[2];
    r[3] = uint(v);
    return r;
}

//=====================================================================================================================
uint CalcQbvhInternalNodeIndex(
    uint offset,
    bool useFp16BoxNodesInBlas)
{
    // Node offset includes header size
    offset -= sizeof(AccelStructHeader);

#if GPURT_BUILD_RTIP3
    if (Settings.highPrecisionBoxNodeEnable)
    {
        return (offset >> BVH4_NODE_HIGH_PRECISION_STRIDE_SHIFT);
    }
    else
#endif
    {
        return useFp16BoxNodesInBlas ? (offset >> BVH4_NODE_16_STRIDE_SHIFT)
                                     : (offset >> BVH4_NODE_32_STRIDE_SHIFT);
    }
}

//=====================================================================================================================
BoundingBox GenerateBoxNode16BoundingBox(
    in Float16BoxNode node)
{
    BoundingBox bbox0;
    if (node.child0 == INVALID_IDX)
    {
        bbox0.min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
        bbox0.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    }
    else
    {
        bbox0 = UncompressBBoxFromUint3(node.bbox0);
    }

    BoundingBox bbox1;
    if (node.child1 == INVALID_IDX)
    {
        bbox1.min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
        bbox1.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    }
    else
    {
        bbox1 = UncompressBBoxFromUint3(node.bbox1);
    }

    BoundingBox bbox2;
    if (node.child2 == INVALID_IDX)
    {
        bbox2.min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
        bbox2.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    }
    else
    {
        bbox2 = UncompressBBoxFromUint3(node.bbox2);
    }

    BoundingBox bbox3;
    if (node.child3 == INVALID_IDX)
    {
        bbox3.min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
        bbox3.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    }
    else
    {
        bbox3 = UncompressBBoxFromUint3(node.bbox3);
    }

    BoundingBox bbox;
    bbox.min = min(min(min(bbox0.min, bbox1.min), bbox2.min), bbox3.min);
    bbox.max = max(max(max(bbox0.max, bbox1.max), bbox2.max), bbox3.max);

    return bbox;
}

//=====================================================================================================================
BoundingBox GenerateBoxNode32BoundingBox(
    in Float32BoxNode node)
{
    if (node.child0 == INVALID_IDX)
    {
        node.bbox0_min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
        node.bbox0_max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    }

    if (node.child1 == INVALID_IDX)
    {
        node.bbox1_min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
        node.bbox1_max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    }

    if (node.child2 == INVALID_IDX)
    {
        node.bbox2_min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
        node.bbox2_max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    }

    if (node.child3 == INVALID_IDX)
    {
        node.bbox3_min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
        node.bbox3_max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    }

    BoundingBox bbox;
    bbox.min = min(min(min(node.bbox0_min, node.bbox1_min), node.bbox2_min), node.bbox3_min);
    bbox.max = max(max(max(node.bbox0_max, node.bbox1_max), node.bbox2_max), node.bbox3_max);

    return bbox;
}

//=====================================================================================================================
BoundingBox TransformBoundingBox(
    in BoundingBox box,
    in float4      inputTransform[3])
{
    // convert to center/extents box representation
    float3 center  = (box.max + box.min) * 0.5;
    float3 extents = (box.max - center);

    float4x4 transform;
    transform[0] = inputTransform[0];
    transform[1] = inputTransform[1];
    transform[2] = inputTransform[2];
    transform[3] = float4(0, 0, 0, 1);

    // transform center
    float3 transformedCenter = mul(transform, float4(center, 1.0)).xyz;

    // In order to maximize the 8 resulting dot products, it is sufficient to take the dot product of the extents
    // with the absolute value of the matrix elements.
    // In order to minimize the 8 resulting dot products, it is sufficient to negate the maximum dot product
    //
    // transform extents (take absolute)
    float3x3 absMat = float3x3(abs(inputTransform[0].xyz), abs(inputTransform[1].xyz), abs(inputTransform[2].xyz));
    float3 transformedExtents = mul(absMat, extents);

    // convert back to min/max box representation
    float3 min = transformedCenter - transformedExtents;
    float3 max = transformedCenter + transformedExtents;

    BoundingBox transformedBbox;

    transformedBbox.min = min;
    transformedBbox.max = max;

    return transformedBbox;
}

//=====================================================================================================================
// Overload of TransformBoundingBox ^ with a float3x4 as the type of the inputTransform, instead of float4[3]
BoundingBox TransformBoundingBox(
    in BoundingBox box,
    in float3x4    inputTransform)
{
    float4 arrayTransform[3] = {inputTransform[0], inputTransform[1], inputTransform[2]};
    return TransformBoundingBox(box, arrayTransform);
}

//=====================================================================================================================
BoundingBox GenerateInstanceBoundingBox(
    in float4      instanceTransform[3],
    in BoundingBox blasRootBounds)
{
    BoundingBox instanceBbox = TransformBoundingBox(blasRootBounds, instanceTransform);

    if (any(isinf(instanceBbox.min)) || any(isinf(instanceBbox.max)))
    {
        instanceBbox = InvalidBoundingBox;
    }

    return instanceBbox;
}

//=====================================================================================================================
uint3 Float3ToUint3(in float3 v)
{
    uint3 ui = uint3(FloatToUint(v.x), FloatToUint(v.y), FloatToUint(v.z));

    return ui;
}

//=====================================================================================================================
int FloatToInt(float v)
{
    // Don't invert the sign bit when converting to signed integers. Negative floats will correctly compare as smaller
    // than positive. We still need to invert the remaining bits since higher values there are smaller floats.
    return asint(asuint(v) ^ (1 + ~(asuint(v) >> 31)));
}

//=====================================================================================================================
float UintToFloat(uint v)
{
    const uint bitShift = 31;
    const uint bitMask = 0x80000000;

    v ^= (((v >> bitShift) - 1) | bitMask);

    return asfloat(v);
}

//=====================================================================================================================
float2 Uint2ToFloat2(in uint2 v)
{
    return float2(UintToFloat(v.x), UintToFloat(v.y));
}

//=====================================================================================================================
float3 Uint3ToFloat3(in uint3 v)
{
    return float3(UintToFloat(v.x), UintToFloat(v.y), UintToFloat(v.z));
}

//=====================================================================================================================
static uint32_t GetNumInternalNodeCount(
    in uint32_t primitiveCount)
{
    uint32_t maxNumChildren = 4u;
    uint32_t minPrimsPerLastInternalNode = 2u;

    {
#if GPURT_BUILD_RTIP3
#if GPURT_BUILD_RTIP3_1
        // When pairs are enabled (range size at least 2), there are at least 3 primitives (one pair and one single
        // triangle) in a last level internal node.
        minPrimsPerLastInternalNode = (Settings.maxPrimRangeSize >= 2) ? 3u : 2u;
#endif
        maxNumChildren = Settings.bvh8Enable ? 8u : 4u;
#endif
    }

    return CalcAccelStructInternalNodeCount(primitiveCount, maxNumChildren, minPrimsPerLastInternalNode);
}

#if GPURT_BUILD_RTIP3
//=====================================================================================================================
static bool IsBoxNode(uint pointer)
{
    return IsBoxNode(pointer,
                     Settings.highPrecisionBoxNodeEnable,
                     Settings.bvh8Enable
#if GPURT_BUILD_RTIP3_1
                   , EnableCompressedFormat()
#endif
                    );
}

//=====================================================================================================================
static uint CreateRootNodePointer()
{
    return CreateRootNodePointer(Settings.highPrecisionBoxNodeEnable,
                                 Settings.bvh8Enable
#if GPURT_BUILD_RTIP3_1
                               , EnableCompressedFormat()
#endif
                                );
}
#endif

//=====================================================================================================================
uint ReadParentPointer(
    in uint             metadataSizeInBytes,
    in uint             nodePtr)
{
    // Read parent pointer in metadata buffer in reverse order
    return SrcBuffer.Load(metadataSizeInBytes - CalcParentPtrOffset(nodePtr));
}

//=====================================================================================================================
void WriteParentPointer(
    in uint             metadataSizeInBytes,
    in uint             nodePtr,
    in uint             parentPtr)
{
    // Store parent pointer in metadata buffer in reverse order
    DstMetadata.Store(metadataSizeInBytes - CalcParentPtrOffset(nodePtr), parentPtr);
}

//=====================================================================================================================
uint GetBlasRebraidChildCount(
    uint64_t blasBasePtr,
    uint     nodePointer)
{
    const uint64_t blasBaseAddr = ExtractInstanceAddr(blasBasePtr);

    uint count = 0;

#if GPURT_BUILD_RTIP3_1
    if (EnableCompressedFormat())
    {
        count = GetQuantizedBoxRebraidChildCount(blasBaseAddr, nodePointer);
    }
    else
#endif
    if (
#if GPURT_BUILD_RTIP3
        !Settings.highPrecisionBoxNodeEnable &&
#endif
        IsBoxNode16(nodePointer))
    {
        const Float16BoxNode node = FetchFloat16BoxNode(blasBaseAddr, nodePointer);
        count = 1 + (node.child1 != INVALID_IDX) + (node.child2 != INVALID_IDX) + (node.child3 != INVALID_IDX);
    }
    else
    {
        const Float32BoxNode node = FetchFloat32BoxNode(blasBaseAddr,
#if GPURT_BUILD_RTIP3
                                                        Settings.highPrecisionBoxNodeEnable,
#endif
                                                        nodePointer);
        count = 1 + (node.child1 != INVALID_IDX) + (node.child2 != INVALID_IDX) + (node.child3 != INVALID_IDX);
    }

    return count;
}

//=====================================================================================================================
uint GetInternalNodeType()
{
#if GPURT_BUILD_RTIP3
    uint nodeType;

#if GPURT_BUILD_RTIP3_1
    if (EnableCompressedFormat())
    {
        nodeType = NODE_TYPE_BOX_QUANTIZED_BVH8;
    }
    else
#endif
    {
        nodeType = Settings.highPrecisionBoxNodeEnable ? NODE_TYPE_BOX_HP64 : NODE_TYPE_BOX_FLOAT32;

        if (Settings.bvh8Enable)
        {
            nodeType = Settings.highPrecisionBoxNodeEnable ? NODE_TYPE_BOX_HP64x2 : NODE_TYPE_BOX_FLOAT32x2;
        }
    }

    return nodeType;
#else
    return NODE_TYPE_BOX_FLOAT32;
#endif
}

//=====================================================================================================================
static Float32BoxNode FetchFloat32BoxNode(
#if GPURT_BUILD_RTIP3
    bool                highPrecisionBoxNodeEnable,
#endif
    in uint             offset)
{
#if GPURT_BUILD_RTIP3
    if (highPrecisionBoxNodeEnable)
    {

        HighPrecisionBoxNode hpBoxNode = (HighPrecisionBoxNode)0;

        uint4 d0, d1, d2, d3;
        d0 = DstMetadata.Load4(offset);
        d1 = DstMetadata.Load4(offset + 0x10);
        d2 = DstMetadata.Load4(offset + 0x20);
        d3 = DstMetadata.Load4(offset + 0x30);

        hpBoxNode.dword[0] = d0.x;
        hpBoxNode.dword[1] = d0.y;
        hpBoxNode.dword[2] = d0.z;
        hpBoxNode.dword[3] = d0.w;
        hpBoxNode.dword[4] = d1.x;
        hpBoxNode.dword[5] = d1.y;
        hpBoxNode.dword[6] = d1.z;
        hpBoxNode.dword[7] = d1.w;
        hpBoxNode.dword[8] = d2.x;
        hpBoxNode.dword[9] = d2.y;
        hpBoxNode.dword[10] = d2.z;
        hpBoxNode.dword[11] = d2.w;
        hpBoxNode.dword[12] = d3.x;
        hpBoxNode.dword[13] = d3.y;
        hpBoxNode.dword[14] = d3.z;
        hpBoxNode.dword[15] = d3.w;

        return DecodeHighPrecisionBoxNode(hpBoxNode);
    }
#endif
    ///

    uint4 d0, d1, d2, d3, d4, d5, d6, d7;
    d0 = DstMetadata.Load4(offset);
    d1 = DstMetadata.Load4(offset + 0x10);
    d2 = DstMetadata.Load4(offset + 0x20);
    d3 = DstMetadata.Load4(offset + 0x30);
    d4 = DstMetadata.Load4(offset + 0x40);
    d5 = DstMetadata.Load4(offset + 0x50);
    d6 = DstMetadata.Load4(offset + 0x60);
    d7 = DstMetadata.Load4(offset + 0x70);

    Float32BoxNode fp32BoxNode = (Float32BoxNode)0;
    fp32BoxNode.child0 = d0.x;
    fp32BoxNode.child1 = d0.y;
    fp32BoxNode.child2 = d0.z;
    fp32BoxNode.child3 = d0.w;
    fp32BoxNode.bbox0_min = asfloat(uint3(d1.x, d1.y, d1.z));
    fp32BoxNode.bbox0_max = asfloat(uint3(d1.w, d2.x, d2.y));
    fp32BoxNode.bbox1_min = asfloat(uint3(d2.z, d2.w, d3.x));
    fp32BoxNode.bbox1_max = asfloat(uint3(d3.y, d3.z, d3.w));
    fp32BoxNode.bbox2_min = asfloat(uint3(d4.x, d4.y, d4.z));
    fp32BoxNode.bbox2_max = asfloat(uint3(d4.w, d5.x, d5.y));
    fp32BoxNode.bbox3_min = asfloat(uint3(d5.z, d5.w, d6.x));
    fp32BoxNode.bbox3_max = asfloat(uint3(d6.y, d6.z, d6.w));
    fp32BoxNode.flags         = d7.x;
    fp32BoxNode.numPrimitives = d7.y;
#if GPURT_BUILD_RTIP3_1
    // RTIP3.1 also implies RTIP3.0. These bits are not used but are needed for compiling when RTIP3_1 is enabled.
    fp32BoxNode.instanceMask  = d7.z;
    fp32BoxNode.obbMatrixIndex = d7.w;
#else
    fp32BoxNode.padding2      = d7.z;
    fp32BoxNode.padding3      = d7.w;
#endif

    return fp32BoxNode;
}

//=====================================================================================================================
uint GetInstanceMask(InstanceDesc desc)
{
    return desc.InstanceID_and_Mask >> 24;
}

#if GPURT_BUILD_RTIP3
//=====================================================================================================================
void WriteInstanceNode3_0(
    in InstanceDesc instanceDesc,
    in uint         geometryType,
    in uint         instanceIndex,
    in uint         instNodePtr,
    in uint         blasRootNodePointer,
    in uint         blasMetadataSize,
    in uint         tlasMetadataSize)
{
    GpuVirtualAddress blasBaseAddr = MakeGpuVirtualAddress(instanceDesc.accelStructureAddressLo,
                                                           instanceDesc.accelStructureAddressHiAndFlags);
    blasBaseAddr += blasMetadataSize;

    const uint64_t childBasePtr =
        PackInstanceBasePointer(blasBaseAddr,
                                instanceDesc.InstanceContributionToHitGroupIndex_and_Flags >> 24,
                                geometryType);

    const uint instanceNodeOffset = tlasMetadataSize + ExtractNodePointerOffset(instNodePtr);

    HwInstanceTransformNode hwInstanceNode = (HwInstanceTransformNode)0;

    hwInstanceNode.childBasePtr = childBasePtr;
    hwInstanceNode.childRootNodeOrParentPtr = blasRootNodePointer;
    hwInstanceNode.userDataAndInstanceMask =
        (instanceDesc.InstanceContributionToHitGroupIndex_and_Flags & 0x00FFFFFF) |
        (instanceDesc.InstanceID_and_Mask & 0xFF000000);

    // Invert instance to world matrix
    float3x4 temp = CreateMatrix(instanceDesc.Transform);
    float3x4 inverse = Inverse3x4(temp);

    hwInstanceNode.worldToObject[0][0] = inverse[0].x;
    hwInstanceNode.worldToObject[0][1] = inverse[0].y;
    hwInstanceNode.worldToObject[0][2] = inverse[0].z;
    hwInstanceNode.worldToObject[0][3] = inverse[0].w;
    hwInstanceNode.worldToObject[1][0] = inverse[1].x;
    hwInstanceNode.worldToObject[1][1] = inverse[1].y;
    hwInstanceNode.worldToObject[1][2] = inverse[1].z;
    hwInstanceNode.worldToObject[1][3] = inverse[1].w;
    hwInstanceNode.worldToObject[2][0] = inverse[2].x;
    hwInstanceNode.worldToObject[2][1] = inverse[2].y;
    hwInstanceNode.worldToObject[2][2] = inverse[2].z;
    hwInstanceNode.worldToObject[2][3] = inverse[2].w;

    DstMetadata.Store<HwInstanceTransformNode>(instanceNodeOffset, hwInstanceNode);

    InstanceSidebandData sideband = (InstanceSidebandData)0;
    sideband.instanceIndex      = instanceIndex;
    sideband.instanceIdAndFlags =
        (instanceDesc.InstanceID_and_Mask & 0x00FFFFFF) |
        (instanceDesc.InstanceContributionToHitGroupIndex_and_Flags & 0xFF000000);
    sideband.blasMetadataSize = blasMetadataSize;

    sideband.objectToWorld[0][0] = instanceDesc.Transform[0].x;
    sideband.objectToWorld[0][1] = instanceDesc.Transform[0].y;
    sideband.objectToWorld[0][2] = instanceDesc.Transform[0].z;
    sideband.objectToWorld[0][3] = instanceDesc.Transform[0].w;
    sideband.objectToWorld[1][0] = instanceDesc.Transform[1].x;
    sideband.objectToWorld[1][1] = instanceDesc.Transform[1].y;
    sideband.objectToWorld[1][2] = instanceDesc.Transform[1].z;
    sideband.objectToWorld[1][3] = instanceDesc.Transform[1].w;
    sideband.objectToWorld[2][0] = instanceDesc.Transform[2].x;
    sideband.objectToWorld[2][1] = instanceDesc.Transform[2].y;
    sideband.objectToWorld[2][2] = instanceDesc.Transform[2].z;
    sideband.objectToWorld[2][3] = instanceDesc.Transform[2].w;

    const uint sidebandOffset = instanceNodeOffset + sizeof(HwInstanceTransformNode);
    DstMetadata.Store<InstanceSidebandData>(sidebandOffset, sideband);
}
#endif

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
// Load internal node child info from the specified node address
static ChildInfo LoadChildInfo(
    uint64_t nodeAddr,
    uint     childIndex,
    uint     validChildCount)
{
    const uint childOffset = GetChildInfoOffset(childIndex);
    const uint3 packedChildInfo = LoadDwordAtAddrx3(nodeAddr + childOffset);
    ChildInfo childInfo;
    if (childIndex < validChildCount)
    {
        childInfo.Load(packedChildInfo);
    }
    else
    {
        childInfo.Invalidate();
    }
    return childInfo;
}

#define PACKED_SOURCE_BOX_BIT_SHIFT 16u
#define PACKED_SOURCE_BOX_BIT_MASK  (0xffu << PACKED_SOURCE_BOX_BIT_SHIFT)

//=====================================================================================================================
uint PackObbMatrixIndexBits(
    uint obbMatrixIdx,
    uint packedSourceBoxes)
{
    // Store the source box bitfield in the upper bits of the OBB DWORD (unused by HW). This is used to construct
    // four filter boxes for instances.
    return obbMatrixIdx | ((packedSourceBoxes >> 8) << PACKED_SOURCE_BOX_BIT_SHIFT);
}

//=====================================================================================================================
// Returns unpacked OBB matrix index and packedSourceBoxes field.
uint ExtractPackedSourceBoxBits(
    uint packedObbMatrixIdx)
{
    return (packedObbMatrixIdx & PACKED_SOURCE_BOX_BIT_MASK) >> PACKED_SOURCE_BOX_BIT_SHIFT;
}

//=====================================================================================================================
void WriteRebraidFilterNode3_1(
    uint64_t blasBaseAddr,
    uint     blasRootNodePointer,
    uint     boxNodeDstOffset)
{
    const uint64_t nodeAddr = GetBlasAabbNodeAddr(blasBaseAddr, blasRootNodePointer);

    // Load original origin
    const uint3 origin = LoadDwordAtAddrx3(nodeAddr + QUANTIZED_BVH8_NODE_OFFSET_ORIGIN);

    // Load original exponents
    uint expChildIndxAndCount = LoadDwordAtAddr(nodeAddr + QUANTIZED_BVH8_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT);

    const uint validChildCount = ExtractValidChildCount(expChildIndxAndCount);

    // Set instance child count to 4
    expChildIndxAndCount = bitFieldInsert(expChildIndxAndCount, 28, 3, 4 - 1);

    DstMetadata.Store4(boxNodeDstOffset + QUANTIZED_BVH4_NODE_OFFSET_ORIGIN,
                       uint4(origin, expChildIndxAndCount));

    // Repurposed bits [23:16] of the OBB DWORD specify which of the 4 leftmost boxes shares a common parent with each
    // of the 4 rightmost boxes. The bitfield contains 2 bits per child 4-7 in the node.
    //
    // To constuct the 4 filter node boxes, all boxes with the same source box index should be merged.
    const uint packedSourceBoxes =
        ExtractPackedSourceBoxBits(LoadDwordAtAddr(nodeAddr + QUANTIZED_BVH8_NODE_OFFSET_OBB_MATRIX_INDEX));

    [unroll]
    for (uint i = 0; i < 4; ++i)
    {
        const ChildInfo childInfo0 = LoadChildInfo(nodeAddr, i, validChildCount);

        uint3 quantizedMin = childInfo0.Min();
        uint3 quantizedMax = childInfo0.Max();

        // Find the children sharing the same parent as the ith child and merge their boxes
        [unroll]
        for (uint j = 0; j < 4; ++j)
        {
            if (((packedSourceBoxes >> (j * 2)) & 0b11) == i)
            {
                const ChildInfo childInfo = LoadChildInfo(nodeAddr, j + 4, validChildCount);

                quantizedMin = min(quantizedMin, childInfo.Min());
                quantizedMax = max(quantizedMax, childInfo.Max());
            }
        }

        const uint3 mergedChildInfo = GetInstanceChildInfo(i, blasRootNodePointer, quantizedMin, quantizedMax);
        DstMetadata.Store3(boxNodeDstOffset + GetBvh4ChildInfoOffset(i), mergedChildInfo);
    }
}

//=====================================================================================================================
void WriteInstanceNode3_1(
    in InstanceDesc       instanceDesc,
    in uint               geometryType,
    in uint               instanceIndex,
    in uint               instNodePtr,
    in uint               blasRootNodePointer,
    in uint               blasMetadataSize,
    in AccelStructOffsets offsets,
    in uint               parentNodePointer)
{
    GpuVirtualAddress blasBaseAddr = MakeGpuVirtualAddress(instanceDesc.accelStructureAddressLo,
                                                           instanceDesc.accelStructureAddressHiAndFlags);
    blasBaseAddr += blasMetadataSize;

    const uint64_t childBasePtr =
        PackInstanceBasePointer(blasBaseAddr,
                                instanceDesc.InstanceContributionToHitGroupIndex_and_Flags >> 24,
                                geometryType);

    const uint instanceNodeOffset = ExtractNodePointerOffset(instNodePtr);

    uint tlasMetadataSize = 0;
    if (IsUpdate())
    {
        tlasMetadataSize = SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET);
    }

    HwInstanceTransformNode hwInstanceNode = (HwInstanceTransformNode)0;

    hwInstanceNode.childBasePtr             = childBasePtr;
    hwInstanceNode.childRootNodeOrParentPtr = parentNodePointer;
    hwInstanceNode.userDataAndInstanceMask  =
        (instanceDesc.InstanceContributionToHitGroupIndex_and_Flags & 0x00FFFFFF) |
        (instanceDesc.InstanceID_and_Mask & 0xFF000000);

    // Invert instance to world matrix
    float3x4 temp = CreateMatrix(instanceDesc.Transform);
    float3x4 inverse = Inverse3x4(temp);

    hwInstanceNode.worldToObject[0][0] = inverse[0].x;
    hwInstanceNode.worldToObject[0][1] = inverse[0].y;
    hwInstanceNode.worldToObject[0][2] = inverse[0].z;
    hwInstanceNode.worldToObject[0][3] = inverse[0].w;
    hwInstanceNode.worldToObject[1][0] = inverse[1].x;
    hwInstanceNode.worldToObject[1][1] = inverse[1].y;
    hwInstanceNode.worldToObject[1][2] = inverse[1].z;
    hwInstanceNode.worldToObject[1][3] = inverse[1].w;
    hwInstanceNode.worldToObject[2][0] = inverse[2].x;
    hwInstanceNode.worldToObject[2][1] = inverse[2].y;
    hwInstanceNode.worldToObject[2][2] = inverse[2].z;
    hwInstanceNode.worldToObject[2][3] = inverse[2].w;

    if (IsUpdate())
    {
        DstMetadata.Store<HwInstanceTransformNode>(tlasMetadataSize + instanceNodeOffset, hwInstanceNode);
    }
    else
    {
        DstBuffer.Store<HwInstanceTransformNode>(instanceNodeOffset, hwInstanceNode);
    }

    InstanceSidebandData sideband = (InstanceSidebandData)0;
    sideband.instanceIndex      = instanceIndex;
    sideband.instanceIdAndFlags =
        (instanceDesc.InstanceID_and_Mask & 0x00FFFFFF) |
        (instanceDesc.InstanceContributionToHitGroupIndex_and_Flags & 0xFF000000);
    sideband.blasMetadataSize = blasMetadataSize;

    sideband.objectToWorld[0][0] = instanceDesc.Transform[0].x;
    sideband.objectToWorld[0][1] = instanceDesc.Transform[0].y;
    sideband.objectToWorld[0][2] = instanceDesc.Transform[0].z;
    sideband.objectToWorld[0][3] = instanceDesc.Transform[0].w;
    sideband.objectToWorld[1][0] = instanceDesc.Transform[1].x;
    sideband.objectToWorld[1][1] = instanceDesc.Transform[1].y;
    sideband.objectToWorld[1][2] = instanceDesc.Transform[1].z;
    sideband.objectToWorld[1][3] = instanceDesc.Transform[1].w;
    sideband.objectToWorld[2][0] = instanceDesc.Transform[2].x;
    sideband.objectToWorld[2][1] = instanceDesc.Transform[2].y;
    sideband.objectToWorld[2][2] = instanceDesc.Transform[2].z;
    sideband.objectToWorld[2][3] = instanceDesc.Transform[2].w;

    // Add additional buffer offset here to account for Update.
    // Note, leafNodes and geometryInfo offsets do not account for
    // metadata size
    const uint sidebandOffset = ComputeInstanceSidebandOffset(instanceNodeOffset,
                                                              offsets.leafNodes,
                                                              offsets.geometryInfo);

    if (IsUpdate())
    {
        DstMetadata.Store<InstanceSidebandData>(tlasMetadataSize + sidebandOffset, sideband);
    }
    else
    {
        DstBuffer.Store<InstanceSidebandData>(sidebandOffset, sideband);
    }

    const uint boxNodeOffset = instanceNodeOffset + sizeof(HwInstanceTransformNode);
    if ((Settings.enableRebraid == false) || (blasRootNodePointer == CreateRootNodePointer()))
    {
        // Copy pre-built intersectable instance node data from BLAS metadata to TLAS
        const uint64_t blasMetadataBaseAddr = blasBaseAddr - blasMetadataSize;
        uint4 d0 = LoadDwordAtAddrx4(blasMetadataBaseAddr + ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET);
        uint4 d1 = LoadDwordAtAddrx4(blasMetadataBaseAddr + ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET + 16);
        uint4 d2 = LoadDwordAtAddrx4(blasMetadataBaseAddr + ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET + 32);
        uint4 d3 = LoadDwordAtAddrx4(blasMetadataBaseAddr + ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET + 48);

        if (IsUpdate())
        {
            DstMetadata.Store4(tlasMetadataSize + boxNodeOffset,      d0);
            DstMetadata.Store4(tlasMetadataSize + boxNodeOffset + 16, d1);
            DstMetadata.Store4(tlasMetadataSize + boxNodeOffset + 32, d2);
            DstMetadata.Store4(tlasMetadataSize + boxNodeOffset + 48, d3);
        }
        else
        {
            DstBuffer.Store4(boxNodeOffset,      d0);
            DstBuffer.Store4(boxNodeOffset + 16, d1);
            DstBuffer.Store4(boxNodeOffset + 32, d2);
            DstBuffer.Store4(boxNodeOffset + 48, d3);
        }
    }
    else if ((Settings.instanceMode == InstanceMode::Passthrough) || IsTriangleNode3_1(blasRootNodePointer))
    {
        if (IsUpdate() == false)
        {
            tlasMetadataSize = DstMetadata.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET);
        }

        // Compute instance bounds from rebraided child
        const BoundingBox instanceBounds = DecodeRebraidChildBoundsBVH4(blasBaseAddr, blasRootNodePointer);

        WriteInstancePassthrough(blasRootNodePointer, instanceBounds, tlasMetadataSize + boxNodeOffset);
    }
    else if (Settings.instanceMode == InstanceMode::FilterNode)
    {
        if (IsUpdate() == false)
        {
            tlasMetadataSize = DstMetadata.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET);
        }

        WriteRebraidFilterNode3_1(blasBaseAddr, blasRootNodePointer, tlasMetadataSize + boxNodeOffset);
    }
}
#endif

//=====================================================================================================================
void WriteInstanceNode1_1(
    in InstanceDesc instanceDesc,
    in uint         geometryType,
    in uint         instanceIndex,
    in uint         instNodePtr,
    in uint         blasRootNodePointer,
    in uint         blasMetadataSize,
    in uint         tlasMetadataSize)
{
    GpuVirtualAddress blasBaseAddr = MakeGpuVirtualAddress(instanceDesc.accelStructureAddressLo,
                                                           instanceDesc.accelStructureAddressHiAndFlags);
    blasBaseAddr += blasMetadataSize;

    const uint64_t childBasePtr =
        PackInstanceBasePointer(blasBaseAddr,
                                instanceDesc.InstanceContributionToHitGroupIndex_and_Flags >> 24,
                                geometryType);

    const uint instanceNodeOffset = tlasMetadataSize + ExtractNodePointerOffset(instNodePtr);

    InstanceNode node = (InstanceNode)0;

    node.desc.InstanceID_and_Mask                           = instanceDesc.InstanceID_and_Mask;
    node.desc.InstanceContributionToHitGroupIndex_and_Flags = instanceDesc.InstanceContributionToHitGroupIndex_and_Flags;
    node.desc.accelStructureAddressLo                       = LowPart(childBasePtr);
    node.desc.accelStructureAddressHiAndFlags               = HighPart(childBasePtr);

    node.sideband.Transform[0]     = instanceDesc.Transform[0];
    node.sideband.Transform[1]     = instanceDesc.Transform[1];
    node.sideband.Transform[2]     = instanceDesc.Transform[2];
    node.sideband.instanceIndex    = instanceIndex;
    node.sideband.padding0         = 0;
    node.sideband.blasMetadataSize = blasMetadataSize;
    node.sideband.blasNodePointer  = blasRootNodePointer;

    // invert matrix
    float3x4 temp    = CreateMatrix(instanceDesc.Transform);
    float3x4 inverse = Inverse3x4(temp);

    node.desc.Transform[0] = inverse[0];
    node.desc.Transform[1] = inverse[1];
    node.desc.Transform[2] = inverse[2];

    DstMetadata.Store<InstanceNode>(instanceNodeOffset, node);

    // Write bottom-level root box node into fused instance node
    if (Settings.enableFusedInstanceNode)
    {
        Float32BoxNode blasRootNode = (Float32BoxNode)0;

        if (IsBoxNode32(blasRootNodePointer))
        {
            blasRootNode = FetchFloat32BoxNode(blasBaseAddr,
#if GPURT_BUILD_RTIP3
                                               false,
#endif
                                               blasRootNodePointer);
        }
        else if (IsBoxNode16(blasRootNodePointer))
        {
            blasRootNode = FetchFloat16BoxNodeAsFp32(blasBaseAddr, blasRootNodePointer);
        }
        else
        {
            // Procedural or triangle node
            //
            // Note, we only rebraid one level of the BLAS, the parent pointer is guaranteed to be the
            // root node pointer.
            blasRootNode = FetchFloat32BoxNode(blasBaseAddr,
#if GPURT_BUILD_RTIP3
                                               false,
#endif
                                               CreateRootNodePointer());

            // Copy triangle box data from root node
            if (blasRootNode.child1 == blasRootNodePointer)
            {
                blasRootNode.bbox0_min = blasRootNode.bbox1_min;
                blasRootNode.bbox0_max = blasRootNode.bbox1_max;
            }
            if (blasRootNode.child2 == blasRootNodePointer)
            {
                blasRootNode.bbox0_min = blasRootNode.bbox2_min;
                blasRootNode.bbox0_max = blasRootNode.bbox2_max;
            }
            if (blasRootNode.child3 == blasRootNodePointer)
            {
                blasRootNode.bbox0_min = blasRootNode.bbox3_min;
                blasRootNode.bbox0_max = blasRootNode.bbox3_max;
            }

            // Disable all other child nodes
            blasRootNode.child0 = blasRootNodePointer;
            blasRootNode.child1 = INVALID_IDX;
            blasRootNode.child2 = INVALID_IDX;
            blasRootNode.child3 = INVALID_IDX;
        }

        // Transform child boxes
        //

        // Child 0
        BoundingBox transformedBounds = (BoundingBox)0;
        transformedBounds.min = blasRootNode.bbox0_min;
        transformedBounds.max = blasRootNode.bbox0_max;

        transformedBounds = TransformBoundingBox(transformedBounds, instanceDesc.Transform);

        blasRootNode.bbox0_min = transformedBounds.min;
        blasRootNode.bbox0_max = transformedBounds.max;

        // Child 1
        if (blasRootNode.child1 != INVALID_IDX)
        {
            transformedBounds.min = blasRootNode.bbox1_min;
            transformedBounds.max = blasRootNode.bbox1_max;

            transformedBounds = TransformBoundingBox(transformedBounds, instanceDesc.Transform);

            blasRootNode.bbox1_min = transformedBounds.min;
            blasRootNode.bbox1_max = transformedBounds.max;
        }

        // Child 2
        if (blasRootNode.child2 != INVALID_IDX)
        {
            transformedBounds.min = blasRootNode.bbox2_min;
            transformedBounds.max = blasRootNode.bbox2_max;

            transformedBounds = TransformBoundingBox(transformedBounds, instanceDesc.Transform);

            blasRootNode.bbox2_min = transformedBounds.min;
            blasRootNode.bbox2_max = transformedBounds.max;
        }

        // Child 3
        if (blasRootNode.child3 != INVALID_IDX)
        {
            transformedBounds.min = blasRootNode.bbox3_min;
            transformedBounds.max = blasRootNode.bbox3_max;

            transformedBounds = TransformBoundingBox(transformedBounds, instanceDesc.Transform);

            blasRootNode.bbox3_min = transformedBounds.min;
            blasRootNode.bbox3_max = transformedBounds.max;
        }

        // Clear flags in box node. During traversal the pointer flags are not updated at the time of
        // fused instance intersection. It is better to disable node culling for this intersection
        // than using accurate pointer flags (which requires an early fetch prior to intersection)
        //
        blasRootNode.flags = 0;

        const uint fusedBoxNodeOffset = instanceNodeOffset + FUSED_INSTANCE_NODE_ROOT_OFFSET;

        DstMetadata.Store<uint>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_CHILD0_OFFSET, blasRootNode.child0);
        DstMetadata.Store<uint>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_CHILD1_OFFSET, blasRootNode.child1);
        DstMetadata.Store<uint>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_CHILD2_OFFSET, blasRootNode.child2);
        DstMetadata.Store<uint>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_CHILD3_OFFSET, blasRootNode.child3);

        DstMetadata.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB0_MIN_OFFSET, blasRootNode.bbox0_min);
        DstMetadata.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB0_MAX_OFFSET, blasRootNode.bbox0_max);
        DstMetadata.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB1_MIN_OFFSET, blasRootNode.bbox1_min);
        DstMetadata.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB1_MAX_OFFSET, blasRootNode.bbox1_max);
        DstMetadata.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB2_MIN_OFFSET, blasRootNode.bbox2_min);
        DstMetadata.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB2_MAX_OFFSET, blasRootNode.bbox2_max);
        DstMetadata.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB3_MIN_OFFSET, blasRootNode.bbox3_min);
        DstMetadata.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB3_MAX_OFFSET, blasRootNode.bbox3_max);

        DstMetadata.Store<uint>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET, blasRootNode.flags);
    }
}

//=====================================================================================================================
uint PackInstanceMaskAndNodeFlags(
    uint instanceInclusionMask,
    uint nodeFlags)
{
    // Note, we store the instance exclusion mask (instead of inclusion) so that we can combine the
    // masks together using a AND operation similar to box node flags.
    const uint exclusionMask = (~instanceInclusionMask & 0xff);
    const uint packedFlags = ((exclusionMask << 8) | nodeFlags);

    return packedFlags;
}

//=====================================================================================================================
uint PackScratchNodeFlags(
    uint instanceInclusionMask,
    uint nodeFlags,
    uint quadSwizzle)
{
    const uint packedFlags = (quadSwizzle << 16) | PackInstanceMaskAndNodeFlags(instanceInclusionMask, nodeFlags);
    return packedFlags;
}

//=====================================================================================================================
static void WriteAccelStructHeaderRootBoundingBox(
    in BoundingBox bbox)
{
    DstBuffer.Store<BoundingBox>(ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET, bbox);
}

//=====================================================================================================================
static uint IncrementAccelStructHeaderField(
    in uint offset,
    in uint value)
{
    uint origValue;
    DstBuffer.InterlockedAdd(offset, value, origValue);

    return origValue;
}

//=====================================================================================================================
static void WriteAccelStructHeaderField(
    in uint offset,
    in uint data)
{
    DstBuffer.Store(offset, data);
}

//=====================================================================================================================
static uint ReadAccelStructHeaderField(
    in uint offset)
{
    return DstBuffer.Load(offset);
}

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
// Write the indirect dispatch group count for Updates.
static void WriteUpdateGroupCount(
    uint updateThreadsPerFatLeaf)
{
    if (IsUpdateAllowed())
    {
        const uint updateThreadGroupSize = (Settings.rtIpLevel == GPURT_RTIP3_1) ? UPDATE3_1_THREADGROUP_SIZE :
                                                                                   BUILD_THREADGROUP_SIZE;

        const uint numFatLeafNodes = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP16_NODES_OFFSET);

        const uint numWorkItems    = numFatLeafNodes * updateThreadsPerFatLeaf;
        const uint numThreadGroups = RoundUpQuotient(numWorkItems, updateThreadGroupSize);
        const uint3 dispatchCount  = uint3(numThreadGroups, 1, 1);
        DstMetadata.Store3(ACCEL_STRUCT_METADATA_UPDATE_GROUP_COUNT_OFFSET, dispatchCount);
    }
}
#endif

//======================================================================================================================
bool EnableLatePairCompression()
{
    return (Settings.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
           (Settings.topLevelBuild == false) &&
           (Settings.enableEarlyPairCompression == false);
}

//=====================================================================================================================
bool UsePrimIndicesArray()
{
#if GPURT_BUILD_RTIP3_1
    if (
        0)
    {
        return true;
    }
#endif

    return false;
}

//=====================================================================================================================
bool CollapseAnyPairs()
{
#if GPURT_BUILD_RTIP3_1 && (BUILD_PARALLEL == 0)
    return Settings.maxPrimRangeSize == 2;
#else
    return false;
#endif
}

//=====================================================================================================================
uint CalcAtomicFlagsOffset(uint index, uint atomicFlagsScratchOffset, uint type)
{
    const uint idx = (index * NUM_DLB_VALID_TYPES) + type;
    const uint offset = idx * sizeof(Flags);

    return atomicFlagsScratchOffset + offset;
}

//=====================================================================================================================
void WriteFlagsDLB(
    uint    atomicFlagsScratchOffset,
    uint    index,
    uint    type,
    Flags   flags)
{
    const uint offset = CalcAtomicFlagsOffset(index, atomicFlagsScratchOffset, type);

    ScratchBuffer.Store(offset + FLAGS_PREFIX_SUM_OFFSET, flags.prefixSum);

    DeviceMemoryBarrier();

    ScratchBuffer.Store(offset + FLAGS_DATA_VALID_OFFSET, flags.dataValid);
}

//=====================================================================================================================
uint ReadPrefixSumDLB(
    uint atomicFlagsScratchOffset,
    uint index,
    uint type)
{
    const uint offset = CalcAtomicFlagsOffset(index, atomicFlagsScratchOffset, type);

    return ScratchBuffer.Load(offset + FLAGS_PREFIX_SUM_OFFSET);
}

//=====================================================================================================================
uint ReadValidDLB(
    uint atomicFlagsScratchOffset,
    uint index,
    uint type)
{
    const uint offset = CalcAtomicFlagsOffset(index, atomicFlagsScratchOffset, type);

    return ScratchBuffer.Load(offset + FLAGS_DATA_VALID_OFFSET);
}

//=====================================================================================================================
// Loads InstanceDesc from RWByteAddressBuffer InstanceDescBuffer. Takes care of arrayOfPointers case.
// Requires following forward declarations:
template<typename T> T LoadInstanceDescBuffer(uint offset);
static InstanceDesc FetchInstanceDescAddr(in GpuVirtualAddress instanceAddr);

InstanceDesc LoadInstanceDesc(
    in uint instanceId,
    in uint offsetInBytes)
{
    if (Settings.encodeArrayOfPointers != 0)
    {
        const GpuVirtualAddress addr = LoadInstanceDescBuffer<GpuVirtualAddress>(
            instanceId * sizeof(GpuVirtualAddress));

        return FetchInstanceDescAddr(addr + offsetInBytes);
    }
    else
    {
        return LoadInstanceDescBuffer<InstanceDesc>(instanceId * INSTANCE_DESC_SIZE + offsetInBytes);
    }
}

//=====================================================================================================================
uint GetNumPersistentThreadGroups(
    uint numWorkItems,
    uint threadGroupSize,
    uint wavesPerSimd,
    uint optimalNumThreadGroups)
{
    return min(optimalNumThreadGroups * wavesPerSimd, RoundUpQuotient(numWorkItems, threadGroupSize));
}

#endif
