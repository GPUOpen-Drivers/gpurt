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
//
//=====================================================================================================================
// The functions defined below require the following variables/resources defined before including this header
//
// uint numActivePrims

#ifndef _BUILDCOMMON_HLSL
#define _BUILDCOMMON_HLSL

#include "IntersectCommon.hlsl"
#include "MortonCodes.hlsl"

//=====================================================================================================================
#define LEAFIDX(i) ((numActivePrims-1) + (i))
#define NODEIDX(i) (i)
#define IS_LEAF(i) ((i) >= (numActivePrims - 1))
#define LEAF_OFFSET(i) ((i) - (numActivePrims - 1))

#define MAX_COLLAPSED_TRIANGLES 8

//=====================================================================================================================
struct RefitArgs
{
    uint topLevelBuild;
    uint numActivePrims;
    uint baseScratchNodesOffset;
    uint doCollapse;
    uint doTriangleSplitting;
    uint enablePairCompression;
    uint enablePairCostCheck;
    uint splitBoxesOffset;
    uint numBatchesOffset;
    uint baseBatchIndicesOffset;
    uint fp16BoxNodesMode;
    float fp16BoxModeMixedSaThreshold;
    uint centroidBoxesOffset;
    uint enableCentroidBoxes;
    bool ltdPackCentroids;
    int4 numMortonBits;
    uint enableInstancePrimCount;
    uint unsortedNodesBaseOffset;
    uint enableEarlyPairCompression;
    uint reserved0;
    uint reserved1;
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
//https://github.com/Microsoft/DirectX-Graphics-Samples/blob/master/Libraries/D3D12RaytracingFallback/src/RayTracingHelper.hlsli
// The MIT License (MIT)
//
// Copyright(c) 2015 Microsoft
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
static float Determinant(in float3x4 transform)
{
    return transform[0][0] * transform[1][1] * transform[2][2] -
        transform[0][0] * transform[2][1] * transform[1][2] -
        transform[1][0] * transform[0][1] * transform[2][2] +
        transform[1][0] * transform[2][1] * transform[0][2] +
        transform[2][0] * transform[0][1] * transform[1][2] -
        transform[2][0] * transform[1][1] * transform[0][2];
}

//=====================================================================================================================
//https://github.com/Microsoft/DirectX-Graphics-Samples/blob/master/Libraries/D3D12RaytracingFallback/src/RayTracingHelper.hlsli
// The MIT License (MIT)
//
// Copyright(c) 2015 Microsoft
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
static float3x4 Inverse3x4(in float3x4 transform)
{
    const float invDet = rcp(Determinant(transform));

    float3x4 invertedTransform;
    invertedTransform[0][0] = invDet * (transform[1][1] * (transform[2][2] * 1.0f - 0.0f * transform[2][3]) + transform[2][1] * (0.0f * transform[1][3] - transform[1][2] * 1.0f) + 0.0f * (transform[1][2] * transform[2][3] - transform[2][2] * transform[1][3]));
    invertedTransform[1][0] = invDet * (transform[1][2] * (transform[2][0] * 1.0f - 0.0f * transform[2][3]) + transform[2][2] * (0.0f * transform[1][3] - transform[1][0] * 1.0f) + 0.0f * (transform[1][0] * transform[2][3] - transform[2][0] * transform[1][3]));
    invertedTransform[2][0] = invDet * (transform[1][3] * (transform[2][0] * 0.0f - 0.0f * transform[2][1]) + transform[2][3] * (0.0f * transform[1][1] - transform[1][0] * 0.0f) + 1.0f * (transform[1][0] * transform[2][1] - transform[2][0] * transform[1][1]));
    invertedTransform[0][1] = invDet * (transform[2][1] * (transform[0][2] * 1.0f - 0.0f * transform[0][3]) + 0.0f * (transform[2][2] * transform[0][3] - transform[0][2] * transform[2][3]) + transform[0][1] * (0.0f * transform[2][3] - transform[2][2] * 1.0f));
    invertedTransform[1][1] = invDet * (transform[2][2] * (transform[0][0] * 1.0f - 0.0f * transform[0][3]) + 0.0f * (transform[2][0] * transform[0][3] - transform[0][0] * transform[2][3]) + transform[0][2] * (0.0f * transform[2][3] - transform[2][0] * 1.0f));
    invertedTransform[2][1] = invDet * (transform[2][3] * (transform[0][0] * 0.0f - 0.0f * transform[0][1]) + 1.0f * (transform[2][0] * transform[0][1] - transform[0][0] * transform[2][1]) + transform[0][3] * (0.0f * transform[2][1] - transform[2][0] * 0.0f));
    invertedTransform[0][2] = invDet * (0.0f * (transform[0][2] * transform[1][3] - transform[1][2] * transform[0][3]) + transform[0][1] * (transform[1][2] * 1.0f - 0.0f * transform[1][3]) + transform[1][1] * (0.0f * transform[0][3] - transform[0][2] * 1.0f));
    invertedTransform[1][2] = invDet * (0.0f * (transform[0][0] * transform[1][3] - transform[1][0] * transform[0][3]) + transform[0][2] * (transform[1][0] * 1.0f - 0.0f * transform[1][3]) + transform[1][2] * (0.0f * transform[0][3] - transform[0][0] * 1.0f));
    invertedTransform[2][2] = invDet * (1.0f * (transform[0][0] * transform[1][1] - transform[1][0] * transform[0][1]) + transform[0][3] * (transform[1][0] * 0.0f - 0.0f * transform[1][1]) + transform[1][3] * (0.0f * transform[0][1] - transform[0][0] * 0.0f));
    invertedTransform[0][3] = invDet * (transform[0][1] * (transform[2][2] * transform[1][3] - transform[1][2] * transform[2][3]) + transform[1][1] * (transform[0][2] * transform[2][3] - transform[2][2] * transform[0][3]) + transform[2][1] * (transform[1][2] * transform[0][3] - transform[0][2] * transform[1][3]));
    invertedTransform[1][3] = invDet * (transform[0][2] * (transform[2][0] * transform[1][3] - transform[1][0] * transform[2][3]) + transform[1][2] * (transform[0][0] * transform[2][3] - transform[2][0] * transform[0][3]) + transform[2][2] * (transform[1][0] * transform[0][3] - transform[0][0] * transform[1][3]));
    invertedTransform[2][3] = invDet * (transform[0][3] * (transform[2][0] * transform[1][1] - transform[1][0] * transform[2][1]) + transform[1][3] * (transform[0][0] * transform[2][1] - transform[2][0] * transform[0][1]) + transform[2][3] * (transform[1][0] * transform[0][1] - transform[0][0] * transform[1][1]));

    return invertedTransform;
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
BoundingBox TransformBoundingBox(BoundingBox box, float4 inputTransform[3])
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
BoundingBox GenerateInstanceBoundingBox(
    float4         instanceTransform[3],
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
float UintToFloat(in uint v)
{
    v ^= (((v >> 31) - 1) | 0x80000000);

    return asfloat(v);
}

//=====================================================================================================================
float3 Uint3ToFloat3(in uint3 v)
{
    const uint bitShift = 31;
    const uint bitMask = 0x80000000;

    v.x ^= (((v.x >> bitShift) - 1) | bitMask);
    v.y ^= (((v.y >> bitShift) - 1) | bitMask);
    v.z ^= (((v.z >> bitShift) - 1) | bitMask);

    return asfloat(v);
}

//=====================================================================================================================
// Divide uints and round up
uint RoundUpQuotient(
    uint dividend,
    uint divisor)
{
    return (dividend + divisor - 1) / divisor;
}

//=====================================================================================================================
// Divide ints and round up
int RoundUpQuotient(
    int dividend,
    int divisor)
{
    return (dividend + divisor - 1) / divisor;
}

//=====================================================================================================================
// Divide ints and round up
uint64_t RoundUpQuotient(
    uint64_t dividend,
    uint64_t divisor)
{
    return (dividend + divisor - 1) / divisor;
}

//=====================================================================================================================
static uint32_t GetNumInternalNodeCount(
    in uint32_t primitiveCount)
{
    return CalcAccelStructInternalNodeCount(primitiveCount, 4u);
}

//=====================================================================================================================
TriangleData FetchTriangleFromNode(
    RWByteAddressBuffer DstBuffer,
    uint                metadataSize,
    uint                nodePointer)
{
    const uint  nodeOffset    = metadataSize + ExtractNodePointerOffset(nodePointer);
    const uint  nodeType      = GetNodeType(nodePointer);
    const uint3 vertexOffsets = CalcTriangleVertexOffsets(nodeType);

    TriangleData tri;
    tri.v0 = DstBuffer.Load<float3>(nodeOffset + vertexOffsets.x);
    tri.v1 = DstBuffer.Load<float3>(nodeOffset + vertexOffsets.y);
    tri.v2 = DstBuffer.Load<float3>(nodeOffset + vertexOffsets.z);

    return tri;
}

//=====================================================================================================================
uint GetUpdateStackOffset(
    uint baseUpdateStackScratchOffset,
    uint stackIdx)
{
    return baseUpdateStackScratchOffset + (stackIdx * sizeof(uint));
}

//=====================================================================================================================
void PushNodeToUpdateStack(
    uint                baseUpdateStackScratchOffset,
    uint                parentNodePointer)
{
    uint stackPtr;
    ScratchBuffer.InterlockedAdd(UPDATE_SCRATCH_STACK_NUM_ENTRIES_OFFSET, 1, stackPtr);

    uint offset = GetUpdateStackOffset(baseUpdateStackScratchOffset, stackPtr);
    ScratchBuffer.Store(offset, parentNodePointer);
}

//=====================================================================================================================
// Note, SrcBuffer and DstMetadata point to the beginning of the acceleration structure buffer
void CopyChildPointersAndFlags(
    uint                nodePointer,
    uint                metadataSize)
{
    const uint nodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);

    {
        const uint4 childPointers = SrcBuffer.Load<uint4>(nodeOffset);
        DstMetadata.Store<uint4>(nodeOffset, childPointers);

        if (IsBoxNode32(nodePointer))
        {
            const uint sourceFlags = SrcBuffer.Load(nodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET);
            DstMetadata.Store(nodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET, sourceFlags);
        }

    }
}

//=====================================================================================================================
uint4 LoadBoxNodeChildPointers(
    in uint              nodeOffset)
{
    return SrcBuffer.Load<uint4>(nodeOffset);
}

//=====================================================================================================================
uint ComputeChildIndexAndValidBoxCount(
    in uint              metadataSize,
    in uint              parentNodePointer,
    in uint              childNodePointer,
    out_param(uint)      boxNodeCount)
{

    const uint parentNodeOffset = metadataSize + ExtractNodePointerOffset(parentNodePointer);
    const uint4 childPointers = LoadBoxNodeChildPointers(parentNodeOffset);

    // Find child index in parent (assumes child pointer 0 is always valid)
    uint childIdx = 0;
    if (childNodePointer == childPointers.y)
    {
        childIdx = 1;
    }

    if (childNodePointer == childPointers.z)
    {
        childIdx = 2;
    }

    if (childNodePointer == childPointers.w)
    {
        childIdx = 3;
    }

    // Note, IsBoxNode() will return false for invalid nodes.
    boxNodeCount = 0;
    boxNodeCount += IsBoxNode(childPointers.x) ? 1 : 0;
    boxNodeCount += IsBoxNode(childPointers.y) ? 1 : 0;
    boxNodeCount += IsBoxNode(childPointers.z) ? 1 : 0;
    boxNodeCount += IsBoxNode(childPointers.w) ? 1 : 0;

    return childIdx;
}

//=====================================================================================================================
uint ReadParentPointer(
    in uint             metadataSizeInBytes,
    in uint             packedNodePtr)
{
    // Read parent pointer in metadata buffer in reverse order
    const uint nodePtr = ExtractNodePointerCollapse(packedNodePtr);
    return SrcBuffer.Load(metadataSizeInBytes - CalcParentPtrOffset(nodePtr));
}

//=====================================================================================================================
void WriteParentPointer(
    in uint             metadataSizeInBytes,
    in uint             packedNodePtr,
    in uint             parentPtr)
{
    // Store parent pointer in metadata buffer in reverse order
    const uint nodePtr = ExtractNodePointerCollapse(packedNodePtr);
    DstMetadata.Store(metadataSizeInBytes - CalcParentPtrOffset(nodePtr), parentPtr);
}

//=====================================================================================================================
uint GetBlasInternalNodeChildCount(
    uint64_t blasBasePtr,
    uint     nodePointer)
{
    const uint64_t blasBaseAddr = ExtractInstanceAddr(blasBasePtr);

    uint count = 0;

    if (
        IsBoxNode16(nodePointer))
    {
        const Float16BoxNode node = FetchFloat16BoxNode(blasBaseAddr, nodePointer);
        count = 1 + (node.child1 != INVALID_IDX) + (node.child2 != INVALID_IDX) + (node.child3 != INVALID_IDX);
    }
    else
    {
        const Float32BoxNode node = FetchFloat32BoxNode(blasBaseAddr,
                                                        nodePointer);
        count = 1 + (node.child1 != INVALID_IDX) + (node.child2 != INVALID_IDX) + (node.child3 != INVALID_IDX);
    }

    return count;
}

//=====================================================================================================================
uint GetInternalNodeType()
{
    return NODE_TYPE_BOX_FLOAT32;
}

//=====================================================================================================================
void WriteBoxNode(
    in uint             offset,
    in Float32BoxNode   f32BoxNode)
{
    DstMetadata.Store<Float32BoxNode>(offset, f32BoxNode);
}

//=====================================================================================================================
void WriteFp16BoxNode(
    in uint             offset,
    in Float16BoxNode   f16BoxNode)
{
    DstMetadata.Store<Float16BoxNode>(offset, f16BoxNode);
}

//=====================================================================================================================
static Float32BoxNode FetchFloat32BoxNode(
    in uint             offset)
{
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
    fp32BoxNode.padding2      = d7.z;
    fp32BoxNode.padding3      = d7.w;

    return fp32BoxNode;
}

//=====================================================================================================================
void WriteInstanceDescriptor(
    in uint               bufferOffset,   // Additional buffer offset
    in InstanceDesc       desc,
    in uint               instanceIndex,
    in uint               instNodePtr,
    in uint               blasMetadataSize,
    in uint               blasRootNodePointer,
    in uint               boxNodeFlags,
    in bool               isFusedInstanceNode)
{
    const uint instanceNodeOffset = ExtractNodePointerOffset(instNodePtr);
    const uint dstNodeOffset = bufferOffset + instanceNodeOffset;
    const GpuVirtualAddress blasBaseAddr =
        GetInstanceAddr(desc.accelStructureAddressLo, desc.accelStructureAddressHiAndFlags);

    {
        InstanceNode node;

        node.desc.InstanceID_and_Mask                           = desc.InstanceID_and_Mask;
        node.desc.InstanceContributionToHitGroupIndex_and_Flags = desc.InstanceContributionToHitGroupIndex_and_Flags;
        node.desc.accelStructureAddressLo                       = desc.accelStructureAddressLo;
        node.desc.accelStructureAddressHiAndFlags               = desc.accelStructureAddressHiAndFlags;

        node.sideband.Transform[0]     = desc.Transform[0];
        node.sideband.Transform[1]     = desc.Transform[1];
        node.sideband.Transform[2]     = desc.Transform[2];
        node.sideband.instanceIndex    = instanceIndex;
        node.sideband.padding0         = 0;
        node.sideband.blasMetadataSize = blasMetadataSize;
        node.sideband.blasNodePointer  = blasRootNodePointer;

        // invert matrix
        float3x4 temp    = CreateMatrix(desc.Transform);
        float3x4 inverse = Inverse3x4(temp);

        node.desc.Transform[0] = inverse[0];
        node.desc.Transform[1] = inverse[1];
        node.desc.Transform[2] = inverse[2];

        DstMetadata.Store<InstanceNode>(dstNodeOffset, node);

        // Write bottom-level root box node into fused instance node
        if (isFusedInstanceNode)
        {
            Float32BoxNode blasRootNode = (Float32BoxNode)0;

            if (IsBoxNode32(blasRootNodePointer))
            {
                blasRootNode = FetchFloat32BoxNode(blasBaseAddr,
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

            transformedBounds = TransformBoundingBox(transformedBounds, desc.Transform);

            blasRootNode.bbox0_min = transformedBounds.min;
            blasRootNode.bbox0_max = transformedBounds.max;

            // Child 1
            if (blasRootNode.child1 != INVALID_IDX)
            {
                transformedBounds.min = blasRootNode.bbox1_min;
                transformedBounds.max = blasRootNode.bbox1_max;

                transformedBounds = TransformBoundingBox(transformedBounds, desc.Transform);

                blasRootNode.bbox1_min = transformedBounds.min;
                blasRootNode.bbox1_max = transformedBounds.max;
            }

            // Child 2
            if (blasRootNode.child2 != INVALID_IDX)
            {
                transformedBounds.min = blasRootNode.bbox2_min;
                transformedBounds.max = blasRootNode.bbox2_max;

                transformedBounds = TransformBoundingBox(transformedBounds, desc.Transform);

                blasRootNode.bbox2_min = transformedBounds.min;
                blasRootNode.bbox2_max = transformedBounds.max;
            }

            // Child 3
            if (blasRootNode.child3 != INVALID_IDX)
            {
                transformedBounds.min = blasRootNode.bbox3_min;
                transformedBounds.max = blasRootNode.bbox3_max;

                transformedBounds = TransformBoundingBox(transformedBounds, desc.Transform);

                blasRootNode.bbox3_min = transformedBounds.min;
                blasRootNode.bbox3_max = transformedBounds.max;
            }

            // Clear flags in box node. During traversal the pointer flags are not updated at the time of
            // fused instance intersection. It is better to disable node culling for this intersection
            // than using accurate pointer flags (which requires an early fetch prior to intersection)
            //
            blasRootNode.flags = 0;

            const uint fusedBoxNodeOffset = dstNodeOffset + FUSED_INSTANCE_NODE_ROOT_OFFSET;

            // The DXC compiler fails validation due to undefined writes to UAV if we use the templated Store
            // instruction. Unrolling the stores works around this issue.
#if 0
            DstMetadata.Store<Float32BoxNode>(fusedBoxNodeOffset, blasRootNode);
#else
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
#endif
        }
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
static void WriteAccelStructHeaderRootBoundingBox(
    in BoundingBox bbox)
{
    DstBuffer.Store<BoundingBox>(ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET, bbox);
}

//=====================================================================================================================
static void WriteAccelStructHeaderCostOrNumChildPrims(
    in uint4 costOrNumChildPrims)
{
    DstBuffer.Store<uint4>(ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET, costOrNumChildPrims);
}

//=====================================================================================================================
static void IncrementAccelStructHeaderField(
    in uint offset,
    in uint value)
{
    DstBuffer.InterlockedAdd(offset, value);
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

//======================================================================================================================
bool EnableLatePairCompression()
{
    return (Settings.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
           (Settings.topLevelBuild == false) &&
           (Settings.enableEarlyPairCompression == false);
}

//=====================================================================================================================
// Task Counter Macros
// INIT_TASK is used at the beginning of a shader to allow for waves to atomically fetch and task index
// BEGIN_TASK is used at the start of a pass that needs to be completed before the next pass
// END_TASK is used at the end of of a pass that makes sure the task is completed before moving onto the next pass
//=====================================================================================================================
#define INIT_TASK           if(localId == 0)\
                            {\
                                DstMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET, 1, SharedMem[0]);\
                            }\
                            GroupMemoryBarrierWithGroupSync();\
                            waveId = SharedMem[0];

#define BEGIN_TASK(n)       while((waveId >= numTasksWait) && (waveId < (numTasksWait + n)))\
                            {\
                                uint globalId = (waveId - numTasksWait) * BUILD_THREADGROUP_SIZE + localId;\
                                uint groupId = (waveId - numTasksWait);

#define END_TASK(n)             DeviceMemoryBarrierWithGroupSync();\
                                if(localId == 0)\
                                {\
                                    DstMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET, 1, SharedMem[0]);\
                                    DstMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_NUM_TASKS_DONE_OFFSET, 1);\
                                }\
                                GroupMemoryBarrierWithGroupSync();\
                                waveId = SharedMem[0];\
                            }\
                            numTasksWait += n;\
                            do\
                            {\
                                DeviceMemoryBarrier();\
                            } while (DstMetadata.Load(ACCEL_STRUCT_METADATA_NUM_TASKS_DONE_OFFSET) < numTasksWait);

#endif
