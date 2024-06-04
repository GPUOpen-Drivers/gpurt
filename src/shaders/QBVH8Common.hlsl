/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _QBVHCOMMON_HLSL
#define _QBVHCOMMON_HLSL

namespace QBVH8
{
// All node types for compressed format BVH are 128-bytes
static const uint NodeSizeInBytes = 128;

//=====================================================================================================================
static void WriteInternalNodeBaseOffset(
    in uint nodeOffset,
    in uint offset)
{
    DstBuffer.Store(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_INTERNAL_NODE_BASE_OFFSET, PackNodePointer(0, offset));
}

//=====================================================================================================================
static void WriteLeafNodeBaseOffset(
    in uint nodeOffset,
    in uint offset)
{
    DstBuffer.Store(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_LEAF_NODE_BASE_OFFSET, PackNodePointer(0, offset));
}

//=====================================================================================================================
static void WriteParentPointer(
    in uint nodeOffset,
    in uint parentNodePtr)
{
    DstBuffer.Store(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_PARENT_POINTER, parentNodePtr);
}

//=====================================================================================================================
static void WriteOrigin(
    in uint   nodeOffset,
    in float3 origin)
{
    DstBuffer.Store<float3>(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_ORIGIN, origin);
}

//=====================================================================================================================
static void WritePackedExpChildIdxAndCount(
    in uint  nodeOffset,
    in uint3 exponents,
    in uint  indexInParent,
    in uint  validChildCount)
{
    const uint packedData =
        QuantizedBVH8BoxNode::PackExpChildIdxAndCount(exponents, 0, indexInParent, validChildCount);

    DstBuffer.Store(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT, packedData);
}

//=====================================================================================================================
static void WriteObbMatrixIndex(
    in uint nodeOffset,
    in uint obbMatrixIndex)
{
    DstBuffer.Store(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_OBB_MATRIX_INDEX, obbMatrixIndex);
}

//=====================================================================================================================
static void WritePackedChildInfo(
    in uint  nodeOffset,
    in uint  childIdx,
    in uint3 childInfo)
{
    const uint childInfoOffset = nodeOffset + GetChildInfoOffset(childIdx);
    DstBuffer.Store<uint3>(childInfoOffset, childInfo);
}

//=====================================================================================================================
static ChildInfo ReadPackedChildInfo(
    in uint nodeOffset,
    in uint childIdx)
{
    const uint childInfoOffset = nodeOffset + GetChildInfoOffset(childIdx);
    return DstBuffer.Load<ChildInfo>(childInfoOffset);
}

//=====================================================================================================================
static void WriteInvalidChildInfo(
    in uint nodeOffset,
    in uint childIdx)
{
    const uint childInfoOffset = nodeOffset + GetChildInfoOffset(childIdx);
    DstBuffer.Store<uint3>(childInfoOffset, ChildInfo::Invalid);
}

//=====================================================================================================================
static uint3 ComputePackedChildInfo(
    in BoundingBox   childBounds,
    in uint          instanceMask,
    in uint          boxNodeFlags,
    in uint          nodeType,
    in uint          nodeRangeLength,
    in float3        rcpExponents,
    in float3        origin)
{
    const uint3 quantMax = ComputeQuantizedMax(childBounds.max, origin, rcpExponents, 12);
    const uint3 quantMin = ComputeQuantizedMin(childBounds.min, origin, rcpExponents, 12);

    const uint3 childInfo = ChildInfo::BuildPacked(quantMin,
                                                   quantMax,
                                                   boxNodeFlags,
                                                   instanceMask,
                                                   nodeType,
                                                   nodeRangeLength);
    return childInfo;
}

} // namespace QBVH8

#endif
