/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _QUANTIZEDBVH8BOXNODE_HLSL
#define _QUANTIZEDBVH8BOXNODE_HLSL

#include "internalNode.hlsli"
#include "../Common.hlsli"

//=====================================================================================================================
static uint GetChildInfoOffset(
    in uint childIdx)
{
    return QUANTIZED_BVH8_NODE_OFFSET_CHILD_INFO_0 + (childIdx * QUANTIZED_NODE_CHILD_INFO_STRIDE);
}

//=====================================================================================================================
static uint ExtractValidChildCount(
    uint packedData)
{
    return  1 + bitFieldExtract(packedData, 28, 3);
}

//=====================================================================================================================
static uint GetBvh4ChildInfoOffset(
    in uint childIdx)
{
    return QUANTIZED_BVH4_NODE_OFFSET_CHILD_INFO_0 + (childIdx * QUANTIZED_NODE_CHILD_INFO_STRIDE);
}

//=====================================================================================================================
static uint ExtractBvh4ValidChildCount(
    uint packedData)
{
    return  1 + bitFieldExtract(packedData, 28, 2);
}

//=====================================================================================================================
static uint3 ExtractPackedExponents(
    uint packedData)
{
    uint3 exponents;
    exponents.x = bitFieldExtract(packedData, 0, 8);
    exponents.y = bitFieldExtract(packedData, 8, 8);
    exponents.z = bitFieldExtract(packedData, 16, 8);

    return exponents;
}

#if defined(GPURT_DEVELOPER) && defined(__cplusplus)
static_assert(offsetof(QuantizedBVH8BoxNode, childInfos[0]) == 32, "Alignment Issue");
static_assert(offsetof(QuantizedBVH8BoxNode, childInfos[1]) == 44, "Alignment Issue");
#endif

#endif
