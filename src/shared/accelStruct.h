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
#ifndef _ACCELSTRUCT_H
#define _ACCELSTRUCT_H

#include "../../gpurt/gpurtAccelStruct.h"

// =====================================================================================================================
// Calculate acceleration structure internal node count for resulting BVH
inline uint32_t CalcAccelStructInternalNodeCount(
    uint32_t primitiveCount,    // Primitive node count (instances or triangle/aabb geometry)
    uint32_t maxChildCount)     // Maximum child nodes per internal node
{
    const uint32_t n = primitiveCount;
    const uint32_t m = maxChildCount;

    return ((n * m) / (2 * (m - 1u))) + 1u;
}

//=====================================================================================================================
static uint32_t CalcMetadataSizeInBytes(
    uint32_t internalNodeSizeInBytes,
    uint32_t leafNodeSizeInBytes)
{
    ///@note Each 64-bytes in acceleration structure occupies 4-Bytes of parent pointer memory
    ///      for each primitive in the leaf node. E.g. an acceleration structure with 4 primitives
    ///
    ///   | A | 0 | 1 | 2 | 3 | (A: internal node, 0-3 are leaf nodes)
    ///
    ///   Parent pointer memory layout (-1 indicates root node)
    ///
    /// -- 1x and Pair Triangle Compression
    ///   |-1 | x |
    ///   | A | A |
    ///   | A | A |
    ///

    const uint32_t num64ByteChunks     = (internalNodeSizeInBytes + leafNodeSizeInBytes) / 64;
    const uint32_t numLinks            = num64ByteChunks;
    const uint32_t linkDataSizeInBytes = numLinks * sizeof(uint32_t);
    const uint32_t metadataSizeInBytes = ACCEL_STRUCT_METADATA_HEADER_SIZE + linkDataSizeInBytes;

    return metadataSizeInBytes;
}
#endif
