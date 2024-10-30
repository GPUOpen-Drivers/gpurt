/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef BOX_NODE_1_1_HLSLI
#define BOX_NODE_1_1_HLSLI

#include "../../../shared/assert.h"

//=====================================================================================================================
// Hardware 32-bit box node format and offsets
#define FLOAT32_BBOX_STRIDE                    24
#define FLOAT32_BOX_NODE_CHILD0_OFFSET         0
#define FLOAT32_BOX_NODE_CHILD1_OFFSET         4
#define FLOAT32_BOX_NODE_CHILD2_OFFSET         8
#define FLOAT32_BOX_NODE_CHILD3_OFFSET         12
#define FLOAT32_BOX_NODE_BB0_MIN_OFFSET        16
#define FLOAT32_BOX_NODE_BB0_MAX_OFFSET        28
#define FLOAT32_BOX_NODE_BB1_MIN_OFFSET        40
#define FLOAT32_BOX_NODE_BB1_MAX_OFFSET        52
#define FLOAT32_BOX_NODE_BB2_MIN_OFFSET        64
#define FLOAT32_BOX_NODE_BB2_MAX_OFFSET        76
#define FLOAT32_BOX_NODE_BB3_MIN_OFFSET        88
#define FLOAT32_BOX_NODE_BB3_MAX_OFFSET        100
#define FLOAT32_BOX_NODE_FLAGS_OFFSET          112
#define FLOAT32_BOX_NODE_NUM_PRIM_OFFSET       116
#define FLOAT32_BOX_NODE_UNUSED2_OFFSET        120
#define FLOAT32_BOX_NODE_UNUSED3_OFFSET        124
#define FLOAT32_BOX_NODE_SIZE                  128

//=====================================================================================================================
// Float32 box node flags contains 4 1-byte fields, 1 per child node:
// Child 0 [ 7: 0]
// Child 1 [15: 8]
// Child 2 [23:16]
// Child 3 [31:24]
//
// Each child node's 1-byte field contains these flags:
// Only Opaque     [  0]
// Only Non-Opaque [  1]
// Only Triangles  [  2]
// Only Procedural [  3]
// Unused          [7:4]
#define BOX_NODE_FLAGS_BIT_STRIDE 8

#define BOX_NODE_FLAGS_ONLY_OPAQUE_SHIFT     0
#define BOX_NODE_FLAGS_ONLY_NON_OPAQUE_SHIFT 1
#define BOX_NODE_FLAGS_ONLY_TRIANGLES_SHIFT  2
#define BOX_NODE_FLAGS_ONLY_PROCEDURAL_SHIFT 3

//=====================================================================================================================
struct Float32BoxNode
{
    uint   child0;    /// Child node pointer 0
    uint   child1;    /// Child node pointer 1
    uint   child2;    /// Child node pointer 2
    uint   child3;    /// Child node pointer 3

    float3 bbox0_min; /// Node bounding box 0 minimum bounds
    float3 bbox0_max; /// Node bounding box 0 maximum bounds

    float3 bbox1_min; /// Node bounding box 1 minimum bounds
    float3 bbox1_max; /// Node bounding box 1 maximum bounds

    float3 bbox2_min; /// Node bounding box 2 minimum bounds
    float3 bbox2_max; /// Node bounding box 2 maximum bounds

    float3 bbox3_min; /// Node bounding box 3 minimum bounds
    float3 bbox3_max; /// Node bounding box 3 maximum bounds

    uint   flags;          /// Reserved for RTIP 2.0
    uint   numPrimitives;  /// Padding for 64-byte alignment
    uint   padding2;       /// Padding for 64-byte alignment
    uint   padding3;       /// Padding for 64-byte alignment

};

GPURT_STATIC_ASSERT(FLOAT32_BOX_NODE_SIZE == sizeof(Float32BoxNode), "Float32BoxNode structure mismatch");

//=====================================================================================================================
// Hardware 16-bit box node format and offsets
#define FLOAT16_BBOX_STRIDE                 12
#define FLOAT16_BOX_NODE_CHILD0_OFFSET      0
#define FLOAT16_BOX_NODE_CHILD1_OFFSET      4
#define FLOAT16_BOX_NODE_CHILD2_OFFSET      8
#define FLOAT16_BOX_NODE_CHILD3_OFFSET      12
#define FLOAT16_BOX_NODE_BB0_OFFSET         16
#define FLOAT16_BOX_NODE_BB1_OFFSET         28
#define FLOAT16_BOX_NODE_BB2_OFFSET         40
#define FLOAT16_BOX_NODE_BB3_OFFSET         52
#define FLOAT16_BOX_NODE_SIZE               64

//=====================================================================================================================
struct Float16BoxNode
{
    uint  child0;   /// Child node pointer 0
    uint  child1;   /// Child node pointer 1
    uint  child2;   /// Child node pointer 2
    uint  child3;   /// Child node pointer 3

    uint3 bbox0;    /// Node bounding box 0, packed, uses float16: minx, miny | minz, maxx | maxy, maxz
    uint3 bbox1;    /// Node bounding box 1, packed, uses float16: minx, miny | minz, maxx | maxy, maxz
    uint3 bbox2;    /// Node bounding box 2, packed, uses float16: minx, miny | minz, maxx | maxy, maxz
    uint3 bbox3;    /// Node bounding box 3, packed, uses float16: minx, miny | minz, maxx | maxy, maxz

    // NOTE: each bounding box is defined as uint3 for simplicity
    // Each 32 bits pack 2x float16s. Order above is written as: a, b
    // with a located in the lower 16 bits, b in the upper 16 bits
    // bbox0.x stores minx, miny
    //
    // Alternatively, one can define each bbox as a pair of float16_t3
    // similar to FLOAT32_BOX_NODE. Indexing in hlsl would require extra work
};

GPURT_STATIC_ASSERT(FLOAT16_BOX_NODE_SIZE == sizeof(Float16BoxNode), "Float16BoxNode structure mismatch");

#endif
