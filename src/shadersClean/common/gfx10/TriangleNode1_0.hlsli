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
#ifndef TRIANGLE_NODE_1_0_HLSLI
#define TRIANGLE_NODE_1_0_HLSLI

#include "../TempAssert.hlsli"

//=====================================================================================================================
// Hardware triangle node format and offsets
// Note: GPURT limits triangle compression to 2 triangles per node. As a result the remaining bytes in the triangle node
// are used for sideband data. The geometry index is packed in bottom 24 bits and geometry flags in bits 25-26.
#define TRIANGLE_NODE_V0_OFFSET 0
#define TRIANGLE_NODE_V1_OFFSET 12
#define TRIANGLE_NODE_V2_OFFSET 24
#define TRIANGLE_NODE_V3_OFFSET 36
#define TRIANGLE_NODE_GEOMETRY_INDEX_AND_FLAGS_OFFSET 48
#define TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET         52
#define TRIANGLE_NODE_PRIMITIVE_INDEX1_OFFSET         56
#define TRIANGLE_NODE_ID_OFFSET 60
#define TRIANGLE_NODE_SIZE      64

//=====================================================================================================================
// Triangle ID contains 4 1-byte fields, 1 per triangle:
// Triangle 0 [ 7: 0]
// Triangle 1 [15: 8]
// Triangle 2 [23:16]
// Triangle 3 [31:24]
//
// Each triangle's 8-bit segment contains these fields:
// I SRC        [1:0] Specifies which vertex in triangle 0 corresponds to the I barycentric value
// J SRC        [3:2] Specifies which vertex in triangle 0 corresponds to the J barycentric value
// Double Sided [  4] Specifies whether triangle 0 should be treated as double sided for culling
// Flip Winding [  5] Specifies whether triangle 0 should have its facedness flipped
// Procedural   [  6] Specifies whether it is a procedural node
// Opaque       [  7] Specifies whether triangle 0 should be considered as opaque
#define TRIANGLE_ID_BIT_STRIDE 8

#define TRIANGLE_ID_I_SRC_SHIFT        0
#define TRIANGLE_ID_J_SRC_SHIFT        2
#define TRIANGLE_ID_DOUBLE_SIDED_SHIFT 4
#define TRIANGLE_ID_FLIP_WINDING_SHIFT 5
#define TRIANGLE_ID_PROCEDURAL_SHIFT   6
#define TRIANGLE_ID_OPAQUE_SHIFT       7

//=====================================================================================================================
struct TriangleNode
{
    float3 v0;                      // Vertex 0
    float3 v1;                      // Vertex 1
    float3 v2;                      // Vertex 2
    float3 v3;                      // Vertex 3
    uint   geometryIndexAndFlags;   // Geometry index and flags for pair of triangles
    uint   primitiveIndex0;         // Primitive index for triangle 0
    uint   primitiveIndex1;         // Primitive index for triangle 1
    uint   triangleId;              // Triangle ID
};

GPURT_STATIC_ASSERT(TRIANGLE_NODE_SIZE == sizeof(TriangleNode), "TriangleNode structure mismatch");

#endif
