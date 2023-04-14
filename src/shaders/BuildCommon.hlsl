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
// For morton code functions based on libmorton:
// Copyright(c) 2016 Jeroen Baert
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

//=====================================================================================================================
// The functions defined below require the following variables/resources defined before including this header
//
// uint numActivePrims

#ifndef _BUILDCOMMON_HLSL
#define _BUILDCOMMON_HLSL

#include "IntersectCommon.hlsl"
#include "CompactCommon.hlsl"

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
bool IsLeafNode(in uint x, uint numActivePrims)
{
    return (x >= (numActivePrims - 1));
}

//=====================================================================================================================
// Expands 10 bits unsigned into into 30 bits unsigned int
uint ExpandBits(in uint ui)
{
    const uint factors[4] = { 0x00010001u, 0x00000101u, 0x00000011u, 0x00000005u };
    const uint bitMasks[4] = { 0xFF0000FFu , 0x0F00F00Fu, 0xC30C30C3u, 0x49249249u };

    ui = (ui * factors[0]) & bitMasks[0];
    ui = (ui * factors[1]) & bitMasks[1];
    ui = (ui * factors[2]) & bitMasks[2];
    ui = (ui * factors[3]) & bitMasks[3];

    return ui;
}

//=====================================================================================================================
//https://github.com/inkblot-sdnbhd/Morton-Z-Code-C-library/blob/master/MZC2D32.h
// The MIT License(MIT)
//
// Copyright(c) 2015 Inkblot Sdn.Bhd.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files(the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
uint ExpandBits2D(uint w)
{
    w &= 0x0000ffff;                  /* w = ---- ---- ---- ---- fedc ba98 7654 3210 */
    w = (w ^ (w << 8)) & 0x00ff00ff;  /* w = ---- ---- fedc ba98 ---- ---- 7654 3210 */
    w = (w ^ (w << 4)) & 0x0f0f0f0f;  /* w = ---- fedc ---- ba98 ---- 7654 ---- 3210 */
    w = (w ^ (w << 2)) & 0x33333333;  /* w = --fe --dc --ba --98 --76 --54 --32 --10 */
    w = (w ^ (w << 1)) & 0x55555555;  /* w = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0 */
    return w;
}

//=====================================================================================================================
uint64_t ExpandBits4D(uint a)
{
    uint64_t w = a & 0x7fff; // we only look at the first 15 bits

    uint64_t mask = (((1ULL << 7) - 1) << 32) | ((1ULL << 8) - 1);

    w = (w | (w << 24)) & mask;

    mask = (((1ULL << 3) - 1) << 48) |
           (((1ULL << 4) - 1) << 32) |
           (((1ULL << 4) - 1) << 16) |
           ((1ULL << 4) - 1);

    w = (w | (w << 12)) & mask;

    mask = (((1ULL << 1) - 1) << 56) |
           (((1ULL << 2) - 1) << 48) |
           (((1ULL << 2) - 1) << 40) |
           (((1ULL << 2) - 1) << 32) |
           (((1ULL << 2) - 1) << 24) |
           (((1ULL << 2) - 1) << 16) |
           (((1ULL << 2) - 1) << 8) |
           ((1ULL << 2) - 1);

    w = (w | (w << 6)) & mask;

    mask = 0;

    [unroll]
    for (uint b = 0; b < 15; b++)
    {
        mask |= 1ULL << (b * 4);
    }

    w = (w | (w << 3)) & mask;

    return w;
}

//=====================================================================================================================
// Calculates a 30-bit Morton code for the
// given 3D point located within the unit cube [0,1].
uint CalculateMortonCode(in float3 p)
{
    const float x = min(max(p.x * 1024.0, 0.0), 1023.0);
    const float y = min(max(p.y * 1024.0, 0.0), 1023.0);
    const float z = min(max(p.z * 1024.0, 0.0), 1023.0);
    const uint xx = ExpandBits(uint(x));
    const uint yy = ExpandBits(uint(y));
    const uint zz = ExpandBits(uint(z));
    return xx * 4 + yy * 2 + zz;
}

//=====================================================================================================================
// Expands 21 bits into 63 bits
// The following two functions are based on functions from
// https://github.com/Forceflow/libmorton/blob/main/libmorton/morton3D.h
// MIT License
//
// Copyright(c) 2016 Jeroen Baert
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
uint64_t ExpandBits64(in uint64_t a)
{
    uint64_t x = a & 0x1fffff; // we only look at the first 21 bits

    uint64_t temp = 0x1f00000000ffffull;
    x = (x | x << 32) & temp;

    temp = 0x1f0000ff0000ffull;
    x = (x | x << 16) & temp;

    temp = 0x100f00f00f00f00full;
    x = (x | x << 8) & temp;

    temp = 0x10c30c30c30c30c3ull;
    x = (x | x << 4) & temp;

    temp = 0x1249249249249249ull;
    x = (x | x << 2) & temp;

    return x;
}

//=====================================================================================================================
uint64_t ExpandBits2D64(uint64_t w)
{
    w &= 0x00000000ffffffff;
    w = (w ^ (w << 16)) & 0x0000ffff0000ffff;
    w = (w ^ (w << 8)) & 0x00ff00ff00ff00ff;
    w = (w ^ (w << 4)) & 0x0f0f0f0f0f0f0f0f;
    w = (w ^ (w << 2)) & 0x3333333333333333;
    w = (w ^ (w << 1)) & 0x5555555555555555;
    return w;
}

//=====================================================================================================================
// Calculates a 63-bit Morton code for the
// given 3D point located within the unit cube [0,1].
uint64_t CalculateMortonCode64(in float3 p)
{
    const float x = min(max(p.x * 2097152.0, 0.0), 2097151.0);
    const float y = min(max(p.y * 2097152.0, 0.0), 2097151.0);
    const float z = min(max(p.z * 2097152.0, 0.0), 2097151.0);
    const uint64_t xx = ExpandBits64(uint(x));
    const uint64_t yy = ExpandBits64(uint(y));
    const uint64_t zz = ExpandBits64(uint(z));

    return  xx * 4 + yy * 2 + zz;
}

//=====================================================================================================================
bool IsNodeActive(ScratchNode node)
{
    // Inactive nodes force v0.x to NaN during encode
    return !isnan(node.bbox_min_or_v0.x);
}

//=====================================================================================================================
bool IsNodeLinkedOnly(ScratchNode node)
{
    // if this the "2nd" triangle of a set of paired triangle
    return (node.splitBox_or_nodePointer == PAIRED_TRI_LINKONLY);
}

//=====================================================================================================================
uint CalcScratchNodeOffset(
    uint baseScratchNodesOffset,
    uint nodeIndex)
{
    return baseScratchNodesOffset + (nodeIndex * SCRATCH_NODE_SIZE);
}

//=====================================================================================================================
uint ExtractNodeFlagsField(uint nodeFlags, uint childIndex)
{
    return (nodeFlags >> (childIndex * BOX_NODE_FLAGS_BIT_STRIDE)) & 0xFF;
}

//=====================================================================================================================
uint CalcNodeFlags(ScratchNode node)
{
    uint nodeFlags = 0;

    if (IsTriangleNode(node.type) || IsUserNodeProcedural(node.type))
    {
        // Determine opacity from geometry flags
        nodeFlags |= (node.flags & D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE) ? 1u << BOX_NODE_FLAGS_ONLY_OPAQUE_SHIFT
                                                                          : 1u << BOX_NODE_FLAGS_ONLY_NON_OPAQUE_SHIFT;

        nodeFlags |= IsTriangleNode(node.type) ? 1u << BOX_NODE_FLAGS_ONLY_TRIANGLES_SHIFT
                                               : 1u << BOX_NODE_FLAGS_ONLY_PROCEDURAL_SHIFT;
    }
    else if (IsUserNodeInstance(node.type))
    {
        // The node flags in instance nodes were already set up during Encode from the instance flags and geometry type.
        nodeFlags = node.flags;
    }
    else
    {
        // Merge the internal node flags
        const uint leftFlags  = ExtractNodeFlagsField(node.flags, 0);
        const uint rightFlags = ExtractNodeFlagsField(node.flags, 1);

        nodeFlags = leftFlags & rightFlags;
    }

    return nodeFlags;
}

//=====================================================================================================================
uint ClearNodeFlagsField(uint nodeFlags, uint childIndex)
{
    return nodeFlags & ~(0xFFu << (childIndex * BOX_NODE_FLAGS_BIT_STRIDE));
}

//=====================================================================================================================
uint SetNodeFlagsField(uint nodeFlags, uint childFlags, uint childIndex)
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
uint CalcQbvhInternalNodeOffset(uint index, bool useFp16BoxNodesInBlas)
{
    const uint nodeOffset = useFp16BoxNodesInBlas ? (index << BVH4_NODE_16_STRIDE_SHIFT)
                                                  : (index << BVH4_NODE_32_STRIDE_SHIFT);

    // Node offset includes header size
    return ACCEL_STRUCT_HEADER_SIZE + nodeOffset;
}

//=====================================================================================================================
uint CalcQbvhInternalNodeIndex(
    uint offset,
    bool useFp16BoxNodesInBlas)
{
    // Node offset includes header size
    offset -= ACCEL_STRUCT_HEADER_SIZE;

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
    in uint64_t instanceBasePointer,
    float4      instanceTransform[3])
{
    // BLAS instance address points to acceleration structure header
    const uint64_t instanceBaseAddr = GetInstanceAddr(LowPart(instanceBasePointer), HighPart(instanceBasePointer));

    const uint4 d0 = LoadDwordAtAddrx4(instanceBaseAddr + ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET);
    const uint2 d1 = LoadDwordAtAddrx2(instanceBaseAddr + ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET + 0x10);

    BoundingBox rootBbox = (BoundingBox)0;
    rootBbox.min = asfloat(d0.xyz);
    rootBbox.max = asfloat(uint3(d0.w, d1.xy));

    BoundingBox instanceBbox = TransformBoundingBox(rootBbox, instanceTransform);

    if (any(isinf(instanceBbox.min)) || any(isinf(instanceBbox.max)))
    {
        instanceBbox.min = float3(0, 0, 0);
        instanceBbox.max = float3(0, 0, 0);
    }

    return instanceBbox;
}

//=====================================================================================================================
uint FloatToUint(float v)
{
    const uint bitShift = 31;
    const uint bitMask = 0x80000000;

    uint ui = uint(asuint(v));
    ui ^= (1 + ~(ui >> bitShift) | bitMask);

    return ui;
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
    RWByteAddressBuffer scratchBuffer,
    uint                baseUpdateStackScratchOffset,
    uint                parentNodePointer)
{
    uint stackPtr;
    scratchBuffer.InterlockedAdd(UPDATE_SCRATCH_STACK_NUM_ENTRIES_OFFSET, 1, stackPtr);

    uint offset = GetUpdateStackOffset(baseUpdateStackScratchOffset, stackPtr);
    scratchBuffer.Store(offset, parentNodePointer);
}

//=====================================================================================================================
// Note, srcBuffer and dstBuffer point to the beginning of the acceleration structure buffer
void CopyChildPointersAndFlags(
    RWByteAddressBuffer srcBuffer,
    RWByteAddressBuffer dstBuffer,
    uint                nodePointer,
    uint                metadataSize)
{
    const uint nodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);

    const uint4 childPointers = srcBuffer.Load<uint4>(nodeOffset);
    dstBuffer.Store<uint4>(nodeOffset, childPointers);

    if (IsBoxNode32(nodePointer))
    {
        const uint sourceFlags = srcBuffer.Load(nodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET);
        dstBuffer.Store(nodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET, sourceFlags);
    }

}

//=====================================================================================================================
uint ComputeChildIndexAndValidBoxCount(
    in BuildSettingsData settings,
    RWByteAddressBuffer  srcBuffer,
    in uint              parentNodeOffset,
    in uint              childNodePointer,
    out_param(uint)      boxNodeCount)
{
    const uint4 childPointers = srcBuffer.Load<uint4>(parentNodeOffset);

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
    RWByteAddressBuffer SrcBuffer,
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
uint GetChildCount(
    GpuVirtualAddress address,
    uint              nodePointer)
{
    uint count = 0;

    if (
        IsBoxNode16(nodePointer))
    {
        const Float16BoxNode node = FetchFloat16BoxNode(address, nodePointer);
        count = 1 + (node.child1 != INVALID_IDX) + (node.child2 != INVALID_IDX) + (node.child3 != INVALID_IDX);
    }
    else
    {
        const Float32BoxNode node = FetchFloat32BoxNode(address,
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
    RWByteAddressBuffer DstBuffer,
    in uint             offset,
    in Float32BoxNode   f32BoxNode)
{
    DstBuffer.Store<Float32BoxNode>(offset, f32BoxNode);
}

//=====================================================================================================================
void WriteFp16BoxNode(
    RWByteAddressBuffer DstBuffer,
    in uint             offset,
    in Float16BoxNode   f16BoxNode)
{
    DstBuffer.Store<Float16BoxNode>(offset, f16BoxNode);
}

//=====================================================================================================================
static Float32BoxNode FetchFloat32BoxNode(
    RWByteAddressBuffer DataBuffer,
    in uint             offset)
{
    ///

    uint4 d0, d1, d2, d3, d4, d5, d6, d7;
    d0 = DataBuffer.Load4(offset);
    d1 = DataBuffer.Load4(offset + 0x10);
    d2 = DataBuffer.Load4(offset + 0x20);
    d3 = DataBuffer.Load4(offset + 0x30);
    d4 = DataBuffer.Load4(offset + 0x40);
    d5 = DataBuffer.Load4(offset + 0x50);
    d6 = DataBuffer.Load4(offset + 0x60);
    d7 = DataBuffer.Load4(offset + 0x70);

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
    RWByteAddressBuffer   dstBuffer,
    in InstanceDesc       desc,
    in uint               index,
    in uint               instanceNodeOffset,
    in uint               metadataSize,
    in uint               blasRootNodePointer,
    in bool               isFusedInstanceNode)
{
    {
        InstanceNode node;

        node.desc.InstanceID_and_Mask                           = desc.InstanceID_and_Mask;
        node.desc.InstanceContributionToHitGroupIndex_and_Flags = desc.InstanceContributionToHitGroupIndex_and_Flags;
        node.desc.accelStructureAddressLo                       = desc.accelStructureAddressLo;
        node.desc.accelStructureAddressHiAndFlags               = desc.accelStructureAddressHiAndFlags;

        node.sideband.Transform[0]     = desc.Transform[0];
        node.sideband.Transform[1]     = desc.Transform[1];
        node.sideband.Transform[2]     = desc.Transform[2];
        node.sideband.instanceIndex    = index;
        node.sideband.padding0         = 0;
        node.sideband.blasMetadataSize = metadataSize;
        node.sideband.blasNodePointer  = blasRootNodePointer;

        // invert matrix
        float3x4 temp    = CreateMatrix(desc.Transform);
        float3x4 inverse = Inverse3x4(temp);

        node.desc.Transform[0] = inverse[0];
        node.desc.Transform[1] = inverse[1];
        node.desc.Transform[2] = inverse[2];

        dstBuffer.Store<InstanceNode>(instanceNodeOffset, node);

        // Write bottom-level root box node into fused instance node
        if (isFusedInstanceNode)
        {
            const GpuVirtualAddress address =
                GetInstanceAddr(desc.accelStructureAddressLo, desc.accelStructureAddressHiAndFlags);

            Float32BoxNode blasRootNode = (Float32BoxNode)0;

            if (IsBoxNode32(blasRootNodePointer))
            {
                blasRootNode = FetchFloat32BoxNode(address,
                                                   blasRootNodePointer);
            }
            else if (IsBoxNode16(blasRootNodePointer))
            {
                blasRootNode = FetchFloat16BoxNodeAsFp32(address, blasRootNodePointer);
            }
            else
            {
                // Procedural or triangle node
                //
                // Note, we only rebraid one level of the BLAS, the parent pointer is guaranteed to be the
                // root node pointer.
                blasRootNode = FetchFloat32BoxNode(address,
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

            const uint fusedBoxNodeOffset = instanceNodeOffset + FUSED_INSTANCE_NODE_ROOT_OFFSET;

            // The DXC compiler fails validation due to undefined writes to UAV if we use the templated Store
            // instruction. Unrolling the stores works around this issue.
#if 0
            dstBuffer.Store<Float32BoxNode>(fusedBoxNodeOffset, blasRootNode);
#else
            dstBuffer.Store<uint>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_CHILD0_OFFSET, blasRootNode.child0);
            dstBuffer.Store<uint>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_CHILD1_OFFSET, blasRootNode.child1);
            dstBuffer.Store<uint>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_CHILD2_OFFSET, blasRootNode.child2);
            dstBuffer.Store<uint>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_CHILD3_OFFSET, blasRootNode.child3);

            dstBuffer.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB0_MIN_OFFSET, blasRootNode.bbox0_min);
            dstBuffer.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB0_MAX_OFFSET, blasRootNode.bbox0_max);
            dstBuffer.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB1_MIN_OFFSET, blasRootNode.bbox1_min);
            dstBuffer.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB1_MAX_OFFSET, blasRootNode.bbox1_max);
            dstBuffer.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB2_MIN_OFFSET, blasRootNode.bbox2_min);
            dstBuffer.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB2_MAX_OFFSET, blasRootNode.bbox2_max);
            dstBuffer.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB3_MIN_OFFSET, blasRootNode.bbox3_min);
            dstBuffer.Store<float3>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_BB3_MAX_OFFSET, blasRootNode.bbox3_max);

            dstBuffer.Store<uint>(fusedBoxNodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET, blasRootNode.flags);
#endif
        }
    }
}

//=====================================================================================================================
uint64_t CalculateVariableBitsMortonCode64(float3 sceneExtent,
                                           float3 normalizedPos,
                                           uint numSizeBits,
                                           out uint3 values,
                                           out uint numAxisBits,
                                           out uint4 numMortonBitsPerAxis)
{
    uint numMortonBits = 62;

    if (numSizeBits > 0)
    {
        numMortonBits -= numSizeBits;
    }

    numMortonBitsPerAxis = uint4(0, 0, 0, numSizeBits);
    int3 numBits = 0;
    int3 numPrebits;
    int3 startAxis;

    // find the largest start axis
    // and how many prebits are needed between largest and two other axes
    if (sceneExtent.x < sceneExtent.y)
    {
        if (sceneExtent.x < sceneExtent.z)
        {
            if (sceneExtent.y < sceneExtent.z)
            {
                // z, y, x
                startAxis[0] = 2;
                numPrebits[0] = log2(sceneExtent.z / sceneExtent.y);

                startAxis[1] = 1;
                numPrebits[1] = log2(sceneExtent.y / sceneExtent.x);

                startAxis[2] = 0;
                numPrebits[2] = log2(sceneExtent.z / sceneExtent.x);
            }
            else
            {
                // y, z, x
                startAxis[0] = 1;
                numPrebits[0] = log2(sceneExtent.y / sceneExtent.z);

                startAxis[1] = 2;
                numPrebits[1] = log2(sceneExtent.z / sceneExtent.x);

                startAxis[2] = 0;
                numPrebits[2] = log2(sceneExtent.y / sceneExtent.x);
            }
        }
        else
        {
            // y, x, z
            startAxis[0] = 1;
            numPrebits[0] = log2(sceneExtent.y / sceneExtent.x);

            startAxis[1] = 0;
            numPrebits[1] = log2(sceneExtent.x / sceneExtent.z);

            startAxis[2] = 2;
            numPrebits[2] = log2(sceneExtent.y / sceneExtent.z);
        }
    }
    else
    {
        if (sceneExtent.y < sceneExtent.z)
        {
            if (sceneExtent.x < sceneExtent.z)
            {
                // z, x, y
                startAxis[0] = 2;
                numPrebits[0] = log2(sceneExtent.z / sceneExtent.x);

                startAxis[1] = 0;
                numPrebits[1] = log2(sceneExtent.x / sceneExtent.y);

                startAxis[2] = 1;
                numPrebits[2] = log2(sceneExtent.z / sceneExtent.y);
            }
            else
            {
                // x, z, y
                startAxis[0] = 0;
                numPrebits[0] = log2(sceneExtent.x / sceneExtent.z);

                startAxis[1] = 2;
                numPrebits[1] = log2(sceneExtent.z / sceneExtent.y);

                startAxis[2] = 1;
                numPrebits[2] = log2(sceneExtent.x / sceneExtent.y);
            }
        }
        else
        {
            // x, y, z
            startAxis[0] = 0;
            numPrebits[0] = log2(sceneExtent.x / sceneExtent.y);

            startAxis[1] = 1;
            numPrebits[1] = log2(sceneExtent.y / sceneExtent.z);

            startAxis[2] = 2;
            numPrebits[2] = log2(sceneExtent.x / sceneExtent.z);
        }
    }

    if (sceneExtent[startAxis[2]] == 0)
    {
        numPrebits[1] = 0;
        numPrebits[2] = 0;
    }

    // say x > y > z
    // prebits[0] = 3
    // prebits[1] = 2
    // if swap == 1
    // xxx xy xy x yxz yxz ...
    // if swap == 0
    // xxx xy xy xyz xyz ...
    int swap = numPrebits[2] > (numPrebits[0] + numPrebits[1]) ? 1 : 0;

    numPrebits[0] = min(numPrebits[0], numMortonBits);
    numPrebits[1] = min(numPrebits[1] * 2, numMortonBits - numPrebits[0]) / 2;

    int numPrebitsSum = numPrebits[0] + numPrebits[1] * 2;

    if (numPrebitsSum != numMortonBits)
    {
        numPrebitsSum += swap;
    }
    else
    {
        swap = 0;
    }

    // the scene might be 2D so check for the smallest axis
    numBits[2] = (sceneExtent[startAxis[2]] != 0) ? max(0, (numMortonBits - numPrebitsSum) / 3) : 0;

    if (swap > 0)
    {
        numBits[0] = max(0, (numMortonBits - numBits[2] - numPrebitsSum) / 2 + numPrebits[1] + numPrebits[0] + 1);
        numBits[1] = numMortonBits - numBits[0] - numBits[2];
    }
    else
    {
        numBits[1] = max(0, (numMortonBits - numBits[2] - numPrebitsSum) / 2 + numPrebits[1]);
        numBits[0] = numMortonBits - numBits[1] - numBits[2];
    }

    const int delta = numBits[0] - 31; // clamp axis values to avoid overflow of a uint

    if (delta > 0)
    {
        numBits[0] -= delta;

        numPrebits[0] = min(numPrebits[0], numBits[0]);

        if (numBits[0] == numPrebits[0])
            swap = 0;

        numBits[1] = max(0, numBits[1] - delta);

        numPrebits[1] = min(numPrebits[1], numBits[1]);

        numBits[2] = max(0, numBits[2] - delta);

        numPrebitsSum = numPrebits[0] + numPrebits[1] * 2 + swap;
    }

    numAxisBits = numBits[2] + numBits[1] + numBits[0];

    uint64_t mortonCode = 0;
    uint64_t3 axisCode;

    // based on the number of bits, calculate each code per axis
    [unroll]
    for (uint a = 0; a < 3; a++)
    {
        axisCode[a] = min(max(uint(normalizedPos[startAxis[a]] * (1UL << numBits[a])), 0), (1UL << numBits[a]) - 1);
        numMortonBitsPerAxis[startAxis[a]] = numBits[a];
    }

    values[startAxis[0]] = uint(axisCode[0]);
    values[startAxis[1]] = uint(axisCode[1]);
    values[startAxis[2]] = uint(axisCode[2]);

    uint delta0 = 0;
    uint delta1 = 0;

    // if there are prebits, set them in the morton code:
    // if swap == 1
    // [xxx xy xy x] yxz yxz ...
    // if swap == 0
    // [xxx xy xy xyz] xyz ...
    if (numPrebitsSum > 0)
    {
        numBits[0] -= numPrebits[0];
        mortonCode = axisCode[0] & (((1ULL << numPrebits[0]) - 1) << numBits[0]);
        mortonCode >>= numBits[0];

        mortonCode <<= numPrebits[1] * 2;
        numBits[0] -= numPrebits[1];
        numBits[1] -= numPrebits[1];
        uint64_t temp0 = axisCode[0] & (((1ULL << numPrebits[1]) - 1) << numBits[0]);
        temp0 >>= numBits[0];
        temp0 = ExpandBits2D64(temp0);

        uint64_t temp1 = axisCode[1] & (((1ULL << numPrebits[1]) - 1) << numBits[1]);
        temp1 >>= numBits[1];
        temp1 = ExpandBits2D64(temp1);

        mortonCode |= temp0 * 2 + temp1;

        if (swap > 0)
        {
            mortonCode <<= 1;
            numBits[0] -= 1;
            uint64_t temp = axisCode[0] & (1ULL << numBits[0]);
            temp >>= numBits[0];
            mortonCode |= temp;
        }

        mortonCode <<= numBits[0] + numBits[1] + numBits[2];

        axisCode[0] &= ((1ULL << numBits[0]) - 1);
        axisCode[1] &= ((1ULL << numBits[1]) - 1);

        if (swap > 0)
        {
            uint64_t temp = axisCode[0];
            axisCode[0] = axisCode[1];
            axisCode[1] = temp;

            uint temp2 = numBits[0];
            numBits[0] = numBits[1];
            numBits[1] = temp2;
        }
    }

    // 2D case, just use xy xy xy...
    if (numBits[2] == 0)
    {
        [unroll]
        for (int r = 0; r < 2; r++)
        {
            axisCode[r] = ExpandBits2D64(axisCode[r]);
        }

        uint delta = numBits[0] - numBits[1];

        mortonCode |= (axisCode[0] << (1 - delta)) + (axisCode[1] << delta);
    }
    else // 3D case, just use if swap == 0 xyz xyz xyz..., if swap == 1 yxz yxz yxz...
    {
        int i;
        [unroll]
        for (i = 0; i < 3; i++)
        {
            axisCode[i] = (axisCode[i] > 0) ? ExpandBits64(axisCode[i]) : 0;
        }

        uint delta = numBits[0] - numBits[1];
        uint delta2 = numBits[0] - numBits[2];

        mortonCode |= (((axisCode[0] << (1 - delta)) + (axisCode[1] << (2 * delta))) << (1 - delta2)) +
                      (axisCode[2] << ((1 + (1 - delta)) * delta2));
    }

    return mortonCode;
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
