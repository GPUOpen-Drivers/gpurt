/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2022 Advanced Micro Devices, Inc. All Rights Reserved.
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

#include "IntersectCommon.hlsl"
#include "CompactCommon.hlsl"

//=====================================================================================================================
#define LEAFIDX(i) ((numActivePrims-1) + (i))
#define NODEIDX(i) (i)
#define IS_LEAF(i) ((i) >= (numActivePrims - 1))
#define LEAF_OFFSET(i) ((i) - (numActivePrims - 1))

#define MAX_COLLAPSED_TRIANGLES 8

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
    return !isnan(node.bbox_min_or_v0.x);
}

//=====================================================================================================================
struct ScratchNodeResourceInfo
{
    RWByteAddressBuffer scratchBuffer;
    uint                scratchBufferScratchNodesOffset;
};

//=====================================================================================================================
uint CalcScratchNodeOffset(
    uint baseScratchNodesOffset,
    uint nodeIndex)
{
    return baseScratchNodesOffset + (nodeIndex * SCRATCH_NODE_SIZE);
}

//=====================================================================================================================
ScratchNode FetchScratchNode(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                nodeIndex)
{
    const uint offset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    return buffer.Load<ScratchNode>(offset);
}

//=====================================================================================================================
ScratchNode FetchScratchNodeImpl(
    ScratchNodeResourceInfo resourceInfo,
    uint                    nodeIndex)
{
    return FetchScratchNode(resourceInfo.scratchBuffer, resourceInfo.scratchBufferScratchNodesOffset, nodeIndex);
}

//=====================================================================================================================
void WriteScratchNode(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                nodeIndex,
    ScratchNode         node)
{
    const uint offset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    buffer.Store<ScratchNode>(offset, node);
}

//=====================================================================================================================
BoundingBox FetchScratchNodeBoundingBox(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                nodeIndex)
{
    const ScratchNode scratchNode = FetchScratchNode(buffer, baseScratchNodesOffset, nodeIndex);

    return GetScratchNodeBoundingBox(scratchNode);
}

//=====================================================================================================================
BoundingBox FetchScratchNodeBoundingBoxTS(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                baseSplitBoxesOffset,
    uint                nodeIndex)
{
    const ScratchNode scratchNode = FetchScratchNode(buffer, baseScratchNodesOffset, nodeIndex);

    BoundingBox bbox;

    // For triangle geometry we need to generate bounding box from split boxes
    if (IsTriangleNode(scratchNode.type))
    {
        bbox = buffer.Load<BoundingBox>(baseSplitBoxesOffset +
                                        sizeof(BoundingBox) * scratchNode.splitBox_or_nodePointer);
    }
    else
    {
        // Internal nodes and AABB geometry encodes bounding box in scratch node
        bbox.min = scratchNode.bbox_min_or_v0;
        bbox.max = scratchNode.bbox_max_or_v1;
    }

    return bbox;
}

//=====================================================================================================================
uint FetchScratchNodeFlags(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                nodeIndex)
{
    const ScratchNode scratchNode = FetchScratchNode(buffer, baseScratchNodesOffset, nodeIndex);

    return scratchNode.flags;
}

//=====================================================================================================================
void WriteScratchNodeBoundingBox(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                nodeIndex,
    float3              bboxMin,
    float3              bboxMax)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    buffer.Store<float3>(nodeOffset + SCRATCH_NODE_BBOX_MIN_OFFSET, bboxMin);
    buffer.Store<float3>(nodeOffset + SCRATCH_NODE_BBOX_MAX_OFFSET, bboxMax);
}

//=====================================================================================================================
void WriteScratchNodeInstanceNumPrims(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                nodeIndex,
    uint                value)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    buffer.Store(nodeOffset + SCRATCH_NODE_INSTANCE_NUM_PRIMS_OFFSET, value);
}

//=====================================================================================================================
void WriteScratchNodeNumMortonCells(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                nodeIndex,
    uint                value)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    buffer.Store(nodeOffset + SCRATCH_NODE_NUM_MORTON_CELLS_OFFSET, value);
}

//=====================================================================================================================
void WriteScratchNodeType(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                fp16BoxNodesInBlasMode,
    float               fp16BoxModeMixedSaThresh,
    uint                nodeIndex,
    uint                leftNodeType,
    uint                rightNodeType,
    float3              nodeBboxMin,
    float3              nodeBboxMax)
{
    // Parents of leaves are marked to avoid propagating FP16 throughout the entire tree
    const uint nodeTypeLeafParentFp16 = (NODE_TYPE_BOX_FLOAT16 | NODE_POINTER_MASK_MSB);

    // Should this node be fp16 or fp32 box node?
    bool isParentOfLeaf  = false;
    bool writeNodeAsFp16 = false;
    if (fp16BoxNodesInBlasMode == ALL_INTERIOR_NODES_IN_BLAS_AS_FP16)
    {
        writeNodeAsFp16 = true;
    }
    else if (fp16BoxNodesInBlasMode == LEAF_NODES_IN_BLAS_AS_FP16)
    {
        // Mark node as fp16 IFF:
        // 1. directly attached to leaf
        // 2. child is directly attached to leaf
        const bool isParentOfFp16 = ( leftNodeType == nodeTypeLeafParentFp16) ||
                                    (rightNodeType == nodeTypeLeafParentFp16);
        isParentOfLeaf  = IsTriangleNode      ( leftNodeType) ||
                          IsTriangleNode      (rightNodeType) ||
                          IsUserNodeProcedural( leftNodeType) ||
                          IsUserNodeProcedural(rightNodeType);
        writeNodeAsFp16 = (isParentOfLeaf || isParentOfFp16);
    }
    else if (fp16BoxNodesInBlasMode == MIXED_NODES_IN_BLAS_AS_FP16)
    {
        // NOTE: it may help to skip nodes that would never be written out (maybe besed on depth from root?)

        // Mark node as fp16 if its bounds compress well:
        // increase in surface area should be < threshold
        const BoundingBox parentBboxFp32 = { nodeBboxMin, nodeBboxMax };
        const uint3       parentBboxFp16 = CompressBBoxToUint3(parentBboxFp32);

        const float saAsFp32 = ComputeBoxSurfaceArea(parentBboxFp32);
        const float saAsFp16 = ComputeBoxSurfaceArea(parentBboxFp16);

        const float saAsFp32Scaled = saAsFp32 * fp16BoxModeMixedSaThresh;
        writeNodeAsFp16 = (saAsFp16 < saAsFp32Scaled);
    }

    // Root node is always fp32, regardless of mode for fp16 box nodes
    if (nodeIndex == 0)
    {
        isParentOfLeaf  = false;
        writeNodeAsFp16 = false;
    }

    const uint nodeTypeAsFp16 = (isParentOfLeaf  ? nodeTypeLeafParentFp16 : NODE_TYPE_BOX_FLOAT16);
    const uint nodeType       = (writeNodeAsFp16 ? nodeTypeAsFp16         : NODE_TYPE_BOX_FLOAT32);
    const uint nodeOffset     = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);
    buffer.Store<uint>(nodeOffset + SCRATCH_NODE_TYPE_OFFSET, nodeType);
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
void WriteScratchNodeFlags(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                nodeIndex,
    ScratchNode         leftNode,
    ScratchNode         rightNode)
{
    const uint leftNodeFlags  = CalcNodeFlags(leftNode);
    const uint rightNodeFlags = CalcNodeFlags(rightNode);

    uint nodeFlags = 0;
    nodeFlags = SetNodeFlagsField(nodeFlags, leftNodeFlags, 0);
    nodeFlags = SetNodeFlagsField(nodeFlags, rightNodeFlags, 1);

    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);
    buffer.Store<uint>(nodeOffset + SCRATCH_NODE_FLAGS_OFFSET, nodeFlags);
}

//=====================================================================================================================
void WriteScratchNodeNumPrimitives(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                nodeIndex,
    uint                numPrimitivesAndDoCollapse)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    buffer.Store(nodeOffset + SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET, numPrimitivesAndDoCollapse);
}

//=====================================================================================================================
uint FetchScratchNodeNumPrimitives(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                nodeIndex)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    return buffer.Load(nodeOffset + SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET);
}

//=====================================================================================================================
void WriteScratchNodeCost(
    RWByteAddressBuffer buffer,
    uint  baseScratchNodesOffset,
    uint  nodeIndex,
    float cost)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    buffer.Store<float>(nodeOffset + SCRATCH_NODE_COST_OFFSET, cost);
}

//=====================================================================================================================
float FetchScratchNodeCost(
    RWByteAddressBuffer buffer,
    uint  baseScratchNodesOffset,
    uint  nodeIndex)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    return buffer.Load<float>(nodeOffset + SCRATCH_NODE_COST_OFFSET);
}

//=====================================================================================================================
void WriteScratchNodeSurfaceArea(
    RWByteAddressBuffer buffer,
    uint                baseScratchNodesOffset,
    uint                nodeIndex,
    float               surfaceArea)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    buffer.Store<float>(nodeOffset + SCRATCH_NODE_SA_OFFSET, surfaceArea);
}

//=====================================================================================================================
float FetchScratchNodeSurfaceArea(
    RWByteAddressBuffer buffer,
    uint  baseScratchNodesOffset,
    uint  nodeIndex)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    return buffer.Load<float>(nodeOffset + SCRATCH_NODE_SA_OFFSET);
}

//=====================================================================================================================
void WriteScratchNodeChild(
    RWByteAddressBuffer buffer,
    uint  baseScratchNodesOffset,
    uint  nodeIndex,
    uint  child,
    bool  isLeft)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    const uint direction = isLeft ? SCRATCH_NODE_LEFT_OFFSET : SCRATCH_NODE_RIGHT_OFFSET;

    buffer.Store(nodeOffset + direction, child);
}

//=====================================================================================================================
void WriteScratchNodeParent(
    RWByteAddressBuffer buffer,
    uint  baseScratchNodesOffset,
    uint  nodeIndex,
    uint  parent)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    buffer.Store(nodeOffset + SCRATCH_NODE_PARENT_OFFSET, parent);
}

//=====================================================================================================================
void WriteBatchIndex(
    RWByteAddressBuffer buffer,
    uint                numBatchesOffset,
    uint                baseBatchIndicesOffset,
    uint                index)
{
    uint numBatches;
    buffer.InterlockedAdd(numBatchesOffset, 1, numBatches);

    buffer.Store(baseBatchIndicesOffset + (numBatches * sizeof(uint)), index);
}

//=====================================================================================================================
uint FetchMortonCode(RWByteAddressBuffer buffer, uint mortonCodesOffset, uint primitiveIndex)
{
    const uint offset = mortonCodesOffset + (primitiveIndex << 2);
    return buffer.Load(offset);
}

//=====================================================================================================================
uint64_t FetchMortonCode64(RWByteAddressBuffer buffer, uint mortonCodesOffset, uint primitiveIndex)
{
    const uint offset = mortonCodesOffset + (primitiveIndex << 3);
    return buffer.Load<uint64_t>(offset);
}

//=====================================================================================================================
void WriteMortonCode(RWByteAddressBuffer buffer, uint mortonCodesOffset, uint primitiveIndex, uint code)
{
    const uint offset = mortonCodesOffset + (primitiveIndex << 2);
    buffer.Store(offset, code);
}

//=====================================================================================================================
void WriteMortonCode64(RWByteAddressBuffer buffer, uint mortonCodesOffset, uint primitiveIndex, uint64_t code)
{
    const uint offset = mortonCodesOffset + (primitiveIndex << 3);
    buffer.Store<uint64_t>(offset, code);
}

//=====================================================================================================================
uint FetchSortedPrimIndex(
    RWByteAddressBuffer buffer,
    uint                baseSortedPrimIndicesOffset,
    uint                index)
{
    const uint indexOffset = baseSortedPrimIndicesOffset + (index * sizeof(uint));

    return buffer.Load(indexOffset);
}

//=====================================================================================================================
void WriteSortedPrimIndex(
    RWByteAddressBuffer buffer,
    uint                baseSortedPrimIndicesOffset,
    uint                index,
    uint                primIndex)
{
    const uint indexOffset = baseSortedPrimIndicesOffset + (index * sizeof(uint));

    buffer.Store(indexOffset, primIndex);
}

//=====================================================================================================================
uint CalcQbvhInternalNodeOffset(uint index, bool useFp16BoxNodesInBlas)
{
    const uint nodeOffset = useFp16BoxNodesInBlas ? (index << QBVH_NODE_16_STRIDE_SHIFT)
                                                  : (index << QBVH_NODE_32_STRIDE_SHIFT);

    // Node offset includes header size
    return ACCEL_STRUCT_HEADER_SIZE + nodeOffset;
}

//=====================================================================================================================
uint CalcQbvhInternalNodeIndex(uint offset, bool useFp16BoxNodesInBlas)
{
    // Node offset includes header size
    offset -= ACCEL_STRUCT_HEADER_SIZE;

    return useFp16BoxNodesInBlas ? (offset >> QBVH_NODE_16_STRIDE_SHIFT)
                                 : (offset >> QBVH_NODE_32_STRIDE_SHIFT);
}

//=====================================================================================================================
uint CalcInstanceNodeOffset(
    uint baseInstanceNodesOffset,
    uint index)
{
    return baseInstanceNodesOffset + (index * INSTANCE_NODE_SIZE);
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
void UpdateSceneSize(RWByteAddressBuffer scratchBuffer, uint byteOffset, float size)
{
    // Calculate the combined AABB for the entire wave.
    const float waveSizeMin = WaveActiveMin(size);
    const float waveSizeMax = WaveActiveMax(size);

    //TODO: can just use centroids rather than boxes

    // Calculate the AABB for the entire scene using memory atomics.
    // Scalarize the atomic min/max writes by only using the first lane.
    if (WaveIsFirstLane())
    {
        // Convert the wave bounds to uints so we can atomically min/max them against the scene bounds in memory.
        const uint waveMinUint = FloatToUint(waveSizeMin);
        const uint waveMaxUint = FloatToUint(waveSizeMax);

        uint outValue;
        scratchBuffer.InterlockedMin(byteOffset,     waveMinUint, outValue);
        scratchBuffer.InterlockedMax(byteOffset + 4, waveMaxUint, outValue);
    }
}

//=====================================================================================================================
void UpdateSceneBounds(RWByteAddressBuffer scratchBuffer, uint byteOffset, BoundingBox boundingBox)
{
    // Calculate the combined AABB for the entire wave.
    const float3 waveBoundsMin = WaveActiveMin(boundingBox.min);
    const float3 waveBoundsMax = WaveActiveMax(boundingBox.max);

    //TODO: can just use centroids rather than boxes

    // Calculate the AABB for the entire scene using memory atomics.
    // Scalarize the atomic min/max writes by only using the first lane.
    if (WaveIsFirstLane())
    {
        // Convert the wave bounds to uints so we can atomically min/max them against the scene bounds in memory.
        const uint3 waveMinUint = Float3ToUint3(waveBoundsMin);
        const uint3 waveMaxUint = Float3ToUint3(waveBoundsMax);

        uint outValue;
        scratchBuffer.InterlockedMin(byteOffset,     waveMinUint.x, outValue);
        scratchBuffer.InterlockedMin(byteOffset + 4, waveMinUint.y, outValue);
        scratchBuffer.InterlockedMin(byteOffset + 8, waveMinUint.z, outValue);

        scratchBuffer.InterlockedMax(byteOffset + 12, waveMaxUint.x, outValue);
        scratchBuffer.InterlockedMax(byteOffset + 16, waveMaxUint.y, outValue);
        scratchBuffer.InterlockedMax(byteOffset + 20, waveMaxUint.z, outValue);
    }
}

//=====================================================================================================================
void UpdateSceneBoundsUsingCentroid(RWByteAddressBuffer scratchBuffer, uint byteOffset, float3 centroidPoint)
{
    // Calculate the combined AABB for the entire wave.
    const float3 waveBoundsMin = WaveActiveMin(centroidPoint);
    const float3 waveBoundsMax = WaveActiveMax(centroidPoint);

    //TODO: can just use centroids rather than boxes

    // Calculate the AABB for the entire scene using memory atomics.
    // Scalarize the atomic min/max writes by only using the first lane.
    if (WaveIsFirstLane())
    {
        // Convert the wave bounds to uints so we can atomically min/max them against the scene bounds in memory.
        const uint3 waveMinUint = Float3ToUint3(waveBoundsMin);
        const uint3 waveMaxUint = Float3ToUint3(waveBoundsMax);

        uint outValue;
        scratchBuffer.InterlockedMin(byteOffset,     waveMinUint.x, outValue);
        scratchBuffer.InterlockedMin(byteOffset + 4, waveMinUint.y, outValue);
        scratchBuffer.InterlockedMin(byteOffset + 8, waveMinUint.z, outValue);

        scratchBuffer.InterlockedMax(byteOffset + 12, waveMaxUint.x, outValue);
        scratchBuffer.InterlockedMax(byteOffset + 16, waveMaxUint.y, outValue);
        scratchBuffer.InterlockedMax(byteOffset + 20, waveMaxUint.z, outValue);
    }
}

//=====================================================================================================================
void UpdateCentroidSceneBoundsWithSize(RWByteAddressBuffer scratchBuffer, uint byteOffset, BoundingBox boundingBox)
{
    const float3 centroidPoint = (0.5 * (boundingBox.max + boundingBox.min));

    UpdateSceneBoundsUsingCentroid(scratchBuffer, byteOffset, centroidPoint);

    UpdateSceneSize(scratchBuffer, byteOffset + 24, ComputeBoxSurfaceArea(boundingBox));
}

//=====================================================================================================================
BoundingBox FetchSceneBounds(
    RWByteAddressBuffer scratchBuffer,
    uint                sceneBoundsOffset)
{
    UintBoundingBox sceneBounds;

    uint4 data;
    data            = scratchBuffer.Load4(sceneBoundsOffset);
    sceneBounds.min = data.xyz;
    data.xy         = scratchBuffer.Load2(sceneBoundsOffset + 0x10);
    sceneBounds.max = data.wxy;

    BoundingBox bbox;
    bbox.min = Uint3ToFloat3(sceneBounds.min);
    bbox.max = Uint3ToFloat3(sceneBounds.max);

    return bbox;
}

//=====================================================================================================================
float2 FetchSceneSize(
    RWByteAddressBuffer scratchBuffer,
    uint                sceneBoundsOffset)
{
    uint2 data;
    data.x          = scratchBuffer.Load(sceneBoundsOffset + 24);
    data.y          = scratchBuffer.Load(sceneBoundsOffset + 28);

    float2 minMax;
    minMax.x = UintToFloat(data.x);
    minMax.y = UintToFloat(data.y);

    return minMax;
}

//=====================================================================================================================
TriangleData FetchTriangleFromNode(
    RWByteAddressBuffer ResultBuffer,
    uint                metadataSize,
    uint                nodePointer)
{
    const uint  nodeOffset    = metadataSize + ExtractNodePointerOffset(nodePointer);
    const uint  nodeType      = GetNodeType(nodePointer);
    const uint3 vertexOffsets = CalcTriangleVertexOffsets(nodeType);

    TriangleData tri;
    tri.v0 = ResultBuffer.Load<float3>(nodeOffset + vertexOffsets.x);
    tri.v1 = ResultBuffer.Load<float3>(nodeOffset + vertexOffsets.y);
    tri.v2 = ResultBuffer.Load<float3>(nodeOffset + vertexOffsets.z);

    return tri;
}

//=====================================================================================================================
uint GetUpdateStackOffset(
    uint baseUpdateStackScratchOffset,
    uint stackIdx)
{
    // The update stack memory follows the stack pointer DWORD and each element is a DWORD
    return baseUpdateStackScratchOffset + sizeof(uint) + (stackIdx * sizeof(uint));
}

//=====================================================================================================================
void PushNodeToUpdateStack(
    RWByteAddressBuffer scratchBuffer,
    uint                baseUpdateStackScratchOffset,
    uint                parentNodePointer)
{
    uint stackPtr;
    scratchBuffer.InterlockedAdd(baseUpdateStackScratchOffset, 1, stackPtr);

    uint offset = GetUpdateStackOffset(baseUpdateStackScratchOffset, stackPtr);
    scratchBuffer.Store(offset, parentNodePointer);
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
    RWByteAddressBuffer DestBuffer,
    in uint             metadataSizeInBytes,
    in uint             packedNodePtr,
    in uint             parentPtr)
{
    // Store parent pointer in metadata buffer in reverse order
    const uint nodePtr = ExtractNodePointerCollapse(packedNodePtr);
    DestBuffer.Store(metadataSizeInBytes - CalcParentPtrOffset(nodePtr), parentPtr);
}

//=====================================================================================================================
uint GetChildCount(GpuVirtualAddress address, uint nodePointer)
{
    uint count = 0;
    if (IsBoxNode16(nodePointer))
    {
        const Float16BoxNode node = FetchFloat16BoxNode(address, nodePointer);
        count = 1 + (node.child1 != INVALID_IDX) + (node.child2 != INVALID_IDX) + (node.child3 != INVALID_IDX);
    }
    else
    {
        const Float32BoxNode node = FetchFloat32BoxNode(address, nodePointer);
        count = 1 + (node.child1 != INVALID_IDX) + (node.child2 != INVALID_IDX) + (node.child3 != INVALID_IDX);
    }

    return count;
}

//=====================================================================================================================
void WriteBoxNode(
    RWByteAddressBuffer ResultBuffer,
    in uint             offset,
    in Float32BoxNode   f32BoxNode)
{
    ResultBuffer.Store<Float32BoxNode>(offset, f32BoxNode);
}

//=====================================================================================================================
void WriteFp16BoxNode(
    RWByteAddressBuffer ResultBuffer,
    in uint             offset,
    in Float16BoxNode   f16BoxNode)
{
    ResultBuffer.Store<Float16BoxNode>(offset, f16BoxNode);
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
// Task Counter Macros
// INIT_TASK is used at the beginning of a shader to allow for waves to atomically fetch and task index
// BEGIN_TASK is used at the start of a pass that needs to be completed before the next pass
// END_TASK is used at the end of of a pass that makes sure the task is completed before moving onto the next pass
//
// INIT_LOOP_TASK is used at the beginning of a dynamic loop to allow for waves to atomically fetch and task index
// BEGIN_LOOP_TASK is used at the start of a pass that needs to be completed before the next pass inside a dynamic loop
// END_LOOP_TASK is used at the end of of a pass that makes sure the task is completed before moving onto the next pass
//=====================================================================================================================
#define INIT_TASK           if(localId == 0)\
                            {\
                                ResultMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET, 1, SharedIndex);\
                            }\
                            GroupMemoryBarrierWithGroupSync();\
                            waveId = SharedIndex;

#define BEGIN_TASK(n)       while((waveId >= numTasksWait) && (waveId < (numTasksWait + n)))\
                            {\
                                uint globalId = (waveId - numTasksWait) * BUILD_THREADGROUP_SIZE + localId;\
                                uint groupId = (waveId - numTasksWait);

#define END_TASK(n)             DeviceMemoryBarrierWithGroupSync();\
                                if(localId == 0)\
                                {\
                                    ResultMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET, 1, SharedIndex);\
                                    ResultMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_NUM_TASKS_DONE_OFFSET, 1);\
                                }\
                                GroupMemoryBarrierWithGroupSync();\
                                waveId = SharedIndex;\
                            }\
                            numTasksWait += n;\
                            do\
                            {\
                                DeviceMemoryBarrier();\
                            } while (ResultMetadata.Load(ACCEL_STRUCT_METADATA_NUM_TASKS_DONE_OFFSET) < numTasksWait);

#define INIT_LOOP_TASK      if(localId == 0)\
                            {\
                                ResultMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_LOOP_TASK_COUNTER_OFFSET, 1, SharedIndex);\
                            }\
                            GroupMemoryBarrierWithGroupSync();\
                            waveId2 = SharedIndex;

#define BEGIN_LOOP_TASK(n)  while((waveId2 >= numTasksWait2) && (waveId2 < (numTasksWait2 + n)))\
                            {\
                                uint globalId = (waveId2 - numTasksWait2) * BUILD_THREADGROUP_SIZE + localId;\
                                uint groupId = (waveId2 - numTasksWait2);

#define END_LOOP_TASK(n)        DeviceMemoryBarrierWithGroupSync();\
                                if(localId == 0)\
                                {\
                                    ResultMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_LOOP_TASK_COUNTER_OFFSET, 1, SharedIndex);\
                                    ResultMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_NUM_LOOP_TASKS_DONE_OFFSET, 1);\
                                }\
                                GroupMemoryBarrierWithGroupSync();\
                                waveId2 = SharedIndex;\
                            }\
                            numTasksWait2 += n;\
                            do\
                            {\
                                DeviceMemoryBarrier();\
                            } while ((ResultMetadata.Load(ACCEL_STRUCT_METADATA_NUM_LOOP_TASKS_DONE_OFFSET) < numTasksWait2));

#define INIT_LOOP_TASK2     if(localId == 0)\
                            {\
                                ResultMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_LOOP2_TASK_COUNTER_OFFSET, 1, SharedIndex);\
                            }\
                            GroupMemoryBarrierWithGroupSync();\
                            waveId3 = SharedIndex;

#define BEGIN_LOOP_TASK2(n) while((waveId3 >= numTasksWait3) && (waveId3 < (numTasksWait3 + n)))\
                            {\
                                uint globalId = (waveId3 - numTasksWait3) * BUILD_THREADGROUP_SIZE + localId;\
                                uint groupId = (waveId3 - numTasksWait3);

#define END_LOOP_TASK2(n)       DeviceMemoryBarrierWithGroupSync();\
                                if(localId == 0)\
                                {\
                                    ResultMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_LOOP2_TASK_COUNTER_OFFSET, 1, SharedIndex);\
                                    ResultMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_NUM_LOOP2_TASKS_DONE_OFFSET, 1);\
                                }\
                                GroupMemoryBarrierWithGroupSync();\
                                waveId3 = SharedIndex;\
                            }\
                            numTasksWait3 += n;\
                            do\
                            {\
                                DeviceMemoryBarrier();\
                            } while ((ResultMetadata.Load(ACCEL_STRUCT_METADATA_NUM_LOOP2_TASKS_DONE_OFFSET) < numTasksWait3));
