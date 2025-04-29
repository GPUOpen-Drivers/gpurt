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

#include "BoxNode1_0.hlsli"

#include "../BoundingBox.hlsli"
#include "../ObbCommon.hlsli"

#include "../Extensions.hlsli"

#if GPURT_BUILD_RTIP3
#include "../gfx12/HighPrecisionBoxNode.hlsli"
#endif

//=====================================================================================================================
static uint FetchFloat32BoxNodeNumPrimitives(in GpuVirtualAddress bvhAddress, in uint nodePointer)
{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    const GpuVirtualAddress nodeAddr = bvhAddress + byteOffset + FLOAT32_BOX_NODE_NUM_PRIM_OFFSET;

    return LoadDwordAtAddr(nodeAddr);
}

//=====================================================================================================================
static Float32BoxNode FetchFloat32BoxNode(in GpuVirtualAddress bvhAddress,
#if GPURT_BUILD_RTIP3
                                          in bool              highPrecisionBoxNodeEnable,
#endif
                                          in uint              nodePointer)

{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    const GpuVirtualAddress nodeAddr = bvhAddress + byteOffset;

    uint4 d0, d1, d2, d3, d4, d5, d6;
    d0 = LoadDwordAtAddrx4(nodeAddr);
    d1 = LoadDwordAtAddrx4(nodeAddr + 0x10);
    d2 = LoadDwordAtAddrx4(nodeAddr + 0x20);
    d3 = LoadDwordAtAddrx4(nodeAddr + 0x30);

#if GPURT_BUILD_RTIP3
    if (highPrecisionBoxNodeEnable)
    {
        HighPrecisionBoxNode hpBoxNode = (HighPrecisionBoxNode)0;

        hpBoxNode.dwords[0] = d0.x;
        hpBoxNode.dwords[1] = d0.y;
        hpBoxNode.dwords[2] = d0.z;
        hpBoxNode.dwords[3] = d0.w;
        hpBoxNode.dwords[4] = d1.x;
        hpBoxNode.dwords[5] = d1.y;
        hpBoxNode.dwords[6] = d1.z;
        hpBoxNode.dwords[7] = d1.w;
        hpBoxNode.dwords[8] = d2.x;
        hpBoxNode.dwords[9] = d2.y;
        hpBoxNode.dwords[10] = d2.z;
        hpBoxNode.dwords[11] = d2.w;
        hpBoxNode.dwords[12] = d3.x;
        hpBoxNode.dwords[13] = d3.y;
        hpBoxNode.dwords[14] = d3.z;
        hpBoxNode.dwords[15] = d3.w;

        const Float32BoxNode node = DecodeHighPrecisionBoxNode(hpBoxNode);

        return node;
    }
#endif
    d4 = LoadDwordAtAddrx4(nodeAddr + 0x40);
    d5 = LoadDwordAtAddrx4(nodeAddr + 0x50);
    d6 = LoadDwordAtAddrx4(nodeAddr + 0x60);

    Float32BoxNode node;

    node.child0 = d0.x;
    node.child1 = d0.y;
    node.child2 = d0.z;
    node.child3 = d0.w;

    node.bbox0_min = asfloat(d1.xyz);
    node.bbox0_max = float3(asfloat(d1.w), asfloat(d2.xy));
    node.bbox1_min = float3(asfloat(d2.zw), asfloat(d3.x));
    node.bbox1_max = asfloat(d3.yzw);
    node.bbox2_min = asfloat(d4.xyz);
    node.bbox2_max = float3(asfloat(d4.w), asfloat(d5.xy));
    node.bbox3_min = float3(asfloat(d5.zw), asfloat(d6.x));
    node.bbox3_max = asfloat(d6.yzw);

    node.flags = LoadDwordAtAddr(nodeAddr + FLOAT32_BOX_NODE_FLAGS_OFFSET);

#if GPURT_BUILD_RTIP3_1
    node.obbMatrixIndex = LoadDwordAtAddr(nodeAddr + FLOAT32_BOX_NODE_OBB_OFFSET);
#endif

    return node;
}

//=====================================================================================================================
static Float32BoxNode FetchFloat16BoxNodeAsFp32(in GpuVirtualAddress bvhAddress, in uint nodePointer)
{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    const GpuVirtualAddress nodeAddr = bvhAddress + byteOffset;

    uint4 d0, d1, d2, d3;
    d0 = LoadDwordAtAddrx4(nodeAddr);
    d1 = LoadDwordAtAddrx4(nodeAddr + 0x10);
    d2 = LoadDwordAtAddrx4(nodeAddr + 0x20);
    d3 = LoadDwordAtAddrx4(nodeAddr + 0x30);

    Float32BoxNode node;

    node.child0 = d0.x;
    node.child1 = d0.y;
    node.child2 = d0.z;
    node.child3 = d0.w;

    const BoundingBox b0 = UncompressBBoxFromUint3(d1.xyz);
    const BoundingBox b1 = UncompressBBoxFromUint3(uint3(d1.w,  d2.xy));
    const BoundingBox b2 = UncompressBBoxFromUint3(uint3(d2.zw, d3.x ));
    const BoundingBox b3 = UncompressBBoxFromUint3(d3.yzw);

    node.bbox0_min = b0.min;
    node.bbox0_max = b0.max;
    node.bbox1_min = b1.min;
    node.bbox1_max = b1.max;
    node.bbox2_min = b2.min;
    node.bbox2_max = b2.max;
    node.bbox3_min = b3.min;
    node.bbox3_max = b3.max;

#if GPURT_BUILD_RTIP3_1
    node.obbMatrixIndex = INVALID_OBB;
#endif

    // fp16 node does not have space to store flags,
    // initialize the field to 0.
    node.flags = 0;

    return node;
}

//=====================================================================================================================
static Float16BoxNode FetchFloat16BoxNode(in GpuVirtualAddress bvhAddress, in uint nodePointer)
{
    const uint byteOffset = ExtractNodePointerOffset(nodePointer);
    const GpuVirtualAddress nodeAddr = bvhAddress + byteOffset;

    uint4 d0, d1, d2, d3;
    d0 = LoadDwordAtAddrx4(nodeAddr);
    d1 = LoadDwordAtAddrx4(nodeAddr + 0x10);
    d2 = LoadDwordAtAddrx4(nodeAddr + 0x20);
    d3 = LoadDwordAtAddrx4(nodeAddr + 0x30);

    Float16BoxNode node;

    node.child0 = d0.x;
    node.child1 = d0.y;
    node.child2 = d0.z;
    node.child3 = d0.w;

    node.bbox0 = d1.xyz;
    node.bbox1 = uint3(d1.w,  d2.xy);
    node.bbox2 = uint3(d2.zw, d3.x );
    node.bbox3 = d3.yzw;

    return node;
}

//=====================================================================================================================
// Get internal BVH node size in bytes
static uint GetBvhNodeSizeInternal()
{
    return FLOAT32_BOX_NODE_SIZE;
}
