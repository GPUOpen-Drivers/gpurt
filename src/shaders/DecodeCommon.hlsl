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
#ifndef _DECODECOMMON_HLSL
#define _DECODECOMMON_HLSL

#include "../shadersClean/common/Common.hlsli"

// The following defines match this D3D12 structure
//
/*
typedef struct D3D12DDI_GPU_VIRTUAL_ADDRESS_RANGE_AND_STRIDE
{
    D3D12DDI_GPU_VIRTUAL_ADDRESS    StartAddress;
    UINT64                          SizeInBytes;
    UINT64                          StrideInBytes;
} D3D12DDI_GPU_VIRTUAL_ADDRESS_RANGE_AND_STRIDE;

typedef struct D3D12DDI_RAYTRACING_GEOMETRY_TRIANGLES_DESC_0054
{
    D3D12DDI_GPU_VIRTUAL_ADDRESS            ColumnMajorTransform3x4;
    DXGI_FORMAT                             IndexFormat;
    DXGI_FORMAT                             VertexFormat;
    UINT                                    IndexCount;
    UINT                                    VertexCount;
    D3D12DDI_GPU_VIRTUAL_ADDRESS            IndexBuffer;
    D3D12DDI_GPU_VIRTUAL_ADDRESS_AND_STRIDE VertexBuffer;
} D3D12DDI_RAYTRACING_GEOMETRY_TRIANGLES_DESC_0054;
*/

#define GEOMETRY_DESC_TRANSFORM_OFFSET      8
#define GEOMETRY_DESC_INDEX_FORMAT_OFFSET  16
#define GEOMETRY_DESC_VERTEX_FORMAT_OFFSET 20
#define GEOMETRY_DESC_INDEX_COUNT_OFFSET   24
#define GEOMETRY_DESC_VERTEX_COUNT_OFFSET  28
#define GEOMETRY_DESC_INDEX_BUFFER_OFFSET  32
#define GEOMETRY_DESC_VERTEX_BUFFER_OFFSET 40

#define GEOMETRY_DESC_AABB_COUNT_OFFSET  8
#define GEOMETRY_DESC_AABBS_OFFSET      16

#if GPURT_BUILD_RTIP3
//=====================================================================================================================
static HwInstanceNode FetchHwInstanceNode(
    in uint                bufferOffset,
    in AccelStructHeader   header,
    in uint                nodeOffset)
{
    const uint offset = bufferOffset + header.metadataSizeInBytes + nodeOffset;

    HwInstanceNode node = (HwInstanceNode)0;

    const uint4 d0 = SrcBuffer.Load4(offset + 0x0);
    const uint4 d1 = SrcBuffer.Load4(offset + 0x10);
    const uint4 d2 = SrcBuffer.Load4(offset + 0x20);
    const uint4 d3 = SrcBuffer.Load4(offset + 0x30);

    node.data.worldToObject[0][0] = asfloat(d0.x);
    node.data.worldToObject[0][1] = asfloat(d0.y);
    node.data.worldToObject[0][2] = asfloat(d0.z);
    node.data.worldToObject[0][3] = asfloat(d0.w);
    node.data.worldToObject[1][0] = asfloat(d1.x);
    node.data.worldToObject[1][1] = asfloat(d1.y);
    node.data.worldToObject[1][2] = asfloat(d1.z);
    node.data.worldToObject[1][3] = asfloat(d1.w);
    node.data.worldToObject[2][0] = asfloat(d2.x);
    node.data.worldToObject[2][1] = asfloat(d2.y);
    node.data.worldToObject[2][2] = asfloat(d2.z);
    node.data.worldToObject[2][3] = asfloat(d2.w);

    node.data.childBasePtr             = PackUint64(d3.x, d3.y);
    node.data.childRootNodeOrParentPtr = d3.z;
    node.data.userDataAndInstanceMask  = d3.w;

    const uint sidebandOffset =
        bufferOffset + header.metadataSizeInBytes + GetInstanceSidebandOffset(header, nodeOffset);

    const uint4 d4 = SrcBuffer.Load4(sidebandOffset + 0x00);
    const uint4 d5 = SrcBuffer.Load4(sidebandOffset + 0x10);
    const uint4 d6 = SrcBuffer.Load4(sidebandOffset + 0x20);
    const uint4 d7 = SrcBuffer.Load4(sidebandOffset + 0x30);

    node.sideband.instanceIndex      = d4.x;
    node.sideband.instanceIdAndFlags = d4.y;
    node.sideband.blasMetadataSize   = d4.z;
    node.sideband.padding0           = d4.w;

    node.sideband.objectToWorld[0][0] = asfloat(d5.x);
    node.sideband.objectToWorld[0][1] = asfloat(d5.y);
    node.sideband.objectToWorld[0][2] = asfloat(d5.z);
    node.sideband.objectToWorld[0][3] = asfloat(d5.w);
    node.sideband.objectToWorld[1][0] = asfloat(d6.x);
    node.sideband.objectToWorld[1][1] = asfloat(d6.y);
    node.sideband.objectToWorld[1][2] = asfloat(d6.z);
    node.sideband.objectToWorld[1][3] = asfloat(d6.w);
    node.sideband.objectToWorld[2][0] = asfloat(d7.x);
    node.sideband.objectToWorld[2][1] = asfloat(d7.y);
    node.sideband.objectToWorld[2][2] = asfloat(d7.z);
    node.sideband.objectToWorld[2][3] = asfloat(d7.w);

    return node;
}

//=====================================================================================================================
static uint32_t FetchInstanceIndex3_0(
    in uint32_t            bufferOffset,
    in AccelStructHeader   header,
    in uint32_t            nodeOffset)
{
    const uint32_t sidebandOffset = GetInstanceSidebandOffset(header, nodeOffset);
    return SrcBuffer.Load(
        bufferOffset + header.metadataSizeInBytes + sidebandOffset + RTIP3_INSTANCE_SIDEBAND_INSTANCE_INDEX_OFFSET);
}
#endif

//=====================================================================================================================
static uint32_t FetchInstanceIndex(
    in uint32_t            bufferOffset,
    in AccelStructHeader   header,
    in uint32_t            nodeOffset)
{
    const uint32_t sidebandOffset = GetInstanceSidebandOffset(header, nodeOffset);
    return SrcBuffer.Load(
        bufferOffset + header.metadataSizeInBytes + sidebandOffset + RTIP1_1_INSTANCE_SIDEBAND_INSTANCE_INDEX_OFFSET);
}

//=====================================================================================================================
static InstanceDesc DecodeApiInstanceDesc(
    in AccelStructHeader header,
    in uint              nodeOffset)
{
    InstanceDesc apiInstanceDesc = (InstanceDesc)0;
    uint64_t gpuVa = 0;
    uint32_t blasMetadataSize = 0;

#if GPURT_BUILD_RTIP3
    if (header.UsesHardwareInstanceNode())
    {
        const HwInstanceNode hwInstanceNode = FetchHwInstanceNode(0, header, nodeOffset);

        apiInstanceDesc.Transform[0].x = hwInstanceNode.sideband.objectToWorld[0][0];
        apiInstanceDesc.Transform[0].y = hwInstanceNode.sideband.objectToWorld[0][1];
        apiInstanceDesc.Transform[0].z = hwInstanceNode.sideband.objectToWorld[0][2];
        apiInstanceDesc.Transform[0].w = hwInstanceNode.sideband.objectToWorld[0][3];
        apiInstanceDesc.Transform[1].x = hwInstanceNode.sideband.objectToWorld[1][0];
        apiInstanceDesc.Transform[1].y = hwInstanceNode.sideband.objectToWorld[1][1];
        apiInstanceDesc.Transform[1].z = hwInstanceNode.sideband.objectToWorld[1][2];
        apiInstanceDesc.Transform[1].w = hwInstanceNode.sideband.objectToWorld[1][3];
        apiInstanceDesc.Transform[2].x = hwInstanceNode.sideband.objectToWorld[2][0];
        apiInstanceDesc.Transform[2].y = hwInstanceNode.sideband.objectToWorld[2][1];
        apiInstanceDesc.Transform[2].z = hwInstanceNode.sideband.objectToWorld[2][2];
        apiInstanceDesc.Transform[2].w = hwInstanceNode.sideband.objectToWorld[2][3];

        apiInstanceDesc.InstanceID_and_Mask =
            (hwInstanceNode.data.userDataAndInstanceMask & 0xff000000) |
            (hwInstanceNode.sideband.instanceIdAndFlags & 0x00ffffff);

        apiInstanceDesc.InstanceContributionToHitGroupIndex_and_Flags =
            (hwInstanceNode.data.userDataAndInstanceMask & 0x00ffffff) |
            (hwInstanceNode.sideband.instanceIdAndFlags & 0xff000000);

        gpuVa = hwInstanceNode.data.childBasePtr;

        // Mask off instance flags in the upper bits of the acceleration structure address.
        gpuVa = ExtractInstanceAddr(gpuVa);

        blasMetadataSize = hwInstanceNode.sideband.blasMetadataSize;
    }
    else
#endif
    {
        const uint offset = header.metadataSizeInBytes + nodeOffset;
        apiInstanceDesc = SrcBuffer.Load<InstanceDesc>(offset);

        const InstanceSidebandData1_1 sideband =
            SrcBuffer.Load<InstanceSidebandData1_1>(offset + sizeof(InstanceDesc));

        apiInstanceDesc.Transform[0] = sideband.Transform[0];
        apiInstanceDesc.Transform[1] = sideband.Transform[1];
        apiInstanceDesc.Transform[2] = sideband.Transform[2];

        gpuVa = PackUint64(apiInstanceDesc.accelStructureAddressLo, apiInstanceDesc.accelStructureAddressHiAndFlags);

        // Mask off instance flags in the upper bits of the acceleration structure address.
        gpuVa = ExtractInstanceAddr(gpuVa);

        blasMetadataSize =
            SrcBuffer.Load(offset + INSTANCE_NODE_EXTRA_OFFSET + RTIP1_1_INSTANCE_SIDEBAND_CHILD_METADATA_SIZE_OFFSET);
    }

    // Skip null BLAS
    if (gpuVa != 0)
    {
        // Offset to base of bottom level acceleration structure
        gpuVa -= blasMetadataSize;
    }

    // Patch instance address
    apiInstanceDesc.accelStructureAddressLo = LowPart(gpuVa);
    apiInstanceDesc.accelStructureAddressHiAndFlags = HighPart(gpuVa);

    return apiInstanceDesc;
}

//=====================================================================================================================
static void WriteDriverDecodeHeader(
    in AccelStructHeader header,
    in uint              offset)
{
    DstBuffer.Store<AccelStructHeaderInfo>(offset, header.info);
    DstBuffer.Store<AccelStructHeaderInfo2>(offset + sizeof(AccelStructHeaderInfo), header.info2);
}

#endif
