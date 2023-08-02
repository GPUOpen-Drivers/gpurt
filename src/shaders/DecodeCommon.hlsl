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
#ifndef _DECODECOMMON_HLSL
#define _DECODECOMMON_HLSL

#include "Common.hlsl"

// D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_TOOLS_VISUALIZATION_HEADER
#define VISUALIZATION_HEADER_SIZE 8

#define VISUALIZATION_HEADER_TYPE_OFFSET      0
#define VISUALIZATION_HEADER_NUM_DESCS_OFFSET 4

// D3D12_RAYTRACING_GEOMETRY_DESC
#define GEOMETRY_DESC_SIZE 56

#define GEOMETRY_DESC_TYPE_OFFSET  0 // D3D12_RAYTRACING_GEOMETRY_TYPE
#define GEOMETRY_DESC_FLAGS_OFFSET 4 // D3D12_RAYTRACING_GEOMETRY_FLAGS

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

//=====================================================================================================================
static uint32_t FetchInstanceIndex(
    in uint32_t            bufferOffset,
    in AccelStructHeader   header,
    in uint32_t            nodePtr)
{
    const uint32_t sidebandOffset = GetInstanceSidebandOffset(header, nodePtr);
    return SrcBuffer.Load(
        bufferOffset + header.metadataSizeInBytes + sidebandOffset + RTIP1_1_INSTANCE_SIDEBAND_INSTANCE_INDEX_OFFSET);
}

//=====================================================================================================================
static InstanceDesc DecodeApiInstanceDesc(
    in AccelStructHeader header,
    in uint nodePtr)
{
    InstanceDesc apiInstanceDesc = (InstanceDesc)0;

    uint32_t blasMetadataSizeInBytes = 0;

    {
        const uint offset = header.metadataSizeInBytes + ExtractNodePointerOffset(nodePtr);
        apiInstanceDesc = SrcBuffer.Load<InstanceDesc>(offset);

        const InstanceSidebandData1_1 sideband =
            SrcBuffer.Load<InstanceSidebandData1_1>(offset + sizeof(InstanceDesc));

        apiInstanceDesc.Transform[0] = sideband.Transform[0];
        apiInstanceDesc.Transform[1] = sideband.Transform[1];
        apiInstanceDesc.Transform[2] = sideband.Transform[2];

        blasMetadataSizeInBytes =
            SrcBuffer.Load(offset + INSTANCE_NODE_EXTRA_OFFSET + RTIP1_1_INSTANCE_SIDEBAND_CHILD_METADATA_SIZE_OFFSET);
    }

    uint64_t gpuVa =
        PackUint64(apiInstanceDesc.accelStructureAddressLo, apiInstanceDesc.accelStructureAddressHiAndFlags);

    // Mask off instance flags in the upper bits of the acceleration structure address
    gpuVa = ExtractInstanceAddr(gpuVa);

    // Skip null BLAS
    if (gpuVa != 0)
    {
        // Offset to base of bottom level acceleration structure
        gpuVa -= blasMetadataSizeInBytes;
    }

    // Patch instance address
    apiInstanceDesc.accelStructureAddressLo = LowPart(gpuVa);
    apiInstanceDesc.accelStructureAddressHiAndFlags = HighPart(gpuVa);

    return apiInstanceDesc;
}

//=====================================================================================================================
// Additional driver decode header data
static uint GetDriverDecodeHeaderSize()
{
    return sizeof(DriverDecodeHeader);
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
