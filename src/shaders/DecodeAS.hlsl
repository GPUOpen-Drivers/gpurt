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
#define RootSig "RootConstants(num32BitConstants=5, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL)"

//=====================================================================================================================
// 32 bit constants
struct InputArgs
{
    uint AddressLo;
    uint AddressHi;
    uint NumThreads;
    uint rtIpLevel;
    uint isDriverDecode;
};

[[vk::push_constant]] ConstantBuffer<InputArgs> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer      DstBuffer     : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer                       SrcBuffer     : register(u1);

// unused buffer
[[vk::binding(2, 0)]] RWByteAddressBuffer                       DstMetadata   : register(u2);

#include "Common.hlsl"
#include "DecodeCommon.hlsl"

//=====================================================================================================================
// DecodeAS
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void DecodeAS(in uint3 globalThreadId : SV_DispatchThreadID)
{
    const uint globalID = globalThreadId.x;

    // The following code assumes the base address of SrcBuffer and DstBuffer includes the acceleration structure
    // metadata header

    // Fetch acceleration structure metadata size
    const uint32_t metadataSizeInBytes = SrcBuffer.Load<uint32_t>(ACCEL_STRUCT_METADATA_SIZE_OFFSET);
    const AccelStructHeader header = SrcBuffer.Load<AccelStructHeader>(metadataSizeInBytes);

    const uint type           = (header.info & ACCEL_STRUCT_HEADER_INFO_TYPE_MASK);
    const uint numPrimitives  = header.numPrimitives;
    const uint numActivePrims = header.numActivePrims;

    const uint triangleCompressionMode =
        (header.info >> ACCEL_STRUCT_HEADER_INFO_TRI_COMPRESS_SHIFT) & ACCEL_STRUCT_HEADER_INFO_TRI_COMPRESS_MASK;

    const AccelStructOffsets baseOffsets = header.offsets;

    const uint geometryType               = header.geometryType;
    const uint baseGeometryInfoOffset     = metadataSizeInBytes + baseOffsets.geometryInfo;
    const uint basePrimNodePointersOffset = metadataSizeInBytes + baseOffsets.primNodePtrs;

    if (globalID == 0)
    {
        uint apiHeaderOffset = 0;
        if (ShaderConstants.isDriverDecode)
        {
            WriteDriverDecodeHeader(header, 0);
            apiHeaderOffset += GetDriverDecodeHeaderSize();
        }

        // D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_TOOLS_VISUALIZATION_HEADER
        DstBuffer.Store(apiHeaderOffset + VISUALIZATION_HEADER_TYPE_OFFSET, type);
        DstBuffer.Store(apiHeaderOffset + VISUALIZATION_HEADER_NUM_DESCS_OFFSET, header.numDescs);
    }

    uint headerSize = VISUALIZATION_HEADER_SIZE;
    if (ShaderConstants.isDriverDecode)
    {
        headerSize += GetDriverDecodeHeaderSize();
    }

    if (type == TOP_LEVEL)
    {
        // @note Prim node pointers of rebraided instances are allocated in memory after the prim node pointers of the
        //       API input instances, so we can just use numDescs since we do not want rebraided instances here.
        const uint numInstanceDescs = header.numDescs;

        for (uint i = globalID; i < numInstanceDescs; i += ShaderConstants.NumThreads)
        {
            const uint dstInstanceDescOffset = headerSize + (i * INSTANCE_DESC_SIZE);
            const uint nodePointer           = SrcBuffer.Load(basePrimNodePointersOffset + (i * NODE_PTR_SIZE));

            if (nodePointer != INVALID_IDX)
            {
                InstanceDesc apiInstanceDesc = DecodeApiInstanceDesc(header, nodePointer);
                DstBuffer.Store<InstanceDesc>(dstInstanceDescOffset, apiInstanceDesc);
            }
            else
            {
                InstanceDesc apiInstanceDesc = (InstanceDesc)0;
                DstBuffer.Store<InstanceDesc>(dstInstanceDescOffset, apiInstanceDesc);
            }
        }
    }
    else
    {
        const uint numGeometryDescs = header.numDescs;

        const uint baseVertexAndAabbOffset = headerSize + (numGeometryDescs * GEOMETRY_DESC_SIZE);

        // Init geometry descs
        for (uint geomIdx = globalID; geomIdx < numGeometryDescs; geomIdx += ShaderConstants.NumThreads)
        {
            const uint geometryInfoOffset     = baseGeometryInfoOffset + (geomIdx * GEOMETRY_INFO_SIZE);
            const uint geometryBufferOffset   = SrcBuffer.Load(geometryInfoOffset + GEOMETRY_INFO_GEOM_BUFFER_OFFSET);
            const uint packedGeometryInfoData = SrcBuffer.Load(geometryInfoOffset + GEOMETRY_INFO_FLAGS_AND_NUM_PRIMS_OFFSET);
            const uint geometryNumPrimitives  = ExtractGeometryInfoNumPrimitives(packedGeometryInfoData);
            const uint geometryFlags          = ExtractGeometryInfoFlags(packedGeometryInfoData);

            uint addressLo = ShaderConstants.AddressLo;
            uint addressHi = ShaderConstants.AddressHi;

            addressLo += baseVertexAndAabbOffset + geometryBufferOffset;

            if (addressLo < ShaderConstants.AddressLo)
            {
                addressHi++;
            }

            const uint dstGeometryDescOffset = headerSize + (geomIdx * GEOMETRY_DESC_SIZE);

            DstBuffer.Store(dstGeometryDescOffset, geometryType);
            DstBuffer.Store(dstGeometryDescOffset + GEOMETRY_DESC_FLAGS_OFFSET, geometryFlags);

            if (geometryType == GEOMETRY_TYPE_TRIANGLES)
            {
                const uint vertexCount = geometryNumPrimitives * 3;

                DstBuffer.Store2(dstGeometryDescOffset + GEOMETRY_DESC_TRANSFORM_OFFSET,     uint2(0, 0));
                DstBuffer.Store(dstGeometryDescOffset  + GEOMETRY_DESC_INDEX_FORMAT_OFFSET,  DXGI_FORMAT_UNKNOWN);
                DstBuffer.Store(dstGeometryDescOffset  + GEOMETRY_DESC_VERTEX_FORMAT_OFFSET, DXGI_FORMAT_R32G32B32_FLOAT);
                DstBuffer.Store(dstGeometryDescOffset  + GEOMETRY_DESC_INDEX_COUNT_OFFSET,   0);
                DstBuffer.Store(dstGeometryDescOffset  + GEOMETRY_DESC_VERTEX_COUNT_OFFSET,  vertexCount);
                DstBuffer.Store2(dstGeometryDescOffset + GEOMETRY_DESC_INDEX_BUFFER_OFFSET,  uint2(0, 0));
                DstBuffer.Store4(dstGeometryDescOffset + GEOMETRY_DESC_VERTEX_BUFFER_OFFSET,
                                 uint4(addressLo, addressHi, DECODE_VERTEX_STRIDE, 0));
            }
            else // GEOMETRY_TYPE_AABBS
            {
                DstBuffer.Store(dstGeometryDescOffset + GEOMETRY_DESC_AABB_COUNT_OFFSET, geometryNumPrimitives);
                DstBuffer.Store4(dstGeometryDescOffset + GEOMETRY_DESC_AABBS_OFFSET,
                                 uint4(addressLo, addressHi, DECODE_PRIMITIVE_STRIDE_AABB, 0));
            }
        }

        for (uint i = globalID; i < numPrimitives; i += ShaderConstants.NumThreads)
        {
            // Initialize all vertex/AABB buffer primitives to inactive.
            if (geometryType == GEOMETRY_TYPE_TRIANGLES)
            {
                const uint primOffset = baseVertexAndAabbOffset + (i * DECODE_PRIMITIVE_STRIDE_TRIANGLE);
                DstBuffer.Store<float>(primOffset,                              NaN);
                DstBuffer.Store<float>(primOffset + DECODE_VERTEX_STRIDE,       NaN);
                DstBuffer.Store<float>(primOffset + (DECODE_VERTEX_STRIDE * 2), NaN);
            }
            else
            {
                const uint primOffset = baseVertexAndAabbOffset + (i * DECODE_PRIMITIVE_STRIDE_AABB);
                DstBuffer.Store<float>(primOffset, NaN);
            }
        }

        const uint64_t srcBvhAddress = SrcBuffer.Load<uint64_t>(ACCEL_STRUCT_METADATA_VA_LO_OFFSET);

        {
            // Copy vertices and aabbs
            for (uint i = globalID; i < numPrimitives; i += ShaderConstants.NumThreads)
            {
                const uint nodePointer = SrcBuffer.Load(basePrimNodePointersOffset + (i * NODE_PTR_SIZE));
                if (nodePointer != INVALID_IDX)
                {
                    // The stored node pointers do not account for the metadata, so add the metadata size to the node
                    // pointer address field before using it for offset calculations.
                    const uint srcLeafNodeOffset = ExtractNodePointerOffset(nodePointer) + metadataSizeInBytes;

                    PrimitiveData primitiveData;
                    const GpuVirtualAddress nodeAddress = srcBvhAddress + ExtractNodePointerOffset(nodePointer);

                    {
                        primitiveData = FetchPrimitiveDataAddr(nodePointer, nodeAddress);
                    }

                    const uint geometryIndex  = primitiveData.geometryIndex;
                    const uint primitiveIndex = primitiveData.primitiveIndex;

                    const uint geometryInfoOffset = baseGeometryInfoOffset  + (geometryIndex * GEOMETRY_INFO_SIZE);
                    const uint baseGeometryOffset = baseVertexAndAabbOffset +
                                                    SrcBuffer.Load(geometryInfoOffset + GEOMETRY_INFO_GEOM_BUFFER_OFFSET);

                    if (geometryType == GEOMETRY_TYPE_TRIANGLES)
                    {
                        const uint geometryIndex  = primitiveData.geometryIndex;
                        const uint primitiveIndex = primitiveData.primitiveIndex;

                        const uint geometryInfoOffset = baseGeometryInfoOffset  + (geometryIndex * GEOMETRY_INFO_SIZE);
                        const uint baseGeometryOffset = baseVertexAndAabbOffset +
                                                        SrcBuffer.Load(geometryInfoOffset + GEOMETRY_INFO_GEOM_BUFFER_OFFSET);

                        uint3 v0;
                        uint3 v1;
                        uint3 v2;

                        if (triangleCompressionMode != NO_TRIANGLE_COMPRESSION)
                        {
                            const uint  triangleId = SrcBuffer.Load(srcLeafNodeOffset + TRIANGLE_NODE_ID_OFFSET);
                            const uint  nodeType = GetNodeType(nodePointer);
                            const uint3 vertexOffsets = CalcTriangleCompressionVertexOffsets(nodeType, triangleId);

                            v0 = SrcBuffer.Load3(srcLeafNodeOffset + vertexOffsets.x);
                            v1 = SrcBuffer.Load3(srcLeafNodeOffset + vertexOffsets.y);
                            v2 = SrcBuffer.Load3(srcLeafNodeOffset + vertexOffsets.z);
                        }
                        else
                        {
                            v0 = SrcBuffer.Load3(srcLeafNodeOffset + TRIANGLE_NODE_V0_OFFSET);
                            v1 = SrcBuffer.Load3(srcLeafNodeOffset + TRIANGLE_NODE_V1_OFFSET);
                            v2 = SrcBuffer.Load3(srcLeafNodeOffset + TRIANGLE_NODE_V2_OFFSET);
                        }

                        const uint byteOffset = baseGeometryOffset + (primitiveIndex * DECODE_PRIMITIVE_STRIDE_TRIANGLE);
                        DstBuffer.Store3(byteOffset,                              v0);
                        DstBuffer.Store3(byteOffset + DECODE_VERTEX_STRIDE,       v1);
                        DstBuffer.Store3(byteOffset + (DECODE_VERTEX_STRIDE * 2), v2);
                    }
                    else
                    {
                        const uint3 min = SrcBuffer.Load3(srcLeafNodeOffset + USER_NODE_PROCEDURAL_MIN_OFFSET);
                        const uint3 max = SrcBuffer.Load3(srcLeafNodeOffset + USER_NODE_PROCEDURAL_MAX_OFFSET);

                        const uint byteOffset = baseGeometryOffset + (primitiveIndex * DECODE_PRIMITIVE_STRIDE_AABB);
                        DstBuffer.Store3(byteOffset, min);
                        DstBuffer.Store3(byteOffset + 12, max);
                    }
                }
            }
        }
    }
}
