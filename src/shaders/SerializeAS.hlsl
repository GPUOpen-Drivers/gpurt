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
#define RootSig "RootConstants(num32BitConstants=1, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL)"

//=====================================================================================================================
// 32 bit constants
struct InputArgs
{
    uint numWaves;
};

[[vk::push_constant]] ConstantBuffer<InputArgs> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] RWByteAddressBuffer DstBuffer     : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer SrcBuffer     : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer SrcMetadata   : register(u2);

// unused buffer
[[vk::binding(3, 0)]] RWByteAddressBuffer DstMetadata   : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer ScratchBuffer : register(u4);
[[vk::binding(5, 0)]] RWByteAddressBuffer EmitBuffer    : register(u5);

#include "Common.hlsl"
#include "SerializeCommon.hlsl"
#include "DecodeCommon.hlsl"

//=====================================================================================================================
// SerializeAS
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void SerializeAS(in uint3 globalThreadId : SV_DispatchThreadID)
{
    const uint globalID = globalThreadId.x;

    // Acceleration structure header follows metadata
    const uint metadataSizeInBytes = SrcMetadata.Load<uint>(ACCEL_STRUCT_METADATA_SIZE_OFFSET);
    const AccelStructHeader header = SrcBuffer.Load<AccelStructHeader>(metadataSizeInBytes);
    const uint type = (header.info & ACCEL_STRUCT_HEADER_INFO_TYPE_MASK);

    const uint sizeOfPtrs = GPUVA_SIZE * header.numDescs;

    uint serializedHeaderSize = 0;
    if (type == TOP_LEVEL)
    {
        serializedHeaderSize = SERIALIZED_AS_HEADER_SIZE + sizeOfPtrs;
    }
    else
    {
        serializedHeaderSize = SERIALIZED_AS_HEADER_SIZE;
    }

    // write D3D12_SERIALIZED_ACCELERATION_STRUCTURE_HEADER
    if (globalID == 0)
    {
        // 0 - DataDriverMatchingIdentifier
        DstBuffer.Store4(0, uint4(GPURT_AMD_GUID_0, GPURT_AMD_GUID_1, GPURT_AMD_GUID_2, GPURT_AMD_GUID_3));
        DstBuffer.Store4(16, uint4(header.uuidLo, header.uuidHi, header.accelStructVersion, 0));

        // 1 - SerializedSizeInBytesIncludingHeader
        DstBuffer.Store2(SERIALIZED_AS_HEADER_SERIALIZED_SIZE_OFFSET, uint2(header.sizeInBytes + serializedHeaderSize, 0));

        // 2 - DeserializedSizeInBytes
        DstBuffer.Store2(SERIALIZED_AS_HEADER_DESERIALIZED_SIZE_OFFSET, uint2(header.sizeInBytes, 0));

        if (type == TOP_LEVEL)
        {
            // 3 - NumBottomLevelAccelerationStructurePointersAfterHeader
            DstBuffer.Store2(SERIALIZED_AS_HEADER_NUM_BLAS_PTRS_OFFSET, uint2(header.numDescs, 0));
        }
        else
        {
            // 3 - NumBottomLevelAccelerationStructurePointersAfterHeader
            DstBuffer.Store<uint64_t>(SERIALIZED_AS_HEADER_NUM_BLAS_PTRS_OFFSET, 0);
        }
    }

    if (type == TOP_LEVEL)
    {
        // Bottom tree GPUVAs after header (SERIALIZED_AS_HEADER_SIZE)
        const uint basePrimNodePtrsOffset = metadataSizeInBytes + header.offsets.primNodePtrs;

        // Loop over active primitives since there may be more or less valid instances than the original API
        // instance count when rebraid is enabled.
        const uint rebraid =
            (header.info >> ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_SHIFT) &
                ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_MASK;

        const uint numInstances = (rebraid != 0) ? header.numActivePrims : header.numDescs;

        for (uint i = globalID; i < numInstances; i += BUILD_THREADGROUP_SIZE * ShaderConstants.numWaves)
        {
            const uint currentInstNodePtrOffset = basePrimNodePtrsOffset + (i * NODE_PTR_SIZE);
            const uint currentInstNodePtr = SrcBuffer.Load(currentInstNodePtrOffset);

            // Note, without rebraid the instance nodes are in API order with deactivated instances
            // indicated by an invalid node pointer.
            uint32_t apiInstanceIndex = i;

            // Skip inactive instances which have their node pointers set to invalid. Note, these only appear with
            // rebraid disabled
            uint64_t gpuVa = 0;

            if (currentInstNodePtr != INVALID_IDX)
            {
                InstanceDesc apiInstanceDesc = DecodeApiInstanceDesc(header, currentInstNodePtr);

                gpuVa = PackUint64(apiInstanceDesc.accelStructureAddressLo, apiInstanceDesc.accelStructureAddressHiAndFlags);

                // Fetch API instance index from instance node. With rebraid enabled the instance node pointers
                // in memory are in sorted order with no deactivated instances in between. The re-braided instances
                // are mixed in this array so we need to read the instance index from memory to account for
                // all API instances. There is some duplication here since we may have multiple leaf nodes
                // pointing to same instance but Serialize performance is not of great concern.

                {
                    apiInstanceIndex = FetchInstanceIndex(0, header, currentInstNodePtr);
                }
            }

            if (apiInstanceIndex < header.numDescs)
            {
                // Write original BLAS address to destination buffer
                DstBuffer.Store<uint64_t>((apiInstanceIndex * sizeof(uint64_t)) + SERIALIZED_AS_HEADER_SIZE, gpuVa);
            }
        }
    }

    const uint sizeInDwords = header.sizeInBytes >> 2;

    // copy from SrcBuffer->DstBuffer skipping over D3D12_SERIALIZED_ACCELERATION_STRUCTURE_HEADER and list of ptrs
    for (uint i = globalID; i < sizeInDwords; i += BUILD_THREADGROUP_SIZE * ShaderConstants.numWaves)
    {
        const uint byteOffset = i << 2;

        DstBuffer.Store(byteOffset + serializedHeaderSize, SrcBuffer.Load(byteOffset));
    }
}
