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
#include "Common.hlsl"
#include "SerializeCommon.hlsl"
#include "DecodeCommon.hlsl"

#define RootSig "RootConstants(num32BitConstants=1, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL) "

//=====================================================================================================================
// 32 bit constants
struct InputArgs
{
    uint numWaves;
};

[[vk::push_constant]] ConstantBuffer<InputArgs> ShaderConstants : register(b0);

//=====================================================================================================================
[[vk::binding(0, 0)]] RWByteAddressBuffer DstBuffer   : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer SrcBuffer   : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer SrcMetadata : register(u2);

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
        // 1 - SerializedSizeInBytesIncludingHeader
        DstBuffer.Store2(SERIALIZED_AS_HEADER_SERIALIZED_SIZE_OFFSET, uint2(header.sizeInBytes + serializedHeaderSize, 0));

        // 2 - DeserializedSizeInBytes
        DstBuffer.Store2(SERIALIZED_AS_HEADER_DESERIALIZED_SIZE_OFFSET, uint2(header.sizeInBytes, 0));

        if (type == TOP_LEVEL)
        {
            const uint basePrimNodePtrsOffset = metadataSizeInBytes + header.offsets.primNodePtrs;

            // 3 - NumBottomLevelAccelerationStructurePointersAfterHeader
            DstBuffer.Store2(SERIALIZED_AS_HEADER_NUM_BLAS_PTRS_OFFSET, uint2(header.numDescs, 0));

            // Bottom tree GPUVAs after header (SERIALIZED_AS_HEADER_SIZE)
            for (uint i = 0; i < header.numDescs; i++)
            {
                const uint currentInstNodePtrOffset  = basePrimNodePtrsOffset + (i * NODE_PTR_SIZE);
                const uint currentInstNodePtr        = SrcBuffer.Load(currentInstNodePtrOffset);

                uint64_t gpuVA = 0;

                if (currentInstNodePtr != INVALID_IDX)
                {
                    const uint currentInstNodeOffset = metadataSizeInBytes +
                                                       ExtractNodePointerOffset(currentInstNodePtr);

                    // Fetch acceleration structure base address in TLAS instance
                    gpuVA = FetchApiInstanceBaseAddress(SrcBuffer, currentInstNodeOffset);
                }

                // Write original BLAS address to destination buffer
                DstBuffer.Store<uint64_t>((i * GPUVA_SIZE) + SERIALIZED_AS_HEADER_SIZE, gpuVA);
            }
        }
        else
        {
            // 3 - NumBottomLevelAccelerationStructurePointersAfterHeader
            DstBuffer.Store<uint64_t>(SERIALIZED_AS_HEADER_NUM_BLAS_PTRS_OFFSET, 0);
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
