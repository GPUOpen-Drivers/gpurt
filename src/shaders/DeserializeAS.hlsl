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
#include "BuildCommon.hlsl"

#define RootSig "RootConstants(num32BitConstants=3, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894))"\

//=====================================================================================================================
// 32 bit constants
struct InputArgs
{
    uint AddressLo; //top only
    uint AddressHi;
    uint numWaves;
};

[[vk::push_constant]] ConstantBuffer<InputArgs> ShaderConstants : register(b0);

//=====================================================================================================================
[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer DstBuffer : register(u0);
[[vk::binding(1, 0)]]                  RWByteAddressBuffer SrcBuffer : register(u1);

// @note Workaround for TASK macros using ResultMetadata specifically
#define ResultMetadata DstBuffer

groupshared uint SharedIndex;

//=====================================================================================================================
// DeserializeAS
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void DeserializeAS(
    in uint3 globalIdIn : SV_DispatchThreadID,
    in uint  localIdIn  : SV_GroupThreadID)
{
    const uint globalId = globalIdIn.x;
    const uint localId  = localIdIn;

    uint numTasksWait = 0;
    uint waveId       = 0;

    INIT_TASK;

    // NumBottomLevelAccelerationStructurePointersAfterHeader
    const uint numberOfPtrs         = SrcBuffer.Load(SERIALIZED_AS_HEADER_NUM_BLAS_PTRS_OFFSET);
    const uint sizeOfPtrs           = GPUVA_SIZE * numberOfPtrs;
    const uint serializedHeaderSize = SERIALIZED_AS_HEADER_SIZE + sizeOfPtrs;

    // Fetch acceleration structure metadata size
    const uint metadataSizeInBytes =
        SrcBuffer.Load<uint32_t>(serializedHeaderSize + ACCEL_STRUCT_METADATA_SIZE_OFFSET);

    const AccelStructHeader accelStructHeader = SrcBuffer.Load<AccelStructHeader>(serializedHeaderSize +
                                                                                  metadataSizeInBytes);

    // Total number of DWORDs in acceleration structure excluding metadata header
    const uint sizeInDwords = (accelStructHeader.sizeInBytes - sizeof(AccelStructMetadataHeader)) >> 2;
    const uint type         = (accelStructHeader.info & ACCEL_STRUCT_HEADER_INFO_TYPE_MASK);

    BEGIN_TASK(ShaderConstants.numWaves);

    // Copy from SrcBuffer->DstBuffer skipping over D3D12_SERIALIZED_ACCELERATION_STRUCTURE_HEADER and list of ptrs.
    // Skip metadata header too for now since we are using the task counters and do not want to overwrite it.
    for (uint i = globalId; i < sizeInDwords; i += (ShaderConstants.numWaves * BUILD_THREADGROUP_SIZE))
    {
        const uint byteOffset = sizeof(AccelStructMetadataHeader) + (i << 2);

        DstBuffer.Store(byteOffset, SrcBuffer.Load(serializedHeaderSize + byteOffset));
    }

    END_TASK(ShaderConstants.numWaves);

    if (type == TOP_LEVEL)
    {
        // Update BLAS pointers in the instance nodes
        const uint basePrimNodePtrsOffset = metadataSizeInBytes + accelStructHeader.offsets.primNodePtrs;

        for (uint i = globalId; i < accelStructHeader.numPrimitives; i += (ShaderConstants.numWaves * BUILD_THREADGROUP_SIZE))
        {
            const uint currentInstNodePtrOffset = basePrimNodePtrsOffset + (i * NODE_PTR_SIZE);
            const uint currentInstNodePtr       = SrcBuffer.Load(serializedHeaderSize + currentInstNodePtrOffset);

            if (currentInstNodePtr != INVALID_IDX)
            {
                const uint currentInstNodeOffset = metadataSizeInBytes + ExtractNodePointerOffset(currentInstNodePtr);

                const uint64_t oldGpuVa = SrcBuffer.Load<uint64_t>(serializedHeaderSize + currentInstNodeOffset +
                                                                   INSTANCE_DESC_VA_LO_OFFSET);

                // Fetch potential API instance index from rebraided instance node
                const uint blasInstanceIndex = SrcBuffer.Load(serializedHeaderSize + currentInstNodeOffset +
                                                                                     INSTANCE_NODE_EXTRA_OFFSET +
                                                                                     INSTANCE_EXTRA_INDEX_OFFSET);

                uint64_t newGpuVa = SrcBuffer.Load<uint64_t>(SERIALIZED_AS_HEADER_SIZE + (blasInstanceIndex * GPUVA_SIZE));

                // Handle null BLAS address
                if (newGpuVa != 0)
                {
                    const uint blasMetadataSize = SrcBuffer.Load(serializedHeaderSize + currentInstNodeOffset +
                                                                 INSTANCE_NODE_EXTRA_OFFSET +
                                                                 INSTANCE_EXTRA_METADATA_SIZE_OFFSET);
                    newGpuVa = newGpuVa + blasMetadataSize;
                }

                // Preserve the instance flags in the high bits of the address.
                newGpuVa = (oldGpuVa & ~(INSTANCE_BASE_POINTER_ADDRESS_USED_MASK)) | (newGpuVa >> 3);
                DstBuffer.Store<uint64_t>(currentInstNodeOffset + INSTANCE_DESC_VA_LO_OFFSET, newGpuVa);
            }
        }
    }

    // Patch acceleration structure metadata with updated address
    if (globalId == 0)
    {
        // Offset to acceleration structure header
        uint64_t gpuVa = MakeGpuVirtualAddress(ShaderConstants.AddressLo, ShaderConstants.AddressHi);
        gpuVa += metadataSizeInBytes;

        if (type == TOP_LEVEL)
        {
            if (accelStructHeader.numActivePrims > 0)
            {
                DstBuffer.Store<uint64_t>(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, gpuVa);
                DstBuffer.Store(ACCEL_STRUCT_METADATA_SIZE_OFFSET,  metadataSizeInBytes);
            }
            else
            {
                DstBuffer.Store<uint64_t>(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, 0);
                DstBuffer.Store(ACCEL_STRUCT_METADATA_SIZE_OFFSET,  0);
            }
        }
        else
        {
            DstBuffer.Store<uint64_t>(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, gpuVa);
            DstBuffer.Store(ACCEL_STRUCT_METADATA_SIZE_OFFSET,  metadataSizeInBytes);
        }
    }
}
