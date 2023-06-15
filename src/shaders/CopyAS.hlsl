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
#include "Common.hlsl"

#define RootSig "RootConstants(num32BitConstants=3, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL)  "

//=====================================================================================================================
// 32 bit constants
struct InputArgs
{
    uint numWaves;
    uint AddressLo; // top only
    uint AddressHi;
};

[[vk::push_constant]] ConstantBuffer<InputArgs> ShaderConstants : register(b0);

//=====================================================================================================================
[[vk::binding(0, 0)]] RWByteAddressBuffer DstMetadata   : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer SrcBuffer     : register(u1);

// unused buffer
[[vk::binding(2, 0)]] RWByteAddressBuffer DstBuffer     : register(u2);
[[vk::binding(3, 0)]] RWByteAddressBuffer EmitBuffer    : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer ScratchBuffer : register(u4);

//=====================================================================================================================
// CopyAS
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void CopyAS(in uint3 globalThreadId : SV_DispatchThreadID)
{
    const uint globalID = globalThreadId.x;

    // The following code assumes the base address of SrcBuffer and DstMetadata includes the acceleration structure
    // metadata header

    // Fetch acceleration structure metadata size
    const uint32_t metadataSizeInBytes = SrcBuffer.Load<uint32_t>(ACCEL_STRUCT_METADATA_SIZE_OFFSET);
    const AccelStructHeader header = SrcBuffer.Load<AccelStructHeader>(metadataSizeInBytes);

    // Copy acceleration structure data including header
    const uint sizeInDwords = header.sizeInBytes >> 2;
    for (uint i = globalID; i < sizeInDwords; i += ShaderConstants.numWaves * BUILD_THREADGROUP_SIZE)
    {
        uint byteOffset = (i << 2);
        DstMetadata.Store(byteOffset, SrcBuffer.Load(byteOffset));
    }

    // Patch acceleration structure headers
    if (globalID == 0)
    {
        // Offset to acceleration structure header
        uint64_t gpuVa = MakeGpuVirtualAddress(ShaderConstants.AddressLo, ShaderConstants.AddressHi);
        gpuVa += metadataSizeInBytes;

        // Patch metadata header
        const uint type = (header.info & ACCEL_STRUCT_HEADER_INFO_TYPE_MASK);
        if (type == TOP_LEVEL)
        {
            // Deactivate acceleration structure
            if (header.numActivePrims > 0)
            {
                DstMetadata.Store<uint64_t>(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, gpuVa);
                DstMetadata.Store<uint32_t>(ACCEL_STRUCT_METADATA_SIZE_OFFSET,  metadataSizeInBytes);
            }
            else
            {
                DstMetadata.Store<uint64_t>(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, 0);
                DstMetadata.Store<uint32_t>(ACCEL_STRUCT_METADATA_SIZE_OFFSET,  0);
            }

        }
        else
        {
            DstMetadata.Store<uint64_t>(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, gpuVa);
            DstMetadata.Store<uint32_t>(ACCEL_STRUCT_METADATA_SIZE_OFFSET,  metadataSizeInBytes);
        }

        DstMetadata.Store(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET, 0);
        DstMetadata.Store<uint32_t>(ACCEL_STRUCT_METADATA_NUM_TASKS_DONE_OFFSET, 0);
    }
}
