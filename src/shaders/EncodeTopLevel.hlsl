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
#define RootSig "RootConstants(num32BitConstants=12, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b255),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL)"

//=====================================================================================================================
// 32 bit constants
struct Constants
{
    uint metadataSizeInBytes;           // Size of the metadata header
    uint basePrimNodePtrOffset;         // Offset of the prim node pointeres
    uint numPrimitives;                 // Number of instances
    uint leafNodeDataByteOffset;        // Leaf node data byte offset
    uint sceneBoundsByteOffset;         // Scene bounds byte offset
    uint propagationFlagsScratchOffset; // Offset of update flags in scratch memory
    uint baseUpdateStackScratchOffset;  // Offset of update scratch
    uint internalFlags;                 // Internal flags
    uint buildFlags;                    // Build flags
    uint leafNodeExpansionFactor;       // Leaf node expansion factor (> 1 for rebraid)
    uint sceneBoundsCalculationType;
    uint enableFastLBVH;
};

[[vk::push_constant]] ConstantBuffer<Constants> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] RWByteAddressBuffer                 DstMetadata        : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer                 ScratchBuffer      : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer                 SrcBuffer          : register(u2);
[[vk::binding(3, 0)]] RWByteAddressBuffer                 InstanceDescBuffer : register(u3);

// unused buffer
[[vk::binding(4, 0)]] RWByteAddressBuffer                 DstBuffer          : register(u4);
[[vk::binding(5, 0)]] RWByteAddressBuffer                 EmitBuffer         : register(u5);

#include "IntersectCommon.hlsl"
#include "BuildCommonScratch.hlsl"

#include "EncodeTopLevelBuild.hlsl"
#include "EncodeTopLevelUpdate.hlsl"

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeInstances(
    in uint3 globalThreadId : SV_DispatchThreadID)
{
    const bool allowUpdate = AllowUpdate(ShaderConstants.buildFlags);

    uint index = globalThreadId.x;

    if (index < ShaderConstants.numPrimitives)
    {
        InstanceDesc desc;

        if (ShaderConstants.internalFlags & ENCODE_FLAG_ARRAY_OF_POINTERS)
        {
            GpuVirtualAddress addr = InstanceDescBuffer.Load<GpuVirtualAddress>(index * GPU_VIRTUAL_ADDRESS_SIZE);
            desc = FetchInstanceDescAddr(addr);
        }
        else
        {
            desc = InstanceDescBuffer.Load<InstanceDesc>(index * INSTANCE_DESC_SIZE);
        }

        const uint tlasMetadataSize = ShaderConstants.metadataSizeInBytes;

        const uint basePrimNodePointersOffset = ShaderConstants.basePrimNodePtrOffset;

        const uint primNodePointerOffset = tlasMetadataSize + basePrimNodePointersOffset + (index * sizeof(uint));

        const uint destScratchNodeOffset = (index * ByteStrideScratchNode) + ShaderConstants.leafNodeDataByteOffset;

        uint64_t baseAddrAccelStructHeader = GetAccelStructBaseAddr(desc, allowUpdate);

        uint numActivePrims = 0;
        if (baseAddrAccelStructHeader != 0)
        {
            numActivePrims = FetchHeaderField(baseAddrAccelStructHeader, ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
            if ((allowUpdate == false) && (numActivePrims == 0))
            {
                // Mark instance inactive with a NULL address
                baseAddrAccelStructHeader = 0;
            }
        }

        if (IsUpdate())
        {
            EncodeInstancesUpdate(index,
                                  desc,
                                  tlasMetadataSize,
                                  primNodePointerOffset,
                                  baseAddrAccelStructHeader,
                                  numActivePrims);
        }
        else
        {
            EncodeInstancesBuild(index,
                                 desc,
                                 primNodePointerOffset,
                                 destScratchNodeOffset,
                                 baseAddrAccelStructHeader,
                                 numActivePrims);
        }

        // ClearFlags for refit and update
        const uint stride = ShaderConstants.leafNodeExpansionFactor * sizeof(uint);
        const uint flagOffset = ShaderConstants.propagationFlagsScratchOffset + (index * stride);
        const uint initValue = ShaderConstants.enableFastLBVH ? 0xffffffffu : 0;
        for (uint i = 0; i < ShaderConstants.leafNodeExpansionFactor; ++i)
        {
            ScratchBuffer.Store(flagOffset + (i * sizeof(uint)), initValue);
        }

        DeviceMemoryBarrier();
        DstMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET, 1);
    }
}
