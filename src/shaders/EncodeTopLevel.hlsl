/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#define RootSig "RootConstants(num32BitConstants=11, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b255),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u6, visibility=SHADER_VISIBILITY_ALL)"

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
    uint buildFlags;                    // Build flags
    uint leafNodeExpansionFactor;       // Leaf node expansion factor (> 1 for rebraid)
    uint enableFastLBVH;
    uint encodeTaskCounterScratchOffset;
};

[[vk::push_constant]] ConstantBuffer<Constants> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] RWByteAddressBuffer                 DstMetadata        : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer                 ScratchBuffer      : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer                 ScratchGlobal      : register(u2);
[[vk::binding(3, 0)]] RWByteAddressBuffer                 SrcBuffer          : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer                 InstanceDescBuffer : register(u4);

// unused buffer
[[vk::binding(5, 0)]] RWByteAddressBuffer                 DstBuffer          : register(u5);
[[vk::binding(6, 0)]] RWByteAddressBuffer                 EmitBuffer         : register(u6);

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

        if (Settings.encodeArrayOfPointers)
        {
            GpuVirtualAddress addr = InstanceDescBuffer.Load<GpuVirtualAddress>(index * GPU_VIRTUAL_ADDRESS_SIZE);
            desc = FetchInstanceDescAddr(addr);
        }
        else
        {
            desc = InstanceDescBuffer.Load<InstanceDesc>(index * INSTANCE_DESC_SIZE);
        }

        const uint tlasMetadataSize =
            IsUpdate() ? SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : ShaderConstants.metadataSizeInBytes;

        // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
        const AccelStructOffsets offsets =
            SrcBuffer.Load<AccelStructOffsets>(tlasMetadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

        const uint basePrimNodePointersOffset =
            IsUpdate() ? offsets.primNodePtrs : ShaderConstants.basePrimNodePtrOffset;

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
                                 numActivePrims,
                                 allowUpdate);
        }

        // ClearFlags for refit and update
        const uint stride = ShaderConstants.leafNodeExpansionFactor * sizeof(uint);
        const uint flagOffset = ShaderConstants.propagationFlagsScratchOffset + (index * stride);
        const uint initValue = ShaderConstants.enableFastLBVH ? 0xffffffffu : 0;
        for (uint i = 0; i < ShaderConstants.leafNodeExpansionFactor; ++i)
        {
            ScratchBuffer.Store(flagOffset + (i * sizeof(uint)), initValue);
        }
    }

    const uint wavePrimCount = WaveActiveCountBits(index < ShaderConstants.numPrimitives);
    if (WaveIsFirstLane())
    {
        IncrementTaskCounter(ShaderConstants.encodeTaskCounterScratchOffset + ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET,
                             wavePrimCount);
    }
}
