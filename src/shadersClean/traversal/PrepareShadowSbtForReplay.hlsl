/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "../../../gpurt/gpurtDispatch.h"

#define RootSig "CBV(b0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL)"

[[vk::binding(0, 1)]] ConstantBuffer<PrepareShadowSbtForReplayConstants> Constants : register(b0);

[[vk::binding(0, 0)]] RWByteAddressBuffer ShadowRayGenerationTable   : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer ShadowHitGroupTable        : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer ShadowMissTable            : register(u2);
[[vk::binding(3, 0)]] RWByteAddressBuffer ShadowCallableTable        : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer CaptureReplayMappingBuffer : register(u4);

enum SourceTable : uint
{
    RayGenTable = 0,
    HitGroupTable = 1,
    MissTable = 2,
    CallableTable = 3,
};

[RootSignature(RootSig)]
[numthreads(64, 1, 1)]
void PrepareShadowSbtForReplay(
    uint globalId : SV_DispatchThreadId)
{
    // NOTE: In HLSL, we are allow to do:
    //     RWByteAddressBuffer srcTable = one of the table above by condition
    //     srcTable.Load/Store()
    // This requires VariablePointers capability in SPIR-V, but DXC converts RWByteAddressBuffer (actually all kinds of
    // buffer) to SPIR-V with Uniform storage class, while VariablePointers only allows StorageBuffer storage class.
    // So here we use an enum and switch-case to workaround the invalid SPIR-V.
    SourceTable srcTable = RayGenTable;
    uint offset = 0;

    const uint hitGroupShaderIdCount = Constants.hitGroupTableEntryCount * 3;
    const uint missShaderIdCount = Constants.missTableEntryCount;
    const uint callableShaderIdCount = Constants.callableTableEntryCount;

    const uint hitGroupShaderIdRangeStart = 1;
    const uint missShaderIdRangeStart = hitGroupShaderIdRangeStart + hitGroupShaderIdCount;
    const uint callableShaderIdRangeStart = missShaderIdRangeStart + missShaderIdCount;

    // Each thread takes care of one shader ID in the SBT
    if (globalId == 0)
    {
        srcTable = RayGenTable;
        offset = 0;
    }
    else if ((hitGroupShaderIdRangeStart <= globalId) && (globalId < missShaderIdRangeStart))
    {
        srcTable = HitGroupTable;
        uint idInTable = globalId - hitGroupShaderIdRangeStart;
        offset = (idInTable / 3) * Constants.hitGroupTableStrideInBytes;
        offset += (idInTable % 3) * sizeof(uint64_t);
    }
    else if ((missShaderIdRangeStart <= globalId) && (globalId < callableShaderIdRangeStart))
    {
        srcTable = MissTable;
        uint idInTable = globalId - missShaderIdRangeStart;
        offset = idInTable * Constants.missTableStrideInBytes;
    }
    else if ((callableShaderIdRangeStart <= globalId) && (globalId < callableShaderIdRangeStart + callableShaderIdCount))
    {
        srcTable = CallableTable;
        uint idInTable = globalId - callableShaderIdRangeStart;
        offset = idInTable * Constants.callableTableStrideInBytes;
    }
    else
    {
        return;
    }

    uint checkingShaderId = 0;

    switch(srcTable)
    {
        case RayGenTable:
            checkingShaderId = ShadowRayGenerationTable.Load(offset);
            break;
        case HitGroupTable:
            checkingShaderId = ShadowHitGroupTable.Load(offset);
            break;
        case MissTable:
            checkingShaderId = ShadowMissTable.Load(offset);
            break;
        case CallableTable:
            checkingShaderId = ShadowCallableTable.Load(offset);
            break;
    }

    // Look up the mapping buffer and replace the shader ID in SBT with the replay shader ID
    uint replayShaderId = 0;
    if (Constants.captureReplayMappingBufferEntryCount > 0)
    {
        const uint stride = sizeof(CaptureReplayMappingBufferEntry);
        for (uint i = 0; i < Constants.captureReplayMappingBufferEntryCount; i++)
        {
            const uint capturedShaderId = CaptureReplayMappingBuffer.Load(i * stride);
            if (capturedShaderId == checkingShaderId)
            {
                replayShaderId = CaptureReplayMappingBuffer.Load(i * stride + sizeof(uint));
                break;
            }
        }
    }

    switch(srcTable)
    {
        case RayGenTable:
            ShadowRayGenerationTable.Store(offset, replayShaderId);
            break;
        case HitGroupTable:
            ShadowHitGroupTable.Store(offset, replayShaderId);
            break;
        case MissTable:
            ShadowMissTable.Store(offset, replayShaderId);
            break;
        case CallableTable:
            ShadowCallableTable.Store(offset, replayShaderId);
            break;
    }
}
