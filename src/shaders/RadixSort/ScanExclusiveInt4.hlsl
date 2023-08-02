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
#if NO_SHADER_ENTRYPOINT == 0
#define RootSig "RootConstants(num32BitConstants=2, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL)"

//=====================================================================================================================
struct InputArgs
{
    uint NumElements;
    uint InOutArrayScratchOffset;
};

[[vk::push_constant]] ConstantBuffer<InputArgs> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] RWByteAddressBuffer DstBuffer     : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer DstMetadata   : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer ScratchBuffer : register(u2);

// unused buffer
[[vk::binding(3, 0)]] RWByteAddressBuffer SrcBuffer     : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer EmitBuffer    : register(u4);

#include "ScanCommon.hlsli"

//=====================================================================================================================
groupshared int SharedMem[GROUP_SIZE];
#endif

//=====================================================================================================================
void GroupScanExclusiveInt4(
    uint localId,
    uint groupSize)
{
    uint stride;
    for (stride = 1; stride <= (groupSize >> 1); stride <<= 1)
    {
        if (localId < groupSize / (2 * stride))
        {
            SharedMem[2*(localId + 1)*stride-1] = SharedMem[2*(localId + 1)*stride-1] + SharedMem[(2*localId + 1)*stride-1];
        }

        // Wait for all threads to finish
        GroupMemoryBarrierWithGroupSync();
    }

    if (localId == 0)
    {
        SharedMem[groupSize - 1] = 0;
    }

    GroupMemoryBarrierWithGroupSync();

    for (stride = (groupSize >> 1); stride > 0; stride >>= 1)
    {
        if (localId < groupSize / (2 * stride))
        {
            int temp = SharedMem[(2 * localId + 1)*stride - 1];
            SharedMem[(2 * localId + 1)*stride - 1] = SharedMem[2 * (localId + 1)*stride - 1];
            SharedMem[2 * (localId + 1)*stride - 1] = SharedMem[2 * (localId + 1)*stride - 1] + temp;
        }

        GroupMemoryBarrierWithGroupSync();
    }
}

//=====================================================================================================================
void ScanExclusiveInt4Impl(
    uint globalId,
    uint localId,
    uint inOutDataOffset,
    uint numElements)
{
    int4 v1 = safe_load_int4(inOutDataOffset, 2 * globalId, numElements);
    int4 v2 = safe_load_int4(inOutDataOffset, 2 * globalId + 1, numElements);

    v1.y += v1.x; v1.w += v1.z; v1.w += v1.y;
    v2.y += v2.x; v2.w += v2.z; v2.w += v2.y;
    v2.w += v1.w;

    SharedMem[localId] = v2.w;

    // Sync shared memory writes
    GroupMemoryBarrierWithGroupSync();

    GroupScanExclusiveInt4(localId, GROUP_SIZE);

    v2.w = SharedMem[localId];

    int t = v1.w; v1.w = v2.w; v2.w += t;
    t = v1.y; v1.y = v1.w; v1.w += t;
    t = v2.y; v2.y = v2.w; v2.w += t;
    t = v1.x; v1.x = v1.y; v1.y += t;
    t = v2.x; v2.x = v2.y; v2.y += t;
    t = v1.z; v1.z = v1.w; v1.w += t;
    t = v2.z; v2.z = v2.w; v2.w += t;

    safe_store_int4(v2, inOutDataOffset, 2 * globalId + 1, numElements);
    safe_store_int4(v1, inOutDataOffset, 2 * globalId, numElements);
}

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(GROUP_SIZE, 1, 1)]
void ScanExclusiveInt4(
    uint globalId : SV_DispatchThreadID,
    uint localId  : SV_GroupThreadID)
{
    ScanExclusiveInt4Impl(
        globalId,
        localId,
        ShaderConstants.InOutArrayScratchOffset,
        ShaderConstants.NumElements);
}
#endif
