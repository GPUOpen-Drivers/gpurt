/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
// Note, CBV(b255) must be the last used binding in the root signature.
#define RootSig "RootConstants(num32BitConstants=3, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "CBV(b255),"\

#define DISABLE_BUILD_ROOT_SIGNATURE
//=====================================================================================================================
// 32 bit constants
struct InputArgs
{
    uint numThreads;
    uint addressLo; // top only
    uint addressHi;
};

[[vk::push_constant]] ConstantBuffer<InputArgs> ShaderConstants : register(b0);

//=====================================================================================================================
[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer    DstMetadata   : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer                     SrcBuffer     : register(u1);

// unused buffer
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer    DstBuffer     : register(u2);
[[vk::binding(3, 0)]] globallycoherent RWByteAddressBuffer    ScratchBuffer : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer                     EmitBuffer    : register(u4);

#include "../shadersClean/common/Common.hlsli"
#include "../shadersClean/build/BuildCommon.hlsli"
#include "CompactCommon.hlsl"
#include "CompactAS1_1.hlsl"

#if GPURT_BUILD_RTIP3_1
#include "CompactAS3_1.hlsl"
#endif

//=====================================================================================================================
// CompactAS
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void CompactAS(in uint3 globalThreadId : SV_DispatchThreadID)
{
    const uint globalId = globalThreadId.x;

    switch (Settings.rtIpLevel)
    {
#if GPURT_BUILD_RTIP3_1
    case GPURT_RTIP3_1:
        CompactASImpl3_1(globalId);
        return;
#endif
    default:
        CompactASImpl1_1(globalId);
        return;
    }
}
