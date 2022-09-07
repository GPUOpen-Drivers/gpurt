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
#include "IntersectCommon.hlsl"
#include "BuildCommon.hlsl"

#define RootSig "RootConstants(num32BitConstants=8, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u0, space=2147420894, visibility=SHADER_VISIBILITY_ALL)"

//=====================================================================================================================
struct Constants
{
    uint isUpdateInPlace;
    uint addressLo;
    uint addressHi;
    uint propagationFlagsScratchOffset; // Offset to flags in scratch buffer
    uint baseUpdateStackScratchOffset;
    uint triangleCompressionMode;
    uint fp16BoxNodesInBlasMode;
    uint numPrimitives;
};

//=====================================================================================================================
[[vk::push_constant]] ConstantBuffer<Constants>            ShaderConstants : register(b0);
[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer ResultBuffer    : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer ScratchBuffer   : register(u1);
[[vk::binding(2, 0)]]                  RWByteAddressBuffer SourceBuffer    : register(u2);

#include "UpdateQBVHImpl.hlsl"

//=====================================================================================================================
// Main Function : UpdateQBVH
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void UpdateQBVH(
    in uint3 globalThreadId : SV_DispatchThreadID)
{
    // Fetch number of nodes to process
    uint numWorkItems = ScratchBuffer.Load(ShaderConstants.baseUpdateStackScratchOffset);

    if (globalThreadId.x >= numWorkItems)
    {
        return;
    }

    UpdateQBVHImpl(globalThreadId.x,
                   ResultBuffer,
                   ScratchBuffer,
                   SourceBuffer,
                   ShaderConstants.propagationFlagsScratchOffset);
}
