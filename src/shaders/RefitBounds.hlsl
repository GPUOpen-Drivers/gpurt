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

#define RootSig "RootConstants(num32BitConstants=13, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894))"

//=====================================================================================================================
struct Constants
{
    uint  FlagsScratchOffset;        // Offset to flags in scratch buffer
    uint  ScratchNodesScratchOffset; // Offset to scratch nodes in build scratch buffer
    uint  Fp16BoxNodesInBlasMode;    // Mode used for which BLAS interior nodes are FP16
    float Fp16BoxModeMixedSaThresh;  // For fp16 mode "mixed", surface area threshold
    uint  DoCollapse;
    uint  DoTriangleSplitting;
    uint  EnablePairCompression;
    uint  EnablePairCostCheck;
    uint  SplitBoxesByteOffset;
    uint  NumBatchesScratchOffset;
    uint  BatchIndicesScratchOffset;
    uint  NoCopySortedNodes;
    uint  sortedPrimIndicesOffset;
};

//=====================================================================================================================
[[vk::push_constant]] ConstantBuffer<Constants>             ShaderConstants : register(b0);
[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer  ResultBuffer    : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer                   ResultMetadata  : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer  ScratchBuffer   : register(u2);

#include "RefitBoundsImpl.hlsl"

//=====================================================================================================================
// Main Function : RefitBounds
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void RefitBounds(
    in uint globalId : SV_DispatchThreadID)
{
    const uint numActivePrims = ResultBuffer.Load(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);

    if (globalId < numActivePrims)
    {
        RefitBoundsImpl(globalId,
                        numActivePrims,
                        ShaderConstants.FlagsScratchOffset,
                        ShaderConstants.ScratchNodesScratchOffset,
                        ShaderConstants.sortedPrimIndicesOffset,
                        ShaderConstants.DoCollapse,
                        ShaderConstants.DoTriangleSplitting,
                        ShaderConstants.NoCopySortedNodes,
                        ShaderConstants.EnablePairCompression,
                        ShaderConstants.EnablePairCostCheck,
                        ShaderConstants.SplitBoxesByteOffset,
                        ShaderConstants.NumBatchesScratchOffset,
                        ShaderConstants.BatchIndicesScratchOffset,
                        ShaderConstants.Fp16BoxNodesInBlasMode,
                        ShaderConstants.Fp16BoxModeMixedSaThresh,
                        0,
                        false,
                        false,
                        0,
                        false);
    }
}
