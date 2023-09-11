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
#define RootSig "RootConstants(num32BitConstants=8, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b1),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL)"

//=====================================================================================================================
struct Constants
{
    uint  FlagsScratchOffset;        // Offset to flags in scratch buffer
    uint  ScratchNodesScratchOffset; // Offset to scratch nodes in build scratch buffer
    uint  UnsortedNodesBaseOffset;   // Offset to unsorted leaf nodes in build scratch buffer
    uint  SplitBoxesByteOffset;
    uint  NumBatchesScratchOffset;
    uint  BatchIndicesScratchOffset;
    uint  sortedPrimIndicesOffset;
    uint  NumLeafNodes;
};

[[vk::push_constant]] ConstantBuffer<Constants> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer  DstBuffer     : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer                   DstMetadata   : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer  ScratchBuffer : register(u2);

// unused buffer
[[vk::binding(3, 0)]] RWByteAddressBuffer                   SrcBuffer     : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer                   EmitBuffer    : register(u4);

#include "IntersectCommon.hlsl"
#include "RefitBoundsImpl.hlsl"

//=====================================================================================================================
// Main Function : RefitBounds
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void RefitBounds(
    in uint globalId : SV_DispatchThreadID)
{
    const uint numActivePrims = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
    const uint bvhNodes = CalculateScratchBvhNodesOffset(
                              numActivePrims,
                              ShaderConstants.NumLeafNodes,
                              ShaderConstants.ScratchNodesScratchOffset,
                              Settings.noCopySortedNodes);
    if (globalId < numActivePrims)
    {
        RefitBoundsImpl(globalId,
                        numActivePrims,
                        ShaderConstants.FlagsScratchOffset,
                        bvhNodes,
                        ShaderConstants.UnsortedNodesBaseOffset,
                        ShaderConstants.sortedPrimIndicesOffset,
                        Settings.doCollapse,
                        Settings.doTriangleSplitting,
                        Settings.noCopySortedNodes,
                        Settings.enableEarlyPairCompression,
                        EnableLatePairCompression(),
                        Settings.enablePairCostCheck,
                        ShaderConstants.SplitBoxesByteOffset,
                        ShaderConstants.NumBatchesScratchOffset,
                        ShaderConstants.BatchIndicesScratchOffset,
                        Settings.fp16BoxNodesMode,
                        Settings.fp16BoxModeMixedSaThreshold,
                        0,
                        false,
                        false,
                        0,
                        false,
                        false,
                        0);
    }
}
