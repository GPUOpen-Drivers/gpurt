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
#if NO_SHADER_ENTRYPOINT == 0

//=====================================================================================================================
#include "../../shadersClean/common/ShaderDefs.hlsli"

#include "../../shadersClean/build/BuildRootSignature.hlsli"

#include "ScanCommon.hlsli"
#endif

//=====================================================================================================================
// Distribute partial sums from top level scan into bottom level
void DistributePartSumInt4Impl(
    uint globalId,
    uint groupId,
    uint inOutDataOffset,
    uint partialSumsOffset,
    uint numElements)
{
    int4 v1 = safe_load_int4(inOutDataOffset, globalId, numElements);
    int sum = ScratchBuffer.Load(partialSumsOffset + ((groupId >> 1) * sizeof(int)));
    v1.xyzw += sum;

    safe_store_int4(v1, inOutDataOffset, globalId, numElements);
}

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
// Main Function : Distribute partial sum for RadixSort
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(GROUP_SIZE, 1, 1)]
void DistributePartSumInt4(
    in uint globalId : SV_DispatchThreadID,
    in uint groupId  : SV_GroupID)
{
    DistributePartSumInt4Impl(
        globalId,
        groupId,
        ShaderRootConstants.InOutArrayScratchOffset(),
        ShaderRootConstants.PartSumsScratchOffset(),
        ShaderRootConstants.NumElements());
}
#endif
