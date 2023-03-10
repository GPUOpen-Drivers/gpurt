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
// Main Function : CopyBufferRaw
//=====================================================================================================================
[[vk::binding(0, 0)]] RWStructuredBuffer<uint> Src : register(u0);
[[vk::binding(1, 0)]] RWStructuredBuffer<uint> Dst : register(u1);

struct Constants
{
	uint numDwords;
};

[[vk::push_constant]] ConstantBuffer<Constants> CB : register(b0);

#define RootSig "RootConstants(num32BitConstants=1, b0), UAV(u0), UAV(u1)"

[numthreads(64, 1, 1)]
[RootSignature(RootSig)]
void CopyBufferRaw(uint3 DispatchThreadID : SV_DispatchThreadID)
{
    if (DispatchThreadID.x < CB.numDwords)
    {
        Dst[DispatchThreadID.x] = Src[DispatchThreadID.x];
    }
}
