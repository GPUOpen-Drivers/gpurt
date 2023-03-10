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
// The functions defined below require the following resources defined before including this header
//
// SharedMem[]
//
//=====================================================================================================================
// Perform scan within a block of threads
int BlockScanExclusiveAdd(int key, uint localId)
{
    SharedMem[localId] = 0;

    GroupMemoryBarrierWithGroupSync();

    // Peform scan within a subgroup (wave)
    int wavePrefixSum = WavePrefixSum(key);

    uint widx = localId / WaveGetLaneCount();
    uint wlidx = WaveGetLaneIndex();

    // Last element in each subgroup writes partial sum into LDS
    if (wlidx == WaveGetLaneCount() - 1)
    {
        SharedMem[widx] = wavePrefixSum + key;
    }

    GroupMemoryBarrierWithGroupSync();

    // Then first subgroup scans partial sums
    if (widx == 0)
    {
        SharedMem[localId] = WavePrefixSum(SharedMem[localId]);
    }

    GroupMemoryBarrierWithGroupSync();

    // And we add partial sums back to each subgroup-scanned element
    wavePrefixSum += SharedMem[widx];

    return wavePrefixSum;
}
