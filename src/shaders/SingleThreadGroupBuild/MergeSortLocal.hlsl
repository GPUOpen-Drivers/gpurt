/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
void CmpSwap(uint i, uint j)
{
    uint64_t m0 = LDS.ReadMortonCode(i);
    uint32_t p0 = LDS.ReadPrimRefIdx(i);
    uint64_t m1 = LDS.ReadMortonCode(j);
    uint32_t p1 = LDS.ReadPrimRefIdx(j);

    if (m0 > m1)
    {
        LDS.WriteMortonCode(j, m0);
        LDS.WritePrimRefIdx(j, p0);
        LDS.WriteMortonCode(i, m1);
        LDS.WritePrimRefIdx(i, p1);
    }
}

//=====================================================================================================================
void OddEvenMergeSort(
    uint32_t localId,
    uint32_t groupSize,
    uint32_t partitionSize)
{
    const uint numSteps = log2(partitionSize);

    for (uint i = 0; i < numSteps; i++)
    {
        uint cmpGap = 1U << i;
        uint splitGap = 1U << (i + 1);

        const uint splitGapFixed = splitGap;

        for (uint j = 0; j < i + 1; j++)
        {
            uint localIndex = ((localId % splitGap) < cmpGap) ? localId : localId + groupSize - cmpGap;

            if (localIndex < partitionSize)
            {
                if (j == 0)
                {
                    CmpSwap(localIndex, localIndex + cmpGap);
                }
                else
                {
                    localIndex += cmpGap;
                    if (localIndex + cmpGap < (localIndex / splitGapFixed + 1) * splitGapFixed)
                    {
                        CmpSwap(localIndex, localIndex + cmpGap);
                    }
                }
            }

            GroupMemoryBarrierWithGroupSync();

            cmpGap >>= 1;
            splitGap >>= 1;
        }
    }
}
