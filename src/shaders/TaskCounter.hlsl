/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#if !defined(__cplusplus)
#include "BuildSettings.hlsli"
#endif

//=====================================================================================================================
// Increment task counter to mark a task / primitive as done
uint IncrementTaskCounter(uint offset, uint value)
{
    DeviceMemoryBarrier();

    uint originalVal = 0;
    ScratchGlobal.InterlockedAdd(offset, value, originalVal);

    return originalVal;
}

//=====================================================================================================================
uint FetchTaskCounter(uint offset)
{
    return ScratchGlobal.Load(offset);
}

//=====================================================================================================================
void WriteTaskCounterData(uint offset, uint dataOffset, uint data)
{
    ScratchGlobal.Store(offset + dataOffset, data);
}

//======================================================================================================================
// Helper function that increments ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET
// and ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET for INDIRECT_BUILD.
// Direct builds have access to primitive count during dispatch cmd recording and set
// ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET to correct value beforehand.
void IncrementPrimitiveTaskCounters(
    in uint encodeTaskCounterScratchOffset,
    in uint primitiveIndex,
    in uint numPrimitives,
    in uint maxNumPrimitives)
{
    // 1st task counter is being used as primitive counter, it means how many triangles is there to build
    // in case of indirect build we dispatch maxPrimCount waves, but less that that can be encoded
    const uint encodedPrimCount = WaveActiveCountBits(primitiveIndex < numPrimitives);
    // 2nd task counter is being used as spin-lock that build step waits for,
    // counter=GeometryConstants.numPrimitives means encoding is done
    const uint dispatchedPrimCount = WaveActiveCountBits(primitiveIndex < maxNumPrimitives);
    if (WaveIsFirstLane())
    {
        // compression and splitting update primRefCounter on their own,
        // direct builds set ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET to correct value during cmd recording
        if (Settings.isIndirectBuild && !Settings.enableEarlyPairCompression && !Settings.doTriangleSplitting)
        {
            IncrementTaskCounter(encodeTaskCounterScratchOffset + ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET,
                                 encodedPrimCount);
        }

        IncrementTaskCounter(encodeTaskCounterScratchOffset + ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET,
                             dispatchedPrimCount);
    }
}
