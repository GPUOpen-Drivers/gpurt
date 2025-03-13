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
bool EnablePrimitiveCompressionPass()
{
    return ((Settings.primCompressionFlags & PrimCompFlags::MultiPrim) ||
            (Settings.maxPrimRangeSize > 2));
}

namespace EncodeHwBvh {
//=====================================================================================================================
void WriteCounter(
    uint index, uint value)
{
#if USE_LDS_HW_ENCODE_COUNTERS
    SharedMem[GetLdsOffset(STGB_THREADGROUP_SIZE - 1) + index] = value;
#else
    ScratchBuffer.Store(ShaderConstants.offsets.qbvhGlobalStackPtrs + (index * sizeof(uint)), value);
#endif
}

//=====================================================================================================================
uint ReadCounter(
    uint index)
{
#if USE_LDS_HW_ENCODE_COUNTERS
    return SharedMem[GetLdsOffset(STGB_THREADGROUP_SIZE - 1) + index];
#else
    return ScratchBuffer.Load(ShaderConstants.offsets.qbvhGlobalStackPtrs + (index * sizeof(uint)));
#endif
}

//=====================================================================================================================
uint IncrementCounter(
    uint index, uint value)
{
    uint orig;
#if USE_LDS_HW_ENCODE_COUNTERS
    InterlockedAdd(SharedMem[GetLdsOffset(STGB_THREADGROUP_SIZE - 1) + index], value, orig);
#else
    ScratchBuffer.InterlockedAdd(ShaderConstants.offsets.qbvhGlobalStackPtrs + (index * sizeof(uint)), value, orig);
#endif
    return orig;
}
} // namespace EncodeHwBvh
