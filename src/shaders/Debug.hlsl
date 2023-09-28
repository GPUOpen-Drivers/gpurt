/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _DEBUG_HLSL
#define _DEBUG_HLSL

#include "../shared/debugBuffer.h"
#include "Common.hlsl"
#include "Extensions.hlsl"

#if GPURT_ENABLE_GPU_DEBUG
    #if BUILD_PARALLEL
        #define GPU_ASSERT_IMPL(id, cond) DoGpuAssert(id, (cond))
        #define GPU_DPF_IMPL(id, msg, ...) \
        do \
        { \
            uint args[] = { 0, __VA_ARGS__ }; \
            DoGpuPrintf(Settings.gpuDebugFlags, id, args); \
        } while (false)
    #else
        #define GPU_ASSERT_IMPL(id, cond) if (!(cond)) { Halt(); }
        #define GPU_DPF_IMPL(msg, ...)
    #endif
#else
    #define GPU_ASSERT(cond)  do {} while (false)
    #define GPU_DPF(msg, ...) do {} while (false)
#endif

//======================================================================================================================
void Halt()
{
    AmdExtD3DShaderIntrinsics_Halt();
}

#if GPURT_ENABLE_GPU_DEBUG && BUILD_PARALLEL

globallycoherent RWByteAddressBuffer DebugBuffer : register(u5);

//======================================================================================================================
// Reserve write space in the debug ring buffer
uint DebugBufferReserve(
    uint dwords,
    out uint commitIdx)
{
    while (true)
    {
        uint orig;
        DebugBuffer.InterlockedCompareExchange(DEBUG_CONTROL_LOCK_OFFSET, 0, 1, orig);
        if (orig == 0)
        {
            break;
        }
        DeviceMemoryBarrier();
    }

    const uint writeIdx  = DebugBuffer.Load(DEBUG_CONTROL_WRITE_OFFSET);

    commitIdx = writeIdx + dwords;

    // If the host debugger is broken in on an assert, the GPU might hang waiting for space.
    // Might be useful to have a toggle between waiting and dropping messages.
    while (true)
    {
        const uint readIdx = DebugBuffer.Load(DEBUG_CONTROL_READ_OFFSET);

        if ((commitIdx - readIdx) <= DEBUG_BUFFER_SIZE)
        {
            break;
        }

        DeviceMemoryBarrier();
    }

    return writeIdx;
}

//======================================================================================================================
// Write a value into the buffer and return the next index
uint DebugBufferWrite(
    uint writeIdx,
    uint data)
{
    const uint offset = ((writeIdx % DEBUG_BUFFER_SIZE) + DEBUG_CONTROL_DWORDS) * sizeof(uint);
    DebugBuffer.Store(offset, data);
    return writeIdx + 1;
}

//======================================================================================================================
// Commit written data to the ring
void DebugBufferCommit(
    uint commitIdx)
{
    DeviceMemoryBarrier();
    DebugBuffer.Store(DEBUG_CONTROL_WRITE_OFFSET, commitIdx);
    DeviceMemoryBarrier();
    DebugBuffer.Store(DEBUG_CONTROL_LOCK_OFFSET, 0);
}

//======================================================================================================================
// Log the specified message ID and arguments to the debug buffer
template<typename T>
void DoGpuPrintf(
    uint gpuDebugFlags,
    uint messageId,
    T    arguments)
{
    // The HLSL compiler crashes when the Settings constant buffer is referenced directly here. Passing in the flags
    // as an argument works around the issue.
    if ((gpuDebugFlags & GpuDebugFlags::HostPrint) == 0)
    {
        return;
    }

    // The first argument is a dummy value to allow prints with 0 arguments. Ignore that one.
    const uint argCount = (sizeof(arguments) / sizeof(uint)) - 1;

    const uint laneSize  = argCount + 1;
    const uint totalSize = WaveActiveSum(laneSize);

    uint baseOffset;
    uint commitIdx;
    if (WaveIsFirstLane())
    {
        baseOffset = DebugBufferReserve(totalSize, commitIdx);
    }
    uint offset = WaveReadLaneFirst(baseOffset) + WavePrefixSum(laneSize);

    const uint packet =
        messageId |
        (argCount << DEBUG_PACKET_ARG_COUNT_SHIFT) |
        (WaveGetLaneIndex() << DEBUG_PACKET_LANE_ID_SHIFT);

    offset = DebugBufferWrite(offset, packet);

    for (uint i = 0; i < argCount; i++)
    {
        offset = DebugBufferWrite(offset, arguments[i+1]);
    }

    if (WaveIsFirstLane())
    {
        DebugBufferCommit(commitIdx);
    }
}

//======================================================================================================================
// Log the assert ID to the debug buffer if triggered
void DoGpuAssert(
    uint messageId,
    bool condition)
{
    if (condition == false)
    {
        if ((Settings.gpuDebugFlags & GpuDebugFlags::HostAssert) ||
            (Settings.gpuDebugFlags & GpuDebugFlags::HostPrint))
        {
            if (WaveIsFirstLane())
            {
                uint commitIdx;
                const uint offset = DebugBufferReserve(1, commitIdx);
                DebugBufferWrite(offset, messageId |
                                         (1 << DEBUG_PACKET_ASSERT_SHIFT) |
                                         (WaveGetLaneIndex() << DEBUG_PACKET_LANE_ID_SHIFT));
                DebugBufferCommit(commitIdx);
            }
        }

        if (Settings.gpuDebugFlags & GpuDebugFlags::ShaderHalt)
        {
            Halt();
        }
    }
}
#endif

#endif
