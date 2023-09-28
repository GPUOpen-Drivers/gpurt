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
#if GPURT_ENABLE_GPU_DEBUG
#include "gpurtDebugMonitor.h"
#include "gpurtInternal.h"
#include "shared/debugBuffer.h"

#include "g_gpurtDebugInfo.h"

#include <atomic>

namespace GpuRt
{
// =====================================================================================================================
DebugMonitor::DebugMonitor(
    Internal::Device* pDevice)
    :
    m_pDevice(pDevice)
{
    if (m_pDevice->Settings().gpuDebugFlags != 0)
    {
        InitGpuDebugging();
    }
}

// =====================================================================================================================
DebugMonitor::~DebugMonitor()
{
    m_exiting = true;
    m_debugThread.Join();

    if (m_pGpuDebugMem != nullptr)
    {
        m_pGpuDebugMem->Destroy();
        PAL_FREE(m_pGpuDebugMem, m_pDevice);
        m_pGpuDebugMem = nullptr;
    }
}

// =====================================================================================================================
// Initialize GPU debugging thread and resources
void DebugMonitor::InitGpuDebugging()
{
    Pal::IDevice* pPalDevice = m_pDevice->GetInitInfo().pPalDevice;

    Pal::GpuMemoryCreateInfo createInfo
    {
        .flags = { .gl2Uncached = 1 },
        .size = (DEBUG_BUFFER_SIZE + DEBUG_CONTROL_DWORDS) * sizeof(uint32),
        .heapAccess = Pal::GpuHeapAccessCpuReadMostly,
    };

    Pal::Result result = Pal::Result::Success;

    void* pObject =
        PAL_MALLOC(pPalDevice->GetGpuMemorySize(createInfo, &result), m_pDevice, Util::SystemAllocType::AllocObject);

    if (pObject == nullptr)
    {
        result = Pal::Result::ErrorOutOfMemory;
    }

    if (result == Pal::Result::Success)
    {
        result = pPalDevice->CreateGpuMemory(createInfo, pObject, &m_pGpuDebugMem);
    }

    if (result == Pal::Result::Success)
    {
        result = m_pGpuDebugMem->Map(reinterpret_cast<void**>(&m_pGpuDebugData));
    }

    if (result == Pal::Result::Success)
    {
        Pal::GpuMemoryRef ref
        {
            .pGpuMemory = m_pGpuDebugMem,
        };

        result = pPalDevice->AddGpuMemoryReferences(1, &ref, nullptr, Pal::GpuMemoryRefMustSucceed);
    }

    PAL_ASSERT(result == Pal::Result::Success);

    if (result == Pal::Result::Success)
    {
        m_debugThread.Begin(&GpuDebugThread, this);
    }
}

// =====================================================================================================================
// Print the specified message ID and arguments to debug output
void DebugMonitor::PrintMessage(
    DebugPacket packet,
    uint32 readIndex,
    volatile uint32* pDebugData)
{
#define PRINT_GPU_MESSAGE(...) \
    Util::DbgPrintf(Util::DbgPrintCatInfoMsg, Util::DbgPrintStyleNoPrefix, \
                    GpuDebugMessages[packet.messageId], packet.laneId, __VA_ARGS__);

    PAL_ASSERT(packet.messageId < std::size(GpuDebugMessages));

    auto ReadArg = [&](uint32 i)
    {
        return pDebugData[(readIndex + i) % DEBUG_BUFFER_SIZE];
    };

    switch (packet.argCount)
    {
    case 0:
        PRINT_GPU_MESSAGE();
        break;
    case 1:
        PRINT_GPU_MESSAGE(ReadArg(0));
        break;
    case 2:
        PRINT_GPU_MESSAGE(ReadArg(0), ReadArg(1));
        break;
    case 3:
        PRINT_GPU_MESSAGE(ReadArg(0), ReadArg(1), ReadArg(2));
        break;
    case 4:
        PRINT_GPU_MESSAGE(ReadArg(0), ReadArg(1), ReadArg(2), ReadArg(3));
        break;
    case 5:
        PRINT_GPU_MESSAGE(ReadArg(0), ReadArg(1), ReadArg(2), ReadArg(3), ReadArg(4));
        break;
    case 6:
        PRINT_GPU_MESSAGE(ReadArg(0), ReadArg(1), ReadArg(2), ReadArg(3), ReadArg(4), ReadArg(5));
        break;
    case 7:
        PRINT_GPU_MESSAGE(ReadArg(0), ReadArg(1), ReadArg(2), ReadArg(3), ReadArg(4), ReadArg(5), ReadArg(6));
        break;
    case 8:
        PRINT_GPU_MESSAGE(ReadArg(0), ReadArg(1), ReadArg(2), ReadArg(3), ReadArg(4), ReadArg(5), ReadArg(6), ReadArg(7));
        break;
    default:
        PAL_TRIGGER_ASSERT("Exceeded max supported arguments");
        break;
    }

#undef PRINT_GPU_MESSAGE
}

// =====================================================================================================================
// GPU debugging thread
void DebugMonitor::GpuDebugThread(
    void* pParam)
{
    DebugMonitor* pDebugMonitor = static_cast<DebugMonitor*>(pParam);

    const bool enableAssertBreak = pDebugMonitor->m_pDevice->Settings().gpuDebugFlags & GpuDebugFlags::HostAssert;

    volatile uint32* pWriteIndex = &pDebugMonitor->m_pGpuDebugData[0];
    volatile uint32* pReadIndex  = &pDebugMonitor->m_pGpuDebugData[1];
    volatile uint32* pLock       = &pDebugMonitor->m_pGpuDebugData[2];
    volatile uint32* pDebugData  = &pDebugMonitor->m_pGpuDebugData[3];

    uint32 readIndex = 0;

    *pWriteIndex = 0;
    *pReadIndex  = 0;
    *pLock       = 0;

    while (true)
    {
        while (true)
        {
            if (readIndex == *pWriteIndex)
            {
                break;
            }

            static_assert(sizeof(DebugPacket) == sizeof(uint32));

            const DebugPacket debugPacket { .u32All = pDebugData[readIndex % DEBUG_BUFFER_SIZE] };

            if (debugPacket.assert)
            {
                PAL_ASSERT(debugPacket.argCount == 0);
                Util::DbgPrintf(Util::DbgPrintCatErrorMsg, Util::DbgPrintStyleNoPrefix,
                                GpuDebugMessages[debugPacket.messageId], debugPacket.laneId);

                if (enableAssertBreak)
                {
                    PAL_DEBUG_BREAK();
                }
            }
            else
            {
                PrintMessage(debugPacket, readIndex + 1, pDebugData);
            }

            readIndex += debugPacket.argCount + 1;
            std::atomic_thread_fence(std::memory_order::release);
            *pReadIndex = readIndex;
        }

        if (pDebugMonitor->m_exiting)
        {
            break;
        }

        Util::SleepMs(500);
    }
}
} // namespace GpuRt
#endif
