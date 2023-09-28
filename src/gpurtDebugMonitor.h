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
#pragma once

#include "gpurt.h"
#include "shared/debugBuffer.h"

#include "palGpuMemory.h"
#include "palThread.h"

#include <atomic>

#if GPURT_ENABLE_GPU_DEBUG
namespace GpuRt
{

namespace Internal
{
    class Device;
}

class DebugMonitor
{
public:
    explicit DebugMonitor(Internal::Device* pDevice);
    ~DebugMonitor();

    gpusize DebugBufferVa() const
    {
        return (m_pGpuDebugMem != nullptr) ? m_pGpuDebugMem->Desc().gpuVirtAddr : 0;
    }

private:
    void InitGpuDebugging();

    static void PrintMessage(DebugPacket packet, uint32 readIndex, volatile uint32* pDebugData);
    static void GpuDebugThread(void* pParam);

    Internal::Device* m_pDevice;
    Pal::IGpuMemory*  m_pGpuDebugMem{};
    uint32*           m_pGpuDebugData{};
    Util::Thread      m_debugThread{};
    std::atomic<bool> m_exiting;
};

} // namespace GpuRt
#endif
