/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2022 Advanced Micro Devices, Inc. All Rights Reserved.
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

#include "gpurtTraceSource.h"
#include "gpurt/gpurt.h"
#include "util/ddStructuredReader.h"

namespace GpuRt
{
#if PAL_CLIENT_INTERFACE_MAJOR_VERSION >= 712
// =====================================================================================================================
// Process updated trace config
void AccelStructTraceSource::OnConfigUpdated(
    DevDriver::StructuredValue* pConfig)
{
    DevDriver::StructuredValue enabled;

    if (pConfig->GetValueByKey("enabled", &enabled))
    {
        enabled.GetBool(&m_enabled);
    }
}
#else
// =====================================================================================================================
// Process updated trace config
void AccelStructTraceSource::OnConfigUpdated(
    const char* pJsonConfig)
{
    // Assume a config update means the trace is enabled on the old interface.
    m_enabled = true;
}
#endif

// =====================================================================================================================
// Begin accumulating a list of all TLAS built
void AccelStructTraceSource::OnTraceBegin(
    uint32           gpuIndex,
    Pal::ICmdBuffer* pCmdBuf)
{
    if (m_pDevice->AccelStructTrackerGpuAddr() != 0)
    {
        // Before starting the trace set tracking to enabled.
        pCmdBuf->CmdWriteImmediate(Pal::HwPipeBottom, 1, Pal::ImmediateDataWidth::ImmediateData32Bit,
                                   m_pDevice->AccelStructTrackerGpuAddr() + offsetof(AccelStructTracker, enabled));
    }
    m_pDevice->BeginBvhTrace();
}

// =====================================================================================================================
// Stop tracking TLAS builds
void AccelStructTraceSource::OnTraceEnd(
    uint32           gpuIndex,
    Pal::ICmdBuffer* pCmdBuf)
{
    if (m_pDevice->AccelStructTrackerGpuAddr() != 0)
    {
        // Disable tracking.
        pCmdBuf->CmdWriteImmediate(Pal::HwPipeBottom, 0, Pal::ImmediateDataWidth::ImmediateData32Bit,
                                   m_pDevice->AccelStructTrackerGpuAddr() + offsetof(AccelStructTracker, enabled));
    }
    m_pDevice->EndBvhTrace();
}

// =====================================================================================================================
void AccelStructTraceSource::OnTraceFinished()
{
    m_pDevice->WriteCapturedBvh(this);
}
}
