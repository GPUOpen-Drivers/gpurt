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

#pragma once
#include "palTraceSession.h"

namespace Pal
{
class ICmdBuffer;
};

namespace GpuRt
{
class Device;
};

namespace GpuRt
{

class AccelStructTraceSource : public GpuUtil::ITraceSource
{
public:
    AccelStructTraceSource(GpuRt::Device* pDevice)
        :
        m_pDevice(pDevice),
        m_enabled(false)
    {
    }

    virtual ~AccelStructTraceSource()
    {
    }

#if PAL_CLIENT_INTERFACE_MAJOR_VERSION >= 712
    virtual void OnConfigUpdated(DevDriver::StructuredValue* pConfig) override;
#else
    virtual void OnConfigUpdated(const char* pJsonConfig) override;
#endif

    // We only support one GPU currently.
    virtual Pal::uint64 QueryGpuWorkMask() const override
    {
        return 1;
    }

    // Using this notification to do any preparation work that might be required before the trace begins.
    virtual void OnTraceAccepted() override
    {
    }

    // Begin accumulating a list of all TLAS built
    virtual void OnTraceBegin(Pal::uint32 gpuIndex, Pal::ICmdBuffer* pCmdBuf) override;

    // Stop tracking TLAS builds
    virtual void OnTraceEnd(Pal::uint32 gpuIndex, Pal::ICmdBuffer* pCmdBuf) override;

    virtual void OnTraceFinished() override;

    virtual const char* GetName() const override
    {
        return "AccelStruct";
    }

    virtual Pal::uint32 GetVersion() const override
    {
        return AccelStructTraceSourceVersion;
    }

    bool Enabled() const
    {
        return m_enabled;
    }

private:

    static const Pal::uint32 AccelStructTraceSourceVersion = 0;

    GpuRt::Device* m_pDevice;
    bool m_enabled;
};
}
