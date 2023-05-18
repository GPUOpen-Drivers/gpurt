/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
    namespace Internal
    {
        class Device;
    };
};

namespace GpuRt
{

// =====================================================================================================================
// Trace source for acceleration structure
class AccelStructTraceSource : public GpuUtil::ITraceSource
{
public:
    AccelStructTraceSource(GpuRt::Internal::Device* pDevice)
        :
        m_pDevice(pDevice),
        m_enabled(false)
    {
    }

    virtual ~AccelStructTraceSource()
    {
    }

    virtual void OnConfigUpdated(DevDriver::StructuredValue* pConfig) override;

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

    GpuRt::Internal::Device* m_pDevice;
    bool m_enabled;
};

// =====================================================================================================================
// Trace source for ray history
class RayHistoryTraceSource : public GpuUtil::ITraceSource
{
public:
    RayHistoryTraceSource(GpuRt::Internal::Device* pDevice)
        :
        m_pDevice(pDevice),
        m_active(false),
        m_available(false),
        m_bufferSizeInBytes(67108864),
        m_dispatchID(0)
    {
    }

    virtual ~RayHistoryTraceSource()
    {
    }

    virtual void OnConfigUpdated(DevDriver::StructuredValue* pConfig) override;

    // We only support one GPU currently.
    virtual Pal::uint64 QueryGpuWorkMask() const override
    {
        return 1;
    }

    // Using this notification to do any preparation work that might be required before the trace begins.
    virtual void OnTraceAccepted() override
    {
    }

    virtual void OnTraceBegin(Pal::uint32 gpuIndex, Pal::ICmdBuffer* pCmdBuf) override;

    virtual void OnTraceEnd(Pal::uint32 gpuIndex, Pal::ICmdBuffer* pCmdBuf) override;

    virtual void OnTraceFinished() override;

    virtual const char* GetName() const override
    {
        return "RayHistory";
    }

    virtual Pal::uint32 GetVersion() const override
    {
        return RayHistoryTraceSourceVersion;
    }

    bool Available() const
    {
        return m_available;
    }

    bool Active() const
    {
        return m_active;
    }

    Pal::uint64 GetBufferSizeInBytes()
    {
        return m_bufferSizeInBytes;
    }

    Pal::uint32 GetDispatchID()
    {
        return m_dispatchID++;
    }

private:

    static const Pal::uint32 RayHistoryTraceSourceVersion = 0;

    GpuRt::Internal::Device* m_pDevice;
    bool m_available;
    bool m_active;
    Pal::uint64 m_bufferSizeInBytes;
    Pal::uint32 m_dispatchID;
};
}
