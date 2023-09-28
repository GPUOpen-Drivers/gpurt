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

#include "gpurt/gpurt.h"

namespace Pal
{
class ICmdBuffer;
}

namespace GpuRt
{
class BvhBuilder;

// =====================================================================================================================
// Helper class used by GPURT to manage BVH batches.
class BvhBatcher
{
public:
    explicit BvhBatcher(
        Pal::ICmdBuffer*             pCmdBuf,
        Internal::Device*      const pDevice,
        const Pal::DeviceProperties& deviceProps,
        ClientCallbacks              clientCb,
        const DeviceSettings&        deviceSettings);

    void BuildAccelerationStructureBatch(
        Util::Span<const AccelStructBuildInfo> buildInfos);

    void BuildMultiDispatch(
        Util::Span<BvhBuilder> builders);

    void RadixSort(
        Util::Span<BvhBuilder> builders);

private:
    void BuildRaytracingAccelerationStructureBatch(
        Util::Span<BvhBuilder> builders,
        Util::Span<BvhBuilder> updaters);

    void DispatchInitAccelerationStructure(
        Util::Span<BvhBuilder> builders);

    void Barrier();

    template <typename BatchBuilderFunc>
    void BuildFunction(
        const char*            rgpMarkerName,
        Util::Span<BvhBuilder> builders,
        BatchBuilderFunc       func);

    template <typename BuilderPhase>
    void BuildPhase(
        const char*            rgpMarkerName,
        Util::Span<BvhBuilder> builders,
        BuilderPhase           pBuilderPhase);

    template <typename BuilderPhase>
    void BuildPhase(
        Util::Span<BvhBuilder> builders,
        BuilderPhase           pBuilderPhase);

#if GPURT_DEVELOPER
    // Driver generated RGP markers are only added in internal builds because they expose details about the
    // construction of acceleration structure.
    template <class... Args>
    void PushRGPMarker(const char* pFormat, Args&&... args);
    void PopRGPMarker();

    void OutputPipelineName(InternalRayTracingCsType type);
#endif

    Internal::Device*           const m_pDevice;             // GPURT device
    Pal::ICmdBuffer*                  m_pPalCmdBuffer;       // The associated PAL cmdbuffer
    const Pal::DeviceProperties&      m_deviceProps;
    ClientCallbacks                   m_clientCb;
    const DeviceSettings&             m_deviceSettings;      // Device settings
};

};
