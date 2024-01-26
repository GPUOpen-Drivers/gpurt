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

#include "gpurt.h"
#include "gpurtPalBackend.h"

#include "gpurtInternal.h"

namespace GpuRt
{

// =====================================================================================================================
// Create GpuRt device using client-provided memory.
Pal::Result GPURT_API_ENTRY CreateDevice(
    const DeviceInitInfo&  info,
    const ClientCallbacks& callbacks,
    void*            const pMemory,
    IDevice**        const ppDevice)
{
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 40
    GPURT_EXPORT_UNMANGLED_SYMBOL_MSVC
#endif

    Pal::Result result = Pal::Result::Success;

    {
        size_t internalDeviceSize = Internal::GetDeviceSize();

        // Currently, client memory management assumes that the allocated GpuRt device will be at the beginning of
        // pMemory, so we allocate the backend after the device memory to avoid a runtime error on client-side device
        // destruction.
        PalBackend* pBackend = PAL_PLACEMENT_NEW(Util::VoidPtrInc(pMemory, internalDeviceSize))
                                                 PalBackend(info.pPalDevice, info.deviceSettings);
        result = Internal::CreateDevice(
            info,
            callbacks,
            pBackend,
            pMemory,
            ppDevice);
    }

    return result;
}

// =====================================================================================================================
// Calculate the total size in bytes needed to allocate GpuRt's device.
size_t GPURT_API_ENTRY GetDeviceSize()
{
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 40
    GPURT_EXPORT_UNMANGLED_SYMBOL_MSVC
#endif

    size_t totalSize = 0;

    {
        // Report a higher size requirement to the client so we have space to allocate the backend.
        totalSize += sizeof(PalBackend);
        totalSize += Internal::GetDeviceSize();
    }

    return totalSize;
}

// =====================================================================================================================
PipelineShaderCode GPURT_API_ENTRY GetShaderLibraryCode(
    ShaderLibraryFeatureFlags flags)
{
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 40
    GPURT_EXPORT_UNMANGLED_SYMBOL_MSVC
#endif

    return Internal::GetShaderLibraryCode(flags);
}

// =====================================================================================================================
Pal::Result GPURT_API_ENTRY QueryRayTracingEntryFunctionTable(
    const Pal::RayTracingIpLevel   rayTracingIpLevel,
    EntryFunctionTable* const      pEntryFunctionTable)
{
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 40
    GPURT_EXPORT_UNMANGLED_SYMBOL_MSVC
#endif

    return Internal::QueryRayTracingEntryFunctionTable(
        rayTracingIpLevel,
        pEntryFunctionTable
    );
}

} // namespace GpuRt
