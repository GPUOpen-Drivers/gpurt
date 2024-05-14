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
#pragma once
#include "gpurt.h"
#include "gpurtBuildSettings.h"
#include "gpurtCounter.h"
#include "palMutex.h"

namespace GpuRt
{

// =====================================================================================================================
enum class BufferViewFormat : uint32
{
    Undefined = 0,
    R32G32B32_Float,
    R32G32_Float,
    R16G16B16A16_Float,
    R16G16_Float,
    R16G16B16A16_Snorm,
    R16G16_Snorm,
    R16G16B16A16_Unorm,
    R16G16_Unorm,
    R10G10B10A2_Unorm,
    R8G8B8A8_Snorm,
    R8G8_Snorm,
    R8G8B8A8_Unorm,
    R8G8_Unorm,

    R32_Float,
    R16_Float,
    R16_Snorm,
    R16_Unorm,
    R8_Snorm,
    R8_Unorm,
};

// =====================================================================================================================
enum class BufferViewSwizzle : uint32
{
    Undefined = 0,
    SingleChannelMapping,
    TwoChannelMapping,
    ThreeChannelMapping,
};

struct BufferViewInfo
{
    gpusize           gpuAddr;
    gpusize           range;
    gpusize           stride;
    BufferViewFormat  format;
    BufferViewSwizzle swizzle;
};

// =====================================================================================================================
// Copy of Pal::HwPipePoint with values we use.
enum class HwPipePoint : uint32
{
    HwPipeTop    = 0x0,
    HwPipePreCs  = 0x1,
    HwPipeBottom = 0x7,
};

// =====================================================================================================================
enum BarrierFlags : uint32
{
    BarrierFlagSyncDispatch    = 0x1, // Stall the following dispatch until all previous dispatch done
    BarrierFlagSyncIndirectArg = 0x2, // Prepare previous shader output for indirect argument use
    BarrierFlagSyncPostCpWrite = 0x4, // Prepare data set by CP for shader use
};

// =====================================================================================================================
// Copy of Pal::ImmediateDataWidth.
enum class ImmediateDataWidth : uint32
{
    ImmediateData32Bit = 0x0,
    ImmediateData64Bit = 0x1,
    Count
};

class IBackend
{
public:
    // Saves compute state, typically prior to using an RT pipeline.
    virtual void SaveComputeState(ClientCmdBufferHandle cmdBuffer) const = 0;

    // Restores compute state, typically after using an RT pipeline.
    virtual void RestoreComputeState(ClientCmdBufferHandle cmdBuffer) const = 0;

    // Sets up data for an RT pipeline.
    virtual void SetUserData(
        ClientCmdBufferHandle cmdBuffer,
        uint32                firstEntry,
        uint32                entryCount,
        const uint32*         pEntryValues) const = 0;

    // Binds a raytracing pipeline, which may be lazily initialized in the pipeline map.
    virtual void BindPipeline(
        ClientCmdBufferHandle cmdBuffer,
        ClientPipelineHandle  pipeline,
        uint64                apiPsoHash) const = 0;

    // Dispatches using a given size.
    virtual void Dispatch(ClientCmdBufferHandle cmdBuffer, uint32 x, uint32 y, uint32 z) const = 0;

    virtual void DispatchIndirect(ClientCmdBufferHandle cmdBuffer, uint64 indirectArgumentAddr) const = 0;

    // Inserts a comment string.
    virtual void CommentString(ClientCmdBufferHandle cmdBuffer, const char* comment) const = 0;

    // Writes immediate data after UAV barriers are done waiting.
    virtual void WriteImmediateSingle(
        ClientCmdBufferHandle cmdBuffer,
        gpusize               destVa,
        uint64                value,
        ImmediateDataWidth    width) const = 0;

    // Calculates the maximum number of geometries that will fit in an SRD table.
    virtual uint32 GetMaxDescriptorTableSize(ClientCmdBufferHandle cmdBuffer) const = 0;

    // Requests temporary mapped GPU memory.
    // May return a nullptr if the request is too large to be allocated.
    virtual uint32* RequestTemporaryGpuMemory(
        ClientCmdBufferHandle cmdBuffer,
        uint32                sizeInDwords,
        gpusize*              pGpuAddress) const = 0;

    // Performs a generic barrier that's used to synchronize internal ray tracing shaders
    virtual void InsertBarrier(ClientCmdBufferHandle cmdBuffer, uint32 flags) const = 0;

    // Creates typed or untyped buffer view SRDs, typically for writing descriptor tables.
    virtual void CreateBufferViewSrds(
        uint32                count,
        const BufferViewInfo& bufferViewInfo,
        void*                 pOut,
        bool                  isTyped) const = 0;

    // Returns the optimal number of thread groups to use for the current hardware, given a desired thread group size.
    virtual uint32 GetOptimalNumThreadGroups(uint32 threadGroupSize) const = 0;

    // Computes a hash value for build settings.
    virtual uint32 HashBuildSettings(const CompileTimeBuildSettings& buildSettings) const = 0;

    // Copies memory from one slice of GPU memory to another.
    virtual void CopyGpuMemoryRegion(
        ClientCmdBufferHandle cmdBuffer,
        gpusize               srcVa,
        gpusize               srcOffset,
        gpusize               dstVa,
        gpusize               dstOffset,
        gpusize               copySize) const = 0;

    // Uploads data to video memory using CP DMA.
    virtual void UpdateMemory(
        ClientCmdBufferHandle  cmdBuffer,
        const Pal::IGpuMemory& cpsVidMem,
        Pal::gpusize           offset,
        Pal::gpusize           size,
        const uint32_t*        pData) const = 0;

    // Writes a timestamp at a given offset into video memory.
    // Will eventually replaced with a callback or other abstraction to avoid referencing video memory.
    virtual void WriteTimestamp(
        ClientCmdBufferHandle  cmdBuffer,
        HwPipePoint            hwPipePoint,
        const Pal::IGpuMemory& timeStampVidMem,
        uint64                 offset) const = 0;

    virtual void PushRGPMarker(ClientCmdBufferHandle cmdBuffer, const IDevice* pDevice, const char* pMarker) const = 0;

    virtual void PopRGPMarker(ClientCmdBufferHandle cmdBuffer, const IDevice* pDevice) const = 0;
};

}
