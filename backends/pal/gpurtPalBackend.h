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
#include "gpurtInlineFuncs.h"
#include "gpurtAccelStruct.h"
#include "gpurtBackend.h"
#include "gpurtInternal.h"

#include "palAssert.h"
#include "palCmdBuffer.h"
#include "palDevice.h"
#include "palMetroHash.h"
#include "palMutex.h"

namespace GpuRt
{

// =====================================================================================================================
class PalBackend : public IBackend
{
public:
    PalBackend(Pal::IDevice*         pDevice,
               const DeviceSettings& deviceSettings)
        : m_pDevice(pDevice)
        , m_deviceSettings(deviceSettings)
        , m_deviceProperties{}
    {
        pDevice->GetProperties(const_cast<Pal::DeviceProperties*>(&m_deviceProperties));
    }

    virtual void SaveComputeState(ClientCmdBufferHandle cmdBuffer) const override;

    virtual void RestoreComputeState(ClientCmdBufferHandle cmdBuffer) const override;

    virtual void SetUserData(
        ClientCmdBufferHandle cmdBuffer,
        uint32                firstEntry,
        uint32                entryCount,
        const uint32*         pEntryValues
    ) const override;

    virtual void BindPipeline(
        ClientCmdBufferHandle cmdBuffer,
        ClientPipelineHandle  pipeline,
        uint64                apiPsoHash
    ) const override;

    virtual void Dispatch(ClientCmdBufferHandle cmdBuffer, uint32 x, uint32 y, uint32 z) const override;

    virtual void DispatchIndirect(ClientCmdBufferHandle cmdBuffer, uint64 indirectArgumentAddr) const override;

    virtual void CommentString(ClientCmdBufferHandle cmdBuffer, const char* comment) const override;

    virtual void WriteImmediateSingle(
        ClientCmdBufferHandle cmdBuffer,
        gpusize               destVa,
        uint64                value,
        ImmediateDataWidth    width
    ) const override;

    virtual uint32 GetMaxDescriptorTableSize(ClientCmdBufferHandle cmdBuffer) const override;

    virtual uint32* RequestTemporaryGpuMemory(
        ClientCmdBufferHandle cmdBuffer,
        uint32                sizeInDwords,
        gpusize*              pGpuAddress
    ) const override;

    virtual void InsertBarrier(ClientCmdBufferHandle cmdBuffer, uint32 flags) const override;

    virtual void CreateBufferViewSrds(
        uint32                count,
        const BufferViewInfo& bufferViewInfo,
        void*                 pOut,
        bool                  isTyped
    ) const override;

    virtual uint32 GetOptimalNumThreadGroups(uint32 threadGroupSize) const override;

    virtual uint32 HashBuildSettings(const CompileTimeBuildSettings& buildSettings) const override;

    virtual void CopyGpuMemoryRegion(
        ClientCmdBufferHandle cmdBuffer,
        gpusize               srcVa,
        gpusize               srcOffset,
        gpusize               dstVa,
        gpusize               dstOffset,
        gpusize               copySize
    ) const override;

    virtual void UpdateMemory(
        ClientCmdBufferHandle  cmdBuffer,
        const Pal::IGpuMemory& cpsVidMem,
        Pal::gpusize           offset,
        Pal::gpusize           size,
        const uint32_t*        pData
    ) const override;

    virtual void WriteTimestamp(
        ClientCmdBufferHandle  cmdBuffer,
        HwPipePoint            hwPipePoint,
        const Pal::IGpuMemory& timeStampVidMem,
        uint64                 offset
    ) const override;

    virtual void PushRGPMarker(
        ClientCmdBufferHandle cmdBuffer,
        const IDevice*        pDevice,
        const char*           pMarker
    ) const override;

    virtual void PopRGPMarker(ClientCmdBufferHandle cmdBuffer, const IDevice* pDevice) const override;

private:
    Pal::IDevice* const         m_pDevice;
    const DeviceSettings        m_deviceSettings;
    const Pal::DeviceProperties m_deviceProperties;

    static inline Pal::ICmdBuffer* GetCmdBuffer(ClientCmdBufferHandle handle)
    {
        return reinterpret_cast<Pal::ICmdBuffer*>(handle);
    }

    static inline const Pal::ChannelMapping SingleChannelMapping =
        { Pal::ChannelSwizzle::X, Pal::ChannelSwizzle::Zero, Pal::ChannelSwizzle::Zero, Pal::ChannelSwizzle::Zero };

    static inline const Pal::ChannelMapping TwoChannelMapping =
        { Pal::ChannelSwizzle::X, Pal::ChannelSwizzle::Y, Pal::ChannelSwizzle::Zero, Pal::ChannelSwizzle::Zero };

    static inline const Pal::ChannelMapping ThreeChannelMapping =
        { Pal::ChannelSwizzle::X, Pal::ChannelSwizzle::Y, Pal::ChannelSwizzle::Z, Pal::ChannelSwizzle::Zero };

    static Pal::BufferViewInfo ConvertBufferViewToPalBufferView(const BufferViewInfo& bufferViewInfo);

    // Queries how many DWORDs of embedded data the command buffer can allocate in one call to AllocateEmbeddedData.
    uint32 GetEmbeddedDataLimit(ClientCmdBufferHandle cmdBuffer) const;

    // Allocates embedded data.
    uint32* AllocateEmbeddedData(
        ClientCmdBufferHandle cmdBuffer,
        uint32                sizeInDwords,
        uint32                alignment,
        gpusize*              pGpuAddress
    ) const;

    // Queries how many DWORDs of embedded data the command buffer can allocate in one call to AllocateLargeEmbeddedData.
    uint32 GetLargeEmbeddedDataLimit(ClientCmdBufferHandle cmdBuffer) const;

    // Allocates embedded data.
    uint32* AllocateLargeEmbeddedData(
        ClientCmdBufferHandle cmdBuffer,
        uint32                sizeInDwords,
        uint32                alignment,
        gpusize*              pGpuAddress
    ) const;
};

} // namespace GpuRt
