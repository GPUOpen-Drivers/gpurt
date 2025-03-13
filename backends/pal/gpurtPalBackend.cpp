/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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

#include "gpurtPalBackend.h"

namespace GpuRt
{

// =====================================================================================================================
static Pal::ImmediateDataWidth GpuRtToPalImmediateDataWidth(
    ImmediateDataWidth gpurtImmediateDataWidth)
{
#define IMMEDIATEDATAWIDTHCASE(x) case static_cast<uint32>(Pal::ImmediateDataWidth::x): \
                                      return Pal::ImmediateDataWidth::x
    switch (static_cast<uint32>(gpurtImmediateDataWidth))
    {
        IMMEDIATEDATAWIDTHCASE(ImmediateData32Bit);
        IMMEDIATEDATAWIDTHCASE(ImmediateData64Bit);
        IMMEDIATEDATAWIDTHCASE(Count);
        default:
            PAL_ASSERT_ALWAYS_MSG("Unhandled ImmediateDataWidth value in conversion: %u\n",
                                  static_cast<uint32>(gpurtImmediateDataWidth));
            return Pal::ImmediateDataWidth::Count;
    }
#undef IMMEDIATEDATAWIDTHCASE
}

// =====================================================================================================================
void PalBackend::SaveComputeState(
    ClientCmdBufferHandle cmdBuffer
    ) const
{
    GetCmdBuffer(cmdBuffer)->CmdSaveComputeState(Pal::ComputeStateAll);
}

// =====================================================================================================================
void PalBackend::RestoreComputeState(
    ClientCmdBufferHandle cmdBuffer
    ) const
{
    GetCmdBuffer(cmdBuffer)->CmdRestoreComputeState(Pal::ComputeStateAll);
}

// =====================================================================================================================
void PalBackend::SetUserData(
    ClientCmdBufferHandle cmdBuffer,
    uint32                firstEntry,
    uint32                entryCount,
    const uint32*         pEntryValues
    ) const
{
    GetCmdBuffer(cmdBuffer)->CmdSetUserData(Pal::PipelineBindPoint::Compute,
                                            firstEntry,
                                            entryCount,
                                            pEntryValues);
}

// =====================================================================================================================
void PalBackend::BindPipeline(
    ClientCmdBufferHandle cmdBuffer,
    ClientPipelineHandle  pipeline,
    uint64                apiPsoHash
    ) const
{
    Pal::PipelineBindParams bindParam = {};
    bindParam.pipelineBindPoint       = Pal::PipelineBindPoint::Compute;
    bindParam.apiPsoHash              = apiPsoHash;
    bindParam.pPipeline               = reinterpret_cast<Pal::IPipeline*>(pipeline);

    GetCmdBuffer(cmdBuffer)->CmdBindPipeline(bindParam);
}

// =====================================================================================================================
void PalBackend::DispatchIndirect(
    ClientCmdBufferHandle cmdBuffer,
    uint64                indirectArgumentAddr
    ) const
{
    // PAL major version 838 added dispatch indirect by GPUVA. It is available before the client updates to 838, so
    // there is no need to check the PAL client version.
#if PAL_INTERFACE_MAJOR_VERSION >= 838
    GetCmdBuffer(cmdBuffer)->CmdDispatchIndirect(indirectArgumentAddr);
#endif
}

// =====================================================================================================================
void PalBackend::Dispatch(
    ClientCmdBufferHandle cmdBuffer,
    uint32                x,
    uint32                y,
    uint32                z
    ) const
{
#if PAL_INTERFACE_MAJOR_VERSION >= 909
    GetCmdBuffer(cmdBuffer)->CmdDispatch({ x, y, z }, {});
#else
    GetCmdBuffer(cmdBuffer)->CmdDispatch({ x, y, z });
#endif
}

// =====================================================================================================================
void PalBackend::CommentString(
    ClientCmdBufferHandle cmdBuffer,
    const char*           comment
    ) const
{
    GetCmdBuffer(cmdBuffer)->CmdCommentString(comment);
}

// =====================================================================================================================
void PalBackend::WriteImmediateSingle(
    ClientCmdBufferHandle cmdBuffer,
    gpusize               destVa,
    uint64                value,
    ImmediateDataWidth    width
    ) const
{
    // We want to use StagePostPrefetch (ME) so that the writes do not occur before UAV barriers are done waiting.
    // Both internal barriers during the build and application barriers synchronizing access to acceleration
    // structure memory wait at StagePostPrefetch.
    GetCmdBuffer(cmdBuffer)->CmdWriteImmediate(
        Pal::PipelineStageFlag::PipelineStagePostPrefetch,
        value,
        GpuRtToPalImmediateDataWidth(width),
        destVa);
}

// =====================================================================================================================
uint32 PalBackend::GetMaxDescriptorTableSize(
    ClientCmdBufferHandle cmdBuffer
    ) const
{
    const uint32 bufferSrdSizeDw = Util::Max(m_deviceProperties.gfxipProperties.srdSizes.typedBufferView,
                                             m_deviceProperties.gfxipProperties.srdSizes.untypedBufferView) /
                                   sizeof(uint32);
    return GetCmdBuffer(cmdBuffer)->GetLargeEmbeddedDataLimit() / bufferSrdSizeDw;
}

// =====================================================================================================================
uint32 PalBackend::GetEmbeddedDataLimit(ClientCmdBufferHandle cmdBuffer) const
{
    return GetCmdBuffer(cmdBuffer)->GetEmbeddedDataLimit();
}

// =====================================================================================================================
uint32* PalBackend::AllocateEmbeddedData(
    ClientCmdBufferHandle cmdBuffer,
    uint32                sizeInDwords,
    uint32                alignment,
    gpusize*              pGpuAddress
    ) const
{
    return GetCmdBuffer(cmdBuffer)->CmdAllocateEmbeddedData(sizeInDwords, alignment, pGpuAddress);
}

// =====================================================================================================================
uint32 PalBackend::GetLargeEmbeddedDataLimit(ClientCmdBufferHandle cmdBuffer) const
{
    return GetCmdBuffer(cmdBuffer)->GetLargeEmbeddedDataLimit();
}

// =====================================================================================================================
uint32* PalBackend::AllocateLargeEmbeddedData(
    ClientCmdBufferHandle cmdBuffer,
    uint32                sizeInDwords,
    uint32                alignment,
    gpusize*              pGpuAddress
    ) const
{
    return GetCmdBuffer(cmdBuffer)->CmdAllocateLargeEmbeddedData(sizeInDwords, alignment, pGpuAddress);
}

// =====================================================================================================================
uint32* PalBackend::RequestTemporaryGpuMemory(
    ClientCmdBufferHandle cmdBuffer,
    uint32                sizeInDwords,
    gpusize*              pGpuAddress
    ) const
{
    const uint32 embeddedDataLimitInDwords = GetEmbeddedDataLimit(cmdBuffer);
    const uint32 largeEmbeddedDataLimitInDwords = GetLargeEmbeddedDataLimit(cmdBuffer);

    uint32* pMappedData = nullptr;
    if (sizeInDwords <= embeddedDataLimitInDwords)
    {
        pMappedData = AllocateEmbeddedData(cmdBuffer, sizeInDwords, 1, pGpuAddress);
    }
    else if (sizeInDwords <= largeEmbeddedDataLimitInDwords)
    {
        pMappedData = AllocateLargeEmbeddedData(cmdBuffer, sizeInDwords, 1, pGpuAddress);
    }

    return pMappedData;
}

// =====================================================================================================================
void PalBackend::InsertBarrier(
    ClientCmdBufferHandle cmdBuffer,
    uint32                flags
    ) const
{
    const bool syncDispatch     = flags & BarrierFlagSyncDispatch;
    const bool syncIndirectArgs = flags & BarrierFlagSyncIndirectArg;
    const bool syncPreCpWrite   = flags & BarrierFlagSyncPreCpWrite;
    const bool syncPostCpWrite  = flags & BarrierFlagSyncPostCpWrite;

    Pal::ICmdBuffer* pCmdBuffer = GetCmdBuffer(cmdBuffer);

    Pal::AcquireReleaseInfo acqRelInfo  = {};
    Pal::MemBarrier memoryBarrier       = {};

    if (syncDispatch || syncIndirectArgs)
    {
        memoryBarrier.srcStageMask  |= Pal::PipelineStageCs;
        memoryBarrier.srcAccessMask |= Pal::CoherShader;
    }

    if (syncPreCpWrite)
    {
#if PAL_BUILD_GFX12
        // Clients are expected to wait at PipelineStagePostPrefetch for API-level AS-related barrier operations.
        // However, the CoherCp access transition (GL2 flush on GFX12) is deferred until GPURT requires it.
#endif
        memoryBarrier.srcStageMask  |= Pal::PipelineStagePostPrefetch;
        memoryBarrier.srcAccessMask |= Pal::CoherShader;
        memoryBarrier.dstStageMask  |= Pal::PipelineStagePostPrefetch;
        memoryBarrier.dstAccessMask |= Pal::CoherCp;
    }

    if (syncPostCpWrite)
    {
        memoryBarrier.srcStageMask  |= Pal::PipelineStagePostPrefetch;
        memoryBarrier.srcAccessMask |= Pal::CoherCp;
    }

    if (syncDispatch || syncPostCpWrite)
    {
        memoryBarrier.dstStageMask  = Pal::PipelineStageCs;
        memoryBarrier.dstAccessMask = Pal::CoherShader;
    }

    if (syncIndirectArgs)
    {
        memoryBarrier.dstStageMask  |= Pal::PipelineStageFetchIndirectArgs;
        memoryBarrier.dstAccessMask |= Pal::CoherIndirectArgs;
    }

    acqRelInfo.memoryBarrierCount = 1;
    acqRelInfo.pMemoryBarriers    = &memoryBarrier;
    acqRelInfo.reason             = m_deviceSettings.rgpBarrierReason;

    pCmdBuffer->CmdReleaseThenAcquire(acqRelInfo);
}

// =====================================================================================================================
void PalBackend::CreateTypedBufferViewSrds(
    const BufferViewInfo& bufferViewInfo,
    void*                 pOut) const
{
    const uint32 bufferSrdSizeDw = m_deviceProperties.gfxipProperties.srdSizes.typedBufferView  / sizeof(uint32);

    const Pal::BufferViewInfo palBufferViewInfo = ConvertBufferViewToPalBufferView(bufferViewInfo);
    const void* pNullBuffer = m_deviceProperties.gfxipProperties.nullSrds.pNullBufferView;

    if (bufferViewInfo.gpuAddr == 0)
    {
        memcpy(pOut, pNullBuffer, bufferSrdSizeDw * sizeof(uint32));
    }
    else
    {
        m_pDevice->CreateTypedBufferViewSrds(1, &palBufferViewInfo, pOut);
    }
}

// =====================================================================================================================
void PalBackend::CreateUntypedBufferViewSrds(
    const BufferViewInfo& bufferViewInfo,
    void*                 pOut) const
{
    const uint32 bufferSrdSizeDw = m_deviceProperties.gfxipProperties.srdSizes.untypedBufferView / sizeof(uint32);

    const Pal::BufferViewInfo palBufferViewInfo = ConvertBufferViewToPalBufferView(bufferViewInfo);
    const void* pNullBuffer = m_deviceProperties.gfxipProperties.nullSrds.pNullBufferView;

    if (bufferViewInfo.gpuAddr == 0)
    {
        memcpy(pOut, pNullBuffer, bufferSrdSizeDw * sizeof(uint32));
    }
    else
    {
        m_pDevice->CreateUntypedBufferViewSrds(1, &palBufferViewInfo, pOut);
    }
}

// =====================================================================================================================
uint32 PalBackend::GetOptimalNumThreadGroups(
    uint32 threadGroupSize
    ) const
{
    const auto* pProps = &m_deviceProperties.gfxipProperties.shaderCore;
    const uint32 wavesPerGroup = Util::RoundUpQuotient(threadGroupSize, pProps->nativeWavefrontSize);

    return Util::RoundUpQuotient((pProps->numAvailableCus * pProps->numSimdsPerCu), wavesPerGroup);
}

// =====================================================================================================================
uint32 PalBackend::HashBuildSettings(
    const CompileTimeBuildSettings& buildSettings
    ) const
{
    Util::MetroHash::Hash hash = {};
    Util::MetroHash64::Hash(reinterpret_cast<const uint8*>(&buildSettings), sizeof(buildSettings), &hash.bytes[0]);

    return Util::MetroHash::Compact32(&hash);
}

// =====================================================================================================================
void PalBackend::CopyGpuMemoryRegion(
    ClientCmdBufferHandle cmdBuffer,
    gpusize               srcVa,
    gpusize               srcOffset,
    gpusize               dstVa,
    gpusize               dstOffset,
    gpusize               copySize
    ) const
{
    Pal::MemoryCopyRegion region = {};
    region.srcOffset = srcOffset;
    region.dstOffset = dstOffset;
    region.copySize  = copySize;
    GetCmdBuffer(cmdBuffer)->CmdCopyMemoryByGpuVa(srcVa, dstVa, 1, &region);
}

// =====================================================================================================================
void PalBackend::UpdateMemory(
    ClientCmdBufferHandle  cmdBuffer,
    const Pal::IGpuMemory& cpsVidMem,
    Pal::gpusize           offset,
    Pal::gpusize           size,
    const uint32_t*        pData) const
{
    GetCmdBuffer(cmdBuffer)->CmdUpdateMemory(cpsVidMem, offset, size, pData);
}

// =====================================================================================================================
void PalBackend::WriteTimestamp(
    ClientCmdBufferHandle  cmdBuffer,
    const Pal::IGpuMemory& timeStampVidMem,
    uint64                 offset
    ) const
{
    GetCmdBuffer(cmdBuffer)->CmdWriteTimestamp(Pal::PipelineStageBottomOfPipe, timeStampVidMem, offset);
}

// =====================================================================================================================
void PalBackend::PushRGPMarker(
    ClientCmdBufferHandle cmdBuffer,
    const IDevice*        pDevice,
    const char*           pMarker
    ) const
{
#if GPURT_DEVELOPER
    pDevice->GetClientCallbacks().pfnInsertRGPMarker(cmdBuffer, pMarker, true);
#endif
}

// =====================================================================================================================
void PalBackend::PopRGPMarker(
    ClientCmdBufferHandle cmdBuffer,
    const IDevice*        pDevice
    ) const
{
#if GPURT_DEVELOPER
    pDevice->GetClientCallbacks().pfnInsertRGPMarker(cmdBuffer, nullptr, false);
#endif
}

// =====================================================================================================================
Pal::BufferViewInfo PalBackend::ConvertBufferViewToPalBufferView(
    const BufferViewInfo& bufferViewInfo)
{
    Pal::BufferViewInfo palBufferViewInfo = {};
    palBufferViewInfo.gpuAddr = bufferViewInfo.gpuAddr;
    palBufferViewInfo.stride  = bufferViewInfo.stride;
    palBufferViewInfo.range   = bufferViewInfo.range;
    switch (bufferViewInfo.format)
    {
        case BufferViewFormat::Undefined:
            break;
        case BufferViewFormat::R32G32B32_Float:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X32Y32Z32_Float;
            break;
        case BufferViewFormat::R32G32_Float:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X32Y32_Float;
            break;
        case BufferViewFormat::R16G16B16A16_Float:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X16Y16Z16W16_Float;
            break;
        case BufferViewFormat::R16G16_Float:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X16Y16_Float;
            break;
        case BufferViewFormat::R16G16B16A16_Snorm:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X16Y16Z16W16_Snorm;
            break;
        case BufferViewFormat::R16G16_Snorm:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X16Y16_Snorm;
            break;
        case BufferViewFormat::R16G16B16A16_Unorm:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X16Y16Z16W16_Unorm;
            break;
        case BufferViewFormat::R16G16_Unorm:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X16Y16_Unorm;
            break;
        case BufferViewFormat::R10G10B10A2_Unorm:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X10Y10Z10W2_Unorm;
            break;
        case BufferViewFormat::R8G8B8A8_Snorm:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X8Y8Z8W8_Snorm;
            break;
        case BufferViewFormat::R8G8_Snorm:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X8Y8_Snorm;
            break;
        case BufferViewFormat::R8G8B8A8_Unorm:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X8Y8Z8W8_Unorm;
            break;
        case BufferViewFormat::R8G8_Unorm:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X8Y8_Unorm;
            break;
        case BufferViewFormat::R32_Float:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X32_Float;
            break;
        case BufferViewFormat::R16_Float:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X16_Float;
            break;
        case BufferViewFormat::R16_Snorm:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X16_Snorm;
            break;
        case BufferViewFormat::R16_Unorm:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X16_Unorm;
            break;
        case BufferViewFormat::R8_Snorm:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X8_Snorm;
            break;
        case BufferViewFormat::R8_Unorm:
            palBufferViewInfo.swizzledFormat.format = Pal::ChNumFormat::X8_Unorm;
            break;
        default:
            PAL_ASSERT_ALWAYS();
            break;
    }
    switch (bufferViewInfo.swizzle)
    {
        case BufferViewSwizzle::Undefined:
            break;
        case BufferViewSwizzle::SingleChannelMapping:
            palBufferViewInfo.swizzledFormat.swizzle = SingleChannelMapping;
            break;
        case BufferViewSwizzle::TwoChannelMapping:
            palBufferViewInfo.swizzledFormat.swizzle = TwoChannelMapping;
            break;
        case BufferViewSwizzle::ThreeChannelMapping:
            palBufferViewInfo.swizzledFormat.swizzle = ThreeChannelMapping;
            break;
        default:
            PAL_ASSERT_ALWAYS();
            break;
    }
    return palBufferViewInfo;
}

} // namespace GpuRt
