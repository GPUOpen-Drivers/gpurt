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

#include "gpurtPalBackend.h"

namespace GpuRt
{

// =====================================================================================================================
// GPURT to PAL enum conversions without undefined behavior.
static Pal::HwPipePoint GpuRtToPalHwPipePoint(
    HwPipePoint gpurtHwPipePoint)
{
#define HWPIPEPOINTCASE(x) case static_cast<uint32>(Pal::HwPipePoint::x): return Pal::HwPipePoint::x
    switch (static_cast<uint32>(gpurtHwPipePoint))
    {
        HWPIPEPOINTCASE(HwPipeTop);
        HWPIPEPOINTCASE(HwPipePreCs);
        HWPIPEPOINTCASE(HwPipeBottom);
        default:
            PAL_ASSERT_ALWAYS_MSG("Unhandled HwPipePoint value in conversion: %u\n",
                                  static_cast<uint32>(gpurtHwPipePoint));
            return Pal::HwPipePoint::HwPipeTop;
    }
#undef HWPIPEPOINTCASE
}

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
void PalBackend::Dispatch(
    ClientCmdBufferHandle cmdBuffer,
    uint32                x,
    uint32                y,
    uint32                z
    ) const
{
    GetCmdBuffer(cmdBuffer)->CmdDispatch({ x, y, z });
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
    // We want to use HwPipePreCs (ME) so that the writes do not occur before UAV barriers are done waiting.
    // Both internal barriers during the build and application barriers synchronizing access to acceleration
    // structure memory wait at HwPipePreCs.
    GetCmdBuffer(cmdBuffer)->CmdWriteImmediate(
        Pal::HwPipePoint::HwPipePreCs,
        value,
        GpuRtToPalImmediateDataWidth(width),
        destVa);
}

// =====================================================================================================================
uint32 PalBackend::GetMaxDescriptorTableSize(
    ClientCmdBufferHandle cmdBuffer
    ) const
{
    const uint32 bufferSrdSizeDw = m_deviceProperties.gfxipProperties.srdSizes.bufferView / sizeof(uint32);
    return GetCmdBuffer(cmdBuffer)->GetEmbeddedDataLimit() / bufferSrdSizeDw;
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
void PalBackend::InsertBarrier(
    ClientCmdBufferHandle cmdBuffer
    ) const
{
    Pal::ICmdBuffer* pCmdBuffer = GetCmdBuffer(cmdBuffer);
    if (m_deviceSettings.enableAcquireReleaseInterface)
    {
        Pal::AcquireReleaseInfo acqRelInfo = {};
        acqRelInfo.srcGlobalStageMask  = Pal::PipelineStageCs;
        acqRelInfo.dstGlobalStageMask  = Pal::PipelineStageCs;
        acqRelInfo.srcGlobalAccessMask = Pal::CoherShader;
        acqRelInfo.dstGlobalAccessMask = Pal::CoherShader;

        acqRelInfo.reason = m_deviceSettings.rgpBarrierReason;

        pCmdBuffer->CmdReleaseThenAcquire(acqRelInfo);
    }
    else
    {
        Pal::BarrierInfo barrierInfo = {};
        barrierInfo.waitPoint = Pal::HwPipePreCs;

        const Pal::HwPipePoint pipePoint = Pal::HwPipePostCs;
        barrierInfo.pipePointWaitCount = 1;
        barrierInfo.pPipePoints = &pipePoint;

        Pal::BarrierTransition transition = {};
        transition.srcCacheMask = Pal::CoherShader;
        transition.dstCacheMask = Pal::CoherShader;

        barrierInfo.transitionCount = 1;
        barrierInfo.pTransitions = &transition;

        barrierInfo.reason = m_deviceSettings.rgpBarrierReason;

        pCmdBuffer->CmdBarrier(barrierInfo);
    }
}

// =====================================================================================================================
uint32* PalBackend::AllocateDescriptorTable(
    ClientCmdBufferHandle cmdBuffer,
    uint32                count,
    gpusize*              pGpuAddress,
    uint32*               pSrdSizeOut
    ) const
{
    const uint32 bufferSrdSizeDw = m_deviceProperties.gfxipProperties.srdSizes.bufferView / sizeof(uint32);
    *pSrdSizeOut = bufferSrdSizeDw * sizeof(uint32);
    return GetCmdBuffer(cmdBuffer)->CmdAllocateEmbeddedData(count * bufferSrdSizeDw, bufferSrdSizeDw, pGpuAddress);
}

// =====================================================================================================================
void PalBackend::CreateBufferViewSrds(
    uint32                count,
    const BufferViewInfo& bufferViewInfo,
    void*                 pOut,
    bool                  isTyped
    ) const
{
    const uint32 bufferSrdSizeDw = m_deviceProperties.gfxipProperties.srdSizes.bufferView / sizeof(uint32);
    const Pal::BufferViewInfo palBufferViewInfo = ConvertBufferViewToPalBufferView(bufferViewInfo);
    const void* pNullBuffer = m_deviceProperties.gfxipProperties.nullSrds.pNullBufferView;

    if (bufferViewInfo.gpuAddr == 0)
    {
        memcpy(pOut, pNullBuffer, bufferSrdSizeDw * sizeof(uint32));
    }
    else if (isTyped)
    {
        m_pDevice->CreateTypedBufferViewSrds(count, &palBufferViewInfo, pOut);
    }
    else
    {
        m_pDevice->CreateUntypedBufferViewSrds(count, &palBufferViewInfo, pOut);
    }
}

// =====================================================================================================================
uint32 PalBackend::GetOptimalNumThreadGroups(
    uint32 threadGroupSize
    ) const
{
    const auto* pProps = &m_deviceProperties.gfxipProperties.shaderCore;
    const uint32 wavesPerGroup = Util::RoundUpQuotient(threadGroupSize, pProps->nativeWavefrontSize);

    return (pProps->numAvailableCus * pProps->numSimdsPerCu) / wavesPerGroup;
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
void PalBackend::UploadCpuMemory(
    ClientCmdBufferHandle cmdBuffer,
    gpusize               dstBufferVa,
    const void*           pSrcData,
    uint32                sizeInBytes
    ) const
{
    Pal::ICmdBuffer* pCmdBuffer = GetCmdBuffer(cmdBuffer);
    const uint32 embeddedDataLimitDwords = pCmdBuffer->GetEmbeddedDataLimit();
    const uint32 uploadSizeDwords = Util::Pow2Align(sizeInBytes, 4);
    PAL_ASSERT(uploadSizeDwords <= embeddedDataLimitDwords);

    gpusize srcGpuVa = 0;
    uint32* pMappedData = pCmdBuffer->CmdAllocateEmbeddedData(uploadSizeDwords, 1, &srcGpuVa);
    std::memcpy(pMappedData, pSrcData, sizeInBytes);

    CopyGpuMemoryRegion(cmdBuffer, srcGpuVa, 0, dstBufferVa, 0, sizeInBytes);
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
void PalBackend::WriteTimestamp(
    ClientCmdBufferHandle  cmdBuffer,
    HwPipePoint            hwPipePoint,
    const Pal::IGpuMemory& timeStampVidMem,
    uint64                 offset
    ) const
{
    GetCmdBuffer(cmdBuffer)->CmdWriteTimestamp(GpuRtToPalHwPipePoint(hwPipePoint), timeStampVidMem, offset);
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
