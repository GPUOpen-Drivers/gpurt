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

#include "palCmdBuffer.h"
#include "palMetroHash.h"
#include "palVectorImpl.h"

#include "gpurt/gpurt.h"
#include "gpurt/gpurtLib.h"
#include "gpurt/gpurtAccelStruct.h"
#include "gpurtInternal.h"
#include "gpurtInternalShaderBindings.h"
#include "gpurtBvhBatcher.h"
#include "gpurtBvhBuilder.h"
#include "gpurtBvhBuilderCommon.h"
#include "shared/accelStruct.h"

namespace GpuRt
{

// =====================================================================================================================
// Explicit ray tracing bvh batcher constructor
BvhBatcher::BvhBatcher(
    ClientCmdBufferHandle        cmdBuffer,       // The associated cmdbuffer handle
    const IBackend&              backend,         // Backend interface
    Internal::Device*      const pDevice,         // GPURT device pointer
    const Pal::DeviceProperties& deviceProps,     // PAL device properties
    ClientCallbacks              clientCb,        // Client cb table
    const DeviceSettings&        deviceSettings)  // Device settings
    :
    m_cmdBuffer(cmdBuffer),
    m_backend(backend),
    m_pDevice(pDevice),
    m_deviceProps(deviceProps),
    m_clientCb(clientCb),
    m_deviceSettings(deviceSettings)
{
}

// =====================================================================================================================
// Parses the batch of build infos to execute the batched build
void BvhBatcher::BuildAccelerationStructureBatch(
    Util::Span<const AccelStructBuildInfo> buildInfos) // Batch of BVH build infos
{
    Util::Vector<BvhBuilder, 1, Internal::Device> blasBuilders(m_pDevice);
    Util::Vector<BvhBuilder, 1, Internal::Device> blasUpdaters(m_pDevice);
    Util::Vector<BvhBuilder, 1, Internal::Device> tlasBuilders(m_pDevice);
    Util::Vector<BvhBuilder, 1, Internal::Device> tlasUpdaters(m_pDevice);

    Util::Vector<BvhBuilder, 1, Internal::Device> emptyBuilders(m_pDevice);

    blasBuilders.Reserve(static_cast<uint32>(buildInfos.size()));
    blasUpdaters.Reserve(static_cast<uint32>(buildInfos.size()));

    RGP_PUSH_MARKER("Batched BVH Builds");

    for (const AccelStructBuildInfo& info : buildInfos)
    {
        BvhBuilder builder(m_cmdBuffer,
                           m_backend,
                           m_pDevice,
                           m_deviceProps,
                           m_clientCb,
                           m_deviceSettings,
                           info);

        const bool isUpdate = Util::TestAnyFlagSet(info.inputs.flags, AccelStructBuildFlagPerformUpdate);
        const bool isTlas = info.inputs.type == AccelStructType::TopLevel;
        if (builder.m_buildConfig.numPrimitives == 0)
        {
            // Empty BVH builds need to initialize their headers and may emit post build information,
            // but otherwise do not participate in the rest of the build.
            if (isUpdate)
            {
                builder.EmitPostBuildInfo();
            }
            else
            {
                emptyBuilders.EmplaceBack(std::move(builder));
            }
        }
        else if (isUpdate)
        {
            if (isTlas)
            {
                tlasUpdaters.EmplaceBack(std::move(builder));
            }
            else
            {
                blasUpdaters.EmplaceBack(std::move(builder));
            }
        }
        else
        {
            if (isTlas)
            {
                tlasBuilders.EmplaceBack(std::move(builder));
            }
            else
            {
                blasBuilders.EmplaceBack(std::move(builder));
            }
        }
    }

    if (emptyBuilders.IsEmpty() == false)
    {
        RGP_PUSH_MARKER("Process Empty BVH builds");
        DispatchInitAccelerationStructure<false>(emptyBuilders);
        BuildPhase(emptyBuilders, &BvhBuilder::EmitPostBuildInfo);
        RGP_POP_MARKER();
    }

    if ((blasBuilders.IsEmpty() == false) || (blasUpdaters.IsEmpty() == false))
    {
        RGP_PUSH_MARKER("BLAS: %u builds, %u updates", blasBuilders.size(), blasUpdaters.size());
        BuildRaytracingAccelerationStructureBatch<false>(blasBuilders, blasUpdaters);
        RGP_POP_MARKER();
    }
    if ((tlasBuilders.IsEmpty() == false) || (tlasUpdaters.IsEmpty() == false))
    {
        RGP_PUSH_MARKER("TLAS: %u builds, %u updates", tlasBuilders.size(), tlasUpdaters.size());
        BuildRaytracingAccelerationStructureBatch<true>(tlasBuilders, tlasUpdaters);
        RGP_POP_MARKER();
    }

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes each build phase for each builder within the batch
template <bool IsTlas>
void BvhBatcher::BuildRaytracingAccelerationStructureBatch(
    Util::Span<BvhBuilder> builders, // Batch of BVH builds
    Util::Span<BvhBuilder> updaters) // Batch of BVH updates
{
    if (builders.IsEmpty() && updaters.IsEmpty())
    {
        return;
    }

    RGP_PUSH_MARKER("InitAccelerationStructure");
    if (updaters.IsEmpty() == false)
    {
        RGP_PUSH_MARKER("Updates");
        DispatchInitAccelerationStructure<true>(updaters);
        RGP_POP_MARKER();
    }
    if (builders.IsEmpty() == false)
    {
        RGP_PUSH_MARKER("Builds");
        DispatchInitAccelerationStructure<false>(builders);
        RGP_POP_MARKER();
    }
    Barrier();
    RGP_POP_MARKER();

    if (builders.IsEmpty() == false)
    {
        RGP_PUSH_MARKER("Builds");

        // TODO: Disable per-bvh timestamp events for batched builds
        BuildPhase(builders, &BvhBuilder::PreBuildDumpEvents);

        BuildPhase("EncodePrimitives", builders, &BvhBuilder::EncodePrimitives);

        if (m_deviceSettings.enableParallelBuild)
        {
            BuildPhase("BuildParallel", builders, &BvhBuilder::BuildParallel);
        }
        else
        {
            RGP_PUSH_MARKER("BuildMultiDispatch");
            if constexpr (IsTlas)
            {
                // Build one TLAS at a time.
                // Workaround for crashes in cases where there are multiple TLAS and some are TopDown
                // Games usually have one TLAS per batch, so this shouldn't cause a significant performance impact
                for (size_t index = 0; index < builders.size(); ++index)
                {
                    BuildMultiDispatch(builders.Subspan(index, 1u));
                }
            }
            else
            {
                BuildMultiDispatch(builders);
            }
            RGP_POP_MARKER();
        }

        BuildPhase(builders, &BvhBuilder::PostBuildDumpEvents);

        RGP_POP_MARKER();
    }

    if (updaters.IsEmpty() == false)
    {
        RGP_PUSH_MARKER("Updates");

        BuildPhase("EncodePrimitives", updaters, &BvhBuilder::EncodePrimitives);

        BuildPhase("UpdateAccelerationStructure", updaters, &BvhBuilder::UpdateAccelerationStructure);

        RGP_POP_MARKER();
    }

    {
        RGP_PUSH_MARKER("EmitPostBuildInfo");

        BuildPhase("Updates", updaters, &BvhBuilder::EmitPostBuildInfo);
        BuildPhase("Builds", builders, &BvhBuilder::EmitPostBuildInfo);

        RGP_POP_MARKER();
    }
}

// =====================================================================================================================
// BVH build implementation with multiple separate compute dispatches for each phase.
// Each phase will execute for the entire batch, with one barrier in between each phase.
void BvhBatcher::BuildMultiDispatch(Util::Span<BvhBuilder> builders)
{
    if (builders.IsEmpty())
    {
        return;
    }

    // Wait for the encoding operation to complete.
    Barrier();

    // Rebraid the TLAS leaves
    BuildPhase("Rebraid", builders, &BvhBuilder::Rebraid);

    // Wait until Rebraid has finished
    Barrier();

    if (builders[0].m_buildConfig.topDownBuild)
    {
        BuildPhase("BuildBVHTD", builders, &BvhBuilder::BuildBVHTD);

        Barrier();

        BuildPhase("BuildQBVH", builders, &BvhBuilder::BuildQBVH);
        return;
    }

    // Generate morton codes from leaf nodes
    BuildPhase("GenerateMortonCodes", builders, &BvhBuilder::GenerateMortonCodes);

    // Wait for the morton code generation to complete
    Barrier();

    if (m_deviceSettings.enableMergeSort)
    {
        BuildPhase("MergeSort", builders, &BvhBuilder::MergeSort);
    }
    else
    {
        if (builders.size() == 1)
        {
            // Use legacy multidispatch radix sort for single builds
            BuildPhase("SortRadixInt32", builders, &BvhBuilder::SortRadixInt32);
        }
        else
        {
            RGP_PUSH_MARKER("RadixSort");
            RadixSort(builders);
            RGP_POP_MARKER();
        }
    }

    // Wait for the sorting to complete
    Barrier();

    BuildPhase("BuildLbvhOrSortLeaves", builders, &BvhBuilder::BuildLbvhOrSortLeaves);

    // Wait for the BVH build to complete
    Barrier();

    BuildPhase("BuildBvhPlocOrRefit", builders, &BvhBuilder::BuildBvhPlocOrRefit);

    Barrier();

    BuildPhase("PairCompression", builders, &BvhBuilder::PairCompression);

    Barrier();

    BuildPhase("BuildQBVH", builders, &BvhBuilder::BuildQBVH);

}

// =====================================================================================================================
// Initialize the builders' acceleration structure data into a valid begin state using a compute shader
template<bool IsUpdate>
void BvhBatcher::DispatchInitAccelerationStructure(
    Util::Span<BvhBuilder> builders)
{
    RGP_PUSH_MARKER("InitAccelerationStructure");

    const CompileTimeBuildSettings initAsBuildSettings
    {
        .enableBVHBuildDebugCounters = m_deviceSettings.enableBVHBuildDebugCounters,
    };

    Util::MetroHash::Hash hash = {};
    Util::MetroHash64::Hash(reinterpret_cast<const uint8*>(&initAsBuildSettings),
                            sizeof(initAsBuildSettings),
                            &hash.bytes[0]);

    const uint32 buildSettingsHash = Util::MetroHash::Compact32(&hash);

    const InternalRayTracingCsType initAsPipeline = IsUpdate ? InternalRayTracingCsType::InitUpdateAccelerationStructure :
                                                               InternalRayTracingCsType::InitAccelerationStructure;

    m_pDevice->BindPipeline(m_cmdBuffer,
                            initAsPipeline,
                            initAsBuildSettings,
                            buildSettingsHash);

    // Limit the number of builders per dispatch to avoid allocating excess embedded data
    static constexpr size_t MaxNumBuildersPerDispatch = 64;
    for (size_t offset = 0; offset < builders.size(); offset += MaxNumBuildersPerDispatch)
    {
        const uint32 numBuildersToDispatch = static_cast<uint32>(Util::Min(MaxNumBuildersPerDispatch,
                                                                           builders.size() - offset));
        const Util::Span<BvhBuilder> buildersToDispatch(builders.Data() + offset, numBuildersToDispatch);

        Util::Vector<BufferViewInfo, MaxNumBuildersPerDispatch, Internal::Device> constants(m_pDevice);
        Util::Vector<BufferViewInfo, MaxNumBuildersPerDispatch, Internal::Device> headerBuffers(m_pDevice);
        Util::Vector<BufferViewInfo, MaxNumBuildersPerDispatch, Internal::Device> scratchBuffers(m_pDevice);

        const InitAccelerationStructure::RootConstants rootConstants
        {
            numBuildersToDispatch,
        };

        uint32 entryOffset = 0;
        entryOffset = m_pDevice->WriteUserDataEntries(m_cmdBuffer,
                                                      &rootConstants,
                                                      InitAccelerationStructure::NumRootEntries,
                                                      entryOffset);
        for (auto& builder : buildersToDispatch)
        {
            scratchBuffers.EmplaceBack(BufferViewInfo
            {
                .gpuAddr = builder.ScratchBufferBaseVa(),
                .range = 0xFFFFFFFF,
            });

            if constexpr (IsUpdate == false)
            {
                headerBuffers.EmplaceBack(BufferViewInfo
                {
                    .gpuAddr = builder.HeaderBufferBaseVa(),
                    .range = 0xFFFFFFFF,
                });
                const InitAccelerationStructure::Constants shaderConstants =
                {
                    .numLeafNodes                         = builder.m_buildConfig.numLeafNodes,
                    .debugCountersScratchOffset           = builder.m_scratchOffsets.debugCounters,
                    .sceneBoundsScratchOffset             = builder.m_scratchOffsets.sceneBounds,
                    .numBatchesScratchOffset              = builder.m_scratchOffsets.numBatches,
                    .rebraidTaskQueueCounterScratchOffset = builder.m_scratchOffsets.rebraidTaskQueueCounter,
                    .tdTaskQueueCounterScratchOffset      = builder.m_scratchOffsets.tdTaskQueueCounter,
                    .plocTaskQueueCounterScratchOffset    = builder.m_scratchOffsets.plocTaskQueueCounter,
                    .encodeTaskCounterScratchOffset       = builder.m_scratchOffsets.encodeTaskCounter,
                    .taskLoopCountersOffset               = builder.m_scratchOffsets.taskLoopCounters,
                    .header                               = builder.InitAccelStructHeader(),
                    .metadataHeader                       = builder.InitAccelStructMetadataHeader(),
                };
                // Set constants CBV table
                constexpr uint32 AlignedSizeDw = Util::Pow2Align(InitAccelerationStructure::NumEntries, 4);

                gpusize constantsVa;
                void* pConstants = m_backend.AllocateEmbeddedData(m_cmdBuffer, AlignedSizeDw, 1, &constantsVa);

                memcpy(pConstants, &shaderConstants, sizeof(shaderConstants));

                constants.EmplaceBack(BufferViewInfo
                {
                    .gpuAddr = constantsVa,
                    .range = AlignedSizeDw * sizeof(uint32),
                    .stride = 16,
                });
            }
        }
        if constexpr (IsUpdate == false)
        {
            entryOffset = m_pDevice->WriteBufferSrdTable(m_cmdBuffer, constants.Data(), numBuildersToDispatch, false, entryOffset);
            entryOffset = m_pDevice->WriteBufferSrdTable(m_cmdBuffer, headerBuffers.Data(), numBuildersToDispatch, false, entryOffset);
        }
        entryOffset = m_pDevice->WriteBufferSrdTable(m_cmdBuffer, scratchBuffers.Data(), numBuildersToDispatch, false, entryOffset);

        m_backend.Dispatch(m_cmdBuffer, numBuildersToDispatch, 1, 1);
    }

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes all passes of RadixSort for each builder in the batch
void BvhBatcher::RadixSort(Util::Span<BvhBuilder> builders)
{
    const auto& radixSortConfig = builders[0].m_radixSortConfig;
    const uint32 numPasses = radixSortConfig.numPasses;
    uint32 bitShiftSize = 0;

    for (uint32 passIndex = 0; passIndex < numPasses; ++passIndex)
    {
        BuildFunction("BitHistogram", builders, [&](BvhBuilder& builder)
        {
            builder.BitHistogram(bitShiftSize, builder.m_buildConfig.numLeafNodes);
        });

        // Wait for the bit histogram to complete
        Barrier();

        // Force DLB for now
        BuildPhase("ScanExclusiveAddDLBInit", builders, &BvhBuilder::ScanExclusiveAddDLBInit);

        Barrier();

        BuildPhase("ScanExclusiveAddDLBScan", builders, &BvhBuilder::ScanExclusiveAddDLBScan);

        // Wait for the scan operation to complete
        Barrier();

        BuildFunction("ScatterKeysAndValues", builders, [&](BvhBuilder& builder)
        {
            builder.ScatterKeysAndValues(bitShiftSize, builder.m_buildConfig.numLeafNodes);
        });

        // If this isn't the last pass, then issue a barrier before we continue so the passes don't overlap.
        if ((passIndex + 1) < numPasses)
        {
            Barrier();
        }

        bitShiftSize += radixSortConfig.bitsPerPass;
    }
}

// =====================================================================================================================
// Performs a generic barrier that's used to synchronize internal ray tracing shaders
void BvhBatcher::Barrier()
{
    m_pDevice->RaytracingBarrier(m_cmdBuffer);
}

// =====================================================================================================================
// Applies the provided BatchBuilderFunc to all builders in builder span
template <typename BatchBuilderFunc>
void BvhBatcher::BuildFunction(
    const char*            rgpMarkerName,
    Util::Span<BvhBuilder> builders,
    BatchBuilderFunc       func)
{
    if (rgpMarkerName != nullptr)
    {
        RGP_PUSH_MARKER(rgpMarkerName);
    }
    for (auto& builder : builders)
    {
        func(builder);
    }
    if (rgpMarkerName != nullptr)
    {
        RGP_POP_MARKER();
    }
}

// =====================================================================================================================
// Invokes the builder phase function for each builder
template<typename BuilderPhase>
void BvhBatcher::BuildPhase(
    const char*            rgpMarkerName,
    Util::Span<BvhBuilder> builders,
    BuilderPhase           pBuilderPhase)
{
    if (rgpMarkerName != nullptr)
    {
        RGP_PUSH_MARKER(rgpMarkerName);
    }
    for (auto& builder : builders)
    {
        (builder.*pBuilderPhase)();
    }
    if (rgpMarkerName != nullptr)
    {
        RGP_POP_MARKER();
    }
}

// =====================================================================================================================
template<typename BuilderPhase>
void BvhBatcher::BuildPhase(
    Util::Span<BvhBuilder> builders,
    BuilderPhase           pBuilderPhase)
{
    BuildPhase(nullptr, builders, pBuilderPhase);
}

#if GPURT_DEVELOPER
// =====================================================================================================================
template<class ...Args>
void BvhBatcher::PushRGPMarker(
    const char* pFormat,
    Args&&...   args)
{
    m_pDevice->PushRGPMarker(m_cmdBuffer, pFormat, std::forward<Args>(args)...);
}

// =====================================================================================================================
void BvhBatcher::PopRGPMarker()
{
    m_pDevice->PopRGPMarker(m_cmdBuffer);
}

// =====================================================================================================================
void BvhBatcher::OutputPipelineName(InternalRayTracingCsType type)
{
    m_pDevice->OutputPipelineName(m_cmdBuffer, type);
}
#endif

}
