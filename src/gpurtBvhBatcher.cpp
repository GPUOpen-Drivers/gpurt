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

#include "palCmdBuffer.h"
#include "palHashMapImpl.h"
#include "palMetroHash.h"
#include "palVectorImpl.h"

#include "gpurt/gpurt.h"
#include "gpurt/gpurtLib.h"
#include "gpurt/gpurtAccelStruct.h"
#include "gpurt/gpurtInlineFuncs.h"
#include "gpurtInternal.h"
#include "gpurtInternalShaderBindings.h"
#include "gpurtBvhBatcher.h"
#include "gpurtBvhBuilder.h"
#include "gpurtBvhBuilderCommon.h"
#include "shared/accelStruct.h"

namespace GpuRt
{
// =====================================================================================================================
static bool PhaseFlagSet(BuildPhaseFlags src, BuildPhaseFlags test)
{
    return Util::TestAnyFlagSet(uint32(src), uint32(test));
}

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
        const AccelStructBuildInputs buildInputs = m_pDevice->OverrideBuildInputs(info.inputs);
        BvhBuilder builder(m_cmdBuffer,
                           m_backend,
                           m_pDevice,
                           m_deviceProps,
                           m_clientCb,
                           m_deviceSettings,
                           info);

        UpdateEnabledPhaseFlags(builder.EnabledPhases());

        const bool isUpdate = Util::TestAnyFlagSet(buildInputs.flags, AccelStructBuildFlagPerformUpdate);
        const bool isTlas = buildInputs.type == AccelStructType::TopLevel;
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
                {
                    blasBuilders.EmplaceBack(std::move(builder));
                }
            }
        }
    }

    if (m_deviceSettings.checkBufferOverlapsInBatch)
    {
        CheckOverlappingBuffers(buildInfos);
    }

    if (emptyBuilders.IsEmpty() == false)
    {
        RGP_PUSH_MARKER("Process Empty BVH builds");
        DispatchInitAccelerationStructure<false>(emptyBuilders);
        BuildPhase(emptyBuilders, &BvhBuilder::EmitPostBuildInfo);
        RGP_POP_MARKER();
    }

    if (
        (blasBuilders.IsEmpty() == false) ||
        (blasUpdaters.IsEmpty() == false))
    {
        RGP_PUSH_MARKER("BLAS: %u builds, %u updates", blasBuilders.size(), blasUpdaters.size());
        BuildRaytracingAccelerationStructureBatch<false>(
            blasBuilders,
            blasUpdaters);
        RGP_POP_MARKER();
    }
    if ((tlasBuilders.IsEmpty() == false) || (tlasUpdaters.IsEmpty() == false))
    {
        RGP_PUSH_MARKER("TLAS: %u builds, %u updates", tlasBuilders.size(), tlasUpdaters.size());
        BuildRaytracingAccelerationStructureBatch<true>(
            tlasBuilders,
            tlasUpdaters);
        RGP_POP_MARKER();
    }

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes each build phase for each builder within the batch
template <bool IsTlas>
void BvhBatcher::BuildRaytracingAccelerationStructureBatch(
    Util::Span<BvhBuilder> builders,        // Batch of BVH builds
    Util::Span<BvhBuilder> updaters)        // Batch of BVH updates
{
    if (
        builders.IsEmpty() && updaters.IsEmpty())
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
    if ((updaters.IsEmpty() == false) || (builders.IsEmpty() == false))
    {
        uint32 barrierFlags = BarrierFlagSyncDispatch;
        if (updaters.IsEmpty() == false)
        {
            // Updates can be launched with indirect dispatch. We need to avoid fetching the indirect arguments
            // from the header before they are written by a previous build/update/copy.
            barrierFlags |= BarrierFlagSyncIndirectArg;
        }
        Barrier(barrierFlags);
    }
    RGP_POP_MARKER();

    if (PhaseEnabled(BuildPhaseFlags::BuildDumpEvents))
    {
        // TODO: Disable per-bvh timestamp events for batched builds
        BuildPhase(builders, &BvhBuilder::PreBuildDumpEvents);
        BuildPhase(updaters, &BvhBuilder::PreBuildDumpEvents);
    }

    if (builders.IsEmpty() == false)
    {
        RGP_PUSH_MARKER("Builds");

        if (PhaseEnabled(BuildPhaseFlags::EncodeQuadPrimitives))
        {
            BuildPhase(BuildPhaseFlags::EncodeQuadPrimitives, builders, &BvhBuilder::EncodeQuadPrimitives);
        }

        if (PhaseEnabled(BuildPhaseFlags::EncodePrimitives))
        {
            BuildPhase(BuildPhaseFlags::EncodePrimitives, builders, &BvhBuilder::EncodePrimitives);
        }

        if (m_deviceSettings.enableParallelBuild)
        {
            BuildPhase(BuildPhaseFlags::BuildParallel, builders, &BvhBuilder::BuildParallel);
        }
        else
        {
            RGP_PUSH_MARKER("BuildMultiDispatch");
            BuildMultiDispatch(builders);
            RGP_POP_MARKER();
        }

        RGP_POP_MARKER();
    }

    if (updaters.IsEmpty() == false)
    {
        RGP_PUSH_MARKER("Updates");

        if (PhaseEnabled(BuildPhaseFlags::EncodePrimitives))
        {
            BuildPhase(BuildPhaseFlags::EncodePrimitives, updaters, &BvhBuilder::EncodePrimitives);
        }

        BuildPhase("UpdateAccelerationStructure", updaters, &BvhBuilder::UpdateAccelerationStructure);

        RGP_POP_MARKER();
    }

    if (PhaseEnabled(BuildPhaseFlags::SeparateEmitPostBuildInfoPass))
    {
        RGP_PUSH_MARKER("EmitPostBuildInfo");
        Barrier();
        BuildPhase("Updates", updaters, &BvhBuilder::EmitPostBuildInfo);
        BuildPhase("Builds", builders, &BvhBuilder::EmitPostBuildInfo);

        RGP_POP_MARKER();
    }
    else
    {
        // Execute EmitPostBuildInfo without any RGP markers
        BuildPhase(updaters, &BvhBuilder::EmitPostBuildInfo);
        BuildPhase(builders, &BvhBuilder::EmitPostBuildInfo);
    }

    if (PhaseEnabled(BuildPhaseFlags::BuildDumpEvents))
    {
        BuildPhase(builders, &BvhBuilder::PostBuildDumpEvents);
        BuildPhase(updaters, &BvhBuilder::PostBuildDumpEvents);
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

    if (PhaseEnabled(BuildPhaseFlags::Rebraid))
    {
        Barrier();
        BuildPhase(BuildPhaseFlags::Rebraid, builders, &BvhBuilder::Rebraid);
    }
    if (PhaseEnabled(BuildPhaseFlags::BuildBVHTD))
    {
        Barrier();
        BuildPhase(BuildPhaseFlags::BuildBVHTD, builders, &BvhBuilder::BuildBVHTD);
    }
    if (PhaseEnabled(BuildPhaseFlags::GenerateMortonCodes))
    {
        Barrier();
        BuildPhase(BuildPhaseFlags::GenerateMortonCodes, builders, &BvhBuilder::GenerateMortonCodes);
    }
    if (PhaseEnabled(BuildPhaseFlags::MergeSort))
    {
        Barrier();

        if (builders.size() > 1)
        {
            const uint32 wavesPerSimd = 2U;
            BuildFunction(BuildPhaseFlags::MergeSort, builders, [wavesPerSimd](BvhBuilder& builder)
            {
                builder.MergeSort(wavesPerSimd);
            });
        }
        else
        {
            RGP_PUSH_MARKER("Merge Sort");

            // Batch local sorts together.
            BuildPhase("Merge Sort (Local)", builders, &BvhBuilder::MergeSortLocal);

            Barrier();

            // Batch global sort iterations together. Compute max iterations amongst the builder batch
            uint32 maxMergeSortTreeLevel = 0;

            bool batchNeedsLastLevelCopy = false;

            for (const auto& builder : builders)
            {
                const uint32 mergeSortTreeLevel = builder.GetMaxMergeSortTreeLevel();
                maxMergeSortTreeLevel = Util::Max(maxMergeSortTreeLevel, mergeSortTreeLevel);
                batchNeedsLastLevelCopy |= ((mergeSortTreeLevel & 1) == 1);
            }

            if (maxMergeSortTreeLevel > 0)
            {
                RGP_PUSH_MARKER("Merge Sort (Global Iteration)");
                for (uint32 level = 1; level <= maxMergeSortTreeLevel; level++)
                {
                    Barrier();

                    BuildFunction(nullptr, builders, [level](BvhBuilder& builder)
                    {
                        if (level <= builder.GetMaxMergeSortTreeLevel())
                        {
                            builder.MergeSortGlobalIteration(level);
                        }
                    });
                }
                RGP_POP_MARKER();

                if (batchNeedsLastLevelCopy)
                {
                    Barrier();

                    RGP_PUSH_MARKER("Merge Sort (Copy Last Level)");
                    BuildFunction(nullptr, builders, [](BvhBuilder& builder)
                    {
                        const uint32 mergeSortTreeLevel = builder.GetMaxMergeSortTreeLevel();
                        if ((mergeSortTreeLevel & 1) == 1)
                        {
                            builder.MergeSortCopyLastLevel();
                        }
                    });
                    RGP_POP_MARKER();
                }
            }

            RGP_POP_MARKER();
        }
    }
    if (PhaseEnabled(BuildPhaseFlags::RadixSort))
    {
        Barrier();
        RGP_PUSH_MARKER("Radix Sort");
        RadixSort(builders);
        RGP_POP_MARKER();
    }
    if (PhaseEnabled(BuildPhaseFlags::BuildBVH))
    {
        Barrier();
        BuildPhase(BuildPhaseFlags::BuildBVH, builders, &BvhBuilder::BuildBVH);
    }
    if (PhaseEnabled(BuildPhaseFlags::BuildFastAgglomerativeLbvh))
    {
        Barrier();
        BuildPhase(BuildPhaseFlags::BuildFastAgglomerativeLbvh, builders, &BvhBuilder::BuildFastAgglomerativeLbvh);
    }
    if (PhaseEnabled(BuildPhaseFlags::BuildPLOC))
    {
        Barrier();
        const uint32 wavesPerSimd = builders.size() == 1 ? 8U : 1U;
        BuildFunction(BuildPhaseFlags::BuildPLOC, builders, [wavesPerSimd](BvhBuilder& builder)
        {
            builder.BuildPLOC(wavesPerSimd);
        });
    }
    if (PhaseEnabled(BuildPhaseFlags::RefitBounds))
    {
        Barrier();
        BuildPhase(BuildPhaseFlags::RefitBounds, builders, &BvhBuilder::RefitBounds);
    }
    if (PhaseEnabled(BuildPhaseFlags::PairCompression))
    {
        Barrier();
        BuildPhase(BuildPhaseFlags::PairCompression, builders, &BvhBuilder::PairCompression);
    }
    if (PhaseEnabled(BuildPhaseFlags::EncodeHwBvh))
    {
        Barrier();
        BuildPhase(BuildPhaseFlags::EncodeHwBvh, builders, &BvhBuilder::EncodeHwBvh);
    }
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

    constexpr uint32 AlignedConstantsSizeDwords = Util::Pow2Align(InitAccelerationStructure::NumEntries, 4);
    constexpr uint32 AlignedConstantsSizeBytes = AlignedConstantsSizeDwords * sizeof(uint32);

    gpusize constantsGpuVa;
    void* pConstantsBufferMap;

#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 46
    // Limit the number of builders per dispatch to avoid allocating excess embedded data
    static constexpr size_t DefaultNumBuildersPerDispatch = 64;
    for (size_t builderOffset = 0; builderOffset < builders.size(); builderOffset += DefaultNumBuildersPerDispatch)
    {
        const uint32 numBuildersToDispatch = static_cast<uint32>(Util::Min(DefaultNumBuildersPerDispatch,
                                                                           builders.size() - builderOffset));
        const Util::Span<BvhBuilder> buildersToDispatch(builders.Data() + builderOffset, numBuildersToDispatch);
#else
    static constexpr size_t DefaultNumBuildersPerDispatch = 128;

    {
        const uint32 builderOffset = 0;
        const uint32 numBuildersToDispatch = static_cast<uint32>(builders.size());
        const Util::Span<BvhBuilder> buildersToDispatch(builders.Data(), numBuildersToDispatch);

        const uint32 bufferSizeInBytes = numBuildersToDispatch * AlignedConstantsSizeBytes;
        pConstantsBufferMap = m_pDevice->AllocateTemporaryData(m_cmdBuffer, bufferSizeInBytes, &constantsGpuVa);
#endif

        Util::Vector<BufferViewInfo, DefaultNumBuildersPerDispatch, Internal::Device> constants(m_pDevice);
        Util::Vector<BufferViewInfo, DefaultNumBuildersPerDispatch, Internal::Device> headerBuffers(m_pDevice);
        Util::Vector<BufferViewInfo, DefaultNumBuildersPerDispatch, Internal::Device> scratchBuffers(m_pDevice);
        Util::Vector<BufferViewInfo, DefaultNumBuildersPerDispatch, Internal::Device> scratchGlobals(m_pDevice);

        for (size_t builderIdx = builderOffset; builderIdx < (builderOffset + numBuildersToDispatch); ++builderIdx)
        {
            auto& builder = builders[builderIdx];
            scratchBuffers.EmplaceBack(BufferViewInfo
            {
                .gpuAddr = builder.RemappedScratchBufferBaseVa(),
                .range = 0xFFFFFFFF,
            });

            scratchGlobals.EmplaceBack(BufferViewInfo
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
                // Early triangle pairing, triangle splitting and indirect BLAS builds dynamically increment
                // primitive reference counter. Initialise counters to 0 when these features are enabled.
                const bool dynamicallyIncrementsPrimRefCount = builder.m_buildConfig.enableEarlyPairCompression
                    || builder.m_buildConfig.triangleSplitting
                    || builder.m_buildSettings.isIndirectBuild;
                const InitAccelerationStructure::Constants shaderConstants =
                {
                    .maxNumPrimitives                     = builder.m_buildConfig.maxNumPrimitives,
                    .debugCountersScratchOffset           = builder.m_scratchOffsets.debugCounters,
                    .sceneBoundsScratchOffset             = builder.m_scratchOffsets.sceneBounds,
                    .numBatchesScratchOffset              = builder.m_scratchOffsets.numBatches,
                    .rebraidTaskQueueCounterScratchOffset = builder.m_scratchOffsets.rebraidTaskQueueCounter,
                    .tdTaskQueueCounterScratchOffset      = builder.m_scratchOffsets.tdTaskQueueCounter,
                    .plocTaskQueueCounterScratchOffset    = builder.m_scratchOffsets.plocTaskQueueCounter,
                    .encodeTaskCounterScratchOffset       = builder.m_scratchOffsets.encodeTaskCounter,
                    .taskLoopCountersOffset               = builder.m_scratchOffsets.taskLoopCounters,
                    .isIndirectBuild                      = builder.m_buildSettings.isIndirectBuild,
                    .dynamicallyIncrementsPrimRefCount    = dynamicallyIncrementsPrimRefCount ? 1u : 0u,
                    .header                               = builder.InitAccelStructHeader(),
                    .metadataHeader                       = builder.InitAccelStructMetadataHeader(),
                };
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 46
                const size_t currBuilderConstantsOffsetBytes = 0;
                pConstantsBufferMap =
                    m_pDevice->AllocateTemporaryData(m_cmdBuffer, AlignedConstantsSizeBytes, &constantsGpuVa);
#else
                const size_t currBuilderConstantsOffsetBytes = builderIdx * AlignedConstantsSizeBytes;
#endif
                void* pCurrBuilderConstants = Util::VoidPtrInc(pConstantsBufferMap, currBuilderConstantsOffsetBytes);
                memcpy(pCurrBuilderConstants, &shaderConstants, sizeof(shaderConstants));

                // Set constants CBV table
                constants.EmplaceBack(BufferViewInfo
                {
                    .gpuAddr = constantsGpuVa + currBuilderConstantsOffsetBytes,
                    .range   = AlignedConstantsSizeBytes,
                    .stride  = 16,
                });
            }
        }
        uint32 entryOffset = 0;

        entryOffset = m_pDevice->WriteBufferSrdTable(m_cmdBuffer, scratchGlobals.Data(), numBuildersToDispatch, false, entryOffset);
        if constexpr (IsUpdate == false)
        {
            entryOffset = m_pDevice->WriteBufferSrdTable(m_cmdBuffer, constants.Data(), numBuildersToDispatch, false, entryOffset);
            entryOffset = m_pDevice->WriteBufferSrdTable(m_cmdBuffer, headerBuffers.Data(), numBuildersToDispatch, false, entryOffset);
            entryOffset = m_pDevice->WriteBufferSrdTable(m_cmdBuffer, scratchBuffers.Data(), numBuildersToDispatch, false, entryOffset);
        }

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

    // DLB Init only needs to run once before ScanExclusiveAddDLBScan is dispatched
    BuildPhase("ScanExclusiveAddDLBInit", builders, &BvhBuilder::ScanExclusiveAddDLBInit);

    for (uint32 passIndex = 0; passIndex < numPasses; ++passIndex)
    {
        BuildFunction("BitHistogram", builders, [&](BvhBuilder& builder)
        {
            builder.BitHistogram(bitShiftSize, builder.m_buildConfig.maxNumPrimitives);
        });

        // Wait for the bit histogram to complete
        Barrier();

        BuildFunction("ScanExclusiveAddDLBScan", builders, [&](BvhBuilder& builder)
        {
            builder.ScanExclusiveAddDLBScan(passIndex);
        });

        // Wait for the scan operation to complete
        Barrier();

        BuildFunction("ScatterKeysAndValues", builders, [&](BvhBuilder& builder)
        {
            builder.ScatterKeysAndValues(bitShiftSize, builder.m_buildConfig.maxNumPrimitives);
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
void BvhBatcher::Barrier(
    uint32 flags)
{
    m_pDevice->RaytracingBarrier(m_cmdBuffer, flags);
}

// =====================================================================================================================
// Updates the enabled phases for the current batch using the flags provided by a BvhBuilder
void BvhBatcher::UpdateEnabledPhaseFlags(
    BuildPhaseFlags builderPhaseFlags)
{
    m_enabledPhaseFlags |= builderPhaseFlags;
}

// =====================================================================================================================
// Checks if the provided phase is enabled for the current batch
bool BvhBatcher::PhaseEnabled(BuildPhaseFlags phase)
{
    return PhaseFlagSet(m_enabledPhaseFlags, phase);
}

// =====================================================================================================================
// Helper to invoke the BatchBuilderFunc to a builder
template <typename BatchBuilderFunc>
void BvhBatcher::InvokeBuildFunction(
    BvhBuilder&      builder,
    BatchBuilderFunc func)
{
    func(builder);

    if (m_deviceSettings.enableInsertBarriersInBuildAS)
    {
        Barrier();
    }
}

// =====================================================================================================================
// Helper to invoke the BuildPhase of a builder
template<typename BuilderPhase>
void BvhBatcher::InvokeBuildPhase(
    BvhBuilder&  builder,
    BuilderPhase pBuilderPhase)
{
    (builder.*pBuilderPhase)();

    if (m_deviceSettings.enableInsertBarriersInBuildAS)
    {
        Barrier();
    }
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
        InvokeBuildFunction(builder, func);
    }
    if (rgpMarkerName != nullptr)
    {
        RGP_POP_MARKER();
    }
}

// =====================================================================================================================
// Applies the provided BatchBuilderFunc for each builder that enables the phase
template <typename BatchBuilderFunc>
void BvhBatcher::BuildFunction(
    BuildPhaseFlags        phase,
    Util::Span<BvhBuilder> builders,
    BatchBuilderFunc       func)
{
    RGP_PUSH_MARKER(BuildPhaseName(phase));

    for (auto& builder : builders)
    {
        if (PhaseFlagSet(builder.EnabledPhases(), phase))
        {
            InvokeBuildFunction(builder, func);
        }
    }

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Invokes the builder phase function for each builder that enables the phase
template<typename BuilderPhase>
void BvhBatcher::BuildPhase(
    BuildPhaseFlags        phase,
    Util::Span<BvhBuilder> builders,
    BuilderPhase           pBuilderPhase)
{
    RGP_PUSH_MARKER(BuildPhaseName(phase));

    for (auto& builder : builders)
    {
        if (PhaseFlagSet(builder.EnabledPhases(), phase))
        {
            InvokeBuildPhase(builder, pBuilderPhase);
        }
    }

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Invokes the builder phase function for each builder in the batch
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
        InvokeBuildPhase(builder, pBuilderPhase);
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
    for (auto& builder : builders)
    {
        InvokeBuildPhase(builder, pBuilderPhase);
    }
}

#if GPURT_DEVELOPER
// =====================================================================================================================
template<class ...Args>
void BvhBatcher::PushRGPMarker(
    const char* pFormat,
    Args&&...   args)
{
    if (Util::TestAnyFlagSet(uint32(m_deviceSettings.rgpMarkerGranularityFlags),
                             uint32(RgpMarkerGranularityFlags::PerBatch)))
    {
        m_pDevice->PushRGPMarker(m_cmdBuffer, pFormat, std::forward<Args>(args)...);
    }
}

// =====================================================================================================================
void BvhBatcher::PopRGPMarker()
{
    if (Util::TestAnyFlagSet(uint32(m_deviceSettings.rgpMarkerGranularityFlags),
                             uint32(RgpMarkerGranularityFlags::PerBatch)))
    {
        m_pDevice->PopRGPMarker(m_cmdBuffer);
    }
}

// =====================================================================================================================
void BvhBatcher::OutputPipelineName(InternalRayTracingCsType type)
{
    m_pDevice->OutputPipelineName(m_cmdBuffer, type);
}
#endif

// =====================================================================================================================
void BvhBatcher::CheckOverlappingBuffers(
    Util::Span<const AccelStructBuildInfo> buildInfos)
{
    struct BufferRange
    {
        gpusize addrStart;
        gpusize addrEnd;
    };
    struct AsBufferRanges
    {
        BufferRange scratch;
        BufferRange result;
    };
    const auto CalculateAsBufferRanges = [&](const AccelStructBuildInfo& buildInfo)
    {
        AsBufferRanges          ranges       = {};
        AccelStructPrebuildInfo prebuildInfo = {};
        m_pDevice->GetAccelStructPrebuildInfo(buildInfo.inputs, &prebuildInfo);

        const bool isUpdate            = Util::TestAnyFlagSet(buildInfo.inputs.flags, AccelStructBuildFlagPerformUpdate);
        const size_t scratchBuffersize = isUpdate ? prebuildInfo.updateScratchDataSizeInBytes :
                                                    prebuildInfo.scratchDataSizeInBytes;

        ranges.scratch.addrStart = buildInfo.scratchAddr.gpu;
        ranges.scratch.addrEnd   = ranges.scratch.addrStart + scratchBuffersize;

        ranges.result.addrStart = buildInfo.dstAccelStructGpuAddr;
        ranges.result.addrEnd   = ranges.result.addrStart + prebuildInfo.resultDataMaxSizeInBytes;
        return ranges;
    };

    const auto IsRangeOverlapping = [&](const BufferRange& lhs, const BufferRange& rhs)
    {
        return ((lhs.addrStart < rhs.addrEnd) && (rhs.addrStart < lhs.addrEnd));
    };

    for (size_t currentIndex = 0; currentIndex < buildInfos.size(); ++currentIndex)
    {
        const AsBufferRanges currentRanges = CalculateAsBufferRanges(buildInfos[currentIndex]);
        for (size_t otherIndex = currentIndex + 1; otherIndex < buildInfos.size(); ++otherIndex)
        {
            const AsBufferRanges otherRanges = CalculateAsBufferRanges(buildInfos[otherIndex]);

            PAL_ASSERT_MSG(IsRangeOverlapping(currentRanges.scratch, otherRanges.scratch) == false,
                           "Found Scratch Buffers Overlapping in batch. "
                           "Build %d 0x%llx-0x%llx, Build %d 0x%llx-0x%llx",
                           currentIndex, currentRanges.scratch.addrStart, currentRanges.scratch.addrEnd,
                           otherIndex, otherRanges.scratch.addrStart, otherRanges.scratch.addrEnd);

            PAL_ASSERT_MSG(IsRangeOverlapping(currentRanges.result, otherRanges.result) == false,
                           "Found Result Buffers Overlapping in batch. "
                           "Build %d 0x%llx-0x%llx, Build %d 0x%llx-0x%llx",
                           currentIndex, currentRanges.result.addrStart, currentRanges.result.addrEnd,
                           otherIndex, otherRanges.result.addrStart, otherRanges.result.addrEnd);

            PAL_ASSERT_MSG(IsRangeOverlapping(currentRanges.scratch, otherRanges.result) == false,
                           "Found Scratch-Result Buffers Overlapping in batch. "
                           "Build %d 0x%llx-0x%llx, Build %d 0x%llx-0x%llx",
                           currentIndex, currentRanges.scratch.addrStart, currentRanges.scratch.addrEnd,
                           otherIndex, otherRanges.result.addrStart, otherRanges.result.addrEnd);

            PAL_ASSERT_MSG(IsRangeOverlapping(currentRanges.result, otherRanges.scratch) == false,
                           "Found Result-Scratch Buffers Overlapping in batch. "
                           "Build %d 0x%llx-0x%llx, Build %d 0x%llx-0x%llx",
                           currentIndex, currentRanges.result.addrStart, currentRanges.result.addrEnd,
                           otherIndex, otherRanges.scratch.addrStart, otherRanges.scratch.addrEnd);
        }
    }
}
}
