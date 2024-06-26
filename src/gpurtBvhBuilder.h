/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "gpurt/gpurtBackend.h"

namespace GpuRt
{
enum class BuildPhaseFlags : uint32;

namespace EncodeNodes
{
struct Constants;
}

// =====================================================================================================================
// Helper class used by GPURT to perform various BVH operations like building, copying, etc.
class BvhBuilder
{
    friend class BvhBatcher;
public:
    // Constructor for the ray tracing bvh builder class that will perform a build/update
    explicit BvhBuilder(ClientCmdBufferHandle        cmdBuffer,
                        const IBackend&              backend,
                        Internal::Device*      const pDevice,
                        const Pal::DeviceProperties& deviceProps,
                        ClientCallbacks              clientCb,
                        const DeviceSettings&        deviceSettings,
                        const AccelStructBuildInfo&  info);

    // Constructor for the ray tracing bvh builder class that will perform a copy or emit build info
    explicit BvhBuilder(ClientCmdBufferHandle        cmdBuffer,
                        const IBackend&              backend,
                        Internal::Device*      const pDevice,
                        const Pal::DeviceProperties& deviceProps,
                        ClientCallbacks              clientCb,
                        const DeviceSettings&        deviceSettings);

    // Destructor for the ray tracing bvh builder class
    ~BvhBuilder();

    // Helper function to determine internal nodes size
    uint32 CalculateInternalNodesSize();

    // Helper function to determine leaf nodes size
    uint32 CalculateLeafNodesSize();

    // Helper function to determine nodes size
    uint32 CalculateNodesSize();

    // Helper function to determine geometry info size
    static uint32 CalculateGeometryInfoSize(
        uint32 numGeometryDescs);

    // Builds or updates an acceleration structure and stores it in a result buffer
    void BuildRaytracingAccelerationStructure();

    // Handles pre-build dump event invocation
    void PreBuildDumpEvents();

    // Handles post-build dump event invocation
    void PostBuildDumpEvents();

    void InitializeBuildConfigs();

    void EncodePrimitives();
    void EncodeQuadPrimitives();

    // data offset and size in ResultBuffer
    struct ResultBufferInfo
    {
        uint32 baseOffset;
        uint32 nodeSize;
        uint32 dataSize;
    };

    // data offset and size in ScratchBuffer
    struct ScratchBufferInfo
    {
        uint32 baseOffset;
        uint32 bvh2PhaseSize;
        uint32 qbvhPhaseSize;
    };

    bool AllowRemappingScratchBuffer() const;

    gpusize RemappedScratchBufferBaseVa() const;

    uint32 CalculateScratchBufferSize(
        const ResultBufferInfo& resultBufferInfo,
        const ScratchBufferInfo& scratchBufferInfo);

    ResultBufferInfo CalculateResultBufferInfo(
        AccelStructDataOffsets* pOffsets,
        uint32* pMetadataSizeInBytes);

    ScratchBufferInfo CalculateScratchBufferInfo(
        RayTracingScratchDataOffsets* pOffsets);

    uint32 CalculateUpdateScratchBufferInfo(
        RayTracingScratchDataOffsets* pOffsets);

    // Encodes instance nodes into a scratch buffer
    void EncodeInstances(
        uint32             numDesc,
        InputElementLayout descLayout);

    void GetAccelerationStructurePrebuildInfo(
        const AccelStructBuildInputs& buildInfo,
        AccelStructPrebuildInfo*      pPrebuildInfo);

    // Performs a barrier that synchronizes any active BVH builder operations.
    void Barrier(uint32 flags = BarrierFlagSyncDispatch);

    // Emit postbuild info
    void EmitAccelerationStructurePostBuildInfo(
        const AccelStructPostBuildInfo& postBuildInfo);

    void EmitASCurrentSize(
        const AccelStructPostBuildInfo& postBuildInfo);

    void EmitASCompactedType(
        const AccelStructPostBuildInfo& postBuildInfo);

    void EmitASToolsVisualizationType(
        const AccelStructPostBuildInfo& postBuildInfo);

    void EmitASSerializationType(
        const AccelStructPostBuildInfo& postBuildInfo);

    // Copy Acceleration Structure
    void CopyAccelerationStructure(
        const AccelStructCopyInfo& copyArgs);

    void CopyASCloneMode(
        const AccelStructCopyInfo& copyArgs);

    void CopyASCompactMode(
        const AccelStructCopyInfo& copyArgs);

    void CopyASSerializeMode(
        const AccelStructCopyInfo& copyArgs);

    void CopyASDeserializeMode(
        const AccelStructCopyInfo& copyArgs);

    void CopyASToolsVisualizationMode(
        const AccelStructCopyInfo& copyArgs);

    BuildPhaseFlags EnabledPhases() const;

private:

    // Configs that change within build calls, private to the bvh builder.
    struct BuildConfig
    {
        BvhBuildMode                    buildMode;
        BvhCpuBuildMode                 cpuBuildMode;

        uint32                          numPrimitives;
        uint32                          maxNumPrimitives;

        // All function calls requiring Geometry/RebraidType should pass it in instead. For now leave this.
        GeometryType                    geometryType;
        RebraidType                     rebraidType;
        TriangleCompressionMode         triangleCompressionMode;      // Triangle compression modes.
        Fp16BoxNodesInBlasMode          fp16BoxNodesInBlasMode;       // Mode for which interior nodes in BLAS are FP16

        // Radix sort info
        uint32                          numHistogramElements;
        uint32                          radixSortScanLevel;

        uint32                          numMortonSizeBits;

        uint32                          trianglePairBlockCount;       // For early pair compression
        SceneBoundsCalculation          sceneCalcType;
        bool                            topLevelBuild;
        bool                            triangleSplitting;            // Triangle Splitting Enabled
        bool                            allowTopDownBuild;            // Is accel structure top level
                                                                      // and has top down enabled or rebraid
        // Top down build in TLAS (topDownAllowed && prim count is not larger than max count)
        bool                            topDownBuild;
        bool                            needEncodeDispatch;
        bool                            enableEarlyPairCompression;
        bool                            enableFastLBVH;
        bool                            enableMergeSort;
    };

    BvhBuilder(
        Internal::Device*    const   pDevice,
        const Pal::DeviceProperties& deviceProps,
        ClientCallbacks              clientCb,
        const DeviceSettings&        deviceSettings);

    uint32 GetLeafNodeSize(
        const DeviceSettings& settings, const BuildConfig& config);

    static uint32 GetNumHistogramElements(
        const RadixSortConfig& config,
        uint32                 primitiveCount);

    void InitBuildConfig(
        const AccelStructBuildInfo& buildArgs);

    BuildShaderConstants GetBuildShaderConstants() const;
    void AllocateBuildShaderConstants(
        const BuildShaderConstants& buildShaderConstants);

    void InitGeometryConstants();

    GeometryType GetGeometryType(
        const AccelStructBuildInputs inputs);

    // Returns true if this BVH update should be treated as a build instead.
    bool ForceRebuild() const
    {
        return false;
    }

    bool UpdateAllowed() const
    {
        return Util::TestAnyFlagSet(m_buildArgs.inputs.flags, AccelStructBuildFlagAllowUpdate);
    }

    bool IsUpdate() const
    {
        return Util::TestAnyFlagSet(m_buildArgs.inputs.flags, AccelStructBuildFlagPerformUpdate)
            && (ForceRebuild() == false);
    }

    gpusize HeaderBufferBaseVa() const
    {
        return m_buildArgs.dstAccelStructGpuAddr;
    }

    gpusize SourceHeaderBufferBaseVa() const
    {
        return IsUpdate() ? m_buildArgs.srcAccelStructGpuAddr : m_buildArgs.dstAccelStructGpuAddr;
    }

    gpusize ResultBufferBaseVa() const
    {
        PAL_ASSERT(IsUpdate() == false);
        return m_buildArgs.dstAccelStructGpuAddr + m_metadataSizeInBytes;
    }

    gpusize SourceBufferBaseVa() const
    {
        PAL_ASSERT(IsUpdate() == false);
        return SourceHeaderBufferBaseVa() + m_metadataSizeInBytes;
    }

    bool IsUpdateInPlace() const
    {
        return m_buildArgs.srcAccelStructGpuAddr == m_buildArgs.dstAccelStructGpuAddr;
    }

    AccelStructMetadataHeader InitAccelStructMetadataHeader();

    AccelStructHeader InitAccelStructHeader() const;

    BvhBuildMode OverrideBuildMode(
        const AccelStructBuildInfo& buildInfo);

    // BVH construction shader functions

    void InitAccelerationStructure();

    // Builds a new acceleration structure and stores it in a result buffer
    void BuildAccelerationStructure();

    // Updates an existing acceleration structure and stores it in a result buffer
    void UpdateAccelerationStructure();

    void EmitPostBuildInfo();

    void EncodeUpdate();

    uint32 GetParallelBuildNumThreadGroups();

    void BuildParallel();

    void BuildTrivialBvh();

    void Rebraid();

    void GenerateMortonCodes();

    void DispatchBuildBVHPipeline(
        InternalRayTracingCsType pipeline);

    void BuildBVH();

    void BuildBVHTD();

    void BuildBVHPLOC(uint32 wavesPerSimd);

    void BuildFastAgglomerativeLbvh();

    void UpdateQBVH();

    void UpdateParallel();

    void EncodeHwBvh();

    void RefitBounds();

    void PairCompression();

    void ClearBuffer(
        gpusize bufferVa,
        uint32 numDwords,
        uint32 clearValue);

    void CopyBufferRaw(
        gpusize dstBufferVa,
        gpusize srcBufferVa,
        uint32  numDwords);

    void BitHistogram(
        uint32 bitShiftSize,
        uint32 numElems);

    void ScatterKeysAndValues(
        uint32 bitShiftSize,
        uint32 numElems);

    void MergeSort(uint32 wavesPerSimd);
    void SortRadixInt32();

    void ScanExclusiveAdd(
        uint32 numElems);

    void ScanExclusiveAddOneLevel(
        uint32 inOutArrayOffset,
        uint32 numElems,
        uint32 numWorkGroups);

    void ScanExclusiveAddPartial(
        uint32 inOutArrayOffset,
        uint32 partSumsOffset,
        uint32 numElems,
        uint32 numWorkGroups);
    void ScanExclusiveDistributeSums(
        uint32 inOutArrayOffset,
        uint32 partSumsOffset,
        uint32 numElems,
        uint32 numWorkGroups);

    void ScanExclusiveAddTwoLevel(
        uint32 numElems);
    void ScanExclusiveAddThreeLevel(
        uint32 numElems);

    void ScanExclusiveAddDLB(
        uint32 numElems);

    void ScanExclusiveAddDLBInit();
    void ScanExclusiveAddDLBScan(uint32 passIdx);

    // Optional phase checks
    bool AllowRebraid() const;
    bool AllowLatePairCompression() const;
    bool NeedsPostBuildEmitPass() const;
    bool HasBuildDumpEvents() const;

    // Helper functions
    void Dispatch(
        uint32 numGroups);

    void DispatchIndirect(
        gpusize indirectArgumentAddr);

    void BindPipeline(InternalRayTracingCsType type);

    uint32 WriteUserDataEntries(
        const void* pEntries,
        uint32      numEntries,
        uint32      entryOffset);

    uint32 WriteBufferVa(
        gpusize virtualAddress,
        uint32  entryOffset);

    uint32 WriteBufferBindings(uint32 entryOffset, uint32 geometryIndex = 0);
    uint32 WriteUpdateBuffers(uint32 entryOffset);

    uint32 WriteBuildShaderConstantBuffer(uint32 entryOffset);

    uint32 NumPrimitivesAfterSplit(
        uint32 primitiveCount,
        float  splitFactor);

    gpusize ScratchBufferBaseVa() const
    {
        return m_buildArgs.scratchAddr.gpu;
    }

    uint32 GetNumThreadGroupsCopy()
    {
        return m_backend.GetOptimalNumThreadGroups(DefaultThreadGroupSize);
    }

    uint32 GetNumPersistentThreadGroups(
        uint32 numWorkItems,
        uint32 threadGroupSize = DefaultThreadGroupSize,
        uint32 wavesPerSimd = 1)
    {
        return Util::Min(m_backend.GetOptimalNumThreadGroups(threadGroupSize) * wavesPerSimd,
                         Util::RoundUpQuotient(numWorkItems, threadGroupSize));
    }

    template<typename T>
    void WriteImmediateData(
        gpusize  destVa,
        const T& data);

    void ZeroDataImmediate(
        gpusize destVa,
        uint32  dwordCount)
    {
        WriteOrZeroDataImmediate(destVa, nullptr, dwordCount);
    }

    void WriteOrZeroDataImmediate(
        gpusize       destVa,
        const uint32* pData,
        uint32        dwordCount);

    void WriteImmediateSingle(
        gpusize            destVa,
        uint64             value,
        ImmediateDataWidth width);

    void ResetTaskCounter(
        gpusize metadataHeaderGpuVa);

    void ResetTaskQueueCounters(
        uint32 offset);

    BufferViewInfo SetupVertexBuffer(
        const GeometryTriangles& desc,
        uint32* pStride,
        uint32* pVertexCompCount) const;

    BufferViewInfo SetupAabbBuffer(
        const GeometryAabbs& desc,
        uint32* pStride) const;

    uint32 GetLeafNodeExpansion() const;

    uint32 GetNumInternalNodeCount() const;

#if GPURT_DEVELOPER
    // Driver generated RGP markers are only added in internal builds because they expose details about the
    // construction of acceleration structure.
    template <class... Args>
    void PushRGPMarker(const char* pFormat, Args&&... args);
    void PopRGPMarker();

    void OutputBuildInfo();
    void OutputPipelineName(InternalRayTracingCsType type);
#endif
    void InitBuildSettings();
    void InitCopySettings();

    const char* ConvertBuildModeToString();
    const char* ConvertRebraidTypeToString();
    const char* ConvertTriCompressionTypeToString();
    const char* ConvertFp16ModeToString();

    Internal::Device*           const m_pDevice;             // GPURT device
    ClientCallbacks                   m_clientCb;            // Function Cb table
    const DeviceSettings&             m_deviceSettings;      // Device settings
    BuildConfig                       m_buildConfig;         // Build info on the accel struct
    AccelStructDataOffsets            m_resultOffsets;       // Result offsets for the build
    AccelStructBuildInfo              m_buildArgs;           // Accel struct build arguments
    const Pal::DeviceProperties&      m_deviceProps;         // PAL device properties
    uint32                            m_metadataSizeInBytes; // Metadata size in bytes
    ClientCmdBufferHandle             m_cmdBuffer;           // Associated command buffer handle
    RayTracingScratchDataOffsets      m_scratchOffsets;      // Scratch offsets for the build
    const IBackend&                   m_backend;             // Backend interface
    CompileTimeBuildSettings          m_buildSettings;
    gpusize                           m_shaderConstantsGpuVa{};
    gpusize                           m_geomConstSrdTable{};
    gpusize                           m_geomBufferSrdTable{};
    const RadixSortConfig             m_radixSortConfig;
    uint64                            m_emitCompactDstGpuVa;
    uint32                            m_buildSettingsHash;
    ResultBufferInfo                  m_resultBufferInfo;
    ScratchBufferInfo                 m_scratchBufferInfo;
    AccelStructInfo                   m_dumpInfo;
};

};
