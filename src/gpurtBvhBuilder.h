/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2022 Advanced Micro Devices, Inc. All Rights Reserved.
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
enum class ImmediateDataWidth : uint32;
}

namespace GpuRt
{

using Pal::gpusize;

// =====================================================================================================================
// Helper class used by GPURT to perform various BVH operations like building, copying, etc.
class BvhBuilder
{
public:
    virtual ~BvhBuilder();

    // Helper function to determine buffer size
    uint32 CalculateResultBufferInfo(
        AccelStructDataOffsets* pOffsets,
        uint32*                 pMetadataSizeInBytes);

    // Helper function to determine internal nodes size
    uint32 CalculateInternalNodesSize();

    // Helper function to determine leaf nodes size
    uint32 CalculateLeafNodesSize();

    // Helper function to determine nodes size
    uint32 CalculateNodesSize();

    // Helper function to determine Metadata size
    uint32 CalcMetadataSizeInBytes(
        uint32 internalNodeSize,
        uint32 leafNodeSize);

    // Helper function to determine geometry info size
    static uint32 CalculateGeometryInfoSize(
        uint32 numGeometryDescs);

    // Helper function to determine index buffer info size
    static uint32 CalculateIndexBufferInfoSize(
        uint32 numGeometryDescs);

    // Builds or updates an acceleration structure and stores it in a result buffer
    virtual void BuildRaytracingAccelerationStructure(
        const AccelStructBuildInfo& buildInfo) = 0;

protected:

    // Configs that change within build calls, private to the bvh builder.
    struct BuildConfig
    {
        BvhBuildMode                    buildMode;
        BvhCpuBuildMode                 cpuBuildMode;

        uint32                          numPrimitives;
        uint32                          numLeafNodes;

        // All function calls requiring Geometry/RebraidType should pass it in instead. For now leave this.
        GeometryType                    geometryType;
        RebraidType                     rebraidType;
        TriangleCompressionMode         triangleCompressionMode;      // Triangle compression modes.
        Fp16BoxNodesInBlasMode          fp16BoxNodesInBlasMode;       // Mode for which interior nodes in BLAS are FP16

        // Radix sort info
        uint32                          numHistogramElements;
        uint32                          radixSortScanLevel;

        uint32                          numMortonSizeBits;

        // bvhBuilder NodeSort Heuristics
        BvhBuilderNodeSortType          bvhBuilderNodeSortType;
        BvhBuilderNodeSortHeuristic     bvhBuilderNodeSortHeuristic;
        bool                            topLevelBuild;
        bool                            triangleSplitting;            // Triangle Splitting Enabled
        bool                            collapse;                     // Collapse Enabled
        bool                            allowTopDownBuild;            // Is accel structure top level
                                                                      // and has top down enabled or rebraid
        // Top down build in TLAS (topDownAllowed && prim count is not larger than max count)
        bool                            topDownBuild;
    };

    BvhBuilder(
        Device*                      pDevice,
        const Pal::DeviceProperties& deviceProps,
        const DeviceSettings&        deviceSettings);

    static uint32 GetLeafNodeSize(
        bool topLevelBuild);

    static uint32 GetNumHistogramElements(
        const RadixSortConfig& config,
        uint32                 primitiveCount);

    void InitBuildConfig(
        const AccelStructBuildInfo& buildArgs);

    void UpdateBuildConfig();

    bool UpdateAllowed() const
    {
        return Util::TestAnyFlagSet(m_buildArgs.inputs.flags, AccelStructBuildFlagAllowUpdate);
    }

    bool IsUpdate() const
    {
        return Util::TestAnyFlagSet(m_buildArgs.inputs.flags, AccelStructBuildFlagPerformUpdate);
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

    Device*                           m_pDevice;             // GPURT device
    const DeviceSettings&             m_deviceSettings;      // Device settings
    BuildConfig                       m_buildConfig;         // Build info on the accel struct
    AccelStructDataOffsets            m_resultOffsets;       // Result offsets for the build
    AccelStructBuildInfo              m_buildArgs;           // Accel struct build arguments
    const Pal::DeviceProperties&      m_deviceProps;         // PAL device properties
    uint32                            m_metadataSizeInBytes; // Metadata size in bytes
};

// =====================================================================================================================
// Helper class used by GPURT to perform various BVH operations like building, copying, etc.
class GpuBvhBuilder : public BvhBuilder
{
public:
    // Constructor for the ray tracing bvh builder class
    explicit GpuBvhBuilder(Pal::ICmdBuffer*             pCmdBuf,
                           Device*                      pDevice,
                           const Pal::DeviceProperties& deviceProps,
                           const DeviceSettings&        deviceSettings);

    // Destructor for the ray tracing bvh builder class
    ~GpuBvhBuilder();

    uint32 CalculateScratchBufferInfo(
        RayTracingScratchDataOffsets* pOffsets);

    uint32 CalculateUpdateScratchBufferInfo(
        RayTracingScratchDataOffsets* pOffsets);

    // Encodes triangle nodes into a scratch buffer
    void EncodeTriangleNodes(
        uint32                                             primitiveOffset,
        const GeometryTriangles*                           pDesc,
        uint32                                             primitiveCount,
        uint32                                             geometryIndex,
        GeometryFlags                                      geometryFlags,
        uint64                                             resultLeafOffset,
        gpusize                                            indirectGpuVa);

    // Encodes AABB nodes into a scratch buffer
    void EncodeAABBNodes(
        uint32                                         primitiveOffset,
        const GeometryAabbs*                           pDesc,
        uint32                                         primitiveCount,
        uint32                                         geometryIndex,
        GeometryFlags                                  geometryFlags,
        uint64                                         resultLeafOffset);

    // Encodes instance nodes into a scratch buffer
    void EncodeInstances(
        gpusize            instanceDescVa,
        uint32             numDesc,
        InputElementLayout descLayout);

    void GetAccelerationStructurePrebuildInfo(
        const AccelStructBuildInputs& buildInfo,
        AccelStructPrebuildInfo*      pPrebuildInfo);

    // Builds or updates an acceleration structure and stores it in a result buffer
    void BuildRaytracingAccelerationStructure(
        const AccelStructBuildInfo& buildInfo) override;

    // Performs a barrier that synchronizes any active BVH builder operations.
    void Barrier();

    // Emit postbuild info
    void EmitAccelerationStructurePostBuildInfo(
        const AccelStructPostBuildInfo& postBuildInfo);

    void EmitASCurrentType(
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

private:

    void InitBuildConfig(
        const AccelStructBuildInfo& buildArgs);

    BvhBuildMode OverrideBuildMode(
        const AccelStructBuildInfo& buildInfo);

    // BVH construction shader functions

    void InitAccelerationStructure(
        uint32 accelStructSize);

    // Builds a new acceleration structure and stores it in a result buffer
    void BuildAccelerationStructure();

    // Updates an existing acceleration structure and stores it in a result buffer
    void UpdateAccelerationStructure();

    void GenerateMortonCodes();

    void BuildBVH();

    void BuildBVHTD();

    void BuildBVHPLOC();

    void UpdateQBVH();

    void UpdateParallel();

    void BuildQBVH();

    void BuildQBVHTop();

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
        uint32 inputArrayOffset,
        uint32 outputArrayOffset,
        uint32 bitShiftSize,
        uint32 numElems);

    void ScatterKeysAndValues(
        uint32 inputKeysOffset,
        uint32 inputValuesOffset,
        uint32 histogramsOffset,
        uint32 outputKeysOffset,
        uint32 outputValuesOffset,
        uint32 bitShiftSize,
        uint32 numElems);

    void MergeSort();
    void SortRadixInt32();
    void BuildParallel();

    void ScanExclusiveAdd(
        uint32 inOutArrayOffset,
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
        uint32 inOutArrayOffset,
        uint32 numElems);
    void ScanExclusiveAddThreeLevel(
        uint32 inOutArrayOffset,
        uint32 numElems);

    void ScanExclusiveAddDLB(
        uint32 inOutArrayOffset,
        uint32 numElems);

    // Helper functions

    void BindPipeline(InternalRayTracingCsType type);

    uint32 WriteUserDataEntries(
        const void* pEntries,
        uint32      numEntries,
        uint32      entryOffset);

    uint32 WriteBufferVa(
        gpusize virtualAddress,
        uint32  entryOffset);

    uint32 WriteDestBuffers(uint32 entryOffset);

    uint32 NumPrimitivesAfterSplit(
        uint32 primitiveCount,
        float  splitFactor);

    gpusize ScratchBufferBaseVa() const
    {
        return m_buildArgs.scratchAddr.gpu;
    }

    uint32 GetOptimalNumThreadGroups(uint32 threadGroupSize);

    uint32 GetNumThreadGroupsCopy()
    {
        return GetOptimalNumThreadGroups(DefaultThreadGroupSize);
    }

    uint32 GetNumPersistentThreadGroups(
        uint32 numWorkItems,
        uint32 threadGroupSize = DefaultThreadGroupSize,
        uint32 wavesPerSimd = 1)
    {
        return Util::Min(GetOptimalNumThreadGroups(threadGroupSize) * wavesPerSimd,
                         Util::RoundUpQuotient(numWorkItems, threadGroupSize));
    }

    uint32 BuildModeFlags();

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
        gpusize                 destVa,
        uint64                  value,
        Pal::ImmediateDataWidth width);

    void ResetTaskCounter(
        gpusize metadataHeaderGpuVa);

    void ResetTaskQueueCounters(
        uint32 offset);

#if GPURT_DEVELOPER
    // Driver generated RGP markers are only added in internal builds because they expose details about the
    // construction of acceleration structure.
    void PushRGPMarker(const char* pFormat, ...);
    void PopRGPMarker();

    void OutputBuildInfo();
    void OutputPipelineName(InternalRayTracingCsType type);
#endif
    void InitBuildSettings();

    const char* ConvertBuildModeToString();
    const char* ConvertRebraidTypeToString();
    const char* ConvertTriCompressionTypeToString();
    const char* ConvertFp16ModeToString();

    Pal::ICmdBuffer*                  m_pPalCmdBuffer;      // The associated PAL cmdbuffer
    RayTracingScratchDataOffsets      m_scratchOffsets;     // Scratch offsets for the build
    CompileTimeBuildSettings          m_buildSettings;
    const RadixSortConfig             m_radixSortConfig;
    uint64                            m_emitCompactDstGpuVa;
    uint32                            m_buildSettingsHash;
};

};