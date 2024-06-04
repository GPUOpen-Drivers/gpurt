/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2019-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "gpurt/gpurtAccelStruct.h"
#include "gpurt/gpurtBackend.h"
#include "gpurt/gpurtBuildSettings.h"
#include "gpurt/gpurtCounter.h"
#include "shared/gpurtBuildConstants.h"
#include "shared/rayTracingDefs.h"
#include "gpurtDebugMonitor.h"

#include "palInlineFuncs.h"
#include "palHashLiteralString.h"

#include <unordered_map>

// __declspec(dllexport) has limitations. "No name decoration is applied to exported C functions or C++ extern "C"
// functions using the __cdecl calling convention." In order to export a unmangled symbol for a function that uses a
// different calling convention you must invoke the linker directly on MSVC. NOTE: Name mangling doesn't occur for
// exported C functions on 64-bit platforms.
//
// Place this inside the function body you want to export. And ensure the name of the function is the symbol name
// you want to export
#define GPURT_EXPORT_UNMANGLED_SYMBOL_MSVC

namespace GpuRt
{

// =====================================================================================================================
// Constants needed for raytracing

// Sizes of the node structures used by the BVH building shaders

static constexpr size_t RayTracingQBVHLeafSize          = 64;
static constexpr size_t RayTracingScratchNodeSize       = 64;
static constexpr size_t RayTracingStatePLOCSize         = 16;   // PLOCState size without ploc task counters
                                                                // taskCounter size is tracked by
                                                                // RayTracingTaskQueueCounterSize
static constexpr size_t RayTracingBlasParentPtrsSize    = 4;
static constexpr size_t RayTracingPLOCFlags             = 8 * 4;

static constexpr size_t RayTracingScanDLBFlagsSize      = 8 * 2;

static constexpr size_t RayTracingTDRefScratchSize      = 48;
static constexpr size_t RayTracingTDNodeSize            = 32;
static constexpr size_t RayTracingStateTDBuildSize      = 56;
static constexpr size_t RayTracingTDBinsSize            = 360;

static constexpr size_t RayTracingTDTRRefScratchSize    = 52;
static constexpr size_t RayTracingTDTRNodeSize          = 48;
static constexpr size_t RayTracingStateTDTRBuildSize    = 64;

static constexpr size_t RayTracingTSRefScratchSize      = 36;
static constexpr size_t RayTracingStateTSBuildSize      = 20;
static constexpr size_t RayTracingAtomicFlags           = 8;

static constexpr size_t RayTracingStateRebraidBuildSize = 20;

static constexpr size_t RayTracingBuildDebugCounters    = 11;

static constexpr size_t RayTracingTaskQueueCounterSize  = 20;
static constexpr size_t RayTracingTaskQueueCounters     = 5;

static_assert(((RayTracingTDNodeSize % 8) == 0),
    "TDNode size must be 8-byte aligned to ensure that 64-bit atomic operations on the first field work correctly.");

constexpr uint32 DefaultThreadGroupSize = 64;

// =====================================================================================================================
// Constant string hashes for compiler options
namespace PipelineOptionName
{
constexpr uint32_t waveSize = Util::HashLiteralString("waveSize");
constexpr uint32_t Wave32   = Util::HashLiteralString("Wave32");
constexpr uint32_t Wave64   = Util::HashLiteralString("Wave64");
}

enum EncodeFlags : uint32
{
    EncodeFlagArrayOfPointers       = 0x00000001,
    EncodeFlagUpdateInPlace         = 0x00000002,
    EncodeFlagRebraidEnabled        = 0x00000004,
    EncodeFlagFusedInstanceNode     = 0x00000008,
};

struct RadixSortConfig
{
    uint32 workGroupSize;
    uint32 keysPerThread;
    uint32 bitsPerPass;
    uint32 numBins;
    uint32 numPasses30;
    uint32 numPasses;
    uint32 blocksPerGroup;
    uint32 numScanElemsPerWorkItem;
    uint32 numScanElemsPerWorkGroup;
    uint32 groupBlockSize;
    uint32 scanThresholdOneLevel;
    uint32 scanThresholdTwoLevel;
    uint32 scanThresholdThreeLevel;
    uint32 groupBlockSizeScan;
    uint32 groupBlockSizeDistribute;
};

// =====================================================================================================================
// Calculate radix sort configuration constants for the provided build configuration.
static RadixSortConfig GetRadixSortConfig(
    const DeviceSettings& deviceSettings)
{
    RadixSortConfig config = {};

    const uint32 threadGroupSize = DefaultThreadGroupSize;

    config.workGroupSize            = threadGroupSize;
    config.keysPerThread            = 4;
    config.bitsPerPass              = 4;
    config.numBins                  = 1 << config.bitsPerPass;
    config.numPasses30              = 32 / config.bitsPerPass;
    config.numPasses                = 64 / config.bitsPerPass;
    config.blocksPerGroup           = 1;
    config.numScanElemsPerWorkItem  = 8;
    config.numScanElemsPerWorkGroup = config.workGroupSize * config.numScanElemsPerWorkItem;
    config.groupBlockSize           = config.workGroupSize * config.bitsPerPass * config.blocksPerGroup;
    config.scanThresholdOneLevel    = config.numScanElemsPerWorkGroup;
    config.scanThresholdTwoLevel    = config.scanThresholdOneLevel * config.numScanElemsPerWorkGroup;
    config.scanThresholdThreeLevel  = config.scanThresholdTwoLevel * config.numScanElemsPerWorkGroup;
    config.groupBlockSizeScan       = config.workGroupSize << 3;
    config.groupBlockSizeDistribute = config.workGroupSize << 2;

    return config;
}

#pragma pack(push, 4)
struct RayHistoryMetadata
{
    RayHistoryMetadataInfo   counterInfo;
    CounterInfo              counter;
    RayHistoryMetadataInfo   traversalFlagsInfo;
    RayHistoryTraversalFlags traversalFlags;
};
#pragma pack(pop)

struct RayHistoryTraceListInfo
{
    RayHistoryRdfChunkHeader    rayHistoryRdfChunkHeader;
    void*                       pRayHistoryTraceBuffer;

    uint32                      shaderTableRayGenOffset;
    ShaderTableInfo             shaderTableRayGenInfo;

    uint32                      shaderTableMissOffset;
    ShaderTableInfo             shaderTableMissInfo;

    uint32                      shaderTableHitGroupOffset;
    ShaderTableInfo             shaderTableHitGroupInfo;

    uint32                      shaderTableCallableOffset;
    ShaderTableInfo             shaderTableCallableInfo;

    ClientGpuMemHandle          traceBufferGpuMem;

    CounterInfo                 counterInfo;
    RayHistoryTraversalFlags    traversalFlags;

    IndirectCounterMetadata*    pIndirectCounterMetadata;
};

typedef Util::Vector<RayHistoryTraceListInfo, 8, Internal::Device> RayHistoryBufferList;

// Starting logical ID value for all logical IDs used by internal pipelines.
const uint32 ReservedLogicalIdCount = 1;

// Array of internal pipeline source code
extern const PipelineBuildInfo InternalPipelineBuildInfo[size_t(InternalRayTracingCsType::Count)];

// Map key for map of internal pipelines
struct InternalPipelineKey
{
    InternalRayTracingCsType shaderType;
    uint32                   settingsHash;

    bool operator==(const InternalPipelineKey& other) const
    {
        return (shaderType   == other.shaderType)
            && (settingsHash == other.settingsHash);
    }
};

struct InternalPipelineKeyHasher
{
    std::size_t operator()(const InternalPipelineKey& key) const
    {
        return static_cast<std::size_t>((key.settingsHash << 5) + static_cast<uint32>(key.shaderType));
    }
};

struct InternalPipelineMemoryPair
{
    ClientPipelineHandle pPipeline;
    void*                pMemory;
};

using InternalPipelineMap = std::unordered_map<InternalPipelineKey,
                                               InternalPipelineMemoryPair,
                                               InternalPipelineKeyHasher>;

//=====================================================================================================================
// different ways to encode the scene bounds used to generate morton codes
enum class SceneBoundsCalculation : uint32
{
    BasedOnGeometry = 0,
    BasedOnGeometryWithSize
};

namespace Internal {

// =====================================================================================================================
// Create GPURT device
//
Pal::Result GPURT_API_ENTRY CreateDevice(
    const DeviceInitInfo&  info,
    const ClientCallbacks& callbacks,
    const IBackend*        pBackend,
    void*            const pMemory,
    IDevice**        const ppDevice);

// =====================================================================================================================
size_t GPURT_API_ENTRY GetDeviceSize();

// =====================================================================================================================
// Get GPURT shader library data
//
// @param flags [in] Feature flags to enable in GPURT library
//
// @return Shader code for the shader library
//
PipelineShaderCode GPURT_API_ENTRY GetShaderLibraryCode(
    ShaderLibraryFeatureFlags flags);

// =====================================================================================================================
// Returns GPURT shader library function table for input ray tracing IP level.
//
// @param rayTracingIpLevel   [in]  Pal IP level
// @param pEntryFunctionTable [out] Requested function table, if found
//
// @return whether the function table was found successfully
Pal::Result GPURT_API_ENTRY QueryRayTracingEntryFunctionTable(
    const Pal::RayTracingIpLevel   rayTracingIpLevel,
    EntryFunctionTable* const      pEntryFunctionTable);

// =====================================================================================================================
class Device : public IDevice
{
public:

    Device(const DeviceInitInfo& info, const ClientCallbacks& clientCb, const IBackend* pBackend);

    // Destroy device
    //
    virtual void Destroy() override;

    // Initializes the device
    //
    // @returns Initialization success status
    virtual Pal::Result Init() override;

    // Allocate system memory
    //
    // @param [in] allocInfo Contains information about the requested allocation.
    void* Alloc(
        const Util::AllocInfo& allocInfo);

    // Free system memory
    //
    // @param [in] freeInfo Contains information about the requested free.
    void Free(
        const Util::FreeInfo& freeInfo);

    // Returns GPURT shader library function table for input ray tracing IP level.
    //
    // @param rayTracingIpLevel   [in]  Pal IP level
    // @param pEntryFunctionTable [out] Requested function table, if found
    //
    // @return whether the function table was found successfully
    virtual Pal::Result QueryRayTracingEntryFunctionTable(
        const Pal::RayTracingIpLevel   rayTracingIpLevel,
        EntryFunctionTable* const      pEntryFunctionTable) override;

    // Determines required memory allocation size for use by ray continuation stacks
    //
    // Note: this method is non-trivial, i.e. it returns a different number than cpsThreadStackBytes * numThreads, that
    // encompasses other memory requirements as well.
    //
    // @param cpsThreadStackBytes (in) Maximum CPS stack size per thread, in bytes.
    // @param numThreads          (in) Number of Threads.
    //
    // @return the required global memory allocation size in bytes
    virtual gpusize GetCpsMemoryBytes(gpusize cpsThreadStackBytes, uint32 numThreads) override
    {
        const gpusize stackTotalBytes = numThreads * cpsThreadStackBytes;
        gpusize requestedMemoryBytes = stackTotalBytes;

        {
            // Bin headers must be 8-byte aligned due to atomic 64-bit operations, and come at the end of the memory.
            // Ensure the end boundary of the memory is itself 8-byte aligned:
            requestedMemoryBytes = Util::Pow2Align(requestedMemoryBytes, sizeof(uint64));
        }

        return requestedMemoryBytes;
    }

    // Populates the GPU addresses in the DispatchRaysConstants structure
    //
    // @param pDispatchRaysConstants  (in/out) Non-null pointer to a DispatchRaysConstants
    // @param cpsMemoryGpuAddr        (in) GPU address pointing to the beginning of cps memory
    // @param cpsMemoryBytes          (in) Cps allocated memory size in bytes
    //
    // @return the required global memory allocation size in bytes
    virtual void PatchDispatchRaysConstants(
        DispatchRaysConstants* pDispatchRaysConstants,
        const gpusize          cpsMemoryGpuAddr,
        const gpusize          cpsMemoryBytes) override;

    //
    // @param cpsVideoMem          [in] Cps video memory
    // @param cpsMemoryBytes       [in] Cps allocated memory size in bytes
    //
    // @return whether the memory initialization was uploaded successfully
    virtual Pal::Result InitializeCpsMemory(
        const Pal::IGpuMemory& cpsVideoMem,
        const gpusize          cpsMemoryBytes) override;

    // Returns the static pipeline mask shader constant values for a particular pipeline given a compatible
    // AS memory layout parameters.
    //
    // @param skipTriangles             (in) Force skip all triangle intersections
    // @param skipProceduralPrims       (in) Force skip all procedural AABB intersections
    // @param enableAccelStructTracking (in) Enable AccelStruct tracking
    // @param enableTraversalCounter    (in) Enable Traversal Counter
    //
    // @return Static pipeline mask literal
    //
    // This is the value that drivers must return back to the shader by implementing the AmdTraceRayGetStaticFlags()
    // driver stub.
    virtual uint32 GetStaticPipelineFlags(
        bool  skipTriangles,
        bool  skipProceduralPrims,
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 37
        bool  unused0,
#endif
        bool  enableAccelStructTracking,
        bool  enableTraversalCounter) override;

    // Writes commands into a command buffer to build an acceleration structure
    //
    // @param cmdBuffer            [in] Opaque handle to command buffer where commands will be written
    // @param buildInfo            [in] Acceleration structure build info
    virtual void BuildAccelStruct(
        ClientCmdBufferHandle         cmdBuffer,
        const AccelStructBuildInfo&   buildInfo
    ) override;

    // Writes commands into a command buffer to build multiple acceleration structures
    //
    // @param cmdBuffer            [in] Opaque handle to command buffer where commands will be written
    // @param buildInfo            [in] Acceleration structure build info
    virtual void BuildAccelStructs(
        ClientCmdBufferHandle                  cmdBuffer,
        Util::Span<const AccelStructBuildInfo> buildInfo
    ) override;

    // Writes commands into a command buffer to emit post-build information about an acceleration structure
    //
    // @param cmdBuffer            [in] Opaque handle to command buffer where commands will be written
    // @param postBuildInfo        [in] Post-build event info
    virtual void EmitAccelStructPostBuildInfo(
        ClientCmdBufferHandle           cmdBuffer,
        const AccelStructPostBuildInfo& postBuildInfo
    ) override;

    // Writes commands into a command buffer to execute an acceleration structure copy/update/compress operation
    //
    // @param cmdBuffer            [in] Opaque handle to command buffer where commands will be written
    // @param copyInfo             [in] Copy operation info
    virtual void CopyAccelStruct(
        ClientCmdBufferHandle         cmdBuffer,
        const AccelStructCopyInfo&    copyInfo
    ) override;

    // Prepares the input buffer (indirect arguments, bindings, constants) for an indirect raytracing dispatch
    //
    // @param cmdBuffer            [in/out] Opaque handle to ommand buffer where commands will be written
    // @param userData             [in] Addresses of input/output buffers
    // @param maxDispatchCount     Max indirect dispatches
    // @param pipelineCount        Number of pipelines to dispatch
    virtual void InitExecuteIndirect(
        ClientCmdBufferHandle              cmdBuffer,
        const InitExecuteIndirectUserData& userData,
        uint32                             maxDispatchCount,
        uint32                             pipelineCount)
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 40
        const
#endif
        override;

    // Calculates and returns prebuild information about some given future acceleration structure.
    //
    // @param inputs          [in]  Struct describing the inputs for building an acceleration structure
    // @param pPrebuildInfo   [out] Resulting prebuild information
    virtual void GetAccelStructPrebuildInfo(
        const AccelStructBuildInputs& inputs,
        AccelStructPrebuildInfo*      pPrebuildInfo) override;

    // Decodes the acceleration structure memory and returns information about the built acceleration structure.
    // @param pAccelStructData [in]  CPU pointer to acceleration structure data
    // @param pAccelStructInfo [out] Result acceleration structure info
    virtual void GetAccelStructInfo(
        const void*            pAccelStructData,
        AccelStructInfo* const pAccelStructInfo) override;

    // Parses acceleration structure header to read acceleration structure UUID
    // @param pData        [in]  Serialized acceleration structure pointer
    // @param pVersion     [out] Result UUID
    virtual void GetSerializedAccelStructVersion(
        const void*     pData,
        uint64_t*       pVersion) override;

    // Check serialized acceleration structure GUID and version
    // @param identifier [in] Serialized acceleration structure identifier
    // Returns the matching status
    virtual DataDriverMatchingIdentifierStatus CheckSerializedAccelStructVersion(
        const DataDriverMatchingIdentifier* identifier) override;

    // Returns true if the acceleration structure trace source is currently enabled.
    virtual bool AccelStructTraceEnabled() const override
    {
        return m_accelStructTraceSource.Enabled();
    }

    // Returns the GPUVA of the AccelStructTracker
    gpusize AccelStructTrackerGpuAddr() const { return m_info.accelStructTrackerGpuAddr; }

    static uint32 NumPrimitivesAfterSplit(uint32 primitiveCount, float splitFactor);

    void BeginBvhTrace();

    void EndBvhTrace();

    void WriteCapturedBvh(
        GpuUtil::ITraceSource* pTraceSource);

    void WriteAccelStructChunk(
        GpuUtil::ITraceSource* pTraceSource,
        gpusize                gpuVa,
        uint32                 isBlas,
        const void*            pData,
        size_t                 dataSize);

    void NotifyTlasBuild(gpusize address);

    // Returns true if the ray history trace source is currently available for use.
    virtual bool RayHistoryTraceAvailable() const override
    {
        return m_rayHistoryTraceSource.Available();
    }

    // Returns true if a trace is active. If true, the client should call TraceRtDispatch() for each direct dispatch
    // and TraceIndirectRtDispatch() for each potential indirect dispatch.
    virtual bool RayHistoryTraceActive() const override
    {
        return m_rayHistoryTraceSource.Active();
    }

    // Notifies GPURT about an RT dispatch, triggers a trace buffer allocation, and outputs a buffer view for the trace
    // buffer.
    virtual void TraceRtDispatch(
        ClientCmdBufferHandle           cmdBuffer,
        RtPipelineType                  pipelineType,
        RtDispatchInfo                  dispatchInfo,
        DispatchRaysConstants*          pConstants) override;

    // Notifies GPURT about an indirect RT dispatch similar to TraceRtDispatch(). Multiple disaptches may occur in
    // one invocation, and the max count must be provided. At most MaxSupportedIndirectCounters will be traced.
    //
    // @param pCounterMetadataVa [out] GPUVA pointing to an array of maxDispatchCount IndirectCounterMetadata structs.
    //                                 This GPUVA must be used to initialize InitExecuteIndirectUserData.outputCounterMetaVa.
    //
    // @param pIndirectConstants [out] InitExecuteIndirectConstants for RT pipelines and DispatchRaysConstants otherwise.
    virtual void TraceIndirectRtDispatch(
        RtPipelineType                type,
        RtDispatchInfo                dispatchInfo,
        uint32                        maxDispatchCount,
        gpusize*                      pCounterMetadataVa,
        void*                         pIndirectConstants) override;

    virtual const PipelineBuildInfo& GetPipelineBuildInfo(
        InternalRayTracingCsType shaderType) const override
    {
        return InternalPipelineBuildInfo[static_cast<uint32>(shaderType)];
    }

    void AddMetadataToList(
        RtDispatchInfo              dispatchInfo,
        RtPipelineType              pipelineType,
        RayHistoryTraceListInfo*    pTraceListInfo);

    // Fill shader table info and copy the application's shader table data
    //
    // @param cmdBuffer           [in]  Opaque handle to command buffer
    // @param runningOffsetGpuVa  [in]  Offset GpuVa for shader table copy to
    // @param destGpuVa           [in]  Base GpuVa for shader table
    // @param runningOffset       [in]  Offset for read back shader table info
    // @param pMappedData         [in]  Data of trace and shader table memory allocation
    // @param shaderTableType     [in]  RayGen=0, Miss=1, HitGroup=2, Callable=3
    // @param shaderTable         [in]  Shader table from dispatch arguments
    // @param pOutShaderTableInfo [out] Output Shader table with user data filled in
    void WriteShaderTableData(
        ClientCmdBufferHandle  cmdBuffer,
        uint64                 runningOffsetGpuVa,
        gpusize                destGpuVa,
        uint64                 runningOffset,
        void*                  pMappedData,
        ShaderTableType        shaderTableType,
        ShaderTable            shaderTable,
        ShaderTableInfo*       pOutShaderTableInfo);

    void WriteRayHistoryChunks(
        GpuUtil::ITraceSource* pTraceSource);

    void WriteRayHistoryMetaDataChunks(
        const RayHistoryTraceListInfo& traceListInfo,
        GpuUtil::ITraceSource*         pTraceSource);

    void WriteShaderTableChunks(
        const RayHistoryTraceListInfo& traceListInfo,
        GpuUtil::ITraceSource*         pTraceSource);

    uint64 GetRayHistoryBufferSizeInBytes() { return m_rayHistoryTraceSource.GetBufferSizeInBytes(); }

    uint32 GetRayHistoryLightCounterMask() const;

    ClientPipelineHandle GetInternalPipeline(
        InternalRayTracingCsType        type,
        const CompileTimeBuildSettings& buildSettings,
        uint32                          buildSettingsHash);

    void BindPipeline(
        ClientCmdBufferHandle           cmdBuffer,
        InternalRayTracingCsType        type,
        const CompileTimeBuildSettings& buildSettings,
        uint32                          buildSettingsHash);

    void* AllocateTemporaryData(
        ClientCmdBufferHandle cmdBuffer,
        gpusize               sizeInBytes,
        gpusize*              pGpuAddress) const;

    void* AllocateDescriptorTable(
        ClientCmdBufferHandle cmdBuffer,
        uint32                count,
        gpusize*              pGpuAddress) const;

    uint32 WriteBufferSrdTable(
        ClientCmdBufferHandle cmdBuffer,
        const BufferViewInfo* pBufferViews,
        uint32                count,
        bool                  typedBuffer,
        uint32                entryOffset) const;

    // Computes size for decoded acceleration structure
    //
    // @param type              Type of post build info
    // @param pGpuVas     [in]  Gpu virtual address
    // @param count             Number of elements
    // @param sizeInByte  [out] Post build size of accelerate structure
    //
    // @return Post build size success status
    Pal::Result GetAccelStructPostBuildSize(
        AccelStructPostBuildInfoType type,
        const gpusize*               pGpuVas,
        uint32                       count,
        uint64*                      pSizeInBytes);

    // Executes the copy buffer shader
    //
    // @param cmdBuffer      [in] Opaque handle to command buffer for command submission
    // @param dstBufferVa         Destination buffer GPU VA
    // @param srcBufferVa         Source buffer GPU VA
    // @param numDwords           Number of Dwords to copy
    void CopyBufferRaw(
        ClientCmdBufferHandle  cmdBuffer,
        gpusize                dstBufferVa,
        gpusize                srcBufferVa,
        uint32                 numDwords);

    // Uploads CPU memory to a GPU buffer
    //
    // @param cmdBuffer      [in] Opaque handle to command buffer for command submission
    // @param dstBufferVa         Destination buffer GPU VA
    // @param pSrcData            Pointer to the CPU memory to upload
    // @param sizeInBytes         Size of the CPU memory to upload, in bytes. Must be less than or equal to
    //                            the size reported by pCmdBuffer->GetEmbeddedDataLimit();
    void UploadCpuMemory(
        ClientCmdBufferHandle  cmdBuffer,
        gpusize                dstBufferVa,
        const void*            pSrcData,
        uint32                 sizeInBytes);

    // Writes the provided entries into the compute shader user data slots
    //
    // @param pCmdBuffer       [in] Command buffer where commands will be written
    // @param pEntries              User data entries
    // @param numEntries            Number of entries
    // @param entryOffset           Offset of the first entry
    //
    // @return Data entry
    uint32 WriteUserDataEntries(
        ClientCmdBufferHandle cmdBuffer,
        const void*           pEntries,
        uint32                numEntries,
        uint32                entryOffset) const;

    // Writes a gpu virtual address for a buffer into the compute shader user data slots
    //
    // @param pCmdBuffer       [in] Command buffer where commands will be written
    // @param virtualAddress        GPUVA of the buffer
    // @param entryOffset           Offset of the first entry
    //
    // @return Data entry
    uint32 WriteBufferVa(
        ClientCmdBufferHandle cmdBuffer,
        gpusize               virtualAddress,
        uint32                entryOffset);

    // Binds an internal pipeline
    //
    // @param cmdBuffer           Opaque handle to command buffer to use for the bind command
    // @param shaderType          Type of internal shader to bind
    // @param buildSettings       Current settings for BVH builds
    // @param buildSettingsHash   Metro hash of current build settings
    // @param apiPsoHash          Calculated internal API PSO hash
    void BindPipeline(
        ClientCmdBufferHandle           cmdBuffer,
        InternalRayTracingCsType        shaderType,
        const CompileTimeBuildSettings& buildSettings,
        uint32                          buildSettingsHash,
        uint32                          apiPsoHash);

    // Performs a generic barrier that's used to synchronize internal ray tracing shaders
    //
    // @param pCmdBuffer       [in] Command buffer where commands will be written
    // @param flags            Barrier control flags (GpuRt::BarrierFlags)
    void RaytracingBarrier(
        ClientCmdBufferHandle pCmdBuffer,
        uint32                flags);

    Pal::IDevice* GetPalDevice() const { return m_info.pPalDevice; };

    virtual const ClientCallbacks& GetClientCallbacks() const override { return m_clientCb; }
    virtual const DeviceInitInfo& GetInitInfo() const override { return m_info; }

    // Returns size in DWORDs of a buffer view SRD
    uint32 GetBufferSrdSizeDw() const { return m_bufferSrdSizeDw; };

    Pal::RayTracingIpLevel GetRtIpLevel() const { return m_rtIpLevel; }

    const DeviceSettings& Settings() const { return m_info.deviceSettings; }

#if GPURT_ENABLE_GPU_DEBUG
    gpusize GetGpuDebugBufferVa() const
    {
        return m_debugMonitor.DebugBufferVa();
    }
#endif

#if GPURT_DEVELOPER
    // Driver generated RGP markers are only added in internal builds because they expose details about the
    // construction of acceleration structure.
    void PushRGPMarker(ClientCmdBufferHandle cmdBuffer, const char* pFormat, ...);
    void PopRGPMarker(ClientCmdBufferHandle cmdBuffer);

    void OutputPipelineName(
        ClientCmdBufferHandle cmdBuffer,
        InternalRayTracingCsType type) const;
#endif

private:

    void SetUpClientCallbacks(
        ClientCallbacks* pClientCb);

    virtual ~Device() override;

    DeviceInitInfo  m_info;

    Util::GenericAllocatorTracked            m_allocator;
    Util::RWLock                             m_internalPipelineLock;
    InternalPipelineMap                      m_pipelineMap;
    Util::Vector<gpusize, 8, Device>         m_tlasCaptureList;
    Util::Mutex                              m_traceBvhLock;
    bool                                     m_isTraceActive;
    GpuRt::AccelStructTraceSource            m_accelStructTraceSource;
    uint32                                   m_bufferSrdSizeDw;
    ClientCallbacks                          m_clientCb;
    Pal::RayTracingIpLevel                   m_rtIpLevel;           // the actual RTIP level GPURT is using,
                                                                    // is based on emulatedRtIpLevel and the actual device.

    GpuRt::RayHistoryTraceSource             m_rayHistoryTraceSource;
    RayHistoryBufferList                     m_rayHistoryTraceList;
    Util::Mutex                              m_traceRayHistoryLock;
    const IBackend*                          m_pBackend;

#if GPURT_ENABLE_GPU_DEBUG
    GpuRt::DebugMonitor                      m_debugMonitor;
#endif
};
}  // namespace Internal
} // namespace GpuRt
