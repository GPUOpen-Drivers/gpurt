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

#include "palInlineFuncs.h"
#include "palCmdBuffer.h"
#include "palMetroHash.h"
#include "palFormatInfo.h"
#include "palAutoBuffer.h"

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
#include "shared/rayTracingDefs.h"

#include <vector>

#include <float.h>

#define GPURT_ARRAY_SIZE(x) ((sizeof(x))/sizeof((x)[0]))

namespace GpuRt
{

static std::atomic<uint32> g_buildCounter;

struct VertexFormatInfo
{
    Pal::ChNumFormat numFormat;           // Native corresponding PAL format
    Pal::ChNumFormat singleCompNumFormat; // Single component format used when the buffer is not aligned
    uint32           validChannels;       // Number of valid channels
};

const Pal::ChannelMapping SingleChannelMapping =
    { Pal::ChannelSwizzle::X, Pal::ChannelSwizzle::Zero, Pal::ChannelSwizzle::Zero, Pal::ChannelSwizzle::Zero };

const Pal::ChannelMapping TwoChannelMapping =
    { Pal::ChannelSwizzle::X, Pal::ChannelSwizzle::Y, Pal::ChannelSwizzle::Zero, Pal::ChannelSwizzle::Zero };

const Pal::ChannelMapping ThreeChannelMapping =
    { Pal::ChannelSwizzle::X, Pal::ChannelSwizzle::Y, Pal::ChannelSwizzle::Z, Pal::ChannelSwizzle::Zero };

static VertexFormatInfo VertexFormatInfoTable[] =
{
    // Invalid
    {},
    // R32G32B32_Float
    { Pal::ChNumFormat::X32Y32Z32_Float,    Pal::ChNumFormat::X32_Float, 3 },
    // R32G32_Float
    { Pal::ChNumFormat::X32Y32_Float,       Pal::ChNumFormat::X32_Float, 2 },
    // R16G16B16A16_Float
    { Pal::ChNumFormat::X16Y16Z16W16_Float, Pal::ChNumFormat::X16_Float, 3 },
    // R16G16_Float
    { Pal::ChNumFormat::X16Y16_Float,       Pal::ChNumFormat::X16_Float, 2 },
    // R16G16B16A16_Snorm
    { Pal::ChNumFormat::X16Y16Z16W16_Snorm, Pal::ChNumFormat::X16_Snorm, 3 },
    // R16G16_Snorm
    { Pal::ChNumFormat::X16Y16_Snorm,       Pal::ChNumFormat::X16_Snorm, 2 },
    // R16G16B16A16_Unorm
    { Pal::ChNumFormat::X16Y16Z16W16_Unorm, Pal::ChNumFormat::X16_Unorm, 3 },
    // R16G16_Unorm
    { Pal::ChNumFormat::X16Y16_Unorm,       Pal::ChNumFormat::X16_Unorm, 2},
    // R10G10B10A2_Unorm
    { Pal::ChNumFormat::X10Y10Z10W2_Unorm,  Pal::ChNumFormat::Undefined, 3 },
    // R8G8B8A8_Snorm
    { Pal::ChNumFormat::X8Y8Z8W8_Snorm,     Pal::ChNumFormat::X8_Snorm, 3 },
    // R8G8_Snorm
    { Pal::ChNumFormat::X8Y8_Snorm,         Pal::ChNumFormat::X8_Snorm, 2 },
    // R8G8B8A8_Unorm
    { Pal::ChNumFormat::X8Y8Z8W8_Unorm,     Pal::ChNumFormat::X8_Unorm, 3 },
    // R8G8_Unorm
    { Pal::ChNumFormat::X8Y8_Unorm,         Pal::ChNumFormat::X8_Unorm, 2 },
};

// =====================================================================================================================
// Helper structure for encapsulating triangle index buffer information
struct IndexBufferInfo
{
    uint32 format;
    uint64 byteOffset;
    uint64 gpuVa;
};

// =====================================================================================================================
// Helper function to convert triangle geometry information into index buffer info
IndexBufferInfo GetIndexBufferInfo(
    const GeometryTriangles& geometry)
{
    IndexBufferInfo indexBuffer = {};
    indexBuffer.gpuVa = geometry.indexBufferAddr.gpu;
    indexBuffer.format = 0;
    indexBuffer.byteOffset = 0;

    // Set index format for valid indices
    if (indexBuffer.gpuVa != 0)
    {
        PAL_ASSERT(geometry.indexFormat != IndexFormat::Unknown);
        indexBuffer.format = static_cast<uint32>(geometry.indexFormat);

        // Buffer loads require a be 4-byte aligned GPU VA. For 16-bit index buffers the GPU VA can be
        // 2-byte aligned, we align the GPU VA  down to 4-bytes and pass an offset for the compute shader to add
        // to the address
        constexpr uint64 BufferGpuVaAlign = 4;
        if (Util::IsPow2Aligned(indexBuffer.gpuVa, BufferGpuVaAlign) == false)
        {
            indexBuffer.gpuVa = Util::Pow2AlignDown(indexBuffer.gpuVa, BufferGpuVaAlign);
            indexBuffer.byteOffset = (static_cast<uint64>(geometry.indexBufferAddr.gpu) - indexBuffer.gpuVa);
        }
    }

    return indexBuffer;
}

// =====================================================================================================================
// Helper function that calculates the correct dispatch size to use for Ray Tracing shaders
static uint32 DispatchSize(
    uint32 numWorkItems)
{
    return Util::RoundUpQuotient(numWorkItems, DefaultThreadGroupSize);
}

// =====================================================================================================================
// Helper function that calculates the block count for input number of triangles
static uint32 TrianglePairBlockCount(
    uint32 numTriangles)
{
    constexpr uint32 TrianglePairBlockSize = 64;
    return Util::RoundUpQuotient(numTriangles, TrianglePairBlockSize);
}

// =====================================================================================================================
// Helper function that reserves the number of bytes in the running offset, returning the offset for the reservation
static uint32 ReserveBytes(
    uint32  numBytes,
    uint32* pRunningOffset)
{
    const uint32 requestOffset = *pRunningOffset;
    *pRunningOffset = requestOffset + numBytes;
    return requestOffset;
}

// =====================================================================================================================
// Returns the number of primitives in a triangle or procedural AABB geometry
static uint32 GetGeometryPrimCount(
    const Geometry& geometry)
{
    uint32 primCount = 0;

    if (geometry.type == GeometryType::Triangles)
    {
        if (geometry.triangles.indexFormat != IndexFormat::Unknown)
        {
            PAL_ASSERT((geometry.triangles.indexCount % 3) == 0);
            primCount = (geometry.triangles.indexCount / 3);
        }
        else
        {
            PAL_ASSERT((geometry.triangles.vertexCount % 3) == 0);
            primCount = (geometry.triangles.vertexCount / 3);
        }
    }
    else
    {
        // Procedural AABB geometry does not require any additional data. Note that AABBCount is 64-bit; in practice we
        // can't support more than 4 billion AABBs so for now just assert that it's a 32-bit value.
        PAL_ASSERT(Util::HighPart(geometry.aabbs.aabbCount) == 0);
        primCount = uint32(geometry.aabbs.aabbCount);
    }

    return primCount;
}

// =====================================================================================================================
// Helper function that dispatches of size {numGroups, 1, 1}
void BvhBuilder::Dispatch(
    uint32 numGroups)
{
    m_backend.Dispatch(m_cmdBuffer, numGroups, 1, 1);
}

// =====================================================================================================================
// Indirect dispatch using dimensions at the specified address
void BvhBuilder::DispatchIndirect(
    gpusize indirectArgumentAddr)
{
    m_backend.DispatchIndirect(m_cmdBuffer, indirectArgumentAddr);
}

// =====================================================================================================================
// Helper function that check if we need to force fp16 mode off
// Returns True if need to force the setting off, false otherwise.
static bool ForceDisableFp16BoxNodes(
    const AccelStructBuildInputs& buildArgs,
    const DeviceSettings& deviceSettings)
{
    const bool allowUpdate = buildArgs.flags & AccelStructBuildFlag::AccelStructBuildFlagAllowUpdate;
    const bool allowCompaction = buildArgs.flags & AccelStructBuildFlag::AccelStructBuildFlagAllowCompaction;

    // Disable fp16 mode if
    // 1-) AllowUpdate set when not allowed by panel setting
    // 2-) TLAS
    // 3-) AllowCompaction not set when required by panel setting
    // Otherwise, respect the fp16 mode passed in from device settings
    const bool forceFp16BoxOff = (buildArgs.type == AccelStructType::TopLevel) ||
        ((deviceSettings.allowFp16BoxNodesInUpdatableBvh == false) && allowUpdate) ||
        (deviceSettings.fp16BoxNodesRequireCompaction && (allowCompaction == false));

    return forceFp16BoxOff;
}

// =====================================================================================================================
// Helper function that check if pair compression should be enabled
// Returns True if triangle compression should be enabled based on the current build flags and settings.
static bool AutoSelectTriangleCompressMode(
    const AccelStructBuildInputs& buildArgs,
    const DeviceSettings& deviceSettings)
{
    const bool allowCompaction = Util::TestAnyFlagSet(buildArgs.flags, AccelStructBuildFlagAllowCompaction);
    const bool isFastTrace = Util::TestAnyFlagSet(buildArgs.flags, AccelStructBuildFlagPreferFastTrace);
    const bool isFastBuild = Util::TestAnyFlagSet(buildArgs.flags, AccelStructBuildFlagPreferFastBuild);
    const bool isDefaultBuild = (isFastTrace == false) && (isFastBuild == false);

    bool enableTriCompression = false;

    switch (deviceSettings.triangleCompressionAutoMode)
    {
    case TriangleCompressionAutoMode::Disabled:
        break;
    case TriangleCompressionAutoMode::AlwaysEnabled:
        enableTriCompression = true;
        break;
    case TriangleCompressionAutoMode::DefaultBuild:
        enableTriCompression = isDefaultBuild || isFastTrace;
        break;
    case TriangleCompressionAutoMode::FastTrace:
        enableTriCompression = isFastTrace;
        break;
    case TriangleCompressionAutoMode::Compaction:
        enableTriCompression = allowCompaction;
        break;
    case TriangleCompressionAutoMode::DefaultBuildWithCompaction:
        enableTriCompression = (isDefaultBuild || isFastTrace) && allowCompaction;
        break;
    case TriangleCompressionAutoMode::FastTraceWithCompaction:
        enableTriCompression = isFastTrace && allowCompaction;
        break;
    case TriangleCompressionAutoMode::DefaultBuildOrCompaction:
        enableTriCompression = isDefaultBuild || isFastTrace || allowCompaction;
        break;
    case TriangleCompressionAutoMode::FastTraceOrCompaction:
        enableTriCompression = isFastTrace || allowCompaction;
        break;
    default:
        break;
    }

    return enableTriCompression == true;
}

// =====================================================================================================================
// Calculate historgram element count for radix sort
uint32 BvhBuilder::GetNumHistogramElements(
    const RadixSortConfig& radixSortConfig,
    uint32                 primitiveCount)
{
    const uint32 numGroups = Util::RoundUpQuotient(primitiveCount, radixSortConfig.groupBlockSize);
    const uint32 numHistogramElements = numGroups * radixSortConfig.numBins;

    return numHistogramElements;
}

// =====================================================================================================================
// Helper function to determine leaf nodes size
uint32 BvhBuilder::GetLeafNodeSize(
    const DeviceSettings& settings,
    const BuildConfig&    config)
{
    uint32 size = 0;

    {
        if (config.topLevelBuild)
        {
            size = settings.enableFusedInstanceNode ? RayTracingFusedInstanceNodeSize : RayTracingInstanceNodeSize;
        }
        else
        {
            {
                size = RayTracingQBVHLeafSize;
            }
        }
    }

    return size;
}

// =====================================================================================================================
uint32 BvhBuilder::GetNumInternalNodeCount() const
{
    uint32 maxNumChildren = 4u;
    uint32 minPrimsPerLastInternalNode = 2u;

    {
    }

    return CalcAccelStructInternalNodeCount(m_buildConfig.maxNumPrimitives, maxNumChildren, minPrimsPerLastInternalNode);
}

// =====================================================================================================================
// Calculate the size required for internal nodes
uint32 BvhBuilder::CalculateInternalNodesSize()
{
    uint32 numInternalNodes = GetNumInternalNodeCount();
    uint32 numBox16Nodes = 0;
    switch (m_buildConfig.fp16BoxNodesInBlasMode)
    {
    case Fp16BoxNodesInBlasMode::NoNodes:
        // All nodes are fp32
        break;

    case Fp16BoxNodesInBlasMode::LeafNodes:
        // Conservative estimate how many interior nodes can be converted to fp16
        numBox16Nodes = (m_buildConfig.maxNumPrimitives / 4);
        numInternalNodes -= numBox16Nodes;
        break;

    case Fp16BoxNodesInBlasMode::MixedNodes:
        // Conservative estimate: no fp32 nodes could be converted to fp16
        // BVH storage savings realized after compaction copy
        break;

    case Fp16BoxNodesInBlasMode::AllNodes:
        numBox16Nodes = numInternalNodes - 1;
        numInternalNodes = 1;
        break;

    default:
        PAL_ASSERT_ALWAYS();
        break;
    }

    uint32 internalNodeSize;
    {
        internalNodeSize = RayTracingQBVH32NodeSize;
    }
    const uint32 sizeInBytes = (RayTracingQBVH16NodeSize * numBox16Nodes) +
                               (internalNodeSize * numInternalNodes);

    return sizeInBytes;
}

// =====================================================================================================================
// Static helper function that calculates the size of the buffer for leaf nodes
uint32 BvhBuilder::CalculateLeafNodesSize()
{
    {
        return m_buildConfig.maxNumPrimitives * GetLeafNodeSize(m_deviceSettings, m_buildConfig);
    }
}

// =====================================================================================================================
// Static helper function that calculates the size of the buffer for acceleration structure nodes
uint32 BvhBuilder::CalculateNodesSize()
{
    return CalculateInternalNodesSize() + CalculateLeafNodesSize();
}

// =====================================================================================================================
// Static helper function that calculates the size of the geometry desc info
uint32 BvhBuilder::CalculateGeometryInfoSize(
    uint32 numGeometryDescs) // Number of geometry descs
{
    struct GeometryInfo
    {
        uint32 geometryFlags;
        uint32 numPrimitives;
        uint32 primNodePtrsOffset;
    };

    return numGeometryDescs * sizeof(GeometryInfo);
}

// =====================================================================================================================
// Helper function that swaps the input arguments
template <typename T>
void Swap(
    T* pA, // First argument
    T* pB) // Second argument
{
    T c = *pA;
    *pA = *pB;
    *pB = c;
}

// =====================================================================================================================
// Explicit ray tracing bvh builder constructor that will perform a build/update
BvhBuilder::BvhBuilder(
    ClientCmdBufferHandle        cmdBuffer,       // The associated command buffer handle
    const IBackend&              backend,         // Backend interface
    Internal::Device*      const pDevice,         // GPURT device pointer.
    const Pal::DeviceProperties& deviceProps,     // PAL device properties
    ClientCallbacks              clientCb,        // Client cb table
    const DeviceSettings&        deviceSettings,  // Device settings
    const AccelStructBuildInfo&  buildInfo)       // Build args
    :
    m_pDevice(pDevice),
    m_deviceSettings(deviceSettings),
    m_clientCb(clientCb),
    m_buildArgs(buildInfo),
    m_deviceProps(deviceProps),
    m_cmdBuffer(cmdBuffer),
    m_backend(backend),
    m_buildSettings({}),
    m_radixSortConfig(GetRadixSortConfig(deviceSettings)),
    m_emitCompactDstGpuVa(0ull),
    m_buildSettingsHash(0)
{
    InitializeBuildConfigs();

    {
        const BuildShaderConstants shaderConstants = GetBuildShaderConstants();
        AllocateBuildShaderConstants(shaderConstants);
    }

    if (m_buildArgs.inputs.type == GpuRt::AccelStructType::BottomLevel)
    {
        InitGeometryConstants();
    }
}

// =====================================================================================================================
// Explicit ray tracing bvh builder constructor that will perform a copy or emit build info
BvhBuilder::BvhBuilder(
    ClientCmdBufferHandle        cmdBuffer,       // The associated command buffer handle
    const IBackend&              backend,         // Backend interface
    Internal::Device*      const pDevice,         // GPURT device pointer.
    const Pal::DeviceProperties& deviceProps,     // PAL device properties
    ClientCallbacks              clientCb,        // Client cb table
    const DeviceSettings&        deviceSettings)  // Device settings
    :
    m_pDevice(pDevice),
    m_deviceSettings(deviceSettings),
    m_clientCb(clientCb),
    m_buildArgs(AccelStructBuildInfo{}),
    m_deviceProps(deviceProps),
    m_cmdBuffer(cmdBuffer),
    m_backend(backend),
    m_buildSettings({}),
    m_radixSortConfig(GetRadixSortConfig(deviceSettings)),
    m_emitCompactDstGpuVa(0ull),
    m_buildSettingsHash(0)
{
    InitCopySettings();
}

// =====================================================================================================================
// Explicit ray tracing bvh builder destructor
BvhBuilder::~BvhBuilder()
{
}

// =====================================================================================================================
// Override build mode
BvhBuildMode BvhBuilder::OverrideBuildMode(
    const AccelStructBuildInfo& buildInfo)
{
    BvhBuildMode mode = m_buildConfig.buildMode;
    if ((m_deviceSettings.lbvhBuildThreshold != 0) &&
        (mode == BvhBuildMode::PLOC) &&
        (m_buildConfig.maxNumPrimitives <= m_deviceSettings.lbvhBuildThreshold))
    {
        // The memory size is required to be monotonically increasing when primitive # increases. Since mode linear
        // uses less memory than PLOC, we are good to switch the mode without breaking the requirement.
        mode = BvhBuildMode::Linear;
    }

    // Override the build mode if BLAS override mode is set and AS is bottom level.
    if ((m_deviceSettings.bvhBuildModeOverrideBLAS != BvhBuildMode::Auto) &&
        (buildInfo.inputs.type == AccelStructType::BottomLevel))
    {
        mode = m_deviceSettings.bvhBuildModeOverrideBLAS;
    }

    // Override the build mode if TLAS override mode is set and AS is top level.
    if ((m_deviceSettings.bvhBuildModeOverrideTLAS != BvhBuildMode::Auto) &&
        (buildInfo.inputs.type == AccelStructType::TopLevel))
    {
        mode = m_deviceSettings.bvhBuildModeOverrideTLAS;
    }

    PAL_ASSERT(mode != BvhBuildMode::Auto);

    return mode;
}

// =====================================================================================================================
// Remapped scratch buffer base address
bool BvhBuilder::AllowRemappingScratchBuffer() const
{
    return
        (m_deviceSettings.enableRemapScratchBuffer == true) &&
        (IsUpdate() == false) &&
        (m_deviceSettings.enableBuildAccelStructScratchDumping == false);
}

// =====================================================================================================================
// Remapped scratch buffer base address
gpusize BvhBuilder::RemappedScratchBufferBaseVa() const
{
    gpusize addr;
    if ((AllowRemappingScratchBuffer() == false) || (m_scratchBufferInfo.bvh2PhaseSize > m_resultBufferInfo.nodeSize))
    {
        addr = ScratchBufferBaseVa() + m_scratchBufferInfo.baseOffset;
    }
    else
    {
        addr = ResultBufferBaseVa() + m_resultBufferInfo.baseOffset;
    }
    return addr;
}

// =====================================================================================================================
// Calculates the actual scratch buffer size
uint32 BvhBuilder::CalculateScratchBufferSize(
    const ResultBufferInfo& resultBufferInfo,
    const ScratchBufferInfo& scratchBufferInfo)
{
    uint32 size = scratchBufferInfo.qbvhPhaseSize;

    if ((AllowRemappingScratchBuffer() == false) || (scratchBufferInfo.bvh2PhaseSize > resultBufferInfo.nodeSize))
    {
        size = Util::Max(size, scratchBufferInfo.baseOffset + scratchBufferInfo.bvh2PhaseSize);
    }

    return size;
}

// =====================================================================================================================
// Calculates the result buffer offsets and returns the total result memory size
BvhBuilder::ResultBufferInfo BvhBuilder::CalculateResultBufferInfo(
    AccelStructDataOffsets* pOffsets,
    uint32* pMetadataSizeInBytes)
{
    uint32 runningOffset = 0;

    //-----------------------------------------------------------------------------------------------------------//
    //  DestAccelerationStructureData layout
    //
    //-------------- Type: All ----------------------------------------------------------------------------------//
    //  AccelStructMetadataHeader
    //  Parent pointers
    //  AccelStructHeader
    //  Internal Nodes                              : BVHNode (BVH2) or Float32BoxNode (BVH4)
    //  Leaf Nodes                                  : BVHNode (BVH2), TriangleNode (BVH4) or InstanceNode (TopLevel)
    //  Per-geometry description info               : Geometry info in Bottom Level
    //  Per-leaf node pointers (uint32)
    //
    // Note: When earlyPairCompression is enabled, the leaf node data section is used to store the per-wave
    //       triangle pair counts and compute the prefix sum across all waves for a BLAS build. The encode
    //       phase reads the computed primitive reference offsets to determine where to write the leaf node
    //       data in scratch buffer. The per-wave counters are guaranteed to fit within the leaf node data section
    //       as worst case we need maxNumPrimitives (2^29) / 32 or 64 = 2^24 or 2^23 DWORD sized entries per build.
    //
    //-----------------------------------------------------------------------------------------------------------//
    AccelStructDataOffsets offsets = {};

    // Acceleration structure data starts with the header
    ReserveBytes(sizeof(AccelStructHeader), &runningOffset);

    uint32 internalNodeSize = 0;
    uint32 leafNodeSize = 0;

    if (m_buildConfig.maxNumPrimitives > 0)
    {
        internalNodeSize = CalculateInternalNodesSize();
        leafNodeSize = CalculateLeafNodesSize();

        offsets.internalNodes = ReserveBytes(internalNodeSize, &runningOffset);
        offsets.leafNodes = ReserveBytes(leafNodeSize, &runningOffset);

        if (m_buildConfig.topLevelBuild == false)
        {
            const uint32 geometryInfoSize = CalculateGeometryInfoSize(m_buildArgs.inputs.inputElemCount);
            offsets.geometryInfo = ReserveBytes(geometryInfoSize, &runningOffset);
        }

        offsets.primNodePtrs = ReserveBytes(m_buildConfig.maxNumPrimitives * sizeof(uint32), &runningOffset);
    }

    uint32 totalSizeInBytes = runningOffset;

    // Metadata section is at the beginning of the acceleration structure buffer
    uint32 metadataSizeInBytes;
    {
        metadataSizeInBytes = CalcMetadataSizeInBytes(internalNodeSize, leafNodeSize);
        // Align metadata size to cache line
        metadataSizeInBytes = Util::Pow2Align(metadataSizeInBytes, 128);

        totalSizeInBytes += metadataSizeInBytes;
    }

    if (pOffsets != nullptr)
    {
        memcpy(pOffsets, &offsets, sizeof(offsets));
    }

    if (pMetadataSizeInBytes != nullptr)
    {
        *pMetadataSizeInBytes = metadataSizeInBytes;
    }

    ResultBufferInfo info;
    info.baseOffset = metadataSizeInBytes + sizeof(AccelStructHeader);
    info.nodeSize = internalNodeSize + leafNodeSize;
    info.dataSize = totalSizeInBytes;
    return info;
}

// =====================================================================================================================
// Calculates the scratch buffer offsets and returns the total scratch memory size
BvhBuilder::ScratchBufferInfo BvhBuilder::CalculateScratchBufferInfo(
    RayTracingScratchDataOffsets* pOffsets)
{
    //--------------------------------------------
    // ScratchBuffer layout in each pass:
    //        [4]           [0]     [1]    [2]
    // Counter | BVH2 & QBVH | State | TS/Rebraid
    // Counter | BVH2 & QBVH | State | TD
    // Counter | BVH2 & QBVH | State | BVH2 | Sort
    // Counter | BVH2 & QBVH | State | BVH2 | PLOC/LTD
    // Counter | BVH2 & QBVH | QBVH
    // Counter | PostProc Kdops

    const bool willDumpScratchOffsets = m_deviceSettings.enableBuildAccelStructScratchDumping;

    // Reserve the beginning of the scratch buffer for the offsets data if dumping.
    uint32 runningOffset = (willDumpScratchOffsets ? sizeof(RayTracingScratchDataOffsets) : 0);

    // The offsets of bvh2 data are relative to the baseOffset of ScratchBuffer or ResultBuffer.
    // If remapScratchBuffer is enabled, the baseOffset is baseOffset0; otherwise, the baseOffset is 0.
    uint32 baseOffset0 = 0; // [0] end of BVH2 & QBVH section
    uint32 baseOffset1 = 0; // [1] end of State section
    uint32 baseOffset2 = 0; // [2] end of BVH2 section

    uint32 bvh2PhaseMaxSize = 0; // max bvh2 data size from baseOffset
    uint32 qbvhPhaseMaxSize = 0; // max qbvh data size from 0

    // The scratch acceleration structure is built as a BVH2.
    const uint32 aabbCount = m_buildConfig.maxNumPrimitives;
    const uint32 nodeCount = (aabbCount > 0) ? ((2 * aabbCount) - 1) : 0;

    //--------------------------------------------
    // BVH2 & QBVH
    uint32 bvhNodeData = 0xFFFFFFFF;
    uint32 bvhLeafNodeData = 0xFFFFFFFF;
    uint32 triangleSplitBoxes = 0xFFFFFFFF;
    uint32 fastLBVHRootNodeIndex = 0xFFFFFFFF;
    uint32 numBatches = 0xFFFFFFFF;

    //--------------------------------------------
    // Task Loop Counters
    uint32 encodeTaskCounter = 0xFFFFFFFF;
    uint32 taskLoopCounters = 0xFFFFFFFF;

    //--------------------------------------------
    // TaskQueueCounter
    uint32 triangleSplitTaskQueueCounter = 0xFFFFFFFF;
    uint32 rebraidTaskQueueCounter = 0xFFFFFFFF;
    uint32 tdTaskQueueCounter = 0xFFFFFFFF;
    uint32 plocTaskQueueCounter = 0xFFFFFFFF;
    uint32 debugCounters = 0;

    //--------------------------------------------
    // State
    uint32 propagationFlags = 0xFFFFFFFF;
    uint32 batchIndices = 0xFFFFFFFF;
    uint32 triangleSplitState = 0xFFFFFFFF;
    uint32 rebraidState = 0xFFFFFFFF;
    uint32 currentState = 0xFFFFFFFF;
    uint32 tdState = 0xFFFFFFFF;
    uint32 dynamicBlockIndex = 0xFFFFFFFF;
    uint32 sceneBounds = 0xFFFFFFFF;

    //--------------------------------------------
    // BVH2
    uint32 primIndicesSorted = 0xFFFFFFFF;

    //--------------------------------------------
    // TS / Rebraid
    uint32 triangleSplitRefs0 = 0xFFFFFFFF;
    uint32 triangleSplitRefs1 = 0xFFFFFFFF;
    uint32 splitPriorities = 0xFFFFFFFF;
    uint32 atomicFlagsTS = 0xFFFFFFFF;

    //--------------------------------------------
    // TD
    uint32 refList = 0xFFFFFFFF;
    uint32 refOffsets = 0xFFFFFFFF; // not used
    uint32 tdBins = 0xFFFFFFFF;
    uint32 tdNodeList = 0xFFFFFFFF;

    //--------------------------------------------
    // Sort + Linear/LBvh
    uint32 mortonCodes = 0xFFFFFFFF;
    uint32 mortonCodesSorted = 0xFFFFFFFF;
    uint32 primIndicesSortedSwap = 0xFFFFFFFF;
    uint32 histogram = 0xFFFFFFFF;
    uint32 tempKeys = 0xFFFFFFFF;
    uint32 tempVals = 0xFFFFFFFF;
    uint32 atomicFlags = 0xFFFFFFFF;
    uint32 distributedPartSums = 0xFFFFFFFF;

    //--------------------------------------------
    // PLOC / LTD
    uint32 clustersList0 = 0xFFFFFFFF;
    uint32 clustersList1 = 0xFFFFFFFF;
    uint32 neighbourIndices = 0xFFFFFFFF;
    uint32 atomicFlagsPloc = 0xFFFFFFFF;
    uint32 clusterOffsets = 0xFFFFFFFF;

    //--------------------------------------------
    // QBVH
    uint32 qbvhGlobalStack = 0;
    uint32 qbvhGlobalStackPtrs = 0;

    // ============ TaskLoopCounters ============
    {
        encodeTaskCounter = ReserveBytes(sizeof(EncodeTaskCounters), &runningOffset);
        taskLoopCounters = ReserveBytes(sizeof(TaskLoopCounters), &runningOffset);
    }

    // ============ TaskQueueCounter ============
    {
        if (m_buildConfig.triangleSplitting)
        {
            triangleSplitTaskQueueCounter = ReserveBytes(RayTracingTaskQueueCounterSize, &runningOffset);
        }

        if (m_buildConfig.rebraidType == GpuRt::RebraidType::V2)
        {
            rebraidTaskQueueCounter = ReserveBytes(RayTracingTaskQueueCounterSize, &runningOffset);
        }

        if (m_buildConfig.topDownBuild)
        {
            tdTaskQueueCounter = ReserveBytes(RayTracingTaskQueueCounterSize, &runningOffset);
        }
        else
        {
            if (m_buildConfig.buildMode == BvhBuildMode::PLOC)
            {
                plocTaskQueueCounter = ReserveBytes(RayTracingTaskQueueCounterSize, &runningOffset);
            }
        }
    }

    // ============ BVH2 & QBVH ============
    {
        // Align scratch nodes to fit two per cache line with no straddling
        runningOffset = Util::Pow2Align(runningOffset, RayTracingScratchNodeSize);

        // Scratch data for storing internal BVH node data
        bvhNodeData = ReserveBytes(nodeCount * RayTracingScratchNodeSize, &runningOffset);

        // Unsorted leaf buffer
        if (m_buildConfig.topDownBuild == false)
        {
            // Scratch data for storing unsorted leaf nodes. No additional memory is
            // used, so no additional scratch memory needs to be reserved.
            bvhLeafNodeData = bvhNodeData + (m_buildConfig.maxNumPrimitives - 1) * RayTracingScratchNodeSize;
        }
        else
        {
            // Additional scratch data for storing unsorted leaf nodes
            bvhLeafNodeData = ReserveBytes(aabbCount * RayTracingScratchNodeSize, &runningOffset);
        }

        if (m_buildConfig.triangleSplitting)
        {
            triangleSplitBoxes = ReserveBytes(aabbCount * sizeof(Aabb), &runningOffset);
        }

        if (m_buildConfig.enableFastLBVH)
        {
            fastLBVHRootNodeIndex = ReserveBytes(sizeof(uint32), &runningOffset);
        }

        if ((m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair) &&
            (m_buildConfig.enableEarlyPairCompression == false))
        {
            numBatches = ReserveBytes(sizeof(uint32), &runningOffset);
        }
    }
    baseOffset0 = runningOffset;

    // ============ QBVH ============
    {
        // The 2 DWORD stack must be 8 byte aligned to atomically store both DWORDs
        runningOffset = Util::Pow2Align(runningOffset, 8);

        // ..and QBVH global stack
        qbvhGlobalStack = runningOffset;

        const uint32 maxStackEntry = GetNumInternalNodeCount();

        // Stack pointers require 2 entries per node when fp16 and fp32 box nodes intermix in BLAS
        const Fp16BoxNodesInBlasMode intNodeTypes = m_buildConfig.fp16BoxNodesInBlasMode;
        const bool intNodeTypesMix = (intNodeTypes != Fp16BoxNodesInBlasMode::NoNodes) &&
            (intNodeTypes != Fp16BoxNodesInBlasMode::AllNodes);
        const bool intNodeTypesMixInBlas = intNodeTypesMix &&
            (m_buildArgs.inputs.type == AccelStructType::BottomLevel);

        bool stackHasTwoEntries = intNodeTypesMixInBlas;

        const uint32 stackEntrySize = stackHasTwoEntries ? 2u : 1u;
        ReserveBytes(maxStackEntry * stackEntrySize * sizeof(uint32), &runningOffset);

        qbvhGlobalStackPtrs = ReserveBytes(sizeof(StackPtrs), &runningOffset);

    }
    qbvhPhaseMaxSize = runningOffset;

    if (AllowRemappingScratchBuffer())
    {
        runningOffset = 0;
    }
    else if (willDumpScratchOffsets == false)
    {
        runningOffset = baseOffset0;
    }

    // ============ State ============
    {
        // Round up to multiple of primitive count. Encode* clears the flags based on the expansion factor which
        // must be a multiple of numPrimitives.
        const uint32 propagationFlagSlotCount =
            (m_buildConfig.numPrimitives == 0) ? 0 : Util::RoundUpToMultiple(aabbCount, m_buildConfig.numPrimitives);
        propagationFlags = ReserveBytes(propagationFlagSlotCount * sizeof(uint32), &runningOffset);

        if ((m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair) &&
            (m_buildConfig.enableEarlyPairCompression == false))
        {
            batchIndices = ReserveBytes(aabbCount * sizeof(uint32), &runningOffset);
        }

        if (m_buildConfig.triangleSplitting)
        {
            triangleSplitState = ReserveBytes(RayTracingStateTSBuildSize, &runningOffset);
        }

        if (m_buildConfig.rebraidType == GpuRt::RebraidType::V2)
        {
            rebraidState = ReserveBytes(RayTracingStateRebraidBuildSize, &runningOffset);
        }

        if ((m_buildConfig.topDownBuild == false) && (m_buildConfig.buildMode == BvhBuildMode::PLOC))
        {
            // PLOC state
            currentState = ReserveBytes(RayTracingStatePLOCSize, &runningOffset);
        }

        if (m_buildConfig.topDownBuild)
        {
            uint32 tdStateSize = 0;
            if (m_buildConfig.rebraidType == GpuRt::RebraidType::V1)
            {
                tdStateSize = RayTracingStateTDTRBuildSize;
            }
            else
            {
                tdStateSize = RayTracingStateTDBuildSize;
            }
            tdState = ReserveBytes(tdStateSize, &runningOffset);
        }

        dynamicBlockIndex = ReserveBytes(sizeof(uint32), &runningOffset);

        // scene bounding box + min/max prim size
        sceneBounds = ReserveBytes(sizeof(Aabb) + 2 * sizeof(float), &runningOffset);
        if (m_buildConfig.topLevelBuild == true)
        {
            ReserveBytes(sizeof(Aabb), &runningOffset); // scene bounding box for rebraid
        }
    }
    baseOffset1 = runningOffset;

    // ============ BVH2 ============
    {
        // Sorted primitive indices buffer size.
        primIndicesSorted = ReserveBytes(aabbCount * sizeof(uint32), &runningOffset);
    }
    baseOffset2 = runningOffset;

    // ============ TS/Rebraid ============
    if (willDumpScratchOffsets == false)
    {
        runningOffset = baseOffset1;
    }
    if (m_buildConfig.triangleSplitting)
    {
        triangleSplitRefs0 = ReserveBytes(aabbCount * RayTracingTSRefScratchSize, &runningOffset);
        triangleSplitRefs1 = ReserveBytes(aabbCount * RayTracingTSRefScratchSize, &runningOffset);

        splitPriorities = ReserveBytes(aabbCount * sizeof(float), &runningOffset);

        // TODO: calculate number of blocks based on KEYS_PER_THREAD
        atomicFlagsTS = ReserveBytes(aabbCount * RayTracingAtomicFlags, &runningOffset);
    }
    else if (m_buildConfig.rebraidType == GpuRt::RebraidType::V2)
    {
        const uint32 numFlags = Util::RoundUpToMultiple(aabbCount, uint32(REBRAID_KEYS_PER_THREAD));
        atomicFlagsTS = ReserveBytes(numFlags * RayTracingScanDLBFlagsSize, &runningOffset);;
    }
    bvh2PhaseMaxSize = Util::Max(bvh2PhaseMaxSize, runningOffset);

    // ============ TD ============
    if (willDumpScratchOffsets == false)
    {
        runningOffset = baseOffset1;
    }
    if (m_buildConfig.topDownBuild)
    {
        const uint32 refListSize = (m_buildConfig.rebraidType == GpuRt::RebraidType::V1) ?
                                   RayTracingTDTRRefScratchSize : RayTracingTDRefScratchSize;
        refList = ReserveBytes(refListSize * aabbCount, &runningOffset);

        // Align the beginning of the TDBins structs to 8 bytes so that 64-bit atomic operations on the first field in
        // the struct work correctly.
        runningOffset = Util::RoundUpToMultiple(runningOffset, 8u);
        tdBins = ReserveBytes(RayTracingTDBinsSize * (aabbCount / 3), &runningOffset);

        const uint32 tdNodeListSize = (m_buildConfig.rebraidType == GpuRt::RebraidType::V1) ?
                                      RayTracingTDTRNodeSize : RayTracingTDNodeSize;
        tdNodeList = ReserveBytes(RayTracingTDTRNodeSize * (aabbCount - 1), &runningOffset);
    }
    bvh2PhaseMaxSize = Util::Max(bvh2PhaseMaxSize, runningOffset);

    // ============ Sort ============
    if (willDumpScratchOffsets == false)
    {
        runningOffset = baseOffset2;
    }
    if (m_buildConfig.topDownBuild == false)
    {
        // Align to 8 bytes to prevent cache line straddling for 64 bit morton codes
        runningOffset = Util::Pow2Align(runningOffset, sizeof(uint64));

        const uint32 dataSize = m_deviceSettings.enableMortonCode30 ? sizeof(uint32) : sizeof(uint64);

        // Morton codes buffer size
        mortonCodes = ReserveBytes(aabbCount * dataSize, &runningOffset);
        // Sorted morton codes buffer size
        mortonCodesSorted = ReserveBytes(aabbCount * dataSize, &runningOffset);

        // GetAccelerationStructurePrebuildInfo may be called for an AS with number of elements being greater
        // than the MergeSort override threshold, but the actual AS build may have less elements and use MergeSort.
        // Currently, RadixSort always requires more scratch than MergeSort, so the size reported in the PrebuildInfo
        // will be sufficient, but this assumption must always hold.

        // Merge Sort
        if (m_buildConfig.enableMergeSort)
        {
            primIndicesSortedSwap = ReserveBytes(aabbCount * sizeof(uint32), &runningOffset);
        }
        // Radix Sort
        else
        {
            // Radix sort temporary buffers
            const uint32 numBlocks = (aabbCount + m_radixSortConfig.groupBlockSize - 1) /
                                     m_radixSortConfig.groupBlockSize;

            // device histograms buffer (int4)
            const uint32 numHistogramElements = numBlocks * m_radixSortConfig.numBins;
            histogram = ReserveBytes(numHistogramElements * sizeof(uint32), &runningOffset);

            // device temp keys buffer (int)
            tempKeys = ReserveBytes(aabbCount * dataSize, &runningOffset);

            // device temp vals buffer (int)
            tempVals = ReserveBytes(aabbCount * sizeof(uint32), &runningOffset);

            if (m_buildConfig.radixSortScanLevel == 0)
            {
                const uint32 blockSize = m_radixSortConfig.workGroupSize;
                const uint32 numKeysPerThread = m_radixSortConfig.keysPerThread;
                const uint32 numDynamicBlocks = (numHistogramElements +
                    ((blockSize * numKeysPerThread) - 1)) / (blockSize * numKeysPerThread);

                atomicFlags = ReserveBytes(numDynamicBlocks * RayTracingScanDLBFlagsSize, &runningOffset);
            }
            else
            {
                // partial sum scratch memory
                const uint32 numGroupsBottomLevelScan =
                    Util::RoundUpQuotient(m_buildConfig.numHistogramElements, m_radixSortConfig.groupBlockSizeScan);

                distributedPartSums = ReserveBytes(numGroupsBottomLevelScan * sizeof(uint32), &runningOffset);
                if (m_buildConfig.numHistogramElements >= m_radixSortConfig.scanThresholdTwoLevel)
                {
                    const uint32 numGroupsMidLevelScan =
                        Util::RoundUpQuotient(numGroupsBottomLevelScan, m_radixSortConfig.groupBlockSizeScan);
                    ReserveBytes(numGroupsMidLevelScan * sizeof(uint32), &runningOffset);
                }
            }
        }
    }
    bvh2PhaseMaxSize = Util::Max(bvh2PhaseMaxSize, runningOffset);

    // ============ PLOC/LTD ============
    if (willDumpScratchOffsets == false)
    {
        runningOffset = baseOffset2;
    }
    if (m_buildConfig.topDownBuild == false)
    {
        if (m_buildConfig.buildMode == BvhBuildMode::PLOC)
        {
            clustersList0 = ReserveBytes(aabbCount * sizeof(uint32), &runningOffset);
            clustersList1 = ReserveBytes(aabbCount * sizeof(uint32), &runningOffset);
            neighbourIndices = ReserveBytes(aabbCount * sizeof(uint32), &runningOffset);
            // TODO: calculate number of blocks based on KEYS_PER_THREAD
            atomicFlagsPloc = ReserveBytes(aabbCount * RayTracingPLOCFlags, &runningOffset);
            clusterOffsets = ReserveBytes(aabbCount * sizeof(uint32), &runningOffset);
        }
    }
    bvh2PhaseMaxSize = Util::Max(bvh2PhaseMaxSize, runningOffset);

    // If the caller requested offsets, return them.
    if (pOffsets != nullptr)
    {
        pOffsets->bvhNodeData = bvhNodeData;
        pOffsets->triangleSplitBoxes = triangleSplitBoxes;
        pOffsets->triangleSplitRefs0 = triangleSplitRefs0;
        pOffsets->triangleSplitRefs1 = triangleSplitRefs1;
        pOffsets->splitPriorities = splitPriorities;
        pOffsets->triangleSplitState = triangleSplitState;
        pOffsets->encodeTaskCounter = encodeTaskCounter;
        pOffsets->taskLoopCounters = taskLoopCounters;
        pOffsets->triangleSplitTaskQueueCounter = triangleSplitTaskQueueCounter;
        pOffsets->rebraidState = rebraidState;
        pOffsets->rebraidTaskQueueCounter = rebraidTaskQueueCounter;
        pOffsets->splitAtomicFlags = atomicFlagsTS;
        pOffsets->tdRefs = refList;
        pOffsets->tdNodeList = tdNodeList;
        pOffsets->tdBins = tdBins;
        pOffsets->tdState = tdState;
        pOffsets->tdTaskQueueCounter = tdTaskQueueCounter;
        pOffsets->refOffsets = refOffsets;
        pOffsets->bvhLeafNodeData = bvhLeafNodeData;
        pOffsets->clusterList0 = clustersList0;
        pOffsets->clusterList1 = clustersList1;
        pOffsets->neighborIndices = neighbourIndices;
        pOffsets->currentState = currentState;
        pOffsets->plocTaskQueueCounter = plocTaskQueueCounter;
        pOffsets->atomicFlagsPloc = atomicFlagsPloc;
        pOffsets->clusterOffsets = clusterOffsets;
        pOffsets->sceneBounds = sceneBounds;
        pOffsets->mortonCodes = mortonCodes;
        pOffsets->mortonCodesSorted = mortonCodesSorted;
        pOffsets->primIndicesSorted = primIndicesSorted;
        pOffsets->primIndicesSortedSwap = primIndicesSortedSwap;
        pOffsets->propagationFlags = propagationFlags;
        pOffsets->histogram = histogram;
        pOffsets->tempKeys = tempKeys;
        pOffsets->tempVals = tempVals;
        pOffsets->dynamicBlockIndex = dynamicBlockIndex;
        pOffsets->prefixSumAtomicFlags = atomicFlags;
        pOffsets->distributedPartialSums = distributedPartSums;
        pOffsets->qbvhGlobalStack = qbvhGlobalStack;
        pOffsets->qbvhGlobalStackPtrs = qbvhGlobalStackPtrs;
        pOffsets->debugCounters = debugCounters;
        pOffsets->numBatches = numBatches;
        pOffsets->batchIndices = batchIndices;
        pOffsets->fastLBVHRootNodeIndex = fastLBVHRootNodeIndex;

    }

    ScratchBufferInfo info;
    info.baseOffset = (AllowRemappingScratchBuffer() ? baseOffset0 : 0);
    info.bvh2PhaseSize = bvh2PhaseMaxSize;
    info.qbvhPhaseSize = qbvhPhaseMaxSize;
    return info;
}

// =====================================================================================================================
// Calculates the update scratch buffer offsets and returns the total update scratch memory size
uint32 BvhBuilder::CalculateUpdateScratchBufferInfo(
    RayTracingScratchDataOffsets* pOffsets)
{
    uint32 runningOffset = 0;
    //-----------------------------------------------------------------------------------------------------------//
    //  Update Scratch Data layout
    //
    //-------------- Type: All ----------------------------------------------------------------------------------//
    // UpdateStackPointer      uint32
    // UpdateTaskCount         uint32
    // UpdateEncodeTaskCount   uint32
    // UpdateEncodeTasksDone   uint32
    // PropagationFlags        (uint32 PropagationFlags[NumPrimitives])
    // UpdateStackElements     uint32[NumPrimitives]
    //-----------------------------------------------------------------------------------------------------------//
    RayTracingScratchDataOffsets offsets = {};

    // Update stack pointer
    ReserveBytes(sizeof(uint32), &runningOffset);

    // Done count
    ReserveBytes(sizeof(uint32), &runningOffset);

    // Encode task loop counter
    offsets.encodeTaskCounter = ReserveBytes(sizeof(uint32), &runningOffset);

    // Encode tasks done count
    ReserveBytes(sizeof(uint32), &runningOffset);

    // Allocate space for the node flags
    offsets.propagationFlags = ReserveBytes(m_buildConfig.maxNumPrimitives * sizeof(uint32), &runningOffset);

    // Allocate space for update stack. Note, for a worst case tree, each leaf node enqueues a single parent
    // node pointer for updating
    offsets.updateStack = ReserveBytes(m_buildConfig.maxNumPrimitives * sizeof(uint32), &runningOffset);

    if (pOffsets != nullptr)
    {
        memcpy(pOffsets, &offsets, sizeof(offsets));
    }

    return runningOffset;
}

// =====================================================================================================================
// Gets geometry type for BLAS build inputs
GeometryType BvhBuilder::GetGeometryType(
    const AccelStructBuildInputs inputs)
{
    GeometryType type;

    const bool isBottomLevel = (inputs.type == AccelStructType::BottomLevel);

    if (isBottomLevel && (inputs.inputElemCount > 0))
    {
        const Geometry geometry = m_clientCb.pfnConvertAccelStructBuildGeometry(inputs, 0);
        type = geometry.type;
    }
    else
    {
        // No geometries, so pick an arbitrary geometry type to initialize the variable
        type = GeometryType::Triangles;
    }

    return type;
}

// =====================================================================================================================
// Initialize buildConfig
void BvhBuilder::InitBuildConfig(
    const AccelStructBuildInfo& buildArgs) // Input build args
{
    m_buildConfig = {};

    // For top-level acceleration structure, inputElementCount represents the number of instances
    uint32 primitiveCount =
        (buildArgs.inputs.type == AccelStructType::BottomLevel) ? 0 : buildArgs.inputs.inputElemCount;

    uint32 trianglePairBlockCount = 0;

    if (buildArgs.inputs.type == AccelStructType::BottomLevel)
    {
        for (uint32 i = 0; i < buildArgs.inputs.inputElemCount; ++i)
        {
            const Geometry geometry = m_clientCb.pfnConvertAccelStructBuildGeometry(buildArgs.inputs, i);
            const uint32 geometryPrimCount = GetGeometryPrimCount(geometry);

            trianglePairBlockCount += TrianglePairBlockCount(geometryPrimCount);
            primitiveCount += geometryPrimCount;
        }
    }

    if (Util::TestAnyFlagSet(buildArgs.inputs.flags, AccelStructBuildFlagPreferFastTrace))
    {
        m_buildConfig.buildMode = m_deviceSettings.bvhBuildModeFastTrace;
        m_buildConfig.cpuBuildMode = m_deviceSettings.bvhCpuBuildModeFastTrace;
    }
    else if (Util::TestAnyFlagSet(buildArgs.inputs.flags, AccelStructBuildFlagPreferFastBuild))
    {
        m_buildConfig.buildMode = m_deviceSettings.bvhBuildModeFastBuild;
        m_buildConfig.cpuBuildMode = m_deviceSettings.bvhCpuBuildModeFastBuild;
    }
    else
    {
        m_buildConfig.buildMode = m_deviceSettings.bvhBuildModeDefault;
        m_buildConfig.cpuBuildMode = m_deviceSettings.bvhCpuBuildModeDefault;
    }

    m_buildConfig.numPrimitives = primitiveCount;
    m_buildConfig.maxNumPrimitives = primitiveCount;
    m_buildConfig.rebraidType = m_deviceSettings.rebraidType;
    m_buildConfig.topLevelBuild = buildArgs.inputs.type == AccelStructType::TopLevel;
    m_buildConfig.geometryType = GetGeometryType(buildArgs.inputs);

    const bool isTrianglePrimitiveBuild = (buildArgs.inputs.type == AccelStructType::BottomLevel) &&
                                          (m_buildConfig.geometryType == GeometryType::Triangles);

    {
        m_buildConfig.fp16BoxNodesInBlasMode = ForceDisableFp16BoxNodes(m_buildArgs.inputs, m_deviceSettings) ?
            Fp16BoxNodesInBlasMode::NoNodes : m_deviceSettings.fp16BoxNodesInBlasMode;

        if (isTrianglePrimitiveBuild)
        {
            m_buildConfig.triangleCompressionMode =
                AutoSelectTriangleCompressMode(m_buildArgs.inputs, m_deviceSettings) ?
                    TriangleCompressionMode::Pair : TriangleCompressionMode::None;
        }
    }

    m_buildConfig.allowTopDownBuild = (m_buildConfig.topLevelBuild) &&
        ((m_deviceSettings.topDownBuild == true) || (m_deviceSettings.rebraidType == RebraidType::V1));

    m_buildConfig.topDownBuild = m_buildConfig.allowTopDownBuild &&
        (buildArgs.inputs.inputElemCount <= m_deviceSettings.maxTopDownBuildInstances);

    if ((Util::TestAnyFlagSet(m_buildArgs.inputs.flags, AccelStructBuildFlagAllowUpdate) == false) &&
        m_buildConfig.topLevelBuild)
    {
        if (m_buildConfig.rebraidType == RebraidType::V1)
        {
            // inputs > maxTopDownBuildInstances turn off rebraid
            if (m_buildConfig.topDownBuild == false)
            {
                m_buildConfig.rebraidType = RebraidType::Off;
            }
        }
        else
        {
            m_buildConfig.rebraidType = m_buildConfig.rebraidType;
        }
    }
    else
    {
        m_buildConfig.rebraidType = RebraidType::Off;
    }

    m_buildConfig.triangleSplitting = (m_deviceSettings.enableParallelBuild) &&
        m_deviceSettings.enableTriangleSplitting && (buildArgs.inputs.type == AccelStructType::BottomLevel) &&
        (Util::TestAnyFlagSet(buildArgs.inputs.flags, AccelStructBuildFlagAllowUpdate) == false) &&
        Util::TestAnyFlagSet(buildArgs.inputs.flags, AccelStructBuildFlagPreferFastTrace);

    m_buildConfig.buildMode = OverrideBuildMode(buildArgs);

    if (m_buildConfig.rebraidType != RebraidType::Off)
    {
        m_buildConfig.maxNumPrimitives *= m_deviceSettings.rebraidFactor;
    }

    if (m_buildConfig.triangleSplitting == true)
    {
        m_buildConfig.maxNumPrimitives = NumPrimitivesAfterSplit(m_buildConfig.maxNumPrimitives,
                                                                 m_deviceSettings.triangleSplittingFactor);
    }

    // Merge sort outperforms Radix sort in most cases in the batched builder
    bool forceMergeSort = m_deviceSettings.enableParallelBuild == false;
    if (m_deviceSettings.enableParallelBuild)
    {
        // Merge sort outperforms Radix sort for small BVH builds in the parallel builder
        static constexpr uint32 MergeSortThreshold = 50000u;
        forceMergeSort = m_buildConfig.numPrimitives <= MergeSortThreshold;
    }
    m_buildConfig.enableMergeSort = forceMergeSort ? true : m_deviceSettings.enableMergeSort;

    m_buildConfig.radixSortScanLevel = 0;
    m_buildConfig.numHistogramElements = GetNumHistogramElements(m_radixSortConfig, m_buildConfig.maxNumPrimitives);

    if (m_deviceSettings.enablePrefixScanDLB == false)
    {
        if (m_buildConfig.numHistogramElements < m_radixSortConfig.scanThresholdOneLevel)
        {
            m_buildConfig.radixSortScanLevel = 1;
        }
        else if (m_buildConfig.numHistogramElements < m_radixSortConfig.scanThresholdTwoLevel)
        {
            m_buildConfig.radixSortScanLevel = 2;
        }
        else if (m_buildConfig.numHistogramElements < m_radixSortConfig.scanThresholdThreeLevel)
        {
            m_buildConfig.radixSortScanLevel = 3;
        }
        else
        {
            PAL_ASSERT_ALWAYS_MSG("%s", "The maximum number of elements for scan exceeded\n");
        }
    }

    m_buildConfig.numMortonSizeBits = m_deviceSettings.numMortonSizeBits;

    m_buildConfig.enableFastLBVH = (m_buildConfig.topDownBuild == false) && (m_deviceSettings.enableFastLBVH == true) &&
        ((m_buildConfig.buildMode == BvhBuildMode::Linear)
             );

    m_buildConfig.sceneCalcType = SceneBoundsCalculation::BasedOnGeometry;

    // Only enable earlyPairCompression if
    // 1. triangleCompressionMode is set to be "Pair", and
    // 2. deviceSettings chose to enable EarlyPairCompression
    // 3. This is not an update
    m_buildConfig.enableEarlyPairCompression = false;

    if (IsUpdate() == false)
    {
        if (m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair)
        {
            m_buildConfig.enableEarlyPairCompression = m_deviceSettings.enableEarlyPairCompression;
        }
    }

    if (m_buildConfig.enableEarlyPairCompression)
    {
        m_buildConfig.trianglePairBlockCount = trianglePairBlockCount;
    }

    // Max geometries we can fit in the SRD table for merged encode buiping ld/update
    const uint32 maxDescriptorTableSize = (m_cmdBuffer != nullptr) ?
        m_backend.GetMaxDescriptorTableSize(m_cmdBuffer) : 0;

    m_buildConfig.needEncodeDispatch =
        (IsUpdate() &&
         (m_deviceSettings.enableMergedEncodeUpdate == 0)) ||
        ((IsUpdate() == false) && (m_deviceSettings.enableMergedEncodeBuild == 0)) ||
        (buildArgs.inputs.type == AccelStructType::TopLevel) ||
        ((IsUpdate() == false) && (m_buildConfig.geometryType == GeometryType::Aabbs))
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 46
        || (buildArgs.inputs.inputElemCount > maxDescriptorTableSize)
#endif
        ;

}

// =====================================================================================================================
BuildShaderConstants BvhBuilder::GetBuildShaderConstants() const
{
    // Make sure the address for empty top levels is 0 so we avoid tracing them. Set a valid address for empty
    // bottom levels so the top level build can still read the bottom level header.
    const bool emptyTopLevel = (m_buildConfig.topLevelBuild) && (m_buildConfig.numPrimitives == 0);
    const gpusize resultBufferAddress =
        IsUpdate() ? HeaderBufferBaseVa() :
        emptyTopLevel ? 0 : ResultBufferBaseVa();

    const BuildShaderConstants constants = {
        .resultBufferAddrLo      = Util::LowPart(resultBufferAddress),
        .resultBufferAddrHi      = Util::HighPart(resultBufferAddress),
        .numPrimitives           = m_buildConfig.numPrimitives,
        .tsBudgetPerTriangle     = IsUpdate() ? 0 : m_deviceSettings.tsBudgetPerTriangle,

        .maxNumPrimitives        = IsUpdate() ? 0 : m_buildConfig.maxNumPrimitives,
        .rebraidFactor           = IsUpdate() ? 0 : m_deviceSettings.rebraidFactor,

        .indirectArgBufferStride = m_buildArgs.indirect.indirectStride,
        .numDescs                = m_buildArgs.inputs.inputElemCount,
        .leafNodeExpansionFactor = GetLeafNodeExpansion(),
        .numMortonSizeBits       = m_buildConfig.numMortonSizeBits,

        .header                  = IsUpdate() ? AccelStructHeader{} : InitAccelStructHeader(),
        .offsets                 = m_scratchOffsets,
    };
    return constants;
}

// =====================================================================================================================
void BvhBuilder::AllocateBuildShaderConstants(
    const BuildShaderConstants& buildShaderConstants)
{
    // Add 4 DWORD padding to avoid page faults when the compiler uses a multi-DWORD load straddling the end of the
    // constant buffer
    static constexpr uint32 BufferPadding = 4u;
    static constexpr uint32 NumDwordsInBuildShaderConstants = (sizeof(BuildShaderConstants) / sizeof(uint32));
    static constexpr uint32 AllocationSizeBytes = (NumDwordsInBuildShaderConstants + BufferPadding) * sizeof(uint32);
    void* pData = m_pDevice->AllocateTemporaryData(m_cmdBuffer,
                                                   AllocationSizeBytes,
                                                   &m_shaderConstantsGpuVa);
    std::memcpy(pData, &buildShaderConstants, sizeof(BuildShaderConstants));
}

// =====================================================================================================================
void BvhBuilder::InitGeometryConstants()
{
    const uint32 geometryCount = m_buildArgs.inputs.inputElemCount;

    uint32 primitiveOffset = 0;
    uint32 blockOffset = 0;

    const gpusize sizeInBytes = geometryCount * sizeof(BuildShaderGeometryConstants);

    gpusize geometryConstGpuVa = 0ull;
    BuildShaderGeometryConstants* pConstants = reinterpret_cast<BuildShaderGeometryConstants*>(
        m_pDevice->AllocateTemporaryData(m_cmdBuffer, sizeInBytes, &geometryConstGpuVa));

    void* pVbvTable = m_pDevice->AllocateDescriptorTable(m_cmdBuffer, geometryCount, &m_geomBufferSrdTable);
    void* pCbvTable = m_pDevice->AllocateDescriptorTable(m_cmdBuffer, geometryCount, &m_geomConstSrdTable);
    const uint32 srdSizeBytes = m_pDevice->GetBufferSrdSizeDw() * sizeof(uint32);

    for (uint32 i = 0; i < geometryCount; i++)
    {
        const Geometry geometry = m_clientCb.pfnConvertAccelStructBuildGeometry(m_buildArgs.inputs, i);
        const uint32 primitiveCount = GetGeometryPrimCount(geometry);

        uint32 stride = 0;
        uint32 vertexComponentCount = 0;

        IndexBufferInfo indexBufferInfo = {};

        BufferViewInfo vertexBufferViewInfo = {};

        uint32 vertexComponentSize = 0;
        if (geometry.type == GeometryType::Triangles)
        {
            vertexBufferViewInfo = SetupVertexBuffer(geometry.triangles, &stride, &vertexComponentCount);
            indexBufferInfo = GetIndexBufferInfo(geometry.triangles);
            vertexComponentSize = uint32(GetBytesPerComponentForFormat(geometry.triangles.vertexFormat));
        }
        else
        {
            vertexBufferViewInfo = SetupAabbBuffer(geometry.aabbs, &stride);
        }

        m_backend.CreateBufferViewSrds(1, vertexBufferViewInfo, pVbvTable, true);
        pVbvTable = Util::VoidPtrInc(pVbvTable, srdSizeBytes);

        pConstants[i] =
        {
            .numPrimitives                   = primitiveCount,
            .primitiveOffset                 = primitiveOffset,
            .blockOffset                     = blockOffset,
            .geometryStride                  = stride,
            .indexBufferGpuVaLo              = Util::LowPart(indexBufferInfo.gpuVa),
            .indexBufferGpuVaHi              = Util::HighPart(indexBufferInfo.gpuVa),
            .indexBufferByteOffset           = uint32(indexBufferInfo.byteOffset),
            .indexBufferFormat               = indexBufferInfo.format,
            .transformBufferGpuVaLo          = Util::LowPart(geometry.triangles.columnMajorTransform3x4.gpu),
            .transformBufferGpuVaHi          = Util::HighPart(geometry.triangles.columnMajorTransform3x4.gpu),
            .geometryFlags                   = uint32(geometry.flags),
            .vertexCount                     = geometry.triangles.vertexCount,
            .vertexComponentCount            = vertexComponentCount,
            .vertexComponentSize             = vertexComponentSize,
        };

        const BufferViewInfo constBufferViewInfo =
        {
            .gpuAddr = geometryConstGpuVa + (i * sizeof(BuildShaderGeometryConstants)),
            .range   = sizeof(BuildShaderGeometryConstants),
            .stride  = 1,
        };

        m_backend.CreateBufferViewSrds(1, constBufferViewInfo, pCbvTable, false);
        pCbvTable = Util::VoidPtrInc(pCbvTable, srdSizeBytes);

        blockOffset += TrianglePairBlockCount(primitiveCount);
        primitiveOffset += primitiveCount;
    }
}

// =====================================================================================================================
AccelStructMetadataHeader BvhBuilder::InitAccelStructMetadataHeader()
{
    // Make sure the address for empty top levels is 0 so we avoid tracing them. Set a valid address for empty bottom
    // levels so the top level build can still read the bottom level header.
    const bool emptyTopLevel = (m_buildConfig.topLevelBuild) && (m_buildConfig.numPrimitives == 0);

    const AccelStructMetadataHeader metaHeader =
    {
        .addressLo   = emptyTopLevel ? 0 : Util::LowPart(ResultBufferBaseVa()),
        .addressHi   = emptyTopLevel ? 0 : Util::HighPart(ResultBufferBaseVa()),
        .sizeInBytes = m_metadataSizeInBytes,
    };
    return metaHeader;
}

// =====================================================================================================================
AccelStructHeader BvhBuilder::InitAccelStructHeader() const
{
    const uint32 accelStructSize = m_resultBufferInfo.dataSize;

    AccelStructHeader      header = {};
    AccelStructHeaderInfo  info   = {};
    AccelStructHeaderInfo2 info2  = {};

    info.type                       = static_cast<uint32>(m_buildArgs.inputs.type);
    info.buildType                  = static_cast<uint32>(AccelStructBuilderType::Gpu);
    info.mode                       = m_buildSettings.buildMode;
    info.triCompression             = static_cast<uint32>(m_buildConfig.triangleCompressionMode);
    info.fp16BoxNodesInBlasMode     = static_cast<uint32>(m_buildConfig.fp16BoxNodesInBlasMode);
    info.triangleSplitting          = m_buildConfig.triangleSplitting;
    info.rebraid                    = m_buildConfig.rebraidType != RebraidType::Off;
    info.fusedInstanceNode          = m_deviceSettings.enableFusedInstanceNode;
    info.flags                      = m_buildArgs.inputs.flags;

    header.info                     = info;
    header.info2                    = info2;
    header.accelStructVersion       = GPURT_ACCEL_STRUCT_VERSION;
    header.metadataSizeInBytes      = m_metadataSizeInBytes;
    header.sizeInBytes              = accelStructSize;
    header.numPrimitives            = m_buildConfig.maxNumPrimitives; // Is this correct?
    header.numDescs                 = m_buildArgs.inputs.inputElemCount;
    header.geometryType             = static_cast<uint32>(m_buildConfig.geometryType);
    header.uuidLo                   = Util::LowPart(m_deviceSettings.accelerationStructureUUID);
    header.uuidHi                   = Util::HighPart(m_deviceSettings.accelerationStructureUUID);
    header.rtIpLevel                = uint32(m_pDevice->GetRtIpLevel());

    if (m_buildConfig.topLevelBuild)
    {
        header.numLeafNodes = m_buildArgs.inputs.inputElemCount;
    }

    header.offsets.internalNodes    = m_resultOffsets.internalNodes;
    header.offsets.leafNodes        = m_resultOffsets.leafNodes;
    header.offsets.geometryInfo     = m_resultOffsets.geometryInfo;
    header.offsets.primNodePtrs     = m_resultOffsets.primNodePtrs;

    return header;
}

// =====================================================================================================================
// Setup a typed buffer view for the provided triangle geometry.
BufferViewInfo BvhBuilder::SetupVertexBuffer(
    const GeometryTriangles& desc,
    uint32*                  pStride,
    uint32*                  pVertexCompCount
    ) const
{
    const BufferViewFormat format = VertexFormatToBufferViewFormat(desc.vertexFormat);
    const BufferViewFormat singleCompNumFormat = GetSingleComponentFormatForFormat(format);
    const uint8 validChannels = GetNumComponentsForVertexFormat(desc.vertexFormat);

    const gpusize vbGpuAddr     = desc.vertexBufferAddr.gpu;
    const gpusize vbStrideBytes = desc.vertexBufferByteStride;
    const gpusize reqAlignment  =
        (format == BufferViewFormat::R32G32B32_Float) ?
            sizeof(uint32) :
            GetBytesPerPixelForFormat(format);

    const bool isAligned =
        Util::IsPow2Aligned(vbGpuAddr, reqAlignment) && Util::IsPow2Aligned(vbStrideBytes, reqAlignment);

    BufferViewInfo bufferInfo = {};
    bufferInfo.gpuAddr = vbGpuAddr;
    bufferInfo.range   = desc.vertexCount * vbStrideBytes;

    // Vertex buffers are only required to be aligned to the format's component size, not the full format element size.
    // If the buffer address and stride are sufficiently aligned, we can use a multi-component format to load all vertex
    // components at once. If not, we need to load each component separately using a single channel typed buffer.
    // Indirect builds can set vertex buffer offset with component-size precision. We cannot check if offset aligns
    // with element size before dispatch, so we set SRD to allow perComponent fetches just in case.
    if (!m_buildSettings.isIndirectBuild && isAligned)
    {
        // Setup vertex buffer as a 2 or 3 channel typed buffer to fetch all components in one load
        bufferInfo.stride  = vbStrideBytes;
        bufferInfo.format  = format;
        bufferInfo.swizzle = (validChannels == 2) ? BufferViewSwizzle::TwoChannelMapping
                                                  : BufferViewSwizzle::ThreeChannelMapping;

        // Stride is handled in the SRD
        *pStride = 0;
    }
    else
    {
        // Setup vertex buffer as a single channel typed buffer
        PAL_ASSERT(singleCompNumFormat != BufferViewFormat::Undefined);

        const uint32 componentBytes = GetBytesPerPixelForFormat(singleCompNumFormat);

        PAL_ASSERT(Util::IsPow2Aligned(vbGpuAddr, componentBytes) &&
                   Util::IsPow2Aligned(vbStrideBytes, componentBytes));

        bufferInfo.stride  = componentBytes;
        bufferInfo.format  = singleCompNumFormat;
        bufferInfo.swizzle = BufferViewSwizzle::SingleChannelMapping;

        *pStride = desc.vertexBufferByteStride / componentBytes;
    }

    *pVertexCompCount = validChannels;

    return bufferInfo;
}

// =====================================================================================================================
// Return integer expansion factor which determines the number of flag slots each thread clears during Encode.
uint32 BvhBuilder::GetLeafNodeExpansion() const
{
    return (m_buildConfig.numPrimitives == 0) ?
        0 : Util::RoundUpQuotient(m_buildConfig.maxNumPrimitives, m_buildConfig.numPrimitives);
}

// =====================================================================================================================
// Setup a typed buffer view for the provided procedural geometry.
BufferViewInfo BvhBuilder::SetupAabbBuffer(
    const GeometryAabbs& desc,
    uint32*              pStride
    ) const
{
    // API alignment reqirements
    PAL_ASSERT(Util::IsPow2Aligned(desc.aabbAddr.gpu, 8));
    PAL_ASSERT(Util::IsPow2Aligned(desc.aabbByteStride, 8));

    const uint32 inputBufferSize = desc.aabbCount * desc.aabbByteStride;

    // The input buffer stride is required to be 0 or a multiple of 8.
    // It doesn't make sense to be smaller than an AABB. DXR and VK specs don't specify what happens in this case.
    PAL_ASSERT((desc.aabbByteStride == 0) || (desc.aabbByteStride >= sizeof(Aabb)));
    const uint32 inputByteStride = Util::Max<uint32>(desc.aabbByteStride, sizeof(Aabb));

    // Setup a typed buffer which can fetch 2 floats at a time.
    const BufferViewInfo bufferInfo =
    {
        .gpuAddr = desc.aabbAddr.gpu,
        .range   = inputBufferSize,
        .stride  = 8,
        .format  = BufferViewFormat::R32G32_Float,
        .swizzle = BufferViewSwizzle::TwoChannelMapping,
    };

    // Stride constant is in terms of X32Y32 elements.
    *pStride = inputByteStride / 8;

    return bufferInfo;
}

// =====================================================================================================================
// Executes the encode instances shader
void BvhBuilder::EncodeInstances(
    uint32             numDesc,        // Number of instance descriptions
    InputElementLayout descLayout)     // The layout of the instance descriptions
{
    BindPipeline(InternalRayTracingCsType::EncodeInstances);

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    RGP_PUSH_MARKER("Encode Instances (NumDescs=%u)", numDesc);
    Dispatch(DispatchSize(numDesc));

    RGP_POP_MARKER();
}

// =====================================================================================================================
void BvhBuilder::WriteImmediateSingle(
    gpusize                 destVa,
    uint64                  value,
    ImmediateDataWidth      width)
{
    m_backend.WriteImmediateSingle(m_cmdBuffer, destVa, value, width);
}

// =====================================================================================================================
// Use pre-CS (ME) CP writes to write the specified value to the destination VA.
template<typename T>
void BvhBuilder::WriteImmediateData(
    gpusize  destVa, // Destination address
    const T& data)   // Data value to write
{
    static_assert((sizeof(T) % sizeof(uint32)) == 0, "Data type must be 4 byte aligned for WriteImmediateData()");
    const uint32  count = sizeof(T) / sizeof(uint32);
    const uint32* pData = reinterpret_cast<const uint32*>(&data);

    WriteOrZeroDataImmediate(destVa, pData, count);
}

// =====================================================================================================================
// Use pre-CS (ME) CP writes to write the specified value to the destination VA.
void BvhBuilder::WriteOrZeroDataImmediate(
    gpusize       destVa,       // Destination address
    const uint32* pData,        // Data to write or null. Zeros will be written if no data is provided.
    uint32        dwordCount)   // Number of dwords to write.
{
    // Alignment required for 32 bit write immediate
    PAL_ASSERT(Util::IsPow2Aligned(destVa, 4));

    // Align up to 8 bytes if needed
    if ((dwordCount > 0) && (Util::IsPow2Aligned(destVa, 8) == false))
    {
        const uint64 writeData = (pData != nullptr) ? *pData : 0;

        WriteImmediateSingle(destVa, writeData, ImmediateDataWidth::ImmediateData32Bit);

        destVa += sizeof(uint32);
        if (pData != nullptr)
        {
            pData++;
        }
        dwordCount--;

        // Alignment required for 64 bit write immediate
        PAL_ASSERT(Util::IsPow2Aligned(destVa, 8));
    }

    // Use 64 bit writes while there is enough data available
    while (dwordCount >= 2)
    {
        const uint64 writeData =
            (pData != nullptr) ?
                (pData[0] | (static_cast<uint64>(pData[1]) << 32)) : 0;

        WriteImmediateSingle(destVa, writeData, ImmediateDataWidth::ImmediateData64Bit);

        destVa += sizeof(uint64);
        if (pData != nullptr)
        {
            pData += 2;
        }
        dwordCount -= 2;
    }

    // Write the final dword if one exists
    if (dwordCount > 0)
    {
        const uint64 writeData = (pData != nullptr) ? *pData : 0;

        WriteImmediateSingle(destVa, writeData, ImmediateDataWidth::ImmediateData32Bit);
    }
}

// =====================================================================================================================
// This encodes the float bits in such a way that the shader can do InterlockedMin/Max on floats
uint32 FloatToUintForCompare(
    float v)
{
    const uint32 bitShift = 31;
    const uint32 bitMask = 0x80000000;

    uint32 ui = *reinterpret_cast<uint32*>(&v);
    ui ^= (1 + ~(ui >> bitShift) | bitMask);

    return ui;
}

// =====================================================================================================================
// Initialize an acceleration structure into a valid state to begin building
void BvhBuilder::InitAccelerationStructure()
{
    RGP_PUSH_MARKER("Init Acceleration Structure");

    if (m_buildConfig.needEncodeDispatch && (m_buildConfig.maxNumPrimitives > 0))
    {
        const uint32 InitialMax = FloatToUintForCompare(-FLT_MAX);
        const uint32 InitialMin = FloatToUintForCompare(FLT_MAX);

        const GpuRt::gpusize sceneBoundVa = RemappedScratchBufferBaseVa() + m_scratchOffsets.sceneBounds;
        if (m_buildConfig.rebraidType == RebraidType::V2)
        {
            uint32 sceneBounds[] =
            {
                InitialMin, InitialMin, InitialMin,
                InitialMax, InitialMax, InitialMax,
                InitialMin, InitialMax, // size

                InitialMin, InitialMin, InitialMin, //used for rebraid
                InitialMax, InitialMax, InitialMax,
            };

            WriteImmediateData(sceneBoundVa, sceneBounds);
        }
        else
        {
            uint32 sceneBounds[] =
            {
                InitialMin, InitialMin, InitialMin,
                InitialMax, InitialMax, InitialMax,
                InitialMin, InitialMax, // size
            };

            WriteImmediateData(sceneBoundVa, sceneBounds);
        }

        if ((m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair) &&
            (m_buildConfig.enableEarlyPairCompression == false))
        {
            const gpusize numBatchesVa = ScratchBufferBaseVa() + m_scratchOffsets.numBatches;
            ZeroDataImmediate(numBatchesVa, 1);
        }
    }

    // Merged encode/build writes the header using the shader. However, we don't launch the build shader in the case
    // of an empty BVH.
    if (m_buildConfig.needEncodeDispatch || (m_buildConfig.maxNumPrimitives == 0))
    {
        WriteImmediateData(HeaderBufferBaseVa(), InitAccelStructMetadataHeader());
        WriteImmediateData(ResultBufferBaseVa(), InitAccelStructHeader());
    }
    else
    {
        ResetTaskCounter(HeaderBufferBaseVa());
    }

    // Now Init the Build debug counters
    if (m_deviceSettings.enableBVHBuildDebugCounters)
    {
        const gpusize counterPtrVa = ScratchBufferBaseVa() + m_scratchOffsets.debugCounters;
        ZeroDataImmediate(counterPtrVa, RayTracingBuildDebugCounters);
    }

    // Early triangle pairing, triangle splitting and indirect BLAS builds dynamically increment
    // primitive reference counter. Initialise counters to 0 when these features are enabled.
    const bool dynamicallyIncrementsPrimRefCount =
        m_buildConfig.enableEarlyPairCompression || m_buildConfig.triangleSplitting || m_buildSettings.isIndirectBuild;
    const uint32 primRefInitCount =
        (dynamicallyIncrementsPrimRefCount) ? 0 : m_buildConfig.numPrimitives;

    // Note, initialising unused encode task counters via CP has a measurable overhead in the build parallel path. Multi-dispatch
    // initialises the counters via a CS with zero impact.
    if (m_deviceSettings.enableParallelBuild)
    {
        const EncodeTaskCountersBuildParallel encodeTaskCounters = {
            .numPrimitives = 0,
            // BLAS increment primRefs during encode step to enable indirect builds of less primitives than maxPrimitiveCount
            .primRefs = primRefInitCount,
        };

        const gpusize encodeTaskCountersOffset = ScratchBufferBaseVa() + m_scratchOffsets.encodeTaskCounter;
        WriteImmediateData(encodeTaskCountersOffset, encodeTaskCounters);
    }
    else
    {
        const EncodeTaskCounters encodeTaskCounters = {
            .numPrimitives = 0,
            // BLAS increment primRefs during encode step to enable indirect builds of less primitives than maxPrimitiveCount
            .primRefs = primRefInitCount,
            .groupCountX = 0,
            .groupCountY = 1,
            .groupCountZ = 1,
        };

        const gpusize encodeTaskCountersOffset = ScratchBufferBaseVa() + m_scratchOffsets.encodeTaskCounter;
        WriteImmediateData(encodeTaskCountersOffset, encodeTaskCounters);
    }

    const gpusize taskLoopCountersOffset = ScratchBufferBaseVa() + m_scratchOffsets.taskLoopCounters;
    ZeroDataImmediate(taskLoopCountersOffset, TASK_LOOP_COUNTERS_NUM_DWORDS);

    ResetTaskQueueCounters(m_scratchOffsets.rebraidTaskQueueCounter);
    ResetTaskQueueCounters(m_scratchOffsets.triangleSplitTaskQueueCounter);
    ResetTaskQueueCounters(m_scratchOffsets.tdTaskQueueCounter);
    ResetTaskQueueCounters(m_scratchOffsets.plocTaskQueueCounter);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Reset the task counter in the metadata header to 0
void BvhBuilder::ResetTaskCounter(
    gpusize metadataHeaderGpuVa)
{
    const gpusize taskCounterVa =
        metadataHeaderGpuVa + offsetof(AccelStructMetadataHeader, taskCounter);

    // Reset taskCounter and numTasksDone
    ZeroDataImmediate(taskCounterVa, 2);
}

// =====================================================================================================================
// Reset the taskQueue build counters
void BvhBuilder::ResetTaskQueueCounters(
    uint32 offset)
{
    // Task queue counters packed at the beginning of PLOCState or TDState struct
    if (offset != 0xffffffff)
    {
        const gpusize taskCounterVa = ScratchBufferBaseVa() + offset;

        ZeroDataImmediate(taskCounterVa, RayTracingTaskQueueCounters);
    }
}

#if GPURT_DEVELOPER
// =====================================================================================================================
template<class ...Args>
void BvhBuilder::PushRGPMarker(
    const char* pFormat,
    Args&&...   args)
{
    if (Util::TestAnyFlagSet(uint32(m_deviceSettings.rgpMarkerGranularityFlags),
                             uint32(RgpMarkerGranularityFlags::PerBuild)))
    {
        m_pDevice->PushRGPMarker(m_cmdBuffer, pFormat, std::forward<Args>(args)...);
    }
}

// =====================================================================================================================
void BvhBuilder::PopRGPMarker()
{
    if (Util::TestAnyFlagSet(uint32(m_deviceSettings.rgpMarkerGranularityFlags),
                             uint32(RgpMarkerGranularityFlags::PerBuild)))
    {
        m_pDevice->PopRGPMarker(m_cmdBuffer);
    }
}

// =====================================================================================================================
const char* BvhBuilder::ConvertBuildModeToString()
{
    static_assert(static_cast<uint32>(BvhBuildMode::Linear)   == 0, "BvhBuildMode enum mismatch");
    static_assert(static_cast<uint32>(BvhBuildMode::PLOC)     == 2, "BvhBuildMode enum mismatch");
    static_assert(static_cast<uint32>(BvhBuildMode::Auto)     == 4, "BvhBuildMode enum mismatch");

    constexpr const char* BuildModeStr[] =
    {
        "LBVH",     // BvhBuildMode::Linear,
        "Reserved",
        "PLOC",     // BvhBuildMode::PLOC,
        "Reserved",
        "Auto",     // BvhBuildMode::Auto,
    };

    static_assert(GPURT_ARRAY_SIZE(BuildModeStr) == static_cast<uint32>(BvhBuildMode::Count),
            "Mismatched array. Must match BvhBuildMode::Count");
    return BuildModeStr[static_cast<uint32>(m_buildConfig.buildMode)];
}

// =====================================================================================================================
const char* BvhBuilder::ConvertRebraidTypeToString()
{
    static_assert(static_cast<uint32>(RebraidType::Off) == 0, "RebraidType enum mismatch");
    static_assert(static_cast<uint32>(RebraidType::V1)  == 1, "RebraidType enum mismatch");
    static_assert(static_cast<uint32>(RebraidType::V2)  == 2, "RebraidType enum mismatch");

    constexpr const char* RebraidTypeStr[] =
    {
        "Off", // RebraidTypeOff,
        "V1",  // RebraidTypeV1,
        "V2",  // RebraidTypeV2,
    };

    static_assert(GPURT_ARRAY_SIZE(RebraidTypeStr) == static_cast<uint32>(RebraidType::Count),
        "Mismatched array. Must match RebraidType::Count");
    return RebraidTypeStr[static_cast<uint32>(m_buildConfig.rebraidType)];
}

// =====================================================================================================================
const char* BvhBuilder::ConvertTriCompressionTypeToString()
{
    static_assert(static_cast<uint32>(TriangleCompressionMode::None) == 0,
        "TriangleCompressionMode enum mismatch");
    static_assert(static_cast<uint32>(TriangleCompressionMode::Reserved) == 1,
        "TriangleCompressionMode enum mismatch");
    static_assert(static_cast<uint32>(TriangleCompressionMode::Pair) == 2,
        "TriangleCompressionMode enum mismatch");

    constexpr const char* TriCompressionStr[] =
    {
        "None", // NoTriangleCompression,
        "Reserved",
        "Pair", // PairTriangleCompression,
    };

    static_assert(GPURT_ARRAY_SIZE(TriCompressionStr) == static_cast<uint32>(TriangleCompressionMode::Count),
        "Mismatched array. Must match TriangleCompressionMode::Count");
    return TriCompressionStr[static_cast<uint32>(m_buildConfig.triangleCompressionMode)];
}

// =====================================================================================================================
const char* BvhBuilder::ConvertFp16ModeToString()
{
    static_assert(static_cast<uint32>(Fp16BoxNodesInBlasMode::NoNodes)    == 0,
        "Fp16BoxNodesInBlasMode enum mismatch");
    static_assert(static_cast<uint32>(Fp16BoxNodesInBlasMode::LeafNodes)  == 1,
        "Fp16BoxNodesInBlasMode enum mismatch");
    static_assert(static_cast<uint32>(Fp16BoxNodesInBlasMode::MixedNodes) == 2,
        "Fp16BoxNodesInBlasMode enum mismatch");
    static_assert(static_cast<uint32>(Fp16BoxNodesInBlasMode::AllNodes)   == 3,
        "Fp16BoxNodesInBlasMode enum mismatch");

    // Fp16 mode is only available for bottomLevel
    constexpr const char* Fp16ModeStr[] =
    {
        "NoNodes",    // Fp16BoxNodesInBlasMode::NoNodes,
        "LeafNodes",  // Fp16BoxNodesInBlasMode::LeafNodes,
        "MixedNodes", // Fp16BoxNodesInBlasMode::MixedNodes,
        "AllNodes",   // Fp16BoxNodesInBlasMode::AllNodes,
    };

    static_assert(GPURT_ARRAY_SIZE(Fp16ModeStr) == static_cast<uint32>(Fp16BoxNodesInBlasMode::Count),
        "Mismatched array. Must match Fp16BoxNodesInBlasMode::Count");

    return Fp16ModeStr[static_cast<uint32>(m_buildConfig.fp16BoxNodesInBlasMode)];
}

// =====================================================================================================================
void BvhBuilder::OutputBuildInfo()
{
    constexpr uint32 MaxStrLength     = 1024;
    char buildShaderInfo[MaxStrLength];

    Util::Snprintf(buildShaderInfo, MaxStrLength, "BVH Build Settings:%s",
        m_buildConfig.topLevelBuild ? "TopLevel" : "BottomLevel");

    constexpr uint32 MaxInfoStrLength = 128;
    char buildModeString[MaxInfoStrLength];
    Util::Snprintf(buildModeString, MaxInfoStrLength, ", BuildMode:%s",
        m_buildConfig.topDownBuild ? "TopDown" : ConvertBuildModeToString());
    Util::Strncat(buildShaderInfo, MaxInfoStrLength, buildModeString);

    if (m_buildSettings.rebraidType != static_cast<uint32>(RebraidType::Off))
    {
        char infoString[MaxInfoStrLength];
        Util::Snprintf(infoString, MaxInfoStrLength, ", RebraidType:%s", ConvertRebraidTypeToString());
        Util::Strncat(buildShaderInfo, MaxInfoStrLength, infoString);
    }

    if (m_buildConfig.fp16BoxNodesInBlasMode != Fp16BoxNodesInBlasMode::NoNodes)
    {
        char infoString[MaxInfoStrLength];
        Util::Snprintf(infoString, MaxInfoStrLength, ", FP16Mode:%s", ConvertFp16ModeToString());
        Util::Strncat(buildShaderInfo, MaxInfoStrLength, infoString);
    }

    if (m_buildConfig.triangleCompressionMode != TriangleCompressionMode::None)
    {
        char infoString[MaxInfoStrLength];
        Util::Snprintf(infoString, MaxInfoStrLength, ", TriangleCompressionMode:%s",
            ConvertTriCompressionTypeToString());
        Util::Strncat(buildShaderInfo, MaxInfoStrLength, infoString);
    }

    if (m_buildSettings.doTriangleSplitting > 0)
    {
        char infoString[MaxInfoStrLength];
        Util::Strncat(buildShaderInfo, MaxInfoStrLength, ", TriangleSplitting");
        Util::Snprintf(infoString, MaxInfoStrLength, ", TriangleSplittingBudgetPerTriangle:%d, TriangleSplittingPriority:%f",
            m_deviceSettings.tsBudgetPerTriangle, m_deviceSettings.tsPriority);
        Util::Strncat(buildShaderInfo, MaxInfoStrLength, infoString);
    }

    if (m_buildConfig.enableMergeSort)
    {
        Util::Strncat(buildShaderInfo, MaxInfoStrLength, ", MergeSort");
    }
    else
    {
        char infoString[MaxInfoStrLength];
        if (m_buildConfig.radixSortScanLevel == 0)
        {
            Util::Snprintf(infoString, MaxInfoStrLength, ", RadixSort:PrefixScanDLB");
        }
        else
        {
            char infoString[MaxInfoStrLength];
            Util::Snprintf(infoString, MaxInfoStrLength, ", RadixSort:ScanLevel%d",
                m_buildConfig.radixSortScanLevel);
        }
        Util::Strncat(buildShaderInfo, MaxInfoStrLength, infoString);
    }
    m_backend.CommentString(m_cmdBuffer, buildShaderInfo);
}

// =====================================================================================================================
void BvhBuilder::OutputPipelineName(
    InternalRayTracingCsType type)
{
    m_pDevice->OutputPipelineName(m_cmdBuffer, type);
}
#endif

// =====================================================================================================================
// Initalize the compile time constant buffer containing settings for the build shaders.
void BvhBuilder::InitBuildSettings()
{
    m_buildSettings = {};

    const BvhBuildMode buildMode = m_buildConfig.buildMode;

    m_buildSettings.geometryType                 = static_cast<uint32>(m_buildConfig.geometryType);
    m_buildSettings.topLevelBuild                = (m_buildArgs.inputs.type == AccelStructType::TopLevel);
    m_buildSettings.buildMode                    = static_cast<uint32>(buildMode);
    m_buildSettings.triangleCompressionMode      = static_cast<uint32>(m_buildConfig.triangleCompressionMode);
    m_buildSettings.isIndirectBuild              = m_buildArgs.indirect.indirectGpuAddr > 0;
    m_buildSettings.doTriangleSplitting          = m_buildConfig.triangleSplitting;
    m_buildSettings.fp16BoxNodesMode             = (m_buildSettings.topLevelBuild) ?
                                                   static_cast<uint32>(Fp16BoxNodesInBlasMode::NoNodes) :
                                                   static_cast<uint32>(m_buildConfig.fp16BoxNodesInBlasMode);
    m_buildSettings.fp16BoxModeMixedSaThreshold  = m_deviceSettings.fp16BoxModeMixedSaThresh;
    m_buildSettings.enableBVHBuildDebugCounters  = m_deviceSettings.enableBVHBuildDebugCounters;
    m_buildSettings.plocRadius                   = m_deviceSettings.plocRadius;
    m_buildSettings.enablePairCostCheck          = m_deviceSettings.enablePairCompressionCostCheck;
    m_buildSettings.enableVariableBitsMortonCode = m_deviceSettings.enableVariableBitsMortonCodes;

    m_buildSettings.rebraidType                  = static_cast<uint32>(m_buildConfig.rebraidType);
    m_buildSettings.enableTopDownBuild           = m_buildConfig.topDownBuild;
    m_buildSettings.useMortonCode30              = m_deviceSettings.enableMortonCode30;
    m_buildSettings.fastBuildThreshold           = m_deviceSettings.fastBuildThreshold;
    m_buildSettings.enableFusedInstanceNode      = m_deviceSettings.enableFusedInstanceNode;
    m_buildSettings.enableMergeSort              = m_buildConfig.enableMergeSort;

    // m_buildConfig.rebraidType is only enabled on TLAS builds, as result we need a separate compile time
    // setting to enable rebraid support in BLAS build shaders.
    if ((m_deviceSettings.rebraidType != RebraidType::Off) && (m_buildSettings.topLevelBuild == 0))
    {
        // Enable instance rebraid support in build shaders. Note, currently this only enables additional
        // code paths in BLAS build shaders.
        m_buildSettings.enableInstanceRebraid = 1;
    }

    m_buildSettings.tsPriority                   = m_deviceSettings.tsPriority;
    // Force priority to 1 if the client set it to 0
    if (m_buildSettings.tsPriority <= 0.f)
    {
        m_buildSettings.tsPriority = 1.0f;
    }

    m_buildSettings.numRebraidIterations    = Util::Max(1u, m_deviceSettings.numRebraidIterations);
    m_buildSettings.rebraidQualityHeuristic = m_deviceSettings.rebraidQualityHeuristic;
    m_buildSettings.radixSortScanLevel      = m_buildConfig.radixSortScanLevel;

    m_buildSettings.enableEarlyPairCompression = m_buildConfig.enableEarlyPairCompression;
    m_buildSettings.enableFastLBVH      = m_buildConfig.enableFastLBVH;

    m_buildSettings.rtIpLevel = static_cast<uint32>(m_pDevice->GetRtIpLevel());

    uint32 emitBufferCount = 0;
    for (uint32 i = 0; i < m_buildArgs.postBuildInfoDescCount; ++i)
    {
        AccelStructPostBuildInfo args = m_clientCb.pfnConvertAccelStructPostBuildInfo(m_buildArgs, i);
        if (args.desc.infoType == AccelStructPostBuildInfoType::CompactedSize)
        {
            // Cache emit destination GPU VA for inlined emit from build shaders
            m_emitCompactDstGpuVa = args.desc.postBuildBufferAddr.gpu;
            emitBufferCount++;
        }
    }

    if (emitBufferCount == 1)
    {
        // We only support one compacted emit size from the build shaders. If we have more than one emit
        // destination buffers, we use the compute shader path
        m_buildSettings.emitCompactSize = 1;
    }

    m_buildSettings.doEncode = (m_buildConfig.needEncodeDispatch == false);

    m_buildSettings.gpuDebugFlags = m_deviceSettings.gpuDebugFlags;

    m_buildSettings.updateFlags =
        m_buildArgs.inputs.flags & (AccelStructBuildFlagPerformUpdate | AccelStructBuildFlagAllowUpdate);

    if (ForceRebuild())
    {
        m_buildSettings.updateFlags &= ~AccelStructBuildFlagPerformUpdate;
    }

    m_buildSettings.isUpdateInPlace = IsUpdateInPlace();
    m_buildSettings.encodeArrayOfPointers =
        (m_buildArgs.inputs.inputElemLayout == InputElementLayout::ArrayOfPointers);
    m_buildSettings.sceneBoundsCalculationType = static_cast<uint32>(m_buildConfig.sceneCalcType);

    m_buildSettingsHash = m_backend.HashBuildSettings(m_buildSettings);

#if GPURT_DEVELOPER
    OutputBuildInfo();
#endif
}

// =====================================================================================================================
// Initalize the compile time constant buffer containing settings for the copy shaders.
void BvhBuilder::InitCopySettings()
{
    m_buildSettings = {};

    m_buildSettings.enableFusedInstanceNode = m_deviceSettings.enableFusedInstanceNode;

    m_buildSettings.rtIpLevel = static_cast<uint32>(m_pDevice->GetRtIpLevel());

    m_buildSettings.gpuDebugFlags = m_deviceSettings.gpuDebugFlags;

    m_buildSettingsHash = m_backend.HashBuildSettings(m_buildSettings);
}

// =====================================================================================================================
// Gets prebuild information about the acceleration structure to be built eventually.
void BvhBuilder::GetAccelerationStructurePrebuildInfo(
    const AccelStructBuildInputs& buildInfo,     // Build args
    AccelStructPrebuildInfo*      pPrebuildInfo) // Output prebuild info struct
{
    m_buildArgs.inputs = buildInfo;
    InitBuildConfig(m_buildArgs);

    AccelStructPrebuildInfo prebuildInfo = {};

    // Calculate the amount of space needed to store the result.
    const ResultBufferInfo resultBufferInfo = CalculateResultBufferInfo(nullptr, nullptr);
    const uint32 resultDataSize = resultBufferInfo.dataSize;

    // Calculate the amount of scratch space needed during the construction process.
    const ScratchBufferInfo scratchBufferInfo = CalculateScratchBufferInfo(nullptr);
    uint32 scratchDataSize = CalculateScratchBufferSize(resultBufferInfo, scratchBufferInfo);

    uint32 updateDataSize = 0;
    if (Util::TestAnyFlagSet(buildInfo.flags, AccelStructBuildFlagAllowUpdate))
    {
        updateDataSize = CalculateUpdateScratchBufferInfo(nullptr);
    }
    // Scratch size for builds may be smaller than updates, some apps will still try to use the scratch size from
    // the build when performing the update causing page faults.
    scratchDataSize = Util::Max(scratchDataSize, updateDataSize);

    prebuildInfo.scratchDataSizeInBytes       = scratchDataSize;
    prebuildInfo.updateScratchDataSizeInBytes = updateDataSize;

    // @note We set the ScratchData and UpdateScratchData size to 1 instead of 0, because some apps crash otherwise.
    prebuildInfo.scratchDataSizeInBytes =
        Util::Max(static_cast<uint64>(1u), prebuildInfo.scratchDataSizeInBytes);
    prebuildInfo.updateScratchDataSizeInBytes =
        Util::Max(static_cast<uint64>(1u), prebuildInfo.updateScratchDataSizeInBytes);

    prebuildInfo.resultDataMaxSizeInBytes = resultDataSize;
    prebuildInfo.maxPrimitiveCount        = m_buildConfig.maxNumPrimitives;

    // The reported size may legally be used for a build with fewer input elements. It's possible that a build with
    // fewer inputs produces a larger size if we disabled top down builds (especially with rebraid) due to the input
    // element count. We must make sure to report the max size of each possible build type in this case.
    if (m_buildConfig.allowTopDownBuild && (m_buildConfig.topDownBuild == false))
    {
        PAL_ASSERT(buildInfo.type == AccelStructType::TopLevel);
        PAL_ASSERT(buildInfo.inputElemCount > m_deviceSettings.maxTopDownBuildInstances);

        AccelStructPrebuildInfo alternatePrebuildInfo = {};
        AccelStructBuildInputs  alternateBuildInfo    = buildInfo;

        alternateBuildInfo.inputElemCount = m_deviceSettings.maxTopDownBuildInstances;

        GetAccelerationStructurePrebuildInfo(alternateBuildInfo, &alternatePrebuildInfo);

        prebuildInfo.resultDataMaxSizeInBytes =
            Util::Max(prebuildInfo.resultDataMaxSizeInBytes, alternatePrebuildInfo.resultDataMaxSizeInBytes);
        prebuildInfo.scratchDataSizeInBytes =
            Util::Max(prebuildInfo.scratchDataSizeInBytes, alternatePrebuildInfo.scratchDataSizeInBytes);
        prebuildInfo.updateScratchDataSizeInBytes =
            Util::Max(prebuildInfo.updateScratchDataSizeInBytes, alternatePrebuildInfo.updateScratchDataSizeInBytes);
        prebuildInfo.maxPrimitiveCount =
            Util::Max(prebuildInfo.maxPrimitiveCount, alternatePrebuildInfo.maxPrimitiveCount);
    }

    *pPrebuildInfo = prebuildInfo;
}

// =====================================================================================================================
// Builds or updates an acceleration structure by executing several shaders
void BvhBuilder::BuildRaytracingAccelerationStructure()
{
    if (m_deviceSettings.enableInsertBarriersInBuildAS == true)
    {
        // Intentionally insert Barrier() for debugging purpose
        Barrier();
    }

    const uint32 resultDataSize = m_resultBufferInfo.dataSize;

    if (IsUpdate())
    {
        RGP_PUSH_MARKER(
            "Update%sLevelAccelerationStructure(NumDescs=%u, NumPrims=%u, Flags=%u, SizeInBytes=%u, NumEmits=%u)",
            m_buildConfig.topLevelBuild ? "Top" : "Bottom",
            m_buildArgs.inputs.inputElemCount,
            m_buildConfig.numPrimitives,
            m_buildArgs.inputs.flags,
            resultDataSize,
            m_buildArgs.postBuildInfoDescCount);
    }
    else
    {
        RGP_PUSH_MARKER(
            "Build%sLevelAccelerationStructure(NumDescs=%u, NumPrims=%u, Flags=%u, SizeInBytes=%u, NumEmits=%u)",
            m_buildConfig.topLevelBuild ? "Top" : "Bottom",
            m_buildArgs.inputs.inputElemCount,
            m_buildConfig.numPrimitives,
            m_buildArgs.inputs.flags,
            resultDataSize,
            m_buildArgs.postBuildInfoDescCount);
    }

    if (IsUpdate())
    {
        // Reset update stack pointer and update task counters.
        ZeroDataImmediate(ScratchBufferBaseVa(), 4);
    }
    else
    {
        InitAccelerationStructure();
    }
    Barrier(BarrierFlagSyncPostCpWrite);

    if (HasBuildDumpEvents())
    {
        PreBuildDumpEvents();
    }

    if (m_buildConfig.maxNumPrimitives > 0)
    {
        if (m_buildSettings.enableEarlyPairCompression)
        {
            EncodeQuadPrimitives();
            Barrier();
        }
        else if (m_buildConfig.needEncodeDispatch)
        {
            EncodePrimitives();
        }

        if (m_buildConfig.numPrimitives != 0)
        {
            // Build or update the acceleration structure
            if (IsUpdate() == false)
            {
                // Build a new BVH
                BuildAccelerationStructure();
            }
            else
            {
                UpdateAccelerationStructure();
            }
        }
    }

    if (m_buildArgs.postBuildInfoDescCount > 0)
    {
        if (NeedsPostBuildEmitPass())
        {
            // Make sure build is complete before emitting
            Barrier();
        }

        EmitPostBuildInfo();
    }

    if (HasBuildDumpEvents())
    {
        PostBuildDumpEvents();
    }

    if (m_deviceSettings.enableInsertBarriersInBuildAS == true)
    {
        // Intentionally insert Barrier() for debugging purpose
        Barrier();
    }

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Handles pre-build dump event invocation
void BvhBuilder::PreBuildDumpEvents()
{
    PAL_ASSERT(HasBuildDumpEvents());

    const ResultBufferInfo resultBufferInfo = CalculateResultBufferInfo(nullptr, nullptr);
    const uint32 resultDataSize = resultBufferInfo.dataSize;

    // Initialise acceleration structure information for dump purposes
    m_dumpInfo = {};

    m_dumpInfo.type                    = m_buildArgs.inputs.type;
    m_dumpInfo.numDesc                 = m_buildArgs.inputs.inputElemCount;
    m_dumpInfo.numPrimitives           = m_buildConfig.numPrimitives;
    m_dumpInfo.buildFlags              = m_buildArgs.inputs.flags;
    m_dumpInfo.buildType               = AccelStructBuilderType::Gpu;
    m_dumpInfo.buildMode               = static_cast<uint32>(m_buildConfig.buildMode);
    m_dumpInfo.triangleCompressionMode = m_buildConfig.triangleCompressionMode;
    m_dumpInfo.fp16BoxNodesInBlasMode  = m_buildConfig.fp16BoxNodesInBlasMode;
    m_dumpInfo.gpuVa                   = HeaderBufferBaseVa();
    m_dumpInfo.sizeInBytes             = resultDataSize;
    m_dumpInfo.scratchGpuVa            = ScratchBufferBaseVa();
    m_dumpInfo.pTimeStampVidMem        = nullptr;
    m_dumpInfo.timeStampVidMemoffset   = 0;

    if (IsUpdate() == false)
    {
        const ScratchBufferInfo scratchBufferInfo = CalculateScratchBufferInfo(nullptr);
        m_dumpInfo.scratchSizeInBytes = CalculateScratchBufferSize(resultBufferInfo, scratchBufferInfo);
    }
    else
    {
        m_dumpInfo.scratchSizeInBytes = CalculateUpdateScratchBufferInfo(nullptr);
    }

    if (m_deviceSettings.enableBuildAccelStructStats)
    {
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION >= 39
        Pal::Result result = m_clientCb.pfnAccelStatsBuildDumpEvent(m_cmdBuffer, &m_dumpInfo);
#else
        Pal::Result result =
            m_clientCb.pfnAccelStatsBuildDumpEvent(
                m_cmdBuffer, m_dumpInfo, &m_dumpInfo.pTimeStampVidMem, &m_dumpInfo.timeStampVidMemoffset);
#endif

        if (result == Pal::Result::Success)
        {
            m_backend.WriteTimestamp(m_cmdBuffer,
                                     HwPipePoint::HwPipeBottom,
                                     *m_dumpInfo.pTimeStampVidMem,
                                     m_dumpInfo.timeStampVidMemoffset);
        }
    }
}
// =====================================================================================================================
// Handles post-build dump event invocation
void BvhBuilder::PostBuildDumpEvents()
{
    PAL_ASSERT(HasBuildDumpEvents());

    if (m_deviceSettings.enableBuildAccelStructStats)
    {
        if (m_dumpInfo.pTimeStampVidMem != nullptr)
        {
            m_backend.WriteTimestamp(m_cmdBuffer,
                                     HwPipePoint::HwPipeBottom,
                                     *m_dumpInfo.pTimeStampVidMem,
                                     m_dumpInfo.timeStampVidMemoffset + sizeof(uint64));
        }
    }

    // Dump Acceleration Structure
    if (m_deviceSettings.enableBuildAccelStructDumping)
    {
        AccelStructInfo dumpInfo = m_dumpInfo;

        const uint32 resultDataSize = dumpInfo.sizeInBytes;
        uint64 allocationSize = resultDataSize;

        if (m_deviceSettings.enableBuildAccelStructScratchDumping)
        {
            allocationSize += dumpInfo.scratchSizeInBytes;
        }
        else
        {
            // Don't allocate for scratch memory if we're not dumping it.
            dumpInfo.scratchSizeInBytes = 0;
        }

        // This assert will fail on some really large cases like san_miguel when also dumping scratch.
        // Attempting to dump more than 4GB will fail when it reaches PAL.
        // Not expected to be a problem with any actual titles.
        PAL_ASSERT(allocationSize <= UINT32_MAX);

        Barrier();

        gpusize dumpGpuVirtAddr = 0;
        Pal::Result result =
            m_clientCb.pfnAccelStructBuildDumpEvent(
                m_cmdBuffer, dumpInfo, m_buildArgs, &dumpGpuVirtAddr);

        if (result == Pal::Result::Success)
        {
            m_pDevice->CopyBufferRaw(
                m_cmdBuffer,
                dumpGpuVirtAddr,
                HeaderBufferBaseVa(),
                resultDataSize >> 2);

            if (m_deviceSettings.enableBuildAccelStructScratchDumping)
            {
                m_pDevice->CopyBufferRaw(m_cmdBuffer,
                    dumpGpuVirtAddr + resultDataSize,
                    ScratchBufferBaseVa(),
                    dumpInfo.scratchSizeInBytes >> 2);

                // Upload the scratch buffer offsets data to the reserved portion of the scratch buffer
                m_pDevice->UploadCpuMemory(m_cmdBuffer,
                    dumpGpuVirtAddr + resultDataSize,
                    &m_scratchOffsets,
                    sizeof(m_scratchOffsets));
            }
        }
    }
}

// =====================================================================================================================
// Initializes build configuration members
void BvhBuilder::InitializeBuildConfigs()
{
    // Initialize/Update BuildConfig
    InitBuildConfig(m_buildArgs);
    InitBuildSettings();

    m_resultBufferInfo = CalculateResultBufferInfo(&m_resultOffsets, &m_metadataSizeInBytes);

    m_scratchBufferInfo = {};
    if (IsUpdate() == false)
    {
        // Compute the offsets into the scratch buffer for all of our scratch resources.
        m_scratchBufferInfo = CalculateScratchBufferInfo(&m_scratchOffsets);
    }
    else
    {
        // Compute the offsets into the scratch buffer for all of our scratch resources.
        CalculateUpdateScratchBufferInfo(&m_scratchOffsets);
    }

    // Add tlas to m_tlas
    if (m_buildArgs.inputs.type == AccelStructType::TopLevel)
    {
        m_pDevice->NotifyTlasBuild(HeaderBufferBaseVa());
    }
}

// =====================================================================================================================
// Prepares the inputs for the primitive encode shaders
void BvhBuilder::EncodeQuadPrimitives()
{
    // Early pair compression passes are only enabled on bottom level re-builds. For updates, we need
    // to run the EncodePrimitives pass. This function must not be called for an Update, which should be using
    // the regular EncodePrimitives() path.
    PAL_ASSERT(IsUpdate() == false);

    PAL_ASSERT(m_buildArgs.inputs.type == AccelStructType::BottomLevel);

    // TODO: handle indirect build argument buffer
    BindPipeline(InternalRayTracingCsType::EncodeQuadNodes);

    const EncodePrimitive::Constants constants =
    {
        .geometryIndex = m_buildConfig.trianglePairBlockCount
    };

    uint32 entryOffset = 0;

    entryOffset = WriteUserDataEntries(&constants, EncodePrimitive::NumEntries, entryOffset);

    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    const uint32 cbvSrdTableGpuVaLo = Util::LowPart(m_geomConstSrdTable);
    entryOffset = WriteUserDataEntries(&cbvSrdTableGpuVaLo, 1, entryOffset);

    const uint32 vbvSrdTableGpuVaLo = Util::LowPart(m_geomBufferSrdTable);
    entryOffset = WriteUserDataEntries(&vbvSrdTableGpuVaLo, 1, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    RGP_PUSH_MARKER("Encode Quad Nodes (NumPrimitives=%u)(NumBlocks=%u)",
                     m_buildConfig.maxNumPrimitives,
                     m_buildConfig.trianglePairBlockCount);

    Dispatch(m_buildConfig.trianglePairBlockCount);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Prepares the inputs for the primitive encode shaders
void BvhBuilder::EncodePrimitives()
{
    const bool isBottomLevel = (m_buildArgs.inputs.type == AccelStructType::BottomLevel);

    if (isBottomLevel)
    {
        const InternalRayTracingCsType pipelineType =
            (m_buildConfig.geometryType == GeometryType::Triangles) ? InternalRayTracingCsType::EncodeTriangleNodes :
                                                                      InternalRayTracingCsType::EncodeAABBNodes;

        BindPipeline(pipelineType);

        constexpr uint32 EncodeRootConstEntryOffset = 0;
        uint32 entryOffset = EncodePrimitive::NumEntries;

        entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

        const uint32 cbvSrdTableGpuVaLo = Util::LowPart(m_geomConstSrdTable);
        entryOffset = WriteUserDataEntries(&cbvSrdTableGpuVaLo, 1, entryOffset);

        const uint32 vbvSrdTableGpuVaLo = Util::LowPart(m_geomBufferSrdTable);
        entryOffset = WriteUserDataEntries(&vbvSrdTableGpuVaLo, 1, entryOffset);

        // Set result and scratch buffers
        entryOffset = WriteBufferBindings(entryOffset);

        // Prepare merged source AABB buffer data from geometry
        for (uint32 geometryIndex = 0; geometryIndex < m_buildArgs.inputs.inputElemCount; ++geometryIndex)
        {
            const Geometry geometry =
                m_clientCb.pfnConvertAccelStructBuildGeometry(m_buildArgs.inputs, geometryIndex);

            const uint32 primitiveCount = GetGeometryPrimCount(geometry);

            // Mixing geometry types within a bottom-level acceleration structure is not allowed.
            PAL_ASSERT(geometry.type == m_buildConfig.geometryType);

            const EncodePrimitive::Constants constants =
            {
                .geometryIndex = geometryIndex,
            };

            WriteUserDataEntries(&constants, EncodePrimitive::NumEntries, EncodeRootConstEntryOffset);

            if (geometry.type == GeometryType::Triangles)
            {
                const bool isIndexed = (geometry.triangles.indexFormat != IndexFormat::Unknown);

                if ((isIndexed == false) || (geometry.triangles.indexBufferAddr.gpu != 0))
                {
                    RGP_PUSH_MARKER("Encode Triangle Nodes (NumPrimitives=%u)", primitiveCount);

                    // Dispatch at least one group to ensure geometry info is written
                    Dispatch(Util::Max(DispatchSize(primitiveCount), 1u));

                    RGP_POP_MARKER();
                }
                else
                {
                    // Application specified a NULL index buffer with non-zero IndexCount and format.
                    // This may be unintentional, but will just waste memory in the destination acceleration
                    // structure.
                    PAL_ALERT_ALWAYS();
                }
            }
            else if (geometry.type == GeometryType::Aabbs)
            {
                RGP_PUSH_MARKER("Encode AABB Nodes (NumPrimitives=%u)", primitiveCount);

                // Dispatch at least one group to ensure geometry info is written
                Dispatch(Util::Max(DispatchSize(primitiveCount), 1u));

                RGP_POP_MARKER();
            }
            else
            {
                // Invalid geometry type
                PAL_ASSERT_ALWAYS();
            }

            if (m_buildSettings.isIndirectBuild)
            {
                // Indirect build needs a Barrier because indirect build reads the GeometryInfo for previous geometries.
                Barrier();
            }
        }
    }
    else
    {
        EncodeInstances(m_buildArgs.inputs.inputElemCount,
                        m_buildArgs.inputs.inputElemLayout);
    }
}

// =====================================================================================================================
// Handles writing any requested postbuild information.
void BvhBuilder::EmitPostBuildInfo()
{
    if (m_buildArgs.postBuildInfoDescCount == 0)
    {
        return;
    }

    const uint32 resultDataSize = m_resultBufferInfo.dataSize;

    const bool isBottomLevel = (m_buildArgs.inputs.type == AccelStructType::BottomLevel);
    const bool useSeparateEmitPass = NeedsPostBuildEmitPass();
    for (uint32 i = 0; i < m_buildArgs.postBuildInfoDescCount; i++)
    {
        const AccelStructPostBuildInfo args = m_clientCb.pfnConvertAccelStructPostBuildInfo(m_buildArgs, i);
        switch (args.desc.infoType)
        {
        case AccelStructPostBuildInfoType::CompactedSize:
            // If maxNumPrimitives == 0, we never execute a BVH build, so we always need a separateEmitPass
            if (useSeparateEmitPass || (m_buildConfig.maxNumPrimitives == 0))
            {
                EmitAccelerationStructurePostBuildInfo(args);
            }
            break;

        case AccelStructPostBuildInfoType::ToolsVisualization:
            {
                uint64 decodedSizeInBytes = RayTracingVisualisationHeaderSize;
                if (m_buildConfig.topLevelBuild)
                {
                    decodedSizeInBytes += sizeof(InstanceDesc) * m_buildConfig.numPrimitives;
                }
                else
                {
                    decodedSizeInBytes += RayTracingGeometryDescSize * m_buildArgs.inputs.inputElemCount;

                    decodedSizeInBytes += RayTracingDecodedLeafDataSize * m_buildConfig.numPrimitives;
                }

                WriteImmediateData(args.desc.postBuildBufferAddr.gpu, decodedSizeInBytes);
            }
            break;

        case AccelStructPostBuildInfoType::Serialization:
            {
                struct SerializationInfo
                {
                    uint64 serializedSizeInBytes;
                    uint64 numBlasPointers;
                } info;

                info.numBlasPointers = (isBottomLevel) ? 0 : m_buildArgs.inputs.inputElemCount;

                info.serializedSizeInBytes = RayTracingSerializedAsHeaderSize +
                                             resultDataSize                   +
                                             (info.numBlasPointers * sizeof(uint64));

                WriteImmediateData(args.desc.postBuildBufferAddr.gpu, info);
            }
            break;

        case AccelStructPostBuildInfoType::CurrentSize:
            WriteImmediateData(args.desc.postBuildBufferAddr.gpu, resultDataSize);
            break;

        default:
            PAL_ASSERT_ALWAYS();
            break;
        }
    }
}

// =====================================================================================================================
// Emits post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void BvhBuilder::EmitAccelerationStructurePostBuildInfo(
    const AccelStructPostBuildInfo& postBuildInfo) // Postbuild info
{
    switch (postBuildInfo.desc.infoType)
    {
    case AccelStructPostBuildInfoType::CurrentSize:
        EmitASCurrentSize(postBuildInfo);
        break;

    case AccelStructPostBuildInfoType::CompactedSize:
        EmitASCompactedType(postBuildInfo);
        break;

    case AccelStructPostBuildInfoType::ToolsVisualization:
        EmitASToolsVisualizationType(postBuildInfo);
        break;

    case AccelStructPostBuildInfoType::Serialization:
        EmitASSerializationType(postBuildInfo);
        break;

    default:
        PAL_ASSERT_ALWAYS();
        break;
    }
}

// =====================================================================================================================
// Emits post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void BvhBuilder::EmitASCurrentSize(
    const AccelStructPostBuildInfo& postBuildInfo) // Postbuild info
{
    uint32 entryOffset = 0;
    BindPipeline(InternalRayTracingCsType::EmitCurrentSize);

    for (uint32 i = 0; i < postBuildInfo.srcAccelStructCount; i++)
    {
        const EmitSize::Constants shaderConstants =
        {
            .offset                 = i * static_cast<uint32>(sizeof(AccelStructPostBuildInfoCurrentSizeDesc)),
            .fp16BoxNodesInBlasMode = static_cast<uint32>(m_deviceSettings.fp16BoxNodesInBlasMode),
        };
        // Set shader constants
        entryOffset = WriteUserDataEntries(&shaderConstants, EmitSize::NumEntries, entryOffset);

        // Set destination buffer
        entryOffset = WriteBufferVa(postBuildInfo.desc.postBuildBufferAddr.gpu, entryOffset);

        // Set Source header
        entryOffset = WriteBufferVa(postBuildInfo.pSrcAccelStructGpuAddrs[i], entryOffset);

        RGP_PUSH_MARKER("Emit Post Build Info (Current)");
        Dispatch(1);

        RGP_POP_MARKER();

        entryOffset = 0;
    }
}

// =====================================================================================================================
// Emits post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void BvhBuilder::EmitASCompactedType(
    const AccelStructPostBuildInfo& postBuildInfo) // Postbuild info
{
    uint32 entryOffset = 0;
    BindPipeline(InternalRayTracingCsType::EmitCompactSize);

    for (uint32 i = 0; i < postBuildInfo.srcAccelStructCount; i++)
    {
        const EmitSize::Constants shaderConstants =
        {
            .offset                 = i * static_cast<uint32>(sizeof(AccelStructPostBuildInfoCompactedSizeDesc)),
            .fp16BoxNodesInBlasMode = static_cast<uint32>(m_deviceSettings.fp16BoxNodesInBlasMode),
        };
        // Set shader constants
        entryOffset = WriteUserDataEntries(&shaderConstants, EmitSize::NumEntries, entryOffset);

        // Set destination buffer
        entryOffset = WriteBufferVa(postBuildInfo.desc.postBuildBufferAddr.gpu, entryOffset);

        // Set Source header
        entryOffset = WriteBufferVa(postBuildInfo.pSrcAccelStructGpuAddrs[i], entryOffset);

        RGP_PUSH_MARKER("Emit Post Build Info (Compacted)");
        Dispatch(1);

        RGP_POP_MARKER();

        entryOffset = 0;
    }
}

// =====================================================================================================================
// Emits post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void BvhBuilder::EmitASToolsVisualizationType(
    const AccelStructPostBuildInfo& postBuildInfo) // Postbuild info
{
    uint32 entryOffset = 0;
    BindPipeline(InternalRayTracingCsType::EmitToolVisDesc);

    for (uint32 i = 0; i < postBuildInfo.srcAccelStructCount; i++)
    {
        const EmitSize::Constants shaderConstants =
        {
            .offset                 = i * static_cast<uint32>(sizeof(AccelStructPostBuildInfoToolsVisualizationDesc)),
            .fp16BoxNodesInBlasMode = static_cast<uint32>(m_deviceSettings.fp16BoxNodesInBlasMode),
        };
        // Set shader constants
        entryOffset = WriteUserDataEntries(&shaderConstants, EmitSize::NumEntries, entryOffset);

        // Set destination buffer
        entryOffset = WriteBufferVa(postBuildInfo.desc.postBuildBufferAddr.gpu, entryOffset);

        // Set source acceleration structure header
        entryOffset = WriteBufferVa(postBuildInfo.pSrcAccelStructGpuAddrs[i], entryOffset);

        RGP_PUSH_MARKER("Emit Post Build Info (Tools/Visualization)");
        Dispatch(1);

        RGP_POP_MARKER();

        entryOffset = 0;
    }
}

// =====================================================================================================================
// Emits post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void BvhBuilder::EmitASSerializationType(
    const AccelStructPostBuildInfo& postBuildInfo) // Postbuild info
{
    uint32 entryOffset = 0;
    BindPipeline(InternalRayTracingCsType::EmitSerializeDesc);

    for (uint32 i = 0; i < postBuildInfo.srcAccelStructCount; i++)
    {
        const EmitSize::Constants shaderConstants =
        {
            .offset                 = i * static_cast<uint32>(sizeof(AccelStructPostBuildInfoSerializationDesc)),
            .fp16BoxNodesInBlasMode = static_cast<uint32>(m_deviceSettings.fp16BoxNodesInBlasMode),
        };
        // Set shader constants
        entryOffset = WriteUserDataEntries(&shaderConstants, EmitSize::NumEntries, entryOffset);

        // Set destination buffer
        entryOffset = WriteBufferVa(postBuildInfo.desc.postBuildBufferAddr.gpu, entryOffset);

        // Set source acceleration structure header
        entryOffset = WriteBufferVa(postBuildInfo.pSrcAccelStructGpuAddrs[i], entryOffset);

        RGP_PUSH_MARKER("Emit Post Build Info (Serialization)");
        Dispatch(1);

        RGP_POP_MARKER();

        entryOffset = 0;
    }
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it to destination memory
void BvhBuilder::CopyAccelerationStructure(
    const AccelStructCopyInfo& copyArgs) // Copy arguments
{
    switch (copyArgs.mode)
    {
    case AccelStructCopyMode::Clone:
        CopyASCloneMode(copyArgs);
        break;

    case AccelStructCopyMode::Compact:
        CopyASCompactMode(copyArgs);
        break;

    case AccelStructCopyMode::Serialize:
        CopyASSerializeMode(copyArgs);
        break;

    case AccelStructCopyMode::Deserialize:
        CopyASDeserializeMode(copyArgs);
        break;

    case AccelStructCopyMode::VisualizationDecodeForTools:
    case AccelStructCopyMode::DriverDecode:
        CopyASToolsVisualizationMode(copyArgs);
        break;

    default:
        PAL_ASSERT_ALWAYS();
        break;
    }
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it to a same sized destination memory
void BvhBuilder::CopyASCloneMode(
    const AccelStructCopyInfo& copyArgs) // Copy arguments
{
    uint32 entryOffset = 0;
    BindPipeline(InternalRayTracingCsType::CopyAS);

    const uint32 numThreadGroups = GetNumThreadGroupsCopy();
    const CopyAS::Constants shaderConstants =
    {
        .numWaves  = numThreadGroups,
        .addressLo = static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu),
        .addressHi = static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu >> 32),
    };

    entryOffset = WriteUserDataEntries(&shaderConstants, CopyAS::NumEntries, entryOffset);

    // Dest
    entryOffset = WriteBufferVa(copyArgs.dstAccelStructAddr.gpu, entryOffset);

    // Src
    entryOffset = WriteBufferVa(copyArgs.srcAccelStructAddr.gpu, entryOffset);

    RGP_PUSH_MARKER("Copy Acceleration Structure (Clone)");
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Compacts a source acceleration structure into a smaller destination memory
void BvhBuilder::CopyASCompactMode(
    const AccelStructCopyInfo& copyArgs) // Copy arguments
{
    uint32 entryOffset = 0;
    BindPipeline(InternalRayTracingCsType::CompactAS);

    const uint32 numThreadGroups = GetNumThreadGroupsCopy();
    const CompactAS::Constants shaderConstants =
    {
        .numThreads = numThreadGroups* DefaultThreadGroupSize,
        .addressLo  = static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu),
        .addressHi  = static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu >> 32)
    };

    entryOffset = WriteUserDataEntries(&shaderConstants, CompactAS::NumEntries, entryOffset);

    // Dest
    entryOffset = WriteBufferVa(copyArgs.dstAccelStructAddr.gpu, entryOffset);

    // Src
    entryOffset = WriteBufferVa(copyArgs.srcAccelStructAddr.gpu, entryOffset);

    RGP_PUSH_MARKER("Copy Acceleration Structure (Compact)");
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it in a serialized manner to the destination memory
void BvhBuilder::CopyASSerializeMode(
    const AccelStructCopyInfo& copyArgs) // Copy arguments
{
    uint32 entryOffset = 0;

    BindPipeline(InternalRayTracingCsType::SerializeAS);

    const uint32 numThreadGroups = GetNumThreadGroupsCopy();
    const SerializeAS::Constants shaderConstants =
    {
        .numWaves = numThreadGroups
    };

    entryOffset = WriteUserDataEntries(&shaderConstants, SerializeAS::NumEntries, entryOffset);

    // Header dst
    entryOffset = WriteBufferVa(copyArgs.dstAccelStructAddr.gpu, entryOffset);

    // Header src
    entryOffset = WriteBufferVa(copyArgs.srcAccelStructAddr.gpu, entryOffset);

    entryOffset = WriteBufferVa(copyArgs.srcAccelStructAddr.gpu, entryOffset);

    RGP_PUSH_MARKER("Copy Acceleration Structure (Serialize)");
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it in a deserialized manner to the destination memory
void BvhBuilder::CopyASDeserializeMode(
    const AccelStructCopyInfo& copyArgs) // Copy arguments
{
    BindPipeline(InternalRayTracingCsType::DeserializeAS);

    const uint64 address = copyArgs.dstAccelStructAddr.gpu;

    const uint32 numThreadGroups = GetNumThreadGroupsCopy();

    const DeserializeAS::Constants shaderConstants =
    {
        .addressLo = static_cast<uint32>(address),
        .addressHi = static_cast<uint32>(address >> 32),
        .numWaves  = numThreadGroups
    };

    // Reset the task counter in destination buffer.
    ResetTaskCounter(copyArgs.dstAccelStructAddr.gpu);
    Barrier(BarrierFlagSyncPostCpWrite);

    uint32 entryOffset = 0;

    entryOffset = WriteUserDataEntries(&shaderConstants, DeserializeAS::NumEntries, entryOffset);

    // Header dst
    entryOffset = WriteBufferVa(copyArgs.dstAccelStructAddr.gpu, entryOffset);

    // Header src
    entryOffset = WriteBufferVa(copyArgs.srcAccelStructAddr.gpu, entryOffset);

    RGP_PUSH_MARKER("Copy Acceleration Structure (Deserialize)");
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it to the destination memory
void BvhBuilder::CopyASToolsVisualizationMode(
    const AccelStructCopyInfo& copyArgs) // Copy arguments
{
    BindPipeline(InternalRayTracingCsType::DecodeAS);

    const uint32 numThreadGroups = GetNumThreadGroupsCopy();

    // Note, build settings are not available for Copy paths.
    const DecodeAS::Constants shaderConstants =
    {
        .addressLo      = static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu),
        .addressHi      = static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu >> 32),
        .numThreads     = numThreadGroups* DefaultThreadGroupSize,
        .isDriverDecode = ((copyArgs.mode == AccelStructCopyMode::DriverDecode) ? 1u : 0u),
    };

    uint32 entryOffset = 0;
    entryOffset = WriteUserDataEntries(&shaderConstants, DecodeAS::NumEntries, entryOffset);

    // Destination buffer
    entryOffset = WriteBufferVa(copyArgs.dstAccelStructAddr.gpu, entryOffset);

    // Source buffer
    entryOffset = WriteBufferVa(copyArgs.srcAccelStructAddr.gpu, entryOffset);

    RGP_PUSH_MARKER("Copy Acceleration Structure (Tools/Visualization)");
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Returns BuildPhaseFlags with all phases executing for this builder enabled
BuildPhaseFlags BvhBuilder::EnabledPhases() const
{
    BuildPhaseFlags flags{};

    if (NeedsPostBuildEmitPass())
    {
        flags |= BuildPhaseFlags::SeparateEmitPostBuildInfoPass;
    }

    if (IsUpdate())
    {
        // Updates do not execute any other build phases
        if (m_buildConfig.needEncodeDispatch)
        {
            flags |= BuildPhaseFlags::EncodePrimitives;
        }
    }
    else
    {
        if (HasBuildDumpEvents())
        {
            flags |= BuildPhaseFlags::BuildDumpEvents;
        }

        // Early pair compression passes are only enabled on bottom level re-builds. For updates, we need
        // to run the EncodePrimitives pass which is handled in the IsUpdate() case above.
        if ((m_buildConfig.topLevelBuild == false) && m_buildConfig.enableEarlyPairCompression)
        {
            flags |= BuildPhaseFlags::EncodeQuadPrimitives;
        }
        else if (m_buildConfig.needEncodeDispatch)
        {
            flags |= BuildPhaseFlags::EncodePrimitives;
        }

        if (m_deviceSettings.enableParallelBuild)
        {
            // BuildParallel does not execute any other build phases
            flags |= BuildPhaseFlags::BuildParallel;
        }
        else
        {
            if (AllowRebraid())
            {
                flags |= BuildPhaseFlags::Rebraid;
            }

            if (m_buildConfig.topDownBuild)
            {
                flags |= BuildPhaseFlags::BuildBVHTD;
                flags |= BuildPhaseFlags::EncodeHwBvh;
                // Top Down Builds do not execute any other build phases
            }
            else
            {
                flags |= BuildPhaseFlags::GenerateMortonCodes;

                if (m_buildConfig.enableMergeSort)
                {
                    flags |= BuildPhaseFlags::MergeSort;
                }
                else
                {
                    flags |= BuildPhaseFlags::RadixSort;
                }

                if ((m_buildConfig.buildMode == BvhBuildMode::Linear)
                    )
                {
                    if (m_buildConfig.enableFastLBVH)
                    {
                        flags |= BuildPhaseFlags::BuildFastAgglomerativeLbvh;
                    }
                    else
                    {
                        flags |= BuildPhaseFlags::BuildBVH;
                        flags |= BuildPhaseFlags::RefitBounds;
                    }
                }
                if (m_buildConfig.buildMode == BvhBuildMode::PLOC)
                {
                    flags |= BuildPhaseFlags::BuildBVHPLOC;
                }
                if (AllowLatePairCompression())
                {
                    flags |= BuildPhaseFlags::PairCompression;
                }

                flags |= BuildPhaseFlags::EncodeHwBvh;

            }
        }
    }

    return flags;
}

// =====================================================================================================================
// Builds an acceleration structure by executing several shaders
void BvhBuilder::BuildAccelerationStructure()
{
    // if rebraid type == v1 then force to non-build parallel for now
    // BuildParallel only supports RebraidType::v2
    if (m_deviceSettings.enableParallelBuild && (m_buildConfig.rebraidType != RebraidType::V1))
    {
        BuildParallel();
    }
    else
    {
        Util::Span<BvhBuilder> builder(this, 1);
        BvhBatcher batcher(m_cmdBuffer, m_backend, m_pDevice, m_deviceProps, m_clientCb, m_deviceSettings);
        batcher.UpdateEnabledPhaseFlags(EnabledPhases());
        batcher.BuildMultiDispatch(builder);
    }
}

// =====================================================================================================================
// Updates an acceleration structure by executing several shaders
void BvhBuilder::UpdateAccelerationStructure()
{
    // Update an existing BVH
    if (m_buildConfig.needEncodeDispatch == false)
    {
        EncodeUpdate();
    }
    else
    {
        if (m_deviceSettings.enableParallelUpdate)
        {
            UpdateParallel();
        }
        else
        {
            // Wait for encode to finish
            Barrier();

            UpdateQBVH();
        }
    }
}

// =====================================================================================================================
// Performs a generic barrier that's used to synchronize internal ray tracing shaders
void BvhBuilder::Barrier(
    uint32 flags)
{
    m_pDevice->RaytracingBarrier(m_cmdBuffer, flags);
}

// =====================================================================================================================
// Executes merge sort shader to sort the input keys and values
void BvhBuilder::MergeSort(
    uint32 wavesPerSimd)
{
    PAL_ASSERT(m_buildConfig.enableMergeSort);

    BindPipeline(InternalRayTracingCsType::MergeSort);

    // Set shader constants
    uint32 entryOffset = 0;
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    RGP_PUSH_MARKER("Merge Sort (maxNumPrimitives %u)", m_buildConfig.maxNumPrimitives);

    const uint32 tgSize          = 512;
    const uint32 numThreadGroups = GetNumPersistentThreadGroups(m_buildConfig.maxNumPrimitives, tgSize, wavesPerSimd);
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Returns true when the builder uses the Rebraid phase
bool BvhBuilder::AllowRebraid() const
{
    const bool enableRebraid = (m_buildConfig.topLevelBuild) &&
                               (m_buildConfig.rebraidType == GpuRt::RebraidType::V2) &&
                               (m_buildConfig.numPrimitives > 0);
    return enableRebraid;
}

// =====================================================================================================================
// Returns true when the builder uses the Late Pair Compression phase
bool BvhBuilder::AllowLatePairCompression() const
{
    const bool enableLatePairCompression = (m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair) &&
                                           (m_buildConfig.enableEarlyPairCompression == false);
    return enableLatePairCompression;
}

// =====================================================================================================================
// Returns true when the builder will require a separate dispatch for emitting build info
bool BvhBuilder::NeedsPostBuildEmitPass() const
{
    const bool usesSeparateEmitPass = (m_buildArgs.postBuildInfoDescCount == 0) &&
                                      (m_emitCompactDstGpuVa != 0) && (m_buildSettings.emitCompactSize == 0);
    return usesSeparateEmitPass;
}

// =====================================================================================================================
// Returns true when the builder has dumping events
bool BvhBuilder::HasBuildDumpEvents() const
{
    const bool hasBuildDumpEvents = m_deviceSettings.enableBuildAccelStructDumping ||
                                    m_deviceSettings.enableBuildAccelStructStats;
    return hasBuildDumpEvents;
}

// =====================================================================================================================
// Executes the Rebraid shader
// Preconditions: Rebraid can only be called on TLAS builds.
void BvhBuilder::Rebraid()
{
    PAL_ASSERT(AllowRebraid());

    // TODO: Determine numThreadGroups without relying on BuildParallel's logic
    const uint32 numThreadGroups = GetParallelBuildNumThreadGroups();

    const Rebraid::Constants shaderConstants =
    {
        .numThreadGroups = numThreadGroups,
    };

    BindPipeline(InternalRayTracingCsType::Rebraid);

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, Rebraid::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    RGP_PUSH_MARKER("Rebraid (numPrims %u)", m_buildConfig.numPrimitives);
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the generates morton codes shader
void BvhBuilder::GenerateMortonCodes()
{
    BindPipeline(InternalRayTracingCsType::GenerateMortonCodes);

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    RGP_PUSH_MARKER("Generate Morton Codes (maxNumPrimitives %u)", m_buildConfig.maxNumPrimitives);
    Dispatch(DispatchSize(m_buildConfig.maxNumPrimitives));

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Binds and dispatches the provided build BVH shader
void BvhBuilder::DispatchBuildBVHPipeline(
    InternalRayTracingCsType pipeline)
{
    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    BindPipeline(pipeline);

    RGP_PUSH_MARKER("Build BVH (maxNumPrimitives %u)", m_buildConfig.maxNumPrimitives);
    Dispatch(DispatchSize(m_buildConfig.maxNumPrimitives));

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the build BVH shader
void BvhBuilder::BuildBVH()
{
    DispatchBuildBVHPipeline(InternalRayTracingCsType::BuildBVH);
}

// =====================================================================================================================
// Executes the build BVH shader
void BvhBuilder::BuildBVHTD()
{
    PAL_ASSERT(m_buildConfig.topDownBuild);

    const uint32 threadGroupSize = DefaultThreadGroupSize;
    const uint32 numThreadGroups = GetNumPersistentThreadGroups(m_buildConfig.maxNumPrimitives, threadGroupSize);

    const BuildBVHTD::Constants shaderConstants =
    {
        .numThreads = numThreadGroups * threadGroupSize,
    };

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, BuildBVHTD::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    if (m_buildConfig.rebraidType == RebraidType::V1)
    {
        BindPipeline(InternalRayTracingCsType::BuildBVHTDTR);
    }
    else
    {
        BindPipeline(InternalRayTracingCsType::BuildBVHTD);
    }

    RGP_PUSH_MARKER("Build BVHTD");
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the build BVH PLOC shader
void BvhBuilder::BuildBVHPLOC(
    uint32 wavesPerSimd)
{
    const uint32 tgSize = 256u;
    const uint32 numThreadGroups = GetNumPersistentThreadGroups(m_buildConfig.maxNumPrimitives, tgSize, wavesPerSimd);

    uint32 entryOffset = 0;

    const BuildBVHPLOC::Constants shaderConstants =
    {
        .numThreads = numThreadGroups * tgSize,
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, BuildBVHPLOC::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    BindPipeline(InternalRayTracingCsType::BuildBVHPLOC);

    RGP_PUSH_MARKER("Build PLOC BVH (maxNumPrimitives %u)", m_buildConfig.maxNumPrimitives);
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the build Fast Agglomerative LBVH shader
void BvhBuilder::BuildFastAgglomerativeLbvh()
{
    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    BindPipeline(InternalRayTracingCsType::BuildFastAgglomerativeLbvh);

    RGP_PUSH_MARKER("Build Fast Agglomerative LBVH (maxNumPrimitives %u)", m_buildConfig.maxNumPrimitives);
    Dispatch(DispatchSize(m_buildConfig.maxNumPrimitives));

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the update QBVH shader
// Refits internal node bounding boxes based on updated geometry
void BvhBuilder::UpdateQBVH()
{
    BindPipeline(InternalRayTracingCsType::UpdateQBVH);

    const uint32 numWorkItems = Util::Max(1u, (m_buildConfig.numPrimitives / 2));
    const Update::Constants shaderConstants =
    {
        .numThreads = numWorkItems,
    };

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, Update::NumEntries, entryOffset);

    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result/scratch/source buffers
    entryOffset = WriteUpdateBuffers(entryOffset);

    RGP_PUSH_MARKER("Update QBVH");

    Dispatch(DispatchSize(numWorkItems));

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the UpdateParallel shader
void BvhBuilder::UpdateParallel()
{
    BindPipeline(InternalRayTracingCsType::UpdateParallel);

    const uint32 numWorkItems     = Util::Max(1u, (m_buildConfig.numPrimitives / 2));
    const uint32 threadGroupSize  = DefaultThreadGroupSize;
    const uint32 wavesPerSimd     = 8;
    const uint32 numThreadGroups  = GetNumPersistentThreadGroups(numWorkItems, threadGroupSize, wavesPerSimd);
    const uint32 numThreads       = numThreadGroups * threadGroupSize;

    uint32 entryOffset = 0;

    const Update::Constants shaderConstants =
    {
        .numThreads = numThreads,
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, Update::NumEntries, entryOffset);

    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result/scratch/source buffers
    entryOffset = WriteUpdateBuffers(entryOffset);

    RGP_PUSH_MARKER("Update Parallel");
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Perform geometry encoding and update in one dispatch
void BvhBuilder::EncodeUpdate()
{
    BindPipeline((m_buildConfig.geometryType == GeometryType::Triangles) ?
        InternalRayTracingCsType::UpdateTriangles : InternalRayTracingCsType::UpdateAabbs);

    const uint32 numWorkItems = Util::Max(1u, m_buildConfig.numPrimitives);

    const uint32 threadGroupSize  = DefaultThreadGroupSize;
    const uint32 wavesPerSimd     = 8;
    const uint32 numThreadGroups  = GetNumPersistentThreadGroups(numWorkItems, threadGroupSize, wavesPerSimd);
    const uint32 numThreads       = numThreadGroups * threadGroupSize;

    uint32 entryOffset = 0;

    const Update::Constants shaderConstants =
    {
        .numThreads = numThreads,
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, Update::NumEntries, entryOffset);

    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result/scratch/source buffers
    entryOffset = WriteUpdateBuffers(entryOffset);

    const uint32 cbvSrdTableGpuVaLo = Util::LowPart(m_geomConstSrdTable);
    entryOffset = WriteUserDataEntries(&cbvSrdTableGpuVaLo, 1, entryOffset);

    const uint32 vbvSrdTableGpuVaLo = Util::LowPart(m_geomBufferSrdTable);
    entryOffset = WriteUserDataEntries(&vbvSrdTableGpuVaLo, 1, entryOffset);

    RGP_PUSH_MARKER("Update");
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Calculates the number of thread groups to dispatch based on the configured waves per SIMD for parallel builds
uint32 BvhBuilder::GetParallelBuildNumThreadGroups()
{
    uint32 wavesPerSimd = m_deviceSettings.parallelBuildWavesPerSimd;

    if (wavesPerSimd == 0)
    {
        switch (m_buildConfig.buildMode)
        {
        case BvhBuildMode::Linear:
            if (m_buildConfig.triangleSplitting)
            {
                wavesPerSimd = 5;
            }
            else
            {
                wavesPerSimd = 8;
            }
            break;
        case BvhBuildMode::PLOC:
            wavesPerSimd = 2;
            break;
        default:
            wavesPerSimd = 1;
            break;
        }
    }

    const uint32 threadGroupSize = DefaultThreadGroupSize;
    const uint32 numThreadGroups = GetNumPersistentThreadGroups(m_buildConfig.numPrimitives, threadGroupSize, wavesPerSimd);

    return numThreadGroups;
}

// =====================================================================================================================
// Executes the EncodeHwBvh shader
void BvhBuilder::EncodeHwBvh()
{
    const uint32 nodeCount       = GetNumInternalNodeCount();
    const uint32 numThreadGroups =
        m_buildSettings.topLevelBuild ? Util::RoundUpQuotient(nodeCount, DefaultThreadGroupSize) :
                                        GetNumPersistentThreadGroups(nodeCount);

    const EncodeHwBvh::Constants shaderConstants =
    {
        numThreadGroups * DefaultThreadGroupSize,
    };

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, EncodeHwBvh::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    {
        BindPipeline(InternalRayTracingCsType::BuildQBVH);
    }

    RGP_PUSH_MARKER("Encode HW BVH (nodeCount %u)", nodeCount);
    Dispatch(numThreadGroups);
    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the refit bounds shader
void BvhBuilder::RefitBounds()
{
    BindPipeline(InternalRayTracingCsType::RefitBounds);

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    RGP_PUSH_MARKER("Refit Bounds");
    Dispatch(DispatchSize(m_buildConfig.maxNumPrimitives));

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the pair compression shader
void BvhBuilder::PairCompression()
{
    PAL_ASSERT(AllowLatePairCompression());

    BindPipeline(InternalRayTracingCsType::PairCompression);

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    const uint32 cbvSrdTableGpuVaLo = Util::LowPart(m_geomConstSrdTable);
    entryOffset = WriteUserDataEntries(&cbvSrdTableGpuVaLo, 1, entryOffset);

    const uint32 vbvSrdTableGpuVaLo = Util::LowPart(m_geomBufferSrdTable);
    entryOffset = WriteUserDataEntries(&vbvSrdTableGpuVaLo, 1, entryOffset);

    RGP_PUSH_MARKER("Pair Compression");
    Dispatch(DispatchSize(m_buildConfig.maxNumPrimitives));

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the clear buffer shader
void BvhBuilder::ClearBuffer(
    gpusize bufferVa,   // GPUVA of the buffer to clear
    uint32  numDwords,  // Number of dwords to clear
    uint32  clearValue) // Value to clear the buffer to
{
    BindPipeline(InternalRayTracingCsType::ClearBuffer);

    uint32 entryOffset = 0;

    const ClearBuffer::Constants shaderConstants =
    {
        .numDwords = numDwords,
        .value     = clearValue
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, ClearBuffer::NumEntries, entryOffset);

    // Set buffer
    entryOffset = WriteBufferVa(bufferVa, entryOffset);

    RGP_PUSH_MARKER("Clear Buffer");
    Dispatch(DispatchSize(numDwords));

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the copy buffer shader
void BvhBuilder::CopyBufferRaw(
    gpusize dstBufferVa,    // Destination buffer GPU VA
    gpusize srcBufferVa,    // Source buffer GPU VA
    uint32  numDwords)      // Number of Dwords to copy
{
    BindPipeline(InternalRayTracingCsType::CopyBufferRaw);

    uint32 entryOffset = 0;

    const CopyBufferRaw::Constants shaderConstants =
    {
        .numDwords = numDwords,
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, CopyBufferRaw::NumEntries, entryOffset);

    // Set buffer addresses
    entryOffset = WriteBufferVa(srcBufferVa, entryOffset);
    entryOffset = WriteBufferVa(dstBufferVa, entryOffset);

    RGP_PUSH_MARKER("Copy Buffer");
    Dispatch(DispatchSize(numDwords));

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the bit histogram shader
void BvhBuilder::BitHistogram(
    uint32 bitShiftSize,      // Number of bits to shift
    uint32 numElems)          // Number of elements
{
    BindPipeline(InternalRayTracingCsType::BitHistogram);

    const uint32 numGroups = Util::RoundUpQuotient(numElems, m_radixSortConfig.groupBlockSize);

    uint32 entryOffset = 0;

    const BitHistogram::Constants shaderConstants =
    {
        .bitShiftSize = bitShiftSize,
        .numGroups    =  numGroups,
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, BitHistogram::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    RGP_PUSH_MARKER("Bit Histogram");
    Dispatch(numGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the scatter keys and values shader
void BvhBuilder::ScatterKeysAndValues(
    uint32 bitShiftSize,       // Number of bits to shift
    uint32 numElems)           // Number of elements
{
    BindPipeline(InternalRayTracingCsType::ScatterKeysAndValues);

    const uint32 numGroups = Util::RoundUpQuotient(numElems, m_radixSortConfig.groupBlockSize);

    uint32 entryOffset = 0;

    const RadixSort::Constants shaderConstants =
    {
        .bitShiftSize = bitShiftSize,
        .numGroups    =  numGroups,
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, RadixSort::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    RGP_PUSH_MARKER("Scatter Keys And Values");
    Dispatch(numGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes multiple passes of several shaders to sort the input keys and values
void BvhBuilder::SortRadixInt32()
{
    uint32 bitShiftSize = 0;
    const uint32 numPasses = m_deviceSettings.enableMortonCode30 ?
                             m_radixSortConfig.numPasses30 :
                             m_radixSortConfig.numPasses;

    for (uint32 passIndex = 0; passIndex < numPasses; ++passIndex)
    {
        // Bit histogram
        BitHistogram(bitShiftSize, m_buildConfig.maxNumPrimitives);

        // Wait for the bit histogram to complete
        Barrier();

        // Scan histograms
        ScanExclusiveAdd(m_buildConfig.numHistogramElements);

        // Wait for the scan operation to complete
        Barrier();

        // Scatter keys
        ScatterKeysAndValues(bitShiftSize, m_buildConfig.maxNumPrimitives);

        // If this isn't the last pass, then issue a barrier before we continue so the passes don't overlap.
        if ((passIndex + 1) < numPasses)
        {
            Barrier();
        }

        bitShiftSize += m_radixSortConfig.bitsPerPass;
    }
}

// =====================================================================================================================
// BVH build implementation with no barriers and a single dispatch.
void BvhBuilder::BuildParallel()
{
    const auto buildParallelShaderType = InternalRayTracingCsType::BuildParallel;
    if (m_buildConfig.radixSortScanLevel > 0)
    {
        PAL_ASSERT(m_buildConfig.numPrimitives < (m_radixSortConfig.numScanElemsPerWorkGroup *
                                                  m_radixSortConfig.numScanElemsPerWorkGroup *
                                                  m_radixSortConfig.numScanElemsPerWorkGroup));
    }
    const uint32 numThreadGroups = GetParallelBuildNumThreadGroups();

    const BuildParallel::Constants shaderConstants =
    {
        .numThreadGroups = numThreadGroups,
    };

    BindPipeline(buildParallelShaderType);

    uint32 entryOffset = 0;
    entryOffset = WriteUserDataEntries(&shaderConstants, BuildParallel::NumEntries, entryOffset);

    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    {
        entryOffset = WriteBufferVa(0, entryOffset);
    }

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

#if GPURT_ENABLE_GPU_DEBUG
    entryOffset = WriteBufferVa(m_pDevice->GetGpuDebugBufferVa(), entryOffset);
#endif

    const uint32 cbvSrdTableGpuVaLo = Util::LowPart(m_geomConstSrdTable);
    entryOffset = WriteUserDataEntries(&cbvSrdTableGpuVaLo, 1, entryOffset);

    const uint32 vbvSrdTableGpuVaLo = Util::LowPart(m_geomBufferSrdTable);
    entryOffset = WriteUserDataEntries(&vbvSrdTableGpuVaLo, 1, entryOffset);

    RGP_PUSH_MARKER("BVH build");
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the appropriate exclusive scan shader depending on the number of elements
void BvhBuilder::ScanExclusiveAdd(
    uint32 numElems)          // Number of elements
{
    if (numElems < m_radixSortConfig.scanThresholdOneLevel)
    {
        ScanExclusiveAddOneLevel(m_scratchOffsets.histogram, numElems, 1);
    }
    else if (m_buildConfig.radixSortScanLevel == 0)
    {
        ScanExclusiveAddDLB(numElems);
    }
    else if (numElems < m_radixSortConfig.scanThresholdTwoLevel)
    {
        ScanExclusiveAddTwoLevel(numElems);
    }
    else if (numElems < m_radixSortConfig.scanThresholdThreeLevel)
    {
        ScanExclusiveAddThreeLevel(numElems);
    }
    else
    {
        PAL_ASSERT_ALWAYS_MSG("%s", "The maximum number of elements for scan exceeded\n");
    }
}

// =====================================================================================================================
// Executes the work group exclusive scan shader
void BvhBuilder::ScanExclusiveAddOneLevel(
    uint32 inOutArrayOffset,  // Scratch offset of the input/output array buffer
    uint32 numElems,          // Number of elements
    uint32 numWorkGroups)     // Number of work groups
{
    BindPipeline(InternalRayTracingCsType::ScanExclusiveInt4);

    uint32 entryOffset = 0;

    const ScanExclusiveAddWG::Constants shaderConstants =
    {
        .numElements             = numElems,
        .inOutArrayScratchOffset = inOutArrayOffset,
    };

    // Set constants
    entryOffset = WriteUserDataEntries(&shaderConstants, ScanExclusiveAddWG::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    RGP_PUSH_MARKER("Scan Exclusive");
    Dispatch(numWorkGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the partial scan shader
void BvhBuilder::ScanExclusiveAddPartial(
    uint32 inOutArrayOffset,  // Scratch offset of the input/output array buffer
    uint32 partSumsOffset,    // Partial sum offset in scratch
    uint32 numElems,          // Number of elements
    uint32 numWorkGroups)     // Number of work groups
{
    BindPipeline(InternalRayTracingCsType::ScanExclusivePartInt4);

    uint32 entryOffset = 0;

    // Set up shader constants
    const ScanExclusivePartSum::Constants scanConstants =
    {
        .numElements             = numElems,
        .inOutArrayScratchOffset = inOutArrayOffset,
        .partSumsScratchOffset   = partSumsOffset
    };

    entryOffset = WriteUserDataEntries(&scanConstants, ScanExclusivePartSum::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    RGP_PUSH_MARKER("Scan Exclusive (Partial)");

    // bottom level scan
    Dispatch(numWorkGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the partial sum distribution shader
void BvhBuilder::ScanExclusiveDistributeSums(
    uint32 inOutArrayOffset,  // Scratch offset of the input/output array buffer
    uint32 partSumsOffset,    // Partial sum offset in scratch
    uint32 numElems,          // Number of elements
    uint32 numWorkGroups)     // Number of work groups
{
    // distribute sums
    BindPipeline(InternalRayTracingCsType::DistributePartSumInt4);

    uint32 entryOffset = 0;

    // Set up shader constants
    const DistributePartSum::Constants distributeConstants =
    {
        .numElements              = numElems,
        .outputArrayScratchOffset = inOutArrayOffset,
        .partSumsScratchOffset    = partSumsOffset
    };

    entryOffset = WriteUserDataEntries(&distributeConstants, DistributePartSum::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    RGP_PUSH_MARKER("Scan Exclusive Distribute Partial Sum");
    Dispatch(numWorkGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the two level exclusive scan pass
void BvhBuilder::ScanExclusiveAddTwoLevel(
    uint32 numElems)          // Number of elements
{
    BindPipeline(InternalRayTracingCsType::ScanExclusivePartInt4);

    const uint32 numGroupsBottomLevelScan =
        Util::RoundUpQuotient(numElems, m_radixSortConfig.groupBlockSizeScan);

    const uint32 numGroupsTopLevelScan =
        Util::RoundUpQuotient(numGroupsBottomLevelScan, m_radixSortConfig.groupBlockSizeScan);

    const uint32 numGroupsBottomLevelDistribute =
        Util::RoundUpQuotient(numElems, m_radixSortConfig.groupBlockSizeDistribute);

    const uint32 inOutArrayOffset = m_scratchOffsets.histogram;
    const uint32 partSumsOffset   = m_scratchOffsets.distributedPartialSums;

    RGP_PUSH_MARKER("Scan Exclusive Two Level");

    // bottom level scan
    ScanExclusiveAddPartial(inOutArrayOffset, partSumsOffset, numElems, numGroupsBottomLevelScan);

    // Wait for the bottom level scan to complete
    Barrier();

    // top level scan
    ScanExclusiveAddOneLevel(partSumsOffset, numGroupsBottomLevelScan, numGroupsTopLevelScan);

    // Wait for the top level scan to complete
    Barrier();

    // Distribute partial sums
    ScanExclusiveDistributeSums(inOutArrayOffset, partSumsOffset, numElems, numGroupsBottomLevelDistribute);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the three level exclusive scan pass
void BvhBuilder::ScanExclusiveAddThreeLevel(
    uint32 numElems)          // Number of elements
{
    BindPipeline(InternalRayTracingCsType::ScanExclusivePartInt4);

    const uint32 numGroupsBottomLevelScan =
        Util::RoundUpQuotient(numElems, m_radixSortConfig.groupBlockSizeScan);

    const uint32 numGroupsMidLevelScan =
        Util::RoundUpQuotient(numGroupsBottomLevelScan, m_radixSortConfig.groupBlockSizeScan);

    const uint32 numGroupsTopLevelScan =
        Util::RoundUpQuotient(numGroupsMidLevelScan, m_radixSortConfig.groupBlockSizeScan);

    const uint32 numGroupsBottomLevelDistribute =
        Util::RoundUpQuotient(numElems, m_radixSortConfig.groupBlockSizeDistribute);

    const uint32 numGroupsMidLevelDistribute =
        Util::RoundUpQuotient(numGroupsBottomLevelDistribute, m_radixSortConfig.groupBlockSizeDistribute);

    const uint32 inOutArrayOffset          = m_scratchOffsets.histogram;
    const uint32 partSumsBottomLevelOffset = m_scratchOffsets.distributedPartialSums;
    const uint32 partSumsMidLevelOffset    = partSumsBottomLevelOffset + (numGroupsBottomLevelScan * sizeof(uint32));

    RGP_PUSH_MARKER("Scan Exclusive Three Level");

    ScanExclusiveAddPartial(
        inOutArrayOffset, partSumsBottomLevelOffset, numElems, numGroupsBottomLevelScan);
    Barrier();

    ScanExclusiveAddPartial(
        partSumsBottomLevelOffset, partSumsMidLevelOffset, numGroupsBottomLevelScan, numGroupsMidLevelScan);
    Barrier();

    ScanExclusiveAddOneLevel(
        partSumsMidLevelOffset, numGroupsMidLevelScan, numGroupsTopLevelScan);
    Barrier();

    ScanExclusiveDistributeSums(
        partSumsBottomLevelOffset, partSumsMidLevelOffset, numGroupsBottomLevelDistribute, numGroupsMidLevelDistribute);
    Barrier();

    ScanExclusiveDistributeSums(
        inOutArrayOffset, partSumsBottomLevelOffset, numElems, numGroupsBottomLevelDistribute);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the DLB exclusive scan pass
void BvhBuilder::ScanExclusiveAddDLB(
    uint32 numElems)          // Number of elements
{
    BindPipeline(InternalRayTracingCsType::InitScanExclusiveInt4DLB);

    const uint32 blockSize     = m_radixSortConfig.workGroupSize;
    const uint32 keysPerThread = m_radixSortConfig.keysPerThread;

    const uint32 elementsPerBlock = blockSize * keysPerThread;
    const uint32 numBlocks        = Util::RoundUpQuotient(numElems, elementsPerBlock);

    const uint32 threadGroupSize = blockSize;
    const uint32 numInitGroups   = Util::RoundUpQuotient(numBlocks, threadGroupSize);

    uint32 entryOffset = 0;

    const ScanExclusiveAddDLB::Constants shaderConstants =
    {
        .numElements = numElems,
    };

    // Set number of elements
    entryOffset = WriteUserDataEntries(&shaderConstants, ScanExclusiveAddDLB::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    RGP_PUSH_MARKER("Init Scan Exclusive Int 4 DLB");

    // bottom level scan
    Dispatch(numInitGroups);

    RGP_POP_MARKER();

    // Wait for the bottom level scan to complete
    Barrier();

    BindPipeline(InternalRayTracingCsType::ScanExclusiveInt4DLB);

    RGP_PUSH_MARKER("Scan Exclusive Int 4 DLB");
    Dispatch(numBlocks);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the DLB initialization pass
void BvhBuilder::ScanExclusiveAddDLBInit()
{
    const uint32 numElems = m_buildConfig.numHistogramElements;

    BindPipeline(InternalRayTracingCsType::InitScanExclusiveInt4DLB);

    const uint32 blockSize = m_radixSortConfig.workGroupSize;
    const uint32 keysPerThread = m_radixSortConfig.keysPerThread;

    const uint32 elementsPerBlock = blockSize * keysPerThread;
    const uint32 numBlocks = Util::RoundUpQuotient(numElems, elementsPerBlock);

    const uint32 threadGroupSize = blockSize;
    const uint32 numInitGroups = Util::RoundUpQuotient(numBlocks, threadGroupSize);

    uint32 entryOffset = 0;

    const ScanExclusiveAddDLB::Constants shaderConstants =
    {
        .numElements = numElems,
    };

    // Set number of elements
    entryOffset = WriteUserDataEntries(&shaderConstants, ScanExclusiveAddDLB::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    RGP_PUSH_MARKER("Init Scan Exclusive Int 4 DLB");

    // bottom level scan
    Dispatch(numInitGroups);

    RGP_POP_MARKER();

}

// =====================================================================================================================
// Executes the DLB exclusive scan pass
void BvhBuilder::ScanExclusiveAddDLBScan(
    uint32 passIdx
)
{
    const uint32 numElems = m_buildConfig.numHistogramElements;

    const uint32 blockSize = m_radixSortConfig.workGroupSize;
    const uint32 keysPerThread = m_radixSortConfig.keysPerThread;

    const uint32 elementsPerBlock = blockSize * keysPerThread;
    const uint32 numBlocks = Util::RoundUpQuotient(numElems, elementsPerBlock);

    uint32 entryOffset = 0;

    const ScanExclusiveAddDLB::Constants shaderConstants =
    {
        .numElements = numElems,
        .passIndex   = passIdx,
    };

    // Set number of elements
    entryOffset = WriteUserDataEntries(&shaderConstants, ScanExclusiveAddDLB::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferBindings(entryOffset);

    BindPipeline(InternalRayTracingCsType::ScanExclusiveInt4DLB);

    RGP_PUSH_MARKER("Scan Exclusive Int 4 DLB");
    Dispatch(numBlocks);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Binds the pipeline that corresponds with the provided compute shader type
void BvhBuilder::BindPipeline(
    InternalRayTracingCsType type) // Ray Tracing pipeline type
{
    m_pDevice->BindPipeline(m_cmdBuffer, type, m_buildSettings, m_buildSettingsHash);
}

// =====================================================================================================================
// Writes the provided entries into the compute shader user data slots
uint32 BvhBuilder::WriteUserDataEntries(
    const void* pEntries,    // User data entries
    uint32      numEntries,  // Number of entries
    uint32      entryOffset) // Offset of the first entry
{
    return m_pDevice->WriteUserDataEntries(m_cmdBuffer, pEntries, numEntries, entryOffset);
}

// =====================================================================================================================
// Writes a gpu virtual address for a buffer into the compute shader user data slots
uint32 BvhBuilder::WriteBufferVa(
    gpusize virtualAddress, // GPUVA of the buffer
    uint32  entryOffset)    // Offset of the first entry
{
    return m_pDevice->WriteBufferVa(m_cmdBuffer, virtualAddress, entryOffset);
}

// =====================================================================================================================
// Helper function to that calculates the number of primitives after applying the triangle split factor
uint32 BvhBuilder::NumPrimitivesAfterSplit(
    uint32 primitiveCount, // Number of primitives
    float  splitFactor)    // Number of triangle split factor
{
    return static_cast<uint32>(static_cast<float>(primitiveCount) * splitFactor);
}

// =====================================================================================================================
uint32 BvhBuilder::WriteBufferBindings(
    uint32 entryOffset,
    uint32 geometryIndex)
{
    // u0: SrcBuffer
    entryOffset = WriteBufferVa(SourceHeaderBufferBaseVa(), entryOffset);
    // u1: DstBuffer
    // Set DstBuffer to null for Updates since the metadata size recalculated may not match with what it was during the
    // Build and create a bad binding.
    gpusize dstBufferGpuVa = IsUpdate() ? 0 : ResultBufferBaseVa();
    entryOffset = WriteBufferVa(dstBufferGpuVa, entryOffset);
    // u2: DstMetadata
    entryOffset = WriteBufferVa(HeaderBufferBaseVa(), entryOffset);
    // u3: ScratchBuffer
    entryOffset = WriteBufferVa(RemappedScratchBufferBaseVa(), entryOffset);
    // u4: ScratchGlobal
    entryOffset = WriteBufferVa(ScratchBufferBaseVa(), entryOffset);

    // u5: InstanceDescBuffer
    if (m_buildConfig.topLevelBuild)
    {
        entryOffset = WriteBufferVa(m_buildArgs.inputs.instances.gpu, entryOffset);
    }
    else
    {
        entryOffset = WriteBufferVa(0, entryOffset);
    }

    // u6: EmitBuffer
    if (m_buildSettings.emitCompactSize)
    {
        entryOffset = WriteBufferVa(m_emitCompactDstGpuVa, entryOffset);
    }
    else
    {
        entryOffset = WriteBufferVa(0, entryOffset);
    }

    // u7: IndirectArgBuffer
    if (m_buildSettings.isIndirectBuild)
    {
        entryOffset = WriteBufferVa(m_buildArgs.indirect.indirectGpuAddr + (geometryIndex * m_buildArgs.indirect.indirectStride), entryOffset);
    }
    else
    {
        entryOffset = WriteBufferVa(0, entryOffset);
    }

    return entryOffset;
}

// =====================================================================================================================
uint32 BvhBuilder::WriteUpdateBuffers(
    uint32 entryOffset) // Offset of the first entry
{
    // Set output buffer
    entryOffset = WriteBufferVa(HeaderBufferBaseVa(), entryOffset);

    // Set scratch buffer
    entryOffset = WriteBufferVa(ScratchBufferBaseVa(), entryOffset);

    // Set source buffers
    entryOffset = WriteBufferVa(SourceHeaderBufferBaseVa(), entryOffset);

    return entryOffset;
}

// =====================================================================================================================
uint32 BvhBuilder::WriteBuildShaderConstantBuffer(uint32 entryOffset)
{
    entryOffset = WriteBufferVa(m_shaderConstantsGpuVa, entryOffset);

    return entryOffset;
}

}
