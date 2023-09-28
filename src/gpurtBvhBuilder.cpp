/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2019-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "gpurtInternal.h"
#include "gpurtInternalShaderBindings.h"
#include "gpurtBvhBatcher.h"
#include "gpurtBvhBuilder.h"
#include "gpurtBvhBuilderCommon.h"
#include "shared/accelStruct.h"

#include <float.h>

#define GPURT_ARRAY_SIZE(x) ((sizeof(x))/sizeof((x)[0]))

namespace GpuRt
{

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
// Helper function that calculates the correct dispatch size to use for Ray Tracing shaders
static uint32 DispatchSize(
    uint32 numWorkItems)
{
    return Util::RoundUpQuotient(numWorkItems, DefaultThreadGroupSize);
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
// Helper function that calculates the total number of AABBs required
static uint32 CalculateRayTracingAABBCount(
    const AccelStructBuildInputs& buildInfo,
    const ClientCallbacks& clientCb)
{
    uint32 aabbCount = 0;

    if (buildInfo.type == AccelStructType::BottomLevel)
    {
        // Bottom level
        for (uint32 i = 0; i < buildInfo.inputElemCount; ++i)
        {
            const Geometry geometry = clientCb.pfnConvertAccelStructBuildGeometry(buildInfo, i);

            aabbCount += GetGeometryPrimCount(geometry);
        }
    }
    else
    {
        // Top level

        // One AABB per instance
        aabbCount = buildInfo.inputElemCount;
    }
    return aabbCount;
}

// =====================================================================================================================
// Helper function that dispatches of size {numGroups, 1, 1}
void BvhBuilder::Dispatch(
    uint32 numGroups)
{
#if PAL_CLIENT_INTERFACE_MAJOR_VERSION < 771
    m_pPalCmdBuffer->CmdDispatch(numGroups, 1, 1);
#else
    m_pPalCmdBuffer->CmdDispatch({ numGroups, 1, 1 });
#endif
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

    return size;
}

// =====================================================================================================================
uint32 BvhBuilder::GetNumInternalNodeCount() const
{
    return CalcAccelStructInternalNodeCount(m_buildConfig.numLeafNodes, 4u);
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
        numBox16Nodes = (m_buildConfig.numLeafNodes / 4);
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

    const uint32 internalNodeSize = RayTracingQBVH32NodeSize;

    const uint32 sizeInBytes = (RayTracingQBVH16NodeSize * numBox16Nodes) +
                               (internalNodeSize * numInternalNodes);

    return sizeInBytes;
}

// =====================================================================================================================
// Static helper function that calculates the size of the buffer for leaf nodes
uint32 BvhBuilder::CalculateLeafNodesSize()
{
    return m_buildConfig.numLeafNodes * GetLeafNodeSize(m_deviceSettings, m_buildConfig);
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
// Static helper function that calculates the size of the scratch index buffer info
uint32 BvhBuilder::CalculateIndexBufferInfoSize(
    uint32 numGeometryDescs) ///< Number of geometry descs
{
    struct IndexBufferInfo
    {
        uint32 gpuVaLo;
        uint32 gpuVaHi;
        uint32 byteOffset;
        uint32 format;
    };

    return numGeometryDescs * sizeof(IndexBufferInfo);
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
    Pal::ICmdBuffer*             pCmdBuf,         // The associated PAL cmdbuffer
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
    m_pPalCmdBuffer(pCmdBuf),
    m_buildSettings({}),
    m_radixSortConfig(GetRadixSortConfig(deviceSettings)),
    m_emitCompactDstGpuVa(0ull),
    m_buildSettingsHash(0)
{
    InitializeBuildConfigs();
    if (IsUpdate() == false)
    {
        InitBuildShaderConstants();
    }
}

// =====================================================================================================================
// Explicit ray tracing bvh builder constructor that will perform a copy or emit build info
BvhBuilder::BvhBuilder(
    Pal::ICmdBuffer*             pCmdBuf,         // The associated PAL cmdbuffer
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
    m_pPalCmdBuffer(pCmdBuf),
    m_buildSettings({}),
    m_radixSortConfig(GetRadixSortConfig(deviceSettings)),
    m_emitCompactDstGpuVa(0ull),
    m_buildSettingsHash(0)
{
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
        (m_buildConfig.numLeafNodes <= m_deviceSettings.lbvhBuildThreshold))
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
// Calculates the result buffer offsets and returns the total result memory size
uint32 BvhBuilder::CalculateResultBufferInfo(
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
    //-----------------------------------------------------------------------------------------------------------//
    AccelStructDataOffsets offsets = {};

    // Acceleration structure data starts with the header
    runningOffset += sizeof(AccelStructHeader);

    uint32 internalNodeSize = 0;
    uint32 leafNodeSize = 0;

    if (m_buildConfig.numLeafNodes > 0)
    {
        internalNodeSize = CalculateInternalNodesSize();
        leafNodeSize = CalculateLeafNodesSize();

        offsets.internalNodes = runningOffset;
        runningOffset += internalNodeSize;

        offsets.leafNodes = runningOffset;
        runningOffset += leafNodeSize;

        if (m_buildConfig.topLevelBuild == false)
        {
            offsets.geometryInfo = runningOffset;
            runningOffset += CalculateGeometryInfoSize(m_buildArgs.inputs.inputElemCount);
        }

        offsets.primNodePtrs = runningOffset;
        runningOffset += m_buildConfig.numLeafNodes * sizeof(uint32);
    }

    // Metadata section is at the beginning of the acceleration structure buffer
    uint32 metadataSizeInBytes = CalcMetadataSizeInBytes(internalNodeSize, leafNodeSize);

    // Align metadata size to cache line
    metadataSizeInBytes = Util::Pow2Align(metadataSizeInBytes, 128);

    // Add in metadata size at the end of the calculation, as we do not want to include it in any of the offsets.
    runningOffset += metadataSizeInBytes;

    if (pOffsets != nullptr)
    {
        memcpy(pOffsets, &offsets, sizeof(offsets));
    }

    if (pMetadataSizeInBytes != nullptr)
    {
        *pMetadataSizeInBytes = metadataSizeInBytes;
    }

    return runningOffset;
}

// =====================================================================================================================
// Calculates the scratch buffer offsets and returns the total scratch memory size
uint32 BvhBuilder::CalculateScratchBufferInfo(
    RayTracingScratchDataOffsets* pOffsets)
{
    //--------------------------------------------
    // ScratchBuffer layout in each pass:
    //            [0]     [1]    [2]
    // BVH2 & QBVH | State        | TS/Rebraid
    // BVH2 & QBVH | State        | TD
    // BVH2 & QBVH | State | BVH2 | Sort
    // BVH2 & QBVH | State | BVH2 | PLOC/LTD
    // BVH2 & QBVH | State        | QBVH

    const bool willDumpScratchOffsets = m_deviceSettings.enableBuildAccelStructScratchDumping;
    // Reserve the beginning of the scratch buffer for the offsets data if dumping.
    uint32 runningOffset = willDumpScratchOffsets ? sizeof(RayTracingScratchDataOffsets) : 0;

    uint32 baseOffset1 = 0; // [1] end of State section
    uint32 baseOffset2 = 0; // [2] end of BVH2 section
    uint32 maxSize = 0;

    // The scratch acceleration structure is built as a BVH2.
    const uint32 aabbCount = m_buildConfig.numLeafNodes;
    const uint32 nodeCount = (aabbCount > 0) ? ((2 * aabbCount) - 1) : 0;

    //--------------------------------------------
    // BVH2 & QBVH
    uint32 bvhNodeData = 0xFFFFFFFF;
    uint32 bvhLeafNodeData = 0xFFFFFFFF;
    uint32 triangleSplitBoxes = 0xFFFFFFFF;
    uint32 fastLBVHRootNodeIndex = 0xFFFFFFFF;

    //--------------------------------------------
    // TaskQueueCounter
    uint32 triangleSplitTaskQueueCounter = 0xFFFFFFFF;
    uint32 rebraidTaskQueueCounter = 0xFFFFFFFF;
    uint32 tdTaskQueueCounter = 0xFFFFFFFF;
    uint32 plocTaskQueueCounter = 0xFFFFFFFF;

    //--------------------------------------------
    // State
    uint32 triangleSplitState = 0xFFFFFFFF;
    uint32 rebraidState = 0xFFFFFFFF;
    uint32 currentState = 0xFFFFFFFF;
    uint32 tdState = 0xFFFFFFFF;
    uint32 numBatches = 0xFFFFFFFF;
    uint32 debugCounters = 0;
    uint32 dynamicBlockIndex = 0xFFFFFFFF;
    uint32 sceneBounds = 0xFFFFFFFF;

    //--------------------------------------------
    // BVH2
    uint32 propagationFlags = 0xFFFFFFFF;
    uint32 batchIndices = 0xFFFFFFFF;
    uint32 indexBufferInfo = 0xFFFFFFFF;
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

    // ============ BVH2 & QBVH ============
    {
        // Scratch data for storing internal BVH node data
        bvhNodeData = runningOffset;
        runningOffset += nodeCount * RayTracingScratchNodeSize;

        // Unsorted leaf buffer
        if (m_buildConfig.noCopySortedNodes)
        {
            // Scratch data for storing unsorted leaf nodes. No additional memory is
            // used, so runningOffset doesn't need to be updated.
            bvhLeafNodeData = bvhNodeData + (m_buildConfig.numLeafNodes - 1) * RayTracingScratchNodeSize;
        }
        else
        {
            // Additional scratch data for storing unsorted leaf nodes
            bvhLeafNodeData = runningOffset;
            runningOffset += aabbCount * RayTracingScratchNodeSize;
        }

        if (m_buildConfig.triangleSplitting)
        {
            triangleSplitBoxes = runningOffset;
            runningOffset += aabbCount * sizeof(Aabb);
        }

        if (m_buildConfig.enableFastLBVH)
        {
            fastLBVHRootNodeIndex = runningOffset;
            runningOffset += sizeof(uint32);
        }

    }

    // ============ TaskQueueCounter ============
    {
        if (m_buildConfig.triangleSplitting)
        {
            triangleSplitTaskQueueCounter = runningOffset;
            runningOffset += RayTracingTaskQueueCounterSize;
        }

        if (m_buildConfig.rebraidType == GpuRt::RebraidType::V2)
        {
            rebraidTaskQueueCounter = runningOffset;
            runningOffset += RayTracingTaskQueueCounterSize;
        }

        if (m_buildConfig.topDownBuild)
        {
            tdTaskQueueCounter = runningOffset;               // td /tdtr taskCounter
            runningOffset += RayTracingTaskQueueCounterSize;
        }

        if ((m_buildConfig.topDownBuild == false) && (m_buildConfig.buildMode == BvhBuildMode::PLOC))
        {
            plocTaskQueueCounter = runningOffset;             // PLOC task counters
            runningOffset += RayTracingTaskQueueCounterSize;
        }

    }

    // ============ State ============
    {
        if (m_buildConfig.triangleSplitting)
        {
            triangleSplitState = runningOffset;
            runningOffset += RayTracingStateTSBuildSize;
        }

        if (m_buildConfig.rebraidType == GpuRt::RebraidType::V2)
        {
            rebraidState = runningOffset;
            runningOffset += RayTracingStateRebraidBuildSize;
        }

        if ((m_buildConfig.topDownBuild == false) && (m_buildConfig.buildMode == BvhBuildMode::PLOC))
        {
            currentState = runningOffset;
            runningOffset += RayTracingStatePLOCSize;           // PLOC state
        }

        if (m_buildConfig.topDownBuild)
        {
            tdState = runningOffset;
            if (m_buildConfig.rebraidType == GpuRt::RebraidType::V1)
            {
                runningOffset += RayTracingStateTDTRBuildSize;
            }
            else
            {
                runningOffset += RayTracingStateTDBuildSize;
            }
        }

        if ((m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair) &&
            (m_buildConfig.enableEarlyPairCompression == false))
        {
            numBatches = runningOffset;
            runningOffset += sizeof(uint32);
        }

        if (m_deviceSettings.enableBVHBuildDebugCounters)
        {
            // Adding a Build debug counter
            // Allocate memory for counters
            debugCounters = runningOffset;
            runningOffset += sizeof(uint32) * RayTracingBuildDebugCounters;
        }

        dynamicBlockIndex = runningOffset;
        runningOffset += sizeof(uint32);

        sceneBounds = runningOffset;
        runningOffset += sizeof(Aabb) + 2 * sizeof(float);  // scene bounding box + min/max prim size
        if (m_buildConfig.topLevelBuild == true)
        {
            runningOffset += sizeof(Aabb);  // scene bounding box for rebraid
        }
    }
    baseOffset1 = runningOffset;

    // ============ BVH2 ============
    {
        // Propagation flags
        propagationFlags = runningOffset;
            // Round up to multiple of primitive count. Encode* clears the flags based on the expansion factor which
            // must be a multiple of numPrimitives.
            const uint32 propagationFlagSlotCount =
            (m_buildConfig.numPrimitives == 0) ? 0 : Util::RoundUpToMultiple(aabbCount, m_buildConfig.numPrimitives);
        runningOffset += propagationFlagSlotCount * sizeof(uint32);

        if ((m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair) &&
            (m_buildConfig.enableEarlyPairCompression == false))
        {
            batchIndices = runningOffset;
            runningOffset += aabbCount * sizeof(uint32);

            indexBufferInfo = runningOffset;
            runningOffset += BvhBuilder::CalculateIndexBufferInfoSize(m_buildArgs.inputs.inputElemCount);
        }

        // Sorted primitive indices buffer size.
        primIndicesSorted = runningOffset;
        runningOffset += aabbCount * sizeof(uint32);
    }
    baseOffset2 = runningOffset;

    // ============ TS/Rebraid ============
    if (willDumpScratchOffsets == false)
    {
        runningOffset = baseOffset2;
    }
    if (m_buildConfig.triangleSplitting)
    {
        triangleSplitRefs0 = runningOffset;
        runningOffset += aabbCount * RayTracingTSRefScratchSize;

        triangleSplitRefs1 = runningOffset;
        runningOffset += aabbCount * RayTracingTSRefScratchSize;

        splitPriorities = runningOffset;
        runningOffset += aabbCount * sizeof(float);

        atomicFlagsTS = runningOffset;  // TODO: calculate number of blocks based on KEYS_PER_THREAD
        runningOffset += aabbCount * RayTracingAtomicFlags;
    }
    else if (m_buildConfig.rebraidType == GpuRt::RebraidType::V2)
    {
        atomicFlagsTS = runningOffset;  // TODO: calculate number of blocks based on KEYS_PER_THREAD
        runningOffset += aabbCount * RayTracingAtomicFlags;
    }
    maxSize = Util::Max(maxSize, runningOffset);

    // ============ TD ============
    if (willDumpScratchOffsets == false)
    {
        runningOffset = baseOffset2;
    }
    if (m_buildConfig.topDownBuild)
    {
        refList = runningOffset;
        if (m_buildConfig.rebraidType == GpuRt::RebraidType::V1)
        {
            runningOffset += RayTracingTDTRRefScratchSize * aabbCount;
        }
        else
        {
            runningOffset += RayTracingTDRefScratchSize * aabbCount;
        }

        // Align the beginning of the TDBins structs to 8 bytes so that 64-bit atomic operations on the first field in
        // the struct work correctly.
        runningOffset = Util::RoundUpToMultiple(runningOffset, 8u);
        tdBins = runningOffset;
        runningOffset += RayTracingTDBinsSize * (aabbCount / 3);

        tdNodeList = runningOffset;
        if (m_buildConfig.rebraidType == GpuRt::RebraidType::V1)
        {
            runningOffset += RayTracingTDTRNodeSize * (aabbCount - 1);
        }
        else
        {
            runningOffset += RayTracingTDNodeSize * (aabbCount - 1);
        }
    }
    maxSize = Util::Max(maxSize, runningOffset);

    // ============ Sort ============
    if (willDumpScratchOffsets == false)
    {
        runningOffset = baseOffset2;
    }
    if (m_buildConfig.topDownBuild == false)
    {
        const uint32 dataSize = m_deviceSettings.enableMortonCode30 ? sizeof(uint32) : sizeof(uint64);

        // Morton codes buffer size
        mortonCodes = runningOffset;
        runningOffset += aabbCount * dataSize;

        // Sorted morton codes buffer size
        mortonCodesSorted = runningOffset;
        runningOffset += aabbCount * dataSize;

        // Merge Sort
        if (m_deviceSettings.enableMergeSort)
        {
            primIndicesSortedSwap = runningOffset;
            runningOffset += aabbCount * sizeof(uint32);
        }
        // Radix Sort
        else
        {
            // Radix sort temporary buffers
            const uint32 numBlocks = (aabbCount + m_radixSortConfig.groupBlockSize - 1) /
                m_radixSortConfig.groupBlockSize;

            // device histograms buffer (int4)
            histogram = runningOffset;
            const uint32 numHistogramElements = numBlocks * m_radixSortConfig.numBins;
            runningOffset += numHistogramElements * sizeof(uint32);

            // device temp keys buffer (int)
            tempKeys = runningOffset;
            runningOffset += aabbCount * dataSize;

            // device temp vals buffer (int)
            tempVals = runningOffset;
            runningOffset += aabbCount * sizeof(uint32);

            if (m_buildConfig.radixSortScanLevel == 0)
            {
                const uint32 blockSize = m_radixSortConfig.workGroupSize;
                const uint32 numKeysPerThread = m_radixSortConfig.keysPerThread;
                const uint32 numDynamicBlocks = (numHistogramElements +
                    ((blockSize * numKeysPerThread) - 1)) / (blockSize * numKeysPerThread);

                atomicFlags = runningOffset;
                runningOffset += numDynamicBlocks * RayTracingScanDLBFlagsSize;
            }
            else
            {
                // partial sum scratch memory
                const uint32 numGroupsBottomLevelScan =
                    Util::RoundUpQuotient(m_buildConfig.numHistogramElements, m_radixSortConfig.groupBlockSizeScan);

                distributedPartSums = runningOffset;
                runningOffset += numGroupsBottomLevelScan * sizeof(uint32);
                if (m_buildConfig.numHistogramElements >= m_radixSortConfig.scanThresholdTwoLevel)
                {
                    const uint32 numGroupsMidLevelScan =
                        Util::RoundUpQuotient(numGroupsBottomLevelScan, m_radixSortConfig.groupBlockSizeScan);

                    runningOffset += numGroupsMidLevelScan * sizeof(uint32);
                }
            }
        }
    }
    maxSize = Util::Max(maxSize, runningOffset);

    // ============ PLOC/LTD ============
    if (willDumpScratchOffsets == false)
    {
        runningOffset = baseOffset2;
    }
    if (m_buildConfig.topDownBuild == false)
    {
        if (m_buildConfig.buildMode == BvhBuildMode::PLOC)
        {
            clustersList0 = runningOffset;
            runningOffset += aabbCount * sizeof(uint32);

            clustersList1 = runningOffset;
            runningOffset += aabbCount * sizeof(uint32);

            neighbourIndices = runningOffset;
            runningOffset += aabbCount * sizeof(uint32);

            atomicFlagsPloc = runningOffset; // TODO: calculate number of blocks based on KEYS_PER_THREAD
            runningOffset += aabbCount * RayTracingPLOCFlags;

            clusterOffsets = runningOffset;
            runningOffset += aabbCount * sizeof(uint32);
        }
    }
    maxSize = Util::Max(maxSize, runningOffset);

    // ============ QBVH ============
    if (willDumpScratchOffsets == false)
    {
        runningOffset = baseOffset2;
    }
    {
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
        runningOffset += maxStackEntry * stackEntrySize * sizeof(uint32);

        qbvhGlobalStackPtrs = runningOffset;
        runningOffset += RayTracingQBVHStackPtrsSize;
    }
    maxSize = Util::Max(maxSize, runningOffset);

    // If the caller requested offsets, return them.
    if (pOffsets != nullptr)
    {
        pOffsets->bvhNodeData = bvhNodeData;
        pOffsets->triangleSplitBoxes = triangleSplitBoxes;
        pOffsets->triangleSplitRefs0 = triangleSplitRefs0;
        pOffsets->triangleSplitRefs1 = triangleSplitRefs1;
        pOffsets->splitPriorities = splitPriorities;
        pOffsets->triangleSplitState = triangleSplitState;
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
        pOffsets->indexBufferInfo = indexBufferInfo;
        pOffsets->fastLBVHRootNodeIndex = fastLBVHRootNodeIndex;

    }

    // Return maxSize which now contains the total scratch size.
    return maxSize;
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
    // PropagationFlags        (uint32 PropagationFlags[NumPrimitives])
    // UpdateStackElements     uint32[NumPrimitives]
    //-----------------------------------------------------------------------------------------------------------//
    RayTracingScratchDataOffsets offsets = {};

    // Update stack pointer
    runningOffset += sizeof(uint32);

    // Done count
    runningOffset += sizeof(uint32);

    // Allocate space for the node flags
    offsets.propagationFlags = runningOffset;
    runningOffset += m_buildConfig.numLeafNodes * sizeof(uint32);

    // Allocate space for update stack
    offsets.updateStack = runningOffset;

    // Update stack elements. Note, for a worst case tree, each leaf node enqueues a single parent
    // node pointer for updating
    runningOffset += m_buildConfig.numLeafNodes * sizeof(uint32);

    if (pOffsets != nullptr)
    {
        memcpy(pOffsets, &offsets, sizeof(offsets));
    }

    return runningOffset;
}

// =====================================================================================================================
// If DeviceSettings has Auto Select Triangles or Updatable Fp16 Box Nodes enabled, test to select or turn off in build.
void BvhBuilder::UpdateBuildConfig()
{
    m_buildConfig.fp16BoxNodesInBlasMode = ForceDisableFp16BoxNodes(m_buildArgs.inputs, m_deviceSettings) ?
        Fp16BoxNodesInBlasMode::NoNodes : m_deviceSettings.fp16BoxNodesInBlasMode;

    m_buildConfig.triangleCompressionMode = AutoSelectTriangleCompressMode(m_buildArgs.inputs, m_deviceSettings) ?
        TriangleCompressionMode::Pair : TriangleCompressionMode::None;
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
    uint32 primitiveCount = CalculateRayTracingAABBCount(buildArgs.inputs, m_clientCb);

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
    m_buildConfig.numLeafNodes = primitiveCount;
    m_buildConfig.rebraidType = m_deviceSettings.rebraidType;
    m_buildConfig.topLevelBuild = buildArgs.inputs.type == AccelStructType::TopLevel;
    m_buildConfig.geometryType = GetGeometryType(buildArgs.inputs);
    UpdateBuildConfig();
    m_buildConfig.triangleCompressionMode = (buildArgs.inputs.type == AccelStructType::BottomLevel &&
        m_buildConfig.geometryType == GeometryType::Triangles) ?
        m_buildConfig.triangleCompressionMode :
        TriangleCompressionMode::None;

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

    m_buildConfig.triangleSplitting = m_deviceSettings.enableTriangleSplitting &&
        (buildArgs.inputs.type == AccelStructType::BottomLevel) &&
        (Util::TestAnyFlagSet(buildArgs.inputs.flags, AccelStructBuildFlagAllowUpdate) == false) &&
        Util::TestAnyFlagSet(buildArgs.inputs.flags, AccelStructBuildFlagPreferFastTrace);

    m_buildConfig.buildMode = OverrideBuildMode(buildArgs);

    if (m_buildConfig.rebraidType != RebraidType::Off)
    {
        m_buildConfig.numLeafNodes *= m_deviceSettings.rebraidFactor;
    }

    if (m_buildConfig.triangleSplitting == true)
    {
        m_buildConfig.numLeafNodes = NumPrimitivesAfterSplit(m_buildConfig.numLeafNodes,
                                                             m_deviceSettings.triangleSplittingFactor);
    }

    m_buildConfig.radixSortScanLevel = 0;
    m_buildConfig.numHistogramElements = GetNumHistogramElements(m_radixSortConfig, m_buildConfig.numLeafNodes);

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
    // Todo: fix NoCopySortedNodes for TopDown builder, for now disable it for TopDown
    m_buildConfig.noCopySortedNodes  = m_buildConfig.topDownBuild ? 0 : m_deviceSettings.noCopySortedNodes;
    m_buildConfig.enableFastLBVH     = (m_buildConfig.topDownBuild == false) && (m_deviceSettings.enableFastLBVH == true) &&
        (m_buildConfig.buildMode == BvhBuildMode::Linear
        );

    m_buildConfig.sceneCalcType = SceneBoundsCalculation::BasedOnGeometry;

    // Only enable earlyPairCompression if
    // -) triangleCompressionMode is set to be "Pair", and
    // -) deviceSettings chose to enable EarlyPairCompression
    // -) "NoCopySortedNodes" is set to true
    //     (because it preserves the unsorted scratch leaf without additional memory required)
    m_buildConfig.enableEarlyPairCompression = false;
    if ((m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair) &&
        (m_buildConfig.noCopySortedNodes == true))
    {
        m_buildConfig.enableEarlyPairCompression = m_deviceSettings.enableEarlyPairCompression;
    }

    // Max geometries we can fit in the SRD table for merged encode build/update
    const uint32 maxGeometriesInSrdTable =
        (m_pPalCmdBuffer != nullptr) ?
            (m_pPalCmdBuffer->GetEmbeddedDataLimit() / m_pDevice->GetBufferSrdSizeDw()) : 0;

    m_buildConfig.needEncodeDispatch =
        (IsUpdate() && (m_deviceSettings.enableMergedEncodeUpdate == 0)) ||
        ((IsUpdate() == false) && (m_deviceSettings.enableMergedEncodeBuild == 0)) ||
        (buildArgs.inputs.type == AccelStructType::TopLevel) ||
        (m_buildConfig.geometryType == GeometryType::Aabbs) ||
        (buildArgs.inputs.inputElemCount > maxGeometriesInSrdTable);
}

// =====================================================================================================================
void BvhBuilder::InitBuildShaderConstants()
{
    m_buildShaderConstants = {};

    // Make sure the address for empty top levels is 0 so we avoid tracing them. Set a valid address for empty bottom
    // levels so the top level build can still read the bottom level header.
    const bool emptyTopLevel = (m_buildConfig.topLevelBuild) && (m_buildConfig.numPrimitives == 0);

    m_buildShaderConstants.resultBufferAddrLo = emptyTopLevel ? 0 : Util::LowPart(ResultBufferBaseVa());
    m_buildShaderConstants.resultBufferAddrHi = emptyTopLevel ? 0 : Util::HighPart(ResultBufferBaseVa());
    m_buildShaderConstants.numPrimitives = m_buildConfig.numPrimitives;
    m_buildShaderConstants.tsBudgetPerTriangle = m_deviceSettings.tsBudgetPerTriangle;
    m_buildShaderConstants.maxNumPrimitives = NumPrimitivesAfterSplit(m_buildConfig.numPrimitives, m_deviceSettings.triangleSplittingFactor);
    m_buildShaderConstants.rebraidFactor = m_deviceSettings.rebraidFactor;
    m_buildShaderConstants.numLeafNodes = m_buildConfig.numLeafNodes;
    m_buildShaderConstants.header = InitAccelStructHeader();
    m_buildShaderConstants.numDescs = m_buildArgs.inputs.inputElemCount;

    if (m_buildConfig.topLevelBuild)
    {
        m_buildShaderConstants.encodeArrayOfPointers =
            (m_buildArgs.inputs.inputElemLayout == InputElementLayout::ArrayOfPointers);
    }
    else
    {
        m_buildShaderConstants.encodeArrayOfPointers = 0;
    }

    m_buildShaderConstants.offsets = m_scratchOffsets;

    // Add 4 DWORD padding to avoid page faults when the compiler uses a multi-DWORD load straddling the end of the
    // constant buffer
    static constexpr uint32 BufferPadding = 4u;
    static constexpr uint32 NumDwordsInBuildShaderConstants = (sizeof(BuildShaderConstants) / sizeof(uint32));
    uint32* pData = m_pPalCmdBuffer->CmdAllocateEmbeddedData(NumDwordsInBuildShaderConstants + BufferPadding,
                                                             1,
                                                             &m_shaderConstantsGpuVa);
    std::memcpy(pData, &m_buildShaderConstants, sizeof(BuildShaderConstants));
}

// =====================================================================================================================
AccelStructMetadataHeader BvhBuilder::InitAccelStructMetadataHeader()
{
    AccelStructMetadataHeader metaHeader = {};
    // Make sure the address for empty top levels is 0 so we avoid tracing them. Set a valid address for empty bottom
    // levels so the top level build can still read the bottom level header.
    const bool emptyTopLevel = (m_buildConfig.topLevelBuild) && (m_buildConfig.numPrimitives == 0);

    metaHeader.addressLo    = emptyTopLevel ? 0 : Util::LowPart(ResultBufferBaseVa());
    metaHeader.addressHi    = emptyTopLevel ? 0 : Util::HighPart(ResultBufferBaseVa());
    metaHeader.sizeInBytes  = m_metadataSizeInBytes;

    return metaHeader;
}

// =====================================================================================================================
AccelStructHeader BvhBuilder::InitAccelStructHeader()
{
    const uint32 accelStructSize = CalculateResultBufferInfo(&m_resultOffsets, &m_metadataSizeInBytes);

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
    header.numPrimitives            = m_buildConfig.numLeafNodes;
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
Pal::BufferViewInfo BvhBuilder::SetupVertexBuffer(
    const GeometryTriangles& desc,
    uint32*                  pStride,
    uint32*                  pVertexCompCount
    ) const
{
    const VertexFormatInfo formatInfo = VertexFormatInfoTable[uint32(desc.vertexFormat)];

    const gpusize vbGpuAddr     = desc.vertexBufferAddr.gpu;
    const gpusize vbStrideBytes = desc.vertexBufferByteStride;
    const gpusize reqAlignment  =
        (formatInfo.numFormat == Pal::ChNumFormat::X32Y32Z32_Float) ?
            sizeof(uint32) :
            Pal::Formats::BytesPerPixel(formatInfo.numFormat);

    const bool isAligned =
        Util::IsPow2Aligned(vbGpuAddr, reqAlignment) && Util::IsPow2Aligned(vbStrideBytes, reqAlignment);

    Pal::BufferViewInfo bufferInfo = {};
    bufferInfo.gpuAddr = vbGpuAddr;
    bufferInfo.range   = desc.vertexCount * vbStrideBytes;

    // Vertex buffers are only required to be aligned to the format's component size, not the full format element size.
    // If the buffer address and stride are sufficiently aligned, we can use a multi-component format to load all vertex
    // components at once. If not, we need to load each component separately using a single channel typed buffer.
    if (isAligned)
    {
        // Setup vertex buffer as a 2 or 3 channel typed buffer to fetch all components in one load
        bufferInfo.stride                 = vbStrideBytes;
        bufferInfo.swizzledFormat.format  = formatInfo.numFormat;
        bufferInfo.swizzledFormat.swizzle =
            (formatInfo.validChannels == 2) ? TwoChannelMapping : ThreeChannelMapping;

        // Stride is handled in the SRD
        *pStride = 0;
    }
    else
    {
        // Setup vertex buffer as a single channel typed buffer
        PAL_ASSERT(formatInfo.singleCompNumFormat != Pal::ChNumFormat::Undefined);

        const uint32 componentBytes = Pal::Formats::BytesPerPixel(formatInfo.singleCompNumFormat);

        PAL_ASSERT(Util::IsPow2Aligned(vbGpuAddr, componentBytes) &&
                   Util::IsPow2Aligned(vbStrideBytes, componentBytes));

        bufferInfo.stride                 = Pal::Formats::BytesPerPixel(formatInfo.singleCompNumFormat);
        bufferInfo.swizzledFormat.format  = formatInfo.singleCompNumFormat;
        bufferInfo.swizzledFormat.swizzle = SingleChannelMapping;

        *pStride = desc.vertexBufferByteStride / componentBytes;
    }

    *pVertexCompCount = formatInfo.validChannels;

    return bufferInfo;
}

// =====================================================================================================================
// Create a descriptor table containing a vertex buffer SRD and write the address to user data
uint32 BvhBuilder::WriteVertexBufferTable(
    const GeometryTriangles* pTriGeometry,
    EncodeNodes::Constants*  pEncodeConstants,
    uint32                   userDataOffset)
{
    gpusize tableGpuVa = 0;
    void* pTable = m_pPalCmdBuffer->CmdAllocateEmbeddedData(
        m_pDevice->GetBufferSrdSizeDw(),
        m_pDevice->GetBufferSrdSizeDw(),
        &tableGpuVa);

    const Pal::BufferViewInfo bufferInfo =
        SetupVertexBuffer(*pTriGeometry, &pEncodeConstants->geometryStride, &pEncodeConstants->vertexComponentCount);

    m_pDevice->CreateTypedBufferViewSrds(1, &bufferInfo, pTable);

    const uint32 tableGpuVaLo = Util::LowPart(tableGpuVa);
    userDataOffset = WriteUserDataEntries(&tableGpuVaLo, 1, userDataOffset);

    return userDataOffset;
}

// =====================================================================================================================
// Return integer expansion factor which determines the number of flag slots each thread clears during Encode.
uint32 BvhBuilder::GetLeafNodeExpansion() const
{
    return (m_buildConfig.numPrimitives == 0) ?
        0 : Util::RoundUpQuotient(m_buildConfig.numLeafNodes, m_buildConfig.numPrimitives);
}

// =====================================================================================================================
// Executes the encode triangle nodes shader
void BvhBuilder::EncodeTriangleNodes(
    uint32                                             primitiveOffset,  // Offset of the primitive
    const GeometryTriangles*                           pDesc,            // Triangles description
    uint32                                             primitiveCount,   // Number of primitives
    uint32                                             geometryIndex,    // Index in the geometry description array
    GeometryFlags                                      geometryFlags,    // Flags for the current geometry
    uint64                                             resultLeafOffset, // Offset in result data for current geometry
    gpusize                                            indirectGpuVa)    // Indirect offset buffer
{
    uint32 indexFormat           = static_cast<uint32>(IndexFormat::Unknown);
    uint64 indexBufferByteOffset = 0;
    uint64 indexBufferGpuVa      = pDesc->indexBufferAddr.gpu;

    // Set index format for valid indices
    if (indexBufferGpuVa != 0)
    {
        PAL_ASSERT(pDesc->indexFormat != IndexFormat::Unknown);
        indexFormat = static_cast<uint32>(pDesc->indexFormat);

        // Buffer loads require a be 4-byte aligned GPU VA. For 16-bit index buffers the GPU VA can be
        // 2-byte aligned, we align the GPU VA  down to 4-bytes and pass an offset for the compute shader to add
        // to the address
        constexpr uint64 BufferGpuVaAlign = 4;
        if (Util::IsPow2Aligned(indexBufferGpuVa, BufferGpuVaAlign) == false)
        {
            indexBufferGpuVa = Util::Pow2AlignDown(indexBufferGpuVa, BufferGpuVaAlign);
            indexBufferByteOffset = (static_cast<uint64>(pDesc->indexBufferAddr.gpu) - indexBufferGpuVa);
        }
    }

    PAL_ASSERT(pDesc->vertexFormat != VertexFormat::Invalid);

    EncodeNodes::Constants shaderConstants =
    {
        m_metadataSizeInBytes,
        primitiveCount,
        m_scratchOffsets.bvhLeafNodeData,
        primitiveOffset,
        m_scratchOffsets.sceneBounds,
        m_scratchOffsets.propagationFlags,
        m_scratchOffsets.updateStack,
        static_cast<uint32>(indexBufferByteOffset),
        (pDesc->columnMajorTransform3x4.gpu == 0) ? 0U : 1U,
        indexFormat,
        0,
        geometryIndex,
        m_resultOffsets.geometryInfo,
        m_resultOffsets.primNodePtrs,
        static_cast<uint32>(m_buildArgs.inputs.flags),
        IsUpdateInPlace(),
        static_cast<uint32>(geometryFlags),
        0,
        pDesc->vertexCount,
        static_cast<uint32>(resultLeafOffset),
        GetLeafNodeExpansion(),
        m_scratchOffsets.indexBufferInfo,
        Util::LowPart(indexBufferGpuVa),
        Util::HighPart(indexBufferGpuVa),
        static_cast<uint32>(m_buildConfig.sceneCalcType),
        m_deviceSettings.trianglePairingSearchRadius,
    };

    InternalRayTracingCsType encodePipeline = (indirectGpuVa > 0) ?
                                        InternalRayTracingCsType::EncodeTriangleNodesIndirect :
                                        InternalRayTracingCsType::EncodeTriangleNodes;
    BindPipeline(encodePipeline);

    uint32 entryOffset = 0;

    // Allocate embedded data memory for shader root constant buffer
    // and add 4 dword alignment to avoid SC out of bounds read.
    gpusize shaderConstantsGpuVa;
    uint32* pData = m_pPalCmdBuffer->CmdAllocateEmbeddedData(EncodeNodes::NumEntries,
                                                             4,
                                                             &shaderConstantsGpuVa);

    // Set shader root constant buffer
    entryOffset = WriteBufferVa(shaderConstantsGpuVa, entryOffset);

    entryOffset = WriteVertexBufferTable(pDesc, &shaderConstants, entryOffset);

    // Set index buffer
    entryOffset = WriteBufferVa(indexBufferGpuVa, entryOffset);

    // Set transform buffer
    entryOffset = WriteBufferVa(pDesc->columnMajorTransform3x4.gpu, entryOffset);

    // Set header buffer
    entryOffset = WriteBufferVa(HeaderBufferBaseVa(), entryOffset);

    // Set scratch buffer
    entryOffset = WriteBufferVa(ScratchBufferBaseVa(), entryOffset);

    // Set source buffer
    entryOffset = WriteBufferVa(SourceHeaderBufferBaseVa(), entryOffset);

    if (indirectGpuVa > 0)
    {
        // Set indirect buffer
        entryOffset = WriteBufferVa(indirectGpuVa, entryOffset);
    }

    memcpy(pData, &shaderConstants, sizeof(shaderConstants));

    RGP_PUSH_MARKER("Encode Triangle Nodes (NumPrimitives=%u)", primitiveCount);

    // Dispatch at least one group to ensure geometry info is written
    Dispatch(Util::Max(DispatchSize(primitiveCount), 1u));

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Create a descriptor table containing a typed buffer SRD pointing to AABBs and write the address to user data
uint32 BvhBuilder::WriteAabbGeometryTable(
    const GeometryAabbs* pAabbGeometry,
    uint32*              pStrideConstant,
    uint32               userDataOffset)
{
    gpusize tableGpuVa = 0;
    void* pTable = m_pPalCmdBuffer->CmdAllocateEmbeddedData(
        m_pDevice->GetBufferSrdSizeDw(),
        m_pDevice->GetBufferSrdSizeDw(),
        &tableGpuVa);

    // API alignment reqirements
    PAL_ASSERT(Util::IsPow2Aligned(pAabbGeometry->aabbAddr.gpu, 8));
    PAL_ASSERT(Util::IsPow2Aligned(pAabbGeometry->aabbByteStride, 8));

    const uint32 inputBufferSize = pAabbGeometry->aabbCount * pAabbGeometry->aabbByteStride;

    // The input buffer stride is required to be 0 or a multiple of 8.
    // It doesn't make sense to be smaller than an AABB. DXR and VK specs don't specify what happens in this case.
    PAL_ASSERT((pAabbGeometry->aabbByteStride == 0) || (pAabbGeometry->aabbByteStride >= sizeof(Aabb)));
    const uint32 inputByteStride = Util::Max<uint32>(pAabbGeometry->aabbByteStride, sizeof(Aabb));

    // Setup a typed buffer which can fetch 2 floats at a time.
    Pal::BufferViewInfo bufferInfo = {};
    bufferInfo.gpuAddr = pAabbGeometry->aabbAddr.gpu;
    bufferInfo.range   = inputBufferSize;
    bufferInfo.stride  = 8;
    bufferInfo.swizzledFormat.format  = Pal::ChNumFormat::X32Y32_Float;
    bufferInfo.swizzledFormat.swizzle = TwoChannelMapping;

    // Stride constant is in terms of X32Y32 elements.
    *pStrideConstant = inputByteStride / 8;

    m_pDevice->CreateTypedBufferViewSrds(1, &bufferInfo, pTable);

    const uint32 tableGpuVaLo = Util::LowPart(tableGpuVa);
    userDataOffset = WriteUserDataEntries(&tableGpuVaLo, 1, userDataOffset);

    return userDataOffset;
}

// =====================================================================================================================
// Executes the encode AABB nodes shader
void BvhBuilder::EncodeAABBNodes(
    uint32                                         primitiveOffset,  // Offset of the primitive
    const GeometryAabbs*                           pDesc,            // AABBs description
    uint32                                         primitiveCount,   // Number of primitives
    uint32                                         geometryIndex,    // Index in the geometry description array
    GeometryFlags                                  geometryFlags,    // Flags for the current geometry
    uint64                                         resultLeafOffset) // Offset in result data for current geometry
{
    EncodeNodes::Constants shaderConstants =
    {
        m_metadataSizeInBytes,
        primitiveCount,
        m_scratchOffsets.bvhLeafNodeData,
        primitiveOffset,
        m_scratchOffsets.sceneBounds,
        m_scratchOffsets.propagationFlags,
        m_scratchOffsets.updateStack,
        0,
        0,
        static_cast<uint32>(IndexFormat::Unknown),
        0,
        geometryIndex,
        m_resultOffsets.geometryInfo,
        m_resultOffsets.primNodePtrs,
        static_cast<uint32>(m_buildArgs.inputs.flags),
        IsUpdateInPlace(),
        static_cast<uint32>(geometryFlags),
        static_cast<uint32>(VertexFormat::Invalid),
        0,
        static_cast<uint32>(resultLeafOffset),
        m_buildConfig.numLeafNodes,
        m_scratchOffsets.indexBufferInfo,
        0,
        0,
        static_cast<uint32>(m_buildConfig.sceneCalcType),
        0,
    };

    BindPipeline(InternalRayTracingCsType::EncodeAABBNodes);

    uint32 entryOffset = 0;

    // Allocate embedded data memory for shader constant buffer
    // and add 4 dword alignment to avoid SC out of bounds read.
    gpusize shaderConstantsGpuVa;
    uint32* pData = m_pPalCmdBuffer->CmdAllocateEmbeddedData(EncodeNodes::NumEntries,
                                                             4,
                                                             &shaderConstantsGpuVa);

    // Set shader root constant buffer
    entryOffset = WriteBufferVa(shaderConstantsGpuVa, entryOffset);

    // Set geometry buffer
    entryOffset = WriteAabbGeometryTable(pDesc, &shaderConstants.geometryStride, entryOffset);

    // Set index buffer (isnt used by AABB, just triangles)
    entryOffset = WriteBufferVa(0, entryOffset);

    // Set transform buffer (isnt used by AABB, just triangles)
    entryOffset = WriteBufferVa(0, entryOffset);

    // Set header buffer
    entryOffset = WriteBufferVa(HeaderBufferBaseVa(), entryOffset);

    // Set scratch buffer
    entryOffset = WriteBufferVa(ScratchBufferBaseVa(), entryOffset);

    // Set source buffer
    entryOffset = WriteBufferVa(SourceHeaderBufferBaseVa(), entryOffset);

    memcpy(pData, &shaderConstants, sizeof(shaderConstants));

    RGP_PUSH_MARKER("Encode AABB Nodes (NumPrimitives=%u)", primitiveCount);

    // Dispatch at least one group to ensure geometry info is written
    Dispatch(Util::Max(DispatchSize(primitiveCount), 1u));

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the encode instances shader
void BvhBuilder::EncodeInstances(
    gpusize            instanceDescVa, // GPUVA of the instance description buffer
    uint32             numDesc,        // Number of instance descriptions
    InputElementLayout descLayout)     // The layout of the instance descriptions
{
    const uint32 internalFlags =
        ((descLayout == InputElementLayout::ArrayOfPointers) ? EncodeFlagArrayOfPointers : 0) |
        (IsUpdateInPlace() ? EncodeFlagUpdateInPlace : 0) |
        (m_buildConfig.rebraidType != RebraidType::Off ? EncodeFlagRebraidEnabled : 0) |
        (m_deviceSettings.enableFusedInstanceNode ? EncodeFlagFusedInstanceNode : 0);

    const EncodeInstances::Constants shaderConstants =
    {
        m_metadataSizeInBytes,
        m_resultOffsets.primNodePtrs,
        numDesc,
        m_scratchOffsets.bvhLeafNodeData,
        m_scratchOffsets.sceneBounds,
        m_scratchOffsets.propagationFlags,
        m_scratchOffsets.updateStack,
        internalFlags,
        m_buildArgs.inputs.flags,
        GetLeafNodeExpansion(),
        static_cast<uint32>(m_buildConfig.sceneCalcType),
        m_buildConfig.enableFastLBVH
    };

    BindPipeline(InternalRayTracingCsType::EncodeInstances);

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, EncodeInstances::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferVa(HeaderBufferBaseVa(), entryOffset);

    // Set scratch buffer
    entryOffset = WriteBufferVa(ScratchBufferBaseVa(), entryOffset);

    // Set source buffer
    entryOffset = WriteBufferVa(SourceHeaderBufferBaseVa(), entryOffset);

    // Set instance description buffer
    entryOffset = WriteBufferVa(instanceDescVa, entryOffset);

    RGP_PUSH_MARKER("Encode Instances (NumDescs=%u)", numDesc);
    Dispatch(DispatchSize(numDesc));

    RGP_POP_MARKER();
}

// =====================================================================================================================
void BvhBuilder::WriteImmediateSingle(
    gpusize                 destVa,
    uint64                  value,
    Pal::ImmediateDataWidth width)
{
#if PAL_CLIENT_INTERFACE_MAJOR_VERSION >= 697
    // We want to use HwPipePreCs (ME) so that the writes do not occur before UAV barriers are done waiting. Both
    // internal barriers during the build and application barriers synchronizing access to acceleration structure
    // memory wait at HwPipePreCs.
    m_pPalCmdBuffer->CmdWriteImmediate(Pal::HwPipePreCs, value, width, destVa);
#else
    m_pPalCmdBuffer->CmdWriteImmediate(Pal::HwPipeTop, value, width, destVa);
#endif
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

        WriteImmediateSingle(destVa, writeData, Pal::ImmediateDataWidth::ImmediateData32Bit);

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

        WriteImmediateSingle(destVa, writeData, Pal::ImmediateDataWidth::ImmediateData64Bit);

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

        WriteImmediateSingle(destVa, writeData, Pal::ImmediateDataWidth::ImmediateData32Bit);
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

    if (m_buildConfig.needEncodeDispatch && (m_buildConfig.numLeafNodes > 0))
    {
        const uint32 InitialMax = FloatToUintForCompare(-FLT_MAX);
        const uint32 InitialMin = FloatToUintForCompare(FLT_MAX);

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

            WriteImmediateData(ScratchBufferBaseVa() + m_scratchOffsets.sceneBounds, sceneBounds);
        }
        else
        {
            uint32 sceneBounds[] =
            {
                InitialMin, InitialMin, InitialMin,
                InitialMax, InitialMax, InitialMax,
                InitialMin, InitialMax, // size
            };

            WriteImmediateData(ScratchBufferBaseVa() + m_scratchOffsets.sceneBounds, sceneBounds);
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
    if (m_buildConfig.needEncodeDispatch || (m_buildConfig.numLeafNodes == 0))
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
    m_pDevice->PushRGPMarker(m_pPalCmdBuffer, pFormat, std::forward<Args>(args)...);
}

// =====================================================================================================================
void BvhBuilder::PopRGPMarker()
{
    m_pDevice->PopRGPMarker(m_pPalCmdBuffer);
}

// =====================================================================================================================
const char* BvhBuilder::ConvertBuildModeToString()
{
    static_assert(static_cast<uint32>(BvhBuildMode::Linear)   == 0, "BvhBuildMode enum mismatch");
    static_assert(static_cast<uint32>(BvhBuildMode::Reserved) == 1, "BvhBuildMode enum mismatch");
    static_assert(static_cast<uint32>(BvhBuildMode::PLOC)     == 2, "BvhBuildMode enum mismatch");
    static_assert(static_cast<uint32>(BvhBuildMode::Auto)     == 4, "BvhBuildMode enum mismatch");

    constexpr const char* BuildModeStr[] =
    {
        "LBVH",     // BvhBuildMode::Linear,
        "Reserved", // BvhBuildMode::Reserved,
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
        m_deviceSettings.topDownBuild ? "TopDown" : ConvertBuildModeToString());
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

    if (m_buildSettings.doCollapse > 0)
    {
        Util::Strncat(buildShaderInfo, MaxInfoStrLength, ", Collapse");
    }

    if (m_deviceSettings.enableMergeSort)
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

    m_pPalCmdBuffer->CmdCommentString(buildShaderInfo);
}

// =====================================================================================================================
void BvhBuilder::OutputPipelineName(
    InternalRayTracingCsType type)
{
    m_pDevice->OutputPipelineName(m_pPalCmdBuffer, type);
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
    m_buildSettings.doCollapse                   = m_buildConfig.collapse;
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
    m_buildSettings.enableMergeSort              = m_deviceSettings.enableMergeSort;
    m_buildSettings.fastBuildThreshold           = m_deviceSettings.fastBuildThreshold;
    m_buildSettings.enableFusedInstanceNode      = m_deviceSettings.enableFusedInstanceNode;

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

    m_buildSettings.noCopySortedNodes   = m_buildConfig.noCopySortedNodes;
    m_buildSettings.enableSAHCost       = m_deviceSettings.enableSAHCost;
    m_buildSettings.radixSortScanLevel  = m_buildConfig.radixSortScanLevel;

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

    m_buildSettings.isUpdate = IsUpdate();

    Util::MetroHash::Hash hash = {};
    Util::MetroHash64::Hash(reinterpret_cast<uint8*>(&m_buildSettings), sizeof(m_buildSettings), &hash.bytes[0]);

    m_buildSettingsHash = Util::MetroHash::Compact32(&hash);

#if GPURT_DEVELOPER
    OutputBuildInfo();
#endif
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
    const uint32 resultDataMaxSize = CalculateResultBufferInfo(nullptr, nullptr);

    if (m_buildConfig.numLeafNodes != 0)
    {
        // Calculate the amount of scratch space needed during the construction process.
        const uint32 scratchDataSize = CalculateScratchBufferInfo(nullptr);

        uint32 updateDataSize = 0;
        if (Util::TestAnyFlagSet(buildInfo.flags, AccelStructBuildFlagAllowUpdate))
        {
            updateDataSize = CalculateUpdateScratchBufferInfo(nullptr);
        }

        prebuildInfo.scratchDataSizeInBytes       = scratchDataSize;
        prebuildInfo.updateScratchDataSizeInBytes = updateDataSize;
    }
    else
    {
        // Empty acceleration structure
        // @note We set the ScratchData and UpdateScratchData size to 1 instead of 0, because some apps crash otherwise.
        prebuildInfo.scratchDataSizeInBytes       = 1;
        prebuildInfo.updateScratchDataSizeInBytes = 1;
    }

    prebuildInfo.resultDataMaxSizeInBytes = resultDataMaxSize;
    prebuildInfo.maxPrimitiveCount        = m_buildConfig.numLeafNodes;

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

    const uint32 resultDataSize = CalculateResultBufferInfo(&m_resultOffsets, &m_metadataSizeInBytes);

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
        InitAccelerationStructure();

        RGP_PUSH_MARKER(
            "Build%sLevelAccelerationStructure(NumDescs=%u, NumPrims=%u, Flags=%u, SizeInBytes=%u, NumEmits=%u)",
            m_buildConfig.topLevelBuild ? "Top" : "Bottom",
            m_buildArgs.inputs.inputElemCount,
            m_buildConfig.numPrimitives,
            m_buildArgs.inputs.flags,
            resultDataSize,
            m_buildArgs.postBuildInfoDescCount);
    }

    PreBuildDumpEvents();

    if (m_buildConfig.numLeafNodes > 0)
    {
        const uint32 totalPrimitiveCount = EncodePrimitives();
        PAL_ASSERT(totalPrimitiveCount == m_buildConfig.numPrimitives);

        if (totalPrimitiveCount != 0)
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
        EmitPostBuildInfo();
    }

    PostBuildDumpEvents();

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
    const bool hasBuildDumpEvents = m_deviceSettings.enableBuildAccelStructDumping ||
                                    m_deviceSettings.enableBuildAccelStructStats;
    if (hasBuildDumpEvents == false)
    {
        return;
    }

    const uint32 resultDataSize = CalculateResultBufferInfo(nullptr, nullptr);

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
        m_dumpInfo.scratchSizeInBytes = CalculateScratchBufferInfo(nullptr);
    }
    else
    {
        m_dumpInfo.scratchSizeInBytes = CalculateUpdateScratchBufferInfo(nullptr);
    }

    if (m_deviceSettings.enableBuildAccelStructStats)
    {
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION >= 39
        Pal::Result result = m_clientCb.pfnAccelStatsBuildDumpEvent(m_pPalCmdBuffer, &m_dumpInfo);
#else
        Pal::Result result =
            m_clientCb.pfnAccelStatsBuildDumpEvent(
                m_pPalCmdBuffer, m_dumpInfo, &m_dumpInfo.pTimeStampVidMem, &m_dumpInfo.timeStampVidMemoffset);
#endif

        if (result == Pal::Result::Success)
        {
            m_pPalCmdBuffer->CmdWriteTimestamp(Pal::HwPipeBottom,
                                               *m_dumpInfo.pTimeStampVidMem,
                                               m_dumpInfo.timeStampVidMemoffset);
        }
    }
}

// =====================================================================================================================
// Handles post-build dump event invocation
void BvhBuilder::PostBuildDumpEvents()
{
    if (m_deviceSettings.enableBuildAccelStructStats)
    {
        if (m_dumpInfo.pTimeStampVidMem != nullptr)
        {
            m_pPalCmdBuffer->CmdWriteTimestamp(Pal::HwPipeBottom,
                                               *m_dumpInfo.pTimeStampVidMem,
                                               m_dumpInfo.timeStampVidMemoffset + sizeof(uint64));
        }
    }

    // Dump Acceleration Structure
    if (m_deviceSettings.enableBuildAccelStructDumping)
    {
        const uint32 resultDataSize    = m_dumpInfo.sizeInBytes;
        const uint32 scratchBufferSize = m_dumpInfo.scratchSizeInBytes;

        uint64 allocationSize = resultDataSize;

        if (m_deviceSettings.enableBuildAccelStructScratchDumping)
        {
            allocationSize += scratchBufferSize;
        }

        // This assert will fail on some really large cases like san_miguel when also dumping scratch.
        // Attempting to dump more than 4GB will fail when it reaches PAL.
        // Not expected to be a problem with any actual titles.
        PAL_ASSERT(allocationSize <= UINT32_MAX);

        Barrier();

        gpusize dumpGpuVirtAddr = 0;
        Pal::Result result =
            m_clientCb.pfnAccelStructBuildDumpEvent(
                m_pPalCmdBuffer, m_dumpInfo, m_buildArgs, &dumpGpuVirtAddr);

        if (result == Pal::Result::Success)
        {
            m_pDevice->CopyBufferRaw(
                m_pPalCmdBuffer,
                dumpGpuVirtAddr,
                HeaderBufferBaseVa(),
                resultDataSize >> 2);

            if (m_deviceSettings.enableBuildAccelStructScratchDumping)
            {
                m_pDevice->CopyBufferRaw(m_pPalCmdBuffer,
                    dumpGpuVirtAddr + resultDataSize,
                    ScratchBufferBaseVa(),
                    scratchBufferSize >> 2);

                // Upload the scratch buffer offsets data to the reserved portion of the scratch buffer
                m_pDevice->UploadCpuMemory(m_pPalCmdBuffer,
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

    const uint32 resultDataSize = CalculateResultBufferInfo(&m_resultOffsets, &m_metadataSizeInBytes);

    if (IsUpdate() == false)
    {
        // Compute the offsets into the scratch buffer for all of our scratch resources.
        CalculateScratchBufferInfo(&m_scratchOffsets);
    }
    else
    {
        // Compute the offsets into the scratch buffer for all of our scratch resources.
        CalculateUpdateScratchBufferInfo(&m_scratchOffsets);

        // Reset the task counter for update parallel.
        ResetTaskCounter(HeaderBufferBaseVa());

        // Reset update stack pointer and update task counter.
        ZeroDataImmediate(ScratchBufferBaseVa(), 2);
    }

    // Add tlas to m_tlas
    if (m_buildArgs.inputs.type == AccelStructType::TopLevel)
    {
        m_pDevice->NotifyTlasBuild(HeaderBufferBaseVa());
    }
}

// =====================================================================================================================
// Prepares the inputs for the primitive encode shaders
uint32 BvhBuilder::EncodePrimitives()
{
    const bool isBottomLevel = (m_buildArgs.inputs.type == AccelStructType::BottomLevel);

    uint64 resultLeafOffset = 0;

    const uint64 leafNodeSize = GetLeafNodeSize(m_deviceSettings, m_buildConfig);

    const bool needEncodeDispatch = m_buildConfig.needEncodeDispatch;
    uint32 totalPrimitiveCount = 0;
    if (isBottomLevel)
    {
        // Prepare merged source AABB buffer data from geometry
        for (uint32 geometryIndex = 0; geometryIndex < m_buildArgs.inputs.inputElemCount; ++geometryIndex)
        {
            const Geometry geometry =
                m_clientCb.pfnConvertAccelStructBuildGeometry(m_buildArgs.inputs, geometryIndex);

            const uint32 primitiveCount = GetGeometryPrimCount(geometry);
            const uint32 primitiveOffset = totalPrimitiveCount;

            // Mixing geometry types within a bottom-level acceleration structure is not allowed.
            PAL_ASSERT(geometry.type == m_buildConfig.geometryType);

            if (geometry.type == GeometryType::Triangles)
            {
                const bool isIndexed = (geometry.triangles.indexFormat != IndexFormat::Unknown);

                if ((isIndexed == false) || (geometry.triangles.indexBufferAddr.gpu != 0))
                {
                    const gpusize indirectGpuAddress = m_buildArgs.indirect.indirectGpuAddr +
                                                       (geometryIndex * m_buildArgs.indirect.indirectStride);

                    if (needEncodeDispatch)
                    {
                        EncodeTriangleNodes(primitiveOffset,
                                            &geometry.triangles,
                                            primitiveCount,
                                            geometryIndex,
                                            geometry.flags,
                                            resultLeafOffset,
                                            indirectGpuAddress);
                    }

                    if (indirectGpuAddress > 0)
                    {
                        // Indirect build needs a Barrier because indirect build reads the GeometryInfo for previous geometries.
                        Barrier();
                    }
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
                if (needEncodeDispatch)
                {
                    EncodeAABBNodes(primitiveOffset,
                                    &geometry.aabbs,
                                    primitiveCount,
                                    geometryIndex,
                                    geometry.flags,
                                    resultLeafOffset);
                }
            }
            else
            {
                // Invalid geometry type
                PAL_ASSERT_ALWAYS();
            }

            totalPrimitiveCount += primitiveCount;
            resultLeafOffset += primitiveCount * leafNodeSize;
        }
    }
    else
    {
        // The primitives for the top level structure are just instances
        totalPrimitiveCount = m_buildArgs.inputs.inputElemCount;
        if (needEncodeDispatch)
        {
            EncodeInstances(m_buildArgs.inputs.instances.gpu,
                            m_buildArgs.inputs.inputElemCount,
                            m_buildArgs.inputs.inputElemLayout);
        }
    }
    return totalPrimitiveCount;
}

// =====================================================================================================================
// Handles writing any requested postbuild information.
void BvhBuilder::EmitPostBuildInfo()
{
    if (m_buildArgs.postBuildInfoDescCount == 0)
    {
        return;
    }
    const uint32 resultDataSize = CalculateResultBufferInfo(&m_resultOffsets, &m_metadataSizeInBytes);

    const bool isBottomLevel = (m_buildArgs.inputs.type == AccelStructType::BottomLevel);
    // We only need a barrier if there are more than one compacted size emits in this batch
    const bool useSeparateEmitPass = (m_emitCompactDstGpuVa != 0) && (m_buildSettings.emitCompactSize == 0);
    if (useSeparateEmitPass)
    {
        // Make sure build is complete before emitting
        Barrier();
    }

    for (uint32 i = 0; i < m_buildArgs.postBuildInfoDescCount; i++)
    {
        const AccelStructPostBuildInfo args = m_clientCb.pfnConvertAccelStructPostBuildInfo(m_buildArgs, i);
        switch (args.desc.infoType)
        {
        case AccelStructPostBuildInfoType::CompactedSize:
            // If numLeafNodes == 0, we never execute a BVH build, so we always need a separateEmitPass
            if (useSeparateEmitPass || (m_buildConfig.numLeafNodes == 0))
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
            i * static_cast<uint32>(sizeof(AccelStructPostBuildInfoCurrentSizeDesc)),
            static_cast<uint32>(m_deviceSettings.fp16BoxNodesInBlasMode),
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
            i * static_cast<uint32>(sizeof(AccelStructPostBuildInfoCompactedSizeDesc)),
            static_cast<uint32>(m_deviceSettings.fp16BoxNodesInBlasMode),
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
            i * static_cast<uint32>(sizeof(AccelStructPostBuildInfoToolsVisualizationDesc)),
            static_cast<uint32>(m_deviceSettings.fp16BoxNodesInBlasMode),
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
            i * static_cast<uint32>(sizeof(AccelStructPostBuildInfoSerializationDesc)),
            static_cast<uint32>(m_deviceSettings.fp16BoxNodesInBlasMode),
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
        numThreadGroups,
        static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu),
        static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu >> 32),
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
        numThreadGroups * DefaultThreadGroupSize,
        static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu),
        static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu >> 32)
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
        numThreadGroups
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
        static_cast<uint32>(address),
        static_cast<uint32>(address >> 32),
        numThreadGroups
    };

    // Reset the task counter in destination buffer.
    ResetTaskCounter(copyArgs.dstAccelStructAddr.gpu);

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
        static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu),
        static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu >> 32),
        numThreadGroups * DefaultThreadGroupSize,
        static_cast<uint32>(m_pDevice->GetRtIpLevel()),
        ((copyArgs.mode == AccelStructCopyMode::DriverDecode) ? 1u : 0u),
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
        BvhBatcher batcher(m_pPalCmdBuffer, m_pDevice, m_deviceProps, m_clientCb, m_deviceSettings);
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
void BvhBuilder::Barrier()
{
    m_pDevice->RaytracingBarrier(m_pPalCmdBuffer);
}

// =====================================================================================================================
// Executes merge sort shader to sort the input keys and values
void BvhBuilder::MergeSort()
{
    BindPipeline(InternalRayTracingCsType::MergeSort);

    ResetTaskCounter(HeaderBufferBaseVa());

    // Set shader constants
    uint32 entryOffset = 0;
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    RGP_PUSH_MARKER("Merge Sort (numLeafNodes %u)", m_buildConfig.numLeafNodes);

    const uint32 numThreadGroups = Util::RoundUpQuotient(m_buildConfig.numLeafNodes, DefaultThreadGroupSize);
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the Rebraid shader
// Preconditions: Rebraid can only be called on TLAS builds.
void BvhBuilder::Rebraid()
{
    const bool enableRebraid = (m_buildConfig.topLevelBuild) && (m_buildConfig.rebraidType == GpuRt::RebraidType::V2) &&
                               (m_buildConfig.numPrimitives > 0);
    if (enableRebraid == false)
    {
        return;
    }
    // TODO: Determine numThreadGroups without relying on BuildParallel's logic
    const uint32 numThreadGroups = GetParallelBuildNumThreadGroups();
    const uint32 encodeArrayOfPointers = (m_buildArgs.inputs.inputElemLayout == InputElementLayout::ArrayOfPointers);

    const Rebraid::Constants shaderConstants =
    {
        numThreadGroups,
    };

    ResetTaskQueueCounters(m_scratchOffsets.rebraidTaskQueueCounter);

    BindPipeline(InternalRayTracingCsType::Rebraid);

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, Rebraid::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);
    // Set DstBuffer, DstMetadata and ScratchBuffer
    entryOffset = WriteDestBuffers(entryOffset);
    // Set instance description buffer
    entryOffset = WriteBufferVa(m_buildArgs.inputs.instances.gpu, entryOffset);

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
    entryOffset = WriteDestBuffers(entryOffset);

    RGP_PUSH_MARKER("Generate Morton Codes (numLeafNodes %u)", m_buildConfig.numLeafNodes);
    Dispatch(DispatchSize(m_buildConfig.numLeafNodes));

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
    entryOffset = WriteDestBuffers(entryOffset);

    BindPipeline(pipeline);

    RGP_PUSH_MARKER("Build BVH (numLeafNodes %u)", m_buildConfig.numLeafNodes);
    Dispatch(DispatchSize(m_buildConfig.numLeafNodes));

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the build BVH shader
void BvhBuilder::BuildBVH()
{
    DispatchBuildBVHPipeline(InternalRayTracingCsType::BuildBVH);
}

// =====================================================================================================================
// Executes the build BVH SortScratchLeaves shader
void BvhBuilder::SortScratchLeaves()
{
    DispatchBuildBVHPipeline(InternalRayTracingCsType::BuildBVHSortLeaves);
}

// =====================================================================================================================
// Executes the first phase of BVH building based on the respective device settings
// TODO: Better name and logic
void BvhBuilder::BuildLbvhOrSortLeaves()
{
    if (m_buildConfig.buildMode == BvhBuildMode::Linear)
    {
        BuildBVH();
    }
    else if (m_buildSettings.noCopySortedNodes == false)
    {
        SortScratchLeaves();
    }
}

// =====================================================================================================================
// Executes the second phase of BVH building based on the respective device settings
// TODO: Better name, this may include LTD as well
void BvhBuilder::BuildBvhPlocOrRefit()
{
    if (m_buildConfig.buildMode == BvhBuildMode::PLOC)
    {
        BuildBVHPLOC();
    }
    else if (m_buildConfig.buildMode == BvhBuildMode::Linear)
    {
        RefitBounds();
    }
}

// =====================================================================================================================
// Executes the build BVH shader
void BvhBuilder::BuildBVHTD()
{
    const uint32 threadGroupSize = DefaultThreadGroupSize;
    const uint32 numThreadGroups = GetNumPersistentThreadGroups(m_buildConfig.numLeafNodes, threadGroupSize);

    const BuildBVHTD::Constants shaderConstants =
    {
        numThreadGroups * threadGroupSize,
    };

    ResetTaskCounter(HeaderBufferBaseVa());

    ResetTaskQueueCounters(m_scratchOffsets.tdTaskQueueCounter);

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, BuildBVHTD::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    // Set instance description buffer
    entryOffset = WriteBufferVa(m_buildArgs.inputs.instances.gpu, entryOffset);

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
void BvhBuilder::BuildBVHPLOC()
{
    const uint32 numThreadGroups = GetNumPersistentThreadGroups(m_buildConfig.numLeafNodes);

    uint32 entryOffset = 0;

    const BuildBVHPLOC::Constants shaderConstants =
    {
        numThreadGroups * DefaultThreadGroupSize,
    };

    ResetTaskCounter(HeaderBufferBaseVa());

    ResetTaskQueueCounters(m_scratchOffsets.plocTaskQueueCounter);

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, BuildBVHPLOC::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    BindPipeline(InternalRayTracingCsType::BuildBVHPLOC);

    RGP_PUSH_MARKER("Build PLOC BVH (numLeafNodes %u)", m_buildConfig.numLeafNodes);
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the update QBVH shader
// Refits internal node bounding boxes based on updated geometry
void BvhBuilder::UpdateQBVH()
{
    BindPipeline(InternalRayTracingCsType::UpdateQBVH);

    uint32 entryOffset = 0;

    const UpdateQBVH::Constants shaderConstants =
    {
        IsUpdateInPlace(),
        Util::LowPart(HeaderBufferBaseVa()),
        Util::HighPart(HeaderBufferBaseVa()),
        m_scratchOffsets.propagationFlags,
        m_scratchOffsets.updateStack,
        static_cast<uint32>(m_buildConfig.triangleCompressionMode),
        m_buildSettings.fp16BoxNodesMode,
        static_cast<uint32>(m_buildConfig.numPrimitives),
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, UpdateQBVH::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferVa(HeaderBufferBaseVa(), entryOffset);

    // Set scratch buffer
    entryOffset = WriteBufferVa(ScratchBufferBaseVa(), entryOffset);

    // Set source buffers
    entryOffset = WriteBufferVa(SourceHeaderBufferBaseVa(), entryOffset);

    RGP_PUSH_MARKER("Update QBVH");

    const uint32 numWorkItems = Util::Max(1u, (m_buildConfig.numPrimitives / 2));
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

    const UpdateParallel::Constants shaderConstants =
    {
        IsUpdateInPlace(),
        Util::LowPart(HeaderBufferBaseVa()),
        Util::HighPart(HeaderBufferBaseVa()),
        m_scratchOffsets.propagationFlags,
        m_scratchOffsets.updateStack,
        static_cast<uint32>(m_buildConfig.triangleCompressionMode),
        m_buildSettings.fp16BoxNodesMode,
        numThreads,
        m_buildConfig.numPrimitives,
        m_buildArgs.inputs.inputElemCount,
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, UpdateParallel::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferVa(HeaderBufferBaseVa(), entryOffset);

    // Set scratch buffer
    entryOffset = WriteBufferVa(ScratchBufferBaseVa(), entryOffset);

    // Set source buffer
    entryOffset = WriteBufferVa(SourceHeaderBufferBaseVa(), entryOffset);

    RGP_PUSH_MARKER("Update Parallel");
    Dispatch(numThreadGroups);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Setup per-geometry constant buffer
Pal::BufferViewInfo BvhBuilder::AllocGeometryConstants(
    const Geometry& geometry,
    uint32          geometryIndex,
    uint32*         pPrimitiveOffset,
    uint32          stride,
    uint32          vertexComponentCount,
    uint64*         pIbVa)
{
    uint32 indexFormat           = static_cast<uint32>(IndexFormat::Unknown);
    uint64 indexBufferByteOffset = 0;
    uint64 indexBufferGpuVa      = geometry.triangles.indexBufferAddr.gpu;

    // Set index format for valid indices
    if (indexBufferGpuVa != 0)
    {
        PAL_ASSERT(geometry.triangles.indexFormat != IndexFormat::Unknown);
        indexFormat = static_cast<uint32>(geometry.triangles.indexFormat);

        // Buffer loads require a be 4-byte aligned GPU VA. For 16-bit index buffers the GPU VA can be
        // 2-byte aligned, we align the GPU VA  down to 4-bytes and pass an offset for the compute shader to add
        // to the address
        constexpr uint64 BufferGpuVaAlign = 4;
        if (Util::IsPow2Aligned(indexBufferGpuVa, BufferGpuVaAlign) == false)
        {
            indexBufferGpuVa = Util::Pow2AlignDown(indexBufferGpuVa, BufferGpuVaAlign);
            indexBufferByteOffset = (static_cast<uint64>(geometry.triangles.indexBufferAddr.gpu) - indexBufferGpuVa);
        }
    }

    *pIbVa = indexBufferGpuVa;

    const uint32 primitiveCount = GetGeometryPrimCount(geometry);
    const uint32 primitiveOffset = *pPrimitiveOffset;

    *pPrimitiveOffset += primitiveCount;

    const EncodeNodes::Constants shaderConstants =
    {
        m_metadataSizeInBytes,
        primitiveCount,
        m_scratchOffsets.bvhLeafNodeData,
        primitiveOffset,
        m_scratchOffsets.sceneBounds,
        m_scratchOffsets.propagationFlags,
        m_scratchOffsets.updateStack,
        uint32(indexBufferByteOffset),
        (geometry.triangles.columnMajorTransform3x4.gpu == 0) ? 0U : 1U,
        indexFormat,
        stride,
        geometryIndex,
        m_resultOffsets.geometryInfo,
        m_resultOffsets.primNodePtrs,
        uint32(m_buildArgs.inputs.flags),
        IsUpdateInPlace(),
        uint32(geometry.flags),
        vertexComponentCount,
        geometry.triangles.vertexCount,
        0, // Result leaf offset unused
        GetLeafNodeExpansion(),
        m_scratchOffsets.indexBufferInfo,
        Util::LowPart(indexBufferGpuVa),
        Util::HighPart(indexBufferGpuVa),
        uint32(m_buildConfig.sceneCalcType),
        m_deviceSettings.trianglePairingSearchRadius,
    };

    constexpr uint32 AlignedSizeDw = Util::Pow2Align(EncodeNodes::NumEntries, 4);

    gpusize constantsVa;
    void* pConstants = m_pPalCmdBuffer->CmdAllocateEmbeddedData(AlignedSizeDw, 1, &constantsVa);

    memcpy(pConstants, &shaderConstants, sizeof(shaderConstants));

    Pal::BufferViewInfo viewInfo = {};
    viewInfo.gpuAddr = constantsVa;
    viewInfo.stride  = 16;
    viewInfo.range   = AlignedSizeDw * sizeof(uint32);

    return viewInfo;
}

// =====================================================================================================================
// Setup the SRD tables for each per-geometry input buffer (constants, vertex, index, transform)
uint32 BvhBuilder::WriteBufferSrdTable(
    const Pal::BufferViewInfo* pBufferViews,
    uint32                     count,
    bool                       typedBuffer,
    uint32                     entryOffset)
{
    return m_pDevice->WriteBufferSrdTable(m_pPalCmdBuffer, pBufferViews, count, typedBuffer, entryOffset);
}

// =====================================================================================================================
// Setup the SRD tables for each per-geometry input buffer (constants, vertex, index, transform)
uint32 BvhBuilder::WriteTriangleGeometrySrdTables(
    uint32 entryOffset) // Current user data entry offset
{
    const uint32 geometryCount = m_buildArgs.inputs.inputElemCount;

    Util::AutoBuffer<Pal::BufferViewInfo, 2, GpuRt::Internal::Device> constantBuffers(geometryCount, m_pDevice);
    Util::AutoBuffer<Pal::BufferViewInfo, 2, GpuRt::Internal::Device> indexBuffers(geometryCount, m_pDevice);
    Util::AutoBuffer<Pal::BufferViewInfo, 2, GpuRt::Internal::Device> vertexBuffers(geometryCount, m_pDevice);
    Util::AutoBuffer<Pal::BufferViewInfo, 2, GpuRt::Internal::Device> transformBuffers(geometryCount, m_pDevice);

    if ((constantBuffers.Capacity() >= geometryCount) &&
        (indexBuffers.Capacity() >= geometryCount) &&
        (vertexBuffers.Capacity() >= geometryCount) &&
        (transformBuffers.Capacity() >= geometryCount))
    {
        uint32 primitiveOffset = 0;

        for (uint32 i = 0; i < geometryCount; i++)
        {
            Geometry geometry = m_clientCb.pfnConvertAccelStructBuildGeometry(m_buildArgs.inputs, i);

            uint32 stride = 0;
            uint32 vertexCompCount = 0;
            vertexBuffers[i] = SetupVertexBuffer(geometry.triangles, &stride, &vertexCompCount);

            uint64 ibva = 0;
            constantBuffers[i] =
                AllocGeometryConstants(geometry, i, &primitiveOffset, stride, vertexCompCount, &ibva);

            indexBuffers[i]         = {};
            indexBuffers[i].gpuAddr = ibva;
            indexBuffers[i].range   = 0xFFFFFFFF;

            transformBuffers[i]         = {};
            transformBuffers[i].gpuAddr = geometry.triangles.columnMajorTransform3x4.gpu;
            transformBuffers[i].stride  = 4 * sizeof(float);
            transformBuffers[i].range   = transformBuffers[i].stride * 3;
        }

        entryOffset = WriteBufferSrdTable(&constantBuffers[0], geometryCount, false, entryOffset);
        entryOffset = WriteBufferSrdTable(&vertexBuffers[0], geometryCount, true, entryOffset);
        entryOffset = WriteBufferSrdTable(&indexBuffers[0], geometryCount, false, entryOffset);
        entryOffset = WriteBufferSrdTable(&transformBuffers[0], geometryCount, false, entryOffset);
    }
    else
    {
        PAL_ASSERT_ALWAYS();
    }

    return entryOffset;
}

// =====================================================================================================================
// Perform geometry encoding and update in one dispatch
void BvhBuilder::EncodeUpdate()
{
    BindPipeline(InternalRayTracingCsType::Update);

    const uint32 numWorkItems = Util::Max(1u, m_buildConfig.numPrimitives);

    const uint32 threadGroupSize  = DefaultThreadGroupSize;
    const uint32 wavesPerSimd     = 8;
    const uint32 numThreadGroups  = GetNumPersistentThreadGroups(numWorkItems, threadGroupSize, wavesPerSimd);
    const uint32 numThreads       = numThreadGroups * threadGroupSize;

    uint32 entryOffset = 0;

    const UpdateParallel::Constants shaderConstants =
    {
        IsUpdateInPlace(),
        Util::LowPart(HeaderBufferBaseVa()),
        Util::HighPart(HeaderBufferBaseVa()),
        m_scratchOffsets.propagationFlags,
        m_scratchOffsets.updateStack,
        static_cast<uint32>(m_buildConfig.triangleCompressionMode),
        m_buildSettings.fp16BoxNodesMode,
        numThreads,
        m_buildConfig.numPrimitives,
        m_buildArgs.inputs.inputElemCount,
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, UpdateParallel::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteBufferVa(HeaderBufferBaseVa(), entryOffset);

    // Set scratch buffer
    entryOffset = WriteBufferVa(ScratchBufferBaseVa(), entryOffset);

    // Set source buffer
    entryOffset = WriteBufferVa(SourceHeaderBufferBaseVa(), entryOffset);

    // Setup encode constants and resources
    entryOffset = WriteTriangleGeometrySrdTables(entryOffset);

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
// Executes the build QBVH shader
void BvhBuilder::BuildQBVH()
{
    const uint32 nodeCount       = GetNumInternalNodeCount();
    const uint32 numThreadGroups = GetNumPersistentThreadGroups(nodeCount);

    const BuildQBVH::Constants shaderConstants =
    {
        numThreadGroups * DefaultThreadGroupSize,
    };

    ResetTaskCounter(HeaderBufferBaseVa());

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, BuildQBVH::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    const gpusize instanceBufferVa = m_buildConfig.topLevelBuild ? m_buildArgs.inputs.instances.gpu : 0u;
    entryOffset = WriteBufferVa(instanceBufferVa, entryOffset);

    // Set optional emit compact size buffer GPU VA
    if (m_buildSettings.emitCompactSize == 1)
    {
        entryOffset = WriteBufferVa(m_emitCompactDstGpuVa, entryOffset);
    }

    BindPipeline(InternalRayTracingCsType::BuildQBVH);

    RGP_PUSH_MARKER("Build QBVH (nodeCount %u)", nodeCount);
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
    entryOffset = WriteDestBuffers(entryOffset);

    RGP_PUSH_MARKER("Refit Bounds");
    Dispatch(DispatchSize(m_buildConfig.numLeafNodes));

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the pair compression shader
void BvhBuilder::PairCompression()
{
    const bool enablePairCompression = (m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair) &&
                                       (m_buildConfig.enableEarlyPairCompression == false);
    if (enablePairCompression == false)
    {
        return;
    }
    BindPipeline(InternalRayTracingCsType::PairCompression);

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    RGP_PUSH_MARKER("Pair Compression");
    Dispatch(DispatchSize(m_buildConfig.numLeafNodes));

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
        numDwords,
        clearValue
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
        numDwords,
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
        bitShiftSize,
        numGroups,
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, BitHistogram::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

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
        bitShiftSize,
        numGroups,
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, RadixSort::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

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
        BitHistogram(bitShiftSize, m_buildConfig.numLeafNodes);

        // Wait for the bit histogram to complete
        Barrier();

        // Scan histograms
        ScanExclusiveAdd(m_buildConfig.numHistogramElements);

        // Wait for the scan operation to complete
        Barrier();

        // Scatter keys
        ScatterKeysAndValues(bitShiftSize, m_buildConfig.numLeafNodes);

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
    if (m_buildConfig.radixSortScanLevel > 0)
    {
        PAL_ASSERT(m_buildConfig.numPrimitives < (m_radixSortConfig.numScanElemsPerWorkGroup *
                                                  m_radixSortConfig.numScanElemsPerWorkGroup *
                                                  m_radixSortConfig.numScanElemsPerWorkGroup));
    }
    const uint32 numThreadGroups = GetParallelBuildNumThreadGroups();

    BuildParallel::Constants shaderConstants = {};
    shaderConstants.numThreadGroups = numThreadGroups;

    ResetTaskQueueCounters(m_scratchOffsets.rebraidTaskQueueCounter);
    ResetTaskQueueCounters(m_scratchOffsets.triangleSplitTaskQueueCounter);
    ResetTaskQueueCounters(m_scratchOffsets.tdTaskQueueCounter);
    ResetTaskQueueCounters(m_scratchOffsets.plocTaskQueueCounter);

    BindPipeline(InternalRayTracingCsType::BuildParallel);

    uint32 entryOffset = 0;
    entryOffset = WriteUserDataEntries(&shaderConstants, BuildParallel::NumEntries, entryOffset);

    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    entryOffset = WriteDestBuffers(entryOffset);

    if (m_buildConfig.topLevelBuild)
    {
        // Set instance description buffer
        entryOffset = WriteBufferVa(m_buildArgs.inputs.instances.gpu, entryOffset);
    }
    else
    {
        // No instance description buffer to set
        entryOffset = WriteBufferVa(0, entryOffset);
    }

    if (m_buildSettings.emitCompactSize == 1)
    {
        entryOffset = WriteBufferVa(m_emitCompactDstGpuVa, entryOffset);
    }
    else
    {
        entryOffset = WriteBufferVa(0, entryOffset);
    }

#if GPURT_ENABLE_GPU_DEBUG
    entryOffset = WriteBufferVa(m_pDevice->GetGpuDebugBufferVa(), entryOffset);
#endif

    if (m_buildConfig.needEncodeDispatch == false)
    {
        entryOffset = WriteTriangleGeometrySrdTables(entryOffset);
    }

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

    ScanExclusiveAddWG::Constants shaderConstants =
    {
        numElems,
        inOutArrayOffset,
    };

    // Set constants
    entryOffset = WriteUserDataEntries(&shaderConstants, ScanExclusiveAddWG::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

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
    ScanExclusivePartSum::Constants scanConstants =
    {
        numElems,
        inOutArrayOffset,
        partSumsOffset
    };

    entryOffset = WriteUserDataEntries(&scanConstants, ScanExclusivePartSum::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

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
    DistributePartSum::Constants distributeConstants =
    {
        numElems,
        inOutArrayOffset,
        partSumsOffset
    };

    entryOffset = WriteUserDataEntries(&distributeConstants, DistributePartSum::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

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

    ScanExclusiveAddDLB::Constants shaderConstants =
    {
        numElems,
    };

    // Set number of elements
    entryOffset = WriteUserDataEntries(&shaderConstants, ScanExclusiveAddDLB::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

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

    ScanExclusiveAddDLB::Constants shaderConstants =
    {
        numElems,
    };

    // Set number of elements
    entryOffset = WriteUserDataEntries(&shaderConstants, ScanExclusiveAddDLB::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    RGP_PUSH_MARKER("Init Scan Exclusive Int 4 DLB");

    // bottom level scan
    Dispatch(numInitGroups);

    RGP_POP_MARKER();

}

// =====================================================================================================================
// Executes the DLB exclusive scan pass
void BvhBuilder::ScanExclusiveAddDLBScan()
{
    const uint32 numElems = m_buildConfig.numHistogramElements;

    const uint32 blockSize = m_radixSortConfig.workGroupSize;
    const uint32 keysPerThread = m_radixSortConfig.keysPerThread;

    const uint32 elementsPerBlock = blockSize * keysPerThread;
    const uint32 numBlocks = Util::RoundUpQuotient(numElems, elementsPerBlock);

    uint32 entryOffset = 0;

    ScanExclusiveAddDLB::Constants shaderConstants =
    {
        numElems,
    };

    // Set number of elements
    entryOffset = WriteUserDataEntries(&shaderConstants, ScanExclusiveAddDLB::NumEntries, entryOffset);
    entryOffset = WriteBuildShaderConstantBuffer(entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

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
    m_pDevice->BindPipeline(m_pPalCmdBuffer, type, m_buildSettings, m_buildSettingsHash);
}

// =====================================================================================================================
// Writes the provided entries into the compute shader user data slots
uint32 BvhBuilder::WriteUserDataEntries(
    const void* pEntries,    // User data entries
    uint32      numEntries,  // Number of entries
    uint32      entryOffset) // Offset of the first entry
{
    return m_pDevice->WriteUserDataEntries(m_pPalCmdBuffer, pEntries, numEntries, entryOffset);
}

// =====================================================================================================================
// Writes a gpu virtual address for a buffer into the compute shader user data slots
uint32 BvhBuilder::WriteBufferVa(
    gpusize virtualAddress, // GPUVA of the buffer
    uint32  entryOffset)    // Offset of the first entry
{
    return m_pDevice->WriteBufferVa(m_pPalCmdBuffer, virtualAddress, entryOffset);
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
// Calculates the optimal number of thread groups to be launched based on the current hardware being run
uint32 BvhBuilder::GetOptimalNumThreadGroups(
    uint32 threadGroupSize) // Calculate number of thread groups to launch based on thread group size
{
    const auto*  pProps        = &m_deviceProps.gfxipProperties.shaderCore;
    const uint32 wavesPerGroup = Util::RoundUpQuotient(threadGroupSize, pProps->nativeWavefrontSize);

    return (pProps->numAvailableCus * pProps->numSimdsPerCu) / wavesPerGroup;
}

// =====================================================================================================================
uint32 BvhBuilder::WriteDestBuffers(
    uint32 entryOffset) // Offset of the first entry
{
    // Set output buffer
    entryOffset = WriteBufferVa(ResultBufferBaseVa(), entryOffset);

    // Set header buffer
    entryOffset = WriteBufferVa(HeaderBufferBaseVa(), entryOffset);

    // Set scratch buffer
    entryOffset = WriteBufferVa(ScratchBufferBaseVa(), entryOffset);

    return entryOffset;
}

// =====================================================================================================================
uint32 BvhBuilder::WriteBuildShaderConstantBuffer(uint32 entryOffset)
{
    entryOffset = WriteBufferVa(m_shaderConstantsGpuVa, entryOffset);

    return entryOffset;
}

}
