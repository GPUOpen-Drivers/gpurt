/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2019-2022 Advanced Micro Devices, Inc. All Rights Reserved.
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

#include "gpurt/gpurtLib.h"
#include "gpurt/gpurtAccelStruct.h"
#include "gpurtInternal.h"
#include "gpurtInternalShaderBindings.h"
#include "gpurtBvhBuilder.h"
#include "gpurtCpuBvhBuilder.h"

#include <float.h>

#if GPURT_DEVELOPER
#if defined(__GNUC__)
#define RGP_PUSH_MARKER(format, ...) PushRGPMarker(format, ##__VA_ARGS__)
#else
#define RGP_PUSH_MARKER(format, ...) PushRGPMarker(format, __VA_ARGS__)
#endif
#define RGP_POP_MARKER() PopRGPMarker()
#else
#define RGP_PUSH_MARKER(format, ...) (static_cast<void>(0))
#define RGP_POP_MARKER() (static_cast<void>(0))
#endif

#define GPURT_ARRAY_SIZE(x) ((sizeof(x))/sizeof((x)[0]))

namespace GpuRt
{

// =====================================================================================================================
// Helper function to that calculates the correct dispatch size to use for Ray Tracing shaders
static uint32 DispatchSize(
    uint32 numWorkItems)
{
    return Util::RoundUpQuotient(numWorkItems, DefaultThreadGroupSize);
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
    bool topLevelBuild)
{
    uint32 size = 0;

    if (topLevelBuild)
    {
        size = RayTracingInstanceNodeSize;
    }
    else
    {
        size = RayTracingQBVHLeafSize;
    }

    return size;
}

// =====================================================================================================================
// Helper function to determine Metadata size
uint32 BvhBuilder::CalcMetadataSizeInBytes(
    uint32 internalNodeSize,
    uint32 leafNodeSize)
{
    // @note Each 64-bytes in acceleration structure occupies 4-Bytes of parent pointer memory
    // for each primitive in the leaf node. E.g. an acceleration strucuture with 4 primitives
    //
    // | A | 0 | 1 | 2 | 3 | (A: internal node, 0-3 are leaf nodes)
    //
    // Parent pointer memory layout (-1 indicates root node)
    //
    // -- 1x Triangle Compression
    // |-1 | x |
    // | A | A |
    // | A | A |
    //

    const uint32 num64ByteChunks = (internalNodeSize + leafNodeSize) / 64;
    const uint32 numLinks = num64ByteChunks;
    const uint32 linkDataSizeInBytes = numLinks * sizeof(uint32);
    const uint32 metadataSizeInBytes = sizeof(AccelStructMetadataHeader) + linkDataSizeInBytes;

    return metadataSizeInBytes;
}

// =====================================================================================================================
// Static helper function that calculates the size of the buffer for internal nodes
uint32 BvhBuilder::CalculateInternalNodesSize()
{
    const bool enableHalfBoxNode32 = m_deviceSettings.enableHalfBoxNode32;
    const uint32 leafAabbCount = m_buildConfig.numLeafNodes;
    const uint32 numInternalNodes = CalcNumQBVHInternalNodes(leafAabbCount);

    // When half f32 box node is enabled, the worst case QBVH has all the internal nodes with 3 children.
    // Thus, the worst case QBVH internal node calculation is as follows
    //
    // nodeCount = lastLevelNodeCount * (1/3 + 1/9 + 1/27 + 1/81 + ... + (1/3)^n)
    // nodeCount = lastLevelNodeCount/2, when n = +inf
    //
    // We need at least 1 internal node.
    //
    const uint32 numInternalNodesHalfBoxNode32 = Util::Max(1u, leafAabbCount / 2);

    uint32 numBox16Nodes = 0;
    uint32 numBox32Nodes = 0;
    switch (m_buildConfig.fp16BoxNodesInBlasMode)
    {
    case Fp16BoxNodesInBlasMode::NoNodes:
        // All nodes are fp32
        numBox32Nodes = enableHalfBoxNode32 ? numInternalNodesHalfBoxNode32 : numInternalNodes;
        break;

    case Fp16BoxNodesInBlasMode::LeafNodes:
        // Conservative estimate how many interior nodes can be converted to fp16
        numBox16Nodes = (leafAabbCount / 4);
        numBox32Nodes = numInternalNodes - numBox16Nodes;
        break;

    case Fp16BoxNodesInBlasMode::MixedNodes:
        // Conservative estimate: no fp32 nodes could be converted to fp16
        // BVH storage savings realized after compaction copy
        numBox32Nodes = enableHalfBoxNode32 ? numInternalNodesHalfBoxNode32 : numInternalNodes;
        break;

    case Fp16BoxNodesInBlasMode::AllNodes:
        numBox16Nodes = numInternalNodes - 1;
        numBox32Nodes = 1;
        break;

    default:
        PAL_ASSERT_ALWAYS();
        break;
    }

    const uint32 internalNodeSize = RayTracingQBVH32NodeSize;

    const uint32 sizeInBytes = (RayTracingQBVH16NodeSize * numBox16Nodes) +
                               (internalNodeSize * numBox32Nodes);
    return sizeInBytes;
}

// =====================================================================================================================
// Static helper function that calculates the size of the buffer for leaf nodes
uint32 BvhBuilder::CalculateLeafNodesSize()
{
    return m_buildConfig.numLeafNodes * GetLeafNodeSize(m_buildConfig.topLevelBuild);
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
// Explicit ray tracing bvh builder constructor
BvhBuilder::BvhBuilder(
    Device*                      pDevice,         // GPURT device pointer.
    const Pal::DeviceProperties& deviceProps,     // PAL device properties
    const DeviceSettings&        deviceSettings)  // Device settings
    :
    m_pDevice(pDevice),
    m_deviceSettings(deviceSettings),
    m_buildArgs(),
    m_deviceProps(deviceProps)
{
}

// =====================================================================================================================
// Explicit ray tracing bvh builder destructor
BvhBuilder::~BvhBuilder()
{
}

// =====================================================================================================================
// Override build mode
BvhBuildMode GpuBvhBuilder::OverrideBuildMode(
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
GpuBvhBuilder::GpuBvhBuilder(
    Pal::ICmdBuffer*             pCmdBuf,
    Device*                      pDevice,
    const Pal::DeviceProperties& deviceProps,
    const DeviceSettings&        deviceSettings)
    :
    BvhBuilder(pDevice, deviceProps, deviceSettings),
    m_pPalCmdBuffer(pCmdBuf),
    m_buildSettings({}),
    m_radixSortConfig(GetRadixSortConfig(m_deviceSettings)),
    m_emitCompactDstGpuVa(0ull),
    m_buildSettingsHash(0)
{
}

// =====================================================================================================================
GpuBvhBuilder::~GpuBvhBuilder()
{
}

// =====================================================================================================================
// Forward BuildConfig initialization to BvhBuilder
void GpuBvhBuilder::InitBuildConfig(
    const AccelStructBuildInfo& buildArgs)
{
    BvhBuilder::InitBuildConfig(buildArgs);

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

    m_buildConfig.collapse = m_deviceSettings.bvhCollapse &&
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
}

// =====================================================================================================================
// Executes the encode triangle nodes shader
void GpuBvhBuilder::EncodeTriangleNodes(
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
    uint32 vertexFormat = static_cast<uint32>(pDesc->vertexFormat);

    const EncodeNodes::Constants shaderConstants =
    {
        primitiveCount,
        m_scratchOffsets.bvhLeafNodeData,
        primitiveOffset,
        m_scratchOffsets.sceneBounds,
        m_scratchOffsets.propagationFlags,
        m_scratchOffsets.updateStack,
        static_cast<uint32>(indexBufferByteOffset),
        (pDesc->columnMajorTransform3x4.gpu == 0) ? 0U : 1U,
        indexFormat,
        static_cast<uint32>(pDesc->vertexBufferByteStride),
        geometryIndex,
        static_cast<uint32>(m_buildArgs.inputs.flags),
        IsUpdateInPlace(),
        static_cast<uint32>(geometryFlags),
        vertexFormat,
        pDesc->vertexCount,
        static_cast<uint32>(resultLeafOffset),
        m_buildConfig.numLeafNodes,
        static_cast<uint32>(m_buildConfig.triangleCompressionMode),
        m_scratchOffsets.indexBufferInfo,
        Util::LowPart(indexBufferGpuVa),
        Util::HighPart(indexBufferGpuVa),
        false,
        m_buildConfig.triangleSplitting
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
    memcpy(pData, &shaderConstants, sizeof(shaderConstants));

    // Set shader root constant buffer
    entryOffset = WriteBufferVa(shaderConstantsGpuVa, entryOffset);

    // Set geometry buffer
    entryOffset = WriteBufferVa(pDesc->vertexBufferAddr.gpu, entryOffset);

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

    RGP_PUSH_MARKER("Encode Triangle Nodes (NumPrimitives=%u)", primitiveCount);

    // Dispatch at least one group to ensure geometry info is written
    m_pPalCmdBuffer->CmdDispatch(Util::Max(DispatchSize(primitiveCount), 1u), 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the encode AABB nodes shader
void GpuBvhBuilder::EncodeAABBNodes(
    uint32                                         primitiveOffset,  // Offset of the primitive
    const GeometryAabbs*                           pDesc,            // AABBs description
    uint32                                         primitiveCount,   // Number of primitives
    uint32                                         geometryIndex,    // Index in the geometry description array
    GeometryFlags                                  geometryFlags,    // Flags for the current geometry
    uint64                                         resultLeafOffset) // Offset in result data for current geometry
{
    const EncodeNodes::Constants shaderConstants =
    {
        primitiveCount,
        m_scratchOffsets.bvhLeafNodeData,
        primitiveOffset,
        m_scratchOffsets.sceneBounds,
        m_scratchOffsets.propagationFlags,
        m_scratchOffsets.updateStack,
        0,
        0,
        static_cast<uint32>(IndexFormat::Unknown),
        static_cast<uint32>(pDesc->aabbByteStride),
        geometryIndex,
        static_cast<uint32>(m_buildArgs.inputs.flags),
        IsUpdateInPlace(),
        static_cast<uint32>(geometryFlags),
        static_cast<uint32>(VertexFormat::Invalid),
        0,
        static_cast<uint32>(resultLeafOffset),
        m_buildConfig.numLeafNodes,
        static_cast<uint32>(m_buildConfig.triangleCompressionMode),
        m_scratchOffsets.indexBufferInfo,
        0,
        0,
        false,
        false
    };

    BindPipeline(InternalRayTracingCsType::EncodeAABBNodes);

    uint32 entryOffset = 0;

    // Allocate embedded data memory for shader constant buffer
    // and add 4 dword alignment to avoid SC out of bounds read.
    gpusize shaderConstantsGpuVa;
    uint32* pData = m_pPalCmdBuffer->CmdAllocateEmbeddedData(EncodeNodes::NumEntries,
                                                             4,
                                                             &shaderConstantsGpuVa);
    memcpy(pData, &shaderConstants, sizeof(shaderConstants));

    // Set shader root constant buffer
    entryOffset = WriteBufferVa(shaderConstantsGpuVa, entryOffset);

    // Set geometry buffer
    entryOffset = WriteBufferVa(pDesc->aabbAddr.gpu, entryOffset);

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

    RGP_PUSH_MARKER("Encode AABB Nodes (NumPrimitives=%u)", primitiveCount);

    // Dispatch at least one group to ensure geometry info is written
    m_pPalCmdBuffer->CmdDispatch(Util::Max(DispatchSize(primitiveCount), 1u), 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the encode instances shader
void GpuBvhBuilder::EncodeInstances(
    gpusize            instanceDescVa, // GPUVA of the instance description buffer
    uint32             numDesc,        // Number of instance descriptions
    InputElementLayout descLayout)     // The layout of the instance descriptions
{
    const uint32 internalFlags =
        ((descLayout == InputElementLayout::ArrayOfPointers) ? EncodeFlagArrayOfPointers : 0) |
        (IsUpdateInPlace() ? EncodeFlagUpdateInPlace : 0) |
        (m_buildConfig.rebraidType != RebraidType::Off ? EncodeFlagRebraidEnabled : 0);

    const EncodeInstances::Constants shaderConstants =
    {
        numDesc,
        m_scratchOffsets.bvhLeafNodeData,
        m_scratchOffsets.sceneBounds,
        m_scratchOffsets.propagationFlags,
        m_scratchOffsets.updateStack,
        internalFlags,
        m_buildArgs.inputs.flags,
        false
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

    m_pPalCmdBuffer->CmdDispatch(DispatchSize(numDesc), 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
void GpuBvhBuilder::WriteImmediateSingle(
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
void GpuBvhBuilder::WriteImmediateData(
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
void GpuBvhBuilder::WriteOrZeroDataImmediate(
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
void GpuBvhBuilder::InitAccelerationStructure(
    uint32 accelStructSize) // Total size in bytes including headers and metadata
{
    RGP_PUSH_MARKER("Init Acceleration Structure");

    const uint32 PosFloatMax = FloatToUintForCompare(-FLT_MAX);
    const uint32 NegFloatMax = FloatToUintForCompare(FLT_MAX);

    if (m_buildConfig.rebraidType == RebraidType::V2)
    {
        uint32 sceneBounds[] =
        {
            NegFloatMax, NegFloatMax, NegFloatMax,
            PosFloatMax, PosFloatMax, PosFloatMax,
            NegFloatMax, PosFloatMax, // size

            NegFloatMax, NegFloatMax, NegFloatMax, //used for rebraid
            PosFloatMax, PosFloatMax, PosFloatMax,
        };

        WriteImmediateData(ScratchBufferBaseVa() + m_scratchOffsets.sceneBounds, sceneBounds);
    }
    else
    {
        uint32 sceneBounds[] =
        {
            NegFloatMax, NegFloatMax, NegFloatMax,
            PosFloatMax, PosFloatMax, PosFloatMax,
            NegFloatMax, PosFloatMax, // size
        };

        WriteImmediateData(ScratchBufferBaseVa() + m_scratchOffsets.sceneBounds, sceneBounds);
    }

    AccelStructMetadataHeader metaHeader = {};

    // Make sure the address for empty top levels is 0 so we avoid tracing them. Set a valid address for empty bottom
    // levels so the top level build can still read the bottom level header.
    const bool emptyTopLevel =
        (m_buildConfig.topLevelBuild) && (m_buildConfig.numPrimitives == 0);

    metaHeader.addressLo   = emptyTopLevel ? 0 : Util::LowPart(ResultBufferBaseVa());
    metaHeader.addressHi   = emptyTopLevel ? 0 : Util::HighPart(ResultBufferBaseVa());
    metaHeader.sizeInBytes = m_metadataSizeInBytes;

    WriteImmediateData(HeaderBufferBaseVa(), metaHeader);
    AccelStructHeaderInfo info = {};

    info.type                     = static_cast<uint32>(m_buildArgs.inputs.type);
    info.buildType                = static_cast<uint32>(AccelStructBuilderType::Gpu);
    info.mode                     = m_buildSettings.buildMode;
    info.triCompression           = static_cast<uint32>(m_buildConfig.triangleCompressionMode);
    info.fp16BoxNodesInBlasMode   = static_cast<uint32>(m_buildConfig.fp16BoxNodesInBlasMode);
    info.triangleSplitting        = m_buildConfig.triangleSplitting;
    info.rebraid                  = m_buildConfig.rebraidType != RebraidType::Off;
    info.halfBoxNode              = m_deviceSettings.enableHalfBoxNode32;
    info.flags                    = m_buildArgs.inputs.flags;

    AccelStructHeaderInfo2 info2 = {};

    AccelStructHeader header = {};

    header.info                = info;
    header.info2               = info2;
    header.accelStructVersion  = GPURT_ACCEL_STRUCT_VERSION;
    header.metadataSizeInBytes = m_metadataSizeInBytes;
    header.sizeInBytes         = accelStructSize;
    header.numPrimitives       = m_buildConfig.numLeafNodes;
    header.numDescs            = m_buildArgs.inputs.inputElemCount;
    header.geometryType        = static_cast<uint32>(m_buildConfig.geometryType);
    header.uuidLo              = Util::LowPart(m_deviceSettings.accelerationStructureUUID);
    header.uuidHi              = Util::HighPart(m_deviceSettings.accelerationStructureUUID);

    // The leaf node count for bottom levels is initialized to zero and incremented during the build
    if (m_buildConfig.topLevelBuild)
    {
        header.numLeafNodes = m_buildArgs.inputs.inputElemCount;
    }

    header.offsets.internalNodes = m_resultOffsets.internalNodes;
    header.offsets.leafNodes     = m_resultOffsets.leafNodes;
    header.offsets.geometryInfo  = m_resultOffsets.geometryInfo;
    header.offsets.primNodePtrs  = m_resultOffsets.primNodePtrs;

    WriteImmediateData(ResultBufferBaseVa(), header);

    // reset taskQueue counters
    ResetTaskQueueCounters(m_scratchOffsets.currentState);
    ResetTaskQueueCounters(m_scratchOffsets.tdTaskQueueCounter);
    ResetTaskQueueCounters(m_scratchOffsets.rebraidState);

    if (m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair)
    {
        const gpusize numBatchesVa = ScratchBufferBaseVa() + m_scratchOffsets.numBatches;
        ZeroDataImmediate(numBatchesVa, 1);
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
void GpuBvhBuilder::ResetTaskCounter(
    gpusize metadataHeaderGpuVa)
{
    const gpusize taskCounterVa =
        metadataHeaderGpuVa + offsetof(AccelStructMetadataHeader, taskCounter);

    ZeroDataImmediate(taskCounterVa, 6);
}

// =====================================================================================================================
// Reset the taskQueue build counters
void GpuBvhBuilder::ResetTaskQueueCounters(
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
void GpuBvhBuilder::PushRGPMarker(
    const char* pFormat,
    ...)
{
    va_list args;
    va_start(args, pFormat);

    char strBuffer[256];
    Util::Vsnprintf(strBuffer, sizeof(strBuffer), pFormat, args);

    va_end(args);

    ClientInsertRGPMarker(m_pPalCmdBuffer, strBuffer, true);
}

// =====================================================================================================================
void GpuBvhBuilder::PopRGPMarker()
{
    ClientInsertRGPMarker(m_pPalCmdBuffer, nullptr, false);
}

// =====================================================================================================================
const char* GpuBvhBuilder::ConvertBuildModeToString()
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
const char* GpuBvhBuilder::ConvertRebraidTypeToString()
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
const char* GpuBvhBuilder::ConvertTriCompressionTypeToString()
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
const char* GpuBvhBuilder::ConvertFp16ModeToString()
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
void GpuBvhBuilder::OutputBuildInfo()
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
        Util::Strncat(buildShaderInfo, MaxInfoStrLength, ", TriangleSplitting");
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
void GpuBvhBuilder::OutputPipelineName(
    InternalRayTracingCsType type)
{
    constexpr uint32 MaxStrLength = 256;
    char buildShaderInfo[MaxStrLength];
    const PipelineBuildInfo* pPipelineBuildInfo = &InternalPipelineBuildInfo[static_cast<uint32>(type)];
    Util::Snprintf(buildShaderInfo, MaxStrLength, "BVH Build Pipeline: %s", pPipelineBuildInfo->pPipelineName);

    m_pPalCmdBuffer->CmdCommentString(buildShaderInfo);
}
#endif

// =====================================================================================================================
// Initalize the compile time constant buffer containing settings for the build shaders.
void GpuBvhBuilder::InitBuildSettings()
{
    m_buildSettings = {};

    const BvhBuildMode buildMode = m_buildConfig.buildMode;

    m_buildSettings.topLevelBuild                = (m_buildArgs.inputs.type == AccelStructType::TopLevel);
    m_buildSettings.buildMode                    = static_cast<uint32>(buildMode);
    m_buildSettings.triangleCompressionMode      = static_cast<uint32>(m_buildConfig.triangleCompressionMode);
    m_buildSettings.doCollapse                   = m_buildConfig.collapse;
    m_buildSettings.doTriangleSplitting          = m_buildConfig.triangleSplitting;
    m_buildSettings.fp16BoxNodesMode             = (m_buildSettings.topLevelBuild) ?
                                                   static_cast<uint32>(Fp16BoxNodesInBlasMode::NoNodes) :
                                                   static_cast<uint32>(m_buildConfig.fp16BoxNodesInBlasMode);
    m_buildSettings.fp16BoxModeMixedSaThreshhold = m_deviceSettings.fp16BoxModeMixedSaThresh;
    m_buildSettings.enableBVHBuildDebugCounters  = m_deviceSettings.enableBVHBuildDebugCounters;
    m_buildSettings.plocRadius                   = m_deviceSettings.plocRadius;
    m_buildSettings.enablePairCostCheck          = m_deviceSettings.enablePairCompressionCostCheck;
    m_buildSettings.enableVariableBitsMortonCode = m_deviceSettings.enableVariableBitsMortonCodes;

    m_buildSettings.rebraidType                  = static_cast<uint32>(m_buildConfig.rebraidType);
    m_buildSettings.enableTopDownBuild           = m_buildConfig.topDownBuild;
    m_buildSettings.useMortonCode30              = m_deviceSettings.enableMortonCode30;
    m_buildSettings.enableMergeSort              = m_deviceSettings.enableMergeSort;
    m_buildSettings.fastBuildThreshold           = m_deviceSettings.fastBuildThreshold;

    m_buildSettings.bvhBuilderNodeSortType       = static_cast<uint32>(m_buildConfig.bvhBuilderNodeSortType);
    m_buildSettings.bvhBuilderNodeSortHeuristic  = static_cast<uint32>(m_buildConfig.bvhBuilderNodeSortHeuristic);

    m_buildSettings.enableHalfBoxNode32          = m_deviceSettings.enableHalfBoxNode32;
    m_buildSettings.sahQbvh                      = m_deviceSettings.sahQbvh;

    m_buildSettings.radixSortScanLevel = m_buildConfig.radixSortScanLevel;

    uint32 emitBufferCount = 0;
    for (uint32 i = 0; i < m_buildArgs.postBuildInfoDescCount; ++i)
    {
        AccelStructPostBuildInfo args = ClientConvertAccelStructPostBuildInfo(m_buildArgs, i);
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

    Util::MetroHash::Hash hash = {};
    Util::MetroHash64::Hash(reinterpret_cast<uint8*>(&m_buildSettings), sizeof(m_buildSettings), &hash.bytes[0]);

    m_buildSettingsHash = Util::MetroHash::Compact32(&hash);

#if GPURT_DEVELOPER
    OutputBuildInfo();
#endif
}

// =====================================================================================================================
// Gets prebuild information about the acceleration structure to be built eventually.
void GpuBvhBuilder::GetAccelerationStructurePrebuildInfo(
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

        // Calculate the amount of scratch space needed during the construction process.
        prebuildInfo.cpuScratchSizeInBytes = CpuBvhBuilder::CalculateScratchBufferInfo(m_buildConfig, nullptr);

        prebuildInfo.cpuUpdateScratchSizeInBytes = 0;

        if (Util::TestAnyFlagSet(buildInfo.flags, AccelStructBuildFlagAllowUpdate))
        {
            prebuildInfo.cpuUpdateScratchSizeInBytes = CpuBvhBuilder::CalculateUpdateScratchBufferInfo(m_buildConfig, nullptr);
        }
    }
    else
    {
        // Empty acceleration structure
        // @note We set the ScratchData and UpdateScratchData size to 1 instead of 0, because some apps crash otherwise.
        prebuildInfo.scratchDataSizeInBytes       = 1;
        prebuildInfo.updateScratchDataSizeInBytes = 1;

        prebuildInfo.cpuScratchSizeInBytes        = 0;
        prebuildInfo.cpuUpdateScratchSizeInBytes  = 0;
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
        prebuildInfo.cpuScratchSizeInBytes =
            Util::Max(prebuildInfo.cpuScratchSizeInBytes, alternatePrebuildInfo.cpuScratchSizeInBytes);
        prebuildInfo.cpuUpdateScratchSizeInBytes =
            Util::Max(prebuildInfo.cpuUpdateScratchSizeInBytes, alternatePrebuildInfo.cpuUpdateScratchSizeInBytes);
        prebuildInfo.maxPrimitiveCount =
            Util::Max(prebuildInfo.maxPrimitiveCount, alternatePrebuildInfo.maxPrimitiveCount);
    }

    *pPrebuildInfo = prebuildInfo;
}

// =====================================================================================================================
// Builds or updates an acceleration structure by executing several shaders
void GpuBvhBuilder::BuildRaytracingAccelerationStructure(
    const AccelStructBuildInfo& buildArgs) // Build args
{
    if (m_deviceSettings.enableInsertBarriersInBuildAS == true)
    {
        // Intentionally insert Barrier() for debugging purpose
        Barrier();
    }

    m_buildArgs = buildArgs;

    const bool isBottomLevel = (m_buildArgs.inputs.type == AccelStructType::BottomLevel);
    // Initialize/Update BuildConfig
    InitBuildConfig(m_buildArgs);
    InitBuildSettings();

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
        RGP_PUSH_MARKER(
            "Build%sLevelAccelerationStructure(NumDescs=%u, NumPrims=%u, Flags=%u, SizeInBytes=%u, NumEmits=%u)",
            m_buildConfig.topLevelBuild ? "Top" : "Bottom",
            m_buildArgs.inputs.inputElemCount,
            m_buildConfig.numPrimitives,
            m_buildArgs.inputs.flags,
            resultDataSize,
            m_buildArgs.postBuildInfoDescCount);
    }

    Pal::IGpuMemory* pTimeStampVidMem = nullptr;
    uint64 offset = 0;

    // Initialise acceleration structure information for dump purposes
    AccelStructInfo info = {};

    info.type                    = m_buildArgs.inputs.type;
    info.numDesc                 = m_buildArgs.inputs.inputElemCount;
    info.numPrimitives           = m_buildConfig.numPrimitives;
    info.buildFlags              = m_buildArgs.inputs.flags;
    info.buildType               = AccelStructBuilderType::Gpu;
    info.buildMode               = static_cast<uint32>(m_buildConfig.buildMode);
    info.triangleCompressionMode = m_buildConfig.triangleCompressionMode;
    info.fp16BoxNodesInBlasMode  = m_buildConfig.fp16BoxNodesInBlasMode;
    info.gpuVa                   = HeaderBufferBaseVa();
    info.sizeInBytes             = resultDataSize;
    info.scratchGpuVa            = ScratchBufferBaseVa();

    uint32 scratchBufferSize = 0;
    if (IsUpdate() == false)
    {
        // Compute the offsets into the scratch buffer for all of our scratch resources.
        scratchBufferSize = CalculateScratchBufferInfo(&m_scratchOffsets);
        info.scratchSizeInBytes = scratchBufferSize;

        InitAccelerationStructure(resultDataSize);

        // Dump out ScratchMem info
        if (m_deviceSettings.enableBuildAccelStructStats)
        {
            Pal::Result result = ClientAccelStatsBuildDumpEvent(m_pPalCmdBuffer, info, &pTimeStampVidMem, &offset);

            if (result == Pal::Result::Success)
            {
                m_pPalCmdBuffer->CmdWriteTimestamp(Pal::HwPipeBottom, *pTimeStampVidMem, offset);
            }
        }
    }
    else
    {
        // Compute the offsets into the scratch buffer for all of our scratch resources.
        scratchBufferSize = CalculateUpdateScratchBufferInfo(&m_scratchOffsets);
        info.scratchSizeInBytes = scratchBufferSize;

        // Reset the task counter for update parallel.
        ResetTaskCounter(HeaderBufferBaseVa());

        // Reset TaskQ counters for update parallel.
        ResetTaskQueueCounters(m_scratchOffsets.currentState);
        ResetTaskQueueCounters(m_scratchOffsets.tdTaskQueueCounter);
        ResetTaskQueueCounters(m_scratchOffsets.rebraidState);

        // Reset update stack pointer for UpdateQBVH
        const gpusize stackPtrVa = ScratchBufferBaseVa() + m_scratchOffsets.updateStack;

        ZeroDataImmediate(stackPtrVa, 1);
    }

    // Add tlas to m_tlas
    if (info.type == AccelStructType::TopLevel)
    {
        m_pDevice->NotifyTlasBuild(HeaderBufferBaseVa());
    }

    uint32 totalPrimitiveCount = 0;
    if (m_buildConfig.numLeafNodes > 0)
    {
        uint64 resultLeafOffset = 0;

        const uint64 leafNodeSize = GetLeafNodeSize(m_buildConfig.topLevelBuild);

        if (m_buildConfig.topLevelBuild == false)
        {
            // Prepare merged source AABB buffer data from geometry
            for (uint32 geometryIndex = 0; geometryIndex < m_buildArgs.inputs.inputElemCount; ++geometryIndex)
            {
                uint32 primitiveCount = 0;
                const uint32 primitiveOffset = totalPrimitiveCount;

                const Geometry geometry = ClientConvertAccelStructBuildGeometry(m_buildArgs.inputs, geometryIndex);

                // Mixing geometry types within a bottom-level acceleration structure is not allowed.
                PAL_ASSERT(geometry.type == m_buildConfig.geometryType);

                if (geometry.type == GeometryType::Triangles)
                {
                    bool isIndexed = false;

                    if (geometry.triangles.indexFormat != IndexFormat::Unknown)
                    {
                        PAL_ASSERT((geometry.triangles.indexCount % 3) == 0);
                        primitiveCount = (geometry.triangles.indexCount / 3);
                        isIndexed = true;
                    }
                    else
                    {
                        PAL_ASSERT((geometry.triangles.vertexCount % 3) == 0);
                        primitiveCount = (geometry.triangles.vertexCount / 3);
                    }

                    if ((isIndexed == false) || (geometry.triangles.indexBufferAddr.gpu != 0))
                    {
                        const gpusize indirectGpuAddress = buildArgs.indirect.indirectGpuAddr +
                                                           (geometryIndex * buildArgs.indirect.indirectStride);

                        EncodeTriangleNodes(primitiveOffset,
                                            &geometry.triangles,
                                            primitiveCount,
                                            geometryIndex,
                                            geometry.flags,
                                            resultLeafOffset,
                                            indirectGpuAddress);

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
                    primitiveCount = static_cast<uint32>(geometry.aabbs.aabbCount);
                    EncodeAABBNodes(primitiveOffset,
                                    &geometry.aabbs,
                                    primitiveCount,
                                    geometryIndex,
                                    geometry.flags,
                                    resultLeafOffset);
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

            EncodeInstances(m_buildArgs.inputs.instances.gpu,
                            m_buildArgs.inputs.inputElemCount,
                            m_buildArgs.inputs.inputElemLayout);
        }

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
                // Update an existing BVH
                UpdateAccelerationStructure();
            }
        }

        // Handle the post build info feature
        if (m_buildArgs.postBuildInfoDescCount > 0)
        {
            // We only need a barrier if there are more than one compacted size emits in this batch
            const bool useSeparateEmitPass = (m_emitCompactDstGpuVa != 0) && (m_buildSettings.emitCompactSize == 0);
            if (useSeparateEmitPass)
            {
                // Make sure build is complete before emitting
                Barrier();
            }

            for (uint32 i = 0; i < m_buildArgs.postBuildInfoDescCount; i++)
            {
                const AccelStructPostBuildInfo args = ClientConvertAccelStructPostBuildInfo(m_buildArgs, i);
                switch (args.desc.infoType)
                {
                case AccelStructPostBuildInfoType::CompactedSize:
                    if (useSeparateEmitPass)
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
                            decodedSizeInBytes +=
                                RayTracingGeometryDescSize * m_buildArgs.inputs.inputElemCount;

                            decodedSizeInBytes +=
                                RayTracingDecodedLeafDataSize * m_buildConfig.numPrimitives;
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

                        info.numBlasPointers =
                            (isBottomLevel) ? 0 : m_buildConfig.numLeafNodes;

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
    }

    if (m_deviceSettings.enableBuildAccelStructStats)
    {
        if (pTimeStampVidMem != nullptr)
        {
            m_pPalCmdBuffer->CmdWriteTimestamp(Pal::HwPipeBottom, *pTimeStampVidMem, offset + sizeof(uint64));
        }
    }

    // Dump Acceleration Structure
    if (m_deviceSettings.enableBuildAccelStructDumping)
    {
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

        Pal::gpusize dumpGpuVirtAddr = 0;
        Pal::Result result = ClientAccelStructBuildDumpEvent(m_pPalCmdBuffer, info, m_buildArgs, &dumpGpuVirtAddr);

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
            }
        }
    }

    if (m_deviceSettings.enableInsertBarriersInBuildAS == true)
    {
        // Intentionally insert Barrier() for debugging purpose
        Barrier();
    }

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Emits post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void GpuBvhBuilder::EmitAccelerationStructurePostBuildInfo(
    const AccelStructPostBuildInfo& postBuildInfo) // Postbuild info
{
    switch (postBuildInfo.desc.infoType)
    {
    case AccelStructPostBuildInfoType::CurrentSize:
        EmitASCurrentType(postBuildInfo);
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
void GpuBvhBuilder::EmitASCurrentType(
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

        m_pPalCmdBuffer->CmdDispatch(1, 1, 1);

        RGP_POP_MARKER();

        entryOffset = 0;
    }
}

// =====================================================================================================================
// Emits post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void GpuBvhBuilder::EmitASCompactedType(
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

        m_pPalCmdBuffer->CmdDispatch(1, 1, 1);

        RGP_POP_MARKER();

        entryOffset = 0;
    }
}

// =====================================================================================================================
// Emits post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void GpuBvhBuilder::EmitASToolsVisualizationType(
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

        m_pPalCmdBuffer->CmdDispatch(1, 1, 1);

        RGP_POP_MARKER();

        entryOffset = 0;
    }
}

// =====================================================================================================================
// Emits post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void GpuBvhBuilder::EmitASSerializationType(
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

        m_pPalCmdBuffer->CmdDispatch(1, 1, 1);

        RGP_POP_MARKER();

        entryOffset = 0;
    }
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it to destination memory
void GpuBvhBuilder::CopyAccelerationStructure(
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
        CopyASToolsVisualizationMode(copyArgs);
        break;

    default:
        PAL_ASSERT_ALWAYS();
        break;
    }
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it to a same sized destination memory
void GpuBvhBuilder::CopyASCloneMode(
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

    m_pPalCmdBuffer->CmdDispatch(numThreadGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Compacts a source acceleration structure into a smaller destination memory
void GpuBvhBuilder::CopyASCompactMode(
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

    m_pPalCmdBuffer->CmdDispatch(numThreadGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it in a serialized manner to the destination memory
void GpuBvhBuilder::CopyASSerializeMode(
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

    m_pPalCmdBuffer->CmdDispatch(numThreadGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it in a deserialized manner to the destination memory
void GpuBvhBuilder::CopyASDeserializeMode(
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

    m_pPalCmdBuffer->CmdDispatch(numThreadGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it to the destination memory
void GpuBvhBuilder::CopyASToolsVisualizationMode(
    const AccelStructCopyInfo& copyArgs) // Copy arguments
{
    if (m_deviceSettings.bvhCollapse)
    {
        BindPipeline(InternalRayTracingCsType::DecodeCollapse);
    }
    else
    {
        BindPipeline(InternalRayTracingCsType::DecodeAS);
    }

    const uint32 numThreadGroups = GetNumThreadGroupsCopy();

    const DecodeAS::Constants shaderConstants =
    {
        static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu),
        static_cast<uint32>(copyArgs.dstAccelStructAddr.gpu >> 32),
        numThreadGroups * DefaultThreadGroupSize
    };

    uint32 entryOffset = 0;
    entryOffset = WriteUserDataEntries(&shaderConstants, DecodeAS::NumEntries, entryOffset);

    // Destination buffer
    entryOffset = WriteBufferVa(copyArgs.dstAccelStructAddr.gpu, entryOffset);

    // Source buffer
    entryOffset = WriteBufferVa(copyArgs.srcAccelStructAddr.gpu, entryOffset);

    RGP_PUSH_MARKER("Copy Acceleration Structure (Tools/Visualization)");

    m_pPalCmdBuffer->CmdDispatch(numThreadGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Builds an acceleration structure by executing several shaders
void GpuBvhBuilder::BuildAccelerationStructure()
{
    // if rebraid type == v1 then force to non-build parallel for now
    // BuildParallel only supports RebraidType::v2
    if (m_deviceSettings.enableParallelBuild && (m_buildConfig.rebraidType != RebraidType::V1))
    {
        BuildParallel();
    }
    else if (m_buildConfig.topDownBuild)
    {
        // Wait for the encoding operation to complete.
        Barrier();

        BuildBVHTD();

        Barrier();

        BuildQBVHTop();
    }
    else
    {

        // Wait for the encoding operation to complete.
        Barrier();

        // Generate morton codes from leaf nodes
        GenerateMortonCodes();

        // Wait for the morton code generation to complete
        Barrier();

        // Sort primitives according to their Morton codes
        if (m_deviceSettings.enableMergeSort)
        {
            MergeSort();
        }
        else
        {
            SortRadixInt32();
        }

        // Wait for the sorting to complete
        Barrier();

        // Build BVH2 in destination buffer
        BuildBVH();

        // Wait for the BVH build to complete
        Barrier();

        if (m_buildConfig.buildMode == BvhBuildMode::PLOC)
        {
            BuildBVHPLOC();

            Barrier();
        }

        if (m_buildConfig.topLevelBuild == false)
        {
            if (m_buildConfig.buildMode == BvhBuildMode::Linear)
            {
                RefitBounds();

                Barrier();
            }
        }
        else if (m_buildConfig.buildMode == BvhBuildMode::Linear)
        {
            RefitBounds();

            Barrier();
        }

        if (m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair)
        {
            PairCompression();
            Barrier();
        }

        if (m_buildConfig.topLevelBuild == false)
        {
            BuildQBVH();
        }
        else
        {
            BuildQBVHTop();
        }
    }
}

// =====================================================================================================================
// Updates an acceleration structure by executing several shaders
void GpuBvhBuilder::UpdateAccelerationStructure()
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

// =====================================================================================================================
// Performs a generic barrier that's used to synchronize internal ray tracing shaders
void GpuBvhBuilder::Barrier()
{
    if (m_deviceSettings.enableAcquireReleaseInterface)
    {
        Pal::AcquireReleaseInfo acqRelInfo = {};
        acqRelInfo.srcStageMask        = Pal::PipelineStageCs;
        acqRelInfo.dstStageMask        = Pal::PipelineStageCs;
        acqRelInfo.srcGlobalAccessMask = Pal::CoherShader;
        acqRelInfo.dstGlobalAccessMask = Pal::CoherShader;

        acqRelInfo.reason = m_deviceSettings.rgpBarrierReason;

        m_pPalCmdBuffer->CmdReleaseThenAcquire(acqRelInfo);
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

        m_pPalCmdBuffer->CmdBarrier(barrierInfo);
    }
}

// =====================================================================================================================
// Executes merge sort shader to sort the input keys and values
void GpuBvhBuilder::MergeSort()
{
    BindPipeline(InternalRayTracingCsType::MergeSort);

    const MergeSort::Constants shaderConstants =
    {
        m_buildConfig.numLeafNodes,
        m_scratchOffsets.mortonCodes,
        m_scratchOffsets.mortonCodesSorted,
        m_scratchOffsets.primIndicesSorted,
        m_scratchOffsets.primIndicesSortedSwap,
        m_deviceSettings.enableMortonCode30
    };

    ResetTaskCounter(HeaderBufferBaseVa());

    // Set shader constants
    uint32 entryOffset = 0;
    entryOffset = WriteUserDataEntries(&shaderConstants, MergeSort::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    RGP_PUSH_MARKER("Merge Sort");

    const uint32 numThreadGroups = Util::RoundUpQuotient(m_buildConfig.numLeafNodes, DefaultThreadGroupSize);
    m_pPalCmdBuffer->CmdDispatch(numThreadGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the generates morton codes shader
void GpuBvhBuilder::GenerateMortonCodes()
{
    const GenerateMortonCodes::Constants shaderConstants =
    {
        m_buildConfig.numLeafNodes,
        m_scratchOffsets.bvhLeafNodeData,
        m_scratchOffsets.sceneBounds,
        m_scratchOffsets.mortonCodes,
        m_buildConfig.triangleSplitting,
        m_scratchOffsets.triangleSplitBoxes,
        m_deviceSettings.enableVariableBitsMortonCodes,
        m_deviceSettings.enableMortonCode30
    };

    BindPipeline(InternalRayTracingCsType::GenerateMortonCodes);

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, GenerateMortonCodes::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    RGP_PUSH_MARKER("Generate Morton Codes");

    m_pPalCmdBuffer->CmdDispatch(DispatchSize(m_buildConfig.numLeafNodes), 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the build BVH shader
void GpuBvhBuilder::BuildBVH()
{
    uint32 entryOffset = 0;

    const BuildBVH::Constants shaderConstants =
    {
        (UpdateAllowed() ? 1U : 0U),
        m_scratchOffsets.bvhNodeData,
        m_scratchOffsets.bvhLeafNodeData,
        m_scratchOffsets.mortonCodesSorted,
        m_scratchOffsets.primIndicesSorted,
        m_deviceSettings.enableMortonCode30
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, BuildBVH::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    if (m_buildConfig.buildMode == BvhBuildMode::PLOC)
    {
        BindPipeline(InternalRayTracingCsType::BuildBVHSortLeaves);
    }
    else
    {
        BindPipeline(InternalRayTracingCsType::BuildBVH);
    }

    RGP_PUSH_MARKER("Build BVH");

    m_pPalCmdBuffer->CmdDispatch(DispatchSize(m_buildConfig.numLeafNodes), 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the build BVH shader
void GpuBvhBuilder::BuildBVHTD()
{
    const uint32 threadGroupSize = DefaultThreadGroupSize;
    const uint32 numThreadGroups = GetNumPersistentThreadGroups(m_buildConfig.numLeafNodes, threadGroupSize);

    const BuildBVHTD::Constants shaderConstants =
    {
        m_buildConfig.numPrimitives,
        UpdateAllowed(),
        numThreadGroups * threadGroupSize,
        m_buildConfig.numLeafNodes,
        m_deviceSettings.rebraidLengthPercentage,
        m_scratchOffsets.bvhNodeData,
        m_scratchOffsets.bvhLeafNodeData,
        m_scratchOffsets.sceneBounds,
        m_scratchOffsets.refList,
        m_scratchOffsets.tdNodeList,
        m_scratchOffsets.tdBins,
        m_scratchOffsets.tdState,
        m_scratchOffsets.tdTaskQueueCounter,
        m_scratchOffsets.refOffsets,
        (m_buildArgs.inputs.inputElemLayout == InputElementLayout::ArrayOfPointers)
    };

    ResetTaskCounter(HeaderBufferBaseVa());

    ResetTaskQueueCounters(m_scratchOffsets.tdTaskQueueCounter);

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, BuildBVHTD::NumEntries, entryOffset);

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

    m_pPalCmdBuffer->CmdDispatch(numThreadGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the build BVH PLOC shader
void GpuBvhBuilder::BuildBVHPLOC()
{
    const uint32 numThreadGroups = GetNumPersistentThreadGroups(m_buildConfig.numLeafNodes);

    uint32 entryOffset = 0;

    const BuildBVHPLOC::Constants shaderConstants =
    {
        numThreadGroups * DefaultThreadGroupSize,
        m_scratchOffsets.bvhNodeData,
        m_scratchOffsets.clusterList0,
        m_scratchOffsets.clusterList1,
        m_scratchOffsets.neighbourIndices,
        m_scratchOffsets.currentState,
        m_scratchOffsets.atomicFlagsPloc,
        m_scratchOffsets.clusterOffsets,
        m_scratchOffsets.dynamicBlockIndex,
        m_scratchOffsets.numBatches,
        m_scratchOffsets.batchIndices,
        m_buildSettings.fp16BoxNodesMode,
        m_deviceSettings.fp16BoxModeMixedSaThresh,
        BuildModeFlags(),
        m_scratchOffsets.triangleSplitBoxes,
        m_deviceSettings.plocRadius,
    };

    ResetTaskCounter(HeaderBufferBaseVa());

    ResetTaskQueueCounters(m_scratchOffsets.currentState);

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, BuildBVHPLOC::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    BindPipeline(InternalRayTracingCsType::BuildBVHPLOC);

    RGP_PUSH_MARKER("Build PLOC BVH");

    m_pPalCmdBuffer->CmdDispatch(numThreadGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the update QBVH shader
// Refits internal node bounding boxes based on updated geometry
void GpuBvhBuilder::UpdateQBVH()
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

    const uint32 numWorkItems =
        Util::Max(1u,
        (m_buildConfig.numPrimitives / 2));

    m_pPalCmdBuffer->CmdDispatch(DispatchSize(numWorkItems), 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the UpdateParallel shader
void GpuBvhBuilder::UpdateParallel()
{
    BindPipeline(InternalRayTracingCsType::UpdateParallel);

    const uint32 numWorkItems =
        Util::Max(1u,
            (m_buildConfig.numPrimitives / 2));

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

    m_pPalCmdBuffer->CmdDispatch(numThreadGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the build QBVH shader
void GpuBvhBuilder::BuildQBVH()
{
    const uint32 nodeCount       = CalcNumQBVHInternalNodes(m_buildConfig.numLeafNodes);
    const uint32 numThreadGroups = GetNumPersistentThreadGroups(nodeCount);

    const BuildQBVH::Constants shaderConstants =
    {
        m_buildConfig.numLeafNodes,
        m_metadataSizeInBytes,
        numThreadGroups * DefaultThreadGroupSize,
        m_scratchOffsets.bvhNodeData,
        m_scratchOffsets.qbvhGlobalStack,
        m_scratchOffsets.qbvhGlobalStackPtrs,
        static_cast<uint32>(m_buildConfig.triangleCompressionMode),
        m_buildSettings.fp16BoxNodesMode,
        BuildModeFlags(),
        m_scratchOffsets.triangleSplitBoxes,
        m_buildSettings.emitCompactSize,
        0,
        0,
        m_buildSettings.bvhBuilderNodeSortType,
        m_buildSettings.bvhBuilderNodeSortHeuristic,
        m_buildSettings.enableHalfBoxNode32,
        m_buildSettings.sahQbvh,
        0,
    };

    ResetTaskCounter(HeaderBufferBaseVa());

    BindPipeline(InternalRayTracingCsType::InitBuildQBVH);

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, BuildQBVH::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    // No instance description buffer to set
    entryOffset = WriteBufferVa(0, entryOffset);

    // Set optional emit compact size buffer GPU VA
    if (m_buildSettings.emitCompactSize == 1)
    {
        entryOffset = WriteBufferVa(m_emitCompactDstGpuVa, entryOffset);
    }

    RGP_PUSH_MARKER("Init Build QBVH");

    m_pPalCmdBuffer->CmdDispatch(DispatchSize(nodeCount), 1, 1);

    RGP_POP_MARKER();

    Barrier();

    if (m_buildConfig.collapse)
    {
        BindPipeline(InternalRayTracingCsType::BuildQBVHCollapse);

        RGP_PUSH_MARKER("Build QBVH Collapse");
    }
    else
    {
        BindPipeline(InternalRayTracingCsType::BuildQBVH);

        RGP_PUSH_MARKER("Build QBVH");
    }

    m_pPalCmdBuffer->CmdDispatch(numThreadGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the build top QBVH shader
void GpuBvhBuilder::BuildQBVHTop()
{
    const uint32 nodeCount       = CalcNumQBVHInternalNodes(m_buildConfig.numLeafNodes);
    const uint32 numThreadGroups = GetNumPersistentThreadGroups(nodeCount);

    const BuildQBVH::Constants shaderConstants =
    {
        m_buildConfig.numLeafNodes,
        m_metadataSizeInBytes,
        numThreadGroups * DefaultThreadGroupSize,
        m_scratchOffsets.bvhNodeData,
        m_scratchOffsets.qbvhGlobalStack,
        m_scratchOffsets.qbvhGlobalStackPtrs,
        static_cast<uint32>(TriangleCompressionMode::None),
        static_cast<uint32>(Fp16BoxNodesInBlasMode::NoNodes),
        false,
        0,
        m_buildSettings.emitCompactSize,
        (m_buildArgs.inputs.inputElemLayout == InputElementLayout::ArrayOfPointers),
        m_buildConfig.topDownBuild,
        m_buildSettings.bvhBuilderNodeSortType,
        m_buildSettings.bvhBuilderNodeSortHeuristic,
        m_buildSettings.enableHalfBoxNode32,
        m_buildSettings.sahQbvh,
        0,
    };

    ResetTaskCounter(HeaderBufferBaseVa());

    BindPipeline(InternalRayTracingCsType::InitBuildQBVH);

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, BuildQBVH::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    // Set instance description buffer
    entryOffset = WriteBufferVa(m_buildArgs.inputs.instances.gpu, entryOffset);

    // Set optional emit compact size buffer GPU VA
    if (m_buildSettings.emitCompactSize == 1)
    {
        entryOffset = WriteBufferVa(m_emitCompactDstGpuVa, entryOffset);
    }

    RGP_PUSH_MARKER("Init Build QBVH");

    m_pPalCmdBuffer->CmdDispatch(DispatchSize(nodeCount), 1, 1);

    RGP_POP_MARKER();

    Barrier();

    BindPipeline(InternalRayTracingCsType::BuildQBVH);

    RGP_PUSH_MARKER("Build QBVH Top");

    m_pPalCmdBuffer->CmdDispatch(numThreadGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the refit bounds shader
void GpuBvhBuilder::RefitBounds()
{
    BindPipeline(InternalRayTracingCsType::RefitBounds);

    const RefitBounds::Constants shaderConstants =
    {
        m_scratchOffsets.propagationFlags,
        m_scratchOffsets.bvhNodeData,
        m_buildSettings.fp16BoxNodesMode,
        m_deviceSettings.fp16BoxModeMixedSaThresh,
        m_buildConfig.collapse,
        m_buildConfig.triangleSplitting,
        (m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair),
        m_deviceSettings.enablePairCompressionCostCheck,
        m_scratchOffsets.triangleSplitBoxes,
        m_scratchOffsets.numBatches,
        m_scratchOffsets.batchIndices
    };

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, RefitBounds::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    RGP_PUSH_MARKER("Refit Bounds");

    m_pPalCmdBuffer->CmdDispatch(DispatchSize(m_buildConfig.numLeafNodes), 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the pair compression shader
void GpuBvhBuilder::PairCompression()
{
    BindPipeline(InternalRayTracingCsType::PairCompression);

    const PairCompression::Constants shaderConstants =
    {
        m_scratchOffsets.bvhNodeData,
        m_scratchOffsets.numBatches,
        m_scratchOffsets.batchIndices,
        m_scratchOffsets.indexBufferInfo,
        m_scratchOffsets.propagationFlags,
        m_buildArgs.inputs.flags
    };

    uint32 entryOffset = 0;

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, PairCompression::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    RGP_PUSH_MARKER("Pair Compression");

    m_pPalCmdBuffer->CmdDispatch(DispatchSize(m_buildConfig.numLeafNodes), 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the clear buffer shader
void GpuBvhBuilder::ClearBuffer(
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

    m_pPalCmdBuffer->CmdDispatch(DispatchSize(numDwords), 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the copy buffer shader
void GpuBvhBuilder::CopyBufferRaw(
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

    m_pPalCmdBuffer->CmdDispatch(DispatchSize(numDwords), 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the bit histogram shader
void GpuBvhBuilder::BitHistogram(
    uint32 inputArrayOffset,  // Scratch offset of the input array
    uint32 outputArrayOffset, // Scratch offset of the output array
    uint32 bitShiftSize,      // Number of bits to shift
    uint32 numElems)          // Number of elements
{
    BindPipeline(InternalRayTracingCsType::BitHistogram);

    const uint32 numGroups = Util::RoundUpQuotient(numElems, m_radixSortConfig.groupBlockSize);

    uint32 entryOffset = 0;

    const BitHistogram::Constants shaderConstants =
    {
        bitShiftSize,
        numElems,
        numGroups,
        inputArrayOffset,
        outputArrayOffset,
        m_deviceSettings.enableMortonCode30
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, BitHistogram::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    RGP_PUSH_MARKER("Bit Histogram");

    m_pPalCmdBuffer->CmdDispatch(numGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the scatter keys and values shader
void GpuBvhBuilder::ScatterKeysAndValues(
    uint32 inputKeysOffset,    // Scratch offset of the input keys buffer
    uint32 inputValuesOffset,  // Scratch offset of the input values buffer
    uint32 histogramsOffset,   // Scratch offset of the histograms buffer
    uint32 outputKeysOffset,   // Scratch offset of the output keys buffer
    uint32 outputValuesOffset, // Scratch offset of the output values buffer
    uint32 bitShiftSize,       // Number of bits to shift
    uint32 numElems)           // Number of elements
{
    BindPipeline(InternalRayTracingCsType::ScatterKeysAndValues);

    const uint32 numGroups = Util::RoundUpQuotient(numElems, m_radixSortConfig.groupBlockSize);

    uint32 entryOffset = 0;

    const RadixSort::Constants shaderConstants =
    {
        bitShiftSize,
        numElems,
        numGroups,
        inputKeysOffset,
        inputValuesOffset,
        histogramsOffset,
        outputKeysOffset,
        outputValuesOffset,
        m_deviceSettings.enableMortonCode30
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(&shaderConstants, RadixSort::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    RGP_PUSH_MARKER("Scatter Keys And Values");

    m_pPalCmdBuffer->CmdDispatch(numGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes multiple passes of several shaders to sort the input keys and values
void GpuBvhBuilder::SortRadixInt32()
{
    const uint32 inputKeysOffset    = m_scratchOffsets.mortonCodes;
    const uint32 outputKeysOffset   = m_scratchOffsets.mortonCodesSorted;
    // Passing in 0 for initial primitive indices since we auto-generate them for the first pass
    const uint32 inputValuesOffset  = 0;
    const uint32 outputValuesOffset = m_scratchOffsets.primIndicesSorted;

    const uint32 scratchKeysOffset   = m_scratchOffsets.tempKeys;
    const uint32 scratchValuesOffset = m_scratchOffsets.tempVals;

    uint32 currentFromKeys = inputKeysOffset;
    uint32 currentFromVals = inputValuesOffset;
    uint32 currentToKeys   = scratchKeysOffset;
    uint32 currentToVals   = scratchValuesOffset;

    uint32 bitShiftSize = 0;
    const uint32 numPasses = m_deviceSettings.enableMortonCode30 ?
                             m_radixSortConfig.numPasses30 :
                             m_radixSortConfig.numPasses;

    for (uint32 passIndex = 0; passIndex < numPasses; ++passIndex)
    {
        // Bit histogram
        BitHistogram(currentFromKeys, m_scratchOffsets.histogram, bitShiftSize, m_buildConfig.numLeafNodes);

        // Wait for the bit histogram to complete
        Barrier();

        // Scan histograms
        ScanExclusiveAdd(m_scratchOffsets.histogram, m_buildConfig.numHistogramElements);

        // Wait for the scan operation to complete
        Barrier();

        // Scatter keys
        ScatterKeysAndValues(currentFromKeys,
                             currentFromVals,
                             m_scratchOffsets.histogram,
                             currentToKeys,
                             currentToVals,
                             bitShiftSize,
                             m_buildConfig.numLeafNodes);

        if (bitShiftSize == 0)
        {
            currentFromKeys = outputKeysOffset;
            currentFromVals = outputValuesOffset;
        }

        // Swap resources
        Swap(&currentFromKeys, &currentToKeys);
        Swap(&currentFromVals, &currentToVals);

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
void GpuBvhBuilder::BuildParallel()
{
    if (m_buildConfig.radixSortScanLevel > 0)
    {
        PAL_ASSERT(m_buildConfig.numPrimitives < (m_radixSortConfig.numScanElemsPerWorkGroup *
                                                  m_radixSortConfig.numScanElemsPerWorkGroup *
                                                  m_radixSortConfig.numScanElemsPerWorkGroup));
    }

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

    BuildParallel::Constants shaderConstants = {};

    shaderConstants.numThreadGroups         = numThreadGroups;
    shaderConstants.numPrimitives           = m_buildConfig.numPrimitives;
    shaderConstants.maxNumPrimitives        = NumPrimitivesAfterSplit(m_buildConfig.numPrimitives, m_deviceSettings.triangleSplittingFactor);
    shaderConstants.rebraidFactor           = m_deviceSettings.rebraidFactor;

    if (m_buildConfig.topLevelBuild)
    {
        shaderConstants.encodeArrayOfPointers =
            (m_buildArgs.inputs.inputElemLayout == InputElementLayout::ArrayOfPointers);
    }
    else
    {
        shaderConstants.encodeArrayOfPointers = 0;
    }

    shaderConstants.offsets.mortonCodes          = m_scratchOffsets.mortonCodes;
    shaderConstants.offsets.mortonCodesSorted    = m_scratchOffsets.mortonCodesSorted;
    shaderConstants.offsets.primIndicesSorted    = m_scratchOffsets.primIndicesSorted;
    shaderConstants.offsets.primIndicesSortedSwap= m_scratchOffsets.primIndicesSortedSwap;
    shaderConstants.offsets.tempKeys             = m_scratchOffsets.tempKeys;
    shaderConstants.offsets.tempVals             = m_scratchOffsets.tempVals;
    shaderConstants.offsets.histogram            = m_scratchOffsets.histogram;
    shaderConstants.offsets.bvhNodes             = m_scratchOffsets.bvhNodeData;
    shaderConstants.offsets.unsortedBvhLeafNodes = m_scratchOffsets.bvhLeafNodeData;
    shaderConstants.offsets.sceneBounds          = m_scratchOffsets.sceneBounds;
    shaderConstants.offsets.partialSums          = m_scratchOffsets.distributedPartSums;
    shaderConstants.offsets.propagationFlags     = m_scratchOffsets.propagationFlags;
    shaderConstants.offsets.dynamicBlockIndex    = m_scratchOffsets.dynamicBlockIndex;
    shaderConstants.offsets.prefixSumAtomicFlags = m_scratchOffsets.atomicFlags;

    shaderConstants.offsets.clusterList0         = m_scratchOffsets.clusterList0;
    shaderConstants.offsets.clusterList1         = m_scratchOffsets.clusterList1;

    shaderConstants.offsets.neighborIndices      = m_scratchOffsets.neighbourIndices;

    shaderConstants.offsets.currentState         = m_scratchOffsets.currentState;
    shaderConstants.offsets.atomicFlagsPloc      = m_scratchOffsets.atomicFlagsPloc;
    shaderConstants.offsets.clusterOffsets       = m_scratchOffsets.clusterOffsets;

    shaderConstants.offsets.clusterList          = m_scratchOffsets.clusterList0;
    shaderConstants.offsets.numClusterList       = m_scratchOffsets.numClusterList0;
    shaderConstants.offsets.internalNodesIndex   = m_scratchOffsets.internalNodesIndex0;

    shaderConstants.offsets.qbvhStack            = m_scratchOffsets.qbvhGlobalStack;
    shaderConstants.offsets.stackPtrs            = m_scratchOffsets.qbvhGlobalStackPtrs;

    shaderConstants.offsets.splitBoxes          = m_scratchOffsets.triangleSplitBoxes;
    shaderConstants.offsets.refList0            = m_scratchOffsets.triangleSplitRefs0;
    shaderConstants.offsets.refList1            = m_scratchOffsets.triangleSplitRefs1;
    shaderConstants.offsets.splitPriorities     = m_scratchOffsets.splitPriorities;

    if (m_buildConfig.rebraidType == RebraidType::V2)
    {
        shaderConstants.offsets.currentSplitState = m_scratchOffsets.rebraidState;
    }
    else
    {
        shaderConstants.offsets.currentSplitState = m_scratchOffsets.triangleSplitState;
    }

    shaderConstants.offsets.splitAtomicFlags    = m_scratchOffsets.atomicFlagsTS;

    shaderConstants.offsets.numBatches          = m_scratchOffsets.numBatches;
    shaderConstants.offsets.batchIndices        = m_scratchOffsets.batchIndices;
    shaderConstants.offsets.indexBufferInfo     = m_scratchOffsets.indexBufferInfo;

    if (m_buildConfig.topDownBuild)
    {
        shaderConstants.offsets.tdRefs              = m_scratchOffsets.refList;
        shaderConstants.offsets.tdNodes             = m_scratchOffsets.tdNodeList;
        shaderConstants.offsets.tdBins              = m_scratchOffsets.tdBins;
        shaderConstants.offsets.tdState             = m_scratchOffsets.tdState;
        shaderConstants.offsets.tdTaskCounters      = m_scratchOffsets.tdTaskQueueCounter;
    }

    shaderConstants.offsets.debugCounters       = m_scratchOffsets.debugCounters;

    ResetTaskQueueCounters(m_scratchOffsets.tdTaskQueueCounter);
    ResetTaskQueueCounters(m_scratchOffsets.currentState);
    ResetTaskQueueCounters(m_scratchOffsets.rebraidState);
    ResetTaskQueueCounters(m_scratchOffsets.triangleSplitState);

    BindPipeline(InternalRayTracingCsType::BuildParallel);

    uint32 entryOffset = 0;

    entryOffset = WriteUserDataEntries(&shaderConstants, BuildParallel::NumEntries, entryOffset);
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

    RGP_PUSH_MARKER("BVH build");

    m_pPalCmdBuffer->CmdDispatch(numThreadGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the appropriate exclusive scan shader depending on the number of elements
void GpuBvhBuilder::ScanExclusiveAdd(
    uint32 inOutArrayOffset,  // Scratch offset of the input/output array buffer
    uint32 numElems)          // Number of elements
{
    if (numElems < m_radixSortConfig.scanThresholdOneLevel)
    {
        ScanExclusiveAddOneLevel(inOutArrayOffset, numElems, 1);
    }
    else if (numElems < m_radixSortConfig.scanThresholdTwoLevel)
    {
        if (m_buildConfig.radixSortScanLevel == 0)
        {
            ScanExclusiveAddDLB(inOutArrayOffset, numElems);
        }
        else
        {
            ScanExclusiveAddTwoLevel(inOutArrayOffset, numElems);
        }
    }
    else if (numElems < m_radixSortConfig.scanThresholdThreeLevel)
    {
        if (m_buildConfig.radixSortScanLevel == 0)
        {
            ScanExclusiveAddDLB(inOutArrayOffset, numElems);
        }
        else
        {
            ScanExclusiveAddThreeLevel(inOutArrayOffset, numElems);
        }
    }
    else
    {
        if (m_buildConfig.radixSortScanLevel == 0)
        {
            ScanExclusiveAddDLB(inOutArrayOffset, numElems);
        }
        else
        {
            PAL_ASSERT_ALWAYS_MSG("%s", "The maximum number of elements for scan exceeded\n");
        }
    }
}

// =====================================================================================================================
// Executes the work group exclusive scan shader
void GpuBvhBuilder::ScanExclusiveAddOneLevel(
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

    m_pPalCmdBuffer->CmdDispatch(numWorkGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the partial scan shader
void GpuBvhBuilder::ScanExclusiveAddPartial(
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
    m_pPalCmdBuffer->CmdDispatch(numWorkGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the partial sum distribution shader
void GpuBvhBuilder::ScanExclusiveDistributeSums(
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

    m_pPalCmdBuffer->CmdDispatch(numWorkGroups, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Executes the two level exclusive scan pass
void GpuBvhBuilder::ScanExclusiveAddTwoLevel(
    uint32 inOutArrayOffset,  // Scratch offset of the input/output array buffer
    uint32 numElems)          // Number of elements
{
    BindPipeline(InternalRayTracingCsType::ScanExclusivePartInt4);

    const uint32 numGroupsBottomLevelScan =
        Util::RoundUpQuotient(numElems, m_radixSortConfig.groupBlockSizeScan);

    const uint32 numGroupsTopLevelScan =
        Util::RoundUpQuotient(numGroupsBottomLevelScan, m_radixSortConfig.groupBlockSizeScan);

    const uint32 numGroupsBottomLevelDistribute =
        Util::RoundUpQuotient(numElems, m_radixSortConfig.groupBlockSizeDistribute);

    const uint32 partSumsOffset = m_scratchOffsets.distributedPartSums;

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
void GpuBvhBuilder::ScanExclusiveAddThreeLevel(
    uint32 inOutArrayOffset,  // Scratch offset of the input/output array buffer
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

    const uint32 partSumsBottomLevelOffset = m_scratchOffsets.distributedPartSums;
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
void GpuBvhBuilder::ScanExclusiveAddDLB(
    uint32 inOutArrayOffset,  // Scratch offset of the input/output array buffer
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
        inOutArrayOffset,
        m_scratchOffsets.dynamicBlockIndex,
        m_scratchOffsets.atomicFlags
    };

    // Set number of elements
    entryOffset = WriteUserDataEntries(&shaderConstants, ScanExclusiveAddDLB::NumEntries, entryOffset);

    // Set result and scratch buffers
    entryOffset = WriteDestBuffers(entryOffset);

    RGP_PUSH_MARKER("Init Scan Exclusive Int 4 DLB");

    // bottom level scan
    m_pPalCmdBuffer->CmdDispatch(numInitGroups, 1, 1);

    RGP_POP_MARKER();

    // Wait for the bottom level scan to complete
    Barrier();

    BindPipeline(InternalRayTracingCsType::ScanExclusiveInt4DLB);

    RGP_PUSH_MARKER("Scan Exclusive Int 4 DLB");

    m_pPalCmdBuffer->CmdDispatch(numBlocks, 1, 1);

    RGP_POP_MARKER();
}

// =====================================================================================================================
// Binds the pipeline that corresponds with the provided compute shader type
void GpuBvhBuilder::BindPipeline(
    InternalRayTracingCsType type) // Ray Tracing pipeline type
{
    const Pal::IPipeline* pPipeline = m_pDevice->GetInternalPipeline(type, m_buildSettings, m_buildSettingsHash);

    Pal::PipelineBindParams bindParam = {};
    bindParam.pipelineBindPoint       = Pal::PipelineBindPoint::Compute;
    bindParam.pPipeline               = pPipeline;
    bindParam.apiPsoHash              = GetInternalPsoHash(type, m_buildSettings);

#if GPURT_DEVELOPER
    OutputPipelineName(type);
#endif

    m_pPalCmdBuffer->CmdBindPipeline(bindParam);
}

// =====================================================================================================================
// Writes the provided entries into the compute shader user data slots
uint32 GpuBvhBuilder::WriteUserDataEntries(
    const void* pEntries,    // User data entries
    uint32      numEntries,  // Number of entries
    uint32      entryOffset) // Offset of the first entry
{
    return m_pDevice->WriteUserDataEntries(m_pPalCmdBuffer, pEntries, numEntries, entryOffset);
}

// =====================================================================================================================
// Writes a gpu virtual address for a buffer into the compute shader user data slots
uint32 GpuBvhBuilder::WriteBufferVa(
    gpusize virtualAddress, // GPUVA of the buffer
    uint32  entryOffset)    // Offset of the first entry
{
    return m_pDevice->WriteBufferVa(m_pPalCmdBuffer, virtualAddress, entryOffset);
}

// =====================================================================================================================
// Helper function to that calculates the number of primitives after applying the triangle split factor
uint32 GpuBvhBuilder::NumPrimitivesAfterSplit(
    uint32 primitiveCount, // Number of primitives
    float  splitFactor)    // Number of triangle split factor
{
    return static_cast<uint32>(static_cast<float>(primitiveCount) * splitFactor);
}

// =====================================================================================================================
// Calculates the optimal number of thread groups to be launched based on the current hardware being run
uint32 GpuBvhBuilder::GetOptimalNumThreadGroups(
    uint32 threadGroupSize) // Calculate number of thread groups to launch based on thread group size
{
    const auto*  pProps        = &m_deviceProps.gfxipProperties.shaderCore;
    const uint32 wavesPerGroup = Util::RoundUpQuotient(threadGroupSize, pProps->nativeWavefrontSize);

    return (pProps->numAvailableCus * pProps->numSimdsPerCu) / wavesPerGroup;
}

// =====================================================================================================================
uint32 GpuBvhBuilder::WriteDestBuffers(
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
uint32 GpuBvhBuilder::BuildModeFlags()
{
    uint32 buildModeFlags = 0;

    buildModeFlags |= m_buildConfig.triangleSplitting ? BuildModeTriangleSplitting : BuildModeNone;
    buildModeFlags |= m_buildConfig.collapse ? BuildModeCollapse : BuildModeNone;
    buildModeFlags |= (m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair) ?
                      BuildModePairCompression :
                      BuildModeNone;
    buildModeFlags |= m_deviceSettings.enablePairCompressionCostCheck ? BuildModePairCostCheck : BuildModeNone;

    return buildModeFlags;
}

}
