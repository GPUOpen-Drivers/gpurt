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
#pragma once
#include "gpurt/gpurt.h"
#include "gpurt/gpurtAccelStruct.h"
#include "palInlineFuncs.h"

namespace GpuRt
{

// =====================================================================================================================
// Constants needed for raytracing

// Sizes of the node structures used by the BVH building shaders
static constexpr size_t RayTracingQBVHLeafSize          = 64;
static constexpr size_t RayTracingScratchNodeSize       = 64;
static constexpr size_t RayTracingQBVHStackPtrsSize     = 12;
static constexpr size_t RayTracingCollapseTaskSize      = 16;
static constexpr size_t RayTracingCollapseTaskPtrSize   = 8;
static constexpr size_t RayTracingQBVHCollapseTaskSize  = 24;
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
static constexpr size_t RayTracingStateTSBuildSize      = 40;
static constexpr size_t RayTracingAtomicFlags           = 8;

static constexpr size_t RayTracingStateRebraidBuildSize = 28;

static constexpr size_t RayTracingBuildDebugCounters    = 12;

static constexpr size_t RayTracingTaskQueueCounterSize  = 20;
static constexpr size_t RayTracingTaskQueueCounters     = 5;

static_assert(((RayTracingTDNodeSize % 8) == 0),
    "TDNode size must be 8-byte aligned to ensure that 64-bit atomic operations on the first field work correctly.");

constexpr uint32 DefaultThreadGroupSize = 64;

enum EncodeFlags : uint32
{
    EncodeFlagArrayOfPointers = 1,
    EncodeFlagUpdateInPlace   = 2,
    EncodeFlagRebraidEnabled  = 4,
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

// =====================================================================================================================
// Scratch data offsets required for BVH construction
struct RayTracingScratchDataOffsets
{
    uint32 bvhNodeData;
    uint32 triangleSplitBoxes;
    uint32 triangleSplitRefs0;
    uint32 triangleSplitRefs1;
    uint32 splitPriorities;
    uint32 triangleSplitState;
    uint32 rebraidState;
    uint32 atomicFlagsTS;
    uint32 refList;
    uint32 tdNodeList;
    uint32 tdBins;
    uint32 tdState;
    uint32 tdTaskQueueCounter;
    uint32 refOffsets;
    uint32 bvhLeafNodeData;
    uint32 clusterList0;        // BVH AC build only
    uint32 clusterList1;        // BVH AC build only
    uint32 numClusterList0;     // BVH AC build only
    uint32 numClusterList1;     // BVH AC build only
    uint32 internalNodesIndex0; // BVH AC build only
    uint32 internalNodesIndex1; // BVH AC build only
    uint32 neighbourIndices;    // BVH PLOC build only
    uint32 currentState;        // BVH PLOC build only
    uint32 atomicFlagsPloc;     // BVH PLOC build only
    uint32 clusterOffsets;      // BVH PLOC build only
    uint32 sceneBounds;
    uint32 mortonCodes;
    uint32 mortonCodesSorted;
    uint32 primIndicesSorted;
    uint32 primIndicesSortedSwap;
    uint32 propagationFlags;
    uint32 numBatches;
    uint32 batchIndices;
    uint32 indexBufferInfo;
    uint32 updateStack;
    uint32 histogram;
    uint32 tempKeys;
    uint32 tempVals;
    uint32 dynamicBlockIndex;
    uint32 atomicFlags;
    uint32 distributedPartSums;
    uint32 qbvhGlobalStack;
    uint32 qbvhGlobalStackPtrs;
    uint32 debugCounters;
};

// Starting logical ID value for all logical IDs used by internal pipelines.
const uint32 ReservedLogicalIdCount = 1;

// Array of internal pipeline source code
extern const PipelineBuildInfo InternalPipelineBuildInfo[size_t(InternalRayTracingCsType::Count)];

extern uint32 CalculateRayTracingAABBCount(
    const AccelStructBuildInputs& buildInfo);

// =====================================================================================================================
// Calculate acceleration structure internal node count for QBVH
inline uint32 CalcNumQBVHInternalNodes(
    uint32 primitiveCount)    // Primitive node count (instances or triangle/aabb geometry)
{
    return Util::Max(1U, (2 * primitiveCount) / 3);
}

// =====================================================================================================================
// Gets geometry type for BLAS build inputs
static GeometryType GetGeometryType(
    const AccelStructBuildInputs inputs)
{
    GeometryType type;

    const bool isBottomLevel = (inputs.type == AccelStructType::BottomLevel);

    if (isBottomLevel && (inputs.inputElemCount > 0))
    {
        const Geometry geometry = ClientConvertAccelStructBuildGeometry(inputs, 0);
        type = geometry.type;
    }
    else
    {
        // No geometries, so pick an arbitrary geometry type to initialize the variable
        type = GeometryType::Triangles;
    }

    return type;
}

// Layout used for build shader PSO hashes.
union BuildShaderPsoHash
{
    struct
    {
        uint64 shaderType              : 18;
        uint64 topLevelBuild           : 1;
        uint64 buildMode               : 2;
        uint64 reserved                : 1;
        uint64 triangleCompressionMode : 2;
        uint64 doTriangleSplitting     : 1;
        uint64 doCollapse              : 1;
        uint64 fp16BoxNodesMode        : 2;
        uint64 radixSortScanLevel      : 2;
        uint64 rebraidType             : 2;
        uint64 hashPrefix              : 32;
    };

    uint64 u64All;
};

//=====================================================================================================================
// Generate 64-bit PSO hash for internal build shader type
static uint64 GetInternalPsoHash(
    InternalRayTracingCsType        type,
    const CompileTimeBuildSettings& buildSettings)
{
    // Identifies ray-tracing-related dispatches for tools
    constexpr uint64 RayTracingPsoHashPrefix = 0xEEE5FFF6;

    BuildShaderPsoHash hash{};
    hash.shaderType              = static_cast<uint64>(type);
    hash.topLevelBuild           = buildSettings.topLevelBuild;
    hash.buildMode               = buildSettings.buildMode;
    hash.triangleCompressionMode = buildSettings.triangleCompressionMode;
    hash.doTriangleSplitting     = buildSettings.doTriangleSplitting;
    hash.doCollapse              = buildSettings.doCollapse;
    hash.fp16BoxNodesMode        = buildSettings.fp16BoxNodesMode;
    hash.radixSortScanLevel      = buildSettings.radixSortScanLevel;
    hash.rebraidType             = buildSettings.rebraidType;
    hash.hashPrefix              = RayTracingPsoHashPrefix;

    return hash.u64All;
}
}; // namespace GpuRt
