/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _GPURT_BUILD_SETTINGS_H
#define _GPURT_BUILD_SETTINGS_H

// This file provides the definition of the GPURT structures used by the GPURT BVH build shader code.
//
// Note this file is designed to be compilable as HLSL.

#ifdef __cplusplus
namespace GpuRt
{
#endif

typedef uint32_t uint32;
typedef uint64_t uint64;

#ifndef __cplusplus
#define constexpr static const
#endif

namespace GpuDebugFlags
{
    static const uint32 HostAssert = 1;
    static const uint32 HostPrint  = 2;
    static const uint32 ShaderHalt = 4;
};

namespace MortonFlags
{
    static const uint32 DefaultVariableBitMortonCodes = 0;
    static const uint32 EnableRegularMortonCodes = 1;
    static const uint32 EnableCentroidBounds = 2;
    static const uint32 EnableConciseBounds = 4;
    static const uint32 EnableCubeBounds = 8;
    static const uint32 EnablePerfectRectangleBounds = 16;
};

// Settings passed to the build shaders via compile time constant buffer. Note that adding settings here will cause
// additional shaders to be compiled when the setting changes.
struct CompileTimeBuildSettings
{
    uint32 topLevelBuild;
    uint32 buildMode;
    uint32 triangleCompressionMode;
    uint32 cullIllegalInstances;
    uint32 isIndirectBuild;
    uint32 fp16BoxNodesMode;
    float  fp16BoxModeMixedSaThreshold;
    uint32 radixSortScanLevel;
    uint32 emitCompactSize;
    uint32 enableBVHBuildDebugCounters;
    uint32 nnSearchRadius;
    uint32 enablePairCostCheck;
    uint32 mortonFlags;
    uint32 enableRebraid;
    uint32 unused1;
    uint32 useMortonCode30;
    uint32 enableMergeSort;
    uint32 unused2;
    uint32 enableFusedInstanceNode;
    uint32 unused3;
    uint32 numRebraidIterations;
    uint32 unused4;
    uint32 doEncode;
    uint32 unused5;
    uint32 enableEarlyPairCompression;
    uint32 unused6;
    uint32 rtIpLevel;
    uint32 geometryType;
    uint32 reserved2;
    uint32 reserved3;
    uint32 reserved4;
    uint32 reserved5;
    uint32 reserved6;
    uint32 reserved7;
    uint32 reserved8;
    uint32 reserved9;
    uint32 reserved10;
    uint32 reserved11;
    uint32 enableInstanceRebraid;
    uint32 gpuDebugFlags;
    uint32 updateFlags;
    uint32 isUpdateInPlace;
    uint32 encodeArrayOfPointers;
    uint32 sceneBoundsCalculationType;
    uint32 rebraidQualityHeuristic;
    uint32 reserved12;
    uint32 reserved13;
    uint32 disableCompaction;
    uint32 disableDegenPrims;
    uint32 reserved14;
    uint32 reserved15;
};

#define BUILD_SETTINGS_DATA_TOP_LEVEL_BUILD_ID                        0
#define BUILD_SETTINGS_DATA_BUILD_MODE_ID                             1
#define BUILD_SETTINGS_DATA_TRIANGLE_COMPRESSION_MODE_ID              2
#define BUILD_SETTINGS_DATA_CULL_ILLEGAL_INSTANCES_ID                 3
#define BUILD_SETTINGS_DATA_IS_INDIRECT_BUILD_ID                      4
#define BUILD_SETTINGS_DATA_FP16_BOX_NODES_MODE_ID                    5
#define BUILD_SETTINGS_DATA_FP16_BOX_MODE_MIXED_SA_THRESHOLD_ID       6
#define BUILD_SETTINGS_DATA_RADIX_SORT_SCAN_LEVEL_ID                  7
#define BUILD_SETTINGS_DATA_EMIT_COMPACT_SIZE_ID                      8
#define BUILD_SETTINGS_DATA_ENABLE_BVH_BUILD_DEBUG_COUNTERS_ID        9
#define BUILD_SETTINGS_DATA_NN_SEARCH_RADIUS_ID                       10
#define BUILD_SETTINGS_DATA_ENABLE_PAIR_COST_CHECK_ID                 11
#define BUILD_SETTINGS_DATA_MORTON_FLAGS_ID                           12
#define BUILD_SETTINGS_DATA_ENABLE_REBRAID_ID                         13
// unused1 id                                                         14
#define BUILD_SETTINGS_DATA_USE_MORTON_CODE_30_ID                     15
#define BUILD_SETTINGS_DATA_ENABLE_MERGE_SORT_ID                      16
// unused2 id                                                         17
#define BUILD_SETTINGS_DATA_ENABLE_FUSED_INSTANCE_NODE_ID             18
// unused3 id                                                         19
#define BUILD_SETTINGS_DATA_NUM_REBRAID_ITERATIONS_ID                 20
// unused4 id                                                         21
#define BUILD_SETTINGS_DATA_DO_ENCODE_ID                              22
// unused5 id                                                         23
#define BUILD_SETTINGS_DATA_ENABLE_EARLY_PAIR_COMPRESSION_ID          24
// unused6 id                                                         25
#define BUILD_SETTINGS_DATA_RTIP_LEVEL_ID                             26
#define BUILD_SETTINGS_DATA_GEOMETRY_TYPE_ID                          27
#define BUILD_SETTINGS_DATA_ENABLE_INSTANCE_REBRAID_ID                38
#define BUILD_SETTINGS_DATA_GPU_DEBUG_FLAGS_ID                        39
#define BUILD_SETTINGS_DATA_UPDATE_FLAGS_ID                           40
#define BUILD_SETTINGS_DATA_IS_UPDATE_IN_PLACE_ID                     41
#define BUILD_SETTINGS_DATA_ENCODE_ARRAY_OF_POINTERS_ID               42
#define BUILD_SETTINGS_DATA_SCENE_BOUNDS_CALCULATION_TYPE_ID          43
#define BUILD_SETTINGS_DATA_REBRAID_QUALITY_HEURISTIC_ID              44
#define BUILD_SETTINGS_DATA_DISABLE_COMPACTION_ID                     47
#define BUILD_SETTINGS_DATA_DISABLE_DEGEN_PRIMS_ID                    48

#ifdef __cplusplus
} // namespace GpuRt
#endif

#endif
