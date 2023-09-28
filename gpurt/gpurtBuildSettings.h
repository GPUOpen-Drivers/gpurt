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

// Settings passed to the build shaders via compile time constant buffer. Note that adding settings here will cause
// additional shaders to be compiled when the setting changes.
struct CompileTimeBuildSettings
{
    uint32 topLevelBuild;
    uint32 buildMode;
    uint32 triangleCompressionMode;
    uint32 doTriangleSplitting;
    uint32 doCollapse;
    uint32 fp16BoxNodesMode;
    float  fp16BoxModeMixedSaThreshold;
    uint32 radixSortScanLevel;
    uint32 emitCompactSize;
    uint32 enableBVHBuildDebugCounters;
    uint32 plocRadius;
    uint32 enablePairCostCheck;
    uint32 enableVariableBitsMortonCode;
    uint32 rebraidType;
    uint32 enableTopDownBuild;
    uint32 useMortonCode30;
    uint32 enableMergeSort;
    uint32 fastBuildThreshold;
    uint32 enableFusedInstanceNode;
    float  tsPriority;
    uint32 noCopySortedNodes;
    uint32 enableSAHCost;
    uint32 unused0;
    uint32 doEncode;
    uint32 unused1;
    uint32 enableEarlyPairCompression;
    uint32 enableFastLBVH;
    uint32 rtIpLevel;
    uint32 geometryType;
    uint32 unused2;
    uint32 unused3;
    uint32 unused4;
    uint32 unused5;
    uint32 unused6;
    uint32 unused7;
    uint32 unused8;
    uint32 enableInstanceRebraid;
    uint32 gpuDebugFlags;
    uint32 isUpdate;
};

#define BUILD_SETTINGS_DATA_TOP_LEVEL_BUILD_ID                        0
#define BUILD_SETTINGS_DATA_BUILD_MODE_ID                             1
#define BUILD_SETTINGS_DATA_TRIANGLE_COMPRESSION_MODE_ID              2
#define BUILD_SETTINGS_DATA_DO_TRIANGLE_SPLITTING_ID                  3
#define BUILD_SETTINGS_DATA_DO_COLLAPSE_ID                            4
#define BUILD_SETTINGS_DATA_FP16_BOX_NODES_MODE_ID                    5
#define BUILD_SETTINGS_DATA_FP16_BOX_MODE_MIXED_SA_THRESHOLD_ID       6
#define BUILD_SETTINGS_DATA_RADIX_SORT_SCAN_LEVEL_ID                  7
#define BUILD_SETTINGS_DATA_EMIT_COMPACT_SIZE_ID                      8
#define BUILD_SETTINGS_DATA_ENABLE_BVH_BUILD_DEBUG_COUNTERS_ID        9
#define BUILD_SETTINGS_DATA_PLOC_RADIUS_ID                            10
#define BUILD_SETTINGS_DATA_ENABLE_PAIR_COST_CHECK_ID                 11
#define BUILD_SETTINGS_DATA_ENABLE_VARIABLE_BITS_MC_ID                12
#define BUILD_SETTINGS_DATA_REBRAID_TYPE_ID                           13
#define BUILD_SETTINGS_DATA_ENABLE_TOP_DOWN_BUILD_ID                  14
#define BUILD_SETTINGS_DATA_USE_MORTON_CODE_30_ID                     15
#define BUILD_SETTINGS_DATA_ENABLE_MERGE_SORT_ID                      16
#define BUILD_SETTINGS_DATA_FAST_BUILD_THRESHOLD_ID                   17
#define BUILD_SETTINGS_DATA_ENABLE_FUSED_INSTANCE_NODE_ID             18
#define BUILD_SETTINGS_DATA_TS_PRIORITY_ID                            19
#define BUILD_SETTINGS_DATA_NO_COPY_SORTED_NODES_ID                   20
#define BUILD_SETTINGS_DATA_ENABLE_SAH_COST_ID                        21
#define BUILD_SETTINGS_DATA_DO_ENCODE_ID                              23
#define BUILD_SETTINGS_DATA_ENABLE_EARLY_PAIR_COMPRESSION_ID          25
#define BUILD_SETTINGS_DATA_ENABLE_FAST_LBVH_ID                       26
#define BUILD_SETTINGS_DATA_RTIP_LEVEL_ID                             27
#define BUILD_SETTINGS_DATA_GEOMETRY_TYPE_ID                          28
#define BUILD_SETTINGS_DATA_ENABLE_INSTANCE_REBRAID_ID                36
#define BUILD_SETTINGS_DATA_GPU_DEBUG_FLAGS_ID                        37
#define BUILD_SETTINGS_DATA_IS_UPDATE_ID                              38

#ifdef __cplusplus
} // namespace GpuRt
#endif

#endif
