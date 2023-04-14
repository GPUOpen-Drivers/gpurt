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
    float  fp16BoxModeMixedSaThreshhold;
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
    uint32 unused2;
    uint32 unused3;
    uint32 unused4;
};

#define BUILD_SETTINGS_DATA_TOP_LEVEL_BUILD_OFFSET                        0
#define BUILD_SETTINGS_DATA_BUILD_MODE_OFFSET                             4
#define BUILD_SETTINGS_DATA_TRIANGLE_COMPRESSION_MODE_OFFSET              8
#define BUILD_SETTINGS_DATA_DO_TRIANGLE_SPLITTING_OFFSET                  12
#define BUILD_SETTINGS_DATA_DO_COLLAPSE_OFFSET                            16
#define BUILD_SETTINGS_DATA_FP16_BOX_NODES_MODE_OFFSET                    20
#define BUILD_SETTINGS_DATA_FP16_BOX_MODE_MIXED_SA_THRESHHOLD_OFFSET      24
#define BUILD_SETTINGS_DATA_RADIX_SORT_SCAN_LEVEL_OFFSET                  28
#define BUILD_SETTINGS_DATA_EMIT_COMPACT_SIZE_OFFSET                      32
#define BUILD_SETTINGS_DATA_ENABLE_BVH_BUILD_DEBUG_COUNTERS_OFFSET        36
#define BUILD_SETTINGS_DATA_PLOC_RADIUS                                   40
#define BUILD_SETTINGS_DATA_ENABLE_PAIR_COST_CHECK_OFFSET                 44
#define BUILD_SETTINGS_DATA_ENABLE_VARIABLE_BITS_MC_OFFSET                48
#define BUILD_SETTINGS_DATA_REBRAID_TYPE_OFFSET                           52
#define BUILD_SETTINGS_DATA_ENABLE_TOP_DOWN_BUILD_OFFSET                  56
#define BUILD_SETTINGS_DATA_USE_MORTON_CODE_30_OFFSET                     60
#define BUILD_SETTINGS_DATA_ENABLE_MERGE_SORT_OFFSET                      64
#define BUILD_SETTINGS_DATA_FAST_BUILD_THRESHOLD_OFFSET                   68
#define BUILD_SETTINGS_DATA_ENABLE_FUSED_INSTANCE_NODE_OFFSET             72
#define BUILD_SETTINGS_DATA_TS_PRIORITY_OFFSET                            76
#define BUILD_SETTINGS_DATA_NO_COPY_SORTED_NODES_OFFSET                   80
#define BUILD_SETTINGS_DATA_ENABLE_SAH_COST_OFFSET                        84
#define BUILD_SETTINGS_DATA_UNUSED0_OFFSET                                88
#define BUILD_SETTINGS_DATA_DO_ENCODE                                     92
#define BUILD_SETTINGS_DATA_UNUSED1_OFFSET                                96
#define BUILD_SETTINGS_DATA_ENABLE_EARLY_PAIR_COMPRESSION_OFFSET          100
#define BUILD_SETTINGS_DATA_ENABLE_FAST_LBVH_OFFSET                       104
#define BUILD_SETTINGS_DATA_RTIP_LEVEL_OFFSET                             108
#define BUILD_SETTINGS_DATA_UNUSED2_OFFSET                                112
#define BUILD_SETTINGS_DATA_UNUSED3_OFFSET                                116
#define BUILD_SETTINGS_DATA_UNUSED4_OFFSET                                120
#define BUILD_SETTINGS_DATA_SIZE                                          124

#define BUILD_SETTINGS_DATA_TOP_LEVEL_BUILD_ID                        (BUILD_SETTINGS_DATA_TOP_LEVEL_BUILD_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_BUILD_MODE_ID                             (BUILD_SETTINGS_DATA_BUILD_MODE_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_TRIANGLE_COMPRESSION_MODE_ID              (BUILD_SETTINGS_DATA_TRIANGLE_COMPRESSION_MODE_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_DO_TRIANGLE_SPLITTING_ID                  (BUILD_SETTINGS_DATA_DO_TRIANGLE_SPLITTING_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_DO_COLLAPSE_ID                            (BUILD_SETTINGS_DATA_DO_COLLAPSE_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_FP16_BOX_NODES_MODE_ID                    (BUILD_SETTINGS_DATA_FP16_BOX_NODES_MODE_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_FP16_BOX_MODE_MIXED_SA_THRESHHOLD_ID      (BUILD_SETTINGS_DATA_FP16_BOX_MODE_MIXED_SA_THRESHHOLD_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_RADIX_SORT_SCAN_LEVEL_ID                  (BUILD_SETTINGS_DATA_RADIX_SORT_SCAN_LEVEL_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_EMIT_COMPACT_SIZE_ID                      (BUILD_SETTINGS_DATA_EMIT_COMPACT_SIZE_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_BVH_BUILD_DEBUG_COUNTERS_ID        (BUILD_SETTINGS_DATA_ENABLE_BVH_BUILD_DEBUG_COUNTERS_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_PLOC_RADIUS_ID                            (BUILD_SETTINGS_DATA_PLOC_RADIUS  / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_PAIR_COST_CHECK_ID                 (BUILD_SETTINGS_DATA_ENABLE_PAIR_COST_CHECK_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_VARIABLE_BITS_MC_ID                (BUILD_SETTINGS_DATA_ENABLE_VARIABLE_BITS_MC_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_REBRAID_TYPE_ID                           (BUILD_SETTINGS_DATA_REBRAID_TYPE_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_TOP_DOWN_BUILD_ID                  (BUILD_SETTINGS_DATA_ENABLE_TOP_DOWN_BUILD_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_USE_MORTON_CODE_30_ID                     (BUILD_SETTINGS_DATA_USE_MORTON_CODE_30_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_MERGE_SORT_ID                      (BUILD_SETTINGS_DATA_ENABLE_MERGE_SORT_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_FAST_BUILD_THRESHOLD_ID                   (BUILD_SETTINGS_DATA_FAST_BUILD_THRESHOLD_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_FUSED_INSTANCE_NODE_ID             (BUILD_SETTINGS_DATA_ENABLE_FUSED_INSTANCE_NODE_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_TS_PRIORITY_ID                            (BUILD_SETTINGS_DATA_TS_PRIORITY_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_NO_COPY_SORTED_NODES_ID                   (BUILD_SETTINGS_DATA_NO_COPY_SORTED_NODES_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_SAH_COST_ID                        (BUILD_SETTINGS_DATA_ENABLE_SAH_COST_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_DO_ENCODE_ID                              (BUILD_SETTINGS_DATA_DO_ENCODE / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_EARLY_PAIR_COMPRESSION_ID          (BUILD_SETTINGS_DATA_ENABLE_EARLY_PAIR_COMPRESSION_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_FAST_LBVH_ID                       (BUILD_SETTINGS_DATA_ENABLE_FAST_LBVH_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_RTIP_LEVEL_ID                             (BUILD_SETTINGS_DATA_RTIP_LEVEL_OFFSET / sizeof(uint))

#ifdef __cplusplus
} // namespace GpuRt
#endif

#endif
