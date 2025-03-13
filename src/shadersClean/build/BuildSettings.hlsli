/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
//When you include this file, don't forget to bind b255 to the shader root signature.
#ifndef _BUILDSETTINGS_HLSLI
#define _BUILDSETTINGS_HLSLI

#include "../common/ShaderDefs.hlsli"

[[vk::constant_id(BUILD_SETTINGS_DATA_TOP_LEVEL_BUILD_ID)]]                        uint topLevelBuild                 = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_BUILD_MODE_ID)]]                             uint buildMode                     = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_TRIANGLE_COMPRESSION_MODE_ID)]]              uint triangleCompressionMode       = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_CULL_ILLEGAL_INSTANCES_ID)]]                 uint cullIllegalInstances          = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_IS_INDIRECT_BUILD_ID)]]                      uint isIndirectBuild               = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_FP16_BOX_NODES_MODE_ID)]]                    uint fp16BoxNodesMode              = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_FP16_BOX_MODE_MIXED_SA_THRESHOLD_ID)]]       float fp16BoxModeMixedSaThreshold  = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_RADIX_SORT_SCAN_LEVEL_ID)]]                  uint radixSortScanLevel            = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_EMIT_COMPACT_SIZE_ID)]]                      uint emitCompactSize               = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_BVH_BUILD_DEBUG_COUNTERS_ID)]]        uint enableBVHBuildDebugCounters   = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_NN_SEARCH_RADIUS_ID)]]                       uint nnSearchRadius                = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_PAIR_COST_CHECK_ID)]]                 uint enablePairCostCheck           = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_MORTON_FLAGS_ID)]]                           uint mortonFlags                   = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_REBRAID_ID)]]                         uint enableRebraid                 = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_USE_MORTON_CODE_30_ID)]]                     uint useMortonCode30               = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_MERGE_SORT_ID)]]                      uint enableMergeSort               = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_FUSED_INSTANCE_NODE_ID)]]             uint enableFusedInstanceNode       = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_NUM_REBRAID_ITERATIONS_ID)]]                 uint numRebraidIterations          = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_DO_ENCODE_ID)]]                              uint doEncode                      = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_EARLY_PAIR_COMPRESSION_ID)]]          uint enableEarlyPairCompression    = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_RTIP_LEVEL_ID)]]                             uint rtIpLevel                     = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_GEOMETRY_TYPE_ID)]]                          uint geometryType                  = 0;
#if GPURT_BUILD_RTIP3
[[vk::constant_id(BUILD_SETTINGS_DATA_HIGH_PRECISION_BOX_NODE_ENABLE_ID)]]         uint highPrecisionBoxNodeEnable    = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_BVH8_ENABLE_ID)]]                            uint bvh8Enable                    = 0;
#endif
#if GPURT_BUILD_RTIP3_1
[[vk::constant_id(BUILD_SETTINGS_DATA_PRIM_COMPRESSION_FLAGS_ID)]]                 uint primCompressionFlags          = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_ORIENTED_BOUNDING_BOXES_ID)]]         uint enableOrientedBoundingBoxes   = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_OBB_NUM_LEVELS_ID)]]                         uint obbNumLevels                  = 0xFFFFFFFF;
[[vk::constant_id(BUILD_SETTINGS_DATA_INSTANCE_MODE_ID)]]                          uint instanceMode                  = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_MAX_PRIM_RANGE_SIZE_ID)]]                    uint maxPrimRangeSize              = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_BOX_SPLITTING_FLAGS_ID)]]                    uint boxSplittingFlags             = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_BVH_CHANNEL_REBALANCING_ID)]]         uint enableBvhChannelBalancing     = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_TLAS_REFITTING_MODE)]]                       uint tlasRefittingMode             = 0;
#endif
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_INSTANCE_REBRAID_ID)]]                uint enableInstanceRebraid         = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_GPU_DEBUG_FLAGS_ID)]]                        uint gpuDebugFlags                 = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_UPDATE_FLAGS_ID)]]                           uint updateFlags                   = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_IS_UPDATE_IN_PLACE_ID)]]                     uint isUpdateInPlace               = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENCODE_ARRAY_OF_POINTERS_ID)]]               uint encodeArrayOfPointers         = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_SCENE_BOUNDS_CALCULATION_TYPE_ID)]]          uint sceneBoundsCalculationType    = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_REBRAID_QUALITY_HEURISTIC_ID)]]              uint rebraidQualityHeuristic       = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_DISABLE_COMPACTION_ID)]]                     uint disableCompaction             = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_DISABLE_DEGEN_PRIMS_ID)]]                    uint disableDegenPrims             = 0;
#if GPURT_BUILD_RTIP3_1
[[vk::constant_id(BUILD_SETTINGS_DATA_REBRAID_OPEN_SA_FACTOR_ID)]]                 float rebraidOpenSAFactor          = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_REBRAID_OPEN_MIN_PRIMS_ID)]]                 uint rebraidOpenMinPrims           = 0;
#endif

static const CompileTimeBuildSettings Settings = {
    topLevelBuild,
    buildMode,
    triangleCompressionMode,
    cullIllegalInstances,
    isIndirectBuild,
    fp16BoxNodesMode,
    fp16BoxModeMixedSaThreshold,
    radixSortScanLevel,
    emitCompactSize,
    enableBVHBuildDebugCounters,
    nnSearchRadius,
    enablePairCostCheck,
    mortonFlags,
    enableRebraid,
    0,
    useMortonCode30,
    enableMergeSort,
    0,
    enableFusedInstanceNode,
    0,
    numRebraidIterations,
    0,
    doEncode,
    0,
    enableEarlyPairCompression,
    0,
    rtIpLevel,
    geometryType,
#if GPURT_BUILD_RTIP3
    highPrecisionBoxNodeEnable,
    bvh8Enable,
#else
    0,
    0,
#endif
#if GPURT_BUILD_RTIP3_1
    primCompressionFlags,
    enableOrientedBoundingBoxes,
    obbNumLevels,
    instanceMode,
    maxPrimRangeSize,
    boxSplittingFlags,
    enableBvhChannelBalancing,
    tlasRefittingMode,
#else
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
#endif
    enableInstanceRebraid,
    gpuDebugFlags,
    updateFlags,
    isUpdateInPlace,
    encodeArrayOfPointers,
    sceneBoundsCalculationType,
    rebraidQualityHeuristic,
    0,
    0,
    disableCompaction,
    disableDegenPrims,
#if GPURT_BUILD_RTIP3_1
    rebraidOpenSAFactor,
    rebraidOpenMinPrims,
#else
    0,
    0,
#endif
};

#endif
