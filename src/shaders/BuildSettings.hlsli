/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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

[[vk::constant_id(BUILD_SETTINGS_DATA_TOP_LEVEL_BUILD_ID)]]                        uint topLevelBuild                 = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_BUILD_MODE_ID)]]                             uint buildMode                     = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_TRIANGLE_COMPRESSION_MODE_ID)]]              uint triangleCompressionMode       = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_DO_TRIANGLE_SPLITTING_ID)]]                  uint doTriangleSplitting           = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_DO_COLLAPSE_ID)]]                            uint doCollapse                    = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_FP16_BOX_NODES_MODE_ID)]]                    uint fp16BoxNodesMode              = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_FP16_BOX_MODE_MIXED_SA_THRESHOLD_ID)]]       float fp16BoxModeMixedSaThreshold  = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_RADIX_SORT_SCAN_LEVEL_ID)]]                  uint radixSortScanLevel            = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_EMIT_COMPACT_SIZE_ID)]]                      uint emitCompactSize               = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_BVH_BUILD_DEBUG_COUNTERS_ID)]]        uint enableBVHBuildDebugCounters   = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_PLOC_RADIUS_ID)]]                            uint plocRadius                    = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_PAIR_COST_CHECK_ID)]]                 uint enablePairCostCheck           = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_VARIABLE_BITS_MC_ID)]]                uint enableVariableBitsMortonCode  = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_REBRAID_TYPE_ID)]]                           uint rebraidType                   = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_TOP_DOWN_BUILD_ID)]]                  uint enableTopDownBuild            = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_USE_MORTON_CODE_30_ID)]]                     uint useMortonCode30               = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_MERGE_SORT_ID)]]                      uint enableMergeSort               = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_FAST_BUILD_THRESHOLD_ID)]]                   uint fastBuildThreshold            = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_FUSED_INSTANCE_NODE_ID)]]             uint enableFusedInstanceNode       = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_TS_PRIORITY_ID)]]                            float tsPriority                   = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_NO_COPY_SORTED_NODES_ID)]]                   uint noCopySortedNodes             = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_SAH_COST_ID)]]                        uint enableSAHCost                 = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_DO_ENCODE_ID)]]                              uint doEncode                      = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_EARLY_PAIR_COMPRESSION_ID)]]          uint enableEarlyPairCompression    = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_FAST_LBVH_ID)]]                       uint enableFastLBVH                = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_RTIP_LEVEL_ID)]]                             uint rtIpLevel                     = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_GEOMETRY_TYPE_ID)]]                          uint geometryType                  = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_INSTANCE_REBRAID_ID)]]                uint enableInstanceRebraid         = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_GPU_DEBUG_FLAGS_ID)]]                        uint gpuDebugFlags                 = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_IS_UPDATE_ID)]]                              uint isUpdate                      = 0;

static const BuildSettingsData Settings = {
    topLevelBuild,
    buildMode,
    triangleCompressionMode,
    doTriangleSplitting,
    doCollapse,
    fp16BoxNodesMode,
    fp16BoxModeMixedSaThreshold,
    radixSortScanLevel,
    emitCompactSize,
    enableBVHBuildDebugCounters,
    plocRadius,
    enablePairCostCheck,
    enableVariableBitsMortonCode,
    rebraidType,
    enableTopDownBuild,
    useMortonCode30,
    enableMergeSort,
    fastBuildThreshold,
    enableFusedInstanceNode,
    tsPriority,
    noCopySortedNodes,
    enableSAHCost,
    0,
    doEncode,
    0,
    enableEarlyPairCompression,
    enableFastLBVH,
    rtIpLevel,
    geometryType,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    enableInstanceRebraid,
    gpuDebugFlags,
    isUpdate,
};

#endif
