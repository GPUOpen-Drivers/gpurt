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
// This header file contains shared definitions between the HLSL raytracing shaders and the prototype c++ code
//
#ifndef _RAYTRACING_DEF_H
#define _RAYTRACING_DEF_H

#include "../../gpurt/gpurtAccelStruct.h"
#include "../../gpurt/gpurtBuildSettings.h"
#include "../../gpurt/gpurtDispatch.h"
#ifndef GPURT_BVH_BUILD_SHADER
#include "../../gpurt/gpurtTraceSettings.h"
#endif
#include "accelStruct.h"
#include "gpurtBuildConstants.h"
#ifdef __cplusplus
#include "hlslTypes.h"
#endif

// Due to lack of enum support in HLSL, we have these defines which should match Pal::RayTracingIpLevel in palDevice.h.
#define GPURT_RTIP1_0 1
#define GPURT_RTIP1_1 2
#define GPURT_RTIP2_0 3
#if GPURT_BUILD_RTIP3
#define GPURT_RTIP3_0 4
#endif

#if GPURT_BUILD_RTIP3_1
#if PAL_CLIENT_INTERFACE_MAJOR_VERSION >= 888
#define GPURT_RTIP3_1 5
#else
#define GPURT_RTIP3_1 6
#endif
#endif

#ifdef __cplusplus
static_assert(GPURT_RTIP1_0 == uint32_t(Pal::RayTracingIpLevel::RtIp1_0), "GPURT_HLSL_RTIP mismatch.");
static_assert(GPURT_RTIP1_1 == uint32_t(Pal::RayTracingIpLevel::RtIp1_1), "GPURT_HLSL_RTIP mismatch.");
static_assert(GPURT_RTIP2_0 == uint32_t(Pal::RayTracingIpLevel::RtIp2_0), "GPURT_HLSL_RTIP mismatch.");
#if GPURT_BUILD_RTIP3
static_assert(GPURT_RTIP3_0 == uint32_t(Pal::RayTracingIpLevel::RtIp3_0), "GPURT_HLSL_RTIP mismatch.");
#endif
#if GPURT_BUILD_RTIP3_1
static_assert(GPURT_RTIP3_1 == uint32_t(Pal::RayTracingIpLevel::RtIp3_1), "GPURT_HLSL_RTIP mismatch.");
#endif
#endif

//=====================================================================================================================
#define REBRAID_PHASE_CALC_SUM                 0
#define REBRAID_PHASE_OPEN                     1
#define REBRAID_PHASE_DONE                     2

struct RebraidState
{
    float                   sumValue[2];
    uint                    mutex;
    uint                    numLeafIndices;
    uint                    iterationCount;
};

#define STATE_REBRAID_SUM_VALUE_OFFSET          0
#define STATE_REBRAID_MUTEX_OFFSET              (STATE_REBRAID_SUM_VALUE_OFFSET + 8)
#define STATE_REBRAID_NUM_LEAF_INDICES_OFFSET   (STATE_REBRAID_MUTEX_OFFSET + 4)
#define STATE_REBRAID_ITERATION_COUNT_OFFSET    (STATE_REBRAID_NUM_LEAF_INDICES_OFFSET + 4)

#define REBRAID_KEYS_PER_THREAD                 4
#define REBRAID_KEYS_PER_GROUP                  (BUILD_THREADGROUP_SIZE * REBRAID_KEYS_PER_THREAD)

//======================================================================================================================
// matches VkAccelerationStructureBuildRangeInfoKHR
struct IndirectBuildRangeInfo
{
    uint primitiveCount;
    uint primitiveOffset;
    uint firstVertex;
    uint transformOffset;
};

//=====================================================================================================================
#if GPURT_BUILD_RTIP3_1

// All nodes are 128 bytes
#define RTIP3_1_NODE_SIZE                      128

//=====================================================================================================================
struct ObbKdop
{
    float2 minMax[13];
    uint obbMatrixIndex;
    uint __padding[5];
};

#define OBBKDOP_OBBMATRIX_INDEX_OFFSET 104
#define OBBKDOP_SIZE 128

//=====================================================================================================================
// The TLAS builder requires information about the root node and its 4 children
#define BLAS_OBB_METADATA_NODE_COUNT 5
#define AABBROOT_SIZE_RESULT (RTIP3_1_NODE_SIZE * BLAS_OBB_METADATA_NODE_COUNT)

struct BlasObbMetadata
{
    // Root kDOP for TLAS refitting
    uint kdop[OBBKDOP_SIZE / 4];
    // Original root AABB nodes for rebraid
    uint aabbRootNodeData[AABBROOT_SIZE_RESULT / 4];
};

#define BLAS_OBB_METADATA_KDOP_OFFSET 0
#define BLAS_OBB_METADATA_ROOT_NODE_DATA_OFFSET OBBKDOP_SIZE
#define BLAS_OBB_METADATA_SIZE (BLAS_OBB_METADATA_ROOT_NODE_DATA_OFFSET + AABBROOT_SIZE_RESULT)

#ifdef __cplusplus
static_assert((OBBKDOP_SIZE == sizeof(ObbKdop)), "ObbKdop structure mismatch");
static_assert(BLAS_OBB_METADATA_SIZE == sizeof(BlasObbMetadata));
#endif

#define PRIM_COMP_MAX_RANGE_SIZE_3_1 2
#define PRIM_COMP_GROUP_SIZE         32

#define OBB_TABLE_ENCODED_BITS_OFFSET    0
#define OBB_TABLE_SA_OFFSET              4
#define OBB_TABLE_ENTRY_SIZE             8
#endif

//=====================================================================================================================
struct StackPtrs
{
    uint stackPtrSrcNodeId;     // source node index in linear memory
    uint stackPtrNodeDest;      // node destination in linear memory. Counts in 64B chunks (FP16 box node size)
    uint numLeafsDone;
#if GPURT_BUILD_RTIP3_1
    uint primCompBatchCount;    // Number of normal primitive compression batches
    uint primCompGroupCountX;   // Number of groups to launch for CompressPrims
    uint primCompGroupCountY;   // Set to 1 (used as indirect dispatch Y dimension)
    uint primCompGroupCountZ;   // Set to 1 (used as indirect dispatch Z dimension)
    uint primCompSingleCount;   // Number of single nodes (lone triangle or pair) not part of a compression batch
#endif
};

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
struct STGBHwEncodeCounters
{
    uint primRefCount;
    uint boxRefCount;
    uint nodeAllocCount;
    uint primCompBatchCount;
    uint primCompSingleCount;
};

#define STGB_COUNTER_IDX_PRIM_REF               0
#define STGB_COUNTER_IDX_BOX_REF                1
#define STGB_COUNTER_IDX_NODE_ALLOC             2
#define STGB_COUNTER_IDX_PRIM_COMP_BATCH        3
#define STGB_COUNTER_IDX_PRIM_COMP_BATCH_SINGLE 4

#ifdef __cplusplus
static_assert(STGB_COUNTER_IDX_PRIM_REF == offsetof(STGBHwEncodeCounters, primRefCount) / sizeof(uint32_t), "");
static_assert(STGB_COUNTER_IDX_BOX_REF == offsetof(STGBHwEncodeCounters, boxRefCount) / sizeof(uint32_t), "");
static_assert(STGB_COUNTER_IDX_NODE_ALLOC == offsetof(STGBHwEncodeCounters, nodeAllocCount) / sizeof(uint32_t), "");
static_assert(STGB_COUNTER_IDX_PRIM_COMP_BATCH == offsetof(STGBHwEncodeCounters, primCompBatchCount) / sizeof(uint32_t), "");
static_assert(STGB_COUNTER_IDX_PRIM_COMP_BATCH_SINGLE == offsetof(STGBHwEncodeCounters, primCompSingleCount) / sizeof(uint32_t), "");
#endif

//=====================================================================================================================
struct ObbRefitStackPtrs
{
    uint groupCountX;   // Number of groups to launch for OBB refit
    uint groupCountY;   // Set to 1 (used as indirect dispatch Y dimension)
    uint groupCountZ;   // Set to 1 (used as indirect dispatch Z dimension)
    uint batchCount;    // Number of fat leaf nodes (internal nodes with all leaf children)
};

#define STACK_PTRS_OBB_REFIT_GROUP_COUNT_X 0
#define STACK_PTRS_OBB_REFIT_GROUP_COUNT_Y 4
#define STACK_PTRS_OBB_REFIT_GROUP_COUNT_Z 8
#define STACK_PTRS_OBB_REFIT_BATCH_COUNT   12

#define REFIT_ORIENTED_BOUNDS3_1_THREADGROUP_SIZE 32

#endif

#define STACK_PTRS_SRC_PTR_OFFSET         0
#define STACK_PTRS_DST_PTR_OFFSET         4
#define STACK_PTRS_NUM_LEAFS_DONE_OFFSET  8
#if GPURT_BUILD_RTIP3_1
#define STACK_PTRS_PRIM_COMP_BATCH_COUNT   12
#define STACK_PTRS_PRIM_COMP_GROUP_COUNT_X 16
#define STACK_PTRS_PRIM_COMP_GROUP_COUNT_Y 20
#define STACK_PTRS_PRIM_COMP_GROUP_COUNT_Z 24
#define STACK_PTRS_PRIM_COMP_SINGLE_COUNT  28
#define STACK_PTR_SIZE                     32
#else
#define STACK_PTR_SIZE                     12
#endif

#ifdef __cplusplus
static_assert(STACK_PTR_SIZE                   == sizeof(StackPtrs), "StackPtrs structure mismatch");
static_assert(STACK_PTRS_SRC_PTR_OFFSET        == offsetof(StackPtrs, stackPtrSrcNodeId), "");
static_assert(STACK_PTRS_DST_PTR_OFFSET        == offsetof(StackPtrs, stackPtrNodeDest),  "");
static_assert(STACK_PTRS_NUM_LEAFS_DONE_OFFSET == offsetof(StackPtrs, numLeafsDone), "");
#if GPURT_BUILD_RTIP3_1
static_assert(STACK_PTRS_DST_PTR_OFFSET          == (STACK_PTRS_SRC_PTR_OFFSET + 4), "");
static_assert(STACK_PTRS_PRIM_COMP_BATCH_COUNT   == offsetof(StackPtrs, primCompBatchCount), "");
static_assert(STACK_PTRS_PRIM_COMP_GROUP_COUNT_X == offsetof(StackPtrs, primCompGroupCountX), "");
static_assert(STACK_PTRS_PRIM_COMP_GROUP_COUNT_Y == offsetof(StackPtrs, primCompGroupCountY), "");
static_assert(STACK_PTRS_PRIM_COMP_GROUP_COUNT_Z == offsetof(StackPtrs, primCompGroupCountZ), "");
static_assert(STACK_PTRS_PRIM_COMP_GROUP_COUNT_Y == STACK_PTRS_PRIM_COMP_GROUP_COUNT_X + sizeof(uint), "");
static_assert(STACK_PTRS_PRIM_COMP_GROUP_COUNT_Z == STACK_PTRS_PRIM_COMP_GROUP_COUNT_Y + sizeof(uint), "");
static_assert(Util::IsPow2Aligned(STACK_PTRS_PRIM_COMP_GROUP_COUNT_X, 16),
              "Indirect dispatch args must not cross a 16-byte boundary");
static_assert(STACK_PTRS_PRIM_COMP_SINGLE_COUNT  == offsetof(StackPtrs, primCompSingleCount), "");
#endif
#endif

//=====================================================================================================================
// Counters used in encode phase

//=====================================================================================================================
// Task counters common between BVH builds and updates
struct EncodeTaskCountersCommon
{
    uint numPrimitives;
    uint primRefs;
};

//=====================================================================================================================
// There is DXC bug that doesn't properly compile HLSL->SPRIV using structure inheritance.
// Once it is fixed, EncodeTaskCountersBuild, EncodeTaskCountersUpdate can inherit from EncodeTaskCountersCommon
// https://github.com/microsoft/DirectXShaderCompiler/issues/6986
struct EncodeTaskCountersBuild
{
    uint numPrimitives;
    uint primRefs;

    // The following indirect arguments are only used in mult-dispatch path. Note, currently only HPLOC dispatch uses
    // these, but it will be extended to other passes when early pair compression is enabled.
    uint groupCountX;
    uint groupCountY;
    uint groupCountZ;
};

#define ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET 0
#define ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET      4
#define ENCODE_TASK_COUNTER_INDIRECT_ARGS         8

#ifdef __cplusplus
static_assert(ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET == offsetof(EncodeTaskCountersBuild, numPrimitives));
static_assert(ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET == offsetof(EncodeTaskCountersBuild, primRefs));
#endif

//=====================================================================================================================
// Update scratch memory fields
struct EncodeTaskCountersUpdate
{
    uint numPrimitives;
    uint primRefs;
    uint refitTaskCounter;
    uint taskCount;
    uint tasksDone;
};

#define UPDATE_SCRATCH_COUNTER_NUM_PRIMITIVES_OFFSET 0  // Written by Encode phase.
#define UPDATE_SCRATCH_STACK_NUM_ENTRIES_OFFSET      4  // Written by Encode phase.
#define UPDATE_SCRATCH_REFIT_TASK_COUNT_OFFSET       8  // Update phase refit task counter
#define UPDATE_SCRATCH_TASK_COUNT_OFFSET            12  // Update task queue counters.
#define UPDATE_SCRATCH_TASKS_DONE_OFFSET            16  // Update task queue counters.

#ifdef __cplusplus
// The following counter offsets shared between build and update encode must match.
static_assert(UPDATE_SCRATCH_COUNTER_NUM_PRIMITIVES_OFFSET == ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET);
static_assert(UPDATE_SCRATCH_STACK_NUM_ENTRIES_OFFSET == ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET);
#endif

//=====================================================================================================================
struct TaskLoopCounters
{
    uint buildParallelTaskCounter;
    uint buildParallelTasksDone;
    uint mergeSortTaskCounter;
    uint mergeSortTasksDone;
    uint qbvhTaskCounter;
    uint qbvhTasksDone;
};

#define TASK_LOOP_BUILD_PARALLEL_COUNTER_OFFSET    0
#define TASK_LOOP_BUILD_PARALLEL_TASKS_DONE_OFFSET 4
#define TASK_LOOP_MERGE_SORT_COUNTER_OFFSET        8
#define TASK_LOOP_MERGE_SORT_TASKS_DONE_OFFSET     12
#define TASK_LOOP_QBVH_COUNTER_OFFSET              16
#define TASK_LOOP_QBVH_TASKS_DONE_OFFSET           20
#define TASK_LOOP_COUNTERS_NUM_DWORDS              (sizeof(TaskLoopCounters) / sizeof(uint))

#ifdef __cplusplus
static_assert(TASK_LOOP_BUILD_PARALLEL_COUNTER_OFFSET    == offsetof(TaskLoopCounters, buildParallelTaskCounter));
static_assert(TASK_LOOP_BUILD_PARALLEL_TASKS_DONE_OFFSET == offsetof(TaskLoopCounters, buildParallelTasksDone));
static_assert(TASK_LOOP_MERGE_SORT_COUNTER_OFFSET        == offsetof(TaskLoopCounters, mergeSortTaskCounter));
static_assert(TASK_LOOP_MERGE_SORT_TASKS_DONE_OFFSET     == offsetof(TaskLoopCounters, mergeSortTasksDone));
static_assert(TASK_LOOP_QBVH_COUNTER_OFFSET              == offsetof(TaskLoopCounters, qbvhTaskCounter));
static_assert(TASK_LOOP_QBVH_TASKS_DONE_OFFSET           == offsetof(TaskLoopCounters, qbvhTasksDone));
#endif

#if GPURT_BUILD_RTIP3_1
#define OBB_QUALITY 2u
#define OBB_VERTEX_COUNT ((OBB_QUALITY * OBB_QUALITY * 6u) + 2u)
#define OBB_FACET_COUNT ((OBB_QUALITY * OBB_QUALITY * 6u) * 2u)
#define KDOP_PLANE_COUNT (OBB_VERTEX_COUNT / 2u)

//=====================================================================================================================
struct ObbData
{
    float4 obbAxesLut[KDOP_PLANE_COUNT];
    float4 obbKdopDirections[KDOP_PLANE_COUNT];
    uint4  obbKdopFacets[OBB_FACET_COUNT];
    uint4  obbMatrixIndexLUT[1024];
};
#define LutData ObbData
#else
struct LutData {};
#endif

//=====================================================================================================================
// different ways to encode the scene bounds used to generate morton codes

#ifdef __cplusplus
enum SceneBoundsCalculation : uint
#else
enum class SceneBoundsCalculation : uint32
#endif
{
    BasedOnGeometry = 0x0,
    BasedOnGeometryWithSize = 0x1
};

#endif
