/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

#ifdef __cplusplus
static_assert(GPURT_RTIP1_0 == uint32_t(Pal::RayTracingIpLevel::RtIp1_0), "GPURT_HLSL_RTIP mismatch.");
static_assert(GPURT_RTIP1_1 == uint32_t(Pal::RayTracingIpLevel::RtIp1_1), "GPURT_HLSL_RTIP mismatch.");
static_assert(GPURT_RTIP2_0 == uint32_t(Pal::RayTracingIpLevel::RtIp2_0), "GPURT_HLSL_RTIP mismatch.");
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

//=====================================================================================================================
struct StackPtrs
{
    uint stackPtrSrcNodeId;     // source node index in linear memory
    uint stackPtrNodeDest;      // node destination in linear memory. Counts in 64B chunks (FP16 box node size)
    uint numLeafsDone;
};

#define STACK_PTRS_SRC_PTR_OFFSET         0
#define STACK_PTRS_DST_PTR_OFFSET         4
#define STACK_PTRS_NUM_LEAFS_DONE_OFFSET  8
#define STACK_PTR_SIZE                     12

#ifdef __cplusplus
static_assert(STACK_PTR_SIZE                   == sizeof(StackPtrs), "StackPtrs structure mismatch");
static_assert(STACK_PTRS_SRC_PTR_OFFSET        == offsetof(StackPtrs, stackPtrSrcNodeId), "");
static_assert(STACK_PTRS_DST_PTR_OFFSET        == offsetof(StackPtrs, stackPtrNodeDest),  "");
static_assert(STACK_PTRS_NUM_LEAFS_DONE_OFFSET == offsetof(StackPtrs, numLeafsDone), "");
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
struct EncodeTaskCountersBuild : EncodeTaskCountersCommon
{
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
struct EncodeTaskCountersUpdate : EncodeTaskCountersCommon
{
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

//=====================================================================================================================

struct LutData {};

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
