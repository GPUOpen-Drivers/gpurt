/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2022 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "BuildCommon.hlsl"

#define RootSig "RootConstants(num32BitConstants=53, b0),"\
                "CBV(b1),"\
                "UAV(u0),"\
                "UAV(u1),"\
                "UAV(u2),"\
                "UAV(u3),"\
                "UAV(u4),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894))"

struct ScratchOffsets
{
    uint mortonCodes;
    uint mortonCodesSorted;
    uint primIndicesSorted;
    uint primIndicesSortedSwap;
    uint tempKeys;
    uint tempVals;
    uint histogram;
    uint bvhNodes;
    uint unsortedBvhLeafNodes;
    uint sceneBounds;
    uint partialSums;
    uint propagationFlags;
    uint dynamicBlockIndex;
    uint prefixSumAtomicFlags;

    uint clusterList0;
    uint clusterList1;

    // PLOC
    uint neighborIndices;
    uint currentState;
    uint atomicFlagsPloc;
    uint clusterOffsets;

    // AC
    uint clusterList;
    uint numClusterList;
    uint internalNodesIndex;

    // Build QBVH
    uint qbvhStack;
    uint stackPtrs;

    // Tri Split
    uint splitBoxes;
    uint refList0;
    uint refList1;
    uint splitPriorities;
    uint currentSplitState;
    uint splitAtomicFlags;

    // Pair compression
    uint numBatches;
    uint batchIndices;
    uint indexBufferInfo;

    // TD Build
    uint tdRefs;
    uint tdNodes;
    uint tdBins;
    uint tdState;
    uint tdTaskCounters;
    uint reservedUint4;
    uint reservedUint5;
    uint reservedUint6;
    uint reservedUint7;
    uint reservedUint8;

    // debug
    uint debugCounters;
};

struct Constants
{
    uint numPrimitives;
    uint numThreadGroups;
    uint padding;
    uint maxNumPrimitives;

    uint encodeArrayOfPointers;
    uint rebraidFactor;
    uint numMortonSizeBits;
    float reservedFloat;

    // Warning: following struct will be 4 dword aligned according to HLSL CB packing rules.

    ScratchOffsets offsets; // Scratch buffer layout
};

[[vk::push_constant]] ConstantBuffer<Constants>         ShaderConstants : register(b0);

[[vk::constant_id(BUILD_SETTINGS_DATA_TOP_LEVEL_BUILD_ID)]]                        uint topLevelBuild                 = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_BUILD_MODE_ID)]]                             uint buildMode                     = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_TRIANGLE_COMPRESSION_MODE_ID)]]              uint triangleCompressionMode       = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_DO_TRIANGLE_SPLITTING_ID)]]                  uint doTriangleSplitting           = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_DO_COLLAPSE_ID)]]                            uint doCollapse                    = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_FP16_BOX_NODES_MODE_ID)]]                    uint fp16BoxNodesMode              = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_FP16_BOX_MODE_MIXED_SA_THRESHHOLD_ID)]]      float fp16BoxModeMixedSaThreshhold = 0;
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
[[vk::constant_id(BUILD_SETTINGS_DATA_BVH_BUILDER_NODE_SORT_TYPE_ID)]]             uint bvhBuilderNodeSortType        = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_BVH_BUILDER_NODE_SORT_HEURISTIC_ID)]]        uint bvhBuilderNodeSortHeuristic   = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_ENABLE_HALF_BOX_NODE_32_ID)]]                uint enableHalfBoxNode32           = 0;
[[vk::constant_id(BUILD_SETTINGS_DATA_SAH_QBVH_ID)]]                               uint sahQbvh                       = 0;

static const BuildSettingsData Settings = {
    topLevelBuild,
    buildMode,
    triangleCompressionMode,
    doTriangleSplitting,
    doCollapse,
    fp16BoxNodesMode,
    fp16BoxModeMixedSaThreshhold,
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
    bvhBuilderNodeSortType,
    bvhBuilderNodeSortHeuristic,
    enableHalfBoxNode32,
    sahQbvh,
};

[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer ResultBuffer       : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer ResultMetadata     : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer ScratchBuffer      : register(u2);
[[vk::binding(3, 0)]]                  RWByteAddressBuffer InstanceDescBuffer : register(u3);
[[vk::binding(4, 0)]]                  RWByteAddressBuffer EmitBuffer         : register(u4);

#define MAX_LDS_ELEMENTS (16 * BUILD_THREADGROUP_SIZE)
groupshared uint SharedMem[MAX_LDS_ELEMENTS];

groupshared uint SharedIndex;

//======================================================================================================================
// Returns number of threads launched
uint GetNumThreads()
{
    return ShaderConstants.numThreadGroups * BUILD_THREADGROUP_SIZE;
}

//======================================================================================================================
// Wait for the task counter to reach the specified value
void WaitForTasksToFinish(
    uint numTasksWait)
{
    do
    {
        DeviceMemoryBarrier();
    } while (ResultMetadata.Load(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET) < numTasksWait);
}

// Include implementations for each pass without shader entry points and resource declarations
#define NO_SHADER_ENTRYPOINT 1

#include "GenerateMortonCodes.hlsl"
#include "RadixSort/ScanExclusiveInt4DLBCommon.hlsl"
#include "RadixSort/RadixSortParallel.hlsl"
#include "BuildBVHPLOC.hlsl"
#include "BuildQBVH.hlsl"
#include "BuildQBVHCollapseImpl.hlsl"
#include "BuildBVHTDTR.hlsl"
#include "BuildBVH.hlsl"
#include "RefitBoundsImpl.hlsl"
#include "TriangleSplitting.hlsl"
#include "PairCompression.hlsl"
#include "Rebraid.hlsl"
#include "MergeSort.hlsl"

//======================================================================================================================
void GenerateMortonCodes(
    uint globalId,
    uint numPrimitives)
{
    const BoundingBox sceneBounds = FetchSceneBounds(ScratchBuffer, ShaderConstants.offsets.sceneBounds);

    const float3 sceneExtent = sceneBounds.max - sceneBounds.min;
    const float3 sceneMin    = sceneBounds.min;

    const uint primCount = numPrimitives;

    float2  sizeMinMax = float2(0, 0);

    if (ShaderConstants.numMortonSizeBits > 0)
    {
        sizeMinMax = FetchSceneSize(ScratchBuffer, ShaderConstants.offsets.sceneBounds);
    }

    for (uint primitiveIndex = globalId; primitiveIndex < primCount; primitiveIndex += GetNumThreads())
    {
        GenerateMortonCodesImpl(
            sceneMin,
            sceneExtent,
            primitiveIndex,
            ShaderConstants.offsets.unsortedBvhLeafNodes,
            ShaderConstants.offsets.mortonCodes,
            Settings.doTriangleSplitting,
            ShaderConstants.offsets.splitBoxes,
            Settings.enableVariableBitsMortonCode,
            Settings.useMortonCode30,
            ShaderConstants.numMortonSizeBits,
            sizeMinMax,
            false,
            0);
    }
}

//======================================================================================================================
// Reorder scratch leaves based on sorted prim indices
void SortScratchLeaves(
    uint globalId,
    uint numActivePrims)
{
    for (uint primIndex = globalId; primIndex < numActivePrims; primIndex += GetNumThreads())
    {
        CopyUnsortedScratchLeafNode(
            primIndex,
            numActivePrims,
            ShaderConstants.offsets.primIndicesSorted,
            ShaderConstants.offsets.unsortedBvhLeafNodes,
            ShaderConstants.offsets.bvhNodes,
            0,
            0,
            false);
    }
}

//======================================================================================================================
void BuildBvhLinear(
    uint globalId,
    uint numActivePrims)
{
    const uint numInternalNodes = numActivePrims - 1;

    for (uint nodeIndex = globalId; nodeIndex < numInternalNodes; nodeIndex += GetNumThreads())
    {
        SplitInternalNodeLbvh(
            nodeIndex,
            numActivePrims,
            ShaderConstants.offsets.bvhNodes,
            ShaderConstants.offsets.mortonCodesSorted,
            Settings.useMortonCode30);
    }
}

//======================================================================================================================
bool EnablePairCompression()
{
    return (Settings.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) && (Settings.topLevelBuild == false);
}

//======================================================================================================================
void RefitBounds(
    uint globalId,
    uint numActivePrims)
{
    for (uint primIndex = globalId; primIndex < numActivePrims; primIndex += GetNumThreads())
    {
        RefitBoundsImpl(
            primIndex,
            numActivePrims,
            ShaderConstants.offsets.propagationFlags,
            ShaderConstants.offsets.bvhNodes,
            Settings.doCollapse,
            Settings.doTriangleSplitting,

            EnablePairCompression(),
            Settings.enablePairCostCheck,
            ShaderConstants.offsets.splitBoxes,
            ShaderConstants.offsets.numBatches,
            ShaderConstants.offsets.batchIndices,
            Settings.fp16BoxNodesMode,
            Settings.fp16BoxModeMixedSaThreshhold,
            0,
            false,
            false);
    }
}

//======================================================================================================================
uint BuildModeFlags()
{
    uint flags = 0;
    flags |= Settings.doCollapse ? BUILD_FLAGS_COLLAPSE : 0;
    flags |= Settings.doTriangleSplitting ? BUILD_FLAGS_TRIANGLE_SPLITTING : 0;
    flags |= EnablePairCompression() ? BUILD_FLAGS_PAIR_COMPRESSION : 0;
    flags |= Settings.enablePairCostCheck ? BUILD_FLAGS_PAIR_COST_CHECK : 0;

    return flags;
}

//======================================================================================================================
void TriangleSplitting(
    uint       globalId,
    uint       localId,
    uint       groupId)
{
    TriangleSplittingArgs args;

    args.numThreadGroups                = ShaderConstants.numThreadGroups;
    args.numPrimitives                  = ShaderConstants.numPrimitives;
    args.maxNumPrimitives               = ShaderConstants.maxNumPrimitives;
    args.numSizeBits                    = ShaderConstants.numMortonSizeBits;
    args.scratchLeafNodesScratchOffset  = ShaderConstants.offsets.unsortedBvhLeafNodes;
    args.splitBoxesScratchOffset        = ShaderConstants.offsets.splitBoxes;
    args.refList0ScratchOffset          = ShaderConstants.offsets.refList0;
    args.refList1ScratchOffset          = ShaderConstants.offsets.refList1;
    args.splitPrioritiesScratchOffset   = ShaderConstants.offsets.splitPriorities;
    args.currentStateScratchOffset      = ShaderConstants.offsets.currentSplitState;
    args.atomicFlagsScratchOffset       = ShaderConstants.offsets.splitAtomicFlags;
    args.dynamicBlockIndexScratchOffset = ShaderConstants.offsets.dynamicBlockIndex;
    args.sceneBoundsByteOffset          = ShaderConstants.offsets.sceneBounds;

    TriangleSplittingImpl(globalId, localId, groupId, args);
}

//======================================================================================================================
void BuildBvhPloc(
    inout uint numTasksWait,
    inout uint waveId,
    uint       globalId,
    uint       localId,
    uint       groupId,
    uint       numActivePrims)
{
    BuildPlocArgs plocArgs;

    plocArgs.numThreads                     = GetNumThreads();
    plocArgs.scratchNodesScratchOffset      = ShaderConstants.offsets.bvhNodes;
    plocArgs.clusterList0ScratchOffset      = ShaderConstants.offsets.clusterList0;
    plocArgs.clusterList1ScratchOffset      = ShaderConstants.offsets.clusterList1;
    plocArgs.neighbourIndicesScratchOffset  = ShaderConstants.offsets.neighborIndices;
    plocArgs.currentStateScratchOffset      = ShaderConstants.offsets.currentState;
    plocArgs.atomicFlagsScratchOffset       = ShaderConstants.offsets.atomicFlagsPloc;
    plocArgs.offsetsScratchOffset           = ShaderConstants.offsets.clusterOffsets;
    plocArgs.dynamicBlockIndexScratchOffset = ShaderConstants.offsets.dynamicBlockIndex;
    plocArgs.numBatchesScratchOffset        = ShaderConstants.offsets.numBatches;
    plocArgs.baseBatchIndicesScratchOffset  = ShaderConstants.offsets.batchIndices;
    plocArgs.fp16BoxNodesInBlasMode         = Settings.fp16BoxNodesMode;
    plocArgs.fp16BoxModeMixedSaThresh       = Settings.fp16BoxModeMixedSaThreshhold;
    plocArgs.fp16BoxModeMixedSaThresh       = Settings.fp16BoxModeMixedSaThreshhold;
    plocArgs.plocRadius                     = Settings.plocRadius;
    plocArgs.flags                          = BuildModeFlags();
    plocArgs.splitBoxesByteOffset           = ShaderConstants.offsets.splitBoxes;

    BuildBvhPlocImpl(globalId, localId, groupId, numActivePrims, plocArgs);
}

//======================================================================================================================
void Rebraid(
    uint       globalId,
    uint       localId,
    uint       groupId)
{
    RebraidArgs args;

    args.numPrimitives                      = ShaderConstants.numPrimitives;
    args.numThreadGroups                    = ShaderConstants.numThreadGroups;
    args.maxNumPrims                        = ShaderConstants.rebraidFactor * ShaderConstants.numPrimitives;
    args.bvhLeafNodeDataScratchOffset       = ShaderConstants.offsets.unsortedBvhLeafNodes;
    args.sceneBoundsOffset                  = ShaderConstants.offsets.sceneBounds;
    args.stateScratchOffset                 = ShaderConstants.offsets.currentSplitState;
    args.atomicFlagsScratchOffset           = ShaderConstants.offsets.splitAtomicFlags;
    args.encodeArrayOfPointers              = ShaderConstants.encodeArrayOfPointers;
    RebraidImpl(globalId, localId, groupId, args);
}

//======================================================================================================================
void BuildBvhTD(
    uint       globalId,
    uint       localId,
    uint       groupId,
    uint       numPrimitives)
{
    TDArgs args;

    args.NumPrimitives = numPrimitives;
    args.AllowUpdate = false;  //not used
    args.NumThreads = GetNumThreads();
    args.MaxRefCountSize = numPrimitives * ShaderConstants.rebraidFactor;
    args.LengthPercentage = 0.1;
    args.BvhNodeDataScratchOffset = ShaderConstants.offsets.bvhNodes;
    args.BvhLeafNodeDataScratchOffset = ShaderConstants.offsets.unsortedBvhLeafNodes;
    args.SceneBoundsOffset = ShaderConstants.offsets.sceneBounds;
    args.RefScratchOffset = ShaderConstants.offsets.tdRefs;
    args.TDNodeScratchOffset = ShaderConstants.offsets.tdNodes;
    args.TDBinsScratchOffset = ShaderConstants.offsets.tdBins;
    args.CurrentStateScratchOffset = ShaderConstants.offsets.tdState;
    args.TdTaskCounterScratchOffset = ShaderConstants.offsets.tdTaskCounters;
    args.OffsetsScratchOffset = 0; //not used
    args.EncodeArrayOfPointers = ShaderConstants.encodeArrayOfPointers;

    BuildBVHTDImpl(globalId, localId, groupId, args);
}
//======================================================================================================================
void PairCompression(
    uint globalId)
{
    const uint buildInfo  = ResultBuffer.Load(ACCEL_STRUCT_HEADER_INFO_OFFSET);
    const uint buildFlags = (buildInfo >> ACCEL_STRUCT_HEADER_INFO_FLAGS_SHIFT) & ACCEL_STRUCT_HEADER_INFO_FLAGS_MASK;

    PairCompressionArgs args;

    args.scratchNodesScratchOffset    = ShaderConstants.offsets.bvhNodes;
    args.numBatchesScratchOffset      = ShaderConstants.offsets.numBatches;
    args.batchIndicesScratchOffset    = ShaderConstants.offsets.batchIndices;
    args.indexBufferInfoScratchOffset = ShaderConstants.offsets.indexBufferInfo;
    args.flagsScratchOffset           = ShaderConstants.offsets.propagationFlags;
    args.buildFlags                   = buildFlags;

    PairCompressionImpl(globalId, args);
}

//======================================================================================================================
void InitBuildQbvh(
    uint globalId,
    uint numPrimitives)
{
    BuildQbvhArgs qbvhArgs;

    qbvhArgs.numPrimitives               = numPrimitives;
    qbvhArgs.metadataSizeInBytes         = ResultMetadata.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET);
    qbvhArgs.numThreads                  = GetNumThreads();
    qbvhArgs.scratchNodesScratchOffset   = ShaderConstants.offsets.bvhNodes;
    qbvhArgs.qbvhStackScratchOffset      = ShaderConstants.offsets.qbvhStack;
    qbvhArgs.stackPtrsScratchOffset      = ShaderConstants.offsets.stackPtrs;
    qbvhArgs.splitBoxesByteOffset        = ShaderConstants.offsets.splitBoxes;
    qbvhArgs.triangleCompressionMode     = Settings.triangleCompressionMode;
    qbvhArgs.fp16BoxNodesInBlasMode      = Settings.fp16BoxNodesMode;
    qbvhArgs.flags                       = BuildModeFlags();
    qbvhArgs.encodeArrayOfPointers       = ShaderConstants.encodeArrayOfPointers;
    qbvhArgs.topDownBuild                = 0;
    qbvhArgs.bvhBuilderNodeSortType      = Settings.bvhBuilderNodeSortType;
    qbvhArgs.bvhBuilderNodeSortHeuristic = Settings.bvhBuilderNodeSortHeuristic;
    qbvhArgs.enableHalfBoxNode32         = Settings.enableHalfBoxNode32;
    qbvhArgs.sahQbvh                     = Settings.sahQbvh;

    if (!Settings.topLevelBuild)
    {
        qbvhArgs.captureChildNumPrimsForRebraid = true;
    }
    else
    {
        qbvhArgs.captureChildNumPrimsForRebraid = false;
    }

    InitBuildQbvhImpl(globalId, qbvhArgs);
}

//======================================================================================================================
void BuildQbvh(
    uint globalId,
    uint numPrimitives)
{
    BuildQbvhArgs qbvhArgs;

    qbvhArgs.numPrimitives               = numPrimitives;
    qbvhArgs.metadataSizeInBytes         = ResultMetadata.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET);
    qbvhArgs.numThreads                  = GetNumThreads();
    qbvhArgs.scratchNodesScratchOffset   = ShaderConstants.offsets.bvhNodes;
    qbvhArgs.qbvhStackScratchOffset      = ShaderConstants.offsets.qbvhStack;
    qbvhArgs.stackPtrsScratchOffset      = ShaderConstants.offsets.stackPtrs;
    qbvhArgs.splitBoxesByteOffset        = ShaderConstants.offsets.splitBoxes;
    qbvhArgs.triangleCompressionMode     = Settings.triangleCompressionMode;
    qbvhArgs.fp16BoxNodesInBlasMode      = Settings.fp16BoxNodesMode;
    qbvhArgs.flags                       = BuildModeFlags();
    qbvhArgs.encodeArrayOfPointers       = ShaderConstants.encodeArrayOfPointers;
    qbvhArgs.topDownBuild                = Settings.rebraidType != RebraidType::Off;
    qbvhArgs.bvhBuilderNodeSortType      = Settings.bvhBuilderNodeSortType;
    qbvhArgs.bvhBuilderNodeSortHeuristic = Settings.bvhBuilderNodeSortHeuristic;
    qbvhArgs.enableHalfBoxNode32         = Settings.enableHalfBoxNode32;
    qbvhArgs.sahQbvh                     = Settings.sahQbvh;

    if (!Settings.topLevelBuild)
    {
        qbvhArgs.captureChildNumPrimsForRebraid = true;
    }
    else
    {
        qbvhArgs.captureChildNumPrimsForRebraid = false;
    }

    if ((Settings.topLevelBuild == 0) && Settings.doCollapse)
    {
        BuildQbvhCollapseImpl(globalId, qbvhArgs);
    }
    else
    {
        BuildQbvhImpl(globalId, qbvhArgs, (Settings.topLevelBuild != 0));
    }
}

//======================================================================================================================
void writeDebugCounter(uint counterStageOffset)
{
    if (Settings.enableBVHBuildDebugCounters)
    {
        ScratchBuffer.InterlockedAdd(ShaderConstants.offsets.debugCounters + counterStageOffset, 1);
    }
}

//======================================================================================================================
void MergeSort(inout uint numTasksWait, inout uint waveId, uint localId, uint groupId, uint numPrimitives)
{
    MergeSortImpl(numTasksWait,
                  waveId,
                  localId,
                  groupId,
                  numPrimitives,
                  ShaderConstants.offsets.mortonCodes,
                  ShaderConstants.offsets.mortonCodesSorted,
                  ShaderConstants.offsets.primIndicesSorted,
                  ShaderConstants.offsets.primIndicesSortedSwap,
                  Settings.useMortonCode30);
}

//======================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void BuildBvh(
    uint globalIdIn : SV_DispatchThreadID,
    uint localIdIn  : SV_GroupThreadID,
    uint groupIdIn  : SV_GroupID)
{
    uint globalId = globalIdIn;
    uint localId = localIdIn;
    uint groupId = groupIdIn;

    uint numTasksWait = ShaderConstants.numPrimitives;

    // Wait for encode to finish
    WaitForTasksToFinish(numTasksWait);

    uint waveId = 0;

    INIT_TASK;

    // Take into account the encode tasks that are done
    if ((waveId == numTasksWait) && (localId == 0))
    {
        ResultMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_NUM_TASKS_DONE_OFFSET, numTasksWait);
    }

    DeviceMemoryBarrierWithGroupSync();

    uint numPrimitives = ShaderConstants.numPrimitives;

    if (Settings.doTriangleSplitting)
    {
        TriangleSplitting(globalId, localId, groupId);

        numPrimitives = ResultBuffer.Load(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET);
    }
    else if (Settings.rebraidType == RebraidType::V2)
    {
        Rebraid(globalId, localId, groupId);

        numPrimitives = ResultBuffer.Load(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET);
    }

    uint numActivePrims;

    if (Settings.enableTopDownBuild)
    {
        BuildBvhTD(globalId, localId, groupId, numPrimitives);

        numActivePrims = ResultBuffer.Load(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
    }
    else
    {
        bool needRefit = false;

        if ((Settings.fastBuildThreshold) && (numPrimitives <= Settings.fastBuildThreshold) && (numPrimitives <= WaveGetLaneCount()))
        {
            BEGIN_TASK(1);

            FastBuildBVH(globalId, numPrimitives, ShaderConstants.offsets.unsortedBvhLeafNodes, ShaderConstants.offsets.bvhNodes);

            END_TASK(1);
            needRefit = true;
            numActivePrims = ResultBuffer.Load(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
        }
        else
        {
            BEGIN_TASK(ShaderConstants.numThreadGroups);

            GenerateMortonCodes(globalId, numPrimitives);

            END_TASK(ShaderConstants.numThreadGroups);
            writeDebugCounter(COUNTER_MORTONGEN_OFFSET);
            numActivePrims = ResultBuffer.Load(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);

            if (numActivePrims > 0)
            {
                if (Settings.enableMergeSort)
                {
                    MergeSort(numTasksWait, waveId, localId, groupId, numPrimitives);
                }
                else
                {
                    RadixSort(numTasksWait, waveId, globalId, localId, groupId, numPrimitives, Settings.radixSortScanLevel, Settings.useMortonCode30);
                }
                writeDebugCounter(COUNTER_MORTON_SORT_OFFSET);
                // Note there is an implicit sync on the last pass of the sort

                BEGIN_TASK(ShaderConstants.numThreadGroups);

                SortScratchLeaves(globalId, numActivePrims);

                END_TASK(ShaderConstants.numThreadGroups);
                writeDebugCounter(COUNTER_SORTLEAF_OFFSET);

                if (Settings.buildMode == BUILD_MODE_PLOC)
                {
                    BuildBvhPloc(numTasksWait, waveId, globalId, localId, groupId, numActivePrims);
                    writeDebugCounter(COUNTER_BUILDPLOC_OFFSET);
                }
                else
                {
                    BEGIN_TASK(ShaderConstants.numThreadGroups);

                    BuildBvhLinear(globalId, numActivePrims);

                    END_TASK(ShaderConstants.numThreadGroups);
                    writeDebugCounter(COUNTER_BUILDLBVH_OFFSET);
                    needRefit = true;
                }
            }
        }

        if (numActivePrims > 0)
        {
            if (needRefit)
            {
                BEGIN_TASK(ShaderConstants.numThreadGroups);

                RefitBounds(globalId, numActivePrims);

                END_TASK(ShaderConstants.numThreadGroups);
                writeDebugCounter(COUNTER_REFIT_OFFSET);
            }
            const uint geometryType = ResultBuffer.Load(ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET);

            if (EnablePairCompression() && (geometryType == GEOMETRY_TYPE_TRIANGLES))
            {
                const uint numBatches = ScratchBuffer.Load(ShaderConstants.offsets.numBatches);

                BEGIN_TASK(RoundUpQuotient(numBatches, BUILD_THREADGROUP_SIZE));

                PairCompression(globalId);

                END_TASK(RoundUpQuotient(numBatches, BUILD_THREADGROUP_SIZE));
            }
        }
        else if (Settings.topLevelBuild)
        {
            BEGIN_TASK(1);

            if (globalId == 0)
            {
                // This is an empty TLAS, but we didn't know it yet when we were setting up the header writes in the
                // command buffer. Overwrite the GPU VA to 0 to properly designate the TLAS as empty.
                ResultMetadata.Store<GpuVirtualAddress>(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, 0);
            }

            END_TASK(1);
            writeDebugCounter(COUNTER_EMPTYPRIM_OFFSET);
        }
    }

    if (numActivePrims > 0)
    {
        BEGIN_TASK(ShaderConstants.numThreadGroups);

        InitBuildQbvh(globalId, numPrimitives);

        END_TASK(ShaderConstants.numThreadGroups);
        writeDebugCounter(COUNTER_INITQBVH_OFFSET);

        const uint numLeafNodes = ResultBuffer.Load(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET);
        const uint numNodesToProcess = EnablePairCompression() ? numLeafNodes : numActivePrims;

        BEGIN_TASK(RoundUpQuotient(CalcNumQBVHInternalNodes(numNodesToProcess), BUILD_THREADGROUP_SIZE));

        BuildQbvh(globalId, numPrimitives);

        END_TASK(RoundUpQuotient(CalcNumQBVHInternalNodes(numNodesToProcess), BUILD_THREADGROUP_SIZE));
        writeDebugCounter(COUNTER_BUILDQBVH_OFFSET);
    }

    BEGIN_TASK(1);

    if (localId == 0)
    {
        WriteCompactedSize(ResultBuffer,
                           EmitBuffer,
                           Settings.emitCompactSize,
                           Settings.topLevelBuild ? TOP_LEVEL : BOTTOM_LEVEL);
    }

    END_TASK(1);
    writeDebugCounter(COUNTER_EMITCOMPACTSIZE_OFFSET);
}