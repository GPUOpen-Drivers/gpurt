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
#define RootSig "RootConstants(num32BitConstants=91, b0),"\
                "UAV(u0),"\
                "UAV(u1),"\
                "UAV(u2),"\
                "UAV(u3),"\
                "UAV(u4),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 2)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 3)),"\
                "DescriptorTable(CBV(b0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b1)"/*Build Settings binding*/

#include "RayTracingDefs.h"

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
    uint fastLBVHRootNodeIndex;

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
    uint reservedUint9;

    // debug
    uint debugCounters;
};

struct Constants
{
    uint resultBufferAddrLo;
    uint resultBufferAddrHi;
    uint numPrimitives;
    uint numThreadGroups;
    uint tsBudgetPerTriangle;
    uint maxNumPrimitives;

    uint encodeArrayOfPointers;
    uint rebraidFactor;
    uint numMortonSizeBits;
    float reservedFloat;
    uint numLeafNodes;
    uint numDescs;

    // Warning: following struct will be 4 dword aligned according to HLSL CB packing rules.
    AccelStructHeader header;
    ScratchOffsets offsets; // Scratch buffer layout
};

[[vk::push_constant]] ConstantBuffer<Constants> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer DstBuffer          : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer DstMetadata        : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer ScratchBuffer      : register(u2);
[[vk::binding(3, 0)]]                  RWByteAddressBuffer InstanceDescBuffer : register(u3);
[[vk::binding(4, 0)]]                  RWByteAddressBuffer EmitBuffer         : register(u4);

#include "BuildCommonScratch.hlsl"

#define MAX_LDS_ELEMENTS (16 * BUILD_THREADGROUP_SIZE)
groupshared uint SharedMem[MAX_LDS_ELEMENTS];

// The encode path uses SrcBuffer and DstBuffer as the true acceleration structure base.
#define SrcBuffer DstMetadata
#include "EncodeCommon.hlsl"
#include "EncodePairedTriangleImpl.hlsl"
#undef SrcBuffer

[[vk::binding(0, 2)]] ConstantBuffer<GeometryArgs> GeometryConstants[] : register(b0, space1);

[[vk::binding(0, 3)]] RWBuffer<float3>           GeometryBuffer[]  : register(u0, space1);
[[vk::binding(0, 4)]] RWByteAddressBuffer        IndexBuffer[]     : register(u0, space2);
#if !AMD_VULKAN
// DXC does not support arrays of structured buffers for SPIRV currently. See issue 3281 / PR 4663).
[[vk::binding(0, 5)]] RWStructuredBuffer<float4> TransformBuffer[] : register(u0, space3);
#endif

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
    } while (DstMetadata.Load(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET) < numTasksWait);
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
uint CalculateBvhNodesOffset(uint numActivePrims)
{
    return (Settings.enableTopDownBuild == false) && Settings.noCopySortedNodes ?
        ShaderConstants.offsets.bvhNodes + (ShaderConstants.numLeafNodes - numActivePrims) * SCRATCH_NODE_SIZE :
        ShaderConstants.offsets.bvhNodes;
}

//======================================================================================================================
void GenerateMortonCodes(
    uint globalId,
    uint numPrimitives)
{
    const BoundingBox sceneBounds = FetchSceneBounds(ShaderConstants.offsets.sceneBounds);

    const float3 sceneExtent = sceneBounds.max - sceneBounds.min;
    const float3 sceneMin    = sceneBounds.min;

    const uint primCount = numPrimitives;

    uint numSizeBits = ShaderConstants.numMortonSizeBits;

    float2  sizeMinMax = float2(0, 0);

    if (ShaderConstants.numMortonSizeBits > 0)
    {
        sizeMinMax = FetchSceneSize(ShaderConstants.offsets.sceneBounds);

        if (sizeMinMax.x == sizeMinMax.y)
        {
            numSizeBits = 0;
        }
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
            false,
            false,
            0);
    }
}

//======================================================================================================================
void BuildBvhLinear(
    uint globalId,
    uint numActivePrims,
    uint numPrimitives)
{
    const uint numInternalNodes = numActivePrims - 1;

    for (uint nodeIndex = globalId; nodeIndex < numInternalNodes; nodeIndex += GetNumThreads())
    {
        SplitInternalNodeLbvh(
            nodeIndex,
            numActivePrims,
            CalculateBvhNodesOffset(numActivePrims),
            ShaderConstants.offsets.primIndicesSorted,
            ShaderConstants.offsets.mortonCodesSorted,
            Settings.useMortonCode30,
            Settings.noCopySortedNodes);
    }
}

//======================================================================================================================
bool EnableLatePairCompression()
{
    return (Settings.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
           (Settings.topLevelBuild == false) &&
           (Settings.enableEarlyPairCompression == false);
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
            CalculateBvhNodesOffset(numActivePrims),
            ShaderConstants.offsets.unsortedBvhLeafNodes,
            ShaderConstants.offsets.primIndicesSorted,
            Settings.doCollapse,
            Settings.doTriangleSplitting,
            Settings.noCopySortedNodes,
            Settings.enableEarlyPairCompression,
            EnableLatePairCompression(),
            Settings.enablePairCostCheck,
            ShaderConstants.offsets.splitBoxes,
            ShaderConstants.offsets.numBatches,
            ShaderConstants.offsets.batchIndices,
            Settings.fp16BoxNodesMode,
            Settings.fp16BoxModeMixedSaThreshhold,
            0,
            false,
            false,
            0,
            false);
    }
}

//======================================================================================================================
void FastAgglomerativeLbvh(
    uint globalId,
    uint numActivePrims)
{
    FastLBVHArgs args;

    args.rootNodeIndexOffset         = ShaderConstants.offsets.fastLBVHRootNodeIndex;
    args.topLevelBuild               = Settings.topLevelBuild;
    args.numActivePrims              = numActivePrims;
    args.baseFlagsOffset             = ShaderConstants.offsets.propagationFlags;
    args.baseScratchNodesOffset      = CalculateBvhNodesOffset(numActivePrims);
    args.sortedMortonCodesOffset     = ShaderConstants.offsets.mortonCodesSorted;
    args.useMortonCode30             = Settings.useMortonCode30;
    args.doCollapse                  = Settings.doCollapse;
    args.doTriangleSplitting         = Settings.doTriangleSplitting;
    args.splitBoxesOffset            = ShaderConstants.offsets.splitBoxes;
    args.numBatchesOffset            = ShaderConstants.offsets.numBatches;
    args.baseBatchIndicesOffset      = ShaderConstants.offsets.batchIndices;
    args.fp16BoxNodesMode            = Settings.fp16BoxNodesMode;
    args.fp16BoxModeMixedSaThreshold = Settings.fp16BoxModeMixedSaThreshhold;
    args.noCopySortedNodes           = Settings.noCopySortedNodes;
    args.sortedPrimIndicesOffset     = ShaderConstants.offsets.primIndicesSorted;
    args.enableEarlyPairCompression  = Settings.enableEarlyPairCompression;
    args.unsortedNodesBaseOffset     = ShaderConstants.offsets.unsortedBvhLeafNodes,

    args.enablePairCompression       = EnableLatePairCompression();
    args.enablePairCostCheck         = Settings.enablePairCostCheck;
    args.centroidBoxesOffset         = 0;
    args.enableCentroidBoxes         = false;
    args.ltdPackCentroids            = false;
    args.numMortonBits               = 0;
    args.enableInstancePrimCount     = false;

    for (uint primIndex = globalId; primIndex < numActivePrims; primIndex += GetNumThreads())
    {
        FastAgglomerativeLbvhImpl(primIndex, args);
    }
}

//======================================================================================================================
uint BuildModeFlags()
{
    uint flags = 0;
    flags |= Settings.doCollapse ? BUILD_FLAGS_COLLAPSE : 0;
    flags |= Settings.doTriangleSplitting ? BUILD_FLAGS_TRIANGLE_SPLITTING : 0;
    flags |= EnableLatePairCompression() ? BUILD_FLAGS_LATE_PAIR_COMPRESSION : 0;
    flags |= Settings.enablePairCostCheck ? BUILD_FLAGS_PAIR_COST_CHECK : 0;

    return flags;
}

//======================================================================================================================
void TriangleSplitting(
    uint       globalId,
    uint       localId,
    uint       groupId)
{
    TriangleSplittingArgs args          = (TriangleSplittingArgs)0;

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
    args.tsBudgetPerTriangle            = ShaderConstants.tsBudgetPerTriangle;

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
    plocArgs.scratchNodesScratchOffset      = CalculateBvhNodesOffset(numActivePrims);
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
    plocArgs.primIndicesSortedScratchOffset = ShaderConstants.offsets.primIndicesSorted;
    plocArgs.numLeafNodes                   = ShaderConstants.numLeafNodes;
    plocArgs.noCopySortedNodes              = Settings.noCopySortedNodes;
    plocArgs.enableEarlyPairCompression     = Settings.enableEarlyPairCompression;
    plocArgs.unsortedBvhLeafNodesOffset     = ShaderConstants.offsets.unsortedBvhLeafNodes;

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
    args.enableSAHCost                      = Settings.enableSAHCost;
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

    args.NumPrimitives                = numPrimitives;
    args.AllowUpdate                  = false;  //not used
    args.NumThreads                   = GetNumThreads();
    args.MaxRefCountSize              = numPrimitives * ShaderConstants.rebraidFactor;
    args.LengthPercentage             = 0.1;
    args.BvhNodeDataScratchOffset     = ShaderConstants.offsets.bvhNodes;
    args.BvhLeafNodeDataScratchOffset = ShaderConstants.offsets.unsortedBvhLeafNodes;
    args.SceneBoundsOffset            = ShaderConstants.offsets.sceneBounds;
    args.RefScratchOffset             = ShaderConstants.offsets.tdRefs;
    args.TDNodeScratchOffset          = ShaderConstants.offsets.tdNodes;
    args.TDBinsScratchOffset          = ShaderConstants.offsets.tdBins;
    args.CurrentStateScratchOffset    = ShaderConstants.offsets.tdState;
    args.TdTaskCounterScratchOffset   = ShaderConstants.offsets.tdTaskCounters;
    args.OffsetsScratchOffset         = 0; //not used
    args.EncodeArrayOfPointers        = ShaderConstants.encodeArrayOfPointers;

    BuildBVHTDImpl(globalId, localId, groupId, args);
}

//======================================================================================================================
void PairCompression(
    uint globalId,
    uint localId,
    uint numActivePrims)
{
    const uint buildInfo  = DstBuffer.Load(ACCEL_STRUCT_HEADER_INFO_OFFSET);
    const uint buildFlags = (buildInfo >> ACCEL_STRUCT_HEADER_INFO_FLAGS_SHIFT) & ACCEL_STRUCT_HEADER_INFO_FLAGS_MASK;

    PairCompressionArgs args;

    args.scratchNodesScratchOffset    = CalculateBvhNodesOffset(numActivePrims);
    args.numBatchesScratchOffset      = ShaderConstants.offsets.numBatches;
    args.batchIndicesScratchOffset    = ShaderConstants.offsets.batchIndices;
    args.indexBufferInfoScratchOffset = ShaderConstants.offsets.indexBufferInfo;
    args.flagsScratchOffset           = ShaderConstants.offsets.propagationFlags;
    args.buildFlags                   = buildFlags;

    PairCompressionImpl(globalId, localId, args);
}

//======================================================================================================================
void InitBuildQbvh(
    uint globalId,
    uint numLeafNodes,
    uint numActivePrims)
{
    BuildQbvhArgs qbvhArgs;

    qbvhArgs.numPrimitives               = numLeafNodes;
    qbvhArgs.metadataSizeInBytes         = DstMetadata.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET);
    qbvhArgs.numThreads                  = GetNumThreads();
    qbvhArgs.scratchNodesScratchOffset   = CalculateBvhNodesOffset(numActivePrims);
    qbvhArgs.qbvhStackScratchOffset      = ShaderConstants.offsets.qbvhStack;
    qbvhArgs.stackPtrsScratchOffset      = ShaderConstants.offsets.stackPtrs;
    qbvhArgs.splitBoxesByteOffset        = ShaderConstants.offsets.splitBoxes;
    qbvhArgs.triangleCompressionMode     = Settings.triangleCompressionMode;
    qbvhArgs.fp16BoxNodesInBlasMode      = Settings.fp16BoxNodesMode;
    qbvhArgs.flags                       = BuildModeFlags();
    qbvhArgs.encodeArrayOfPointers       = ShaderConstants.encodeArrayOfPointers;
    qbvhArgs.rebraidEnabled              = (Settings.rebraidType != RebraidType::Off);
    qbvhArgs.enableFusedInstanceNode     = Settings.enableFusedInstanceNode;
    qbvhArgs.enableFastLBVH              = Settings.enableFastLBVH;
    qbvhArgs.fastLBVHRootNodeIndex       = Settings.enableFastLBVH ?
        ScratchBuffer.Load(ShaderConstants.offsets.fastLBVHRootNodeIndex) : 0;
    qbvhArgs.enableEarlyPairCompression  = Settings.enableEarlyPairCompression;
    qbvhArgs.unsortedBvhLeafNodesOffset  = ShaderConstants.offsets.unsortedBvhLeafNodes;

    if (!Settings.topLevelBuild)
    {
        qbvhArgs.captureChildNumPrimsForRebraid = true;
    }
    else
    {
        qbvhArgs.captureChildNumPrimsForRebraid = false;
    }

    qbvhArgs.enableSAHCost = Settings.enableSAHCost;

    InitBuildQbvhImpl(globalId, qbvhArgs);
}

//======================================================================================================================
void BuildQbvh(
    uint globalId,
    uint localId,
    uint numLeafNodes,
    uint numActivePrims)
{
    BuildQbvhArgs qbvhArgs;

    qbvhArgs.numPrimitives               = numLeafNodes;
    qbvhArgs.metadataSizeInBytes         = DstMetadata.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET);
    qbvhArgs.numThreads                  = GetNumThreads();
    qbvhArgs.scratchNodesScratchOffset   = CalculateBvhNodesOffset(numActivePrims);
    qbvhArgs.qbvhStackScratchOffset      = ShaderConstants.offsets.qbvhStack;
    qbvhArgs.stackPtrsScratchOffset      = ShaderConstants.offsets.stackPtrs;
    qbvhArgs.splitBoxesByteOffset        = ShaderConstants.offsets.splitBoxes;
    qbvhArgs.triangleCompressionMode     = Settings.triangleCompressionMode;
    qbvhArgs.fp16BoxNodesInBlasMode      = Settings.fp16BoxNodesMode;
    qbvhArgs.flags                       = BuildModeFlags();
    qbvhArgs.encodeArrayOfPointers       = ShaderConstants.encodeArrayOfPointers;
    qbvhArgs.rebraidEnabled              = (Settings.rebraidType != RebraidType::Off);
    qbvhArgs.enableFusedInstanceNode     = Settings.enableFusedInstanceNode;
    qbvhArgs.enableEarlyPairCompression  = Settings.enableEarlyPairCompression;
    qbvhArgs.enableFastLBVH              = Settings.enableFastLBVH;
    qbvhArgs.fastLBVHRootNodeIndex       = Settings.enableFastLBVH ?
        ScratchBuffer.Load(ShaderConstants.offsets.fastLBVHRootNodeIndex) : 0;
    qbvhArgs.unsortedBvhLeafNodesOffset  = ShaderConstants.offsets.unsortedBvhLeafNodes;

    if (!Settings.topLevelBuild)
    {
        qbvhArgs.captureChildNumPrimsForRebraid = true;
    }
    else
    {
        qbvhArgs.captureChildNumPrimsForRebraid = false;
    }

    qbvhArgs.enableSAHCost = Settings.enableSAHCost;

    if ((Settings.topLevelBuild == 0) && Settings.doCollapse)
    {
        BuildQbvhCollapseImpl(globalId, numActivePrims, qbvhArgs);
    }
    else
    {
        BuildQbvhImpl(globalId, localId, numActivePrims, qbvhArgs, (Settings.topLevelBuild != 0));
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
// Set a scratch buffer counter to 0 if it has a valid index
void InitScratchCounter(uint offset)
{
    if (offset != INVALID_IDX)
    {
        ScratchBuffer.Store(offset, 0);
    }
}
//======================================================================================================================
// Initialize headers and task counters
void InitAccelerationStructure()
{
    // Initalize headers (metadata task counter is initialized using the CP)
    if (Settings.topLevelBuild && (ShaderConstants.numPrimitives == 0))
    {
        DstMetadata.Store(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, 0);
        DstMetadata.Store(ACCEL_STRUCT_METADATA_VA_HI_OFFSET, 0);
    }
    else
    {
        DstMetadata.Store(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, ShaderConstants.resultBufferAddrLo);
        DstMetadata.Store(ACCEL_STRUCT_METADATA_VA_HI_OFFSET, ShaderConstants.resultBufferAddrHi);
    }
    DstMetadata.Store(ACCEL_STRUCT_METADATA_SIZE_OFFSET, ShaderConstants.header.metadataSizeInBytes);

    DstBuffer.Store(0, ShaderConstants.header);

    // Initialize valid scratch buffer counters to 0
    InitScratchCounter(ShaderConstants.offsets.currentState);
    InitScratchCounter(ShaderConstants.offsets.tdTaskCounters);
    InitScratchCounter(ShaderConstants.offsets.currentSplitState);
    InitScratchCounter(ShaderConstants.offsets.numBatches);

    // Initialize scene bounds
    const uint maxVal = FloatToUint(FLT_MAX);
    const uint minVal = FloatToUint(-FLT_MAX);

    uint offset = ShaderConstants.offsets.sceneBounds;
    ScratchBuffer.Store3(offset, maxVal.xxx);
    offset += sizeof(uint3);
    ScratchBuffer.Store3(offset, minVal.xxx);
    offset += sizeof(uint3);
    ScratchBuffer.Store2(offset, uint2(maxVal, minVal));
    offset += sizeof(uint2);

    if (Settings.rebraidType == RebraidType::V2)
    {
        ScratchBuffer.Store3(offset, maxVal.xxx);
        offset += sizeof(uint3);
        ScratchBuffer.Store3(offset, minVal.xxx);
    }
}

#if !AMD_VULKAN
//======================================================================================================================
// Encode primitives for each geometry.
void EncodePrimitives(
    uint globalId,
    uint localId)
{
    for (uint geometryIndex = 0; geometryIndex < ShaderConstants.numDescs; geometryIndex++)
    {
        const uint primCount = GeometryConstants[geometryIndex].NumPrimitives;
        const uint geometryBasePrimOffset = GeometryConstants[geometryIndex].PrimitiveOffset;

        GeometryArgs geometryArgs = GeometryConstants[geometryIndex];
        geometryArgs.BuildFlags = 0; // Indicate this is not an update

        if (globalId == 0)
        {
            WriteGeometryInfo(
                geometryArgs, geometryBasePrimOffset, geometryArgs.NumPrimitives, DECODE_PRIMITIVE_STRIDE_TRIANGLE);
        }

        for (uint primitiveIndex = globalId; primitiveIndex < primCount; primitiveIndex += GetNumThreads())
        {
            // not an update, just check the "enableEarlyPairCompression" and decide which Encode path to use
            if (Settings.enableEarlyPairCompression == true)
            {
                EncodePairedTriangleNodeImpl(
                    GeometryBuffer[geometryIndex],
                    IndexBuffer[geometryIndex],
                    TransformBuffer[geometryIndex],
                    geometryArgs,
                    primitiveIndex,
                    localId,
                    geometryBasePrimOffset,
                    0,
                    false); // Don't write to the update stack
            }
            else
            {
                EncodeTriangleNode(
                    GeometryBuffer[geometryIndex],
                    IndexBuffer[geometryIndex],
                    TransformBuffer[geometryIndex],
                    geometryArgs,
                    primitiveIndex,
                    geometryBasePrimOffset,
                    0,
                    false); // Don't write to the update stack
            }
        }
    }
}
#endif

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

    uint waveId = 0;
    uint numTasksWait = 0;

    if (Settings.doEncode == false)
    {
        numTasksWait = ShaderConstants.numPrimitives;

        // Wait for encode to finish
        WaitForTasksToFinish(numTasksWait);
    }

    INIT_TASK;

#if !AMD_VULKAN
    if (Settings.doEncode)
    {
        BEGIN_TASK(1);

        if (globalId == 0)
        {
            InitAccelerationStructure();
        }

        END_TASK(1);

        BEGIN_TASK(ShaderConstants.numThreadGroups);

        EncodePrimitives(globalId, localId);

        END_TASK(ShaderConstants.numThreadGroups);
    }
    else
#endif
    {
        // Take into account the encode tasks that are done
        if ((waveId == numTasksWait) && (localId == 0))
        {
            DstMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_NUM_TASKS_DONE_OFFSET, numTasksWait);
        }
    }

    DeviceMemoryBarrierWithGroupSync();

    uint numPrimitives = ShaderConstants.numPrimitives;

    if (Settings.doTriangleSplitting)
    {
        TriangleSplitting(globalId, localId, groupId);

        numPrimitives = DstBuffer.Load(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET);
    }
    else if (Settings.rebraidType == RebraidType::V2)
    {
        Rebraid(globalId, localId, groupId);

        numPrimitives = DstBuffer.Load(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET);
    }

    uint numActivePrims;

    if (Settings.enableTopDownBuild)
    {
        BuildBvhTD(globalId, localId, groupId, numPrimitives);

        numActivePrims = DstBuffer.Load(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
    }
    else
    {
        bool needRefit = false;

        if ((Settings.fastBuildThreshold) && (numPrimitives <= Settings.fastBuildThreshold) && (numPrimitives <= WaveGetLaneCount()))
        {
            BEGIN_TASK(1);

            FastBuildBVH(globalId,
                         numPrimitives,
                         ShaderConstants.offsets.unsortedBvhLeafNodes,
                         ShaderConstants.offsets.bvhNodes);

            END_TASK(1);
            needRefit = true;
            numActivePrims = DstBuffer.Load(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
        }
        else
        {
            BEGIN_TASK(ShaderConstants.numThreadGroups);

            GenerateMortonCodes(globalId, numPrimitives);

            END_TASK(ShaderConstants.numThreadGroups);
            writeDebugCounter(COUNTER_MORTONGEN_OFFSET);
            numActivePrims = DstBuffer.Load(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);

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

                if (Settings.noCopySortedNodes == false)
                {
                    BEGIN_TASK(ShaderConstants.numThreadGroups);

                    SortScratchLeaves(globalId, numActivePrims);

                    END_TASK(ShaderConstants.numThreadGroups);
                    writeDebugCounter(COUNTER_SORTLEAF_OFFSET);
                }
                // If noCopySortedNodes is on, the unsorted leaves will stay where the
                // Encode step put them. On top of that, if TS or Rebraid is also on,
                // there might be a gap between the last inner node and the first leaf
                // if we place the root of the tree at ShaderConstants.offsets.bvhNodes.
                // To avoid that gap, the root is moved forward by numLeafNodes - numActivePrims
                // nodes from this point onwards.

                if (Settings.buildMode == BUILD_MODE_PLOC)
                {
                    BuildBvhPloc(numTasksWait, waveId, globalId, localId, groupId, numActivePrims);
                    writeDebugCounter(COUNTER_BUILDPLOC_OFFSET);
                }
                else
                {
                    if (Settings.enableFastLBVH == false)
                    {
                        BEGIN_TASK(ShaderConstants.numThreadGroups);

                        BuildBvhLinear(globalId, numActivePrims, numPrimitives);

                        END_TASK(ShaderConstants.numThreadGroups);
                        writeDebugCounter(COUNTER_BUILDLBVH_OFFSET);
                        needRefit = true;
                    }
                    else
                    {
                        BEGIN_TASK(ShaderConstants.numThreadGroups);

                        FastAgglomerativeLbvh(globalId, numActivePrims);

                        END_TASK(ShaderConstants.numThreadGroups);
                        writeDebugCounter(COUNTER_BUILDFASTLBVH_OFFSET);
                    }
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

            const uint geometryType = DstBuffer.Load(ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET);

            if (EnableLatePairCompression() && (geometryType == GEOMETRY_TYPE_TRIANGLES))
            {
                const uint numBatches = ScratchBuffer.Load(ShaderConstants.offsets.numBatches);

                BEGIN_TASK(RoundUpQuotient(numBatches, BUILD_THREADGROUP_SIZE));

                PairCompression(globalId, localId, numActivePrims);

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
                DstMetadata.Store<GpuVirtualAddress>(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, 0);
            }

            END_TASK(1);
            writeDebugCounter(COUNTER_EMPTYPRIM_OFFSET);
        }
    }

    if (numActivePrims > 0)
    {
        // Note, BuildQBVH only needs to run for valid leaf nodes. numActivePrims already represents
        // valid leaf nodes when rebraid or triangle splitting is enabled.
        uint numLeafNodes = numActivePrims;

        // Fetch leaf node count when triangle compression is enabled.
        if (EnableLatePairCompression())
        {
            numLeafNodes = DstBuffer.Load(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET);
        }

        BEGIN_TASK(ShaderConstants.numThreadGroups);

        InitBuildQbvh(globalId, numLeafNodes, numActivePrims);

        END_TASK(ShaderConstants.numThreadGroups);
        writeDebugCounter(COUNTER_INITQBVH_OFFSET);

        BEGIN_TASK(RoundUpQuotient(CalcNumQBVHInternalNodes(numLeafNodes), BUILD_THREADGROUP_SIZE));

        BuildQbvh(globalId, localId, numLeafNodes, numActivePrims);

        END_TASK(RoundUpQuotient(CalcNumQBVHInternalNodes(numLeafNodes), BUILD_THREADGROUP_SIZE));
        writeDebugCounter(COUNTER_BUILDQBVH_OFFSET);
    }

    BEGIN_TASK(1);

    if (localId == 0)
    {
        WriteCompactedSize(DstBuffer,
                           EmitBuffer,
                           Settings.emitCompactSize,
                           Settings.topLevelBuild ? TOP_LEVEL : BOTTOM_LEVEL);
    }

    END_TASK(1);
    writeDebugCounter(COUNTER_EMITCOMPACTSIZE_OFFSET);
}
