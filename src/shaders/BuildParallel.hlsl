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
#define BUILD_PARALLEL 1

#if GPURT_BUILD_RTIP3|| GPURT_BUILD_RTIP3_1
#if GPURT_BUILD_PARALLEL_ENABLE_RTIP3 == 0

#if GPURT_BUILD_RTIP3
#undef GPURT_BUILD_RTIP3
#endif

#if GPURT_BUILD_RTIP3_1
#undef GPURT_BUILD_RTIP3_1
#endif

#endif
#endif

#include "../shadersClean/common/ShaderDefs.hlsli"

#define TASK_COUNTER_BUFFER   ScratchGlobal
#define TASK_COUNTER_OFFSET   (ShaderConstants.offsets.taskLoopCounters + TASK_LOOP_BUILD_PARALLEL_COUNTER_OFFSET)
#define NUM_TASKS_DONE_OFFSET (ShaderConstants.offsets.taskLoopCounters + TASK_LOOP_BUILD_PARALLEL_TASKS_DONE_OFFSET)
#include "TaskMacros.hlsl"

#define GC_DSTBUFFER
#define GC_DSTMETADATA
#define GC_SCRATCHBUFFER
#include "../shadersClean/build/BuildRootSignature.hlsli"

template<typename T>
T LoadInstanceDescBuffer(uint offset)
{
    return InstanceDescBuffer.Load<T>(offset);
}
#include "IndirectArgBufferUtils.hlsl"
#define SrcBuffer InstanceDescBuffer
#include "../shadersClean/build/BuildCommonScratch.hlsli"
#include "CompactCommon.hlsl"
#undef SrcBuffer

#define MAX_ELEMENT_PER_THREAD 16
#define MAX_LDS_ELEMENTS (MAX_ELEMENT_PER_THREAD * BUILD_THREADGROUP_SIZE)
groupshared uint SharedMem[MAX_LDS_ELEMENTS];
uint GetSharedMem(uint index)
{
    return SharedMem[index];
}
void SetSharedMem(uint index, uint value)
{
    SharedMem[index] = value;
}

#include "TaskQueueCounter.hlsl"

// The encode path uses SrcBuffer and DstBuffer as the true acceleration structure base.
#define SrcBuffer DstMetadata
#include "EncodeCommon.hlsl"
#include "EncodePairedTriangleImpl.hlsl"
#undef SrcBuffer

//======================================================================================================================
// Returns number of threads launched
uint GetNumThreads()
{
    return ShaderRootConstants.NumThreadGroups() * BUILD_THREADGROUP_SIZE;
}

//======================================================================================================================
// Wait for the task counter to reach the specified value
void WaitForEncodeTasksToFinish(
    uint numTasksWait)
{
    const uint offset = ShaderConstants.offsets.encodeTaskCounter + ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET;
    do
    {
        DeviceMemoryBarrier();
    } while (FetchTaskCounter(offset) < numTasksWait);
}

// Include implementations for each pass without shader entry points and resource declarations
#define NO_SHADER_ENTRYPOINT 1

#include "../shadersClean/build/EncodeTopLevelCommon.hlsli"
#include "GenerateMortonCodes.hlsl"
#include "RadixSort/ScanExclusiveInt4DLBCommon.hlsl"
#include "RadixSort/RadixSortParallel.hlsl"
#include "BuildPLOC.hlsl"
#include "BuildQBVH.hlsl"
#include "BuildFastAgglomerativeLbvh.hlsl"
#include "PairCompression.hlsl"
#include "Rebraid.hlsl"
#include "MergeSort.hlsl"

#if GPURT_BUILD_RTIP3_1
#include "EncodeHwBvh3_1.hlsl"
#include "PrimitiveStructure3_1.hlsl"
#include "RefitOrientedBounds3_1.hlsl"
#include "RefitOrientedBoundsTopLevel3_1.hlsl"
#endif

//======================================================================================================================
void GenerateMortonCodes(
    uint globalId,
    uint numPrimitives)
{
    const uint primCount = numPrimitives;

    const MortonBoundingBox mortonBounds = FetchMortonBounds(ShaderConstants.offsets.sceneBounds, ShaderConstants.numMortonSizeBits);

    for (uint primitiveIndex = globalId; primitiveIndex < primCount; primitiveIndex += GetNumThreads())
    {
        GenerateMortonCodesImpl(
            mortonBounds.min,
            mortonBounds.extent,
            mortonBounds.sizeMinMax,
            primitiveIndex,
            ShaderConstants.offsets.bvhLeafNodeData,
            ShaderConstants.offsets.mortonCodes,
            Settings.useMortonCode30,
            mortonBounds.numSizeBits);
    }
}

//======================================================================================================================
void FastAgglomerativeLbvh(
    uint globalId,
    uint numActivePrims)
{
    const FastLBVHArgs args = GetFastLbvhArgs(numActivePrims);

    for (uint primIndex = globalId; primIndex < numActivePrims; primIndex += GetNumThreads())
    {
        FastAgglomerativeLbvhImpl(primIndex, args);
    }
}

//======================================================================================================================
void BuildPloc(
    inout uint numTasksWait,
    inout uint waveId,
    uint       globalId,
    uint       localId,
    uint       groupId,
    uint       numActivePrims)
{
    BuildPlocArgs plocArgs;

    plocArgs.numThreads                     = GetNumThreads();
    plocArgs.scratchNodesScratchOffset      = CalculateBvhNodesOffset(ShaderConstants, numActivePrims);
    plocArgs.clusterList0ScratchOffset      = ShaderConstants.offsets.clusterList0;
    plocArgs.clusterList1ScratchOffset      = ShaderConstants.offsets.clusterList1;
    plocArgs.neighbourIndicesScratchOffset  = ShaderConstants.offsets.neighborIndices;
    plocArgs.currentStateScratchOffset      = ShaderConstants.offsets.currentState;
    plocArgs.taskQueueCounterScratchOffset  = ShaderConstants.offsets.plocTaskQueueCounter;
    plocArgs.atomicFlagsScratchOffset       = ShaderConstants.offsets.atomicFlagsPloc;
    plocArgs.dynamicBlockIndexScratchOffset = ShaderConstants.offsets.dynamicBlockIndex;
    plocArgs.numBatchesScratchOffset        = ShaderConstants.offsets.numBatches;
    plocArgs.baseBatchIndicesScratchOffset  = ShaderConstants.offsets.batchIndices;
    plocArgs.fp16BoxNodesInBlasMode         = Settings.fp16BoxNodesMode;
    plocArgs.fp16BoxModeMixedSaThresh       = Settings.fp16BoxModeMixedSaThreshold;
    plocArgs.plocRadius                     = Settings.nnSearchRadius;
    plocArgs.primIndicesSortedScratchOffset = ShaderConstants.offsets.primIndicesSorted;
    plocArgs.unsortedBvhLeafNodesOffset     = ShaderConstants.offsets.bvhLeafNodeData;

    BuildPlocImpl(globalId, localId, groupId, numActivePrims, plocArgs);
}

//======================================================================================================================
void Rebraid(
    uint       globalId,
    uint       localId,
    uint       groupId)
{
    RebraidArgs args;

    args.numPrimitives                      = ShaderConstants.numPrimitives;
    args.numThreadGroups                    = ShaderRootConstants.NumThreadGroups();
    args.maxNumPrims                        = ShaderConstants.rebraidFactor * ShaderConstants.numPrimitives;
    args.bvhLeafNodeDataScratchOffset       = ShaderConstants.offsets.bvhLeafNodeData;
    args.sceneBoundsOffset                  = ShaderConstants.offsets.sceneBounds;
    args.stateScratchOffset                 = ShaderConstants.offsets.rebraidState;
    args.taskQueueCounterScratchOffset      = ShaderConstants.offsets.rebraidTaskQueueCounter;
    args.atomicFlagsScratchOffset           = ShaderConstants.offsets.rebraidPrefixSumFlags;
    args.enableMortonSize                   = IsMortonSizeBitsEnabled(ShaderConstants.numMortonSizeBits);
    args.enableMortonCentroids              = IsCentroidMortonBoundsEnabled() || IsConciseMortonBoundsEnabled();
    args.numIterations                      = Settings.numRebraidIterations;
    args.qualityHeuristic                   = Settings.rebraidQualityHeuristic;

    RebraidImpl(globalId, localId, groupId, args);
}

//======================================================================================================================
void PairCompression(
    uint globalId,
    uint localId,
    uint numActivePrims)
{
    PairCompressionImpl(globalId, localId, numActivePrims);
}

//======================================================================================================================
void InitEncodeHwBvh(
    uint globalId,
    uint maxInternalNodeCount)
{
#if GPURT_BUILD_RTIP3_1
    if (Settings.rtIpLevel >= GPURT_RTIP3_1)
    {
        ClearStackPtrs(globalId, maxInternalNodeCount, GetNumThreads());
    }
    else
#endif
    {
        InitBuildQBVHImpl(globalId, maxInternalNodeCount, GetNumThreads());
    }
}

//======================================================================================================================
void EncodeHwBvh(
    inout uint numTasksWait,
    inout uint waveId,
    uint       globalId,
    uint       localId,
    uint       numLeafNodes,
    uint       numActivePrims,
    uint       maxInternalNodeCount)
{
#if GPURT_BUILD_RTIP3_1
    if (Settings.rtIpLevel == GPURT_RTIP3_1)
    {
        const uint numThreads = maxInternalNodeCount * 8;

        BEGIN_TASK(RoundUpQuotient(numThreads, BUILD_THREADGROUP_SIZE));
        Gfx12::EncodeHwBvhImpl(globalId, localId, numActivePrims);
        END_TASK(RoundUpQuotient(numThreads, BUILD_THREADGROUP_SIZE));
    }
    else
#endif
    {
        BEGIN_TASK(RoundUpQuotient(maxInternalNodeCount, BUILD_THREADGROUP_SIZE));
        BuildQBVHImpl(globalId, localId, numLeafNodes, numActivePrims);
        END_TASK(RoundUpQuotient(maxInternalNodeCount, BUILD_THREADGROUP_SIZE));
    }
}

#if GPURT_BUILD_RTIP3_1
//======================================================================================================================
void RefitOrientedBounds(
    inout uint numTasksWait,
    inout uint waveId,
    in    uint localId)
{
    const uint obbRefitBatchCount =
        ScratchGlobal.Load(ShaderConstants.offsets.obbRefitStackPtrs + STACK_PTRS_OBB_REFIT_BATCH_COUNT);

    // Note, a group of 8 lanes within a threadgroup works on a single internal node.
    const uint numThreadGroups = RoundUpQuotient(obbRefitBatchCount, BUILD_THREADGROUP_SIZE / 8);

    BEGIN_TASK(numThreadGroups);
    if (Settings.topLevelBuild == true)
    {
        RefitOrientedBoundsTopLevelImpl(globalId, localId, obbRefitBatchCount);
    }
    else
    {
        RefitOrientedBoundsImpl(globalId, localId, obbRefitBatchCount);
    }
    END_TASK(numThreadGroups);
}

//======================================================================================================================
void CompressPrimsPass(
    inout uint numTasksWait,
    inout uint waveId,
    in    uint localId)
{
    const uint batchCount = FetchCompressPrimBatchCount(ShaderConstants.offsets.qbvhGlobalStackPtrs);

    const AccelStructHeader header = DstBuffer.Load<AccelStructHeader>(0);

    BEGIN_TASK(batchCount);

    // The primitive compression pass is designed for wave and group size 32
    if (localId < 32)
    {
        const uint rangeBatchOffset = groupId * 8 * sizeof(uint);

        PrimStruct3_1::WaveCompressLeafChildren(
            header.offsets,
            header.numActivePrims,
            rangeBatchOffset);
    }

    END_TASK(batchCount);

    const uint singlePrimCount  = FetchCompressPrimSingleCount(ShaderConstants.offsets.qbvhGlobalStackPtrs);
    const uint singlePrimGroups = RoundUpQuotient(RoundUpQuotient(singlePrimCount, 2), BUILD_THREADGROUP_SIZE);

    BEGIN_TASK(singlePrimGroups);

    PrimStruct3_1::CompressSinglePrims(globalId);

    END_TASK(singlePrimGroups);
}
#endif

//======================================================================================================================
void WriteDebugCounter(uint counterStageOffset)
{
    if (Settings.enableBVHBuildDebugCounters)
    {
        IncreaseDebugCounters(ShaderConstants.offsets.debugCounters, counterStageOffset);
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
                  ShaderConstants.offsets.primIndicesSortedSwap,
                  ShaderConstants.offsets.primIndicesSorted,
                  Settings.useMortonCode30);
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

    if (Settings.doEncode)
    {
        // Initialise encode counters
        WriteTaskCounterData(
            ShaderConstants.offsets.encodeTaskCounter, ENCODE_TASK_COUNTER_NUM_PRIMITIVES_OFFSET, 0);

        // Early triangle pairing and indirect builds dynamically increment primitive reference counter. Initialise
        // counters to 0 when these features are enabled
        const bool dynamicallyIncrementsPrimRefCount = Settings.enableEarlyPairCompression || Settings.isIndirectBuild;
        const uint primRefInitCount = (dynamicallyIncrementsPrimRefCount) ? 0 : ShaderConstants.numPrimitives;

        WriteTaskCounterData(
            ShaderConstants.offsets.encodeTaskCounter, ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET, primRefInitCount);

        // Initialize valid scratch buffer counters to 0
        InitScratchCounter(ShaderConstants.offsets.plocTaskQueueCounter);
        InitScratchCounter(ShaderConstants.offsets.rebraidTaskQueueCounter);
        ClearNumBatches(ShaderConstants.offsets.numBatches);

        // Initialize scene bounds
        InitSceneBounds(ShaderConstants.offsets.sceneBounds);
    }
}

//======================================================================================================================
// Encode primitives for each geometry.
void EncodePrimitives(
    uint globalId,
    uint localId)
{
    for (uint geometryIndex = 0; geometryIndex < ShaderConstants.numDescs; geometryIndex++)
    {
        const BuildShaderGeometryConstants geomConstants = GeometryConstants[geometryIndex];
        const NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrim(geometryIndex, true);

        if (globalId == 0)
        {
            WriteGeometryInfo(
                geomConstants,
                inputOffsets,
                geometryIndex,
                DECODE_PRIMITIVE_STRIDE_TRIANGLE);

            if (Settings.isIndirectBuild)
            {
                IncrementTaskCounter(
                    ShaderConstants.offsets.encodeTaskCounter + ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET,
                    inputOffsets.numPrimitives);
            }
        }

        for (uint primitiveIndex = globalId; primitiveIndex < inputOffsets.numPrimitives; primitiveIndex += GetNumThreads())
        {
            EncodeTriangleNode(
                geomConstants,
                inputOffsets,
                geometryIndex,
                primitiveIndex,
                false); // Don't write to the update stack
        }
    }
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

    uint waveId = 0;
    uint numTasksWait = 0;

    if (Settings.doEncode == false)
    {
        WaitForEncodeTasksToFinish(ShaderConstants.numPrimitives);
    }

    INIT_TASK;

    BEGIN_TASK(1);

    if (globalId == 0)
    {
        InitAccelerationStructure();
    }

    END_TASK(1);

    if (Settings.doEncode)
    {
        BEGIN_TASK(ShaderRootConstants.NumThreadGroups());

        EncodePrimitives(globalId, localId);

        END_TASK(ShaderRootConstants.NumThreadGroups());
    }

    DeviceMemoryBarrierWithGroupSync();

    if (Settings.enableRebraid)
    {
        Rebraid(globalId, localId, groupId);
    }

    // Note, the primitive reference counter in scratch represents the following.
    //
    // BLAS Default: Number of triangles processed by Encode.
    // BLAS EarlyPairCompression: Number of triangle references including paired triangles.
    // TLAS Default: Number of instances processed by Encode.
    // TLAS Rebraid: Number of rebraided instances created by the rebraid process.
    //
    // TODO: Support early triangle pairing with triangle splitting.
    //
    const uint numPrimitives = FetchTaskCounter(
        ShaderConstants.offsets.encodeTaskCounter + ENCODE_TASK_COUNTER_PRIM_REFS_OFFSET);

    uint numActivePrims;

    BEGIN_TASK(ShaderRootConstants.NumThreadGroups());

    GenerateMortonCodes(globalId, numPrimitives);

    END_TASK(ShaderRootConstants.NumThreadGroups());
    WriteDebugCounter(COUNTER_MORTONGEN_OFFSET);
    numActivePrims = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);

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
        WriteDebugCounter(COUNTER_MORTON_SORT_OFFSET);
        // Note there is an implicit sync on the last pass of the sort

        // If the top down builder is off, the unsorted leaves will stay where the
            // Encode step put them. On top of that, if TS or Rebraid is also on,
            // there might be a gap between the last inner node and the first leaf
            // if we place the root of the tree at ShaderConstants.offsets.bvhNodeData.
        // To avoid that gap, the root is moved forward by numLeafNodes - numActivePrims
        // nodes from this point onwards.

        if (Settings.buildMode == BUILD_MODE_PLOC)
        {
            BuildPloc(numTasksWait, waveId, globalId, localId, groupId, numActivePrims);
            WriteDebugCounter(COUNTER_BUILDPLOC_OFFSET);
        }
        else
        {
            BEGIN_TASK(ShaderRootConstants.NumThreadGroups());

            FastAgglomerativeLbvh(globalId, numActivePrims);

            END_TASK(ShaderRootConstants.NumThreadGroups());
            WriteDebugCounter(COUNTER_BUILDFASTLBVH_OFFSET);
        }
    }

    if (numActivePrims > 0)
    {
        const uint geometryType = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET);

        if (EnableLatePairCompression() && (geometryType == GEOMETRY_TYPE_TRIANGLES))
        {
            const uint numBatches = FetchNumBatches(ShaderConstants.offsets.numBatches);

            BEGIN_TASK(RoundUpQuotient(numBatches, BUILD_THREADGROUP_SIZE));

            PairCompression(globalId, localId, numActivePrims);

            END_TASK(RoundUpQuotient(numBatches, BUILD_THREADGROUP_SIZE));
        }
    }
    else if (Settings.topLevelBuild)
    {
        WriteDebugCounter(COUNTER_EMPTYPRIM_OFFSET);
    }

    if (numActivePrims > 0)
    {
        // Note, EncodeHwBvh only needs to run for valid leaf nodes. numActivePrims already represents
        // valid leaf nodes when rebraid or triangle splitting is enabled.
        uint numLeafNodes = numActivePrims;

        // Fetch leaf node count when triangle compression is enabled.
        if (EnableLatePairCompression())
        {
            numLeafNodes = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET);
        }

        const uint maxInternalNodeCount = GetMaxInternalNodeCount(numLeafNodes);

#if GPURT_BUILD_RTIP3_1
        BEGIN_TASK(1)
        if ((globalId == 0) && (Settings.rtIpLevel >= GPURT_RTIP3_1))
        {
            InitStackPtrs();
            if (Settings.rtIpLevel == GPURT_RTIP3_1)
            {
                Gfx12::InitObbRefitStackPtrs();
            }
        }
        END_TASK(1);
#endif

        BEGIN_TASK(ShaderRootConstants.NumThreadGroups());

        InitEncodeHwBvh(globalId, maxInternalNodeCount);

        END_TASK(ShaderRootConstants.NumThreadGroups());
        WriteDebugCounter(COUNTER_INITENCODEHWBVH_OFFSET);

        EncodeHwBvh(numTasksWait, waveId, globalId, localId, numLeafNodes, numActivePrims, maxInternalNodeCount);

        WriteDebugCounter(COUNTER_ENCODEHWBVH_OFFSET);

#if GPURT_BUILD_RTIP3_1
        if (Gfx12::EnableWaveLeafCompressionPass())
        {
            CompressPrimsPass(numTasksWait, waveId, localId);
        }

        const bool blasWithOBB = (Settings.topLevelBuild == false) && (Settings.enableOrientedBoundingBoxes != 0);
        if (blasWithOBB)
        {
            uint doRebraid;

            if (EnableNonPrioritySortingRebraid())
            {
                const uint tempInfo = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_INFO_OFFSET);
                doRebraid = (tempInfo >> ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_SHIFT) &
                            ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_MASK;
            }
            else
            {
                doRebraid = Settings.enableInstanceRebraid;
            }

            if (doRebraid)
            {
                const uint numInternalNodes =
                    ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET);

                BEGIN_TASK(1)
                InitObbBlasMetadata(globalId, numInternalNodes, BUILD_THREADGROUP_SIZE);
                END_TASK(1)
            }

            RefitOrientedBounds(numTasksWait, waveId, localId);
        }
#endif
    }

    BEGIN_TASK(1);

    PostHwBvhBuild(localId, numActivePrims);

    END_TASK(1);
    WriteDebugCounter(COUNTER_EMITCOMPACTSIZE_OFFSET);
}
