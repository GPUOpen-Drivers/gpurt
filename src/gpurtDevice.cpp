/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2019-2022 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "gpurtInternal.h"
#include "gpurtBvhBuilder.h"
#include "gpurtCpuBvhBuilder.h"
#include "gpurt/gpurtLib.h"
#include "gpurt/gpurtCounter.h"
#include "gpurtInternalShaderBindings.h"

#include "palCmdBuffer.h"
#include "palInlineFuncs.h"
#include "palHashMapImpl.h"
#include "palMemTrackerImpl.h"
#include "palHashSetImpl.h"
#include "palVectorImpl.h"

#if GPURT_DEVELOPER
#if defined(__GNUC__)
#define RGP_PUSH_MARKER(format1, format2, ...) PushRGPMarker(format1, format2, ##__VA_ARGS__)
#else
#define RGP_PUSH_MARKER(format1, format2, ...) PushRGPMarker(format1, format2, __VA_ARGS__)
#endif
#define RGP_POP_MARKER(format) PopRGPMarker(format)
#else
#define RGP_PUSH_MARKER(format1, format2, ...) (static_cast<void>(0))
#define RGP_POP_MARKER(format) (static_cast<void>(0))
#endif

#define GPURT_ARRAY_SIZE(x) ((sizeof(x))/sizeof((x)[0]))

namespace GpuRt
{

#include "pipelines/g_internal_shaders.h"

};

namespace GpuRt
{

//=====================================================================================================================
// Enumeration for supported intrinsic functions in GPURT
enum class RtIntrinsicFunction : uint32
{
    RayQueryProceed,
    TraceRayInline,
    TraceRay,
    TraceRayUsingHitToken,
    TraceRayUsingRayQuery,
    GetInstanceID,
    GetInstanceIndex,
    GetObjectToWorldTransform,
    GetWorldToObjectTransform,
    _Count
};

//=====================================================================================================================
// Function table for ray tracing IP1.1
constexpr const char* FunctionTableRTIP1_1[] =
{
    "\01?RayQueryProceed1_1@@YA_NURayQueryInternal@@IV?$vector@I$02@@@Z",
    "\01?TraceRayInline1_1@@YAXURayQueryInternal@@IIIIIURayDesc@@V?$vector@I$02@@@Z",
    "\01?TraceRay1_1@@YAXIIIIIIIMMMMMMMM@Z",
    "\01?TraceRayUsingHitToken1_1@@YAXIIIIIIIMMMMMMMMII@Z",
    "\01?TraceRayUsingRayQuery1_1@@YAXIIIIIIIMMMMMMMM@Z",
    "\01?GetInstanceID@@YAI_K@Z",
    "\01?GetInstanceIndex@@YAI_K@Z",
    "\01?GetObjectToWorldTransform@@YAM_KII@Z",
    "\01?GetWorldToObjectTransform@@YAM_KII@Z",
};

// =====================================================================================================================
Device::Device()
    :
    m_pipelineMap(64, &m_allocator),
    m_tlasCaptureList(this),
    m_isTraceActive(false),
    m_accelStructTraceSource(this)
{
}

// =====================================================================================================================
Device::~Device()
{
    for (InternalPipelineMap::Iterator itr = m_pipelineMap.Begin(); itr.Get(); itr.Next())
    {
        ClientDestroyInternalComputePipeline(m_info, itr.Get()->value.pPipeline, itr.Get()->value.pMemory);
    }
}

// =====================================================================================================================
// Initialize the GPURT device.
Pal::Result Device::Init(
    const DeviceInitInfo& info)
{
    Pal::Result result = Pal::Result::Success;
    m_info = info;
    result = m_pipelineMap.Init();
    *m_info.pAccelStructTracker = {};

#if PAL_BUILD_RDF
    Pal::IPlatform* pPlatform = m_info.pPalPlatform;
    GpuUtil::TraceSession* pTraceSession = pPlatform->GetTraceSession();
    pTraceSession->RegisterSource(&m_accelStructTraceSource);
#endif

    return result;
}

//=====================================================================================================================
// Maps Pal::RayTracingIpLevel to the appropriate function table.
Pal::Result Device::QueryRayTracingEntryFunctionTable(
    const Pal::RayTracingIpLevel   rayTracingIpLevel,
    EntryFunctionTable* const      pEntryFunctionTable)
{
    Pal::Result result = Pal::Result::Success;

    const char* const* ppFuncTable = nullptr;
    switch (rayTracingIpLevel)
    {
        case Pal::RayTracingIpLevel::RtIp1_0:
        case Pal::RayTracingIpLevel::RtIp1_1:
            ppFuncTable = FunctionTableRTIP1_1;
            break;
        case Pal::RayTracingIpLevel::None:
        default:
            result = Pal::Result::ErrorInvalidValue;
            PAL_ASSERT_ALWAYS();
            break;
    }

    if (ppFuncTable)
    {
        pEntryFunctionTable->traceRay.pTraceRay =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::TraceRay)];
        pEntryFunctionTable->traceRay.pTraceRayUsingHitToken =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::TraceRayUsingHitToken)];
        pEntryFunctionTable->traceRay.pTraceRayUsingRayQuery =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::TraceRayUsingRayQuery)];

        pEntryFunctionTable->rayQuery.pTraceRayInline =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::TraceRayInline)];
        pEntryFunctionTable->rayQuery.pProceed =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::RayQueryProceed)];

        pEntryFunctionTable->intrinsic.pGetInstanceID =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::GetInstanceID)];
        pEntryFunctionTable->intrinsic.pGetInstanceIndex =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::GetInstanceIndex)];
        pEntryFunctionTable->intrinsic.pGetObjectToWorldTransform =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::GetObjectToWorldTransform)];
        pEntryFunctionTable->intrinsic.pGetWorldToObjectTransform =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::GetWorldToObjectTransform)];

    }

    return result;
}

// =====================================================================================================================
// Allocate system memory
void* Device::Alloc(
    const Util::AllocInfo& allocInfo)
{
    return m_allocator.Alloc(allocInfo);
}

// =====================================================================================================================
// Free system memory
void Device::Free(
    const Util::FreeInfo& freeInfo)
{
    m_allocator.Free(freeInfo);
}

// =====================================================================================================================
PipelineShaderCode Device::GetShaderLibraryData(
    ShaderLibraryFeatureFlags flags)
{
    const bool enableDevFeatures =
        Util::TestAnyFlagSet(flags, static_cast<uint32>(ShaderLibraryFeatureFlag::Developer));

    const bool enableSwTraversal =
        Util::TestAnyFlagSet(flags, static_cast<uint32>(ShaderLibraryFeatureFlag::SoftwareTraversal));

    PipelineShaderCode code = {};

#define CHOOSE_SHADER(x) { code.pDxilCode = (x); code.dxilSize = sizeof(x); }

    if (enableSwTraversal)
    {
        if (enableDevFeatures)
        {
            CHOOSE_SHADER(CsGpuRtLibrarySwDev);
        }
        else
        {
            CHOOSE_SHADER(CsGpuRtLibrarySw);
        }
    }
    else
    {
        if (enableDevFeatures)
        {
            CHOOSE_SHADER(CsGpuRtLibraryDev);
        }
        else
        {
            CHOOSE_SHADER(CsGpuRtLibrary);
        }
    }
#undef CHOOSE_SHADER

    return code;
}

//=====================================================================================================================
uint32 Device::GetStaticPipelineFlags(
    bool  skipTriangles,
    bool  skipProceduralPrims,
    bool  useRayQueryForTraceRays,
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 27
    bool  unused,
#endif
    bool  enableAccelStructTracking,
    bool  enableTraversalCounter)
{
    uint32 pipelineFlags = 0;

    if (skipTriangles)
    {
        pipelineFlags |= static_cast<uint32>(GpuRt::StaticPipelineFlag::SkipTriangles);
    }

    if (skipProceduralPrims)
    {
        pipelineFlags |= static_cast<uint32>(GpuRt::StaticPipelineFlag::SkipProceduralPrims);
    }

    if (m_info.deviceSettings.bvhCollapse)
    {
        pipelineFlags |= static_cast<uint32>(GpuRt::StaticPipelineFlag::BvhCollapse);
    }

    if (useRayQueryForTraceRays)
    {
        pipelineFlags |= static_cast<uint32>(GpuRt::StaticPipelineFlag::UseRayQuery);
    }

    if (m_info.deviceSettings.rebraidType != RebraidType::Off)
    {
        pipelineFlags |= static_cast<uint32>(GpuRt::StaticPipelineFlag::UseTreeRebraid);
    }

    if (enableAccelStructTracking)
    {
        pipelineFlags |= static_cast<uint32>(GpuRt::StaticPipelineFlag::EnableAccelStructTracking);
    }

    if (enableTraversalCounter)
    {
        pipelineFlags |= static_cast<uint32>(GpuRt::StaticPipelineFlag::EnableTraversalCounter);
    }

    return pipelineFlags;
}

// =====================================================================================================================
// Returns a specific internal pipeline or initialize specified internal pipeline
Pal::IPipeline* Device::GetInternalPipeline(
    InternalRayTracingCsType        shaderType,
    const CompileTimeBuildSettings& buildSettings,
    uint32                          buildSettingsHash
    ) const
{
    Util::RWLock* pPipelineLock = const_cast<Util::RWLock*>(&m_internalPipelineLock);

    // Only BuildParallel is using compile time constants for now. Avoid creating unnecessary variations of other
    // pipelines when the build settings change.
    const bool enableCompileTimeSettings = (shaderType == InternalRayTracingCsType::BuildParallel);

    InternalPipelineKey key = {};
    key.shaderType   = shaderType;
    key.settingsHash = enableCompileTimeSettings ? buildSettingsHash : 0;

    InternalPipelineMemoryPair* pPipelinePair = nullptr;

    {
        Util::RWLockAuto<Util::RWLock::LockType::ReadOnly> lock(pPipelineLock);
        pPipelinePair = m_pipelineMap.FindKey(key);
    }

    if (pPipelinePair == nullptr)
    {
        Util::RWLockAuto<Util::RWLock::LockType::ReadWrite> lock(pPipelineLock);

        bool existed = false;
        Pal::Result result = const_cast<InternalPipelineMap&>(m_pipelineMap).FindAllocate(key, &existed, &pPipelinePair);

        if ((existed == false) && (result == Pal::Result::Success) && (pPipelinePair != nullptr))
        {
            const PipelineBuildInfo* pPipelineBuildInfo = &InternalPipelineBuildInfo[static_cast<uint32>(shaderType)];

            NodeMapping nodes[MaxInternalPipelineNodes];

            uint32 nodeOffset = 0;
            uint32 uavCount   = 0;
            uint32 cbvCount   = 0;

            for (uint32 nodeIndex = 0; nodeIndex < pPipelineBuildInfo->nodeCount; ++nodeIndex)
            {
                // Make sure we haven't exceeded our maximum number of nodes.
                PAL_ASSERT(nodeIndex < MaxInternalPipelineNodes);

                nodes[nodeIndex] = pPipelineBuildInfo->pNodes[nodeIndex];

                // These must be defined:
                const NodeType nodeType = pPipelineBuildInfo->pNodes[nodeIndex].type;
                const uint32 nodeSize = nodes[nodeIndex].dwSize;

                PAL_ASSERT(nodeSize > 0);

                // These are calculated dynamically below into a tightly-packed top-level resource representation
                PAL_ASSERT(nodes[nodeIndex].dwOffset == 0);
                PAL_ASSERT(nodes[nodeIndex].srdStartIndex == 0);
                PAL_ASSERT(nodes[nodeIndex].srdStride == 0);
                PAL_ASSERT(nodes[nodeIndex].logicalId == 0);

                nodes[nodeIndex].dwOffset  = nodeOffset;
                nodes[nodeIndex].srdStride = nodeSize;

                // Descriptor sets are assigned as follows:
                // 0  Root UAVs
                // 1  Root constants and CBVs
                // 2+ Desciptor tables (UAV or CBV)
                uint32 tableSet = 2;

                switch (nodeType)
                {
                case NodeType::Constant:
                case NodeType::ConstantBuffer:
                    nodes[nodeIndex].logicalId     = cbvCount + ReservedLogicalIdCount;
                    nodes[nodeIndex].srdStartIndex = cbvCount;
                    nodes[nodeIndex].binding       = cbvCount;
                    nodes[nodeIndex].descSet       = 1;
                    cbvCount++;
                    break;
                case NodeType::ConstantBufferTable:
                    nodes[nodeIndex].logicalId     = cbvCount + ReservedLogicalIdCount;
                    nodes[nodeIndex].srdStartIndex = 0;
                    nodes[nodeIndex].binding       = 0;
                    nodes[nodeIndex].descSet       = tableSet++;
                    cbvCount++;
                    break;
                case NodeType::Uav:
                    nodes[nodeIndex].logicalId     = uavCount + ReservedLogicalIdCount;
                    nodes[nodeIndex].srdStartIndex = uavCount;
                    nodes[nodeIndex].binding       = uavCount;
                    nodes[nodeIndex].descSet       = 0;
                    uavCount++;
                    break;
                case NodeType::UavTable:
                case NodeType::TypedUavTable:
                    nodes[nodeIndex].logicalId     = uavCount + ReservedLogicalIdCount;
                    nodes[nodeIndex].srdStartIndex = 0;
                    nodes[nodeIndex].binding       = 0;
                    nodes[nodeIndex].descSet       = tableSet++;
                    uavCount++;
                default:
                    PAL_ASSERT_ALWAYS();
                }

                nodeOffset += nodeSize;
            }

            PipelineBuildInfo buildInfo = *pPipelineBuildInfo;

            buildInfo.pNodes = nodes;
            buildInfo.apiPsoHash = GetInternalPsoHash(buildInfo.shaderType, buildSettings);

            CompileTimeConstants compileConstants = {};

#if GPURT_DEVELOPER
            constexpr uint32 MaxStrLength = 256;
            char pipelineName[MaxStrLength];
#endif

            if (enableCompileTimeSettings)
            {
                const uint32 lastNodeIndex = pPipelineBuildInfo->nodeCount;

                nodes[lastNodeIndex].type          = NodeType::ConstantBuffer;
                nodes[lastNodeIndex].dwSize        = 2;
                nodes[lastNodeIndex].dwOffset      = nodeOffset;
                nodes[lastNodeIndex].logicalId     = cbvCount + ReservedLogicalIdCount;
                nodes[lastNodeIndex].srdStartIndex = cbvCount;
                nodes[lastNodeIndex].srdStride     = nodes[lastNodeIndex].dwSize;

                // Set binding and descSet to irrelevant value to avoid messing up the resource mapping for Vulkan.
                nodes[lastNodeIndex].binding       = ~0u;
                nodes[lastNodeIndex].descSet       = ~0u;

                buildInfo.nodeCount++;

                compileConstants.pConstants          = reinterpret_cast<const uint32*>(&buildSettings);
                compileConstants.numConstants        = sizeof(CompileTimeBuildSettings) / sizeof(uint32);
                compileConstants.logicalId           = nodes[lastNodeIndex].logicalId;
                compileConstants.constantBufferIndex = nodes[lastNodeIndex].srdStartIndex;

#if GPURT_DEVELOPER
                // Append appropriate strings based on build settings
                if (buildInfo.pPipelineName != nullptr)
                {
                    constexpr const char* BuildModeStr[] =
                    {
                        "LBVH",     // BvhBuildMode::Linear,
                        "Reserved", // BvhBuildMode::Reserved,
                        "PLOC",     // BvhBuildMode::PLOC,
                        "Reserved",
                        "Auto",     // BvhBuildMode::Auto,
                    };

                    constexpr const char* RebraidTypeStr[] =
                    {
                        "",           // GpuRt::RebraidType::Off,
                        "_RebraidV1", // GpuRt::RebraidType::V1,
                        "_RebraidV2", // GpuRt::RebraidType::V2,
                    };

                    Util::Snprintf(pipelineName, MaxStrLength, "%s%s_%s%s%s%s_RadixSortLevel_%d",
                                   buildInfo.pPipelineName,
                                   buildSettings.topLevelBuild ? "_TLAS" : "_BLAS",
                                   buildSettings.enableTopDownBuild ? "TopDown" : BuildModeStr[buildSettings.buildMode],
                                   buildSettings.doTriangleSplitting ? "_TriSplit" : "",
                                   buildSettings.triangleCompressionMode ? "_TriCompr" : "",
                                   RebraidTypeStr[buildSettings.rebraidType],
                                   buildSettings.radixSortScanLevel);

                    buildInfo.pPipelineName = &pipelineName[0];
                }
#endif
            }

            result = ClientCreateInternalComputePipeline(
                m_info,
                buildInfo,
                compileConstants,
                &pPipelinePair->pPipeline,
                &pPipelinePair->pMemory);

            PAL_ASSERT(result == Pal::Result::Success);
        }
    }

    return (pPipelinePair != nullptr) ? pPipelinePair->pPipeline : nullptr;
}

// =====================================================================================================================
// Calculates the total number of AABBs required
uint32 CalculateRayTracingAABBCount(
    const AccelStructBuildInputs& buildInfo)
{
    uint32 aabbCount = 0;

    if (buildInfo.type == AccelStructType::BottomLevel)
    {
        // Bottom level
        for (uint32 i = 0; i < buildInfo.inputElemCount; ++i)
        {
            const Geometry geometry = ClientConvertAccelStructBuildGeometry(buildInfo, i);

            if (geometry.type == GeometryType::Triangles)
            {
                // There is one axis aligned bounding box per face.
                if (geometry.triangles.indexFormat != IndexFormat::Unknown)
                {
                    // Indexed triangle vertices
                    PAL_ASSERT((geometry.triangles.indexCount % 3) == 0);
                    aabbCount += (geometry.triangles.indexCount / 3);
                }
                else
                {
                    // Auto-indexed triangle vertices
                    PAL_ASSERT((geometry.triangles.vertexCount % 3) == 0);
                    aabbCount += (geometry.triangles.vertexCount / 3);
                }
            }
            else
            {
                // Procedural AABB geometry does not require any additional data. Note that AABBCount is 64-bit; in
                // practice we can't support more than 4 billion AABBs so for now just assert that it's a 32-bit value.
                PAL_ASSERT(Util::HighPart(geometry.aabbs.aabbCount) == 0);
                aabbCount += static_cast<uint32>(geometry.aabbs.aabbCount);
            }
        }
    }
    else
    {
        // Top level

        // One AABB per instance
        aabbCount = buildInfo.inputElemCount;
    }
    return aabbCount;
}

// =====================================================================================================================
// Calculates the result buffer offsets and returns the total result memory size
uint32 BvhBuilder::CalculateResultBufferInfo(
    AccelStructDataOffsets* pOffsets,
    uint32*                 pMetadataSizeInBytes)
{
    uint32 runningOffset = 0;

    //-----------------------------------------------------------------------------------------------------------//
    //  DestAccelerationStructureData layout
    //
    //-------------- Type: All ----------------------------------------------------------------------------------//
    //  AccelStructMetadataHeader
    //  Parent pointers
    //  AccelStructHeader
    //  Internal Nodes                              : BVHNode (BVH2) or Float32BoxNode (BVH4)
    //  Leaf Nodes                                  : BVHNode (BVH2), TriangleNode (BVH4) or InstanceNode (TopLevel)
    //  Per-geometry description info               : Bottom Level Only
    //  Per-leaf node pointers (uint32)
    //-----------------------------------------------------------------------------------------------------------//
    AccelStructDataOffsets offsets = {};

    // Acceleration structure data starts with the header
    runningOffset += sizeof(AccelStructHeader);

    uint32 internalNodeSize = 0;
    uint32 leafNodeSize     = 0;

    if (m_buildConfig.numLeafNodes > 0)
    {
        internalNodeSize = BvhBuilder::CalculateInternalNodesSize();
        leafNodeSize     = BvhBuilder::CalculateLeafNodesSize();

        offsets.internalNodes = runningOffset;
        runningOffset += internalNodeSize;

        offsets.leafNodes = runningOffset;
        runningOffset += leafNodeSize;

        if (m_buildConfig.topLevelBuild == false)
        {
            offsets.geometryInfo = runningOffset;
            runningOffset += BvhBuilder::CalculateGeometryInfoSize(m_buildArgs.inputs.inputElemCount);
        }

        offsets.primNodePtrs = runningOffset;
        runningOffset += m_buildConfig.numLeafNodes * sizeof(uint32);
    }

    // Metadata section is at the beginning of the acceleration structure buffer
    uint32 metadataSizeInBytes = CalcMetadataSizeInBytes(internalNodeSize,
                                                         leafNodeSize);

    // Align metadata size to cache line
    metadataSizeInBytes = Util::Pow2Align(metadataSizeInBytes, 128);

    // Add in metadata size at the end of the calculation, as we do not want to include it in any of the offsets.
    runningOffset += metadataSizeInBytes;

    if (pOffsets != nullptr)
    {
        memcpy(pOffsets, &offsets, sizeof(offsets));
    }

    if (pMetadataSizeInBytes != nullptr)
    {
        *pMetadataSizeInBytes = metadataSizeInBytes;
    }

    return runningOffset;
}

// =====================================================================================================================
// Calculates the scratch buffer offsets and returns the total scratch memory size
uint32 GpuBvhBuilder::CalculateScratchBufferInfo(
    RayTracingScratchDataOffsets* pOffsets)
{
    //-----------------------------------------------------------------------------------------------------------//
    //  ScratchAccelerationStructureData layout
    //
    //-------------- Type: All ----------------------------------------------------------------------------------//
    //  TaskQueue Counters (phase, taskCounter, startIndex, endIndex, numTaskDone)
    //  AABB               (ScratchNode InternalAABB[node_count = (2 * aabbCount) - 1])...AABB + Sorted Leaf
    //  PropagationFlags   (uint32_t PropagationFlags[NumPrimitives])
    //  TriangleSplitBox   (BoundingBox TriangleSplitBoxes[NumPrimitives])                     - Bottom Level Only

    //  ============ PASS 1 ============
    //  AABB              (ScratchNode LeafAABB[NumPrimitives])
    //  TriangleSplitRef  (ScratchTSRef TriangleSplitRefs[NumPrimitives])                     - Bottom Level Only
    //  SceneAABB         (D3D12_RAYTRACING_AABB SceneAABB)
    //  MortonCodes       (uint32/uint64 MortonCodes[NumPrimitives])
    //  MortonCodesSorted (uint32/uint64 MortonCodesSorted[NumPrimitives])
    //  PrimIndicesSorted (uint32 PrimIndicesSorted[NumPrimitives])
    //  DeviceHistogram
    //  TempKeys          (uint32/uint64 TempKeys[NumPrimitives])
    //  TempVals          (uint32 TempVals[NumPrimitives])
    //  DevicePartialSum

    //  ============ PASS 2 ============
    //  ClusterList0      (uint32 ClusterList0[NumPrimitives])                                - BVH AC only
    //  ClusterList1      (uint32 ClusterList1[NumPrimitives])                                - BVH AC only
    //  NumClusterList0   (uint32 NumClusterList0)                                            - BVH AC only
    //  NumClusterList1   (uint32 NumClusterList1)                                            - BVH AC only
    //  InternalNodesIndex0   (uint32 NumClusterList0)                                        - BVH AC only
    //  InternalNodesIndex1   (uint32 NumClusterList1)                                        - BVH AC only

    //  ============ PASS 3 ============
    //  QBVH Global Stack (uint2 GlobalStack[internal_node_count])
    //  QBVH Stack Ptrs   (StackPtrs StackPtrs)
    //  BVH2Prims         (BVHNode bvh2prims[NumPrimitives])                                    -Bottom Level Only
    //  Collapse Stack    (CTask CollapseStack[node_count])                                     -Bottom Level Only
    //  Collapse Stack Ptrs (StackPtrs StackPtrs)                                               -Bottom Level Only
    //-----------------------------------------------------------------------------------------------------------//

    uint32 runningOffset = 0;

    // Scratch data for storing internal BVH node data
    const uint32 bvhNodeData = runningOffset;
    const uint32 aabbCount = m_buildConfig.numLeafNodes;

    // The scratch acceleration structure is built as a BVH2.
    const uint32 nodeCount = (aabbCount > 0) ? ((2 * aabbCount) - 1) : 0;
    runningOffset += nodeCount * RayTracingScratchNodeSize;

    // Propagation flags
    const uint32 propagationFlags = runningOffset;
    runningOffset += aabbCount * sizeof(uint32);

    uint32 triangleSplitState = 0xFFFFFFFF;
    uint32 triangleSplitBoxes = 0xFFFFFFFF;

    if (m_buildConfig.triangleSplitting)
    {
        triangleSplitState = runningOffset;
        runningOffset += RayTracingStateTSBuildSize;

        triangleSplitBoxes = runningOffset;
        runningOffset += aabbCount * sizeof(Aabb);
    }

    uint32 currentState = 0xFFFFFFFF;

    if ((m_buildConfig.topDownBuild == false) && (m_buildConfig.buildMode == BvhBuildMode::PLOC))
    {
        currentState = runningOffset;
        runningOffset += RayTracingTaskQueueCounterSize;    // PLOC task counters
        runningOffset += RayTracingStatePLOCSize;           // PLOC state
    }

    uint32 tdState = 0xFFFFFFFF;
    uint32 tdTaskQueueCounter = 0xFFFFFFFF;

    if (m_buildConfig.topDownBuild)
    {
        tdState = runningOffset;

        if (m_buildConfig.rebraidType == GpuRt::RebraidType::V1)
        {
            runningOffset += RayTracingStateTDTRBuildSize;
        }
        else
        {
            runningOffset += RayTracingStateTDBuildSize;
        }

        tdTaskQueueCounter = runningOffset;               // td /tdtr taskCounter
        runningOffset += RayTracingTaskQueueCounterSize;
    }

    const uint32 dynamicBlockIndex = runningOffset;
    runningOffset += sizeof(uint32);

    uint32 numBatches      = 0xFFFFFFFF;
    uint32 batchIndices    = 0xFFFFFFFF;
    uint32 indexBufferInfo = 0xFFFFFFFF;

    if (m_buildConfig.triangleCompressionMode == TriangleCompressionMode::Pair)
    {
        numBatches = runningOffset;
        runningOffset += sizeof(uint32);

        batchIndices = runningOffset;
        runningOffset += aabbCount * sizeof(uint32);

        indexBufferInfo = runningOffset;
        runningOffset += BvhBuilder::CalculateIndexBufferInfoSize(m_buildArgs.inputs.inputElemCount);
    }

    uint32 debugCounters = 0;

    if (m_deviceSettings.enableBVHBuildDebugCounters)
    {
        // Adding a Build debug counter
        // Allocate memory for counters
        debugCounters = runningOffset;
        runningOffset += sizeof(uint32) * RayTracingBuildDebugCounters;
    }

    uint32 rebraidState = 0xFFFFFFFF;

    if (m_buildConfig.rebraidType == GpuRt::RebraidType::V2)
    {
        rebraidState = runningOffset;
        runningOffset += RayTracingStateRebraidBuildSize;
    }

    uint32 maxSize = runningOffset;

    const uint32 passOffset = runningOffset;

    // ============ PASS 1 ============

    runningOffset = passOffset;

    uint32 bvhLeafNodeData   = 0xFFFFFFFF;
    uint32 sceneBounds       = 0xFFFFFFFF;
    uint32 mortonCodes       = 0xFFFFFFFF;
    uint32 mortonCodesSorted = 0xFFFFFFFF;
    uint32 primIndicesSorted = 0xFFFFFFFF;
    uint32 primIndicesSortedSwap = 0xFFFFFFFF;
    uint32 histogram         = 0xFFFFFFFF;
    uint32 tempKeys          = 0xFFFFFFFF;
    uint32 tempVals          = 0xFFFFFFFF;

    uint32 atomicFlags         = 0xFFFFFFFF;
    uint32 distributedPartSums = 0xFFFFFFFF;

    uint32 refList             = 0xFFFFFFFF;
    uint32 tdNodeList          = 0xFFFFFFFF;
    uint32 refOffsets          = 0xFFFFFFFF;
    uint32 tdBins              = 0xFFFFFFFF;

    // Scratch data for storing unsorted leaf nodes
    bvhLeafNodeData = runningOffset;
    runningOffset += aabbCount * RayTracingScratchNodeSize;

    uint32 triangleSplitRefs0 = 0xFFFFFFFF;
    uint32 triangleSplitRefs1 = 0xFFFFFFFF;
    uint32 splitPriorities    = 0xFFFFFFFF;
    uint32 atomicFlagsTS      = 0xFFFFFFFF;

    if (m_buildConfig.triangleSplitting)
    {
        triangleSplitRefs0 = runningOffset;
        runningOffset += aabbCount * RayTracingTSRefScratchSize;

        triangleSplitRefs1 = runningOffset;
        runningOffset += aabbCount * RayTracingTSRefScratchSize;

        splitPriorities = runningOffset;
        runningOffset += aabbCount * sizeof(float);

        atomicFlagsTS = runningOffset;  // TODO: calculate number of blocks based on KEYS_PER_THREAD
        runningOffset += aabbCount * RayTracingAtomicFlags;
    }
    else if (m_buildConfig.rebraidType == GpuRt::RebraidType::V2)
    {
        atomicFlagsTS = runningOffset;  // TODO: calculate number of blocks based on KEYS_PER_THREAD
        runningOffset += aabbCount * RayTracingAtomicFlags;
    }

    sceneBounds = runningOffset;
    runningOffset += sizeof(Aabb) + 2 * sizeof(float);  // scene bounding box + min/max prim size

    if (m_buildConfig.topLevelBuild == true)
    {
        runningOffset += sizeof(Aabb);  // scene bounding box for rebraid
    }

    if ((m_buildConfig.topLevelBuild == false) || (m_buildConfig.topDownBuild == false))
    {
        const uint32 dataSize = m_deviceSettings.enableMortonCode30 ? sizeof(uint32) : sizeof(uint64);

        // Morton codes buffer size
        mortonCodes = runningOffset;
        runningOffset += aabbCount * dataSize;

        // Sorted morton codes buffer size
        mortonCodesSorted = runningOffset;
        runningOffset += aabbCount * dataSize;

        // Sorted primitive indices buffer size
        primIndicesSorted = runningOffset;
        runningOffset += aabbCount * sizeof(uint32);

        // Merge Sort
        if (m_deviceSettings.enableMergeSort)
        {
            primIndicesSortedSwap = runningOffset;
            runningOffset += aabbCount * sizeof(uint32);
        }
        // Radix Sort
        else
        {
            // Radix sort temporary buffers
            const uint32 numBlocks = (aabbCount + m_radixSortConfig.groupBlockSize - 1) /
                                     m_radixSortConfig.groupBlockSize;

            // device histograms buffer (int4)
            histogram = runningOffset;
            const uint32 numHistogramElements = numBlocks * m_radixSortConfig.numBins;
            runningOffset += numHistogramElements * sizeof(uint32);

            // device temp keys buffer (int)
            tempKeys = runningOffset;
            runningOffset += aabbCount * dataSize;

            // device temp vals buffer (int)
            tempVals = runningOffset;
            runningOffset += aabbCount * sizeof(uint32);

            if (m_buildConfig.radixSortScanLevel == 0)
            {
                const uint32 blockSize        = m_radixSortConfig.workGroupSize;
                const uint32 numKeysPerThread = m_radixSortConfig.keysPerThread;
                const uint32 numDynamicBlocks = (numHistogramElements +
                    ((blockSize * numKeysPerThread) - 1)) / (blockSize * numKeysPerThread);

                atomicFlags = runningOffset;
                runningOffset += numDynamicBlocks * RayTracingScanDLBFlagsSize;
            }
            else
            {
                // partial sum scratch memory
                const uint32 numGroupsBottomLevelScan =
                    Util::RoundUpQuotient(m_buildConfig.numHistogramElements, m_radixSortConfig.groupBlockSizeScan);

                distributedPartSums = runningOffset;
                runningOffset += numGroupsBottomLevelScan * sizeof(uint32);

                if (m_buildConfig.numHistogramElements >= m_radixSortConfig.scanThresholdTwoLevel)
                {
                    const uint32 numGroupsMidLevelScan =
                        Util::RoundUpQuotient(numGroupsBottomLevelScan, m_radixSortConfig.groupBlockSizeScan);

                    runningOffset += numGroupsMidLevelScan * sizeof(uint32);
                }
            }
        }
    }
    else
    {
        refList = runningOffset;

        if (m_buildConfig.rebraidType == GpuRt::RebraidType::V1)
        {
            runningOffset += RayTracingTDTRRefScratchSize * aabbCount;
        }
        else
        {
            runningOffset += RayTracingTDRefScratchSize * aabbCount;
        }

        // Align the beginning of the TDBins structs to 8 bytes so that 64-bit atomic operations on the first field in
        // the struct work correctly.
        runningOffset = Util::RoundUpToMultiple(runningOffset, 8u);

        tdBins = runningOffset;

        runningOffset += RayTracingTDBinsSize * (aabbCount / 3);

        tdNodeList = runningOffset;

        if (m_buildConfig.rebraidType == GpuRt::RebraidType::V1)
        {
            runningOffset += RayTracingTDTRNodeSize * (aabbCount - 1);
        }
        else
        {
            runningOffset += RayTracingTDNodeSize * (aabbCount - 1);
        }

    }

    maxSize = Util::Max(maxSize, runningOffset);

    // ============ PASS 2 ============

    runningOffset = passOffset;

    uint32 clustersList0        = 0xFFFFFFFF;
    uint32 clustersList1        = 0xFFFFFFFF;
    uint32 numClusterList0      = 0xFFFFFFFF;
    uint32 numClusterList1      = 0xFFFFFFFF;
    uint32 internalNodesIndex0  = 0xFFFFFFFF;
    uint32 internalNodesIndex1  = 0xFFFFFFFF;
    uint32 neighbourIndices     = 0xFFFFFFFF;
    uint32 atomicFlagsPloc      = 0xFFFFFFFF;
    uint32 clusterOffsets       = 0xFFFFFFFF;

    if (m_buildConfig.topDownBuild == false)
    {
        if (m_buildConfig.buildMode == BvhBuildMode::PLOC)
        {
            clustersList0 = runningOffset;
            runningOffset += aabbCount * sizeof(uint32);

            clustersList1 = runningOffset;
            runningOffset += aabbCount * sizeof(uint32);

            neighbourIndices = runningOffset;
            runningOffset += aabbCount * sizeof(uint32);

            atomicFlagsPloc = runningOffset; // TODO: calculate number of blocks based on KEYS_PER_THREAD
            runningOffset += aabbCount * RayTracingPLOCFlags;

            clusterOffsets = runningOffset;
            runningOffset += aabbCount * sizeof(uint32);
        }
    }

    maxSize = Util::Max(maxSize, runningOffset);

    // ============ PASS 3 ============

    runningOffset = passOffset;

    uint32 qbvhGlobalStack = 0;
    uint32 qbvhGlobalStackPtrs = 0;

    uint32 collapseBVHStack = 0;
    uint32 collapseBVHStackPtrs = 0;
    uint32 bvh2Prims = 0;

    // ..and QBVH global stack
    qbvhGlobalStack = runningOffset;

    const uint32 maxStackEntry = CalcNumQBVHInternalNodes(m_buildConfig.numLeafNodes);

    if ((m_buildConfig.topLevelBuild == false) && m_buildConfig.collapse)
    {
        runningOffset += maxStackEntry * RayTracingQBVHCollapseTaskSize;
    }
    else
    {
        // Stack pointers require 2 entries per node when fp16 and fp32 box nodes intermix in BLAS
        const Fp16BoxNodesInBlasMode intNodeTypes = m_buildConfig.fp16BoxNodesInBlasMode;
        const bool intNodeTypesMix = (intNodeTypes != Fp16BoxNodesInBlasMode::NoNodes) &&
                                     (intNodeTypes != Fp16BoxNodesInBlasMode::AllNodes);
        const bool intNodeTypesMixInBlas = intNodeTypesMix &&
                                           (m_buildArgs.inputs.type == AccelStructType::BottomLevel);
        const uint32 stackEntrySize      = ((intNodeTypesMixInBlas ||
                                             m_deviceSettings.enableHalfBoxNode32) ? 2u : 1u);

        runningOffset += maxStackEntry * stackEntrySize * sizeof(uint32);
    }

    qbvhGlobalStackPtrs = runningOffset;

    runningOffset += RayTracingQBVHStackPtrsSize;

    maxSize = Util::Max(maxSize, runningOffset);

    // If the caller requested offsets, return them.
    if (pOffsets != nullptr)
    {
        pOffsets->bvhNodeData          = bvhNodeData;
        pOffsets->triangleSplitBoxes   = triangleSplitBoxes;
        pOffsets->triangleSplitRefs0   = triangleSplitRefs0;
        pOffsets->triangleSplitRefs1   = triangleSplitRefs1;
        pOffsets->splitPriorities      = splitPriorities;
        pOffsets->triangleSplitState   = triangleSplitState;
        pOffsets->rebraidState         = rebraidState;
        pOffsets->atomicFlagsTS        = atomicFlagsTS;
        pOffsets->refList              = refList;
        pOffsets->tdNodeList           = tdNodeList;
        pOffsets->tdBins               = tdBins;
        pOffsets->tdState              = tdState;
        pOffsets->tdTaskQueueCounter   = tdTaskQueueCounter;
        pOffsets->refOffsets           = refOffsets;
        pOffsets->bvhLeafNodeData      = bvhLeafNodeData;
        pOffsets->clusterList0         = clustersList0;
        pOffsets->clusterList1         = clustersList1;
        pOffsets->numClusterList0      = numClusterList0;
        pOffsets->numClusterList1      = numClusterList1;
        pOffsets->internalNodesIndex0  = internalNodesIndex0;
        pOffsets->internalNodesIndex1  = internalNodesIndex1;
        pOffsets->neighbourIndices     = neighbourIndices;
        pOffsets->currentState         = currentState;
        pOffsets->atomicFlagsPloc      = atomicFlagsPloc;
        pOffsets->clusterOffsets       = clusterOffsets;
        pOffsets->sceneBounds          = sceneBounds;
        pOffsets->mortonCodes          = mortonCodes;
        pOffsets->mortonCodesSorted    = mortonCodesSorted;
        pOffsets->primIndicesSorted    = primIndicesSorted;
        pOffsets->primIndicesSortedSwap= primIndicesSortedSwap;
        pOffsets->propagationFlags     = propagationFlags;
        pOffsets->histogram            = histogram;
        pOffsets->tempKeys             = tempKeys;
        pOffsets->tempVals             = tempVals;
        pOffsets->dynamicBlockIndex    = dynamicBlockIndex;
        pOffsets->atomicFlags          = atomicFlags;
        pOffsets->distributedPartSums  = distributedPartSums;
        pOffsets->qbvhGlobalStack      = qbvhGlobalStack;
        pOffsets->qbvhGlobalStackPtrs  = qbvhGlobalStackPtrs;
        pOffsets->debugCounters        = debugCounters;
        pOffsets->numBatches           = numBatches;
        pOffsets->batchIndices         = batchIndices;
        pOffsets->indexBufferInfo      = indexBufferInfo;
    }

    // Return maxSize which now contains the total scratch size.
    return maxSize;
}

// =====================================================================================================================
// Calculates the update scratch buffer offsets and returns the total update scratch memory size
uint32 GpuBvhBuilder::CalculateUpdateScratchBufferInfo(
    RayTracingScratchDataOffsets* pOffsets)
{
    uint32 runningOffset = 0;
    //-----------------------------------------------------------------------------------------------------------//
    //  Update Scratch Data layout
    //
    //-------------- Type: All ----------------------------------------------------------------------------------//
    // PropagationFlags        (uint32 PropagationFlags[NumPrimitives])
    // UpdateStackPointer      uint32
    // UpdateStackElements     uint32[NumPrimitives]
    //-----------------------------------------------------------------------------------------------------------//
    RayTracingScratchDataOffsets offsets = {};

    // Allocate space for the node flags
    offsets.propagationFlags = runningOffset;
    runningOffset +=
        m_buildConfig.numLeafNodes * sizeof(uint32);

    // Allocate space for update stack
    offsets.updateStack = runningOffset;

    // Update stack pointer
    runningOffset += sizeof(uint32);

    // Update stack elements. Note, for a worst case tree, each leaf node enqueues a single parent
    // node pointer for updating
    runningOffset += m_buildConfig.numLeafNodes * sizeof(uint32);

    if (pOffsets != nullptr)
    {
        memcpy(pOffsets, &offsets, sizeof(offsets));
    }

    return runningOffset;
}

// =====================================================================================================================
// Check if need to force fp16 mode off
// Returns True if need to force the setting off, false otherwise.
static bool ForceDisableFp16BoxNodes(
    const AccelStructBuildInputs& buildArgs,
    const DeviceSettings&         deviceSettings)
{
    const bool allowUpdate = buildArgs.flags & AccelStructBuildFlag::AccelStructBuildFlagAllowUpdate;
    const bool allowCompaction = buildArgs.flags & AccelStructBuildFlag::AccelStructBuildFlagAllowCompaction;

    // Disable fp16 mode if
    // 1-) AllowUpdate set when not allowed by panel setting
    // 2-) TLAS
    // 3-) AllowCompaction not set when required by panel setting
    // Otherwise, respect the fp16 mode passed in from device settings
    const bool forceFp16BoxOff = (buildArgs.type == AccelStructType::TopLevel) ||
        ((deviceSettings.allowFp16BoxNodesInUpdatableBvh == false) && allowUpdate) ||
        (deviceSettings.fp16BoxNodesRequireCompaction && (allowCompaction == false));

    return forceFp16BoxOff;
}

// =====================================================================================================================
// Returns True if triangle compression should be enabled based on the current build flags and settings.
static bool AutoSelectTriangleCompressMode(
    const AccelStructBuildInputs& buildArgs,
    const DeviceSettings& deviceSettings)
{
    const bool allowCompaction = Util::TestAnyFlagSet(buildArgs.flags, AccelStructBuildFlagAllowCompaction);
    const bool isFastTrace = Util::TestAnyFlagSet(buildArgs.flags, AccelStructBuildFlagPreferFastTrace);
    const bool isFastBuild = Util::TestAnyFlagSet(buildArgs.flags, AccelStructBuildFlagPreferFastBuild);
    const bool isDefaultBuild = (isFastTrace == false) && (isFastBuild == false);

    bool enableTriCompression = false;

    switch (deviceSettings.triangleCompressionAutoMode)
    {
    case TriangleCompressionAutoMode::Disabled:
        break;
    case TriangleCompressionAutoMode::AlwaysEnabled:
        enableTriCompression = true;
        break;
    case TriangleCompressionAutoMode::DefaultBuild:
        enableTriCompression = isDefaultBuild || isFastTrace;
        break;
    case TriangleCompressionAutoMode::FastTrace:
        enableTriCompression = isFastTrace;
        break;
    case TriangleCompressionAutoMode::Compaction:
        enableTriCompression = allowCompaction;
        break;
    case TriangleCompressionAutoMode::DefaultBuildWithCompaction:
        enableTriCompression = (isDefaultBuild || isFastTrace) && allowCompaction;
        break;
    case TriangleCompressionAutoMode::FastTraceWithCompaction:
        enableTriCompression = isFastTrace && allowCompaction;
        break;
    case TriangleCompressionAutoMode::DefaultBuildOrCompaction:
        enableTriCompression = isDefaultBuild || isFastTrace || allowCompaction;
        break;
    case TriangleCompressionAutoMode::FastTraceOrCompaction:
        enableTriCompression = isFastTrace || allowCompaction;
        break;
    default:
        break;
    }

    return enableTriCompression == true;
}

// =====================================================================================================================
// If DeviceSettings has Auto Select Triangles or Updatable Fp16 Box Nodes enabled, test to select or turn off in build.
void BvhBuilder::UpdateBuildConfig()
{
    m_buildConfig.fp16BoxNodesInBlasMode = ForceDisableFp16BoxNodes(m_buildArgs.inputs, m_deviceSettings) ?
        Fp16BoxNodesInBlasMode::NoNodes : m_deviceSettings.fp16BoxNodesInBlasMode;

    m_buildConfig.triangleCompressionMode = AutoSelectTriangleCompressMode(m_buildArgs.inputs, m_deviceSettings) ?
        TriangleCompressionMode::Pair : TriangleCompressionMode::None;
}

// =====================================================================================================================
// Initialize buildConfig
void BvhBuilder::InitBuildConfig(
    const AccelStructBuildInfo& buildArgs) // Input build args
{
    m_buildConfig = {};
    uint32 primitiveCount = CalculateRayTracingAABBCount(buildArgs.inputs);

    if (Util::TestAnyFlagSet(buildArgs.inputs.flags, AccelStructBuildFlagPreferFastTrace))
    {
        m_buildConfig.buildMode = m_deviceSettings.bvhBuildModeFastTrace;
        m_buildConfig.cpuBuildMode = m_deviceSettings.bvhCpuBuildModeFastTrace;
    }
    else if (Util::TestAnyFlagSet(buildArgs.inputs.flags, AccelStructBuildFlagPreferFastBuild))
    {
        m_buildConfig.buildMode = m_deviceSettings.bvhBuildModeFastBuild;
        m_buildConfig.cpuBuildMode = m_deviceSettings.bvhCpuBuildModeFastBuild;
    }
    else
    {
        m_buildConfig.buildMode = m_deviceSettings.bvhBuildModeDefault;
        m_buildConfig.cpuBuildMode = m_deviceSettings.bvhCpuBuildModeDefault;
    }

    m_buildConfig.numPrimitives = primitiveCount;
    m_buildConfig.numLeafNodes = primitiveCount;
    m_buildConfig.rebraidType = m_deviceSettings.rebraidType;
    m_buildConfig.topLevelBuild = buildArgs.inputs.type == AccelStructType::TopLevel;
    m_buildConfig.geometryType = GetGeometryType(buildArgs.inputs);
    UpdateBuildConfig();
    m_buildConfig.triangleCompressionMode = (buildArgs.inputs.type == AccelStructType::BottomLevel &&
                                            m_buildConfig.geometryType == GeometryType::Triangles) ?
                                            m_buildConfig.triangleCompressionMode :
                                            TriangleCompressionMode::None;

    m_buildConfig.bvhBuilderNodeSortType = m_deviceSettings.bvhBuilderNodeSortType;
    m_buildConfig.bvhBuilderNodeSortHeuristic = m_deviceSettings.bvhBuilderNodeSortHeuristic;
}

// =====================================================================================================================
void Device::BeginBvhTrace()
{
    // Make sure trace isn't active already
    // We don't support multiple traces occurring at the same time.
    PAL_ASSERT(m_isTraceActive == false);

    m_isTraceActive = true;

    if ((m_info.pAccelStructTracker != nullptr) && (m_info.accelStructTrackerGpuAddr == 0))
    {
        // Clear the tracker and enable it
        m_info.pAccelStructTracker->enabled = true;
    }
}

// =====================================================================================================================
void Device::EndBvhTrace()
{
    m_isTraceActive = false;
}

// =====================================================================================================================
void Device::WriteCapturedBvh(
    GpuUtil::ITraceSource* pTraceSource)
{
    Pal::IDevice* palDevice = m_info.pPalDevice;
    Pal::IPlatform* pPlatform = m_info.pPalPlatform;

#if PAL_BUILD_RDF
    GpuUtil::TraceSession* pTraceSession = pPlatform->GetTraceSession();
#endif
    // Get post-build information for top-level acceleration structures
    uint64 sizeInBytes = 0;

    uint32 tlasCount = 0;
    gpusize* pTlasAddrs = nullptr;

    if (m_info.pAccelStructTracker != nullptr)
    {
        tlasCount  = m_info.pAccelStructTracker->count;
        pTlasAddrs = m_info.pAccelStructTracker->accelStructAddrs;
    }
    else
    {
        tlasCount  = m_tlasCaptureList.NumElements();
        pTlasAddrs = m_tlasCaptureList.Data();
    }

    Pal::Result result = Pal::Result::Success;

    if (tlasCount > 0)
    {
        result = GetAccelStructPostBuildSize(GpuRt::AccelStructPostBuildInfoType::Serialization,
                                             pTlasAddrs,
                                             tlasCount,
                                             &sizeInBytes);

        Pal::gpusize destGpuVa = 0;
        ClientGpuMemHandle gpuMem = nullptr;
        void* pMappedData = nullptr;

        if (result == Pal::Result::Success)
        {
            result = ClientAllocateGpuMemory(m_info, sizeInBytes, &gpuMem, &destGpuVa, &pMappedData);
        }

        if (result == Pal::Result::Success)
        {
            AccelStructCopyInfo copyInfo = {};
            copyInfo.mode = GpuRt::AccelStructCopyMode::Serialize;
            copyInfo.dstAccelStructAddr.gpu = destGpuVa;

            // BLAS list for copying
            Util::Vector<Pal::gpusize, 8, GpuRt::Device> blasList(this);

            // Hash map for logging unique BLASs
            Util::HashSet<Pal::gpusize, GpuRt::Device> blas(8, this);
            result = blas.Init();
            if (result == Pal::Result::Success)
            {
                for (uint32 i = 0; i < tlasCount; ++i)
                {
                    // Source from TLAS
                    copyInfo.srcAccelStructAddr.gpu = pTlasAddrs[i];

                    // Decode on GPU
                    ClientCmdContextHandle context = nullptr;
                    Pal::ICmdBuffer* pCmdBuffer = nullptr;

                    result = ClientAcquireCmdContext(m_info, &context, &pCmdBuffer);

                    if (result != Pal::Result::Success)
                    {
                        // Error occurs
                        break;
                    }

                    CopyAccelStruct(pCmdBuffer, copyInfo);

                    result = ClientFlushCmdContext(context);

                    if (result != Pal::Result::Success)
                    {
                        // Failed to submit command buffer
                        break;
                    }

                    // Gather all unique BLAS references
                    auto* pHeaderTopLevel =
                        reinterpret_cast<SerializedAccelStructHeader*>(pMappedData);

                    const uint32 numBlas =
                        static_cast<uint32>(pHeaderTopLevel->numBottomLevelAccelerationStructurePointersAfterHeader);

                    uint64* pBlasPointers = reinterpret_cast<uint64*>(pHeaderTopLevel + 1);

                    for (uint32 blasIndex = 0; blasIndex < numBlas; ++blasIndex)
                    {
                        // Skip null bottom level acceleration structures
                        const uint64 baseAddr = pBlasPointers[blasIndex];
                        if (baseAddr != 0)
                        {
                            // Enqueue unique bottom level acceleration structures for decode
                            if (blas.Contains(baseAddr) == 0)
                            {
                                blas.Insert(baseAddr);
                                blasList.PushBack(baseAddr);
                            }
                        }
                    }
                    void* pTlasData = pBlasPointers + numBlas;

                    WriteAccelStructChunk(pTraceSource,
                                          copyInfo.srcAccelStructAddr.gpu,
                                          0,
                                          pTlasData,
                                          pHeaderTopLevel->deserializedSizeInBytes);
                }

                if (pMappedData != nullptr)
                {
                    ClientFreeGpuMem(m_info, gpuMem);
                }

                if (result == Pal::Result::Success)
                {
                    for (uint32 i = 0; i < blasList.NumElements(); ++i)
                    {
                        const uint64 gpuVa = blasList.At(i);

                        // Get post-build information for bottom-level acceleration structures
                        result = GetAccelStructPostBuildSize(GpuRt::AccelStructPostBuildInfoType::CurrentSize,
                                                             &gpuVa,
                                                             1,
                                                             &sizeInBytes);
                        if (result != Pal::Result::Success)
                        {
                            // Error occurs
                            break;
                        }

                        if (sizeInBytes > 0)
                        {
                            PAL_ASSERT(sizeInBytes < UINT32_MAX);

                            ClientCmdContextHandle context = nullptr;
                            Pal::ICmdBuffer* pCmdBuffer = nullptr;

                            result = ClientAcquireCmdContext(m_info, &context, &pCmdBuffer);

                            if (result != Pal::Result::Success)
                            {
                                // Error occurs
                                break;
                            }

                            result = ClientAllocateGpuMemory(m_info, sizeInBytes, &gpuMem, &destGpuVa, &pMappedData);

                            if (result != Pal::Result::Success)
                            {
                                // Failed to allocate memory
                                break;
                            }
                            const uint32 sizeInDwords = static_cast<uint32>(sizeInBytes) >> 2;

                            CopyBufferRaw(pCmdBuffer, destGpuVa, gpuVa, sizeInDwords);

                            result = ClientFlushCmdContext(context);

                            if (result != Pal::Result::Success)
                            {
                                // Failed to submit command buffer
                                ClientFreeGpuMem(m_info, gpuMem);
                                break;
                            }

                            WriteAccelStructChunk(pTraceSource, gpuVa, 1, pMappedData, sizeInBytes);

                            ClientFreeGpuMem(m_info, gpuMem);
                        }
                    }
                }
                else
                {
                    // Error occurs, stop dumping
                }
            }
            else
            {
                // Failed to initrialize blas
            }
        }
        else
        {
            // Failed to allocate memory
        }
    }
    else
    {
        // Failed to get accel structure post build size
    }
    m_tlasCaptureList.Clear();

    // Disable and clear the accel struct tracker
    if (m_info.pAccelStructTracker != nullptr)
    {
        *m_info.pAccelStructTracker = {};
    }
}

// =====================================================================================================================
void Device::WriteAccelStructChunk(
    GpuUtil::ITraceSource* pTraceSource,
    Pal::gpusize           gpuVa,
    uint32                 isBlas,
    const void*            pData,
    size_t                 dataSize)
{
#if PAL_BUILD_RDF
    Pal::IPlatform* pPlatform = m_info.pPalPlatform;
    GpuUtil::TraceSession* pTraceSession = pPlatform->GetTraceSession();
#endif

    // Write Accel data to the TraceChunkInfo
    GpuUtil::TraceChunkInfo info = {};

    GpuRt::RawAccelStructRdfChunkHeader header = {};
    header.accelStructBaseVaLo = Util::LowPart(gpuVa);
    header.accelStructBaseVaHi = Util::HighPart(gpuVa);
    header.metaHeaderOffset    = 0;
    header.metaHeaderSize      = sizeof(GpuRt::AccelStructMetadataHeader);

    const auto* pMetadataHdr = static_cast<const AccelStructMetadataHeader*>(pData);
    header.headerOffset = pMetadataHdr->sizeInBytes;
    header.headerSize   = sizeof(GpuRt::AccelStructHeader);
    header.flags.blas   = isBlas;

    const char chunkId[] = "RawAccelStruct";
    Util::Strncpy(info.id, chunkId, Util::StringLength(chunkId));

    info.version           = GPURT_ACCEL_STRUCT_VERSION;
    info.headerSize        = sizeof(GpuRt::RawAccelStructRdfChunkHeader);
    info.pHeader           = &header;
    info.pData             = pData;
    info.dataSize          = dataSize;
    info.enableCompression = true;
#if PAL_BUILD_RDF
    pTraceSession->WriteDataChunk(pTraceSource, info);
#endif
}

// =====================================================================================================================
void Device::NotifyTlasBuild(
    gpusize address)
{
    // Only capture builds when TraceRay accel struct tracking is not enabled
    if (m_isTraceActive && (m_info.pAccelStructTracker == nullptr))
    {
        Util::MutexAuto lock(&m_traceBvhLock);
        m_tlasCaptureList.PushBack(address);
    }
}

// =====================================================================================================================
void Device::GetAccelStructPrebuildInfo(
    const AccelStructBuildInputs& buildInfo,
    AccelStructPrebuildInfo*      pPrebuildInfo)
{
    GpuBvhBuilder builder(nullptr,
        this,
        *m_info.pDeviceProperties,
        m_info.deviceSettings);

    builder.GetAccelerationStructurePrebuildInfo(buildInfo, pPrebuildInfo);
}

// =====================================================================================================================
void Device::GetSerializedAccelStructVersion(
    const void*                 pData,
    uint64_t*                   pVersion)
{
    PAL_ASSERT(pData != nullptr);

    // NumBottomLevelAccelerationStructurePointersAfterHeader
    const uint32* pNumberOfPtrs = static_cast<const uint32*>(Util::VoidPtrInc(pData,
        RayTracingSerializedAsHeaderNumBlasPtrsOffset));

    const uint32 sizeOfPtrs = RayTracingGpuVaSize * *pNumberOfPtrs;
    const uint32 serializedHeaderSize = RayTracingSerializedAsHeaderSize + sizeOfPtrs;

    // Fetch acceleration structure metadata size
    const uint32* pMetadataSizeInBytes = static_cast<const uint32*>(Util::VoidPtrInc(pData,
        serializedHeaderSize + offsetof(AccelStructMetadataHeader, sizeInBytes)));

    const AccelStructHeader* pHeader =
        static_cast<const AccelStructHeader*>(Util::VoidPtrInc(pData,
        serializedHeaderSize + *pMetadataSizeInBytes));

    *pVersion = static_cast<uint64>(pHeader->uuidHi) << 32 | pHeader->uuidLo;
}

// =====================================================================================================================
void Device::GetAccelStructInfo(
    const void*            pAccelStructData,
    AccelStructInfo* const pAccelStructInfo)
{
    // Extract relevant information from header
    const auto* pMetadataHdr = static_cast<const AccelStructMetadataHeader*>(pAccelStructData);

    const auto* pAccelStructHdr =
        static_cast<const AccelStructHeader*>(
            Util::VoidPtrInc(pMetadataHdr, pMetadataHdr->sizeInBytes));

    pAccelStructInfo->type          = static_cast<AccelStructType>(pAccelStructHdr->info.type);
    pAccelStructInfo->numDesc       = pAccelStructHdr->numDescs;
    pAccelStructInfo->numPrimitives = pAccelStructHdr->numPrimitives;
    pAccelStructInfo->buildFlags    = pAccelStructHdr->info.flags;
    pAccelStructInfo->buildType     = static_cast<AccelStructBuilderType>(pAccelStructHdr->info.buildType);
    pAccelStructInfo->buildMode     = pAccelStructHdr->info.mode;
    pAccelStructInfo->gpuVa         = (static_cast<uint64>(pMetadataHdr->addressHi) << 32) | pMetadataHdr->addressLo;
    pAccelStructInfo->sizeInBytes   = pAccelStructHdr->sizeInBytes;

    pAccelStructInfo->triangleCompressionMode =
        static_cast<TriangleCompressionMode>(pAccelStructHdr->info.triCompression);
    pAccelStructInfo->fp16BoxNodesInBlasMode =
        static_cast<Fp16BoxNodesInBlasMode>(pAccelStructHdr->info.fp16BoxNodesInBlasMode);
}

// =====================================================================================================================
// Computes size for decoded acceleration structure
Pal::Result Device::GetAccelStructPostBuildSize(
    AccelStructPostBuildInfoType type,
    const Pal::gpusize*          pGpuVas,
    uint32                       count,
    uint64*                      pSizeInBytes)
{
    uint32 infoSize = 0;

    switch (type)
    {
    case GpuRt::AccelStructPostBuildInfoType::CompactedSize:
        infoSize = sizeof(AccelStructPostBuildInfoCompactedSizeDesc);
        break;
    case GpuRt::AccelStructPostBuildInfoType::ToolsVisualization:
        infoSize = sizeof(AccelStructPostBuildInfoToolsVisualizationDesc);
        break;
    case GpuRt::AccelStructPostBuildInfoType::Serialization:
        infoSize = sizeof(AccelStructPostBuildInfoSerializationDesc);
        break;
    case GpuRt::AccelStructPostBuildInfoType::CurrentSize:
        infoSize = sizeof(AccelStructPostBuildInfoCurrentSizeDesc);
        break;
    }

    uint64 sizeInBytes = 0;
    uint64 allocationSizeInBytes = count * infoSize;

    Pal::gpusize destGpuVa = 0;
    ClientGpuMemHandle gpuMem = nullptr;
    AccelStructPostBuildInfoToolsVisualizationDesc* pInfo = nullptr;

    Pal::Result result = Pal::Result::Unsupported;

    result = ClientAllocateGpuMemory(
        m_info,
        allocationSizeInBytes,
        &gpuMem,
        &destGpuVa,
        reinterpret_cast<void**>(&pInfo));

    if (result == Pal::Result::Success)
    {

        GpuRt::AccelStructPostBuildInfo postBuildInfo = {};

        postBuildInfo.desc.infoType = type;
        postBuildInfo.desc.postBuildBufferAddr.gpu = destGpuVa;
        postBuildInfo.srcAccelStructCount = count;
        postBuildInfo.pSrcAccelStructGpuAddrs = pGpuVas;

        ClientCmdContextHandle context = nullptr;
        Pal::ICmdBuffer* pCmdBuffer = nullptr;

        result = ClientAcquireCmdContext(m_info, &context, &pCmdBuffer);

        if (result == Pal::Result::Success)
        {
            EmitAccelStructPostBuildInfo(pCmdBuffer, postBuildInfo);

            result = ClientFlushCmdContext(context);

            if (result == Pal::Result::Success)
            {
                for (uint32 i = 0; i < count; ++i)
                {
                    sizeInBytes = Util::Max(pInfo[i].decodedSizeInBytes, sizeInBytes);
                }
                *pSizeInBytes = sizeInBytes;
            }
            else
            {
                // Failed to submit command buffer
            }
        }
        else
        {
            // Error occurs
        }
    }
    else
    {
        // Failed to allocate memory
    }

    ClientFreeGpuMem(m_info, gpuMem);
    return result;
}

// =====================================================================================================================
void Device::BuildAccelStruct(
    Pal::ICmdBuffer*              pCmdBuffer,
    const AccelStructBuildInfo&   buildInfo
)
{
    DeviceSettings deviceSettings;
    deviceSettings = m_info.deviceSettings;

    if (pCmdBuffer != nullptr)
    {
        pCmdBuffer->CmdSaveComputeState(Pal::ComputeStateAll);

        GpuBvhBuilder builder(pCmdBuffer,
                              this,
                              *m_info.pDeviceProperties,
                              deviceSettings);

        builder.BuildRaytracingAccelerationStructure(buildInfo);

        pCmdBuffer->CmdRestoreComputeState(Pal::ComputeStateAll);
    }
    else
    {
        CpuBvhBuilder builder(this,
                              *m_info.pDeviceProperties,
                              deviceSettings);

        builder.BuildRaytracingAccelerationStructure(buildInfo);
    }
}

// =====================================================================================================================
void Device::EmitAccelStructPostBuildInfo(
    Pal::ICmdBuffer*                pCmdBuffer,
    const AccelStructPostBuildInfo& postBuildInfo
)
{
    DeviceSettings deviceSettings;
    deviceSettings = m_info.deviceSettings;

    if (pCmdBuffer != nullptr)
    {
        pCmdBuffer->CmdSaveComputeState(Pal::ComputeStateAll);

        GpuBvhBuilder builder(pCmdBuffer,
                              this,
                              *m_info.pDeviceProperties,
                              deviceSettings);

        builder.EmitAccelerationStructurePostBuildInfo(postBuildInfo);

        pCmdBuffer->CmdRestoreComputeState(Pal::ComputeStateAll);
    }
    else
    {
        CpuBvhBuilder builder(this,
                              *m_info.pDeviceProperties,
                              deviceSettings);

        builder.EmitAccelerationStructurePostBuildInfo(postBuildInfo);
    }
}

// =====================================================================================================================
void Device::CopyAccelStruct(
    Pal::ICmdBuffer*              pCmdBuffer,
    const AccelStructCopyInfo&    copyInfo
)
{
    DeviceSettings deviceSettings;
    deviceSettings = m_info.deviceSettings;

    if (pCmdBuffer != nullptr)
    {
        pCmdBuffer->CmdSaveComputeState(Pal::ComputeStateAll);

        GpuBvhBuilder builder(pCmdBuffer,
                              this,
                              *m_info.pDeviceProperties,
                              deviceSettings);

        builder.CopyAccelerationStructure(copyInfo);

        pCmdBuffer->CmdRestoreComputeState(Pal::ComputeStateAll);
    }
    else
    {
        CpuBvhBuilder builder(this,
                              *m_info.pDeviceProperties,
                              deviceSettings);

        builder.CopyAccelerationStructure(copyInfo);
    }
}

// =====================================================================================================================
void Device::InitExecuteIndirect(
    Pal::ICmdBuffer*                   pCmdBuffer,
    const InitExecuteIndirectUserData& userData,
    uint32                             maxDispatchCount,
    uint32                             pipelineCount
    ) const
{
    pCmdBuffer->CmdSaveComputeState(Pal::ComputeStateAll);

    pCmdBuffer->CmdSetUserData(Pal::PipelineBindPoint::Compute,
                               0,
                               sizeof(userData) / sizeof(uint32),
                               reinterpret_cast<const uint32*>(&userData));

    Pal::IPipeline* pPipeline = GetInternalPipeline(InternalRayTracingCsType::InitExecuteIndirect, {}, 0);

    Pal::PipelineBindParams bindParams = {};
    bindParams.pPipeline = pPipeline;
    bindParams.pipelineBindPoint = Pal::PipelineBindPoint::Compute;

    pCmdBuffer->CmdBindPipeline(bindParams);

    const uint32 threadGroupDim = 8;
    pCmdBuffer->CmdDispatch(Util::RoundUpQuotient(maxDispatchCount, threadGroupDim),
                            Util::RoundUpQuotient(pipelineCount, threadGroupDim),
                            1);

    pCmdBuffer->CmdRestoreComputeState(Pal::ComputeStateAll);
}

// =====================================================================================================================
// Executes the copy buffer shader
void Device::CopyBufferRaw(
    Pal::ICmdBuffer* pCmdBuffer,
    Pal::gpusize     dstBufferVa,    // Destination buffer GPU VA
    Pal::gpusize     srcBufferVa,    // Source buffer GPU VA
    uint32           numDwords)      // Number of Dwords to copy
{
#if GPURT_DEVELOPER
    OutputPipelineName(pCmdBuffer, InternalRayTracingCsType::CopyBufferRaw);
#endif

    Pal::PipelineBindParams bindParams = {};
    bindParams.pipelineBindPoint = Pal::PipelineBindPoint::Compute;
    bindParams.pPipeline = GetInternalPipeline(InternalRayTracingCsType::CopyBufferRaw, {}, 0);

    pCmdBuffer->CmdBindPipeline(bindParams);

    uint32 entryOffset = 0;

    const CopyBufferRaw::Constants shaderConstants =
    {
        numDwords,
    };

    // Set shader constants
    entryOffset = WriteUserDataEntries(pCmdBuffer , &shaderConstants, CopyBufferRaw::NumEntries, entryOffset);

    // Set buffer addresses
    entryOffset = WriteBufferVa(pCmdBuffer, srcBufferVa, entryOffset);
    entryOffset = WriteBufferVa(pCmdBuffer, dstBufferVa, entryOffset);

    // Calculates the correct dispatch size to use for Ray Tracing shaders
    uint32 dispatchSize = Util::RoundUpQuotient(numDwords, DefaultThreadGroupSize);

    RGP_PUSH_MARKER(pCmdBuffer, "Copy Buffer");

    pCmdBuffer->CmdDispatch(dispatchSize, 1, 1);

    RGP_POP_MARKER(pCmdBuffer);
}

// =====================================================================================================================
// Writes the provided entries into the compute shader user data slots
uint32 Device::WriteUserDataEntries(
    Pal::ICmdBuffer* pCmdBuffer,
    const void*      pEntries,    // User data entries
    uint32           numEntries,  // Number of entries
    uint32           entryOffset) // Offset of the first entry
{
    pCmdBuffer->CmdSetUserData(Pal::PipelineBindPoint::Compute,
        entryOffset,
        numEntries,
        static_cast<const uint32*>(pEntries));

    return entryOffset + numEntries;
}

// =====================================================================================================================
// Writes a gpu virtual address for a buffer into the compute shader user data slots
uint32 Device::WriteBufferVa(
    Pal::ICmdBuffer* pCmdBuffer,
    Pal::gpusize     virtualAddress, // GPUVA of the buffer
    uint32           entryOffset)    // Offset of the first entry
{
    const uint32 entries[] = { Util::LowPart(virtualAddress), Util::HighPart(virtualAddress) };
    return WriteUserDataEntries(pCmdBuffer, entries, GPURT_ARRAY_SIZE(entries), entryOffset);
}

#if GPURT_DEVELOPER
// =====================================================================================================================
void Device::PushRGPMarker(
    Pal::ICmdBuffer* pCmdBuffer,
    const char* pFormat,
    ...)
{
    va_list args;
    va_start(args, pFormat);

    char strBuffer[256];
    Util::Vsnprintf(strBuffer, sizeof(strBuffer), pFormat, args);

    va_end(args);

    ClientInsertRGPMarker(pCmdBuffer, strBuffer, true);
}

// =====================================================================================================================
void Device::PopRGPMarker(Pal::ICmdBuffer* pCmdBuffer)
{
    ClientInsertRGPMarker(pCmdBuffer, nullptr, false);
}

// =====================================================================================================================
// Driver generated RGP markers
void Device::OutputPipelineName(
    Pal::ICmdBuffer*         pCmdBuffer,
    InternalRayTracingCsType type)
{
    constexpr uint32 MaxStrLength = 256;
    char buildShaderInfo[MaxStrLength];
    const PipelineBuildInfo* pPipelineBuildInfo = &InternalPipelineBuildInfo[static_cast<uint32>(type)];
    Util::Snprintf(buildShaderInfo, MaxStrLength, "BVH Build Pipeline: %s", pPipelineBuildInfo->pPipelineName);

    pCmdBuffer->CmdCommentString(buildShaderInfo);
}
#endif
};
