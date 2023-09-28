/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2019-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "gpurtBvhBatcher.h"
#include "gpurtBvhBuilder.h"
#include "gpurt/gpurtLib.h"
#include "gpurt/gpurtCounter.h"
#include "gpurtInternalShaderBindings.h"

#include "palCmdBuffer.h"
#include "palInlineFuncs.h"
#include "palHashMapImpl.h"
#include "palMemTrackerImpl.h"
#include "palHashSetImpl.h"
#include "palVectorImpl.h"

// __declspec(dllexport) has limitations. "No name decoration is applied to exported C functions or C++ extern "C"
// functions using the __cdecl calling convention." In order to export a unmangled symbol for a function that uses a
// different calling convention you must invoke the linker directly on MSVC. NOTE: Name mangling doesn't occur for
// exported C functions on 64-bit platforms.
//
// Place this inside the function body you want to export. And ensure the name of the function is the symbol name
// you want to export
#define GPURT_EXPORT_UNMANGLED_SYMBOL_MSVC

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

#include <pipelines/g_internal_shaders.h>

//=====================================================================================================================
// Enumeration for supported intrinsic functions in GPURT
enum class RtIntrinsicFunction : uint32
{
    RayQueryProceed,
    TraceRayInline,
    TraceRay,
    TraceRayUsingHitToken,
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 37
    TraceRayUsingRayQuery,
#endif
    GetInstanceID,
    GetInstanceIndex,
    GetObjectToWorldTransform,
    GetWorldToObjectTransform,
    FetchTrianglePositionFromNodePointer,
    FetchTrianglePositionFromRayQuery,
    GetRayQuery64BitInstanceNodePtr,
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
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 37
    "\01?TraceRay1_1@@YAXIIIIIIIMMMMMMMM@Z",
#endif
    "\01?GetInstanceID@@YAI_K@Z",
    "\01?GetInstanceIndex@@YAI_K@Z",
    "\01?GetObjectToWorldTransform@@YAM_KII@Z",
    "\01?GetWorldToObjectTransform@@YAM_KII@Z",
    "\01?FetchTrianglePositionFromNodePointer@@YA?AUTriangleData@@_KI@Z",
    "\01?FetchTrianglePositionFromRayQuery@@YA?AUTriangleData@@URayQueryInternal@@_N@Z",
    "\01?GetRayQuery64BitInstanceNodePtr@@YA_K_KI@Z",
};

#if GPURT_BUILD_RTIP2
//=====================================================================================================================
// Function table for ray tracing IP2.0
constexpr const char* FunctionTableRTIP2_0[] =
{
    "\01?RayQueryProceed2_0@@YA_NURayQueryInternal@@IV?$vector@I$02@@@Z",
    "\01?TraceRayInline2_0@@YAXURayQueryInternal@@IIIIIURayDesc@@V?$vector@I$02@@@Z",
    "\01?TraceRay2_0@@YAXIIIIIIIMMMMMMMM@Z",
    "\01?TraceRayUsingHitToken2_0@@YAXIIIIIIIMMMMMMMMII@Z",
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 37
    "\01?TraceRay2_0@@YAXIIIIIIIMMMMMMMM@Z",
#endif
    "\01?GetInstanceID@@YAI_K@Z",
    "\01?GetInstanceIndex@@YAI_K@Z",
    "\01?GetObjectToWorldTransform@@YAM_KII@Z",
    "\01?GetWorldToObjectTransform@@YAM_KII@Z",
    "\01?FetchTrianglePositionFromNodePointer@@YA?AUTriangleData@@_KI@Z",
    "\01?FetchTrianglePositionFromRayQuery@@YA?AUTriangleData@@URayQueryInternal@@_N@Z",
    "\01?GetRayQuery64BitInstanceNodePtr@@YA_K_KI@Z",
};
#endif

//=====================================================================================================================
// Maps Pal::RayTracingIpLevel to the appropriate function table.
static Pal::Result QueryRayTracingEntryFunctionTableInternal(
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
#if GPURT_BUILD_RTIP2
        case Pal::RayTracingIpLevel::RtIp2_0:
            ppFuncTable = FunctionTableRTIP2_0;
            break;
#endif
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
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 37
        pEntryFunctionTable->traceRay.pTraceRayUsingRayQuery =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::TraceRayUsingRayQuery)];
#endif

        pEntryFunctionTable->rayQuery.pTraceRayInline =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::TraceRayInline)];
        pEntryFunctionTable->rayQuery.pProceed =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::RayQueryProceed)];
        pEntryFunctionTable->rayQuery.pGet64BitInstanceNodePtr =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::GetRayQuery64BitInstanceNodePtr)];

        pEntryFunctionTable->intrinsic.pGetInstanceID =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::GetInstanceID)];
        pEntryFunctionTable->intrinsic.pGetInstanceIndex =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::GetInstanceIndex)];
        pEntryFunctionTable->intrinsic.pGetObjectToWorldTransform =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::GetObjectToWorldTransform)];
        pEntryFunctionTable->intrinsic.pGetWorldToObjectTransform =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::GetWorldToObjectTransform)];

        pEntryFunctionTable->intrinsic.pFetchTrianglePositionFromNodePointer =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::FetchTrianglePositionFromNodePointer)];
        pEntryFunctionTable->intrinsic.pFetchTrianglePositionFromRayQuery =
            ppFuncTable[static_cast<uint32>(RtIntrinsicFunction::FetchTrianglePositionFromRayQuery)];
    }

    return result;
}

// =====================================================================================================================
// Create GPURT device
//
Pal::Result GPURT_API_ENTRY CreateDevice(
    const DeviceInitInfo&  info,
    const ClientCallbacks& callbacks,
    void*            const pMemory,
    IDevice**        const ppDevice)
{
    GPURT_EXPORT_UNMANGLED_SYMBOL_MSVC

    Pal::Result result = Pal::Result::ErrorInvalidValue;
    if ((ppDevice != nullptr) && (pMemory != nullptr))
    {
        Internal::Device* pDevice = PAL_PLACEMENT_NEW(pMemory) Internal::Device(info, callbacks);
        result = pDevice->Init();

        if (result != Pal::Result::Success)
        {
            pDevice->Destroy();
            pDevice = nullptr;
        }

        *ppDevice = pDevice;
    }

    return result;
}

// =====================================================================================================================
size_t GPURT_API_ENTRY GetDeviceSize()
{
    GPURT_EXPORT_UNMANGLED_SYMBOL_MSVC

    return sizeof(Internal::Device);
}

// =====================================================================================================================
PipelineShaderCode GPURT_API_ENTRY GetShaderLibraryCode(
    ShaderLibraryFeatureFlags flags)
{
    GPURT_EXPORT_UNMANGLED_SYMBOL_MSVC

    const bool enableDevFeatures =
        Util::TestAnyFlagSet(flags, static_cast<uint32>(ShaderLibraryFeatureFlag::Developer));

    const bool enableSwTraversal =
        Util::TestAnyFlagSet(flags, static_cast<uint32>(ShaderLibraryFeatureFlag::SoftwareTraversal));

    PipelineShaderCode code = {};

#define CHOOSE_SHADER(x) { code.pDxilCode = (x); code.dxilSize = sizeof(x); \
                           code.pSpvCode  = (x); code.spvSize  = sizeof(x); }

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
// Maps Pal::RayTracingIpLevel to the appropriate function table.
Pal::Result GPURT_API_ENTRY QueryRayTracingEntryFunctionTable(
    const Pal::RayTracingIpLevel   rayTracingIpLevel,
    EntryFunctionTable* const      pEntryFunctionTable)
{
    return QueryRayTracingEntryFunctionTableInternal(rayTracingIpLevel, pEntryFunctionTable);
}

namespace Internal {

//=====================================================================================================================
uint32 Device::GetStaticPipelineFlags(
    bool  skipTriangles,
    bool  skipProceduralPrims,
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 37
    bool  unused0,
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

    if (m_info.deviceSettings.enableFusedInstanceNode)
    {
        pipelineFlags |= static_cast<uint32>(GpuRt::StaticPipelineFlag::EnableFusedInstanceNodes);
    }

    return pipelineFlags;
}

// =====================================================================================================================
void Device::SetUpClientCallbacks(ClientCallbacks* pClientCb)
{
    *pClientCb = m_clientCb;
}

// =====================================================================================================================
void Device::GetAccelStructPrebuildInfo(
    const AccelStructBuildInputs& buildInfo,
    AccelStructPrebuildInfo* pPrebuildInfo)
{
    DeviceSettings deviceSettings;
    ClientCallbacks clientCb = {};

    SetUpClientCallbacks(&clientCb);

    deviceSettings = m_info.deviceSettings;

    BvhBuilder builder(nullptr,
                       this,
                       *m_info.pDeviceProperties,
                       clientCb,
                       deviceSettings);

    builder.GetAccelerationStructurePrebuildInfo(buildInfo, pPrebuildInfo);
}

// =====================================================================================================================
Device::Device(
    const DeviceInitInfo&  info,
    const ClientCallbacks& clientCb)
    :
    m_info(info),
    m_clientCb(clientCb),
    m_pipelineMap(64, &m_allocator),
    m_tlasCaptureList(this),
    m_isTraceActive(false),
    m_accelStructTraceSource(this),
    m_rayHistoryTraceSource(this),
#if GPURT_ENABLE_GPU_DEBUG
    m_debugMonitor(this),
#endif
    m_rayHistoryTraceList(this)
{
}

// =====================================================================================================================
Device::~Device()
{
    for (InternalPipelineMap::Iterator itr = m_pipelineMap.Begin(); itr.Get(); itr.Next())
    {
        m_clientCb.pfnDestroyInternalComputePipeline(m_info, itr.Get()->value.pPipeline, itr.Get()->value.pMemory);
    }
}

// =====================================================================================================================
void Device::Destroy()
{
    this->~Device();
}

// =====================================================================================================================
// Initialize the GPURT device.
Pal::Result Device::Init()
{
    Pal::Result result = Pal::Result::Success;
    result = m_pipelineMap.Init();
    *m_info.pAccelStructTracker = {};

    Pal::DeviceProperties props = {};
    m_info.pPalDevice->GetProperties(&props);

    m_bufferSrdSizeDw = props.gfxipProperties.srdSizes.bufferView / sizeof(uint32);

    if (m_info.deviceSettings.emulatedRtIpLevel == Pal::RayTracingIpLevel::None)
    {
        m_rtIpLevel = m_info.pDeviceProperties->gfxipProperties.rayTracingIp;
    }
    else
    {
        m_rtIpLevel = m_info.deviceSettings.emulatedRtIpLevel;
    }

#if PAL_BUILD_RDF
    Pal::IPlatform* pPlatform = m_info.pPalPlatform;
    GpuUtil::TraceSession* pTraceSession = pPlatform->GetTraceSession();
    pTraceSession->RegisterSource(&m_accelStructTraceSource);
    pTraceSession->RegisterSource(&m_rayHistoryTraceSource);
#endif

    // Merged encode/build only works with parallel build
    if (m_info.deviceSettings.enableParallelBuild == false)
    {
        m_info.deviceSettings.enableMergedEncodeBuild = false;
    }

#if GPURT_BUILD_RTIP2
    // Fused instance node is currently only supported on RTIP2.0
    if (m_rtIpLevel != Pal::RayTracingIpLevel::RtIp2_0)
    {
        m_info.deviceSettings.enableFusedInstanceNode = false;
    }
#endif

    return result;
}

//=====================================================================================================================
// Maps Pal::RayTracingIpLevel to the appropriate function table.
Pal::Result Device::QueryRayTracingEntryFunctionTable(
    const Pal::RayTracingIpLevel   rayTracingIpLevel,
    EntryFunctionTable* const      pEntryFunctionTable)
{
    return QueryRayTracingEntryFunctionTableInternal(rayTracingIpLevel,
                                                     pEntryFunctionTable);
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
// Returns a specific internal pipeline or initialize specified internal pipeline
Pal::IPipeline* Device::GetInternalPipeline(
    InternalRayTracingCsType        shaderType,
    const CompileTimeBuildSettings& buildSettings,
    uint32                          buildSettingsHash
    ) const
{
    Util::RWLock* pPipelineLock = const_cast<Util::RWLock*>(&m_internalPipelineLock);

    InternalPipelineKey key = {};
    key.shaderType   = shaderType;
    key.settingsHash = buildSettingsHash;

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

            uint32 nodeOffset       = 0;
            uint32 uavCount         = 0;
            uint32 uavBindingCount  = 0;
            uint32 cbvCount         = 0;
            uint32 cbvBindingCount  = 0;

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
                    nodes[nodeIndex].srdStartIndex = cbvBindingCount;
                    nodes[nodeIndex].binding       = cbvBindingCount;
                    nodes[nodeIndex].descSet       = 1;
                    cbvCount++;
                    cbvBindingCount++;
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
                    nodes[nodeIndex].srdStartIndex = uavBindingCount;
                    nodes[nodeIndex].binding       = uavBindingCount;
                    nodes[nodeIndex].descSet       = 0;
                    uavCount++;
                    uavBindingCount++;
                    break;
                case NodeType::UavTable:
                case NodeType::TypedUavTable:
                    nodes[nodeIndex].logicalId     = uavCount + ReservedLogicalIdCount;
                    nodes[nodeIndex].srdStartIndex = 0;
                    nodes[nodeIndex].binding       = 0;
                    nodes[nodeIndex].descSet       = tableSet++;
                    uavCount++;
                    break;
                default:
                    PAL_ASSERT_ALWAYS();
                }
                nodeOffset += nodeSize;
            }

            PipelineBuildInfo buildInfo = *pPipelineBuildInfo;

            buildInfo.pNodes = nodes;
            buildInfo.apiPsoHash = GetInternalPsoHash(buildInfo.shaderType, buildSettings);
            PipelineCompilerOption wave64Option[1] = {
                {PipelineOptionName::waveSize, PipelineOptionName::Wave64} };

            switch (buildInfo.shaderType)
            {
                case InternalRayTracingCsType::BuildBVHTD:
                case InternalRayTracingCsType::BuildBVHTDTR:
                case InternalRayTracingCsType::BuildParallel:
                    buildInfo.hashedCompilerOptionCount = 1;
                    buildInfo.pHashedCompilerOptions = wave64Option;
                    break;
                default:
                    buildInfo.hashedCompilerOptionCount = 0;
                    buildInfo.pHashedCompilerOptions = nullptr;
                    break;
            }

            CompileTimeConstants compileConstants = {};

#if GPURT_DEVELOPER
            constexpr uint32 MaxStrLength = 256;
            char pipelineName[MaxStrLength];
#endif

            const uint32 lastNodeIndex = pPipelineBuildInfo->nodeCount;

            static constexpr uint32 ReservedBuildSettingsCBVIndex = 255;
            nodes[lastNodeIndex].type          = NodeType::ConstantBuffer;
            nodes[lastNodeIndex].dwSize        = 2;
            nodes[lastNodeIndex].dwOffset      = nodeOffset;
            nodes[lastNodeIndex].logicalId     = cbvCount + ReservedLogicalIdCount;
            nodes[lastNodeIndex].srdStartIndex = ReservedBuildSettingsCBVIndex;
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

                constexpr const char* GeometryTypeStr[] =
                {
                    "_Tri",  // GpuRt::GeometryType::Triangles,
                    "_Aabb", // GpuRt::GeometryType::Aabbs,
                };

                char radixSortLevelStr[MaxStrLength];
                Util::Snprintf(radixSortLevelStr, MaxStrLength, "_RadixSortLevel_%d", buildSettings.radixSortScanLevel);

                Util::Snprintf(pipelineName, MaxStrLength, "%s%s%s_%s%s%s%s%s",
                                buildInfo.pPipelineName,
                                buildSettings.topLevelBuild ? "_TLAS" : "_BLAS",
                                buildSettings.topLevelBuild ? "" : GeometryTypeStr[buildSettings.geometryType],
                                buildSettings.enableTopDownBuild ? "TopDown" : BuildModeStr[buildSettings.buildMode],
                                buildSettings.doTriangleSplitting ? "_TriSplit" : "",
                                buildSettings.triangleCompressionMode ? "_TriCompr" : "",
                                RebraidTypeStr[buildSettings.rebraidType],
                                buildSettings.enableMergeSort ? "_MergeSort" : radixSortLevelStr);

                buildInfo.pPipelineName = &pipelineName[0];
            }
#endif

            result = m_clientCb.pfnCreateInternalComputePipeline(m_info,
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
// Binds the pipeline that corresponds with the provided internal shader type
void Device::BindPipeline(
    Pal::ICmdBuffer*                pCmdBuffer,
    InternalRayTracingCsType        type,
    const CompileTimeBuildSettings& buildSettings,
    uint32                          buildSettingsHash) const
{
    const Pal::IPipeline* pPipeline = GetInternalPipeline(type, buildSettings, buildSettingsHash);

    Pal::PipelineBindParams bindParam = {};
    bindParam.pipelineBindPoint       = Pal::PipelineBindPoint::Compute;
    bindParam.pPipeline               = pPipeline;
    bindParam.apiPsoHash              = GetInternalPsoHash(type, buildSettings);

#if GPURT_DEVELOPER
    OutputPipelineName(pCmdBuffer, type);
#endif

    pCmdBuffer->CmdBindPipeline(bindParam);
}

// =====================================================================================================================
// Setup the SRD tables for the provided buffer views
uint32 Device::WriteBufferSrdTable(
    Pal::ICmdBuffer*           pCmdBuffer,
    const Pal::BufferViewInfo* pBufferViews,
    uint32                     count,
    bool                       typedBuffer,
    uint32                     entryOffset) const
{
    const uint32  bufferSrdSizeDw = GetBufferSrdSizeDw();
    Pal::IDevice* pPalDevice      = GetPalDevice();
    const void*   pNullBuffer     = GetInitInfo().pDeviceProperties->gfxipProperties.nullSrds.pNullBufferView;

    gpusize tableVa;
    void* pTable = pCmdBuffer->CmdAllocateEmbeddedData(bufferSrdSizeDw * count, bufferSrdSizeDw, &tableVa);

    for (uint32 i = 0; i < count; i++)
    {
        const Pal::BufferViewInfo* pCurrentBufInfo = &pBufferViews[i];

        if (pCurrentBufInfo->gpuAddr == 0)
        {
            memcpy(pTable, pNullBuffer, bufferSrdSizeDw * sizeof(uint32));
        }
        else if (typedBuffer)
        {
            pPalDevice->CreateTypedBufferViewSrds(1, pCurrentBufInfo, pTable);
        }
        else
        {
            pPalDevice->CreateUntypedBufferViewSrds(1, pCurrentBufInfo, pTable);
        }
        pTable = Util::VoidPtrInc(pTable, bufferSrdSizeDw * sizeof(uint32));
    }

    entryOffset = WriteUserDataEntries(pCmdBuffer, &tableVa, 1, entryOffset);

    return entryOffset;
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

        gpusize destGpuVa = 0;
        ClientGpuMemHandle gpuMem = nullptr;
        void* pMappedData = nullptr;

        if (result == Pal::Result::Success)
        {
            result = m_clientCb.pfnAllocateGpuMemory(m_info, sizeInBytes, &gpuMem, &destGpuVa, &pMappedData);
        }

        if (result == Pal::Result::Success)
        {
            AccelStructCopyInfo copyInfo = {};
            copyInfo.mode = AccelStructCopyMode::Serialize;
            copyInfo.dstAccelStructAddr.gpu = destGpuVa;

            // BLAS list for copying
            Util::Vector<gpusize, 8, Internal::Device> blasList(this);

            // Hash map for logging unique BLASs
            Util::HashSet<gpusize, Internal::Device> blas(8, this);
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

                    result = m_clientCb.pfnAcquireCmdContext(m_info, &context, &pCmdBuffer);

                    if (result != Pal::Result::Success)
                    {
                        // Error occurs
                        break;
                    }

                    CopyAccelStruct(pCmdBuffer, copyInfo);

                    result = m_clientCb.pfnFlushCmdContext(context);

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
                    m_clientCb.pfnFreeGpuMem(m_info, gpuMem);
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

                            result = m_clientCb.pfnAcquireCmdContext(m_info, &context, &pCmdBuffer);

                            if (result != Pal::Result::Success)
                            {
                                // Error occurs
                                break;
                            }

                            result = m_clientCb.pfnAllocateGpuMemory(m_info, sizeInBytes, &gpuMem, &destGpuVa, &pMappedData);

                            if (result != Pal::Result::Success)
                            {
                                // Failed to allocate memory
                                break;
                            }
                            const uint32 sizeInDwords = static_cast<uint32>(sizeInBytes) >> 2;

                            CopyBufferRaw(pCmdBuffer, destGpuVa, gpuVa, sizeInDwords);

                            result = m_clientCb.pfnFlushCmdContext(context);

                            if (result != Pal::Result::Success)
                            {
                                // Failed to submit command buffer
                                m_clientCb.pfnFreeGpuMem(m_info, gpuMem);
                                break;
                            }

                            WriteAccelStructChunk(pTraceSource, gpuVa, 1, pMappedData, sizeInBytes);

                            m_clientCb.pfnFreeGpuMem(m_info, gpuMem);
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
    gpusize                gpuVa,
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

    RawAccelStructRdfChunkHeader header = {};
    header.accelStructBaseVaLo = Util::LowPart(gpuVa);
    header.accelStructBaseVaHi = Util::HighPart(gpuVa);
    header.metaHeaderOffset    = 0;
    header.metaHeaderSize      = sizeof(GpuRt::AccelStructMetadataHeader);

    const auto* pMetadataHdr = static_cast<const AccelStructMetadataHeader*>(pData);
    header.headerOffset = pMetadataHdr->sizeInBytes;
    header.headerSize   = sizeof(GpuRt::AccelStructHeader);
    header.flags.blas   = isBlas;

    const char chunkId[] = "RawAccelStruct";
    Util::Strncpy(info.id, chunkId, sizeof(info.id));

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
uint32 Device::GetRayHistoryLightCounterMask() const
{
    return (1 << RayHistoryTokenBegin_v2) |
           (1 << RayHistoryTokenAnyHitStatus) |
           (1 << RayHistoryTokenCandidateIntersectionResult) |
           (1 << RayHistoryTokenIntersectionResult_v2);
}

// =====================================================================================================================
void Device::TraceRtDispatch(
    Pal::ICmdBuffer*                pCmdBuffer,
    RtPipelineType                  pipelineType,
    RtDispatchInfo                  dispatchInfo,
    DispatchRaysConstants*          pConstants)
{
    Util::MutexAuto lock(&m_traceRayHistoryLock);

    // TraceRayCounterRayHistoryLight
    pConstants->constData.counterMode            = 1;
    pConstants->constData.counterRayIdRangeBegin = 0;
    pConstants->constData.counterRayIdRangeEnd   = 0xFFFFFFFF;
    pConstants->constData.counterMask            = GetRayHistoryLightCounterMask();
    pConstants->constData.rayDispatchWidth       = dispatchInfo.dimX;
    pConstants->constData.rayDispatchHeight      = dispatchInfo.dimY;
    pConstants->constData.rayDispatchDepth       = dispatchInfo.dimZ;
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION >= 38
    // When not equal to 0 dispatch is rayquery.
    if (dispatchInfo.threadGroupSizeX != 0)
    {
        pConstants->constData.rayDispatchWidth  = dispatchInfo.dimX * dispatchInfo.threadGroupSizeX;
        pConstants->constData.rayDispatchHeight = dispatchInfo.dimY * dispatchInfo.threadGroupSizeY;
        pConstants->constData.rayDispatchDepth  = dispatchInfo.dimZ * dispatchInfo.threadGroupSizeZ;
    }
#endif

    RayHistoryTraceListInfo traceListInfo = {};

    // The header for all ray history chunks
    RayHistoryRdfChunkHeader header = {};
    header.dispatchID = m_rayHistoryTraceSource.GetDispatchID();

    traceListInfo.rayHistoryRdfChunkHeader = header;

    uint64 traceSizeInBytes = GetRayHistoryBufferSizeInBytes();
    uint32 shaderTableSizeInBytes = dispatchInfo.raygenShaderTable.size +
                                    dispatchInfo.missShaderTable.size +
                                    dispatchInfo.hitGroupTable.size;

    // Ray history trace
    // Shader table info * 3
    // Total of Shader table data
    uint64 totalSizeInBytes = traceSizeInBytes + (sizeof(ShaderTableInfo) * 3) + shaderTableSizeInBytes;

    gpusize destGpuVa = 0;
    ClientGpuMemHandle gpuMem = nullptr;
    void* pMappedData = nullptr;

    // Memory allocation for ray history trace buffer data
    Pal::Result result = m_clientCb.pfnAllocateGpuMemory(m_info, totalSizeInBytes, &gpuMem, &destGpuVa, &pMappedData);
    if (result == Pal::Result::Success)
    {
        // Create inlined raw buffer view
        Pal::BufferViewInfo viewInfo = {};
        viewInfo.gpuAddr = destGpuVa;
        viewInfo.range = traceSizeInBytes;

        Pal::IDevice* pPalDevice = m_info.pPalDevice;
        pPalDevice->CreateUntypedBufferViewSrds(1, &viewInfo, pConstants->descriptorTable.internalUavBufferSrd);

        // First data slot is reserved for token counts
        memset(pMappedData, 0, RAY_TRACING_COUNTER_RESERVED_BYTE_SIZE);
        traceListInfo.pRayHistoryTraceBuffer = pMappedData;
        traceListInfo.traceBufferGpuMem      = gpuMem;
        traceListInfo.isIndirect             = false;

        AddMetadataToList(dispatchInfo, pipelineType, &traceListInfo);

        uint64 runningOffsetGpuVa = 0;

        traceListInfo.shaderTableRayGenOffset = traceSizeInBytes;
        runningOffsetGpuVa = traceSizeInBytes + sizeof(ShaderTableInfo);

        WriteShaderTableData(pCmdBuffer,
                             runningOffsetGpuVa,
                             destGpuVa,
                             traceListInfo.shaderTableRayGenOffset,
                             pMappedData,
                             ShaderTableType::RayGen,
                             dispatchInfo.raygenShaderTable,
                             &traceListInfo.shaderTableRayGenInfo);

        traceListInfo.shaderTableMissOffset =
            traceListInfo.shaderTableRayGenOffset + dispatchInfo.raygenShaderTable.size;

        runningOffsetGpuVa += dispatchInfo.raygenShaderTable.size + sizeof(ShaderTableInfo);

        WriteShaderTableData(pCmdBuffer,
                             runningOffsetGpuVa,
                             destGpuVa,
                             traceListInfo.shaderTableMissOffset,
                             pMappedData,
                             ShaderTableType::Miss,
                             dispatchInfo.missShaderTable,
                             &traceListInfo.shaderTableMissInfo);

        traceListInfo.shaderTableHitGroupOffset =
            traceListInfo.shaderTableMissOffset + dispatchInfo.missShaderTable.size;

        runningOffsetGpuVa += dispatchInfo.missShaderTable.size + sizeof(ShaderTableInfo);

        WriteShaderTableData(pCmdBuffer,
                             runningOffsetGpuVa,
                             destGpuVa,
                             traceListInfo.shaderTableHitGroupOffset,
                             pMappedData,
                             ShaderTableType::HitGroup,
                             dispatchInfo.hitGroupTable,
                             &traceListInfo.shaderTableHitGroupInfo);

        traceListInfo.shaderTableCallableOffset =
            traceListInfo.shaderTableHitGroupOffset + dispatchInfo.hitGroupTable.size;

        runningOffsetGpuVa += dispatchInfo.hitGroupTable.size + sizeof(ShaderTableInfo);

        WriteShaderTableData(pCmdBuffer,
                             runningOffsetGpuVa,
                             destGpuVa,
                             traceListInfo.shaderTableCallableOffset,
                             pMappedData,
                             ShaderTableType::Callable,
                             dispatchInfo.callableShaderTable,
                             &traceListInfo.shaderTableCallableInfo);

        m_rayHistoryTraceList.PushBack(traceListInfo);
    }
    else
    {
        // Failed to allocate memory
    }
}

// =====================================================================================================================
void Device::AddMetadataToList(
    RtDispatchInfo              dispatchInfo,
    RtPipelineType              pipelineType,
    RayHistoryTraceListInfo*    pTraceListInfo)
{
    CounterInfo counterInfo = {};
    counterInfo.dispatchRayDimensionX  = dispatchInfo.dimX;
    counterInfo.dispatchRayDimensionY  = dispatchInfo.dimY;
    counterInfo.dispatchRayDimensionZ  = dispatchInfo.dimZ;
    counterInfo.pipelineType           = uint32(pipelineType);
    counterInfo.rayCounterDataSize     = GetRayHistoryBufferSizeInBytes();
    counterInfo.counterMask            = 0;
    counterInfo.counterStride          = sizeof(uint32);
    counterInfo.counterMode            = 1;
    counterInfo.counterRayIdRangeBegin = 0;
    counterInfo.counterRayIdRangeEnd   = 0xFFFFFFFF;

    if (dispatchInfo.hitGroupTable.stride > 0ULL)
    {
        counterInfo.hitGroupShaderRecordCount =
            uint32(dispatchInfo.hitGroupTable.size / dispatchInfo.hitGroupTable.stride);
    }
    if (dispatchInfo.missShaderTable.stride > 0ULL)
    {
        counterInfo.missShaderRecordCount =
            uint32(dispatchInfo.missShaderTable.size / dispatchInfo.missShaderTable.stride);
    }
    pTraceListInfo->counterInfo = counterInfo;

    DispatchDimensions dispatchDims = {};
    dispatchDims.dimX = dispatchInfo.dimX;
    dispatchDims.dimY = dispatchInfo.dimY;
    dispatchDims.dimZ = dispatchInfo.dimZ;

    pTraceListInfo->dispatchDims = dispatchDims;

    RayHistoryTraversalFlags traversalFlags = {};
    traversalFlags.boxSortMode      = dispatchInfo.boxSortMode;
    traversalFlags.usesNodePtrFlags = dispatchInfo.usesNodePtrFlags;

    pTraceListInfo->traversalFlags = traversalFlags;
}

// =====================================================================================================================
void Device::WriteShaderTableData(
    Pal::ICmdBuffer*       pCmdBuffer,
    uint64                 runningOffsetGpuVa,
    gpusize                destGpuVa,
    uint64                 runningOffset,
    void*                  pMappedData,
    ShaderTableType        shaderTableType,
    ShaderTable            shaderTable,
    ShaderTableInfo*       pOutShaderTableInfo
    )
{
    // Store ShaderTableInfo in the GPU allocation so the allocation layout matches the shader table RDF chunk.
    // It is passed directly to WriteDataChunk().
    auto* pShaderTableInfo = static_cast<ShaderTableInfo*>(Util::VoidPtrInc(pMappedData, runningOffset));
    pShaderTableInfo->type        = shaderTableType;
    pShaderTableInfo->stride      = shaderTable.stride;
    pShaderTableInfo->sizeInBytes = shaderTable.size;

    pOutShaderTableInfo->type = shaderTableType;
    pOutShaderTableInfo->stride = shaderTable.stride;
    pOutShaderTableInfo->sizeInBytes = shaderTable.size;

    Pal::MemoryCopyRegion region = {};
    region.srcOffset = 0;
    region.copySize  = shaderTable.size;

    pCmdBuffer->CmdCopyMemoryByGpuVa(
        shaderTable.addr,
        destGpuVa + runningOffsetGpuVa,
        1,
        &region);
}

// =====================================================================================================================
void Device::TraceIndirectRtDispatch(
    RtPipelineType                pipelineType,
    RtDispatchInfo                dispatchInfo,
    uint32                        maxDispatchCount,
    gpusize*                      pCounterMetadataVa,
    void*                         pIndirectConstants)
{
    const uint32 maxSrdCount = (pipelineType != GpuRt::RtPipelineType::RayTracing) ?
        Util::Min(maxDispatchCount, 1u) :
        Util::Min(maxDispatchCount, GpuRt::MaxSupportedIndirectCounters);

    const uint64 traceSizeInBytes = GetRayHistoryBufferSizeInBytes();

    gpusize destGpuVa = 0;
    ClientGpuMemHandle gpuMem = nullptr;
    void* pMappedData = nullptr;

    const gpusize bufferSize = maxSrdCount * (sizeof(IndirectCounterMetadata) + traceSizeInBytes);

    Pal::Result result = m_clientCb.pfnAllocateGpuMemory(m_info, bufferSize, &gpuMem, &destGpuVa, &pMappedData);
    if (result == Pal::Result::Success)
    {
        uint64 traceBufferGpuVa = 0;
        void* pIndirectCounterMetadata = nullptr;
        if (pCounterMetadataVa != nullptr)
        {
            // All metadata at the start then trace buffers follow
            *pCounterMetadataVa = destGpuVa;

            traceBufferGpuVa = *pCounterMetadataVa + (maxSrdCount * sizeof(IndirectCounterMetadata));

            pIndirectCounterMetadata = pMappedData;

            // Offset to trace buffer data
            pMappedData = Util::VoidPtrInc(pMappedData, (maxSrdCount * sizeof(IndirectCounterMetadata)));
        }
        else
        {
            traceBufferGpuVa = destGpuVa;
        }

        for (uint32 i = 0; i < maxSrdCount; ++i)
        {
            RayHistoryTraceListInfo traceListInfo = {};

            const uint64 runningOffset = i * traceSizeInBytes;

            Pal::BufferViewInfo viewInfo = {};
            viewInfo.gpuAddr = traceBufferGpuVa + runningOffset;
            viewInfo.range = traceSizeInBytes;

            Pal::IDevice* pPalDevice = m_info.pPalDevice;

            if (pipelineType != GpuRt::RtPipelineType::RayTracing)
            {
                auto* pConstants = static_cast<DispatchRaysConstants*>(pIndirectConstants);
                pPalDevice->CreateUntypedBufferViewSrds(1, &viewInfo, &pConstants->descriptorTable.internalUavBufferSrd[0]);

                // TraceRayCounterRayHistoryLight
                pConstants->constData.counterMode            = 1;
                pConstants->constData.counterRayIdRangeBegin = 0;
                pConstants->constData.counterRayIdRangeEnd   = 0xFFFFFFFF;

                pConstants->constData.counterMask = GetRayHistoryLightCounterMask();
            }
            else
            {
                auto* pConstants = static_cast<InitExecuteIndirectConstants*>(pIndirectConstants);
                pPalDevice->CreateUntypedBufferViewSrds(1, &viewInfo, &pConstants->internalUavSrd[i]);

                // TraceRayCounterRayHistoryLight
                pConstants->counterMode            = 1;
                pConstants->counterRayIdRangeBegin = 0;
                pConstants->counterRayIdRangeEnd   = 0xFFFFFFFF;

                pConstants->counterMask = GetRayHistoryLightCounterMask();
            }

            traceListInfo.pRayHistoryTraceBuffer             = Util::VoidPtrInc(pMappedData, runningOffset);

            // First data slot is reserved for token counts
            memset(traceListInfo.pRayHistoryTraceBuffer, 0, RAY_TRACING_COUNTER_RESERVED_BYTE_SIZE);

            // On indirect dispatches, we only allocate Gpu memory once for all trace buffers, set Gpu handle to last entry
            traceListInfo.traceBufferGpuMem                  = (i == (maxSrdCount - 1)) ? gpuMem : nullptr;

#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION >= 38
            traceListInfo.counterInfo.dispatchRayDimensionX = dispatchInfo.threadGroupSizeX;
            traceListInfo.counterInfo.dispatchRayDimensionY = dispatchInfo.threadGroupSizeY;
            traceListInfo.counterInfo.dispatchRayDimensionZ = dispatchInfo.threadGroupSizeZ;
#endif
            traceListInfo.counterInfo.counterMask            = 0;
            traceListInfo.counterInfo.counterStride          = sizeof(uint32);
            traceListInfo.counterInfo.counterMode            = 1;
            traceListInfo.counterInfo.counterRayIdRangeBegin = 0;
            traceListInfo.counterInfo.counterRayIdRangeEnd   = 0xFFFFFFFF;
            traceListInfo.counterInfo.stateObjectHash        = dispatchInfo.stateObjectHash;
            traceListInfo.counterInfo.pipelineShaderCount    = dispatchInfo.pipelineShaderCount;
            traceListInfo.counterInfo.pipelineType           = uint32(pipelineType);
            traceListInfo.counterInfo.rayCounterDataSize     = traceSizeInBytes;
            traceListInfo.traversalFlags.boxSortMode         = dispatchInfo.boxSortMode;
            traceListInfo.traversalFlags.usesNodePtrFlags    = dispatchInfo.usesNodePtrFlags;
            traceListInfo.isIndirect                         = true;

            if (pCounterMetadataVa != nullptr)
            {
                traceListInfo.pIndirectCounterMetadata =
                    static_cast<IndirectCounterMetadata*>(Util::VoidPtrInc(pIndirectCounterMetadata,
                        (i * sizeof(IndirectCounterMetadata))));
            }
            else
            {
                traceListInfo.pIndirectCounterMetadata = nullptr;
            }

            m_rayHistoryTraceList.PushBack(traceListInfo);
        }
    }
    else
    {
        // Failed to allocate memory
    }
}

// =====================================================================================================================
void Device::WriteRayHistoryChunks(
    GpuUtil::ITraceSource* pTraceSource)
{
#if PAL_BUILD_RDF
    Pal::IPlatform* pPlatform = m_info.pPalPlatform;
    GpuUtil::TraceSession* pTraceSession = pPlatform->GetTraceSession();
#endif
    GpuUtil::TraceChunkInfo info = {};
    RayHistoryRdfChunkHeader header = {};

    // HistoryTokensRaw fits exactly in the 16 characters of the ID and no null terminator is required
    const char chunkId[] = "HistoryTokensRaw";
    memcpy(info.id, chunkId, strlen(chunkId));

    info.version           = GPURT_COUNTER_VERSION;
    info.headerSize        = sizeof(GpuRt::RayHistoryRdfChunkHeader);
    info.enableCompression = true;

    uint32 traceBufferCount = m_rayHistoryTraceList.NumElements();
    for (uint32 i = 0; i < traceBufferCount; ++i)
    {
        bool isIndirect = m_rayHistoryTraceList.At(i).isIndirect;

        if (m_rayHistoryTraceList.At(i).pIndirectCounterMetadata)
        {
            auto* pIndirectCounterMetadata = m_rayHistoryTraceList.At(i).pIndirectCounterMetadata;
            uint32 dispatchSizeX = pIndirectCounterMetadata->dispatchRayDimensionX;
            uint32 dispatchSizeY = pIndirectCounterMetadata->dispatchRayDimensionY;
            uint32 dispatchSizeZ = pIndirectCounterMetadata->dispatchRayDimensionZ;

            // Multiply by the thread group size if one is provided
            if (m_rayHistoryTraceList.At(i).counterInfo.dispatchRayDimensionX != 0)
            {
                dispatchSizeX *= m_rayHistoryTraceList.At(i).counterInfo.dispatchRayDimensionX;
                dispatchSizeY *= m_rayHistoryTraceList.At(i).counterInfo.dispatchRayDimensionY;
                dispatchSizeZ *= m_rayHistoryTraceList.At(i).counterInfo.dispatchRayDimensionZ;
            }
            m_rayHistoryTraceList.At(i).counterInfo.dispatchRayDimensionX = dispatchSizeX;
            m_rayHistoryTraceList.At(i).counterInfo.dispatchRayDimensionY = dispatchSizeY;
            m_rayHistoryTraceList.At(i).counterInfo.dispatchRayDimensionZ = dispatchSizeZ;

            m_rayHistoryTraceList.At(i).dispatchDims.dimX = dispatchSizeX;
            m_rayHistoryTraceList.At(i).dispatchDims.dimY = dispatchSizeY;
            m_rayHistoryTraceList.At(i).dispatchDims.dimZ = dispatchSizeZ;
        }

        CounterInfo counterInfo = m_rayHistoryTraceList.At(i).counterInfo;

        // First two slot contain the token request and log counts
        const char* pDispatchData = static_cast<char*>(m_rayHistoryTraceList.At(i).pRayHistoryTraceBuffer);

        const uint32 tokenRequests = *(reinterpret_cast<const uint32*>(pDispatchData));
        if (tokenRequests > 0)
        {
            counterInfo.rayCounterDataSize -= RAY_TRACING_COUNTER_RESERVED_BYTE_SIZE;

            // if we lost tokens, adjust the counter size to skip partial counter dump
            if (tokenRequests > counterInfo.rayCounterDataSize)
            {
                // determine valid counter data size
                const uint32 validCounterSize =
                    Util::RoundDownToMultiple(counterInfo.rayCounterDataSize, counterInfo.counterStride);

                // Report partial tokens as lost
                counterInfo.lostTokenBytes = tokenRequests - validCounterSize;

                // Set valid counter size
                counterInfo.rayCounterDataSize = validCounterSize;
            }
            else
            {
                counterInfo.rayCounterDataSize = tokenRequests;
            }
        }
        else
        {
            counterInfo.rayCounterDataSize = 0;
        }
        m_rayHistoryTraceList.At(i).counterInfo = counterInfo;

        header = m_rayHistoryTraceList.At(i).rayHistoryRdfChunkHeader;
        info.pHeader = &header;

        info.pData = Util::VoidPtrInc(m_rayHistoryTraceList.At(i).pRayHistoryTraceBuffer,
            RAY_TRACING_COUNTER_RESERVED_BYTE_SIZE);
        info.dataSize = counterInfo.rayCounterDataSize;

#if PAL_BUILD_RDF
        pTraceSession->WriteDataChunk(pTraceSource, info);
#endif

        WriteRayHistoryMetaDataChunks(
            m_rayHistoryTraceList.At(i),
            pTraceSource);

        // Shader tables are not captured for indirect dispatches
        if (!isIndirect)
        {
            WriteShaderTableChunks(
                m_rayHistoryTraceList.At(i),
                pTraceSource);
        }

        if (isIndirect)
        {
            if (m_rayHistoryTraceList.At(i).traceBufferGpuMem != nullptr)
            {
                m_clientCb.pfnFreeGpuMem(m_info, m_rayHistoryTraceList.At(i).traceBufferGpuMem);
            }
        }
        else
        {
            m_clientCb.pfnFreeGpuMem(m_info, m_rayHistoryTraceList.At(i).traceBufferGpuMem);
        }
    }
    m_rayHistoryTraceList.Clear();
}

// =====================================================================================================================
void Device::WriteRayHistoryMetaDataChunks(
    const RayHistoryTraceListInfo& traceListInfo,
    GpuUtil::ITraceSource*         pTraceSource)
{
#if PAL_BUILD_RDF
    Pal::IPlatform* pPlatform = m_info.pPalPlatform;
    GpuUtil::TraceSession* pTraceSession = pPlatform->GetTraceSession();
#endif

    GpuUtil::TraceChunkInfo info = {};

    const char chunkId[] = "HistoryMetadata";
    memcpy(info.id, chunkId, strlen(chunkId));

    info.version           = GPURT_COUNTER_VERSION;
    info.headerSize        = sizeof(GpuRt::RayHistoryRdfChunkHeader);
    info.pHeader           = &traceListInfo.rayHistoryRdfChunkHeader;
    info.enableCompression = true;

    RayHistoryMetadata rayHistoryMetadata = {};

    rayHistoryMetadata.counterInfo.kind              = RayHistoryMetadataKind::CounterInfo;
    rayHistoryMetadata.counterInfo.sizeInByte        = sizeof(CounterInfo);
    rayHistoryMetadata.counter                       = traceListInfo.counterInfo;

    rayHistoryMetadata.dispatchDimsInfo.kind         = RayHistoryMetadataKind::DispatchDimensions;
    rayHistoryMetadata.dispatchDimsInfo.sizeInByte   = sizeof(DispatchDimensions);
    rayHistoryMetadata.dispatchDims                  = traceListInfo.dispatchDims;

    rayHistoryMetadata.traversalFlagsInfo.kind       = RayHistoryMetadataKind::TraversalFlags;
    rayHistoryMetadata.traversalFlagsInfo.sizeInByte = sizeof(RayHistoryTraversalFlags);
    rayHistoryMetadata.traversalFlags                = traceListInfo.traversalFlags;

    info.pData = &rayHistoryMetadata;
    info.dataSize = sizeof(RayHistoryMetadata);

#if PAL_BUILD_RDF
    pTraceSession->WriteDataChunk(&m_rayHistoryTraceSource, info);
#endif
}

// =====================================================================================================================
void Device::WriteShaderTableChunks(
    const RayHistoryTraceListInfo& traceListInfo,
    GpuUtil::ITraceSource*         pTraceSource)
{
#if PAL_BUILD_RDF
    Pal::IPlatform* pPlatform = m_info.pPalPlatform;
    GpuUtil::TraceSession* pTraceSession = pPlatform->GetTraceSession();
#endif
    RayHistoryRdfChunkHeader pRayHistoryRdfChunkHeader =
        traceListInfo.rayHistoryRdfChunkHeader;

    GpuUtil::TraceChunkInfo info = {};
    const char chunkId[] = "ShaderTable";
    memcpy(info.id, chunkId, strlen(chunkId));

    info.version           = GPURT_COUNTER_VERSION;
    info.headerSize        = sizeof(RayHistoryRdfChunkHeader);
    info.pHeader           = &pRayHistoryRdfChunkHeader;
    info.enableCompression = true;

    // Point to the first shader table info (RayGen)
    auto* pShaderTableInfo = static_cast<ShaderTableInfo*>(Util::VoidPtrInc(traceListInfo.pRayHistoryTraceBuffer,
        GetRayHistoryBufferSizeInBytes()));

    info.pData    = pShaderTableInfo;
    info.dataSize = (sizeof(ShaderTableInfo) * 3) +
        traceListInfo.shaderTableRayGenInfo.sizeInBytes +
        traceListInfo.shaderTableMissInfo.sizeInBytes +
        traceListInfo.shaderTableHitGroupInfo.sizeInBytes;

#if PAL_BUILD_RDF
    pTraceSession->WriteDataChunk(pTraceSource, info);
#endif
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
DataDriverMatchingIdentifierStatus Device::CheckSerializedAccelStructVersion(
    const DataDriverMatchingIdentifier* pIdentifier)
{
    PAL_ASSERT(pIdentifier != nullptr);

    const uint32 amdGUID[4] =
    {
        GPURT_AMD_GUID_0,
        GPURT_AMD_GUID_1,
        GPURT_AMD_GUID_2,
        GPURT_AMD_GUID_3
    };

    DataDriverMatchingIdentifierStatus status = DataDriverMatchingIdentifierStatus::CompatibleWithDevice;
    if (memcmp(pIdentifier->driverOpaqueGUID, amdGUID, sizeof(amdGUID)) != 0)
    {
        status = DataDriverMatchingIdentifierStatus::Unrecognized;
    }
    else
    {
        // driverOpaqueVersioningData:
        //  0 - 31: uuidLo
        // 32 - 63: uuidHi
        // 64 - 79: GPURT_ACCEL_STRUCT_MINOR_VERSION
        // 80 - 95: GPURT_ACCEL_STRUCT_MAJOR_VERSION
        // 96 -127: 0
        uint64 uuid = *(reinterpret_cast<const uint64*>(pIdentifier->driverOpaqueVersioningData));
        uint32 version = *(reinterpret_cast<const uint32*>(pIdentifier->driverOpaqueVersioningData + sizeof(uint64)));
        if ((uuid != m_info.deviceSettings.accelerationStructureUUID) ||
            ((version >> 16) != GPURT_ACCEL_STRUCT_MAJOR_VERSION))
        {
            status = DataDriverMatchingIdentifierStatus::IncompatibleVersion;
        }
    }
    return status;
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
    const gpusize*               pGpuVas,
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
    case GpuRt::AccelStructPostBuildInfoType::BottomLevelASPointerCount:
        // This mode is not used with this function
        PAL_ASSERT_ALWAYS();
        break;
    }

    uint64 sizeInBytes = 0;
    uint64 allocationSizeInBytes = count * infoSize;

    gpusize destGpuVa = 0;
    ClientGpuMemHandle gpuMem = nullptr;
    AccelStructPostBuildInfoToolsVisualizationDesc* pInfo = nullptr;

    Pal::Result result = Pal::Result::Unsupported;

    result = m_clientCb.pfnAllocateGpuMemory(
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

        result = m_clientCb.pfnAcquireCmdContext(m_info, &context, &pCmdBuffer);

        if (result == Pal::Result::Success)
        {
            EmitAccelStructPostBuildInfo(pCmdBuffer, postBuildInfo);

            result = m_clientCb.pfnFlushCmdContext(context);

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

    m_clientCb.pfnFreeGpuMem(m_info, gpuMem);
    return result;
}

// =====================================================================================================================
void Device::BuildAccelStruct(
    Pal::ICmdBuffer*              pCmdBuffer,
    const AccelStructBuildInfo&   buildInfo
)
{
    PAL_ASSERT(pCmdBuffer != nullptr);

    DeviceSettings deviceSettings;
    ClientCallbacks clientCb = {};

    SetUpClientCallbacks(&clientCb);

    deviceSettings = m_info.deviceSettings;

    pCmdBuffer->CmdSaveComputeState(Pal::ComputeStateAll);

    BvhBuilder builder(pCmdBuffer,
                       this,
                       *m_info.pDeviceProperties,
                       clientCb,
                       deviceSettings,
                       buildInfo);

    builder.BuildRaytracingAccelerationStructure();

    pCmdBuffer->CmdRestoreComputeState(Pal::ComputeStateAll);
}

// =====================================================================================================================
void Device::BuildAccelStructs(
    Pal::ICmdBuffer*                       pCmdBuffer,
    Util::Span<const AccelStructBuildInfo> buildInfo)
{
    PAL_ASSERT(pCmdBuffer != nullptr);

    ClientCallbacks clientCb = {};
    SetUpClientCallbacks(&clientCb);

    pCmdBuffer->CmdSaveComputeState(Pal::ComputeStateAll);

    BvhBatcher batcher(pCmdBuffer,
                       this,
                       *m_info.pDeviceProperties,
                       clientCb,
                       m_info.deviceSettings);

    batcher.BuildAccelerationStructureBatch(buildInfo);

    pCmdBuffer->CmdRestoreComputeState(Pal::ComputeStateAll);
}

// =====================================================================================================================
void Device::EmitAccelStructPostBuildInfo(
    Pal::ICmdBuffer*                pCmdBuffer,
    const AccelStructPostBuildInfo& postBuildInfo
)
{
    PAL_ASSERT(pCmdBuffer != nullptr);

    DeviceSettings deviceSettings;
    ClientCallbacks clientCb = {};

    SetUpClientCallbacks(&clientCb);

    deviceSettings = m_info.deviceSettings;

    pCmdBuffer->CmdSaveComputeState(Pal::ComputeStateAll);

    BvhBuilder builder(pCmdBuffer,
                       this,
                       *m_info.pDeviceProperties,
                       clientCb,
                       deviceSettings);

    builder.EmitAccelerationStructurePostBuildInfo(postBuildInfo);

    pCmdBuffer->CmdRestoreComputeState(Pal::ComputeStateAll);
}

// =====================================================================================================================
void Device::CopyAccelStruct(
    Pal::ICmdBuffer*              pCmdBuffer,
    const AccelStructCopyInfo&    copyInfo
)
{
    PAL_ASSERT(pCmdBuffer != nullptr);

    DeviceSettings deviceSettings;
    ClientCallbacks clientCb = {};

    SetUpClientCallbacks(&clientCb);

    deviceSettings = m_info.deviceSettings;

    pCmdBuffer->CmdSaveComputeState(Pal::ComputeStateAll);

    BvhBuilder builder(pCmdBuffer,
                       this,
                       *m_info.pDeviceProperties,
                       clientCb,
                       deviceSettings);

    builder.CopyAccelerationStructure(copyInfo);

    pCmdBuffer->CmdRestoreComputeState(Pal::ComputeStateAll);
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

    Pal::PipelineBindParams bindParams = {};
    bindParams.pipelineBindPoint = Pal::PipelineBindPoint::Compute;
    bindParams.pPipeline = GetInternalPipeline(InternalRayTracingCsType::InitExecuteIndirect, {}, 0);
    bindParams.apiPsoHash = GetInternalPsoHash(InternalRayTracingCsType::InitExecuteIndirect, {});

    pCmdBuffer->CmdBindPipeline(bindParams);

    const uint32 threadGroupDim = 8;
#if PAL_CLIENT_INTERFACE_MAJOR_VERSION < 771
    pCmdBuffer->CmdDispatch(Util::RoundUpQuotient(maxDispatchCount, threadGroupDim),
                            Util::RoundUpQuotient(pipelineCount, threadGroupDim),
                            1);
#else
    pCmdBuffer->CmdDispatch({ Util::RoundUpQuotient(maxDispatchCount, threadGroupDim),
                              Util::RoundUpQuotient(pipelineCount, threadGroupDim),
                              1});
#endif

    pCmdBuffer->CmdRestoreComputeState(Pal::ComputeStateAll);
}

// =====================================================================================================================
// Executes the copy buffer shader
void Device::CopyBufferRaw(
    Pal::ICmdBuffer* pCmdBuffer,
    gpusize          dstBufferVa,    // Destination buffer GPU VA
    gpusize          srcBufferVa,    // Source buffer GPU VA
    uint32           numDwords)      // Number of Dwords to copy
{
#if GPURT_DEVELOPER
    OutputPipelineName(pCmdBuffer, InternalRayTracingCsType::CopyBufferRaw);
#endif

    Pal::PipelineBindParams bindParams = {};
    bindParams.pipelineBindPoint = Pal::PipelineBindPoint::Compute;
    bindParams.pPipeline = GetInternalPipeline(InternalRayTracingCsType::CopyBufferRaw, {}, 0);
    bindParams.apiPsoHash = GetInternalPsoHash(InternalRayTracingCsType::CopyBufferRaw, {});

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
#if PAL_CLIENT_INTERFACE_MAJOR_VERSION < 771
    pCmdBuffer->CmdDispatch(dispatchSize, 1, 1);
#else
    pCmdBuffer->CmdDispatch({ dispatchSize, 1, 1 });
#endif

    RGP_POP_MARKER(pCmdBuffer);
}

// =====================================================================================================================
// Uploads CPU memory to a GPU buffer
void Device::UploadCpuMemory(
    Pal::ICmdBuffer* pCmdBuffer,
    gpusize          dstBufferVa,
    const void*      pSrcData,
    uint32           sizeInBytes)
{
    const uint32 embeddedDataLimitDwords = pCmdBuffer->GetEmbeddedDataLimit();
    const uint32 uploadSizeDwords = Util::Pow2Align(sizeInBytes, 4);
    PAL_ASSERT(uploadSizeDwords <= embeddedDataLimitDwords);

    gpusize srcGpuVa = 0;
    uint32* pMappedData = pCmdBuffer->CmdAllocateEmbeddedData(uploadSizeDwords, 1, &srcGpuVa);
    std::memcpy(pMappedData, pSrcData, sizeInBytes);

    Pal::MemoryCopyRegion region{};
    region.srcOffset = 0;
    region.dstOffset = 0;
    region.copySize = sizeInBytes;
    pCmdBuffer->CmdCopyMemoryByGpuVa(srcGpuVa, dstBufferVa, 1, &region);
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
    gpusize          virtualAddress, // GPUVA of the buffer
    uint32           entryOffset)    // Offset of the first entry
{
    const uint32 entries[] = { Util::LowPart(virtualAddress), Util::HighPart(virtualAddress) };
    return WriteUserDataEntries(pCmdBuffer, entries, GPURT_ARRAY_SIZE(entries), entryOffset);
}

// =====================================================================================================================
// Creates one or more Typed Buffer View SRDs on the device.
void Device::CreateTypedBufferViewSrds(
    uint32                     count,
    const Pal::BufferViewInfo* pBufferViewInfo,
    void* pOut)
{
    const Pal::IDevice* pDevice = m_info.pPalDevice;
    pDevice->CreateTypedBufferViewSrds(count, pBufferViewInfo, pOut);
}

// =====================================================================================================================
// Performs a generic barrier that's used to synchronize internal ray tracing shaders
void Device::RaytracingBarrier(
    Pal::ICmdBuffer* pCmdBuffer)
{
    if (m_info.deviceSettings.enableAcquireReleaseInterface)
    {
        Pal::AcquireReleaseInfo acqRelInfo = {};
        acqRelInfo.srcGlobalStageMask  = Pal::PipelineStageCs;
        acqRelInfo.dstGlobalStageMask  = Pal::PipelineStageCs;
        acqRelInfo.srcGlobalAccessMask = Pal::CoherShader;
        acqRelInfo.dstGlobalAccessMask = Pal::CoherShader;

        acqRelInfo.reason = m_info.deviceSettings.rgpBarrierReason;

        pCmdBuffer->CmdReleaseThenAcquire(acqRelInfo);
    }
    else
    {
        Pal::BarrierInfo barrierInfo = {};
        barrierInfo.waitPoint = Pal::HwPipePreCs;

        const Pal::HwPipePoint pipePoint = Pal::HwPipePostCs;
        barrierInfo.pipePointWaitCount = 1;
        barrierInfo.pPipePoints = &pipePoint;

        Pal::BarrierTransition transition = {};
        transition.srcCacheMask = Pal::CoherShader;
        transition.dstCacheMask = Pal::CoherShader;

        barrierInfo.transitionCount = 1;
        barrierInfo.pTransitions = &transition;

        barrierInfo.reason = m_info.deviceSettings.rgpBarrierReason;

        pCmdBuffer->CmdBarrier(barrierInfo);
    }
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

    m_clientCb.pfnInsertRGPMarker(pCmdBuffer, strBuffer, true);
}

// =====================================================================================================================
void Device::PopRGPMarker(Pal::ICmdBuffer* pCmdBuffer)
{
    m_clientCb.pfnInsertRGPMarker(pCmdBuffer, nullptr, false);
}

// =====================================================================================================================
// Driver generated RGP markers
void Device::OutputPipelineName(
    Pal::ICmdBuffer*         pCmdBuffer,
    InternalRayTracingCsType type) const
{
    constexpr uint32 MaxStrLength = 256;
    char buildShaderInfo[MaxStrLength];
    const PipelineBuildInfo* pPipelineBuildInfo = &InternalPipelineBuildInfo[static_cast<uint32>(type)];
    Util::Snprintf(buildShaderInfo, MaxStrLength, "BVH Build Pipeline: %s", pPipelineBuildInfo->pPipelineName);

    pCmdBuffer->CmdCommentString(buildShaderInfo);
}
#endif
}   // namespace Internal
} // namespace GpuRt
