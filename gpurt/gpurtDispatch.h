/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _GPURT_DISPATCH_H
#define _GPURT_DISPATCH_H

// This file provides the definition of the GPURT structures used by the GPURT traversal shader code.
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

#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION >= 47
constexpr uint32 MaxBufferSrdSize = 8;
#else
constexpr uint32 MaxBufferSrdSize = 4;
#endif

#define DISPATCHRAYSCONSTANTDATA_STRUCT_OFFSET_DISPATCHID  48

// Dispatch rays constant buffer data (GPU structure). Note, using unaligned uint64_t in HLSL constant buffers requires
// -no-legacy-cbuf-layout for cpp style structure alignment to work. But currently that support is incomplete in DXC
// and until that is resolved we need to use uint32's explicitly.
//
#if __cplusplus
#pragma pack(push, 4)
#endif
struct DispatchRaysConstantData
{
    uint32 rayGenerationTableAddressLo; // Ray generation table base address low 32-bits
    uint32 rayGenerationTableAddressHi; // Ray generation table base address high 32-bits
    uint32 rayDispatchWidth;            // Width of the ray dispatch
    uint32 rayDispatchHeight;           // Height of the ray dispatch
    uint32 rayDispatchDepth;            // Depth of the ray dispatch
    uint32 missTableBaseAddressLo;      // Miss shader table base address low 32-bits
    uint32 missTableBaseAddressHi;      // Miss shader table base address high 32-bits
    uint32 missTableStrideInBytes;      // Miss shader table record byte stride
    uint32 rayDispatchMaxGroups;        // Max groups dispatched if persistent launch is enabled, else 0
    uint32 hitGroupTableBaseAddressLo;  // Hit group table base address low 32-bits
    uint32 hitGroupTableBaseAddressHi;  // Hit group table base address high 32-bits
    uint32 hitGroupTableStrideInBytes;  // Hit group table record byte stride
    uint32 cpsDispatchId;               // Continuations DispatchId, written in the persistent mode.
                                        // This value should not be read via constant buffer.
    uint32 callableTableBaseAddressLo;  // Callable shader table base address low 32-bits
    uint32 callableTableBaseAddressHi;  // Callable shader table base address high 32-bits
    uint32 callableTableStrideInBytes;  // Callable shader table byte stride
    uint32 profileRayFlags;             // Ray flags for profiling
    uint32 profileMaxIterations;        // Maximum traversal iterations for profiling
    uint32 traceRayGpuVaLo;             // Traversal shader (shader table) base address low 32-bits
    uint32 traceRayGpuVaHi;             // Traversal shader (shader table) base address high 32-bits
    uint32 counterMode;                 // Counter capture mode. see TraceRayCounterMode
    uint32 counterRayIdRangeBegin;      // Counter capture ray ID range begin
    uint32 counterRayIdRangeEnd;        // Counter capture ray ID range end
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 36
    uint32 cpsStackOffsetInBytes;       // The scratch memory used as stacks are divided into two parts:
                                        //  (a) Used by a compiler backend, start at offset 0.
#else
    uint32 cpsBackendStackSize;         // The scratch memory used as stacks are divided into two parts:
                                        //  (a) Used by a compiler backend, start at offset 0.
#endif
    uint32 cpsFrontendStackSize;        //  (b) Used by IR (Intermediate Representation), for a continuation passing shader.
    uint32 cpsGlobalMemoryAddressLo;    // Separate CPS stack memory base address low 32-bits
    uint32 cpsGlobalMemoryAddressHi;    // Separate CPS stack memory base address high 32-bits
    uint32 counterMask;                 // Mask for filtering ray history token
    uint32 cpsDispatchIdAddressLo;      // Continuations cpsDispatchId address low 32-bits
    uint32 cpsDispatchIdAddressHi;      // Continuations cpsDispatchId address high 32-bits
};
#if __cplusplus
#pragma pack(pop)
#endif

// Dispatch rays arguments top-level descriptor table (GPU structure)
struct DispatchRaysTopLevelData
{
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 56
    uint64                   dispatchRaysConstGpuVa;                  // DispatchRays info constant buffer GPU VA
#else
    DispatchRaysConstantData constData;                               // Dispatch rays constant buffer data
#endif
    uint32                   internalUavBufferSrd[MaxBufferSrdSize];  // Internal UAV shader resource descriptor
    uint32                   accelStructTrackerSrd[MaxBufferSrdSize]; // Structured buffer SRD pointing to the accel struct tracker
};

// GPU structure containing all data for DXR/VK ray dispatch command
struct DispatchRaysConstants
{
    DispatchRaysTopLevelData descriptorTable;  // Top-level internal dispatch bindings (includes pointer to infoData)
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 56
    DispatchRaysConstantData constData;        // Dispatch rays constant buffer data
#endif
};

#if __cplusplus
static_assert((sizeof(DispatchRaysConstants) % sizeof(uint32)) == 0,
              "DispatchRaysConstants is not dword-aligned");
static_assert(DISPATCHRAYSCONSTANTDATA_STRUCT_OFFSET_DISPATCHID == offsetof(DispatchRaysConstantData, cpsDispatchId),
              "DISPATCHRAYSCONSTANTDATA_STRUCT_OFFSET_DISPATCHID mismatches to cpsDispatchId");

constexpr uint32 DispatchRaysConstantsDw = sizeof(DispatchRaysConstants) / sizeof(uint32);
#endif

constexpr uint32 MaxSupportedIndirectCounters = 8;

// Resource bindings required for InitExecuteIndirect
struct InitExecuteIndirectUserData
{
    uint64 constantsVa;         // InitExecuteIndirectConstants struct
    uint64 countVa;             // Count buffer (number of indirect dispatches)
    uint64 raygenIdsVa;         // Array of raygen shader IDs for unified pipelines
    uint64 inputBufferVa;       // Input indirect args specified by the application
    uint64 outputBufferVa;      // Output indirect arg buffer consumed by indirect dispatch and/or command generator
    uint64 outputConstantsVa;   // DispatchRaysConstantData for each dispatch
    uint64 outputDescriptorsVa; // Internal descriptor tables with DispatchRays CBV
    uint64 outputCounterMetaVa; // Counter metadata UAV. Must be valid when
                                // InitExecuteIndirectConstants.counterMode != 0
};

// Constants for InitExecuteIndirect shader
struct InitExecuteIndirectConstants
{
#if __cplusplus
    // Internal counter buffer SRDs
    uint32 internalUavSrd[MaxSupportedIndirectCounters][MaxBufferSrdSize];

    // Internal acceleration structure tracker buffer SRD.
    uint32 accelStructTrackerSrd[MaxBufferSrdSize];
#else
    uint4 internalUavSrd[MaxSupportedIndirectCounters][MaxBufferSrdSize / 4];
    uint4 accelStructTrackerSrd[MaxBufferSrdSize / 4];
#endif

    uint32 inputBytesPerDispatch;   // Size of application indirect arguments
    uint32 outputBytesPerDispatch;  // Size of resulting driver internal arguments
    uint32 bindingArgsSize;         // Size of binding arguments in the app buffer preceeding the dispatch
    uint32 maxDispatchCount;        // Max number of dispatches requested by the app
    uint32 indirectMode;            // 0: vkCmdTraceRaysIndirectKHR - ray trace query dimensions
                                    // 1: vkCmdTraceRaysIndirect2KHR- shaderTable + ray trace query dimensions
    uint32 dispatchDimSwizzleMode;  // Swizzle mode for mapping user specified trace ray dimension to internal
                                    // dispatch dimension.
                                    // 0: native mapping (width -> x, height -> y, depth -> z)
                                    // 1: flatten width and height to x, and depth to y. Thread group is 1D
                                    // and defined by rtThreadGroupSizeX.
    uint32 rtThreadGroupSizeX;      // Internal RT threadgroup size X
    uint32 rtThreadGroupSizeY;      // Internal RT threadgroup size Y
    uint32 rtThreadGroupSizeZ;      // Internal RT threadgroup size Z
    uint32 rayDispatchMaxGroups;    // Max groups dispatched if persistent launch is enabled, else 0
    uint32 counterMask;             // Mask for filtering ray history token
    uint32 pipelineCount;           // Number of pipelines to launch (1 for indirect launch, raygen count for unified)
    uint32 maxIterations;           // Max traversal interations for profiling
    uint32 profileRayFlags;         // Profiling flags
    uint32 traceRayFunctionAddrLo;  // Address of trace indirect function
    uint32 traceRayFunctionAddrHi;  // Address of trace indirect function
    uint32 internalTableVaLo;       // Address of the internal descriptor tables
    uint32 outputConstantsVaLo;     // Address of the output constant buffer for initializing the descriptor table
    uint32 outputConstantsVaHi;     // Address of the output constant buffer for initializing the descriptor table
    uint32 counterMode;             // Counter mode
    uint32 counterRayIdRangeBegin;  // Counter ray ID range begin
    uint32 counterRayIdRangeEnd;    // Counter ray ID range end
    uint32 cpsBackendStackSize;     // Scratch memory used by a compiler backend, start at offset 0
    uint32 cpsFrontendStackSize;    // Scratch memory used by IR (Intermediate Representation), for a continuation passing shader

    uint32 cpsGlobalMemoryAddressLo;    // Separate CPS stack memory base address low 32-bits
    uint32 cpsGlobalMemoryAddressHi;    // Separate CPS stack memory base address high 32-bits
};

// Resource bindings required for PrepareShadowSbtForReplay
struct PrepareShadowSbtForReplayUserData
{
    uint64 constantsVa;                  // PrepareShadowSbtForReplayConstants struct
    uint64 shadowRayGenerationTableVa;   // Shadow ray generation table
    uint64 shadowHitGroupTableVa;        // Shadow hit group table
    uint64 shadowMissTableVa;            // Shadow miss shader table
    uint64 shadowCallableTableVa;        // Shadow callable shader table
    uint64 captureReplayMappingBufferVa; // Capture replay mapping buffer
};

// Constants for PrepareShadowSbtForReplay shader
struct PrepareShadowSbtForReplayConstants
{
    uint32 hitGroupTableStrideInBytes;           // Hit group table record byte stride
    uint32 hitGroupTableEntryCount;              // Hit group table entry count
    uint32 missTableStrideInBytes;               // Miss shader table record byte stride
    uint32 missTableEntryCount;                  // Miss shader table entry count
    uint32 callableTableStrideInBytes;           // Callable shader table record byte stride
    uint32 callableTableEntryCount;              // Callable shader table entry count
    uint32 captureReplayMappingBufferEntryCount; // Capture replay mapping buffer entry count
};

struct CaptureReplayMappingBufferEntry
{
    uint32 capturedVa;
    uint32 replayVa;
};

constexpr uint32 InitExecuteIndirectConstantsDw = sizeof(InitExecuteIndirectConstants) / sizeof(uint32);
constexpr uint32 PrepareShadowSbtForReplayConstantsDw = sizeof(PrepareShadowSbtForReplayConstants) / sizeof(uint32);

#if __cplusplus
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION >= 47
static_assert((MaxBufferSrdSize == 8), "Buffer SRD size changed, affected shaders and constants need update");
#else
static_assert((MaxBufferSrdSize == 4), "Buffer SRD size changed, affected shaders and constants need update");
#endif
static_assert((sizeof(InitExecuteIndirectConstants) % sizeof(uint32)) == 0,
              "InitExecuteIndirectConstants is not dword-aligned");
}       // namespace GpuRt
#endif

#endif
