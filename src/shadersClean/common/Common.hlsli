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
//
// For matrix functions based on Microsoft's D3D12RaytracingFallback:
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.

#ifndef _COMMON_HLSL
#define _COMMON_HLSL

#include "InstanceDesc.hlsli"
#include "../../../gpurt/gpurtAccelStruct.h"

// Added as cpp doesn't allow for the `StructType s = (StructType)0;` pattern
#ifdef __cplusplus
#define INIT_VAR(type, name) type name = {}
#else
#define INIT_VAR(type, name) type name = (type)0
#endif

typedef AccelStructDataOffsets AccelStructOffsets;

//=====================================================================================================================
// static definitions

#ifndef UINT32_MAX
#define UINT32_MAX 0xFFFFFFFFu
#endif
#ifndef UINT64_MAX
#define UINT64_MAX 0xFFFFFFFFFFFFFFFFull
#endif
#ifndef INT_MAX
#define INT_MAX 0x7FFFFFFF
#endif
#ifndef FLT_MAX
#define FLT_MAX 3.402823466e+38F
#endif

#ifndef FLT_MIN
#define FLT_MIN 1.175494351e-38F
#endif

// Note: log(+/-0) always produces -inf. Reverse polarity here
#ifndef INFINITY
#define INFINITY -log(0.0)
#endif

#define INVALID_IDX       0xffffffff

// Node pointer values with special meanings
#define INVALID_NODE            0xffffffff
#define TERMINAL_NODE           0xfffffffe
#define SKIP_0_3                0xfffffffd
#define SKIP_4_7                0xfffffffb
#define SKIP_0_7                0xfffffff9
#define END_SEARCH              0xfffffff8

#include "Extensions.hlsli"
#include "Math.hlsli"
#include "BoundingBox.hlsli"
#include "NodePointers.hlsli"

#ifdef __cplusplus
static const float NaN = std::numeric_limits<float>::quiet_NaN();
#else
static const float NaN =  (0.0 / 0.0);
#endif

#define FLAG_IS_SET(x, flagName)   (((x) & (1ull << (flagName##_SHIFT))) != 0)
#define FLAG_IS_CLEAR(x, flagName) (FLAG_IS_SET(x, flagName) == 0)

//=====================================================================================================================
#define RAY_FLAG_NONE                            0x00
#define RAY_FLAG_FORCE_OPAQUE                    0x01
#define RAY_FLAG_FORCE_NON_OPAQUE                0x02
#define RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH 0x04
#define RAY_FLAG_SKIP_CLOSEST_HIT_SHADER         0x08
#define RAY_FLAG_CULL_BACK_FACING_TRIANGLES      0x10
#define RAY_FLAG_CULL_FRONT_FACING_TRIANGLES     0x20
#define RAY_FLAG_CULL_OPAQUE                     0x40
#define RAY_FLAG_CULL_NON_OPAQUE                 0x80
#define RAY_FLAG_SKIP_TRIANGLES                  0x100
#define RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES      0x200

#if !defined(__cplusplus)
#include "../build/BuildSettings.hlsli"
#include "../debug/Debug.hlsli"
#endif

#define HIT_KIND_TRIANGLE_FRONT_FACE 0xFE
#define HIT_KIND_TRIANGLE_BACK_FACE  0xFF
#define HIT_KIND_EARLY_RAY_TERMINATE 0x100

#define HIT_STATUS_IGNORE                0
#define HIT_STATUS_ACCEPT                1
#define HIT_STATUS_ACCEPT_AND_END_SEARCH 2

#define GEOMETRY_TYPE_TRIANGLES                0
#define GEOMETRY_TYPE_AABBS                    1
#define GEOMETRY_TYPE_COMPRESSED_TRIANGLES     2
#define GEOMETRY_TYPE_COMPRESSED_TRIANGLES_OMM 3

#define DDI_BUILD_FLAG_NONE              0x00
#define DDI_BUILD_FLAG_ALLOW_UPDATE      0x01
#define DDI_BUILD_FLAG_ALLOW_COMPACTION  0x02
#define DDI_BUILD_FLAG_PREFER_FAST_TRACE 0x04
#define DDI_BUILD_FLAG_PREFER_FAST_BUILD 0x08
#define DDI_BUILD_FLAG_MINIMIZE_MEMORY   0x10
#define DDI_BUILD_FLAG_PERFORM_UPDATE    0x20

#define D3D12_RAYTRACING_INSTANCE_FLAG_NONE                             0x0
#define D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_CULL_DISABLE            0x1
#define D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_FRONT_COUNTERCLOCKWISE  0x2
#define D3D12_RAYTRACING_INSTANCE_FLAG_FORCE_OPAQUE                     0x4
#define D3D12_RAYTRACING_INSTANCE_FLAG_FORCE_NON_OPAQUE                 0x8

#define D3D12_RAYTRACING_GEOMETRY_FLAG_NONE                             0x0
#define D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE                           0x1
#define D3D12_RAYTRACING_GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION   0x2

// Workaround for newer DXC, which aborts with error when firstbit*() is used with 64-bit type. That is because
// officially SPIR-V spec limits FindUMsb/FindSMsb to 32-bit width components. But internally we already support
// FindUMsb/FindSMsb/FindILsb with 64-bit type, so use GL_EXT_spirv_intrinsics to bypass the check.
[[vk::ext_instruction(/* FindUMsb */ 75, "GLSL.std.450")]]
uint spirv_FindUMsb(uint64_t value);

[[vk::ext_instruction(/* FindILsb */ 73, "GLSL.std.450")]]
uint spirv_FindILsb64(uint64_t value);

[[vk::ext_instruction(/* FindILsb */ 73, "GLSL.std.450")]]
uint spirv_FindILsb16(uint16_t value);

#define FIRSTBITHIGH_U64(val) spirv_FindUMsb(val)
#define FIRSTBITLOW_U64(val) spirv_FindILsb64(val)
#define FIRSTBITLOW_U16(val) spirv_FindILsb16(val)

//=====================================================================================================================
// The following functions depend on static flags
static bool UseUnbiasedOrigin();

static uint IsBvhRebraid();

static uint EnableAccelStructTracking();

static uint EnableFusedInstanceNodes();

#if GPURT_BUILD_RTIP3
static uint IsBvhHighPrecisionBoxNodeEnabled();

static uint IsBvh8();

#if GPURT_BUILD_RTIP3_1
static uint IsOrientedBoundingBoxesEnabled();
#endif
#endif

#if DEVELOPER
static bool EnableTraversalCounter();
#endif

//=====================================================================================================================
// Helper function to replace HLSL "all" intrinsic since it does not work well on SPIRV path in DXC
static bool all_equal(
    float3 v0,
    float3 v1);

//=====================================================================================================================
static GpuVirtualAddress MakeGpuVirtualAddress(uint lowBits, uint highBits);

//=====================================================================================================================
static GpuVirtualAddress MakeGpuVirtualAddress(uint2 lowHigh);

//=====================================================================================================================
static GpuVirtualAddress ExtractInstanceAddr(uint64_t instanceBasePointer);

//=====================================================================================================================
static GpuVirtualAddress GetInstanceAddr(in InstanceDesc desc);

//=====================================================================================================================
static GpuVirtualAddress GetInstanceAddr(in uint instanceBasePointerLo, in uint instanceBasePointerHi);

//=====================================================================================================================
static uint64_t GetNodeAddr(uint64_t baseNodePtr, uint nodePtr);

//=====================================================================================================================
static uint64_t GetInstanceBasePointer(in InstanceDesc desc);

//=====================================================================================================================
static uint64_t PackInstanceBasePointer(GpuVirtualAddress instanceVa, uint instanceFlags, uint geometryType);

//======================================================================================================================
// Produces a node pointer at the provided address with API ray flags encoded into the appropriate node pointer flags.
static uint64_t EncodeBasePointer(
    uint64_t accelStructAddr,
    uint     rayFlags);

//=====================================================================================================================
static bool NodeTypeIsBoxNode32(uint nodeType);

//=====================================================================================================================
static bool IsBoxNode32(uint pointer);

//=====================================================================================================================
static bool NodeTypeIsBoxNode16(uint nodeType);

//=====================================================================================================================
static bool IsBoxNode16(uint pointer);

#if GPURT_BUILD_RTIP3
//=====================================================================================================================
static bool NodeTypeIsHighPrecisionBoxNode64(uint nodeType);

//=====================================================================================================================
static bool IsHighPrecisionBoxNode64(uint pointer);

//=====================================================================================================================
static bool NodeTypeIsHighPrecisionBoxNode64x2(uint nodeType);

//=====================================================================================================================
static bool IsHighPrecisionBoxNode64x2(uint pointer);

//=====================================================================================================================
static bool NodeTypeIsBoxNode32x2(uint nodeType);

//=====================================================================================================================
static bool IsBoxNode32x2(uint pointer);

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
static bool NodeTypeIsQuantizedBVH8BoxNode(uint nodeType);

//=====================================================================================================================
static bool IsQuantizedBVH8BoxNode(uint pointer);
#endif
#endif

//=====================================================================================================================
static bool NodeTypeIsBoxNode(
    uint nodeType
#if GPURT_BUILD_RTIP3
  , bool highPrecisionBoxNodeEnable,
    bool bvh8Enable
#if GPURT_BUILD_RTIP3_1
  , bool enableCompressedFormat
#endif
#endif
);

//=====================================================================================================================
static bool IsBoxNode(
    uint pointer
#if GPURT_BUILD_RTIP3
  , bool highPrecisionBoxNodeEnable,
    bool bvh8Enable
#if GPURT_BUILD_RTIP3_1
  , bool enableCompressedFormat
#endif
#endif
);

//=====================================================================================================================
static uint CreateRootNodePointer(
#if GPURT_BUILD_RTIP3
    bool highPrecisionBoxNodeEnable,
    bool bvh8Enable
#if GPURT_BUILD_RTIP3_1
  , bool enableCompressedFormat
#endif
#endif
);

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
static uint CreateRootNodePointer3_1();
#endif

//=====================================================================================================================
static bool IsTriangleNode1_1(
    uint nodePtr);

static bool IsBoxNode1_1(
    uint nodePtr);

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
static bool NodeTypeIsTriangleNode3_1(uint nodeType);

//=====================================================================================================================
static bool IsTriangleNode3_1(
    uint nodePtr);

//=====================================================================================================================
static bool NodeTypeIsBoxNode3_1(
    uint nodeType);

//=====================================================================================================================
static bool IsBoxNode3_1(
    uint nodePtr);
#endif

//=====================================================================================================================
static bool NodeTypeIsUserNodeInstance(uint nodeType);

//=====================================================================================================================
static bool IsUserNodeInstance(uint nodePtr);

//=====================================================================================================================
static bool IsUserNodeProcedural(uint nodePtr);

//=====================================================================================================================
// Get internal BVH node size in bytes
static uint GetBvhNodeSizeLeaf(
    uint primitiveType,
    uint enableFusedInstanceNode);

//=====================================================================================================================
static bool CheckHandleTriangleNode(in uint pointerOrType);

//=====================================================================================================================
static bool CheckHandleProceduralUserNode(in uint nodePointer);

//=====================================================================================================================
static uint3 ComputeQuadTriangleVertexIndex(
    uint triangleIndex, // Numeric constant (0 or 1)
    uint rotation);

//=====================================================================================================================
static uint WriteTriangleIdField(uint triangleId, uint nodeType, uint rotation, uint boxNodeFlags);

//=====================================================================================================================
// Extract the order of the triangle vertices from the node's triangle ID field.
static uint3 CalcTriangleCompressionVertexIndices(
    uint nodeType,
    uint triangleId);

//=====================================================================================================================
// Extract the order of the triangle vertices from the node's triangle ID field.
static uint3 CalcTriangleCompressionVertexOffsets(uint nodeType, uint triangleId);

//=====================================================================================================================
// Gets the offsets for a node type's vertex slots. This function does not account for the rotation information stored
// in the node's triangle ID field (see CalcTriangleCompressionVertexOffsets), so it should only be used when the vertex
// order does not matter (e.g. for generating a bounding box).
static uint3 CalcTriangleVertexOffsets(uint nodeType);

//=====================================================================================================================
static bool IsDegenerateTriangle(TriangleData tri);

//=====================================================================================================================
static bool IsUpdateAllowed();

//=====================================================================================================================
static bool IsMortonSizeBitsEnabled(uint numSizeBits);

//=====================================================================================================================
static bool IsCentroidMortonBoundsEnabled();

//=====================================================================================================================
static bool IsConciseMortonBoundsEnabled();

//=====================================================================================================================
static bool IsCubeMortonBoundsEnabled();

//=====================================================================================================================
static bool IsPerfectRectangleMortonBoundsEnabled();

//=====================================================================================================================
static bool IsRegularMortonCodeEnabled();

//=====================================================================================================================
// HLSL implementation of OpenCL clz. This function counts the number of leading 0's from MSB
static int clz(int value);

//=====================================================================================================================
static int clz64(uint64_t value);

//=====================================================================================================================
// HLSL emulation of min3. We can use intrinsics here if available
static float min3(float3 val);

//=====================================================================================================================
// HLSL emulation of max3. We can use intrinsics here if available
static float max3(float3 val);

//=====================================================================================================================
// Ballot returning a uint64
static uint64_t WaveActiveBallot64(
    bool flag);

//=====================================================================================================================
static bool IsUpdate();

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
// Transform a ray for a given OBB transform. TODO: Remove, opt for refactor of InstanceTransform instead.
static void OBBTransform(
    in  float3x3      transform,
    in  float3        origin,
    in  float3        direction,
    out_param(float3) newOrigin,
    out_param(float3) newDirection);
#endif

//=====================================================================================================================
// Transform a ray for the given instance. Hand-unrolled version since the version above results in less efficient code
static void InstanceTransform(
    in  InstanceDesc  instanceDesc,
    in  float3        origin,
    in  float3        direction,
    out_param(float3) newOrigin,
    out_param(float3) newDirection);

//=====================================================================================================================
// For pair compression, ignore the triangle type during comparisons for stackless traversal
static bool ComparePointers(uint nodePointer, uint lastNodePointer);

//=====================================================================================================================
static uint CalcPrimitiveIndexOffset(
    uint nodePointer);

//=====================================================================================================================
// Return base address pointing to acceleration structure header
static uint64_t GetAccelStructHeaderAddr(
    in uint64_t baseAddrAccelStruct);

//=====================================================================================================================
static uint FetchHeaderField(
    in uint64_t baseAddrAccelStruct,
    const uint  offset);

//=====================================================================================================================
static uint FetchHeaderOffsetField(
    in uint64_t baseAddrAccelStruct,
    const uint  offset);

//=====================================================================================================================
static BoundingBox FetchHeaderRootBoundingBox(
    in uint64_t baseAddrAccelStruct);

//=====================================================================================================================
static uint FetchParentNodePointer(
    in GpuVirtualAddress bvhAddress,
    in uint              nodePtr);

//=====================================================================================================================
static PrimitiveData FetchPrimitiveDataAddr(
    in uint              nodePointer,
    in GpuVirtualAddress nodeAddress);

#if GPURT_BUILD_RTIP3
//=====================================================================================================================
static PrimitiveData FetchPrimitiveDataAddr3_0(
    in uint              nodePointer,
    in GpuVirtualAddress nodeAddress);
#endif

//=====================================================================================================================
static PrimitiveData FetchPrimitiveData(
    in GpuVirtualAddress bvhAddress,
    in uint              nodePointer);

//=====================================================================================================================
#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
// Compute instance sideband offset from node offset when compressed node formats are enabled.
static uint32_t ComputeInstanceSidebandOffset(
    in uint32_t instanceNodeOffset,
    in uint32_t leafNodeOffset,
    in uint32_t sidebandDataOffset);
#endif

//=====================================================================================================================
static uint32_t GetInstanceSidebandOffset(
    in AccelStructHeader header,
    in uint32_t          instanceNodeOffset);

//=====================================================================================================================
// Node pointers with all upper bits set are sentinels: INVALID_NODE, TERMINAL_NODE, SKIP_*
static bool IsValidNode(uint nodePtr);

//======================================================================================================================
static void OutOfRangeNodePointerAssert(
    in uint     rtIpLevel,
    in uint     nodePointer,
    in uint64_t currBvhAddr,
    in uint64_t tlasAddr);

//=====================================================================================================================
static InstanceDesc FetchInstanceDescAddr(in GpuVirtualAddress instanceAddr);

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
// Compute N-bit quantized bounds
static UintBoundingBox ComputeQuantizedBounds(
    in BoundingBox   bounds,
    in float3        origin,
    in float3        rcpExponents,
    in uint          numQuantBits);
#endif

//=====================================================================================================================
static bool UsesFastLbvhLayout();

#ifndef LIBRARY_COMPILATION
#include "Common.hlsl"
#endif

#endif
