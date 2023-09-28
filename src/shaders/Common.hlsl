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

#include "../shared/rayTracingDefs.h"
#include "../shared/scratchNode.h"

#if !defined(__cplusplus)
#define out_param(x) out x
#define inout_param(x) inout x
#endif

#if !defined(__cplusplus)
#include "BuildSettings.hlsli"
#include "Debug.hlsl"
#endif

#ifdef AMD_VULKAN_GLSLANG
// Vulkan workarounds for glslangValidator.
#define export

struct BuiltInTriangleIntersectionAttributes
{
    float2 barycentrics;
};

#endif

//=====================================================================================================================
// static definitions

#ifndef INT_MAX
#define INT_MAX 0x7FFFFFFF
#endif
#ifndef FLT_MAX
#define FLT_MAX 3.402823466e+38F
#endif

// Note: log(+/-0) always produces -inf. Reverse polarity here
#ifndef INFINITY
#define INFINITY -log(0.0)
#endif

#define INVALID_IDX       0xffffffff

// Node pointer values with special meanings
#define INVALID_NODE      0xffffffff
#define TERMINAL_NODE     0xfffffffe
#define SKIP_0_3          0xfffffffd
#define SKIP_4_7          0xfffffffb
#define SKIP_0_7          0xfffffff9
#define END_SEARCH        0xfffffff8

#include "Extensions.hlsl"
#include "../shared/math.h"

// Workaround for lack of 64 bit literals and casts in glslang
static const uint64_t OneU64 = 1;

#ifdef __cplusplus
static const float NaN = std::numeric_limits<float>::quiet_NaN();
#else
static const float NaN =  (0.0 / 0.0);
#endif

static const BoundingBox InvalidBoundingBox =
{
    float3(FLT_MAX, FLT_MAX, FLT_MAX),
    float3(-FLT_MAX, -FLT_MAX, -FLT_MAX)
};

#define FLAG_IS_SET(x, flagName)   (((x) & (OneU64 << (flagName##_SHIFT))) != 0)
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

#define PIPELINE_FLAG_BVH_COLLAPSE                   0x80000000
#define PIPELINE_FLAG_USE_REBRAID                    0x40000000
#define PIPELINE_FLAG_ENABLE_AS_TRACKING             0x20000000
#define PIPELINE_FLAG_ENABLE_TRAVERSAL_CTR           0x10000000
#define PIPELINE_FLAG_RESERVED                       0x08000000
#define PIPELINE_FLAG_ENABLE_FUSED_INSTANCE          0x04000000
#define PIPELINE_FLAG_RESERVED1                      0x02000000
#define PIPELINE_FLAG_RESERVED2                      0x01000000
#define PIPELINE_FLAG_RESERVED3                      0x00800000

#define HIT_KIND_TRIANGLE_FRONT_FACE 0xFE
#define HIT_KIND_TRIANGLE_BACK_FACE  0xFF
#define HIT_KIND_EARLY_RAY_TERMINATE 0x100

#define HIT_STATUS_IGNORE                0
#define HIT_STATUS_ACCEPT                1
#define HIT_STATUS_ACCEPT_AND_END_SEARCH 2

#define GEOMETRY_TYPE_TRIANGLES 0 // D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES
#define GEOMETRY_TYPE_AABBS     1 // D3D12_RAYTRACING_GEOMETRY_TYPE_PROCEDURAL_PRIMITIVE_AABBS

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

//=====================================================================================================================
// The following functions depend on static flags
static uint IsBvhCollapse()
{
    return (AmdTraceRayGetStaticFlags() & PIPELINE_FLAG_BVH_COLLAPSE);
}
static uint IsBvhRebraid()
{
    return (AmdTraceRayGetStaticFlags() & PIPELINE_FLAG_USE_REBRAID);
}
static uint EnableAccelStructTracking()
{
    return (AmdTraceRayGetStaticFlags() & PIPELINE_FLAG_ENABLE_AS_TRACKING);
}
static uint EnableFusedInstanceNodes()
{
    return (AmdTraceRayGetStaticFlags() & PIPELINE_FLAG_ENABLE_FUSED_INSTANCE);
}

#if DEVELOPER
static bool EnableTraversalCounter()
{
    return (AmdTraceRayGetStaticFlags() & PIPELINE_FLAG_ENABLE_TRAVERSAL_CTR);
}
#endif

//=====================================================================================================================
static uint FloatToUint(float v)
{
    const uint bitShift = 31;
    const uint bitMask = 0x80000000;

    uint ui = uint(asuint(v));
    ui ^= (1 + ~(ui >> bitShift) | bitMask);

    return ui;
}

//=====================================================================================================================
static GpuVirtualAddress MakeGpuVirtualAddress(uint lowBits, uint highBits)
{
    return PackUint64(lowBits, highBits);
}

//=====================================================================================================================
static GpuVirtualAddress MakeGpuVirtualAddress(uint2 lowHigh)
{
    return PackUint64(lowHigh.x, lowHigh.y);
}

//=====================================================================================================================
static GpuVirtualAddress ExtractInstanceAddr(uint64_t instanceBasePointer)
{
    // Since the zero field should be zero, its mask is included here to avoid extra ALU for masking the low bits.
    return (instanceBasePointer & (INSTANCE_BASE_POINTER_ADDRESS_USED_MASK | INSTANCE_BASE_POINTER_ZERO_MASK)) << 3;
}

//=====================================================================================================================
static GpuVirtualAddress GetInstanceAddr(in InstanceDesc desc)
{
    return ExtractInstanceAddr(PackUint64(desc.accelStructureAddressLo, desc.accelStructureAddressHiAndFlags));
}

//=====================================================================================================================
static GpuVirtualAddress GetInstanceAddr(in uint instanceBasePointerLo, in uint instanceBasePointerHi)
{
    return ExtractInstanceAddr(PackUint64(instanceBasePointerLo, instanceBasePointerHi));
}

//=====================================================================================================================
static uint64_t GetNodeAddr(uint64_t baseNodePtr, uint nodePtr)
{
    return ExtractInstanceAddr(baseNodePtr) + ExtractNodePointerOffset(nodePtr);
}

//=====================================================================================================================
static uint64_t GetInstanceBasePointer(in InstanceDesc desc)
{
    // Mask out the extra address bits, which are repurposed to contain the geometry type.
    const uint highMask = HighPart(INSTANCE_BASE_POINTER_FLAGS_MASK | INSTANCE_BASE_POINTER_ADDRESS_USED_MASK);

    return PackUint64(desc.accelStructureAddressLo,
                      desc.accelStructureAddressHiAndFlags & highMask);
}

//=====================================================================================================================
static uint FetchInstanceIdx(in uint64_t instanceNodePtr)
{
    return LoadDwordAtAddr(instanceNodePtr + INSTANCE_DESC_SIZE + RTIP1_1_INSTANCE_SIDEBAND_INSTANCE_INDEX_OFFSET);
}

//=====================================================================================================================
static uint64_t PackInstanceBasePointer(GpuVirtualAddress instanceVa, uint instanceFlags, uint geometryType)
{
    uint64_t instanceBasePointer = instanceVa >> 3;

    instanceBasePointer |= (instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_CULL_DISABLE)
                           ? (1ull << NODE_POINTER_DISABLE_TRIANGLE_CULL_SHIFT) : 0;

    instanceBasePointer |= (instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_FRONT_COUNTERCLOCKWISE)
                           ? (1ull << NODE_POINTER_FLIP_FACEDNESS_SHIFT) : 0;

    instanceBasePointer |= (instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_FORCE_OPAQUE)
                           ? (1ull << NODE_POINTER_FORCE_OPAQUE_SHIFT) : 0;

    instanceBasePointer |= (instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_FORCE_NON_OPAQUE)
                           ? (1ull << NODE_POINTER_FORCE_NON_OPAQUE_SHIFT) : 0;

    {
        // Set 'Skip Procedural' for triangles and 'Skip Triangles' for procedural geometry
        instanceBasePointer |= (geometryType == GEOMETRY_TYPE_TRIANGLES)
                            ? (1ull << NODE_POINTER_SKIP_PROCEDURAL_SHIFT)
                            : (1ull << NODE_POINTER_SKIP_TRIANGLES_SHIFT);
    }

    return instanceBasePointer;
}

//======================================================================================================================
// Produces a node pointer at the provided address with API ray flags encoded into the appropriate node pointer flags.
static uint64_t EncodeBasePointer(
    uint64_t accelStructAddr,
    uint     rayFlags)
{
    // Encode API ray flags into pointer flags resetting ACCEPT_FIRST_HIT and SKIP_CLOSEST_HIT which are not implemented
    // using the pointer flags feature.
    const uint64_t rayFlagsToInclude    = rayFlags & ~RAY_FLAG_EXCLUDE_MASK;
    const uint64_t topLevelPointerFlags = rayFlagsToInclude << NODE_POINTER_FLAGS_SHIFT;
    const uint64_t basePointer          = (accelStructAddr >> 3) | topLevelPointerFlags;

    return basePointer;
}

//=====================================================================================================================
static bool IsBoxNode32(uint pointer)
{
    return (GetNodeType(pointer) == NODE_TYPE_BOX_FLOAT32);
}

//=====================================================================================================================
static bool IsBoxNode16(uint pointer)
{
    return (GetNodeType(pointer) == NODE_TYPE_BOX_FLOAT16);
}

//=====================================================================================================================
static bool IsBoxNode(
    uint pointer
)
{
    {
        return IsBoxNode16(pointer) || IsBoxNode32(pointer);
    }
}

//=====================================================================================================================
static uint CreateRootNodePointer(
)
{
    return PackNodePointer(NODE_TYPE_BOX_FLOAT32, sizeof(AccelStructHeader));
}

#if GPURT_BVH_BUILD_SHADER
#endif

//=====================================================================================================================
static bool IsTriangleNode1_1(
    uint nodePtr)
{
    return (GetNodeType(nodePtr) <= NODE_TYPE_TRIANGLE_1);
}

//=====================================================================================================================
static bool IsUserNodeInstance(uint nodePtr)
{
    return (GetNodeType(nodePtr) == NODE_TYPE_USER_NODE_INSTANCE);
}

//=====================================================================================================================
static bool IsUserNodeProcedural(uint nodePtr)
{
    return (GetNodeType(nodePtr) == NODE_TYPE_USER_NODE_PROCEDURAL);
}

//=====================================================================================================================
static bool CheckHandleTriangleNode(in uint pointerOrType)
{
    bool skipTriangleFlag = false;

    const uint pipelineRayFlag = AmdTraceRayGetStaticFlags();
    skipTriangleFlag = (pipelineRayFlag & PIPELINE_FLAG_SKIP_TRIANGLES) ? true : false;

    return (IsTriangleNode1_1(pointerOrType) && (!skipTriangleFlag));
}

//=====================================================================================================================
static bool CheckHandleProceduralUserNode(in uint nodePointer)
{
    bool skipProceduralFlag = false;

    const uint pipelineRayFlag = AmdTraceRayGetStaticFlags();
    skipProceduralFlag =
        (pipelineRayFlag & PIPELINE_FLAG_SKIP_PROCEDURAL_PRIMITIVES) ? true : false;

    return (IsUserNodeProcedural(nodePointer) && (!skipProceduralFlag));
}

//=====================================================================================================================
static uint WriteTriangleIdField(uint triangleId, uint nodeType, uint rotation, uint geometryFlags)
{
    const uint triangleShift = nodeType * TRIANGLE_ID_BIT_STRIDE;

    // Compute the barycentrics mapping table that is stored in triangle_id for RT IP 1.1
    triangleId |= ((rotation + 1) % 3) << (triangleShift + TRIANGLE_ID_I_SRC_SHIFT);
    triangleId |= ((rotation + 2) % 3) << (triangleShift + TRIANGLE_ID_J_SRC_SHIFT);

#if GPURT_BUILD_RTIP2
    // Add in the flags stored in triangle_id for RT IP 2.0
    if (geometryFlags & D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE)
    {
        triangleId |= 1u << (triangleShift + TRIANGLE_ID_OPAQUE_SHIFT);
    }
#endif

    return triangleId;
}

//=====================================================================================================================
static uint CalcUncompressedTriangleId(uint geometryFlags)
{
    return WriteTriangleIdField(0, NODE_TYPE_TRIANGLE_0, 0, geometryFlags);
}

//=====================================================================================================================
// Extract the order of the triangle vertices from the node's triangle ID field.
static uint3 CalcTriangleCompressionVertexOffsets(uint nodeType, uint triangleId)
{
    uint3 vertexSwizzle;
    vertexSwizzle.y = (triangleId >> (TRIANGLE_ID_BIT_STRIDE * nodeType + TRIANGLE_ID_I_SRC_SHIFT)) % 4;
    vertexSwizzle.z = (triangleId >> (TRIANGLE_ID_BIT_STRIDE * nodeType + TRIANGLE_ID_J_SRC_SHIFT)) % 4;
    vertexSwizzle.x = 3 - vertexSwizzle.y - vertexSwizzle.z;

    // Packed triangle vertex mapping similar to the one used by the compression algorithm.
    // node type 0 -> V0, V1, V2
    // node type 1 -> V1, V3, V2
    // node type 2 -> V2, V3, V4
    // node type 3 -> V2, V4, V0
    const uint nodeVertexMapping[4] = { 0x210, 0x231, 0x432, 0x042 };

    uint3 nodeVertexIndices;
    nodeVertexIndices.x = (nodeVertexMapping[nodeType] >> (vertexSwizzle.x * 4)) & 0xf;
    nodeVertexIndices.y = (nodeVertexMapping[nodeType] >> (vertexSwizzle.y * 4)) & 0xf;
    nodeVertexIndices.z = (nodeVertexMapping[nodeType] >> (vertexSwizzle.z * 4)) & 0xf;

    const uint nodeVertexStride = 12;

    return nodeVertexIndices * nodeVertexStride;
}

//=====================================================================================================================
// Gets the offsets for a node type's vertex slots. This function does not account for the rotation information stored
// in the node's triangle ID field (see CalcTriangleCompressionVertexOffsets), so it should only be used when the vertex
// order does not matter (e.g. for generating a bounding box).
static uint3 CalcTriangleVertexOffsets(uint nodeType)
{
    uint3 offsets;

    switch (nodeType)
    {
    case NODE_TYPE_TRIANGLE_0:
    default:
        offsets.x = TRIANGLE_NODE_V0_OFFSET;
        offsets.y = TRIANGLE_NODE_V1_OFFSET;
        offsets.z = TRIANGLE_NODE_V2_OFFSET;
        break;
    case NODE_TYPE_TRIANGLE_1:
        offsets.x = TRIANGLE_NODE_V1_OFFSET;
        offsets.y = TRIANGLE_NODE_V3_OFFSET;
        offsets.z = TRIANGLE_NODE_V2_OFFSET;
        break;
    }

    return offsets;
}

//=====================================================================================================================
// Calculate an AABB from the 3 points of the triangle.
static BoundingBox GenerateTriangleBoundingBox(float3 v0, float3 v1, float3 v2)
{
    BoundingBox bbox =
    {
        min(min(v0, v1), v2),
        max(max(v0, v1), v2),
    };

    // Generate degenerate bounding box for triangles that degenerate into line or point
#ifdef AMD_VULKAN
    if (((v0.x == v1.x) && (v0.y == v1.y) && (v0.z == v1.z)) ||
        ((v0.x == v2.x) && (v0.y == v2.y) && (v0.z == v2.z)) ||
        ((v1.x == v2.x) && (v1.y == v2.y) && (v1.z == v2.z)))
#else
    if (all(v0 == v1) || all(v0 == v2) || all(v1 == v2))
#endif
    {
        bbox = InvalidBoundingBox;
    }

    return bbox;
};

//=====================================================================================================================
// Compresses a single bounding box into a uint3 representing each bounding plane using half floats
static uint3 CompressBBoxToUint3(in BoundingBox box)
{
    // Note: Converted bounds must be conservative and, assuming infinite precision,
    // resulting 16-bit bound representation must contain original 32-bit bounds
    // Thus: round down for min and round up for max

    // HLSL spec forces rounding modes towards 0, rather than -inf or +inf
    // So use AMDIL extension to call appropriate rounding function

    // For prototype/testing, can help to force default HLSL conversion mode
#if FORCE_FP16_CONV_MODE_TO_DEF
    const uint3 boxMin = f32tof16(box.min);
    const uint3 boxMax = f32tof16(box.max);
#else
    const uint3 boxMin = AmdExtD3DShaderIntrinsics_ConvertF32toF16NegInf(box.min);
    const uint3 boxMax = AmdExtD3DShaderIntrinsics_ConvertF32toF16PosInf(box.max);
#endif

    // Interleave into the following format:
    //         low,   high
    // word 0: min.x, min.y
    // word 1: min.z, max.x
    // word 2: max.y, max.z
    const uint3 loRet = { boxMin.x, boxMin.z, boxMax.y };
    const uint3 hiRet = { boxMin.y, boxMax.x, boxMax.z };

    // Combine component-wise to return
    // f32tof16 documentation specifies (loRet & 0xFFFF) is unnecessary
    return ((hiRet << 16) | loRet);
}

//=====================================================================================================================
// Uncompresses a single bounding box from uint3
// Intended for software path only
static BoundingBox UncompressBBoxFromUint3(in uint3 box)
{
    // See notes above about rounding modes

    // Swizzle into individual min,max bounds
    const uint3  loU = (box & 0x0000FFFF);
    const uint3  hiU = (box >> 16);
    const uint3 minU = { loU.x, hiU.x, loU.y };
    const uint3 maxU = { hiU.y, loU.z, hiU.z };

    // We should convert fp16->fp32 conservatively (which would likely expand the bbox)
    // but seems like op codes for f16->f32 conversion with +-inf rounding are not exposed and
    // this path is for SW only, so use default
    BoundingBox toRet;
    toRet.min = f16tof32(minU);
    toRet.max = f16tof32(maxU);

    return toRet;
}

//=====================================================================================================================
static float ComputeBoxSurfaceArea(BoundingBox aabb)
{
    // check for degenerate
    float3 dim = aabb.max - aabb.min;
    return (aabb.min.x > aabb.max.x) ? 0 : 2.0f * (dim.x * dim.y + dim.x * dim.z + dim.y * dim.z);
}

//=====================================================================================================================
static float ComputeBoxSurfaceArea(const uint3 aabb)
{
    const BoundingBox aabbFp32 = UncompressBBoxFromUint3(aabb);
    return ComputeBoxSurfaceArea(aabbFp32);
}

//=====================================================================================================================
// HLSL implementation of OpenCL clz. This function counts the number of leading 0's from MSB
static int clz(int value)
{
    return (31 - firstbithigh(value));
}

//=====================================================================================================================
static int clz64(uint64_t value)
{
    return (63 - firstbithigh(value));
}

//=====================================================================================================================
// HLSL emulation of min3. We can use intrinsics here if available
static float min3(float3 val)
{
    return min(min(val.x, val.y), val.z);
}

//=====================================================================================================================
// HLSL emulation of max3. We can use intrinsics here if available
static float max3(float3 val)
{
    return max(max(val.x, val.y), val.z);
}

//=====================================================================================================================
static bool IsUpdate()
{
    return Settings.isUpdate;
}

//=====================================================================================================================
static bool AllowUpdate(uint buildFlags)
{
    return buildFlags & DDI_BUILD_FLAG_ALLOW_UPDATE;
}

//=====================================================================================================================
// Transform a ray for the given instance. Hand-unrolled version since the version above results in less efficient code
static void InstanceTransform(
    in  InstanceDesc  instanceDesc,
    in  float3        origin,
    in  float3        direction,
    out_param(float3) newOrigin,
    out_param(float3) newDirection)
{
    float4 t0 = instanceDesc.Transform[0];
    float4 t1 = instanceDesc.Transform[1];
    float4 t2 = instanceDesc.Transform[2];

    float r0x = mad(origin.z, t0.z, t0.w);
    float r0y = mad(origin.z, t1.z, t1.w);
    float r0z = mad(origin.z, t2.z, t2.w);

    float r1x = mul(direction.z, t0.z);
    float r1y = mul(direction.z, t1.z);
    float r1z = mul(direction.z, t2.z);

    r0x = mad(origin.y, t0.y, r0x);
    r0y = mad(origin.y, t1.y, r0y);
    r0z = mad(origin.y, t2.y, r0z);

    r1x = mad(direction.y, t0.y, r1x);
    r1y = mad(direction.y, t1.y, r1y);
    r1z = mad(direction.y, t2.y, r1z);

    r0x = mad(origin.x, t0.x, r0x);
    r0y = mad(origin.x, t1.x, r0y);
    r0z = mad(origin.x, t2.x, r0z);

    r1x = mad(direction.x, t0.x, r1x);
    r1y = mad(direction.x, t1.x, r1y);
    r1z = mad(direction.x, t2.x, r1z);

    newOrigin = float3(r0x, r0y, r0z);
    newDirection = float3(r1x, r1y, r1z);
}

//=====================================================================================================================
// Mask Out Primitive Count for Collapse
static uint ExtractNodePointerCollapse(uint nodePointer)
{
    return nodePointer & ((1 << 29) - 1); // mask out the triangle count
}

//=====================================================================================================================
static uint ExtractNodePointer(uint nodePointer)
{
    if (IsBvhCollapse())
    {
        return ExtractNodePointerCollapse(nodePointer); // mask out the triangle count
    }
    else
    {
        return nodePointer;
    }
}

//=====================================================================================================================
// Get Num Primitives For Collapse
static uint ExtractPrimitiveCount(uint nodePointer)
{
    return nodePointer >> 29;
}

//=====================================================================================================================
// For BVH Collapse, need to compare this node Pointer which points to the start of the primitive list to the end which
// is the lastNodePointer
static bool ComparePointers(uint nodePointer, uint lastNodePointer)
{
    if (IsBvhCollapse())
    {
        const uint NodeCountPtr = (1 << 3);
        return ExtractNodePointer((nodePointer + ExtractPrimitiveCount(nodePointer) * NodeCountPtr)) == lastNodePointer;
    }
    else if ((AmdTraceRayGetTriangleCompressionMode() == PAIR_TRIANGLE_COMPRESSION) ||
             (AmdTraceRayGetTriangleCompressionMode() == AUTO_TRIANGLE_COMPRESSION))
    {
        // Ignore the node type, as both triangles in a node share a parent, and all other types will always
        // have different offsets.
        return ClearNodeType(nodePointer) == ClearNodeType(lastNodePointer);
    }
    else
    {
        return nodePointer == lastNodePointer;
    }
}

//=====================================================================================================================
// For BVH Collapse, increment the node pointer to the next primitive and decrement the counter
static uint IncrementNodePointer(uint nodePointer)
{
    return nodePointer - (1 << 29) + (1 << 3);
}

//=====================================================================================================================
static uint CalcPrimitiveIndexOffset(
    uint nodePointer)
{
    uint offset;
    if (IsTriangleNode1_1(nodePointer))
    {
        offset = GetNodeType(nodePointer) * 4;
    }
    else // procedural node
    {
        offset = 1;
    }

    return offset;
}

//=====================================================================================================================
// Return base address pointing to acceleration structure header
static uint64_t GetAccelStructHeaderAddr(
    in uint64_t baseAddrAccelStruct)
{
    // The BVH data memory begins with AccelStructHeader
    return baseAddrAccelStruct;
}

//=====================================================================================================================
static uint FetchHeaderField(
    in uint64_t baseAddrAccelStruct,
    const uint  offset)
{
    const uint64_t baseAddrAccelStructHeader = GetAccelStructHeaderAddr(baseAddrAccelStruct);
    return LoadDwordAtAddr(baseAddrAccelStructHeader + offset);
}

//=====================================================================================================================
static uint FetchHeaderOffsetField(
    in uint64_t baseAddrAccelStruct,
    const uint  offset)
{
    const uint64_t baseAddrAccelStructHeader = GetAccelStructHeaderAddr(baseAddrAccelStruct);
    return FetchHeaderField(baseAddrAccelStructHeader, ACCEL_STRUCT_HEADER_OFFSETS_OFFSET + offset);
}

//=====================================================================================================================
static BoundingBox FetchHeaderRootBoundingBox(
    in uint64_t baseAddrAccelStruct)
{
    const uint64_t baseAddrAccelStructHeader = GetAccelStructHeaderAddr(baseAddrAccelStruct);
    const uint4 d0 = LoadDwordAtAddrx4(baseAddrAccelStructHeader + ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET);
    const uint2 d1 = LoadDwordAtAddrx2(baseAddrAccelStructHeader + ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET + 0x10);

    BoundingBox bbox;
    bbox.min = asfloat(d0.xyz);
    bbox.max = asfloat(uint3(d0.w, d1.xy));

    return bbox;
}

//=====================================================================================================================
static uint4 FetchHeaderCostOrNumChildPrims(
    in uint64_t baseAddrAccelStruct)
{
    uint4 data = uint4(0, 0, 0, 0);
    data.x = FetchHeaderField(baseAddrAccelStruct, ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET);
    data.y = FetchHeaderField(baseAddrAccelStruct, ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 4);
    data.z = FetchHeaderField(baseAddrAccelStruct, ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 8);
    data.w = FetchHeaderField(baseAddrAccelStruct, ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 12);

    return data;
}

//=====================================================================================================================
static uint FetchParentNodePointer(
    in GpuVirtualAddress bvhAddress,
    in uint              packedNodePtr)
{
    // Fetch parent pointer from metadata memory which is allocated before acceleration structure data.
    // bvhAddress points to the beginning of acceleration structure memory
    const uint nodePtr = ExtractNodePointer(packedNodePtr);
    return LoadDwordAtAddr(bvhAddress - CalcParentPtrOffset(nodePtr));
}

//=====================================================================================================================
static PrimitiveData FetchPrimitiveDataAddr(
    in uint              nodePointer,
    in GpuVirtualAddress nodeAddress)
{
    PrimitiveData primitiveData = { 0, 0, 0 };

    // Load sideband data from leaf node
    const uint  geometryIndexAndFlags = LoadDwordAtAddr(nodeAddress + TRIANGLE_NODE_GEOMETRY_INDEX_AND_FLAGS_OFFSET);
    const uint2 primitiveIndex        = LoadDwordAtAddrx2(nodeAddress + TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET);

    // Load the primitive index based on the bottom bit of the node pointer which indicates triangle index.
    // Only 2 triangles can be potentially compressed in this path so we only care about the least significant bit
    // Procedural node primitive index overlaps with primitive index of NODE_TYPE_TRIANGLE_1 and does not need
    // additional checks.
    primitiveData.primitiveIndex = (nodePointer & 0x1) ? primitiveIndex.y : primitiveIndex.x;

    const uint2 unpackedGeometryData  = UnpackGeometryIndexAndFlags(geometryIndexAndFlags);
    primitiveData.geometryIndex       = unpackedGeometryData.x;
    primitiveData.geometryFlags       = unpackedGeometryData.y;

    return primitiveData;
}

//=====================================================================================================================
static PrimitiveData FetchPrimitiveData(
    in GpuVirtualAddress bvhAddress,
    in uint              nodePointer)
{
    const GpuVirtualAddress nodeAddress = bvhAddress + ExtractNodePointerOffset(nodePointer);
    return FetchPrimitiveDataAddr(nodePointer, nodeAddress);
}

//=====================================================================================================================

//=====================================================================================================================
static uint32_t GetInstanceSidebandOffset(
    in AccelStructHeader header,
    in uint32_t nodePtr)
{
    uint32_t sidebandOffset = 0;

    const uint32_t instanceNodeOffset = ExtractNodePointerOffset(nodePtr);

    {
        sidebandOffset = instanceNodeOffset + sizeof(InstanceDesc);
    }

    return sidebandOffset;
}

//=====================================================================================================================
// Node pointers with all upper bits set are sentinels: INVALID_NODE, TERMINAL_NODE, SKIP_*
static bool IsValidNode(uint nodePtr)
{
    return nodePtr < END_SEARCH;
}

#endif
