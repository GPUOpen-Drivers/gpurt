/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef SHADERDEFS_HLSLI
#define SHADERDEFS_HLSLI

// These DUMMY_*_FUNC postfix stubs must be included at the end of every driver stub (AmdTraceRay*) declaration to
// work around a DXC + Spirv issue where the compiler can't deal with calls to functions that don't have bodies.
#define DUMMY_BOOL_FUNC   { return false; }
#define DUMMY_VOID_FUNC   { }
#define DUMMY_UINT_FUNC   { return 0; }
#define DUMMY_UINT2_FUNC  { return uint2(0, 0); }
#define DUMMY_UINT3_FUNC  { return uint3(0, 0, 0); }
#define DUMMY_UINT4_FUNC  { return uint4(0, 0, 0, 0); }
#define DUMMY_FLOAT_FUNC  { return 0; }
#define DUMMY_FLOAT2_FUNC { return float2(0, 0); }
#define DUMMY_FLOAT3_FUNC { return float3(0, 0, 0); }

#include "TempAssert.hlsli"

// TODO: there are functions that use values from these files, but really
// those functions should be in these files, and then the files that use the functions
// should include that file, instead of ShaderDefs.h
#include "gfx10/BoxNode1_0.hlsli"
#include "gfx10/TriangleNode1_0.hlsli"
#include "gfx10/ProceduralNode1_0.hlsli"
#include "gfx10/BoxNode1_0.hlsli"
#include "gfx10/InstanceNode1_0.hlsli"
#include "NodePointers.hlsli"

#define SAH_COST_TRIANGLE_INTERSECTION       1.5
#define SAH_COST_AABBB_INTERSECTION          1

typedef uint64_t GpuVirtualAddress;

//=====================================================================================================================
enum PrimitiveType : uint
{
    Triangle = 0,
    AABB     = 1,
    Instance = 2,
};

//=====================================================================================================================
// BVH node types shared between HW and SW nodes
#define NODE_TYPE_TRIANGLE_0           0
#define NODE_TYPE_TRIANGLE_1           1
#define NODE_TYPE_TRIANGLE_2           2
#define NODE_TYPE_TRIANGLE_3           3
#define NODE_TYPE_BOX_FLOAT16          4
#define NODE_TYPE_BOX_FLOAT32          5
#define NODE_TYPE_USER_NODE_INSTANCE   6
// From the HW IP 2.0 spec: '7: User Node 1 (processed as a Procedural Node for culling)'
#define NODE_TYPE_USER_NODE_PROCEDURAL 7

//=====================================================================================================================
// Acceleration structure type
#define TOP_LEVEL      0
#define BOTTOM_LEVEL   1

//=====================================================================================================================
// Triangle Compression Modes
#define NO_TRIANGLE_COMPRESSION        0
#define RESERVED                       1
#define PAIR_TRIANGLE_COMPRESSION      2
#define AUTO_TRIANGLE_COMPRESSION      3

#define LATE_PAIR_COMP_BATCH_SIZE 8

//=====================================================================================================================
// Amount of ULPs(Unit in Last Place) added to Box node when using hardware intersection instruction
#define BOX_EXPANSION_DEFAULT_AMOUNT 6

//=====================================================================================================================
// Box sorting heuristic value
// 0: closethit
// 1: LargestFirst
// 2: ClosestMidpoint
// 3: undefined / disabled
// 4: LargestFirstOrClosest (auto select with rayFlag)
// 5: BoxSortLargestFirstOrClosestMidPoint  (auto select with rayFlag)
// 6: DisabledOnAcceptFirstHit (disable if bvhNode sort is on, and rayFlag is AcceptFirstHit)
//
// This need to match ILC_BOX_SORT_HEURISTIC_MODE
enum BoxSortHeuristic : uint
{
    Closest                       = 0x0,
    Largest                       = 0x1,
    MidPoint                      = 0x2,
    Disabled                      = 0x3,
    LargestFirstOrClosest         = 0x4,
    LargestFirstOrClosestMidPoint = 0x5,
    DisabledOnAcceptFirstHit      = 0x6,
};

//=====================================================================================================================
// Options for where FP16 box nodes are created within BLAS for QBVH
#define NO_NODES_IN_BLAS_AS_FP16           0
#define LEAF_NODES_IN_BLAS_AS_FP16         1
#define MIXED_NODES_IN_BLAS_AS_FP16        2
#define ALL_INTERIOR_NODES_IN_BLAS_AS_FP16 3

// The highest 3 bits are zero after the right shift in PackNodePointer and may be repurposed.
// Mask for MSB within node pointer
#define NODE_POINTER_MASK_MSB              0x80000000u

//=====================================================================================================================
#define BVH4_NODE_32_STRIDE_SHIFT             7   // Box 32 node
#define BVH4_NODE_16_STRIDE_SHIFT             6   // Box 16 node

#define INVALID_IDX           0xffffffff
#define INACTIVE_PRIM         0xfffffffe

static const uint ByteStrideScratchNode = 64;
static const uint ByteStrideU32         = 12;
static const uint IndexFormatInvalid    = 0;
static const uint IndexFormatU32        = 1;
static const uint IndexFormatU16        = 2;

const static uint TILE_WIDTH = 256;
const static uint TILE_SIZE  = TILE_WIDTH * TILE_WIDTH;

#ifndef BUILD_THREADGROUP_SIZE
#define BUILD_THREADGROUP_SIZE 64
#endif

//======================================================================================================================
// matches VkAccelerationStructureBuildRangeInfoKHR
struct IndirectBuildOffset
{
    uint primitiveCount;
    uint primitiveOffset;
    uint firstVertex;
    uint transformOffset;
};

//=====================================================================================================================
// Function assumes the type passed in is a valid node type
//
static uint PackNodePointer(uint type, uint address)
{
    uint nodePointer = type; // this assumes that the type is valid
    // uint pointer = type & 0x7;

    // The input address is a byte offset, and node_addr is a 64-byte offset that starts at bit 3.
    nodePointer |= (address >> 3); // this assumes that the input address is 64-byte aligned
    // pointer |= (address >> 6) << 3;

    return nodePointer;
}

//=====================================================================================================================
static uint GetNodeType(uint nodePointer)
{
    // From the HW raytracing spec:
    // node_type = node_pointer[ 2:0]
    return nodePointer & 0x7;
}

//=====================================================================================================================
static uint ClearNodeType(uint nodePointer)
{
    return nodePointer & ~0x7;
}

//=====================================================================================================================
// NOTE: The highest 3 bits are excluded. They aren't written when building the QBVH and may have been repurposed. See
// NODE_POINTER_MASK_MSB
static uint ExtractNodePointerOffset(uint nodePointer)
{
    // From the HW raytracing spec:
    // node_addr[60:0] = node_pointer[63:3]
    // Also, based on the following, the node_addr is 64-byte aligned:
    // fetch_addr0 = T#.base_address*256+node_addr*64
    return ClearNodeType(nodePointer) << 3;
}

//=====================================================================================================================
// Removes temp flag (MSB) within node type set by RefitBounds when fp16 nodes mode is LEAF_NODES_IN_BLAS_AS_FP16.
static uint GetNodePointerExclMsbFlag(uint nodePointer)
{
    return nodePointer & (~NODE_POINTER_MASK_MSB);
}

//=====================================================================================================================
// Primitive data structure that includes the unpacked data needed to process a primitive
struct PrimitiveData
{
    uint primitiveIndex; // Primitive index used to indicate what primitive in geometry description
    uint geometryIndex;  // Geometry index used to indicate what geometry description
    uint geometryFlags;  // Geometry flags contains if the geometry is opaque or non opaque
};

//=====================================================================================================================
// Extract the geometry index from the bottom 24 bits
static uint ExtractGeometryIndex(uint geometryIndexAndFlags)
{
    return geometryIndexAndFlags & 0xFFFFFF;
}

//=====================================================================================================================
// Extract the geometry flags from bits 25-26
static uint ExtractGeometryFlags(uint geometryIndexAndFlags)
{
    return (geometryIndexAndFlags >> 24) & 0x3;
}

//=====================================================================================================================
// Extract the geometry index from the bottom 24 bits and geometry flags from bits 25-26
static uint2 UnpackGeometryIndexAndFlags(uint geometryIndexAndFlags)
{
    return uint2(ExtractGeometryIndex(geometryIndexAndFlags), ExtractGeometryFlags(geometryIndexAndFlags));
}

//=====================================================================================================================
// Pack the geometry index in the bottom 24 bits and the geometry flags into bits 25-26
static uint PackGeometryIndexAndFlags(
    uint geometryIndex,
    uint geometryFlags)
{
    return (geometryFlags << 24) | (geometryIndex & 0xFFFFFF);
}

//=====================================================================================================================
// Additional geometry information for bottom level acceleration structures primitives
struct GeometryInfo
{
    uint geometryFlagsAndNumPrimitives;
    uint geometryBufferOffset;
    uint primNodePtrsOffset; // Offset from the base of all prim node ptrs to this geometry's prim node ptrs
};

#define DXGI_FORMAT_UNKNOWN         0
#define DXGI_FORMAT_R32G32B32_FLOAT 6

#define DECODE_VERTEX_STRIDE                     12
#define DECODE_PRIMITIVE_STRIDE_TRIANGLE         36
#define DECODE_PRIMITIVE_STRIDE_AABB             24
#define GEOMETRY_INFO_SIZE                       12
#define GEOMETRY_INFO_FLAGS_AND_NUM_PRIMS_OFFSET  0
#define GEOMETRY_INFO_GEOM_BUFFER_OFFSET          4
#define GEOMETRY_INFO_PRIM_NODE_PTRS_OFFSET       8

#define PIPELINE_FLAG_SKIP_TRIANGLES                0x100
#define PIPELINE_FLAG_SKIP_PROCEDURAL_PRIMITIVES    0x200

GPURT_STATIC_ASSERT(GEOMETRY_INFO_SIZE == sizeof(GeometryInfo), "Geometry info structure mismatch");

//=====================================================================================================================
static uint ExtractGeometryInfoFlags(uint packedGeometryFlagsAndNumPrimitives)
{
    return (packedGeometryFlagsAndNumPrimitives >> 29);
}

//=====================================================================================================================
static uint ExtractGeometryInfoNumPrimitives(uint packedGeometryFlagsAndNumPrimitives)
{
    // ((1 << 29) - 1) = 0x1fffffff
    return (packedGeometryFlagsAndNumPrimitives & 0x1FFFFFFF);
}

//=====================================================================================================================
static uint PackGeometryFlagsAndNumPrimitives(uint geometryFlags, uint numPrimitives)
{
    return (geometryFlags << 29) | numPrimitives;
}

//=====================================================================================================================
static uint64_t PackUint64(uint lowBits, uint highBits)
{
    // Note glslang doesn't like uint64_t casts
    uint64_t addr = highBits;
    addr = (addr << 32) | lowBits;
    return addr;
}

//======================================================================================================================
// Packs the channels of a uint2 into a single uint64_t.
static uint64_t PackUint64(uint2 lowHigh)
{
    // Note glslang doesn't like uint64_t casts
    uint64_t addr = lowHigh.y;
    addr = (addr << 32) | lowHigh.x;
    return addr;
}

//=====================================================================================================================
static uint2 SplitUint64(uint64_t x)
{
    return uint2(x, (x >> 32));
}

//=====================================================================================================================
// Build Stage Counters (Debug only)
// It starts with the qbvhGlobalCounters offset, i.e.,
// qbvhGlobalStack...qbvhGlobalStackPtrs...bvhBuildDebugCounters

#define COUNTER_MORTONGEN_OFFSET        0x0
#define COUNTER_MORTON_SORT_OFFSET      0x4
#define COUNTER_SORTLEAF_OFFSET         0x8
#define COUNTER_BUILDPLOC_OFFSET        0xC
#define COUNTER_BUILDLBVH_OFFSET        0x10
#define COUNTER_REFIT_OFFSET            0x14
#define COUNTER_INITENCODEHWBVH_OFFSET  0x18
#define COUNTER_ENCODEHWBVH_OFFSET      0x1C
#define COUNTER_EMPTYPRIM_OFFSET        0x20
#define COUNTER_EMITCOMPACTSIZE_OFFSET  0x24
#define COUNTER_BUILDFASTLBVH_OFFSET    0x28

//=====================================================================================================================
// Get leaf triangle node size in bytes
static uint GetBvhNodeSizeTriangle()
{
    return TRIANGLE_NODE_SIZE;
}

//=====================================================================================================================
// Get leaf AABB node size in bytes
static uint GetBvhNodeSizeProcedural()
{
    return USER_NODE_PROCEDURAL_SIZE;
}

//=====================================================================================================================
// Get leaf instance node size in bytes
static uint GetBvhNodeSizeInstance(uint enableFusedInstanceNode)
{
    return (enableFusedInstanceNode == 0) ? INSTANCE_NODE_SIZE : FUSED_INSTANCE_NODE_SIZE;
}

//=====================================================================================================================
// Get internal BVH node size in bytes
static uint GetBvhNodeSizeInternal()
{
    return FLOAT32_BOX_NODE_SIZE;
}

//=====================================================================================================================
// Get internal BVH node size in bytes
static uint GetBvhNodeSizeLeaf(
    uint primitiveType,
    uint enableFusedInstanceNode)
{
    uint sizeInBytes = 0;
    switch (primitiveType)
    {
    case PrimitiveType::Triangle:
        sizeInBytes = GetBvhNodeSizeTriangle();
        break;
    case PrimitiveType::AABB:
        sizeInBytes = GetBvhNodeSizeProcedural();
        break;
    case PrimitiveType::Instance:
        sizeInBytes = GetBvhNodeSizeInstance(enableFusedInstanceNode);
        break;
    }

    return sizeInBytes;
}

//=====================================================================================================================
static uint CalcParentPtrOffset(uint nodePtr)
{
    // Subtract 1 from the index to account for negative offset calculations. I.e. index 0 is actually at -4 byte
    // offset from the end of the parent pointer memory
    const uint linkIndex = (nodePtr >> 3) - 1;
    return linkIndex * NODE_PTR_SIZE;
}

//=====================================================================================================================
static uint CalcBottomGeometryInfoSize(uint numGeometries)
{
    return numGeometries * GEOMETRY_INFO_SIZE;
}

//=====================================================================================================================
struct DataOffsetAndSize
{
    uint offset;
    uint size;
};

//=====================================================================================================================
struct StateTaskQueueCounter
{
    uint            phase;
    uint            startPhaseIndex;
    uint            endPhaseIndex;
    uint            taskCounter;
    uint            numTasksDone;
};

#define USE_BLAS_PRIM_COUNT   0

//=====================================================================================================================
struct Flags
{
    uint dataValid;
    uint prefixSum;
};

#define FLAGS_DATA_VALID_OFFSET          0
#define FLAGS_PREFIX_SUM_OFFSET          4

#define DLB_KEYS_PER_THREAD     4
#define DLB_KEYS_PER_GROUP      (BUILD_THREADGROUP_SIZE * DLB_KEYS_PER_THREAD)

#define DLB_VALID_SUM           0
#define DLB_VALID_PREFIX_SUM    1
#define NUM_DLB_VALID_TYPES     2

//=====================================================================================================================

#define PLOC_PHASE_INIT                     0
#define PLOC_PHASE_FIND_NEAREST_NEIGHBOUR   1
#define PLOC_PHASE_UPDATE_CLUSTER_COUNT     2
#define PLOC_PHASE_DONE                     3
struct StatePLOC
{
    uint                    numClusters;
    uint                    internalNodesIndex;
    uint                    clusterListIndex;
    uint                    numClustersAlloc;
};

#define STATE_PLOC_NUM_CLUSTERS_OFFSET                      0
#define STATE_PLOC_INTERNAL_NODES_INDEX_OFFSET              4
#define STATE_PLOC_CLUSTER_LIST_INDEX_OFFSET                8
#define STATE_PLOC_NUM_CLUSTERS_ALLOC_OFFSET                12

//=====================================================================================================================
struct IndexBufferInfo
{
    uint gpuVaLo;
    uint gpuVaHi;
    uint byteOffset;
    uint format;
};

//=====================================================================================================================
enum RebraidType : uint
{
    Off = 0, // No Rebraid
    V1  = 1, // First version of Rebraid
    V2  = 2, // Second version of Rebraid
};

#define BUILD_MODE_LINEAR   0
// BUILD_MODE_AC was 1, but it has been removed.
#define BUILD_MODE_PLOC     2

//=====================================================================================================================
struct TriangleData
{
    float3 v0; ///< Vertex 0
    float3 v1; ///< Vertex 1
    float3 v2; ///< Vertex 2
};

#ifndef LIBRARY_COMPILATION
// This does not include RayTracingDefs.h as the goal is
// to eventually have everything in this file alone
#endif

#endif
