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
// This header file contains shared definitions between the HLSL raytracing shaders and the prototype c++ code
//
#ifndef RAYTRACING_DEF_H
#define RAYTRACING_DEF_H

#ifdef AMD_VULKAN
//=====================================================================================================================
///@note Enum is a reserved keyword in glslang. To workaround this limitation, define static constants to replace the
///      HLSL enums that follow for compatibility.
//=====================================================================================================================
namespace PrimitiveType
{
    static const uint Triangle      = 0;
    static const uint AABB          = 1;
    static const uint Instance      = 2;
}
#else
//=====================================================================================================================
enum PrimitiveType : uint
{
    Triangle,
    AABB,
    Instance,
};
#endif

// These DUMMY_*_FUNC postfix stubs must be included at the end of every driver stub (AmdTraceRay*) declaration to
// work around a Vulkan glslang issue where the compiler can't deal with calls to functions that don't have bodies.
#if defined(AMD_VULKAN)
#define DUMMY_BOOL_FUNC   { return false; }
#define DUMMY_VOID_FUNC   { }
#define DUMMY_UINT_FUNC   { return 0; }
#define DUMMY_UINT2_FUNC  { return uint2(0, 0); }
#define DUMMY_UINT3_FUNC  { return uint3(0, 0, 0); }
#define DUMMY_UINT4_FUNC  { return uint4(0, 0, 0, 0); }
#define DUMMY_WIDE_INTERSECT_FUNC { return (DualIntersectResult)0; }
#else
#define DUMMY_BOOL_FUNC   ;
#define DUMMY_VOID_FUNC   ;
#define DUMMY_UINT_FUNC   ;
#define DUMMY_UINT2_FUNC  ;
#define DUMMY_UINT3_FUNC  ;
#define DUMMY_UINT4_FUNC  ;
#define DUMMY_WIDE_INTERSECT_FUNC ;
#endif

#if defined(__cplusplus)
#define __decl extern
#endif

//=====================================================================================================================
// Acceleration structure type
#define TOP_LEVEL      0
#define BOTTOM_LEVEL   1

//=====================================================================================================================
// BVH node types shared between HW and SW nodes
#define NODE_TYPE_TRIANGLE_0           0
#define NODE_TYPE_TRIANGLE_1           1
#define NODE_TYPE_TRIANGLE_2           2
#define NODE_TYPE_TRIANGLE_3           3
#define NODE_TYPE_BOX_FLOAT16          4
#define NODE_TYPE_BOX_FLOAT32          5
#define NODE_TYPE_USER_NODE_INSTANCE   6
#if GPURT_BUILD_RTIP2
// From the HW IP 2.0 spec: '7: User Node 1 (processed as a Procedural Node for culling)'
#endif
#define NODE_TYPE_USER_NODE_PROCEDURAL 7

//=====================================================================================================================
// Various node size definitions
#define USER_NODE_INSTANCE_SIZE        128

//=====================================================================================================================
// Triangle Compression Modes
#define NO_TRIANGLE_COMPRESSION        0
#define RESERVED                       1
#define PAIR_TRIANGLE_COMPRESSION      2
#define AUTO_TRIANGLE_COMPRESSION      3

//=====================================================================================================================
// Amount of ULPs(Unit in Last Place) added to Box node when using hardware intersection instruction
#define BOX_EXPANSION_DEFAULT_AMOUNT 6

//=====================================================================================================================
// Box sorting heuristic value
// 0: closethit
#if GPURT_BUILD_RTIP2
// 1: LargestFirst
// 2: ClosestMidpoint
#endif
// 3: undefined / disabled
#if GPURT_BUILD_RTIP2
// 4: LargestFirstOrClosest (auto select with rayFlag)
// 5: BoxSortLargestFirstOrClosestMidPoint  (auto select with rayFlag)
#endif
// 6: DisabledOnAcceptFirstHit (disable if bvhNode sort is on, and rayFlag is AcceptFirstHit)
//
// This need to match ILC_BOX_SORT_HEURISTIC_MODE
#ifdef AMD_VULKAN_GLSLANG
//=====================================================================================================================
///@note Enum is a reserved keyword in glslang. To workaround this limitation, define static constants to replace the
///      HLSL enums that follow for compatibility.
//=====================================================================================================================
namespace BoxSortHeuristic
{
    static const uint Closest = 0x0;
#if GPURT_BUILD_RTIP2
    static const uint Largest                       = 0x1;
    static const uint MidPoint                      = 0x2;
#endif
    static const uint Disabled                      = 0x3;
#if GPURT_BUILD_RTIP2
    static const uint LargestFirstOrClosest         = 0x4;
    static const uint LargestFirstOrClosestMidPoint = 0x5;
#endif
    static const uint DisabledOnAcceptFirstHit      = 0x6;
}
#else
enum BoxSortHeuristic : uint
{
    Closest                       = 0x0,
#if GPURT_BUILD_RTIP2
    Largest                       = 0x1,
    MidPoint                      = 0x2,
#endif
    Disabled                      = 0x3,
#if GPURT_BUILD_RTIP2
    LargestFirstOrClosest         = 0x4,
    LargestFirstOrClosestMidPoint = 0x5,
#endif
    DisabledOnAcceptFirstHit      = 0x6,
};
#endif

//=====================================================================================================================
// Bvh build node sort heuristic
// 0: SurfaceAreaLargestFirst
// 1: SurfaceAreaSmallestFirst
// 2: DensityLargestFirst
// 3: DensitySmallestFirst

#ifdef AMD_VULKAN_GLSLANG
//=====================================================================================================================
///@note Enum is a reserved keyword in glslang. To workaround this limitation, define static constants to replace the
///      HLSL enums that follow for compatibility.
//=====================================================================================================================
namespace BvhBuilderNodeSortHeuristic
{
    static const uint SurfaceAreaLargestFirst  = 0x0;
    static const uint SurfaceAreaSmallestFirst = 0x1;
    static const uint DensityLargestFirst      = 0x2;
    static const uint DensitySmallestFirst     = 0x3;
}
#else
enum BvhBuilderNodeSortHeuristic : uint
{
    SurfaceAreaLargestFirst  = 0x0,
    SurfaceAreaSmallestFirst = 0x1,
    DensityLargestFirst      = 0x2,
    DensitySmallestFirst     = 0x3,
};
#endif

#ifdef AMD_VULKAN_GLSLANG
//=====================================================================================================================
///@note Enum is a reserved keyword in glslang. To workaround this limitation, define static constants to replace the
///      HLSL enums that follow for compatibility.
//=====================================================================================================================
namespace BvhBuilderNodeSortType
{
    static const uint SortOff               = 0x0;
    static const uint FourWaySortOnTLAS     = 0x1;
    static const uint FourWaySortOnBLAS     = 0x2;
    static const uint FourWaySortOnBoth     = 0x3;
    static const uint TwoLevelSortOnTLAS    = 0x4;
    static const uint TwoLevelSortOnBLAS    = 0x5;
    static const uint TwoLevelSortOnBoth    = 0x6;
}
#else
enum BvhBuilderNodeSortType : uint
{
    SortOff             = 0x0,
    FourWaySortOnTLAS   = 0x1,
    FourWaySortOnBLAS   = 0x2,
    FourWaySortOnBoth   = 0x3,
    TwoLevelSortOnTLAS  = 0x4,
    TwoLevelSortOnBLAS  = 0x5,
    TwoLevelSortOnBoth  = 0x6
};
#endif

#ifdef AMD_VULKAN_GLSLANG
//=====================================================================================================================
///@note Enum is a reserved keyword in glslang. To workaround this limitation, define static constants to replace the
///      HLSL enums that follow for compatibility.
//=====================================================================================================================
namespace SceneBoundsCalculation
{
    static const uint SceneBoundsBasedOnGeometry = 0x0;
    static const uint SceneBoundsBasedOnGeometryWithSize = 0x1;
    static const uint SceneBoundsBasedOnCentroidWithSize = 0x2;
}
#else
enum SceneBoundsCalculation : uint
{
    SceneBoundsBasedOnGeometry = 0x0,
    SceneBoundsBasedOnGeometryWithSize = 0x1,
    SceneBoundsBasedOnCentroidWithSize = 0x2
};
#endif

//=====================================================================================================================
// Options for where FP16 box nodes are created within BLAS for QBVH
#define NO_NODES_IN_BLAS_AS_FP16           0
#define LEAF_NODES_IN_BLAS_AS_FP16         1
#define MIXED_NODES_IN_BLAS_AS_FP16        2
#define ALL_INTERIOR_NODES_IN_BLAS_AS_FP16 3

// Mask for MSB within node pointer - used to mark nodes in RefitBounds
#define NODE_POINTER_MASK_MSB              0x80000000u

//=====================================================================================================================
#define BVH_NODE_STRIDE_SHIFT                 6
#define INSTANCE_NODE_STRIDE_SHIFT            7
#define QBVH_NODE_32_STRIDE_SHIFT             7   // Box 32 node
#define QBVH_NODE_16_STRIDE_SHIFT             6   // Box 16 node
#define QBVH_LEAF_STRIDE_SHIFT                6
#define QBVH_COMPRESSED_NODE_STRIDE_SHIFT     6

#define INVALID_IDX     0xffffffff

#define COMPRESS_SHADER_PROGRESS_COUNTERS 1

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

//=====================================================================================================================
// Common structure definitions
typedef uint64_t GpuVirtualAddress;

#define GPU_VIRTUAL_ADDRESS_SIZE 8

#ifdef __cplusplus
static_assert(GPU_VIRTUAL_ADDRESS_SIZE == sizeof(GpuVirtualAddress), "GPU Virtual Address mismatch");
#endif

//=====================================================================================================================
struct BoundingBox // matches D3D12_RAYTRACING_AABB
{
    float3 min;
    float3 max;
};

//=====================================================================================================================
// Internal bounding box type for scene bounds.
struct UintBoundingBox
{
    uint3 min;
    uint3 max;
};

struct UintBoundingBox4
{
    uint4 min;
    uint4 max;
};

struct PackedUintBoundingBox4
{
    uint64_t min;
    uint64_t max;
};

//=====================================================================================================================
struct AccelStructMetadataHeader
{
    uint addressLo;   // Address of acceleration structure data section (low bits)
    uint addressHi;   // Address of acceleration structure data section (high bits)
    uint sizeInBytes; // Metadata size in bytes for tooling purposes (including this header)
    uint reserved0;
    uint taskCounter; // Task counter for dispatch-wide spin loop synchronization
    uint numTasksDone;// Number of tasks done
    uint reserved1;
    uint reserved2;
    uint reserved3;
};

#define ACCEL_STRUCT_METADATA_VA_LO_OFFSET              0
#define ACCEL_STRUCT_METADATA_VA_HI_OFFSET              4
#define ACCEL_STRUCT_METADATA_SIZE_OFFSET               8
#define ACCEL_STRUCT_METADATA_RESERVED_0                12
#define ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET       16
#define ACCEL_STRUCT_METADATA_NUM_TASKS_DONE_OFFSET     20
#define ACCEL_STRUCT_METADATA_RESERVED_1                24
#define ACCEL_STRUCT_METADATA_RESERVED_2                28
#define ACCEL_STRUCT_METADATA_RESERVED_3                32
#define ACCEL_STRUCT_METADATA_HEADER_SIZE               36

#ifdef __cplusplus
static_assert(ACCEL_STRUCT_METADATA_HEADER_SIZE         == sizeof(AccelStructMetadataHeader), "Acceleration structure header mismatch");
static_assert(ACCEL_STRUCT_METADATA_VA_LO_OFFSET        == offsetof(AccelStructMetadataHeader, addressLo), "");
static_assert(ACCEL_STRUCT_METADATA_VA_HI_OFFSET        == offsetof(AccelStructMetadataHeader, addressHi), "");
static_assert(ACCEL_STRUCT_METADATA_SIZE_OFFSET         == offsetof(AccelStructMetadataHeader, sizeInBytes), "");
static_assert(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET == offsetof(AccelStructMetadataHeader, taskCounter), "");
#endif

//=====================================================================================================================
// Acceleration structure result data offsets
struct AccelStructOffsets
{
    uint internalNodes;     // Offset to internal box nodes
    uint leafNodes;         // Offset to leaf nodes
    uint geometryInfo;      // Offset to geometry desc info (bottom level only)
    uint primNodePtrs;      // Offset to prim node pointers (BVH4 with triangle compression or ALLOW_UPDATE only)
};

#define ACCEL_STRUCT_OFFSETS_INTERNAL_NODES_OFFSET        0
#define ACCEL_STRUCT_OFFSETS_LEAF_NODES_OFFSET            4
#define ACCEL_STRUCT_OFFSETS_GEOMETRY_INFO_OFFSET         8
#define ACCEL_STRUCT_OFFSETS_PRIM_NODE_PTRS_OFFSET       12
#define ACCEL_STRUCT_OFFSETS_SIZE                        16

#ifdef __cplusplus
static_assert(ACCEL_STRUCT_OFFSETS_SIZE == sizeof(AccelStructOffsets), "Acceleration structure offsets mismatch");
static_assert(ACCEL_STRUCT_OFFSETS_INTERNAL_NODES_OFFSET    == offsetof(AccelStructOffsets, internalNodes), "");
static_assert(ACCEL_STRUCT_OFFSETS_LEAF_NODES_OFFSET        == offsetof(AccelStructOffsets, leafNodes), "");
static_assert(ACCEL_STRUCT_OFFSETS_GEOMETRY_INFO_OFFSET     == offsetof(AccelStructOffsets, geometryInfo), "");
static_assert(ACCEL_STRUCT_OFFSETS_PRIM_NODE_PTRS_OFFSET    == offsetof(AccelStructOffsets, primNodePtrs), "");
#endif

//=====================================================================================================================
#ifdef __cplusplus
union RayTracingAccelStructBuildInfo
{
    struct
    {
        uint type                       : 1;  /// AccelStructType
        uint buildType                  : 1;  /// AccelStructBuilderType
        uint mode                       : 4;  /// BvhBuildMode/BvhCpuBuildMode based on buildType
        uint triCompression             : 3;  /// TriangleCompressionMode (Bottom level only)
        uint fp16BoxNodesInBlasMode     : 2;  /// fp16BoxNodesInBlasMode
        uint triangleSplitting          : 1;  /// enable TriangleSplitting
        uint rebraid                    : 1;  /// enable Rebraid
        uint fusedInstanceNode          : 1;  /// Acceleration structure uses fused instance nodes
        uint reserved                   : 2;  /// Unused bits
        uint flags                      : 16; /// AccelStructBuildFlags
    };

    uint u32All;
};
#endif

#define ACCEL_STRUCT_HEADER_INFO_TYPE_SHIFT                             0
#define ACCEL_STRUCT_HEADER_INFO_TYPE_MASK                              0x1
#define ACCEL_STRUCT_HEADER_INFO_BUILD_TYPE_SHIFT                       1
#define ACCEL_STRUCT_HEADER_INFO_BUILD_TYPE_MASK                        0x1
#define ACCEL_STRUCT_HEADER_INFO_MODE_SHIFT                             2
#define ACCEL_STRUCT_HEADER_INFO_MODE_MASK                              0xf
#define ACCEL_STRUCT_HEADER_INFO_TRI_COMPRESS_SHIFT                     6
#define ACCEL_STRUCT_HEADER_INFO_TRI_COMPRESS_MASK                      0x7
#define ACCEL_STRUCT_HEADER_INFO_FP16_BOXNODE_IN_BLAS_MODE_SHIFT        9
#define ACCEL_STRUCT_HEADER_INFO_FP16_BOXNODE_IN_BLAS_MODE_MASK         0x3
#define ACCEL_STRUCT_HEADER_INFO_TRIANGLE_SPLITTING_FLAGS_SHIFT         11
#define ACCEL_STRUCT_HEADER_INFO_TRIANGLE_SPLITTING_FLAGS_MASK          0x1
#define ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_SHIFT                    12
#define ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_MASK                     0x1
#define ACCEL_STRUCT_HEADER_INFO_FUSED_INSTANCE_NODE_FLAGS_SHIFT        13
#define ACCEL_STRUCT_HEADER_INFO_FUSED_INSTANCE_NODE_FLAGS_MASK         0x1
#define ACCEL_STRUCT_HEADER_INFO_FLAGS_SHIFT                            16
#define ACCEL_STRUCT_HEADER_INFO_FLAGS_MASK                             0xffff

//=====================================================================================================================
#ifdef __cplusplus
union RayTracingAccelStructBuildInfo2
{
    struct
    {
        uint compacted     : 1;       /// This BVH has been compacted
        uint reserved      : 1;       /// Unused bits
        uint reserved2     : 30;      /// Unused bits
    };

    uint u32All;
};
#endif

#define ACCEL_STRUCT_HEADER_INFO_2_BVH_COMPACTION_FLAGS_SHIFT    0
#define ACCEL_STRUCT_HEADER_INFO_2_BVH_COMPACTION_FLAGS_MASK     0x1

//=====================================================================================================================
// 128 byte aligned acceleration structure header
struct AccelStructHeader
{
    uint               info;                    // Type of acceleration structure (Top level or bottom level)
    uint               metadataSizeInBytes;     // Total size of the metadata in bytes
    uint               sizeInBytes;             // Total size of the structure in bytes (Including this header)
    uint               numPrimitives;           // Number of primitives encoded in the structure
    uint               numActivePrims;          // Tracks the number of active prims to add to bvh
    uint               taskIdCounter;           // Counter for allocting IDs to tasks in a persistent thread group
    uint               numDescs;                // Number of instance/geometry descs in the structure
    uint               geometryType;            // Type of geometry contained in the bottom level structure
    AccelStructOffsets offsets;                 // Offsets to different sections of the acceleration structure
    uint               numInternalNodesFp32;    // Number of fp32 internal nodes used by the acceleration structure
    uint               numInternalNodesFp16;    // Number of fp16 internal nodes used by the acceleration structure
    uint               numLeafNodes;            // Number of leaf nodes used by the acceleration structure
    uint               version;                 // GPURT version
    uint               uuidLo;                  // Compatibility info - uuidLo
    uint               uuidHi;                  // Compatibility info - uuidHi
    uint               reserved;                // Reserved bits
    uint               fp32RootBoxNode0;        // Root Float32BoxNode for bottom level acceleration structures
    uint               fp32RootBoxNode1;
    uint               fp32RootBoxNode2;
    uint               fp32RootBoxNode3;
    uint               fp32RootBoxNode4;
    uint               fp32RootBoxNode5;
    uint               info2;                   // Acceleration structure information
    uint               nodeFlags;               // BLAS node flags
    uint               compactedSizeInBytes;    // Total compacted size of the accel struct
    uint               numChildPrims0;          // Number of primitives for 4 children for rebraid
    uint               numChildPrims1;
    uint               numChildPrims2;
    uint               numChildPrims3;
};

#define ACCEL_STRUCT_HEADER_SIZE                               128
#define ACCEL_STRUCT_HEADER_INFO_OFFSET                          0
#define ACCEL_STRUCT_HEADER_METADATA_SIZE_OFFSET                 4
#define ACCEL_STRUCT_HEADER_BYTE_SIZE_OFFSET                     8
#define ACCEL_STRUCT_HEADER_NUM_PRIMS_OFFSET                    12
#define ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET             16
#define ACCEL_STRUCT_HEADER_TASK_ID_COUNTER                     20
#define ACCEL_STRUCT_HEADER_NUM_DESCS_OFFSET                    24
#define ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET                28
#define ACCEL_STRUCT_HEADER_OFFSETS_OFFSET                      32
#define ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET      ACCEL_STRUCT_HEADER_OFFSETS_OFFSET + ACCEL_STRUCT_OFFSETS_SIZE
#define ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP16_NODES_OFFSET      ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET      + 4
#define ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET               ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP16_NODES_OFFSET      + 4
#define ACCEL_STRUCT_HEADER_VERSION_OFFSET                      ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET               + 4
#define ACCEL_STRUCT_HEADER_UUID_LO_OFFSET                      ACCEL_STRUCT_HEADER_VERSION_OFFSET                      + 4
#define ACCEL_STRUCT_HEADER_UUID_HI_OFFSET                      ACCEL_STRUCT_HEADER_UUID_LO_OFFSET                      + 4
#define ACCEL_STRUCT_HEADER_RESERVED_OFFSET                     ACCEL_STRUCT_HEADER_UUID_HI_OFFSET                      + 4
#define ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET                ACCEL_STRUCT_HEADER_RESERVED_OFFSET                     + 4
#define ACCEL_STRUCT_HEADER_INFO_2_OFFSET                       ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET                + 24
#define ACCEL_STRUCT_HEADER_NODE_FLAGS_OFFSET                   ACCEL_STRUCT_HEADER_INFO_2_OFFSET                       + 4
#define ACCEL_STRUCT_HEADER_COMPACTED_BYTE_SIZE_OFFSET          ACCEL_STRUCT_HEADER_NODE_FLAGS_OFFSET                   + 4
#define ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET              ACCEL_STRUCT_HEADER_COMPACTED_BYTE_SIZE_OFFSET          + 4

// Acceleration structure root node starts immediately after the header
#define ACCEL_STRUCT_ROOT_NODE_OFFSET ACCEL_STRUCT_HEADER_SIZE

#ifdef __cplusplus
static_assert(ACCEL_STRUCT_HEADER_SIZE == sizeof(AccelStructHeader), "Acceleration structure header mismatch");
static_assert(ACCEL_STRUCT_HEADER_INFO_OFFSET                         == offsetof(AccelStructHeader, info),                 "");
static_assert(ACCEL_STRUCT_HEADER_METADATA_SIZE_OFFSET                == offsetof(AccelStructHeader, metadataSizeInBytes),  "");
static_assert(ACCEL_STRUCT_HEADER_BYTE_SIZE_OFFSET                    == offsetof(AccelStructHeader, sizeInBytes),          "");
static_assert(ACCEL_STRUCT_HEADER_NUM_PRIMS_OFFSET                    == offsetof(AccelStructHeader, numPrimitives),        "");
static_assert(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET             == offsetof(AccelStructHeader, numActivePrims),       "");
static_assert(ACCEL_STRUCT_HEADER_TASK_ID_COUNTER                     == offsetof(AccelStructHeader, taskIdCounter),       "");
static_assert(ACCEL_STRUCT_HEADER_NUM_DESCS_OFFSET                    == offsetof(AccelStructHeader, numDescs),             "");
static_assert(ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET                == offsetof(AccelStructHeader, geometryType),         "");
static_assert(ACCEL_STRUCT_HEADER_OFFSETS_OFFSET                      == offsetof(AccelStructHeader, offsets),              "");
static_assert(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET      == offsetof(AccelStructHeader, numInternalNodesFp32), "");
static_assert(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP16_NODES_OFFSET      == offsetof(AccelStructHeader, numInternalNodesFp16), "");
static_assert(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET               == offsetof(AccelStructHeader, numLeafNodes),         "");
static_assert(ACCEL_STRUCT_HEADER_VERSION_OFFSET                      == offsetof(AccelStructHeader, version),               "");
static_assert(ACCEL_STRUCT_HEADER_UUID_LO_OFFSET                      == offsetof(AccelStructHeader, uuidLo),               "");
static_assert(ACCEL_STRUCT_HEADER_UUID_HI_OFFSET                      == offsetof(AccelStructHeader, uuidHi),               "");
static_assert(ACCEL_STRUCT_HEADER_RESERVED_OFFSET                     == offsetof(AccelStructHeader, reserved),             "");
static_assert(ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET                == offsetof(AccelStructHeader, fp32RootBoxNode0),     "");
static_assert(ACCEL_STRUCT_HEADER_INFO_2_OFFSET                       == offsetof(AccelStructHeader, info2),                "");
static_assert(ACCEL_STRUCT_HEADER_NODE_FLAGS_OFFSET                   == offsetof(AccelStructHeader, nodeFlags),            "");
static_assert(ACCEL_STRUCT_HEADER_COMPACTED_BYTE_SIZE_OFFSET          == offsetof(AccelStructHeader, compactedSizeInBytes), "");
static_assert(ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET              == offsetof(AccelStructHeader, numChildPrims0),       "");
#endif

//=====================================================================================================================
// Hardware 32-bit box node format and offsets
#define FLOAT32_BBOX_STRIDE                 24
#define FLOAT32_BOX_NODE_CHILD0_OFFSET      0
#define FLOAT32_BOX_NODE_CHILD1_OFFSET      4
#define FLOAT32_BOX_NODE_CHILD2_OFFSET      8
#define FLOAT32_BOX_NODE_CHILD3_OFFSET      12
#define FLOAT32_BOX_NODE_BB0_MIN_OFFSET     16
#define FLOAT32_BOX_NODE_BB0_MAX_OFFSET     28
#define FLOAT32_BOX_NODE_BB1_MIN_OFFSET     40
#define FLOAT32_BOX_NODE_BB1_MAX_OFFSET     52
#define FLOAT32_BOX_NODE_BB2_MIN_OFFSET     64
#define FLOAT32_BOX_NODE_BB2_MAX_OFFSET     76
#define FLOAT32_BOX_NODE_BB3_MIN_OFFSET     88
#define FLOAT32_BOX_NODE_BB3_MAX_OFFSET     100
#define FLOAT32_BOX_NODE_FLAGS_OFFSET       112
#define FLOAT32_BOX_NODE_NUM_PRIM_OFFSET    116
#define FLOAT32_BOX_NODE_UNUSED2_OFFSET     120
#define FLOAT32_BOX_NODE_UNUSED3_OFFSET     124
#define FLOAT32_BOX_NODE_SIZE               128

//=====================================================================================================================
// Float32 box node flags contains 4 1-byte fields, 1 per child node:
// Child 0 [ 7: 0]
// Child 1 [15: 8]
// Child 2 [23:16]
// Child 3 [31:24]
//
// Each child node's 1-byte field contains these flags:
// Only Opaque     [  0]
// Only Non-Opaque [  1]
// Only Triangles  [  2]
// Only Procedural [  3]
// Unused          [7:4]
#define BOX_NODE_FLAGS_BIT_STRIDE 8

#define BOX_NODE_FLAGS_ONLY_OPAQUE_SHIFT     0
#define BOX_NODE_FLAGS_ONLY_NON_OPAQUE_SHIFT 1
#define BOX_NODE_FLAGS_ONLY_TRIANGLES_SHIFT  2
#define BOX_NODE_FLAGS_ONLY_PROCEDURAL_SHIFT 3

//=====================================================================================================================
struct Float32BoxNode
{
    uint   child0;    /// Child node pointer 0
    uint   child1;    /// Child node pointer 1
    uint   child2;    /// Child node pointer 2
    uint   child3;    /// Child node pointer 3

    float3 bbox0_min; /// Node bounding box 0 minimum bounds
    float3 bbox0_max; /// Node bounding box 0 maximum bounds

    float3 bbox1_min; /// Node bounding box 1 minimum bounds
    float3 bbox1_max; /// Node bounding box 1 maximum bounds

    float3 bbox2_min; /// Node bounding box 2 minimum bounds
    float3 bbox2_max; /// Node bounding box 2 maximum bounds

    float3 bbox3_min; /// Node bounding box 3 minimum bounds
    float3 bbox3_max; /// Node bounding box 3 maximum bounds

#if GPURT_BUILD_RTIP2
    uint   flags;          /// Reserved for RTIP 2.0
#else
    uint   flags;
#endif
    uint   numPrimitives;  /// Padding for 64-byte alignment
    uint   padding2;       /// Padding for 64-byte alignment
    uint   padding3;       /// Padding for 64-byte alignment

#ifdef __cplusplus
    // parameterised constructor for HLSL compatibility
    Float32BoxNode(uint val)
    {
        memset(this, val, sizeof(Float32BoxNode));
    }

    // default constructor
    Float32BoxNode() : Float32BoxNode(0)
    {
    }
#endif
};

#ifdef __cplusplus
static_assert(FLOAT32_BOX_NODE_SIZE == sizeof(Float32BoxNode), "Float32BoxNode structure mismatch");
static_assert(FLOAT32_BOX_NODE_CHILD0_OFFSET   == offsetof(Float32BoxNode, child0), "");
static_assert(FLOAT32_BOX_NODE_CHILD1_OFFSET   == offsetof(Float32BoxNode, child1), "");
static_assert(FLOAT32_BOX_NODE_CHILD2_OFFSET   == offsetof(Float32BoxNode, child2), "");
static_assert(FLOAT32_BOX_NODE_CHILD3_OFFSET   == offsetof(Float32BoxNode, child3), "");
static_assert(FLOAT32_BOX_NODE_BB0_MIN_OFFSET  == offsetof(Float32BoxNode, bbox0_min), "");
static_assert(FLOAT32_BOX_NODE_BB0_MAX_OFFSET  == offsetof(Float32BoxNode, bbox0_max), "");
static_assert(FLOAT32_BOX_NODE_BB1_MIN_OFFSET  == offsetof(Float32BoxNode, bbox1_min), "");
static_assert(FLOAT32_BOX_NODE_BB1_MAX_OFFSET  == offsetof(Float32BoxNode, bbox1_max), "");
static_assert(FLOAT32_BOX_NODE_BB2_MIN_OFFSET  == offsetof(Float32BoxNode, bbox2_min), "");
static_assert(FLOAT32_BOX_NODE_BB2_MAX_OFFSET  == offsetof(Float32BoxNode, bbox2_max), "");
static_assert(FLOAT32_BOX_NODE_BB3_MIN_OFFSET  == offsetof(Float32BoxNode, bbox3_min), "");
static_assert(FLOAT32_BOX_NODE_BB3_MAX_OFFSET  == offsetof(Float32BoxNode, bbox3_max), "");
static_assert(FLOAT32_BOX_NODE_FLAGS_OFFSET    == offsetof(Float32BoxNode, flags), "");
static_assert(FLOAT32_BOX_NODE_NUM_PRIM_OFFSET == offsetof(Float32BoxNode, numPrimitives), "");
static_assert(FLOAT32_BOX_NODE_UNUSED2_OFFSET  == offsetof(Float32BoxNode, padding2), "");
static_assert(FLOAT32_BOX_NODE_UNUSED3_OFFSET  == offsetof(Float32BoxNode, padding3), "");
#endif

//=====================================================================================================================
// Hardware 16-bit box node format and offsets
#define FLOAT16_BBOX_STRIDE                 12
#define FLOAT16_BOX_NODE_CHILD0_OFFSET      0
#define FLOAT16_BOX_NODE_CHILD1_OFFSET      4
#define FLOAT16_BOX_NODE_CHILD2_OFFSET      8
#define FLOAT16_BOX_NODE_CHILD3_OFFSET      12
#define FLOAT16_BOX_NODE_BB0_OFFSET         16
#define FLOAT16_BOX_NODE_BB1_OFFSET         28
#define FLOAT16_BOX_NODE_BB2_OFFSET         40
#define FLOAT16_BOX_NODE_BB3_OFFSET         52
#define FLOAT16_BOX_NODE_SIZE               64

//=====================================================================================================================
struct Float16BoxNode
{
    uint  child0;   /// Child node pointer 0
    uint  child1;   /// Child node pointer 1
    uint  child2;   /// Child node pointer 2
    uint  child3;   /// Child node pointer 3

    uint3 bbox0;    /// Node bounding box 0, packed, uses float16: minx, miny | minz, maxx | maxy, maxz
    uint3 bbox1;    /// Node bounding box 1, packed, uses float16: minx, miny | minz, maxx | maxy, maxz
    uint3 bbox2;    /// Node bounding box 2, packed, uses float16: minx, miny | minz, maxx | maxy, maxz
    uint3 bbox3;    /// Node bounding box 3, packed, uses float16: minx, miny | minz, maxx | maxy, maxz

    // NOTE: each bounding box is defined as uint3 for simplicity
    // Each 32 bits pack 2x float16s. Order above is written as: a, b
    // with a located in the lower 16 bits, b in the upper 16 bits
    // bbox0.x stores minx, miny
    //
    // Alternatively, one can define each bbox as a pair of float16_t3
    // similar to FLOAT32_BOX_NODE. Indexing in hlsl would require extra work
};

#ifdef __cplusplus
static_assert(FLOAT16_BOX_NODE_SIZE == sizeof(Float16BoxNode), "Float16BoxNode structure mismatch");
static_assert(FLOAT16_BOX_NODE_CHILD0_OFFSET == offsetof(Float16BoxNode, child0), "");
static_assert(FLOAT16_BOX_NODE_CHILD1_OFFSET == offsetof(Float16BoxNode, child1), "");
static_assert(FLOAT16_BOX_NODE_CHILD2_OFFSET == offsetof(Float16BoxNode, child2), "");
static_assert(FLOAT16_BOX_NODE_CHILD3_OFFSET == offsetof(Float16BoxNode, child3), "");
static_assert(FLOAT16_BOX_NODE_BB0_OFFSET    == offsetof(Float16BoxNode, bbox0), "");
static_assert(FLOAT16_BOX_NODE_BB1_OFFSET    == offsetof(Float16BoxNode, bbox1), "");
static_assert(FLOAT16_BOX_NODE_BB2_OFFSET    == offsetof(Float16BoxNode, bbox2), "");
static_assert(FLOAT16_BOX_NODE_BB3_OFFSET    == offsetof(Float16BoxNode, bbox3), "");
#endif

//=====================================================================================================================
// Hardware triangle node format and offsets
#define TRIANGLE_NODE_V0_OFFSET 0
#define TRIANGLE_NODE_V1_OFFSET 12
#define TRIANGLE_NODE_V2_OFFSET 24
#define TRIANGLE_NODE_V3_OFFSET 36
#define TRIANGLE_NODE_V4_OFFSET 48
#define TRIANGLE_NODE_ID_OFFSET 60
#define TRIANGLE_NODE_SIZE      64

//=====================================================================================================================
// Triangle node sideband data offsets when limiting triangle compression to 2 triangles per node
// Geometry Index in bottom 24 bits and Geometry Flags in bits 25-26
#define TRIANGLE_NODE_GEOMETRY_INDEX_AND_FLAGS_OFFSET TRIANGLE_NODE_V4_OFFSET
#define TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET         TRIANGLE_NODE_V4_OFFSET + 4
#define TRIANGLE_NODE_PRIMITIVE_INDEX1_OFFSET         TRIANGLE_NODE_V4_OFFSET + 8
#define TRIANGLE_NODE_PARENT_PTR_OFFSET               TRIANGLE_NODE_V3_OFFSET

//=====================================================================================================================
// Triangle ID contains 4 1-byte fields, 1 per triangle:
// Triangle 0 [ 7: 0]
// Triangle 1 [15: 8]
// Triangle 2 [23:16]
// Triangle 3 [31:24]
//
// Each triangle's 8-bit segment contains these fields:
// I SRC        [1:0] Specifies which vertex in triangle 0 corresponds to the I barycentric value
// J SRC        [3:2] Specifies which vertex in triangle 0 corresponds to the J barycentric value
#if GPURT_BUILD_RTIP2
// Double Sided [  4] Specifies whether triangle 0 should be treated as double sided for culling
// Flip Winding [  5] Specifies whether triangle 0 should have its facedness flipped
// Procedural   [  6] Specifies whether it is a procedural node
// Opaque       [  7] Specifies whether triangle 0 should be considered as opaque
#endif
#define TRIANGLE_ID_BIT_STRIDE 8

#define TRIANGLE_ID_I_SRC_SHIFT        0
#define TRIANGLE_ID_J_SRC_SHIFT        2
#if GPURT_BUILD_RTIP2
#define TRIANGLE_ID_DOUBLE_SIDED_SHIFT 4
#define TRIANGLE_ID_FLIP_WINDING_SHIFT 5
#define TRIANGLE_ID_PROCEDURAL_SHIFT   6
#define TRIANGLE_ID_OPAQUE_SHIFT       7
#endif

//=====================================================================================================================
struct TriangleNode
{
    float3 v0;         // Vertex 0
    float3 v1;         // Vertex 1
    float3 v2;         // Vertex 2
    float3 v3;         // Vertex 3
    float3 v4;         // Vertex 4
    uint   triangleId; // Triangle ID
};

#ifdef __cplusplus
static_assert(TRIANGLE_NODE_SIZE == sizeof(TriangleNode), "TriangleNode structure mismatch");
static_assert(TRIANGLE_NODE_V0_OFFSET == offsetof(TriangleNode, v0), "");
static_assert(TRIANGLE_NODE_V1_OFFSET == offsetof(TriangleNode, v1), "");
static_assert(TRIANGLE_NODE_V2_OFFSET == offsetof(TriangleNode, v2), "");
static_assert(TRIANGLE_NODE_V3_OFFSET == offsetof(TriangleNode, v3), "");
static_assert(TRIANGLE_NODE_V4_OFFSET == offsetof(TriangleNode, v4), "");
static_assert(TRIANGLE_NODE_ID_OFFSET == offsetof(TriangleNode, triangleId), "");
#endif

//=====================================================================================================================
#define USER_NODE_PROCEDURAL_MIN_OFFSET 0
#define USER_NODE_PROCEDURAL_MAX_OFFSET 12
#define USER_NODE_PROCEDURAL_SIZE       64

//=====================================================================================================================
// Procedural node primitive data offsets
#define USER_NODE_PROCEDURAL_PRIMITIVE_INDEX_OFFSET            TRIANGLE_NODE_PRIMITIVE_INDEX1_OFFSET
#define USER_NODE_PROCEDURAL_GEOMETRY_INDEX_AND_FLAGS_OFFSET   TRIANGLE_NODE_GEOMETRY_INDEX_AND_FLAGS_OFFSET
#define USER_NODE_PROCEDURAL_TRIANGLE_ID_OFFSET                TRIANGLE_NODE_ID_OFFSET

//=====================================================================================================================
// User defined procedural node format
struct ProceduralNode
{
    float3 bbox_min;
    float3 bbox_max;
    uint   padding1[6];
    uint   reserved;
    uint   primitiveIndex;
    uint   triangleId;
};

#ifdef __cplusplus
static_assert(USER_NODE_PROCEDURAL_SIZE == sizeof(ProceduralNode), "ProceduralNode structure mismatch");
static_assert(USER_NODE_PROCEDURAL_MIN_OFFSET                       == offsetof(ProceduralNode, bbox_min), "");
static_assert(USER_NODE_PROCEDURAL_MAX_OFFSET                       == offsetof(ProceduralNode, bbox_max), "");
static_assert(USER_NODE_PROCEDURAL_GEOMETRY_INDEX_AND_FLAGS_OFFSET  == offsetof(ProceduralNode, geometryIndexAndFlags), "");
static_assert(USER_NODE_PROCEDURAL_PRIMITIVE_INDEX_OFFSET           == offsetof(ProceduralNode, primitiveIndex), "");
static_assert(USER_NODE_PROCEDURAL_TRIANGLE_ID_OFFSET               == offsetof(ProceduralNode, triangleId), "");
#endif

#ifdef __cplusplus
//=====================================================================================================================
union NodePointer32
{
    struct
    {
        uint32_t type               :  3; // Hardware NODE_TYPE_*
        uint32_t aligned_offset_64b : 29; // 64-byte aligned offset
    };

    uint32_t u32;
};

#if GPURT_BUILD_RTIP2
//=====================================================================================================================
// Instance base pointer layout from the HW raytracing IP 2.0 spec:
// Zero                         [ 2: 0]
// Tree Base Address (64B index)[53: 3]
// Force Opaque                 [   54]
// Force Non-Opaque             [   55]
// Disable Triangle Cull        [   56]
// Flip Facedness               [   57]
// Cull Back Facing Triangles   [   58]
// Cull Front Facing Triangles  [   59]
// Cull Opaque                  [   60]
// Cull Non-Opaque              [   61]
// Skip Triangles               [   62]
// Skip Procedural              [   63]
#endif
union NodePointer64
{
    struct
    {
        uint64_t type                     : 3;   // Hardware NODE_TYPE_*
        uint64_t aligned_addr_64b         : 51;  // 64-byte aligned address
        uint64_t force_opaque             : 1;
        uint64_t force_non_opaque         : 1;
        uint64_t disable_triangle_cull    : 1;
        uint64_t flip_facedness           : 1;
        uint64_t cull_back_face_triangle  : 1;
        uint64_t cull_front_face_triangle : 1;
        uint64_t cull_opaque              : 1;
        uint64_t cull_non_opaque          : 1;
        uint64_t skip_triangles           : 1;
        uint64_t skip_procedural          : 1;
    };

    uint64_t u64;
};

//=====================================================================================================================
union HwTriangleFlags
{
    struct
    {
        uint8_t i            : 2;
        uint8_t j            : 2;
        uint8_t double_sided : 1;
        uint8_t flip_winding : 1;
        uint8_t unused       : 1;
        uint8_t opaque       : 1;
    };

    uint8_t u8;
};

//=====================================================================================================================
union HwTriangleID
{
    struct
    {
        HwTriangleFlags triangle0;
        HwTriangleFlags triangle1;
        uint16_t        unused;
    };

    uint32_t u32;
};

//=====================================================================================================================
union BoxNodeChildFlags
{
    struct
    {
        uint8_t only_opaque     : 1;
        uint8_t only_non_opaque : 1;
        uint8_t only_triangles  : 1;
        uint8_t only_procedural : 1;
        uint8_t unused          : 4;
    };

    uint8_t u8All;
};

//=====================================================================================================================
union BoxNodeFlags
{
    struct
    {
        BoxNodeChildFlags child0;
        BoxNodeChildFlags child1;
        BoxNodeChildFlags child2;
        BoxNodeChildFlags child3;
    };

    uint32_t u32All;
};
#endif

//=====================================================================================================================
// Node pointer size in bytes
#define NODE_PTR_SIZE 4

#ifdef __cplusplus
static_assert(NODE_PTR_SIZE == sizeof(NodePointer32), "Node pointer size mismatch");
#endif

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
// Function assumes the type passed in is a valid node type
//
static uint PackLeafNodePointer(uint type, uint address, uint numPrimitives)
{
    return PackNodePointer(type, address) | ((numPrimitives - 1) << 29);
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
// TODO: The highest 3 bits are not handled since they currently aren't written when building the QBVH.
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

#ifdef __cplusplus
static_assert(GEOMETRY_INFO_SIZE == sizeof(GeometryInfo), "Geometry info structure mismatch");
static_assert(GEOMETRY_INFO_FLAGS_AND_NUM_PRIMS_OFFSET == offsetof(GeometryInfo, geometryFlagsAndNumPrimitives), "");
static_assert(GEOMETRY_INFO_GEOM_BUFFER_OFFSET         == offsetof(GeometryInfo, geometryBufferOffset), "");
static_assert(GEOMETRY_INFO_PRIM_NODE_PTRS_OFFSET      == offsetof(GeometryInfo, primNodePtrsOffset), "");
#endif

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
// 64-byte aligned BVH2 node structure
struct BVHNode
{
    float3 bbox_left_min_or_v0;    /// Left Node bounding box minimum bounds or vertex 0
    uint   left;                   /// Left child node pointer  (Also, primitive ID for leaves, instance ID for instances)

    float3 bbox_left_max_or_v1;    /// Left Node bounding box maximum bounds or vertex 1
    uint   right;                  /// Right child node pointer (Also, geometry Index for leaves)

    float3 bbox_right_min_or_v2;   /// Right Node bounding box min bounds or vertex 2
    uint   flags;                  /// Bottom: geometry flags OR Top: node[0] this is used to hold num instances

    float3 bbox_right_max;         /// Right node bounding box max bounds
    uint   unused;                 /// Unused
};

#define BVH_NODE_SIZE                  64
#define BVH_NODE_LEFT_MIN_OFFSET       0
#define BVH_NODE_V0_OFFSET             BVH_NODE_LEFT_MIN_OFFSET
#define BVH_NODE_LEFT_OFFSET           12
#define BVH_NODE_PRIMITIVE_ID_OFFSET   BVH_NODE_LEFT_OFFSET
#define BVH_NODE_LEFT_MAX_OFFSET       16
#define BVH_NODE_V1_OFFSET             BVH_NODE_LEFT_MAX_OFFSET
#define BVH_NODE_RIGHT_OFFSET          28
#define BVH_NODE_GEOMETRY_INDEX_OFFSET BVH_NODE_RIGHT_OFFSET
#define BVH_NODE_RIGHT_MIN_OFFSET      32
#define BVH_NODE_V2_OFFSET             BVH_NODE_RIGHT_MIN_OFFSET
#define BVH_NODE_FLAGS_OFFSET          44
#define BVH_NODE_RIGHT_MAX_OFFSET      48

#ifdef __cplusplus
static_assert(BVH_NODE_SIZE == sizeof(BVHNode), "BVH2Node structure mismatch");
static_assert(BVH_NODE_LEFT_MIN_OFFSET  == offsetof(BVHNode, bbox_left_min_or_v0), "");
static_assert(BVH_NODE_LEFT_OFFSET      == offsetof(BVHNode, left), "");
static_assert(BVH_NODE_LEFT_MAX_OFFSET  == offsetof(BVHNode, bbox_left_max_or_v1), "");
static_assert(BVH_NODE_RIGHT_OFFSET     == offsetof(BVHNode, right), "");
static_assert(BVH_NODE_RIGHT_MIN_OFFSET == offsetof(BVHNode, bbox_right_min_or_v2), "");
static_assert(BVH_NODE_FLAGS_OFFSET     == offsetof(BVHNode, flags), "");
static_assert(BVH_NODE_RIGHT_MAX_OFFSET == offsetof(BVHNode, bbox_right_max), "");
#endif

//=====================================================================================================================
// The structure is 64-byte aligned
struct ScratchNode
{
    float3 bbox_min_or_v0;
    uint   left_or_primIndex_or_instIndex; // left child for internal nodes /
                                           // primitive index for primitive nodes /
                                           // instance index for instance nodes
    float3 bbox_max_or_v1;
    uint   right_or_geometryIndex;         // right child for internal nodes /
                                           // geometryIndex if it's a leaf node
    float3 sah_or_v2_or_instBasePtr;       // SAH cost, area and density for internal nodes /
                                           // vertex2 for triangle nodes /
                                           // instanceNodeBasePointerLo, instanceNodeBasePointerHi for instance node
    uint   parent;

    uint   type;                           // type [2:0], triangle id [18:3] for max of 2 compressed tris
    uint   flags;
    uint   splitBox_or_nodePointer;        // TriangleSplitBox index for triangle nodes /
                                           // BLAS node pointer for instance nodes
    uint   numPrimitivesAndDoCollapse;     // number of tris collapsed, doCollapse is boolean bit in the LSB /
                                           // scratch node index of the tri in the pair in PAIR_TRIANGLE_COMPRESSION
};

#define SCRATCH_NODE_BBOX_MIN_OFFSET                  0
#define SCRATCH_NODE_V0_OFFSET                        SCRATCH_NODE_BBOX_MIN_OFFSET
#define SCRATCH_NODE_LEFT_OFFSET                      12
#define SCRATCH_NODE_PRIMITIVE_ID_OFFSET              SCRATCH_NODE_LEFT_OFFSET
#define SCRATCH_NODE_INSTANCE_ID_OFFSET               SCRATCH_NODE_PRIMITIVE_ID_OFFSET
#define SCRATCH_NODE_BBOX_MAX_OFFSET                  16
#define SCRATCH_NODE_V1_OFFSET                        SCRATCH_NODE_BBOX_MAX_OFFSET
#define SCRATCH_NODE_RIGHT_OFFSET                     28
#define SCRATCH_NODE_GEOMETRY_INDEX_OFFSET            SCRATCH_NODE_RIGHT_OFFSET
#define SCRATCH_NODE_COST_OFFSET                      32
#define SCRATCH_NODE_SA_OFFSET                        36
#define SCRATCH_NODE_DENSITY_OFFSET                   40
#define SCRATCH_NODE_V2_OFFSET                        SCRATCH_NODE_COST_OFFSET
#define SCRATCH_NODE_INSTANCE_BASE_PTR_OFFSET         SCRATCH_NODE_V2_OFFSET
#define SCRATCH_NODE_INSTANCE_NUM_PRIMS_OFFSET        SCRATCH_NODE_DENSITY_OFFSET
#define SCRATCH_NODE_PARENT_OFFSET                    44
#define SCRATCH_NODE_TYPE_OFFSET                      48
#define SCRATCH_NODE_FLAGS_OFFSET                     52
#define SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET           56
#define SCRATCH_NODE_NODE_POINTER_OFFSET              SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET
#define SCRATCH_NODE_NUM_MORTON_CELLS_OFFSET          SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET
#define SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET 60
#define SCRATCH_NODE_SIZE                             64

//=====================================================================================================================
// 64-byte aligned structure matching D3D12_RAYTRACING_INSTANCE_DESC
struct InstanceDesc
{
    float4 Transform[3];                                    // Inverse transform for traversal
    uint   InstanceID_and_Mask;                             // 24-bit instance ID and 8-bit mask
    uint   InstanceContributionToHitGroupIndex_and_Flags;   // 24-bit instance contribution and 8-bit flags
    uint   accelStructureAddressLo;                         // Lower part of acceleration structure base address
    uint   accelStructureAddressHiAndFlags;                 // Upper part of acceleration structure base address and
#if GPURT_BUILD_RTIP2
                                                            // HW raytracing IP 2.0 flags
#else
                                                            // flags
#endif
};

#define INSTANCE_DESC_SIZE                          64
#define INSTANCE_DESC_WORLD_TO_OBJECT_XFORM_OFFSET   0
#define INSTANCE_DESC_ID_AND_MASK_OFFSET            48
#define INSTANCE_DESC_CONTRIBUTION_AND_FLAGS_OFFSET 52
#define INSTANCE_DESC_VA_LO_OFFSET                  56
#define INSTANCE_DESC_VA_HI_OFFSET                  60

#ifdef __cplusplus
static_assert(INSTANCE_DESC_SIZE == sizeof(InstanceDesc), "InstanceDesc structure mismatch");
static_assert(INSTANCE_DESC_ID_AND_MASK_OFFSET == offsetof(InstanceDesc, InstanceID_and_Mask), "");
static_assert(INSTANCE_DESC_CONTRIBUTION_AND_FLAGS_OFFSET == offsetof(InstanceDesc, InstanceContributionToHitGroupIndex_and_Flags), "");
static_assert(INSTANCE_DESC_VA_LO_OFFSET == offsetof(InstanceDesc, accelStructureAddressLo), "");
static_assert(INSTANCE_DESC_VA_HI_OFFSET == offsetof(InstanceDesc, accelStructureAddressHiAndFlags), "");
#endif

//=====================================================================================================================
struct InstanceExtraData
{
    uint   instanceIndex;
    uint   blasNodePointer; // might not point to root
    uint   blasMetadataSize;
    uint   padding0;
    float4 Transform[3]; // Non-inverse (original D3D12_RAYTRACING_INSTANCE_DESC.Transform)
};

#define INSTANCE_EXTRA_INDEX_OFFSET         0
#define INSTANCE_EXTRA_NODE_POINTER_OFFSET  4
#define INSTANCE_EXTRA_METADATA_SIZE_OFFSET 8
#define INSTANCE_EXTRA_XFORM_OFFSET         16
#define INSTANCE_EXTRA_DATA_SIZE            64
#define INSTANCE_DESC_OBJECT_TO_WORLD_XFORM_OFFSET (INSTANCE_DESC_SIZE + INSTANCE_EXTRA_XFORM_OFFSET)

#ifdef __cplusplus
static_assert(INSTANCE_EXTRA_DATA_SIZE == sizeof(InstanceExtraData), "InstanceExtraData structure mismatch");
static_assert(INSTANCE_EXTRA_INDEX_OFFSET        == offsetof(InstanceExtraData, instanceIndex), "");
static_assert(INSTANCE_EXTRA_NODE_POINTER_OFFSET == offsetof(InstanceExtraData, blasNodePointer), "");
static_assert(INSTANCE_EXTRA_XFORM_OFFSET        == offsetof(InstanceExtraData, Transform[0]), "");
#endif

//=====================================================================================================================
struct FusedInstanceNode
{
    InstanceDesc      desc;
    InstanceExtraData extra;
    Float32BoxNode    blasRootNode;
};

//=====================================================================================================================
struct InstanceNode
{
    InstanceDesc      desc;
    InstanceExtraData extra;
};

#define INSTANCE_NODE_DESC_OFFSET       0
#define INSTANCE_NODE_EXTRA_OFFSET      64
#define INSTANCE_NODE_SIZE              128
#define FUSED_INSTANCE_NODE_ROOT_OFFSET INSTANCE_NODE_SIZE
#define FUSED_INSTANCE_NODE_SIZE        256

#ifdef __cplusplus
static_assert(INSTANCE_NODE_SIZE == sizeof(InstanceNode), "InstanceNode structure mismatch");
static_assert(INSTANCE_NODE_DESC_OFFSET == offsetof(InstanceNode, desc), "InstanceNode structure mismatch");
static_assert(INSTANCE_NODE_EXTRA_OFFSET == offsetof(InstanceNode, extra), "InstanceNode structure mismatch");
#endif

//=====================================================================================================================
static uint64_t PackUint64(uint lowBits, uint highBits)
{
    // Note glslang doesn't like uint64_t casts
    uint64_t addr = highBits;
    addr = (addr << 32) | lowBits;
    return addr;
}

#if GPURT_BUILD_RTIP2
//=====================================================================================================================
// Instance base pointer layout from the HW raytracing IP 2.0 spec:
// Zero                         [ 2: 0]
// Tree Base Address (64B index)[53: 3]
// Force Opaque                 [   54]
// Force Non-Opaque             [   55]
// Disable Triangle Cull        [   56]
// Flip Facedness               [   57]
// Cull Back Facing Triangles   [   58]
// Cull Front Facing Triangles  [   59]
// Cull Opaque                  [   60]
// Cull Non-Opaque              [   61]
// Skip Triangles               [   62]
// Skip Procedural              [   63]
//
// Since GPU VAs can only be 48 bits, only 42 bits of the Tree Base Address field are used:
// Used Address                 [44: 3]
// Unused Address               [53:45]
//
// Note glslang doesn't like 64-bit integer literals
#endif
#define INSTANCE_BASE_POINTER_ZERO_MASK           PackUint64(       0x7,        0x0) //                0x7ull
#define INSTANCE_BASE_POINTER_ADDRESS_USED_MASK   PackUint64(0xFFFFFFF8,     0x1FFF) //     0x1FFFFFFFFFF8ull
#define INSTANCE_BASE_POINTER_ADDRESS_UNUSED_MASK PackUint64(0x00000000,   0x3FE000) //   0x3FE00000000000ull
#define INSTANCE_BASE_POINTER_ADDRESS_MASK        PackUint64(0xFFFFFFF8,   0x3FFFFF) //   0x3FFFFFFFFFFFF8ull
#define INSTANCE_BASE_POINTER_FLAGS_MASK          PackUint64(0x00000000, 0xFFC00000) // 0xFFC0000000000000ull

#define NODE_POINTER_FLAGS_SHIFT                 54
#define NODE_POINTER_FORCE_OPAQUE_SHIFT          54
#define NODE_POINTER_FORCE_NON_OPAQUE_SHIFT      55
#define NODE_POINTER_DISABLE_TRIANGLE_CULL_SHIFT 56
#define NODE_POINTER_FLIP_FACEDNESS_SHIFT        57
#define NODE_POINTER_CULL_BACK_FACING_SHIFT      58
#define NODE_POINTER_CULL_FRONT_FACING_SHIFT     59
#define NODE_POINTER_CULL_OPAQUE_SHIFT           60
#define NODE_POINTER_CULL_NON_OPAQUE_SHIFT       61
#define NODE_POINTER_SKIP_TRIANGLES_SHIFT        62
#define NODE_POINTER_SKIP_PROCEDURAL_SHIFT       63

#define RAY_FLAG_VALID_MASK         0x3ffu
#define RAY_FLAG_EXCLUDE_MASK       (RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER)
#define RAY_FLAG_OVERRIDE_MASK      (RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_FORCE_NON_OPAQUE)   // 0x3
#define RAY_FLAG_PRESERVE_MASK      (RAY_FLAG_VALID_MASK & (~RAY_FLAG_OVERRIDE_MASK))     // 0x3fc

#define POINTER_FLAGS_HIDWORD_SHIFT (NODE_POINTER_FORCE_OPAQUE_SHIFT - 32)                  // 22
#define POINTER_FLAGS_VALID_MASK    (RAY_FLAG_VALID_MASK << POINTER_FLAGS_HIDWORD_SHIFT)    // 0x3ff << 22
#define POINTER_FLAGS_EXCLUDED_MASK  ~(POINTER_FLAGS_VALID_MASK)                            // 0xFFC00000

//=====================================================================================================================
struct StackPtrs
{
    uint stackPtrSrcNodeId;     // source node index in linear memory
    uint stackPtrNodeDest;      // node destination in linear memory. Counts in 64B chunks (FP16 box node size)
    uint numLeafsDone;
};

#define STACK_PTR_SIZE 12

#define STACK_PTRS_SRC_PTR_OFFSET        0
#define STACK_PTRS_DST_PTR_OFFSET        4
#define STACK_PTRS_NUM_LEAFS_DONE_OFFSET 8

#ifdef __cplusplus
static_assert(STACK_PTR_SIZE                   == sizeof(StackPtrs), "StackPtrs structure mismatch");
static_assert(STACK_PTRS_SRC_PTR_OFFSET        == offsetof(StackPtrs, stackPtrSrcNodeId), "");
static_assert(STACK_PTRS_DST_PTR_OFFSET        == offsetof(StackPtrs, stackPtrNodeDest),  "");
static_assert(STACK_PTRS_NUM_LEAFS_DONE_OFFSET == offsetof(StackPtrs, numLeafsDone),      "");
#endif

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
#define COUNTER_INITQBVH_OFFSET         0x18
#define COUNTER_BUILDQBVH_OFFSET        0x1C
#define COUNTER_EMPTYPRIM_OFFSET        0x20
#define COUNTER_EMITCOMPACTSIZE_OFFSET  0x24
#define COUNTER_BUILDFASTLBVH_OFFSET    0x28

//=====================================================================================================================
// Calculate internal BVH nodes
static uint CalcBvhNodeCountInternal(
    uint numPrimitives)
{
    // Note, the worst case QBVH node with a top-down build at the last level contains at least 2 BVH2 nodes.
    // And any preceding level contains at most 1/4 nodes each. Thus, the worst case QBVH internal
    // node calculation is as follows
    //
    // uint nodeCount = 0;
    // uint lastLevelNodeCount = numPrimitives >> 1;
    // while (lastLevelNodeCount > 0)
    // {
    //    nodeCount += lastLevelNodeCount;
    //    lastLevelNodeCount >>= 2;
    // }
    //
    // This results in a geometric progression:
    // where a is n/2 and ratio is 1/4
    //
    // nodeCount = n/2 + n/8 + n/32 + n/128 + ...
    // nodeCount = n/2 * (1 + 1/4 + 1/16 + 1/64 + ...)
    // nodeCount = n/2 * summation(n=0:+inf){ (1/4) ^ n }
    // nodeCount = n/2 * (1 / (1 - 1/4))
    // nodeCount = 2n / 3
    //
    // approximation error approaches ~8.3% at primitive count of 2^29
    //
    // We need at least 1 internal node.
    return max(1u, (2 * numPrimitives) / 3);
}

//=====================================================================================================================
// Calculate total number of BVH nodes
static uint CalcBvhNodeCount(
    uint numPrimitives)
{
    return numPrimitives + CalcBvhNodeCountInternal(numPrimitives);
}

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
static uint GetBvhNodeSizeInstance(in uint enableFusedInstanceNode)
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
static uint CalcBvhNodeSizeInternal(uint numPrimitives)
{
    return CalcBvhNodeCountInternal(numPrimitives) * GetBvhNodeSizeInternal();
}

// =====================================================================================================================
// Static helper function that calculates the size of the buffer for internal nodes
static uint CalcBvhNodeSizeInternal(
    uint numPrimitives,
    uint fp16NodesInQbvhBlasMode,
    bool isBottomLevel)
{
    const uint numInternalNodes = CalcBvhNodeCountInternal(numPrimitives);

    uint numBox16Nodes = 0;
    uint numBox32Nodes = 0;

    if (isBottomLevel)
    {
        switch (fp16NodesInQbvhBlasMode)
        {
            case NO_NODES_IN_BLAS_AS_FP16:
                // All nodes are fp32
                numBox32Nodes = numInternalNodes;
                break;

            case LEAF_NODES_IN_BLAS_AS_FP16:
                // Conservative estimate how many interior nodes can be converted to fp16
                numBox16Nodes = (numPrimitives / 4);
                numBox32Nodes = numInternalNodes - numBox16Nodes;
                break;

            case MIXED_NODES_IN_BLAS_AS_FP16:
                // Conservative estimate: no fp32 nodes could be converted to fp16
                // BVH storage savings realized after compaction copy
                numBox32Nodes = numInternalNodes;
                break;

            case ALL_INTERIOR_NODES_IN_BLAS_AS_FP16:
                // All but the root node are fp16
                numBox16Nodes = numInternalNodes - 1;
                numBox32Nodes = 1;
                break;

            default:
                // Should not get here -- will return 0
                break;
        }
    }
    else
    {
        numBox32Nodes = numInternalNodes;
    }

    const uint sizeInBytes = (FLOAT16_BOX_NODE_SIZE * numBox16Nodes) +
                             (FLOAT32_BOX_NODE_SIZE * numBox32Nodes);
    return sizeInBytes;
}

//=====================================================================================================================
static uint CalcParentPtrOffset(uint nodePtr)
{
    // Mask out node type from node pointer
    const uint alignedNodePtrOffset     = ClearNodeType(nodePtr);

    // Box nodes and user nodes are not compressed. Set prim offset to 0
    const uint nodePtrOffsetShift   = 3;

    // Subtract 1 from the index to account for negative offset calculations. I.e. index 0 is actually at -4 byte
    // offset from the end of the parent pointer memory
    const uint linkIndex = (alignedNodePtrOffset >> nodePtrOffsetShift) - 1;
    return linkIndex * NODE_PTR_SIZE;
}

//=====================================================================================================================
static uint CalcMetadataSizeInBytes(
    uint internalNodeSizeInBytes,
    uint leafNodeSizeInBytes)
{
    ///@note Each 64-bytes in acceleration structure occupies 4-Bytes of parent pointer memory
    ///      for each primitive in the leaf node. E.g. an acceleration structure with 4 primitives
    ///
    ///   | A | 0 | 1 | 2 | 3 | (A: internal node, 0-3 are leaf nodes)
    ///
    ///   Parent pointer memory layout (-1 indicates root node)
    ///
    /// -- 1x and Pair Triangle Compression
    ///   |-1 | x |
    ///   | A | A |
    ///   | A | A |
    ///

    const uint num64ByteChunks     = (internalNodeSizeInBytes + leafNodeSizeInBytes) / 64;
    const uint numLinks            = num64ByteChunks;
    const uint linkDataSizeInBytes = numLinks * NODE_PTR_SIZE;
    const uint metadataSizeInBytes = ACCEL_STRUCT_METADATA_HEADER_SIZE + linkDataSizeInBytes;

    return metadataSizeInBytes;
}

//=====================================================================================================================
static uint CreateRootNodePointer(
)
{
    {
        return PackNodePointer(NODE_TYPE_BOX_FLOAT32, ACCEL_STRUCT_HEADER_SIZE);
    }
}

//=====================================================================================================================
#define QBVH_COLLAPSE_STACK_ENTRY_SIZE 24

//=====================================================================================================================
struct QBVHTaskCollapse
{
    uint nodeIndex;
    uint leafIndex;
    uint numPrimitives;
    uint lastNodeIndex;
    uint parentOfCollapseNodeIndex;
    uint nodeDestIndex;
};

#ifdef __cplusplus
static_assert((QBVH_COLLAPSE_STACK_ENTRY_SIZE == sizeof(QBVHTaskCollapse)), "QBVHTaskCollapse structure mismatch");
#endif

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

#define STATE_TASK_QUEUE_PHASE_OFFSET               0
#define STATE_TASK_QUEUE_START_PHASE_INDEX_OFFSET   4
#define STATE_TASK_QUEUE_END_PHASE_INDEX_OFFSET     8
#define STATE_TASK_QUEUE_TASK_COUNTER_OFFSET        12
#define STATE_TASK_QUEUE_NUM_TASKS_DONE_OFFSET      16

//=====================================================================================================================
#define REF_SCRATCH_SIDE_LEFT       0
#define REF_SCRATCH_SIDE_RIGHT      1
#define REF_SCRATCH_SIDE_LEAF       2

#define USE_BLAS_PRIM_COUNT   0

struct TDRefScratch
{
    uint        primitiveIndex;
    uint        nodeIndex;
    float3      center;
    BoundingBox box;
    uint        side;
#if USE_BVH_REBRAID
    uint        nodePointer; //rebraid only
#endif
#if USE_BLAS_PRIM_COUNT
    uint        numPrimitives;
#endif
};

#define TD_REF_PRIM_INDEX_OFFSET    0
#define TD_REF_NODE_INDEX_OFFSET    4
#define TD_REF_CENTER_OFFSET        8
#define TD_REF_BOX_OFFSET           20
#define TD_REF_SIDE_OFFSET          (TD_REF_BOX_OFFSET + sizeof(BoundingBox))
#define TD_REF_NODE_POINTER_OFFSET  (TD_REF_SIDE_OFFSET + 4)
#if USE_BLAS_PRIM_COUNT
#define TD_REF_NUM_PRIM_OFFSET      (TD_REF_NODE_POINTER_OFFSET + sizeof(uint))
#endif

//=====================================================================================================================
#define NUM_SPLIT_BINS        4

#define TD_NODE_REBRAID_STATE_OPEN   0
#define TD_NODE_REBRAID_STATE_CLOSED 1

struct TDBins
{
    uint64_t        firstRefIndex;

    UintBoundingBox binBoxes[3][NUM_SPLIT_BINS];
    uint            binPrimCount[3][NUM_SPLIT_BINS];

    uint            bestAxis;
    uint            bestSplit;
    uint            numLeft;
    uint            numRight;

#if USE_BLAS_PRIM_COUNT
    uint            binBLASPrimCount[3][NUM_SPLIT_BINS];
#endif
};

#define TD_BINS_FIRST_REF_INDEX_OFFSET        0
#define TD_BINS_BIN_BOXES_OFFSET              (TD_BINS_FIRST_REF_INDEX_OFFSET + 8)
#define TD_BINS_BIN_PRIM_COUNT_OFFSET         (TD_BINS_BIN_BOXES_OFFSET + sizeof(UintBoundingBox) * NUM_SPLIT_BINS * 3)
#define TD_BINS_BEST_AXIS_OFFSET              (TD_BINS_BIN_PRIM_COUNT_OFFSET + sizeof(uint) * NUM_SPLIT_BINS * 3)
#define TD_BINS_BEST_SPLIT_OFFSET             (TD_BINS_BEST_AXIS_OFFSET + 4)
#define TD_BINS_NUM_LEFT_OFFSET               (TD_BINS_BEST_SPLIT_OFFSET + 4)
#define TD_BINS_NUM_RIGHT_OFFSET              (TD_BINS_NUM_LEFT_OFFSET + 4)
#if USE_BLAS_PRIM_COUNT
#define TD_BINS_BLAS_PRIM_COUNT_OFFSET        (TD_BINS_NUM_RIGHT_OFFSET + 4)
#endif

struct TDNode
{
    UintBoundingBox centroidBox;
    uint            binsIndex;
    uint            childCount;

#if USE_BVH_REBRAID
    uint            largestAxis;    // rebraid only
    float           largestWidth;   // rebraid only
    uint            rebraidState;   // rebraid only
    uint            primIndex;      // rebraid only
#endif
};

#define TD_NODE_CENTROID_BOX_OFFSET           0
#define TD_NODE_BINS_INDEX_OFFSET             (TD_NODE_CENTROID_BOX_OFFSET + sizeof(UintBoundingBox))
#define TD_NODE_CHILD_COUNT_OFFSET            (TD_NODE_BINS_INDEX_OFFSET + 4)
#define TD_NODE_LARGEST_AXIS_OFFSET           (TD_NODE_CHILD_COUNT_OFFSET + 4)
#define TD_NODE_LARGEST_WIDTH_OFFSET          (TD_NODE_LARGEST_AXIS_OFFSET + 4)
#define TD_NODE_REBRAID_STATE_OFFSET          (TD_NODE_LARGEST_WIDTH_OFFSET + 4)
#define TD_NODE_PRIM_INDEX_OFFSET             (TD_NODE_REBRAID_STATE_OFFSET + 4)

//=====================================================================================================================

#define TD_REBRAID_STATE_NO_OPEN    0
#define TD_REBRAID_STATE_NEED_OPEN  1
#define TD_REBRAID_STATE_OOM        2

#define TD_PHASE_INIT_STATE                 0
#define TD_PHASE_INIT_REFS_TO_LEAVES        1
#define TD_PHASE_CHECK_NEED_ALLOC           2
#define TD_PHASE_ALLOC_ROOT_NODE            3
#define TD_PHASE_REBRAID_COUNT_OPENINGS     4
#define TD_PHASE_REBRAID_CHECK_TERMINATION  5
#define TD_PHASE_REBRAID_OPEN               6
#define TD_PHASE_REBRAID_UPDATE_NODES       7
#define TD_PHASE_BIN_REFS                   8
#define TD_PHASE_FIND_BEST_SPLIT            9
#define TD_PHASE_SECOND_PASS                10
#define TD_PHASE_UPDATE_NEW_NODES           11
#define TD_PHASE_DONE                       12

struct StateTDBuild
{
    uint            numNodes;
    uint            numProcessedNodes;
    uint            numNodesAllocated;
    uint            numRefs;
    uint            numRefsAllocated;
    uint            numInactiveInstance;
    UintBoundingBox rootCentroidBBox;
    uint            numLeaves;
    uint            binsCounter;

#if USE_BVH_REBRAID
    uint            rebraidState;
    uint            leafAllocOffset;
#endif
};

#define STATE_TD_NUM_NODES_OFFSET               0
#define STATE_TD_NUM_PROCESSED_NODES_OFFSET     4
#define STATE_TD_NUM_NODES_ALLOCATED_OFFSET     8
#define STATE_TD_NUM_REFS_OFFSET                12
#define STATE_TD_NUM_REFS_ALLOCATED_OFFSET      16
#define STATE_TD_NUM_INACTIVE_INSTANCE_OFFSET   20
#define STATE_TD_CENTROID_BBOX_OFFSET           24
#define STATE_TD_NUM_LEAVES_OFFSET              (STATE_TD_CENTROID_BBOX_OFFSET + sizeof(UintBoundingBox))
#define STATE_TD_BINS_COUNTER_OFFSET            (STATE_TD_NUM_LEAVES_OFFSET + 4)
#define STATE_TD_REBRAID_STATE_OFFSET           (STATE_TD_BINS_COUNTER_OFFSET + 4)
#define STATE_TD_LEAF_ALLOC_OFFSET_OFFSET       (STATE_TD_REBRAID_STATE_OFFSET + 4)

//=====================================================================================================================
struct Flags
{
    uint dataValid;
    uint prefixSum;
};

#define FLAGS_DATA_VALID_OFFSET          0
#define FLAGS_PREFIX_SUM_OFFSET          4

//=====================================================================================================================

#define PLOC_PHASE_INIT                     0
#define PLOC_PHASE_FIND_NEAREST_NEIGHBOUR   1
#define PLOC_PHASE_UPDATE_CLUSTER_COUNT     2
#define PLOC_PHASE_DONE                     3

struct StatePLOC
{
    StateTaskQueueCounter   plocTaskCounters;

    uint                    numClusters;
    uint                    internalNodesIndex;
    uint                    clusterListIndex;
    uint                    numClustersAlloc;
};

#define STATE_PLOC_TASK_QUEUE_SIZE                          sizeof(StateTaskQueueCounter)
#define STATE_PLOC_NUM_CLUSTERS_OFFSET                      STATE_PLOC_TASK_QUEUE_SIZE + 0
#define STATE_PLOC_INTERNAL_NODES_INDEX_OFFSET              STATE_PLOC_TASK_QUEUE_SIZE + 4
#define STATE_PLOC_CLUSTER_LIST_INDEX_OFFSET                STATE_PLOC_TASK_QUEUE_SIZE + 8
#define STATE_PLOC_NUM_CLUSTERS_ALLOC_OFFSET                STATE_PLOC_TASK_QUEUE_SIZE + 12

//=====================================================================================================================
#define REBRAID_PHASE_INIT                     0
#define REBRAID_PHASE_CALC_SUM                 1
#define REBRAID_PHASE_OPEN                     2
#define REBRAID_PHASE_DONE                     3

struct RebraidState
{
    StateTaskQueueCounter   taskQueue;
    float                   sumValue;
    uint                    mutex;
};

#define STATE_REBRAID_TASK_QUEUE_SIZE                                 sizeof(StateTaskQueueCounter)
#define STATE_REBRAID_SUM_VALUE_OFFSET                                STATE_REBRAID_TASK_QUEUE_SIZE + 0
#define STATE_REBRAID_MUTEX_OFFSET                                    STATE_REBRAID_SUM_VALUE_OFFSET + 4

//=====================================================================================================================
#define TS_PHASE_INIT                 0
#define TS_PHASE_CALC_SUM             1
#define TS_PHASE_ALLOC_REFS           2
#define TS_PHASE_SPLIT                3
#define TS_PHASE_DONE                 4

struct ScratchTSRef
{
    uint leafIndex;
    uint numSplits;

    uint splitLeafBaseIndex;

    BoundingBox bbox;
};

struct ScratchTSState
{
    StateTaskQueueCounter   taskQueue;
    uint                    refListIndex;
    uint                    numRefs;
    uint                    numRefsAlloc;
    float                   sum;
    uint                    mutex;
};

#define STATE_TS_TASK_QUEUE_SIZE             sizeof(StateTaskQueueCounter)
#define STATE_TS_REF_LIST_INDEX_OFFSET       STATE_TS_TASK_QUEUE_SIZE + 0
#define STATE_TS_NUM_REFS_OFFSET             STATE_TS_REF_LIST_INDEX_OFFSET + 4
#define STATE_TS_NUM_REFS_ALLOC_OFFSET       STATE_TS_NUM_REFS_OFFSET + 4
#define STATE_TS_SUM_OFFSET                  STATE_TS_NUM_REFS_ALLOC_OFFSET + 4
#define STATE_TS_MUTEX_OFFSET                STATE_TS_SUM_OFFSET + 4

//=====================================================================================================================
struct IndexBufferInfo
{
    uint gpuVaLo;
    uint gpuVaHi;
    uint byteOffset;
    uint format;
};

#define INDEX_BUFFER_INFO_GPU_VA_LO_OFFSET    0
#define INDEX_BUFFER_INFO_GPU_VA_HI_OFFSET    4
#define INDEX_BUFFER_INFO_BYTE_OFFSET_OFFSET  8
#define INDEX_BUFFER_INFO_FORMAT_OFFSET      12

//=====================================================================================================================
// Update scratch memory fields
#define UPDATE_SCRATCH_STACK_NUM_ENTRIES_OFFSET 0
#define UPDATE_SCRATCH_TASK_COUNT_OFFSET        4

//=====================================================================================================================
// Layout sans header
struct RayTracingDstBufferLayout
{
    DataOffsetAndSize metadata;          // Offset and size of internal metadata
    DataOffsetAndSize internalNodes;     // Offset and size of internal box nodes
    DataOffsetAndSize leafNodes;         // Offset and size of leaf nodes
    DataOffsetAndSize geometryInfo;      // Offset and size of geometry desc info (bottom level only)
    DataOffsetAndSize sortedPrimIndices; // Offset and size of sorted primitive indices (ALLOW_UPDATE only)
    DataOffsetAndSize activePrimCount;   // Offset and size of active primitive count (ALLOW_UPDATE only)
    DataOffsetAndSize scratchNodes;      // Offset and size of scratch nodes (ALLOW_UPDATE only)
    DataOffsetAndSize primNodePtrs;      // Offset and size of prim node pointers (BVH4 with triangle compression and ALLOW_UPDATE only)
    uint              sizeInBytes;       // Total size in bytes
};

//=====================================================================================================================
struct RayTracingUpdateBufferLayout
{
    DataOffsetAndSize scratchNodes;      // Offset and size of internal scratch nodes
    DataOffsetAndSize propagationFlags;  // Offset and size of propagation flags buffer
    DataOffsetAndSize bvh4Stack;         // Offset and size of stack for BVH4 update
    DataOffsetAndSize bvh4StackPtrs;     // Offset and size of stack pointers for BVH4 update
    uint              sizeInBytes;       // Total size in bytes
};

//=====================================================================================================================
#ifdef AMD_VULKAN
//=====================================================================================================================
///@note Enum is a reserved keyword in glslang. To workaround this limitation, define static constants to replace the
///      HLSL enums that follow for compatibility.
//=====================================================================================================================
namespace RebraidType
{
    static const uint Off = 0x0;
    static const uint V1  = 0x1;
    static const uint V2  = 0x2;
}
#else
enum RebraidType : uint
{
    Off = 0, // No Rebraid
    V1  = 1, // First version of Rebraid
    V2  = 2, // Second version of Rebraid
};
#endif
//=====================================================================================================================
// Settings constant buffer specified at compile time in the driver
struct BuildSettingsData
{
    uint topLevelBuild;
    uint buildMode;
    uint triangleCompressionMode;
    uint doTriangleSplitting;
    uint doCollapse;
    uint fp16BoxNodesMode;
    float fp16BoxModeMixedSaThreshhold;
    uint radixSortScanLevel;
    uint emitCompactSize;
    uint enableBVHBuildDebugCounters;
    uint plocRadius;
    uint enablePairCostCheck;
    uint enableVariableBitsMortonCode;
    uint rebraidType;
    uint enableTopDownBuild;
    uint useMortonCode30;
    uint enableMergeSort;
    uint fastBuildThreshold;
    uint bvhBuilderNodeSortType;
    uint bvhBuilderNodeSortHeuristic;
    uint enableFusedInstanceNode;
    uint sahQbvh;
    float tsPriority;
    uint noCopySortedNodes;
    uint enableSAHCost;
    uint doEncode;
    uint enableEarlyPairCompression;
    uint enableFastLBVH;
};

#define BUILD_SETTINGS_DATA_TOP_LEVEL_BUILD_OFFSET                        0
#define BUILD_SETTINGS_DATA_BUILD_MODE_OFFSET                             4
#define BUILD_SETTINGS_DATA_TRIANGLE_COMPRESSION_MODE_OFFSET              8
#define BUILD_SETTINGS_DATA_DO_TRIANGLE_SPLITTING_OFFSET                  12
#define BUILD_SETTINGS_DATA_DO_COLLAPSE_OFFSET                            16
#define BUILD_SETTINGS_DATA_FP16_BOX_NODES_MODE_OFFSET                    20
#define BUILD_SETTINGS_DATA_FP16_BOX_MODE_MIXED_SA_THRESHHOLD_OFFSET      24
#define BUILD_SETTINGS_DATA_RADIX_SORT_SCAN_LEVEL_OFFSET                  28
#define BUILD_SETTINGS_DATA_EMIT_COMPACT_SIZE_OFFSET                      32
#define BUILD_SETTINGS_DATA_ENABLE_BVH_BUILD_DEBUG_COUNTERS_OFFSET        36
#define BUILD_SETTINGS_DATA_PLOC_RADIUS                                   40
#define BUILD_SETTINGS_DATA_ENABLE_PAIR_COST_CHECK_OFFSET                 44
#define BUILD_SETTINGS_DATA_ENABLE_VARIABLE_BITS_MC_OFFSET                48
#define BUILD_SETTINGS_DATA_REBRAID_TYPE_OFFSET                           52
#define BUILD_SETTINGS_DATA_ENABLE_TOP_DOWN_BUILD_OFFSET                  56
#define BUILD_SETTINGS_DATA_USE_MORTON_CODE_30_OFFSET                     60
#define BUILD_SETTINGS_DATA_ENABLE_MERGE_SORT_OFFSET                      64
#define BUILD_SETTINGS_DATA_FAST_BUILD_THRESHOLD_OFFSET                   68
#define BUILD_SETTINGS_DATA_BVH_BUILDER_NODE_SORT_TYPE_OFFSET             72
#define BUILD_SETTINGS_DATA_BVH_BUILDER_NODE_SORT_HEURISTIC_OFFSET        76
#define BUILD_SETTINGS_DATA_ENABLE_FUSED_INSTANCE_NODE_OFFSET             80
#define BUILD_SETTINGS_DATA_SAH_QBVH_OFFSET                               84
#define BUILD_SETTINGS_DATA_TS_PRIORITY_OFFSET                            88
#define BUILD_SETTINGS_DATA_NO_COPY_SORTED_NODES_OFFSET                   92
#define BUILD_SETTINGS_DATA_ENABLE_SAH_COST_OFFSET                        96
#define BUILD_SETTINGS_DATA_USE_GROWTH_IN_LTD_OFFSET                      100
#define BUILD_SETTINGS_DATA_DO_ENCODE                                     104
#define BUILD_SETTINGS_DATA_LTD_PACK_CENTROIDS_OFFSET                     108
#define BUILD_SETTINGS_DATA_ENABLE_EARLY_PAIR_COMPRESSION_OFFSET          112
#define BUILD_SETTINGS_DATA_ENABLE_FAST_LBVH_OFFSET                       116
#define BUILD_SETTINGS_DATA_SIZE                                          120

#define BUILD_SETTINGS_DATA_TOP_LEVEL_BUILD_ID                        (BUILD_SETTINGS_DATA_TOP_LEVEL_BUILD_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_BUILD_MODE_ID                             (BUILD_SETTINGS_DATA_BUILD_MODE_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_TRIANGLE_COMPRESSION_MODE_ID              (BUILD_SETTINGS_DATA_TRIANGLE_COMPRESSION_MODE_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_DO_TRIANGLE_SPLITTING_ID                  (BUILD_SETTINGS_DATA_DO_TRIANGLE_SPLITTING_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_DO_COLLAPSE_ID                            (BUILD_SETTINGS_DATA_DO_COLLAPSE_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_FP16_BOX_NODES_MODE_ID                    (BUILD_SETTINGS_DATA_FP16_BOX_NODES_MODE_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_FP16_BOX_MODE_MIXED_SA_THRESHHOLD_ID      (BUILD_SETTINGS_DATA_FP16_BOX_MODE_MIXED_SA_THRESHHOLD_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_RADIX_SORT_SCAN_LEVEL_ID                  (BUILD_SETTINGS_DATA_RADIX_SORT_SCAN_LEVEL_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_EMIT_COMPACT_SIZE_ID                      (BUILD_SETTINGS_DATA_EMIT_COMPACT_SIZE_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_BVH_BUILD_DEBUG_COUNTERS_ID        (BUILD_SETTINGS_DATA_ENABLE_BVH_BUILD_DEBUG_COUNTERS_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_PLOC_RADIUS_ID                            (BUILD_SETTINGS_DATA_PLOC_RADIUS  / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_PAIR_COST_CHECK_ID                 (BUILD_SETTINGS_DATA_ENABLE_PAIR_COST_CHECK_OFFSET  / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_VARIABLE_BITS_MC_ID                (BUILD_SETTINGS_DATA_ENABLE_VARIABLE_BITS_MC_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_REBRAID_TYPE_ID                           (BUILD_SETTINGS_DATA_REBRAID_TYPE_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_TOP_DOWN_BUILD_ID                  (BUILD_SETTINGS_DATA_ENABLE_TOP_DOWN_BUILD_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_USE_MORTON_CODE_30_ID                     (BUILD_SETTINGS_DATA_USE_MORTON_CODE_30_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_MERGE_SORT_ID                      (BUILD_SETTINGS_DATA_ENABLE_MERGE_SORT_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_FAST_BUILD_THRESHOLD_ID                   (BUILD_SETTINGS_DATA_FAST_BUILD_THRESHOLD_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_BVH_BUILDER_NODE_SORT_TYPE_ID             (BUILD_SETTINGS_DATA_BVH_BUILDER_NODE_SORT_TYPE_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_BVH_BUILDER_NODE_SORT_HEURISTIC_ID        (BUILD_SETTINGS_DATA_BVH_BUILDER_NODE_SORT_HEURISTIC_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_FUSED_INSTANCE_NODE_ID             (BUILD_SETTINGS_DATA_ENABLE_FUSED_INSTANCE_NODE_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_SAH_QBVH_ID                               (BUILD_SETTINGS_DATA_SAH_QBVH_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_TS_PRIORITY_ID                            (BUILD_SETTINGS_DATA_TS_PRIORITY_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_NO_COPY_SORTED_NODES_ID                   (BUILD_SETTINGS_DATA_NO_COPY_SORTED_NODES_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_SAH_COST_ID                        (BUILD_SETTINGS_DATA_ENABLE_SAH_COST_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_DO_ENCODE_ID                              (BUILD_SETTINGS_DATA_DO_ENCODE / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_EARLY_PAIR_COMPRESSION_ID          (BUILD_SETTINGS_DATA_ENABLE_EARLY_PAIR_COMPRESSION_OFFSET / sizeof(uint))
#define BUILD_SETTINGS_DATA_ENABLE_FAST_LBVH_ID                       (BUILD_SETTINGS_DATA_ENABLE_FAST_LBVH_OFFSET / sizeof(uint))

#ifdef __cplusplus
static_assert(BUILD_SETTINGS_DATA_SIZE                                          == sizeof(BuildSettingsData), "BuildSettingsData structure header mismatch");
static_assert(BUILD_SETTINGS_DATA_TOP_LEVEL_BUILD_OFFSET                        == offsetof(BuildSettingsData, topLevelBuild), "");
static_assert(BUILD_SETTINGS_DATA_BUILD_MODE_OFFSET                             == offsetof(BuildSettingsData, buildMode), "");
static_assert(BUILD_SETTINGS_DATA_TRIANGLE_COMPRESSION_MODE_OFFSET              == offsetof(BuildSettingsData, triangleCompressionMode), "");
static_assert(BUILD_SETTINGS_DATA_DO_TRIANGLE_SPLITTING_OFFSET                  == offsetof(BuildSettingsData, doTriangleSplitting), "");
static_assert(BUILD_SETTINGS_DATA_DO_COLLAPSE_OFFSET                            == offsetof(BuildSettingsData, doCollapse), "");
static_assert(BUILD_SETTINGS_DATA_FP16_BOX_NODES_MODE_OFFSET                    == offsetof(BuildSettingsData, fp16BoxNodesMode), "");
static_assert(BUILD_SETTINGS_DATA_FP16_BOX_MODE_MIXED_SA_THRESHHOLD_OFFSET      == offsetof(BuildSettingsData, fp16BoxModeMixedSaThreshhold), "");
static_assert(BUILD_SETTINGS_DATA_RADIX_SORT_SCAN_LEVEL_OFFSET                  == offsetof(BuildSettingsData, radixSortScanLevel), "");
static_assert(BUILD_SETTINGS_DATA_EMIT_COMPACT_SIZE_OFFSET                      == offsetof(BuildSettingsData, emitCompactSize), "");
static_assert(BUILD_SETTINGS_DATA_PLOC_RADIUS                                   == offsetof(BuildSettingsData, plocRadius), "");
static_assert(BUILD_SETTINGS_DATA_ENABLE_PAIR_COST_CHECK_OFFSET                 == offsetof(BuildSettingsData, enablePairCostCheck), "");
static_assert(BUILD_SETTINGS_DATA_ENABLE_BVH_BUILD_DEBUG_COUNTERS_OFFSET        == offsetof(BuildSettingsData, enableBVHBuildDebugCounters), "");
static_assert(BUILD_SETTINGS_DATA_ENABLE_VARIABLE_BITS_MC_OFFSET                == offsetof(BuildSettingsData, enableVariableBitsMortonCode), "");
static_assert(BUILD_SETTINGS_DATA_USE_MORTON_CODE_30_OFFSET                     == offsetof(BuildSettingsData, useMortonCode30), "");
static_assert(BUILD_SETTINGS_DATA_ENABLE_MERGE_SORT_OFFSET                      == offsetof(BuildSettingsData, enableMergeSort), "");
static_assert(BUILD_SETTINGS_DATA_FAST_BUILD_THRESHOLD_OFFSET                   == offsetof(BuildSettingsData, fastBuildThreshold), "");
static_assert(BUILD_SETTINGS_DATA_BVH_BUILDER_NODE_SORT_TYPE_OFFSET             == offsetof(BuildSettingsData, bvhBuilderNodeSortType), "");
static_assert(BUILD_SETTINGS_DATA_BVH_BUILDER_NODE_SORT_HEURISTIC_OFFSET        == offsetof(BuildSettingsData, bvhBuilderNodeSortHeuristic), "");
static_assert(BUILD_SETTINGS_DATA_ENABLE_FUSED_INSTANCE_NODE_OFFSET             == offsetof(BuildSettingsData, enableFusedInstanceNode), "");
static_assert(BUILD_SETTINGS_DATA_SAH_QBVH_OFFSET                               == offsetof(BuildSettingsData, sahQbvh), "");
static_assert(BUILD_SETTINGS_DATA_TS_PRIORITY_OFFSET                            == offsetof(BuildSettingsData, tsPriority), "");
static_assert(BUILD_SETTINGS_DATA_NO_COPY_SORTED_NODES_OFFSET                   == offsetof(BuildSettingsData, noCopySortedNodes), "");
static_assert(BUILD_SETTINGS_DATA_ENABLE_SAH_COST_OFFSET                        == offsetof(BuildSettingsData, enableSAHCost), "");
static_assert(BUILD_SETTINGS_DATA_DO_ENCODE                                     == offsetof(BuildSettingsData, doEncode), "");
static_assert(BUILD_SETTINGS_DATA_ENABLE_EARLY_PAIR_COMPRESSION_OFFSET          == offsetof(BuildSettingsData, enableEarlyPairCompression), "");
static_assert(BUILD_SETTINGS_DATA_ENABLE_FAST_LBVH_OFFSET                       == offsetof(BuildSettingsData, enableFastLBVH), "");
#endif

#define BUILD_MODE_LINEAR   0
// BUILD_MODE_AC was 1, but it has been removed.
#define BUILD_MODE_PLOC     2

//=====================================================================================================================
static uint CalcNumQBVHInternalNodes(uint numPrimitives)
{
    return CalcBvhNodeCountInternal(numPrimitives);
}

//=====================================================================================================================
static uint CalcNumQBVHNodes(uint numPrimitives)
{
    return CalcBvhNodeCount(numPrimitives);
}

#define BUILD_FLAGS_COLLAPSE                 1
#define BUILD_FLAGS_TRIANGLE_SPLITTING       2
#define BUILD_FLAGS_PAIR_COMPRESSION         4
#define BUILD_FLAGS_PAIR_COST_CHECK          8

#define SAH_COST_TRIANGLE_INTERSECTION       1.5
#define SAH_COST_AABBB_INTERSECTION          1

#define ENCODE_FLAG_ARRAY_OF_POINTERS          0x00000001
#define ENCODE_FLAG_UPDATE_IN_PLACE            0x00000002
#define ENCODE_FLAG_REBRAID_ENABLED            0x00000004
#define ENCODE_FLAG_ENABLE_FUSED_INSTANCE_NODE 0x00000008

//=====================================================================================================================
struct IntersectionResult
{
#if defined(__cplusplus)
    IntersectionResult(int val)
    {
        memset(this, val, sizeof(IntersectionResult));
    }
#endif
    float  t;                     // Relative to tMin
    uint   nodeIndex;
    float2 barycentrics;
    uint   geometryIndex;
    uint   primitiveIndex;
    uint   instNodePtr;
    uint   hitkind;
    uint   instanceContribution;

#if DEVELOPER
    uint   numIterations;
    uint   maxStackDepth;
    uint   numRayBoxTest;
    uint   numCandidateHits;
    uint   numRayTriangleTest;
    uint   numAnyHitInvocation;
    uint   instanceIntersections;
#endif
};

//=====================================================================================================================
// Commit status
typedef uint COMMITTED_STATUS;

#define COMMITTED_NOTHING 0
#define COMMITTED_TRIANGLE_HIT 1
#define COMMITTED_PROCEDURAL_PRIMITIVE_HIT 2

//=====================================================================================================================
// Candidate type
typedef uint CANDIDATE_STATUS;

#define CANDIDATE_NON_OPAQUE_TRIANGLE 0
#define CANDIDATE_PROCEDURAL_PRIMITIVE 1
#define CANDIDATE_NON_OPAQUE_PROCEDURAL_PRIMITIVE 2
#define CANDIDATE_NO_DUPLICATE_ANYHIT_PROCEDURAL_PRIMITIVE 3
#define CANDIDATE_EARLY_RAY_TERMINATE 4

#define INIT_LDS_STATE 0xFFFFFFFF

//=====================================================================================================================
// Data required for system value intrinsics
struct RaySystemData
{
    uint   currNodePtr;
    float  rayTCurrent;
    uint   instNodePtr;
    uint   instanceContribution;
    uint   geometryIndex;
    uint   primitiveIndex;
    float2 barycentrics;
    uint   frontFace;
    float3 origin;
    float3 direction;
};

//=====================================================================================================================
#if DEFINE_RAYDESC
// Ray description matching the D3D12 HLSL header
struct RayDesc
{
    float3 Origin;
    float TMin;
    float3 Direction;
    float TMax;
};
#endif

//=====================================================================================================================
// Internal RayQuery structure initialised at TraceRaysInline()
struct RayQueryInternal
{
#if __cplusplus
    RayQueryInternal(int val) {
        memset(this, val, sizeof(RayQueryInternal));
    }
#endif

    // Internal query data holding address of current BVH and stack information.
    // Additional data that may be required will be stored here.
    uint             bvhLo;
    uint             bvhHi;
    uint             topLevelBvhLo;
    uint             topLevelBvhHi;
    uint             stackPtr;
    uint             stackPtrTop;
    uint             stackNumEntries;
    uint             instNodePtr;
    uint             currNodePtr;
    uint             instanceHitContributionAndFlags;
    uint             prevNodePtr;
    uint             isGoingDown;
    uint             lastInstanceNode;

    RayDesc          rayDesc;
    float            rayTMin;
    uint             rayFlags;
    uint             instanceInclusionMask;

    // Candidate system data
    CANDIDATE_STATUS candidateType;
    RaySystemData    candidate;

    // Committed system data
    COMMITTED_STATUS committedStatus;
    RaySystemData    committed;

    uint             reserved;

    // Counter data
    // @note We don't wrap these in DEVELOPER because it would result in mismatch of RayQuery struct size
    //       on the driver side when we're not using counters.
    uint             numRayBoxTest;
    uint             numRayTriangleTest;
    uint             numIterations;
    uint             maxStackDepth;
    uint             clocks;
    uint             numCandidateHits;
    uint             instanceIntersections;
#ifdef AMD_VULKAN
    uint             rayQueryObjId;
#endif
};

//=====================================================================================================================
struct DispatchRaysInfoData
{
    uint RayGenerationTableLow;                         ///< d1.x
    uint RayGenerationTableHigh;                        ///< d1.y
    uint RayGridWidth;                                  ///< d1.z
    uint RayGridHeight;                                 ///< d1.w
    uint RayGridDepth;                                  ///< d2.x
    uint MissShaderTableStartAddressLow;                ///< d2.y
    uint MissShaderTableStartAddressHigh;               ///< d2.z
    uint MissShaderTableStrideInBytes;                  ///< d2.w
    uint MaxRecursion;                                  ///< d3.x
    uint HitGroupTableStartAddressLow;                  ///< d3.y
    uint HitGroupTableStartAddressHigh;                 ///< d3.z
    uint HitGroupTableStrideInBytes;                    ///< d3.w
    uint MaxAttributeSize;                              ///< d4.x
    uint CallableShaderStartAddressLow;                 ///< d4.y
    uint CallableShaderStartAddressHigh;                ///< d4.z
    uint CallableShaderStrideInBytes;                   ///< d4.w

    // Profiling only
    uint ProfileRayFlags;                               ///< d5.x
    uint MaxIterations;                                 ///< d5.y

    // Driver reserved for indirect function support
    uint TraceRaysGpuVaLo;                              ///< d5.z
    uint TraceRaysGpuVaHi;                              ///< d5.w

    // Counters only
    uint CounterMode;                                   ///< d6.x
    uint RayIdRangeBegin;                               ///< d6.y
    uint RayIdRangeEnd;                                 ///< d6.z
    uint cpsStackOffsetInBytes;                         ///< d6.w
};

//=====================================================================================================================
struct DispatchRaysDynamicInfoData
{
    // The following order matches driver Pal::Rect { origin, extents }
    uint2 TileOrigin;
    uint2 TileDimension; //could be smaller than tile size
    uint  TileDepth;

    uint  CurrentRecursionCount;
    uint  CurrentShaderIndex; // Unused in driver implementation. Driver generates these as compile time constants
};

//=====================================================================================================================
struct HitGroupInfo
{
    uint2 closestHitId;
    uint2 anyHitId;
    uint2 intersectionId;
    uint  tableIndex;
};
#endif
