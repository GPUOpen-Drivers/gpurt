/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _GPURT_ACCEL_STRUCT_H
#define _GPURT_ACCEL_STRUCT_H

// This file provides the definition of the GPURT acceleration structure format produced by the GPURT BVH builder.
// The format is consumed by GPURT's traversal shader code as well as a variety of tools. Format changes must strive
// to be binary compatible in order to keep those tools functioning on old and new acceleration structure data.
//
// Source level compatibility is desirable but not strictly required as long as the DXCP and Vulkan drivers continue
// to build properly.
//
// Major version changes: The major version below must be incremented in the event that an incompatible change is
// required. An incompatible change means that consumers of the data are required to make changes in order to interpret
// the data structures correctly.
//
// Minor version changes: The minor version below must be incremented when fields are added or modified in any
// structures. Fields may be added to the end of structures, and reserved fields may be changed to meaningful values.
// Reserved fields are zeroed by GPURT until they have a purpose. If a structure is smaller than the size of the most
// recent version, tools may assume the non-present fields are zero.
//
//
// Note this file is designed to be compilable as HLSL.

#define GPURT_ACCEL_STRUCT_MAJOR_VERSION 16
#define GPURT_ACCEL_STRUCT_MINOR_VERSION 1
#define GPURT_ACCEL_STRUCT_VERSION       ((GPURT_ACCEL_STRUCT_MAJOR_VERSION << 16) | GPURT_ACCEL_STRUCT_MINOR_VERSION)

#ifdef __cplusplus
namespace GpuRt
{

#define GPURT_STATIC_ASSERT(condition, message) static_assert(condition, message)
#else
#define GPURT_STATIC_ASSERT(condition, message)
#endif

typedef uint32_t uint32;
typedef uint64_t uint64;

// =====================================================================================================================
// Acceleration structure result data offsets
struct AccelStructDataOffsets
{
    uint32 internalNodes;       // Offset to internal box nodes
    uint32 leafNodes;           // Offset to leaf nodes
    uint32 geometryInfo;        // Offset to geometry desc info (bottom level acceleration structure)
    uint32 primNodePtrs;        // Offset to prim node pointers (BVH4 with triangle compression or ALLOW_UPDATE only)
};

#define ACCEL_STRUCT_OFFSETS_INTERNAL_NODES_OFFSET        0
#define ACCEL_STRUCT_OFFSETS_LEAF_NODES_OFFSET            4
#define ACCEL_STRUCT_OFFSETS_GEOMETRY_INFO_OFFSET         8
#define ACCEL_STRUCT_OFFSETS_PRIM_NODE_PTRS_OFFSET       12
#define ACCEL_STRUCT_OFFSETS_SIZE                        16

GPURT_STATIC_ASSERT(ACCEL_STRUCT_OFFSETS_INTERNAL_NODES_OFFSET == offsetof(AccelStructDataOffsets, internalNodes), "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_OFFSETS_LEAF_NODES_OFFSET == offsetof(AccelStructDataOffsets, leafNodes), "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_OFFSETS_GEOMETRY_INFO_OFFSET == offsetof(AccelStructDataOffsets, geometryInfo), "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_OFFSETS_PRIM_NODE_PTRS_OFFSET == offsetof(AccelStructDataOffsets, primNodePtrs), "");
GPURT_STATIC_ASSERT(sizeof(AccelStructDataOffsets) == 16,
    "AccelStructDataOffsets size cannot change because it is embedded in AccelStructHeader.");

// =====================================================================================================================
// Header for acceleration structure metadata
struct AccelStructMetadataHeader
{
    uint32 addressLo;           // Address of acceleration structure data section (low bits)
    uint32 addressHi;           // Address of acceleration structure data section (high bits)
    uint32 sizeInBytes;         // Metadata size in bytes (including this header)
    uint32 taskCounter;         // Task counter for dispatch-wide spin loop sync. Align to 8 bytes so taskCounter and
                                // numTasksDone can be reset in one 64 bit CP write.
    uint32 numTasksDone;        // Number of tasks done
    uint32 reserved0[16];       // Reserved
};

#define ACCEL_STRUCT_METADATA_VA_LO_OFFSET              0
#define ACCEL_STRUCT_METADATA_VA_HI_OFFSET              4
#define ACCEL_STRUCT_METADATA_SIZE_OFFSET               8
#define ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET       12
#define ACCEL_STRUCT_METADATA_NUM_TASKS_DONE_OFFSET     16
#define ACCEL_STRUCT_METADATA_RESERVED_0                20
#define ACCEL_STRUCT_METADATA_HEADER_SIZE               84

GPURT_STATIC_ASSERT(ACCEL_STRUCT_METADATA_HEADER_SIZE == sizeof(AccelStructMetadataHeader), "Acceleration structure header mismatch");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_METADATA_VA_LO_OFFSET == offsetof(AccelStructMetadataHeader, addressLo), "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_METADATA_VA_HI_OFFSET == offsetof(AccelStructMetadataHeader, addressHi), "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_METADATA_SIZE_OFFSET == offsetof(AccelStructMetadataHeader, sizeInBytes), "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET == offsetof(AccelStructMetadataHeader, taskCounter), "");

#ifdef __cplusplus
// =====================================================================================================================
// Miscellaneous packed fields describing the acceleration structure and the build method.
union AccelStructHeaderInfo
{
    struct
    {
        uint32 type                   : 1;  // AccelStructType: TLAS=0, BLAS=1
        uint32 buildType              : 1;  // AccelStructBuilderType: GPU=0, CPU=1
        uint32 mode                   : 4;  // BvhBuildMode/BvhCpuBuildMode based on buildType
                                            // BvhBuildMode: Linear=0, AC=1, PLOC=2
                                            // BvhCpuBuildMode: RecursiveSAH=0, RecursiveLargestExtent=1
        uint32 triCompression         : 3;  // BLAS TriangleCompressionMode: None=0, Two=1, Pair=2
        uint32 fp16BoxNodesInBlasMode : 2;  // BLAS FP16 box mode: None=0, Leaf=1, Mixed=2, All=3
        uint32 triangleSplitting      : 1;  // Enable TriangleSplitting
        uint32 rebraid                : 1;  // Enable Rebraid
        uint32 fusedInstanceNode      : 1;  // Enable fused instance nodes
        uint32 reserved               : 2;  // Unused bits
        uint32 flags                  : 16; // AccelStructBuildFlags
    };

    uint32 u32All;
};
#else
typedef uint32 AccelStructHeaderInfo;
#endif

#ifdef __cplusplus
// =====================================================================================================================
// Miscellaneous packed fields describing the acceleration structure and the build method.
union AccelStructHeaderInfo2
{
    struct
    {
        uint32 compacted              : 1;   // This BVH has been compacted
        uint32 reserved               : 1;   // Unused bits
        uint32 reserved2              : 30;  // Unused bits
    };

    uint32 u32All;
};
#else
typedef uint32 AccelStructHeaderInfo2;
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

#define ACCEL_STRUCT_HEADER_INFO_2_BVH_COMPACTION_FLAGS_SHIFT    0
#define ACCEL_STRUCT_HEADER_INFO_2_BVH_COMPACTION_FLAGS_MASK     0x1

// =====================================================================================================================
// Primary acceleration structure header.
struct AccelStructHeader
{
    AccelStructHeaderInfo  info;                    // Miscellaneous information about the accel struct
    uint32                 metadataSizeInBytes;     // Total size of the metadata in bytes (including metadata header)
    uint32                 sizeInBytes;             // Total size of the accel struct beginning with this header
    uint32                 numPrimitives;           // Number of primitives encoded in the structure
    uint32                 numActivePrims;          // Number of active primitives
    uint32                 taskIdCounter;           // Counter for allocting IDs to tasks in a persistent thread group
    uint32                 numDescs;                // Number of instance/geometry descs in the structure
    uint32                 geometryType;            // Type of geometry contained in a bottom level structure
    AccelStructDataOffsets offsets;                 // Offsets within accel struct (not including the header)
    uint32                 numInternalNodesFp32;    // Number of FP32 internal nodes in the acceleration structure
    uint32                 numInternalNodesFp16;    // Number of FP16 internal nodes in the acceleration structure
    uint32                 numLeafNodes;            // Number of leaf nodes used by the acceleration structure
    uint32                 accelStructVersion;      // GPURT_ACCEL_STRUCT_VERSION
    uint32                 uuidLo;                  // Client-specific UUID (low part)
    uint32                 uuidHi;                  // Client-specific UUID (high part)
    uint32                 rtIpLevel;               // Raytracing hardware IP level
#if __cplusplus
    uint32                 fp32RootBoundingBox[6];  // Root bounding box for bottom level acceleration structures
#else
    uint32                 fp32RootBoundingBox0;
    uint32                 fp32RootBoundingBox1;
    uint32                 fp32RootBoundingBox2;
    uint32                 fp32RootBoundingBox3;
    uint32                 fp32RootBoundingBox4;
    uint32                 fp32RootBoundingBox5;
#endif
    AccelStructHeaderInfo2 info2;
    uint32                 packedFlags;             // Bottom level acceleration structure node flags and instance mask
                                                    // Flags [0:7], Instance Exclusion Mask [8:15]
    uint32                 compactedSizeInBytes;    // Total compacted size of the accel struct

    // if enableSAHCost is enabled,
    // this can be also used to store the actual SAH cost rather than the number of primitives
#if __cpluslus
    uint32                 numChildPrims[4];
#else
    uint32                 numChildPrims0;
    uint32                 numChildPrims1;
    uint32                 numChildPrims2;
    uint32                 numChildPrims3;
#endif

    uint32 GetInfo()
    {
#ifdef __cplusplus
        return info.u32All;
#else
        return info;
#endif
    }

    uint32 GetInfo2()
    {
#ifdef __cplusplus
        return info2.u32All;
#else
        return info2;
#endif
    }

    bool UsesFusedInstanceNode()
    {
        return ((GetInfo() >> ACCEL_STRUCT_HEADER_INFO_FUSED_INSTANCE_NODE_FLAGS_SHIFT) &
            ACCEL_STRUCT_HEADER_INFO_FUSED_INSTANCE_NODE_FLAGS_MASK);
    }

    bool RebraidEnabled()
    {
        return ((GetInfo() >> ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_SHIFT) &
            ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_MASK);
    }

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
#define ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET      48
#define ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP16_NODES_OFFSET      52
#define ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET               56
#define ACCEL_STRUCT_HEADER_VERSION_OFFSET                      60
#define ACCEL_STRUCT_HEADER_UUID_LO_OFFSET                      64
#define ACCEL_STRUCT_HEADER_UUID_HI_OFFSET                      68
#define ACCEL_STRUCT_HEADER_RTIP_LEVEL_OFFSET                   72
#define ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET                76
#define ACCEL_STRUCT_HEADER_INFO_2_OFFSET                       100
#define ACCEL_STRUCT_HEADER_PACKED_FLAGS_OFFSET                 104
#define ACCEL_STRUCT_HEADER_COMPACTED_BYTE_SIZE_OFFSET          108
#define ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET              112

// Acceleration structure root node starts immediately after the header
#define ACCEL_STRUCT_ROOT_NODE_OFFSET ACCEL_STRUCT_HEADER_SIZE

GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_SIZE == sizeof(AccelStructHeader),
    "Acceleration structure header mismatch");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_INFO_OFFSET                    == offsetof(AccelStructHeader, info),                 "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_METADATA_SIZE_OFFSET           == offsetof(AccelStructHeader, metadataSizeInBytes),  "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_BYTE_SIZE_OFFSET               == offsetof(AccelStructHeader, sizeInBytes),          "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_NUM_PRIMS_OFFSET               == offsetof(AccelStructHeader, numPrimitives),        "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET        == offsetof(AccelStructHeader, numActivePrims),       "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_TASK_ID_COUNTER                == offsetof(AccelStructHeader, taskIdCounter),        "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_NUM_DESCS_OFFSET               == offsetof(AccelStructHeader, numDescs),             "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET           == offsetof(AccelStructHeader, geometryType),         "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_OFFSETS_OFFSET                 == offsetof(AccelStructHeader, offsets),              "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET == offsetof(AccelStructHeader, numInternalNodesFp32), "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP16_NODES_OFFSET == offsetof(AccelStructHeader, numInternalNodesFp16), "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET          == offsetof(AccelStructHeader, numLeafNodes),         "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_VERSION_OFFSET                 == offsetof(AccelStructHeader, accelStructVersion),   "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_UUID_LO_OFFSET                 == offsetof(AccelStructHeader, uuidLo),               "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_UUID_HI_OFFSET                 == offsetof(AccelStructHeader, uuidHi),               "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_RTIP_LEVEL_OFFSET              == offsetof(AccelStructHeader, rtIpLevel),            "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET           == offsetof(AccelStructHeader, fp32RootBoundingBox),  "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_INFO_2_OFFSET                  == offsetof(AccelStructHeader, info2),                "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_PACKED_FLAGS_OFFSET            == offsetof(AccelStructHeader, packedFlags),          "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_COMPACTED_BYTE_SIZE_OFFSET     == offsetof(AccelStructHeader, compactedSizeInBytes), "");
GPURT_STATIC_ASSERT(ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET         == offsetof(AccelStructHeader, numChildPrims0),       "");

#ifdef __cplusplus
// =====================================================================================================================
// Miscellaneous flags describing the acceleration structure.
union RdfAccelStructHeaderFlags
{
    struct
    {
        uint32 blas     : 1;  // Indicates a bottom level acceleration structure
        uint32 reserved : 31; // Unused bits
    };

    uint32 u32All;
};
#else
typedef uint32 RdfAccelStructHeaderFlags;
#endif

// =====================================================================================================================
// RDF chunk header for a raw memory dump of GPURT's acceleration structure.
// Chunk Identifier: "RawAccelStruct"
// Chunk Version: GPURT_ACCEL_STRUCT_VERSION
struct RawAccelStructRdfChunkHeader
{
    // GPUVA where the application built the acceleration structure
    uint32 accelStructBaseVaLo;
    uint32 accelStructBaseVaHi;

    // Metadata header (AccelStructMetadataHeader)
    uint32 metaHeaderOffset;            // Offset of the driver metadata header from the start of the chunk payload
    uint32 metaHeaderSize;              // Size of the driver metadata header

    // Primary Header (AccelStructHeader)
    uint32 headerOffset;                // Offset of the driver header from the start of the chunk payload
    uint32 headerSize;                  // Size of the driver header

    RdfAccelStructHeaderFlags flags;    // Miscellaneous flags
};

//=====================================================================================================================
// Additional header data for driver internal decode
struct DriverDecodeHeader
{
    AccelStructHeaderInfo  info;
    AccelStructHeaderInfo2 info2;
};

#ifdef __cplusplus
}
#endif
#endif
