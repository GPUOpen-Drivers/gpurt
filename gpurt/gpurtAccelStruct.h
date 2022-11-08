/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2022 Advanced Micro Devices, Inc. All Rights Reserved.
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

#define GPURT_ACCEL_STRUCT_MAJOR_VERSION 15
#define GPURT_ACCEL_STRUCT_MINOR_VERSION 8
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
    uint32 geometryInfo;        // Offset to geometry desc info (bottom level only)
    uint32 primNodePtrs;        // Offset to prim node pointers (BVH4 with triangle compression or ALLOW_UPDATE only)
};

GPURT_STATIC_ASSERT(sizeof(AccelStructDataOffsets) == 16,
                    "AccelStructDataOffsets size cannot change because it is embedded in AccelStructHeader.");

// =====================================================================================================================
// Header for acceleration structure metadata
struct AccelStructMetadataHeader
{
    uint32 addressLo;           // Address of acceleration structure data section (low bits)
    uint32 addressHi;           // Address of acceleration structure data section (high bits)
    uint32 sizeInBytes;         // Metadata size in bytes (including this header)
    uint32 reserved0;           // Reserved
    uint32 taskCounter;         // Task counter for dispatch-wide spin loop sync. Align to 8 bytes so taskCounter and
                                // numTasksDone can be reset in one 64 bit CP write.
    uint32 numTasksDone;        // Number of tasks done
    uint32 reserved1;           // Reserved
    uint32 reserved2;           // Reserved
    uint32 reserved3;           // Reserved
};

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
        uint32 compacted     : 1;  // This BVH has been compacted
        uint32 reserved      : 31; // Unused bits
    };

    uint32 u32All;
};
#else
typedef uint32 AccelStructHeaderInfo2;
#endif

// =====================================================================================================================
// Primary acceleration structure header.
struct AccelStructHeader
{
    AccelStructHeaderInfo  info;                     // Miscellaneous information about the accel struct
    uint32                 metadataSizeInBytes;      // Total size of the metadata in bytes (including metadata header)
    uint32                 sizeInBytes;              // Total size of the accel struct beginning with this header
    uint32                 numPrimitives;            // Number of primitives encoded in the structure
    uint32                 numActivePrims;           // Number of active primitives
    uint32                 taskIdCounter;            // Counter for allocting IDs to tasks in a persistent thread group
    uint32                 numDescs;                 // Number of instance/geometry descs in the structure
    uint32                 geometryType;             // Type of geometry contained in a bottom level structure
    AccelStructDataOffsets offsets;                  // Offsets within accel struct (not including the header)
    uint32                 numInternalNodesFp32;     // Number of FP32 internal nodes in the acceleration structure
    uint32                 numInternalNodesFp16;     // Number of FP16 internal nodes in the acceleration structure
    uint32                 numLeafNodes;             // Number of leaf nodes used by the acceleration structure
    uint32                 accelStructVersion;       // GPURT_ACCEL_STRUCT_VERSION
    uint32                 uuidLo;                   // Client-specific UUID (low part)
    uint32                 uuidHi;                   // Client-specific UUID (high part)
    uint32                 reserved;                 // Unused bits
#if __cplusplus
    uint32                 fp32RootBoundingBox[6];   // Root bounding box for bottom level acceleration structures
#else
    uint32                 fp32RootBoundingBox0;
    uint32                 fp32RootBoundingBox1;
    uint32                 fp32RootBoundingBox2;
    uint32                 fp32RootBoundingBox3;
    uint32                 fp32RootBoundingBox4;
    uint32                 fp32RootBoundingBox5;
#endif
    AccelStructHeaderInfo2 info2;
    uint32                 nodeFlags;                // Bottom level acceleration structure node flags.
    uint32                 compactedSizeInBytes;     // Total compacted size of the accel struct

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
};

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

#ifdef __cplusplus
};
#endif
#endif
