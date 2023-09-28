/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2020-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
#pragma once
#include "gpurt/gpurt.h"

namespace GpuRt
{

// Constants for Ray Tracing shaders
namespace EncodeNodes
{
    struct Constants
    {
        uint32 metadataSizeInBytes;           // Size of the Metadata Header
        uint32 numPrimitives;                 // Number of primitives
        uint32 leafNodeDataByteOffset;        // Leaf node data byte offset
        uint32 primitiveDataByteOffset;       // Scratch data byte offset
        uint32 sceneBoundsByteOffset;         // Scene bounds byte offset
        uint32 propagationFlagsScratchOffset; // Flags byte offset
        uint32 baseUpdateStackScratchOffset;  // Update stack byte offset
        uint32 indexBufferByteOffset;         // Index buffer byte offset
        uint32 hasValidTransform;             // Has a valid transform
        uint32 indexBufferFormat;             // Index buffer format
        uint32 geometryStride;                // Geometry buffer stride in terms of components for vertices. 0 if the
                                              // stride is accounted for in the SRD. R32G32 elements for AABBs.
        uint32 geometryIndex;                 // Index of the geometry description that owns this node
        uint32 baseGeometryInfoOffset;        // Base offset of the geometry info
        uint32 basePrimNodePtrOffset;         // Base offset for the prim node ptrs
        uint32 buildFlags;                    // Acceleration structure build flags
        uint32 isUpdateInPlace;               // Is update in place
        uint32 geometryFlags;                 // Geometry flags (GpuRt::GeometryFlags)
        uint32 vertexComponentCount;          // Valid components in vertex buffer format
        uint32 vertexCount;                   // Vertex count
        uint32 destLeafNodeOffset;            // Offset of leaf nodes in destination buffer
        uint32 leafNodeExpansionFactor;       // Leaf node expansion factor (> 1 for triangle splitting)
        uint32 indexBufferInfoScratchOffset;
        uint32 indexBufferGpuVaLo;
        uint32 indexBufferGpuVaHi;
        uint32 sceneBoundsCalculationType;
        uint32 trianglePairingSearchRadius;    // Search radius for triangles that could be paired together
    };
    // Add 4 DWORD padding to avoid page faults when the compiler uses a multi-DWORD load straddling the end of the
    // constant buffer
    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32)) + 4;
}

namespace BuildBVHPLOC
{
    struct Constants
    {
        uint32 numThreads;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace UpdateQBVH
{
    struct Constants
    {
        uint32 isInPlace;     // Non-zero if updates are performed in-place
        uint32 addressLo;
        uint32 addressHi;
        uint32 propagationFlagsScratchOffset;
        uint32 baseUpdateStackScratchOffset;
        uint32 triangleCompressionMode;         // Triangle node compression mode
        uint32 fp16BoxNodesInBlasMode;          // Mode for which internal nodes in BLAS are fp16
        uint32 numPrimitives;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace UpdateParallel
{
    struct Constants
    {
        uint32 isInPlace;                        // Non-zero if updates are performed in-place
        uint32 addressLo;
        uint32 addressHi;
        uint32 propagationFlagsScratchOffset;
        uint32 baseUpdateStackScratchOffset;
        uint32 triangleCompressionMode;          // Triangle node compression mode
        uint32 fp16BoxNodesInBlasMode;           // Mode for which internal nodes in BLAS are fp16
        uint32 numThreads;
        uint32 numPrimitives;
        uint32 numDescs;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace EncodeBVH2
{
    struct Constants
    {
        uint32 numPrimitives;       // Number of primitives
        uint32 metadataSizeInBytes; // Acceleration structure metadata size in bytes
        uint32 sizeInBytes;         // Acceleration structure size in bytes
        uint32 addressLo;           // Acceleration structure address low bits
        uint32 addressHi;           // Acceleration structure address high bits
        uint32 numDescs;            // Number of geometry/instance descs
        uint32 geometryType;        // Geometry type (bottom-level only)
        uint32 bvhNodeDataScratchOffset;
        uint32 bvh2PrimsScratchOffset;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace BuildQBVH
{
    struct Constants
    {
        uint32 numThreads;                       // Number of persistent threads
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace BuildBVHTD
{
    struct Constants
    {
        uint32 numThreads;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace EncodeInstances
{
    struct Constants
    {
        uint32 metadataSizeInBytes;           // Size of the metadata header
        uint32 basePrimNodePtrOffset;         // Base Offset to the Prim Nodes
        uint32 numDescs;                      // Number of desc
        uint32 leafNodeDataByteOffset;        // Leaf node data byte offset
        uint32 sceneBoundsByteOffset;         // Scene bounds byte offset
        uint32 propagationFlagsScratchOffset; // Flags byte offset
        uint32 baseUpdateStackScratchOffset;  // Update stack byte offset
        uint32 internalFlags;                 // Flags
        uint32 buildFlags;                    // Build flags
        uint32 leafNodeExpansionFactor;       // Number of leaf nodes per primitive
        uint32 sceneBoundsCalculationType;
        uint32 enableFastLBVH;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace ClearBuffer
{
    struct Constants
    {
        uint32 numDwords; // Number of dwords to clear
        uint32 value;     // Clear value
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace CopyBufferRaw
{
    struct Constants
    {
        uint32 numDwords; // Number of dwords to copy
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace BuildParallel
{
    struct Constants
    {
        uint32 numThreadGroups;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
};

namespace Rebraid
{
    struct Constants
    {
        uint32 numThreadGroups;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace RadixSort
{
    struct Constants
    {
        uint32 bitShiftSize; // Number of bits to shift
        uint32 numGroups;    // Number of groups
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace BitHistogram
{
    struct Constants
    {
        uint32 bitShiftSize; // Number of bits to shift
        uint32 numGroups;    // Number of groups
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace ScanExclusiveAddWG
{
    struct Constants
    {
        uint32 numElements;  // Number of elements
        uint32 inOutArrayScratchOffset;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace ScanExclusivePartSum
{
    struct Constants
    {
        uint32 numElements;  // Number of elements
        uint32 inOutArrayScratchOffset;
        uint32 partSumsScratchOffset;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace DistributePartSum
{
    struct Constants
    {
        uint32 numElements;  // Number of elements
        uint32 outputArrayScratchOffset;
        uint32 partSumsScratchOffset;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace ScanExclusiveAddDLB
{
    struct Constants
    {
        uint32 numElements;  // Number of elements
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace EmitSize
{
    struct Constants
    {
        uint32 offset;                          // Descriptor offset
        uint32 fp16BoxNodesInBlasMode;          // Mode for which internal nodes in BLAS are fp16
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace CopyAS
{
    struct Constants
    {
        uint32 numWaves;  // Optimized number of waves to launch for copying
        uint32 addressLo; // Copy destination address after skipping header- low bits
        uint32 addressHi; // Copy destination address after skipping header- high bits
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace CompactAS
{
    struct Constants
    {
        uint32 numThreads;// Optimized number of threads to launch for compaction copying
        uint32 addressLo; // Copy destination address after skipping header- low bits
        uint32 addressHi; // Copy destination address after skipping header- high bits
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace SerializeAS
{
    struct Constants
    {
        uint32 numWaves; // Optimized number of waves to launch for copying
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace DeserializeAS
{
    struct Constants
    {
        uint32 addressLo;       // Copy destination address after skipping header- low bits
        uint32 addressHi;       // Copy destination address after skipping header- high bits
        uint32 numWaves;        // Optimized number of waves to launch for copying
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace DecodeAS
{
    struct Constants
    {
        uint32 addressLo;               // Copy destination address after skipping header- low bits
        uint32 addressHi;               // Copy destination address after skipping header- high bits
        uint32 numThreads;              // Number of persistent threads
        uint32 rtIpLevel;               // Raytracing IP level
        uint32 isDriverDecode;          // Internal driver decode
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

/* Encode Shaders NodeMappings */

constexpr NodeMapping EncodeTriangleNodesMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::TypedUavTable, 1 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping EncodeTriangleNodesIndirectMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::TypedUavTable, 1 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping EncodeAABBNodesMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::TypedUavTable, 1 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping EncodeInstancesMapping[] =
{
    { NodeType::Constant, EncodeInstances::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

/* Build Shaders NodeMappings */

namespace InitAccelerationStructure
{
    struct RootConstants
    {
        uint32 numBuilders;
    };

    struct Constants
    {
        uint32 rebraidType;
        uint32 triangleCompressionMode;
        uint32 debugCountersScratchOffset;
        uint32 sceneBoundsScratchOffset;

        // Add padding for 16-byte alignment rules
        uint32 numBatchesScratchOffset;
        uint32 numLeafNodes;
        uint32 pad0;
        uint32 pad1;

        AccelStructHeader header;
        AccelStructMetadataHeader metadataHeader;
    };

    constexpr uint32 NumRootEntries = (sizeof(RootConstants) / sizeof(uint32));
    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

constexpr NodeMapping BuildParallelMapping[] =
{
    { NodeType::Constant, BuildParallel::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
#if GPURT_ENABLE_GPU_DEBUG
    { NodeType::Uav, 2 },
#endif
    { NodeType::ConstantBufferTable, 1 },
    { NodeType::UavTable, 1 },
    { NodeType::UavTable, 1 },
    { NodeType::UavTable, 1 },
};

constexpr NodeMapping RebraidMapping[] =
{
    { NodeType::Constant, Rebraid::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
};

constexpr NodeMapping GenerateMortonCodesMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping BuildBVHMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping BuildBVHSortLeavesMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping BuildBVHTDMapping[] =
{
    { NodeType::Constant, BuildBVHTD::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

#define BuildBVHTD32Mapping BuildBVHTDMapping

constexpr NodeMapping BuildBVHTDTRMapping[] =
{
    { NodeType::Constant, BuildBVHTD::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

#define BuildBVHTDTR32Mapping BuildBVHTDTRMapping

constexpr NodeMapping BuildBVHPLOCMapping[] =
{
    { NodeType::Constant, BuildBVHPLOC::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping RefitBoundsMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping PairCompressionMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping InitBuildQBVHMapping[] =
{
    { NodeType::Constant, BuildQBVH::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping BuildQBVHMapping[] =
{
    { NodeType::Constant, BuildQBVH::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping MergeSortMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping BitHistogramMapping[] =
{
    { NodeType::Constant, BitHistogram::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping ScatterKeysAndValuesMapping[] =
{
    { NodeType::Constant, RadixSort::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping ScanExclusiveInt4Mapping[] =
{
    { NodeType::Constant, ScanExclusiveAddWG::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping ScanExclusivePartInt4Mapping[] =
{
    { NodeType::Constant, ScanExclusivePartSum::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping ScanExclusiveInt4DLBMapping[] =
{
    { NodeType::Constant, ScanExclusiveAddDLB::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping InitScanExclusiveInt4DLBMapping[] =
{
    { NodeType::Constant, ScanExclusiveAddDLB::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping DistributePartSumInt4Mapping[] =
{
    { NodeType::Constant, DistributePartSum::NumEntries  },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

/* Update Shaders NodeMappings */

constexpr NodeMapping UpdateQBVHMapping[] =
{
    { NodeType::Constant, UpdateQBVH::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping UpdateParallelMapping[] =
{
    { NodeType::Constant, UpdateParallel::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping UpdateMapping[]
{
    { NodeType::Constant, UpdateParallel::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::ConstantBufferTable, 1 },
    { NodeType::UavTable, 1 },
    { NodeType::UavTable, 1 },
    { NodeType::UavTable, 1 }
};

constexpr NodeMapping ClearBufferMapping[] =
{
    { NodeType::Constant, ClearBuffer::NumEntries },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping CopyBufferRawMapping[] =
{
    { NodeType::Constant, CopyBufferRaw::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping EmitCurrentSizeMapping[] =
{
    { NodeType::Constant, EmitSize::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping EmitCompactSizeMapping[] =
{
    { NodeType::Constant, EmitSize::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping EmitSerializeDescMapping[] =
{
    { NodeType::Constant, EmitSize::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping EmitToolVisDescMapping[] =
{
    { NodeType::Constant, EmitSize::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping CopyASMapping[] =
{
    { NodeType::Constant, CopyAS::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping CompactASMapping[] =
{
    { NodeType::Constant, CompactAS::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping DecodeASMapping[] =
{
    { NodeType::Constant, DecodeAS::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
};

constexpr NodeMapping SerializeASMapping[] =
{
    { NodeType::Constant, SerializeAS::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping DeserializeASMapping[] =
{
    { NodeType::Constant, DeserializeAS::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping InitExecuteIndirectMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
};

constexpr NodeMapping InitAccelerationStructureMapping[] =
{
    { NodeType::Constant, InitAccelerationStructure::NumRootEntries },
    { NodeType::ConstantBufferTable, 1 },
    { NodeType::UavTable, 1 },
    { NodeType::UavTable, 1 },
};

} // namespace GpuRt
