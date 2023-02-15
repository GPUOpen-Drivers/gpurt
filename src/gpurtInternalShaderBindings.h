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
        uint32 triangleCompressionMode;       // Triangle node compression mode
        uint32 indexBufferInfoScratchOffset;
        uint32 indexBufferGpuVaLo;
        uint32 indexBufferGpuVaHi;
        uint32 sceneBoundsCalculationType;
        uint32 enableTriangleSplitting;
        uint32 enableEarlyPairCompression;
        uint32 enableFastLBVH;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace BuildBVH
{
    struct Constants
    {
        uint32 allowUpdates;  // Non-zero if updates are allowed on this BVH
        uint32 bvhNodeDataScratchOffset;
        uint32 bvhLeafNodeDataScratchOffset;
        uint32 mortonCodesSortedScratchOffset;
        uint32 primIndicesSortedScratchOffset;
        uint32 useMortonCode30;
        uint32 noCopySortedNodes;
        uint32 enableFastLBVH;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace BuildBVHPLOC
{
    struct Constants
    {
        uint32 numThreads;
        uint32 scratchNodesScratchOffset;
        uint32 clusterList0ScratchOffset;
        uint32 clusterList1ScratchOffset;
        uint32 neighbourIndicesScratchOffset;
        uint32 currentStateScratchOffset;
        uint32 atomicFlagsPLOCScratchOffset;
        uint32 clusterOffsetsScratchOffset;
        uint32 dynamicBlockIndexScratchOffset;
        uint32 numBatchesScratchOffset;
        uint32 baseBatchIndicesScratchOffset;
        uint32 fp16BoxNodesInBlasMode;
        float  fp16BoxModeMixedSaThresh;
        uint32 flags;
        uint32 splitBoxesByteOffset;
        uint32 plocRadius;
        uint32 primIndicesSortedScratchOffset;
        uint32 numLeafNodes;
        uint32 noCopySortedNodes;
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
        uint32 numPrimitives;                    // Number of primitives
        uint32 metadataSizeInBytes;              // Acceleration structure metadata size in bytes
        uint32 numThreads;                       // Number of persistent threads
        uint32 bvhNodeDataScratchOffset;
        uint32 qbvhGlobalStackScratchOffset;
        uint32 qbvhGlobalStackPtrsScratchOffset;
        uint32 triangleCompressionMode;          // Triangle node compression mode
        uint32 fp16BoxNodesInBlasMode;           // Mode for which internal nodes in BLAS are fp16
        uint32 doTriangleSplitting;
        uint32 splitBoxesByteOffset;
        uint32 emitCompactSize;
        uint32 encodeArrayOfPointers;
        uint32 topDownBuild;

        uint32 bvhBuilderNodeSortType;           // bvhBuilder node sort
        uint32 bvhBuilderNodeSortHeuristic;

        uint32 enableFusedInstanceNode;
        uint32 sahQbvh;                          // Apply SAH into QBVH build

        uint32 captureChildNumPrimsForRebraid;
        uint32 enableSAHCost;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace BuildBVHTD
{
    struct Constants
    {
        uint32 numPrimitives;
        uint32 isUpdateAllowed;
        uint32 numThreads;
        uint32 numRefsAlloced;
        float  lengthPercentage;
        uint32 bvhNodeDataOffset;
        uint32 bvhLeafNodeDataOffset;
        uint32 sceneBoundsOffset;
        uint32 refListOffset;
        uint32 tdNodeListOffset;
        uint32 tdBinsOffset;
        uint32 tdStateOffset;
        uint32 tdTaskQueueCounter;
        uint32 refOffsetsOffset;
        uint32 encodeArrayOfPointers;
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

namespace RefitBounds
{
    struct Constants
    {
        uint32 propagationFlagsScratchOffset;
        uint32 scratchNodesScratchOffset;
        uint32 fp16BoxNodesInBlasMode;      // Mode for which internal nodes in BLAS are fp16
        float  fp16BoxModeMixedSaThresh;    // For fp16 mode "mixed", surface area threshold.
        uint32 doCollapse;
        uint32 doTriangleSplitting;
        uint32 enablePairCompression;
        uint32 enablePairCostCheck;
        uint32 splitBoxesByteOffset;
        uint32 numBatchesScratchOffset;
        uint32 batchIndicesScratchOffset;
        uint32 noCopySortedNodes;
        uint32 sortedPrimIndicesOffset;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace PairCompression
{
    struct Constants
    {
        uint32 scratchNodesScratchOffset;
        uint32 numBatchesScratchOffset;
        uint32 batchIndicesScratchOffset;
        uint32 indexBufferInfoScratchOffset;
        uint32 propagationFlagsScratchOffset;
        uint32 buildFlags;
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
    struct ScratchOffsets
    {
        uint32 mortonCodes;
        uint32 mortonCodesSorted;
        uint32 primIndicesSorted;
        uint32 primIndicesSortedSwap;
        uint32 tempKeys;
        uint32 tempVals;
        uint32 histogram;
        uint32 bvhNodes;
        uint32 unsortedBvhLeafNodes;
        uint32 sceneBounds;
        uint32 partialSums;
        uint32 propagationFlags;
        uint32 dynamicBlockIndex;
        uint32 prefixSumAtomicFlags;
        uint32 fastLBVHRootNodeIndex;

        // PLOC
        uint32 clusterList0;
        uint32 clusterList1;
        uint32 neighborIndices;
        uint32 currentState;
        uint32 atomicFlagsPloc;
        uint32 clusterOffsets;

        // AC
        uint32 clusterList;
        uint32 numClusterList;
        uint32 internalNodesIndex;

        // Build QBVH
        uint32 qbvhStack;
        uint32 stackPtrs;

        // Tri Split
        uint32 splitBoxes;
        uint32 refList0;
        uint32 refList1;
        uint32 splitPriorities;
        uint32 currentSplitState;
        uint32 splitAtomicFlags;

        // Pair compression
        uint32 numBatches;
        uint32 batchIndices;
        uint32 indexBufferInfo;

        // TD Build
        uint32 tdRefs;
        uint32 tdNodes;
        uint32 tdBins;
        uint32 tdState;
        uint32 tdTaskCounters;

        uint32 reservedUint4;
        uint32 reservedUint5;
        uint32 reservedUint6;
        uint32 reservedUint7;
        uint32 reservedUint8;
        uint32 reservedUint9;

        // debug
        uint32 debugCounters;
    };

    struct Constants
    {
        uint32 resultBufferAddrLo;
        uint32 resultBufferAddrHi;
        uint32 numPrimitives;
        uint32 numThreadGroups;
        uint32 tsBudgetPerTriangle;
        uint32 maxNumPrimitives;

        uint32 encodeArrayOfPointers;
        uint32 rebraidFactor;
        uint32 numMortonSizeBits;
        float reservedFloat;
        uint32 numLeafNodes;
        uint32 numDescs;

        // Align the following struct to 4 dwords due to HLSL constant buffer packing rules
        AccelStructHeader header;
        ScratchOffsets offsets;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
};

namespace GenerateMortonCodes
{
    struct Constants
    {
        uint32 numPrimitives;            // Number of bounding boxes
        uint32 leafNodeDataByteOffset;   // Leaf node data byte offset
        uint32 sceneBoundsByteOffset;    // Scene bounds byte offset
        uint32 mortonCodeDataByteOffset; // Morton codes byte offset
        uint32 doTriangleSplitting;
        uint32 splitBoxesByteOffset;
        uint32 enableVariableBitsMortonCode;
        uint32 useMortonCode30;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace MergeSort
{
    struct Constants
    {
        uint32 numPrimitives;
        uint32 inputKeysScratchOffset;
        uint32 outputKeysScratchOffset;
        uint32 outputValuesScratchOffset;
        uint32 outputValuesSwapScratchOffset;
        uint32 useMortonCode30;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace RadixSort
{
    struct Constants
    {
        uint32 bitShiftSize; // Number of bits to shift
        uint32 numElements;  // Number of elements
        uint32 numGroups;    // Number of groups
        uint32 inputKeysScratchOffset;
        uint32 inputValuesScratchOffset;
        uint32 histogramsScratchOffset;
        uint32 outputKeysScratchOffset;
        uint32 outputValuesScratchOffset;
        uint32 useMortonCode30;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace BitHistogram
{
    struct Constants
    {
        uint32 bitShiftSize; // Number of bits to shift
        uint32 numElements;  // Number of elements
        uint32 numGroups;    // Number of groups
        uint32 inputArrayScratchOffset;
        uint32 outputArrayScratchOffset;
        uint32 useMortonCode30;
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
        uint32 inOutArrayScratchOffset;
        uint32 dynamicBlockIndexScratchOffset;
        uint32 atomicFlagsScratchOffset;
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
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

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

constexpr NodeMapping BuildParallelMapping[] =
{
    { NodeType::Constant, BuildParallel::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::ConstantBufferTable, 1 },
    { NodeType::UavTable, 1 },
    { NodeType::UavTable, 1 },
    { NodeType::UavTable, 1 },
};

constexpr NodeMapping GenerateMortonCodesMapping[] =
{
    { NodeType::Constant, GenerateMortonCodes::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping BuildBVHMapping[] =
{
    { NodeType::Constant, BuildBVH::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping BuildBVHSortLeavesMapping[] =
{
    { NodeType::Constant, BuildBVH::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping BuildBVHTDMapping[] =
{
    { NodeType::Constant, BuildBVHTD::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

#define BuildBVHTD32Mapping BuildBVHTDMapping

constexpr NodeMapping BuildBVHTDTRMapping[] =
{
    { NodeType::Constant, BuildBVHTD::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

#define BuildBVHTDTR32Mapping BuildBVHTDTRMapping

constexpr NodeMapping BuildBVHPLOCMapping[] =
{
    { NodeType::Constant, BuildBVHPLOC::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

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

#define UpdateParallel32Mapping UpdateParallelMapping

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

constexpr NodeMapping RefitBoundsMapping[] =
{
    { NodeType::Constant, RefitBounds::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping PairCompressionMapping[] =
{
    { NodeType::Constant, PairCompression::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
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

constexpr NodeMapping InitBuildQBVHMapping[] =
{
    { NodeType::Constant, BuildQBVH::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping BuildQBVHMapping[] =
{
    { NodeType::Constant, BuildQBVH::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping BuildQBVHCollapseMapping[] =
{
    { NodeType::Constant, BuildQBVH::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping MergeSortMapping[] =
{
    { NodeType::Constant, MergeSort::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping BitHistogramMapping[] =
{
    { NodeType::Constant, BitHistogram::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping ScatterKeysAndValuesMapping[] =
{
    { NodeType::Constant, RadixSort::NumEntries },
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
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping InitScanExclusiveInt4DLBMapping[] =
{
    { NodeType::Constant, ScanExclusiveAddDLB::NumEntries },
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

constexpr NodeMapping DecodeCollapseMapping[] =
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

} // namespace GpuRt
