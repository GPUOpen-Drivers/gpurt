/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2020-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

namespace BuildBVHPLOC
{
    struct Constants
    {
        uint32 numThreads;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace Update
{
    struct Constants
    {
        uint32 numThreads;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace EncodeHwBvh
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
        uint32 numElements;
        uint32 passIndex;
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
        uint32 addressLo;       // Copy destination address after skipping header- low bits
        uint32 addressHi;       // Copy destination address after skipping header- high bits
        uint32 numThreads;      // Number of persistent threads
        uint32 isDriverDecode;  // Internal driver decode
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

namespace EncodePrimitive
{
    struct Constants
    {
        uint32 geometryIndex;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

/* Encode Shaders NodeMappings */

constexpr NodeMapping EncodeTriangleNodesMapping[] =
{
    { NodeType::Constant, EncodePrimitive::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::ConstantBufferTable, 1 },
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

#define EncodeAABBNodesMapping EncodeTriangleNodesMapping
#define EncodeQuadNodesMapping EncodeTriangleNodesMapping

constexpr NodeMapping EncodeInstancesMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

/* Build Shaders NodeMappings */

namespace InitAccelerationStructure
{
    struct Constants
    {
        uint32 maxNumPrimitives;
        uint32 debugCountersScratchOffset;
        uint32 sceneBoundsScratchOffset;
        uint32 numBatchesScratchOffset;

        uint32 rebraidTaskQueueCounterScratchOffset;
        uint32 tdTaskQueueCounterScratchOffset;
        uint32 plocTaskQueueCounterScratchOffset;
        uint32 encodeTaskCounterScratchOffset;

        uint32 taskLoopCountersOffset;
        uint32 isIndirectBuild;
        uint32 dynamicallyIncrementsPrimRefCount;
        uint32 padding1;

        AccelStructHeader header;
        AccelStructMetadataHeader metadataHeader;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

constexpr NodeMapping BuildParallelMapping[] =
{
    { NodeType::Constant, BuildParallel::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
#if GPURT_ENABLE_GPU_DEBUG
    { NodeType::Uav, 2 },
#endif
    { NodeType::ConstantBufferTable, 1 },
    { NodeType::TypedUavTable, 1 },
};

constexpr NodeMapping RebraidMapping[] =
{
    { NodeType::Constant, Rebraid::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
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
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping BuildBVHMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping BuildFastAgglomerativeLbvhMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
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
    { NodeType::Uav, 2 },
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
    { NodeType::Uav, 2 },
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
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping RefitBoundsMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping PairCompressionMapping[] =
{
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::ConstantBufferTable, 1 },
    { NodeType::TypedUavTable, 1 },
};

constexpr NodeMapping BuildQBVHMapping[] =
{
    { NodeType::Constant, EncodeHwBvh::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
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
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
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
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
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
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping ScanExclusiveInt4Mapping[] =
{
    { NodeType::Constant, ScanExclusiveAddWG::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping ScanExclusivePartInt4Mapping[] =
{
    { NodeType::Constant, ScanExclusivePartSum::NumEntries },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
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
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
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
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping DistributePartSumInt4Mapping[] =
{
    { NodeType::Constant, DistributePartSum::NumEntries  },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

/* Update Shaders NodeMappings */

constexpr NodeMapping UpdateQBVHMapping[] =
{
    { NodeType::Constant, Update::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping UpdateParallelMapping[] =
{
    { NodeType::Constant, Update::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping UpdateTrianglesMapping[]
{
    { NodeType::Constant, Update::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::ConstantBufferTable, 1 },
    { NodeType::TypedUavTable, 1 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
};

constexpr NodeMapping UpdateAabbsMapping[]
{
    { NodeType::Constant, Update::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::ConstantBufferTable, 1 },
    { NodeType::TypedUavTable, 1 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
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
    { NodeType::UavTable, 1 },
    { NodeType::ConstantBufferTable, 1 },
    { NodeType::UavTable, 1 },
    { NodeType::UavTable, 1 },
};

constexpr NodeMapping InitUpdateAccelerationStructureMapping[] =
{
    { NodeType::UavTable, 1 },
};

} // namespace GpuRt
