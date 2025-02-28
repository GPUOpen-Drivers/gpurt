/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2020-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "src/shared/gpurtBuildConstants.h"

namespace GpuRt
{
namespace Update
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

namespace BuildTrivialBvh
{
    struct Constants
    {
        uint32 maxGeometryCount;
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

namespace BuildBVH
{
    constexpr uint32 NumEntries = (sizeof(BuildShaderRootConstants) / sizeof(uint32));
};

/* Common Build Shaders Mapping */
constexpr NodeMapping BuildBVHMapping[] =
{
    { NodeType::Constant, BuildBVH::NumEntries },   // BuildShaderRootConstants
    { NodeType::ConstantBuffer, 2 },                // BuildShaderConstants
    { NodeType::ConstantBuffer, 2 },                // LutBuffer
    { NodeType::Uav, 2 },                           // SrcBuffer
    { NodeType::Uav, 2 },                           // DstBuffer
    { NodeType::Uav, 2 },                           // DstMetadata
    { NodeType::Uav, 2 },                           // ScratchBuffer
    { NodeType::Uav, 2 },                           // ScratchGlobal
    { NodeType::Uav, 2 },                           // InstanceDescBuffer
    { NodeType::Uav, 2 },                           // EmitBuffer
    { NodeType::Uav, 2 },                           // IndirectArgBuffer
#if GPURT_ENABLE_GPU_DEBUG
    { NodeType::Uav, 2 },                           // DebugBuffer
#endif
    { NodeType::ConstantBufferTable, 1 },           // GeometryConstants
    { NodeType::TypedUavTable, 1 },                 // GeometryBuffer
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
        uint32 plocTaskQueueCounterScratchOffset;
        uint32 encodeTaskCounterScratchOffset;
        uint32 taskLoopCountersOffset;

        uint32 isIndirectBuild;
        uint32 dynamicallyIncrementsPrimRefCount;
        uint32 padding0;
        uint32 padding1;

        AccelStructHeader header;
        AccelStructMetadataHeader metadataHeader;
    };

    constexpr uint32 NumEntries = (sizeof(Constants) / sizeof(uint32));
}

/* Update Shaders NodeMappings */

constexpr NodeMapping UpdateQBVHMapping[] =
{
    { NodeType::Constant, Update::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping UpdateParallelMapping[] =
{
    { NodeType::Constant, Update::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 }
};

constexpr NodeMapping UpdateMapping[]
{
    { NodeType::Constant, Update::NumEntries },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::ConstantBuffer, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::Uav, 2 },
    { NodeType::ConstantBufferTable, 1 },
    { NodeType::TypedUavTable, 1 },
    { NodeType::Uav, 2 }, // NullBuffer mapping.
};

#define UpdateTrianglesMapping UpdateMapping
#define UpdateAabbsMapping UpdateMapping

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
    { NodeType::UavTable, 1 },
    { NodeType::UavTable, 1 },
};

} // namespace GpuRt
