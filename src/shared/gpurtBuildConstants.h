/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _GPURT_BUILD_CONSTANTS_H
#define _GPURT_BUILD_CONSTANTS_H

#include "../../gpurt/gpurtAccelStruct.h"

// This file provides the definition of the GPURT structures used by the GPURT BVH build shader code.
//
// Note this file is designed to be compilable as HLSL.

#ifdef __cplusplus
namespace GpuRt
{
#endif

typedef uint32_t uint32;
typedef uint64_t uint64;

// =====================================================================================================================
// Scratch data offsets required for BVH construction
struct RayTracingScratchDataOffsets
{
    uint32 bvhNodeData;
    uint32 triangleSplitBoxes;
    uint32 triangleSplitRefs0;
    uint32 triangleSplitRefs1;
    uint32 splitPriorities;
    uint32 triangleSplitState;
    uint32 rebraidState;
    uint32 triangleSplitTaskQueueCounter;
    uint32 rebraidTaskQueueCounter;
    uint32 splitAtomicFlags;
    uint32 tdRefs;
    uint32 tdNodeList;
    uint32 tdBins;
    uint32 tdState;
    uint32 tdTaskQueueCounter;
    uint32 refOffsets;
    uint32 bvhLeafNodeData;
    uint32 fastLBVHRootNodeIndex;
    uint32 clusterList0;
    uint32 clusterList1;
    uint32 neighborIndices;     // BVH PLOC build only
    uint32 currentState;        // BVH PLOC build only
    uint32 plocTaskQueueCounter;// BVH PLOC build only
    uint32 atomicFlagsPloc;     // BVH PLOC build only
    uint32 clusterOffsets;      // BVH PLOC build only
    uint32 reserved0;
    uint32 reserved1;
    uint32 reserved2;
    uint32 reserved3;
    uint32 reserved4;
    uint32 reserved5;
    uint32 reserved6;
    uint32 sceneBounds;
    uint32 mortonCodes;
    uint32 mortonCodesSorted;
    uint32 primIndicesSorted;
    uint32 primIndicesSortedSwap;
    uint32 propagationFlags;
    uint32 numBatches;
    uint32 batchIndices;
    uint32 indexBufferInfo;
    uint32 updateStack;
    uint32 histogram;
    uint32 tempKeys;
    uint32 tempVals;
    uint32 dynamicBlockIndex;
    uint32 prefixSumAtomicFlags;
    uint32 distributedPartialSums;
    uint32 qbvhGlobalStack;
    uint32 qbvhGlobalStackPtrs;
    uint32 reserved7;
    uint32 debugCounters;
};

struct BuildShaderConstants
{
    uint32 resultBufferAddrLo;
    uint32 resultBufferAddrHi;
    uint32 numPrimitives;
    uint32 tsBudgetPerTriangle;

    uint32 maxNumPrimitives;
    uint32 encodeArrayOfPointers;
    uint32 rebraidFactor;
    uint32 numMortonSizeBits;

    float reservedFloat;
    uint32 numLeafNodes;
    uint32 numDescs;
    uint32 padding0;

    // Align the following struct to 4 dwords due to HLSL constant buffer packing rules
    AccelStructHeader header;
    RayTracingScratchDataOffsets offsets;
};

#ifdef __cplusplus
static_assert(sizeof(AccelStructHeader) % 16 == 0, "AccelStructHeader must be 16-byte aligned");
static_assert(sizeof(RayTracingScratchDataOffsets) % 16 == 0, "RayTracingScratchDataOffsets must be 16-byte aligned");
static_assert(sizeof(BuildShaderConstants) % 16 == 0, "BuildShaderConstants must be 16-byte aligned");

static_assert(offsetof(BuildShaderConstants, header) % 16 == 0, "AccelStructHeader must be 16-byte aligned");
static_assert(offsetof(BuildShaderConstants, offsets) % 16 == 0, "RayTracingScratchDataOffsets must be 16-byte aligned");
} // namespace GpuRt
#endif

#endif
