/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef BUILD_COMMON_SCRATCH_GLOBAL_HLSLI
#define BUILD_COMMON_SCRATCH_GLOBAL_HLSLI

#include "BuildRootSignature.hlsli"
#include "../common/ShaderDefs.hlsli"
#include "../common/ScratchNode.hlsli"
#include "../common/BoundingBox.hlsli"

//=====================================================================================================================
ScratchNode FetchScratchNode(
    uint baseScratchNodesOffset,
    uint nodeIndex)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);
    return ScratchGlobal.Load<ScratchNode>(nodeOffset);
}

//=====================================================================================================================
ScratchNode FetchScratchNodeAtOffset(
    uint nodeOffset)
{
    return ScratchGlobal.Load<ScratchNode>(nodeOffset);
}

//=====================================================================================================================
void WriteScratchNode(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    ScratchNode node)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);
    ScratchGlobal.Store<ScratchNode>(nodeOffset, node);
}

//=====================================================================================================================
void WriteScratchNodeAtOffset(
    uint nodeOffset,
    ScratchNode node)
{
    ScratchGlobal.Store<ScratchNode>(nodeOffset, node);
}

//=====================================================================================================================

template <typename T>
T FetchScratchNodeData(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint dataOffset)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);
    return ScratchGlobal.Load<T>(nodeOffset + dataOffset);
}

//=====================================================================================================================
template <typename T>
void WriteScratchNodeData(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint dataOffset,
    T data)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);
    ScratchGlobal.Store<T>(nodeOffset + dataOffset, data);
}

//=====================================================================================================================

template <typename T>
void WriteScratchNodeDataAtOffset(
    uint nodeOffset,
    uint dataOffset,
    T data)
{
    ScratchGlobal.Store<T>(nodeOffset + dataOffset, data);
}

//=====================================================================================================================
// Update node flags from child in parent scratch node
void UpdateParentScratchNodeFlags(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint flagsAndInstanceMask)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    ScratchGlobal.InterlockedAnd(nodeOffset + SCRATCH_NODE_FLAGS_OFFSET, flagsAndInstanceMask);
}

//=====================================================================================================================
uint AllocateScratchNodePointer(
    uint baseScratchNodesOffset,
    uint nodeIndex,
    uint nodePointer)
{
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex);

    // check if the pre allocated instance is used; if not, use it, if used, alloc a new instance
    uint original;
    ScratchGlobal.InterlockedCompareExchange(nodeOffset + SCRATCH_NODE_NODE_POINTER_OFFSET, 0, nodePointer, original);
    return original;
}

//=====================================================================================================================
void WriteSplitBoxAtIndex(uint offset, uint index, BoundingBox box)
{
    ScratchGlobal.Store<BoundingBox>(offset + index * sizeof(BoundingBox), box);
}

//=====================================================================================================================
BoundingBox FetchSplitBoxAtIndex(uint offset, uint index)
{
    return ScratchGlobal.Load<BoundingBox>(offset + index * sizeof(BoundingBox));
}

//=====================================================================================================================
void WriteRootNodeIndex(uint offset, uint index)
{
    ScratchGlobal.Store(offset, index);
}

//=====================================================================================================================
uint FetchRootNodeIndex(uint offset)
{
    return ScratchGlobal.Load(offset);
}

//=====================================================================================================================
void IncreaseDebugCounters(uint countersOffset, uint counterStageOffset)
{
    ScratchGlobal.InterlockedAdd(countersOffset + counterStageOffset, 1);
}

//=====================================================================================================================
uint AllocateBatchIndex(uint numBatchesOffset)
{
    uint numBatches;
    ScratchGlobal.InterlockedAdd(numBatchesOffset, 1, numBatches);
    return numBatches;
}

//=====================================================================================================================
uint FetchNumBatches(uint numBatchesOffset)
{
    return ScratchGlobal.Load(numBatchesOffset);
}

//=====================================================================================================================
void ClearNumBatches(uint numBatchesOffset)
{
    if (numBatchesOffset != INVALID_IDX)
    {
        ScratchGlobal.Store(numBatchesOffset, 0);
    }
}

#if GPURT_BUILD_RTIP3_1
//======================================================================================================================
uint FetchCompressPrimBatchCount(uint stackPtrsScratchOffset)
{
    return ScratchGlobal.Load(stackPtrsScratchOffset + STACK_PTRS_PRIM_COMP_BATCH_COUNT);
}

//======================================================================================================================
uint FetchCompressPrimSingleCount(uint stackPtrsScratchOffset)
{
    return ScratchGlobal.Load(stackPtrsScratchOffset + STACK_PTRS_PRIM_COMP_SINGLE_COUNT);
}
#endif

#endif
