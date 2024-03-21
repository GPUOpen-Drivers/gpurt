/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#define FETCH_SCRATCH_NODE_DATA(T, baseScratchNodesOffset, nodeIndex, dataOffset) \
    (ScratchGlobal.Load<T>(CalcScratchNodeOffset((baseScratchNodesOffset), (nodeIndex)) + (dataOffset)))

//=====================================================================================================================
#define WRITE_SCRATCH_NODE_DATA_DEF(T) \
void WriteScratchNodeData( \
    uint baseScratchNodesOffset, \
    uint nodeIndex, \
    uint dataOffset, \
    T data) \
{ \
    const uint nodeOffset = CalcScratchNodeOffset(baseScratchNodesOffset, nodeIndex); \
    ScratchGlobal.Store<T>(nodeOffset + dataOffset, data); \
}

//=====================================================================================================================
#define WRITE_SCRATCH_NODE_DATA_AT_OFFSET_DEF(T) \
void WriteScratchNodeDataAtOffset( \
    uint nodeOffset, \
    uint dataOffset, \
    T data) \
{ \
    ScratchGlobal.Store<T>(nodeOffset + dataOffset, data); \
}

//=====================================================================================================================
// overloading functions
WRITE_SCRATCH_NODE_DATA_DEF(uint)
WRITE_SCRATCH_NODE_DATA_DEF(float)

WRITE_SCRATCH_NODE_DATA_AT_OFFSET_DEF(uint)
WRITE_SCRATCH_NODE_DATA_AT_OFFSET_DEF(uint2)
WRITE_SCRATCH_NODE_DATA_AT_OFFSET_DEF(uint3)
WRITE_SCRATCH_NODE_DATA_AT_OFFSET_DEF(uint4)
WRITE_SCRATCH_NODE_DATA_AT_OFFSET_DEF(float)
WRITE_SCRATCH_NODE_DATA_AT_OFFSET_DEF(float2)
WRITE_SCRATCH_NODE_DATA_AT_OFFSET_DEF(float3)
WRITE_SCRATCH_NODE_DATA_AT_OFFSET_DEF(float4)

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
uint FetchRootNodeIndex(uint enableFastLBVH, uint offset)
{
    return enableFastLBVH ? ScratchGlobal.Load(offset) : 0;
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

