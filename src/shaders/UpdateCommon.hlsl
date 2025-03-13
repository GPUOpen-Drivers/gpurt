/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
//

#ifndef _UPDATECOMMON_HLSL
#define _UPDATECOMMON_HLSL

#include "BuildCommon.hlsl"

#if GPURT_BUILD_RTIP3
//=====================================================================================================================
// Computes index in update scratch memory for a given node pointer.
uint ComputeUpdateScratchBoundingBoxIndex(
    in uint nodePointer,
    in uint childIdx)
{
    // Note, internal nodes start at a 128-byte offset (past the header). Subtract the size here to compute absolute
    // internal node index
    const uint nodeOffset = ExtractNodePointerOffset(nodePointer) - sizeof(AccelStructHeader);

    // The index is computed as follows for the node formats that support it (i.e. HPB64 and BVH8 box)
    //
    // HPB64:    index = (offset / 64)  * 4

    uint index = nodeOffset >> 4;

#if GPURT_BUILD_RTIP3_1
    // 128bBVH8: index = (offset / 128) * 8
#endif

    if ((Settings.rtIpLevel == GPURT_RTIP3_0) &&
        Settings.bvh8Enable)
    {
        // HPB64x2:  index = (offset / 64) * 8
        index = (nodeOffset >> 6) << 3;
    }

    return childIdx + index;
}

//=====================================================================================================================
void WriteUpdateScratchBoundingBox(
    uint           boxNodePointer,
    uint           childIdx,
    in BoundingBox boundingBox)
{
    const uint scratchBoundsIdx = ComputeUpdateScratchBoundingBoxIndex(boxNodePointer, childIdx);
    const uint scratchOffset = ShaderConstants.offsets.bvhNodeData + (scratchBoundsIdx * sizeof(BoundingBox));
    ScratchBuffer.Store<BoundingBox>(scratchOffset, boundingBox);
}

//=====================================================================================================================
BoundingBox ReadUpdateScratchBoundingBoxAtIndex(
    uint scratchBoundsIdx)
{
    const uint scratchOffset = ShaderConstants.offsets.bvhNodeData + (scratchBoundsIdx * sizeof(BoundingBox));
    return ScratchBuffer.Load<BoundingBox>(scratchOffset);
}

//=====================================================================================================================
BoundingBox ReadUpdateScratchBoundingBox(
    uint boxNodePointer,
    uint childIdx)
{
    const uint scratchBoundsIdx = ComputeUpdateScratchBoundingBoxIndex(boxNodePointer, childIdx);
    return ReadUpdateScratchBoundingBoxAtIndex(scratchBoundsIdx);
}
#endif

//=====================================================================================================================
uint GetUpdateStackOffset(
    uint stackIdx)
{
    return ShaderConstants.offsets.updateStack + (stackIdx * sizeof(uint));
}

//=====================================================================================================================
void PushNodeToUpdateStack(
    uint parentNodePointer)
{
    uint stackPtr;
    ScratchBuffer.InterlockedAdd(UPDATE_SCRATCH_STACK_NUM_ENTRIES_OFFSET, 1, stackPtr);

    uint offset = GetUpdateStackOffset(stackPtr);
    ScratchBuffer.Store(offset, parentNodePointer);
}

//=====================================================================================================================
// Note, SrcBuffer and DstMetadata point to the beginning of the acceleration structure buffer
void CopyChildPointersAndFlags(
    uint nodePointer,
    uint metadataSize)
{
    const uint nodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);
#if GPURT_BUILD_RTIP3_1
    if (EnableCompressedFormat())
    {
        // For RTIP3.1 this is already done in UpdateQuantizedBoxNode() skip copying
    }
    else
#endif
#if GPURT_BUILD_RTIP3
    if (Settings.highPrecisionBoxNodeEnable)
    {
        // For high precision box node, the child pointers and flags are packed in the first 8 bytes
        const uint2 packedChildPointersAndFlags = SrcBuffer.Load<uint2>(nodeOffset);
        DstMetadata.Store<uint2>(nodeOffset, packedChildPointersAndFlags);

        if (Settings.bvh8Enable)
        {
            const uint nodeOffset2 = nodeOffset + sizeof(HighPrecisionBoxNode);

            const uint2 packedChildPointersAndFlags = SrcBuffer.Load<uint2>(nodeOffset2);
            DstMetadata.Store<uint2>(nodeOffset2, packedChildPointersAndFlags);
        }
    }
    else
#endif
    {
        const uint4 childPointers = SrcBuffer.Load<uint4>(nodeOffset);
        DstMetadata.Store<uint4>(nodeOffset, childPointers);

        if (IsBoxNode32(nodePointer))
        {
            const uint sourceFlags = SrcBuffer.Load(nodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET);
            DstMetadata.Store(nodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET, sourceFlags);
        }

#if GPURT_BUILD_RTIP3
        if (Settings.bvh8Enable)
        {
            const uint nodeOffset2 = nodeOffset + sizeof(Float32BoxNode);

            const uint4 childPointers2 = SrcBuffer.Load<uint4>(nodeOffset2);
            DstMetadata.Store<uint4>(nodeOffset2, childPointers2);

            const uint sourceFlags2 = SrcBuffer.Load(nodeOffset2 + FLOAT32_BOX_NODE_FLAGS_OFFSET);
            DstMetadata.Store(nodeOffset2 + FLOAT32_BOX_NODE_FLAGS_OFFSET, sourceFlags2);
        }
#endif
    }
}

//=====================================================================================================================
uint4 LoadBoxNodeChildPointers(
    in uint nodeOffset)
{
#if GPURT_BUILD_RTIP3
    uint4 childPointers = uint4(INVALID_NODE, INVALID_NODE, INVALID_NODE, INVALID_NODE);
    if (Settings.highPrecisionBoxNodeEnable)
    {
        childPointers = DecodeHighPrecisionBoxNodeChildPointers(nodeOffset);
    }
    else
    {
        childPointers = SrcBuffer.Load<uint4>(nodeOffset);
    }

    return childPointers;
#else
    return SrcBuffer.Load<uint4>(nodeOffset);
#endif
}

//=====================================================================================================================
uint ComputeChildIndexAndValidBoxCount(
    in uint         metadataSize,
    in uint         parentNodePointer,
    in uint         childNodePointer,
    out_param(uint) boxNodeCount)
{
#if GPURT_BUILD_RTIP3_1
    if (EnableCompressedFormat())
    {
        return ComputeQuantizedBoxChildIndexAndValidBoxCount(metadataSize,
                                                             parentNodePointer,
                                                             childNodePointer,
                                                             boxNodeCount);
    }
#endif

    const uint parentNodeOffset = metadataSize + ExtractNodePointerOffset(parentNodePointer);
    const uint4 childPointers = LoadBoxNodeChildPointers(parentNodeOffset);

    // Find child index in parent (assumes child pointer 0 is always valid)
    uint childIdx = 0;
    if (childNodePointer == childPointers.y)
    {
        childIdx = 1;
    }

    if (childNodePointer == childPointers.z)
    {
        childIdx = 2;
    }

    if (childNodePointer == childPointers.w)
    {
        childIdx = 3;
    }

    // Note, IsBoxNode() will return false for invalid nodes.
    boxNodeCount = 0;
    boxNodeCount += IsBoxNode(childPointers.x) ? 1 : 0;
    boxNodeCount += IsBoxNode(childPointers.y) ? 1 : 0;
    boxNodeCount += IsBoxNode(childPointers.z) ? 1 : 0;
    boxNodeCount += IsBoxNode(childPointers.w) ? 1 : 0;

#if GPURT_BUILD_RTIP3
    if (Settings.bvh8Enable)
    {
        const uint internalNodeSize =
            Settings.highPrecisionBoxNodeEnable ? sizeof(HighPrecisionBoxNode) : sizeof(Float32BoxNode);

        const uint4 childPointers = LoadBoxNodeChildPointers(parentNodeOffset + internalNodeSize);

        if (childNodePointer == childPointers.x)
        {
            childIdx = 4;
        }

        if (childNodePointer == childPointers.y)
        {
            childIdx = 5;
        }

        if (childNodePointer == childPointers.z)
        {
            childIdx = 6;
        }

        if (childNodePointer == childPointers.w)
        {
            childIdx = 7;
        }

        boxNodeCount += IsBoxNode(childPointers.x) ? 1 : 0;
        boxNodeCount += IsBoxNode(childPointers.y) ? 1 : 0;
        boxNodeCount += IsBoxNode(childPointers.z) ? 1 : 0;
        boxNodeCount += IsBoxNode(childPointers.w) ? 1 : 0;
    }
#endif

    return childIdx;
}

//=====================================================================================================================
void PushNodeForUpdate(
    uint           metadataSize,
    uint           childNodePointer,
    uint           parentNodePointer,
    uint           instanceMask,
    in BoundingBox boundingBox,
    bool           writeNodesToUpdateStack)
{
    // Compute box node count and child index in parent node
    uint boxNodeCount = 0;
    const uint childIdx = ComputeChildIndexAndValidBoxCount(metadataSize,
                                                            parentNodePointer,
                                                            childNodePointer,
                                                            boxNodeCount);

    // If even a single child node is a box node, this is a node higher up the tree. Skip queueing parent node as another
    // leaf at the bottom of the tree will queue its parent which will handle our parent node.

    // B B B B --> 4 --> Not possible
    // B x B x --> 2 --> Not possible
    // B L B L --> 4 --> Skip queueing
    // L x B x --> 1 --> Skip queueing
    // L x x x --> 0 --> Queue parent node
    // L x L x --> 0 --> Queue parent node
    // L L L L --> 0 --> Queue parent node

    // Always perform update for out-of-place updates
    bool performUpdate = (Settings.isUpdateInPlace == false);

    uint boxOffset;

    const uint parentNodeOffset = metadataSize + ExtractNodePointerOffset(parentNodePointer);

#if GPURT_BUILD_RTIP3
#if GPURT_BUILD_RTIP3_1
    if (Settings.highPrecisionBoxNodeEnable || EnableCompressedFormat())
#else
    if (Settings.highPrecisionBoxNodeEnable)
#endif
    {
        // Optionally, decode bounding box from parent node and check for redundant box update. Note, this requires
        // a full decode of the destination node. For now, always perform updates

        // If we want to reduce scratch usage, we can explore using the result buffer to store the full precision leaf
        // bounds at the expense of larger result buffer size.
        performUpdate = true;

        // Write leaf bounding box to update scratch memory
        WriteUpdateScratchBoundingBox(parentNodePointer, childIdx, boundingBox);

#if GPURT_BUILD_RTIP3_1
        if (EnableCompressedFormat())
        {
            UpdateQuantizedBoxInstanceMask(parentNodeOffset, childIdx, instanceMask);
        }
#endif
    }
    else if (Settings.bvh8Enable)
    {
        // IsBoxNode32x2()
        boxOffset = (childIdx % 4) * FLOAT32_BBOX_STRIDE;
        const uint fp32BoxNodeOffset = (childIdx > 3) ? parentNodeOffset + sizeof(Float32BoxNode) : parentNodeOffset;

        BoundingBox originalBox;
        originalBox.min = DstMetadata.Load<float3>(fp32BoxNodeOffset + FLOAT32_BOX_NODE_BB0_MIN_OFFSET + boxOffset);
        originalBox.max = DstMetadata.Load<float3>(fp32BoxNodeOffset + FLOAT32_BOX_NODE_BB0_MAX_OFFSET + boxOffset);

        if (any(originalBox.min != boundingBox.min) ||
            any(originalBox.max != boundingBox.max))
        {
            DstMetadata.Store<float3>(fp32BoxNodeOffset + FLOAT32_BOX_NODE_BB0_MIN_OFFSET + boxOffset, boundingBox.min);
            DstMetadata.Store<float3>(fp32BoxNodeOffset + FLOAT32_BOX_NODE_BB0_MAX_OFFSET + boxOffset, boundingBox.max);
            performUpdate = true;
        }
    }
    else
#endif
    {
        if (IsBoxNode32(parentNodePointer))
        {
            BoundingBox originalBox;

            boxOffset = childIdx * FLOAT32_BBOX_STRIDE;
            originalBox.min = DstMetadata.Load<float3>(parentNodeOffset + FLOAT32_BOX_NODE_BB0_MIN_OFFSET + boxOffset);
            originalBox.max = DstMetadata.Load<float3>(parentNodeOffset + FLOAT32_BOX_NODE_BB0_MAX_OFFSET + boxOffset);

            if (any(originalBox.min != boundingBox.min) ||
                any(originalBox.max != boundingBox.max))
            {
                DstMetadata.Store<float3>(parentNodeOffset + FLOAT32_BOX_NODE_BB0_MIN_OFFSET + boxOffset, boundingBox.min);
                DstMetadata.Store<float3>(parentNodeOffset + FLOAT32_BOX_NODE_BB0_MAX_OFFSET + boxOffset, boundingBox.max);
                performUpdate = true;
            }
        }
        else
        {
            boxOffset = childIdx * FLOAT16_BBOX_STRIDE;
            const uint3 originalBox16 = DstMetadata.Load<uint3>(parentNodeOffset + FLOAT16_BOX_NODE_BB0_OFFSET + boxOffset);

            const uint3 boundingBox16 = CompressBBoxToUint3(boundingBox);

            if (any(originalBox16 != boundingBox16))
            {
                DstMetadata.Store<float3>(parentNodeOffset + FLOAT16_BOX_NODE_BB0_OFFSET + boxOffset, boundingBox16);
                performUpdate = true;
            }
        }
    }

    const bool canWriteNodeToUpdateStack = (childIdx == 0) && (boxNodeCount == 0);
    if (canWriteNodeToUpdateStack && performUpdate)
    {
        if (Settings.isUpdateInPlace == false)
        {
            CopyChildPointersAndFlags(parentNodePointer, metadataSize);
        }
    }

    // If this is the first child in the parent node with all leaf children, queue parent pointer to
    // stack in scratch memory.
    // @note Right now we queue the parent node for update regardless of whether the leaf nodes' bounding boxes change
    // or not. We could optimize this by queuing the parent node only if any of the leaf nodes' bounding boxes change.
    if (writeNodesToUpdateStack && canWriteNodeToUpdateStack)
    {
        PushNodeToUpdateStack(parentNodePointer);
    }
}

#endif
