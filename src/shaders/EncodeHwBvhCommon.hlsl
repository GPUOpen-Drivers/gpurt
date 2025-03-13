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
#ifndef _ENCODE_HW_BVH_COMMON_HLSL
#define _ENCODE_HW_BVH_COMMON_HLSL

#include "BuildCommonScratch.hlsl"
#include "CompactCommon.hlsl"

//=====================================================================================================================
uint GetMaxInternalNodeCount(
    in uint numActivePrims)
{
    uint maxInternalNodeCount;

    if ((Settings.topLevelBuild == 0) &&
        (Settings.enableEarlyPairCompression != 0))
    {
        // With early pairing enabled, numActivePrims represents the active primitive references. However,
        // we need to use the actual primitive count to estimate the hardware internal node count.
        maxInternalNodeCount = GetNumInternalNodeCount(ShaderConstants.maxNumPrimitives);
    }
    else
    {
        maxInternalNodeCount = GetNumInternalNodeCount(numActivePrims);
    }

    return maxInternalNodeCount;
}

//=====================================================================================================================
BoundingBox GetScratchNodeBoundingBoxTS(
    in uint        scratchNodesScratchOffset,
    in bool        isLeafNode,
    in ScratchNode node)
{
    const bool enablePairCompression = (Settings.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION);

    const uint baseScratchNodesOffset = Settings.enableEarlyPairCompression ?
                                        ShaderConstants.offsets.bvhLeafNodeData :
                                        scratchNodesScratchOffset;

    return GetScratchNodeBoundingBox(node,
                                     isLeafNode,
                                     enablePairCompression,
                                     baseScratchNodesOffset);
}

//=====================================================================================================================
float GetScratchNodeSurfaceArea(
    in uint          scratchNodesScratchOffset,
    in uint          nodeIdx,
    in ScratchNode   node,
    in uint          numActivePrims)
{
    if (IsLeafNode(nodeIdx, numActivePrims) == false)
    {
        return node.sah_or_v2_or_instBasePtr.y;
    }
    else
    {
        return ComputeBoxSurfaceArea(GetScratchNodeBoundingBoxTS(scratchNodesScratchOffset, true, node));
    }
}

//=====================================================================================================================
static uint CreateBoxNodePointer(
    in uint nodeOffsetInBytes,
    in bool writeAsFp16BoxNode)
{
    uint nodeType = GetInternalNodeType();
    if ((nodeType == NODE_TYPE_BOX_FLOAT32) && writeAsFp16BoxNode)
    {
        nodeType = NODE_TYPE_BOX_FLOAT16;
    }

    return PackNodePointer(nodeType, nodeOffsetInBytes);
}

//=====================================================================================================================
void PostHwBvhBuild(
    uint          localId,
    uint          numActivePrims)
{
    if (localId == 0)
    {
        if (Settings.topLevelBuild)
        {
            if (numActivePrims == 0)
            {
                // This is an empty TLAS, but we didn't know it yet when we were setting up the header writes in the
                // command buffer. Overwrite the GPU VA to 0 to properly designate the TLAS as empty.
                DstMetadata.Store<GpuVirtualAddress>(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, 0);
            }
            WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET, numActivePrims);
        }
        // Early triangle pairing needs to update the leaf node count post-build
        else if ((Settings.enableEarlyPairCompression) ||
                 (Settings.triangleCompressionMode != PAIR_TRIANGLE_COMPRESSION))
        {
            bool compressionEnabled = false;
#if GPURT_BUILD_RTIP3_1
            compressionEnabled |= (Settings.rtIpLevel == GPURT_RTIP3_1);
#endif
            if (compressionEnabled == false)
            {
                // When compression is disabled, the final leaf node count is the active prim count.
                WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET, numActivePrims);
            }
        }

        const AccelStructHeader header = DstBuffer.Load<AccelStructHeader>(0);

        // Unused
        uint metadataSizeInBytes = 0;
        INIT_VAR(AccelStructOffsets, offsets);

        // Calculate compacted size
        uint compactedSize = CalcCompactedSize(header,
                                               Settings.topLevelBuild,
                                               offsets,
                                               metadataSizeInBytes);

        if (Settings.disableCompaction)
        {
            compactedSize = ShaderConstants.header.sizeInBytes;
        }
        WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_COMPACTED_BYTE_SIZE_OFFSET, compactedSize);

        if (Settings.emitCompactSize != 0)
        {
            EmitBuffer.Store2(0, uint2(compactedSize, 0));
        }

#if GPURT_BUILD_RTIP3_1
        if (Settings.rtIpLevel == GPURT_RTIP3_1)
        {
            WriteUpdateGroupCount(RTIP3_1_UPDATE_THREADS_PER_FAT_LEAF);
        }
#endif
    }
}

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
// Returns stack entry at index
uint2 StackPop(
    uint stackIndex)
{
    return ScratchGlobal.Load2(ShaderConstants.offsets.qbvhGlobalStack + (stackIndex * sizeof(uint2)));
}

//=====================================================================================================================
uint NumLeafsDoneOffset()
{
    return (Settings.topLevelBuild == 0) ? STACK_PTRS_NUM_LEAFS_DONE_OFFSET : STACK_PTRS_DST_PTR_OFFSET;
}

//======================================================================================================================
void InitStackPtrs()
{
    StackPtrs stackPtrs = (StackPtrs)0;

    // Nodes are allocated in 128 byte chunks with first chunk reserved for root node
    stackPtrs.stackPtrSrcNodeId = 1;

    // NumLeafsDone for TLAS is counted in stackPtrNodeDest and is initialised to 0. While for BLAS, stackPtrNodeDest
    // is used as global stack counter.
    if (Settings.topLevelBuild == 0)
    {
        stackPtrs.stackPtrNodeDest = 1;
    }

    stackPtrs.primCompGroupCountY = 1;
    stackPtrs.primCompGroupCountZ = 1;

    const uint32_t rootNodeIndex = GetBvh2RootNodeIndex();

    ScratchGlobal.Store<StackPtrs>(ShaderConstants.offsets.qbvhGlobalStackPtrs, stackPtrs);
    ScratchGlobal.Store<uint2>(ShaderConstants.offsets.qbvhGlobalStack, uint2(rootNodeIndex, 0));
}

//======================================================================================================================
void ClearStackPtrs(
    in uint globalId,
    in uint maxInternalNodeCount,
    in uint numThreads)
{
    // Note the first entry is initialized below. Skip initializing it to invalid.
    for (uint stackIndex = globalId + 1; stackIndex < maxInternalNodeCount; stackIndex += numThreads)
    {
        const uint qbvhStackOffset = ShaderConstants.offsets.qbvhGlobalStack + (stackIndex * sizeof(uint2));
        ScratchGlobal.Store<uint2>(qbvhStackOffset, uint2(INVALID_IDX, INVALID_IDX));
    }
}
#endif

//=====================================================================================================================
//
void CullIllegalInstances(
    in    uint         blasMetadataSize,
    in    ScratchNode  scratchNode,
    inout InstanceDesc instanceDesc)
{
    if (Settings.cullIllegalInstances == true)
    {
        const GpuVirtualAddress blasBaseAddr = (blasMetadataSize +
            MakeGpuVirtualAddress(instanceDesc.accelStructureAddressLo, instanceDesc.accelStructureAddressHiAndFlags));

        const GpuVirtualAddress expectedBlasBaseAddr = GetInstanceAddr(asuint(scratchNode.sah_or_v2_or_instBasePtr.x),
                                                                       asuint(scratchNode.sah_or_v2_or_instBasePtr.y));

        if (blasBaseAddr != expectedBlasBaseAddr)
        {
            // Force instance inclusion mask to 0 so that the illegal instance is culled out during traversal.
            instanceDesc.InstanceID_and_Mask &= bits(24);
        }
    }
}

#endif
