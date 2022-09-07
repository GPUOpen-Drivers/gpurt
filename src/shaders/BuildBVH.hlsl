/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2022 Advanced Micro Devices, Inc. All Rights Reserved.
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
#if NO_SHADER_ENTRYPOINT == 0
#include "Common.hlsl"
#include "BuildCommon.hlsl"

#define RootSig "RootConstants(num32BitConstants=6, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL)"

//=====================================================================================================================
// 32 bit constants
struct InputArgs
{
    uint AllowUpdate;
    uint BvhNodeDataScratchOffset;
    uint BvhLeafNodeDataScratchOffset;
    uint MortonCodesSortedScratchOffset;
    uint PrimIndicesSortedScratchOffset;
    uint UseMortonCode30;
};

[[vk::push_constant]] ConstantBuffer<InputArgs> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] RWByteAddressBuffer ResultBuffer   : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer ResultMetadata : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer ScratchBuffer  : register(u2);
#endif

//=====================================================================================================================
// Calculates longest common prefix length of bit representations.
// If representations are equal we consider sucessive indices.
int delta(in int i1, in int i2, int numPrim, uint sortedMortonCodesOffset, bool useMortonCode30)
{
    // Select left end
    const int left = min(i1, i2);
    // Select right end
    const int right = max(i1, i2);
    // This is to ensure the node breaks if the index is out of bounds
    if ((left < 0) || (right >= numPrim))
    {
        return -1;
    }

    // Fetch Morton codes for both ends
    // Special handling of duplicated codes: use their indices as a fallback
    if (useMortonCode30)
    {
        const int leftCode  = ScratchBuffer.Load(sortedMortonCodesOffset + (left * sizeof(int)));
        const int rightCode = ScratchBuffer.Load(sortedMortonCodesOffset + (right * sizeof(int)));
        return (leftCode != rightCode) ? clz(leftCode ^ rightCode) : (32 + clz(left ^ right));
    }
    else
    {
        const uint64_t leftCode  = ScratchBuffer.Load<uint64_t>(sortedMortonCodesOffset + (left * sizeof(uint64_t)));
        const uint64_t rightCode = ScratchBuffer.Load<uint64_t>(sortedMortonCodesOffset + (right * sizeof(uint64_t)));
        return (leftCode != rightCode) ? clz64(leftCode ^ rightCode) : (64 + clz64(left ^ right));
    }
}

//=====================================================================================================================
// Find span occupied by internal node with index idx
int2 FindSpan(int idx, int numPrim, uint sortedMortonCodesOffset, bool useMortonCode30)
{
    // Find the direction of the range
    const int d = sign((float) (delta(idx, idx + 1, numPrim, sortedMortonCodesOffset, useMortonCode30) -
                                delta(idx, idx - 1, numPrim, sortedMortonCodesOffset, useMortonCode30)));

    // Find minimum number of bits for the break on the other side
    const int deltamin = delta(idx, idx - d, numPrim, sortedMortonCodesOffset, useMortonCode30);

    // Search conservative far end
    int lmax = 2;
    while (delta(idx, idx + lmax * d, numPrim, sortedMortonCodesOffset, useMortonCode30) > deltamin)
    {
        lmax *= 2;
    }

    // Search back to find exact bound
    // with binary search
    int l = 0;
    int t = lmax;
    do
    {
        t /= 2;
        if (delta(idx, idx + (l + t)*d, numPrim, sortedMortonCodesOffset, useMortonCode30) > deltamin)
        {
            l = l + t;
        }
    } while (t > 1);

    // Pack span
    return int2(min(idx, idx + l * d), max(idx, idx + l * d));
}

//=====================================================================================================================
// Find split idx within the span
int FindSplit(int2 span, int numPrim, uint sortedMortonCodesOffset, bool useMortonCode30)
{
    // Fetch codes for both ends
    int left = span.x;
    int right = span.y;

    // Calculate the number of identical bits from higher end
    const int numidentical = delta(left, right, numPrim, sortedMortonCodesOffset, useMortonCode30);

    do
    {
        // Proposed split
        const int newsplit = (right + left) / 2;

        // If it has more equal leading bits than left and right accept it
        if (delta(left, newsplit, numPrim, sortedMortonCodesOffset, useMortonCode30) > numidentical)
        {
            left = newsplit;
        }
        else
        {
            right = newsplit;
        }
    } while (right > left + 1);

    return left;
}

//=====================================================================================================================
void CopyUnsortedScratchLeafNode(
    uint primIndex,
    uint numActivePrims,
    uint sortedPrimIndicesOffset,
    uint unsortedNodesBaseOffset,
    uint bvhNodesBaseOffset,
    uint centroidBoxesScratchOffset,
    uint gridPosOffset,
    uint reservedUint)
{
    // Store leaf node bounding boxes
    const uint nodeIndex = FetchSortedPrimIndex(ScratchBuffer, sortedPrimIndicesOffset, primIndex);

    const ScratchNode unsortedNode = FetchScratchNode(ScratchBuffer, unsortedNodesBaseOffset, nodeIndex);

    const uint scratchNodeOffset = CalcScratchNodeOffset(bvhNodesBaseOffset, LEAFIDX(primIndex));

    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_V0_OFFSET,             unsortedNode.bbox_min_or_v0);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_PRIMITIVE_ID_OFFSET,   unsortedNode.left_or_primIndex_or_instIndex);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_V1_OFFSET,             unsortedNode.bbox_max_or_v1);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_GEOMETRY_INDEX_OFFSET, unsortedNode.right_or_geometryIndex);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_V2_OFFSET,             unsortedNode.range_or_v2_or_instBasePtr);

    // DO NOT COPY PARENT!!!

    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_TYPE_OFFSET,            unsortedNode.type);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_FLAGS_OFFSET,           unsortedNode.flags);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET, unsortedNode.splitBox_or_nodePointer);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET, unsortedNode.numPrimitivesAndDoCollapse);
}

//=====================================================================================================================
// Construct scratch tree topology using the LBVH algorithm
void SplitInternalNodeLbvh(
    uint nodeIndex,
    uint numActivePrims,
    uint scratchNodesOffset,
    uint sortedMortonCodesOffset,
    uint useMortonCode30)
{
    // Find span occupied by the current node
    const uint2 range = FindSpan(nodeIndex, numActivePrims, sortedMortonCodesOffset, useMortonCode30);

    // Find split position inside the range
    const uint split = FindSplit(range, numActivePrims, sortedMortonCodesOffset, useMortonCode30);

    // Create child nodes if needed
    const uint c1idx = (split == range.x)     ? LEAFIDX(split)     : NODEIDX(split);
    const uint c2idx = (split + 1 == range.y) ? LEAFIDX(split + 1) : NODEIDX(split + 1);

    const uint c1ScratchNodeOffset = CalcScratchNodeOffset(scratchNodesOffset, c1idx);
    const uint c2ScratchNodeOffset = CalcScratchNodeOffset(scratchNodesOffset, c2idx);

    ScratchBuffer.Store(c1ScratchNodeOffset + SCRATCH_NODE_PARENT_OFFSET, NODEIDX(nodeIndex));
    ScratchBuffer.Store(c2ScratchNodeOffset + SCRATCH_NODE_PARENT_OFFSET, NODEIDX(nodeIndex));

    const uint scratchNodeOffset = CalcScratchNodeOffset(scratchNodesOffset, NODEIDX(nodeIndex));

    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_LEFT_OFFSET,  c1idx);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_RIGHT_OFFSET, c2idx);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_RANGE_OFFSET, range); //TODO: don't think we need this
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_TYPE_OFFSET,  NODE_TYPE_BOX_FLOAT32);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_FLAGS_OFFSET, 0);
}

//======================================================================================================================
void FastBuildBVH(uint globalId, uint numPrims, uint leafNodesOffset, uint bvhNodesOffset)
{
    if (globalId < numPrims)
    {
        const bool active        = IsNodeActive(FetchScratchNode(ScratchBuffer, leafNodesOffset, globalId));
        const int numActivePrims = WaveActiveCountBits(active);

        // Copy only the active nodes to the BVH Nodes list.
        if (active)
        {
            // Prefix Sum on the active nodes computes the target BVH node index.
            const uint targetNodeOffset = CalcScratchNodeOffset(bvhNodesOffset, LEAFIDX(WavePrefixCountBits(active)));
            const ScratchNode node      = FetchScratchNode(ScratchBuffer, leafNodesOffset, globalId);

            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_V0_OFFSET,             node.bbox_min_or_v0);
            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_PRIMITIVE_ID_OFFSET,   node.left_or_primIndex_or_instIndex);
            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_V1_OFFSET,             node.bbox_max_or_v1);
            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_GEOMETRY_INDEX_OFFSET, node.right_or_geometryIndex);
            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_V2_OFFSET,             node.range_or_v2_or_instBasePtr);

            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_TYPE_OFFSET,            node.type);
            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_FLAGS_OFFSET,           node.flags);
            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET, node.splitBox_or_nodePointer);
            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET, node.numPrimitivesAndDoCollapse);
        }

        // Compute internal nodes and construct the BVH topology
        if (globalId < numActivePrims - 1)
        {
            // Compute child nodes' indexes based on the thread ID
            const uint childLeft = (globalId << 1) + 1;
            const uint childRight = (globalId << 1) + 2;

            const uint nodeOffset = CalcScratchNodeOffset(bvhNodesOffset, globalId);

            ScratchBuffer.Store(nodeOffset + SCRATCH_NODE_LEFT_OFFSET,  childLeft);
            ScratchBuffer.Store(nodeOffset + SCRATCH_NODE_RIGHT_OFFSET, childRight);
            ScratchBuffer.Store(nodeOffset + SCRATCH_NODE_TYPE_OFFSET,  NODE_TYPE_BOX_FLOAT32);
            ScratchBuffer.Store(nodeOffset + SCRATCH_NODE_FLAGS_OFFSET, 0);

            const uint childLeftNodeOffset  = CalcScratchNodeOffset(bvhNodesOffset, childLeft);
            const uint childRightNodeOffset = CalcScratchNodeOffset(bvhNodesOffset, childRight);

            ScratchBuffer.Store(childLeftNodeOffset + SCRATCH_NODE_PARENT_OFFSET,  globalId);
            ScratchBuffer.Store(childRightNodeOffset + SCRATCH_NODE_PARENT_OFFSET, globalId);
        }

        if (globalId == 0)
        {
            ResultBuffer.Store(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET, numActivePrims);
        }
    }
}

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
// Main Function : BuildBVH
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void BuildBVH(
    uint globalId : SV_DispatchThreadID)
{
    const int numActivePrims = ResultBuffer.Load(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);

    if (numActivePrims > 0)
    {
        // Initialise leaf nodes with sorted node data. Copy from LeafAABB to InternalAABB leaf section.
        if (globalId < numActivePrims)
        {
            CopyUnsortedScratchLeafNode(
                globalId,
                numActivePrims,
                ShaderConstants.PrimIndicesSortedScratchOffset,
                ShaderConstants.BvhLeafNodeDataScratchOffset,
                ShaderConstants.BvhNodeDataScratchOffset,
                0,
                0,
                0);
        }

        // Set internal nodes
#if !USE_BVH_AC && !USE_BVH_PLOC
        if (globalId < (numActivePrims - 1))
        {
            SplitInternalNodeLbvh(
                globalId,
                numActivePrims,
                ShaderConstants.BvhNodeDataScratchOffset,
                ShaderConstants.MortonCodesSortedScratchOffset,
                ShaderConstants.UseMortonCode30);
        }
#endif
    }
}
#endif
