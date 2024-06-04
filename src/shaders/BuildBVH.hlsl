/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#define RootSig "CBV(b0), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u6, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u7, visibility=SHADER_VISIBILITY_ALL),"\
                "CBV(b255)"

#include "../shared/rayTracingDefs.h"
[[vk::binding(0, 1)]] ConstantBuffer<BuildShaderConstants> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] RWByteAddressBuffer         SrcBuffer           : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer         DstBuffer           : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer         DstMetadata         : register(u2);
[[vk::binding(3, 0)]] RWByteAddressBuffer         ScratchBuffer       : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer         ScratchGlobal       : register(u4);
[[vk::binding(5, 0)]] RWByteAddressBuffer         InstanceDescBuffer  : register(u5);
[[vk::binding(6, 0)]] RWByteAddressBuffer         EmitBuffer          : register(u6);
[[vk::binding(7, 0)]] RWByteAddressBuffer         IndirectArgBuffer   : register(u7);

#include "Common.hlsl"
#include "BuildCommonScratch.hlsl"
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
// Construct scratch tree topology using the LBVH algorithm
void SplitInternalNodeLbvh(
    uint nodeIndex,
    uint numActivePrims,
    uint scratchNodesOffset,
    uint sortedPrimIndicesOffset,
    uint sortedMortonCodesOffset,
    uint useMortonCode30)
{
    // Find span occupied by the current node
    const uint2 range = FindSpan(nodeIndex, numActivePrims, sortedMortonCodesOffset, useMortonCode30);

    // Find split position inside the range
    const uint split = FindSplit(range, numActivePrims, sortedMortonCodesOffset, useMortonCode30);

    // Create child nodes if needed
    const uint splitIdx1 = LEAFIDX(FetchSortedPrimIndex(sortedPrimIndicesOffset, split));
    const uint splitIdx2 = LEAFIDX(FetchSortedPrimIndex(sortedPrimIndicesOffset, split + 1));
    const uint c1idx = (split == range.x)     ? splitIdx1 : NODEIDX(split);
    const uint c2idx = (split + 1 == range.y) ? splitIdx2 : NODEIDX(split + 1);

    // update the parent pointer of each child
    WriteScratchNodeData(scratchNodesOffset, c1idx, SCRATCH_NODE_PARENT_OFFSET, NODEIDX(nodeIndex));
    WriteScratchNodeData(scratchNodesOffset, c2idx, SCRATCH_NODE_PARENT_OFFSET, NODEIDX(nodeIndex));

    // calculate the byte offset of the current node
    const uint scratchNodeOffset = CalcScratchNodeOffset(scratchNodesOffset, NODEIDX(nodeIndex));

    WriteScratchNodeDataAtOffset(scratchNodeOffset, SCRATCH_NODE_LEFT_OFFSET,  c1idx);
    WriteScratchNodeDataAtOffset(scratchNodeOffset, SCRATCH_NODE_RIGHT_OFFSET, c2idx);
    WriteScratchNodeDataAtOffset(scratchNodeOffset, SCRATCH_NODE_FLAGS_OFFSET, 0);

    WriteScratchNodeDataAtOffset(scratchNodeOffset, SCRATCH_NODE_SORTED_PRIM_INDEX_OFFSET, range.x);
}

//======================================================================================================================
void FastBuildBVH(
    uint globalId,
    uint numPrims,
    uint leafNodesOffset,
    uint bvhNodesOffset)
{
    if (globalId < numPrims)
    {
        const bool active        = IsNodeActive(FetchScratchNode(leafNodesOffset, globalId));
        const int numActivePrims = WaveActiveCountBits(active);

        // Copy only the active nodes to the BVH Nodes list.
        if (active)
        {
            // Prefix Sum on the active nodes computes the target BVH node index.
            const uint targetNodeOffset = CalcScratchNodeOffset(bvhNodesOffset, LEAFIDX(WavePrefixCountBits(active)));
            const ScratchNode node      = FetchScratchNode(leafNodesOffset, globalId);

            WriteScratchNodeDataAtOffset(targetNodeOffset, SCRATCH_NODE_V0_OFFSET,             node.bbox_min_or_v0);
            WriteScratchNodeDataAtOffset(targetNodeOffset, SCRATCH_NODE_PRIMITIVE_ID_OFFSET,   node.left_or_primIndex_or_instIndex);
            WriteScratchNodeDataAtOffset(targetNodeOffset, SCRATCH_NODE_V1_OFFSET,             node.bbox_max_or_v1);
            WriteScratchNodeDataAtOffset(targetNodeOffset, SCRATCH_NODE_GEOMETRY_INDEX_OFFSET, node.right_or_geometryIndex);
            WriteScratchNodeDataAtOffset(targetNodeOffset, SCRATCH_NODE_V2_OFFSET,             node.sah_or_v2_or_instBasePtr);

            WriteScratchNodeDataAtOffset(targetNodeOffset, SCRATCH_NODE_FLAGS_OFFSET, node.packedFlags);
            WriteScratchNodeDataAtOffset(targetNodeOffset, SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET, node.splitBox_or_nodePointer);
            WriteScratchNodeDataAtOffset(targetNodeOffset, SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET, node.numPrimitivesAndDoCollapse);
        }

        // Compute internal nodes and construct the BVH topology
        if (globalId < numActivePrims - 1)
        {
            // Compute child nodes' indexes based on the thread ID
            const uint childLeft = (globalId << 1) + 1;
            const uint childRight = (globalId << 1) + 2;

            const uint nodeOffset = CalcScratchNodeOffset(bvhNodesOffset, globalId);

            WriteScratchNodeDataAtOffset(nodeOffset, SCRATCH_NODE_LEFT_OFFSET,  childLeft);
            WriteScratchNodeDataAtOffset(nodeOffset, SCRATCH_NODE_RIGHT_OFFSET, childRight);
            WriteScratchNodeDataAtOffset(nodeOffset, SCRATCH_NODE_FLAGS_OFFSET, 0);
            WriteScratchNodeData(bvhNodesOffset, childLeft, SCRATCH_NODE_PARENT_OFFSET,  globalId);
            WriteScratchNodeData(bvhNodesOffset, childRight, SCRATCH_NODE_PARENT_OFFSET, globalId);
        }

        if (globalId == 0)
        {
            WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET, numActivePrims);
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
    const uint numActivePrims = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);

    if (numActivePrims > 0)
    {
#if USE_BUILD_LBVH == 1
        const uint bvhNodes = CalculateBvhNodesOffset(ShaderConstants, numActivePrims);

        // Set internal nodes
        if (globalId < (numActivePrims - 1))
        {
            SplitInternalNodeLbvh(
                globalId,
                numActivePrims,
                bvhNodes,
                ShaderConstants.offsets.primIndicesSorted,
                ShaderConstants.offsets.mortonCodesSorted,
                Settings.useMortonCode30);
        }
#endif
    }
}
#endif
