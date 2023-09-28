/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
                "CBV(b255),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL)"

#include "../shared/rayTracingDefs.h"
[[vk::binding(1, 0)]] ConstantBuffer<BuildShaderConstants> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] RWByteAddressBuffer DstBuffer     : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer DstMetadata   : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer ScratchBuffer : register(u2);

// unused buffer
[[vk::binding(3, 0)]] RWByteAddressBuffer SrcBuffer     : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer EmitBuffer    : register(u4);

#include "Common.hlsl"
#include "BuildCommonScratch.hlsl"
#endif

//=====================================================================================================================
struct FastLBVHArgs
{
    uint rootNodeIndexOffset;
    uint topLevelBuild;
    uint numActivePrims;
    uint baseFlagsOffset;
    uint baseScratchNodesOffset;
    uint sortedMortonCodesOffset;
    uint useMortonCode30;
    uint doCollapse;
    uint doTriangleSplitting;
    uint splitBoxesOffset;
    uint numBatchesOffset;
    uint baseBatchIndicesOffset;
    uint fp16BoxNodesMode;
    float fp16BoxModeMixedSaThreshold;
    bool noCopySortedNodes;
    uint sortedPrimIndicesOffset;
    uint enablePairCompression;
    uint enablePairCostCheck;
    uint centroidBoxesOffset;
    uint enableCentroidBoxes;
    bool ltdPackCentroids;
    int4 numMortonBits;
    uint enableInstancePrimCount;
    uint enableEarlyPairCompression;
    uint unsortedNodesBaseOffset;
    uint reserved0;
    uint reserved1;
};

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
    uint reservedUint0,
    bool reservedUint1,
    uint4 reservedUint2)
{
    // Store leaf node bounding boxes
    const uint nodeIndex = FetchSortedPrimIndex(sortedPrimIndicesOffset, primIndex);

    const ScratchNode unsortedNode = FetchScratchNode(unsortedNodesBaseOffset, nodeIndex);

    const uint scratchNodeOffset = CalcScratchNodeOffset(bvhNodesBaseOffset, LEAFIDX(primIndex));

    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_V0_OFFSET,             unsortedNode.bbox_min_or_v0);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_PRIMITIVE_ID_OFFSET,   unsortedNode.left_or_primIndex_or_instIndex);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_V1_OFFSET,             unsortedNode.bbox_max_or_v1);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_GEOMETRY_INDEX_OFFSET, unsortedNode.right_or_geometryIndex);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_V2_OFFSET,             unsortedNode.sah_or_v2_or_instBasePtr);

    // DO NOT COPY PARENT!!!

    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_TYPE_OFFSET, unsortedNode.type);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_FLAGS_AND_INSTANCE_MASK_OFFSET, unsortedNode.flags_and_instanceMask);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET, unsortedNode.splitBox_or_nodePointer);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET, unsortedNode.numPrimitivesAndDoCollapse);

}

//=====================================================================================================================
// This function indicates a distance metric between the two keys where each internal node splits the hierarchy
// Optionally, we can use the squared distance to compute the distance between two centroids
uint32_t Delta30(
    uint mortonCodesOffset,
    uint id)
{
    const uint left = id;
    const uint right = id + 1;

    // Special handling of duplicated codes: use their indices as a fallback
    const int leftCode  = ScratchBuffer.Load(mortonCodesOffset + (left * sizeof(int)));
    const int rightCode = ScratchBuffer.Load(mortonCodesOffset + (right * sizeof(int)));

    // logical xor can be used instead of finding the index of the highest differing bit as we can compare the numbers.
    // The higher the index of the differing bit, the larger the number
    return (leftCode != rightCode) ? (leftCode ^ rightCode) : (left ^ right);
}

//=====================================================================================================================
// This function indicates a distance metric between the two keys where each internal node splits the hierarchy
// Optionally, we can use the squared distance to compute the distance between two centroids
uint64_t Delta64(
    uint mortonCodesOffset,
    uint id)
{
    const uint left = id;
    const uint right = id + 1;

    // Special handling of duplicated codes: use their indices as a fallback
    const uint64_t leftCode  = ScratchBuffer.Load<uint64_t>(mortonCodesOffset + (left * sizeof(uint64_t)));
    const uint64_t rightCode = ScratchBuffer.Load<uint64_t>(mortonCodesOffset + (right * sizeof(uint64_t)));

    // logical xor can be used instead of finding the index of the highest differing bit as we can compare the numbers.
    // The higher the index of the differing bit, the larger the number
    return (leftCode != rightCode) ? (leftCode ^ rightCode) : (left ^ right);
}

//=====================================================================================================================
bool IsSplitRight(
    uint useMortonCode30,
    uint mortonCodesOffset,
    uint left,
    uint right)
{
    if (useMortonCode30)
    {
        return (Delta30(mortonCodesOffset, right) < Delta30(mortonCodesOffset, left - 1));
    }
    else
    {
        return (Delta64(mortonCodesOffset, right) < Delta64(mortonCodesOffset, left - 1));
    }
}

//=====================================================================================================================
void FastAgglomerativeLbvhImpl(
    const uint primitiveIndex,
    const FastLBVHArgs  args)
{
    RefitArgs refitArgs;

    refitArgs.topLevelBuild               = args.topLevelBuild;
    refitArgs.numActivePrims              = args.numActivePrims;
    refitArgs.baseScratchNodesOffset      = args.baseScratchNodesOffset;
    refitArgs.doCollapse                  = args.doCollapse;
    refitArgs.doTriangleSplitting         = args.doTriangleSplitting;
    refitArgs.enablePairCompression       = args.enablePairCompression;
    refitArgs.enablePairCostCheck         = args.enablePairCostCheck;
    refitArgs.splitBoxesOffset            = args.splitBoxesOffset;
    refitArgs.numBatchesOffset            = args.numBatchesOffset;
    refitArgs.baseBatchIndicesOffset      = args.baseBatchIndicesOffset;
    refitArgs.fp16BoxNodesMode            = args.fp16BoxNodesMode;
    refitArgs.fp16BoxModeMixedSaThreshold = args.fp16BoxModeMixedSaThreshold;
    refitArgs.centroidBoxesOffset         = args.centroidBoxesOffset;
    refitArgs.enableCentroidBoxes         = args.enableCentroidBoxes;
    refitArgs.ltdPackCentroids            = args.ltdPackCentroids;
    refitArgs.numMortonBits               = args.numMortonBits;
    refitArgs.enableInstancePrimCount     = args.enableInstancePrimCount;
    refitArgs.unsortedNodesBaseOffset     = args.unsortedNodesBaseOffset;
    refitArgs.enableEarlyPairCompression  = args.enableEarlyPairCompression;
    // Total number of internal nodes is N - 1
    const uint numInternalNodes = args.numActivePrims - 1;

    // The root of the tree will be stored in the left child of the n-th internal node, where n represents the size of
    // the key array

    // Generate hierarchy recursively. The construction starts from leaf nodes and walks towards the root by finding
    // the parent at each step. We process an internal node only after it has both it's children set. In order to find
    // the parent at each node we have to look at the nodes that split the heirarchy at the left and right ends of the
    // keys covered by the respective node.

    // Leaf nodes cover exactly one range of keys indexed by the primitive index
    const uint sortedPrimitiveIndex = args.noCopySortedNodes ?
        FetchSortedPrimIndex(args.sortedPrimIndicesOffset, primitiveIndex) : primitiveIndex;
    uint left  = primitiveIndex;
    uint right = primitiveIndex;

    // Initialise current node index to leaf node
    uint currentNodeIndex = numInternalNodes + sortedPrimitiveIndex;

    while (1)
    {
        // Choose parent node
        uint previous = 0;
        uint parentNodeIndex = 0xffffffff;

        // we look at the internal nodes with the index i-1 and i and compare the values returned by the
        // delta function. The one with the lowest value will be the parent because it splits the hierarchy
        // between two more similar clusters (subtrees) than the other node.
        const bool useRightParent = ((left == 0) || ((right != numInternalNodes) &&
                                    IsSplitRight(args.useMortonCode30, args.sortedMortonCodesOffset, left, right)));

        parentNodeIndex = useRightParent ? right : left - 1;
        const uint childOffset = useRightParent ? SCRATCH_NODE_LEFT_OFFSET : SCRATCH_NODE_RIGHT_OFFSET;
        if (parentNodeIndex != numInternalNodes)
        {
            // Make the parent node point to the current child
            ScratchBuffer.Store(CalcScratchNodeOffset(args.baseScratchNodesOffset, parentNodeIndex) + childOffset, currentNodeIndex);
            // Link the child to its parent
            ScratchBuffer.Store(CalcScratchNodeOffset(args.baseScratchNodesOffset, currentNodeIndex) + SCRATCH_NODE_PARENT_OFFSET, parentNodeIndex);
        }

        // ... and pass the opposite range of keys to the parent
        const uint flagOffset = args.baseFlagsOffset + (parentNodeIndex * sizeof(uint));
        const uint rangeLimit = useRightParent ? left : right;
        ScratchBuffer.InterlockedExchange(flagOffset, rangeLimit, previous);
        if (previous != 0xffffffff)
        {
            if (useRightParent)
            {
                right = previous;
            }
            else
            {
                left = previous;
            }
        }

        // Special case root nodes. Alternatively, we can allocate one additional internal node to store
        // the root node index and remove this conditional
        if (parentNodeIndex == numInternalNodes)
        {
            // Store invalid index as parent of root
            ScratchBuffer.Store(CalcScratchNodeOffset(args.baseScratchNodesOffset, currentNodeIndex) + SCRATCH_NODE_PARENT_OFFSET, 0xffffffff);
            // Store the index of the root node
            WriteRootNodeIndex(args.rootNodeIndexOffset, currentNodeIndex);
            // Do not write the parent node since it's invalid.
            break;
        }

        // Both child nodes have been processed and the current thread will write the parent node
        if (previous != 0xffffffff)
        {
            // The number of entries in the range of keys represents the number of primitives underneath this internal node
            const uint numTriangles = (right - left) + 1;
            const uint rootIndex = ((left == 0) && (right == numInternalNodes)) ? parentNodeIndex : -1;

            RefitNode(rootIndex,
                      parentNodeIndex,
                      numTriangles,
                      refitArgs);

            // Traverse up to parent node
            currentNodeIndex = parentNodeIndex;
        }
        else
        {
            break;
        }
    }
}

//=====================================================================================================================
// Construct scratch tree topology using the LBVH algorithm
void SplitInternalNodeLbvh(
    uint nodeIndex,
    uint numActivePrims,
    uint scratchNodesOffset,
    uint sortedPrimIndicesOffset,
    uint sortedMortonCodesOffset,
    uint useMortonCode30,
    uint noCopySortedNodes)
{
    // Find span occupied by the current node
    const uint2 range = FindSpan(nodeIndex, numActivePrims, sortedMortonCodesOffset, useMortonCode30);

    // Find split position inside the range
    const uint split = FindSplit(range, numActivePrims, sortedMortonCodesOffset, useMortonCode30);

    // Create child nodes if needed
    const uint splitIdx1 = noCopySortedNodes  ? LEAFIDX(FetchSortedPrimIndex(sortedPrimIndicesOffset, split))     : LEAFIDX(split);
    const uint splitIdx2 = noCopySortedNodes  ? LEAFIDX(FetchSortedPrimIndex(sortedPrimIndicesOffset, split + 1)) : LEAFIDX(split + 1);
    const uint c1idx = (split == range.x)     ? splitIdx1 : NODEIDX(split);
    const uint c2idx = (split + 1 == range.y) ? splitIdx2 : NODEIDX(split + 1);
    const uint c1ScratchNodeOffset = CalcScratchNodeOffset(scratchNodesOffset, c1idx);
    const uint c2ScratchNodeOffset = CalcScratchNodeOffset(scratchNodesOffset, c2idx);

    // update the parent pointer of each child
    ScratchBuffer.Store(c1ScratchNodeOffset + SCRATCH_NODE_PARENT_OFFSET, NODEIDX(nodeIndex));
    ScratchBuffer.Store(c2ScratchNodeOffset + SCRATCH_NODE_PARENT_OFFSET, NODEIDX(nodeIndex));

    // calculate the byte offset of the current node
    const uint scratchNodeOffset = CalcScratchNodeOffset(scratchNodesOffset, NODEIDX(nodeIndex));

    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_LEFT_OFFSET,  c1idx);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_RIGHT_OFFSET, c2idx);
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_FLAGS_AND_INSTANCE_MASK_OFFSET, 0);

    const uint nodeType = GetInternalNodeType();
    ScratchBuffer.Store(scratchNodeOffset + SCRATCH_NODE_TYPE_OFFSET, nodeType);
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

            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_V0_OFFSET,             node.bbox_min_or_v0);
            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_PRIMITIVE_ID_OFFSET,   node.left_or_primIndex_or_instIndex);
            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_V1_OFFSET,             node.bbox_max_or_v1);
            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_GEOMETRY_INDEX_OFFSET, node.right_or_geometryIndex);
            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_V2_OFFSET,             node.sah_or_v2_or_instBasePtr);

            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_TYPE_OFFSET,            node.type);
            ScratchBuffer.Store(targetNodeOffset + SCRATCH_NODE_FLAGS_AND_INSTANCE_MASK_OFFSET, node.flags_and_instanceMask);
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
            ScratchBuffer.Store(nodeOffset + SCRATCH_NODE_FLAGS_AND_INSTANCE_MASK_OFFSET, 0);

            const uint nodeType = GetInternalNodeType();
            ScratchBuffer.Store(nodeOffset + SCRATCH_NODE_TYPE_OFFSET, nodeType);

            const uint childLeftNodeOffset  = CalcScratchNodeOffset(bvhNodesOffset, childLeft);
            const uint childRightNodeOffset = CalcScratchNodeOffset(bvhNodesOffset, childRight);

            ScratchBuffer.Store(childLeftNodeOffset + SCRATCH_NODE_PARENT_OFFSET,  globalId);
            ScratchBuffer.Store(childRightNodeOffset + SCRATCH_NODE_PARENT_OFFSET, globalId);
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
    const int numActivePrims = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);

    if (numActivePrims > 0)
    {
        // Initialise leaf nodes with sorted node data. Copy from LeafAABB to InternalAABB leaf section.
        if (globalId < numActivePrims && (Settings.noCopySortedNodes == false))
        {
            CopyUnsortedScratchLeafNode(
                globalId,
                numActivePrims,
                ShaderConstants.offsets.primIndicesSorted,
                ShaderConstants.offsets.bvhLeafNodeData,
                ShaderConstants.offsets.bvhNodeData,
                0,
                0,
                0,
                false,
                0);
        }

#if USE_BUILD_LBVH == 1
        const uint bvhNodes = CalculateScratchBvhNodesOffset(
                                  numActivePrims,
                                  ShaderConstants.numLeafNodes,
                                  ShaderConstants.offsets.bvhNodeData,
                                  Settings.noCopySortedNodes);
        // Set internal nodes
        if (globalId < (numActivePrims - 1))
        {
            SplitInternalNodeLbvh(
                globalId,
                numActivePrims,
                bvhNodes,
                ShaderConstants.offsets.primIndicesSorted,
                ShaderConstants.offsets.mortonCodesSorted,
                Settings.useMortonCode30,
                Settings.noCopySortedNodes);
        }
#endif
    }
}
#endif
