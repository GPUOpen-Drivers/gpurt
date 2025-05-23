/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
//=====================================================================================================================
#include "../shadersClean/common/ShaderDefs.hlsli"

#define GC_SCRATCHBUFFER
#include "../shadersClean/build/BuildRootSignature.hlsli"

#include "../shadersClean/common/Common.hlsli"
#include "../shadersClean/build/BuildCommonScratch.hlsli"
#endif

struct FastLBVHArgs
{
    uint rootNodeIndexOffset;
    uint topLevelBuild;
    uint numActivePrims;
    uint baseFlagsOffset;
    uint baseScratchNodesOffset;
    uint sortedMortonCodesOffset;
    uint useMortonCode30;
    uint numBatchesOffset;
    uint baseBatchIndicesOffset;
    uint fp16BoxNodesMode;
    float fp16BoxModeMixedSaThreshold;
    uint sortedPrimIndicesOffset;
    uint enablePairCompression;
    uint enablePairCostCheck;
    uint enableEarlyPairCompression;
    uint unsortedNodesBaseOffset;
};

FastLBVHArgs GetFastLbvhArgs(uint numActivePrims)
{
    FastLBVHArgs args;

    args.rootNodeIndexOffset         = ShaderConstants.offsets.fastLBVHRootNodeIndex;
    args.topLevelBuild               = Settings.topLevelBuild;
    args.numActivePrims              = numActivePrims;
    args.baseFlagsOffset             = ShaderConstants.offsets.propagationFlags;
    args.baseScratchNodesOffset      = CalculateBvhNodesOffset(ShaderConstants, numActivePrims);
    args.sortedMortonCodesOffset     = ShaderConstants.offsets.mortonCodesSorted;
    args.useMortonCode30             = Settings.useMortonCode30;
    args.numBatchesOffset            = ShaderConstants.offsets.numBatches;
    args.baseBatchIndicesOffset      = ShaderConstants.offsets.batchIndices;
    args.fp16BoxNodesMode            = Settings.fp16BoxNodesMode;
    args.fp16BoxModeMixedSaThreshold = Settings.fp16BoxModeMixedSaThreshold;
    args.sortedPrimIndicesOffset     = ShaderConstants.offsets.primIndicesSorted;
    args.enableEarlyPairCompression  = Settings.enableEarlyPairCompression;
    args.unsortedNodesBaseOffset     = ShaderConstants.offsets.bvhLeafNodeData,

    args.enablePairCompression       = EnableLatePairCompression();
    args.enablePairCostCheck         = Settings.enablePairCostCheck;

    return args;
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

    // returns number of matching bits starting from MSB
    return (leftCode != rightCode) ? clz(leftCode ^ rightCode) : (32 + clz(left ^ right));
}

//=====================================================================================================================
// This function indicates a distance metric between the two keys where each internal node splits the hierarchy
// Optionally, we can use the squared distance to compute the distance between two centroids
uint32_t Delta64(
    uint mortonCodesOffset,
    uint id)
{
    const uint left = id;
    const uint right = id + 1;

    // Special handling of duplicated codes: use their indices as a fallback
    const uint64_t leftCode  = ScratchBuffer.Load<uint64_t>(mortonCodesOffset + (left * sizeof(uint64_t)));
    const uint64_t rightCode = ScratchBuffer.Load<uint64_t>(mortonCodesOffset + (right * sizeof(uint64_t)));

    // returns number of matching bits starting from MSB
    return (leftCode != rightCode) ? clz64(leftCode ^ rightCode) : (64 + clz64(left ^ right));
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
        return (Delta30(mortonCodesOffset, right) > Delta30(mortonCodesOffset, left - 1));
    }
    else
    {
        return (Delta64(mortonCodesOffset, right) > Delta64(mortonCodesOffset, left - 1));
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
    refitArgs.enablePairCompression       = args.enablePairCompression;
    refitArgs.enablePairCostCheck         = args.enablePairCostCheck;
    refitArgs.numBatchesOffset            = args.numBatchesOffset;
    refitArgs.baseBatchIndicesOffset      = args.baseBatchIndicesOffset;
    refitArgs.fp16BoxNodesMode            = args.fp16BoxNodesMode;
    refitArgs.fp16BoxModeMixedSaThreshold = args.fp16BoxModeMixedSaThreshold;
    refitArgs.unsortedNodesBaseOffset     = args.unsortedNodesBaseOffset;
    refitArgs.enableEarlyPairCompression  = args.enableEarlyPairCompression;

    // Total number of internal nodes is N - 1
    const uint numInternalNodes = args.numActivePrims - 1;

    if (numInternalNodes == 0)
    {
        if (primitiveIndex == 0)
        {
            const uint rootIndex = FetchSortedPrimIndex(args.sortedPrimIndicesOffset, 0);
#if GPURT_BUILD_RTIP3_1
            // Note, for RTIP3.1 primitive ranges, we reuse the scratch node parent field to store the
            // pointer to the next leaf node in the primitive range. The parent field is only required for RTIP2.0
            // pair compression logic and should be removed once that code is updated.
            if (EnableLatePairCompression())
#endif
            {
                // Store invalid index as parent of root
                WriteScratchNodeData(args.baseScratchNodesOffset, rootIndex, SCRATCH_NODE_PARENT_OFFSET, 0xffffffff);
            }
            if (args.enablePairCompression)
            {
                // Ensure that a batch index is written out for single-primitive acceleration structures.
                WriteScratchBatchIndex(args.numBatchesOffset, args.baseBatchIndicesOffset, 0);
            }

            WriteRootNodeIndex(args.rootNodeIndexOffset, rootIndex);
        }
        return;
    }

    // The root of the tree will be stored in the left child of the n-th internal node, where n represents the size of
    // the key array

    // Generate hierarchy recursively. The construction starts from leaf nodes and walks towards the root by finding
    // the parent at each step. We process an internal node only after it has both it's children set. In order to find
    // the parent at each node we have to look at the nodes that split the heirarchy at the left and right ends of the
    // keys covered by the respective node.

    // Leaf nodes cover exactly one range of keys indexed by the primitive index
    const uint sortedPrimitiveIndex = FetchSortedPrimIndex(args.sortedPrimIndicesOffset, primitiveIndex);
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
            WriteScratchNodeData(args.baseScratchNodesOffset, parentNodeIndex, childOffset, currentNodeIndex);

#if GPURT_BUILD_RTIP3_1
            // Note, for RTIP3.1 primitive ranges, we reuse the scratch node parent field to store the
            // pointer to the next leaf node in the primitive range. The parent field is only required for RTIP2.0
            // pair compression logic and should be removed once that code is updated.
            if (EnableLatePairCompression())
#endif
            {
                // Link the child to its parent
                WriteScratchNodeData(args.baseScratchNodesOffset, currentNodeIndex, SCRATCH_NODE_PARENT_OFFSET, parentNodeIndex);
            }

            if (childOffset == SCRATCH_NODE_LEFT_OFFSET)
            {
                WriteScratchNodeData(args.baseScratchNodesOffset, parentNodeIndex, SCRATCH_NODE_SORTED_PRIM_INDEX_OFFSET, primitiveIndex);
            }
        }

        // The atomic exchange below guarantees a child has written its addresses to its parent before the parent tries
        // to read the address, but a memory barrier is needed to ensure the write is ordered before the atomic
        // operation in other threads.
        DeviceMemoryBarrier();

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
#if GPURT_BUILD_RTIP3_1
            // Note, for RTIP3.1 primitive ranges, we reuse the scratch node parent field to store the
            // pointer to the next leaf node in the primitive range. The parent field is only required for RTIP2.0
            // pair compression logic and should be removed once that code is updated.
            if (EnableLatePairCompression())
#endif
            {
                // Store invalid index as parent of root
                WriteScratchNodeData(args.baseScratchNodesOffset, currentNodeIndex, SCRATCH_NODE_PARENT_OFFSET, 0xffffffff);
            }

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

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
// Main Function : BuildFastAgglomerativeLbvh
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void BuildFastAgglomerativeLbvh(
    uint globalId : SV_DispatchThreadID)
{
    const uint numActivePrims = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
    const FastLBVHArgs args   = GetFastLbvhArgs(numActivePrims);

    if (numActivePrims == 0)
    {
        if (globalId == 0)
        {
            WriteRootNodeIndex(args.rootNodeIndexOffset, 0);
        }
    }
    else if (globalId < numActivePrims)
    {
        FastAgglomerativeLbvhImpl(globalId, args);
    }
}
#endif
