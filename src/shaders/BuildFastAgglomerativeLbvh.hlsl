/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

[[vk::binding(0, 0)]] RWByteAddressBuffer                  SrcBuffer           : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer                  DstBuffer           : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer                  DstMetadata         : register(u2);
[[vk::binding(3, 0)]] globallycoherent RWByteAddressBuffer ScratchBuffer       : register(u3);
[[vk::binding(4, 0)]] globallycoherent RWByteAddressBuffer ScratchGlobal       : register(u4);
[[vk::binding(5, 0)]] RWByteAddressBuffer                  InstanceDescBuffer  : register(u5);
[[vk::binding(6, 0)]] RWByteAddressBuffer                  EmitBuffer          : register(u6);
[[vk::binding(7, 0)]] RWByteAddressBuffer                  IndirectArgBuffer   : register(u7);

#include "Common.hlsl"
#include "BuildCommonScratch.hlsl"
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
    uint doTriangleSplitting;
    uint splitBoxesOffset;
    uint numBatchesOffset;
    uint baseBatchIndicesOffset;
    uint fp16BoxNodesMode;
    float fp16BoxModeMixedSaThreshold;
    uint sortedPrimIndicesOffset;
    uint enablePairCompression;
    uint enablePairCostCheck;
    uint centroidBoxesOffset;
    uint enableCentroidBoxes;
    bool ltdPackCentroids;
    int4 numMortonBits;
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
    args.doTriangleSplitting         = Settings.doTriangleSplitting;
    args.splitBoxesOffset            = ShaderConstants.offsets.triangleSplitBoxes;
    args.numBatchesOffset            = ShaderConstants.offsets.numBatches;
    args.baseBatchIndicesOffset      = ShaderConstants.offsets.batchIndices;
    args.fp16BoxNodesMode            = Settings.fp16BoxNodesMode;
    args.fp16BoxModeMixedSaThreshold = Settings.fp16BoxModeMixedSaThreshold;
    args.sortedPrimIndicesOffset     = ShaderConstants.offsets.primIndicesSorted;
    args.enableEarlyPairCompression  = Settings.enableEarlyPairCompression;
    args.unsortedNodesBaseOffset     = ShaderConstants.offsets.bvhLeafNodeData,

    args.enablePairCompression       = EnableLatePairCompression();
    args.enablePairCostCheck         = Settings.enablePairCostCheck;
    args.centroidBoxesOffset         = 0;
    args.enableCentroidBoxes         = false;
    args.ltdPackCentroids            = false;
    args.numMortonBits               = 0;

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
            // Store invalid index as parent of root
            WriteScratchNodeData(args.baseScratchNodesOffset, currentNodeIndex, SCRATCH_NODE_PARENT_OFFSET, 0xffffffff);
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

    if (globalId < numActivePrims)
    {
        FastAgglomerativeLbvhImpl(globalId, args);
    }
}
#endif
