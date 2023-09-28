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
#define USE_SAH             1
//=====================================================================================================================
// 32 bit constants
struct TDArgs
{
    uint NumPrimitives;
    uint NumThreads;
    uint MaxRefCountSize;
    float LengthPercentage;
    uint BvhNodeDataScratchOffset;
    uint BvhLeafNodeDataScratchOffset; // this IS unsorted leaves  TODO: optimize this out
    uint SceneBoundsOffset;

    uint RefScratchOffset;
    uint TDNodeScratchOffset;
    uint TDBinsScratchOffset;
    uint CurrentStateScratchOffset;
    uint TdTaskQueueCounterScratchOffset;
    uint EncodeArrayOfPointers;
};

#define FLT_MAX         3.402823466e+38F        /* max value */
#define INVALID_IDX     0xffffffff
#define TD_EPSILON      0.99999

#if NO_SHADER_ENTRYPOINT == 0
#define USE_LDS     1

#define RootSig "RootConstants(num32BitConstants=1, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "CBV(b1),"\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b255),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL)"

#include "../shared/rayTracingDefs.h"

struct RootConstants
{
    uint numThreads;
};
[[vk::push_constant]] ConstantBuffer<RootConstants> ShaderRootConstants    : register(b0);
[[vk::binding(1, 1)]] ConstantBuffer<BuildShaderConstants> ShaderConstants : register(b1);

[[vk::binding(0, 0)]]                  RWByteAddressBuffer DstBuffer          : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer DstMetadata        : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer ScratchBuffer      : register(u2);
[[vk::binding(3, 0)]]                  RWByteAddressBuffer InstanceDescBuffer : register(u3);

// unused buffer
[[vk::binding(4, 0)]] RWByteAddressBuffer                  SrcBuffer          : register(u4);
[[vk::binding(5, 0)]] RWByteAddressBuffer                  EmitBuffer         : register(u5);

#include "IntersectCommon.hlsl"
#include "BuildCommon.hlsl"
#include "BuildCommonScratch.hlsl"

groupshared uint SharedMem[1];

#include "TaskQueueCounter.hlsl"

#if USE_LDS
struct LDSData {
    uint numPrimLeft[NUM_SPLIT_BINS - 1];
    uint numPrimRight[NUM_SPLIT_BINS - 1];
    BoundingBox boxLeft[NUM_SPLIT_BINS - 1];
    BoundingBox boxRight[NUM_SPLIT_BINS - 1];
};

groupshared LDSData BuildSharedMem[BUILD_THREADGROUP_SIZE];
#endif
#endif

//=====================================================================================================================
TDRefScratch ReadRefList(uint index, TDArgs args)
{
    return ScratchBuffer.Load<TDRefScratch>(args.RefScratchOffset + (index * sizeof(TDRefScratch)));
}

//=====================================================================================================================
void WriteRefList(uint index, TDRefScratch value, TDArgs args)
{
    ScratchBuffer.Store<TDRefScratch>(args.RefScratchOffset + (index * sizeof(TDRefScratch)), value);
}

//=====================================================================================================================
void UpdateTDRefSide( uint refIndex, uint side, TDArgs args)
{
    uint byteOffset;

    byteOffset = args.RefScratchOffset + sizeof(TDRefScratch) * refIndex + TD_REF_SIDE_OFFSET;

    ScratchBuffer.Store(byteOffset, side);
}

//=====================================================================================================================
void UpdateTDNodeIndex(uint refIndex, uint nodeIndex, TDArgs args)
{
    uint byteOffset;

    byteOffset = args.RefScratchOffset + sizeof(TDRefScratch) * refIndex + TD_REF_NODE_INDEX_OFFSET;

    ScratchBuffer.Store(byteOffset, nodeIndex);
}

//=====================================================================================================================
void InitTDNode(uint nodeIndex, BoundingBox box, uint binsIndex, uint childCount, TDArgs args)
{
    TDNode node;

    node.centroidBox.min = Float3ToUint3(float3(FLT_MAX, FLT_MAX, FLT_MAX));
    node.centroidBox.max = Float3ToUint3(float3(-FLT_MAX, -FLT_MAX, -FLT_MAX));

    node.binsIndex = binsIndex;
    node.childCount = childCount;

    // rebraid stuff
#if USE_BVH_REBRAID
    node.primIndex = INVALID_IDX;
    node.rebraidState = TD_NODE_REBRAID_STATE_CLOSED;

    uint largestAxis;

    // figure out longest axis
    if ((box.max.x - box.min.x) > (box.max.y - box.min.y))
    {
        if ((box.max.x - box.min.x) > (box.max.z - box.min.z))
        {
            largestAxis = 0;
        }
        else
        {
            largestAxis = 2;
        }
    }
    else
    {
        if ((box.max.y - box.min.y) > (box.max.z - box.min.z))
        {
            largestAxis = 1;
        }
        else
        {
            largestAxis = 2;
        }
    }
    node.largestAxis = largestAxis;
    node.largestWidth = (box.max[largestAxis] - box.min[largestAxis]);
#endif

    ScratchBuffer.Store<TDNode>(args.TDNodeScratchOffset + sizeof(TDNode) * nodeIndex, node);
}

//=====================================================================================================================
void InitTDBins(uint binsIndex, TDArgs args)
{
    TDBins bins;

    bins.bestAxis = 0;

    for (uint axis = 0; axis < 3; axis++)
    {
        for (uint i = 0; i < NUM_SPLIT_BINS; i++)
        {
            // This will initialize the minimum value to -inf and the maximum to inf.
            UintBoundingBox bbox;
            bbox.min = Float3ToUint3(float3(FLT_MAX, FLT_MAX, FLT_MAX));
            bbox.max = Float3ToUint3(float3(-FLT_MAX, -FLT_MAX, -FLT_MAX));

            bins.binBoxes[axis][i] = bbox;
            bins.binPrimCount[axis][i] = 0;
#if USE_BLAS_PRIM_COUNT
            bins.binBLASPrimCount[axis][i] = 0;
#endif
        }
    }

    bins.bestSplit = 0;
    bins.numLeft = 0;
    bins.numRight = 0;

    bins.firstRefIndex = 0xffffffffffffffffull;

    ScratchBuffer.Store<TDBins>(args.TDBinsScratchOffset + sizeof(TDBins) * binsIndex, bins);
}

//=====================================================================================================================
uint AllocScratchNode(uint parent, BoundingBox box, TDArgs args)
{
    const uint numNodesAllocOffset = args.CurrentStateScratchOffset + STATE_TD_NUM_NODES_ALLOCATED_OFFSET;

    uint orig;
    ScratchBuffer.InterlockedAdd(numNodesAllocOffset, 1, orig);

    // alloc internal scratch node
    ScratchNode internalNode = (ScratchNode)0;

    internalNode.bbox_min_or_v0 = box.min;
    internalNode.left_or_primIndex_or_instIndex = 0; // not set yet
    internalNode.bbox_max_or_v1 = box.max;
    internalNode.right_or_geometryIndex = 0; // not set yet
    internalNode.sah_or_v2_or_instBasePtr = float3(0, ComputeBoxSurfaceArea(box), 0);
    internalNode.parent = parent;
    internalNode.type = GetInternalNodeType();

    // Initialize box node flags to 1. These will be updated with the leaf node flags as the tree is constructed.
    internalNode.flags_and_instanceMask = 0xffffffff;
    internalNode.splitBox_or_nodePointer = 0;
    internalNode.numPrimitivesAndDoCollapse = 0;

    ScratchBuffer.Store<ScratchNode>(args.BvhNodeDataScratchOffset +
                                     sizeof(ScratchNode) * orig, internalNode);

    return orig;
}

//=====================================================================================================================
#if USE_BVH_REBRAID
uint AllocScratchLeafNode(uint instanceIndex, uint nodePointer, TDArgs args)
{
    // check if the pre allocated instance is used; if not, use it, if used, alloc a new instance
    const uint offset = args.BvhLeafNodeDataScratchOffset + (instanceIndex * sizeof(ScratchNode)) +
                        SCRATCH_NODE_NODE_POINTER_OFFSET;

    uint original;
    ScratchBuffer.InterlockedCompareExchange(offset, 0, nodePointer, original);

    // not used
    if (original == 0)
    {
        return instanceIndex;
    }
    else // used, alloc a new instance
    {
        const uint numLeafNodesOffset = args.CurrentStateScratchOffset + STATE_TD_LEAF_ALLOC_OFFSET_OFFSET;

        uint orig;
        ScratchBuffer.InterlockedAdd(numLeafNodesOffset, 1, orig);

        return orig;
    }
}
#endif

//=====================================================================================================================
uint AllocTDRefScratch(uint numRefs, TDArgs args)
{
    const uint numRefsOffset = args.CurrentStateScratchOffset + STATE_TD_NUM_REFS_ALLOCATED_OFFSET;

    uint orig;
    ScratchBuffer.InterlockedAdd(numRefsOffset, numRefs, orig);

    return orig;
}

//=====================================================================================================================
void SetScratchLeafNode(uint nodeIndex, TDRefScratch ref, uint scratchLeafNodeBaseIndex, TDArgs args)
{
    // alloc internal leaf node copying from the initial encode leaf
    ScratchNode leafNode = ScratchBuffer.Load<ScratchNode>(args.BvhLeafNodeDataScratchOffset +
                                                           (ref.primitiveIndex * sizeof(ScratchNode)));
    leafNode.parent         = ref.nodeIndex;
    leafNode.bbox_min_or_v0 = ref.box.min;
    leafNode.bbox_max_or_v1 = ref.box.max;
#if USE_BVH_REBRAID
    leafNode.splitBox_or_nodePointer = ref.nodePointer;
#endif

    const uint leafIndex  = scratchLeafNodeBaseIndex + nodeIndex;
    const uint nodeOffset = args.BvhNodeDataScratchOffset + (leafIndex * sizeof(ScratchNode));
    ScratchBuffer.Store<ScratchNode>(nodeOffset, leafNode);

    const uint numLeavesOffset = args.CurrentStateScratchOffset + STATE_TD_NUM_LEAVES_OFFSET;
    ScratchBuffer.InterlockedAdd(numLeavesOffset, 1);
}

//=====================================================================================================================
ScratchNode GetScratchNode(uint index, TDArgs args)
{
    return ScratchBuffer.Load<ScratchNode>(args.BvhNodeDataScratchOffset + sizeof(ScratchNode) * index);
}

//=====================================================================================================================
TDNode GetTDNode(uint index, TDArgs args)
{
    return ScratchBuffer.Load<TDNode>(args.TDNodeScratchOffset + sizeof(TDNode) * index);
}

//=====================================================================================================================
TDBins GetTDBins(uint index, TDArgs args)
{
    return ScratchBuffer.Load<TDBins>(args.TDBinsScratchOffset + sizeof(TDBins) * index);
}

//=====================================================================================================================
UintBoundingBox GetTDNodeCentroidBox(uint index, TDArgs args)
{
    return ScratchBuffer.Load<UintBoundingBox>(args.TDNodeScratchOffset + sizeof(TDNode) * index +
                                               TD_NODE_CENTROID_BOX_OFFSET);
}

//=====================================================================================================================
void SetScratchNodeChildLeft(uint index, uint leftIndex, TDArgs args)
{
    ScratchBuffer.Store(args.BvhNodeDataScratchOffset + sizeof(ScratchNode) * index +
                        SCRATCH_NODE_LEFT_OFFSET, leftIndex);
}

//=====================================================================================================================
void SetScratchNodeChildRight(uint index, uint rightIndex, TDArgs args)
{
    ScratchBuffer.Store(args.BvhNodeDataScratchOffset + sizeof(ScratchNode) * index +
                        SCRATCH_NODE_RIGHT_OFFSET, rightIndex);
}

//=====================================================================================================================
void UpdateScratchNodeFlags(uint scratchNodeIndex, uint leafScratchNodeIndex, TDArgs args)
{
    const uint leafNodeOffset =
        args.BvhLeafNodeDataScratchOffset + (leafScratchNodeIndex * sizeof(ScratchNode));

    const uint flagsAndInstanceMask = ScratchBuffer.Load(leafNodeOffset + SCRATCH_NODE_FLAGS_AND_INSTANCE_MASK_OFFSET);

    UpdateParentScratchNodeFlags(args.BvhNodeDataScratchOffset,
                                 scratchNodeIndex,
                                 flagsAndInstanceMask);
}

//=====================================================================================================================
UintBoundingBox FloatBoxToUintBox(BoundingBox box)
{
    UintBoundingBox uintBox;

    uintBox.min = Float3ToUint3(box.min);
    uintBox.max = Float3ToUint3(box.max);

    return uintBox;
}

//=====================================================================================================================
BoundingBox UintBoxToFloatBox(UintBoundingBox box)
{
    BoundingBox floatBox;

    floatBox.min = Uint3ToFloat3(box.min);
    floatBox.max = Uint3ToFloat3(box.max);

    return floatBox;
}

//=====================================================================================================================
void UpdateTDBinsBinBox(uint index, uint axis, uint binIndex, BoundingBox box, TDArgs args)
{
    const uint binsByteOffset = args.TDBinsScratchOffset + sizeof(TDBins) * index +
                                TD_BINS_BIN_BOXES_OFFSET;

    const uint byteOffset = binsByteOffset + sizeof(UintBoundingBox) * NUM_SPLIT_BINS * axis +
                            sizeof(UintBoundingBox) * binIndex;

    UintBoundingBox uintBox = FloatBoxToUintBox(box);

    uint outValue;
    ScratchBuffer.InterlockedMin(byteOffset, uintBox.min.x, outValue);
    ScratchBuffer.InterlockedMin(byteOffset + 4, uintBox.min.y, outValue);
    ScratchBuffer.InterlockedMin(byteOffset + 8, uintBox.min.z, outValue);

    ScratchBuffer.InterlockedMax(byteOffset + 12, uintBox.max.x, outValue);
    ScratchBuffer.InterlockedMax(byteOffset + 16, uintBox.max.y, outValue);
    ScratchBuffer.InterlockedMax(byteOffset + 20, uintBox.max.z, outValue);
}

//=====================================================================================================================
void UpdateTDBinsFirstRefIndex(uint index, uint2 refIndex, TDArgs args)
{
    const uint byteOffset = args.TDBinsScratchOffset + sizeof(TDBins) * index +
                            TD_BINS_FIRST_REF_INDEX_OFFSET;

#if USE_BVH_REBRAID
    AmdExtD3DShaderIntrinsics_AtomicMinU64_gc(ScratchBuffer, byteOffset, refIndex);
#else
    ScratchBuffer.InterlockedMin(byteOffset, refIndex.x);
#endif
}

//=====================================================================================================================
void UpdateTDBinsBinCount(uint index, uint axis, uint binIndex, uint count, TDArgs args)
{
    const uint binsByteOffset = args.TDBinsScratchOffset + sizeof(TDBins) * index +
                                TD_BINS_BIN_PRIM_COUNT_OFFSET;

    const uint byteOffset = binsByteOffset + sizeof(uint) * NUM_SPLIT_BINS * axis + sizeof(uint) * binIndex;

    uint outValue;
    ScratchBuffer.InterlockedAdd(byteOffset, count, outValue);
}

#if USE_BLAS_PRIM_COUNT
//=====================================================================================================================
void UpdateTDBinsBinBLASCount(uint index, uint axis, uint binIndex, uint count)
{
    const uint binsByteOffset = ShaderConstants.TDBinsScratchOffset + sizeof(TDBins) * index +
                                TD_BINS_BLAS_PRIM_COUNT_OFFSET;

    const uint byteOffset = binsByteOffset + sizeof(uint) * NUM_SPLIT_BINS * axis + sizeof(uint) * binIndex;

    uint outValue;
    ScratchBuffer.InterlockedAdd(byteOffset, count, outValue);
}
#endif

//=====================================================================================================================
void UpdateTDNodeLeftAndBinWidth(uint index, TDArgs args)
{
    UintBoundingBox centroidBoxUint = GetTDNodeCentroidBox(index, args);

    BoundingBox centroidBox = UintBoxToFloatBox(centroidBoxUint);

    // stores left and binWidth in the centroidBox to save memory
    for (uint a = 0; a < 3; a++)
    {
        const uint offset = args.TDNodeScratchOffset + sizeof(TDNode) * index + TD_NODE_CENTROID_BOX_OFFSET +
            sizeof(uint) * a;

        // store "left"
        ScratchBuffer.Store(offset, asuint(centroidBox.min[a]));

        // store binWidth
        const float binWidth = (centroidBox.max[a] - centroidBox.min[a]) / NUM_SPLIT_BINS;
        ScratchBuffer.Store(offset + sizeof(uint) * 3, asuint(binWidth));
    }
}

//=====================================================================================================================
void SetTDNodeBinsIndex(uint tdNodeIndex, uint binsIndex, TDArgs args)
{
    const uint offset = args.TDNodeScratchOffset + sizeof(TDNode) * tdNodeIndex + TD_NODE_BINS_INDEX_OFFSET;

    ScratchBuffer.Store(offset, binsIndex);
}

//=====================================================================================================================
uint AddTDNodeChildCount(uint tdNodeIndex, uint count, TDArgs args)
{
    const uint offset = args.TDNodeScratchOffset + sizeof(TDNode) * tdNodeIndex + TD_NODE_CHILD_COUNT_OFFSET;

    uint orig;
    ScratchBuffer.InterlockedAdd(offset, count, orig);

    return orig;
}

//=====================================================================================================================
float GetLeft(TDNode node, uint axis)
{
    return asfloat(node.centroidBox.min[axis]);
}

//=====================================================================================================================
float GetBinWidth(TDNode node, uint axis)
{
    return asfloat(node.centroidBox.max[axis]);
}

//=====================================================================================================================
void UpdateTDNodeCentroidBox(uint nodeIndex, BoundingBox boxFloat, TDArgs args)
{
    const uint byteOffset = args.TDNodeScratchOffset + sizeof(TDNode) * nodeIndex +
                            TD_NODE_CENTROID_BOX_OFFSET;

    UintBoundingBox box = FloatBoxToUintBox(boxFloat);

    uint outValue;
    ScratchBuffer.InterlockedMin(byteOffset, box.min.x, outValue);
    ScratchBuffer.InterlockedMin(byteOffset + 4, box.min.y, outValue);
    ScratchBuffer.InterlockedMin(byteOffset + 8, box.min.z, outValue);

    ScratchBuffer.InterlockedMax(byteOffset + 12, box.max.x, outValue);
    ScratchBuffer.InterlockedMax(byteOffset + 16, box.max.y, outValue);
    ScratchBuffer.InterlockedMax(byteOffset + 20, box.max.z, outValue);
}

//=====================================================================================================================
void UpdateTDNodeCentroidBoxWithPoint(uint nodeIndex, float3 point3D, TDArgs args)
{
    const uint byteOffset = args.TDNodeScratchOffset + sizeof(TDNode) * nodeIndex +
                            TD_NODE_CENTROID_BOX_OFFSET;

    uint3 pointUint = Float3ToUint3(point3D);

    uint outValue;
    ScratchBuffer.InterlockedMin(byteOffset, pointUint.x, outValue);
    ScratchBuffer.InterlockedMin(byteOffset + 4, pointUint.y, outValue);
    ScratchBuffer.InterlockedMin(byteOffset + 8, pointUint.z, outValue);

    ScratchBuffer.InterlockedMax(byteOffset + 12, pointUint.x, outValue);
    ScratchBuffer.InterlockedMax(byteOffset + 16, pointUint.y, outValue);
    ScratchBuffer.InterlockedMax(byteOffset + 20, pointUint.z, outValue);
}

//=====================================================================================================================
#if USE_BVH_REBRAID
void UpdateTDNodeRebraidState(uint nodeIndex, uint state, TDArgs args)
{
    const uint byteOffset = args.TDNodeScratchOffset + sizeof(TDNode) * nodeIndex +
                            TD_NODE_REBRAID_STATE_OFFSET;

    ScratchBuffer.Store(byteOffset, state);
}
#endif
//=====================================================================================================================
void UpdateTDNodePrimIndex(uint nodeIndex, uint primIndex, TDArgs args)
{
#if USE_BVH_REBRAID
    const uint byteOffset = args.TDNodeScratchOffset + sizeof(TDNode) * nodeIndex +
                            TD_NODE_PRIM_INDEX_OFFSET;

    uint original;
    ScratchBuffer.InterlockedCompareExchange(byteOffset, INVALID_IDX, primIndex, original);

    // check if not unified
    if (original != INVALID_IDX && original != primIndex)
    {
        const uint byteOffset2 = args.TDNodeScratchOffset + sizeof(TDNode) * nodeIndex +
                                 TD_NODE_REBRAID_STATE_OFFSET;

        ScratchBuffer.Store(byteOffset2, TD_NODE_REBRAID_STATE_OPEN);

        const uint isOOM = ScratchBuffer.Load(args.CurrentStateScratchOffset +
                           STATE_TD_REBRAID_STATE_OFFSET) == TD_REBRAID_STATE_OOM;

        // signal globally that there needs to be rebraid done
        if (!isOOM)
        {
            ScratchBuffer.Store(args.CurrentStateScratchOffset + STATE_TD_REBRAID_STATE_OFFSET,
                                TD_REBRAID_STATE_NEED_OPEN);
        }
    }
#endif
}

//=====================================================================================================================
#if USE_BVH_REBRAID
void WriteFp32BoxNodeChildrenTreeRefList(
    const GpuVirtualAddress address,
    const InstanceDesc      desc,
    uint                    replaceIndex,
    uint                    startIndex,
    TDRefScratch            ref,
    out BoundingBox         centroidBox,
    TDArgs                  args)
{
    const Float32BoxNode node = FetchFloat32BoxNode(address,
                                                    ref.nodePointer);

    BoundingBox temp;

    ref.nodePointer = node.child0;
    temp.min = node.bbox0_min;
    temp.max = node.bbox0_max;
    ref.box = TransformBoundingBox(temp, desc.Transform);
    ref.center = (0.5 * (ref.box.max + ref.box.min));
#if USE_BLAS_PRIM_COUNT
    ref.numPrimitives = FetchFloat32BoxNodeNumPrimitives(address, ref.nodePointer);
#endif
    ref.side = INVALID_IDX;
    WriteRefList(replaceIndex, ref, args);

    centroidBox.min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
    centroidBox.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    centroidBox.min = min(centroidBox.min, ref.center);
    centroidBox.max = max(centroidBox.max, ref.center);

    if (node.child1 != INVALID_IDX)
    {
        ref.nodePointer = node.child1;
        temp.min = node.bbox1_min;
        temp.max = node.bbox1_max;
        ref.box = TransformBoundingBox(temp, desc.Transform);
        ref.center = (0.5 * (ref.box.max + ref.box.min));
#if USE_BLAS_PRIM_COUNT
        ref.numPrimitives = FetchFloat32BoxNodeNumPrimitives(address, ref.nodePointer);
#endif
        ref.side = INVALID_IDX;
        WriteRefList(startIndex, ref, args);

        centroidBox.min = min(centroidBox.min, ref.center);
        centroidBox.max = max(centroidBox.max, ref.center);

        startIndex++;
    }

    if (node.child2 != INVALID_IDX)
    {
        ref.nodePointer = node.child2;
        temp.min = node.bbox2_min;
        temp.max = node.bbox2_max;
        ref.box = TransformBoundingBox(temp, desc.Transform);
        ref.center = (0.5 * (ref.box.max + ref.box.min));
#if USE_BLAS_PRIM_COUNT
        ref.numPrimitives = FetchFloat32BoxNodeNumPrimitives(address, ref.nodePointer);
#endif
        ref.side = INVALID_IDX;
        WriteRefList(startIndex, ref, args);

        centroidBox.min = min(centroidBox.min, ref.center);
        centroidBox.max = max(centroidBox.max, ref.center);

        startIndex++;
    }

    if (node.child3 != INVALID_IDX)
    {
        ref.nodePointer = node.child3;
        temp.min = node.bbox3_min;
        temp.max = node.bbox3_max;
        ref.box = TransformBoundingBox(temp, desc.Transform);
        ref.center = (0.5 * (ref.box.max + ref.box.min));
#if USE_BLAS_PRIM_COUNT
        ref.numPrimitives = FetchFloat32BoxNodeNumPrimitives(address, ref.nodePointer);
#endif
        ref.side = INVALID_IDX;
        WriteRefList(startIndex, ref, args);

        centroidBox.min = min(centroidBox.min, ref.center);
        centroidBox.max = max(centroidBox.max, ref.center);

        startIndex++;
    }
}

//=====================================================================================================================
void WriteFp16BoxNodeChildrenTreeRefList(
    const GpuVirtualAddress address,
    const InstanceDesc      desc,
    uint                    replaceIndex,
    uint                    startIndex,
    TDRefScratch            ref,
    out BoundingBox         centroidBox,
    TDArgs                  args)
{
    // Note: fp16 path would not work if enable USE_BLAS_PRIM_COUNT
    // because fp16 box node does not store numPrimitives
    const Float16BoxNode node = FetchFloat16BoxNode(address, ref.nodePointer);

    BoundingBox temp = UncompressBBoxFromUint3(node.bbox0);

    ref.nodePointer = node.child0;
    ref.box = TransformBoundingBox(temp, desc.Transform);
    ref.center = (0.5 * (ref.box.max + ref.box.min));
    ref.side = INVALID_IDX;
    WriteRefList(replaceIndex, ref, args);

    centroidBox.min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
    centroidBox.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    centroidBox.min = min(centroidBox.min, ref.center);
    centroidBox.max = max(centroidBox.max, ref.center);

    if (node.child1 != INVALID_IDX)
    {
        ref.nodePointer = node.child1;
        temp = UncompressBBoxFromUint3(node.bbox1);
        ref.box = TransformBoundingBox(temp, desc.Transform);
        ref.center = (0.5 * (ref.box.max + ref.box.min));
        ref.side = INVALID_IDX;
        WriteRefList(startIndex, ref, args);

        centroidBox.min = min(centroidBox.min, ref.center);
        centroidBox.max = max(centroidBox.max, ref.center);

        startIndex++;
    }

    if (node.child2 != INVALID_IDX)
    {
        ref.nodePointer = node.child2;
        temp = UncompressBBoxFromUint3(node.bbox2);
        ref.box = TransformBoundingBox(temp, desc.Transform);
        ref.center = (0.5 * (ref.box.max + ref.box.min));
        ref.side = INVALID_IDX;
        WriteRefList(startIndex, ref, args);

        centroidBox.min = min(centroidBox.min, ref.center);
        centroidBox.max = max(centroidBox.max, ref.center);

        startIndex++;
    }

    if (node.child3 != INVALID_IDX)
    {
        ref.nodePointer = node.child3;
        temp = UncompressBBoxFromUint3(node.bbox3);
        ref.box = TransformBoundingBox(temp, desc.Transform);
        ref.center = (0.5 * (ref.box.max + ref.box.min));
        ref.side = INVALID_IDX;
        WriteRefList(startIndex, ref, args);

        centroidBox.min = min(centroidBox.min, ref.center);
        centroidBox.max = max(centroidBox.max, ref.center);

        startIndex++;
    }
}

//=====================================================================================================================
void WriteChildrenTreeRefList(
    uint            replaceIndex,
    uint            startIndex,
    TDRefScratch    ref,
    out BoundingBox centroidBox,
    TDArgs          args)
{
    ScratchNode node = ScratchBuffer.Load<ScratchNode>(args.BvhLeafNodeDataScratchOffset +
                                                       ref.primitiveIndex * sizeof(ScratchNode));

    const GpuVirtualAddress address = GetInstanceAddr(asuint(node.sah_or_v2_or_instBasePtr.x),
                                                      asuint(node.sah_or_v2_or_instBasePtr.y));

    InstanceDesc desc;
    if (args.EncodeArrayOfPointers != 0)
    {
        GpuVirtualAddress addr = InstanceDescBuffer.Load<GpuVirtualAddress>(ref.primitiveIndex * GPU_VIRTUAL_ADDRESS_SIZE);
        desc = FetchInstanceDescAddr(addr);
    }
    else
    {
        desc = InstanceDescBuffer.Load<InstanceDesc>(ref.primitiveIndex * INSTANCE_DESC_SIZE);
    }

    if (
        IsBoxNode16(ref.nodePointer))
    {
        WriteFp16BoxNodeChildrenTreeRefList(address, desc, replaceIndex, startIndex, ref, centroidBox, args);
    }
    else // box32 node
    {
        WriteFp32BoxNodeChildrenTreeRefList(address, desc, replaceIndex, startIndex, ref, centroidBox, args);
    }
}
#endif

//=====================================================================================================================
// creates list of references with inactive prims filtered out
//=====================================================================================================================
void BuildRefList(uint globalIndex, TDArgs args)
{
    BoundingBox centroidBBox;

    centroidBBox.min = FLT_MAX;
    centroidBBox.max = -FLT_MAX;

    for (uint i = globalIndex; i < args.NumPrimitives; i += args.NumThreads)
    {
        ScratchNode leafNode = ScratchBuffer.Load<ScratchNode>(args.BvhLeafNodeDataScratchOffset +
                                                               i * sizeof(ScratchNode));

        if (IsNodeActive(leafNode))
        {
            uint waveCount = WaveActiveCountBits(true);

            uint writeIndex = 0;

            if (WaveIsFirstLane())
            {
                writeIndex  = AllocTDRefScratch(waveCount, args);
            }

            writeIndex = WaveReadLaneFirst(writeIndex);

            writeIndex += WavePrefixCountBits(true);

            TDRefScratch ref;
            ref.primitiveIndex = i;
            ref.nodeIndex = 0;

            // calc centroid
            // in TDTR, no TriangleSplitting or TriangleCompression
            const BoundingBox bounds = FetchScratchNodeBoundingBox(leafNode, false, false, false, 0, 0);

            ref.box = bounds;

            ref.center = (0.5 * (bounds.max + bounds.min));

#if USE_BVH_REBRAID
            ref.nodePointer = CreateRootNodePointer();
#endif
#if USE_BLAS_PRIM_COUNT
            const GpuVirtualAddress address = GetInstanceAddr(asuint(leafNode.sah_or_v2_or_instBasePtr.x),
                                                              asuint(leafNode.sah_or_v2_or_instBasePtr.y));

            ref.numPrimitives = FetchFloat32BoxNodeNumPrimitives(address, ref.nodePointer);
#endif
            ref.side = INVALID_IDX;

            WriteRefList(writeIndex, ref, args);

            centroidBBox.min = min(centroidBBox.min, ref.center);
            centroidBBox.max = max(centroidBBox.max, ref.center);
        }
    }

    const float3 waveBoundsMin = WaveActiveMin(centroidBBox.min);
    const float3 waveBoundsMax = WaveActiveMax(centroidBBox.max);

    if (WaveIsFirstLane())
    {
        // update root centroid in global memory
        uint outValue;

        const uint3 minUint = Float3ToUint3(waveBoundsMin);
        const uint3 maxUint = Float3ToUint3(waveBoundsMax);

        ScratchBuffer.InterlockedMin(args.CurrentStateScratchOffset + STATE_TD_CENTROID_BBOX_OFFSET, minUint.x, outValue);
        ScratchBuffer.InterlockedMin(args.CurrentStateScratchOffset + STATE_TD_CENTROID_BBOX_OFFSET + 4, minUint.y, outValue);
        ScratchBuffer.InterlockedMin(args.CurrentStateScratchOffset + STATE_TD_CENTROID_BBOX_OFFSET + 8, minUint.z, outValue);

        ScratchBuffer.InterlockedMax(args.CurrentStateScratchOffset + STATE_TD_CENTROID_BBOX_OFFSET + 12, maxUint.x, outValue);
        ScratchBuffer.InterlockedMax(args.CurrentStateScratchOffset + STATE_TD_CENTROID_BBOX_OFFSET + 16, maxUint.y, outValue);
        ScratchBuffer.InterlockedMax(args.CurrentStateScratchOffset + STATE_TD_CENTROID_BBOX_OFFSET + 20, maxUint.z, outValue);
    }
}

//=====================================================================================================================
uint Align(uint value, uint alignment)
{
    return (value + (alignment - 1)) / alignment;
}

//=====================================================================================================================
// Main Function : BuildTD
//=====================================================================================================================
// todo: dont use scratch node and just go from TDNode->QBVH
void BuildBVHTDImpl(
    uint            globalId,
    uint            localId,
    uint            groupId,
    TDArgs          args)
{
    const uint taskQueueOffset          = args.TdTaskQueueCounterScratchOffset;
    const uint numNodesOffset           = args.CurrentStateScratchOffset + STATE_TD_NUM_NODES_OFFSET;
    const uint numProcessedNodesOffset  = args.CurrentStateScratchOffset + STATE_TD_NUM_PROCESSED_NODES_OFFSET;
    const uint numNodesAllocatedOffset  = args.CurrentStateScratchOffset + STATE_TD_NUM_NODES_ALLOCATED_OFFSET;

    const uint numRefsOffset                = args.CurrentStateScratchOffset + STATE_TD_NUM_REFS_OFFSET;
    const uint numRefsAllocatedOffset       = args.CurrentStateScratchOffset + STATE_TD_NUM_REFS_ALLOCATED_OFFSET;
    const uint numInactiveInstanceOffset    = args.CurrentStateScratchOffset + STATE_TD_NUM_INACTIVE_INSTANCE_OFFSET;

    const uint numLeavesOffset          = args.CurrentStateScratchOffset + STATE_TD_NUM_LEAVES_OFFSET;

    const uint numBinsCounterOffset     = args.CurrentStateScratchOffset + STATE_TD_BINS_COUNTER_OFFSET;

#if USE_BVH_REBRAID
    const uint rebraidStateOffset       = args.CurrentStateScratchOffset + STATE_TD_REBRAID_STATE_OFFSET;
#endif

    uint scratchLeafNodeBaseIndex;

#if USE_BVH_REBRAID
    scratchLeafNodeBaseIndex = (args.MaxRefCountSize - 1);
#else
    scratchLeafNodeBaseIndex = (args.NumPrimitives - 1);
#endif

    if (args.NumPrimitives == 0)
    {
        return;
    }

    ///////// init /////////
    const uint numGroups = args.NumThreads / BUILD_THREADGROUP_SIZE;

    if (globalId == 0)
    {
        AllocTasks(1, TD_PHASE_INIT_STATE, taskQueueOffset);
    }

    while (1)
    {
        const uint2 task = BeginTask(localId, taskQueueOffset);

        const uint taskIndex = task.x;
        const uint phase = task.y;

        globalId = taskIndex * BUILD_THREADGROUP_SIZE + localId;

        switch (phase)
        {
            case TD_PHASE_INIT_STATE:
            {
                if (globalId == 0)
                {
                    StateTDBuild initState;
                    initState.numNodes = 0;
                    initState.numProcessedNodes = 0;
                    initState.numNodesAllocated = 0;
                    initState.numLeaves = 0;
#if USE_BVH_REBRAID
                    // rebraid only
                    initState.rebraidState = TD_REBRAID_STATE_NO_OPEN;
                    initState.leafAllocOffset = args.NumPrimitives;
#endif

                    initState.numRefs = 0;
                    initState.numRefsAllocated = 0;
                    initState.numInactiveInstance = 0;

                    initState.rootCentroidBBox.min = FloatToUint(FLT_MAX);
                    initState.rootCentroidBBox.max = FloatToUint(-FLT_MAX);

                    initState.binsCounter = 0;

                    ScratchBuffer.Store<StateTDBuild>(args.CurrentStateScratchOffset, initState);

                    if (EndTask(localId, taskQueueOffset))
                    {
                        AllocTasks(numGroups, TD_PHASE_INIT_REFS_TO_LEAVES, taskQueueOffset);
                    }
                }
                break;
            }

            case TD_PHASE_INIT_REFS_TO_LEAVES:
            {
                BuildRefList(globalId, args);

                if (EndTask(localId, taskQueueOffset))
                {
                    AllocTasks(numGroups, TD_PHASE_CHECK_NEED_ALLOC, taskQueueOffset);
                }

                break;
            }

            case TD_PHASE_CHECK_NEED_ALLOC:
            {
                uint numRefsAllocated = ScratchBuffer.Load(numRefsAllocatedOffset);
                const uint numInactiveInstances = args.NumPrimitives - numRefsAllocated;
                ScratchBuffer.Store(numInactiveInstanceOffset, numInactiveInstances);

                if (numRefsAllocated == 1)
                {
                    if (globalId == 0)
                    {
                        TDRefScratch ref = ReadRefList(0, args);

                        // alloc internal leaf node copying from the initial encode leaf
                        ScratchNode leafNode =
                            ScratchBuffer.Load<ScratchNode>(args.BvhLeafNodeDataScratchOffset +
                                                            ref.primitiveIndex * sizeof(ScratchNode));
#if USE_BVH_REBRAID
                        leafNode.splitBox_or_nodePointer = CreateRootNodePointer();
#endif
                        ScratchBuffer.Store<ScratchNode>(args.BvhNodeDataScratchOffset, leafNode);

                        WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET, 1);
                        AllocTasks(numGroups, TD_PHASE_DONE, taskQueueOffset);
                    }
                }
                else if (numRefsAllocated == 0)
                {
                    if (globalId == 0)
                    {
                        // This is an empty TLAS, but we didn't know it yet when we were setting up the header writes in the
                        // command buffer. Overwrite the GPU VA to 0 to properly designate the TLAS as empty.
                        DstMetadata.Store<GpuVirtualAddress>(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, 0);

                        WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET, 0);

                        AllocTasks(numGroups, TD_PHASE_DONE, taskQueueOffset);
                    }
                }
                else
                {
                    if (EndTask(localId, taskQueueOffset))
                    {
                        AllocTasks(1, TD_PHASE_ALLOC_ROOT_NODE, taskQueueOffset);
                    }
                }
                break;
            }

            case TD_PHASE_ALLOC_ROOT_NODE:
            {
                uint numRefsAllocated = ScratchBuffer.Load(numRefsAllocatedOffset);
                if (globalId == 0)
                {
                    UintBoundingBox sceneBounds;

                    uint4 data;
                    data    = ScratchBuffer.Load4(args.SceneBoundsOffset); // todo: recalc based on ACTIVE nodes
                    sceneBounds.min = data.xyz;
                    data.xy = ScratchBuffer.Load2(args.SceneBoundsOffset + 0x10);
                    sceneBounds.max = data.wxy;

                    BoundingBox bbox;
                    bbox.min = Uint3ToFloat3(sceneBounds.min);
                    bbox.max = Uint3ToFloat3(sceneBounds.max);

                    BoundingBox bboxCentroid;

                    UintBoundingBox boxCentroidUint;

                    data = ScratchBuffer.Load4(args.CurrentStateScratchOffset + STATE_TD_CENTROID_BBOX_OFFSET);
                    boxCentroidUint.min = data.xyz;
                    data.xy = ScratchBuffer.Load2(args.CurrentStateScratchOffset + STATE_TD_CENTROID_BBOX_OFFSET + 0x10);
                    boxCentroidUint.max = data.wxy;

                    bboxCentroid.min = Uint3ToFloat3(boxCentroidUint.min);
                    bboxCentroid.max = Uint3ToFloat3(boxCentroidUint.max);

                    AllocScratchNode(INVALID_IDX, bbox, args);

                    uint binsIndex = INVALID_IDX;

                    if (numRefsAllocated > 2)
                    {
                        binsIndex = 0;
                        InitTDBins(0, args);
                    }

                    InitTDNode(0, bbox, binsIndex, numRefsAllocated, args);

                    //update counts to alloc counts
                    ScratchBuffer.Store(numNodesOffset, 1);
                    ScratchBuffer.Store(numRefsOffset, numRefsAllocated);

                    UpdateTDNodeCentroidBox(0, bboxCentroid, args);
#if !USE_BVH_REBRAID
                    UpdateTDNodeLeftAndBinWidth(0, args);
                    DeviceMemoryBarrier();
                    if (EndTask(localId, taskQueueOffset))
                    {
                        AllocTasks(numGroups, TD_PHASE_BIN_REFS, taskQueueOffset);
                    }
#else
                    UpdateTDNodeRebraidState(0, TD_NODE_REBRAID_STATE_OPEN, args);
                    ScratchBuffer.Store(rebraidStateOffset, TD_REBRAID_STATE_NEED_OPEN);
                    DeviceMemoryBarrier();
                    if (EndTask(localId, taskQueueOffset))
                    {
                        AllocTasks(numGroups, TD_PHASE_REBRAID_COUNT_OPENINGS, taskQueueOffset);
                    }
#endif
                }
                break;
            }

#if USE_BVH_REBRAID
            case TD_PHASE_REBRAID_COUNT_OPENINGS:
            {
                const uint numRefs = ScratchBuffer.Load(numRefsOffset);
                const uint refsPerThread = Align(numRefs, args.NumThreads);
                const uint needsRebraid = ScratchBuffer.Load(rebraidStateOffset) == TD_REBRAID_STATE_NEED_OPEN;

                if (needsRebraid)
                {
                    const uint start = globalId * refsPerThread;
                    const uint end = min(numRefs, start + refsPerThread);

                    // <count openings>
                    for (uint i = start; i < end; i++)
                    {
                        TDRefScratch ref = ReadRefList(i, args);

                        if (ref.side == REF_SCRATCH_SIDE_LEAF)
                        {
                            continue;
                        }

                        // get node data
                        TDNode currentNode = GetTDNode(ref.nodeIndex, args);

                        if ((currentNode.rebraidState != TD_NODE_REBRAID_STATE_CLOSED) &&
                            ((ref.box.max[currentNode.largestAxis] - ref.box.min[currentNode.largestAxis]) >
                                (currentNode.largestWidth * args.LengthPercentage)) &&
                            IsBoxNode(ref.nodePointer))
                        {
                            const uint64_t instanceBasePointer =
                                ScratchBuffer.Load<uint64_t>(args.BvhLeafNodeDataScratchOffset +
                                                             ref.primitiveIndex * sizeof(ScratchNode) +
                                                             SCRATCH_NODE_INSTANCE_BASE_PTR_OFFSET);

                            const uint count = GetBlasInternalNodeChildCount(instanceBasePointer, ref.nodePointer);

                            AllocTDRefScratch(count - 1, args);
                        }
                    }

                    if (EndTask(localId, taskQueueOffset))
                    {
                        AllocTasks(1, TD_PHASE_REBRAID_CHECK_TERMINATION, taskQueueOffset);
                    }
                }
                else
                {
                    if (EndTask(localId, taskQueueOffset))
                    {
                        AllocTasks(numGroups, TD_PHASE_REBRAID_UPDATE_NODES, taskQueueOffset);
                    }
                }
                break;
            }
            case TD_PHASE_REBRAID_CHECK_TERMINATION:
            {
                // <check for termination (out of memory, no openings)>
                const uint numInactiveInstances =  ScratchBuffer.Load(numInactiveInstanceOffset);

                if (globalId == 0)
                {
                    const uint numRefs = ScratchBuffer.Load(numRefsOffset);
                    const uint numRefsNew = ScratchBuffer.Load(numRefsAllocatedOffset);

                    uint phase;

                    if ((numRefsNew + numInactiveInstances) >= args.MaxRefCountSize) // OOM
                    {
                        ScratchBuffer.Store(rebraidStateOffset, TD_REBRAID_STATE_OOM);

                        ScratchBuffer.Store(numRefsAllocatedOffset, numRefs); // reset

                        phase = TD_PHASE_REBRAID_UPDATE_NODES;
                    }
                    else if (numRefsNew == numRefs) // no openings
                    {
                        ScratchBuffer.Store(rebraidStateOffset, TD_REBRAID_STATE_NO_OPEN);

                        phase = TD_PHASE_REBRAID_UPDATE_NODES;
                    }
                    else // openings occured and NOT OOM
                    {
                        ScratchBuffer.Store(numRefsAllocatedOffset, numRefs); // reset

                        phase = TD_PHASE_REBRAID_OPEN;
                    }

                    if (EndTask(localId, taskQueueOffset))
                    {
                        AllocTasks(numGroups, phase, taskQueueOffset);
                    }
                }
                break;
            }

            case TD_PHASE_REBRAID_OPEN:
            {
                const uint numRefs = ScratchBuffer.Load(numRefsOffset);
                const uint refsPerThread = Align(numRefs, args.NumThreads);

                // <reorder the openings + node data>
                // <write new refs>
                const uint start = globalId * refsPerThread;
                const uint end = min(numRefs, start + refsPerThread);

                for (uint i = start; i < end; i++)
                {
                    TDRefScratch ref = ReadRefList(i, args);

                    if (ref.side == REF_SCRATCH_SIDE_LEAF)
                    {
                        continue;
                    }

                    // get node data
                    TDNode currentNode = GetTDNode(ref.nodeIndex, args);

                    if ((currentNode.rebraidState != TD_NODE_REBRAID_STATE_CLOSED) &&
                        ((ref.box.max[currentNode.largestAxis] - ref.box.min[currentNode.largestAxis]) >
                            (currentNode.largestWidth * args.LengthPercentage)) &&
                        IsBoxNode(ref.nodePointer))
                    {
                        const uint64_t instanceBasePointer =
                                ScratchBuffer.Load<uint64_t>(args.BvhLeafNodeDataScratchOffset +
                                                             ref.primitiveIndex * sizeof(ScratchNode) +
                                                             SCRATCH_NODE_INSTANCE_BASE_PTR_OFFSET);

                        const uint count = GetBlasInternalNodeChildCount(instanceBasePointer, ref.nodePointer);

                        uint startIndex = AllocTDRefScratch(count - 1, args);

                        AddTDNodeChildCount(ref.nodeIndex, count - 1, args);

                        BoundingBox centroidBox;

                        WriteChildrenTreeRefList(i, startIndex, ref, centroidBox, args);

                        UpdateTDNodeCentroidBox(ref.nodeIndex, centroidBox, args);
                    }
                }

                if (EndTask(localId, taskQueueOffset))
                {
                    uint numRefsAllocated = ScratchBuffer.Load(numRefsAllocatedOffset);

                    ScratchBuffer.Store(numRefsOffset, numRefsAllocated);

                    AllocTasks(numGroups, TD_PHASE_REBRAID_UPDATE_NODES, taskQueueOffset);
                }
                break;
            }
            case TD_PHASE_REBRAID_UPDATE_NODES:
            {
                const uint numNodes = ScratchBuffer.Load(numNodesOffset);
                const uint numProcessedNodes = ScratchBuffer.Load(numProcessedNodesOffset);

                if (globalId == 0)
                {
                    const uint isOOM = ScratchBuffer.Load(rebraidStateOffset) == TD_REBRAID_STATE_OOM;

                    if (!isOOM)
                    {
                        ScratchBuffer.Store(rebraidStateOffset, TD_REBRAID_STATE_NO_OPEN);
                    }
                }

                for (uint s = numProcessedNodes + globalId; s < numNodes; s += args.NumThreads)
                {
                    TDNode tdNode = GetTDNode(s, args);

                    // update left + bin width
                    UpdateTDNodeLeftAndBinWidth(s, args);

                    if (tdNode.childCount > 2)
                    {
                        uint binsIndex;

                        ScratchBuffer.InterlockedAdd(numBinsCounterOffset, 1, binsIndex);

                        SetTDNodeBinsIndex(s, binsIndex, args);

                        InitTDBins(binsIndex, args);
                    }
                }

                if (EndTask(localId, taskQueueOffset))
                {
                    AllocTasks(numGroups, TD_PHASE_BIN_REFS, taskQueueOffset);
                }
                break;
            }
#endif
            case TD_PHASE_BIN_REFS:
            {
                const uint numRefs = ScratchBuffer.Load(numRefsOffset);
                const uint refsPerThread = Align(numRefs, args.NumThreads);

                const uint numProcessedNodes = ScratchBuffer.Load(numProcessedNodesOffset);

                const uint start = globalId * refsPerThread;
                const uint end = min(numRefs, start + refsPerThread);

                for (uint r = start; r < end; r++)
                {
                    TDRefScratch ref = ReadRefList(r, args);

                    if (ref.side == REF_SCRATCH_SIDE_LEAF)
                    {
                        continue;
                    }

                    // update bins globally
                    TDNode currentNode = GetTDNode(ref.nodeIndex, args);

                    if (currentNode.binsIndex == INVALID_IDX)
                    {
                        continue;
                    }

                    for (uint axis = 0; axis < 3; axis++)
                    {
                        const uint tdBinsIndex = currentNode.binsIndex;
                        const uint binIndex = ((ref.center[axis] - GetLeft(currentNode, axis)) * TD_EPSILON) / GetBinWidth(currentNode, axis);

                        UpdateTDBinsBinBox(tdBinsIndex, axis, binIndex, ref.box, args);

#if USE_BLAS_PRIM_COUNT
                        UpdateTDBinsBinBLASCount(tdBinsIndex, axis, binIndex, ref.numPrimitives, args);
#endif

                        UpdateTDBinsBinCount(tdBinsIndex, axis, binIndex, 1, args);

                        uint2 firstRefIndex;
                        firstRefIndex.x = ref.primitiveIndex;
#if USE_BVH_REBRAID
                        firstRefIndex.y = ref.nodePointer;
#else
                        firstRefIndex.y = 0;
#endif

                        UpdateTDBinsFirstRefIndex(tdBinsIndex, firstRefIndex, args);
                    }
                }

                if (EndTask(localId, taskQueueOffset))
                {
                    AllocTasks(numGroups, TD_PHASE_FIND_BEST_SPLIT, taskQueueOffset);
                }
                break;
            }
            case TD_PHASE_FIND_BEST_SPLIT:
            {
                const uint numNodes = ScratchBuffer.Load(numNodesOffset);
                const uint numProcessedNodes = ScratchBuffer.Load(numProcessedNodesOffset);

                for (uint s = numProcessedNodes + globalId; s < numNodes; s += args.NumThreads)
                {
                    const TDNode tdNode = GetTDNode(s, args);

                    const uint index = tdNode.binsIndex;

                    if (index == INVALID_IDX)
                    {
                        continue;
                    }

                    TDBins bins = GetTDBins(index, args);

#if !USE_LDS
                    uint numPrimLeft[NUM_SPLIT_BINS - 1];
                    uint numPrimRight[NUM_SPLIT_BINS - 1];
                    BoundingBox boxLeft[NUM_SPLIT_BINS - 1];
                    BoundingBox boxRight[NUM_SPLIT_BINS - 1];
#endif

#if USE_BLAS_PRIM_COUNT
                    uint numBLASPrimLeft[NUM_SPLIT_BINS - 1];
                    uint numBLASPrimRight[NUM_SPLIT_BINS - 1];
#endif

                    uint bestSplit = INVALID_IDX;

#if USE_SAH
                    float bestCost = FLT_MAX;
#else
                    float bestCost = 0;
#endif

                    uint bestAxis = 0;
                    uint bestNumLeft;
                    uint bestNumRight;
                    BoundingBox bestBoxLeft;
                    BoundingBox bestBoxRight;

                    for (uint axis = 0; axis < 3; axis++)
                    {
                        BoundingBox tempBox;
                        tempBox.min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
                        tempBox.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

                        uint tempCount = 0;
#if USE_BLAS_PRIM_COUNT
                        uint tempCountBLAS = 0;
#endif

                        // left->right
                        for (uint l = 0; l < NUM_SPLIT_BINS - 1; l++)
                        {
                            tempCount += bins.binPrimCount[axis][l];

#if USE_BLAS_PRIM_COUNT
                            tempCountBLAS += bins.binBLASPrimCount[axis][l];

                            numBLASPrimLeft[l] = tempCountBLAS;
#endif
#if USE_LDS
                            BuildSharedMem[localId].numPrimLeft[l] = tempCount;

                            tempBox.min = min(tempBox.min, UintBoxToFloatBox(bins.binBoxes[axis][l]).min);
                            tempBox.max = max(tempBox.max, UintBoxToFloatBox(bins.binBoxes[axis][l]).max);

                            BuildSharedMem[localId].boxLeft[l] = tempBox;
#else
                            numPrimLeft[l] = tempCount;

                            tempBox.min = min(tempBox.min, UintBoxToFloatBox(bins.binBoxes[axis][l]).min);
                            tempBox.max = max(tempBox.max, UintBoxToFloatBox(bins.binBoxes[axis][l]).max);

                            boxLeft[l] = tempBox;
#endif
                        }

                        tempBox.min = float3(FLT_MAX, FLT_MAX, FLT_MAX);
                        tempBox.max = float3(-FLT_MAX, -FLT_MAX, -FLT_MAX);

                        tempCount = 0;

#if USE_BLAS_PRIM_COUNT
                        tempCountBLAS = 0;
#endif

                        // right->left
                        for (uint l = NUM_SPLIT_BINS - 1; l > 0; l--)
                        {
                            tempCount += bins.binPrimCount[axis][l];

#if USE_BLAS_PRIM_COUNT
                            tempCountBLAS += bins.binBLASPrimCount[axis][l];

                            numBLASPrimRight[l - 1] = tempCountBLAS;
#endif

#if USE_LDS
                            BuildSharedMem[localId].numPrimRight[l - 1] = tempCount;

                            tempBox.min = min(tempBox.min, UintBoxToFloatBox(bins.binBoxes[axis][l]).min);
                            tempBox.max = max(tempBox.max, UintBoxToFloatBox(bins.binBoxes[axis][l]).max);

                            BuildSharedMem[localId].boxRight[l - 1] = tempBox;
#else
                            numPrimRight[l - 1] = tempCount;

                            tempBox.min = min(tempBox.min, UintBoxToFloatBox(bins.binBoxes[axis][l]).min);
                            tempBox.max = max(tempBox.max, UintBoxToFloatBox(bins.binBoxes[axis][l]).max);

                            boxRight[l - 1] = tempBox;
#endif
                        }

                        // find best split
                        for (uint x = 0; x < NUM_SPLIT_BINS - 1; x++)
                        {
#if USE_LDS
                            if ((BuildSharedMem[localId].numPrimLeft[x] != 0) && (BuildSharedMem[localId].numPrimRight[x] != 0))
#else
                            if ((numPrimLeft[x] != 0) && (numPrimRight[x] != 0))
#endif
                            {
#if USE_BLAS_PRIM_COUNT
                                float SAH = (numBLASPrimLeft[x] * ComputeBoxSurfaceArea(boxLeft[x])) +
                                    (numBLASPrimRight[x] * ComputeBoxSurfaceArea(boxRight[x]));
#else
#if USE_LDS
                                float SAH = (BuildSharedMem[localId].numPrimLeft[x] * ComputeBoxSurfaceArea(BuildSharedMem[localId].boxLeft[x])) +
                                    (BuildSharedMem[localId].numPrimRight[x] * ComputeBoxSurfaceArea(BuildSharedMem[localId].boxRight[x]));
#else
                                float SAH = (numPrimLeft[x] * ComputeBoxSurfaceArea(boxLeft[x])) +
                                    (numPrimRight[x] * ComputeBoxSurfaceArea(boxRight[x]));
#endif
#endif

#if USE_SAH
                                if (SAH < bestCost)
#else
                                if (SAH > bestCost)
#endif
                                {
                                    bestCost = SAH;
                                    bestSplit = x;

                                    bestAxis = axis;
#if USE_LDS
                                    bestNumLeft = BuildSharedMem[localId].numPrimLeft[x];
                                    bestNumRight = BuildSharedMem[localId].numPrimRight[x];
                                    bestBoxLeft = BuildSharedMem[localId].boxLeft[x];
                                    bestBoxRight = BuildSharedMem[localId].boxRight[x];
#else
                                    bestNumLeft = numPrimLeft[x];
                                    bestNumRight = numPrimRight[x];
                                    bestBoxLeft = boxLeft[x];
                                    bestBoxRight = boxRight[x];
#endif
                                }
                            }
                        }
                    }

                    const uint bestSplitOffset = args.TDBinsScratchOffset + sizeof(TDBins) * index +
                        TD_BINS_BEST_SPLIT_OFFSET;

                    ScratchBuffer.Store(bestSplitOffset, bestSplit);

                    const uint bestAxisOffset = args.TDBinsScratchOffset + sizeof(TDBins) * index +
                        TD_BINS_BEST_AXIS_OFFSET;

                    ScratchBuffer.Store(bestAxisOffset, bestAxis);

                    uint numLeft;
                    uint numRight;

                    BoundingBox leftBox;
                    BoundingBox rightBox;

                    // alloc new ScratchNodes and TDNodes
                    if (bestSplit == INVALID_IDX) // todo: use collapse in this case
                    {
#if USE_LDS
                        const uint tempNumRefs = BuildSharedMem[localId].numPrimLeft[0] + BuildSharedMem[localId].numPrimRight[0];
#else
                        const uint tempNumRefs = numPrimLeft[0] + numPrimRight[0];
#endif

                        // no split found
                        numLeft = 1; // force left to a leaf
                        numRight = tempNumRefs - numLeft;

                        // use parent box for both left and right since there's no split
                        ScratchNode scratchNode = GetScratchNode(s, args);

                        leftBox.min = scratchNode.bbox_min_or_v0;
                        leftBox.max = scratchNode.bbox_max_or_v1;

                        rightBox.min = leftBox.min;
                        rightBox.max = leftBox.max;
                    }
                    else
                    {
                        numLeft = bestNumLeft;
                        numRight = bestNumRight;

                        leftBox = bestBoxLeft;
                        rightBox = bestBoxRight;
                    }

                    const uint numLeftOffset = args.TDBinsScratchOffset + sizeof(TDBins) * index +
                        TD_BINS_NUM_LEFT_OFFSET;

                    ScratchBuffer.Store(numLeftOffset, numLeft);

                    const uint numRightOffset = args.TDBinsScratchOffset + sizeof(TDBins) * index +
                        TD_BINS_NUM_RIGHT_OFFSET;

                    ScratchBuffer.Store(numRightOffset, numRight);

                    if (numLeft > 1)
                    {
                        // alloc internal node
                        const uint newNodeIndex = AllocScratchNode(s, leftBox, args);

                        SetScratchNodeChildLeft(s, newNodeIndex, args);

                        uint binsIndex = INVALID_IDX;
#if !USE_BVH_REBRAID
                        if (numLeft > 2)
                        {
                            ScratchBuffer.InterlockedAdd(numBinsCounterOffset, 1, binsIndex);
                        }
#endif
                        // init new TDNode
                        InitTDNode(newNodeIndex, leftBox, binsIndex, numLeft, args);
                    }

                    if (numRight > 1)
                    {
                        // alloc internal node
                        const uint newNodeIndex = AllocScratchNode(s, rightBox, args);

                        SetScratchNodeChildRight(s, newNodeIndex, args);

                        uint binsIndex = INVALID_IDX;
#if !USE_BVH_REBRAID
                        if (numRight > 2)
                        {
                            ScratchBuffer.InterlockedAdd(numBinsCounterOffset, 1, binsIndex);
                        }
#endif
                        // init new TDNode
                        InitTDNode(newNodeIndex, rightBox, binsIndex, numRight, args);
                    }
                }

                if (EndTask(localId, taskQueueOffset))
                {
                    AllocTasks(numGroups, TD_PHASE_SECOND_PASS, taskQueueOffset);
                }
                break;
            }

            case TD_PHASE_SECOND_PASS:
            {
                const uint numRefs = ScratchBuffer.Load(numRefsOffset);
                const uint refsPerThread = Align(numRefs, args.NumThreads);

                const uint numNodes = ScratchBuffer.Load(numNodesOffset);
                const uint numProcessedNodes = ScratchBuffer.Load(numProcessedNodesOffset);

                const uint start = globalId * refsPerThread;
                const uint end = min(numRefs, start + refsPerThread);

                for (uint r = start; r < end; r++)
                {
                    TDRefScratch ref = ReadRefList(r, args);

                    if (ref.side == REF_SCRATCH_SIDE_LEAF)
                    {
                        continue;
                    }

                    const TDNode tdNode = GetTDNode(ref.nodeIndex, args);

                    const uint tdBinsIndex = tdNode.binsIndex;

                    if (tdBinsIndex == INVALID_IDX)
                    {
#if USE_BVH_REBRAID
                        const uint leafIndex = AllocScratchLeafNode(ref.primitiveIndex, ref.nodePointer, args);
#else
                        const uint leafIndex = ref.primitiveIndex;
#endif
                        // leaf
                        UpdateTDRefSide(r, REF_SCRATCH_SIDE_LEAF, args);

                        const uint orig = AddTDNodeChildCount(ref.nodeIndex, -1, args);

                        if (orig == 2)
                        {
                            SetScratchNodeChildLeft(ref.nodeIndex, scratchLeafNodeBaseIndex + leafIndex, args);
                        }
                        else
                        {
                            SetScratchNodeChildRight(ref.nodeIndex, scratchLeafNodeBaseIndex + leafIndex, args);
                        }

                        SetScratchLeafNode(leafIndex, ref, scratchLeafNodeBaseIndex, args);

                        UpdateScratchNodeFlags(ref.nodeIndex, ref.primitiveIndex, args);

                        continue;
                    }

                    TDBins bins = GetTDBins(tdBinsIndex, args);

                    // no split plane
                    if (bins.bestSplit == INVALID_IDX)
                    {
                        // this MUST be a leaf
#if USE_BVH_REBRAID
                        if ((ref.primitiveIndex == (bins.firstRefIndex & 0xffffffff)) &&
                            (ref.nodePointer == (bins.firstRefIndex >> 32)))
#else
                        if ((ref.primitiveIndex == (bins.firstRefIndex & 0xffffffff)))
#endif
                        {
#if USE_BVH_REBRAID
                            const uint leafIndex = AllocScratchLeafNode(ref.primitiveIndex, ref.nodePointer, args);
#else
                            const uint leafIndex = ref.primitiveIndex;
#endif

                            SetScratchNodeChildLeft(ref.nodeIndex, scratchLeafNodeBaseIndex + leafIndex, args);

                            // leaf
                            UpdateTDRefSide(r, REF_SCRATCH_SIDE_LEAF, args);

                            SetScratchLeafNode(leafIndex, ref, scratchLeafNodeBaseIndex, args);
                        }
                        else
                        {
                            if (bins.numRight == 1)
                            {
                                // alloc leaf node
#if USE_BVH_REBRAID
                                const uint leafIndex = AllocScratchLeafNode(ref.primitiveIndex, ref.nodePointer, args);
#else
                                const uint leafIndex = ref.primitiveIndex;
#endif
                                SetScratchNodeChildRight(ref.nodeIndex, scratchLeafNodeBaseIndex + leafIndex, args);

                                // leaf
                                UpdateTDRefSide(r, REF_SCRATCH_SIDE_LEAF, args);

                                SetScratchLeafNode(leafIndex, ref, scratchLeafNodeBaseIndex, args);
                            }
                            else
                            {
                                // right
                                UpdateTDRefSide(r, REF_SCRATCH_SIDE_RIGHT, args);

                                // get scratch node
                                ScratchNode scratchNode = GetScratchNode(ref.nodeIndex, args);

                                // update ref's node owner
                                UpdateTDNodeIndex(r, scratchNode.right_or_geometryIndex, args);

                                // update TD node data
                                UpdateTDNodePrimIndex(scratchNode.right_or_geometryIndex, ref.primitiveIndex, args);
                                UpdateTDNodeCentroidBoxWithPoint(scratchNode.right_or_geometryIndex, ref.center, args);
                            }
                        }
                    }
                    else
                    {
                        // figure out if ref is on left or right
                        uint binIndex = ((ref.center[bins.bestAxis] - GetLeft(tdNode, bins.bestAxis)) * TD_EPSILON) /
                            GetBinWidth(tdNode, bins.bestAxis);

                        if (binIndex <= bins.bestSplit)
                        {
                            if (bins.numLeft == 1)
                            {
#if USE_BVH_REBRAID
                                const uint leafIndex = AllocScratchLeafNode(ref.primitiveIndex, ref.nodePointer, args);
#else
                                const uint leafIndex = ref.primitiveIndex;
#endif

                                SetScratchNodeChildLeft(ref.nodeIndex, scratchLeafNodeBaseIndex + leafIndex, args);

                                // leaf
                                UpdateTDRefSide(r, REF_SCRATCH_SIDE_LEAF, args);

                                SetScratchLeafNode(leafIndex, ref, scratchLeafNodeBaseIndex, args);
                            }
                            else
                            {
                                // left
                                UpdateTDRefSide(r, REF_SCRATCH_SIDE_LEFT, args);

                                // get scratch node
                                ScratchNode scratchNode = GetScratchNode(ref.nodeIndex, args);

                                // update ref's node owner
                                UpdateTDNodeIndex(r, scratchNode.left_or_primIndex_or_instIndex, args);

                                // update TD node data
                                UpdateTDNodePrimIndex(scratchNode.left_or_primIndex_or_instIndex, ref.primitiveIndex, args);
                                UpdateTDNodeCentroidBoxWithPoint(scratchNode.left_or_primIndex_or_instIndex, ref.center, args);
                            }
                        }
                        else
                        {
                            if (bins.numRight == 1)
                            {
#if USE_BVH_REBRAID
                                const uint leafIndex = AllocScratchLeafNode(ref.primitiveIndex, ref.nodePointer, args);
#else
                                const uint leafIndex = ref.primitiveIndex;
#endif

                                SetScratchNodeChildRight(ref.nodeIndex, scratchLeafNodeBaseIndex + leafIndex, args);

                                // leaf
                                UpdateTDRefSide(r, REF_SCRATCH_SIDE_LEAF, args);

                                SetScratchLeafNode(leafIndex, ref, scratchLeafNodeBaseIndex, args);
                            }
                            else
                            {
                                // right
                                UpdateTDRefSide(r, REF_SCRATCH_SIDE_RIGHT, args);

                                // get scratch node
                                ScratchNode scratchNode = GetScratchNode(ref.nodeIndex, args);

                                // update ref's node owner
                                UpdateTDNodeIndex(r, scratchNode.right_or_geometryIndex, args);

                                // update TD node data
                                UpdateTDNodePrimIndex(scratchNode.right_or_geometryIndex, ref.primitiveIndex, args);
                                UpdateTDNodeCentroidBoxWithPoint(scratchNode.right_or_geometryIndex, ref.center, args);
                            }
                        }
                    }

                    UpdateScratchNodeFlags(ref.nodeIndex, ref.primitiveIndex, args);
                }

                if (EndTask(localId, taskQueueOffset))
                {
                    if (ScratchBuffer.Load(numLeavesOffset) == numRefs)
                    {
                        WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET, numRefs);

                        AllocTasks(numGroups, TD_PHASE_DONE, taskQueueOffset);
                    }
                    else
                    {
                        // update processed nodes
                        ScratchBuffer.Store(numProcessedNodesOffset, numNodes);

                        ScratchBuffer.Store(numNodesOffset, ScratchBuffer.Load(numNodesAllocatedOffset));

                        AllocTasks(numGroups, TD_PHASE_UPDATE_NEW_NODES, taskQueueOffset);
                    }
                }
                break;
            }

            case TD_PHASE_UPDATE_NEW_NODES:
            {
                const uint numNodes = ScratchBuffer.Load(numNodesOffset);
                const uint numProcessedNodes = ScratchBuffer.Load(numProcessedNodesOffset);
#if !USE_BVH_REBRAID
                for (uint s = numProcessedNodes + globalId; s < numNodes; s += args.NumThreads)
                {
                    // update left + bin width
                    UpdateTDNodeLeftAndBinWidth(s, args);
                }

                // init bins
                const uint start = globalId;
                const uint end = ScratchBuffer.Load(numBinsCounterOffset);

                for (uint index = start; index < end; index += args.NumThreads)
                {
                    InitTDBins(index, args);
                }
#endif
                if (EndTask(localId, taskQueueOffset))
                {
                    // reset count for next pass
                    ScratchBuffer.Store(numBinsCounterOffset, 0);

#if USE_BVH_REBRAID
                    AllocTasks(numGroups, TD_PHASE_REBRAID_COUNT_OPENINGS, taskQueueOffset);
#else
                    AllocTasks(numGroups, TD_PHASE_BIN_REFS, taskQueueOffset);
#endif
                }
                break;
            }

            case TD_PHASE_DONE:
#if USE_BVH_REBRAID
                // initialize the extra prim node pointers to invalid
                const uint maxNumPrimitives       = args.MaxRefCountSize;
                const uint basePrimNodePtrsOffset =
                    ReadAccelStructHeaderField(
                        ACCEL_STRUCT_HEADER_OFFSETS_OFFSET + ACCEL_STRUCT_OFFSETS_PRIM_NODE_PTRS_OFFSET);

                for (uint i = globalId + args.NumPrimitives; i < maxNumPrimitives; i += args.NumThreads)
                {
                    const uint extraPrimNodePtrOffset = basePrimNodePtrsOffset + (i * sizeof(uint));

                    DstBuffer.Store(extraPrimNodePtrOffset, INVALID_IDX);
                }
#endif
                return;
        }
    }
}

#if NO_SHADER_ENTRYPOINT == 0
//====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void BuildBVHTD(
    uint globalId : SV_DispatchThreadID,
    uint groupId  : SV_GroupID,
    uint localId  : SV_GroupThreadID)
{
    uint numPrimitives = ShaderConstants.numPrimitives;

    if (Settings.doTriangleSplitting || (Settings.rebraidType == RebraidType::V2))
    {
        numPrimitives = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET);
    }

    TDArgs args;

    args.NumPrimitives                = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET);
    args.NumThreads                   = ShaderRootConstants.numThreads;
    args.MaxRefCountSize              = numPrimitives * ShaderConstants.rebraidFactor;
    args.LengthPercentage             = 0.1;
    args.BvhNodeDataScratchOffset     = ShaderConstants.offsets.bvhNodeData;
    args.BvhLeafNodeDataScratchOffset = ShaderConstants.offsets.bvhLeafNodeData;
    args.SceneBoundsOffset            = ShaderConstants.offsets.sceneBounds;
    args.RefScratchOffset             = ShaderConstants.offsets.tdRefs;
    args.TDNodeScratchOffset          = ShaderConstants.offsets.tdNodeList;
    args.TDBinsScratchOffset          = ShaderConstants.offsets.tdBins;
    args.CurrentStateScratchOffset    = ShaderConstants.offsets.tdState;
    args.TdTaskQueueCounterScratchOffset = ShaderConstants.offsets.tdTaskQueueCounter;
    args.EncodeArrayOfPointers        = ShaderConstants.encodeArrayOfPointers;

    BuildBVHTDImpl(globalId, localId, groupId, args);
}
#endif
