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
#include "Common.hlsl"
#include "IntersectCommon.hlsl"
#include "BuildCommon.hlsl"
#include "CompactCommon.hlsl"
#endif

struct BuildQbvhArgs
{
    uint numPrimitives;
    uint metadataSizeInBytes;
    uint numThreads;
    uint scratchNodesScratchOffset;
    uint qbvhStackScratchOffset;
    uint stackPtrsScratchOffset;
    uint triangleCompressionMode;
    uint fp16BoxNodesInBlasMode;        // Mode used for which BLAS interior nodes are FP16
    uint flags;
    uint splitBoxesByteOffset;
    uint emitCompactSize;
    uint encodeArrayOfPointers;
    uint topDownBuild;
    uint bvhBuilderNodeSortType;        // bvhBuilder node sort Type
    uint bvhBuilderNodeSortHeuristic;   // bvhBuilder node sort heuristic
    uint enableFusedInstanceNode;
    uint sahQbvh;                       // Apply SAH into QBVH build
    uint captureChildNumPrimsForRebraid;
    uint enableSAHCost;
    uint enableEarlyPairCompression;
};

#if NO_SHADER_ENTRYPOINT == 0
#define RootSig "RootConstants(num32BitConstants=21, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894))"

//=====================================================================================================================
[[vk::push_constant]] ConstantBuffer<BuildQbvhArgs> ShaderConstants : register(b0);

[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer  DstBuffer          : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer  DstMetadata        : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer  ScratchBuffer      : register(u2);
[[vk::binding(3, 0)]]                  RWByteAddressBuffer  InstanceDescBuffer : register(u3);
[[vk::binding(4, 0)]]                  RWByteAddressBuffer  EmitBuffer         : register(u4);

#define MAX_LDS_ELEMENTS (16 * BUILD_THREADGROUP_SIZE)
groupshared uint SharedMem[MAX_LDS_ELEMENTS];
#endif

// Number of child nodes in LDS
#define LDS_CHILD_NODE_COUNT 4

//=====================================================================================================================
bool DoCollapse(BuildQbvhArgs args)
{
    return (args.flags & BUILD_FLAGS_COLLAPSE);
}

//=====================================================================================================================
bool DoTriSplitting(BuildQbvhArgs args)
{
    return (args.flags & BUILD_FLAGS_TRIANGLE_SPLITTING);
}

//======================================================================================================================
bool EnableLatePairCompression(
    in uint triangleCompressionMode,
    in bool topLevelBuild,
    in bool enableEarlyPairCompression)
{
    return (triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
           (topLevelBuild == false) &&
           (enableEarlyPairCompression == false);
}

//=====================================================================================================================
struct Task
{
    uint nodeIndex;
    uint leafIndex;
    uint numPrimitives;
    uint lastNodeIndex;
    uint parentOfCollapseNodeIndex;
    uint nodeDestIndex;
};

#define TASK_NODE_INDEX_OFFSET                     0
#define TASK_LEAF_INDEX_OFFSET                     4
#define TASK_NUM_PRIMS_OFFSET                      8
#define TASK_LAST_NODE_INDEX_OFFSET               12
#define TASK_PARENT_OF_COLLAPSE_NODE_INDEX_OFFSET 16
#define TASK_NODE_DEST_INDEX                      20
#define TASK_SIZE                                 24

//=====================================================================================================================
bool IsMixedNodeSizes(in BuildQbvhArgs args)
{
    const bool mixedNodeSizes = (args.fp16BoxNodesInBlasMode == LEAF_NODES_IN_BLAS_AS_FP16) ||
                                (args.fp16BoxNodesInBlasMode == MIXED_NODES_IN_BLAS_AS_FP16);
    return mixedNodeSizes;
}

//=====================================================================================================================
uint CalcInstanceNodeOffset(
    uint enableFusedInstanceNode,
    uint baseInstanceNodesOffset,
    uint index)
{
    return baseInstanceNodesOffset + (index * GetBvhNodeSizeInstance(enableFusedInstanceNode));
}

//=====================================================================================================================
static uint GetNum64BChunks(in BuildQbvhArgs args, in ScratchNode node, uint numActivePrims)
{
    if (IsBoxNode(node.type) == false)
    {
        return 0;
    }
    else if (IsBoxNode16(node.type))
    {
        return 1;
    }
    else
    {
        return 2;
    }
}

//======================================================================================================================
static void InitBuildQbvhImpl(
    in uint          globalId,
    in BuildQbvhArgs args)
{
    // @note This relies on fp16BoxNodesInBlasMode and collapse build flag being disabled in the shader constants for TLAS

    // Store 2 entries per stack only when mixing fp32, half fp32 and fp16 nodes
    const bool mixedNodeSizes = IsMixedNodeSizes(args);

    // Note the first entry is initialized below. Skip initializing it to invalid.
    for (uint stackIndex = globalId + 1; stackIndex < CalcNumQBVHInternalNodes(args.numPrimitives); stackIndex += args.numThreads)
    {
        if (DoCollapse(args))
        {
            const uint qbvhStackOffset = args.qbvhStackScratchOffset + (stackIndex * TASK_SIZE);
            ScratchBuffer.Store(qbvhStackOffset + TASK_NODE_INDEX_OFFSET, INVALID_IDX);
        }
        else
        {
            if (mixedNodeSizes)
            {
                const uint qbvhStackOffset = args.qbvhStackScratchOffset + (stackIndex * sizeof(uint2));
                ScratchBuffer.Store<uint2>(qbvhStackOffset, uint2(INVALID_IDX, INVALID_IDX));
            }
            else
            {
                const uint qbvhStackOffset = args.qbvhStackScratchOffset + (stackIndex * sizeof(uint));
                ScratchBuffer.Store(qbvhStackOffset, INVALID_IDX);
            }
        }
    }

    if (globalId == 0)
    {
        StackPtrs stackPtrs;
        // Source node index in linear memory
        stackPtrs.stackPtrSrcNodeId = 1;
        // Node destination in linear memory. Counts in 64B chunks.
        stackPtrs.stackPtrNodeDest  = 2;
        stackPtrs.numLeafsDone      = 0;

        ScratchBuffer.Store<StackPtrs>(args.stackPtrsScratchOffset, stackPtrs);

        if (DoCollapse(args))
        {
            Task task;
            task.nodeIndex                  = 0;
            task.leafIndex                  = 0;
            task.numPrimitives              = 0;
            task.lastNodeIndex              = INVALID_IDX;
            task.parentOfCollapseNodeIndex  = INVALID_IDX;
            task.nodeDestIndex              = 0;

            ScratchBuffer.Store<Task>(args.qbvhStackScratchOffset, task);
        }
        else
        {
            if (mixedNodeSizes)
            {
                ScratchBuffer.Store<uint2>(args.qbvhStackScratchOffset, uint2(0, 0));
            }
            else
            {
                ScratchBuffer.Store(args.qbvhStackScratchOffset, 0);
            }
        }
    }
}

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
// Main Function : InitBuildQBVH
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void InitBuildQBVH(
    uint globalId : SV_DispatchThreadID)
{
    const AccelStructHeader header        = DstBuffer.Load<AccelStructHeader>(0);
    const uint              type          = (header.info & ACCEL_STRUCT_HEADER_INFO_TYPE_MASK);
    const bool              topLevelBuild = (type == TOP_LEVEL);

    BuildQbvhArgs args = (BuildQbvhArgs)ShaderConstants;

    // Rebraid is not supported in non-parallel path. numLeafNodes is always equal to numActivePrims.
    //
    args.numPrimitives = header.numActivePrims;

    // With pair compression enabled, numLeafNodes represents the actual primitive count
    if (EnableLatePairCompression(ShaderConstants.triangleCompressionMode,
                                  topLevelBuild,
                                  ShaderConstants.enableEarlyPairCompression))
    {
        args.numPrimitives = header.numLeafNodes;
    }

    InitBuildQbvhImpl(globalId, args);
}
#endif

//=====================================================================================================================
uint WriteInstanceNode(
    in BuildQbvhArgs      args,
    in ScratchNode        scratchNode,
    in uint               scratchNodeIndex,
    in uint               nodeOffset,
    in AccelStructOffsets offsets)
{
    const uint nodeType      = GetNodeType(scratchNode.type);
    const uint instanceIndex = scratchNode.left_or_primIndex_or_instIndex;

    InstanceDesc instanceDesc;
    if (args.encodeArrayOfPointers != 0)
    {
        GpuVirtualAddress addr = InstanceDescBuffer.Load<GpuVirtualAddress>(instanceIndex * GPU_VIRTUAL_ADDRESS_SIZE);
        instanceDesc = FetchInstanceDesc(addr, 0);
    }
    else
    {
        instanceDesc = InstanceDescBuffer.Load<InstanceDesc>(instanceIndex * INSTANCE_DESC_SIZE);
    }

    const GpuVirtualAddress baseAddr = MakeGpuVirtualAddress(instanceDesc.accelStructureAddressLo,
                                                             instanceDesc.accelStructureAddressHiAndFlags);

    uint64_t blasHeaderAddr =
        ExtractInstanceAddr(PackUint64(asuint(scratchNode.sah_or_v2_or_instBasePtr.x),
                                       asuint(scratchNode.sah_or_v2_or_instBasePtr.y)));

    const uint blasMetadataSize = uint(blasHeaderAddr - baseAddr);

    // There is no rebraid in non Top down build, so destination index is just the instance index.
    const uint destIndex = (args.topDownBuild != 0) ? (scratchNodeIndex - args.numPrimitives + 1) : instanceIndex;

    {
        nodeOffset = CalcInstanceNodeOffset(args.enableFusedInstanceNode, offsets.leafNodes, destIndex);
    }

    instanceDesc.accelStructureAddressLo = asuint(scratchNode.sah_or_v2_or_instBasePtr.x);
    instanceDesc.accelStructureAddressHiAndFlags = asuint(scratchNode.sah_or_v2_or_instBasePtr.y);

    WriteInstanceDescriptor(DstBuffer,
                            instanceDesc,
                            instanceIndex,
                            nodeOffset,
                            blasMetadataSize,
                            scratchNode.splitBox_or_nodePointer,
                            args.enableFusedInstanceNode);

    const uint nodePointer = PackNodePointer(nodeType, nodeOffset);
    DstBuffer.Store(offsets.primNodePtrs + (destIndex * sizeof(uint)), nodePointer);

    ScratchBuffer.InterlockedAdd(args.stackPtrsScratchOffset + STACK_PTRS_NUM_LEAFS_DONE_OFFSET, 1);

    return nodePointer;
}

//=====================================================================================================================
uint WritePrimitiveNode(
    in BuildQbvhArgs      args,
    in ScratchNode        scratchNode,
    in uint               nodeOffset,
    in AccelStructOffsets offsets)
{
    uint nodeType = GetNodeType(scratchNode.type);

    // Load geometry info
    const uint geometryInfoOffset = offsets.geometryInfo + (scratchNode.right_or_geometryIndex * sizeof(GeometryInfo));
    const GeometryInfo geometryInfo = DstBuffer.Load<GeometryInfo>(geometryInfoOffset);
    const uint geometryFlags = ExtractGeometryInfoFlags(geometryInfo.geometryFlagsAndNumPrimitives);
    const uint geometryIndexAndFlags = PackGeometryIndexAndFlags(scratchNode.right_or_geometryIndex, geometryFlags);
    const uint geometryPrimNodePtrsOffset = offsets.primNodePtrs + geometryInfo.primNodePtrsOffset;
    const uint flattenedPrimIndex = (geometryInfo.primNodePtrsOffset / sizeof(uint)) + scratchNode.left_or_primIndex_or_instIndex;

    uint numLeafsDone;
    ScratchBuffer.InterlockedAdd(args.stackPtrsScratchOffset + STACK_PTRS_NUM_LEAFS_DONE_OFFSET, 1, numLeafsDone);

    uint destIndex;
    {
        if (IsTriangleNode(nodeType) && ((args.triangleCompressionMode != NO_TRIANGLE_COMPRESSION) || DoTriSplitting(args)))
        {
            destIndex = numLeafsDone;
        }
        else
        {
            destIndex = flattenedPrimIndex;
        }
    }

    const uint triangleId = scratchNode.type >> 3;

    if (nodeType == NODE_TYPE_USER_NODE_PROCEDURAL)
    {
        {
            nodeOffset = offsets.leafNodes + (destIndex * USER_NODE_PROCEDURAL_SIZE);
        }

        DstBuffer.Store(nodeOffset + USER_NODE_PROCEDURAL_PRIMITIVE_INDEX_OFFSET, scratchNode.left_or_primIndex_or_instIndex);
        DstBuffer.Store3(nodeOffset + USER_NODE_PROCEDURAL_MIN_OFFSET, asuint(scratchNode.bbox_min_or_v0));
        DstBuffer.Store3(nodeOffset + USER_NODE_PROCEDURAL_MAX_OFFSET, asuint(scratchNode.bbox_max_or_v1));
        DstBuffer.Store(nodeOffset + USER_NODE_PROCEDURAL_GEOMETRY_INDEX_AND_FLAGS_OFFSET, geometryIndexAndFlags);
    }
    else
    {
        {
            nodeOffset = offsets.leafNodes + (destIndex * TRIANGLE_NODE_SIZE);
        }

        DstBuffer.Store(nodeOffset + TRIANGLE_NODE_ID_OFFSET, triangleId);
        DstBuffer.Store(nodeOffset + TRIANGLE_NODE_GEOMETRY_INDEX_AND_FLAGS_OFFSET, geometryIndexAndFlags);

        uint3 vertexOffsets;
        if (args.triangleCompressionMode != NO_TRIANGLE_COMPRESSION)
        {
            vertexOffsets = CalcTriangleCompressionVertexOffsets(nodeType, triangleId);
        }
        else
        {
            vertexOffsets = CalcTriangleVertexOffsets(nodeType);
        }

        DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V0_OFFSET + vertexOffsets.x, scratchNode.bbox_min_or_v0);
        DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V0_OFFSET + vertexOffsets.y, scratchNode.bbox_max_or_v1);
        DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V0_OFFSET + vertexOffsets.z, scratchNode.sah_or_v2_or_instBasePtr);

        DstBuffer.Store(nodeOffset + TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET + (nodeType * 4),
                        scratchNode.left_or_primIndex_or_instIndex);

        // For PAIR_TRIANGLE_COMPRESSION, the other node is not linked in the BVH tree, so we need to find it and
        // store it as well if it exists.
        if ((args.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
            (scratchNode.splitBox_or_nodePointer != INVALID_IDX) &&
            // TODO: When EarlyPairCompression is ON, will actually need to fetch paired triangle from a different
            // scratch offset. Thus this will be updated once the EarlyPairCompression is implemented.
            (args.enableEarlyPairCompression == false))
        {
            const ScratchNode otherNode = FetchScratchNode(ScratchBuffer,
                                                           args.scratchNodesScratchOffset,
                                                           scratchNode.splitBox_or_nodePointer);
            const uint otherNodeType = GetNodeType(otherNode.type);

            const float3 otherVerts[3] = { otherNode.bbox_min_or_v0,
                                           otherNode.bbox_max_or_v1,
                                           otherNode.sah_or_v2_or_instBasePtr };
            const uint3 otherVertexOffsets = CalcTriangleCompressionVertexOffsets(otherNodeType, triangleId);
            for (uint i = 0; i < 3; ++i)
            {
                // Since the other node will always be of type NODE_TYPE_TRIANGLE_0, it is sufficient to store only
                // the vertex that goes into v0. v1, v2, and v3 were already stored by NODE_TYPE_TRIANGLE_1 above.
                if (otherVertexOffsets[i] == 0)
                {
                    DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V0_OFFSET, otherVerts[i]);
                }
            }

            DstBuffer.Store(nodeOffset + TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET + (otherNodeType * 4),
                            otherNode.left_or_primIndex_or_instIndex);

            const uint otherPrimNodePointer = PackNodePointer(otherNodeType, nodeOffset);
            DstBuffer.Store(geometryPrimNodePtrsOffset + (otherNode.left_or_primIndex_or_instIndex * sizeof(uint)),
                            otherPrimNodePointer);
        }
    }

    const uint nodePointer = PackNodePointer(nodeType, nodeOffset);
    DstBuffer.Store(geometryPrimNodePtrsOffset + (scratchNode.left_or_primIndex_or_instIndex * sizeof(uint)),
                    nodePointer);

    return nodePointer;
}

//=====================================================================================================================
BoundingBox GetScratchNodeBoundingBoxTS(
    in BuildQbvhArgs args,
    in bool          topLevelBuild,  ///< Compile time flag indicating whether this is top or bottom level build
    in ScratchNode   node)           ///< Node whose bbox to write out
{
    BoundingBox bbox;

    // For triangle geometry we need to generate bounding box from triangle vertices
    if ((topLevelBuild == false) && IsTriangleNode(node.type))
    {
        if (DoTriSplitting(args))
        {
            bbox = ScratchBuffer.Load<BoundingBox>(args.splitBoxesByteOffset +
                                                   sizeof(BoundingBox) * node.splitBox_or_nodePointer);
        }
        else
        {
            bbox = GenerateTriangleBoundingBox(node.bbox_min_or_v0,
                                               node.bbox_max_or_v1,
                                               node.sah_or_v2_or_instBasePtr);

            if ((args.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
                (node.splitBox_or_nodePointer != INVALID_IDX) &&
                // TODO: When EarlyPairCompression is ON, will actually need to fetch paired triangle from a different
                // scratch offset. Thus this will be updated once the EarlyPairCompression is implemented.
                (args.enableEarlyPairCompression == false))
            {
                const ScratchNode otherNode = FetchScratchNode(ScratchBuffer,
                                                               args.scratchNodesScratchOffset,
                                                               node.splitBox_or_nodePointer);
                const BoundingBox otherBbox = GenerateTriangleBoundingBox(otherNode.bbox_min_or_v0,
                                                                          otherNode.bbox_max_or_v1,
                                                                          otherNode.sah_or_v2_or_instBasePtr);

                bbox.min = min(bbox.min, otherBbox.min);
                bbox.max = max(bbox.max, otherBbox.max);
            }
        }
    }
    else
    {
        // Internal nodes and AABB geometry encodes bounding box in scratch node
        bbox.min = node.bbox_min_or_v0;
        bbox.max = node.bbox_max_or_v1;
    }

    return bbox;
}

//=====================================================================================================================
// Returns source BVH node index given node. We read stack entries only for some fp16 box node modes
uint QBVHStackPopNodeIdx(
    BuildQbvhArgs args,
    bool          topLevelBuild,
    uint          stackIndex)
{
    // When all nodes are mixed, stack stores two entries per thread
    if (IsMixedNodeSizes(args))
    {
        stackIndex *= 2;
    }

    return ScratchBuffer.Load(args.qbvhStackScratchOffset + (stackIndex * sizeof(uint)));
}

//=====================================================================================================================
// Returns destination offset in memory for a given node.
// The value returned is in terms of 64B, so 1 for fp16 and 2 for fp32 box node
uint QBVHStackPopDestIdx(
    BuildQbvhArgs args,
    bool          topLevelBuild,
    uint          stackIndex)
{
    uint destIndex;

    // Mixed node types or half f32 box node enabled - requires reading stack entries
    const uint fp16BoxNodesInBlasMode = args.fp16BoxNodesInBlasMode;
    if (IsMixedNodeSizes(args))
    {
        // Collapse should not enter this code path because tasks store destination indexes directly
        destIndex = ScratchBuffer.Load(args.qbvhStackScratchOffset + ((2 * stackIndex + 1) * sizeof(uint)));
    }
    // All nodes are fp32
    else if (topLevelBuild || (fp16BoxNodesInBlasMode == NO_NODES_IN_BLAS_AS_FP16))
    {
        destIndex = 2 * stackIndex;
    }
    // All but root node are fp16
    else
    {
        destIndex = stackIndex + (stackIndex > 0 ? 1 : 0);
    }

    return destIndex;
}

//=====================================================================================================================
// Allocates the appropriate number of nodes on the stack, and writes out stack entries
uint AllocQBVHStackNumItems(
    BuildQbvhArgs args,
    bool          topLevelBuild,
    uint          compChildInfo,        ///< flag if child is to be written out
    uint4         intChildNodeIdx,      ///< scratch node index (from parent) for each child node
    inout uint4   intChildDstOffset,    ///< destination address offset for each child node, uses fp16 / fp32 node size
    uint          numDstChunks64B)      ///< number of 64B chunks required for all child nodes
{
    const uint numStackItems = countbits(compChildInfo & 0xF);

    uint origStackIdx;
    ScratchBuffer.InterlockedAdd(args.stackPtrsScratchOffset + STACK_PTRS_SRC_PTR_OFFSET, numStackItems, origStackIdx);

    // compute proper offsets for each child
    const uint4 intChildStackIdx = origStackIdx + uint4(0,
                                                        countbits(compChildInfo & 0x1),
                                                        countbits(compChildInfo & 0x3),
                                                        countbits(compChildInfo & 0x7));

    uint origDest;

    // Uniform nodes - rely on a single stack item per thread
    if (IsMixedNodeSizes(args) == false)
    {
        origDest = QBVHStackPopDestIdx(args, topLevelBuild, origStackIdx);

        // Destination offset in DstBuffer for each child node
        intChildDstOffset += origDest;

        if (topLevelBuild || (DoCollapse(args) == false))
        {
            // Store destination offsets for every box child node
            if ((compChildInfo & 0x1) == 0x1)
            {
                ScratchBuffer.Store(args.qbvhStackScratchOffset + intChildStackIdx.x * sizeof(uint), intChildNodeIdx.x);
            }
            if ((compChildInfo & 0x2) == 0x2)
            {
                ScratchBuffer.Store(args.qbvhStackScratchOffset + intChildStackIdx.y * sizeof(uint), intChildNodeIdx.y);
            }
            if ((compChildInfo & 0x4) == 0x4)
            {
                ScratchBuffer.Store(args.qbvhStackScratchOffset + intChildStackIdx.z * sizeof(uint), intChildNodeIdx.z);
            }
            if ((compChildInfo & 0x8) == 0x8)
            {
                ScratchBuffer.Store(args.qbvhStackScratchOffset + intChildStackIdx.w * sizeof(uint), intChildNodeIdx.w);
            }

            DeviceMemoryBarrier();
        }
    }
    // Mixed nodes - atomically advance destination memory address to avoid holes between nodes
    else
    {
        ScratchBuffer.InterlockedAdd(args.stackPtrsScratchOffset + STACK_PTRS_DST_PTR_OFFSET, numDstChunks64B, origDest);

        // Destination offset in DstBuffer for each child node
        intChildDstOffset += origDest;

        if (DoCollapse(args) == false)
        {
            // Store destination offsets for every box child node
            if ((compChildInfo & 0x1) == 0x1)
            {
                const uint2 toWrite = { intChildNodeIdx.x, intChildDstOffset.x };
                ScratchBuffer.Store<uint2>(args.qbvhStackScratchOffset + intChildStackIdx.x * sizeof(uint2), toWrite);
            }
            if ((compChildInfo & 0x2) == 0x2)
            {
                const uint2 toWrite = { intChildNodeIdx.y, intChildDstOffset.y };
                ScratchBuffer.Store<uint2>(args.qbvhStackScratchOffset + intChildStackIdx.y * sizeof(uint2), toWrite);
            }
            if ((compChildInfo & 0x4) == 0x4)
            {
                const uint2 toWrite = { intChildNodeIdx.z, intChildDstOffset.z };
                ScratchBuffer.Store<uint2>(args.qbvhStackScratchOffset + intChildStackIdx.z * sizeof(uint2), toWrite);
            }
            if ((compChildInfo & 0x8) == 0x8)
            {
                const uint2 toWrite = { intChildNodeIdx.w, intChildDstOffset.w };
                ScratchBuffer.Store<uint2>(args.qbvhStackScratchOffset + intChildStackIdx.w * sizeof(uint2), toWrite);
            }

            DeviceMemoryBarrier();
        }
    }

    return origStackIdx;
}

//=====================================================================================================================
// Note:
// There are 2 sort type
// 1-) 4-Way Sort:   Sort all 4 child of a box node, given 1 sort heuristic
// 2-) 2-Level Sort: Sort left and right child of each level
//
// There are 4 sort heuristic in total
// 1-) Sort nodes by SurfaceArea, Largest  -> Smallest
// 2-) Sort nodes by SurfaceArea, Smallest -> Largest
// 3-) Sort nodes by density (average primitive per surfaceArea), Largest  -> Smallest
// 4-) Sort nodes by density (average primitive per surfaceArea), Smallest -> Largest
//
// TODO
// 1-) Add "write numPrims (WriteScratchNodeNumPrimitives)" in TopDown Builder
// 2-) Add more Sort Heuristic Support in 2-level sort, currently it only uses SurfaceArea
// 3-) Add NodeSort support for Collapse

#define SORT_LARGEST_FIRST(childA,childB,SurfaceA,SurfaceB) \
if ((childB != INVALID_NODE && SurfaceB > SurfaceA) || childA == INVALID_NODE) \
{float t0 = SurfaceA; uint t1 = childA;  childA = childB; SurfaceA = SurfaceB;  childB = t1; SurfaceB = t0;}

// Sort node by SurceArea, Smallest -> Largest
#define SORT_SMALLEST_FIRST(childA,childB,SurfaceA,SurfaceB) \
if ((childB != INVALID_NODE && SurfaceB < SurfaceA) || childA == INVALID_NODE) \
{float t0 = SurfaceA; uint t1 = childA;  childA = childB; SurfaceA = SurfaceB;  childB = t1; SurfaceB = t0;}

//=====================================================================================================================
void SortChildren(
    in    BuildQbvhArgs args,
    in    uint          numPrimitives[4],
    inout uint          childNodePtr[4],
    inout BoundingBox   bbox[4],
    inout uint          nodeFlags[4],
    inout float4        cost)
{
    const uint sortHeuristic = args.bvhBuilderNodeSortHeuristic;

    float sortKeyBySurfaceArea[4];
    float sortKeyByDensity[4];     // Average numPrimitive Per SurfaceArea
    uint  sortedIdx[4];
    uint  sortedChildNodePtr[4];
    uint  sortedNodeFlags[4];
    BoundingBox sortedBbox[4];

    float sortedCost[4];

    for (uint idx = 0; idx < 4; ++idx)
    {
        sortedIdx[idx] = (childNodePtr[idx] == INVALID_NODE) ? INVALID_NODE : idx;
        sortKeyBySurfaceArea[idx] = ComputeBoxSurfaceArea(bbox[idx]);
        sortKeyByDensity[idx] = numPrimitives[idx] / ComputeBoxSurfaceArea(bbox[idx]);
    }

    switch (sortHeuristic)
    {
    case SurfaceAreaLargestFirst:
    {
        SORT_LARGEST_FIRST(sortedIdx[0], sortedIdx[2], sortKeyBySurfaceArea[0], sortKeyBySurfaceArea[2])
        SORT_LARGEST_FIRST(sortedIdx[1], sortedIdx[3], sortKeyBySurfaceArea[1], sortKeyBySurfaceArea[3])
        SORT_LARGEST_FIRST(sortedIdx[0], sortedIdx[1], sortKeyBySurfaceArea[0], sortKeyBySurfaceArea[1])
        SORT_LARGEST_FIRST(sortedIdx[2], sortedIdx[3], sortKeyBySurfaceArea[2], sortKeyBySurfaceArea[3])
        SORT_LARGEST_FIRST(sortedIdx[1], sortedIdx[2], sortKeyBySurfaceArea[1], sortKeyBySurfaceArea[2])
    }
    break;

    case SurfaceAreaSmallestFirst:
    {
        SORT_SMALLEST_FIRST(sortedIdx[0], sortedIdx[2], sortKeyBySurfaceArea[0], sortKeyBySurfaceArea[2])
        SORT_SMALLEST_FIRST(sortedIdx[1], sortedIdx[3], sortKeyBySurfaceArea[1], sortKeyBySurfaceArea[3])
        SORT_SMALLEST_FIRST(sortedIdx[0], sortedIdx[1], sortKeyBySurfaceArea[0], sortKeyBySurfaceArea[1])
        SORT_SMALLEST_FIRST(sortedIdx[2], sortedIdx[3], sortKeyBySurfaceArea[2], sortKeyBySurfaceArea[3])
        SORT_SMALLEST_FIRST(sortedIdx[1], sortedIdx[2], sortKeyBySurfaceArea[1], sortKeyBySurfaceArea[2])
    }
    break;

    case DensityLargestFirst:
    {
        SORT_LARGEST_FIRST(sortedIdx[0], sortedIdx[2], sortKeyByDensity[0], sortKeyByDensity[2])
        SORT_LARGEST_FIRST(sortedIdx[1], sortedIdx[3], sortKeyByDensity[1], sortKeyByDensity[3])
        SORT_LARGEST_FIRST(sortedIdx[0], sortedIdx[1], sortKeyByDensity[0], sortKeyByDensity[1])
        SORT_LARGEST_FIRST(sortedIdx[2], sortedIdx[3], sortKeyByDensity[2], sortKeyByDensity[3])
        SORT_LARGEST_FIRST(sortedIdx[1], sortedIdx[2], sortKeyByDensity[1], sortKeyByDensity[2])
    }
    break;

    case DensitySmallestFirst:
    {
        SORT_SMALLEST_FIRST(sortedIdx[0], sortedIdx[2], sortKeyByDensity[0], sortKeyByDensity[2])
        SORT_SMALLEST_FIRST(sortedIdx[1], sortedIdx[3], sortKeyByDensity[1], sortKeyByDensity[3])
        SORT_SMALLEST_FIRST(sortedIdx[0], sortedIdx[1], sortKeyByDensity[0], sortKeyByDensity[1])
        SORT_SMALLEST_FIRST(sortedIdx[2], sortedIdx[3], sortKeyByDensity[2], sortKeyByDensity[3])
        SORT_SMALLEST_FIRST(sortedIdx[1], sortedIdx[2], sortKeyByDensity[1], sortKeyByDensity[2])
    }
    break;

    default:
        // no sort
        break;
    }

    // sort child node pointers, boxes and node flags using the sorted indices
    uint combinedSortedNodeFlags = 0;
    for (idx = 0; idx < 4; idx++)
    {
        if (sortedIdx[idx] != INVALID_NODE)
        {
            sortedChildNodePtr[idx] = childNodePtr[sortedIdx[idx]];
            sortedBbox[idx]         = bbox[sortedIdx[idx]];
            sortedNodeFlags[idx]    = nodeFlags[sortedIdx[idx]];
            sortedCost[idx]         = cost[sortedIdx[idx]];
        }
        else
        {
            sortedChildNodePtr[idx] = INVALID_NODE;
            sortedCost[idx]         = 0;
        }
    }

    for (idx = 0; idx < 4; idx++)
    {
        childNodePtr[idx] = sortedChildNodePtr[idx];
        bbox[idx]         = sortedBbox[idx];
        nodeFlags[idx]    = sortedNodeFlags[idx];
        cost[idx]         = sortedCost[idx];
    }
}

//=====================================================================================================================
// Currently 2-way sort only support Sort by SurfaceArea, largest fist
// TODO: adding other sort methods here
bool CompareBBoxAndCheckForSwap(
    const BoundingBox bbox0,
    const BoundingBox bbox1)
{
    const float sortKeyBySurfaceArea_0 = ComputeBoxSurfaceArea(bbox0);
    const float sortKeyBySurfaceArea_1 = ComputeBoxSurfaceArea(bbox1);

    // if node 1 is larger than node 0, do the swap
    // otherwise, keep the default node order
    return (sortKeyBySurfaceArea_1 > sortKeyBySurfaceArea_0);
}

//=====================================================================================================================
static uint CreateBoxNodePointer(
    in BuildQbvhArgs args,
    in uint nodeOffsetInBytes,
    in bool writeAsFp16BoxNode)
{
    uint boxNodePointer;

    if (writeAsFp16BoxNode)
    {
        boxNodePointer = PackNodePointer(NODE_TYPE_BOX_FLOAT16, nodeOffsetInBytes);
    }
    else
    {
        boxNodePointer = PackNodePointer(NODE_TYPE_BOX_FLOAT32, nodeOffsetInBytes);
    }

    return boxNodePointer;
}

//=====================================================================================================================
static uint ProcessNode(
    in BuildQbvhArgs      args,             // Build args
    in bool               topLevelBuild,    // Compile time flag indicating whether this is top or bottom level build
    in ScratchNode        scratchNode,      // Scratch node being processed
    in uint               scratchNodeIndex, // Index of the Scratch node in BVH2
    in uint               destIndex,        // Dest index of the node in terms of 64B chunks in DstBuffer
    in uint               parentNodePtr,    // Parent node pointer
    in AccelStructOffsets offsets)          // Header offsets
{
    const uint nodeType   = GetNodeType(scratchNode.type);
    const uint nodeOffset = CalcQbvhInternalNodeOffset(destIndex, true);

    uint nodePointer;

    if (topLevelBuild && IsUserNodeInstance(nodeType))
    {
        nodePointer = WriteInstanceNode(args, scratchNode, scratchNodeIndex, nodeOffset, offsets);
    }

    if ((topLevelBuild == false) && (IsTriangleNode(nodeType) || IsUserNodeProcedural(nodeType)))
    {
        nodePointer = WritePrimitiveNode(args, scratchNode, nodeOffset, offsets);
    }

    if (IsBoxNode(nodeType))
    {
        const bool writeAsFp16BoxNode = ((topLevelBuild == false) && (nodeType == NODE_TYPE_BOX_FLOAT16));
        nodePointer = CreateBoxNodePointer(args, nodeOffset, writeAsFp16BoxNode);
    }

    WriteParentPointer(DstMetadata,
                       args.metadataSizeInBytes,
                       nodePointer,
                       parentNodePtr);

    return nodePointer;
}

//=====================================================================================================================
static void PullUpLeftChildren(
    in    BuildQbvhArgs      args,                   // Build args
    in    uint               numActivePrims,         // Number of active primitives
    in    bool               topLevelBuild,          // Compile time flag indicating whether this is top or bottom level build
    in    bool               use2levelSort,          // Boolean indicating whether to use 2-level sort
    in    bool               doSwapLevel0,           // Boolean indicating whether to swap the level 0 nodes in 2-level sort
    in    bool               use4waySort,            // Boolean indicating whether to use 4-way sort
    in    ScratchNode        node,                   // Parent BVH scratch node
    in    ScratchNode        level0LeftChildNode,    // Level 0 left child BVH scratch node
    in    uint               parentNodePtr,          // Parent QBVH node pointer
    in    AccelStructOffsets offsets,                // Header offsets
    in    uint               numRightChildren,       // Number of children from right BVH node
    in    uint4              intChildDstIdx,         // Destination locations for child nodes
    inout uint               child[4],
    inout BoundingBox        bbox[4],
    inout uint               nodeFlags[4],
    inout uint               numPrimitives[4],
    inout float4             cost)
{
    bool doSwapLevel1Left = false;

    if (IsLeafNode(node.left_or_primIndex_or_instIndex, numActivePrims) == false)
    {
        const ScratchNode c00 =
            FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, level0LeftChildNode.left_or_primIndex_or_instIndex);
        const ScratchNode c01 =
            FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, level0LeftChildNode.right_or_geometryIndex);

        const BoundingBox bb00 = GetScratchNodeBoundingBoxTS(args, topLevelBuild, c00);
        const BoundingBox bb01 = GetScratchNodeBoundingBoxTS(args, topLevelBuild, c01);

        if (use2levelSort)
        {
            // check to see if we need to "swap" the order of the 2 children
            doSwapLevel1Left = CompareBBoxAndCheckForSwap(bb00, bb01);
        }

        if (use4waySort || args.captureChildNumPrimsForRebraid)
        {
            numPrimitives[0] = FetchScratchNodeNumPrimitives(ScratchBuffer,
                                                             args.scratchNodesScratchOffset,
                                                             level0LeftChildNode.left_or_primIndex_or_instIndex,
                                                             IsLeafNode(level0LeftChildNode.left_or_primIndex_or_instIndex,
                                                                        numActivePrims));

            numPrimitives[1] = FetchScratchNodeNumPrimitives(ScratchBuffer,
                                                             args.scratchNodesScratchOffset,
                                                             level0LeftChildNode.right_or_geometryIndex,
                                                             IsLeafNode(level0LeftChildNode.right_or_geometryIndex,
                                                                        numActivePrims));

            cost[0] = IsLeafNode(level0LeftChildNode.left_or_primIndex_or_instIndex, numActivePrims) ?
                      FetchScratchLeafNodeCost(c00) : FetchScratchInternalNodeCost(c00);
            cost[1] = IsLeafNode(level0LeftChildNode.right_or_geometryIndex, numActivePrims) ?
                      FetchScratchLeafNodeCost(c01) : FetchScratchInternalNodeCost(c01);
        }

        const uint idx_c00 = use2levelSort ? ((doSwapLevel0 ? numRightChildren : 0) + (doSwapLevel1Left ? 1 : 0)) : 0;
        const uint idx_c01 = use2levelSort ? ((doSwapLevel0 ? numRightChildren : 0) + (doSwapLevel1Left ? 0 : 1)) : 1;

        const uint c00NodeIndex = level0LeftChildNode.left_or_primIndex_or_instIndex;
        const uint c01NodeIndex = level0LeftChildNode.right_or_geometryIndex;

        child[idx_c00] = ProcessNode(args, topLevelBuild, c00, c00NodeIndex, intChildDstIdx[0], parentNodePtr, offsets);
        child[idx_c01] = ProcessNode(args, topLevelBuild, c01, c01NodeIndex, intChildDstIdx[1], parentNodePtr, offsets);

        bbox[idx_c00] = bb00;
        bbox[idx_c01] = bb01;

        nodeFlags[idx_c00] = ExtractNodeFlagsField(level0LeftChildNode.flags, 0);
        nodeFlags[idx_c01] = ExtractNodeFlagsField(level0LeftChildNode.flags, 1);
    }
    else
    {
        if (use4waySort || args.captureChildNumPrimsForRebraid)
        {
            numPrimitives[0] = FetchScratchNodeNumPrimitives(ScratchBuffer,
                                                             args.scratchNodesScratchOffset,
                                                             node.left_or_primIndex_or_instIndex,
                                                             true);

            cost[0] = FetchScratchNodeCost(ScratchBuffer,
                                           args.scratchNodesScratchOffset,
                                           node.left_or_primIndex_or_instIndex,
                                           true);
        }

        const uint idx_c0 = use2levelSort ? (doSwapLevel0 ? numRightChildren : 0) : 0;

        const uint c0NodeIndex = node.left_or_primIndex_or_instIndex;

        child[idx_c0] = ProcessNode(args, topLevelBuild, level0LeftChildNode, c0NodeIndex, intChildDstIdx[0], parentNodePtr, offsets);

        bbox[idx_c0] = GetScratchNodeBoundingBoxTS(args, topLevelBuild, level0LeftChildNode);

        nodeFlags[idx_c0] = ExtractNodeFlagsField(node.flags, 0);
    }
}

//=====================================================================================================================
static void PullUpRightChildren(
    in    BuildQbvhArgs      args,                  // Build args
    in    uint               numActivePrims,        // Number of active primitives
    in    bool               topLevelBuild,         // Compile time flag indicating whether this is top or bottom level build
    in    bool               use2levelSort,         // Boolean indicating whether to use 2-level sort
    in    bool               doSwapLevel0,          // Boolean indicating whether to swap the level 0 nodes in 2-level sort
    in    bool               use4waySort,           // Boolean indicating whether to use 4-way sort
    in    ScratchNode        node,                  // Parent BVH scratch node
    in    ScratchNode        level0RightChildNode,  // Level 0 right child BVH scratch node
    in    uint               parentNodePtr,         // Parent QBVH node pointer
    in    AccelStructOffsets offsets,               // Header offsets
    in    uint               numLeftChildren,       // Number of children from left BVH node
    in    uint4              intChildDstIdx,        // Destination locations for child nodes
    inout uint               child[4],
    inout BoundingBox        bbox[4],
    inout uint               nodeFlags[4],
    inout uint               numPrimitives[4],
    inout float4             cost)
{
    bool doSwapLevel1Right = false;

    if (IsLeafNode(node.right_or_geometryIndex, numActivePrims) == false)
    {
        const ScratchNode c10 =
            FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, level0RightChildNode.left_or_primIndex_or_instIndex);
        const ScratchNode c11 =
            FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, level0RightChildNode.right_or_geometryIndex);

        const BoundingBox bb10 = GetScratchNodeBoundingBoxTS(args, topLevelBuild, c10);
        const BoundingBox bb11 = GetScratchNodeBoundingBoxTS(args, topLevelBuild, c11);

        if (use2levelSort)
        {
            // check to see if we need to "swap" the order of the 2 children
            doSwapLevel1Right = CompareBBoxAndCheckForSwap(bb10, bb11);
        }

        if (use4waySort || args.captureChildNumPrimsForRebraid)
        {
            numPrimitives[numLeftChildren] =
                FetchScratchNodeNumPrimitives(ScratchBuffer,
                                              args.scratchNodesScratchOffset,
                                              level0RightChildNode.left_or_primIndex_or_instIndex,
                                              IsLeafNode(level0RightChildNode.left_or_primIndex_or_instIndex, numActivePrims));

            numPrimitives[numLeftChildren + 1] =
                FetchScratchNodeNumPrimitives(ScratchBuffer,
                                              args.scratchNodesScratchOffset,
                                              level0RightChildNode.right_or_geometryIndex,
                                              IsLeafNode(level0RightChildNode.right_or_geometryIndex, numActivePrims));

            cost[numLeftChildren] = IsLeafNode(level0RightChildNode.left_or_primIndex_or_instIndex, numActivePrims) ?
                                    FetchScratchLeafNodeCost(c10) : FetchScratchInternalNodeCost(c10);
            cost[numLeftChildren + 1] = IsLeafNode(level0RightChildNode.right_or_geometryIndex, numActivePrims) ?
                                        FetchScratchLeafNodeCost(c11) : FetchScratchInternalNodeCost(c11);
        }

        const uint idx_c10 = use2levelSort ? ((doSwapLevel0 ? 0 : numLeftChildren) + (doSwapLevel1Right ? 1 : 0)) : numLeftChildren;
        const uint idx_c11 = use2levelSort ? ((doSwapLevel0 ? 0 : numLeftChildren) + (doSwapLevel1Right ? 0 : 1)) : (numLeftChildren + 1);

        const uint c10NodeIndex = level0RightChildNode.left_or_primIndex_or_instIndex;
        const uint c11NodeIndex = level0RightChildNode.right_or_geometryIndex;

        child[idx_c10] = ProcessNode(args, topLevelBuild, c10, c10NodeIndex, intChildDstIdx[2], parentNodePtr, offsets);
        child[idx_c11] = ProcessNode(args, topLevelBuild, c11, c11NodeIndex, intChildDstIdx[3], parentNodePtr, offsets);

        bbox[idx_c10] = bb10;
        bbox[idx_c11] = bb11;

        nodeFlags[idx_c10] = ExtractNodeFlagsField(level0RightChildNode.flags, 0);
        nodeFlags[idx_c11] = ExtractNodeFlagsField(level0RightChildNode.flags, 1);
    }
    else
    {
        if (use4waySort || args.captureChildNumPrimsForRebraid)
        {
            numPrimitives[numLeftChildren] = FetchScratchNodeNumPrimitives(ScratchBuffer,
                                                                           args.scratchNodesScratchOffset,
                                                                           node.right_or_geometryIndex,
                                                                           true);

            cost[numLeftChildren] = FetchScratchNodeCost(ScratchBuffer, args.scratchNodesScratchOffset, node.right_or_geometryIndex, true);
        }

        const uint idx_c1 = use2levelSort ? (doSwapLevel0 ? 0 : numLeftChildren) : numLeftChildren;

        const uint c1NodeIndex = node.right_or_geometryIndex;

        child[idx_c1] = ProcessNode(args, topLevelBuild, level0RightChildNode, c1NodeIndex, intChildDstIdx[2], parentNodePtr, offsets);

        bbox[idx_c1] = GetScratchNodeBoundingBoxTS(args, topLevelBuild, level0RightChildNode);

        nodeFlags[idx_c1] = ExtractNodeFlagsField(node.flags, 1);
    }
}

//=====================================================================================================================
static void PullUpChildren(
    in    BuildQbvhArgs           args,
    in    bool                    topLevelBuild,
    in    bool                    use4waySort,
    in    uint                    qbvhNodePtr,
    in    AccelStructOffsets      offsets,
    in    uint4                   nodeIdx,
    in    uint4                   dstIdx,
    inout uint                    child[4],
    inout BoundingBox             bbox[4],
    inout uint                    flags[4],
    inout uint                    numPrimitives[4],
    inout float4                  cost,
    in    uint                    numActivePrims)
{
    [unroll]
    for (uint i = 0; i < 4; i++)
    {
        const uint primIndex = nodeIdx[i];

        if (primIndex != INVALID_IDX)
        {
            if (use4waySort || args.captureChildNumPrimsForRebraid)
            {
                numPrimitives[i] = FetchScratchNodeNumPrimitives(ScratchBuffer,
                                                                 args.scratchNodesScratchOffset,
                                                                 primIndex,
                                                                 IsLeafNode(primIndex, numActivePrims));

                cost[i] = FetchScratchNodeCost(ScratchBuffer,
                                               args.scratchNodesScratchOffset,
                                               primIndex,
                                               IsLeafNode(primIndex, numActivePrims));
            }

            const ScratchNode n = FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, primIndex);
            child[i]            = ProcessNode(args, topLevelBuild, n, primIndex, dstIdx[i], qbvhNodePtr, offsets);
            bbox[i]             = GetScratchNodeBoundingBoxTS(args, topLevelBuild, n);
            flags[i]            = CalcNodeFlags(n);
        }
    }
}

//=====================================================================================================================
// Apply heuristics to choose which 2 nodes to open up in BVH2.
static void ApplyHeuristic(
    in    uint                    localId,
    in    BuildQbvhArgs           args,
    in    uint                    numPrims,
    in    ScratchNode             node,
    inout uint4                   nodeIdxFinal,
    inout uint                    info,
    inout uint4                   dstIdx,
    inout uint                    count64B)
{
    // uint4 nodeIdxToStore
    const uint baseLdsOffset = (localId * LDS_CHILD_NODE_COUNT);
    SharedMem[baseLdsOffset + 0] = node.left_or_primIndex_or_instIndex;
    SharedMem[baseLdsOffset + 1] = node.right_or_geometryIndex;
    SharedMem[baseLdsOffset + 2] = INVALID_IDX;
    SharedMem[baseLdsOffset + 3] = INVALID_IDX;

    uint nodeCounter = 2;

    for (uint i = 0; i < 2; i++)
    {
        float surfaceAreaMax = 0.0;
        uint nodeIdxToOpen   = INVALID_IDX;

        // Pick the largest surface area node.
        for (uint j = 0; j < nodeCounter; j++)
        {
            if (IsLeafNode(SharedMem[baseLdsOffset + j], numPrims) == false)
            {
                const ScratchNode candidateNode =
                    FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, SharedMem[baseLdsOffset + j]);

                const float surfaceArea = candidateNode.sah_or_v2_or_instBasePtr.y;

                // Pick the node with larger surface area. If the nodes have identical surface area, pick the
                // last node for opening.
                if (surfaceArea >= surfaceAreaMax)
                {
                    surfaceAreaMax = surfaceArea;
                    nodeIdxToOpen  = j;
                }
            }
        }

        if (nodeIdxToOpen == INVALID_IDX)
        {
            break;
        }

        const ScratchNode candidateNode =
            FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, SharedMem[baseLdsOffset + nodeIdxToOpen]);

        SharedMem[baseLdsOffset + nodeIdxToOpen] = candidateNode.left_or_primIndex_or_instIndex;
        SharedMem[baseLdsOffset + nodeCounter] = candidateNode.right_or_geometryIndex;
        nodeCounter++;
    }

    for (i = 0; i < nodeCounter; i++)
    {
        nodeIdxFinal[i] = SharedMem[baseLdsOffset + i];
        dstIdx[i]       = count64B;

        const ScratchNode c = FetchScratchNode(ScratchBuffer, args.scratchNodesScratchOffset, SharedMem[baseLdsOffset + i]);
        info               |= (IsBoxNode(GetNodeType(c.type)) ? (1u << i) : 0);
        count64B           += GetNum64BChunks(args, c, numPrims);
    }
}

//=====================================================================================================================
void BuildQbvhImpl(
    uint          globalId,
    uint          localId,
    uint          numActivePrims,
    BuildQbvhArgs args,
    bool          topLevelBuild)  // Compile time flag indicating whether this is top or bottom level build
{
    // Load acceleration structure header
    const AccelStructHeader  header  = DstBuffer.Load<AccelStructHeader>(0);
    const AccelStructOffsets offsets = header.offsets;

    const ScratchNodeResourceInfo resourceInfo =
    {
        ScratchBuffer,
        args.scratchNodesScratchOffset
    };

    if (globalId == 0)
    {
        // Root node begins after the acceleration structure header and is always of type fp32
        // regardless of mode for fp16 box nodes
        const uint        rootNodePtr     = CreateRootNodePointer();
        const ScratchNode rootScratchNode = FetchScratchNodeImpl(resourceInfo, 0);
        const BoundingBox bbox            = GetScratchNodeBoundingBoxTS(args, topLevelBuild, rootScratchNode);

        DstBuffer.Store3(ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET, asuint(bbox.min));
        DstBuffer.Store3(ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET + 12, asuint(bbox.max));

        // Merge the internal node flags
        const uint flags0 = ExtractNodeFlagsField(rootScratchNode.flags, 0);
        const uint flags1 = ExtractNodeFlagsField(rootScratchNode.flags, 1);

        const uint mergedNodeFlags = flags0 & flags1;
        DstBuffer.Store(ACCEL_STRUCT_HEADER_NODE_FLAGS_OFFSET, mergedNodeFlags);

        WriteParentPointer(DstMetadata,
                           args.metadataSizeInBytes,
                           rootNodePtr,
                           INVALID_IDX);

        // Generate box node with a leaf reference in child index 0 for single primitive acceleration structure
        if (args.numPrimitives == 1)
        {
            uint destIndex = 0;
            const uint childNodePtr =
                ProcessNode(args, topLevelBuild, rootScratchNode, 0, destIndex, rootNodePtr, offsets);

            const uint numPrimitives = 1;

            Float32BoxNode fp32BoxNode = (Float32BoxNode)0;
            fp32BoxNode.child0        = childNodePtr;
            fp32BoxNode.child1        = INVALID_IDX;
            fp32BoxNode.child2        = INVALID_IDX;
            fp32BoxNode.child3        = INVALID_IDX;
            fp32BoxNode.bbox0_min     = bbox.min;
            fp32BoxNode.bbox0_max     = bbox.max;
            fp32BoxNode.flags         = mergedNodeFlags;
            fp32BoxNode.numPrimitives = numPrimitives;

            const uint qbvhNodeAddr = CalcQbvhInternalNodeOffset(0, true);

            {
                WriteBoxNode(DstBuffer, qbvhNodeAddr, fp32BoxNode);
            }

            DstBuffer.Store(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET, 1);

            if (args.enableSAHCost)
            {
                const float4 childCosts = float4(0, 0, 0, 0);
                DstBuffer.Store<float4>(ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET, childCosts);
            }
            else
            {
                DstBuffer.Store4(ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET, uint4(numPrimitives, 0, 0, 0));
            }

            return;
        }
    }

    // Each stack item stores data for writes to linear QBVH memory, indexed by stack index
    const uint stackIndex = globalId;

    // Skip threads that do not map to valid internal nodes
    if (stackIndex >= CalcNumQBVHInternalNodes(args.numPrimitives))
    {
        return;
    }

    const bool use4waySort   = ((args.bvhBuilderNodeSortType == FourWaySortOnBoth) ||
                                (topLevelBuild ?
                                 (args.bvhBuilderNodeSortType == FourWaySortOnTLAS) :
                                 (args.bvhBuilderNodeSortType == FourWaySortOnBLAS)));
    const bool use2levelSort = ((args.bvhBuilderNodeSortType == TwoLevelSortOnBoth) ||
                                (topLevelBuild ?
                                 (args.bvhBuilderNodeSortType == TwoLevelSortOnTLAS) :
                                 (args.bvhBuilderNodeSortType == TwoLevelSortOnBLAS)));

    // This isn't necessary, but not using a boolean seems to cause a shader hang. See comment at the end of
    // while loop
    bool isDone = false;

    while (1)
    {
        DeviceMemoryBarrier();

        // Fetch leaf done count
        const uint numLeafsDone = ScratchBuffer.Load(args.stackPtrsScratchOffset + STACK_PTRS_NUM_LEAFS_DONE_OFFSET);

        // Check if we've processed all leaves or have internal nodes on the stack
        if ((numLeafsDone >= args.numPrimitives) || isDone)
        {
            break;
        }

        // Pop a node to process from the stack.
        const uint bvhNodeSrcIdx = QBVHStackPopNodeIdx(args, topLevelBuild, stackIndex);

        // If we have a valid node on the stack, process node
        if (bvhNodeSrcIdx != INVALID_IDX)
        {
            // Fetch BVH2 node
            const ScratchNode node = FetchScratchNodeImpl(resourceInfo, bvhNodeSrcIdx);

            // Declare child nodes so they can be fetched at most once in this code block and reused later.
            // This can be done because the logic checking which grandchildren are valid is the same.
            // TODO: storing all 4 grandchildren seems to use 152 VGPRs, which TDRs on prototype
            ScratchNode c0;
            ScratchNode c1;

            // Pre-allocated locations for our children using their sizes
            uint4 intChildDstIdx  = { 0, 0, 0, 0 };

            // Indices of the nodes in BVH2 that will be the new children in QBVH
            uint4 intChildNodeIdx = { INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX };

            // Fetch and count children, keeping track of their types
            if (IsLeafNode(bvhNodeSrcIdx, numActivePrims) == false)
            {
                // Compressed child bit mask
                uint  compChildInfo    = 0;

                // Total number of 64 byte destination chunks to allocate for the children
                uint  childDstCount64B = 0;

                if (args.sahQbvh)
                {
                    // Apply Surface Area Heuristic to select which 2 nodes to open up in BVH2.
                    // Selectd nodes can be the children or grandchildren of the currently processed node.
                    // Selectd nodes' children will be pulled up to be the new children in the QBVH.
                    ApplyHeuristic(
                        localId,
                        args,
                        numActivePrims,
                        node,
                        intChildNodeIdx,
                        compChildInfo,
                        intChildDstIdx,
                        childDstCount64B);
                }
                else
                {
                    // If this is an internal node, we need to fetch its child nodes
                    c0 = FetchScratchNodeImpl(resourceInfo, node.left_or_primIndex_or_instIndex);
                    c1 = FetchScratchNodeImpl(resourceInfo, node.right_or_geometryIndex);

                    if (IsLeafNode(node.left_or_primIndex_or_instIndex, numActivePrims) == false)
                    {
                        const ScratchNode c00 = FetchScratchNodeImpl(resourceInfo, c0.left_or_primIndex_or_instIndex);
                        const ScratchNode c01 = FetchScratchNodeImpl(resourceInfo, c0.right_or_geometryIndex);

                        intChildNodeIdx[0] = c0.left_or_primIndex_or_instIndex;
                        intChildNodeIdx[1] = c0.right_or_geometryIndex;

                        compChildInfo |= (IsBoxNode(c00.type) ? (1u << 0) : 0);
                        compChildInfo |= (IsBoxNode(c01.type) ? (1u << 1) : 0);

                        intChildDstIdx[0] = childDstCount64B;
                        childDstCount64B += GetNum64BChunks(args, c00, numActivePrims);
                        intChildDstIdx[1] = childDstCount64B;
                        childDstCount64B += GetNum64BChunks(args, c01, numActivePrims);
                    }
                    else
                    {
                        intChildDstIdx[0] = childDstCount64B;
                        childDstCount64B += GetNum64BChunks(args, c0, numActivePrims);
                    }

                    if (IsLeafNode(node.right_or_geometryIndex, numActivePrims) == false)
                    {
                        const ScratchNode c10 = FetchScratchNodeImpl(resourceInfo, c1.left_or_primIndex_or_instIndex);
                        const ScratchNode c11 = FetchScratchNodeImpl(resourceInfo, c1.right_or_geometryIndex);

                        intChildNodeIdx[2] = c1.left_or_primIndex_or_instIndex;
                        intChildNodeIdx[3] = c1.right_or_geometryIndex;

                        compChildInfo |= (IsBoxNode(c10.type) ? (1u << 2) : 0);
                        compChildInfo |= (IsBoxNode(c11.type) ? (1u << 3) : 0);

                        intChildDstIdx[2] = childDstCount64B;
                        childDstCount64B += GetNum64BChunks(args, c10, numActivePrims);
                        intChildDstIdx[3] = childDstCount64B;
                        childDstCount64B += GetNum64BChunks(args, c11, numActivePrims);
                    }
                    else
                    {
                        intChildDstIdx[2] = childDstCount64B;
                        childDstCount64B += GetNum64BChunks(args, c1, numActivePrims);
                    }
                }

                const uint origStackIdx = AllocQBVHStackNumItems(args,
                                                                 topLevelBuild,
                                                                 compChildInfo,
                                                                 intChildNodeIdx,
                                                                 intChildDstIdx,
                                                                 childDstCount64B);
            }

            // Each stack writes to linear QBVH memory indexed by stack index
            const uint qbvhNodeType           = GetNodeType(node.type);
            const bool writeAsFp16BoxNode     = (args.fp16BoxNodesInBlasMode != NO_NODES_IN_BLAS_AS_FP16) &&
                                                (qbvhNodeType == NODE_TYPE_BOX_FLOAT16);
            const uint bvhNodeDstIdx          = QBVHStackPopDestIdx(args, topLevelBuild, stackIndex);
            const uint qbvhNodeAddr           = CalcQbvhInternalNodeOffset(bvhNodeDstIdx, true);
            const uint qbvhNodePtr            = CreateBoxNodePointer(args, qbvhNodeAddr, writeAsFp16BoxNode);

            const uint nodeTypeToAccum =
                   (writeAsFp16BoxNode ? ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP16_NODES_OFFSET
                                       : ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET);

            DstBuffer.InterlockedAdd(nodeTypeToAccum, 1);

            bool doSwapLevel0 = false;
            if (use2levelSort && !args.sahQbvh)
            {
                // check to see if we need to "swap" the order of the 2 children
                doSwapLevel0 = CompareBBoxAndCheckForSwap(GetScratchNodeBoundingBoxTS(args, topLevelBuild, c0),
                                                          GetScratchNodeBoundingBoxTS(args, topLevelBuild, c1));
            }

            // Temporary child node pointers and bounds
            uint        child[4]     = { INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX };
            BoundingBox bbox[4]      = { (BoundingBox)0, (BoundingBox)0, (BoundingBox)0, (BoundingBox)0 };
            uint        nodeFlags[4] = { 0xff, 0xff, 0xff, 0xff};

            // Temporary data used for four-way sort
            uint numPrimitives[4] = { 0, 0, 0, 0 };

            float4 cost = float4(0, 0, 0, 0);

            if (args.sahQbvh)
            {
                // Pull up the nodes chosen by SAH to be the new children in QBVH
                PullUpChildren(args,
                               topLevelBuild,
                               use4waySort,
                               qbvhNodePtr,
                               offsets,
                               intChildNodeIdx,
                               intChildDstIdx,
                               child,
                               bbox,
                               nodeFlags,
                               numPrimitives,
                               cost,
                               numActivePrims);
            }
            else
            {
                // Needed if swapping level-0 nodes in two-level sort
                const uint numRightChildren = IsLeafNode(node.right_or_geometryIndex, numActivePrims) ? 1 : 2;

                // Fetch next level children and pull them into the QBVH
                PullUpLeftChildren(args,
                                   numActivePrims,
                                   topLevelBuild,
                                   use2levelSort,
                                   doSwapLevel0,
                                   use4waySort,
                                   node,
                                   c0,
                                   qbvhNodePtr,
                                   offsets,
                                   numRightChildren,
                                   intChildDstIdx,
                                   child,
                                   bbox,
                                   nodeFlags,
                                   numPrimitives,
                                   cost);

                const uint numLeftChildren = IsLeafNode(node.left_or_primIndex_or_instIndex, numActivePrims) ? 1 : 2;

                // Fetch next level children and pull them into the QBVH
                PullUpRightChildren(args,
                                    numActivePrims,
                                    topLevelBuild,
                                    use2levelSort,
                                    doSwapLevel0,
                                    use4waySort,
                                    node,
                                    c1,
                                    qbvhNodePtr,
                                    offsets,
                                    numLeftChildren,
                                    intChildDstIdx,
                                    child,
                                    bbox,
                                    nodeFlags,
                                    numPrimitives,
                                    cost);
            }

            if (use4waySort)
            {
                SortChildren(args, numPrimitives, child, bbox, nodeFlags, cost);
            }

            // Combine the node flags
            uint combinedNodeFlags = 0;
            for (int idx = 0; idx < 4; idx++)
            {
                combinedNodeFlags = SetNodeFlagsField(combinedNodeFlags, nodeFlags[idx], idx);
            }

            if (writeAsFp16BoxNode)
            {
                Float16BoxNode fp16BoxNode = (Float16BoxNode)0;
                fp16BoxNode.child0 = child[0];
                fp16BoxNode.child1 = child[1];
                fp16BoxNode.child2 = child[2];
                fp16BoxNode.child3 = child[3];
                fp16BoxNode.bbox0  = CompressBBoxToUint3(bbox[0]);
                fp16BoxNode.bbox1  = CompressBBoxToUint3(bbox[1]);
                fp16BoxNode.bbox2  = CompressBBoxToUint3(bbox[2]);
                fp16BoxNode.bbox3  = CompressBBoxToUint3(bbox[3]);

                WriteFp16BoxNode(DstBuffer, qbvhNodeAddr, fp16BoxNode);
            }
            else
            {
                Float32BoxNode fp32BoxNode = (Float32BoxNode)0;
                fp32BoxNode.child0         = child[0];
                fp32BoxNode.child1         = child[1];
                fp32BoxNode.child2         = child[2];
                fp32BoxNode.child3         = child[3];
                fp32BoxNode.bbox0_min      = bbox[0].min;
                fp32BoxNode.bbox0_max      = bbox[0].max;
                fp32BoxNode.bbox1_min      = bbox[1].min;
                fp32BoxNode.bbox1_max      = bbox[1].max;
                fp32BoxNode.bbox2_min      = bbox[2].min;
                fp32BoxNode.bbox2_max      = bbox[2].max;
                fp32BoxNode.bbox3_min      = bbox[3].min;
                fp32BoxNode.bbox3_max      = bbox[3].max;
                fp32BoxNode.flags          = combinedNodeFlags;
                fp32BoxNode.numPrimitives  = FetchScratchNodeNumPrimitives(node, false);

                {
                    WriteBoxNode(DstBuffer, qbvhNodeAddr, fp32BoxNode);
                }
            }

            // Handle root node specific data
            if ((bvhNodeSrcIdx == 0) && args.captureChildNumPrimsForRebraid)
            {
                // store the actual SAH cost rather than the number of primitives
                if (args.enableSAHCost)
                {
                    DstBuffer.Store<float>(ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET, cost[0]);
                    DstBuffer.Store<float>(ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 4, cost[1]);
                    DstBuffer.Store<float>(ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 8, cost[2]);
                    DstBuffer.Store<float>(ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 12, cost[3]);
                }
                else
                {
                    DstBuffer.Store(ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET, numPrimitives[0]);
                    DstBuffer.Store(ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 4, numPrimitives[1]);
                    DstBuffer.Store(ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 8, numPrimitives[2]);
                    DstBuffer.Store(ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + 12, numPrimitives[3]);
                }
            }

            // Converting this to an explict 'break' causes an infinite loop.
            isDone = true;
        }
    }
}

#if NO_SHADER_ENTRYPOINT == 0
//=====================================================================================================================
// Main Function : BuildQBVH
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void BuildQBVH(
    uint globalIdIn : SV_DispatchThreadID,
    uint groupIdIn  : SV_GroupID,
    uint localIdIn  : SV_GroupThreadID)
{
    uint globalId = globalIdIn;
    uint localId  = localIdIn;
    uint groupId  = groupIdIn;

    const AccelStructHeader header        = DstBuffer.Load<AccelStructHeader>(0);
    const uint              type          = (header.info & ACCEL_STRUCT_HEADER_INFO_TYPE_MASK);
    const bool              topLevelBuild = (type == TOP_LEVEL);

    BuildQbvhArgs args = (BuildQbvhArgs)ShaderConstants;

    // Read active primitive count from header
    args.numPrimitives = header.numActivePrims;

    // With pair compresssion enabled, the leaf node count in header represents actual valid
    // primitives
    if (EnableLatePairCompression(ShaderConstants.triangleCompressionMode,
                                  topLevelBuild,
                                  ShaderConstants.enableEarlyPairCompression))
    {
        args.numPrimitives = header.numLeafNodes;
    }

    uint numTasksWait = 0;
    uint waveId       = 0;

    INIT_TASK;

    BEGIN_TASK(RoundUpQuotient(CalcNumQBVHInternalNodes(args.numPrimitives), BUILD_THREADGROUP_SIZE));

    BuildQbvhImpl(globalId,
                  localId,
                  header.numActivePrims,
                  args,
                  topLevelBuild);

    END_TASK(RoundUpQuotient(CalcNumQBVHInternalNodes(args.numPrimitives), BUILD_THREADGROUP_SIZE));

    if (topLevelBuild)
    {
        BEGIN_TASK(1);

        if ((globalId == 0) && (header.numActivePrims == 0))
        {
            // This is an empty TLAS, but we didn't know it yet when we were setting up the header writes in the
            // command buffer. Overwrite the GPU VA to 0 to properly designate the TLAS as empty.
            DstMetadata.Store<GpuVirtualAddress>(ACCEL_STRUCT_METADATA_VA_LO_OFFSET, 0);
        }

        END_TASK(1);
    }

    BEGIN_TASK(1);

    if (localId == 0)
    {
        WriteCompactedSize(DstBuffer,
                           EmitBuffer,
                           ShaderConstants.emitCompactSize,
                           type);
    }

    END_TASK(1);
}

#include "BuildQBVHCollapseImpl.hlsl"

//=====================================================================================================================
// Main Function : BuildQBVHCollapse
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void BuildQBVHCollapse(
    uint globalIdIn : SV_DispatchThreadID,
    uint groupIdIn  : SV_GroupID,
    uint localIdIn  : SV_GroupThreadID)
{
    uint globalId = globalIdIn;
    uint localId = localIdIn;
    uint groupId = groupIdIn;

    const AccelStructHeader header        = DstBuffer.Load<AccelStructHeader>(0);
    const uint              type          = (header.info & ACCEL_STRUCT_HEADER_INFO_TYPE_MASK);
    const bool              topLevelBuild = (type == TOP_LEVEL);

    BuildQbvhArgs args = (BuildQbvhArgs)ShaderConstants;

    // Read active primitive count from header
    args.numPrimitives = header.numActivePrims;

    // With pair compresssion enabled, the leaf node count in header represents actual valid
    // primitives
    if (EnableLatePairCompression(ShaderConstants.triangleCompressionMode,
                                  topLevelBuild,
                                  ShaderConstants.enableEarlyPairCompression))
    {
        args.numPrimitives = header.numLeafNodes;
    }

    uint numTasksWait = 0;
    uint waveId       = 0;

    INIT_TASK;

    BEGIN_TASK(RoundUpQuotient(CalcNumQBVHInternalNodes(args.numPrimitives), BUILD_THREADGROUP_SIZE));

    BuildQbvhCollapseImpl(globalId, header.numActivePrims, args);

    END_TASK(RoundUpQuotient(CalcNumQBVHInternalNodes(args.numPrimitives), BUILD_THREADGROUP_SIZE));

    BEGIN_TASK(1);

    if (localId == 0)
    {
        WriteCompactedSize(DstBuffer,
                           EmitBuffer,
                           ShaderConstants.emitCompactSize,
                           BOTTOM_LEVEL);
    }

    END_TASK(1);
}
#endif
