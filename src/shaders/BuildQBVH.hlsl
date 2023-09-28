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
    uint splitBoxesByteOffset;
    uint emitCompactSize;
    uint encodeArrayOfPointers;
    uint rebraidEnabled;
    uint enableFusedInstanceNode;
    uint enableFastLBVH;                // Enable the Fast LBVH path
    uint fastLBVHRootNodeIndex;
    uint enableSAHCost;
    uint enableEarlyPairCompression;
    uint unsortedBvhLeafNodesOffset;
    uint reservedUint0;
    uint reservedUint1;
};

#if NO_SHADER_ENTRYPOINT == 0
#define RootSig "RootConstants(num32BitConstants=1, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "CBV(b1),"\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b255),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL)"

#include "../shared/rayTracingDefs.h"

struct RootConstants
{
    uint numThreads;
};

[[vk::push_constant]] ConstantBuffer<RootConstants> ShaderRootConstants    : register(b0);
[[vk::binding(1, 1)]] ConstantBuffer<BuildShaderConstants> ShaderConstants : register(b1);

[[vk::binding(0, 0)]] globallycoherent RWByteAddressBuffer  DstBuffer          : register(u0);
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer  DstMetadata        : register(u1);
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer  ScratchBuffer      : register(u2);
[[vk::binding(3, 0)]]                  RWByteAddressBuffer  InstanceDescBuffer : register(u3);
[[vk::binding(4, 0)]]                  RWByteAddressBuffer  EmitBuffer         : register(u4);

// unused buffer
[[vk::binding(5, 0)]] RWByteAddressBuffer                   SrcBuffer          : register(u5);

#include "Common.hlsl"
#include "IntersectCommon.hlsl"
#include "BuildCommonScratch.hlsl"
#include "CompactCommon.hlsl"

// BVH 8 requires up to 8 LDS elements per thread
#define MAX_ELEMENTS_PER_THREAD 8
#define MAX_LDS_ELEMENTS_PER_THREADGROUP (MAX_ELEMENTS_PER_THREAD * BUILD_THREADGROUP_SIZE)
groupshared uint SharedMem[MAX_LDS_ELEMENTS_PER_THREADGROUP];

//=====================================================================================================================
static uint CalculateBvhNodesOffset(
    uint numActivePrims)
{
    return GetBvhNodesOffset(
               numActivePrims,
               ShaderConstants.numLeafNodes,
               ShaderConstants.offsets.bvhNodeData,
               ShaderConstants.offsets.bvhLeafNodeData,
               ShaderConstants.offsets.primIndicesSorted);
}

//=====================================================================================================================
BuildQbvhArgs GetBuildQbvhArgs()
{
    const AccelStructHeader header = DstBuffer.Load<AccelStructHeader>(0);

    BuildQbvhArgs qbvhArgs;

    const uint rootNodeIndex = FetchRootNodeIndex(
        Settings.enableFastLBVH, ShaderConstants.offsets.fastLBVHRootNodeIndex);

    qbvhArgs.numPrimitives               = ShaderConstants.numPrimitives;
    qbvhArgs.metadataSizeInBytes         = DstMetadata.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET);
    qbvhArgs.numThreads                  = ShaderRootConstants.numThreads;
    qbvhArgs.qbvhStackScratchOffset      = ShaderConstants.offsets.qbvhGlobalStack;
    qbvhArgs.stackPtrsScratchOffset      = ShaderConstants.offsets.qbvhGlobalStackPtrs;
    qbvhArgs.splitBoxesByteOffset        = ShaderConstants.offsets.triangleSplitBoxes;
    qbvhArgs.triangleCompressionMode     = Settings.triangleCompressionMode;
    qbvhArgs.fp16BoxNodesInBlasMode      = Settings.fp16BoxNodesMode;
    qbvhArgs.encodeArrayOfPointers       = ShaderConstants.encodeArrayOfPointers;
    qbvhArgs.rebraidEnabled              = (Settings.rebraidType != RebraidType::Off);
    qbvhArgs.enableFusedInstanceNode     = Settings.enableFusedInstanceNode;
    qbvhArgs.enableFastLBVH              = Settings.enableFastLBVH;
    qbvhArgs.fastLBVHRootNodeIndex       = rootNodeIndex;
    qbvhArgs.enableEarlyPairCompression  = Settings.enableEarlyPairCompression;
    qbvhArgs.unsortedBvhLeafNodesOffset  = ShaderConstants.offsets.bvhLeafNodeData;
    qbvhArgs.enableSAHCost               = Settings.enableSAHCost;

    qbvhArgs.scratchNodesScratchOffset = CalculateBvhNodesOffset(header.numActivePrims);

    // Read active primitive count from header
    qbvhArgs.numPrimitives = header.numActivePrims;

    // With pair compresssion enabled, the leaf node count in header represents actual valid
    // primitives
    if (EnableLatePairCompression())
    {
        qbvhArgs.numPrimitives = header.numLeafNodes;
    }

    return qbvhArgs;
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
        instanceDesc = FetchInstanceDescAddr(addr);
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

    // When rebraiding is disabled the destination index is just the instance index.
    const uint destIndex = (args.rebraidEnabled != 0) ? (scratchNodeIndex - args.numPrimitives + 1) : instanceIndex;

    {
        ScratchBuffer.InterlockedAdd(args.stackPtrsScratchOffset + STACK_PTRS_NUM_LEAFS_DONE_OFFSET, 1);

        {
            nodeOffset =
                offsets.leafNodes + (destIndex * GetBvhNodeSizeInstance(args.enableFusedInstanceNode));
        }
    }

    instanceDesc.accelStructureAddressLo = asuint(scratchNode.sah_or_v2_or_instBasePtr.x);
    instanceDesc.accelStructureAddressHiAndFlags = asuint(scratchNode.sah_or_v2_or_instBasePtr.y);

    const uint nodePointer = PackNodePointer(nodeType, nodeOffset);

    WriteInstanceDescriptor(args.metadataSizeInBytes,
                            instanceDesc,
                            instanceIndex,
                            nodePointer,
                            blasMetadataSize,
                            scratchNode.splitBox_or_nodePointer,
                            ExtractScratchNodeFlags(scratchNode.flags_and_instanceMask),
                            args.enableFusedInstanceNode);

    DstBuffer.Store(offsets.primNodePtrs + (destIndex * sizeof(uint)), nodePointer);

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
    if ((topLevelBuild == false) && IsScratchTriangleNode(node))
    {
        if (Settings.doTriangleSplitting)
        {
            bbox = FetchSplitBoxAtIndex(args.splitBoxesByteOffset, node.splitBox_or_nodePointer);
        }
        else
        {
            bbox = GenerateTriangleBoundingBox(node.bbox_min_or_v0,
                                               node.bbox_max_or_v1,
                                               node.sah_or_v2_or_instBasePtr);

            if ((args.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
                (node.splitBox_or_nodePointer != INVALID_IDX))
            {
                ScratchNode otherNode;
                if (args.enableEarlyPairCompression)
                {
                    otherNode = FetchScratchNode(args.unsortedBvhLeafNodesOffset,
                                                 node.splitBox_or_nodePointer);
                }
                else
                {
                    otherNode = FetchScratchNode(args.scratchNodesScratchOffset,
                                                 node.splitBox_or_nodePointer);
                }

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
bool IsDestIndexInStack(in BuildQbvhArgs args)
{
    const bool destIndexInStack = (args.fp16BoxNodesInBlasMode == LEAF_NODES_IN_BLAS_AS_FP16) ||
                                  (args.fp16BoxNodesInBlasMode == MIXED_NODES_IN_BLAS_AS_FP16);
    return destIndexInStack;
}

//=====================================================================================================================
bool StackHasTwoEntries(in BuildQbvhArgs args)
{
    bool stackHasTwoEntries = IsDestIndexInStack(args);
    return stackHasTwoEntries;
}

//=====================================================================================================================
// Returns source BVH node index given node.
uint StackPopNodeIdx(
    BuildQbvhArgs args,
    uint          stackIndex)
{
    if (StackHasTwoEntries(args))
    {
        stackIndex *= 2;
    }

    return ScratchBuffer.Load(args.qbvhStackScratchOffset + (stackIndex * sizeof(uint)));
}

//=====================================================================================================================
// Returns the second dword on the stack for a given stack index.
uint StackPopSecondEntry(
    BuildQbvhArgs args,
    uint          stackIndex)
{
    return ScratchBuffer.Load(args.qbvhStackScratchOffset + ((2 * stackIndex + 1) * sizeof(uint)));
}

//=====================================================================================================================
// Calculate internal node offset from 64-byte chunk index
uint Calc64BChunkOffset(uint chunkIdx64B)
{
    // Node offset includes header size
    return sizeof(AccelStructHeader) + (chunkIdx64B * 64);
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
static uint GetNum64BChunks(
    in bool isTopLevel,
    in bool isLeafNode,
    in bool isFp16)
{
    {
        if (isLeafNode)
        {
            return 0;
        }
        else if (isFp16)
        {
            return 1;
        }
        else
        {
            return 2;
        }
    }
}

//======================================================================================================================
static void InitBuildQbvhImpl(
    in uint          globalId,
    in BuildQbvhArgs args)
{
    // @note This relies on fp16BoxNodesInBlasMode and collapse build flag being disabled in the shader constants for TLAS

    // Note the first entry is initialized below. Skip initializing it to invalid.
    const uint maxInternalNodeCount = GetNumInternalNodeCount(args.numPrimitives);
    for (uint stackIndex = globalId + 1; stackIndex < maxInternalNodeCount; stackIndex += args.numThreads)
    {
        if (StackHasTwoEntries(args))
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

    if (globalId == 0)
    {
        StackPtrs stackPtrs;
        // Source node index in linear memory
        stackPtrs.stackPtrSrcNodeId = 1;
        // Node destination in linear memory. Counts in 64B chunks.
        stackPtrs.stackPtrNodeDest = 2;

        stackPtrs.numLeafsDone = 0;

        ScratchBuffer.Store<StackPtrs>(args.stackPtrsScratchOffset, stackPtrs);

        const uint32_t rootNodeIndex = args.fastLBVHRootNodeIndex;

        if (StackHasTwoEntries(args))
        {
            ScratchBuffer.Store<uint2>(args.qbvhStackScratchOffset, uint2(rootNodeIndex, 0));
        }
        else
        {
            ScratchBuffer.Store(args.qbvhStackScratchOffset, rootNodeIndex);
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
    const BuildQbvhArgs args = GetBuildQbvhArgs();
    InitBuildQbvhImpl(globalId, args);
}
#endif

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
    const uint geometryIndex = scratchNode.right_or_geometryIndex;
    const uint geometryIndexAndFlags = PackGeometryIndexAndFlags(geometryIndex, geometryFlags);
    const uint geometryPrimNodePtrsOffset = offsets.primNodePtrs + geometryInfo.primNodePtrsOffset;
    const uint flattenedPrimIndex = (geometryInfo.primNodePtrsOffset / sizeof(uint)) + scratchNode.left_or_primIndex_or_instIndex;

    uint numLeafsDone;
    ScratchBuffer.InterlockedAdd(args.stackPtrsScratchOffset + STACK_PTRS_NUM_LEAFS_DONE_OFFSET, 1, numLeafsDone);

    {
        uint destIndex;
        if (IsScratchTriangleNode(scratchNode) &&
            ((args.triangleCompressionMode != NO_TRIANGLE_COMPRESSION) || Settings.doTriangleSplitting))
        {
            destIndex = numLeafsDone;
        }
        else
        {
            destIndex = flattenedPrimIndex;
        }

        const uint primitiveNodeSize = (nodeType == NODE_TYPE_USER_NODE_PROCEDURAL) ?
                                       USER_NODE_PROCEDURAL_SIZE :
                                       TRIANGLE_NODE_SIZE;

        nodeOffset = offsets.leafNodes + (destIndex * primitiveNodeSize);
    }

    const uint triangleId = scratchNode.type >> 3;

    if (nodeType == NODE_TYPE_USER_NODE_PROCEDURAL)
    {
        DstBuffer.Store3(nodeOffset + USER_NODE_PROCEDURAL_MIN_OFFSET, asuint(scratchNode.bbox_min_or_v0));
        DstBuffer.Store3(nodeOffset + USER_NODE_PROCEDURAL_MAX_OFFSET, asuint(scratchNode.bbox_max_or_v1));

        {
            DstBuffer.Store(nodeOffset + USER_NODE_PROCEDURAL_PRIMITIVE_INDEX_OFFSET, scratchNode.left_or_primIndex_or_instIndex);
            DstBuffer.Store(nodeOffset + USER_NODE_PROCEDURAL_GEOMETRY_INDEX_AND_FLAGS_OFFSET, geometryIndexAndFlags);
        }
    }
    else
    {
        DstBuffer.Store(nodeOffset + TRIANGLE_NODE_ID_OFFSET, triangleId);

        const bool isPairCompressed = (args.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
                                      (scratchNode.splitBox_or_nodePointer != INVALID_IDX);

        // Pair compressed triangles nodes are referenced by triangle 1
        nodeType = isPairCompressed ? NODE_TYPE_TRIANGLE_1 : NODE_TYPE_TRIANGLE_0;

        {
            DstBuffer.Store(nodeOffset + TRIANGLE_NODE_GEOMETRY_INDEX_AND_FLAGS_OFFSET, geometryIndexAndFlags);
            DstBuffer.Store(nodeOffset + TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET + (nodeType * 4),
                            scratchNode.left_or_primIndex_or_instIndex);
        }

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

        // For PAIR_TRIANGLE_COMPRESSION, the other node is not linked in the BVH tree, so we need to find it and
        // store it as well if it exists.
        if (isPairCompressed)
        {
            const uint baseScratchLeafNodeOffset =
                args.enableEarlyPairCompression ? args.unsortedBvhLeafNodesOffset : args.scratchNodesScratchOffset;

            const ScratchNode otherNode = FetchScratchNode(baseScratchLeafNodeOffset, scratchNode.splitBox_or_nodePointer);

            const float3 otherVerts[3] = { otherNode.bbox_min_or_v0,
                                           otherNode.bbox_max_or_v1,
                                           otherNode.sah_or_v2_or_instBasePtr };
            const uint3 otherVertexOffsets = CalcTriangleCompressionVertexOffsets(NODE_TYPE_TRIANGLE_0, triangleId);
            for (uint i = 0; i < 3; ++i)
            {
                // Since the other node will always be of type NODE_TYPE_TRIANGLE_0, it is sufficient to store only
                // the vertex that goes into v0. v1, v2, and v3 were already stored by NODE_TYPE_TRIANGLE_1 above.
                if (otherVertexOffsets[i] == 0)
                {
                    DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V0_OFFSET, otherVerts[i]);
                }
            }

            {
                DstBuffer.Store(nodeOffset + TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET,
                                otherNode.left_or_primIndex_or_instIndex);
            }

            const uint otherPrimNodePointer = PackNodePointer(NODE_TYPE_TRIANGLE_0, nodeOffset);
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
// Returns destination offset in memory for a given node.
// The value returned is in terms of 64B, so 1 for fp16 and 2 for fp32 box node
uint GetDestIdx(
    BuildQbvhArgs args,
    bool          topLevelBuild,
    uint          stackIndex)
{
    uint destIndex;

    const uint fp16BoxNodesInBlasMode = args.fp16BoxNodesInBlasMode;
    if (IsDestIndexInStack(args))
    {
        // Collapse should not enter this code path because tasks store destination indexes directly
        destIndex = StackPopSecondEntry(args, stackIndex);
    }
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
    uint          compChildInfo,       ///< flag if child is to be written out
    uint4         intChildNodeIdx,     ///< scratch node index (from parent) for each child node
    inout uint4   intChildDstOffset,   ///< destination address offset for each child node, uses fp16 / fp32 node size
    uint          numDstChunks64B)     ///< number of 64B chunks required for all child nodes
{
    uint numStackItems;
    {
        numStackItems = countbits(compChildInfo & 0xF);
    }

    uint origStackIdx;
    ScratchBuffer.InterlockedAdd(args.stackPtrsScratchOffset + STACK_PTRS_SRC_PTR_OFFSET, numStackItems, origStackIdx);

    // compute proper stack index for each box child node
    const uint4 intChildStackIdx = origStackIdx + uint4(0,
                                                        countbits(compChildInfo & 0x1),
                                                        countbits(compChildInfo & 0x3),
                                                        countbits(compChildInfo & 0x7));

    uint origDest;

    // Uniform nodes - rely on a single stack item per thread
    if (IsDestIndexInStack(args) == false)
    {
        origDest = GetDestIdx(args, topLevelBuild, origStackIdx);

        // Destination offset index for each child node
        intChildDstOffset += origDest;

        // Store node index for every box child node
        for (uint i = 0; i < 4; i++)
        {
            if (compChildInfo & bit(i))
            {
                ScratchBuffer.Store(args.qbvhStackScratchOffset + intChildStackIdx[i] * sizeof(uint),
                                    intChildNodeIdx[i]);
            }
        }

        DeviceMemoryBarrier();
    }
    // Mixed nodes - atomically advance destination memory address to avoid holes between nodes
    else
    {
        ScratchBuffer.InterlockedAdd(args.stackPtrsScratchOffset + STACK_PTRS_DST_PTR_OFFSET, numDstChunks64B, origDest);

        // Destination offset index for each child node
        intChildDstOffset += origDest;

        // Store node index, destination offset index for every box child node
        for (uint i = 0; i < 4; i++)
        {
            if (compChildInfo & bit(i))
            {
                const uint2 toWrite = { intChildNodeIdx[i], intChildDstOffset[i] };
                ScratchBuffer.Store<uint2>(args.qbvhStackScratchOffset + intChildStackIdx[i] * sizeof(uint2),
                                           toWrite);
            }
        }

        DeviceMemoryBarrier();
    }

    return origStackIdx;
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
    const uint nodeType = GetNodeType(scratchNode.type);

    uint nodeOffset = Calc64BChunkOffset(destIndex);
    uint nodePointer;

    if (IsBoxNode(nodeType))
    {
        bool writeAsFp16BoxNode = false;
        {
            writeAsFp16BoxNode = ((topLevelBuild == false) && (nodeType == NODE_TYPE_BOX_FLOAT16));
        }
        nodePointer = CreateBoxNodePointer(nodeOffset, writeAsFp16BoxNode);
    }
    else
    {
        if (topLevelBuild)
        {
            nodePointer = WriteInstanceNode(args,
                                            scratchNode,
                                            scratchNodeIndex,
                                            nodeOffset,
                                            offsets);
        }
        else
        {
            nodePointer = WritePrimitiveNode(args, scratchNode, nodeOffset, offsets);
        }
    }

    WriteParentPointer(args.metadataSizeInBytes,
                       nodePointer,
                       parentNodePtr);

    return nodePointer;
}

//=====================================================================================================================
static void PullUpChildren(
    in    BuildQbvhArgs           args,
    in    bool                    topLevelBuild,
    in    uint                    qbvhNodePtr,
    in    AccelStructOffsets      offsets,
    in    uint4                   nodeIdx,
    in    uint4                   dstIdx,
    inout uint                    child[4],
    inout BoundingBox             bbox[4],
    inout uint                    boxNodeFlags,
    inout uint4                   numPrimitives,
    inout float4                  cost,
    in    uint                    numActivePrims)
{
    [unroll]
    for (uint i = 0; i < 4; i++)
    {
        const uint primIndex = nodeIdx[i];

        if (primIndex != INVALID_IDX)
        {
            bool isLeafNode = IsLeafNode(primIndex, numActivePrims);

            if (Settings.enableInstanceRebraid)
            {
                numPrimitives[i] = FetchScratchNodeNumPrimitives(args.scratchNodesScratchOffset,
                                                                 primIndex,
                                                                 isLeafNode);

                cost[i] = FetchScratchNodeCost(args.scratchNodesScratchOffset,
                                               primIndex,
                                               isLeafNode);
            }

            const ScratchNode n = FetchScratchNode(args.scratchNodesScratchOffset, primIndex);

            child[i] = ProcessNode(args,
                                   topLevelBuild,
                                   n,
                                   primIndex,
                                   dstIdx[i],
                                   qbvhNodePtr,
                                   offsets);

            bbox[i]      = GetScratchNodeBoundingBoxTS(args, topLevelBuild, n);
            boxNodeFlags = SetBoxNodeFlagsField(boxNodeFlags, ExtractScratchNodeFlags(n.flags_and_instanceMask), i);
        }
        else
        {
            // Note, box node flags are combined together by using an AND operation. Thus, we need to initialise
            // invalid child flags as 0xff
            boxNodeFlags = SetBoxNodeFlagsField(boxNodeFlags, 0xff, i);
        }
    }

}

//=====================================================================================================================
// Sorts box node indices first, leaf node indices next and then INVALID_NODE towards the end.
static void SortBoxFirst(
    inout uint nodeIndex0,
    inout uint nodeIndex1)
{
    // During BVH2 build phase, there are (numActivePrims - 1) box nodes, (numActivePrims) leaf nodes in order. So
    // the following sort condition sorts the box node indices first, leaf node indices next, and since INVALID_NODE
    // is the max unsigned int value, it gets sorted last.
    if (nodeIndex0 > nodeIndex1)
    {
        uint t0 = nodeIndex0;
        nodeIndex0 = nodeIndex1;
        nodeIndex1 = t0;
    }
}

//=====================================================================================================================
// Determine whether to allocate FP32 box nodes as FP16
static bool OverrideFp32BoxNodeToFp16(
    in uint        nodeIndex,
    in uint        baseScratchNodesOffset,
    in uint        numActivePrims)
{
    bool writeNodeAsFp16 = false;

    const ScratchNode node = FetchScratchNode(baseScratchNodesOffset, nodeIndex);

    if (Settings.fp16BoxNodesMode == ALL_INTERIOR_NODES_IN_BLAS_AS_FP16)
    {
        writeNodeAsFp16 = true;
    }
    else if (Settings.fp16BoxNodesMode == LEAF_NODES_IN_BLAS_AS_FP16)
    {
        // Mark node as fp16 if the box node is directly attached to a leaf node
        const bool isLeftChildLeaf  = IsLeafNode(node.left_or_primIndex_or_instIndex, numActivePrims);
        const bool isRightChildLeaf = IsLeafNode(node.right_or_geometryIndex, numActivePrims);

        writeNodeAsFp16 = isLeftChildLeaf || isRightChildLeaf;

        // Or any of the box nodes are directly attached to leaf nodes.
        if (isLeftChildLeaf == false)
        {
            const ScratchNode c0 = FetchScratchNode(baseScratchNodesOffset, node.left_or_primIndex_or_instIndex);
            writeNodeAsFp16 |= IsLeafNode(c0.left_or_primIndex_or_instIndex, numActivePrims) ||
                               IsLeafNode(c0.right_or_geometryIndex, numActivePrims);
        }

        if (isRightChildLeaf == false)
        {
            const ScratchNode c1 = FetchScratchNode(baseScratchNodesOffset, node.right_or_geometryIndex);
            writeNodeAsFp16 |= IsLeafNode(c1.left_or_primIndex_or_instIndex, numActivePrims) ||
                               IsLeafNode(c1.right_or_geometryIndex, numActivePrims);
        }
    }
    else if (Settings.fp16BoxNodesMode == MIXED_NODES_IN_BLAS_AS_FP16)
    {
        // Mark node as fp16 if its bounds compress well; i.e. increase in surface area
        // should be < threshold
        const BoundingBox bboxFp32 = { node.bbox_min_or_v0, node.bbox_max_or_v1 };
        const uint3       bboxFp16 = CompressBBoxToUint3(bboxFp32);

        const float saAsFp32 = ComputeBoxSurfaceArea(bboxFp32);
        const float saAsFp16 = ComputeBoxSurfaceArea(bboxFp16);

        const float saAsFp32Scaled = (saAsFp32 * Settings.fp16BoxModeMixedSaThreshold);
        writeNodeAsFp16 = (saAsFp16 < saAsFp32Scaled);
    }

    if (writeNodeAsFp16)
    {
        WriteScratchNodeType(baseScratchNodesOffset,
                             nodeIndex,
                             NODE_TYPE_BOX_FLOAT16);
        // Make sure the node type is written before the node is queued to the stack
        DeviceMemoryBarrier();
    }

    return writeNodeAsFp16;
}

//=====================================================================================================================
// Apply heuristics to choose which 2 nodes to open up in BVH2.
static void ApplyHeuristic(
    in    bool                    topLevelBuild,
    in    uint                    localId,
    in    BuildQbvhArgs           args,
    in    uint                    numActivePrims,
    in    ScratchNode             node,
    inout uint                    info,
    inout uint4                   nodeIdxFinal,
    inout uint4                   dstIdx,
    inout uint                    count64B)
{
    uint baseLdsOffset = (localId * 4); // BVH4 has 4 child nodes.

    SharedMem[baseLdsOffset + 0] = node.left_or_primIndex_or_instIndex;
    SharedMem[baseLdsOffset + 1] = node.right_or_geometryIndex;
    SharedMem[baseLdsOffset + 2] = INVALID_IDX;
    SharedMem[baseLdsOffset + 3] = INVALID_IDX;

    uint nodeCounter = 2;

    uint numIterations = 2; // BVH4 needs 2 iterations to find 4 child nodes to open up.

    for (uint i = 0; i < numIterations; i++)
    {
        float surfaceAreaMax = 0.0;
        uint nodeIdxToOpen   = INVALID_IDX;

        // Pick the largest surface area node.
        for (uint j = 0; j < nodeCounter; j++)
        {
            if (IsLeafNode(SharedMem[baseLdsOffset + j], numActivePrims) == false)
            {
                const ScratchNode candidateNode =
                    FetchScratchNode(args.scratchNodesScratchOffset, SharedMem[baseLdsOffset + j]);

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
            FetchScratchNode(args.scratchNodesScratchOffset, SharedMem[baseLdsOffset + nodeIdxToOpen]);

        SharedMem[baseLdsOffset + nodeIdxToOpen] = candidateNode.left_or_primIndex_or_instIndex;
        SharedMem[baseLdsOffset + nodeCounter] = candidateNode.right_or_geometryIndex;
        nodeCounter++;
    }

    for (uint i = 0; (i < nodeCounter) && (i < 4); i++)
    {
        const uint scratchNodeIdx = SharedMem[baseLdsOffset + i];
        nodeIdxFinal[i] = scratchNodeIdx;
        dstIdx[i] = count64B;

        const bool isLeafNode = IsLeafNode(scratchNodeIdx, numActivePrims);
        info |= (isLeafNode ? 0 : (1u << i));

        bool isFp16 = false;
        if ((Settings.fp16BoxNodesMode != NO_NODES_IN_BLAS_AS_FP16) && (isLeafNode == false))
        {
            isFp16 = OverrideFp32BoxNodeToFp16(scratchNodeIdx, args.scratchNodesScratchOffset, numActivePrims);
        }

        count64B += GetNum64BChunks(topLevelBuild, isLeafNode, isFp16);
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

    if (globalId == 0)
    {
        const uint32_t rootNodeIndex = args.fastLBVHRootNodeIndex;

        // Root node begins after the acceleration structure header and is always of type fp32
        // regardless of mode for fp16 box nodes
        const uint rootNodePtr = CreateRootNodePointer();
        const ScratchNode rootScratchNode = FetchScratchNode(args.scratchNodesScratchOffset, rootNodeIndex);
        const BoundingBox bbox            = GetScratchNodeBoundingBoxTS(args, topLevelBuild, rootScratchNode);

        WriteAccelStructHeaderRootBoundingBox(bbox);

        WriteParentPointer(args.metadataSizeInBytes,
                           rootNodePtr,
                           INVALID_IDX);

        // Generate box node with a leaf reference in child index 0 for single primitive acceleration structure
        if (args.numPrimitives == 1)
        {
            uint destIndex = 0;
            const uint childNodePtr = ProcessNode(args,
                                                  topLevelBuild,
                                                  rootScratchNode,
                                                  rootNodeIndex,
                                                  destIndex,
                                                  rootNodePtr,
                                                  offsets);

            const uint numPrimitives = 1;

            Float32BoxNode fp32BoxNode = (Float32BoxNode)0;
            fp32BoxNode.child0        = childNodePtr;
            fp32BoxNode.child1        = INVALID_IDX;
            fp32BoxNode.child2        = INVALID_IDX;
            fp32BoxNode.child3        = INVALID_IDX;
            fp32BoxNode.bbox0_min     = bbox.min;
            fp32BoxNode.bbox0_max     = bbox.max;
            fp32BoxNode.flags         = ExtractScratchNodeFlags(rootScratchNode.flags_and_instanceMask);
            fp32BoxNode.numPrimitives = numPrimitives;

            const uint qbvhNodeAddr = Calc64BChunkOffset(0);

            {
                WriteBoxNode(args.metadataSizeInBytes + qbvhNodeAddr,
                             fp32BoxNode);
            }

            {
                WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET, 1);
            }

            const uint costOrNumPrims = args.enableSAHCost ? 0 : numPrimitives;
            WriteAccelStructHeaderCostOrNumChildPrims(uint4(costOrNumPrims, 0, 0, 0));

            return;
        }
    }

    // Each stack item stores data for writes to linear QBVH memory, indexed by stack index
    const uint stackIndex = globalId;

    // Skip threads that do not map to valid internal nodes
    if (stackIndex >= GetNumInternalNodeCount(args.numPrimitives))
    {
        return;
    }

    // This isn't necessary, but not using a boolean seems to cause a shader hang. See comment at the end of
    // while loop
    bool isDone = false;
    const uint32_t rootNodeIndex = args.fastLBVHRootNodeIndex;

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
        const uint bvhNodeSrcIdx = StackPopNodeIdx(args, stackIndex);

        // If we have a valid node on the stack, process node
        if (bvhNodeSrcIdx != INVALID_IDX)
        {
            // Fetch BVH2 node
            const ScratchNode node = FetchScratchNode(args.scratchNodesScratchOffset, bvhNodeSrcIdx);

            // Each stack writes to linear QBVH memory indexed by stack index
            const uint qbvhNodeType = GetNodeType(node.type);
            bool writeAsFp16BoxNode = false;
            {
                writeAsFp16BoxNode = (args.fp16BoxNodesInBlasMode != NO_NODES_IN_BLAS_AS_FP16) &&
                                     (qbvhNodeType == NODE_TYPE_BOX_FLOAT16);
            }
            const uint bvhNodeDstIdx = GetDestIdx(args, topLevelBuild, stackIndex);
            const uint qbvhNodeAddr  = Calc64BChunkOffset(bvhNodeDstIdx);
            const uint qbvhNodePtr   = CreateBoxNodePointer(qbvhNodeAddr, writeAsFp16BoxNode);

            const uint nodeTypeToAccum =
                   (writeAsFp16BoxNode ? ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP16_NODES_OFFSET
                                       : ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET);

            {
                IncrementAccelStructHeaderField(nodeTypeToAccum, 1);
            }

            // Pre-allocated locations for our children using their sizes
            uint4 intChildDstIdx  = { 0, 0, 0, 0 };

            // Indices of the nodes in BVH2 that will be the new children in QBVH
            uint4 intChildNodeIdx = { INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX };

            // Compressed child bit mask
            uint  compChildInfo    = 0;

            // Total number of 64 byte destination chunks to allocate for the children
            uint  childDstCount64B = 0;

            // Apply Surface Area Heuristic to select which 2 nodes to open up in BVH2.
            // Selected nodes can be the children or grandchildren of the currently processed node.
            // Selected nodes' children will be pulled up to be the new children in the QBVH.
            ApplyHeuristic(topLevelBuild,
                           localId,
                           args,
                           numActivePrims,
                           node,
                           compChildInfo,
                           intChildNodeIdx,
                           intChildDstIdx,
                           childDstCount64B);

            const uint origStackIdx = AllocQBVHStackNumItems(args,
                                                             topLevelBuild,
                                                             compChildInfo,
                                                             intChildNodeIdx,
                                                             intChildDstIdx,
                                                             childDstCount64B);

            // Temporary child node pointers and bounds
            uint        child[4]     = { INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX };
            BoundingBox bbox[4]      = { (BoundingBox)0, (BoundingBox)0, (BoundingBox)0, (BoundingBox)0 };
            uint        boxNodeFlags = 0;

            // Temporary data used for four-way sort and rebraid
            uint4 numPrimitives = uint4(0, 0, 0, 0);

            float4 cost = float4(0, 0, 0, 0);

            // Pull up the nodes chosen by SAH to be the new children in QBVH
            PullUpChildren(args,
                           topLevelBuild,
                           qbvhNodePtr,
                           offsets,
                           intChildNodeIdx,
                           intChildDstIdx,
                           child,
                           bbox,
                           boxNodeFlags,
                           numPrimitives,
                           cost,
                           numActivePrims);

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

                WriteFp16BoxNode(args.metadataSizeInBytes + qbvhNodeAddr, fp16BoxNode);
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
                fp32BoxNode.flags          = boxNodeFlags;
                fp32BoxNode.numPrimitives  = FetchScratchNodeNumPrimitives(node, false);

                {
                    WriteBoxNode(args.metadataSizeInBytes + qbvhNodeAddr,
                                 fp32BoxNode);
                }
            }

            // Handle root node specific data
            if ((bvhNodeSrcIdx == rootNodeIndex) && Settings.enableInstanceRebraid)
            {
                // TODO: Handle for BVH8
                // store the actual SAH cost rather than the number of primitives
                if (args.enableSAHCost)
                {
                    WriteAccelStructHeaderCostOrNumChildPrims(asuint(cost));
                }
                else
                {
                    WriteAccelStructHeaderCostOrNumChildPrims(numPrimitives);
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
    const bool              topLevelBuild = Settings.topLevelBuild;
    const uint              type          = Settings.topLevelBuild ? TOP_LEVEL : BOTTOM_LEVEL;

    const BuildQbvhArgs args = GetBuildQbvhArgs();

    uint numTasksWait = 0;
    uint waveId       = 0;

    INIT_TASK;

    BEGIN_TASK(RoundUpQuotient(GetNumInternalNodeCount(args.numPrimitives), BUILD_THREADGROUP_SIZE));

    InitBuildQbvhImpl(globalId, args);

    END_TASK(RoundUpQuotient(GetNumInternalNodeCount(args.numPrimitives), BUILD_THREADGROUP_SIZE));

    BEGIN_TASK(RoundUpQuotient(GetNumInternalNodeCount(args.numPrimitives), BUILD_THREADGROUP_SIZE));

    {
        BuildQbvhImpl(globalId, localId, header.numActivePrims, args, topLevelBuild);
    }

    END_TASK(RoundUpQuotient(GetNumInternalNodeCount(args.numPrimitives), BUILD_THREADGROUP_SIZE));

    if (topLevelBuild)
    {
        BEGIN_TASK(1);

        if ((globalId == 0) && (args.numPrimitives == 0))
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
        WriteCompactedSize(Settings.emitCompactSize,
                           type);
    }

    END_TASK(1);
}
#endif
