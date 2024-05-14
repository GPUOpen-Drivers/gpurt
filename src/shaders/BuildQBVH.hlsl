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
#define RootSig "RootConstants(num32BitConstants=1, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "CBV(b1),"\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b255),"\
                "UAV(u6, visibility=SHADER_VISIBILITY_ALL)"

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
[[vk::binding(3, 0)]] globallycoherent RWByteAddressBuffer  ScratchGlobal      : register(u3);
[[vk::binding(4, 0)]]                  RWByteAddressBuffer  InstanceDescBuffer : register(u4);
[[vk::binding(5, 0)]]                  RWByteAddressBuffer  EmitBuffer         : register(u5);

// unused buffer
[[vk::binding(6, 0)]] RWByteAddressBuffer                   SrcBuffer          : register(u6);

#define TASK_COUNTER_BUFFER   ScratchGlobal
#define TASK_COUNTER_OFFSET   (ShaderConstants.offsets.taskLoopCounters + TASK_LOOP_QBVH_COUNTER_OFFSET)
#define NUM_TASKS_DONE_OFFSET (ShaderConstants.offsets.taskLoopCounters + TASK_LOOP_QBVH_TASKS_DONE_OFFSET)

#include "Common.hlsl"
#include "IntersectCommon.hlsl"
#include "BuildCommonScratch.hlsl"
#include "CompactCommon.hlsl"

// BVH 8 requires up to 8 LDS elements per thread
#define MAX_ELEMENTS_PER_THREAD 8

#define MAX_LDS_ELEMENTS_PER_THREADGROUP (MAX_ELEMENTS_PER_THREAD * BUILD_THREADGROUP_SIZE)
groupshared uint SharedMem[MAX_LDS_ELEMENTS_PER_THREADGROUP];
#endif

//=====================================================================================================================
// Pack scratch node index for global stack
uint PackScratchNodeIndex(
    uint scratchNodeIdx,
    bool isFp16)
{
    uint packedNodeIdx = scratchNodeIdx;
    packedNodeIdx |= isFp16 ? (1u << 31) : 0;

    return packedNodeIdx;
}

//=====================================================================================================================
uint ExtractPackedNodeIndex(
    uint packedNodeIdx)
{
    return (packedNodeIdx & bits(31));
}

//=====================================================================================================================
bool IsPackedNodeFp16(
    uint packedNodeIdx)
{
    return ((packedNodeIdx >> 31) != 0);
}

//=====================================================================================================================
uint WriteInstanceNode(
    in uint               numActivePrims,
    in ScratchNode        scratchNode,
    in uint               scratchNodeIndex,
    in uint               nodeOffset,
    in AccelStructOffsets offsets)
{
    const uint instanceIndex = scratchNode.left_or_primIndex_or_instIndex;

    // When rebraiding is disabled the destination index is just the instance index.
    const uint destIndex = (Settings.rebraidType != RebraidType::Off) ?
                           (scratchNodeIndex - numActivePrims + 1) :
                           instanceIndex;

    bool numLeafsDoneAlreadyIncremented = false;
    if (numLeafsDoneAlreadyIncremented == false)
    {
        const uint numLeafsDoneOffset = ShaderConstants.offsets.qbvhGlobalStackPtrs + STACK_PTRS_NUM_LEAFS_DONE_OFFSET;
        ScratchGlobal.InterlockedAdd(numLeafsDoneOffset, 1);

        {
            nodeOffset =
                offsets.leafNodes + (destIndex * GetBvhNodeSizeInstance(Settings.enableFusedInstanceNode));
        }
    }

    const uint nodePointer = PackNodePointer(NODE_TYPE_USER_NODE_INSTANCE, nodeOffset);
    const uint metadataSizeInBytes = DstMetadata.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET);

    InstanceDesc instanceDesc;
    if (ShaderConstants.encodeArrayOfPointers != 0)
    {
        GpuVirtualAddress addr = InstanceDescBuffer.Load<GpuVirtualAddress>(instanceIndex * GPU_VIRTUAL_ADDRESS_SIZE);
        instanceDesc = FetchInstanceDescAddr(addr);
    }
    else
    {
        instanceDesc = InstanceDescBuffer.Load<InstanceDesc>(instanceIndex * INSTANCE_DESC_SIZE);
    }

    // Passing allowUpdate = false here since inactive primitives have already been filtered out at this point.
    const uint64_t blasHeaderAddr = GetAccelStructBaseAddr(instanceDesc, false);

    WriteInstanceDescriptor(metadataSizeInBytes,
                            blasHeaderAddr,
                            instanceDesc.InstanceContributionToHitGroupIndex_and_Flags,
                            instanceDesc.InstanceID_and_Mask,
                            instanceDesc.Transform,
                            instanceIndex,
                            nodePointer,
                            scratchNode.splitBox_or_nodePointer);

    DstBuffer.Store(offsets.primNodePtrs + (destIndex * sizeof(uint)), nodePointer);

    return nodePointer;
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
                                     Settings.doTriangleSplitting,
                                     ShaderConstants.offsets.triangleSplitBoxes,
                                     enablePairCompression,
                                     baseScratchNodesOffset);
}

//=====================================================================================================================
bool IsDestIndexInStack()
{
    const bool destIndexInStack = (Settings.fp16BoxNodesMode == LEAF_NODES_IN_BLAS_AS_FP16) ||
                                  (Settings.fp16BoxNodesMode == MIXED_NODES_IN_BLAS_AS_FP16);
    return destIndexInStack;
}

//=====================================================================================================================
bool StackHasTwoEntries()
{
    bool stackHasTwoEntries = IsDestIndexInStack();
    return stackHasTwoEntries;
}

//=====================================================================================================================
// Returns source BVH node index given node.
uint StackPopNodeIdx(
    uint stackIndex)
{
    if (StackHasTwoEntries())
    {
        stackIndex *= 2;
    }

    return ScratchGlobal.Load(ShaderConstants.offsets.qbvhGlobalStack + (stackIndex * sizeof(uint)));
}

//=====================================================================================================================
// Returns the second dword on the stack for a given stack index.
uint StackPopSecondEntry(
    uint          stackIndex)
{
    return ScratchGlobal.Load(ShaderConstants.offsets.qbvhGlobalStack + ((2 * stackIndex + 1) * sizeof(uint)));
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
    in uint globalId,
    in uint numLeafNodes,
    in uint numThreads)
{
    // Note the first entry is initialized below. Skip initializing it to invalid.
    const uint maxInternalNodeCount = GetNumInternalNodeCount(numLeafNodes);
    for (uint stackIndex = globalId + 1; stackIndex < maxInternalNodeCount; stackIndex += numThreads)
    {
        if (StackHasTwoEntries())
        {
            const uint qbvhStackOffset = ShaderConstants.offsets.qbvhGlobalStack + (stackIndex * sizeof(uint2));
            ScratchGlobal.Store<uint2>(qbvhStackOffset, uint2(INVALID_IDX, INVALID_IDX));
        }
        else
        {
            const uint qbvhStackOffset = ShaderConstants.offsets.qbvhGlobalStack + (stackIndex * sizeof(uint));
            ScratchGlobal.Store(qbvhStackOffset, INVALID_IDX);
        }
    }

    if (globalId == 0)
    {
        StackPtrs stackPtrs = (StackPtrs)0;
        // Source node index in linear memory
        stackPtrs.stackPtrSrcNodeId = 1;
        // Node destination in linear memory. Counts in 64B chunks.
        stackPtrs.stackPtrNodeDest = 2;

        stackPtrs.numLeafsDone = 0;

        ScratchGlobal.Store<StackPtrs>(ShaderConstants.offsets.qbvhGlobalStackPtrs, stackPtrs);

        const uint32_t rootNodeIndex =
            FetchRootNodeIndex(Settings.enableFastLBVH, ShaderConstants.offsets.fastLBVHRootNodeIndex);

        if (StackHasTwoEntries())
        {
            ScratchGlobal.Store<uint2>(ShaderConstants.offsets.qbvhGlobalStack, uint2(rootNodeIndex, 0));
        }
        else
        {
            ScratchGlobal.Store(ShaderConstants.offsets.qbvhGlobalStack, rootNodeIndex);
        }
    }
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
        else if (Settings.triangleCompressionMode != PAIR_TRIANGLE_COMPRESSION)
        {
            bool compressionEnabled = false;
            if (compressionEnabled == false)
            {
                // When compression is disabled, the final leaf node count is the active prim count.
                WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET, numActivePrims);
            }
        }

        const AccelStructHeader header = DstBuffer.Load<AccelStructHeader>(0);

        // Unused
        uint metadataSizeInBytes = 0;
        AccelStructOffsets offsets;

        // Calculate compacted size
        const uint accelStructType = Settings.topLevelBuild ? TOP_LEVEL : BOTTOM_LEVEL;
        uint compactedSize = CalcCompactedSize(header, accelStructType, offsets, metadataSizeInBytes);

        WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_COMPACTED_BYTE_SIZE_OFFSET, compactedSize);

        if (Settings.emitCompactSize != 0)
        {
            EmitBuffer.Store2(0, uint2(compactedSize, 0));
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
    uint numLeafNodes;

    // With pair compresssion enabled, the leaf node count in header represents actual valid
    // primitives
    if (EnableLatePairCompression())
    {
        numLeafNodes = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET);;
    }
    else
    {
        numLeafNodes = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
    }

    InitBuildQbvhImpl(globalId, numLeafNodes, ShaderRootConstants.numThreads);
}
#endif

//=====================================================================================================================
void WriteTriangleNodePrimitive0(
    in uint geometryPrimNodePtrsOffset,
    in uint nodeOffset,
    in uint primitiveIndex)
{
    {
        DstBuffer.Store(nodeOffset + TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET, primitiveIndex);
    }

    const uint primNodePtr = PackNodePointer(NODE_TYPE_TRIANGLE_0, nodeOffset);
    DstBuffer.Store(geometryPrimNodePtrsOffset + (primitiveIndex * sizeof(uint)),
                    primNodePtr);
}

//=====================================================================================================================
uint WritePrimitiveNode(
    in uint               scratchNodesScratchOffset,
    in ScratchNode        scratchNode,
    in uint               nodeOffset,
    in AccelStructOffsets offsets)
{
    uint nodeType =
        (Settings.geometryType == GEOMETRY_TYPE_TRIANGLES) ? NODE_TYPE_TRIANGLE_0 : NODE_TYPE_USER_NODE_PROCEDURAL;

    // Load geometry info
    const uint geometryIndex = ExtractScratchNodeGeometryIndex(scratchNode);

    const uint geometryInfoOffset = offsets.geometryInfo + (geometryIndex * sizeof(GeometryInfo));
    const GeometryInfo geometryInfo = DstBuffer.Load<GeometryInfo>(geometryInfoOffset);
    const uint geometryFlags = ExtractGeometryInfoFlags(geometryInfo.geometryFlagsAndNumPrimitives);
    const uint geometryIndexAndFlags = PackGeometryIndexAndFlags(geometryIndex, geometryFlags);
    const uint geometryPrimNodePtrsOffset = offsets.primNodePtrs + geometryInfo.primNodePtrsOffset;

    const uint flattenedPrimIndex =
        (geometryInfo.primNodePtrsOffset / sizeof(uint)) + scratchNode.left_or_primIndex_or_instIndex;

    uint numLeafsDone;
    ScratchGlobal.InterlockedAdd(ShaderConstants.offsets.qbvhGlobalStackPtrs + STACK_PTRS_NUM_LEAFS_DONE_OFFSET,
                                 1,
                                 numLeafsDone);

    {
        uint destIndex;
        if (IsTrianglePrimitiveBuild() &&
            ((Settings.triangleCompressionMode != NO_TRIANGLE_COMPRESSION) || Settings.doTriangleSplitting))
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

    const uint triangleId = ExtractScratchNodeTriangleId(scratchNode.packedFlags);

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

        bool isPairCompressed = (Settings.triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION);
        if (Settings.enableEarlyPairCompression)
        {
            isPairCompressed &= IsScratchNodeEarlyTrianglePair(scratchNode);
        }
        else
        {
            isPairCompressed &= (scratchNode.splitBox_or_nodePointer != INVALID_IDX);
        }

        // Pair compressed triangles nodes are referenced by triangle 1
        nodeType = isPairCompressed ? NODE_TYPE_TRIANGLE_1 : NODE_TYPE_TRIANGLE_0;

        {
            DstBuffer.Store(nodeOffset + TRIANGLE_NODE_GEOMETRY_INDEX_AND_FLAGS_OFFSET, geometryIndexAndFlags);
            DstBuffer.Store(nodeOffset + TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET + (nodeType * 4),
                            scratchNode.left_or_primIndex_or_instIndex);
        }

        if (Settings.enableEarlyPairCompression)
        {
            DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V0_OFFSET, scratchNode.bbox_min_or_v0);
            DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V1_OFFSET, scratchNode.bbox_max_or_v1);
            DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V2_OFFSET, scratchNode.sah_or_v2_or_instBasePtr);

            // For PAIR_TRIANGLE_COMPRESSION, the other node is not linked in the BVH tree, so we need to find it and
            // store it as well if it exists.
            if (isPairCompressed)
            {
                const float3 v3 = float3(asfloat(scratchNode.splitBox_or_nodePointer),
                                         asfloat(scratchNode.numPrimitivesAndDoCollapse),
                                         asfloat(scratchNode.unused));

                DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V3_OFFSET, v3);

                const uint tri0PrimitiveIndex =
                    scratchNode.left_or_primIndex_or_instIndex - ExtractScratchNodePairPrimOffset(scratchNode);

                WriteTriangleNodePrimitive0(geometryPrimNodePtrsOffset,
                                            nodeOffset,
                                            tri0PrimitiveIndex);
            }
        }
        else
        {
            uint3 vertexOffsets = uint3(TRIANGLE_NODE_V0_OFFSET, TRIANGLE_NODE_V0_OFFSET, TRIANGLE_NODE_V0_OFFSET);
            if (Settings.triangleCompressionMode != NO_TRIANGLE_COMPRESSION)
            {
                vertexOffsets += CalcTriangleCompressionVertexOffsets(nodeType, triangleId);
            }
            else
            {
                vertexOffsets += CalcTriangleVertexOffsets(nodeType);
            }

            DstBuffer.Store<float3>(nodeOffset + vertexOffsets.x, scratchNode.bbox_min_or_v0);
            DstBuffer.Store<float3>(nodeOffset + vertexOffsets.y, scratchNode.bbox_max_or_v1);
            DstBuffer.Store<float3>(nodeOffset + vertexOffsets.z, scratchNode.sah_or_v2_or_instBasePtr);

            // For PAIR_TRIANGLE_COMPRESSION, the other node is not linked in the BVH tree, so we need to find it and
            // store it as well if it exists.
            if (isPairCompressed)
            {
                const ScratchNode tri0Node =
                    FetchScratchNode(scratchNodesScratchOffset, scratchNode.splitBox_or_nodePointer);

                const float3 tr0Verts[3] =
                {
                    tri0Node.bbox_min_or_v0,
                    tri0Node.bbox_max_or_v1,
                    tri0Node.sah_or_v2_or_instBasePtr
                };

                const uint3 tri0VertOffsets =
                    CalcTriangleCompressionVertexOffsets(NODE_TYPE_TRIANGLE_0, triangleId);

                for (uint i = 0; i < 3; ++i)
                {
                    // Since the other node will always be of type NODE_TYPE_TRIANGLE_0, it is sufficient to store only
                    // the vertex that goes into v0. v1, v2, and v3 were already stored by NODE_TYPE_TRIANGLE_1 above.
                    if (tri0VertOffsets[i] == 0)
                    {
                        DstBuffer.Store<float3>(nodeOffset + TRIANGLE_NODE_V0_OFFSET, tr0Verts[i]);
                    }
                }

                WriteTriangleNodePrimitive0(geometryPrimNodePtrsOffset,
                                            nodeOffset,
                                            tri0Node.left_or_primIndex_or_instIndex);
            }
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
    bool topLevelBuild,
    uint stackIndex)
{
    uint destIndex;

    if (IsDestIndexInStack())
    {
        // Collapse should not enter this code path because tasks store destination indexes directly
        destIndex = StackPopSecondEntry(stackIndex);
    }
    else if (topLevelBuild || (Settings.fp16BoxNodesMode == NO_NODES_IN_BLAS_AS_FP16))
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
    ScratchGlobal.InterlockedAdd(ShaderConstants.offsets.qbvhGlobalStackPtrs + STACK_PTRS_SRC_PTR_OFFSET,
                                 numStackItems,
                                 origStackIdx);

    // compute proper stack index for each box child node
    const uint4 intChildStackIdx = origStackIdx + uint4(0,
                                                        countbits(compChildInfo & 0x1),
                                                        countbits(compChildInfo & 0x3),
                                                        countbits(compChildInfo & 0x7));

    uint origDest;

    // Uniform nodes - rely on a single stack item per thread
    if (IsDestIndexInStack() == false)
    {
        origDest = GetDestIdx(topLevelBuild, origStackIdx);

        // Destination offset index for each child node
        intChildDstOffset += origDest;

        // Store node index for every box child node
        for (uint i = 0; i < 4; i++)
        {
            if (compChildInfo & bit(i))
            {
                ScratchGlobal.Store(ShaderConstants.offsets.qbvhGlobalStack + intChildStackIdx[i] * sizeof(uint),
                                    intChildNodeIdx[i]);
            }
        }

        DeviceMemoryBarrier();
    }
    // Mixed nodes - atomically advance destination memory address to avoid holes between nodes
    else
    {
        ScratchGlobal.InterlockedAdd(ShaderConstants.offsets.qbvhGlobalStackPtrs + STACK_PTRS_DST_PTR_OFFSET,
                                     numDstChunks64B,
                                     origDest);

        // Destination offset index for each child node
        intChildDstOffset += origDest;

        // Store node index, destination offset index for every box child node
        for (uint i = 0; i < 4; i++)
        {
            if (compChildInfo & bit(i))
            {
                const uint2 toWrite = { intChildNodeIdx[i], intChildDstOffset[i] };
                ScratchGlobal.Store<uint2>(
                    ShaderConstants.offsets.qbvhGlobalStack + intChildStackIdx[i] * sizeof(uint2),
                    toWrite);
            }
        }

        DeviceMemoryBarrier();
    }

    return origStackIdx;
}

//=====================================================================================================================
static uint ProcessNode(
    in uint               scratchNodesScratchOffset, // Scratch nodes offset
    in uint               metadataSizeInBytes,       // Metadata size
    in bool               topLevelBuild,    // Compile time flag indicating whether this is top or bottom level build
    in ScratchNode        scratchNode,      // Scratch node being processed
    in uint               scratchNodeIndex, // Index of the Scratch node in BVH2
    in uint               destIndex,        // Dest index of the node in terms of 64B chunks in DstBuffer
    in uint               parentNodePtr,    // Parent node pointer
    in AccelStructOffsets offsets,          // Header offsets
    in bool               isLeafNode,       // Current node is a leaf node
    in bool               isFp16,           // Current node is marked as fp16 box node
    in uint               numActivePrims)   // Number of active primitives
{
    uint nodeOffset = Calc64BChunkOffset(destIndex);
    uint nodePointer;

    if (isLeafNode == false)
    {
        bool writeAsFp16BoxNode = false;
        {
            writeAsFp16BoxNode = ((topLevelBuild == false) && isFp16);
        }
        nodePointer = CreateBoxNodePointer(nodeOffset, writeAsFp16BoxNode);
    }
    else
    {
        if (topLevelBuild)
        {
            nodePointer = WriteInstanceNode(numActivePrims,
                                            scratchNode,
                                            scratchNodeIndex,
                                            nodeOffset,
                                            offsets);
        }
        else
        {
            nodePointer = WritePrimitiveNode(scratchNodesScratchOffset, scratchNode, nodeOffset, offsets);
        }
    }

    WriteParentPointer(metadataSizeInBytes, nodePointer, parentNodePtr);

    return nodePointer;
}

//=====================================================================================================================
static void PullUpChildren(
    in    uint                    scratchNodesScratchOffset,
    in    uint                    metadataSizeInBytes,
    in    bool                    topLevelBuild,
    in    uint                    qbvhNodePtr,
    in    AccelStructOffsets      offsets,
    in    uint4                   nodeIdx,
    in    uint4                   dstIdx,
    inout uint                    child[4],
    inout BoundingBox             bbox[4],
    inout uint                    boxNodeFlags,
    in    uint                    numActivePrims)
{
    [unroll]
    for (uint i = 0; i < 4; i++)
    {
        if (nodeIdx[i] != INVALID_IDX)
        {
            const uint scratchNodeIdx = ExtractPackedNodeIndex(nodeIdx[i]);
            bool isLeafNode = IsLeafNode(scratchNodeIdx, numActivePrims);

            const ScratchNode n = FetchScratchNode(scratchNodesScratchOffset, scratchNodeIdx);

            child[i] = ProcessNode(scratchNodesScratchOffset,
                                   metadataSizeInBytes,
                                   topLevelBuild,
                                   n,
                                   scratchNodeIdx,
                                   dstIdx[i],
                                   qbvhNodePtr,
                                   offsets,
                                   isLeafNode,
                                   IsPackedNodeFp16(nodeIdx[i]),
                                   numActivePrims);

            bbox[i]      = GetScratchNodeBoundingBoxTS(scratchNodesScratchOffset, isLeafNode, n);
            boxNodeFlags = SetBoxNodeFlagsField(boxNodeFlags, ExtractScratchNodeBoxFlags(n.packedFlags), i);
        }
        else
        {
            {
                // Note, box node flags are combined together by using an AND operation. Thus, we need to initialise
                // invalid child flags as 0xff
                boxNodeFlags = SetBoxNodeFlagsField(boxNodeFlags, 0xff, i);
            }
        }
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

    return writeNodeAsFp16;
}

//=====================================================================================================================
// Apply heuristics to choose which 2 nodes to open up in BVH2.
static void ApplyHeuristic(
    in    bool                    topLevelBuild,
    in    uint                    localId,
    in    uint                    scratchNodesScratchOffset,
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
                    FetchScratchNode(scratchNodesScratchOffset, SharedMem[baseLdsOffset + j]);

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
            FetchScratchNode(scratchNodesScratchOffset, SharedMem[baseLdsOffset + nodeIdxToOpen]);

        SharedMem[baseLdsOffset + nodeIdxToOpen] = candidateNode.left_or_primIndex_or_instIndex;
        SharedMem[baseLdsOffset + nodeCounter] = candidateNode.right_or_geometryIndex;
        nodeCounter++;
    }

    for (uint i = 0; (i < nodeCounter) && (i < 4); i++)
    {
        const uint scratchNodeIdx = SharedMem[baseLdsOffset + i];
        dstIdx[i] = count64B;

        const bool isLeafNode = IsLeafNode(scratchNodeIdx, numActivePrims);
        info |= (isLeafNode ? 0 : (1u << i));

        bool isFp16 = false;
        if ((Settings.fp16BoxNodesMode != NO_NODES_IN_BLAS_AS_FP16) && (isLeafNode == false))
        {
            isFp16 = OverrideFp32BoxNodeToFp16(scratchNodeIdx, scratchNodesScratchOffset, numActivePrims);
        }

        nodeIdxFinal[i] = PackScratchNodeIndex(scratchNodeIdx, isFp16);

        count64B += GetNum64BChunks(topLevelBuild, isLeafNode, isFp16);
    }

}

//=====================================================================================================================
void BuildQbvhImpl(
    uint globalId,
    uint localId,
    uint numLeafNodes,
    uint numActivePrims,
    bool topLevelBuild)  // Compile time flag indicating whether this is top or bottom level build
{
    // Load acceleration structure header
    const AccelStructHeader  header  = DstBuffer.Load<AccelStructHeader>(0);
    const AccelStructOffsets offsets = header.offsets;

    const uint rootNodeIndex =
        FetchRootNodeIndex(Settings.enableFastLBVH, ShaderConstants.offsets.fastLBVHRootNodeIndex);
    const uint scratchNodesScratchOffset = CalculateBvhNodesOffset(ShaderConstants, numActivePrims);
    const uint metadataSizeInBytes = DstMetadata.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET);

    if (globalId == 0)
    {
        // Root node begins after the acceleration structure header and is always of type fp32
        // regardless of mode for fp16 box nodes
        const uint rootNodePtr = CreateRootNodePointer();
        const ScratchNode rootScratchNode = FetchScratchNode(scratchNodesScratchOffset, rootNodeIndex);

        const bool isLeafNode = (numLeafNodes == 1);
        const BoundingBox bbox = GetScratchNodeBoundingBoxTS(scratchNodesScratchOffset, isLeafNode, rootScratchNode);

        WriteAccelStructHeaderRootBoundingBox(bbox);
        WriteParentPointer(metadataSizeInBytes, rootNodePtr, INVALID_IDX);

        // Generate box node with a leaf reference in child index 0 for single primitive acceleration structure
        if (numLeafNodes == 1)
        {
            uint destIndex = 0;
            const uint childNodePtr = ProcessNode(scratchNodesScratchOffset,
                                                  metadataSizeInBytes,
                                                  topLevelBuild,
                                                  rootScratchNode,
                                                  rootNodeIndex,
                                                  destIndex,
                                                  rootNodePtr,
                                                  offsets,
                                                  true,
                                                  false,
                                                  numActivePrims);

            const uint numPrimitives = 1;

            Float32BoxNode fp32BoxNode = (Float32BoxNode)0;
            fp32BoxNode.child0        = childNodePtr;
            fp32BoxNode.child1        = INVALID_IDX;
            fp32BoxNode.child2        = INVALID_IDX;
            fp32BoxNode.child3        = INVALID_IDX;
            fp32BoxNode.bbox0_min     = bbox.min;
            fp32BoxNode.bbox0_max     = bbox.max;
            fp32BoxNode.flags         = ExtractScratchNodeBoxFlags(rootScratchNode.packedFlags);
            fp32BoxNode.numPrimitives = numPrimitives;

            const uint qbvhNodeAddr = Calc64BChunkOffset(0);

            {
                WriteBoxNode(qbvhNodeAddr, fp32BoxNode);
            }

            {
                WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_INTERNAL_FP32_NODES_OFFSET, 1);
            }

            return;
        }
    }

    // Each stack item stores data for writes to linear QBVH memory, indexed by stack index
    const uint stackIndex = globalId;

    // Skip threads that do not map to valid internal nodes
    if (stackIndex >= GetNumInternalNodeCount(numLeafNodes))
    {
        return;
    }

    // This isn't necessary, but not using a boolean seems to cause a shader hang. See comment at the end of
    // while loop
    bool isDone = false;

    while (1)
    {
        DeviceMemoryBarrier();

        // Fetch leaf done count
        const uint numLeafsDoneOffset = ShaderConstants.offsets.qbvhGlobalStackPtrs + STACK_PTRS_NUM_LEAFS_DONE_OFFSET;
        const uint numLeafsDone       = ScratchGlobal.Load(numLeafsDoneOffset);

        // Check if we've processed all leaves or have internal nodes on the stack
        if ((numLeafsDone >= numLeafNodes) || isDone)
        {
            break;
        }

        // Pop a node to process from the stack.
        const uint packedNodeIdx = StackPopNodeIdx(stackIndex);

        // If we have a valid node on the stack, process node
        if (packedNodeIdx != INVALID_IDX)
        {
            const uint bvhNodeSrcIdx = ExtractPackedNodeIndex(packedNodeIdx);

            // Fetch BVH2 node
            const ScratchNode node = FetchScratchNode(scratchNodesScratchOffset, bvhNodeSrcIdx);

            // Each stack writes to linear QBVH memory indexed by stack index
            bool writeAsFp16BoxNode = false;

            {
                const bool isFp16 = IsPackedNodeFp16(packedNodeIdx);
                writeAsFp16BoxNode = (Settings.fp16BoxNodesMode != NO_NODES_IN_BLAS_AS_FP16) &&
                                     isFp16;
            }

            const uint bvhNodeDstIdx = GetDestIdx(topLevelBuild, stackIndex);
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
                           scratchNodesScratchOffset,
                           numActivePrims,
                           node,
                           compChildInfo,
                           intChildNodeIdx,
                           intChildDstIdx,
                           childDstCount64B);

            const uint origStackIdx = AllocQBVHStackNumItems(topLevelBuild,
                                                             compChildInfo,
                                                             intChildNodeIdx,
                                                             intChildDstIdx,
                                                             childDstCount64B);

            // Temporary child node pointers and bounds
            uint        child[4]     = { INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX };
            BoundingBox bbox[4]      = { (BoundingBox)0, (BoundingBox)0, (BoundingBox)0, (BoundingBox)0 };
            uint        boxNodeFlags = 0;

            // Pull up the nodes chosen by SAH to be the new children in QBVH
            PullUpChildren(scratchNodesScratchOffset,
                           metadataSizeInBytes,
                           topLevelBuild,
                           qbvhNodePtr,
                           offsets,
                           intChildNodeIdx,
                           intChildDstIdx,
                           child,
                           bbox,
                           boxNodeFlags,
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

                WriteFp16BoxNode(qbvhNodeAddr, fp16BoxNode);
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
                    WriteBoxNode(qbvhNodeAddr, fp32BoxNode);
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

    const bool topLevelBuild  = Settings.topLevelBuild;
    const uint type           = Settings.topLevelBuild ? TOP_LEVEL : BOTTOM_LEVEL;
    const uint numActivePrims = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);

    uint numLeafNodes = numActivePrims;

    // With pair compresssion enabled, the leaf node count in header represents actual valid
    // primitives
    if (EnableLatePairCompression())
    {
        numLeafNodes = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET);
    }

    uint numTasksWait = 0;
    uint waveId       = 0;

    INIT_TASK;

    BEGIN_TASK(RoundUpQuotient(GetNumInternalNodeCount(numLeafNodes), BUILD_THREADGROUP_SIZE));

    InitBuildQbvhImpl(globalId, numLeafNodes, ShaderRootConstants.numThreads);

    END_TASK(RoundUpQuotient(GetNumInternalNodeCount(numLeafNodes), BUILD_THREADGROUP_SIZE));

    BEGIN_TASK(RoundUpQuotient(GetNumInternalNodeCount(numLeafNodes), BUILD_THREADGROUP_SIZE));

    {
        {
            BuildQbvhImpl(globalId, localId, numLeafNodes, numActivePrims, topLevelBuild);
        }
    }

    END_TASK(RoundUpQuotient(GetNumInternalNodeCount(numLeafNodes), BUILD_THREADGROUP_SIZE));

    BEGIN_TASK(1);

    PostHwBvhBuild(localId, numActivePrims);

    END_TASK(1);
}
#endif
