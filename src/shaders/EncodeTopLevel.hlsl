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
#define RootSig "RootConstants(num32BitConstants=12, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\
                "CBV(b1)"/*Build Settings binding*/

//=====================================================================================================================
// 32 bit constants
struct Constants
{
    uint metadataSizeInBytes;           // Size of the metadata header
    uint basePrimNodePtrOffset;         // Offset of the prim node pointeres
    uint numPrimitives;                 // Number of instances
    uint leafNodeDataByteOffset;        // Leaf node data byte offset
    uint sceneBoundsByteOffset;         // Scene bounds byte offset
    uint propagationFlagsScratchOffset; // Offset of update flags in scratch memory
    uint baseUpdateStackScratchOffset;  // Offset of update scratch
    uint internalFlags;                 // Internal flags
    uint buildFlags;                    // Build flags
    uint leafNodeExpansionFactor;       // Leaf node expansion factor (> 1 for rebraid)
    uint sceneBoundsCalculationType;
    uint enableFastLBVH;
};

//=====================================================================================================================
[[vk::push_constant]] ConstantBuffer<Constants>           ShaderConstants    : register(b0);
[[vk::binding(0, 0)]] RWByteAddressBuffer                 DstMetadata        : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer                 ScratchBuffer      : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer                 SrcBuffer          : register(u2);
[[vk::binding(3, 0)]] RWByteAddressBuffer                 InstanceDescBuffer : register(u3);

#include "IntersectCommon.hlsl"
#include "BuildCommonScratch.hlsl"

//=====================================================================================================================
bool IsUpdateInPlace()
{
    return (ShaderConstants.internalFlags & ENCODE_FLAG_UPDATE_IN_PLACE);
}

//=====================================================================================================================
bool IsRebraidEnabled()
{
    return (ShaderConstants.internalFlags & ENCODE_FLAG_REBRAID_ENABLED);
}

//=====================================================================================================================
bool IsFusedInstanceNode()
{
    return (ShaderConstants.internalFlags & ENCODE_FLAG_ENABLE_FUSED_INSTANCE_NODE);
}

//=====================================================================================================================
void WriteBoundingBoxNode(
    RWByteAddressBuffer buffer,
    uint                offset,
    uint                instanceIndex,
    in BoundingBox      bbox,
    uint                nodeFlags,
    uint                instanceBasePointerLo,
    uint                instanceBasePointerHi,
    uint                numActivePrims,
    float               cost)
{
    uint4 data;

    // LeafNode.bbox_min_or_v0, instanceIndex
    data = uint4(asuint(bbox.min), instanceIndex);
    buffer.Store4(offset + SCRATCH_NODE_V0_OFFSET, data);

    // LeafNode.bbox_max_or_v1, padding
    data = uint4(asuint(bbox.max), 0);
    buffer.Store4(offset + SCRATCH_NODE_V1_OFFSET, data);

    // LeafNode.instanceNodeBasePointerLo, instanceNodeBasePointerHi, numActivePrims, parent
    data = uint4(instanceBasePointerLo, instanceBasePointerHi, numActivePrims, 0xffffffff);
    buffer.Store4(offset + SCRATCH_NODE_INSTANCE_BASE_PTR_OFFSET, data);

    // When rebraid might occur, the traversal shader will read the root node pointer of the bottom level from the
    // instance extra data. If we actually perform rebraid during the build, the node pointer must be set to 0 to
    // indicate the leaf node is unused. The correct pointer will be filled in later. If we have rebraid support
    // enabled in traversal but not during this build, we need to set the pointer to the true root of the bottom level.
    const uint rootNodePointer = IsRebraidEnabled() ? 0 : CreateRootNodePointer();

    // type, flags, nodePointer, numPrimitivesAndDoCollapse
    data = uint4(NODE_TYPE_USER_NODE_INSTANCE, nodeFlags, rootNodePointer, asuint(cost));
    buffer.Store4(offset + SCRATCH_NODE_TYPE_OFFSET, data);
}

//=====================================================================================================================
uint CalcTopNodeFlags(
    uint geometryType,
    uint instanceFlags,
    uint blasNodeFlags)
{
    uint nodeFlags = 0;

    nodeFlags |= (instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_FORCE_OPAQUE)
                 ? 1 << BOX_NODE_FLAGS_ONLY_OPAQUE_SHIFT : 0;
    nodeFlags |= (instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_FORCE_NON_OPAQUE)
                 ? 1 << BOX_NODE_FLAGS_ONLY_NON_OPAQUE_SHIFT : 0;

    // Propagate BLAS root node flags if instance does not override them.
    nodeFlags = (nodeFlags == 0) ? blasNodeFlags : nodeFlags;

    nodeFlags |= (geometryType == GEOMETRY_TYPE_TRIANGLES) ? 1 << BOX_NODE_FLAGS_ONLY_TRIANGLES_SHIFT
                                                           : 1 << BOX_NODE_FLAGS_ONLY_PROCEDURAL_SHIFT;

    return nodeFlags;
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeInstances(
    in uint3 globalThreadId : SV_DispatchThreadID)
{
    const bool isUpdate    = IsUpdate(ShaderConstants.buildFlags);
    const bool allowUpdate = AllowUpdate(ShaderConstants.buildFlags);

    uint index = globalThreadId.x;

    if (index < ShaderConstants.numPrimitives)
    {
        InstanceDesc desc;

        if (ShaderConstants.internalFlags & ENCODE_FLAG_ARRAY_OF_POINTERS)
        {
            GpuVirtualAddress addr = InstanceDescBuffer.Load<GpuVirtualAddress>(index * GPU_VIRTUAL_ADDRESS_SIZE);
            desc = FetchInstanceDescAddr(addr);
        }
        else
        {
            desc = InstanceDescBuffer.Load<InstanceDesc>(index * INSTANCE_DESC_SIZE);
        }

        const uint tlasMetadataSize =
            isUpdate ? SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : ShaderConstants.metadataSizeInBytes;
        // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
        const AccelStructOffsets offsets = SrcBuffer.Load<AccelStructOffsets>(tlasMetadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);
        const uint basePrimNodePointersOffset = isUpdate ? offsets.primNodePtrs : ShaderConstants.basePrimNodePtrOffset;

        const uint primNodePointerOffset = tlasMetadataSize + basePrimNodePointersOffset + (index * sizeof(uint));

        const uint destScratchNodeOffset = (index * ByteStrideScratchNode) + ShaderConstants.leafNodeDataByteOffset;

        const uint instanceMask = (desc.InstanceID_and_Mask >> 24);

        GpuVirtualAddress baseAddr = MakeGpuVirtualAddress(desc.accelStructureAddressLo,
                                                           desc.accelStructureAddressHiAndFlags);
        uint64_t baseAddrAccelStructHeader = 0;
        uint numActivePrims = 0;

        if (baseAddr != 0)
        {
            baseAddrAccelStructHeader =
                    MakeGpuVirtualAddress(LoadDwordAtAddr(baseAddr + ACCEL_STRUCT_METADATA_VA_LO_OFFSET),
                                          LoadDwordAtAddr(baseAddr + ACCEL_STRUCT_METADATA_VA_HI_OFFSET));

            // In some odd cases where the BLAS memory gets trampled, this is a failsafe to skip such BLAS
            // and treat them as inactive rather than causing a page fault.
            const uint metadataSizeInBytes = LoadDwordAtAddr(baseAddr + ACCEL_STRUCT_METADATA_SIZE_OFFSET);
            const uint64_t expectedBaseAddr = baseAddr + metadataSizeInBytes;

            if (baseAddrAccelStructHeader == expectedBaseAddr)
            {
                numActivePrims =
                    FetchHeaderField(baseAddrAccelStructHeader, ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
            }
            else
            {
                numActivePrims = 0;
                baseAddr = 0ull;
            }
        }

        const bool deactivateNonUpdatable = (instanceMask == 0) || (numActivePrims == 0);

        // If the BLAS address is NULL, the instance is inactive. Inactive instances cannot be activated during updates
        // so we always exclude them from the build. Additionally, we deactivate instances in a non-updatable TLAS when
        // the instance mask is 0 or there are no active primitives in the BLAS.
        if ((baseAddr != 0) && ((deactivateNonUpdatable == false) || (allowUpdate == true)))
        {
            const uint geometryType =
                FetchHeaderField(baseAddrAccelStructHeader, ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET);

            const uint64_t instanceBasePointer =
                PackInstanceBasePointer(baseAddrAccelStructHeader,
                                        desc.InstanceContributionToHitGroupIndex_and_Flags >> 24,
                                        geometryType);

            desc.accelStructureAddressLo         = LowPart(instanceBasePointer);
            desc.accelStructureAddressHiAndFlags = HighPart(instanceBasePointer);

            // calc transformed AABB
            BoundingBox boundingBox;

            float cost = 0;

            if (numActivePrims != 0)
            {
                boundingBox = GenerateInstanceBoundingBox(instanceBasePointer, desc.Transform);

                const uint64_t instanceBaseAddr = GetInstanceAddr(LowPart(instanceBasePointer), HighPart(instanceBasePointer));

                const uint4 d0 = LoadDwordAtAddrx4(instanceBaseAddr + ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET);
                const uint2 d1 = LoadDwordAtAddrx2(instanceBaseAddr + ACCEL_STRUCT_HEADER_FP32_ROOT_BOX_OFFSET + 0x10);

                BoundingBox rootBbox = (BoundingBox)0;
                rootBbox.min = asfloat(d0.xyz);
                rootBbox.max = asfloat(uint3(d0.w, d1.xy));

                float origSA = ComputeBoxSurfaceArea(rootBbox);
                float transformedSA = ComputeBoxSurfaceArea(boundingBox);

                if (IsRebraidEnabled())
                {
                    cost = transformedSA / origSA;
                }
                else
                {
                    for (uint c = 0; c < 4; c++)
                    {
                        cost += asfloat(FetchHeaderField(baseAddrAccelStructHeader, ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + (4 * c)));
                    }

                    cost = (transformedSA / origSA) * cost + (transformedSA * SAH_COST_AABBB_INTERSECTION);
                }
            }
            else if (isUpdate == false)
            {
                // Generate a invalid dummy bounding box
                boundingBox = InvalidBoundingBox;
            }

            const uint blasNodeFlags = FetchHeaderField(baseAddrAccelStructHeader, ACCEL_STRUCT_HEADER_NODE_FLAGS_OFFSET);
            const uint nodeFlags = CalcTopNodeFlags(geometryType,
                                                    desc.InstanceContributionToHitGroupIndex_and_Flags >> 24,
                                                    blasNodeFlags);

            if (isUpdate == false)
            {
                // Write scratch node
                WriteBoundingBoxNode(ScratchBuffer,
                                        destScratchNodeOffset,
                                        index,
                                        boundingBox,
                                        nodeFlags,
                                        desc.accelStructureAddressLo,
                                        desc.accelStructureAddressHiAndFlags,
                                        numActivePrims,
                                        cost);

                if (numActivePrims != 0)
                {
                    // Update scene bounding box
                    if (ShaderConstants.sceneBoundsCalculationType == SceneBoundsBasedOnCentroidWithSize)
                    {
                        if (IsRebraidEnabled() == false)
                        {
                            UpdateCentroidSceneBoundsWithSize(ShaderConstants.sceneBoundsByteOffset, boundingBox);
                        }
                        else // remove size as rebraid will set the size
                        {
                            const float3 centroidPoint = (0.5 * (boundingBox.max + boundingBox.min));

                            const uint rebraidSceneOffset = sizeof(BoundingBox) + 2 * sizeof(float);

                            UpdateSceneBoundsUsingCentroid(ShaderConstants.sceneBoundsByteOffset + rebraidSceneOffset,
                                                           centroidPoint);
                        }
                    }
                    else if (ShaderConstants.sceneBoundsCalculationType == SceneBoundsBasedOnGeometryWithSize)
                    {
                        UpdateSceneBoundsWithSize(ShaderConstants.sceneBoundsByteOffset, boundingBox);
                    }
                    else
                    {
                        UpdateSceneBounds(ShaderConstants.sceneBoundsByteOffset, boundingBox);
                    }
                }

                // Store invalid prim node pointer for now during first time builds.
                // If the instance is active, BuildQBVH will write it in.
                DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
            }
            else if (numActivePrims != 0)
            {
                const uint blasMetadataSize = FetchHeaderField(baseAddrAccelStructHeader,
                                                               ACCEL_STRUCT_HEADER_METADATA_SIZE_OFFSET);
                const uint nodePointer = SrcBuffer.Load(primNodePointerOffset);
                const uint nodeOffset  = tlasMetadataSize + ExtractNodePointerOffset(nodePointer);

                // Fetch parent node pointer
                const uint parentNodePointer = ReadParentPointer(SrcBuffer,
                                                                 tlasMetadataSize,
                                                                 nodePointer);

                // Update prim node pointer and parent pointer in out of place destination buffer
                if (IsUpdateInPlace() == false)
                {
                    DstMetadata.Store(primNodePointerOffset, nodePointer);

                    WriteParentPointer(tlasMetadataSize,
                                       nodePointer,
                                       parentNodePointer);
                }

                {
                    const uint parentNodeOffset = tlasMetadataSize + ExtractNodePointerOffset(parentNodePointer);

                    // Compute box node count and child index in parent node
                    uint boxNodeCount = 0;
                    const uint childIdx = ComputeChildIndexAndValidBoxCount(Settings, SrcBuffer, parentNodeOffset, nodePointer, boxNodeCount);

                    // If even a single child node is a box node, this is a node higher up the tree. Skip queueing parent node as another
                    // leaf at the bottom of the tree will queue its parent which will handle our parent node.

                    // B B B B --> 4 --> Not possible
                    // B x B x --> 2 --> Not possible
                    // B L B L --> 4 --> Skip queuing
                    // L x B x --> 1 --> Skip queuing
                    // L x x x --> 0 --> Queue parent node
                    // L x L x --> 0 --> Queue parent node
                    // L L L L --> 0 --> Queue parent node

                    const uint boxOffset = childIdx * FLOAT32_BBOX_STRIDE;
                    {
                        DstMetadata.Store<float3>(parentNodeOffset + FLOAT32_BOX_NODE_BB0_MIN_OFFSET + boxOffset, boundingBox.min);
                        DstMetadata.Store<float3>(parentNodeOffset + FLOAT32_BOX_NODE_BB0_MAX_OFFSET + boxOffset, boundingBox.max);

                        // The instance flags can be changed in an update, so the node flags need to be updated.
                        const uint fieldMask = 0xFFu << (childIdx * BOX_NODE_FLAGS_BIT_STRIDE);
                        DstMetadata.InterlockedAnd(parentNodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET, ~fieldMask);
                        DstMetadata.InterlockedOr(parentNodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET,
                                                  nodeFlags << (childIdx * BOX_NODE_FLAGS_BIT_STRIDE));
                    }

                    // If this is the first child in the parent node with all leaf children, queue parent pointer to
                    // stack in scratch memory
                    if ((childIdx == 0) && (boxNodeCount == 0))
                    {
                        PushNodeToUpdateStack(ScratchBuffer, ShaderConstants.baseUpdateStackScratchOffset, parentNodePointer);

                        if (IsUpdateInPlace() == false)
                        {
                            const uint4 childPointers = SrcBuffer.Load<uint4>(parentNodeOffset);
                            DstMetadata.Store<uint4>(parentNodeOffset, childPointers);

                        }
                    }
                }
                WriteInstanceDescriptor(DstMetadata,
                                        desc,
                                        index,
                                        nodeOffset,
                                        blasMetadataSize,
                                        CreateRootNodePointer(),
                                        IsFusedInstanceNode());
            }
        }
        else if (isUpdate == false)
        {
            // Deactivate instance permanently by setting bbox_min_or_v0.x to NaN
            ScratchBuffer.Store(destScratchNodeOffset, asuint(NaN));

            DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
        }

        // ClearFlags for refit and update
        {
            const uint stride = ShaderConstants.leafNodeExpansionFactor * sizeof(uint);
            const uint flagOffset = ShaderConstants.propagationFlagsScratchOffset + (index * stride);
            const uint initValue = ShaderConstants.enableFastLBVH ? 0xffffffffu : 0;
            for (uint i = 0; i < ShaderConstants.leafNodeExpansionFactor; ++i)
            {
                ScratchBuffer.Store(flagOffset + (i * sizeof(uint)), initValue);
            }
        }

        DeviceMemoryBarrier();
        DstMetadata.InterlockedAdd(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET, 1);
    }
}
