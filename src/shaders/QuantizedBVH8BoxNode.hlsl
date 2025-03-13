/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _QUANTIZEDBVH8BOXNODE_HLSL
#define _QUANTIZEDBVH8BOXNODE_HLSL

#include "../shadersClean/common/Common.hlsli"

#include "../shadersClean/common/gfx12/internalNode.hlsli"

//=====================================================================================================================
static uint GetChildInfoOffset(
    in uint childIdx)
{
    return QUANTIZED_BVH8_NODE_OFFSET_CHILD_INFO_0 + (childIdx * QUANTIZED_NODE_CHILD_INFO_STRIDE);
}

//=====================================================================================================================
static uint ExtractValidChildCount(
    uint packedData)
{
    return  1 + bitFieldExtract(packedData, 28, 3);
}

//=====================================================================================================================
static uint GetBvh4ChildInfoOffset(
    in uint childIdx)
{
    return QUANTIZED_BVH4_NODE_OFFSET_CHILD_INFO_0 + (childIdx * QUANTIZED_NODE_CHILD_INFO_STRIDE);
}

//=====================================================================================================================
static uint ExtractBvh4ValidChildCount(
    uint packedData)
{
    return  1 + bitFieldExtract(packedData, 28, 2);
}

//=====================================================================================================================
static uint3 ExtractPackedExponents(
    uint packedData)
{
    uint3 exponents;
    exponents.x = bitFieldExtract(packedData, 0, 8);
    exponents.y = bitFieldExtract(packedData, 8, 8);
    exponents.z = bitFieldExtract(packedData, 16, 8);

    return exponents;
}

#if GPURT_BVH_BUILD_SHADER
//=====================================================================================================================
// Get the address of the AABB node at the specified BLAS address and node pointer.
// With OBBs enabled, the AABB must be loaded from a separate data section.
static uint64_t GetBlasAabbNodeAddr(
    uint64_t blasBaseAddr,
    uint     nodePtr)
{
    const uint nodeOffset = ExtractNodePointerOffset(nodePtr);

    uint64_t nodeAddr = blasBaseAddr + nodeOffset;

    if ((Settings.enableOrientedBoundingBoxes & bit(BOTTOM_LEVEL)) != 0)
    {
        const uint obbBlasMetadataOffset =
            LoadDwordAtAddr(blasBaseAddr + ACCEL_STRUCT_HEADER_OBB_BLAS_METADATA_OFFSET_OFFSET);

        if (obbBlasMetadataOffset != 0)
        {
            // There is no header data in the OBB metadata section
            nodeAddr -= ACCEL_STRUCT_HEADER_SIZE;
            // Add the metadata offset and the offset of the root nodes within the metadata
            nodeAddr += obbBlasMetadataOffset + BLAS_OBB_METADATA_ROOT_NODE_DATA_OFFSET;
        }
    }

    return nodeAddr;
}

//=====================================================================================================================
static uint GetQuantizedBoxRebraidChildCount(
    uint64_t blasBaseAddr,
    uint32_t nodePtr)
{
    const uint nodeOffset = ExtractNodePointerOffset(nodePtr);
    const uint64_t nodeAddr = blasBaseAddr + nodeOffset;

    ChildInfo childInfo;
    uint validChildCount = 0;
    uint childOffset = GetChildInfoOffset(0);

    [unroll]
    for (uint i = 0; i < 4; ++i)
    {
        uint3 packedChildInfo = LoadDwordAtAddrx4(nodeAddr + childOffset).xyz;
        childInfo.Load(packedChildInfo);

        if (childInfo.Valid())
        {
            validChildCount++;
        }

        childOffset += QUANTIZED_NODE_CHILD_INFO_STRIDE;
    }

    return validChildCount;
}

//=====================================================================================================================
// Decode BVH4 internal root node when Rebraid is enabled.
static void DecodeRebraidChildInfoBVH4(
    uint64_t               blasBaseAddr,
    out_param(BoundingBox) bbox[4],
    out_param(uint4)       child)
{
    const uint64_t rootNodeAddr = GetBlasAabbNodeAddr(blasBaseAddr, CreateRootNodePointer3_1());

    // Note, all internal node children in a BLAS are allocated in contiguous memory, we only need
    // the internal node offset to generate node pointers.
    uint internalNodeOffset =
        LoadDwordAtAddr(rootNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_INTERNAL_NODE_BASE_OFFSET);

    // Fetch origin.xyz and packed exponents, index in parent and valid count.
    const uint4 d0 =
        LoadDwordAtAddrx4(rootNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_ORIGIN);

    const uint3 exponents = ExtractPackedExponents(d0.w);
    const float3 origin = asfloat(d0.xyz);

    // Decode bounds for each child
    uint childOffset = GetChildInfoOffset(0);

    ChildInfo childInfo;

    [unroll]
    for (uint i = 0; i < 4; ++i)
    {
        uint3 packedChildInfo = LoadDwordAtAddrx4(rootNodeAddr + childOffset).xyz;
        childInfo.Load(packedChildInfo);

        // It is possible this child is valid according to the valid child count in the node, but the bounding
        // box is invalid due to a degenerate primitive. For rebraid purposes, consider this an invalid child
        // and avoid creating an instance node for it. Rebraid is incompatible with updatable TLAS so updates
        // are not a concern.
        if (childInfo.Valid())
        {
            child[i] = (i << SCRATCH_NODE_POINTER_CHILD_INDEX_SHIFT) | internalNodeOffset | childInfo.NodeType();
            bbox[i] = childInfo.DecodeBounds(origin, exponents);
        }
        else
        {
            child[i] = INVALID_NODE;
        }

        internalNodeOffset += (childInfo.NodeRangeLength() << 4);
        childOffset += QUANTIZED_NODE_CHILD_INFO_STRIDE;
    }
}

//=====================================================================================================================
static uint ReadQBVH8ParentPointer(
    in uint metadataSize,
    in uint nodePointer)
{
    const uint nodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);
    return SrcBuffer.Load(nodeOffset + QUANTIZED_BVH8_NODE_OFFSET_PARENT_POINTER);
}

//=====================================================================================================================
static uint ComputeQuantizedBoxChildIndexAndValidBoxCount(
    in uint              metadataSize,
    in uint              parentNodePointer,
    in uint              childNodePointer,
    out_param(uint)      boxNodeCount)
{
    const uint parentNodeOffset = metadataSize + ExtractNodePointerOffset(parentNodePointer);

    // Decode valid child count from parent box node
    const uint exponentsChildIndexAndChildCount =
        SrcBuffer.Load(parentNodeOffset + QUANTIZED_BVH8_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT);

    const uint validChildCount = ExtractValidChildCount(exponentsChildIndexAndChildCount);

    // Count valid box node children. Note, internal nodes are sorted to the beginning of the child list
    boxNodeCount = 0;

    for (uint i = 0; i < validChildCount; ++i)
    {
        const uint childInfoOffset = GetChildInfoOffset(i);

        const uint packedChildTypeAndRange = SrcBuffer.Load(
            parentNodeOffset + childInfoOffset + QUANTIZED_NODE_CHILD_INFO_OFFSET_MAXY_MAXZ_NODE_TYPE_AND_RANGE);

        const uint childType = bitFieldExtract(packedChildTypeAndRange, 24, 4);

        if (childType == NODE_TYPE_BOX_QUANTIZED_BVH8)
        {
            boxNodeCount++;
        }
        else
        {
            // Stop counting as soon as we encounter a leaf node
            break;
        }
    }

    uint childIdx = 0;

    const uint nodeType = GetNodeType(childNodePointer);
    if (nodeType == NODE_TYPE_BOX_QUANTIZED_BVH8)
    {
        // Fetch child index in parent from quantized box node
        const uint childNodeOffset = metadataSize + ExtractNodePointerOffset(childNodePointer);

        const uint exponentsChildIndexAndChildCount =
            SrcBuffer.Load(childNodeOffset + QUANTIZED_BVH8_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT);

        childIdx = bitFieldExtract(exponentsChildIndexAndChildCount, 25, 3);
    }
    else
    {
        uint primitiveOffset =
            SrcBuffer.Load(parentNodeOffset + QUANTIZED_BVH8_NODE_OFFSET_LEAF_NODE_BASE_OFFSET);

        // Search for child pointer within leaf node children that are sorted towards the end of the valid
        // child list
        childIdx = boxNodeCount;

        for (; childIdx < validChildCount; ++childIdx)
        {
            const uint childInfoOffset = GetChildInfoOffset(childIdx);

            const uint packedChildTypeAndRange = SrcBuffer.Load(
                parentNodeOffset + childInfoOffset + QUANTIZED_NODE_CHILD_INFO_OFFSET_MAXY_MAXZ_NODE_TYPE_AND_RANGE);

            const uint childNodeType = bitFieldExtract(packedChildTypeAndRange, 24, 4);
            const uint childNodeRangeLength = bitFieldExtract(packedChildTypeAndRange, 28, 4);

            // Generate node pointer and compare against source pointer
            const uint nodePointer = (primitiveOffset | childNodeType);
            if (nodePointer == childNodePointer)
            {
                // We've found our node pointer in the child list. End search.
                break;
            }
            else
            {
                // Continue to next child
                const uint strideShift = 4;
                primitiveOffset += (childNodeRangeLength << strideShift);
            }
        }
    }

    return childIdx;
}

//=====================================================================================================================
// Writes box node culling flags and instance mask to the appropriate child slot. Note, this overwrites the child
// quantized min_x/y that will get merged when the parent gets updated
//
static void UpdateQuantizedBoxNodeFlagsAndInstanceMask(
    in uint             nodeOffset,
    in uint             childIdx,
    in uint             boxNodeFlags,
    in uint             instanceMask)
{
    const uint childInfoOffset = nodeOffset + GetChildInfoOffset(childIdx);

    uint2 packedData = uint2(0, 0);
    packedData.x = bitFieldInsert(packedData.x, 24, 4, boxNodeFlags);
    packedData.y = bitFieldInsert(packedData.y, 24, 8, instanceMask);

    DstMetadata.Store(childInfoOffset + QUANTIZED_NODE_CHILD_INFO_OFFSET_MINX_MINY_CULLING_FLAGS, packedData.x);
    DstMetadata.Store(childInfoOffset + QUANTIZED_NODE_CHILD_INFO_OFFSET_MINZ_MAXX_INSTANCE_MASK, packedData.y);
}

//=====================================================================================================================
// Writes box node instance mask to the appropriate child slot. Note, this overwrites the child quantized min_x/y that
// will get merged when the parent gets updated
//
static void UpdateQuantizedBoxInstanceMask(
    in uint             nodeOffset,
    in uint             childIdx,
    in uint             instanceMask)
{
    const uint childInfoOffset = nodeOffset + GetChildInfoOffset(childIdx);

    const uint packedData = bitFieldInsert(0, 24, 8, instanceMask);

    DstMetadata.Store(childInfoOffset + QUANTIZED_NODE_CHILD_INFO_OFFSET_MINZ_MAXX_INSTANCE_MASK, packedData);
}

//=====================================================================================================================
// Construct the child info for use in an instance node.
//
// The node type and range length fields are repurposed to communicate the rebraid root node pointer while also
// allowing filter node functionality.
//
// - 128B node index is stored in the node type field. This is relative to the BVH base (header base).
// - Child 0's range length is used to indicate whether the root is a primitive or box node. In order to recover the
//   the range length field in traversal, child 0 is forced to miss so the inersection test returns the second child
//   if there is a hit. Pass through mode is assumed for primitive children so there is no impact to forcing a miss
//   on child 0. Child 1-3 have the same box.
//
// Instance test packed return values (N = 128B node index from start of BVH header):
// Miss:     0xFFFFFFFF
// Box hit:  0xFFFFFF0N
// Prim hit: 0xFFFFFF1N
static uint3 GetInstanceChildInfo(
    uint  childIndex,
    uint  blasRootNodePointer,
    uint3 quantMin,
    uint3 quantMax)
{
    // Box culling must be disabled on instance nodes because the culling logic does not account for the flags
    // encoded in the instance node pointer. For example, consider the BLAS is opaque, RAY_FLAG_CULL_OPAQUE is set,
    // and the instance has INSTANCE_FLAG_FORCE_NON_OPAQUE. The hardware would incorrectly cull the instance children
    // if ONLY_OPAQUE is set in the box flags.
    const uint boxCullingFlags = 0;

    // Per-child instance mask is ignored in instance nodes
    const uint instanceMask = 0xFF;

    // Node type field stores the 128B node index.
    const uint encodedNodeType = blasRootNodePointer >> 4;

    // Range length 0 indicates child reuse.
    // - For box nodes, set all lengths to 0. The hardware will return the child 0's node pointer if any box is hit.
    // - For primitive nodes, the first child's range length is set to 1 it is forced to miss. The hardware will always
    //   return child 1's node pointer.
    uint nodeRangeLength = 0;

    if ((childIndex == 0) && IsTriangleNode3_1(blasRootNodePointer))
    {
        // Force miss on child 0 for prim children. This allows traversal to access to child 0's range length.
        // Only pass through mode is supported for primitives (all boxes are the same).
        quantMin = uint3(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF);
        quantMax = uint3(0, 0, 0);
        // Set a bit in child 0's range length for prim children. This will identify prims in the traversal.
        nodeRangeLength = 1;
    }

    return ChildInfo::BuildPacked(quantMin, quantMax, boxCullingFlags, instanceMask, encodedNodeType, nodeRangeLength);
}

//=====================================================================================================================
// Write instance node box as a pass through node
static void WriteInstancePassthrough(
    in uint        blasRootNodePointer,
    in BoundingBox bbox,
    in uint        boxNodeOffset)
{
    const uint3 exponents = ComputeCommonExponent(bbox.min, bbox.max, 12);
    const float3 rcpExponents = ComputeFastExpReciprocal(exponents, 12);

    DstMetadata.Store3(boxNodeOffset + QUANTIZED_BVH4_NODE_OFFSET_ORIGIN, asuint(bbox.min));

    uint packedData = QuantizedBVH8BoxNode::PackExpChildIdxAndCount(exponents, 0, 0, 4);
    DstMetadata.Store(boxNodeOffset + QUANTIZED_BVH4_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT,
                      packedData);

    const UintBoundingBox quantBounds = ComputeQuantizedBounds(bbox, bbox.min, rcpExponents, 12);

    for (uint i = 0; i < 4; ++i)
    {
        const uint3 childInfo = GetInstanceChildInfo(i, blasRootNodePointer, quantBounds.min, quantBounds.max);
        DstMetadata.Store3(boxNodeOffset + GetBvh4ChildInfoOffset(i), childInfo);
    }
}

//=====================================================================================================================
// Update metadata filter node from source internal node
static void UpdateInstanceFilterNode(
    uint srcNodeOffset) // Source node offset from absolute base of BLAS
{
    // Fetch origin.xyz and packed exponents, index in parent and valid count.
    uint4 d0 = DstMetadata.Load4(srcNodeOffset + QUANTIZED_BVH8_NODE_OFFSET_ORIGIN);

    // Update valid count to 4
    d0.w = bitFieldInsert(d0.w, 28, 3, 3);

    // Copy to destination filter node
    DstMetadata.Store4(ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET, d0);

    // Merge two adjacent child info from the BVH8 internal node
    for (uint i = 0; i < 4; ++i)
    {
        uint offsetC0 = GetChildInfoOffset(i * 2);
        uint offsetC1 = GetChildInfoOffset((i * 2) + 1);

        ChildInfo c0 = DstMetadata.Load<ChildInfo>(srcNodeOffset + offsetC0);
        ChildInfo c1 = DstMetadata.Load<ChildInfo>(srcNodeOffset + offsetC1);

        // Since the quantized min/max are relative offsets from the same origin, we can simply merge adjacent
        // bounds by performing integer min/max operations.
        const uint3 quantMin = min(c0.Min(), c1.Min());
        const uint3 quantMax = max(c0.Max(), c1.Max());

        const uint3 childInfo = GetInstanceChildInfo(i, CreateRootNodePointer3_1(), quantMin, quantMax);

        // Write merged child info to metadata
        DstMetadata.Store3(ACCEL_STRUCT_METADATA_INSTANCE_NODE_OFFSET + GetBvh4ChildInfoOffset(i), childInfo);
    }
}

//=====================================================================================================================
// Decode BVH4 child bounds for rebraid
static BoundingBox DecodeRebraidChildBoundsBVH4(
    uint64_t blasBaseAddr,
    uint32_t rebraidNodePtr) // Rebraided child node pointer
{
    const uint64_t rootNodeAddr = GetBlasAabbNodeAddr(blasBaseAddr, CreateRootNodePointer3_1());

    // Note, all internal node children in a BLAS are allocated in contiguous memory, we only need
    // the internal node offset to generate node pointers.
    uint internalNodeOffset =
        LoadDwordAtAddr(rootNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_INTERNAL_NODE_BASE_OFFSET);

    // Fetch origin.xyz and packed exponents, index in parent and valid count.
    const uint4 d0 =
        LoadDwordAtAddrx4(rootNodeAddr + QUANTIZED_BVH8_NODE_OFFSET_ORIGIN);

    const float3 origin = asfloat(d0.xyz);
    const uint3 exponents = ExtractPackedExponents(d0.w);

    // Child index is encoded in the upper bits of the node pointer in the scratch node(DecodeRebraidChildInfoBVH4())
    const uint childIdx = rebraidNodePtr >> SCRATCH_NODE_POINTER_CHILD_INDEX_SHIFT;

    // Decode bounds for child
    const uint childOffset = GetChildInfoOffset(childIdx);
    const uint3 packedChildInfo = LoadDwordAtAddrx4(rootNodeAddr + childOffset).xyz;

    ChildInfo childInfo;
    childInfo.Load(packedChildInfo);

    return childInfo.DecodeBounds(origin, exponents);
}

#endif

#if defined(GPURT_DEVELOPER) && defined(__cplusplus)
static_assert(offsetof(QuantizedBVH8BoxNode, childInfos[0]) == 32, "Alignment Issue");
static_assert(offsetof(QuantizedBVH8BoxNode, childInfos[1]) == 44, "Alignment Issue");
#endif

#endif
