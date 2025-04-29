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
#ifndef _GFX12_INTERNAL_NODE_H
#define _GFX12_INTERNAL_NODE_H

#include "../ShaderDefs.hlsli"
#include "../Common.hlsli"
#include "childInfo.hlsli"
#include "../Bits.hlsli"
#include "../gfx10/BoxNode1_0.hlsli"

static const uint ObbDisabled = 0x7f;

#define QUANTIZED_BVH8_NODE_OFFSET_INTERNAL_NODE_BASE_OFFSET      0 // offsetof(QuantizedBVH8BoxNode, internalNodeBaseOffset);
#define QUANTIZED_BVH8_NODE_OFFSET_LEAF_NODE_BASE_OFFSET          4 // offsetof(QuantizedBVH8BoxNode, leafNodeBaseOffset);
#define QUANTIZED_BVH8_NODE_OFFSET_PARENT_POINTER                 8 // offsetof(QuantizedBVH8BoxNode, parentPointer);
#define QUANTIZED_BVH8_NODE_OFFSET_ORIGIN                        12 // offsetof(QuantizedBVH8BoxNode, origin);
#define QUANTIZED_BVH8_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT 24 // offsetof(QuantizedBVH8BoxNode, exponentsChildIndexAndChildCount);
#define QUANTIZED_BVH8_NODE_OFFSET_OBB_MATRIX_INDEX              28 // offsetof(QuantizedBVH8BoxNode, obbMatrixIndex);
#define QUANTIZED_BVH8_NODE_OFFSET_CHILD_INFO_0                  32 // offsetof(QuantizedBVH8BoxNode, childInfos);
#define QUANTIZED_BVH8_NODE_SIZE                                128

#define QUANTIZED_BVH4_NODE_OFFSET_ORIGIN                        0
#define QUANTIZED_BVH4_NODE_OFFSET_EXP_CHILD_IDX_AND_VALID_COUNT 12
#define QUANTIZED_BVH4_NODE_OFFSET_CHILD_INFO_0                  16

//=====================================================================================================================
// Quantized internal BVH8 node information
//=====================================================================================================================
struct QuantizedBVH8BoxNode
{
    uint32_t  internalNodeBaseOffset;
    uint32_t  leafNodeBaseOffset;
    uint32_t  parentPointer;
    float3    origin;

#ifdef __cplusplus
    union
    {
        struct
        {
            uint32_t xExponent  : 8;
            uint32_t yExponent  : 8;
            uint32_t zExponent  : 8;
            uint32_t childIndex : 4;
            uint32_t childCount : 4;
        };

        uint32_t exponentsChildIndexAndChildCount;
    };
#else
    uint32_t exponentsChildIndexAndChildCount;
#endif

    uint32_t  obbMatrixIndex;
    ChildInfo childInfos[8];

#ifdef __cplusplus
    QuantizedBVH8BoxNode() : QuantizedBVH8BoxNode(0)
    {
    }

    QuantizedBVH8BoxNode(uint val)
    {
        memset(this, val, sizeof(QuantizedBVH8BoxNode));
    }
#endif

    static uint PackExpChildIdxAndCount(uint3 exponents,
                                        uint  disableBoxSort,
                                        uint  indexInParent,
                                        uint  validChildCount);

    void Init();
    uint InternalNodeBaseOffset();
    void SetInternalNodeBaseOffset(uint internalNodeBaseOffsetVal);
    uint OBBMatrixIndex();
    void SetOBBMatrixIndex(uint idx);
    uint LeafNodeBaseOffset();
    void SetLeafNodeBaseOffset(uint leafNodeBaseOffsetVal);
    uint ParentPointer();
    void SetParentPointer(uint parentPointerVal);
    float3 Origin();
    void SetOrigin(float3 originVal);
    uint3 Exponents();
    void SetExponents(uint3 exponents);
    uint IndexInParent();
    void SetIndexInParent(uint indexInParent);
    uint ValidChildCount();
    void SetValidChildCount(uint childCount);
    void SetLeafPointer(
        uint childIndex,
        uint startTriStructOffset,
        uint startType);
    void SetInternalNodeChildInfo(
        uint childIndex,
        uint startInternalNodeOffset);

#if GPURT_DEVELOPER
    bool ValidatePlanes(BoundingBox aabbs[8], uint aabbCount);

#ifdef __cplusplus
    void Print();
#endif
#endif

    void Decode(
        out_param(Float32BoxNode) f32BoxNode0,
        out_param(Float32BoxNode) f32BoxNode1,
        out_param(uint)           parentPointer);

#if GPURT_BVH_BUILD_SHADER
    void DecodeChildrenOffsets(out uint childPointers[8]);
#endif

    void Encode(
        Float32BoxNode f32BoxNode0,
        Float32BoxNode f32BoxNode1,
        uint           parentPointer,
        uint           indexInParent);
};

#if __cplusplus
static_assert(sizeof(QuantizedBVH8BoxNode) == 128);
#endif

//=====================================================================================================================
inline uint QuantizedBVH8BoxNode::PackExpChildIdxAndCount(
    uint3 exponents,
    uint  disableBoxSort,
    uint  indexInParent,
    uint  validChildCount)
{
    uint packedData = 0;
    packedData = bitFieldInsert(packedData, 0, 8, exponents.x);
    packedData = bitFieldInsert(packedData, 8, 8, exponents.y);
    packedData = bitFieldInsert(packedData, 16, 8, exponents.z);
    packedData = bitFieldInsert(packedData, 24, 1, disableBoxSort);
    packedData = bitFieldInsert(packedData, 25, 3, indexInParent);
    packedData = bitFieldInsert(packedData, 28, 3, validChildCount - 1);

    return packedData;
}

//=====================================================================================================================
inline void QuantizedBVH8BoxNode::Init()
{
    internalNodeBaseOffset = 0;
    leafNodeBaseOffset = 0;
    parentPointer = INVALID_NODE;
    origin = float3(0, 0, 0);
    exponentsChildIndexAndChildCount = 0;
    obbMatrixIndex = ObbDisabled;

    for (uint i = 0; i < 8; i++)
    {
        childInfos[i].Init();
    }
}

//=====================================================================================================================
inline uint QuantizedBVH8BoxNode::InternalNodeBaseOffset()
{
    return internalNodeBaseOffset;
}

//=====================================================================================================================
inline void QuantizedBVH8BoxNode::SetInternalNodeBaseOffset(uint internalNodeBaseOffsetVal)
{
    internalNodeBaseOffset = internalNodeBaseOffsetVal;
}

//=====================================================================================================================
inline uint QuantizedBVH8BoxNode::OBBMatrixIndex()
{
    return obbMatrixIndex;
}

//=====================================================================================================================
inline void QuantizedBVH8BoxNode::SetOBBMatrixIndex(uint idx)
{
    obbMatrixIndex = idx;
}

//=====================================================================================================================
inline uint QuantizedBVH8BoxNode::LeafNodeBaseOffset()
{
    return leafNodeBaseOffset;
}

//=====================================================================================================================
inline void QuantizedBVH8BoxNode::SetLeafNodeBaseOffset(uint leafNodeBaseOffsetVal)
{
    leafNodeBaseOffset = leafNodeBaseOffsetVal;
}

//=====================================================================================================================
inline uint QuantizedBVH8BoxNode::ParentPointer()
{
    return parentPointer;
}

//=====================================================================================================================
inline void QuantizedBVH8BoxNode::SetParentPointer(uint parentPointerVal)
{
    parentPointer = parentPointerVal;
}

//=====================================================================================================================
inline float3 QuantizedBVH8BoxNode::Origin()
{
    return origin;
}

//=====================================================================================================================
inline void QuantizedBVH8BoxNode::SetOrigin(float3 originVal)
{
    origin = originVal;
}

//=====================================================================================================================
inline uint3 QuantizedBVH8BoxNode::Exponents()
{
    uint3 exponents;
    exponents.x = bitFieldExtract(exponentsChildIndexAndChildCount, 0, 8);
    exponents.y = bitFieldExtract(exponentsChildIndexAndChildCount, 8, 8);
    exponents.z = bitFieldExtract(exponentsChildIndexAndChildCount, 16, 8);

    return exponents;
}

//=====================================================================================================================
inline void QuantizedBVH8BoxNode::SetExponents(uint3 exponents)
{
    exponentsChildIndexAndChildCount = bitFieldInsert(exponentsChildIndexAndChildCount, 0, 8, exponents.x);
    exponentsChildIndexAndChildCount = bitFieldInsert(exponentsChildIndexAndChildCount, 8, 8, exponents.y);
    exponentsChildIndexAndChildCount = bitFieldInsert(exponentsChildIndexAndChildCount, 16, 8, exponents.z);
}

//=====================================================================================================================
inline uint QuantizedBVH8BoxNode::IndexInParent()
{
    return bitFieldExtract(exponentsChildIndexAndChildCount, 25, 3);
}

//=====================================================================================================================
inline void QuantizedBVH8BoxNode::SetIndexInParent(uint indexInParent)
{
    indexInParent = indexInParent & 0x7;
    exponentsChildIndexAndChildCount = bitFieldInsert(exponentsChildIndexAndChildCount, 25, 3, indexInParent);
}

//=====================================================================================================================
inline uint QuantizedBVH8BoxNode::ValidChildCount()
{
    return 1 + bitFieldExtract(exponentsChildIndexAndChildCount, 28, 3);
}

//=====================================================================================================================
inline void QuantizedBVH8BoxNode::SetValidChildCount(uint childCount)
{
    exponentsChildIndexAndChildCount = bitFieldInsert(exponentsChildIndexAndChildCount, 28, 3, (childCount - 1));
}

//=====================================================================================================================
inline void QuantizedBVH8BoxNode::SetLeafPointer(
    uint childIndex,
    uint startTriStructOffset,
    uint startType)
{
    /* The internal nodes conceptually encode up to 8 primitive ranges. Each primitive range
    * is described using a single start pointer (offset + node type). To generate 8 stack offsets
    * it must be known how many node boundaries each primitive range crosses.
    *
    * To simplify, the actual value for node boundaries is known given 3 pieces of information
    * 1. The internal node base leaf offset
    * 2. The number of node boundaries crossed by earlier leaf children
    * 3. The offset of the node that is beginning this primitive range
    *
    * 1 and 2 determine the offset of the previous primitive range and 3 determines where this primitive range starts.
    * Subtracting where this prim range starts from where the last starts allows us to determine the necessary value
    * for node boundaries without storing any extra state.
    */

    if (leafNodeBaseOffset == INVALID_NODE)
    {
        leafNodeBaseOffset = startTriStructOffset;
    }

#if defined(GPURT_DEVELOPER) && defined(__cplusplus)
    assert(startTriStructOffset >= leafNodeBaseOffset);
#endif

    uint runningOffset = leafNodeBaseOffset;
    uint lastPrimIndex = -1;
    for (uint i = 0; i < childIndex; i++)
    {
        if (childInfos[i].Valid())
        {
            if (childInfos[i].NodeType() != NODE_TYPE_BOX_QUANTIZED_BVH8)
            {
                lastPrimIndex = i;
                runningOffset += childInfos[i].NodeRangeLength() * 128;
            }
        }
    }
    uint nodeBoundaries = (startTriStructOffset - runningOffset) / 128;

    if (lastPrimIndex != -1)
    {
        childInfos[lastPrimIndex].SetNodeRangeLength(nodeBoundaries);
    }
    childInfos[childIndex].SetNodeType(startType);
}

//=====================================================================================================================
inline void QuantizedBVH8BoxNode::SetInternalNodeChildInfo(
    uint childIndex,
    uint startInternalNodeOffset)
{
    uint lastInternalIndex = -1;
    uint runningOffset = internalNodeBaseOffset;
    for (uint i = 0; i < childIndex; i++)
    {
        if (childInfos[i].Valid())
        {
            if (childInfos[i].NodeType() == NODE_TYPE_BOX_QUANTIZED_BVH8)
            {
                runningOffset += childInfos[i].NodeRangeLength() * 128;
                lastInternalIndex = i;
            }
        }
    }
    uint nodeBoundaries = (startInternalNodeOffset - runningOffset) / 128;

    if (lastInternalIndex != -1)
    {
        childInfos[lastInternalIndex].SetNodeRangeLength(nodeBoundaries);
    }
    childInfos[childIndex].SetNodeType(NODE_TYPE_BOX_QUANTIZED_BVH8);
}

#if GPURT_DEVELOPER
//=====================================================================================================================
inline bool QuantizedBVH8BoxNode::ValidatePlanes(BoundingBox aabbs[8], uint aabbCount)
{
    uint3 exponents = Exponents();

    for (uint i = 0; i < aabbCount; i++)
    {
        if (aabbs[i].min.x <= aabbs[i].max.x)
        {
            BoundingBox decoded = childInfos[i].DecodeBounds(origin, exponents);

            uint3 quantmax = childInfos[i].Max() + uint3(1, 1, 1);
            uint3 quantmin = childInfos[i].Min();
            bool correct = all(decoded.min <= aabbs[i].min) && all(decoded.max >= aabbs[i].max);
            if (!correct)
            {
                return false;
            }

            // Make sure that choosing the next most restrict value is invalid (i.e. the most precise value was
            // chosen)

            quantmax.x = (quantmax.x == 1) ? 1 : (quantmax.x - 1);
            quantmax.y = (quantmax.y == 1) ? 1 : (quantmax.y - 1);
            quantmax.z = (quantmax.z == 1) ? 1 : (quantmax.z - 1);

            quantmin.x = (quantmin.x == 4095) ? 4095 : (quantmin.x + 1);
            quantmin.y = (quantmin.y == 4095) ? 4095 : (quantmin.y + 1);
            quantmin.z = (quantmin.z == 4095) ? 4095 : (quantmin.z + 1);

            float3 minPlanes = float3(Dequantize(origin.x, exponents.x, quantmin.x, 12),
                                      Dequantize(origin.y, exponents.y, quantmin.y, 12),
                                      Dequantize(origin.z, exponents.z, quantmin.z, 12));
            float3 maxPlanes = float3(Dequantize(origin.x, exponents.x, quantmax.x, 12),
                                      Dequantize(origin.y, exponents.y, quantmax.y, 12),
                                      Dequantize(origin.z, exponents.z, quantmax.z, 12));

            decoded.min = minPlanes;
            decoded.max = maxPlanes;

            uint3 qmin = childInfos[i].Min();

            // If qmin is 4095 and 4096 is a better choice then it is still the best box that could have been
            // created
            if ((decoded.min.x < aabbs[i].min.x) && (qmin.x != 4095))
            {
                return false;
            }

            if ((decoded.min.y < aabbs[i].min.y) && (qmin.y != 4095))
            {
                return false;
            }

            if ((decoded.min.z < aabbs[i].min.z) && (qmin.z != 4095))
            {
                return false;
            }

            uint3 qmax = childInfos[i].Max() + uint3(1, 1, 1);

            // If qmax is 1 and 0 is a better choice then it is still the best box that could have been created
            if ((decoded.max.x > aabbs[i].max.x) && (qmax.x != 1))
            {
                return false;
            }

            if ((decoded.max.y > aabbs[i].max.y) && (qmax.y != 1))
            {
                return false;
            }

            if ((decoded.max.z > aabbs[i].max.z) && (qmax.z != 1))
            {
                return false;
            }
        }
    }

    return true;
}

#ifdef __cplusplus
//=====================================================================================================================
inline void QuantizedBVH8BoxNode::Print()
{
    printf("Base Internal:%i Base Leaf:%i Parent:%i Obb:%i\n",
           internalNodeBaseOffset,
           leafNodeBaseOffset,
           parentPointer,
           obbMatrixIndex);

    printf("%f %f %f\n", origin.x, origin.y, origin.z);

    uint3 exponents = Exponents();
    printf("%x %x %x\n", exponents.x, exponents.y, exponents.z);

    printf("%i children\n", ValidChildCount());

    for (uint i = 0; i < ValidChildCount(); i++)
    {
        if (childInfos[i].Valid())
        {
            printf("Child %i has type %i. Node range length %i. Instance Mask %i\n",
                   i,
                   childInfos[i].NodeType(),
                   childInfos[i].NodeRangeLength(),
                   childInfos[i].InstanceMask());
            printf("\tMin %i %i %i\n\tMax %i %i %i\n",
                   childInfos[i].Min().x,
                   childInfos[i].Min().y,
                   childInfos[i].Min().z,
                   childInfos[i].Max().x,
                   childInfos[i].Max().y,
                   childInfos[i].Max().z);

            BoundingBox aabb = childInfos[i].DecodeBounds(origin, exponents);
            fprintf(stdout, "\tDecoded Min %f, %f, %f\n", aabb.min.x, aabb.min.y, aabb.min.z);
            fprintf(stdout, "\tDecoded Max %f, %f, %f\n\n", aabb.max.x, aabb.max.y, aabb.max.z);

            float x = aabb.max.x - aabb.min.x;
            float y = aabb.max.y - aabb.min.y;
            float z = aabb.max.z - aabb.min.z;
            printf("\tSurface Area: %f\n", 2 * (x * y + x * z + y * z));
        }
    }
}
#endif
#endif

//=====================================================================================================================
inline void QuantizedBVH8BoxNode::Decode(
    out_param(Float32BoxNode) f32BoxNode0,
    out_param(Float32BoxNode) f32BoxNode1,
    out_param(uint)           parentPointer)
{
    BoundingBox aabbs[8];
    uint childPointers[8];
    uint childReuse[8];

    parentPointer = ParentPointer();

    // Take local copies of base offsets as these are updated as children are decoded.
    // Lower 4 bits of internal offset will always be 5 as this is the only valid internal
    // node type.
    uint internalOffset = internalNodeBaseOffset | NODE_TYPE_BOX_QUANTIZED_BVH8;
    uint primitiveOffset = leafNodeBaseOffset;

    uint aabbCount = ValidChildCount();

    // Track the primitive range and node type of the previous child.
    // Used to determine if child reuse is active for current box.
    uint previousNodeType = 0;
    bool previousRangeZero = false;

    uint3 exponents = Exponents();

    // Valid children are always packed from the start of the node.
    for (uint i = 0; i < 8; i++)
    {
        if (i < aabbCount)
        {
            // Decode the child offset.
            // There are two running offsets: one for internal node children (boxes), and one for
            // primitive node children (all other types).
            // For primitive node children, the node type must be placed into the lower 4 bits.
            // The appropriate running offset must be incremented by the number of consecutive
            // 128-byte nodes the box points to.
            if (childInfos[i].NodeType() == NODE_TYPE_BOX_QUANTIZED_BVH8)
            {
                childPointers[i] = internalOffset;
                internalOffset += (childInfos[i].NodeRangeLength() << 4);
            }
            else
            {
                childPointers[i] = primitiveOffset | childInfos[i].NodeType();
                primitiveOffset += (childInfos[i].NodeRangeLength() << 4);
            }

            // Dequantize the box planes.
            // Min planes range from 0-4095, and max planes from 1-4096.
            // This will result in output planes ranging from 0 - 2^(exponent-127).
            aabbs[i] = childInfos[i].DecodeBounds(origin, exponents);

            // This box is reusing the child offset from the previous box if the node type is the
            // same and the previous box had zero primitive range.
            childReuse[i] = (previousNodeType == childInfos[i].NodeType()) && (previousRangeZero == true);
            previousNodeType = childInfos[i].NodeType();
            previousRangeZero = (childInfos[i].NodeRangeLength() == 0);
        }
        else
        {
            // Invalid children are always packed at the end of the node.
            childPointers[i] = INVALID_NODE;

            // Set the box planes to NaN to force a miss.
            aabbs[i].min = float3(NaN, NaN, NaN);
            aabbs[i].max = float3(NaN, NaN, NaN);

            childReuse[i] = 0;
        }
    }

    f32BoxNode0 = (Float32BoxNode)0;
    f32BoxNode1 = (Float32BoxNode)0;

    f32BoxNode0.child0 = childPointers[0];
    f32BoxNode0.child1 = childPointers[1];
    f32BoxNode0.child2 = childPointers[2];
    f32BoxNode0.child3 = childPointers[3];
    f32BoxNode1.child0 = childPointers[4];
    f32BoxNode1.child1 = childPointers[5];
    f32BoxNode1.child2 = childPointers[6];
    f32BoxNode1.child3 = childPointers[7];

    f32BoxNode0.bbox0_min = aabbs[0].min;
    f32BoxNode0.bbox0_max = aabbs[0].max;
    f32BoxNode0.bbox1_min = aabbs[1].min;
    f32BoxNode0.bbox1_max = aabbs[1].max;
    f32BoxNode0.bbox2_min = aabbs[2].min;
    f32BoxNode0.bbox2_max = aabbs[2].max;
    f32BoxNode0.bbox3_min = aabbs[3].min;
    f32BoxNode0.bbox3_max = aabbs[3].max;
    f32BoxNode0.obbMatrixIndex = obbMatrixIndex;

    f32BoxNode1.bbox0_min = aabbs[4].min;
    f32BoxNode1.bbox0_max = aabbs[4].max;
    f32BoxNode1.bbox1_min = aabbs[5].min;
    f32BoxNode1.bbox1_max = aabbs[5].max;
    f32BoxNode1.bbox2_min = aabbs[6].min;
    f32BoxNode1.bbox2_max = aabbs[6].max;
    f32BoxNode1.bbox3_min = aabbs[7].min;
    f32BoxNode1.bbox3_max = aabbs[7].max;

    for (uint j = 0; j < 4; j++)
    {
        f32BoxNode0.flags |= (childInfos[j].CullingFlags() << (j * 8));
        f32BoxNode1.flags |= (childInfos[j + 4].CullingFlags() << (j * 8));

        f32BoxNode0.instanceMask |= (childInfos[j].InstanceMask() << (j * 8));
        f32BoxNode1.instanceMask |= (childInfos[j + 4].InstanceMask() << (j * 8));
    }
}

#if GPURT_BVH_BUILD_SHADER
//=====================================================================================================================
inline void QuantizedBVH8BoxNode::DecodeChildrenOffsets(out uint childPointers[8])
{
    // Take local copies of base offsets as these are updated as children are decoded.
    // Lower 4 bits of internal offset will always be 5 as this is the only valid internal
    // node type.
    uint internalOffset = internalNodeBaseOffset | NODE_TYPE_BOX_QUANTIZED_BVH8;
    uint primitiveOffset = leafNodeBaseOffset;

    uint aabbCount = ValidChildCount();

    // Track the primitive range and node type of the previous child.
    // Used to determine if child reuse is active for current box.
    uint previousNodeType = 0;
    bool previousRangeZero = false;

    uint3 exponents = Exponents();

    // Valid children are always packed from the start of the node.
    for (uint i = 0; i < 8; i++)
    {
        if (i < aabbCount)
        {
            // Decode the child offset.
            // There are two running offsets: one for internal node children (boxes), and one for
            // primitive node children (all other types).
            // For primitive node children, the node type must be placed into the lower 4 bits.
            // The appropriate running offset must be incremented by the number of consecutive
            // 128-byte nodes the box points to.
            if (childInfos[i].NodeType() == NODE_TYPE_BOX_QUANTIZED_BVH8)
            {
                childPointers[i] = internalOffset;
                internalOffset += (childInfos[i].NodeRangeLength() << 4);
            }
            else
            {
                childPointers[i] = primitiveOffset | childInfos[i].NodeType();
                primitiveOffset += (childInfos[i].NodeRangeLength() << 4);
            }

            // This box is reusing the child offset from the previous box if the node type is the
            // same and the previous box had zero primitive range.
            //childReuse[i] = (previousNodeType == childInfos[i].NodeType()) && (previousRangeZero == true);
            previousNodeType = childInfos[i].NodeType();
            previousRangeZero = (childInfos[i].NodeRangeLength() == 0);
        }
        else
        {
            // Invalid children are always packed at the end of the node.
            childPointers[i] = INVALID_NODE;
        }
    }
}
#endif

//=====================================================================================================================
inline void QuantizedBVH8BoxNode::Encode(
    Float32BoxNode f32BoxNode0,
    Float32BoxNode f32BoxNode1,
    uint           parentPointer,
    uint           indexInParent)
{
    BoundingBox aabbs[8];
    uint        childPointers[8];

    childPointers[0] = f32BoxNode0.child0;
    childPointers[1] = f32BoxNode0.child1;
    childPointers[2] = f32BoxNode0.child2;
    childPointers[3] = f32BoxNode0.child3;
    childPointers[4] = f32BoxNode1.child0;
    childPointers[5] = f32BoxNode1.child1;
    childPointers[6] = f32BoxNode1.child2;
    childPointers[7] = f32BoxNode1.child3;

    aabbs[0].min = f32BoxNode0.bbox0_min;
    aabbs[0].max = f32BoxNode0.bbox0_max;
    aabbs[1].min = f32BoxNode0.bbox1_min;
    aabbs[1].max = f32BoxNode0.bbox1_max;
    aabbs[2].min = f32BoxNode0.bbox2_min;
    aabbs[2].max = f32BoxNode0.bbox2_max;
    aabbs[3].min = f32BoxNode0.bbox3_min;
    aabbs[3].max = f32BoxNode0.bbox3_max;
    aabbs[4].min = f32BoxNode1.bbox0_min;
    aabbs[4].max = f32BoxNode1.bbox0_max;
    aabbs[5].min = f32BoxNode1.bbox1_min;
    aabbs[5].max = f32BoxNode1.bbox1_max;
    aabbs[6].min = f32BoxNode1.bbox2_min;
    aabbs[6].max = f32BoxNode1.bbox2_max;
    aabbs[7].min = f32BoxNode1.bbox3_min;
    aabbs[7].max = f32BoxNode1.bbox3_max;

    for (uint i = 0; i < 4; i++)
    {
        childInfos[i].SetCullingFlags(bitFieldExtract(f32BoxNode0.flags,  (i * 8), 4));
        childInfos[i + 4].SetCullingFlags(bitFieldExtract(f32BoxNode1.flags, (i * 8), 4));
        childInfos[i].SetInstanceMask(bitFieldExtract(f32BoxNode0.instanceMask, i * 8, 8));
        childInfos[i + 4].SetInstanceMask(bitFieldExtract(f32BoxNode1.instanceMask, i * 8, 8));
    }

    uint aabbCount              = 0;
    uint internalNodeBaseOffset = 0;
    uint leafNodeBaseOffset     = 0;

    // Valid children are always packed from the start of the node.
    for (; (aabbCount < 8) && (childPointers[aabbCount] != INVALID_NODE); ++aabbCount)
    {
        if (IsQuantizedBVH8BoxNode(childPointers[aabbCount]))
        {
            if (internalNodeBaseOffset == 0)
            {
                internalNodeBaseOffset = ClearNodeType(childPointers[aabbCount]);
            }
        }
        else if (leafNodeBaseOffset == 0)
        {
            leafNodeBaseOffset = ClearNodeType(childPointers[aabbCount]);
        }

        childInfos[aabbCount].SetNodeType(GetNodeType(childPointers[aabbCount]));
        childInfos[aabbCount].SetNodeRangeLength(1);
    }
    SetOBBMatrixIndex(f32BoxNode0.obbMatrixIndex);
    SetInternalNodeBaseOffset(internalNodeBaseOffset);
    SetLeafNodeBaseOffset(leafNodeBaseOffset);
    SetParentPointer(parentPointer);
    SetIndexInParent(indexInParent);

    // Next section is all about encoding the child bounding boxes

    float3 minOfMins = aabbs[0].min;
    float3 maxOfMaxs = aabbs[0].max;
    for (uint j = 1; j < aabbCount; j++)
    {
        // For degenerate triangles and instances, we write inverted bounds in scratch nodes.
        // Inverted bounds don't get encoded and decoded correctly. For example, when min is FLT_MAX and max is
        // -FLT_MAX, max gets decoded as INFINITY, which is a problem during intersection.
        // We set instance inclusion mask as 0 instead to cull them out.
        minOfMins = min(minOfMins, aabbs[j].min);
        maxOfMaxs = max(maxOfMaxs, aabbs[j].max);
    }

    const uint3 exponents = ComputeCommonExponent(minOfMins, maxOfMaxs, 12);

    const float3 rcpExponents = ComputeFastExpReciprocal(exponents, 12);

    SetOrigin(minOfMins);
    SetExponents(exponents);

    for (uint k = 0; k < aabbCount; k++)
    {
        const UintBoundingBox quantBounds = ComputeQuantizedBounds(aabbs[k], minOfMins, rcpExponents, 12);
        childInfos[k].SetMin(quantBounds.min);
        childInfos[k].SetMax(quantBounds.max);
    }

    for (uint m = aabbCount; m < 8; m++)
    {
        childInfos[m].Invalidate();
    }

#if GPURT_DEVELOPER
    for (uint n = 0; n < aabbCount; n++)
    {
        if (aabbs[n].min.x <= aabbs[n].max.x)
        {
            BoundingBox aabb = childInfos[n].DecodeBounds(origin, exponents);
#ifdef __cplusplus
            assert(all(aabb.max >= aabb.min));
#endif
        }
    }

    if (ValidatePlanes(aabbs, aabbCount) == false)
    {
#ifdef __cplusplus
        printf("Wrong quantization\n");
        exit(1);
#endif
    }
#endif

    SetValidChildCount(aabbCount);
}

#endif
