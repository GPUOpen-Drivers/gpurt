/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2020-2022 Advanced Micro Devices, Inc. All Rights Reserved.
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
#pragma once

// TODO #gpurt: Most of this is copy-paste from RayTracingDefs.h.  Should just use that header.

namespace GpuRt
{

namespace Shaders
{

#define NODE_PTR_SIZE 4

using uint = uint32;

struct float4
{
    float4() {}
    float4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    float4(float s) : x(s), y(s), z(s), w(s) {}

    float x, y, z, w;
};

struct float3
{
    float3() {}
    float3(float x, float y, float z) : x(x), y(y), z(z) {}
    constexpr float3(float s) : x(s), y(s), z(s) {}

    float& operator[](int index)
    {
        return *(&x + index);
    }

    const float& operator[](int index) const
    {
        return *(&x + index);
    }

    float x, y, z;
};

inline float3 operator*(const float3& a, float b)
{
    return float3(a.x * b, a.y * b, a.z * b);
}

inline float3 operator+(const float3& a, const float3& b)
{
    return float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

struct float2
{
    float2() {}
    float2(float x, float y) : x(x), y(y) {}
    float2(float s) : x(s), y(s) {}

    float x, y;
};

struct uint3
{
    uint3() {}
    uint3(uint x, uint y, uint z) : x(x), y(y), z(z) {}
    uint3(uint s) : x(s), y(s), z(s) {}

    uint x, y, z;
};

inline uint3 operator*(const uint3& a, uint b)
{
    return uint3(a.x * b, a.y * b, a.z * b);
}

struct uint2
{
    uint2() {}
    uint2(uint x, uint y) : x(x), y(y) {}
    uint2(uint s) : x(s), y(s) {}

    uint x, y;
};

//=====================================================================================================================
inline uint asuint(float val)
{
    union
    {
        uint u32;
        float f32;
    } v;
    v.f32 = val;

    return v.u32;
}

//=====================================================================================================================
inline int asint(float val)
{
    union
    {
        int i32;
        float f32;
    } v;
    v.f32 = val;

    return v.i32;
}

//=====================================================================================================================
inline int asint(uint val)
{
    union
    {
        int i32;
        uint u32;
    } v;
    v.u32 = val;

    return v.i32;
}

//=====================================================================================================================
inline float asfloat(uint val)
{
    union
    {
        uint u32;
        float f32;
    } v;
    v.u32 = val;

    return v.f32;
}

//=====================================================================================================================
inline float2 asfloat(uint2 val)
{
    return float2(asfloat(val.x), asfloat(val.y));
}

//=====================================================================================================================
inline float3 asfloat(uint3 val)
{
    return float3(asfloat(val.x), asfloat(val.y), asfloat(val.z));
}

//=====================================================================================================================
struct TriangleData
{
    float3 v[3];
};

//=====================================================================================================================
// Internal bounding box type for scene bounds.
struct UintBoundingBox
{
    uint3 Minimum;
    uint3 Maximum;
};

struct GeometryInfo
{
    uint geometryFlags;
    uint numPrimitives;
    uint primNodePtrsOffset; // Offset from the base of all prim node ptrs to this geometry's prim node ptrs
};

struct BoundingBox
{
    float3 min;
    float3 max;
};

//=====================================================================================================================
// TODO: The highest 3 bits are not handled since they currently aren't written when building the QBVH.
inline uint ExtractNodePointerOffset(uint nodePointer)
{
    // From the HW raytracing spec:
    // node_addr[60:0] = node_pointer[63:3]
    // Also, based on the following, the node_addr is 64-byte aligned:
    // fetch_addr0 = T#.base_address*256+node_addr*64
    return (nodePointer & 0xFFFFFFF8) << 3;
}

//=====================================================================================================================
// Hardware BVH node types
#define NODE_TYPE_TRIANGLE_0           0
#define NODE_TYPE_TRIANGLE_1           1
#define NODE_TYPE_TRIANGLE_2           2
#define NODE_TYPE_TRIANGLE_3           3
#define NODE_TYPE_BOX_FLOAT16          4
#define NODE_TYPE_BOX_FLOAT32          5
#define NODE_TYPE_USER_NODE_INSTANCE   6
#define NODE_TYPE_USER_NODE_PROCEDURAL 7

//=====================================================================================================================
// Hardware triangle node format and offsets
#define TRIANGLE_NODE_V0_OFFSET 0
#define TRIANGLE_NODE_V1_OFFSET 12
#define TRIANGLE_NODE_V2_OFFSET 24
#define TRIANGLE_NODE_V3_OFFSET 36
#define TRIANGLE_NODE_V4_OFFSET 48
#define TRIANGLE_NODE_ID_OFFSET 60
#define TRIANGLE_NODE_SIZE      64

//=====================================================================================================================
// Triangle ID contains 4 1-byte fields, 1 per triangle:
// Triangle 0 [ 7: 0]
// Triangle 1 [15: 8]
// Triangle 2 [23:16]
// Triangle 3 [31:24]
//
// Each triangle's 8-bit segment contains these fields:
// I SRC        [1:0] Specifies which vertex in triangle 0 corresponds to the I barycentric value
// J SRC        [3:2] Specifies which vertex in triangle 0 corresponds to the J barycentric value
#define TRIANGLE_ID_BIT_STRIDE 8

#define TRIANGLE_ID_I_SRC_SHIFT        0
#define TRIANGLE_ID_J_SRC_SHIFT        2

//=====================================================================================================================
inline uint GetNodeType(uint nodePointer)
{
    // From the HW raytracing spec:
    // node_type = node_pointer[ 2:0]
    return nodePointer & 0x7;
}

//=====================================================================================================================
inline uint WriteTriangleIdField(uint triangleId, uint nodeType, uint rotation, uint geometryFlags)
{
    const uint triangleShift = nodeType * TRIANGLE_ID_BIT_STRIDE;

    // Compute the barycentrics mapping table that is stored in triangle_id for RT IP 1.1
    triangleId |= ((rotation + 1) % 3) << (triangleShift + TRIANGLE_ID_I_SRC_SHIFT);
    triangleId |= ((rotation + 2) % 3) << (triangleShift + TRIANGLE_ID_J_SRC_SHIFT);

    return triangleId;
}

//=====================================================================================================================
inline uint CalcUncompressedTriangleId(uint geometryFlags)
{
    return WriteTriangleIdField(0, NODE_TYPE_TRIANGLE_0, 0, geometryFlags);
}

//=====================================================================================================================
// Extract the order of the triangle vertices from the node's triangle ID field.
inline uint3 CalcTriangleCompressionVertexOffsets(uint nodePointer, uint triangleId)
{
    const uint nodeType = GetNodeType(nodePointer);

    uint3 vertexSwizzle;
    vertexSwizzle.y = (triangleId >> (TRIANGLE_ID_BIT_STRIDE * nodeType + TRIANGLE_ID_I_SRC_SHIFT)) % 4;
    vertexSwizzle.z = (triangleId >> (TRIANGLE_ID_BIT_STRIDE * nodeType + TRIANGLE_ID_J_SRC_SHIFT)) % 4;
    vertexSwizzle.x = 3 - vertexSwizzle.y - vertexSwizzle.z;

    // Packed triangle vertex mapping similar to the one used by the compression algorithm.
    // node type 0 -> V0, V1, V2
    // node type 1 -> V1, V3, V2
    // node type 2 -> V2, V3, V4
    // node type 3 -> V2, V4, V0
    const uint nodeVertexMapping[4] = { 0x210, 0x231, 0x432, 0x042 };

    uint3 nodeVertexIndices;
    nodeVertexIndices.x = (nodeVertexMapping[nodeType] >> (vertexSwizzle.x * 4)) & 0xf;
    nodeVertexIndices.y = (nodeVertexMapping[nodeType] >> (vertexSwizzle.y * 4)) & 0xf;
    nodeVertexIndices.z = (nodeVertexMapping[nodeType] >> (vertexSwizzle.z * 4)) & 0xf;

    const uint nodeVertexStride = 12;

    return nodeVertexIndices * nodeVertexStride;
}

//=====================================================================================================================
// Pack the geometry index in the bottom 24 bits and the geometry flags into bits 25-26
inline uint PackGeometryIndexAndFlags(
    uint geometryIndex,
    uint geometryFlags)
{
    return (geometryFlags << 24) | (geometryIndex & 0xFFFFFF);
}

//=====================================================================================================================
// Function assumes the type passed in is a valid node type
//
inline uint PackNodePointer(uint type, uint address)
{
    uint nodePointer = type; // this assumes that the type is valid
    // uint pointer = type & 0x7;

    // The input address is a byte offset, and node_addr is a 64-byte offset that starts at bit 3.
    nodePointer |= (address >> 3); // this assumes that the input address is 64-byte aligned
    // pointer |= (address >> 6) << 3;

    return nodePointer;
}

//=====================================================================================================================
// Function assumes the type passed in is a valid node type
//
inline uint PackLeafNodePointer(uint type, uint address, uint numPrimitives)
{
    PAL_ASSERT(numPrimitives > 0);

    return PackNodePointer(type, address) | ((numPrimitives - 1) << 29);
}

////=====================================================================================================================
inline uint CalcParentPtrOffset(uint nodePtr, uint triangleCompressionMode)
{
    // Mask out node type from node pointer
    const uint alignedNodePtrOffset = (nodePtr & 0xFFFFFFF8);
    const uint nodePtrOffsetShift = 3 - triangleCompressionMode;

    // We're using triangle node type to index into the compressed triangle node
    uint compressedPrimOffset = GetNodeType(nodePtr);
    // box nodes and user nodes are not compressed. Set prim offset to 0
    compressedPrimOffset *= (compressedPrimOffset > 3) ? 0 : 1;

    // Subtract 1 from the index to account for negative offset calculations. I.e. index 0 is actually at -4 byte
    // offset from the end of the parent pointer memory
    const uint linkIndex = (alignedNodePtrOffset >> nodePtrOffsetShift) + compressedPrimOffset - 1;
    return linkIndex * NODE_PTR_SIZE;
}

//=====================================================================================================================
struct Float16BoxNode
{
    uint  child0;   // Child node pointer 0
    uint  child1;   // Child node pointer 1
    uint  child2;   // Child node pointer 2
    uint  child3;   // Child node pointer 3

    uint3 bbox0;    // Node bounding box 0, packed, uses float16: minx, miny | minz, maxx | maxy, maxz
    uint3 bbox1;    // Node bounding box 1, packed, uses float16: minx, miny | minz, maxx | maxy, maxz
    uint3 bbox2;    // Node bounding box 2, packed, uses float16: minx, miny | minz, maxx | maxy, maxz
    uint3 bbox3;    // Node bounding box 3, packed, uses float16: minx, miny | minz, maxx | maxy, maxz
};

//=====================================================================================================================
struct Float32BoxNode
{
    uint   child0;    // Child node pointer 0
    uint   child1;    // Child node pointer 1
    uint   child2;    // Child node pointer 2
    uint   child3;    // Child node pointer 3

    float3 bbox0_min; // Node bounding box 0 minimum bounds
    float3 bbox0_max; // Node bounding box 0 maximum bounds

    float3 bbox1_min; // Node bounding box 1 minimum bounds
    float3 bbox1_max; // Node bounding box 1 maximum bounds

    float3 bbox2_min; // Node bounding box 2 minimum bounds
    float3 bbox2_max; // Node bounding box 2 maximum bounds

    float3 bbox3_min; // Node bounding box 3 minimum bounds
    float3 bbox3_max; // Node bounding box 3 maximum bounds

    uint   padding0;  // Padding for 64-byte alignment
    uint   padding1;  // Padding for 64-byte alignment
    uint   padding2;  // Padding for 64-byte alignment
    uint   padding3;  // Padding for 64-byte alignment
};

//=====================================================================================================================
struct TriangleNode
{
    float3 v0;         // Vertex 0
    float3 v1;         // Vertex 1
    float3 v2;         // Vertex 2
    float3 v3;         // Vertex 3
    float3 v4;         // Vertex 4
    uint   triangleId; // Triangle ID
};

//=====================================================================================================================
struct InstanceNode
{
    InstanceDesc      desc;
    InstanceExtraData extra;
};

#define INSTANCE_NODE_DESC_OFFSET  0
#define INSTANCE_NODE_EXTRA_OFFSET 64
#define INSTANCE_NODE_SIZE         128

#define INSTANCE_BASE_POINTER_ZERO_MASK                          0x7ull
#define INSTANCE_BASE_POINTER_ADDRESS_USED_MASK       0x1FFFFFFFFFF8ull
#define INSTANCE_BASE_POINTER_ADDRESS_UNUSED_MASK   0x3FE00000000000ull
#define INSTANCE_BASE_POINTER_ADDRESS_MASK          0x3FFFFFFFFFFFF8ull
#define INSTANCE_BASE_POINTER_FLAGS_MASK          0xFFC0000000000000ull

#define NODE_POINTER_FORCE_OPAQUE_SHIFT          54
#define NODE_POINTER_FORCE_NON_OPAQUE_SHIFT      55
#define NODE_POINTER_DISABLE_TRIANGLE_CULL_SHIFT 56
#define NODE_POINTER_FLIP_FACEDNESS_SHIFT        57
#define NODE_POINTER_CULL_BACK_FACING_SHIFT      58
#define NODE_POINTER_CULL_FRONT_FACING_SHIFT     59
#define NODE_POINTER_CULL_OPAQUE_SHIFT           60
#define NODE_POINTER_CULL_NON_OPAQUE_SHIFT       61
#define NODE_POINTER_SKIP_TRIANGLES_SHIFT        62
#define NODE_POINTER_SKIP_PROCEDURAL_SHIFT       63

//=====================================================================================================================
static uint64 PackInstanceBasePointer(uint64 instanceVa, uint32 instanceFlags, uint32 geometryType)
{
    uint64 instanceBasePointer = instanceVa >> 3;

    instanceBasePointer |= (instanceFlags & static_cast<uint32>(InstanceFlag::TriangleCullDisable))
                           ? 1ull << NODE_POINTER_DISABLE_TRIANGLE_CULL_SHIFT : 0;

    instanceBasePointer |= (instanceFlags & static_cast<uint32>(InstanceFlag::TriangleFrontCounterclockwise))
                           ? 1ull << NODE_POINTER_FLIP_FACEDNESS_SHIFT : 0;

    instanceBasePointer |= (instanceFlags & static_cast<uint32>(InstanceFlag::ForceOpaque))
                           ? 1ull << NODE_POINTER_FORCE_OPAQUE_SHIFT : 0;

    instanceBasePointer |= (instanceFlags & static_cast<uint32>(InstanceFlag::ForceNonOpaque))
                           ? 1ull << NODE_POINTER_FORCE_NON_OPAQUE_SHIFT : 0;

    // Set 'Skip Procedural' for triangles and 'Skip Triangles' for procedural geometry
    instanceBasePointer |= (geometryType == static_cast<uint32>(GeometryType::Triangles))
                           ? 1ull << NODE_POINTER_SKIP_PROCEDURAL_SHIFT
                           : 1ull << NODE_POINTER_SKIP_TRIANGLES_SHIFT;

    return instanceBasePointer;
}

//=====================================================================================================================
static uint64 ExtractInstanceAddress(uint64 instanceBasePointer)
{
    return (instanceBasePointer & (INSTANCE_BASE_POINTER_ADDRESS_USED_MASK | INSTANCE_BASE_POINTER_ZERO_MASK)) << 3;
}

//=====================================================================================================================
static bool IsBoxNode16(uint pointer)
{
    return (GetNodeType(pointer) == NODE_TYPE_BOX_FLOAT16);
}

//=====================================================================================================================
static bool IsBoxNode32(uint pointer)
{
    return (GetNodeType(pointer) == NODE_TYPE_BOX_FLOAT32);
}

//=====================================================================================================================
static bool IsBoxNode(uint pointer)
{
    return IsBoxNode16(pointer) || IsBoxNode32(pointer);
}

//=====================================================================================================================
static bool IsInstanceNode(uint pointer)
{
    return (GetNodeType(pointer) == NODE_TYPE_USER_NODE_INSTANCE);
}

//=====================================================================================================================
static bool IsTriangleNode(uint pointer)
{
    return (GetNodeType(pointer) <= NODE_TYPE_TRIANGLE_3);
}

//=====================================================================================================================
// User defined procedural node format
struct ProceduralNode
{
    float3 bbox_min;
    float3 bbox_max;
    uint   padding[3];
    uint   unused;
    uint   padding2[2];
    uint   geometryIndexAndFlags;
    uint   primitiveIndex;
    uint   padding3[2];
};

};

};
