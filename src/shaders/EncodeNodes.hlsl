/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2022 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include ".\Common.hlsl"
#include ".\BuildCommon.hlsl"

#define RootSig "RootConstants(num32BitConstants=27, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u6, visibility=SHADER_VISIBILITY_ALL),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894))"

//=====================================================================================================================
// 32 bit constants
struct Constants
{
    uint metadataSizeInBytes;           // Size of the metadata header
    uint NumPrimitives;                 // Number of primitives
    uint LeafNodeDataByteOffset;        // Leaf node data byte offset
    uint PrimitiveOffset;               // Primitive Offset
    uint SceneBoundsByteOffset;         // Scene bounds byte offset
    uint PropagationFlagsScratchOffset; // Flags byte offset
    uint BaseUpdateStackScratchOffset;  // Update stack offset
    uint IndexBufferByteOffset;         // Index buffer byte offset
    uint HasValidTransform;             // Has a valid transform
    uint IndexBufferFormat;             // Index buffer format
    uint GeometryBufferStrideInBytes;   // Vertex buffer stride in bytes
    uint GeometryIndex;                 // Index of the geometry description that owns this node
    uint BaseGeometryInfoOffset;        // Base offset for the geometry info
    uint BasePrimNodePtrOffset;         // Base offset for the prim nodes
    uint BuildFlags;                    // DDI acceleration structure build flags
    uint isUpdateInPlace;               // Is update in place
    uint GeometryFlags;                 // Geometry flags (D3D12_RAYTRACING_GEOMETRY_FLAGS)
    uint VertexBufferFormat;            // Vertex buffer format
    uint vertexCount;                   // Vertex count
    uint DestLeafByteOffset;            // Offset to this geometry's location in the dest leaf node buffer
    uint LeafNodeExpansionFactor;       // Leaf node expansion factor (> 1 for triangle splitting)
    uint TriangleCompressionMode;       // Triangle Compression mode
    uint IndexBufferInfoScratchOffset;
    uint IndexBufferVaLo;
    uint IndexBufferVaHi;
    uint enableCentroidSceneBoundsWithSize;
    uint enableTriangleSplitting;
};

//=====================================================================================================================
[[vk::binding(0, 1)]] ConstantBuffer<Constants>   ShaderConstants : register(b0);

[[vk::binding(0, 0)]] RWByteAddressBuffer         GeometryBuffer  : register(u0);

// Triangle nodes only
[[vk::binding(1, 0)]] RWByteAddressBuffer         IndexBuffer     : register(u1);
[[vk::binding(2, 0)]] RWStructuredBuffer<float4>  TransformBuffer : register(u2);

[[vk::binding(3, 0)]] RWByteAddressBuffer ResultBuffer      : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer ScratchBuffer     : register(u4);
[[vk::binding(5, 0)]] RWByteAddressBuffer SourceBuffer      : register(u5);
[[vk::binding(6, 0)]] RWByteAddressBuffer IndirectArgBuffer : register(u6);

struct IndirectBuildOffset
{
    uint primitiveCount;
    uint primitiveOffset;
    uint firstVertex;
    uint transformOffset;
};

//=====================================================================================================================
#define VERTEX_FORMAT_R32G32B32_FLOAT     1
#define VERTEX_FORMAT_R32G32_FLOAT        2
#define VERTEX_FORMAT_R16G16B16A16_FLOAT  3
#define VERTEX_FORMAT_R16G16_FLOAT        4
#define VERTEX_FORMAT_R16G16B16A16_SNORM  5
#define VERTEX_FORMAT_R16G16_SNORM        6
#define VERTEX_FORMAT_R16G16B16A16_UNORM  7
#define VERTEX_FORMAT_R16G16_UNORM        8
#define VERTEX_FORMAT_R10G10B10A2_UNORM   9
#define VERTEX_FORMAT_R8G8B8A8_SNORM     10
#define VERTEX_FORMAT_R8G8_SNORM         11
#define VERTEX_FORMAT_R8G8B8A8_UNORM     12
#define VERTEX_FORMAT_R8G8_UNORM         13

//=====================================================================================================================
#define PRIM_BATCH_STRIDE 16

//=====================================================================================================================
// Get face indices from 16-bit index buffer
uint3 GetFaceIndices16(RWByteAddressBuffer buffer, uint faceIndex, uint indexBufferByteOffset)
{
    const uint stride = 6; // 3 vertices per triangle with 2-byte indices
    uint address = (faceIndex * stride) + indexBufferByteOffset;

    // Load address must be 4-byte aligned
    const bool unalignedRead = (address % 4 == 2);
    if (unalignedRead)
    {
        // Align down load address
        address -= 2;
    }

    // Load index buffer data
    const uint2 data = buffer.Load2(address);

    uint3 faceIndices;
    if (unalignedRead == false)
    {
        faceIndices.x = (data.x & 0xFFFF);
        faceIndices.y = (data.x >> 16);
        faceIndices.z = (data.y & 0xFFFF);
    }
    else
    {
        faceIndices.x = (data.x >> 16);
        faceIndices.y = (data.y & 0xFFFF);
        faceIndices.z = (data.y >> 16);
    }

    return faceIndices;
}

//=====================================================================================================================
// Get face indices from 16-bit index buffer
uint FetchIndex16(RWByteAddressBuffer buffer, uint faceIndex, uint indexOffset, uint indexBufferByteOffset)
{
    const uint indexStride = 2;
    const uint faceStride = 3 * indexStride; // 3 vertices per triangle with 2-byte indices
    uint address = (faceIndex * faceStride) + (indexOffset * indexStride) + indexBufferByteOffset;

    // Load address must be 4-byte aligned
    const bool unalignedRead = (address % 4 == 2);
    if (unalignedRead)
    {
        // Align down load address
        address -= 2;
    }

    // Load index buffer data
    const uint data = buffer.Load(address);

    uint index;
    if (unalignedRead == false)
    {
        index = (data & 0xFFFF);
    }
    else
    {
        index = (data >> 16);
    }

    return index;
}

//=====================================================================================================================
// Get face indices from 32-bit index buffer
uint FetchIndex32(RWByteAddressBuffer buffer, uint faceIndex, uint indexOffset, uint indexBufferByteOffset)
{
    const uint indexStride = 4;
    const uint faceStride = 3 * indexStride; // 3 vertices per triangle with 4-byte indices

    const uint baseOffset = (faceIndex * faceStride) + (indexOffset * indexStride) + indexBufferByteOffset;
    return buffer.Load(baseOffset);
}

//=====================================================================================================================
// Get face indices from 32-bit index buffer
uint3 GetFaceIndices32(RWByteAddressBuffer buffer, uint faceIndex, uint indexBufferByteOffset)
{
    const uint stride = 12; // 3 vertices per triangle with 4-byte indices

    const uint baseOffset = (faceIndex * 12) + indexBufferByteOffset;
    return buffer.Load3(baseOffset);
}

//=====================================================================================================================
// DXGI_FORMAT_R32G32B32_FLOAT
float3 LoadVertexR32G32B32Float(RWByteAddressBuffer buffer, uint byteOffset)
{
    return asfloat(buffer.Load3(byteOffset));
}

//=====================================================================================================================
// DXGI_FORMAT_R32G32_FLOAT
float3 LoadVertexR32G32Float(RWByteAddressBuffer buffer, uint byteOffset)
{
    float3 vertex;
    vertex.xy = asfloat(buffer.Load2(byteOffset));
    vertex.z = 0.0f;

    return vertex;
}

//=====================================================================================================================
// DXGI_FORMAT_R16G16B16A16_FLOAT
float3 LoadVertexR16G16B16A16Float(RWByteAddressBuffer buffer, uint byteOffset)
{
    // Load address must be 4-byte aligned
    uint address = byteOffset;
    const bool unaligned = (byteOffset % 4 == 2);
    if (unaligned)
    {
        // Align load address down
        address -= 2;
    }

    uint2 data = buffer.Load2(address);

    float3 vertex;
    if (unaligned)
    {
        vertex.x = f16tof32(data.x >> 16);
        vertex.y = f16tof32(data.y & 0xFFFF);
        vertex.z = f16tof32(data.y >> 16);
    }
    else
    {
        vertex.x = f16tof32(data.x & 0xFFFF);
        vertex.y = f16tof32(data.x >> 16);
        vertex.z = f16tof32(data.y & 0xFFFF);
    }

    return vertex;
}

//=====================================================================================================================
// DXGI_FORMAT_R16G16_FLOAT
float3 LoadVertexR16G16Float(RWByteAddressBuffer buffer, uint byteOffset)
{
    // Load address must be 4-byte aligned
    uint address = byteOffset;
    const bool unaligned = (byteOffset % 4 == 2);
    if (unaligned)
    {
        // Align load address down
        address -= 2;
    }

    uint2 data = buffer.Load2(address);

    float3 vertex;
    if (unaligned)
    {
        vertex.x = f16tof32(data.x >> 16);
        vertex.y = f16tof32(data.y & 0xFFFF);
    }
    else
    {
        vertex.x = f16tof32(data.x & 0xFFFF);
        vertex.y = f16tof32(data.x >> 16);
    }
    vertex.z = 0.0f;

    return vertex;
}

//=====================================================================================================================
int SignExtend16to32(uint input)
{
    int output = asint(input);
    output <<= 16;
    output >>= 16;
    return output;
}

//=====================================================================================================================
float ConvertSnorm16ToFloat(uint input)
{
    // 16-bit SNORM maps [-INT16_MAX, INT16_MAX] to [-1.0f to 1.0f]; INT16_MIN also maps to -1.0f.
    const uint SNORM16_MAX = 0x7FFF;
    const int inputVal = SignExtend16to32(input);
    return max((inputVal / (float)SNORM16_MAX), -1.0f);
}

//=====================================================================================================================
// DXGI_FORMAT_R16G16B16A16_SNORM
float3 LoadVertexR16G16B16A16Snorm(RWByteAddressBuffer buffer, uint byteOffset)
{
    // Load address must be 4-byte aligned
    uint address = byteOffset;
    const bool unaligned = (byteOffset % 4 == 2);
    if (unaligned)
    {
        // Align load address down
        address -= 2;
    }

    uint2 data = buffer.Load2(address);

    float3 vertex;
    if (unaligned)
    {
        vertex.x = ConvertSnorm16ToFloat(data.x >> 16);
        vertex.y = ConvertSnorm16ToFloat(data.y & 0xFFFF);
        vertex.z = ConvertSnorm16ToFloat(data.y >> 16);
    }
    else
    {
        vertex.x = ConvertSnorm16ToFloat(data.x & 0xFFFF);
        vertex.y = ConvertSnorm16ToFloat(data.x >> 16);
        vertex.z = ConvertSnorm16ToFloat(data.y & 0xFFFF);
    }

    return vertex;
}

//=====================================================================================================================
// DXGI_FORMAT_R16G16_SNORM
float3 LoadVertexR16G16Snorm(RWByteAddressBuffer buffer, uint byteOffset)
{
    // Load address must be 4-byte aligned
    uint address = byteOffset;
    const bool unaligned = (byteOffset % 4 == 2);
    if (unaligned)
    {
        // Align load address down
        address -= 2;
    }

    uint2 data = buffer.Load2(address);

    float3 vertex;
    if (unaligned)
    {
        vertex.x = ConvertSnorm16ToFloat(data.x >> 16);
        vertex.y = ConvertSnorm16ToFloat(data.y & 0xFFFF);
    }
    else
    {
        vertex.x = ConvertSnorm16ToFloat(data.x & 0xFFFF);
        vertex.y = ConvertSnorm16ToFloat(data.x >> 16);
    }

    vertex.z = 0.0f;

    return vertex;
}

//=====================================================================================================================
float ConvertUnorm16ToFloat(uint input)
{
    return input / (float)0xFFFF;
}

//=====================================================================================================================
// DXGI_FORMAT_R16G16B16A16_UNORM
float3 LoadVertexR16G16B16A16Unorm(RWByteAddressBuffer buffer, uint byteOffset)
{
    // Load address must be 4-byte aligned
    uint address = byteOffset;
    const bool unaligned = (byteOffset % 4 == 2);
    if (unaligned)
    {
        // Align load address down
        address -= 2;
    }

    uint2 data = buffer.Load2(address);

    float3 vertex;
    if (unaligned)
    {
        vertex.x = ConvertUnorm16ToFloat(data.x >> 16);
        vertex.y = ConvertUnorm16ToFloat(data.y & 0xFFFF);
        vertex.z = ConvertUnorm16ToFloat(data.y >> 16);
    }
    else
    {
        vertex.x = ConvertUnorm16ToFloat(data.x & 0xFFFF);
        vertex.y = ConvertUnorm16ToFloat(data.x >> 16);
        vertex.z = ConvertUnorm16ToFloat(data.y & 0xFFFF);
    }

    return vertex;
}

//=====================================================================================================================
// DXGI_FORMAT_R16G16_UNORM
float3 LoadVertexR16G16Unorm(RWByteAddressBuffer buffer, uint byteOffset)
{
    // Load address must be 4-byte aligned
    uint address = byteOffset;
    const bool unaligned = (byteOffset % 4 == 2);
    if (unaligned)
    {
        // Align load address down
        address -= 2;
    }

    uint2 data = buffer.Load2(address);

    float3 vertex;
    if (unaligned)
    {
        vertex.x = ConvertUnorm16ToFloat(data.x >> 16);
        vertex.y = ConvertUnorm16ToFloat(data.y & 0xFFFF);
    }
    else
    {
        vertex.x = ConvertUnorm16ToFloat(data.x & 0xFFFF);
        vertex.y = ConvertUnorm16ToFloat(data.x >> 16);
    }

    vertex.z = 0.0f;

    return vertex;
}

//=====================================================================================================================
float ConvertUnorm10ToFloat(uint input)
{
    return input / (float)0x3FF;
}

//=====================================================================================================================
// DXGI_FORMAT_R10G10B10A2_UNORM
float3 LoadVertexR10G10B10A2Unorm(RWByteAddressBuffer buffer, uint byteOffset)
{
    // Load address must be 4-byte aligned
    uint address = byteOffset;
    const bool unaligned = (byteOffset % 4 == 2);
    if (unaligned)
    {
        // Align load address down
        address -= 2;
    }

    uint2 dataRaw = buffer.Load2(address);

    uint data = 0;

    float3 vertex;
    if (unaligned)
    {
        data = (dataRaw.x & 0xFFFF0000) | (dataRaw.y & 0xFFFF);
    }
    else
    {
        data = dataRaw.x;
    }

    vertex.x = ConvertUnorm10ToFloat(data & 0x3FF);
    vertex.y = ConvertUnorm10ToFloat((data >> 10) & 0x3FF);
    vertex.z = ConvertUnorm10ToFloat((data >> 20) & 0x3FF);

    return vertex;
}

//=====================================================================================================================
float ConvertUnorm8ToFloat(uint input)
{
    return input / (float)0xFF;
}

//=====================================================================================================================
// DXGI_FORMAT_R8G8B8A8_UNORM
float3 LoadVertexR8G8B8A8Unorm(RWByteAddressBuffer buffer, uint byteOffset)
{
    // round load address down to multiple of 4
    uint address = byteOffset & ~3;

    uint2 data = buffer.Load2(address);

    float3 vertex;

    // gets tricky with a 3-byte stride
    //                       X Y Z X | Y Z X Y | Z X Y Z
    // byteOffset % 4 == 0 : X Y Z - | - - - - | - - - -
    // byteOffset % 4 == 3 : - - - X | Y Z - - | - - - -
    // byteOffset % 4 == 2 : - - - - | - - X Y | Z - - -
    // byteOffset % 4 == 1 : - - - - | - - - - | - X Y Z
    // (this image is in big-endian, but the load is little-endian, which makes it even weirder)
    // (so really it's       X Z Y X | Y X Z Y | Z Y X Z                                       )
    switch (byteOffset & 3) // equivalent to byteOffset % 4
    {
    case 0:
        vertex.x = ConvertUnorm8ToFloat(data.x & 0xFF);
        vertex.y = ConvertUnorm8ToFloat((data.x >> 8) & 0xFF);
        vertex.z = ConvertUnorm8ToFloat((data.x >> 16) & 0xFF);
        break;
    case 3:
        vertex.x = ConvertUnorm8ToFloat(data.x >> 24);
        vertex.y = ConvertUnorm8ToFloat(data.y & 0xFF);
        vertex.z = ConvertUnorm8ToFloat((data.y >> 8) & 0xFF);
        break;
    case 2:
        vertex.x = ConvertUnorm8ToFloat((data.x >> 16) & 0xFF);
        vertex.y = ConvertUnorm8ToFloat(data.x >> 24);
        vertex.z = ConvertUnorm8ToFloat(data.y & 0xFF);
        break;
    case 1:
        vertex.x = ConvertUnorm8ToFloat((data.x >> 8) & 0xFF);
        vertex.y = ConvertUnorm8ToFloat((data.x >> 16) & 0xFF);
        vertex.z = ConvertUnorm8ToFloat(data.x >> 24);
        break;
    }

    return vertex;
}

//=====================================================================================================================
// DXGI_FORMAT_R8G8_UNORM
float3 LoadVertexR8G8Unorm(RWByteAddressBuffer buffer, uint byteOffset)
{
    // round load address down to multiple of 4
    uint address = byteOffset & ~3;

    uint2 data = buffer.Load2(address);

    float3 vertex;

    switch (byteOffset & 3) // equivalent to byteOffset % 4
    {
    case 0:
        vertex.x = ConvertUnorm8ToFloat(data.x & 0xFF);
        vertex.y = ConvertUnorm8ToFloat((data.x >> 8) & 0xFF);
        break;
    case 3:
        vertex.x = ConvertUnorm8ToFloat(data.x >> 24);
        vertex.y = ConvertUnorm8ToFloat(data.y & 0xFF);
        break;
    case 2:
        vertex.x = ConvertUnorm8ToFloat((data.x >> 16) & 0xFF);
        vertex.y = ConvertUnorm8ToFloat(data.x >> 24);
        break;
    case 1:
        vertex.x = ConvertUnorm8ToFloat((data.x >> 8) & 0xFF);
        vertex.y = ConvertUnorm8ToFloat((data.x >> 16) & 0xFF);
        break;
    }

    vertex.z = 0.0f;

    return vertex;
}

//=====================================================================================================================
int SignExtend8to32(uint input)
{
    int output = asint(input);
    output <<= 24;
    output >>= 24;
    return output;
}

//=====================================================================================================================
float ConvertSnorm8ToFloat(uint input)
{
    const uint SNORM8_MAX = 0x7F;
    const int inputVal = SignExtend8to32(input);
    return max((inputVal / (float)SNORM8_MAX), -1.0f);
}

//=====================================================================================================================
// DXGI_FORMAT_R8G8B8A8_SNORM
float3 LoadVertexR8G8B8A8Snorm(RWByteAddressBuffer buffer, uint byteOffset)
{
    // round load address down to multiple of 4
    uint address = byteOffset & ~3;

    uint2 data = buffer.Load2(address);

    float3 vertex;

    // gets tricky with a 3-byte stride
    //                       X Y Z X | Y Z X Y | Z X Y Z
    // byteOffset % 4 == 0 : X Y Z - | - - - - | - - - -
    // byteOffset % 4 == 3 : - - - X | Y Z - - | - - - -
    // byteOffset % 4 == 2 : - - - - | - - X Y | Z - - -
    // byteOffset % 4 == 1 : - - - - | - - - - | - X Y Z
    // (this image is in big-endian, but the load is little-endian, which makes it even weirder)
    // (so really it's       X Z Y X | Y X Z Y | Z Y X Z                                       )
    switch (byteOffset & 3) // equivalent to byteOffset % 4
    {
    case 0:
        vertex.x = ConvertSnorm8ToFloat(data.x & 0xFF);
        vertex.y = ConvertSnorm8ToFloat((data.x >> 8) & 0xFF);
        vertex.z = ConvertSnorm8ToFloat((data.x >> 16) & 0xFF);
        break;
    case 3:
        vertex.x = ConvertSnorm8ToFloat(data.x >> 24);
        vertex.y = ConvertSnorm8ToFloat(data.y & 0xFF);
        vertex.z = ConvertSnorm8ToFloat((data.y >> 8) & 0xFF);
        break;
    case 2:
        vertex.x = ConvertSnorm8ToFloat((data.x >> 16) & 0xFF);
        vertex.y = ConvertSnorm8ToFloat(data.x >> 24);
        vertex.z = ConvertSnorm8ToFloat(data.y & 0xFF);
        break;
    case 1:
        vertex.x = ConvertSnorm8ToFloat((data.x >> 8) & 0xFF);
        vertex.y = ConvertSnorm8ToFloat((data.x >> 16) & 0xFF);
        vertex.z = ConvertSnorm8ToFloat(data.x >> 24);
        break;
    }

    return vertex;
}

//=====================================================================================================================
// DXGI_FORMAT_R8G8_SNORM
float3 LoadVertexR8G8Snorm(RWByteAddressBuffer buffer, uint byteOffset)
{
    // round load address down to multiple of 4
    uint address = byteOffset & ~3;

    uint2 data = buffer.Load2(address);

    float3 vertex;

    switch (byteOffset & 3) // equivalent to byteOffset % 4
    {
    case 0:
        vertex.x = ConvertSnorm8ToFloat(data.x & 0xFF);
        vertex.y = ConvertSnorm8ToFloat((data.x >> 8) & 0xFF);
        break;
    case 3:
        vertex.x = ConvertSnorm8ToFloat(data.x >> 24);
        vertex.y = ConvertSnorm8ToFloat(data.y & 0xFF);
        break;
    case 2:
        vertex.x = ConvertSnorm8ToFloat((data.x >> 16) & 0xFF);
        vertex.y = ConvertSnorm8ToFloat(data.x >> 24);
        break;
    case 1:
        vertex.x = ConvertSnorm8ToFloat((data.x >> 8) & 0xFF);
        vertex.y = ConvertSnorm8ToFloat((data.x >> 16) & 0xFF);
        break;
    }

    vertex.z = 0.0f;

    return vertex;
}

//=====================================================================================================================
TriangleData FetchTriangleData(RWByteAddressBuffer buffer, uint3 index, uint vertexByteStride, uint vertexOffset)
{
    TriangleData tri;

    const uint3 offset = (index + vertexOffset) * vertexByteStride;

    switch (ShaderConstants.VertexBufferFormat)
    {
    case VERTEX_FORMAT_R32G32B32_FLOAT:
        tri.v0 = LoadVertexR32G32B32Float(buffer, offset.x);
        tri.v1 = LoadVertexR32G32B32Float(buffer, offset.y);
        tri.v2 = LoadVertexR32G32B32Float(buffer, offset.z);
        break;
    case VERTEX_FORMAT_R32G32_FLOAT:
        tri.v0 = LoadVertexR32G32Float(buffer, offset.x);
        tri.v1 = LoadVertexR32G32Float(buffer, offset.y);
        tri.v2 = LoadVertexR32G32Float(buffer, offset.z);
        break;
    case VERTEX_FORMAT_R16G16B16A16_FLOAT:
        tri.v0 = LoadVertexR16G16B16A16Float(buffer, offset.x);
        tri.v1 = LoadVertexR16G16B16A16Float(buffer, offset.y);
        tri.v2 = LoadVertexR16G16B16A16Float(buffer, offset.z);
        break;
    case VERTEX_FORMAT_R16G16_FLOAT:
        tri.v0 = LoadVertexR16G16Float(buffer, offset.x);
        tri.v1 = LoadVertexR16G16Float(buffer, offset.y);
        tri.v2 = LoadVertexR16G16Float(buffer, offset.z);
        break;
    case VERTEX_FORMAT_R16G16B16A16_SNORM:
        tri.v0 = LoadVertexR16G16B16A16Snorm(buffer, offset.x);
        tri.v1 = LoadVertexR16G16B16A16Snorm(buffer, offset.y);
        tri.v2 = LoadVertexR16G16B16A16Snorm(buffer, offset.z);
        break;
    case VERTEX_FORMAT_R16G16_SNORM:
        tri.v0 = LoadVertexR16G16Snorm(buffer, offset.x);
        tri.v1 = LoadVertexR16G16Snorm(buffer, offset.y);
        tri.v2 = LoadVertexR16G16Snorm(buffer, offset.z);
        break;
    case VERTEX_FORMAT_R16G16B16A16_UNORM:
        tri.v0 = LoadVertexR16G16B16A16Unorm(buffer, offset.x);
        tri.v1 = LoadVertexR16G16B16A16Unorm(buffer, offset.y);
        tri.v2 = LoadVertexR16G16B16A16Unorm(buffer, offset.z);
        break;
    case VERTEX_FORMAT_R16G16_UNORM:
        tri.v0 = LoadVertexR16G16Unorm(buffer, offset.x);
        tri.v1 = LoadVertexR16G16Unorm(buffer, offset.y);
        tri.v2 = LoadVertexR16G16Unorm(buffer, offset.z);
        break;
    case VERTEX_FORMAT_R10G10B10A2_UNORM:
        tri.v0 = LoadVertexR10G10B10A2Unorm(buffer, offset.x);
        tri.v1 = LoadVertexR10G10B10A2Unorm(buffer, offset.y);
        tri.v2 = LoadVertexR10G10B10A2Unorm(buffer, offset.z);
        break;
    case VERTEX_FORMAT_R8G8B8A8_UNORM:
        tri.v0 = LoadVertexR8G8B8A8Unorm(buffer, offset.x);
        tri.v1 = LoadVertexR8G8B8A8Unorm(buffer, offset.y);
        tri.v2 = LoadVertexR8G8B8A8Unorm(buffer, offset.z);
        break;
    case VERTEX_FORMAT_R8G8_UNORM:
        tri.v0 = LoadVertexR8G8Unorm(buffer, offset.x);
        tri.v1 = LoadVertexR8G8Unorm(buffer, offset.y);
        tri.v2 = LoadVertexR8G8Unorm(buffer, offset.z);
        break;
    case VERTEX_FORMAT_R8G8B8A8_SNORM:
        tri.v0 = LoadVertexR8G8B8A8Snorm(buffer, offset.x);
        tri.v1 = LoadVertexR8G8B8A8Snorm(buffer, offset.y);
        tri.v2 = LoadVertexR8G8B8A8Snorm(buffer, offset.z);
        break;
    case VERTEX_FORMAT_R8G8_SNORM:
        tri.v0 = LoadVertexR8G8Snorm(buffer, offset.x);
        tri.v1 = LoadVertexR8G8Snorm(buffer, offset.y);
        tri.v2 = LoadVertexR8G8Snorm(buffer, offset.z);
        break;
    }

    return tri;
}

//=====================================================================================================================
float3 FetchVertex(RWByteAddressBuffer buffer, uint index)
{
    float3 vertex;

    switch (ShaderConstants.VertexBufferFormat)
    {
        case VERTEX_FORMAT_R32G32B32_FLOAT:
            vertex = LoadVertexR32G32B32Float(buffer, ShaderConstants.GeometryBufferStrideInBytes * index);
            break;
        case VERTEX_FORMAT_R32G32_FLOAT:
            vertex = LoadVertexR32G32Float(buffer, ShaderConstants.GeometryBufferStrideInBytes * index);
            break;
        case VERTEX_FORMAT_R16G16B16A16_FLOAT:
            vertex = LoadVertexR16G16B16A16Float(buffer, ShaderConstants.GeometryBufferStrideInBytes * index);
            break;
        case VERTEX_FORMAT_R16G16_FLOAT:
            vertex = LoadVertexR16G16Float(buffer, ShaderConstants.GeometryBufferStrideInBytes * index);
            break;
        case VERTEX_FORMAT_R16G16B16A16_SNORM:
            vertex = LoadVertexR16G16B16A16Snorm(buffer, ShaderConstants.GeometryBufferStrideInBytes * index);
            break;
        case VERTEX_FORMAT_R16G16_SNORM:
            vertex = LoadVertexR16G16Snorm(buffer, ShaderConstants.GeometryBufferStrideInBytes * index);
            break;
        case VERTEX_FORMAT_R16G16B16A16_UNORM:
            vertex = LoadVertexR16G16B16A16Unorm(buffer, ShaderConstants.GeometryBufferStrideInBytes * index);
            break;
        case VERTEX_FORMAT_R16G16_UNORM:
            vertex = LoadVertexR16G16Unorm(buffer, ShaderConstants.GeometryBufferStrideInBytes * index);
            break;
        case VERTEX_FORMAT_R10G10B10A2_UNORM:
            vertex = LoadVertexR10G10B10A2Unorm(buffer, ShaderConstants.GeometryBufferStrideInBytes * index);
            break;
        case VERTEX_FORMAT_R8G8B8A8_UNORM:
            vertex = LoadVertexR8G8B8A8Unorm(buffer, ShaderConstants.GeometryBufferStrideInBytes * index);
            break;
        case VERTEX_FORMAT_R8G8_UNORM:
            vertex = LoadVertexR8G8Unorm(buffer, ShaderConstants.GeometryBufferStrideInBytes * index);
            break;
        case VERTEX_FORMAT_R8G8B8A8_SNORM:
            vertex = LoadVertexR8G8B8A8Snorm(buffer, ShaderConstants.GeometryBufferStrideInBytes * index);
            break;
        case VERTEX_FORMAT_R8G8_SNORM:
            vertex = LoadVertexR8G8Snorm(buffer, ShaderConstants.GeometryBufferStrideInBytes * index);
            break;
        default:
            vertex = float3(0, 0, 0);
            break;
    }

    return vertex;
}

//=====================================================================================================================
void WriteScratchTriangleNode(
    RWByteAddressBuffer buffer,
    uint                primitiveOffset,
    uint                primitiveIndex,
    uint                geometryIndex,
    uint                flags,
    in TriangleData     tri)
{
    uint offset = (primitiveIndex  * ByteStrideScratchNode) +
                  (primitiveOffset * ByteStrideScratchNode) +
                  ShaderConstants.LeafNodeDataByteOffset;

    uint4 data;

    // LeafNode.bbox_min_or_v0, primitiveIndex
    data = uint4(asuint(tri.v0), primitiveIndex);
    buffer.Store4(offset + SCRATCH_NODE_V0_OFFSET, data);

    // LeafNode.bbox_max_or_v1, geometryIndex
    data = uint4(asuint(tri.v1), geometryIndex);
    buffer.Store4(offset + SCRATCH_NODE_V1_OFFSET, data);

    // LeafNode.v2, parent
    data = uint4(asuint(tri.v2), 0);
    buffer.Store4(offset + SCRATCH_NODE_V2_OFFSET, data);

    // type, flags, splitBox, numPrimitivesAndDoCollapse
    const uint triangleId        = CalcUncompressedTriangleId(flags);
    const uint triangleTypeAndId = (triangleId << 3) | NODE_TYPE_TRIANGLE_0;
    data = uint4(triangleTypeAndId, flags, INVALID_IDX, 1 << 1);
    buffer.Store4(offset + SCRATCH_NODE_TYPE_OFFSET, data);
}

//=====================================================================================================================
#if INDIRECT_BUILD
uint ComputePrimitiveOffset()
{
    uint primitiveOffset = 0;

    const uint metadataSize = IsUpdate(ShaderConstants.BuildFlags) ?
                              SourceBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : ShaderConstants.metadataSizeInBytes;

    // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
    const AccelStructOffsets offsets =
        SourceBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

    const uint baseGeometryInfoOffset =
        IsUpdate(ShaderConstants.BuildFlags) ? offsets.geometryInfo : ShaderConstants.BaseGeometryInfoOffset;

    for (uint geomIdx = 0; geomIdx < ShaderConstants.GeometryIndex; ++geomIdx)
    {
        const uint geometryInfoOffset =
            metadataSize + baseGeometryInfoOffset +
            (geomIdx * GEOMETRY_INFO_SIZE);

        GeometryInfo info;
        info = ResultBuffer.Load<GeometryInfo>(geometryInfoOffset);
        uint primitiveCount = ExtractGeometryInfoNumPrimitives(info.geometryFlagsAndNumPrimitives);

        primitiveOffset += primitiveCount;
    }

    return primitiveOffset;
}
#endif

//=====================================================================================================================
void WriteGeometryInfo(
        uint globalId,
        uint primitiveOffset,
        uint numPrimitives,
        uint primitiveStride)
{
    if (globalId == 0)
    {
        // For builds and not-in-place updates, write the geometry info.
        if ((IsUpdate(ShaderConstants.BuildFlags) == false) ||
            (IsUpdate(ShaderConstants.BuildFlags) &&
             (ShaderConstants.isUpdateInPlace == false)))
        {
            const uint metadataSize = IsUpdate(ShaderConstants.BuildFlags) ?
                SourceBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : ShaderConstants.metadataSizeInBytes;

            // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
            const AccelStructOffsets offsets =
                SourceBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

            const uint baseGeometryInfoOffset =
                IsUpdate(ShaderConstants.BuildFlags) ? offsets.geometryInfo : ShaderConstants.BaseGeometryInfoOffset;

            const uint geometryInfoOffset =
                metadataSize + baseGeometryInfoOffset +
                (ShaderConstants.GeometryIndex * GEOMETRY_INFO_SIZE);

            GeometryInfo info;
            info.geometryBufferOffset = primitiveOffset * primitiveStride;
            info.primNodePtrsOffset   = primitiveOffset * sizeof(uint);
            info.geometryFlagsAndNumPrimitives  =
                PackGeometryFlagsAndNumPrimitives(ShaderConstants.GeometryFlags, numPrimitives);

            ResultBuffer.Store<GeometryInfo>(geometryInfoOffset, info);
        }

        if ((IsUpdate(ShaderConstants.BuildFlags) == false) &&
            (ShaderConstants.TriangleCompressionMode == PAIR_TRIANGLE_COMPRESSION))
        {
            IndexBufferInfo indexBufferInfo;
            indexBufferInfo.gpuVaLo    = ShaderConstants.IndexBufferVaLo;
            indexBufferInfo.gpuVaHi    = ShaderConstants.IndexBufferVaHi;
            indexBufferInfo.byteOffset = ShaderConstants.IndexBufferByteOffset;
            indexBufferInfo.format     = ShaderConstants.IndexBufferFormat;

            const uint indexBufferInfoOffset = ShaderConstants.IndexBufferInfoScratchOffset +
                                               (ShaderConstants.GeometryIndex * sizeof(IndexBufferInfo));
            ScratchBuffer.Store<IndexBufferInfo>(indexBufferInfoOffset, indexBufferInfo);
        }
    }
}

//=====================================================================================================================
uint UpdateLeafNodeCounters(uint numLeafNodesOffset, uint numLeafNodes)
{
    uint currentNumNodes;
    if (IsUpdate(ShaderConstants.BuildFlags) == false)
    {
        ResultBuffer.InterlockedAdd(numLeafNodesOffset, numLeafNodes, currentNumNodes);
    }
    else
    {
        currentNumNodes = ResultBuffer.Load(numLeafNodesOffset);
    }

    return currentNumNodes;
}

//=====================================================================================================================
TriangleData FetchTransformedTriangleData(
    in RWByteAddressBuffer        geometryBuffer,
    in uint3                      faceIndices,
    in uint                       geometryBufferStrideInBytes,
    in uint                       vertexOffset,
    in bool                       hasValidTransform,
    in RWStructuredBuffer<float4> transformBuffer)
{
    // Fetch triangle vertex data from vertex buffer
    TriangleData tri = FetchTriangleData(geometryBuffer, faceIndices, geometryBufferStrideInBytes, vertexOffset);

    // If this geometry has a valid transformation matrix. Transform each vertex using this matrix.
    if (hasValidTransform)
    {
        float4x4 transform;
        transform[0] = transformBuffer[0];
        transform[1] = transformBuffer[1];
        transform[2] = transformBuffer[2];
        transform[3] = float4(0, 0, 0, 1);

        tri.v0 = mul(transform, float4(tri.v0, 1)).xyz;
        tri.v1 = mul(transform, float4(tri.v1, 1)).xyz;
        tri.v2 = mul(transform, float4(tri.v2, 1)).xyz;
    }

    if (any(isinf(tri.v0)) || any(isinf(tri.v1)) || any(isinf(tri.v2)))
    {
        tri.v0 = float3(0, 0, 0);
        tri.v1 = float3(0, 0, 0);
        tri.v2 = float3(0, 0, 0);
    }

    return tri;
}

//=====================================================================================================================
float3 FetchTransformedVertex(
    in RWByteAddressBuffer        geometryBuffer,
    in uint                       index,
    in uint                       vertexOffset,
    in bool                       hasValidTransform,
    in RWStructuredBuffer<float4> transformBuffer)
{
    // Fetch triangle vertex data from vertex buffer
    float3 vertex = FetchVertex(geometryBuffer, index + vertexOffset);

    // If this geometry has a valid transformation matrix. Transform each vertex using this matrix.
    if (hasValidTransform)
    {
        float4x4 transform;
        transform[0] = transformBuffer[0];
        transform[1] = transformBuffer[1];
        transform[2] = transformBuffer[2];
        transform[3] = float4(0, 0, 0, 1);

        vertex = mul(transform, float4(vertex, 1)).xyz;
    }

    if (any(isinf(vertex)))
    {
        vertex = float3(0, 0, 0);
    }

    return vertex;
}

//=====================================================================================================================
uint3 FetchFaceIndices(
    in RWByteAddressBuffer buffer,
    in uint                index,
    in uint                indexBufferByteOffset,
    in uint                indexBufferFormat)
{
    // Fetch face indices from index buffer
    uint3 faceIndices;
    if (indexBufferFormat == IndexFormatU16)
    {
        faceIndices = GetFaceIndices16(buffer, index, indexBufferByteOffset);
    }
    else if (indexBufferFormat == IndexFormatU32)
    {
        faceIndices = GetFaceIndices32(buffer, index, indexBufferByteOffset);
    }
    else
    {
        const uint startIndex = (index * 3);
        faceIndices.x = startIndex;
        faceIndices.y = startIndex + 1;
        faceIndices.z = startIndex + 2;
    }
    return faceIndices;
}

//=====================================================================================================================
uint FetchIndex(
    in RWByteAddressBuffer buffer,
    in uint                primitiveIndex,
    in uint                indexOffset,
    in uint                indexBufferByteOffset,
    in uint                indexBufferFormat)
{
    // Fetch face indices from index buffer
    uint index;
    if (indexBufferFormat == IndexFormatU16)
    {
        index = FetchIndex16(buffer, primitiveIndex, indexOffset, indexBufferByteOffset);
    }
    else if (indexBufferFormat == IndexFormatU32)
    {
        index = FetchIndex32(buffer, primitiveIndex, indexOffset, indexBufferByteOffset);
    }
    else
    {
        index = (primitiveIndex * 3) + indexOffset;
    }

    return index;
}

//=====================================================================================================================
bool IsActive(TriangleData tri)
{
    return ((isnan(tri.v0.x) == false) && (isnan(tri.v1.x) == false) && (isnan(tri.v2.x) == false));
}

//=====================================================================================================================
// Increment task counter to mark a task / primitive as done
void IncrementTaskCounter()
{
    DeviceMemoryBarrier();
    ResultBuffer.InterlockedAdd(ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET, 1);
}

//=====================================================================================================================
void PushNodeForUpdate(
    RWByteAddressBuffer SourceBuffer,
    RWByteAddressBuffer ResultBuffer,
    RWByteAddressBuffer ScratchBuffer,
    uint                metadataSize,
    uint                baseUpdateStackScratchOffset,
    uint                triangleCompressionMode,
    uint                isUpdateInPlace,
    uint                nodePointer,
    uint                triangleId,
    uint                vertexOffset,
    in BoundingBox      boundingBox)
{
    // Handle two triangles sharing a bounding box when pair compression is enabled.
    uint childNodePointer = nodePointer;
    if ((triangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) && (GetNodeType(nodePointer) == NODE_TYPE_TRIANGLE_0))
    {
        const uint triNodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);

        // Check whether or not there is a triangle 1 in this node.
        if (((triangleId >> (NODE_TYPE_TRIANGLE_1 * TRIANGLE_ID_BIT_STRIDE)) & 0xf) != 0)
        {
            // Triangle 0 is not a child pointer in the parent box node, so correct the node pointer used for the
            // comparison when finding the child index in the parent below.
            childNodePointer |= NODE_TYPE_TRIANGLE_1;

            const uint otherPrimIndex = SourceBuffer.Load(triNodeOffset + TRIANGLE_NODE_PRIMITIVE_INDEX1_OFFSET);

            // Fetch face indices from index buffer.
            const uint3 faceIndices = FetchFaceIndices(IndexBuffer,
                                                       otherPrimIndex,
                                                       ShaderConstants.IndexBufferByteOffset,
                                                       ShaderConstants.IndexBufferFormat);

            // Check if vertex indices are within bounds.
            if ((faceIndices.x < ShaderConstants.vertexCount) &&
                (faceIndices.y < ShaderConstants.vertexCount) &&
                (faceIndices.z < ShaderConstants.vertexCount))
            {
                // Fetch triangle vertex data from vertex buffer.
                const TriangleData tri = FetchTransformedTriangleData(GeometryBuffer,
                                                                      faceIndices,
                                                                      ShaderConstants.GeometryBufferStrideInBytes,
                                                                      vertexOffset,
                                                                      ShaderConstants.HasValidTransform,
                                                                      TransformBuffer);

                const BoundingBox otherBox = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);

                // Merge the bounding boxes of the two triangles.
                boundingBox.min = min(boundingBox.min, otherBox.min);
                boundingBox.max = max(boundingBox.max, otherBox.max);
            }
        }
    }

    // Fetch parent node pointer
    const uint parentNodePointer = ReadParentPointer(SourceBuffer,
                                                     metadataSize,
                                                     nodePointer);

    // Update out of place destination buffer
    if (isUpdateInPlace == 0)
    {
        WriteParentPointer(ResultBuffer,
                           metadataSize,
                           nodePointer,
                           parentNodePointer);
    }

    const uint  nodeOffset = metadataSize + ExtractNodePointerOffset(parentNodePointer);
    const uint4 childPointers = SourceBuffer.Load<uint4>(nodeOffset);

    // Find child index in parent (assumes child pointer 0 is always valid)
    uint childIdx = 0;
    if (childNodePointer == childPointers.y)
    {
        childIdx = 1;
    }

    if (childNodePointer == childPointers.z)
    {
        childIdx = 2;
    }

    if (childNodePointer == childPointers.w)
    {
        childIdx = 3;
    }

    // If even a single child node is a box node, this is a node higher up the tree. Skip queueing parent node as another
    // leaf at the bottom of the tree will queue its parent which will handle our parent node.

    // B B B B --> 4 --> Not possible
    // B x B x --> 2 --> Not possible
    // B L B L --> 4 --> Skip queueing
    // L x B x --> 1 --> Skip queueing
    // L x x x --> 0 --> Queue parent node
    // L x L x --> 0 --> Queue parent node
    // L L L L --> 0 --> Queue parent node

    // Note, IsBoxNode() will return false for invalid nodes.
    uint boxNodeCount = 0;
    boxNodeCount += IsBoxNode(childPointers.x) ? 1 : 0;
    boxNodeCount += IsBoxNode(childPointers.y) ? 1 : 0;
    boxNodeCount += IsBoxNode(childPointers.z) ? 1 : 0;
    boxNodeCount += IsBoxNode(childPointers.w) ? 1 : 0;

    // Always perform update for out-of-place updates
    bool performUpdate = (isUpdateInPlace == false);

    uint3 boundingBox16;

    uint boxOffset;
    if (IsBoxNode32(parentNodePointer))
    {
        BoundingBox originalBox;

        boxOffset = childIdx * FLOAT32_BBOX_STRIDE;
        originalBox.min = ResultBuffer.Load<float3>(nodeOffset + FLOAT32_BOX_NODE_BB0_MIN_OFFSET + boxOffset);
        originalBox.max = ResultBuffer.Load<float3>(nodeOffset + FLOAT32_BOX_NODE_BB0_MAX_OFFSET + boxOffset);

        if (any(originalBox.min != boundingBox.min) ||
            any(originalBox.max != boundingBox.max))
        {
            performUpdate = true;
        }
    }
    else
    {
        boxOffset = childIdx * FLOAT16_BBOX_STRIDE;
        uint3 originalBox16 = ResultBuffer.Load<uint3>(nodeOffset + FLOAT16_BOX_NODE_BB0_OFFSET + boxOffset);

        boundingBox16 = CompressBBoxToUint3(boundingBox);

        if (any(originalBox16 != boundingBox16))
        {
            performUpdate = true;
        }
    }

    if (performUpdate)
    {
        if (IsBoxNode32(parentNodePointer))
        {
            ResultBuffer.Store<float3>(nodeOffset + FLOAT32_BOX_NODE_BB0_MIN_OFFSET + boxOffset, boundingBox.min);
            ResultBuffer.Store<float3>(nodeOffset + FLOAT32_BOX_NODE_BB0_MAX_OFFSET + boxOffset, boundingBox.max);
        }
        else
        {
            ResultBuffer.Store<float3>(nodeOffset + FLOAT16_BOX_NODE_BB0_OFFSET + boxOffset, boundingBox16);
        }

        if ((childIdx == 0) && (boxNodeCount == 0))
        {
            if (isUpdateInPlace == false)
            {
                ResultBuffer.Store<uint4>(nodeOffset, childPointers);

                if (IsBoxNode32(parentNodePointer))
                {
                    const uint sourceFlags = SourceBuffer.Load(nodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET);
                    ResultBuffer.Store(nodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET, sourceFlags);
                }
            }
        }
    }

    // If this is the first child in the parent node with all leaf children, queue parent pointer to
    // stack in scratch memory.
    // @note Right now we queue the parent node for update regardless of whether the leaf nodes' bounding boxes change
    // or not. We could optimize this by queuing the parent node only if any of the leaf nodes' bounding boxes change.
    if ((childIdx == 0) && (boxNodeCount == 0))
    {
        PushNodeToUpdateStack(ScratchBuffer, baseUpdateStackScratchOffset, parentNodePointer);
    }
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeTriangleNodes(
    in uint3 globalThreadId : SV_DispatchThreadID)
{
#if INDIRECT_BUILD
    // Sourced from Indirect Buffers
    const IndirectBuildOffset buildOffsetInfo = IndirectArgBuffer.Load<IndirectBuildOffset>(0);

    const uint numPrimitives      = buildOffsetInfo.primitiveCount;
    const uint primitiveOffset    = ComputePrimitiveOffset();
    const uint destLeafByteOffset = primitiveOffset * TRIANGLE_NODE_SIZE;
    const uint vertexOffset       = buildOffsetInfo.firstVertex;
#else
    const uint numPrimitives      = ShaderConstants.NumPrimitives;
    const uint primitiveOffset    = ShaderConstants.PrimitiveOffset;
    const uint destLeafByteOffset = ShaderConstants.DestLeafByteOffset;
    const uint vertexOffset       = 0;
#endif

    WriteGeometryInfo(globalThreadId.x, primitiveOffset, numPrimitives, DECODE_PRIMITIVE_STRIDE_TRIANGLE);

    uint primitiveIndex = globalThreadId.x;
    if (primitiveIndex < numPrimitives)
    {
        const uint metadataSize = IsUpdate(ShaderConstants.BuildFlags) ?
            SourceBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : ShaderConstants.metadataSizeInBytes;

        // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
        const AccelStructOffsets offsets = SourceBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

        const uint basePrimNodePtr =
            IsUpdate(ShaderConstants.BuildFlags) ? offsets.primNodePtrs : ShaderConstants.BasePrimNodePtrOffset;

        const uint flattenedPrimitiveIndex = primitiveOffset + primitiveIndex;
        const uint primNodePointerOffset =
            metadataSize + basePrimNodePtr + (flattenedPrimitiveIndex * sizeof(uint));

        // Fetch face indices from index buffer
        uint3 faceIndices = FetchFaceIndices(IndexBuffer,
                                             primitiveIndex,
                                             ShaderConstants.IndexBufferByteOffset,
                                             ShaderConstants.IndexBufferFormat);

        // Check if vertex indices are within bounds, otherwise make the triangle inactive
        if ((faceIndices.x < ShaderConstants.vertexCount) &&
            (faceIndices.y < ShaderConstants.vertexCount) &&
            (faceIndices.z < ShaderConstants.vertexCount))
        {
            // Fetch triangle vertex data from vertex buffer
            TriangleData tri = FetchTransformedTriangleData(GeometryBuffer,
                                                            faceIndices,
                                                            ShaderConstants.GeometryBufferStrideInBytes,
                                                            vertexOffset,
                                                            ShaderConstants.HasValidTransform,
                                                            TransformBuffer);

            uint nodePointer = INVALID_IDX;
            uint triangleId  = 0;

            if (IsUpdate(ShaderConstants.BuildFlags))
            {
                nodePointer = SourceBuffer.Load(primNodePointerOffset);

                // If the primitive was active during the initial build, it will have a valid primitive node pointer.
                if (nodePointer != INVALID_IDX)
                {
                    const uint nodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);
                    const uint nodeType   = GetNodeType(nodePointer);

                    uint3 vertexOffsets;

                    if (ShaderConstants.TriangleCompressionMode != NO_TRIANGLE_COMPRESSION)
                    {
                        triangleId    = SourceBuffer.Load(nodeOffset + TRIANGLE_NODE_ID_OFFSET);
                        vertexOffsets = CalcTriangleCompressionVertexOffsets(nodeType, triangleId);
                    }
                    else
                    {
                        triangleId    = CalcUncompressedTriangleId(ShaderConstants.GeometryFlags);
                        vertexOffsets = CalcTriangleVertexOffsets(nodeType);
                    }

                    ResultBuffer.Store3(nodeOffset + vertexOffsets.x, asuint(tri.v0));
                    ResultBuffer.Store3(nodeOffset + vertexOffsets.y, asuint(tri.v1));
                    ResultBuffer.Store3(nodeOffset + vertexOffsets.z, asuint(tri.v2));

                    if (ShaderConstants.isUpdateInPlace == false)
                    {
                        const uint geometryIndexAndFlags = PackGeometryIndexAndFlags(ShaderConstants.GeometryIndex,
                                                                                     ShaderConstants.GeometryFlags);
                        ResultBuffer.Store(nodeOffset + TRIANGLE_NODE_GEOMETRY_INDEX_AND_FLAGS_OFFSET,
                                           geometryIndexAndFlags);

                        const uint primIndexOffset = CalcPrimitiveIndexOffset(nodePointer);
                        ResultBuffer.Store(nodeOffset + TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET + primIndexOffset,
                                           primitiveIndex);

                        ResultBuffer.Store(nodeOffset + TRIANGLE_NODE_ID_OFFSET, triangleId);
                    }
                }
            }

            if (ShaderConstants.TriangleCompressionMode != PAIR_TRIANGLE_COMPRESSION)
            {
                const uint numLeafNodesOffset = metadataSize + ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET;
                UpdateLeafNodeCounters(numLeafNodesOffset, 1);
            }

            // Generate triangle bounds and update scene bounding box
            BoundingBox boundingBox = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);

            if (IsUpdate(ShaderConstants.BuildFlags))
            {
                if (ShaderConstants.isUpdateInPlace == false)
                {
                    ResultBuffer.Store(primNodePointerOffset, nodePointer);
                }

                // The shared bounding box for this pair of triangles will be updated by the thread handling triangle 0.
                const bool skipPairUpdatePush = (ShaderConstants.TriangleCompressionMode == PAIR_TRIANGLE_COMPRESSION) &&
                                                (GetNodeType(nodePointer) == NODE_TYPE_TRIANGLE_1);

                if ((nodePointer != INVALID_IDX) && (skipPairUpdatePush == false))
                {
                    PushNodeForUpdate(SourceBuffer,
                                      ResultBuffer,
                                      ScratchBuffer,
                                      metadataSize,
                                      ShaderConstants.BaseUpdateStackScratchOffset,
                                      ShaderConstants.TriangleCompressionMode,
                                      ShaderConstants.isUpdateInPlace,
                                      nodePointer,
                                      triangleId,
                                      vertexOffset,
                                      boundingBox);
                }
            }
            else
            {
                if (IsActive(tri))
                {
                    if (ShaderConstants.enableCentroidSceneBoundsWithSize && !ShaderConstants.enableTriangleSplitting)
                    {
                        UpdateCentroidSceneBoundsWithSize(ScratchBuffer, ShaderConstants.SceneBoundsByteOffset, boundingBox);
                    }
                    else
                    {
                        UpdateSceneBounds(ScratchBuffer, ShaderConstants.SceneBoundsByteOffset, boundingBox);
                    }
                }

                WriteScratchTriangleNode(ScratchBuffer,
                                         primitiveOffset,
                                         primitiveIndex,
                                         ShaderConstants.GeometryIndex,
                                         ShaderConstants.GeometryFlags,
                                         tri);

                // Store invalid prim node pointer for now during first time builds.
                // If the triangle is active, BuildQBVH will write it in.
                ResultBuffer.Store(primNodePointerOffset, INVALID_IDX);
            }
        }
        else
        {
            if (IsUpdate(ShaderConstants.BuildFlags) == false)
            {
                // Deactivate primitive by setting bbox_min_or_v0.x to NaN
                const uint scratchLeafNodeOffset =
                    ShaderConstants.LeafNodeDataByteOffset + (flattenedPrimitiveIndex * ByteStrideScratchNode);

                ScratchBuffer.Store(scratchLeafNodeOffset, NaN);

                ResultBuffer.Store(primNodePointerOffset, INVALID_IDX);
            }
            else if (ShaderConstants.isUpdateInPlace == false)
            {
                ResultBuffer.Store(primNodePointerOffset, INVALID_IDX);
            }
        }

        // ClearFlags for refit and update
        {
            const uint stride = ShaderConstants.LeafNodeExpansionFactor * sizeof(uint);
            const uint flagOffset = ShaderConstants.PropagationFlagsScratchOffset + (flattenedPrimitiveIndex * stride);
            for (uint i = 0; i < ShaderConstants.LeafNodeExpansionFactor; ++i)
            {
                ScratchBuffer.Store(flagOffset + (i * sizeof(uint)), 0);
            }
        }

        IncrementTaskCounter();
    }
}

//=====================================================================================================================
// Fetch API bounding box from source buffer
BoundingBox FetchBoundingBoxData(RWByteAddressBuffer buffer, uint index, uint boxStrideInBytes)
{
    BoundingBox bbox;
    bbox.min = asfloat(buffer.Load3(index * boxStrideInBytes));
    bbox.max = asfloat(buffer.Load3(index * boxStrideInBytes + 12));

    // Generate degenerate bounding box for zero area bounds
    if (ComputeBoxSurfaceArea(bbox) == 0)
    {
        bbox.min.x = +FLT_MAX;
        bbox.min.y = +FLT_MAX;
        bbox.min.z = +FLT_MAX;
        bbox.max.x = -FLT_MAX;
        bbox.max.y = -FLT_MAX;
        bbox.max.z = -FLT_MAX;
    }

    return bbox;
}

//=====================================================================================================================
void WriteScratchBoundingBoxNode(
    RWByteAddressBuffer buffer,
    uint                primitiveOffset,
    uint                primitiveIndex,
    uint                geometryIndex,
    uint                flags,
    in BoundingBox      bbox)
{
    uint offset = (primitiveIndex * ByteStrideScratchNode) +
                  (primitiveOffset * ByteStrideScratchNode)  +
                  ShaderConstants.LeafNodeDataByteOffset;
    uint4 data;

    // LeafNode.bbox_min_or_v0, primitiveIndex
    data = uint4(asuint(bbox.min), primitiveIndex);
    buffer.Store4(offset + SCRATCH_NODE_BBOX_MIN_OFFSET, data);

    // LeafNode.bbox_max_or_v1, geometryIndex
    data = uint4(asuint(bbox.max), geometryIndex);
    buffer.Store4(offset + SCRATCH_NODE_BBOX_MAX_OFFSET, data);

    // LeafNode.v2, parent
    data = uint4(0xffffffff, 0xffffffff, 0xffffffff, 0);
    buffer.Store4(offset + SCRATCH_NODE_V2_OFFSET, data);

    // type, flags, splitBox, numPrimitivesAndDoCollapse
    uint typeAndId = NODE_TYPE_USER_NODE_PROCEDURAL;
    data = uint4(typeAndId, flags, INVALID_IDX, 1 << 1);
    buffer.Store4(offset + SCRATCH_NODE_TYPE_OFFSET, data);
}

//=====================================================================================================================
void WriteProceduralNodeBoundingBox(
    uint        metadataSize,
    uint        nodePointer,
    BoundingBox bbox)
{
    const uint nodeOffset = metadataSize + ExtractNodePointerOffset(nodePointer);

    ResultBuffer.Store<float3>(nodeOffset + USER_NODE_PROCEDURAL_MIN_OFFSET,bbox.min);
    ResultBuffer.Store<float3>(nodeOffset + USER_NODE_PROCEDURAL_MAX_OFFSET,bbox.max);
}

//=====================================================================================================================
void WriteProceduralNodePrimitiveData(
    uint metadataSize,
    uint nodePointer,
    uint primitiveIndex)
{
    const uint nodeOffset            = metadataSize + ExtractNodePointerOffset(nodePointer);
    const uint geometryIndexAndFlags = PackGeometryIndexAndFlags(ShaderConstants.GeometryIndex,
                                                                 ShaderConstants.GeometryFlags);
    ResultBuffer.Store(nodeOffset + USER_NODE_PROCEDURAL_GEOMETRY_INDEX_AND_FLAGS_OFFSET, geometryIndexAndFlags);
    ResultBuffer.Store(nodeOffset + USER_NODE_PROCEDURAL_PRIMITIVE_INDEX_OFFSET, primitiveIndex);
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeAABBNodes(
    in uint3 globalThreadId : SV_DispatchThreadID)
{
#if INDIRECT_BUILD
    // Sourced from Indirect Buffers
    const IndirectBuildOffset buildOffsetInfo = IndirectArgBuffer.Load<IndirectBuildOffset>(0);

    const uint numPrimitives      = buildOffsetInfo.primitiveCount;
    const uint primitiveOffset    = ComputePrimitiveOffset();
    const uint destLeafByteOffset = primitiveOffset * USER_NODE_PROCEDURAL_SIZE;
#else
    const uint numPrimitives      = ShaderConstants.NumPrimitives;
    const uint primitiveOffset    = ShaderConstants.PrimitiveOffset;
    const uint destLeafByteOffset = ShaderConstants.DestLeafByteOffset;
#endif

    WriteGeometryInfo(globalThreadId.x, primitiveOffset, numPrimitives, DECODE_PRIMITIVE_STRIDE_AABB);

    uint primitiveIndex = globalThreadId.x;
    if (primitiveIndex < ShaderConstants.NumPrimitives)
    {
        const uint metadataSize =
            IsUpdate(ShaderConstants.BuildFlags) ? SourceBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : ShaderConstants.metadataSizeInBytes;

        // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
        const AccelStructOffsets offsets =
            SourceBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

        const uint basePrimNodePtr =
            IsUpdate(ShaderConstants.BuildFlags) ? offsets.primNodePtrs : ShaderConstants.BasePrimNodePtrOffset;

        // Get bounds for this thread
        const BoundingBox boundingBox = FetchBoundingBoxData(GeometryBuffer,
                                                             primitiveIndex,
                                                             ShaderConstants.GeometryBufferStrideInBytes);

        const uint primNodePointerOffset =
            metadataSize + basePrimNodePtr + ((primitiveOffset + primitiveIndex) * sizeof(uint));

        if (IsUpdate(ShaderConstants.BuildFlags))
        {
            const uint nodePointer = SourceBuffer.Load(primNodePointerOffset);

            // If the primitive was active during the initial build, it will have a valid primitive node pointer.
            if (nodePointer != INVALID_IDX)
            {
                WriteProceduralNodeBoundingBox(metadataSize, nodePointer, boundingBox);

                if (ShaderConstants.isUpdateInPlace == false)
                {
                    WriteProceduralNodePrimitiveData(metadataSize, nodePointer, primitiveIndex);

                    ResultBuffer.Store(primNodePointerOffset, nodePointer);
                }

                PushNodeForUpdate(SourceBuffer,
                                  ResultBuffer,
                                  ScratchBuffer,
                                  metadataSize,
                                  ShaderConstants.BaseUpdateStackScratchOffset,
                                  ShaderConstants.TriangleCompressionMode,
                                  ShaderConstants.isUpdateInPlace,
                                  nodePointer,
                                  0,
                                  0,
                                  boundingBox);
            }
            else if (ShaderConstants.isUpdateInPlace == false)
            {
                // For inactive primitives, just copy over the primitive node pointer.
                ResultBuffer.Store(primNodePointerOffset, nodePointer);
            }
        }
        else
        {
            if (ShaderConstants.enableCentroidSceneBoundsWithSize)
            {
                UpdateCentroidSceneBoundsWithSize(ScratchBuffer, ShaderConstants.SceneBoundsByteOffset, boundingBox);
            }
            else
            {
                // Update scene bounding box
                UpdateSceneBounds(ScratchBuffer, ShaderConstants.SceneBoundsByteOffset, boundingBox);
            }

            WriteScratchBoundingBoxNode(ScratchBuffer,
                                        primitiveOffset,
                                        primitiveIndex,
                                        ShaderConstants.GeometryIndex,
                                        ShaderConstants.GeometryFlags,
                                        boundingBox);

            // Store invalid prim node pointer for now during first time builds.
            // If the Procedural node is active, BuildQBVH will update it.
            ResultBuffer.Store(primNodePointerOffset, INVALID_IDX);

            const uint numLeafNodesOffset = metadataSize + ACCEL_STRUCT_HEADER_NUM_LEAF_NODES_OFFSET;
            UpdateLeafNodeCounters(numLeafNodesOffset, 1);
        }

        // ClearFlags for refit and update
        const uint flattenedPrimitiveIndex = primitiveOffset + primitiveIndex;

        {
            const uint flagOffset = ShaderConstants.PropagationFlagsScratchOffset + (flattenedPrimitiveIndex * sizeof(uint));
            ScratchBuffer.Store(flagOffset, 0);
        }

        IncrementTaskCounter();
    }
}
