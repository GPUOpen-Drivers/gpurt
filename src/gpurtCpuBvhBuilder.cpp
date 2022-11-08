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

#include "gpurtCpuBvhBuilder.h"
#include "gpurtCpuUtils.h"
#include "gpurt/gpurtLib.h"
#include "palMutex.h"

#include <float.h>
#include <math.h>
#include <algorithm>

namespace GpuRt
{

using uint16 = Pal::uint16;
using int32  = Pal::int32;

using namespace GpuRt::Shaders;

static const Aabb        NullBound      = { INFINITY, INFINITY, INFINITY, -INFINITY, -INFINITY, -INFINITY };
constexpr uint32         InvalidNode    = 0xffffffff;
constexpr uint32         InvalidIndex   = 0xffffffff;

// Hardware 32-bit box node format and offsets
static constexpr uint32 RayTracingFloat32BoxNodeChild0Offset = 0;
static constexpr uint32 RayTracingFloat32BoxNodeChild1Offset = 4;
static constexpr uint32 RayTracingFloat32BoxNodeChild2Offset = 8;
static constexpr uint32 RayTracingFloat32BoxNodeChild3Offset = 12;
static constexpr uint32 RayTracingFloat32BoxNodeBB0MinOffset = 16;
static constexpr uint32 RayTracingFloat32BoxNodeBB0MaxOffset = 28;
static constexpr uint32 RayTracingFloat32BoxNodeBB1MinOffset = 40;
static constexpr uint32 RayTracingFloat32BoxNodeBB1MaxOffset = 52;
static constexpr uint32 RayTracingFloat32BoxNodeBB2MinOffset = 64;
static constexpr uint32 RayTracingFloat32BoxNodeBB2MaxOffset = 76;
static constexpr uint32 RayTracingFloat32BoxNodeBB3MinOffset = 88;
static constexpr uint32 RayTracingFloat32BoxNodeBB3MaxOffset = 100;
static constexpr uint32 RayTracingFloat32BoxNodeUnused0Offset = 112;
static constexpr uint32 RayTracingFloat32BoxNodeUnused1Offset = 116;
static constexpr uint32 RayTracingFloat32BoxNodeUnused2Offset = 120;
static constexpr uint32 RayTracingFloat32BoxNodeUnused3Offset = 124;
static constexpr uint32 RayTracingFloat32BoxNodeSize          = 128;

//=====================================================================================================================
// Converts a 16-bit float to a 32-bit float
static float ConvertFloat16ToFloat32(
    uint16 value)
{
    uint32 sign = (value >> 15u) & 0x1;
    uint32 exponent = (value >> 10u) & 0x1f;
    uint32 mantissa = (value & 0x3ff);
    uint32 result;

    if ((exponent == 0) && (mantissa == 0))
    {
        // Signed zero
        result = (sign << 31u);
    }
    else if (exponent == 31)
    {
        // Signed infinity and NaN
        result = (sign << 31) | 0x7f800000u | (mantissa << 13);
    }
    else
    {
        if (exponent == 0)
        {
            // Denormalized
            while ((mantissa & 0x00000400u) == 0)
            {
                mantissa <<= 1;
                exponent--;
            }

            exponent++;
            mantissa &= ~0x00000400u;
        }

        exponent += (127 - 15);
        mantissa <<= 13;

        result = (sign << 31) | (exponent << 23u) | mantissa;
    }

    return *reinterpret_cast<const float*>(&result);
}

// =====================================================================================================================
// Generates a bounding box from three points
static Aabb GenerateBoundingBox(
    float3 v0,
    float3 v1,
    float3 v2)
{
    return Aabb {
        Util::Min(Util::Min(v0.x, v1.x), v2.x),
        Util::Min(Util::Min(v0.y, v1.y), v2.y),
        Util::Min(Util::Min(v0.z, v1.z), v2.z),
        Util::Max(Util::Max(v0.x, v1.x), v2.x),
        Util::Max(Util::Max(v0.y, v1.y), v2.y),
        Util::Max(Util::Max(v0.z, v1.z), v2.z)
    };
}

// =====================================================================================================================
// Generates a bounding box that contains two other bounding boxes
static Aabb BoundingBoxUnion(
    const Aabb& a,
    const Aabb& b)
{
    return Aabb {
        Util::Min(a.minX, b.minX),
        Util::Min(a.minY, b.minY),
        Util::Min(a.minZ, b.minZ),
        Util::Max(a.maxX, b.maxX),
        Util::Max(a.maxY, b.maxY),
        Util::Max(a.maxZ, b.maxZ)
    };
}

// =====================================================================================================================
// Generates a bounding box that contains another bounding box and a point
static Aabb BoundingBoxUnion(
    const Aabb&  a,
    const float3 b)
{
    return Aabb {
        Util::Min(a.minX, b.x),
        Util::Min(a.minY, b.y),
        Util::Min(a.minZ, b.z),
        Util::Max(a.maxX, b.x),
        Util::Max(a.maxY, b.y),
        Util::Max(a.maxZ, b.z)
    };
}

// =====================================================================================================================
// Surface area of a bounding box
static float BoundingBoxArea(
    const Aabb& aabb)
{
    float dx = aabb.maxX - aabb.minX;
    float dy = aabb.maxY - aabb.minY;
    float dz = aabb.maxZ - aabb.minZ;

    return 2.0f * (dx * dy + dy * dz + dx * dz);
}

// =====================================================================================================================
// Given 3x4 matrix M and 3x1 vector representing a point, returns M * [point.x, point.y, point.z, 1]
static float3 MatrixTransformPoint(
    const float4 M[3],
    float3       point)
{
    return float3(
        M[0].x * point.x + M[0].y * point.y + M[0].z * point.z + M[0].w,
        M[1].x * point.x + M[1].y * point.y + M[1].z * point.z + M[1].w,
        M[2].x * point.x + M[2].y * point.y + M[2].z * point.z + M[2].w
    );
}

// =====================================================================================================================
// Returns a bounding box that contains another bounding box whose corners have been transformed by a 3x4 matrix
static Aabb MatrixTransformBoundingBox(
    const float4 m[3],
    float3       bmin,
    float3       bmax)
{
    const float3 corner = MatrixTransformPoint(m, bmin);

    Aabb dst = { corner.x, corner.y, corner.z,
                 corner.x, corner.y, corner.z }; // 000

    dst = BoundingBoxUnion(dst, MatrixTransformPoint(m, float3(bmax.x, bmin.y, bmin.z))); // 100
    dst = BoundingBoxUnion(dst, MatrixTransformPoint(m, float3(bmin.x, bmax.y, bmin.z))); // 010
    dst = BoundingBoxUnion(dst, MatrixTransformPoint(m, float3(bmin.x, bmin.y, bmax.z))); // 001
    dst = BoundingBoxUnion(dst, MatrixTransformPoint(m, float3(bmin.x, bmax.y, bmax.z))); // 011
    dst = BoundingBoxUnion(dst, MatrixTransformPoint(m, float3(bmax.x, bmax.y, bmin.z))); // 110
    dst = BoundingBoxUnion(dst, MatrixTransformPoint(m, float3(bmax.x, bmin.y, bmax.z))); // 101
    dst = BoundingBoxUnion(dst, MatrixTransformPoint(m, float3(bmax.x, bmax.y, bmax.z))); // 111

    return dst;
}

// =====================================================================================================================
// 3x4 matrix determinant
static float MatrixDeterminant(
    const float transform[3][4])
{
    return transform[0][0] * transform[1][1] * transform[2][2] -
           transform[0][0] * transform[2][1] * transform[1][2] -
           transform[1][0] * transform[0][1] * transform[2][2] +
           transform[1][0] * transform[2][1] * transform[0][2] +
           transform[2][0] * transform[0][1] * transform[1][2] -
           transform[2][0] * transform[1][1] * transform[0][2];
}

// =====================================================================================================================
// 3x4 matrix inverse
static void MatrixInverse(
    float       invertedTransform[3][4],
    const float transform[3][4])
{
    const float invDet = 1.0f / (MatrixDeterminant(transform));

    invertedTransform[0][0] = invDet * (transform[1][1] * (transform[2][2] * 1.0f - 0.0f * transform[2][3]) + transform[2][1] * (0.0f * transform[1][3] - transform[1][2] * 1.0f) + 0.0f * (transform[1][2] * transform[2][3] - transform[2][2] * transform[1][3]));
    invertedTransform[1][0] = invDet * (transform[1][2] * (transform[2][0] * 1.0f - 0.0f * transform[2][3]) + transform[2][2] * (0.0f * transform[1][3] - transform[1][0] * 1.0f) + 0.0f * (transform[1][0] * transform[2][3] - transform[2][0] * transform[1][3]));
    invertedTransform[2][0] = invDet * (transform[1][3] * (transform[2][0] * 0.0f - 0.0f * transform[2][1]) + transform[2][3] * (0.0f * transform[1][1] - transform[1][0] * 0.0f) + 1.0f * (transform[1][0] * transform[2][1] - transform[2][0] * transform[1][1]));
    invertedTransform[0][1] = invDet * (transform[2][1] * (transform[0][2] * 1.0f - 0.0f * transform[0][3]) + 0.0f * (transform[2][2] * transform[0][3] - transform[0][2] * transform[2][3]) + transform[0][1] * (0.0f * transform[2][3] - transform[2][2] * 1.0f));
    invertedTransform[1][1] = invDet * (transform[2][2] * (transform[0][0] * 1.0f - 0.0f * transform[0][3]) + 0.0f * (transform[2][0] * transform[0][3] - transform[0][0] * transform[2][3]) + transform[0][2] * (0.0f * transform[2][3] - transform[2][0] * 1.0f));
    invertedTransform[2][1] = invDet * (transform[2][3] * (transform[0][0] * 0.0f - 0.0f * transform[0][1]) + 1.0f * (transform[2][0] * transform[0][1] - transform[0][0] * transform[2][1]) + transform[0][3] * (0.0f * transform[2][1] - transform[2][0] * 0.0f));
    invertedTransform[0][2] = invDet * (0.0f * (transform[0][2] * transform[1][3] - transform[1][2] * transform[0][3]) + transform[0][1] * (transform[1][2] * 1.0f - 0.0f * transform[1][3]) + transform[1][1] * (0.0f * transform[0][3] - transform[0][2] * 1.0f));
    invertedTransform[1][2] = invDet * (0.0f * (transform[0][0] * transform[1][3] - transform[1][0] * transform[0][3]) + transform[0][2] * (transform[1][0] * 1.0f - 0.0f * transform[1][3]) + transform[1][2] * (0.0f * transform[0][3] - transform[0][0] * 1.0f));
    invertedTransform[2][2] = invDet * (1.0f * (transform[0][0] * transform[1][1] - transform[1][0] * transform[0][1]) + transform[0][3] * (transform[1][0] * 0.0f - 0.0f * transform[1][1]) + transform[1][3] * (0.0f * transform[0][1] - transform[0][0] * 0.0f));
    invertedTransform[0][3] = invDet * (transform[0][1] * (transform[2][2] * transform[1][3] - transform[1][2] * transform[2][3]) + transform[1][1] * (transform[0][2] * transform[2][3] - transform[2][2] * transform[0][3]) + transform[2][1] * (transform[1][2] * transform[0][3] - transform[0][2] * transform[1][3]));
    invertedTransform[1][3] = invDet * (transform[0][2] * (transform[2][0] * transform[1][3] - transform[1][0] * transform[2][3]) + transform[1][2] * (transform[0][0] * transform[2][3] - transform[2][0] * transform[0][3]) + transform[2][2] * (transform[1][0] * transform[0][3] - transform[0][0] * transform[1][3]));
    invertedTransform[2][3] = invDet * (transform[0][3] * (transform[2][0] * transform[1][1] - transform[1][0] * transform[2][1]) + transform[1][3] * (transform[0][0] * transform[2][1] - transform[2][0] * transform[0][1]) + transform[2][3] * (transform[1][0] * transform[0][1] - transform[0][0] * transform[1][1]));
}

// =====================================================================================================================
// Calculates bounding box centroid
static float3 BoundingBoxCentroid(
    const Aabb& aabb)
{
    return float3((aabb.minX + aabb.maxX) * 0.5f,
                  (aabb.minY + aabb.maxY) * 0.5f,
                  (aabb.minZ + aabb.maxZ) * 0.5f);
}

// =====================================================================================================================
// Returns a pointer to the f32 box node at the given pointer location.
static const Float32BoxNode* FetchFloat32BoxNode(
    const AccelStructHeader* pAccel,
    NodePointer              nodePointer)
{
    const uint32 byteOffset = ExtractNodePointerOffset(nodePointer);
    const void* pNode       = Util::VoidPtrInc(pAccel, byteOffset);

    return static_cast<const Float32BoxNode*>(pNode);
}

// =====================================================================================================================
// Returns true if a given triangle denotes an active primitive
bool CpuBvhBuilder::IsActive(
    TriangleData tri)
{
    return ((isnan(tri.v[0].x) == false) && (isnan(tri.v[1].x) == false) && (isnan(tri.v[2].x) == false));
}

// =====================================================================================================================
// Returns true if a given AABB denotes an active primitive
bool CpuBvhBuilder::IsActive(
    Aabb aabb)
{
    return (isnan(aabb.minX) == false);
}

// =====================================================================================================================
// Get face indices from 16-bit index buffer
static void GetFaceIndices16(
    const void* pIndexBuffer,
    uint32      faceIndex,
    uint32      indices[3])
{
    constexpr uint32 Stride = 6; // 3 vertices per triangle with 2-byte indices

    uint32 address = (faceIndex * Stride);

    const uint16* pData = static_cast<const uint16*>(Util::VoidPtrInc(pIndexBuffer, address));

    indices[0] = pData[0];
    indices[1] = pData[1];
    indices[2] = pData[2];
}

// =====================================================================================================================
// Get face indices from 32-bit index buffer
static void GetFaceIndices32(
    const void* pIndexBuffer,
    uint32      faceIndex,
    uint32      indices[3])
{
    constexpr uint32 Stride = 12; // 3 vertices per triangle with 2-byte indices

    uint32 address = (faceIndex * Stride);

    const uint32* pData = static_cast<const uint32*>(Util::VoidPtrInc(pIndexBuffer, address));

    indices[0] = pData[0];
    indices[1] = pData[1];
    indices[2] = pData[2];
}

//=====================================================================================================================
// Get internal BVH node size in bytes
static uint32 GetBvhNodeSizeLeaf(
    GeometryType primitiveType)
{
    uint32 sizeInBytes = 0;
    switch (primitiveType)
    {
    case GeometryType::Triangles:
        sizeInBytes = RayTracingTriangleNodeSize;
        break;
    case GeometryType::Aabbs:
        sizeInBytes = RayTracingUserNodeProceduralSize;
        break;
    default:
        PAL_NEVER_CALLED();
        break;
    }

    return sizeInBytes;
}

// =====================================================================================================================
// Fetch face indices from index buffer
static void FetchFaceIndices(
    const GeometryTriangles* pDesc,
    uint32                   index,
    uint32                   indices[3])
{
    const IndexFormat indexBufferFormat = pDesc->indexFormat;
    const void* pIndexBuffer            = pDesc->indexBufferAddr.pCpu;

    uint3 faceIndices;

    if (indexBufferFormat == IndexFormat::R16_Uint)
    {
        GetFaceIndices16(pIndexBuffer, index, indices);
    }
    else if (indexBufferFormat == IndexFormat::R32_Uint)
    {
        GetFaceIndices32(pIndexBuffer, index, indices);
    }
    else
    {
        PAL_ASSERT(indexBufferFormat == IndexFormat::Unknown);

        const uint32 startIndex = (index * 3);

        indices[0] = startIndex;
        indices[1] = startIndex + 1;
        indices[2] = startIndex + 2;
    }
}

// =====================================================================================================================
static float3 LoadVertexR32G32B32Float(
    const void* pBuffer,
    uint32      byteOffset)
{
    const float* pData = static_cast<const float*>(Util::VoidPtrInc(pBuffer, byteOffset));

    return float3(pData[0], pData[1], pData[2]);
}

// =====================================================================================================================
static float3 LoadVertexR32G32Float(
    const void* pBuffer,
    uint32      byteOffset)
{
    const float* pData = static_cast<const float*>(Util::VoidPtrInc(pBuffer, byteOffset));

    return float3(pData[0], pData[1], 0.0f);
}

// =====================================================================================================================
static float3 LoadVertexR16G16B16A16Float(
    const void* pBuffer,
    uint32      byteOffset)
{
    const uint16* pData = static_cast<const uint16*>(Util::VoidPtrInc(pBuffer, byteOffset));

    return float3(ConvertFloat16ToFloat32(pData[0]),
                  ConvertFloat16ToFloat32(pData[1]),
                  ConvertFloat16ToFloat32(pData[2]));
}

// =====================================================================================================================
static float3 LoadVertexR16G16Float(
    const void* pBuffer,
    uint32      byteOffset)
{
    const uint16* pData = static_cast<const uint16*>(Util::VoidPtrInc(pBuffer, byteOffset));

    return float3(ConvertFloat16ToFloat32(pData[0]),
                  ConvertFloat16ToFloat32(pData[1]),
                  0.0f);
}

//=====================================================================================================================
// Mask Out Primitive Count for Collapse
static uint32 ExtractNodePointerCollapse(
    uint32 nodePointer)
{
    return nodePointer & ((1 << 29) - 1); // mask out the triangle count
}

//=====================================================================================================================
// Get Num Primitives For Collapse
static uint32 ExtractPrimitiveCount(
    uint32 nodePointer)
{
    return nodePointer >> 29;
}

// =====================================================================================================================
static int32 SignExtend16to32(
    uint32 input)
{
    uint32 output = input;
    uint32 needsExt = input & 0x8000;

    if (needsExt != 0)
    {
        output = input | 0xFFFF0000;
    }

    return asint(output);
}

// =====================================================================================================================
static float ConvertSnorm16ToFloat(
    uint32 input)
{
    // 16-bit SNORM maps [-INT16_MAX, INT16_MAX] to [-1.0f to 1.0f]; INT16_MIN also maps to -1.0f.
    constexpr uint32 MaxSnorm16 = 0x7FFF;
    const int inputVal = SignExtend16to32(input);
    return Util::Max((inputVal / static_cast<float>(MaxSnorm16)), -1.0f);
}

// =====================================================================================================================
static float3 LoadVertexR16G16B16A16Snorm(
    const void* pBuffer,
    uint32      byteOffset)
{
    const uint16* pData = static_cast<const uint16*>(Util::VoidPtrInc(pBuffer, byteOffset));

    return float3(ConvertSnorm16ToFloat(pData[0]),
                  ConvertSnorm16ToFloat(pData[1]),
                  ConvertSnorm16ToFloat(pData[2]));
}

// =====================================================================================================================
static float3 LoadVertexR16G16Snorm(
    const void* pBuffer,
    uint32      byteOffset)
{
    const uint16* pData = static_cast<const uint16*>(Util::VoidPtrInc(pBuffer, byteOffset));

    return float3(ConvertSnorm16ToFloat(pData[0]),
                  ConvertSnorm16ToFloat(pData[1]),
                  0.0f);
}

// =====================================================================================================================
static TriangleData FetchTriangleData(
    VertexFormat vertexBufferFormat,
    const void*  pVertexBuffer,
    const uint32 index[3],
    uint32       vertexByteStride)
{
    TriangleData tri;

    switch (vertexBufferFormat)
    {
    case VertexFormat::R32G32B32_Float:
        tri.v[0] = LoadVertexR32G32B32Float(pVertexBuffer, (index[0] * vertexByteStride));
        tri.v[1] = LoadVertexR32G32B32Float(pVertexBuffer, (index[1] * vertexByteStride));
        tri.v[2] = LoadVertexR32G32B32Float(pVertexBuffer, (index[2] * vertexByteStride));
        break;
    case VertexFormat::R32G32_Float:
        tri.v[0] = LoadVertexR32G32Float(pVertexBuffer, (index[0] * vertexByteStride));
        tri.v[1] = LoadVertexR32G32Float(pVertexBuffer, (index[1] * vertexByteStride));
        tri.v[2] = LoadVertexR32G32Float(pVertexBuffer, (index[2] * vertexByteStride));
        break;
    case VertexFormat::R16G16B16A16_Float:
        tri.v[0] = LoadVertexR16G16B16A16Float(pVertexBuffer, (index[0] * vertexByteStride));
        tri.v[1] = LoadVertexR16G16B16A16Float(pVertexBuffer, (index[1] * vertexByteStride));
        tri.v[2] = LoadVertexR16G16B16A16Float(pVertexBuffer, (index[2] * vertexByteStride));
        break;
    case VertexFormat::R16G16_Float:
        tri.v[0] = LoadVertexR16G16Float(pVertexBuffer, (index[0] * vertexByteStride));
        tri.v[1] = LoadVertexR16G16Float(pVertexBuffer, (index[1] * vertexByteStride));
        tri.v[2] = LoadVertexR16G16Float(pVertexBuffer, (index[2] * vertexByteStride));
        break;
    case VertexFormat::R16G16B16A16_Snorm:
        tri.v[0] = LoadVertexR16G16B16A16Snorm(pVertexBuffer, (index[0] * vertexByteStride));
        tri.v[1] = LoadVertexR16G16B16A16Snorm(pVertexBuffer, (index[1] * vertexByteStride));
        tri.v[2] = LoadVertexR16G16B16A16Snorm(pVertexBuffer, (index[2] * vertexByteStride));
        break;
    case VertexFormat::R16G16_Snorm:
        tri.v[0] = LoadVertexR16G16Snorm(pVertexBuffer, (index[0] * vertexByteStride));
        tri.v[1] = LoadVertexR16G16Snorm(pVertexBuffer, (index[1] * vertexByteStride));
        tri.v[2] = LoadVertexR16G16Snorm(pVertexBuffer, (index[2] * vertexByteStride));
        break;
    default:
        PAL_NEVER_CALLED();
        break;
    }

    return tri;
}

// =====================================================================================================================
static TriangleData FetchTransformedTriangleData(
    const GeometryTriangles* pDesc,
    uint32                   faceIndices[3])
{
    const VertexFormat vertexFormat          = pDesc->vertexFormat;
    const void* pGeometryBuffer              = pDesc->vertexBufferAddr.pCpu;
    const uint32 geometryBufferStrideInBytes = static_cast<uint32>(pDesc->vertexBufferByteStride);
    const float4* pTransformBuffer           = static_cast<const float4*>(pDesc->columnMajorTransform3x4.pCpu);

    // Fetch triangle vertex data from vertex buffer
    TriangleData tri = FetchTriangleData(vertexFormat, pGeometryBuffer, faceIndices, geometryBufferStrideInBytes);

    // If this geometry has a valid transformation matrix. Transform each vertex using this matrix.
    if (pTransformBuffer != nullptr)
    {
        float4 m[3] = {
            pTransformBuffer[0],
            pTransformBuffer[1],
            pTransformBuffer[2],
        };

        tri.v[0] = MatrixTransformPoint(m, tri.v[0]);
        tri.v[1] = MatrixTransformPoint(m, tri.v[1]);
        tri.v[2] = MatrixTransformPoint(m, tri.v[2]);
    }

    return tri;
}

// =====================================================================================================================
CpuBvhBuilder::CpuBvhBuilder(
    Internal::Device*      const pDevice,
    const Pal::DeviceProperties& deviceProps,
    ClientCallbacks              clientCb,
    const DeviceSettings&        deviceSettings)
    :
    BvhBuilder(pDevice, deviceProps, clientCb, deviceSettings)
{
}

// =====================================================================================================================
// Init CpuBuildConfig
void CpuBvhBuilder::InitBuildConfig(
    const AccelStructBuildInfo& buildArgs)
{
    BvhBuilder::InitBuildConfig(buildArgs);

    m_cpuConfig.leafNodeSize = GetLeafNodeSize(m_deviceSettings, m_buildConfig);
    // TODO #gpurt: Handling mixed types of interior box nodes needs trickier computation for size of the result buffer
    //   root node is always fp32 (regardless of fp16 mode)
    //   fp16mode == leaves: all leaf nodes are fp16, others are fp32
    m_cpuConfig.internalNodeSize = GpuRt::RayTracingQBVH32NodeSize;

    PAL_ASSERT((m_cpuConfig.internalNodeSize == RayTracingQBVH16NodeSize) ||
        (m_cpuConfig.internalNodeSize == RayTracingQBVH32NodeSize));

    m_cpuConfig.internalNodeType = (m_cpuConfig.internalNodeSize == RayTracingQBVH16NodeSize) ?
                                     NODE_TYPE_BOX_FLOAT16 :
                                     NODE_TYPE_BOX_FLOAT32;

    // Initialize starting values
    m_cpuConfig.buildNodeCount = 0;
    // Determine build algorithm
    if (m_buildConfig.cpuBuildMode == BvhCpuBuildMode::RecursiveSAH)
    {
        DetermineSAHParams();
    }
}

// =====================================================================================================================
// Rebuild all leaf nodes from given inputs.  This happens for both builds and updates.
//
// Returns the total number of primitives
uint32 CpuBvhBuilder::EncodeLeafPrimitives()
{
    uint32 totalPrimitiveCount = 0;

    m_dstHeader.numLeafNodes   = 0;
    m_dstHeader.numPrimitives  = 0;
    m_dstHeader.numActivePrims = 0;
    m_dstHeader.numDescs       = 0;

    if (m_buildConfig.topLevelBuild == false)
    {
        // Prepare merged source AABB buffer data from geometry
        for (uint32 geometryIndex = 0; geometryIndex < m_buildArgs.inputs.inputElemCount; ++geometryIndex)
        {
            uint32 primitiveCount = 0;
            const uint32 primitiveOffset = totalPrimitiveCount;

            const Geometry geometry =
                m_clientCb.pfnConvertAccelStructBuildGeometry(m_buildArgs.inputs, geometryIndex);

            // Mixing geometry types within a bottom-level acceleration structure is not allowed.
            PAL_ASSERT(geometry.type == m_buildConfig.geometryType);

            if (geometry.type == GeometryType::Triangles)
            {
                bool isIndexed = false;

                if (geometry.triangles.indexCount > 0)
                {
                    PAL_ASSERT((geometry.triangles.indexCount % 3) == 0);

                    primitiveCount += (geometry.triangles.indexCount / 3);
                    isIndexed = true;
                }
                else
                {
                    PAL_ASSERT((geometry.triangles.vertexCount % 3) == 0);

                    primitiveCount += (geometry.triangles.vertexCount / 3);
                }

                if ((isIndexed == false) || (geometry.triangles.indexBufferAddr.pCpu != 0))
                {
                    EncodeTriangleNodes(primitiveOffset,
                                        &geometry.triangles,
                                        primitiveCount,
                                        geometryIndex,
                                        geometry.flags);
                }
                else
                {
                    // Application specified a NULL index buffer with non-zero IndexCount and format.
                    // This may be unintentional, but will just waste memory in the destination acceleration
                    // structure.
                    PAL_ALERT_ALWAYS();
                }
            }
            else if (geometry.type == GeometryType::Aabbs)
            {
                primitiveCount = static_cast<uint32>(geometry.aabbs.aabbCount);

                EncodeAABBNodes(primitiveOffset,
                                &geometry.aabbs,
                                primitiveCount,
                                geometryIndex,
                                geometry.flags);
            }
            else
            {
                // Invalid geometry type
                PAL_ASSERT_ALWAYS();
            }

            totalPrimitiveCount += primitiveCount;
        }
    }
    else
    {
        // The primitives for the top level structure are just instances
        totalPrimitiveCount = m_buildArgs.inputs.inputElemCount;

        EncodeInstances(m_buildArgs.inputs.instances.pCpu,
                        m_buildArgs.inputs.inputElemCount,
                        m_buildArgs.inputs.inputElemLayout);
    }
    PAL_ASSERT(totalPrimitiveCount == m_buildConfig.numPrimitives);

    return totalPrimitiveCount;
}

// =====================================================================================================================
// Main entry point for building/updating acceleration structures on the CPU
void CpuBvhBuilder::BuildRaytracingAccelerationStructure(
    const AccelStructBuildInfo& buildArgs)
{
    m_buildArgs = buildArgs;
    InitBuildConfig(m_buildArgs);

    const uint32 resultDataSize = CalculateResultBufferInfo(&m_resultOffsets, &m_metadataSizeInBytes);

    // Initialize header
    if (IsUpdate() == false)
    {
        m_dstHeader                             = {};
        m_dstHeader.info.type                   = static_cast<uint32>(m_buildArgs.inputs.type);
        m_dstHeader.info.buildType              = static_cast<uint32>(AccelStructBuilderType::Cpu);
        m_dstHeader.info.mode                   = static_cast<uint32>(m_buildConfig.cpuBuildMode);
        m_dstHeader.info.triCompression         = static_cast<uint32>(m_buildConfig.triangleCompressionMode);
        m_dstHeader.info.fp16BoxNodesInBlasMode = static_cast<uint32>(m_buildConfig.fp16BoxNodesInBlasMode);
        m_dstHeader.info.flags                  = m_buildArgs.inputs.flags;
        m_dstHeader.offsets                     = m_resultOffsets;
    }
    else
    {
        m_dstHeader = *SourceHeader();
    }

    uint32 totalPrimitiveCount = 0;

    if (m_buildConfig.numLeafNodes > 0)
    {
        // Compute the offsets into the scratch buffer for all of our scratch resources.
        CalculateScratchBufferInfo(IsUpdate(),
                                   m_buildConfig.numLeafNodes,
                                   m_buildConfig.triangleCompressionMode,
                                   &m_scratchOffsets);

        // Set up pointers to scratch buffer sections
        m_pPrimitives = ScratchBuffer<ScratchPrimitive>(m_scratchOffsets.scratchPrimsOffset);

        if (IsUpdate() == false)
        {
            m_pBuildNodes             = ScratchBuffer<ScratchBuildNode>(m_scratchOffsets.scratchBuildNodesOffset);
            m_pUpdateNodes            = nullptr;
            m_pUpdateParentBuffers[0] = nullptr;
            m_pUpdateParentBuffers[1] = nullptr;
            m_pLeafsToPrims           = nullptr;
            m_pSortedPrims            = ScratchBuffer<uint32>(m_scratchOffsets.sortedPrimIndicesOffset);
        }
        else
        {
            m_pBuildNodes             = nullptr;
            m_pUpdateNodes            = ScratchBuffer<ScratchUpdateNode>(m_scratchOffsets.scratchUpdateNodesOffset);
            m_pUpdateParentBuffers[0] = ScratchBuffer<uint32>(m_scratchOffsets.updateParentsOffsets[0]);
            m_pUpdateParentBuffers[1] = ScratchBuffer<uint32>(m_scratchOffsets.updateParentsOffsets[1]);
            m_pSortedPrims            = nullptr;

            if (m_scratchOffsets.updateLeafsToPrimMappingOffset != 0)
            {
                m_pLeafsToPrims = ScratchBuffer<uint32>(m_scratchOffsets.updateLeafsToPrimMappingOffset);
            }
        }

        totalPrimitiveCount = EncodeLeafPrimitives();

        if (totalPrimitiveCount != 0)
        {
            // Build or update the acceleration structure
            if (IsUpdate() == false)
            {
                // Build a new BVH
                BuildAccelerationStructure(totalPrimitiveCount, m_buildArgs.inputs.type, resultDataSize);
            }
            else
            {
                // Update an existing BVH
                UpdateAccelerationStructure(totalPrimitiveCount, m_buildArgs.inputs.type, resultDataSize);
            }
        }

        // Handle the post build info feature
        if (m_buildArgs.postBuildInfoDescCount > 0)
        {
            PAL_NOT_IMPLEMENTED();
        }
    }

    // Write metadata header
    if ((IsUpdate() == false) || (IsInPlace() == false))
    {
        AccelStructMetadataHeader* pResultMetadata = ResultMetadataHeader();

        pResultMetadata->addressLo   = Util::LowPart(ResultBufferBaseVa());
        pResultMetadata->addressHi   = Util::HighPart(ResultBufferBaseVa());
        pResultMetadata->sizeInBytes = m_metadataSizeInBytes;
    }

    // Write result header
    *ResultHeader() = m_dstHeader;

    if (m_deviceSettings.enableBuildAccelStructStats)
    {
        // Acceleration stats are not currently supported by the CPU builder
        PAL_NOT_IMPLEMENTED();
    }

    // Dump Acceleration Structure
    if (m_deviceSettings.enableBuildAccelStructDumping)
    {
        // Initialise acceleration structure information for dump operations
        AccelStructInfo info = {};

        info.type                    = m_buildArgs.inputs.type;
        info.numDesc                 = m_buildArgs.inputs.inputElemCount;
        info.numPrimitives           = totalPrimitiveCount;
        info.buildFlags              = m_buildArgs.inputs.flags;
        info.buildType               = AccelStructBuilderType::Cpu;
        info.buildMode               = static_cast<uint32>(m_buildConfig.cpuBuildMode);
        info.triangleCompressionMode = m_buildConfig.triangleCompressionMode;
        info.fp16BoxNodesInBlasMode  = m_buildConfig.fp16BoxNodesInBlasMode;
        info.gpuVa                   = HeaderBufferBaseVa();
        info.sizeInBytes             = resultDataSize;

        m_clientCb.pfnAccelStructBuildDumpEvent(nullptr, info, m_buildArgs, nullptr);
    }
}

// =====================================================================================================================
// Returns the axis index of the largest AABB extent
static uint32 LargestExtent(
    const Aabb& aabb)
{
    const float dx = aabb.maxX - aabb.minX;
    const float dy = aabb.maxY - aabb.minY;
    const float dz = aabb.maxZ - aabb.minZ;

    uint32 dim;

    if (dx > dy && dx > dz)
    {
        dim = 0;
    }
    else if (dy > dz)
    {
        dim = 1;
    }
    else
    {
        dim = 2;
    }

    return dim;
}

// =====================================================================================================================
// Given an array of data and a [start, end) range, this function will rearrange the data into two partitions
// [start,mid) and [mid,end) for the returned midpoint, such that pred(pData[i], pParams) = true for all i within
// the first group and false for all i within the second group.
template<typename T, typename Params, typename Predicate>
static uint32 Partition(
    T*            pData,
    uint32        start,
    uint32        end,
    Predicate     pred,
    const Params* pParams)
{
    uint32 mid = start;

    while (mid < end)
    {
        if (pred(pData[mid], pParams) == false)
        {
            break;
        }

        mid++;
    }

    for (uint32 i = mid + 1; i < end; i++)
    {
        if (pred(pData[i], pParams))
        {
            Util::Swap(pData[mid], pData[i]);
            mid++;
        }
    }

    return mid;
}

// =====================================================================================================================
// Sorts values within pData by the given predicate using insertion sort.
template<typename T, typename Params, typename Predicate>
static void InsertionSort(
    T*            pData,
    uint32        count,
    Predicate     pred,
    const Params* pParams)
{
    for (uint32 i = 1; i < count; ++i)
    {
        T x = pData[i];

        int32 j;

        for (j = static_cast<int32>(i - 1); (j >= 0) && pred(x, pData[j], pParams); j--)
        {
            pData[j + 1] = pData[j];
        }

        pData[j + 1] = x;
    }
}

// =====================================================================================================================
// Sorts the subrange [start,end) of unsorted primitives within m_pSortedPrims into two subgroups and returns the
// mid-point.
//
// The sorting is based on calculating the bounding box of the range's primitive centroids, and splitting by the
// mid-point of the largest extent of that bounds.
uint32 CpuBvhBuilder::SortPrimitivesByLargestExtent(
    uint32 start,
    uint32 end)
{
    // Calculate centroid bounds
    Aabb centroidBound = NullBound;

    for (uint32 primIndex = start; primIndex < end; primIndex++)
    {
        const uint32 prim = m_pSortedPrims[primIndex];

        PAL_ASSERT(m_pPrimitives[prim].nodePointer != InvalidNode);

        centroidBound = BoundingBoxUnion(centroidBound, m_pPrimitives[prim].centroid);
    }

    // Sort by axis of largest extent based on primitive bound centroid
    uint32 dim = LargestExtent(centroidBound);

    // Find the first primitives between [start, midPoint) whose centroid along the axis is less
    // than the
    float midExtent;
    bool flat;

    switch (dim)
    {
    case 0:
        midExtent = (centroidBound.minX + centroidBound.maxX) * 0.5f;
        flat      = (centroidBound.minX == centroidBound.maxX);
        break;
    case 1:
        midExtent = (centroidBound.minY + centroidBound.maxY) * 0.5f;
        flat      = (centroidBound.minY == centroidBound.maxY);
        break;
    case 2:
        midExtent = (centroidBound.minZ + centroidBound.maxZ) * 0.5f;
        flat      = (centroidBound.minZ == centroidBound.maxZ);
        break;
    default:
        PAL_NEVER_CALLED();
        midExtent = 0;
        flat = true;
    }

    uint32 midIndex;

    if (flat)
    {
        // This can probably only happen with a completely collapsed set of primitives.
        midIndex = (start + end) / 2;
    }
    else
    {
        struct SortParams
        {
            const ScratchPrimitive* pPrimitives;
            uint32 dim;
            float midExtent;
        };

        SortParams params = { m_pPrimitives, dim, midExtent };

        midIndex = Partition(m_pSortedPrims, start, end, [](uint32 i, const SortParams* pParams) {
            const ScratchPrimitive* pPrimitive = &pParams->pPrimitives[i];

            return pPrimitive->centroid[pParams->dim] < pParams->midExtent;
        }, &params);
    }

    PAL_ASSERT(midIndex >= start && midIndex < end);

    return midIndex;
}

// =====================================================================================================================
// Given a value x along an axis [axisMin, axisMin+axisLength) that is split equally into bucketCount sub-ranges, this
// function will return the integer index identifying the sub-range containing x.
static uint32 SAHBucketIndex(
    float  x,
    float  axisMin,
    float  axisLength,
    uint32 bucketCount)
{
    float offset   = (x - axisMin) / axisLength;
    const uint32 b = Util::Min(uint32(bucketCount * offset), bucketCount - 1);

    return b;
}

// =====================================================================================================================
// Sorts the subrange [start,end) of unsorted primitives within m_pSortedPrims into two subgroups and returns the
// mid-point.
//
// The sorting is based on choosing some number of candidate split positions and choosing the one that
// minimizes the SAH cost function.
uint32 CpuBvhBuilder::SortPrimitivesBySAH(
    uint32 start,
    uint32 end)
{
    // Calculate centroid bounds
    Aabb centroidBound = NullBound;
    Aabb fullBound = NullBound;

    for (uint32 primIndex = start; primIndex < end; primIndex++)
    {
        const uint32 prim = m_pSortedPrims[primIndex];

        PAL_ASSERT(m_pPrimitives[prim].nodePointer != InvalidNode);

        centroidBound = BoundingBoxUnion(centroidBound, m_pPrimitives[prim].centroid);
        fullBound = BoundingBoxUnion(fullBound, m_pPrimitives[prim].bound);
    }

    // Sort by axis of largest extent based on primitive bound centroid
    uint32 dim = LargestExtent(centroidBound);

    float centroidExtent;
    float centroidMin;

    switch (dim)
    {
    case 0:
        centroidMin    = centroidBound.minX;
        centroidExtent = (centroidBound.maxX - centroidBound.minX);
        break;
    case 1:
        centroidMin    = centroidBound.minY;
        centroidExtent = (centroidBound.maxY - centroidBound.minY);
        break;
    case 2:
        centroidMin    = centroidBound.minZ;
        centroidExtent = (centroidBound.maxZ - centroidBound.minZ);
        break;
    default:
        PAL_NEVER_CALLED();
        centroidMin    = 0.0f;
        centroidExtent = 0.0f;
    }

    uint32 cut;

    // All primitives are perfectly on top of each other; arbitrarily cut in half
    if (centroidExtent == 0.0f)
    {
        cut = (start + end) / 2;
    }
    // Don't bother wasting time trying to bucket very few primitives; sort into two equally-sized parts
    else if ((end - start) <= m_cpuConfig.sahMinPrimitives)
    {
        struct SortParams
        {
            const ScratchPrimitive* pPrimitives;
            uint32                  dim;
        };

        SortParams params = { m_pPrimitives, dim };

        InsertionSort(&m_pSortedPrims[start], end - start, [](uint32 a, uint32 b, const SortParams* pParams) {
            const ScratchPrimitive& primA = pParams->pPrimitives[a];
            const ScratchPrimitive& primB = pParams->pPrimitives[b];

            return (primA.centroid[pParams->dim] < primB.centroid[pParams->dim]);
        }, &params);

        cut = (start + end) / 2;
    }
    // Choose a split boundary that minimizes the total SAH cost of the two sub-trees
    else
    {
        constexpr uint32 MaxBuckets = 16;

        // Bucket primitives equidistantly by centroid
        struct Bucket
        {
            uint32 n;
            Aabb   bounds;
        };

        PAL_ASSERT(m_cpuConfig.sahBucketCount > 0);
        PAL_ASSERT(m_cpuConfig.sahBucketCount <= MaxBuckets);

        Bucket buckets[MaxBuckets];

        for (uint32 i = 0; i < m_cpuConfig.sahBucketCount; ++i)
        {
            buckets[i].n = 0;
            buckets[i].bounds = NullBound;
        }

        for (uint32 index = start; index < end; ++index)
        {
            const uint32 prim  = m_pSortedPrims[index];
            float centroidDist = (m_pPrimitives[prim].centroid[dim] - centroidMin) / centroidExtent;

            const uint32 b = SAHBucketIndex(m_pPrimitives[prim].centroid[dim],
                                            centroidMin,
                                            centroidExtent,
                                            m_cpuConfig.sahBucketCount);

            buckets[b].n++;
            buckets[b].bounds = BoundingBoxUnion(buckets[b].bounds, m_pPrimitives[prim].bound);
        }

        // Minimize SAH cost for all possible split points
        float fullArea = BoundingBoxArea(fullBound);

        float minCost = FLT_MAX;
        uint32 bestBucket = 0;

        for (uint32 splitIndex = 0; splitIndex < m_cpuConfig.sahBucketCount - 1; ++splitIndex)
        {
            Aabb boundsA  = buckets[0].bounds;
            uint32 nA     = buckets[0].n;

            Aabb boundsB = buckets[splitIndex + 1].bounds;
            uint32 nB    = buckets[splitIndex + 1].n;

            for (uint32 a = 1; a <= splitIndex; ++a)
            {
                boundsA = BoundingBoxUnion(boundsA, buckets[a].bounds);
                nA += buckets[a].n;
            }

            for (uint32 b = splitIndex + 2; b < m_cpuConfig.sahBucketCount; ++b)
            {
                boundsB = BoundingBoxUnion(boundsB, buckets[b].bounds);
                nB += buckets[b].n;
            }

            float areaA = BoundingBoxArea(boundsA);
            float areaB = BoundingBoxArea(boundsB);

            // Evaluate SAH (intersection cost is factored out and baked into m_sahCostTraversal)
            float cost = m_cpuConfig.sahCostTraversal + (nA * areaA + nB * areaB) / fullArea;

            if (cost < minCost)
            {
                minCost = cost;
                bestBucket = splitIndex;
            }

            PAL_ASSERT((nA + nB) == (end - start));
        }

        // Given the optimal SAH split point, partition all primitives into two groups based on whether
        // they belong in bucket A or B.
        struct SortParams
        {
            const ScratchPrimitive* pPrimitives;
            uint32 dim;
            float centroidMin;
            float centroidExtent;
            uint32 bucketCount;
            uint32 bestBucket;
        };

        SortParams params = { m_pPrimitives, dim, centroidMin, centroidExtent, m_cpuConfig.sahBucketCount, bestBucket };

        cut = Partition(m_pSortedPrims, start, end, [](uint32 i, const SortParams* pParams) {
            const ScratchPrimitive* pPrimitive = &pParams->pPrimitives[i];

            const uint32 b = SAHBucketIndex(pPrimitive->centroid[pParams->dim],
                                            pParams->centroidMin,
                                            pParams->centroidExtent,
                                            pParams->bucketCount);

            return (b <= pParams->bestBucket);
        }, &params);
    }

    return cut;
}

// =====================================================================================================================
// Given a range within m_pSortedPrims, this function will sort the primitives into two groups that act as binary
// tree children and will return the midpoint between them.
uint32 CpuBvhBuilder::SortPrimitives(
    uint32 start,
    uint32 end)
{
    uint32 cut = 0;

    if (m_buildConfig.cpuBuildMode == BvhCpuBuildMode::RecursiveLargestExtent)
    {
        cut = SortPrimitivesByLargestExtent(start, end);
    }
    else if (m_buildConfig.cpuBuildMode == BvhCpuBuildMode::RecursiveSAH)
    {
        cut = SortPrimitivesBySAH(start, end);
    }
    else
    {
        PAL_NEVER_CALLED();
    }

    // Ensure that always at least two partitions are created
    PAL_ASSERT(cut > 0 && cut < end);

    cut = Util::Min(cut, end - 1);

    return cut;
}

// =====================================================================================================================
// Sorts an array range of ScratchPrimitive leaves into a binary BVH topology of ScratchBuildNodes
uint32 CpuBvhBuilder::BuildTree(
    uint32 startPrim,
    uint32 endPrim)
{
    constexpr uint32 MinPrimsPerLeaf = 1;

    const uint32 primCount = (endPrim - startPrim);

    uint32 nodeIndex;

    // Empty node
    if (primCount == 0)
    {
        nodeIndex = InvalidIndex;
    }
    // Interior node
    else if (primCount > MinPrimsPerLeaf)
    {
        nodeIndex = AcquireScratchBuildNode();

        ScratchBuildNode* pNode = &m_pBuildNodes[nodeIndex];

        // Partition the unsorted primitives into two sorted ranges and return the mid index between them
        uint32 midPoint;

        if (primCount > 2)
        {
            midPoint = SortPrimitives(startPrim, endPrim);
        }
        else
        {
            midPoint = startPrim + 1;
        }

        // TODO #gpurt: Parallelize
        uint32 left  = BuildTree(startPrim, midPoint);
        uint32 right = BuildTree(midPoint, endPrim);

        pNode->bound = BoundingBoxUnion(m_pBuildNodes[left].bound, m_pBuildNodes[right].bound);

        pNode->child[0]       = left;
        pNode->child[1]       = right;
        pNode->firstPrimitive = 0;
        pNode->primCount      = 0;
    }
    // Leaf node
    else
    {
        nodeIndex = AcquireScratchBuildNode();

        ScratchBuildNode* pNode = &m_pBuildNodes[nodeIndex];

        PAL_ASSERT(endPrim - startPrim > 0);

        pNode->firstPrimitive = startPrim;
        pNode->primCount      = endPrim - startPrim;
        pNode->child[0]       = InvalidIndex;
        pNode->child[1]       = InvalidIndex;
        pNode->bound          = NullBound;

        for (uint32 primIndex = startPrim; primIndex < endPrim; ++primIndex)
        {
            const uint32 prim = m_pSortedPrims[primIndex];

            PAL_ASSERT(m_pPrimitives[prim].nodePointer != InvalidNode);

            pNode->bound = BoundingBoxUnion(pNode->bound, m_pPrimitives[prim].bound);
        }
    }

    return nodeIndex;
}

// =====================================================================================================================
// Emits post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void CpuBvhBuilder::EmitASSerializationType(
    const AccelStructPostBuildInfo& postBuildInfo) // Postbuild info
{
    const AccelStructMetadataHeader* pMeta =
        static_cast<const AccelStructMetadataHeader*>(postBuildInfo.pSrcAccelStructCpuAddrs);

    const AccelStructHeader* pHeader =
        static_cast<const AccelStructHeader*>(
            Util::VoidPtrInc(postBuildInfo.pSrcAccelStructCpuAddrs,
                             pMeta->sizeInBytes));

    uint32 serializedSizeInBytes = 0;

    if (pHeader->info.type == static_cast<uint32>(AccelStructType::TopLevel))
    {
        const uint32 sizeOfPtrs = RayTracingGpuVaSize * pHeader->numPrimitives;

        serializedSizeInBytes = RayTracingSerializedAsHeaderSize + sizeOfPtrs + pHeader->sizeInBytes;
    }
    else
    {
        serializedSizeInBytes = pHeader->sizeInBytes + RayTracingSerializedAsHeaderSize;
    }

    memcpy(postBuildInfo.desc.postBuildBufferAddr.pCpu, &serializedSizeInBytes, sizeof(uint32));
}

// =====================================================================================================================
// Emits current size post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void CpuBvhBuilder::EmitASCurrentSize(
    const AccelStructPostBuildInfo& postBuildInfo) // Postbuild info
{
    const AccelStructMetadataHeader* pMeta =
        static_cast<const AccelStructMetadataHeader*>(postBuildInfo.pSrcAccelStructCpuAddrs);

    const AccelStructHeader* pHeader =
        static_cast<const AccelStructHeader*>(
            Util::VoidPtrInc(postBuildInfo.pSrcAccelStructCpuAddrs,
                pMeta->sizeInBytes));

    uint32 sizeInBytes = pHeader->sizeInBytes;
    memcpy(postBuildInfo.desc.postBuildBufferAddr.pCpu, &sizeInBytes, sizeof(uint32));
}

// =====================================================================================================================
// Emits blas count post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void CpuBvhBuilder::EmitASBottomLevelAsPointerCount(
    const AccelStructPostBuildInfo& postBuildInfo) // Postbuild info
{
    const AccelStructMetadataHeader* pMeta =
        static_cast<const AccelStructMetadataHeader*>(postBuildInfo.pSrcAccelStructCpuAddrs);

    const AccelStructHeader* pHeader =
        static_cast<const AccelStructHeader*>(
            Util::VoidPtrInc(postBuildInfo.pSrcAccelStructCpuAddrs,
                pMeta->sizeInBytes));

    uint32 bottomLevelASPointersCount = (pHeader->info.type == static_cast<uint32>(AccelStructType::TopLevel)) ?
                                        pHeader->numPrimitives : 0;
    memcpy(postBuildInfo.desc.postBuildBufferAddr.pCpu, &bottomLevelASPointersCount, sizeof(uint32));
}

// =====================================================================================================================
uint32 CpuBvhBuilder::CalcCompactedSize(
    const AccelStructHeader* pSrcHeader,
    AccelStructDataOffsets*  pDstOffsets,
    uint32*                  pMetadataSizeInBytes)
{
    // Acceleration structure data starts with the header (not including the metadata)
    uint32 runningOffset = sizeof(AccelStructHeader);

    pDstOffsets->internalNodes = runningOffset;

    uint32 internalNodeSize = 0;
    uint32 leafNodeSize     = 0;

    if (pSrcHeader->info.type == static_cast<uint32>(AccelStructType::BottomLevel))
    {
        internalNodeSize = pSrcHeader->numInternalNodesFp32 * sizeof(Float32BoxNode) +
                           pSrcHeader->numInternalNodesFp16 * sizeof(Float16BoxNode);
        runningOffset   += internalNodeSize;

        pDstOffsets->leafNodes = runningOffset;
        leafNodeSize           = pSrcHeader->numLeafNodes *
                                 GetBvhNodeSizeLeaf(static_cast<GeometryType>(pSrcHeader->geometryType));
        runningOffset         += leafNodeSize;

        pDstOffsets->geometryInfo = runningOffset;
        runningOffset            += pSrcHeader->numDescs * sizeof(GeometryInfo);

        pDstOffsets->primNodePtrs = runningOffset;
        runningOffset            += pSrcHeader->numPrimitives * sizeof(uint32);
    }
    else
    {
        // TLAS always uses 32-bit internal nodes
        internalNodeSize = pSrcHeader->numInternalNodesFp32 * sizeof(Float32BoxNode);
        runningOffset   += internalNodeSize;

        pDstOffsets->leafNodes = runningOffset;
        leafNodeSize           = pSrcHeader->numLeafNodes * RayTracingInstanceNodeSize;
        runningOffset         += leafNodeSize;

        // Top level acceleration structures do not have geometry info.
        pDstOffsets->geometryInfo = 0;

        pDstOffsets->primNodePtrs = runningOffset;
        runningOffset            += pSrcHeader->numPrimitives * sizeof(uint32);
    }

    // Align metadata size to 128B cache line boundary
    *pMetadataSizeInBytes = pSrcHeader->metadataSizeInBytes;

    // Return total size including metadata
    return runningOffset + *pMetadataSizeInBytes;
}

// =====================================================================================================================
// Emits post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void CpuBvhBuilder::EmitASCompactedType(
    const AccelStructPostBuildInfo& postBuildInfo) // Postbuild info
{
    const AccelStructMetadataHeader* pMeta =
        static_cast<const AccelStructMetadataHeader*>(postBuildInfo.pSrcAccelStructCpuAddrs);

    const AccelStructHeader* pHeader =
        static_cast<const AccelStructHeader*>(
            Util::VoidPtrInc(postBuildInfo.pSrcAccelStructCpuAddrs,
                pMeta->sizeInBytes));

    AccelStructDataOffsets dstOffsets = {};
    uint32 dstMetadataSizeInBytes = 0;

    uint32 compactedSize = CalcCompactedSize(
        pHeader,
        &dstOffsets,
        &dstMetadataSizeInBytes);

    memcpy(postBuildInfo.desc.postBuildBufferAddr.pCpu, &compactedSize, sizeof(uint32));
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it to a same sized destination memory
void CpuBvhBuilder::CopyASCloneMode(
    const AccelStructCopyInfo& copyArgs) // Copy arguments
{
    const AccelStructMetadataHeader* pMeta =
        static_cast<const AccelStructMetadataHeader*>(copyArgs.srcAccelStructAddr.pCpu);

    const AccelStructHeader* pHeader =
        static_cast<const AccelStructHeader*>(
            Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu,
                pMeta->sizeInBytes));

    memcpy(copyArgs.dstAccelStructAddr.pCpu, copyArgs.srcAccelStructAddr.pCpu, pHeader->sizeInBytes);
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it to a same sized destination memory
void CpuBvhBuilder::CopyASCompactMode(
    const AccelStructCopyInfo& copyArgs) // Copy arguments
{
    // The following code assumes the base address of SrcBuffer and DstBuffer includes the acceleration structure
    // metadata header

    // Fetch acceleration structure metadata size
    const AccelStructMetadataHeader* pMeta =
        static_cast<const AccelStructMetadataHeader*>(copyArgs.srcAccelStructAddr.pCpu);

    const AccelStructHeader* pSrcHeader =
        static_cast<const AccelStructHeader*>(
            Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu,
                pMeta->sizeInBytes));

    m_buildArgs.pSrcAccelStructCpuAddr = copyArgs.srcAccelStructAddr.pCpu;
    m_buildArgs.pDstAccelStructCpuAddr = copyArgs.dstAccelStructAddr.pCpu;
    m_metadataSizeInBytes              = pMeta->sizeInBytes;

    const AccelStructDataOffsets srcOffsets = pSrcHeader->offsets;

    uint32 srcMetadataSizeInBytes = pMeta->sizeInBytes;
    uint32 dstMetadataSizeInBytes = 0;

    AccelStructDataOffsets dstOffsets = {};

    const uint32 dstSizeInBytes = CalcCompactedSize(
        pSrcHeader,
        &dstOffsets,
        &dstMetadataSizeInBytes);

    // Patch acceleration structure metadata with updated address
    // Offset to acceleration structure header
    const uint64 gpuVa = *reinterpret_cast<const uint64*>(copyArgs.srcAccelStructAddr.pCpu);

    // Write the destination headers
    AccelStructMetadataHeader* pMetaHeader =
        reinterpret_cast<AccelStructMetadataHeader*>(copyArgs.dstAccelStructAddr.pCpu);

    memset(pMetaHeader, 0, sizeof(*pMetaHeader));

    if (pSrcHeader->numActivePrims > 0)
    {
        pMetaHeader->addressLo   = Util::LowPart(gpuVa);
        pMetaHeader->addressHi   = Util::HighPart(gpuVa);
        pMetaHeader->sizeInBytes = dstMetadataSizeInBytes;
    }

    AccelStructHeader* pDstHeader = reinterpret_cast<AccelStructHeader*>(
        Util::VoidPtrInc(copyArgs.dstAccelStructAddr.pCpu, dstMetadataSizeInBytes));

    *pDstHeader                     = *pSrcHeader;
    pDstHeader->numDescs            = 0;
    pDstHeader->sizeInBytes         = dstSizeInBytes;
    pDstHeader->offsets             = dstOffsets;

    // Add metadata size to get to absolute data offsets in source/destination memory
    const uint32 srcOffsetDataInternalNodes = srcOffsets.internalNodes + srcMetadataSizeInBytes;
    const uint32 srcOffsetDataLeafNodes     = srcOffsets.leafNodes     + srcMetadataSizeInBytes;
    const uint32 srcOffsetDataGeometryInfo  = srcOffsets.geometryInfo  + srcMetadataSizeInBytes;
    const uint32 srcOffsetDataPrimNodePtrs  = srcOffsets.primNodePtrs  + srcMetadataSizeInBytes;

    const uint32 dstOffsetDataInternalNodes = dstOffsets.internalNodes + dstMetadataSizeInBytes;
    const uint32 dstOffsetDataLeafNodes     = dstOffsets.leafNodes     + dstMetadataSizeInBytes;
    const uint32 dstOffsetDataGeometryInfo  = dstOffsets.geometryInfo  + dstMetadataSizeInBytes;
    const uint32 dstOffsetDataPrimNodePtrs  = dstOffsets.primNodePtrs  + dstMetadataSizeInBytes;

    // Copy internal nodes
    // 16-bit internal nodes only apply to BLAS
    if (pSrcHeader->info.type == static_cast<uint32>(AccelStructType::BottomLevel))
    {
        if (m_deviceSettings.fp16BoxNodesInBlasMode == Fp16BoxNodesInBlasMode::NoNodes)
        {
            for (uint32 nodeIndex = 0; nodeIndex < pSrcHeader->numInternalNodesFp32; ++nodeIndex)
            {
                const uint nodeOffset = nodeIndex * sizeof(Float32BoxNode);

                CopyFp32BoxNode(
                    copyArgs.srcAccelStructAddr.pCpu,
                    copyArgs.dstAccelStructAddr.pCpu,
                    nodeOffset,
                    srcOffsetDataInternalNodes,
                    srcMetadataSizeInBytes,
                    dstOffsetDataInternalNodes,
                    dstMetadataSizeInBytes);
            }
       }
        else if (m_deviceSettings.fp16BoxNodesInBlasMode == Fp16BoxNodesInBlasMode::AllNodes)
        {
            // TODO #gpurt: Handle FP16
            PAL_NOT_IMPLEMENTED();
        }
        else
        {
            // Because interior box nodes are mixed (fp16 and fp32), each thread traverses up the tree from leaf nodes
            // following parent pointers, copying each parent. The iteration traversing up the tree continues
            // only for the thread coming from the 1st child of the parent.

            // NOTE: in case this method is slow, we can try other alternatives which use some extra memory:
            // 1. add sideband data which is a bitfield. Each bit corresponds to an interior node and would store
            //    a flag whether the node is fp16 or fp32. Computing offset to a particular node that corresponds
            //    to a bit i requries counting bits [0...i-1] and multiplying by 64B.
            // 2. store atomic counters per node to count how many children were processed. The last thread
            //    continues up the tree. See UpdateQBVHImpl.hlsl
            // 3. traverse tree top-down and use a stack for the next node address to process. The stack passes
            //    indexes to threads. See BuildQBVH.hlsl

            // TODO #gpurt: Handle FP16
            PAL_NOT_IMPLEMENTED();
        }
    }
    else
    {
        // TOP_LEVEL
        for (uint32 nodeIndex = 0; nodeIndex < pSrcHeader->numInternalNodesFp32; ++nodeIndex)
        {
            const uint32 nodeOffset = nodeIndex * sizeof(Float32BoxNode);

            CopyFp32BoxNode(
                copyArgs.srcAccelStructAddr.pCpu,
                copyArgs.dstAccelStructAddr.pCpu,
                nodeOffset,
                srcOffsetDataInternalNodes,
                srcMetadataSizeInBytes,
                dstOffsetDataInternalNodes,
                dstMetadataSizeInBytes);
        }
    }

    // Copy leaf nodes
    if (pSrcHeader->info.type == static_cast<uint32>(AccelStructType::TopLevel))
    {
        for (uint32 nodeIndex = 0; nodeIndex < pSrcHeader->numLeafNodes; ++nodeIndex)
        {
            const uint32 nodeOffset        = nodeIndex * sizeof(InstanceNode);
            const uint32 srcNodeDataOffset = srcOffsetDataLeafNodes + nodeOffset;
            const uint32 dstNodeDataOffset = dstOffsetDataLeafNodes + nodeOffset;

            // Copy instance node
            const InstanceNode* pSrcNode = static_cast<const InstanceNode*>(
                Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu, srcNodeDataOffset));

            InstanceNode* pDstNode = static_cast<InstanceNode*>(
                Util::VoidPtrInc(copyArgs.dstAccelStructAddr.pCpu, dstNodeDataOffset));

            *pDstNode = *pSrcNode;

            // Top level acceleration structures do not have geometry info, or primitive node pointers.
            const uint32 srcNodePointer =
                PackNodePointer(NODE_TYPE_USER_NODE_INSTANCE, srcOffsets.leafNodes + nodeOffset);

            const uint32 dstNodePointer =
                PackNodePointer(NODE_TYPE_USER_NODE_INSTANCE, dstOffsets.leafNodes + nodeOffset);

            // Fix up the child pointer in the parent node
            WriteParentNodeChildPointer(
                copyArgs.srcAccelStructAddr.pCpu,
                copyArgs.dstAccelStructAddr.pCpu,
                srcNodePointer,
                dstNodePointer);
        }
    }
    else if (pSrcHeader->geometryType == static_cast<uint32>(GeometryType::Triangles))
    {
        for (uint32 nodeIndex = 0; nodeIndex < pSrcHeader->numLeafNodes; ++nodeIndex)
        {
            const uint32 nodeOffset = nodeIndex * sizeof(TriangleNode);
            const uint32 srcNodeDataOffset = srcOffsetDataLeafNodes + nodeOffset;
            const uint32 dstNodeDataOffset = dstOffsetDataLeafNodes + nodeOffset;

            // Copy triangle node data
            const TriangleNode* pSrcNode = static_cast<const TriangleNode*>(
                Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu, srcNodeDataOffset));

            TriangleNode* pDstNode = static_cast<TriangleNode*>(
                Util::VoidPtrInc(copyArgs.dstAccelStructAddr.pCpu, dstNodeDataOffset));

            *pDstNode = *pSrcNode;

            // Handle per-primitive data
            for (uint32 nodeType = 0; nodeType < 4; nodeType++)
            {
                if (((pSrcNode->triangleId >> (nodeType * TRIANGLE_ID_BIT_STRIDE)) & 0xf) != 0)
                {
                    const uint srcNodePointer = PackNodePointer(nodeType, srcOffsets.leafNodes + nodeOffset);
                    const uint dstNodePointer = PackNodePointer(nodeType, dstOffsets.leafNodes + nodeOffset);

                    // Fix up the child pointer in the parent node
                    WriteParentNodeChildPointer(
                        copyArgs.srcAccelStructAddr.pCpu,
                        copyArgs.dstAccelStructAddr.pCpu,
                        srcNodePointer,
                        dstNodePointer);
                }
            }
        }
    }
    else // GEOMETRY_TYPE_AABBS
    {
        PAL_ASSERT(pSrcHeader->geometryType == static_cast<uint32>(GeometryType::Aabbs));

        for (uint32 nodeIndex = 0; nodeIndex < pSrcHeader->numLeafNodes; ++nodeIndex)
        {
            const uint32 nodeOffset = nodeIndex * sizeof(ProceduralNode);
            const uint32 srcNodeDataOffset = srcOffsetDataLeafNodes + nodeOffset;
            const uint32 dstNodeDataOffset = dstOffsetDataLeafNodes + nodeOffset;

            // Copy triangle node data
            const ProceduralNode* pSrcNode = static_cast<const ProceduralNode*>(
                Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu, srcNodeDataOffset));

            ProceduralNode* pDstNode = static_cast<ProceduralNode*>(
                Util::VoidPtrInc(copyArgs.dstAccelStructAddr.pCpu, dstNodeDataOffset));

            *pDstNode = *pSrcNode;

            const uint32 srcNodePointer = PackNodePointer(NODE_TYPE_USER_NODE_PROCEDURAL, srcOffsets.leafNodes + nodeOffset);
            const uint32 dstNodePointer = PackNodePointer(NODE_TYPE_USER_NODE_PROCEDURAL, dstOffsets.leafNodes + nodeOffset);

            // Fix up the child pointer in the parent node
            WriteParentNodeChildPointer(
                copyArgs.srcAccelStructAddr.pCpu,
                copyArgs.dstAccelStructAddr.pCpu,
                srcNodePointer,
                dstNodePointer);
        }
    }

    if (pSrcHeader->info.type == static_cast<uint32>(AccelStructType::BottomLevel))
    {
        PAL_ASSERT(m_buildConfig.triangleCompressionMode == TriangleCompressionMode::None);

        // Copy the geometry info
        for (uint32 geometryIndex = 0; geometryIndex < pSrcHeader->numDescs; ++geometryIndex)
        {
            const uint32 srcGeometryInfoOffset = srcOffsetDataGeometryInfo + (geometryIndex * sizeof(GeometryInfo));
            const uint32 dstGeometryInfoOffset = dstOffsetDataGeometryInfo + (geometryIndex * sizeof(GeometryInfo));

            const GeometryInfo* pSrcNode = static_cast<const GeometryInfo*>(
                Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu, srcGeometryInfoOffset));

            GeometryInfo* pDstNode = static_cast<GeometryInfo*>(
                Util::VoidPtrInc(copyArgs.dstAccelStructAddr.pCpu, dstGeometryInfoOffset));

            *pDstNode = *pSrcNode;
        }
    }

    // Copy primitive node pointers
    for (uint32 primIndex = 0; primIndex < pSrcHeader->numPrimitives; ++primIndex)
    {
        const uint32 srcNodePtrOffset = srcOffsetDataPrimNodePtrs + (primIndex * NODE_PTR_SIZE);
        const uint32 dstNodePtrOffset = dstOffsetDataPrimNodePtrs + (primIndex * NODE_PTR_SIZE);

        const uint32 srcNodePointer = *static_cast<const uint32*>(
            Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu, srcNodePtrOffset));

        uint32 dstNodePointer = InvalidIndex;
        if (srcNodePointer != InvalidIndex)
        {
            const uint32 srcNodeType   = GetNodeType(srcNodePointer);
            const uint32 srcNodeOffset = ExtractNodePointerOffset(srcNodePointer);

            // Increment primitive count to offset the decrement in PackLeafNodePointer
            const uint collapsePrimCount = ExtractPrimitiveCount(srcNodePointer) + 1;

            const uint nodeIndex = (srcNodeOffset - srcOffsets.leafNodes) / sizeof(TriangleNode);
            const uint nodeOffset = (nodeIndex * sizeof(TriangleNode));

            dstNodePointer = PackLeafNodePointer(srcNodeType, dstOffsets.leafNodes + nodeOffset, collapsePrimCount);
        }

        uint32* pDstNode = static_cast<uint32*>(
            Util::VoidPtrInc(copyArgs.dstAccelStructAddr.pCpu, dstNodePtrOffset));

        *pDstNode = dstNodePointer;
    }
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it in a serialized manner to the destination memory
void CpuBvhBuilder::CopyASSerializeMode(
    const AccelStructCopyInfo& copyArgs) // Copy arguments
{
    const AccelStructMetadataHeader* pMeta =
        static_cast<const AccelStructMetadataHeader*>(copyArgs.srcAccelStructAddr.pCpu);

    const AccelStructHeader* pHeader = static_cast<const AccelStructHeader*>(
        Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu, pMeta->sizeInBytes));

    const uint32 sizeOfPtrs = RayTracingGpuVaSize * pHeader->numPrimitives;

    uint serializedHeaderSize = RayTracingSerializedAsHeaderSize;
    if (pHeader->info.type == static_cast<uint32>(AccelStructType::TopLevel))
    {
        serializedHeaderSize = serializedHeaderSize + sizeOfPtrs;
    }

    // 1 - SerializedSizeInBytesIncludingHeader
    uint32* pDstCpu = static_cast<uint32*>(Util::VoidPtrInc(copyArgs.dstAccelStructAddr.pCpu,
                                                            RayTracingSerializedAsHeaderSizeOffset));

    *pDstCpu = pHeader->sizeInBytes + serializedHeaderSize;

    // 2 - DeserializedSizeInBytes
    pDstCpu = static_cast<uint32*>(Util::VoidPtrInc(copyArgs.dstAccelStructAddr.pCpu,
                                                    RayTracingDeSerializedAsHeaderSizeOffset));

    *pDstCpu = pHeader->sizeInBytes;

    if (pHeader->info.type == static_cast<uint32>(AccelStructType::TopLevel))
    {
        const uint32 basePrimNodePtrsOffset = pMeta->sizeInBytes + pHeader->offsets.primNodePtrs;

        // 3 - NumBottomLevelAccelerationStructurePointersAfterHeader
        pDstCpu = static_cast<uint32*>(Util::VoidPtrInc(copyArgs.dstAccelStructAddr.pCpu,
                                                        RayTracingSerializedAsHeaderNumBlasPtrsOffset));
        *pDstCpu = pHeader->numPrimitives;

        // bottom tree GPUVAs after header (SERIALIZED_AS_HEADER_SIZE)
        for (uint32 i = 0; i < pHeader->numPrimitives; i++)
        {
            const uint32 currentInstNodePtrOffset = basePrimNodePtrsOffset + (i * sizeof(NodePointer));
            const uint32 currentInstNodePtr = *static_cast<const uint32*>(
                Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu, currentInstNodePtrOffset));

            uint64 gpuVa = 0;

            if (currentInstNodePtr != InvalidNode)
            {
                const uint32 currentInstNodeOffset = pMeta->sizeInBytes + ExtractNodePointerOffset(currentInstNodePtr);

                // Fetch acceleration structure base address in TLAS instance
                gpuVa = *static_cast<const uint64*>(
                    Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu,
                                     currentInstNodeOffset + offsetof(InstanceNode, desc.accelerationStructure)));

                // Mask off instance flags in the upper bits of the acceleration structure address
                gpuVa = ExtractInstanceAddress(gpuVa);

                // Skip null BLAS
                if (gpuVa != 0)
                {
                    const uint32 blasMetadataSizeInBytes = *static_cast<const uint32*>(
                        Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu,
                                         currentInstNodeOffset + offsetof(InstanceNode, extra.blasMetadataSize)));

                    // - offset to base of bottom level acceleration structure
                    gpuVa -= blasMetadataSizeInBytes;
                }
            }

            // Write original BLAS address to destination buffer
            uint32 blasIndex = (i * RayTracingGpuVaSize) + RayTracingSerializedAsHeaderSize;
            uint64* pDstGpuVa = static_cast<uint64*>(Util::VoidPtrInc(copyArgs.dstAccelStructAddr.pCpu, blasIndex));
            *pDstGpuVa = gpuVa;
        }
    }
    else
    {
        // 3 - NumBottomLevelAccelerationStructurePointersAfterHeader
        uint64* pDstGpuVa = static_cast<uint64*>(
            Util::VoidPtrInc(copyArgs.dstAccelStructAddr.pCpu, RayTracingSerializedAsHeaderNumBlasPtrsOffset));
        *pDstGpuVa = 0;
    }

    const uint32* pSrcCpu = static_cast<const uint32*>(copyArgs.srcAccelStructAddr.pCpu);
    pDstCpu = static_cast<uint32*>(Util::VoidPtrInc(copyArgs.dstAccelStructAddr.pCpu, serializedHeaderSize));
    memcpy(pDstCpu, pSrcCpu, pHeader->sizeInBytes);
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it in a deserialized manner to the destination memory
void CpuBvhBuilder::CopyASDeserializeMode(
    const AccelStructCopyInfo& copyArgs) // Copy arguments
{
    const uint32 numberOfPtrs = *static_cast<const uint32*>(
        Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu, RayTracingSerializedAsHeaderNumBlasPtrsOffset));

    const uint32 sizeOfPtrs = RayTracingGpuVaSize * numberOfPtrs;
    const uint32 serializedHeaderSize = RayTracingSerializedAsHeaderSize + sizeOfPtrs;

    // Fetch acceleration structure metadata size
    const uint32 metadataSizeInBytes = *static_cast<const uint32*>(
        Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu, serializedHeaderSize + RayTracingGpuVaSize));

    const AccelStructHeader* pHeader = static_cast<const AccelStructHeader*>(
        Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu, serializedHeaderSize + metadataSizeInBytes));

    const uint32* pSrcCpu = static_cast<const uint32*>(
        Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu, serializedHeaderSize));

    uint32* pDstCpu = static_cast<uint32*>(copyArgs.dstAccelStructAddr.pCpu);

    // Copy from SrcBuffer->DstBuffer skipping over D3D12_SERIALIZED_ACCELERATION_STRUCTURE_HEADER and list of ptrs.
    memcpy(pDstCpu, pSrcCpu, pHeader->sizeInBytes);

    if (pHeader->info.type == static_cast<uint32>(AccelStructType::TopLevel))
    {
        // Update BLAS pointers in the instance nodes
        const uint32 basePrimNodePtrsOffset = metadataSizeInBytes + pHeader->offsets.primNodePtrs;

        for (uint32 i = 0; i < pHeader->numPrimitives; ++i)
        {
            const uint32 currentInstNodePtrOffset = basePrimNodePtrsOffset + (i * sizeof(NodePointer));
            const uint32 currentInstNodePtr = *static_cast<const uint32*>(
                Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu, serializedHeaderSize + currentInstNodePtrOffset));

            if (currentInstNodePtr != InvalidNode)
            {
                const uint32 currentInstNodeOffset = metadataSizeInBytes + ExtractNodePointerOffset(currentInstNodePtr);
                const uint32 serializedInstNodeOffset = serializedHeaderSize + currentInstNodeOffset;

                // Fetch acceleration structure base address in TLAS instance
                const uint64 oldGpuVa = *static_cast<const uint64*>(
                    Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu,
                                     serializedInstNodeOffset + offsetof(InstanceNode, desc.accelerationStructure)));

                uint64 newGpuVa = *static_cast<const uint64*>(
                    Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu,
                                     RayTracingSerializedAsHeaderSize + i * RayTracingGpuVaSize));

                // Handle null BLAS address
                if (newGpuVa != 0)
                {
                    const uint32 blasMetadataSizeInBytes = *static_cast<const uint32*>(
                        Util::VoidPtrInc(copyArgs.srcAccelStructAddr.pCpu,
                                         serializedInstNodeOffset + offsetof(InstanceNode, extra.blasMetadataSize)));

                    newGpuVa = newGpuVa + blasMetadataSizeInBytes;
                }

                // Preserve the instance flags in the high bits of the address.
                uint64* pDstCpu64 = static_cast<uint64*>(
                    Util::VoidPtrInc(copyArgs.dstAccelStructAddr.pCpu,
                                     currentInstNodeOffset + offsetof(InstanceNode, desc.accelerationStructure)));
                *pDstCpu64 = (oldGpuVa & ~(INSTANCE_BASE_POINTER_ADDRESS_USED_MASK)) | (newGpuVa >> 3);

            }
        }
    }

    // Patch acceleration structure metadata with updated address
    // Offset to acceleration structure header
    uint64 gpuVa = *static_cast<uint64*>(copyArgs.dstAccelStructAddr.pCpu);

    AccelStructMetadataHeader* pDstMeta =
        static_cast<AccelStructMetadataHeader*>(copyArgs.dstAccelStructAddr.pCpu);
    gpuVa += pDstMeta->sizeInBytes;

    if (pHeader->info.type == static_cast<uint32>(AccelStructType::TopLevel))
    {
        if (pHeader->numActivePrims > 0)
        {
            pDstMeta->addressLo = Util::LowPart(gpuVa);
            pDstMeta->addressHi = Util::HighPart(gpuVa);
            pDstMeta->sizeInBytes = metadataSizeInBytes;
        }
        else
        {
            pDstMeta->addressLo = 0;
            pDstMeta->addressHi = 0;
            pDstMeta->sizeInBytes = 0;
        }
    }
    else
    {
        pDstMeta->addressLo = Util::LowPart(gpuVa);
        pDstMeta->addressHi = Util::HighPart(gpuVa);
        pDstMeta->sizeInBytes = metadataSizeInBytes;
    }
}

// =====================================================================================================================
// Takes a source acceleration structure and copies it to destination memory
void CpuBvhBuilder::CopyAccelerationStructure(
    const AccelStructCopyInfo& copyArgs) // Copy arguments
{
    switch (copyArgs.mode)
    {
    case AccelStructCopyMode::Clone:
        CopyASCloneMode(copyArgs);
        break;

    case AccelStructCopyMode::Compact:
        CopyASCompactMode(copyArgs);
        break;

    case AccelStructCopyMode::Serialize:
        CopyASSerializeMode(copyArgs);
        break;

    case AccelStructCopyMode::Deserialize:
        CopyASDeserializeMode(copyArgs);
        break;

    default:
        PAL_ASSERT_ALWAYS();
        break;
    }
}

// =====================================================================================================================
// Emits post-build properties for a set of acceleration structures.
// This enables applications to know the output resource requirements for performing acceleration structure
// operations via CopyRaytracingAccelerationStructure()
void CpuBvhBuilder::EmitAccelerationStructurePostBuildInfo(
    const AccelStructPostBuildInfo& postBuildInfo) // Postbuild info
{
    switch (postBuildInfo.desc.infoType)
    {
    case AccelStructPostBuildInfoType::Serialization:
        EmitASSerializationType(postBuildInfo);
        break;

    case AccelStructPostBuildInfoType::CompactedSize:
        EmitASCompactedType(postBuildInfo);
        break;

    case AccelStructPostBuildInfoType::CurrentSize:
        EmitASCurrentSize(postBuildInfo);
        break;

    case AccelStructPostBuildInfoType::BottomLevelASPointerCount:
        EmitASBottomLevelAsPointerCount(postBuildInfo);
        break;

    default:
        PAL_ASSERT_ALWAYS();
        break;
    }
}

// =====================================================================================================================
uint32 CpuBvhBuilder::AcquireScratchBuildNode()
{
    PAL_ASSERT(IsUpdate() == false);

    uint32 nodeIndex = Util::AtomicIncrement(&m_cpuConfig.buildNodeCount) - 1;

    PAL_ASSERT(nodeIndex < m_scratchOffsets.maxScratchBuildNodes);

    return nodeIndex;
}

// =====================================================================================================================
// Acquires an internal box node from the list and returns a node pointer to it
NodePointer CpuBvhBuilder::AcquireInternalNode()
{
    PAL_ASSERT(IsUpdate() == false);
    PAL_ASSERT(m_buildConfig.fp16BoxNodesInBlasMode == Fp16BoxNodesInBlasMode::NoNodes);

    const uint32 nodeIndex  = Util::AtomicIncrement(&m_dstHeader.numInternalNodesFp32) - 1;

    PAL_ASSERT(nodeIndex < m_scratchOffsets.maxQbvhInternalNodes);

    const uint32 nodeOffset = m_dstHeader.offsets.internalNodes + nodeIndex * m_cpuConfig.internalNodeSize;

    NodePointer nodePointer = PackNodePointer(m_cpuConfig.internalNodeType, nodeOffset);

    return nodePointer;
}

// =====================================================================================================================
// Writes an instance leaf node description through the given pointer
void CpuBvhBuilder::WriteInstanceLeafNode(
    NodePointer         nodePointer,
    uint32              instanceIndex,
    const InstanceDesc& desc)
{
    InstanceNode node = {};

    node.desc = desc;

    // Backup original instance transform and replace it with the inverse
    memcpy(node.extra.objectToWorld, desc.transform, sizeof(node.extra.objectToWorld));

    node.extra.instanceIndex = instanceIndex;

    MatrixInverse(node.desc.transform, desc.transform);

    uint32 nodeOffset = ExtractNodePointerOffset(nodePointer);

    *ResultBuffer<InstanceNode>(nodeOffset) = node;
}

// =====================================================================================================================
// Write an internal box node (either 16 or 32-bit) into the result buffer through the given pointer
void CpuBvhBuilder::WriteInternalBoxNode(
    NodePointer nodePointer,
    uint32      childPointers[4],
    Aabb        bounds[4])
{
    PAL_ASSERT(IsBoxNode(nodePointer));
    PAL_ASSERT(ExtractNodePointerOffset(nodePointer) >= m_dstHeader.offsets.internalNodes);

    if (IsBoxNode16(nodePointer))
    {
        // TODO #gpurt: fp16 box nodes
        PAL_NOT_IMPLEMENTED();
    }
    else
    {
        PAL_ASSERT(IsBoxNode32(nodePointer));

        Float32BoxNode box;

        box.child0    = childPointers[0];
        box.child1    = childPointers[1];
        box.child2    = childPointers[2];
        box.child3    = childPointers[3];

        box.bbox0_min.x = bounds[0].minX;
        box.bbox0_min.y = bounds[0].minY;
        box.bbox0_min.z = bounds[0].minZ;
        box.bbox0_max.x = bounds[0].maxX;
        box.bbox0_max.y = bounds[0].maxY;
        box.bbox0_max.z = bounds[0].maxZ;

        box.bbox1_min.x = bounds[1].minX;
        box.bbox1_min.y = bounds[1].minY;
        box.bbox1_min.z = bounds[1].minZ;
        box.bbox1_max.x = bounds[1].maxX;
        box.bbox1_max.y = bounds[1].maxY;
        box.bbox1_max.z = bounds[1].maxZ;

        box.bbox2_min.x = bounds[2].minX;
        box.bbox2_min.y = bounds[2].minY;
        box.bbox2_min.z = bounds[2].minZ;
        box.bbox2_max.x = bounds[2].maxX;
        box.bbox2_max.y = bounds[2].maxY;
        box.bbox2_max.z = bounds[2].maxZ;

        box.bbox3_min.x = bounds[3].minX;
        box.bbox3_min.y = bounds[3].minY;
        box.bbox3_min.z = bounds[3].minZ;
        box.bbox3_max.x = bounds[3].maxX;
        box.bbox3_max.y = bounds[3].maxY;
        box.bbox3_max.z = bounds[3].maxZ;

        box.padding0  = 0;
        box.padding1  = 0;
        box.padding2  = 0;
        box.padding3  = 0;

        const uint32 nodeOffset = ExtractNodePointerOffset(nodePointer);

        Float32BoxNode* pResult32 = ResultBuffer<Float32BoxNode>(nodeOffset);

        *pResult32 = box;
    }

    for (uint32 i = 0; i < 4; ++i)
    {
        if (childPointers[i] != InvalidNode)
        {
            WriteParentPointer(childPointers[i], nodePointer);
        }
    }
}

//=====================================================================================================================
// Update Parent Pointer
NodePointer CpuBvhBuilder::UpdateParentPointer(
    NodePointer srcNodePointer,
    NodePointer dstNodePointer)
{
    uint32 parentNodePointer = ReadParentPointer(srcNodePointer);

    WriteParentPointer(
        dstNodePointer,
        parentNodePointer);

    return parentNodePointer;
}

//=====================================================================================================================
// Copies FP32 box node
void CpuBvhBuilder::CopyFp32BoxNode(
    const void* pSrcBuffer,
    void*       pDstBuffer,
    uint32      nodeOffset,
    uint32      srcOffsetDataInternalNodes,
    uint32      srcMetadataSizeInBytes,
    uint32      dstOffsetDataInternalNodes,
    uint32      dstMetadataSizeInBytes)
{
    const uint srcInternalNodeDataOffset = srcOffsetDataInternalNodes + nodeOffset;
    const uint dstInternalNodeDataOffset = dstOffsetDataInternalNodes + nodeOffset;

    const Float32BoxNode* pSrcNode =
        static_cast<const Float32BoxNode*>(Util::VoidPtrInc(pSrcBuffer, srcInternalNodeDataOffset));

    // Skip leaf child pointers because they will be updated while copying leaf nodes
    if ((pSrcNode->child0 == InvalidIndex) || IsBoxNode(pSrcNode->child0))
    {
        uint32* pDstNode = static_cast<uint32*>(
            Util::VoidPtrInc(pDstBuffer, dstInternalNodeDataOffset + RayTracingFloat32BoxNodeChild0Offset));
        *pDstNode = pSrcNode->child0;
    }

    if ((pSrcNode->child1 == InvalidIndex) || IsBoxNode(pSrcNode->child1))
    {
        uint32* pDstNode = static_cast<uint32*>(
            Util::VoidPtrInc(pDstBuffer, dstInternalNodeDataOffset + RayTracingFloat32BoxNodeChild1Offset));
        *pDstNode = pSrcNode->child1;
    }

    if ((pSrcNode->child2 == InvalidIndex) || IsBoxNode(pSrcNode->child2))
    {
        uint32* pDstNode = static_cast<uint32*>(
            Util::VoidPtrInc(pDstBuffer, dstInternalNodeDataOffset + RayTracingFloat32BoxNodeChild2Offset));
        *pDstNode = pSrcNode->child2;
    }

    if ((pSrcNode->child3 == InvalidIndex) || IsBoxNode(pSrcNode->child3))
    {
        uint32* pDstNode = static_cast<uint32*>(
            Util::VoidPtrInc(pDstBuffer, dstInternalNodeDataOffset + RayTracingFloat32BoxNodeChild3Offset));
        *pDstNode = pSrcNode->child3;
    }

    float3* pDstNodeBB0MinOffset = static_cast<float3*>(
        Util::VoidPtrInc(pDstBuffer, dstInternalNodeDataOffset + RayTracingFloat32BoxNodeBB0MinOffset));
    *pDstNodeBB0MinOffset = pSrcNode->bbox0_min;

    float3* pDstNodeBB0MaxOffset = static_cast<float3*>(
        Util::VoidPtrInc(pDstBuffer, dstInternalNodeDataOffset + RayTracingFloat32BoxNodeBB0MaxOffset));
    *pDstNodeBB0MaxOffset = pSrcNode->bbox0_max;

    float3* pDstNodeBB1MinOffset = static_cast<float3*>(
        Util::VoidPtrInc(pDstBuffer, dstInternalNodeDataOffset + RayTracingFloat32BoxNodeBB1MinOffset));
    *pDstNodeBB1MinOffset = pSrcNode->bbox1_min;

    float3* pDstNodeBB1MaxOffset = static_cast<float3*>(
        Util::VoidPtrInc(pDstBuffer, dstInternalNodeDataOffset + RayTracingFloat32BoxNodeBB1MaxOffset));
    *pDstNodeBB1MaxOffset = pSrcNode->bbox1_max;

    float3* pDstNodeBB2MinOffset = static_cast<float3*>(
        Util::VoidPtrInc(pDstBuffer, dstInternalNodeDataOffset + RayTracingFloat32BoxNodeBB2MinOffset));
    *pDstNodeBB2MinOffset = pSrcNode->bbox2_min;

    float3* pDstNodeBB2MaxOffset = static_cast<float3*>(
        Util::VoidPtrInc(pDstBuffer, dstInternalNodeDataOffset + RayTracingFloat32BoxNodeBB2MaxOffset));
    *pDstNodeBB2MaxOffset = pSrcNode->bbox2_max;

    float3* pDstNodeBB3MinOffset = static_cast<float3*>(
        Util::VoidPtrInc(pDstBuffer, dstInternalNodeDataOffset + RayTracingFloat32BoxNodeBB3MinOffset));
    *pDstNodeBB3MinOffset = pSrcNode->bbox3_min;

    float3* pDstNodeBB3MaxOffset = static_cast<float3*>(
        Util::VoidPtrInc(pDstBuffer, dstInternalNodeDataOffset + RayTracingFloat32BoxNodeBB3MaxOffset));
    *pDstNodeBB3MaxOffset = pSrcNode->bbox3_max;

    const NodePointer srcNodePointer = PackNodePointer(NODE_TYPE_BOX_FLOAT32, srcInternalNodeDataOffset - srcMetadataSizeInBytes);
    const NodePointer dstNodePointer = PackNodePointer(NODE_TYPE_BOX_FLOAT32, dstInternalNodeDataOffset - dstMetadataSizeInBytes);

    // Update parent pointer
    UpdateParentPointer(srcNodePointer, dstNodePointer);
}

// =====================================================================================================================
// BVH root is required to be a box node.  For builds consisting of only a single leaf node, insert an extra box node
// and hook it as the parent and new root.
NodePointer CpuBvhBuilder::AddBoxNodeForSingleLeaf(
    uint32 leafRoot)
{
    PAL_ASSERT(m_cpuConfig.buildNodeCount == 1);
    PAL_ASSERT(m_dstHeader.numInternalNodesFp32 == 0);
    PAL_ASSERT(m_dstHeader.numActivePrims <= 1);
    PAL_ASSERT(m_buildConfig.fp16BoxNodesInBlasMode == Fp16BoxNodesInBlasMode::NoNodes);

    NodePointer childPointers[4] = { leafRoot, InvalidNode, InvalidNode, InvalidNode };
    const Aabb leafBound = (m_dstHeader.numActivePrims == 1) ? m_pBuildNodes[m_pSortedPrims[0]].bound : NullBound;

    Aabb bounds[4] = { leafBound, NullBound, NullBound, NullBound };

    NodePointer rootPointer = AcquireInternalNode();

    WriteInternalBoxNode(rootPointer, childPointers, bounds);

    return rootPointer;
}

// =====================================================================================================================
// Write leaf node pointer in the sideband for a given global primitive index
void CpuBvhBuilder::WritePrimitiveNodePointer(
    uint32      globalPrimitiveIndex,
    NodePointer nodePointer)
{
    *ResultBuffer<uint32>(m_dstHeader.offsets.primNodePtrs + sizeof(uint32) * globalPrimitiveIndex) = nodePointer;
}

// =====================================================================================================================
// Read leaf node pointer from (result buffer) sideband for a given global primitive index
NodePointer CpuBvhBuilder::ReadPrimitiveNodePointer(
    uint32 globalPrimitiveIndex)
{
    return *ResultBuffer<uint32>(m_dstHeader.offsets.primNodePtrs + sizeof(uint32) * globalPrimitiveIndex);
}

// =====================================================================================================================
// Given a node pointer to a box node, returns the index within the interior node array of its box node structure.
uint32 CpuBvhBuilder::CalcInteriorNodeIndex(
    NodePointer nodePointer)
{
    PAL_ASSERT(nodePointer != InvalidNode);
    PAL_ASSERT(IsBoxNode(nodePointer));

    const uint32 nodeOffset = ExtractNodePointerOffset(nodePointer);

    PAL_ASSERT(nodeOffset >= m_dstHeader.offsets.internalNodes);

    uint32 nodeIndex = (nodeOffset - m_dstHeader.offsets.internalNodes) / m_cpuConfig.internalNodeSize;

    PAL_ASSERT(nodeIndex < m_scratchOffsets.maxQbvhInternalNodes);

    return nodeIndex;
}

// =====================================================================================================================
// Given a leaf node pointer, returns its primitive index within the build-time scratch primitive array
uint32 CpuBvhBuilder::CalcLeafNodeGlobalPrimitiveIndex(
    NodePointer nodePointer)
{
    PAL_ASSERT(nodePointer != InvalidNode);
    PAL_ASSERT(IsBoxNode(nodePointer) == false);

    uint32 globalPrimitiveIndex;

    const uint32 nodeOffset = ExtractNodePointerOffset(nodePointer);
    const uint32 nodeIndex  = (nodeOffset - m_dstHeader.offsets.leafNodes) / m_cpuConfig.leafNodeSize;

    if ((IsTriangleNode(nodePointer) == false) ||
        (m_buildConfig.triangleCompressionMode == TriangleCompressionMode::None))
    {
        // AABB leafs or non-compressed triangle leafs simply map as 1 primitive/leaf.
        PAL_ASSERT(nodeOffset >= m_dstHeader.offsets.leafNodes);

        globalPrimitiveIndex = nodeIndex;
    }
    else
    {
        // For compressed triangle nodes, we need to consult the reverse mapping that was created during
        // leaf node encoding (see WriteCompressedNodes() function).
        const uint32 nodeType = GetNodeType(nodePointer);

        PAL_ASSERT(m_pLeafsToPrims != nullptr);

        globalPrimitiveIndex = m_pLeafsToPrims[4 * nodeIndex + nodeType];
    }

    PAL_ASSERT(globalPrimitiveIndex < m_scratchOffsets.maxScratchPrimitives);

    return globalPrimitiveIndex;
}

// =====================================================================================================================
// For recursive SAH, determine related parameters
void CpuBvhBuilder::DetermineSAHParams()
{
    // TODO #gpurt: These should be moved into the config class if this algorithm turns out to be relevant
    constexpr uint32 SahMinPrimitives = 4;  // Min primitives before punting to insertion sort

    // Note: These costs are relative to the traversal cost
    constexpr float SahCostTraversal     = 1.0f; // Cost of traversing an interior node
    constexpr float SahCostInstanceIsect = 1.0f; // Cost of intersecting an instance leaf
    constexpr float SahCostTriangleIsect = 1.0f; // Cost of intersecting a triangle leaf
    constexpr float SahCostAabbIsect     = 1.0f; // Cost of intersecting an AABB

    // Number of SAH candidate buckets {default, fast-build, fast-trace}
    constexpr uint32 SahBucketCounts[3] = { 8, 5, 12 };

    // Figure out SAH costs
    m_cpuConfig.sahMinPrimitives = SahMinPrimitives;
    m_cpuConfig.sahCostTraversal = SahCostTraversal; // Further manipulated below

    float sahCostIntersect;

    if (m_buildArgs.inputs.type == AccelStructType::BottomLevel)
    {
        sahCostIntersect = (m_buildConfig.geometryType == GeometryType::Aabbs) ?
                           SahCostAabbIsect :
                           SahCostTriangleIsect;
    }
    else
    {
        sahCostIntersect = SahCostInstanceIsect;
    }

    if (Util::TestAnyFlagSet(m_buildArgs.inputs.flags, AccelStructBuildFlagPreferFastTrace))
    {
        m_cpuConfig.sahBucketCount = SahBucketCounts[2];
    }
    else if (Util::TestAnyFlagSet(m_buildArgs.inputs.flags, AccelStructBuildFlagPreferFastBuild))
    {
        m_cpuConfig.sahBucketCount = SahBucketCounts[1];
    }
    else
    {
        m_cpuConfig.sahBucketCount = SahBucketCounts[0];
    }

    // Make the traversal cost relative to the intersection cost so the latter can be factored
    // out of the SAH algorithm later.
    m_cpuConfig.sahCostTraversal /= sahCostIntersect;
}

// =====================================================================================================================
// Main function for building an acceleration structure from scratch, after leaf nodes have been parsed
void CpuBvhBuilder::BuildAccelerationStructure(
    uint32          primitiveCount,
    AccelStructType type,
    uint32          size)
{
    m_dstHeader.sizeInBytes         = size;
    m_dstHeader.geometryType        = static_cast<uint32>(m_buildConfig.geometryType);
    m_dstHeader.metadataSizeInBytes = m_metadataSizeInBytes;
    m_dstHeader.uuidLo              = Util::LowPart(m_deviceSettings.accelerationStructureUUID);
    m_dstHeader.uuidHi              = Util::HighPart(m_deviceSettings.accelerationStructureUUID);
    m_dstHeader.accelStructVersion  = GPURT_ACCEL_STRUCT_VERSION;

    // Recursively sort the primitives into a binary tree
    uint32 rootIndex = BuildTree(0, m_dstHeader.numActivePrims);

    // Encode the binary tree into a HW QBVH
    NodePointer rootPointer = EncodeTree(rootIndex);

    if (rootPointer != InvalidNode)
    {
        // The root node has to be a box node, so we need to insert one if one is missing.  We can get here
        // with single-leaf situations.
        if (IsBoxNode(rootPointer) == false)
        {
            rootPointer = AddBoxNodeForSingleLeaf(rootPointer);
        }

        WriteParentPointer(rootPointer, InvalidNode);

        PAL_ASSERT(IsBoxNode(rootPointer));
        PAL_ASSERT(ExtractNodePointerOffset(rootPointer) == sizeof(AccelStructHeader));
    }
}

// =====================================================================================================================
// Given new leaf node bounds, propagates them up an existing tree topology's interior nodes all the way to the root.
//
// Does not change the tree topology.
void CpuBvhBuilder::UpdateAccelerationStructure(
    uint32          primitiveCount,
    AccelStructType type,
    uint32          size)
{
    if (IsInPlace() == false)
    {
        // Copy metadata section for non-inplace updates (header written in the parent function at the end)
        memcpy(ResultMetaData(), SourceMetaData(), m_metadataSizeInBytes - sizeof(AccelStructMetadataHeader));

        // Copy primitive node pointer sideband
        memcpy(ResultBuffer<uint32>(m_dstHeader.offsets.primNodePtrs),
               SourceBuffer<uint32>(m_dstHeader.offsets.primNodePtrs),
               primitiveCount * sizeof(uint32));
    }

    // Start by populating the parent pointers of leaf primitives
    NodePointer* pSrc = m_pUpdateParentBuffers[0];
    NodePointer* pDst = m_pUpdateParentBuffers[1];
    uint32  srcCount  = 0;
    uint32  dstCount  = 0;

    // Initialize all box nodes as untouched
    for (uint32 i = 0; i < m_scratchOffsets.maxQbvhInternalNodes; ++i)
    {
        m_pUpdateNodes[i].touched = 0;
        m_pUpdateNodes[i].bound   = NullBound;
    }

    // TODO #gpurt: Parallel loop
    for (uint32 globalPrimitiveIndex = 0; globalPrimitiveIndex < primitiveCount; ++globalPrimitiveIndex)
    {
        NodePointer primNodePointer = ReadPrimitiveNodePointer(globalPrimitiveIndex);

        if (primNodePointer != InvalidNode)
        {
            NodePointer parentPointer = ReadParentPointer(primNodePointer);

            // Leaf nodes must have a parent
            PAL_ASSERT(parentPointer != InvalidNode);
            PAL_ASSERT(IsBoxNode(parentPointer));

            uint32 parentBoxIndex = CalcInteriorNodeIndex(parentPointer);

            // Add parent box to first iteration update list
            // TODO #gpurt: Atomics
            if (m_pUpdateNodes[parentBoxIndex].touched++ == 0)
            {
                // TODO #gpurt: Atomics
                pSrc[srcCount++] = parentPointer;
            }
        }
    }

    // Update box node bounds bottom-up until we reach the root
    while (srcCount > 0)
    {
        // TODO #gpurt: Parallel loop
        for (uint32 index = 0; index < srcCount; ++index)
        {
            // Fetch box node pointer
            const NodePointer nodePointer = pSrc[index];

            PAL_ASSERT(nodePointer != InvalidNode);
            PAL_ASSERT(IsBoxNode(nodePointer));

            // Get its matching scratch node state
            const uint32 nodeIndex = CalcInteriorNodeIndex(nodePointer);

            ScratchUpdateNode* pBoxState = &m_pUpdateNodes[CalcInteriorNodeIndex(nodePointer)];

            // Extract child pointers from the node
            NodePointer children[4];

            if (IsBoxNode32(nodePointer))
            {
                // Fetch box node children
                const Float32BoxNode* pBox = FetchFloat32BoxNode(SourceHeader(), nodePointer);

                children[0] = pBox->child0;
                children[1] = pBox->child1;
                children[2] = pBox->child2;
                children[3] = pBox->child3;
            }
            else
            {
                PAL_ASSERT(IsBoxNode16(nodePointer));

                // TODO #gpurt: fp16 box node
                PAL_NOT_IMPLEMENTED();

                children[0] = InvalidNode;
                children[1] = InvalidNode;
                children[2] = InvalidNode;
                children[3] = InvalidNode;
            }

            // Get the childrens' updated bounds and calculate the new union for this node
            Aabb bounds[4];

            for (uint32 i = 0; i < 4; ++i)
            {
                if (children[i] == InvalidNode)
                {
                    bounds[i] = NullBound;
                }
                else if (IsBoxNode(children[i]))
                {
                    uint32 childIndex = CalcInteriorNodeIndex(children[i]);

                    bounds[i] = m_pUpdateNodes[childIndex].bound;
                }
                else
                {
                    // TODO #gpurt: I don't think we can simply map from pointer to primitive for
                    // triangle-compressed nodes.  Probably need to include the leaf node type
                    // in the algorithm to figure out which triangle we're on.
                    PAL_ASSERT(m_buildConfig.triangleCompressionMode == TriangleCompressionMode::None);

                    uint32 childIndex = CalcLeafNodeGlobalPrimitiveIndex(children[i]);

                    bounds[i] = m_pPrimitives[childIndex].bound;
                }

                if (children[i] != InvalidNode)
                {
                    pBoxState->bound = BoundingBoxUnion(pBoxState->bound, bounds[i]);
                }
            }

            // Write out the box node with updated bounds (this acts as a copy for non-in-place updates)
            WriteInternalBoxNode(nodePointer, children, bounds);

            // Add this node's parent to the next iteration's list if it's not already present
            const NodePointer parentPointer = ReadParentPointer(nodePointer);

            if (parentPointer != InvalidNode)
            {
                const uint32 parentIndex = CalcInteriorNodeIndex(parentPointer);

                // TODO #gpurt: Atomics
                if (m_pUpdateNodes[parentIndex].touched++ == 0)
                {
                    // TODO #gpurt: Atomics
                    pDst[dstCount++] = parentPointer;
                }
            }
        }

        // Swap the buffers for the next iteration
        srcCount = dstCount;
        dstCount = 0;

        Util::Swap(pSrc, pDst);
    }
}

// =====================================================================================================================
// Writes a given node's parent pointer in that node's assigned offset in the parent metadata section.
void CpuBvhBuilder::WriteParentPointer(
    NodePointer nodePtr,
    NodePointer parentPtr)
{
    void* pResultMetadata = ResultMetadataHeader();

    uint32 parentOffset = m_metadataSizeInBytes - CalcParentPtrOffset(nodePtr,
                                                                      uint(m_buildConfig.triangleCompressionMode));

    PAL_ASSERT(parentOffset > 0 && parentOffset < m_metadataSizeInBytes);

    uint32* pParent = reinterpret_cast<uint32*>(Util::VoidPtrInc(pResultMetadata, parentOffset));

    *pParent = parentPtr;
}

//=====================================================================================================================
// Updates the child pointer in the parent of a dst node.
void CpuBvhBuilder::WriteParentNodeChildPointer(
    const void* pSrcBuffer,
    void*       pDstBuffer,
    NodePointer srcNodePointer,
    NodePointer dstNodePointer)
{
    // Update parent pointer
    NodePointer  parentNodePointer = UpdateParentPointer(srcNodePointer, dstNodePointer);
    const uint32 parentNodeOffset  = ExtractNodePointerOffset(parentNodePointer);

    const uint32* parentChildPointers =
        static_cast<const uint32*>(Util::VoidPtrInc(pSrcBuffer, parentNodeOffset + m_metadataSizeInBytes));

    for (uint childIndex = 0; childIndex < 4; childIndex++)
    {
        if (srcNodePointer == ExtractNodePointerCollapse(parentChildPointers[childIndex]))
        {
            // Increment primitive count to offset the decrement in PackLeafNodePointer
            const uint32 collapsePrimCount = ExtractPrimitiveCount(parentChildPointers[childIndex]) + 1;

            const uint32 dstPackedNodePointer = PackLeafNodePointer(
                GetNodeType(dstNodePointer),
                ExtractNodePointerOffset(dstNodePointer),
                collapsePrimCount);

            uint32 dstOffset = parentNodeOffset + m_metadataSizeInBytes + (childIndex * sizeof(uint));

            uint32* pDstNode = static_cast<uint32*>(Util::VoidPtrInc(pDstBuffer, dstOffset));
            *pDstNode = dstPackedNodePointer;

            break;
        }
    }
}

// =====================================================================================================================
// Reads a given node pointer's parent pointer (from the result buffer)
NodePointer CpuBvhBuilder::ReadParentPointer(
    NodePointer nodePtr)
{
    uint32 parentOffset = m_metadataSizeInBytes - CalcParentPtrOffset(nodePtr,
                                                                      uint(m_buildConfig.triangleCompressionMode));

    PAL_ASSERT(parentOffset > 0 && parentOffset < m_metadataSizeInBytes);

    const void* pSourceMetadata = m_buildArgs.pSrcAccelStructCpuAddr;
    const NodePointer* pParent  = reinterpret_cast<const NodePointer*>(Util::VoidPtrInc(pSourceMetadata, parentOffset));

    return *pParent;
}

// =====================================================================================================================
// Transforms a given node's childrens' bounds and returns the union of those bounding boxes.
static Aabb CalcBoundingBoxFromFloat32BoxNode(
    Float32BoxNode node,
    float4         inputTransform[3])
{
    PAL_ASSERT(node.child0 != InvalidNode);

    Aabb parent = MatrixTransformBoundingBox(inputTransform, node.bbox0_min, node.bbox0_max);

    if (node.child1 != InvalidNode)
    {
        parent = BoundingBoxUnion(parent, MatrixTransformBoundingBox(inputTransform, node.bbox1_min, node.bbox1_max));
    }

    if (node.child2 != InvalidNode)
    {
        parent = BoundingBoxUnion(parent, MatrixTransformBoundingBox(inputTransform, node.bbox2_min, node.bbox2_max));
    }

    if (node.child3 != InvalidNode)
    {
        parent = BoundingBoxUnion(parent, MatrixTransformBoundingBox(inputTransform, node.bbox3_min, node.bbox3_max));
    }

    return parent;
}

// =====================================================================================================================
// Calculates the transformed bounding box of a BLAS described by an instance desc
Aabb CpuBvhBuilder::ComputeInstancedBoundingBox(
    const InstanceDesc&      desc,
    const AccelStructHeader* pBlas)
{
    Aabb bbox;

    // TODO #gpurt: Transform fp16 box bounds
    // Note: root node is always fp32, other nodes can be mixed depending on fp16 box node mode

    // Fetch the root fp32 root node
    const uint32 rootNodePtr = PackNodePointer(NODE_TYPE_BOX_FLOAT32, sizeof(*pBlas));

    const Float32BoxNode& root = *FetchFloat32BoxNode(pBlas, rootNodePtr);

    float4 transform[3] = {
        float4(desc.transform[0][0], desc.transform[0][1], desc.transform[0][2], desc.transform[0][3]),
        float4(desc.transform[1][0], desc.transform[1][1], desc.transform[1][2], desc.transform[1][3]),
        float4(desc.transform[2][0], desc.transform[2][1], desc.transform[2][2], desc.transform[2][3])
    };

    bbox = CalcBoundingBoxFromFloat32BoxNode(root, transform);

    return bbox;
}

// =====================================================================================================================
// Encodes the input instance descriptions into ScratchPrimitives
void CpuBvhBuilder::EncodeInstances(
    const void*        pInstanceCpuAddr,
    uint32             numDesc,
    InputElementLayout descLayout)
{
    constexpr uint64 AccelStructAddressMask = ((1llu << 48) - 1);

    // TODO #gpurt: Parallelize
    for (uint32 instanceIndex = 0; instanceIndex < numDesc; ++instanceIndex)
    {
        ScratchPrimitive* pPrimitive = &m_pPrimitives[instanceIndex];

        const InstanceBottomLevelInfo blasInfo =
            m_clientCb.pfnConvertAccelStructBuildInstanceBottomLevel(m_buildArgs.inputs, instanceIndex);

        const Pal::gpusize blasGpuAddr = (blasInfo.desc.accelerationStructure & AccelStructAddressMask);

        const AccelStructHeader* pBlasHeader = nullptr;
        const AccelStructMetadataHeader*   pBlasMeta   = nullptr;

        bool activePrimitives = true;

        InstanceDesc instanceDesc = blasInfo.desc;

        // Trivially detect inactive instance references
        if (((blasGpuAddr & AccelStructAddressMask) == 0) ||
            (blasInfo.pCpuAddr == nullptr))
        {
            pPrimitive->bound = NullBound;

            activePrimitives = false;
        }
        else
        {
            // Offset into the BLAS and get its header
            pBlasMeta   = static_cast<const AccelStructMetadataHeader*>(blasInfo.pCpuAddr);
            pBlasHeader = static_cast<const AccelStructHeader*>(
                            Util::VoidPtrInc(blasInfo.pCpuAddr,
                                             pBlasMeta->sizeInBytes));

            // Patch the instance desc to offset directly into the BLAS header
            instanceDesc.accelerationStructure =
                PackInstanceBasePointer(blasInfo.desc.accelerationStructure + pBlasMeta->sizeInBytes,
                                        blasInfo.desc.flags,
                                        pBlasHeader->geometryType);

            // Transform BLAS bounds by instance transform
            pPrimitive->bound    = ComputeInstancedBoundingBox(instanceDesc, pBlasHeader);
            pPrimitive->centroid = BoundingBoxCentroid(pPrimitive->bound);

            // Mark BLAS with no active primitives as inactive to save time tracing
            if (pBlasHeader->numActivePrims == 0)
            {
                activePrimitives = false;
            }
        }

        const uint32 nodeOffset       = m_dstHeader.offsets.leafNodes + instanceIndex * m_cpuConfig.leafNodeSize;
        const NodePointer nodePointer = PackNodePointer(NODE_TYPE_USER_NODE_INSTANCE, nodeOffset);

        // Create node pointer to the instance leaf node
        pPrimitive->nodePointer = nodePointer;

        // Add this instance to the set of sorted primitives that produce the tree topology if
        // the instance has active primitives in the BLAS or if future updates can swap the BLAS
        // to an active one.
        if (activePrimitives ||
            Util::TestAnyFlagSet(m_buildArgs.inputs.flags, AccelStructBuildFlagAllowUpdate))
        {
            AddActivePrimitive(instanceIndex);
        }

        if (activePrimitives == false)
        {
            // Disable tracing from entering into the BLAS
            instanceDesc.instanceMask = 0;
        }

        // Write the instance leaf node into the destination buffer
        WriteInstanceLeafNode(nodePointer, instanceIndex, instanceDesc);

        WritePrimitiveNodePointer(instanceIndex, nodePointer);
    }

    m_dstHeader.numDescs      += numDesc;
    m_dstHeader.numLeafNodes  += numDesc;
    m_dstHeader.numPrimitives += numDesc;

    PAL_ASSERT(m_dstHeader.numLeafNodes  <= m_scratchOffsets.maxScratchPrimitives);
    PAL_ASSERT(m_dstHeader.numPrimitives <= m_scratchOffsets.maxScratchPrimitives);
}

// =====================================================================================================================
const CpuBvhBuilder::ScratchBuildNode& CpuBvhBuilder::GetBuildNode(
    uint32 nodeIndex) const
{
    PAL_ASSERT(nodeIndex != InvalidIndex);
    PAL_ASSERT(nodeIndex < m_cpuConfig.buildNodeCount);

    return m_pBuildNodes[nodeIndex];
}

// =====================================================================================================================
// Given a root node index in the scratch nodes array, pulls up a binary tree's children to produce up to four
// child indices.
uint32 CpuBvhBuilder::ConvertTwoToFourChildren(
    uint32 rootIndex,
    uint32 childIndices[4])
{
    PAL_ASSERT(rootIndex != InvalidIndex);

    const ScratchBuildNode* pNode = &GetBuildNode(rootIndex);

    PAL_ASSERT(pNode->primCount == 0); // Not a leaf

    uint32 childCount = 0;

    // Push the node's two children to start
    if (pNode->child[0] != InvalidIndex)
    {
        childIndices[childCount++] = pNode->child[0];
    }

    if (pNode->child[1] != InvalidIndex)
    {
        childIndices[childCount++] = pNode->child[1];
    }

    // Keep expanding nodes breadth-first in the list with their children until we either can't expand
    // any more or we're up to four children
    bool changed;

    do
    {
        changed = false;

        for (uint32 childIdx = 0; (childIdx < childCount); childIdx++)
        {
            PAL_ASSERT(childIndices[childIdx] != InvalidIndex);

            pNode = &GetBuildNode(childIndices[childIdx]);

            if (pNode->primCount == 0)
            {
                if (pNode->child[0] != InvalidIndex)
                {
                    // Check that we have room to add another node; if not, skip
                    if ((pNode->child[1] != InvalidIndex) && (childCount == 4))
                    {
                        continue;
                    }

                    // Replace existing position with its left child
                    childIndices[childIdx++] = pNode->child[0];

                    if (pNode->child[1] != InvalidIndex)
                    {
                        // Shift nodes forward to make room for right child
                        for (uint32 j = childCount; j > childIdx; --j)
                        {
                            childIndices[j] = childIndices[j - 1];
                        }

                        // Insert right child
                        childIndices[childIdx++] = pNode->child[1];
                        childCount++;
                    }

                    changed = true;
                }
                else if (pNode->child[1] != InvalidIndex)
                {
                    childIndices[childIdx] = pNode->child[1];

                    changed = true;
                }
                else
                {
                    // We shouldn't ever have a child-less interior node, but if we do then just keep it
                    PAL_NEVER_CALLED();
                }
            }
        }
    } while (changed);

    return childCount;
}

// =====================================================================================================================
// Given a tree of ScratchBuildNodes describing a tree topology, encode them into a HW-compatible acceleration structure
// based on current build config.
NodePointer CpuBvhBuilder::EncodeTree(
    uint32 nodeScratchIndex)
{
    NodePointer nodePointer;

    // Invalid node:
    if (nodeScratchIndex == InvalidIndex)
    {
        nodePointer = InvalidNode;
    }
    // Box node:
    else if (m_pBuildNodes[nodeScratchIndex].primCount == 0)
    {
        const ScratchBuildNode* pNode = &GetBuildNode(nodeScratchIndex);

        uint32 childIndices[4] = { InvalidIndex, InvalidIndex, InvalidIndex, InvalidIndex };

        // Convert from binary tree to QBVH node by expanding and pulling up children
        uint32 childCount = ConvertTwoToFourChildren(nodeScratchIndex, childIndices);

        nodePointer = AcquireInternalNode();

        NodePointer childPointers[4];
        Aabb childBounds[4];

        for (uint32 i = 0; i < 4; ++i)
        {
            childPointers[i] = EncodeTree(childIndices[i]);
            childBounds[i] = (childIndices[i] != InvalidIndex) ? GetBuildNode(childIndices[i]).bound : NullBound;
        }

        WriteInternalBoxNode(nodePointer, childPointers, childBounds);
    }
    // Leaf node:
    else
    {
        // TODO #gpurt: Collapse primitive ranges here by patching the leaf node pointers to contain
        // a prim count.  They should already be contiguous in memory (though not spatially contiguous.  Wonder
        // if it's better to sort morton-sort primitives first and then do recursive SAH splitting)
        PAL_ASSERT(m_deviceSettings.bvhCollapse == false);
        PAL_ASSERT(GetBuildNode(nodeScratchIndex).primCount == 1);

        const uint32 primIndex = m_pSortedPrims[GetBuildNode(nodeScratchIndex).firstPrimitive];
        const ScratchPrimitive& prim = m_pPrimitives[primIndex];

        nodePointer = prim.nodePointer;
    }

    return nodePointer;
}
// =====================================================================================================================
// Calculates the scratch requirements for a CPU build
uint32 CpuBvhBuilder::CalculateScratchBufferInfo(
    bool                    update,
    uint32                  aabbCount,
    TriangleCompressionMode triCompressionMode,
    ScratchOffsets*         pOffsets)
{
    ScratchOffsets offsets = {};

    offsets.maxScratchPrimitives = aabbCount;
    offsets.maxQbvhInternalNodes = CalcNumQBVHInternalNodes(aabbCount);

    if (update == false)
    {
        offsets.maxScratchBuildNodes = 2 * aabbCount - 1; // This is the build-time scratch binary tree nodes,
                                                          // not the final HW QBVH nodes
    }

    uint32 runningOffset = 0;

    runningOffset += sizeof(ScratchPrimitive) * offsets.maxScratchPrimitives;

    if (update == false)
    {
        offsets.scratchBuildNodesOffset = runningOffset;
        runningOffset += sizeof(ScratchBuildNode) * offsets.maxScratchBuildNodes;

        offsets.sortedPrimIndicesOffset = runningOffset;
        runningOffset += sizeof(uint32) * offsets.maxScratchPrimitives;
    }
    else
    {
        // Node update state
        offsets.scratchUpdateNodesOffset = runningOffset;
        runningOffset += sizeof(ScratchUpdateNode) * offsets.maxQbvhInternalNodes;

        // Need two buffers of box node pointers.  These buffers are used to walk bottom-up the BVH hierarchy
        // to rebuild node bounds during an update operation
        offsets.updateParentsOffsets[0] = runningOffset;
        runningOffset += sizeof(NodePointer) * offsets.maxQbvhInternalNodes;

        offsets.updateParentsOffsets[1] = runningOffset;
        runningOffset += sizeof(NodePointer) * offsets.maxQbvhInternalNodes;

        // Need a buffer that lets us map from triangle leaf node pointer -> global primitive index
        if (triCompressionMode != TriangleCompressionMode::None)
        {
            offsets.updateLeafsToPrimMappingOffset = runningOffset;
            runningOffset += sizeof(uint32) * 4 * offsets.maxScratchPrimitives;
        }
    }

    if (pOffsets != nullptr)
    {
        *pOffsets = offsets;
    }

    return runningOffset;
}

// =====================================================================================================================
// Calculates the scratch requirements for a CPU build
uint32 CpuBvhBuilder::CalculateScratchBufferInfo(
    const BuildConfig& buildConfig,
    ScratchOffsets*    pOffsets)
{
    return CalculateScratchBufferInfo(false,
                                      buildConfig.numLeafNodes,
                                      buildConfig.triangleCompressionMode,
                                      pOffsets);
}

// =====================================================================================================================
// Calculates the scratch requirements for a CPU update
uint32 CpuBvhBuilder::CalculateUpdateScratchBufferInfo(
    const BuildConfig&    buildConfig,
    ScratchOffsets*       pOffsets)
{
    return CalculateScratchBufferInfo(true,
                                      buildConfig.numLeafNodes,
                                      buildConfig.triangleCompressionMode,
                                      pOffsets);
}

// =====================================================================================================================
void CpuBvhBuilder::WriteGeometryInfo(
    uint32 geometryIndex,
    uint32 geometryFlags,
    uint32 numPrimitives,
    uint32 primNodePtrsOffset)
{
    // For builds and not-in-place updates, write the geometry info.
    if ((IsUpdate() == false) || IsInPlace())
    {
        GeometryInfo* pInfo = ResultBuffer<GeometryInfo>(m_dstHeader.offsets.geometryInfo +
                                                         geometryIndex * sizeof(GeometryInfo));

        *pInfo = GeometryInfo {
            geometryFlags,
            numPrimitives,
            static_cast<uint32>(primNodePtrsOffset * sizeof(uint32))
        };
    }
}

// =====================================================================================================================
void CpuBvhBuilder::WriteTriangleNode0(
    NodePointer nodePointer,
    float3      v0,
    float3      v1,
    float3      v2,
    uint32      primitiveIndex,
    uint32      geometryIndex,
    uint32      geometryFlags)
{
    TriangleNode triNode = {};

    uint32 triangleId = CalcUncompressedTriangleId(geometryFlags);

    triNode.v0         = v0;
    triNode.v1         = v1;
    triNode.v2         = v2;
    triNode.v4.x       = asfloat(PackGeometryIndexAndFlags(geometryIndex, geometryFlags));
    triNode.v4.y       = asfloat(primitiveIndex);
    triNode.v4.z       = asfloat(0); // Primitive index1
    triNode.triangleId = triangleId;

    *ResultBuffer<TriangleNode>(ExtractNodePointerOffset(nodePointer)) = triNode;
}

// =====================================================================================================================
// Writes a procedural (user) AABB leaf node
void CpuBvhBuilder::WriteProceduralNode(
    NodePointer nodePointer,
    const Aabb& bound,
    uint32      primitiveIndex,
    uint32      geometryIndex,
    uint32      geometryFlags)
{
    ProceduralNode node = {};

    node.bbox_min.x = bound.minX;
    node.bbox_min.y = bound.minY;
    node.bbox_min.z = bound.minZ;

    node.bbox_max.x = bound.maxX;
    node.bbox_max.y = bound.maxY;
    node.bbox_max.z = bound.maxZ;

    node.geometryIndexAndFlags = PackGeometryIndexAndFlags(geometryIndex, geometryFlags);
    node.primitiveIndex        = primitiveIndex;

    *ResultBuffer<ProceduralNode>(ExtractNodePointerOffset(nodePointer)) = node;
}

// =====================================================================================================================
// Parses input AABB geometry into ScratchPrimitives
void CpuBvhBuilder::EncodeAABBNodes(
    uint32               primitiveOffset,
    const GeometryAabbs* pDesc,
    uint32               primitiveCount,
    uint32               geometryIndex,
    GeometryFlags        geometryFlags)
{
    // TODO #gpurt: Probably (for all leaves), can encode all leaf data within this function (like GPU).  Benefit
    // is that it can cut down scratch primitive size and can simplify the tree-update logic
    WriteGeometryInfo(geometryIndex, geometryFlags, primitiveCount, primitiveOffset);

    // TODO #gpurt: Parallelize loop
    for (uint32 primitiveIndex = 0; primitiveIndex < primitiveCount; ++primitiveIndex)
    {
        // Fetch the input AABB
        Aabb aabb = *static_cast<const Aabb*>(Util::VoidPtrInc(pDesc->aabbAddr.pCpu,
                                                               size_t(primitiveIndex * pDesc->aabbByteStride)));

        const uint32 globalPrimitiveIndex  = primitiveOffset + primitiveIndex;
        const uint32 primNodePointerOffset = m_dstHeader.offsets.primNodePtrs +
                                             (globalPrimitiveIndex * sizeof(NodePointer));

        ScratchPrimitive* pPrimitive = &m_pPrimitives[globalPrimitiveIndex];

        const uint32 nodeOffset = m_dstHeader.offsets.leafNodes +
                                  globalPrimitiveIndex * m_cpuConfig.leafNodeSize;

        const NodePointer nodePointer = PackNodePointer(NODE_TYPE_USER_NODE_PROCEDURAL, nodeOffset);

        pPrimitive->bound    = aabb;
        pPrimitive->centroid = BoundingBoxCentroid(pPrimitive->bound);

        if (IsActive(aabb))
        {
            pPrimitive->nodePointer = nodePointer;

            WriteProceduralNode(nodePointer,
                                pPrimitive->bound,
                                primitiveIndex,
                                geometryIndex,
                                geometryFlags);

            AddActivePrimitive(globalPrimitiveIndex);
        }
        else
        {
            // Disable from inclusion in BVH sorting
            pPrimitive->nodePointer = InvalidNode;
        }

        // Write primitive node pointer
        WritePrimitiveNodePointer(globalPrimitiveIndex, pPrimitive->nodePointer);
    }

    m_dstHeader.numLeafNodes  += primitiveCount;
    m_dstHeader.numPrimitives += primitiveCount;

    PAL_ASSERT(m_dstHeader.numLeafNodes  <= m_scratchOffsets.maxScratchPrimitives);
    PAL_ASSERT(m_dstHeader.numPrimitives <= m_scratchOffsets.maxScratchPrimitives);
}

// =====================================================================================================================
// Parses input triangle geometry into ScratchPrimitives
void CpuBvhBuilder::EncodeTriangleNodes(
    uint32                   primitiveOffset,
    const GeometryTriangles* pDesc,
    uint32                   primitiveCount,
    uint32                   geometryIndex,
    GeometryFlags            geometryFlags)
{
    WriteGeometryInfo(geometryIndex, geometryFlags, primitiveCount, primitiveOffset);

    PAL_ASSERT(m_buildConfig.triangleCompressionMode == TriangleCompressionMode::None);

    const void* pGeometryBuffer = pDesc->vertexBufferAddr.pCpu;
    const void* pIndexBuffer = pDesc->indexBufferAddr.pCpu;
    const float4* pTransformBuffer = static_cast<const float4*>(pDesc->columnMajorTransform3x4.pCpu);

    // TODO #gpurt: Parallelize loop
    for (uint32 primitiveIndex = 0; primitiveIndex < primitiveCount; ++primitiveIndex)
    {
        // Fetch face indices from index buffer
        uint32 faceIndices[3];

        FetchFaceIndices(pDesc, primitiveIndex, faceIndices);

        const uint32 globalPrimitiveIndex = primitiveOffset + primitiveIndex;
        const uint32 primNodePointerOffset = m_dstHeader.offsets.primNodePtrs +
            (globalPrimitiveIndex * sizeof(NodePointer));

        ScratchPrimitive* pPrimitive = &m_pPrimitives[globalPrimitiveIndex];

        pPrimitive->nodePointer = InvalidNode;

        // Check if vertex indices are within bounds, otherwise make the triangle inactive
        if ((faceIndices[0] < pDesc->vertexCount) &&
            (faceIndices[1] < pDesc->vertexCount) &&
            (faceIndices[2] < pDesc->vertexCount))
        {
            // Fetch triangle vertex data from vertex buffer
            TriangleData tri = FetchTransformedTriangleData(pDesc, faceIndices);

            if (IsActive(tri))
            {
                const uint32 nodeOffset = m_dstHeader.offsets.leafNodes +
                    globalPrimitiveIndex * m_cpuConfig.leafNodeSize;

                pPrimitive->bound = GenerateBoundingBox(tri.v[0], tri.v[1], tri.v[2]);
                pPrimitive->nodePointer = PackNodePointer(NODE_TYPE_TRIANGLE_0, nodeOffset);
                pPrimitive->centroid = BoundingBoxCentroid(pPrimitive->bound);

                WriteTriangleNode0(pPrimitive->nodePointer,
                    tri.v[0],
                    tri.v[1],
                    tri.v[2],
                    primitiveIndex,
                    geometryIndex,
                    geometryFlags);
            }
        }

        // Update global scene bounds and active primitive count if the primitive is active
        if (pPrimitive->nodePointer != InvalidNode)
        {
            AddActivePrimitive(globalPrimitiveIndex);
        }
        else
        {
            pPrimitive->bound = NullBound;
        }

        // Write primitive node pointer
        WritePrimitiveNodePointer(globalPrimitiveIndex, pPrimitive->nodePointer);
    }

    m_dstHeader.numDescs = m_buildArgs.inputs.inputElemCount;
    m_dstHeader.numLeafNodes += primitiveCount;
    m_dstHeader.numPrimitives += primitiveCount;

    PAL_ASSERT(m_dstHeader.numLeafNodes <= m_scratchOffsets.maxScratchPrimitives);
    PAL_ASSERT(m_dstHeader.numPrimitives <= m_scratchOffsets.maxScratchPrimitives);
}

//=====================================================================================================================
// Returns true if vertexIndex0 and vertexIndex1 are equivalent for compression purposes.
bool CpuBvhBuilder::CompareVertices(
    uint32        vertexIndex0,
    const float3& vertexPosition0,
    uint32        vertexIndex1,
    const float3& vertexPosition1)
{
    bool match;

    // Vertices match if they have the same vertex index.
    if (vertexIndex0 == vertexIndex1)
    {
        match = true;
    }
    else
    {
        // If we decide to not rerun triangle compression on BVH Updates, we can't assume two vertices
        // with the same position but different vertex indices are the same.
        //
        // Also, speedup triangle compression when FAST_BUILD is used by not comparing vertices of different indices.
        constexpr uint32 DisableEquivalentPositionMask = AccelStructBuildFlagAllowUpdate |
                                                         AccelStructBuildFlagPerformUpdate |
                                                         AccelStructBuildFlagPreferFastBuild;

        if (Util::TestAnyFlagSet(m_buildArgs.inputs.flags, DisableEquivalentPositionMask))
        {
            match = false;
        }
        else if ((vertexIndex0 == InvalidIndex) ||
                 (vertexIndex1 == InvalidIndex))
        {
            match = true;
        }
        else if ((vertexPosition0.x == vertexPosition1.x) &&
                 (vertexPosition0.y == vertexPosition1.y) &&
                 (vertexPosition0.z == vertexPosition1.z))
        {
            // Compress two vertices that have different indices but the same positions together.
            match = true;
        }
        else
        {
            match = false;
        }
    }

    return match;
}

// =====================================================================================================================
// Given an array of ngons with max two/triangles per ngon, insert the given triangle into that array, either as
// a shared triangle of an existin ngon or the start of a new ngon.
void CpuBvhBuilder::FindBestNgonTwoTriangles(
    Ngon*               pNgonArray,     // Ngon array
    uint32*             pNgonCount,     // Pointer to current number of non-empty ngons within the array
                                        // (must also represent a free space within the array)
    uint32              numIndices,     // Max number of indices to consider per ngon
    uint32              primitiveIndex, // Primitive index of the triangle
    const uint32        ind[3],         // Face indices of the triangle
    const TriangleData& tri)            // Transformed triangle data
{
    bool foundAgon   = false;
    uint32 ngonIndex = *pNgonCount;
    uint bestNgon    = ngonIndex;

    // This loop is used to find the Ngon in the ngonArray that the current triangle compresses the best into.
    for (uint n = 0; n < ngonIndex; ++n)
    {
        // Ngon is fully occupied by two primitives
        if ((pNgonArray[n].f[0].primId != InvalidNode) &&
            (pNgonArray[n].f[1].primId != InvalidNode))
        {
            continue;
        }

        for (uint32 i = 0; i < numIndices; i++)
        {
            // Indices within ngon.inds[]
            const uint32 i0 = (i % numIndices);
            const uint32 i1 = ((i + 1) % numIndices);

            const uint32 index0 = pNgonArray[n].inds[i0];
            const uint32 index1 = pNgonArray[n].inds[i1];
            const float3& v0    = pNgonArray[n].v[i0];
            const float3& v1    = pNgonArray[n].v[i1];

            // Note: Only allow edge sharing permutations that do not result in a flipped winding order

            // Compare all three matching vertex conditions with triangle 0
            if ((CompareVertices(ind[2], tri.v[2], index0, v0)) &&
                (CompareVertices(ind[1], tri.v[1], index1, v1)))
            {
                // Place non shared vertex in the node's vertex slot 3
                pNgonArray[n].inds[3]   = ind[0];
                pNgonArray[n].f[1].vOff = 1;
                pNgonArray[n].v[3]      = tri.v[0];
            }
            else if ((CompareVertices(ind[1], tri.v[1], index0, v0)) &&
                     (CompareVertices(ind[0], tri.v[0], index1, v1)))
            {
                // Place non shared vertex in the node's vertex slot 3
                pNgonArray[n].inds[3]   = ind[2];
                pNgonArray[n].f[1].vOff = 2;
                pNgonArray[n].v[3]      = tri.v[2];
            }
            else if ((CompareVertices(ind[0], tri.v[0], index0, v0)) &&
                     (CompareVertices(ind[2], tri.v[2], index1, v1)))
            {
                // Place non shared vertex in the node's vertex slot 3
                pNgonArray[n].inds[3]   = ind[1];
                pNgonArray[n].f[1].vOff = 0;
                pNgonArray[n].v[3]      = tri.v[1];
            }
            else
            {
                continue;
            }

            // Temporary copy for rotating f[0] below
            uint32 tempIndices[3] = {
                pNgonArray[n].inds[0],
                pNgonArray[n].inds[1],
                pNgonArray[n].inds[2]
            };

            float3 tempPos[3] = {
                pNgonArray[n].v[0],
                pNgonArray[n].v[1],
                pNgonArray[n].v[2]
            };

            // Rotate triangle 0 so that the vertices shared with triangle 1
            // are in vertex slots 1 and 2 in the node.
            uint32 rotation[3] = {
                (i + 2) % numIndices,
                i % numIndices,
                (i + 1) % numIndices
            };

            pNgonArray[n].inds[0] = tempIndices[rotation[0]];
            pNgonArray[n].inds[1] = tempIndices[rotation[1]];
            pNgonArray[n].inds[2] = tempIndices[rotation[2]];

            pNgonArray[n].v[0] = tempPos[rotation[0]];
            pNgonArray[n].v[1] = tempPos[rotation[1]];
            pNgonArray[n].v[2] = tempPos[rotation[2]];

            if (i == 0)
            {
                // Rotate triangle 0 once
                pNgonArray[n].f[0].vOff = 1;
            }
            else if (i == 2)
            {
                // Rotate triangle 0 twice
                pNgonArray[n].f[0].vOff = 2;
            }

            pNgonArray[n].f[1].primId  = primitiveIndex;
            bestNgon                   = n;
            foundAgon                  = true;

            break;
        }
    }

    if (foundAgon == false)
    {
        // There wasn't a good Ngon, so allocate a new one and default initialize it.
        pNgonArray[ngonIndex].inds[0]     = ind[0];
        pNgonArray[ngonIndex].inds[1]     = ind[1];
        pNgonArray[ngonIndex].inds[2]     = ind[2];
        pNgonArray[ngonIndex].inds[3]     = InvalidIndex;
        pNgonArray[ngonIndex].f[0].primId = primitiveIndex;
        pNgonArray[ngonIndex].f[1].primId = InvalidIndex;
        pNgonArray[ngonIndex].f[0].vOff   = 0;

        pNgonArray[ngonIndex].v[0] = tri.v[0];
        pNgonArray[ngonIndex].v[1] = tri.v[1];
        pNgonArray[ngonIndex].v[2] = tri.v[2];

        ngonIndex++;

        *pNgonCount = ngonIndex;
    }
}

// =====================================================================================================================
void CpuBvhBuilder::AddActivePrimitive(
    uint32 globalPrimitiveIndex)
{
    PAL_ASSERT(m_pPrimitives[globalPrimitiveIndex].nodePointer != InvalidNode);

    // Increment the active primitive count
    uint32 activeCount = Util::AtomicIncrement(&m_dstHeader.numActivePrims);

    // If we're building, add this primitive index in an empty spot in the unsorted primitive index list
    if (m_pSortedPrims != nullptr)
    {
        m_pSortedPrims[activeCount - 1] = globalPrimitiveIndex;
    }
}
}

