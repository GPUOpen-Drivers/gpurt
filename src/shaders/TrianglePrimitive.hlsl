/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "BuildSettings.hlsli"

//=====================================================================================================================
// Get face indices from 16-bit index buffer
uint3 GetFaceIndices16(GpuVirtualAddress bufferVa, uint faceIndex, uint indexBufferByteOffset)
{
    // 3 vertices per triangle with 2-byte indices
    uint baseOffset = (faceIndex * 6) + indexBufferByteOffset;

    // Load address must be 4-byte aligned
    const bool unalignedRead = (baseOffset % 4 == 2);
    if (unalignedRead)
    {
        // Align down load address
        baseOffset -= 2;
    }

    const GpuVirtualAddress baseAddr = bufferVa + baseOffset;

    // Load index buffer data
    uint2 data;
    data.x = LoadDwordAtAddr(baseAddr);
    data.y = LoadDwordAtAddr(baseAddr + 0x4);

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
// Get face indices from 32-bit index buffer
uint3 GetFaceIndices32(GpuVirtualAddress bufferVa, uint faceIndex, uint indexBufferByteOffset)
{
    // 3 vertices per triangle with 4-byte indices
    const uint baseOffset = (faceIndex * 12) + indexBufferByteOffset;

    const GpuVirtualAddress baseAddr = bufferVa + baseOffset;

    uint3 faceIndices;
    faceIndices.x = LoadDwordAtAddr(baseAddr);
    faceIndices.y = LoadDwordAtAddr(baseAddr + 0x4);
    faceIndices.z = LoadDwordAtAddr(baseAddr + 0x8);

    return faceIndices;
}

//=====================================================================================================================
uint3 FetchFaceIndices(
    uint            primitiveIndex,
    IndexBufferInfo indexBufferInfo)
{
    const GpuVirtualAddress indexBufferVa = MakeGpuVirtualAddress(indexBufferInfo.gpuVaLo, indexBufferInfo.gpuVaHi);

    // Fetch face indices from index buffer
    uint3 faceIndices;
    if (indexBufferInfo.format == IndexFormatU16)
    {
        faceIndices = GetFaceIndices16(indexBufferVa, primitiveIndex, indexBufferInfo.byteOffset);
    }
    else if (indexBufferInfo.format == IndexFormatU32)
    {
        faceIndices = GetFaceIndices32(indexBufferVa, primitiveIndex, indexBufferInfo.byteOffset);
    }
    else
    {
        const uint startIndex = (primitiveIndex * 3);
        faceIndices.x = startIndex;
        faceIndices.y = startIndex + 1;
        faceIndices.z = startIndex + 2;
    }

    return faceIndices;
}

//=====================================================================================================================
// Vertex buffers only require an address and stride alignment of the format component size not the entire element size.
// If the input data is not naturally aligned, we cannot use a single typed fetch for the 2-3 components. In this case,
// we need to fetch each component separately.
float3 FetchVertexPerComponent(
    RWBuffer<float3> buffer,
    uint             firstComponentIndex,
    uint             numComponents)
{
    float3 vertex;
    vertex.x = buffer[firstComponentIndex+0].x;
    vertex.y = buffer[firstComponentIndex+1].x;
    if (numComponents > 2)
    {
        vertex.z = buffer[firstComponentIndex+2].x;
    }
    else
    {
        vertex.z = 0;
    }

    return vertex;
}

//=====================================================================================================================
TriangleData FetchTriangleData(
    RWBuffer<float3> buffer,
    uint             vertexOffsetInComponents,
    uint3            index,
    uint             strideInComponents,
    uint             numComponents)
{
    TriangleData tri;

    const uint3 i = index;

    // Indirect builds can set vertex buffer offset with component-size precision. We cannot check if offset aligns
    // with element size before dispatch, so we set SRD to allow perComponent fetches just in case.
    if (!Settings.isIndirectBuild && strideInComponents == 0)
    {
        tri.v0 = buffer[i.x];
        tri.v1 = buffer[i.y];
        tri.v2 = buffer[i.z];
    }
    else
    {
        const uint3 firstComponentIndex = vertexOffsetInComponents + i * strideInComponents;

        tri.v0 = FetchVertexPerComponent(buffer, firstComponentIndex.x, numComponents);
        tri.v1 = FetchVertexPerComponent(buffer, firstComponentIndex.y, numComponents);
        tri.v2 = FetchVertexPerComponent(buffer, firstComponentIndex.z, numComponents);
    }

    return tri;
}

//======================================================================================================================
uint CalcTriangleBoxNodeFlags(
    in uint geometryFlags)
{
    // Determine opacity from geometry flags
    uint nodeFlags =
        (geometryFlags & D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE) ? 1u << BOX_NODE_FLAGS_ONLY_OPAQUE_SHIFT :
                                                                  1u << BOX_NODE_FLAGS_ONLY_NON_OPAQUE_SHIFT;

    // Note, a bottom-level acceleration structure can only contain a single geometry type.
    nodeFlags |= 1u << BOX_NODE_FLAGS_ONLY_TRIANGLES_SHIFT;

    return nodeFlags;
}

//======================================================================================================================
TriangleData FetchTransformedTriangleData(
    in RWBuffer<float3>           geometryBuffer,
    in uint3                      faceIndices,
    in uint                       geometryStride,
    in uint                       vertexOffsetInComponents,
    in uint                       vertexComponentCount,
    in uint64_t                   transformBufferGpuVa,
    in uint                       transformOffsetInBytes)
{
    // Fetch triangle vertex data from vertex buffer
    TriangleData tri =
        FetchTriangleData(geometryBuffer, vertexOffsetInComponents, faceIndices, geometryStride, vertexComponentCount);

    // If this geometry has a valid transformation matrix. Transform each vertex using this matrix.
    if (transformBufferGpuVa != 0ull)
    {
        float4x4 transform;
        transform[0] = asfloat(LoadDwordAtAddrx4(transformBufferGpuVa + transformOffsetInBytes + 0));
        transform[1] = asfloat(LoadDwordAtAddrx4(transformBufferGpuVa + transformOffsetInBytes + 16));
        transform[2] = asfloat(LoadDwordAtAddrx4(transformBufferGpuVa + transformOffsetInBytes + 32));
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

//======================================================================================================================
bool IsActive(TriangleData tri)
{
    return ((isnan(tri.v0.x) == false) && (isnan(tri.v1.x) == false) && (isnan(tri.v2.x) == false));
}

//======================================================================================================================
TriangleData FetchTrianglePrimitive(
    in BuildShaderGeometryConstants geomConst,
    in uint geomId,
    in uint primId)
{
    // Fetch face indices from index buffer
    const IndexBufferInfo indexBufferInfo =
    {
        geomConst.indexBufferGpuVaLo,
        geomConst.indexBufferGpuVaHi,
        geomConst.indexBufferByteOffset,
        geomConst.indexBufferFormat,
    };

    // Fetch face indices from index buffer
    uint3 faceIndices = FetchFaceIndices(primId, indexBufferInfo);

    TriangleData tri = (TriangleData)0;

    // Check if vertex indices are within bounds, otherwise make the triangle inactive
    const uint maxIndex = max(faceIndices.x, max(faceIndices.y, faceIndices.z));
    if (maxIndex < geomConst.vertexCount)
    {
        const uint64_t transformBufferGpuVa =
            PackUint64(geomConst.transformBufferGpuVaLo, geomConst.transformBufferGpuVaHi);

        // Fetch triangle vertex data from vertex buffer
        tri = FetchTransformedTriangleData(GeometryBuffer[NonUniformResourceIndex(geomId)],
                                           faceIndices,
                                           geomConst.geometryStride,
                                           0,  // TODO: Indirect builds
                                           geomConst.vertexComponentCount,
                                           transformBufferGpuVa,
                                           0); // TODO: Indirect builds
    }

    return tri;
}
