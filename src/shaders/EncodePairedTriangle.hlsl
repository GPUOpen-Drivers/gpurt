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
// Here is to define a piece shared memory to be used, and call EncodePairedTriangleNodeImpl
#define MAX_LDS_ELEMENTS (16 * BUILD_THREADGROUP_SIZE)
groupshared uint SharedMem[MAX_LDS_ELEMENTS];        // log the triangle Vertex, faces, and rotation info

#include "EncodePairedTriangleImpl.hlsl"

//======================================================================================================================
void EncodePairedTriangleNode(
    RWBuffer<float3>           GeometryBuffer,
    RWByteAddressBuffer        IndexBuffer,
    RWStructuredBuffer<float4> TransformBuffer,
    GeometryArgs               geometryArgs,
    uint                       primitiveIndex,
    uint                       localId,
    uint                       geometryBasePrimOffset,
    uint                       vertexOffset,
    bool                       writeNodesToUpdateStack)
{
    EncodePairedTriangleNodeImpl(
        GeometryBuffer,
        IndexBuffer,
        TransformBuffer,
        geometryArgs,
        primitiveIndex,
        localId,
        geometryBasePrimOffset,
        vertexOffset,
        writeNodesToUpdateStack);
}
