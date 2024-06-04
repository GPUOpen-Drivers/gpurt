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
//
// Required bindings:
// - RWByteAddressBuffer IndirectArgBuffer
// - RWByteAddressBuffer SrcBuffer
// - RWByteAddressBuffer DstMetadata

#ifndef _INDIRECTARGBUFFER_HLSL
#define _INDIRECTARGBUFFER_HLSL

#include "../shared/rayTracingDefs.h"
#include "BuildSettings.hlsli"

static bool IsUpdate();

//======================================================================================================================
struct NumPrimAndOffset
{
    uint numPrimitives;
    uint primitiveOffset;
};

//======================================================================================================================
// Similar to NumPrimAndOffset. Has additional offsets to vertex, index and transform buffers.
struct NumPrimAndInputOffset
{
    uint numPrimitives;
    uint primitiveOffset;
    uint vertexOffsetInComponents;
    uint indexOffsetInBytes;
    uint transformOffsetInBytes;
};

//======================================================================================================================
// Fetch data from IndirectArgBuffer when needed by TLAS indirect build, use sane defaults for direct path otherwise.
NumPrimAndOffset LoadNumPrimAndOffset()
{
    NumPrimAndOffset result = (NumPrimAndOffset) 0;

    if (Settings.isIndirectBuild)
    {
        const IndirectBuildOffset buildOffsetInfo =
            IndirectArgBuffer.Load<IndirectBuildOffset>(0);

        result.numPrimitives = buildOffsetInfo.primitiveCount;
        result.primitiveOffset = buildOffsetInfo.primitiveOffset;
    }
    else
    {
        result.numPrimitives = ShaderConstants.numPrimitives;
        result.primitiveOffset = 0;
    }

    return result;
}

//=====================================================================================================================
// Used by indirect build
uint ComputePrimitiveOffset(
    GeometryArgs geometryArgs)
{
    uint primitiveOffset = 0;

    const uint metadataSize = IsUpdate() ?
                              SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : geometryArgs.metadataSizeInBytes;

    // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
    const AccelStructDataOffsets offsets =
        SrcBuffer.Load<AccelStructDataOffsets> (metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

    const uint baseGeometryInfoOffset = IsUpdate() ? offsets.geometryInfo : geometryArgs.BaseGeometryInfoOffset;

    for (uint geomIdx = 0; geomIdx < geometryArgs.GeometryIndex; ++geomIdx)
    {
        const uint geometryInfoOffset =
            metadataSize + baseGeometryInfoOffset +
            (geomIdx * GEOMETRY_INFO_SIZE);

        GeometryInfo info;
        info = DstMetadata.Load<GeometryInfo>(geometryInfoOffset);
        uint primitiveCount = ExtractGeometryInfoNumPrimitives(info.geometryFlagsAndNumPrimitives);

        primitiveOffset += primitiveCount;
    }

    return primitiveOffset;
}

//======================================================================================================================
// Fetch data from IndirectArgBuffer when needed by BLAS indirect build, use sane defaults for direct path otherwise.
NumPrimAndInputOffset LoadInputOffsetsAndNumPrim(GeometryArgs args)
{
    NumPrimAndInputOffset result = (NumPrimAndInputOffset) 0;

    if (Settings.isIndirectBuild)
    {
        // Sourced from Indirect Buffers
        const IndirectBuildOffset buildOffsetInfo =
            IndirectArgBuffer.Load<IndirectBuildOffset>(ShaderConstants.indirectArgBufferStride * args.GeometryIndex);
        const uint firstVertexInComponents = buildOffsetInfo.firstVertex * args.VertexComponentCount;

        result.numPrimitives = buildOffsetInfo.primitiveCount;
        result.primitiveOffset = ComputePrimitiveOffset(args);
        if (Settings.geometryType == PrimitiveType::Triangle)
        {
            if (args.IndexBufferFormat != IndexFormatInvalid)
            {
                result.vertexOffsetInComponents = firstVertexInComponents;
                result.indexOffsetInBytes = buildOffsetInfo.primitiveOffset;
            }
            else
            {
                const uint primitiveOffsetInComponents = buildOffsetInfo.primitiveOffset / args.VertexComponentSize;

                result.vertexOffsetInComponents = primitiveOffsetInComponents + firstVertexInComponents;
                result.indexOffsetInBytes = 0;
            }
            result.transformOffsetInBytes = buildOffsetInfo.transformOffset;
        }
        else if (Settings.geometryType == PrimitiveType::AABB)
        {
            result.vertexOffsetInComponents = buildOffsetInfo.primitiveOffset / sizeof(float);

            result.indexOffsetInBytes = 0;
            result.transformOffsetInBytes = 0;
        }
    }
    else
    {
        result.numPrimitives = args.NumPrimitives;
        result.primitiveOffset = args.PrimitiveOffset;
        result.vertexOffsetInComponents = 0;
        result.indexOffsetInBytes = 0;
        result.transformOffsetInBytes = 0;
    }

    return result;
}

#endif
