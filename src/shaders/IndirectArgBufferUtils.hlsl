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

#include "../shadersClean/common/ShaderDefs.hlsli"
#include "BuildSettings.hlsli"

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
        const IndirectBuildRangeInfo buildRangeInfo =
            IndirectArgBuffer.Load<IndirectBuildRangeInfo>(0);

        result.numPrimitives = buildRangeInfo.primitiveCount;
        result.primitiveOffset = buildRangeInfo.primitiveOffset;
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
    in uint geometryIndex)
{
    uint primitiveOffset = 0;

    for (uint geomIdx = 0; geomIdx < geometryIndex; ++geomIdx)
    {
        const IndirectBuildRangeInfo buildRangeInfo =
            IndirectArgBuffer.Load<IndirectBuildRangeInfo>(ShaderConstants.indirectArgBufferStride * geomIdx);

        primitiveOffset += buildRangeInfo.primitiveCount;
    }

    return primitiveOffset;
}

//======================================================================================================================
// Compute data from IndirectArgBuffer when needed by BLAS indirect build, use sane defaults for direct path otherwise.
NumPrimAndInputOffset GetInputOffsetsAndNumPrim(
    in BuildShaderGeometryConstants geomConstants,
    in uint                         geometryIndex,
    in IndirectBuildRangeInfo       buildRangeInfo) // Sourced from Indirect Buffers
{
    NumPrimAndInputOffset result = (NumPrimAndInputOffset) 0;

    if (Settings.isIndirectBuild)
    {
        const uint firstVertexInComponents = buildRangeInfo.firstVertex * geomConstants.vertexComponentCount;

        result.numPrimitives = buildRangeInfo.primitiveCount;
        result.primitiveOffset = ComputePrimitiveOffset(geometryIndex);
        if (Settings.geometryType == PrimitiveType::Triangle)
        {
            if (geomConstants.indexBufferFormat != IndexFormatInvalid)
            {
                result.vertexOffsetInComponents = firstVertexInComponents;
                result.indexOffsetInBytes = buildRangeInfo.primitiveOffset;
            }
            else
            {
                const uint primitiveOffsetInComponents =
                    buildRangeInfo.primitiveOffset / geomConstants.vertexComponentSize;

                result.vertexOffsetInComponents = primitiveOffsetInComponents + firstVertexInComponents;
                result.indexOffsetInBytes = 0;
            }
            result.transformOffsetInBytes = buildRangeInfo.transformOffset;
        }
        else if (Settings.geometryType == PrimitiveType::AABB)
        {
            result.vertexOffsetInComponents = buildRangeInfo.primitiveOffset / sizeof(float);

            result.indexOffsetInBytes = 0;
            result.transformOffsetInBytes = 0;
        }
    }
    else
    {
        result.numPrimitives = geomConstants.numPrimitives;
        result.primitiveOffset = geomConstants.primitiveOffset;
        result.vertexOffsetInComponents = 0;
        result.indexOffsetInBytes = 0;
        result.transformOffsetInBytes = 0;
    }

    return result;
}

//======================================================================================================================
// Use to fetch data from IndirectArgBuffer.
NumPrimAndInputOffset LoadInputOffsetsAndNumPrim(
    in uint geometryIndex,
    in bool isUniform)       // Whether geometryIndex is uniform across all threads.
{
    IndirectBuildRangeInfo buildRangeInfo;
    if (Settings.isIndirectBuild)
    {
        buildRangeInfo =
            IndirectArgBuffer.Load<IndirectBuildRangeInfo>(ShaderConstants.indirectArgBufferStride * geometryIndex);
    }
    else
    {
        buildRangeInfo = (IndirectBuildRangeInfo)0;
    }

    BuildShaderGeometryConstants geomConstants = (BuildShaderGeometryConstants)0;
    if (isUniform)
    {
        geomConstants = GeometryConstants[geometryIndex];
    }
    else
    {
        geomConstants = GeometryConstants[NonUniformResourceIndex(geometryIndex)];
    }

    return GetInputOffsetsAndNumPrim(geomConstants, geometryIndex, buildRangeInfo);
}

#endif
