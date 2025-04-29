/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#define GC_DSTBUFFER
#define GC_DSTMETADATA
#define GC_SCRATCHBUFFER

#include "../../shadersClean/common/ShaderDefs.hlsli"

#include "../../shadersClean/common/Common.hlsli"
#include "../../shadersClean/build/BuildCommon.hlsli"
#include "../../shadersClean/build/BuildCommonScratch.hlsli"
#include "../EncodeCommon.hlsl"
#include "../EncodePairedTriangleImpl.hlsl"

#include "../../shadersClean/build/DGF/DGFDecompression.hlsl"

groupshared float3 DecodedVertices[BUILD_THREADGROUP_SIZE];

//=====================================================================================================================
void EncodeTriangle(
    BuildShaderGeometryConstants geomConstants,
    NumPrimAndInputOffset        inputOffsets,
    uint                         geometryIndex,
    uint                         primitiveIndex,
    TriangleData                 tri)
{
    // Generate triangle bounds and update scene bounding box
    BoundingBox boundingBox = GenerateTriangleBoundingBox(tri.v0, tri.v1, tri.v2);

#if GPURT_BUILD_RTIP3_1
    if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
    {
        if (IsActive(tri))
        {
            UpdateTriangleKdop(tri.v0, tri.v1, tri.v2);
        }
    }
#endif

    const uint metadataSize = IsUpdate() ?
        SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : ShaderConstants.header.metadataSizeInBytes;

    // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
    const AccelStructOffsets offsets =
        SrcBuffer.Load<AccelStructOffsets>(metadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);

    const uint basePrimNodePtr =
        IsUpdate() ? offsets.primNodePtrs : ShaderConstants.header.offsets.primNodePtrs;

    const uint flattenedPrimitiveIndex = inputOffsets.primitiveOffset + primitiveIndex;
    const uint primNodePointerOffset =
        metadataSize + basePrimNodePtr + (flattenedPrimitiveIndex * sizeof(uint));

    if (IsUpdate())
    {
        // TODO: DGF Updates
    }
    else
    {
        EncodeTriangleForBuild(geomConstants,
                               inputOffsets,
                               geometryIndex,
                               primitiveIndex,
                               flattenedPrimitiveIndex,
                               primNodePointerOffset,
                               tri,
                               boundingBox);
    }
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeDGF(
    in uint globalThreadId : SV_DispatchThreadID,
    in uint groupID : SV_GroupID,
    in uint localId : SV_GroupThreadID)
{
#if GPURT_BUILD_RTIP3_1
    if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
    {
        InitLocalKdop(localId, BUILD_THREADGROUP_SIZE);
    }
#endif

    const uint geometryIndex = ShaderRootConstants.GeometryIndex();
    const BuildShaderGeometryConstants geomConstants = GeometryConstants[geometryIndex];
    const NumPrimAndInputOffset inputOffsets = LoadInputOffsetsAndNumPrim(geometryIndex, true);

    if (globalThreadId == 0)
    {
        WriteGeometryInfo(geomConstants, inputOffsets, geometryIndex, DECODE_PRIMITIVE_STRIDE_TRIANGLE);
    }

    const uint blockId = groupID;
    const DGFBlockInfo dgfBlockInfo = DGFInit(DGFBuffer[geometryIndex], blockId);

    const uint triangleBase = dgfBlockInfo.header.primIDBase;

    const uint triangleIndex = min(localId, dgfBlockInfo.header.numTriangles - 1);
    const uint3 triangleIndices = DGFGetTriangle_BitScan_Wave(dgfBlockInfo, triangleIndex);

    const uint vertexIndex = min(localId, dgfBlockInfo.header.numVerts - 1);

    if (localId < dgfBlockInfo.header.numVerts)
    {
        DecodedVertices[vertexIndex] = DGFGetVertex(dgfBlockInfo, vertexIndex);
    }

    GroupMemoryBarrierWithGroupSync();

    if (localId < dgfBlockInfo.header.numTriangles)
    {
        TriangleData tri =  (TriangleData)0;
        tri.v0 = DecodedVertices[triangleIndices.x];
        tri.v1 = DecodedVertices[triangleIndices.y];
        tri.v2 = DecodedVertices[triangleIndices.z];

        const uint primitiveIndex = localId + triangleBase;

        if (primitiveIndex < inputOffsets.numPrimitives)
        {
            EncodeTriangle(geomConstants,
                           inputOffsets,
                           geometryIndex,
                           primitiveIndex,
                           tri);
        }

        IncrementPrimitiveTaskCounters(primitiveIndex,
                                       inputOffsets.numPrimitives,
                                       geomConstants.numPrimitives);

#if GPURT_BUILD_RTIP3_1
        if (Settings.tlasRefittingMode != TlasRefittingMode::Disabled)
        {
            GroupMemoryBarrierWithGroupSync();
            MergeLocalKdop(localId, BUILD_THREADGROUP_SIZE);
        }
#endif
    }
}
