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

#ifndef _KDOP_HLSL
#define _KDOP_HLSL

//=====================================================================================================================
// Generate a K-DOP vertex from an index between 0 and N * 2 * 6 + 2.
// Vertices are stored with antipodal pairs next to each other, even indices positive, odd negative.
static float3 GenerateKdopVertex(uint index)
{
    float s = index % 2 == 1 ? -1.0 : 1.0;
    return s * LutBuffer.obbKdopDirections[index / 2].xyz;
}

//=====================================================================================================================
// Generate triangle triplet from an index between 0 and 107 inclusive. F = N * N * 2 * 6 = 108.
// Triangle facets are aranged where antipodal pairs are next to one another, so
// even indices are positive, odd negative, Faces are ordered as X, Y, Z.
static uint3 GenerateKdopFacet(uint index)
{
    return LutBuffer.obbKdopFacets[index].xyz;
}

//=====================================================================================================================
static float2 ComputeTriangleKdopExtents(
    uint planeIdx,
    float3 v0,
    float3 v1,
    float3 v2)
{
    const float3 kdopDirection = GenerateKdopVertex(2 * planeIdx);
    const float dist0 = dot(kdopDirection, v0);
    const float dist1 = dot(kdopDirection, v1);
    const float dist2 = dot(kdopDirection, v2);
    const float minExtent = min(dist0, min(dist1, dist2));
    const float maxExtent = max(dist0, max(dist1, dist2));
    return float2(minExtent, maxExtent);
}

//=====================================================================================================================
// Find the K-DOP point index for a given normalized direction. Antipodal pair index is result +- 1.
static uint FindApexPoint(float3 direction)
{
    // Find major axis.
    const float maxComponent = max(max(abs(direction.x), abs(direction.y)), abs(direction.z));

    // Swizzle direction components with major axis in x channel to avoid dynamic indexing into
    // direction vector.
    float3 wuv;
    wuv.x = (maxComponent == abs(direction.y)) ? direction.y : direction.z;
    wuv.y = (maxComponent == abs(direction.y)) ? direction.z : direction.x;
    wuv.z = (maxComponent == abs(direction.y)) ? direction.x : direction.y;

    wuv.x = (maxComponent == abs(direction.x)) ? direction.x : wuv.x;
    wuv.y = (maxComponent == abs(direction.x)) ? direction.y : wuv.y;
    wuv.z = (maxComponent == abs(direction.x)) ? direction.z : wuv.z;

    // Compute major axis index for face index computation
    uint axis = 2;
    axis = (maxComponent == abs(direction.y)) ? 1 : axis;
    axis = (maxComponent == abs(direction.x)) ? 0 : axis;

    // Compute UV from direction vector
    float u = ((wuv.y / abs(wuv.x)) + 1.0) * 0.5;
    float v = ((wuv.z / abs(wuv.x)) + 1.0) * 0.5;

    // Also offset axis into antipodal faces.
    // Note: facets are organized in memory as:
    // X, Y, Z, with antipodal pairs next to each other.
    uint antipodalOffset = 0;

    if (sign(wuv.x) < 0)
    {
        antipodalOffset = 1;
        u = 1.0 - u;
        v = 1.0 - v;
    }

    // Scale UVs by tesselated cube size
    u = min(u * OBB_QUALITY, OBB_QUALITY - 0.001f);
    v = min(v * OBB_QUALITY, OBB_QUALITY - 0.001f);

    const uint uIndex = uint(floor(u));
    const uint vIndex = uint(floor(v));

    uint splitIndex = 0;
    if (frac(v) > frac(u))
    {
        splitIndex = 1;
    }

    const uint planeIndex = 2 * (uIndex + vIndex * OBB_QUALITY) + splitIndex;
    const uint apexPointIndex = (OBB_FACET_COUNT / 3) * axis + (2 * planeIndex) + antipodalOffset;

    return apexPointIndex;
}

//=====================================================================================================================
// Calculate the apex point from 3 vertices of the tesselated cube.
static float3 GetApexPoint(
    in float3x3 normal,
    in float3 kdopPlane)
{
    // Plane-Plane intersection of v0 and v1
    const float dotNormal01 = dot(normal[0], normal[1]);
    const float inverseDeterminant = 1.0f / (1.0f - dotNormal01 * dotNormal01);
    const float planeDot0 = kdopPlane[0];
    const float planeDot1 = kdopPlane[1];
    const float c0 = (planeDot0 - dotNormal01 * planeDot1) * inverseDeterminant;
    const float c1 = (planeDot1 - dotNormal01 * planeDot0) * inverseDeterminant;
    const float3 lineOrigin = c0 * normal[0] + c1 * normal[1];
    const float3 lineDir = cross(normal[0], normal[1]);

    // Line-Plane intersection of line and v2 plane
    const float planeDot2 = kdopPlane[2];
    const float dl2 = dot(lineDir, normal[2]);
    const float lineOriginDist = dot(normal[2], lineOrigin) - planeDot2;
    const float param = -lineOriginDist / dl2;

    // The apex point is where facet normals meet, we take the three
    // K-DOP planes at their vertices and compute their intersection point.
    // This is always non-colinear and thus always solvable:
    return lineOrigin + param * lineDir;
}

//=====================================================================================================================
// Compute the bounds of a given basis vector.
static float2 ComputeOBBFromKdopAddr(
    in uint64_t kdopAddr,
    float3  basis)
{
    const uint apexPointIndex = FindApexPoint(basis);
    const uint3 facet = GenerateKdopFacet(apexPointIndex);

    const float3x3 normal = {
        GenerateKdopVertex(facet.x),
        GenerateKdopVertex(facet.y),
        GenerateKdopVertex(facet.z)
    };

    float2 extentsX = Uint2ToFloat2(LoadDwordAtAddrx2(kdopAddr + ((facet.x / 2) * sizeof(float2))));
    if (facet.x % 2 == 1)
    {
        float mn = extentsX.x;
        extentsX.x = extentsX.y;
        extentsX.y = mn;
        extentsX *= -1.0;
    }
    float2 extentsY = Uint2ToFloat2(LoadDwordAtAddrx2(kdopAddr + ((facet.y / 2) * sizeof(float2))));
    if (facet.y % 2 == 1)
    {
        float mn = extentsY.x;
        extentsY.x = extentsY.y;
        extentsY.y = mn;
        extentsY *= -1.0;
    }
    float2 extentsZ = Uint2ToFloat2(LoadDwordAtAddrx2(kdopAddr + ((facet.z / 2) * sizeof(float2))));
    if (facet.z % 2 == 1)
    {
        float mn = extentsZ.x;
        extentsZ.x = extentsZ.y;
        extentsZ.y = mn;
        extentsZ *= -1.0;
    }

    // Max plane
    float3 kdopPlanes = float3(extentsX.y, extentsY.y, extentsZ.y);
    float3 xpApex = GetApexPoint(normal, kdopPlanes);

    // Min plane
    kdopPlanes = float3(extentsX.x, extentsY.x, extentsZ.x);
    float3 xnApex = GetApexPoint(-normal, kdopPlanes);

    return float2(dot(xpApex, basis), dot(xnApex, -basis));
}

//=====================================================================================================================
// Calculate the bounds of a given node based on the provided instance to world transform.
BoundingBox ComputeInstanceBoundsFromKdop(
    const uint64_t kdopAddress,
    float4 inputTransform[3])
{
    // decompose input matrix to get translation/scale/rotation.
    const float3 translation = float3(inputTransform[0].w, inputTransform[1].w, inputTransform[2].w);

    const float3x3 rotationScale = float3x3(inputTransform[0].xyz,
                                            inputTransform[1].xyz,
                                            inputTransform[2].xyz);

    const float3 scale = float3(length(rotationScale[0]),
                                length(rotationScale[1]),
                                length(rotationScale[2]));

    const float3x3 rotation = float3x3(rotationScale[0] * rcp(scale.x),
                                       rotationScale[1] * rcp(scale.y),
                                       rotationScale[2] * rcp(scale.z));

    const float2 boundsX = ComputeOBBFromKdopAddr(kdopAddress, rotation[0]);
    const float2 boundsY = ComputeOBBFromKdopAddr(kdopAddress, rotation[1]);
    const float2 boundsZ = ComputeOBBFromKdopAddr(kdopAddress, rotation[2]);
    const float3 p0 = float3(boundsX.x, boundsY.x, boundsZ.x);
    const float3 p1 = float3(boundsX.y, boundsY.y, boundsZ.y);

    BoundingBox bbox;
    bbox.min = min(p0, p1);
    bbox.max = max(p0, p1);

    // recompose transform
    const float3 center = (bbox.max + bbox.min) * 0.5;
    const float3 extents = (bbox.max - center);

    // adapted from TransformBoundingBox(), without abs() scale
    const float3 transformedCenter = (scale * center) + translation;
    const float3 transformedExtents = scale * extents;

    // add 6 ULP scaled to the longest extent of padding to extents for watertightness
    const float longestExtent = max(transformedExtents.x, max(transformedExtents.y, transformedExtents.z));
    const float3 epsilonPadding = 7.15254e-7 * float3(longestExtent, longestExtent, longestExtent);

    // convert back to min/max box representation
    bbox.min = transformedCenter - transformedExtents - epsilonPadding;
    bbox.max = transformedCenter + transformedExtents + epsilonPadding;

    return bbox;
}
#endif
