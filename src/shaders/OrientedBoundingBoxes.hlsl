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
#ifndef _ORIENTED_BOUNDING_BOXES_HLSL
#define _ORIENTED_BOUNDING_BOXES_HLSL

#include "../shadersClean/common/ObbCommon.hlsli"
#include "KDOP.hlsl"

#define TWO_PIS 6.283185306

//=====================================================================================================================
bool IsObbEnabled()
{
    return (Settings.enableOrientedBoundingBoxes & bit(Settings.topLevelBuild ? TOP_LEVEL : BOTTOM_LEVEL)) != 0;
}

//=====================================================================================================================
// Map an index between 0-12 inclusive to an oriented bounding box axis.
float3 OBBAxisIndexToFloat3(uint index)
{
    return LutBuffer.obbAxesLut[index].xyz;
}

//=====================================================================================================================
// Convert an orthonormal rotation matrix to an OBB index using a LUT
static uint MatrixToOBBIndex(in float3x3 R)
{
    float3 axis = normalize(float3(R[2].y - R[1].z,
                                   R[0].z - R[2].x,
                                   R[1].x - R[0].y));
    const float diagonal = R[0].x + R[1].y + R[2].z;
    float angle = acos(0.5f * (diagonal - 1.0f));

    if (axis.z < 0.0f)
    {
        axis  *= -1.0f;
        angle *= -1.0f;
    }

    if (angle < 0.0f)
    {
        angle += TWO_PIS;
    }

    // axis.xy should be in the range [-1.0, 1.0] inclusive
    // angle should be in the range [0, TWO_PIS] inclusive

    const int  xIndex   = min(15, (int)((axis.x + 1.0f) / 0.125f));
    const int  yIndex   = min(15, (int)((axis.y + 1.0f) / 0.125f));
    const int  angIndex = min(15, (int)(angle / 0.392699081625f));
    const uint index    = (xIndex << 6) | (yIndex << 2) | (angIndex >> 2);
    return LutBuffer.obbMatrixIndexLUT[index][angIndex & 0x03];
}

//=====================================================================================================================
float ScoreMatrixOrientation(float3x3 A, float3x3 B, int i, int j, int k)
{
    return abs(dot(A[0], B[i])) + abs(dot(A[1], B[j])) + abs(dot(A[2], B[k]));
}

//=====================================================================================================================
// Julien Beasley's version
float MatrixDistAbsDot(float3x3 A, float3x3 B)
{
    float3 a = float3(ScoreMatrixOrientation(A, B, 0, 1, 2),
                      ScoreMatrixOrientation(A, B, 0, 2, 1),
                      ScoreMatrixOrientation(A, B, 1, 0, 2));

    float3 b = float3(ScoreMatrixOrientation(A, B, 1, 2, 0),
                      ScoreMatrixOrientation(A, B, 2, 0, 1),
                      ScoreMatrixOrientation(A, B, 2, 1, 0));

    float3 c = max(a, b);

    return max(c.x, max(c.y, c.z));
}

//=====================================================================================================================
// Convert an orthonormal rotation matrix to an OBB index.
// Similar to MatrixToOBBIndex, but it tests all possible row permutations of mat
// and selects the best one by looking at how well the basis vectors match between
// the candidate result matrix and M.
// Each row permutation of M can potentially yield a different result from MatrixToOBBIndex(M).
uint MatrixToBestOBBIndex(in float3x3 inMat)
{
    // This code assumes that inMat is not symmetric. If inMat can be guaranteed to always have
    // a positive determinant, then we can remove the following check:
    if (determinant(inMat) < 0.0)
    {
        inMat[2] = -inMat[2];
    }

    uint bestIndex = INVALID_OBB;
    float bestDist = -FLT_MAX;

    // Hand unrolling the 3 matrix permutations that do not invert the determinant of inMat
    {
        const float3x3 perm = inMat;
        float3x3 matPerm = float3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
        const uint index = MatrixToOBBIndex(perm);
        if (index != INVALID_OBB)
        {
            matPerm = OBBIndexToMatrix(index);
        }
        const float dist = MatrixDistAbsDot(inMat, matPerm);
        if (dist > bestDist)
        {
            bestDist  = dist;
            bestIndex = index;
        }
    }
    {
        const float3x3 perm = float3x3(inMat[1], inMat[2], inMat[0]);
        float3x3 matPerm = float3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
        const uint index = MatrixToOBBIndex(perm);
        if (index != INVALID_OBB)
        {
            matPerm = OBBIndexToMatrix(index);
        }
        const float dist = MatrixDistAbsDot(inMat, matPerm);
        if (dist > bestDist)
        {
            bestDist  = dist;
            bestIndex = index;
        }
    }
    {
        const float3x3 perm = float3x3(inMat[2], inMat[0], inMat[1]);
        float3x3 matPerm = float3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
        const uint index = MatrixToOBBIndex(perm);
        if (index != INVALID_OBB)
        {
            matPerm = OBBIndexToMatrix(index);
        }
        const float dist = MatrixDistAbsDot(inMat, matPerm);
        if (dist > bestDist)
        {
            bestDist  = dist;
            bestIndex = index;
        }
    }

    return bestIndex;
}

//=====================================================================================================================
// Compute the bounds of a given basis vector.
static float2 ComputeOBBBounds(
    in uint obbKdopOffset,
    float3  basis)
{
    const uint apexPointIndex = FindApexPoint(basis);
    const uint3 facet = GenerateKdopFacet(apexPointIndex);

    const float3x3 normal = {
        GenerateKdopVertex(facet.x),
        GenerateKdopVertex(facet.y),
        GenerateKdopVertex(facet.z)
    };

    float2 extentsX = ScratchGlobal.Load<float2>(obbKdopOffset + ((facet.x / 2) * sizeof(float2)));
    if (facet.x % 2 == 1)
    {
        float mn = extentsX.x;
        extentsX.x = extentsX.y;
        extentsX.y = mn;
        extentsX *= -1.0;
    }
    float2 extentsY = ScratchGlobal.Load<float2>(obbKdopOffset + ((facet.y / 2) * sizeof(float2)));
    if (facet.y % 2 == 1)
    {
        float mn = extentsY.x;
        extentsY.x = extentsY.y;
        extentsY.y = mn;
        extentsY *= -1.0;
    }
    float2 extentsZ = ScratchGlobal.Load<float2>(obbKdopOffset + ((facet.z / 2) * sizeof(float2)));
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
// Compute an OBB for a single triangle specified by 3 vertices
static BoundingBox ComputeOBBFromTri(
    in float3 v0,
    in float3 v1,
    in float3 v2,
    in float3x3 obbMatrix)
{
    BoundingBox obb;

    // Get 3 closest directions to each OBB extent.
    const float distX0 = dot(obbMatrix[0], v0);
    const float distX1 = dot(obbMatrix[0], v1);
    const float distX2 = dot(obbMatrix[0], v2);
    const float minX = min(distX0, min(distX1, distX2));
    const float maxX = max(distX0, max(distX1, distX2));

    const float distY0 = dot(obbMatrix[1], v0);
    const float distY1 = dot(obbMatrix[1], v1);
    const float distY2 = dot(obbMatrix[1], v2);
    const float minY = min(distY0, min(distY1, distY2));
    const float maxY = max(distY0, max(distY1, distY2));

    const float distZ0 = dot(obbMatrix[2], v0);
    const float distZ1 = dot(obbMatrix[2], v1);
    const float distZ2 = dot(obbMatrix[2], v2);
    const float minZ = min(distZ0, min(distZ1, distZ2));
    const float maxZ = max(distZ0, max(distZ1, distZ2));

    const float3 p0 = float3(minX, minY, minZ);
    const float3 p1 = float3(maxX, maxY, maxZ);

    obb.min = min(p0, p1);
    obb.max = max(p0, p1);

    return obb;
}

//=====================================================================================================================
// This is the very first step of DiTO, where the large base triangle is selected.
// Only in this case, the large base triangle is the primitive itself and only one edge
// is considered for the orientation.
inline int FetchTriKdopObbMatrixIndex(
    in  float3 v0,
    in  float3 v1,
    in  float3 v2)
{
    const float3   e0 = normalize(v1 - v0);
    const float3   e1 = normalize(v2 - v1);
    const float3   N  = normalize(cross(e0, e1));
    const float3   e  = cross(e0, N);
    const float3x3 R  = { e0, N, e };

    return MatrixToBestOBBIndex(R);
}

#endif
