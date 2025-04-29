/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef TRAVERSAL_DEFS_HLSLI
#define TRAVERSAL_DEFS_HLSLI

#include "../../shared/assert.h"

#define ENCODE_FLAG_ARRAY_OF_POINTERS          0x00000001
#define ENCODE_FLAG_UPDATE_IN_PLACE            0x00000002
#define ENCODE_FLAG_REBRAID_ENABLED            0x00000004
#define ENCODE_FLAG_ENABLE_FUSED_INSTANCE_NODE 0x00000008

//=====================================================================================================================
struct IntersectionResult
{
    float  t;                     // Relative to tMin
    uint   nodeIndex;
    float2 barycentrics;
    uint   geometryIndex;
    uint   primitiveIndex;
    uint   instNodePtr;
    uint   hitkind;
    uint   instanceContribution;

#if DEVELOPER
    uint   numIterations;
    uint   maxStackDepth;
    uint   numRayBoxTest;
    uint   numCandidateHits;
    uint   numRayTriangleTest;
    uint   numAnyHitInvocation;
    uint   instanceIntersections;
#endif
};

//=====================================================================================================================
// Alternate encoding of state bits (TODO: See impact on generated ISA)
//
// 0: procedural
// 1: opaque
// 2: committed
// 3: no-duplicate anyhit
//
// 0: non_opaque_triangle
// 1: non_opaque_procedural
// 2: opaque_triangle
// 3: opaque_procedural
// 4: committed_non_opaque_triangle
// 5: committed_non_opaque_procedural
// 6: committed_opaque_triangle
// 7: committed_opaque_procedural
//
//=====================================================================================================================
// CANDIDATE_STATUS = (TRAVERSAL_STATE - 1)
#define TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_TRIANGLE 0
#define TRAVERSAL_STATE_CANDIDATE_PROCEDURAL_PRIMITIVE 1
#define TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_PROCEDURAL_PRIMITIVE 2
#define TRAVERSAL_STATE_CANDIDATE_NO_DUPLICATE_ANYHIT_PROCEDURAL_PRIMITIVE 3

// COMMITTED_STATUS = (TRAVERSAL_STATE - 4)
#define TRAVERSAL_STATE_COMMITTED_NOTHING 4
#define TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT 5
#define TRAVERSAL_STATE_COMMITTED_PROCEDURAL_PRIMITIVE_HIT 6

//=====================================================================================================================
// Data required for system value intrinsics
struct RaySystemData
{
    uint   currNodePtr;
    float  rayTCurrent;
    uint   instNodePtr;
    uint   instanceContribution;
    uint   geometryIndex;
    uint   primitiveIndex;
    float2 barycentrics;
    uint   frontFace;
    float3 origin;
    float3 direction;
};

//=====================================================================================================================
#if DEFINE_RAYDESC
// Ray description matching the D3D12 HLSL header
struct RayDesc
{
#ifdef __cplusplus
    RayDesc(uint val)
    {
        memset(this, val, sizeof(RayDesc));
    }

    RayDesc() : RayDesc(0)
    {}
#endif
    float3 Origin;
    float TMin;
    float3 Direction;
    float TMax;
};
#endif

//=====================================================================================================================
struct HitGroupInfo
{
#ifdef __cplusplus
    HitGroupInfo(uint val)
    {
        memset(this, val, sizeof(HitGroupInfo));
    }

    HitGroupInfo() : HitGroupInfo(0)
    {}
#endif
    uint2 closestHitId;
    uint2 anyHitId;
    uint2 intersectionId;
    uint  tableIndex;
};

#endif
