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
#ifndef TRAVERSAL_DEFS_HLSLI
#define TRAVERSAL_DEFS_HLSLI

#include "../common/TempAssert.hlsli"

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
// Commit status
typedef uint COMMITTED_STATUS;

#define COMMITTED_NOTHING 0
#define COMMITTED_TRIANGLE_HIT 1
#define COMMITTED_PROCEDURAL_PRIMITIVE_HIT 2

//=====================================================================================================================
// Candidate type
typedef uint CANDIDATE_STATUS;

#define CANDIDATE_NON_OPAQUE_TRIANGLE 0
#define CANDIDATE_PROCEDURAL_PRIMITIVE 1
#define CANDIDATE_NON_OPAQUE_PROCEDURAL_PRIMITIVE 2
#define CANDIDATE_EARLY_RAY_TERMINATE 4

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
    float3 Origin;
    float TMin;
    float3 Direction;
    float TMax;
};
#endif

//=====================================================================================================================
// Internal RayQuery structure initialised at TraceRaysInline()
struct RayQueryInternal
{
    // Internal query data holding address of current BVH and stack information.
    // Additional data that may be required will be stored here.
    uint             bvhLo;
    uint             bvhHi;
    uint             topLevelBvhLo;
    uint             topLevelBvhHi;
    uint             stackPtr;
    uint             stackPtrTop;
    uint             stackNumEntries;
    uint             instNodePtr;
    uint             currNodePtr;
    uint             instanceHitContributionAndFlags;
    uint             prevNodePtr;
    uint             isGoingDown;
    uint             lastInstanceNode;

    RayDesc          rayDesc;
    float            rayTMin;
    uint             rayFlags;
    uint             instanceInclusionMask;

    // Candidate system data
    CANDIDATE_STATUS candidateType;
    RaySystemData    candidate;

    // Committed system data
    COMMITTED_STATUS committedStatus;
    RaySystemData    committed;

    uint             reserved;

    // Counter data
    // @note We don't wrap these in DEVELOPER because it would result in mismatch of RayQuery struct size
    //       on the driver side when we're not using counters.
    uint             numRayBoxTest;
    uint             numRayTriangleTest;
    uint             numIterations;
    uint             maxStackDepthAndDynamicId;
    uint             clocks;
    uint             numCandidateHits;
    uint             instanceIntersections;
    uint             rayQueryObjId;
};

//=====================================================================================================================
struct HitGroupInfo
{
    uint2 closestHitId;
    uint2 anyHitId;
    uint2 intersectionId;
    uint  tableIndex;
};

#endif
