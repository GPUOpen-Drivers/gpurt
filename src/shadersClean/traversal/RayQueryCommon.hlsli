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
//
#ifndef _RAYQUERY_COMMON_HLSLI_
#define _RAYQUERY_COMMON_HLSLI_

#include "RtIp.hlsli"
#include "TraversalDefs.hlsli"
#include "../common/Common.hlsli"

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

#if GPURT_BUILD_RTIP3
    uint             currNodePtr2; // Second node pointer for dual traversal
#else
    uint             reserved;
#endif

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

#endif
