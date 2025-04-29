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

#include "rtip3_ops.hlsl"

#if GPURT_BUILD_RTIP3_1
#include "rtip3_1.hlsli"
#endif

//======================================================================================================================
static void TraceRayInlineImpl3_1(
    inout_param(RayQueryInternal) rayQuery,
    in    uint                    accelStructLo,
    in    uint                    accelStructHi,
    in    uint                    constRayFlags,
    in    uint                    rayFlags,
    in    uint                    instanceMask,
    in    RayDesc                 rayDesc,
    in    uint3                   dispatchThreadId)
{
    // @note: Initialize all the RayQuery fields since leaving them uninitialized has been shown to increase
    //        register usage when used in loop such as the following, which is common for handling non-opaque primitives.
    //        The compiler assumes that the uninitialized values are carried over around each iteration of the outer
    //        loop, resulting in higher VGPR usage.
    //
    // while()
    // {
    //     RayQuery q1;
    //     q1.TraceRayInline();
    //     while(q1.Proceed()) {}
    //     ...
    //     RayQuery q2;
    //     q2.TraceRayInline();
    //     while(q2.Proceed()) {}
    // }

    rayQuery = (RayQueryInternal)0;

    const GpuVirtualAddress topLevelAddr = FetchAccelStructBaseAddr(accelStructLo, accelStructHi);

    // OR the templated ray flag with the dynamic ray flag function parameter
    const uint allRayFlags = constRayFlags | rayFlags;

    const uint64_t topLevelPtr = EncodeBasePointer(topLevelAddr, allRayFlags);

    rayQuery.bvhLo                 = LowPart(topLevelPtr);
    rayQuery.bvhHi                 = HighPart(topLevelPtr);
    rayQuery.topLevelBvhLo         = LowPart(topLevelAddr);
    rayQuery.topLevelBvhHi         = HighPart(topLevelAddr);

    rayQuery.rayFlags              = allRayFlags;
    rayQuery.committedStatus       = COMMITTED_NOTHING;

    rayQuery.rayTMin               = rayDesc.TMin;
    rayQuery.committed.rayTCurrent = rayDesc.TMax - ApplyTMinBias(rayDesc.TMin);
    rayQuery.candidate.rayTCurrent = rayQuery.committed.rayTCurrent;

    rayQuery.rayDesc.TMax          = rayDesc.TMax - ApplyTMinBias(rayDesc.TMin);
    rayQuery.rayDesc.TMin          = rayDesc.TMin - ApplyTMinBias(rayDesc.TMin);
    rayQuery.rayDesc.Origin        = rayDesc.Origin + (ApplyTMinBias(rayDesc.TMin) * rayDesc.Direction);
    rayQuery.rayDesc.Direction     = rayDesc.Direction;

    rayQuery.candidate.origin      = rayQuery.rayDesc.Origin;
    rayQuery.candidate.direction   = rayQuery.rayDesc.Direction;

    rayQuery.candidate.currNodePtr = INVALID_NODE;
    rayQuery.committed.currNodePtr = INVALID_NODE;

    rayQuery.stackPtr = RtIp3LdsStackInit();

    rayQuery.instanceInclusionMask = instanceMask;

    if (IsValidTrace(rayQuery.rayDesc, topLevelAddr, instanceMask, rayQuery.rayFlags, 0))
    {
        LogAccelStruct(topLevelAddr);

        // Some applications may request candidate/committed intrinsics even when there is no candidate or
        // committed hit. In order to return sane data we initialise candidate instance node pointer to the
        // first instance in the leaf node array.
        const uint32_t leafNodeOffset = FetchHeaderOffsetField(topLevelAddr, ACCEL_STRUCT_OFFSETS_LEAF_NODES_OFFSET);

        rayQuery.candidate.instNodePtr = PackNodePointer(NODE_TYPE_USER_NODE_INSTANCE, leafNodeOffset);
        rayQuery.committed.instNodePtr = PackNodePointer(NODE_TYPE_USER_NODE_INSTANCE, leafNodeOffset);

        rayQuery.currNodePtr = CreateRootNodePointer3_1();
    }
    else
    {
        // If acceleration structure is null or ray descriptor has nan values in it then set
        // currNodePtr to invalid node which will quit traversal right away
        rayQuery.currNodePtr = INVALID_NODE;
    }

    // Reuse isGoingDown to store whether to process tri 0 or 1 in pair in the next iteration.
    rayQuery.isGoingDown = 0;

    // Reuse currNodePtr2 to store parent ptr.
    rayQuery.currNodePtr2 = INVALID_NODE;

    // prevNodePtr field is used for lastNodePointer in RTIP3 traversal.
    rayQuery.prevNodePtr = INVALID_NODE;

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        const uint rayId = GetRayId(dispatchThreadId);
        SetRayQueryDynamicId(rayQuery, AllocateRayHistoryDynamicId());
        WriteRayHistoryTokenBegin(rayId,
                                  dispatchThreadId,
                                  topLevelAddr,
                                  rayQuery.rayFlags,
                                  instanceMask,
                                  rayDesc,
                                  AmdTraceRayGetStaticId(),
                                  GetRayQueryDynamicId(rayQuery),
                                  -1);
    }
#endif
}

//======================================================================================================================
static uint ProcessPrimitiveNode3_1(
    inout_param(RayQueryInternal) rayQuery,
    uint64_t                      currBasePtr,
    uint                          nodePointer,
    Bvh8IntersectResult           result,
    bool                          haveNonOpaquePrimitives,
    bool                          haveProceduralNodes,
    bool                          tri0)
{
    const float tHit = result.T(tri0);

    uint status = PRIM_STATUS_IGNORE;

    const bool isTriangle = (haveProceduralNodes == false) || (result.IsProcedural(tri0) == false);

    if (EvaluateTriangleHit(rayQuery.rayDesc.TMin, tHit, rayQuery.committed.rayTCurrent) || (isTriangle == false))
    {
        rayQuery.candidate.currNodePtr          = nodePointer;
        rayQuery.candidate.instanceContribution = rayQuery.instanceHitContributionAndFlags; // No flags in 3.1 path

        rayQuery.candidate.rayTCurrent    = tHit;
        rayQuery.candidate.barycentrics.x = result.BaryI(tri0);
        rayQuery.candidate.barycentrics.y = result.BaryJ(tri0);
        rayQuery.candidate.frontFace      = result.FrontFace(tri0);

        const bool isOpaque    = (haveNonOpaquePrimitives == false) || (result.IsNonOpaque(tri0) == false);
        const bool isCandidate = (!isOpaque || !isTriangle);

        if (isCandidate == false)
        {
            status = PRIM_STATUS_COMMIT;

            // Let the app know an opaque triangle hit was detected.
            rayQuery.committed.currNodePtr          = rayQuery.candidate.currNodePtr;
            rayQuery.committed.instNodePtr          = rayQuery.candidate.instNodePtr;
            rayQuery.committed.instanceContribution = rayQuery.candidate.instanceContribution;
            rayQuery.committed.rayTCurrent          = rayQuery.candidate.rayTCurrent;
            rayQuery.committed.barycentrics.x       = rayQuery.candidate.barycentrics.x;
            rayQuery.committed.barycentrics.y       = rayQuery.candidate.barycentrics.y;
            rayQuery.committed.frontFace            = rayQuery.candidate.frontFace;
            rayQuery.committed.origin               = rayQuery.candidate.origin;
            rayQuery.committed.direction            = rayQuery.candidate.direction;
            rayQuery.committed.primitiveIndex       = result.PrimitiveIndex(tri0);
            rayQuery.committed.geometryIndex        = result.GeometryIndex(tri0);

            // Commit an opaque triangle hit
            rayQuery.committedStatus = COMMITTED_TRIANGLE_HIT;
        }
        else
        {
            status = PRIM_STATUS_CANDIDATE;

            rayQuery.candidateType =
                isTriangle ? CANDIDATE_NON_OPAQUE_TRIANGLE  :
                isOpaque   ? CANDIDATE_PROCEDURAL_PRIMITIVE :
                             CANDIDATE_NON_OPAQUE_PROCEDURAL_PRIMITIVE;

            rayQuery.candidate.primitiveIndex = result.PrimitiveIndex(tri0);
            rayQuery.candidate.geometryIndex  = result.GeometryIndex(tri0);
        }
#if DEVELOPER
        if (EnableTraversalCounter())
        {
            rayQuery.numCandidateHits++;
        }
#endif
    }

    return status;
}

//======================================================================================================================
// RayQuery::Proceed() implementation for IP level 3.1
static bool RayQueryProceedImpl3_1(
    inout_param(RayQueryInternal) rayQuery,
    in    uint                    constRayFlags,
    in    uint3                   dispatchThreadId)
{
#if DEVELOPER
    uint64_t timerBegin = AmdTraceRaySampleGpuTimer();
    uint rayId = 0;
    if (EnableTraversalCounter())
    {
        rayId = GetRayId(dispatchThreadId);
    }
#endif

    // Terminate if ACCEPT_FIRST_HIT is set and a primitive was committed since the last Proceed.
    if ((rayQuery.rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)    &&
        ((rayQuery.committedStatus == COMMITTED_PROCEDURAL_PRIMITIVE_HIT) ||
         (rayQuery.committedStatus == COMMITTED_TRIANGLE_HIT)))
    {
        return false;
    }

    uint boxHeuristicMode = AmdTraceRayGetBoxSortHeuristicMode();
    if ((boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosest) ||
        (boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosestMidPoint))
    {
        boxHeuristicMode = GetBoxSortingHeuristicFromRayFlags(rayQuery.rayFlags, boxHeuristicMode);
    }

    uint continueTraversal = 0;

    // Note that the original top level address is stored here, but the current address is an encoded base pointer.
    // The driver needs the original top level address in the ray query object in order to load fields from the
    // instance node.
    const GpuVirtualAddress topLevelAddr = GetRayQueryTopBvhAddress(rayQuery);
    const uint64_t tlasBasePtr = EncodeBasePointer(topLevelAddr, rayQuery.rayFlags);
    uint64_t currBasePtr = GetRayQueryBvhAddress(rayQuery);

    const uint blasRootNodePtr = CreateRootNodePointer3_1();

    const bool rayForceOpaque           = (rayQuery.rayFlags & RAY_FLAG_FORCE_OPAQUE);
    const bool rayCullOpaque            = (rayQuery.rayFlags & RAY_FLAG_CULL_OPAQUE);
    const bool rayCullNonOpaque         = (rayQuery.rayFlags & RAY_FLAG_CULL_NON_OPAQUE);
    const bool raySkipProcedural        = (rayQuery.rayFlags & RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES);
    const bool triangleCullFrontFace    = (rayQuery.rayFlags & RAY_FLAG_CULL_FRONT_FACING_TRIANGLES);
    const bool triangleCullBackFace     = (rayQuery.rayFlags & RAY_FLAG_CULL_BACK_FACING_TRIANGLES);

    const bool haveProceduralNodes      = !raySkipProcedural;
    const bool alwaysOpaque             = (rayForceOpaque || rayCullNonOpaque);
    const bool haveNonOpaquePrimitives  = !alwaysOpaque;
    const bool haveCandidatePrims       = (haveProceduralNodes || haveNonOpaquePrimitives);

    // If the app rejected a candidate then revert back to the current committed rayT
    rayQuery.candidate.rayTCurrent = rayQuery.committed.rayTCurrent;

    // prevNodePtr field is used for lastNodePtr in RTIP3 traversal.
    uint lastNodePtr = rayQuery.prevNodePtr;

    // Note trying to use any member variable of rayQuery structure in the exit condition results in a massive amount
    // of v_mov instructions generated by SC which results in a big performance drop. Needs further investigation.
    while (1)
    {
        if (IsValidNode(rayQuery.currNodePtr) == false)
        {
            break;
        }

        // Reset the ray and base address to the top level. Note that this can only be done once the application has
        // processed candidate primitives, so it must occur on the following proceed.
        if (rayQuery.stackPtr & STACK_ADDR_BLAS_TO_TLAS_MASK)
        {
            rayQuery.candidate.origin    = rayQuery.rayDesc.Origin;
            rayQuery.candidate.direction = rayQuery.rayDesc.Direction;

            currBasePtr = tlasBasePtr;

#if DEVELOPER
            if (EnableTraversalCounter())
            {
                WriteRayHistoryTokenTopLevel(rayId, ExtractInstanceAddr(currBasePtr));
            }
#endif
        }

#if DEVELOPER
        if (EnableTraversalCounter())
        {
            rayQuery.numIterations++;

            WriteRayHistoryTokenNodePtr(rayId, rayQuery.currNodePtr);
            UpdateWaveTraversalStatistics(GPURT_RTIP3_1, rayQuery.currNodePtr);
        }
#endif
        const float rayExtent = (lastNodePtr == INVALID_NODE) ? rayQuery.committed.rayTCurrent : rayQuery.rayDesc.TMax;

        Bvh8IntersectResult result =
                image_bvh8_intersect_ray_3_1(currBasePtr,
                                             rayQuery.currNodePtr,
                                             rayQuery.candidate.origin,
                                             rayQuery.candidate.direction,
                                             rayExtent,
                                             rayQuery.instanceInclusionMask,
                                             boxHeuristicMode);

#if GPURT_ENABLE_GPU_DEBUG
        // Check if the child pointers returned are valid.
        if (IsBoxNode3_1(rayQuery.currNodePtr))
        {
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot0.x, currBasePtr, tlasBasePtr);
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot0.y, currBasePtr, tlasBasePtr);
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot0.z, currBasePtr, tlasBasePtr);
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot0.w, currBasePtr, tlasBasePtr);

            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot1.x, currBasePtr, tlasBasePtr);
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot1.y, currBasePtr, tlasBasePtr);
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot1.z, currBasePtr, tlasBasePtr);
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot1.w, currBasePtr, tlasBasePtr);
        }
#endif

        if (IsUserNodeInstance(rayQuery.currNodePtr))
        {
            rayQuery.instNodePtr = rayQuery.currNodePtr;
            rayQuery.candidate.instNodePtr = rayQuery.currNodePtr;
            currBasePtr = result.InstanceChildBasePtr();
            rayQuery.instanceHitContributionAndFlags = result.InstanceHitContribution();

            // Note, the packed node pointer returned by hardware includes relative offsets of child node
            // within the box node. I.e. for a child node encoding of 0x15, 0x25, 0x30, 0x40, the hardware
            // packs the pointers as: 0x05, 0x15, 0x00, 0x20
            //
            // In InstanceMode::Passthrough and FilterNode, we only care about whether we hit any box or none.
            // Thus, we only check for the all miss case (i.e. 0xffffffff)

            if (IsBvhRebraid())
            {
                result.slot1.w = DecodeBlasRootFromInstanceResult(result.slot1.w);
                rayQuery.lastInstanceNode = result.slot1.w;
            }
            else
            {
                // With BVH rebraid, the passthrough node encoding is root_ptr, 0x00, 0x00, 0x00
                // E.g.
                // Encoded child type/length      : 0x35, 0x05, 0x05, 0x05
                // Decoded child node pointers as : 0x05, 0x35, 0x35, 0x35
                //
                // The hardware either returns 0xffffffff (missed), or 0xffff3505 (hit any)
                //
                // Note, child slots 3 and 4 are interpreted as reused pointers and disabled in the return
                // pointer.
                result.slot1.w = (result.slot1.w == INVALID_NODE) ? INVALID_NODE : blasRootNodePtr;
            }
            // result.slot1.z to SKIP_0_7 in order to skip the remaining inputs to ds_stack_push.
            result.slot1.z = SKIP_0_7;

#if DEVELOPER
            if (EnableTraversalCounter() && (result.slot1.w != INVALID_NODE))
            {
                WriteRayHistoryTokenBottomLevel(rayId, ExtractInstanceAddr(currBasePtr));
            }
#endif
        }
        else if (IsTriangleNode3_1(rayQuery.currNodePtr))
        {
            if (haveCandidatePrims == false)
            {
                const float t0 = result.T(true);
                const float t1 = result.T(false);
                const bool closestTri0 = (t0 <= t1);

                const uint status = ProcessPrimitiveNode3_1(rayQuery,
                                                            currBasePtr,
                                                            rayQuery.currNodePtr,
                                                            result,
                                                            haveCandidatePrims,
                                                            haveProceduralNodes,
                                                            closestTri0);

                result.slot1.w = rayQuery.currNodePtr;

                // NavigationBits is only encoded in tri0's return data.
                const uint navigationBits = result.NavigationBits(true);
                lastNodePtr = PRIM_RANGE_UPDATE | navigationBits;

                if ((status == PRIM_STATUS_COMMIT) && (rayQuery.rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH))
                {
                    break;
                }
            }
            else
            {
                // Hardware returns up to 2 triangle intersections in pair order.
                // triangle0, triangle1
                //
                // The traversal loop picks the closest of the two as a candidate hit and defers the farthest one
                // for the next iteration. On the reintersect the intersection result will return a miss for the
                // both the triangles if previous closest triangle was committed by the caller. If the caller
                // rejected the triangle hit, we need to process the "other" triangle on return.
                //
                const float t0 = result.T(true);
                const float t1 = result.T(false);

                // Intersect | Closest | Candidate Result | Reintersect | Process Triangle
                // t0 t1     | t0      | accept           | -1 -1       | none
                // t0 t1     | t0      | reject           | t0 t1       | t1
                // t0 t1     | t1      | accept           | -1 -1       | none
                // t0 t1     | t1      | reject           | t0 t1       | t0
                //
                const float maxT = max(t0, t1);

                bool closestTri0 = (t0 <= t1);

                // Flip triangle index if this is the second half of the pair of triangles.
                if (rayQuery.isGoingDown == 1)
                {
                    closestTri0 = !closestTri0;
                }

                const uint status = ProcessPrimitiveNode3_1(rayQuery,
                                                            currBasePtr,
                                                            rayQuery.currNodePtr,
                                                            result,
                                                            haveCandidatePrims,
                                                            haveProceduralNodes,
                                                            closestTri0);

                continueTraversal = (status == PRIM_STATUS_CANDIDATE);

                if ((status == PRIM_STATUS_COMMIT) && (rayQuery.rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH))
                {
                    break;
                }

                // Reuse isGoingDown to store whether we are reintersecting to process the other half of the
                // pair of triangles.

                // Only process the farthest triangle on return if it is a valid triangle intersection.
                if ((continueTraversal == 1) && (rayQuery.isGoingDown == 0) && (maxT < rayQuery.committed.rayTCurrent))
                {
                    // Save to process the second triangle in the pair on the next Proceed()
                    result.slot1.zw = uint2(SKIP_0_7, rayQuery.currNodePtr);
                    rayQuery.isGoingDown = 1;
                }
                else
                {
                    result.slot1.w = rayQuery.currNodePtr;
                    rayQuery.isGoingDown = 0;

                    // NavigationBits is only encoded in tri0's return data.
                    const uint navigationBits = result.NavigationBits(true);
                    lastNodePtr = PRIM_RANGE_UPDATE | navigationBits;
                }
            }
        }
        else
        {
#if GPURT_ENABLE_GPU_DEBUG
            GPU_ASSERT(IsBoxNode3_1(rayQuery.currNodePtr));
#endif
            // Reuse currNodePtr2 to store parent ptr.
            rayQuery.currNodePtr2 = rayQuery.currNodePtr;
        }

        uint prevNodePtr = rayQuery.currNodePtr;
        rayQuery.currNodePtr =
            ds_stack_push8_pop1(rayQuery.stackPtr, lastNodePtr, result.slot0, result.slot1, GPURT_RTIP3_1);
        lastNodePtr = INVALID_NODE;

#if GPURT_ENABLE_GPU_DEBUG
        // Check if the node pointer popped from stack is valid.
        OutOfRangeNodePointerAssert(GPURT_RTIP3_1,
                                    rayQuery.currNodePtr,
                                    (rayQuery.stackPtr & STACK_ADDR_BLAS_TO_TLAS_MASK) ? tlasBasePtr : currBasePtr,
                                    tlasBasePtr);
#endif

        // Stackless walk back
        if (rayQuery.currNodePtr == INVALID_NODE)
        {
            const uint lastRootNode = IsBvhRebraid() ? rayQuery.lastInstanceNode : blasRootNodePtr;

            // Traverse back to top level
            if ((prevNodePtr == lastRootNode) && (currBasePtr != tlasBasePtr))
            {
                prevNodePtr = rayQuery.instNodePtr;
            }

            if (prevNodePtr == rayQuery.instNodePtr)
            {
                currBasePtr = tlasBasePtr;
                rayQuery.stackPtr |= STACK_ADDR_BLAS_TO_TLAS_MASK;
            }

            lastNodePtr = prevNodePtr;

            if (IsTriangleNode3_1(prevNodePtr))
            {
                rayQuery.currNodePtr = rayQuery.currNodePtr2;
            }
            else
            {
                rayQuery.currNodePtr = FetchParentNodePointer3_1(ExtractInstanceAddr(currBasePtr), prevNodePtr);
            }

#if GPURT_ENABLE_GPU_DEBUG
        // Check if parent pointer fetched is valid.
        OutOfRangeNodePointerAssert(GPURT_RTIP3_1,
                                    rayQuery.currNodePtr,
                                    (rayQuery.stackPtr & STACK_ADDR_BLAS_TO_TLAS_MASK) ? tlasBasePtr : currBasePtr,
                                    tlasBasePtr);
#endif
        }

        // haveCandidatePrims is typically a compile time constant. It ensures that continueTraversal is not read and
        // the loop exit logic is eliminated unless there is a need for candidate processing.
        if (haveCandidatePrims && continueTraversal)
        {
            break;
        }
    }

    rayQuery.bvhLo = LowPart(currBasePtr);
    rayQuery.bvhHi = HighPart(currBasePtr);

    // Save lastNodePtr in unused prevNodePtr field
    rayQuery.prevNodePtr = lastNodePtr;

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        uint64_t timerEnd = AmdTraceRaySampleGpuTimer();
        rayQuery.clocks += (uint)(timerEnd - timerBegin);
    }
#endif

    return continueTraversal;
}
