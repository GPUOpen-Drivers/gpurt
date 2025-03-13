/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2021-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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

// Set in the node pointer to indicate a generated triangle 0. This is required for RayQuery to differentiate it from
// a regular triangle 0 during stackless walkback.
#define GENERATED_TRI0_NODE_FLAG NODE_POINTER_MASK_MSB

//======================================================================================================================
// Set the appropriate skip flags in the result slot we are processing to avoid pushing triangle return data onto the
// stack. If compression is enabled, we insert the next node into the result vector to push it onto the stack next.
static uint4 GetResultVecForStackPush(
    uint4 result,      // Intersection result
    uint  nodePointer, // Current node pointer
    uint  skipFlags)   // Skip flags required to skip all values in this result (SKIP_0_3/SKIP_4_7/SKIP_0_7)
{
    if ((AmdTraceRayGetTriangleCompressionMode() == PAIR_TRIANGLE_COMPRESSION) ||
        (AmdTraceRayGetTriangleCompressionMode() == AUTO_TRIANGLE_COMPRESSION))
    {
        if ((GetNodeType(nodePointer) == NODE_TYPE_TRIANGLE_1) && (skipFlags == SKIP_0_3))
        {
            // Intersect with triangle 0 next and tag it as a generated tri0
            const uint newNodePointer = ClearNodeType(nodePointer) | GENERATED_TRI0_NODE_FLAG;

            // Place the new node pointer in the last result slot to push it onto the stack
            result.zw = uint2(skipFlags, newNodePointer);
        }
        else
        {
            result.w = skipFlags;
        }
    }
    else
    {
        result.w = skipFlags;
    }

    return result;
}

//======================================================================================================================
static void TraceRayInlineImpl3_0(
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
    rayQuery.committed.rayTCurrent = rayDesc.TMax - rayDesc.TMin;
    rayQuery.candidate.rayTCurrent = rayQuery.committed.rayTCurrent;

    rayQuery.rayDesc.TMax          = rayDesc.TMax - rayDesc.TMin;
    rayQuery.rayDesc.TMin          = 0.0f;
    rayQuery.rayDesc.Origin        = mad(rayDesc.Direction, rayDesc.TMin, rayDesc.Origin);
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

        rayQuery.currNodePtr = CreateRootNodePointer1_1();
    }
    else
    {
        // If acceleration structure is null or ray descriptor has nan values in it then set
        // currNodePtr to invalid node which will quit traversal right away
        rayQuery.currNodePtr = INVALID_NODE;
    }

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

#define PRIM_STATUS_IGNORE    0
#define PRIM_STATUS_COMMIT    1
#define PRIM_STATUS_CANDIDATE 2

//======================================================================================================================
static uint ProcessPrimitiveNode(
    inout_param(RayQueryInternal) rayQuery,
    uint64_t                      currBasePtr,
    uint                          nodePointer,
    uint4                         result,
    uint                          geometryIndex,
    bool                          haveNonOpaquePrimitives,
    bool                          haveProceduralNodes)
{
    const float tNum   = asfloat(result.x);
    const float tDenom = asfloat(result.y);
    const float tHit   = tNum / tDenom;

    uint status = PRIM_STATUS_IGNORE;

    // Sign bit of I (result.z) is set for procedural primitives
    const bool isTriangle = (haveProceduralNodes == false) || ((result.z & 0x80000000) != 0);

    if ((tHit < rayQuery.committed.rayTCurrent) || (isTriangle == false))
    {
        rayQuery.candidate.currNodePtr          = nodePointer;
        rayQuery.candidate.instanceContribution = rayQuery.instanceHitContributionAndFlags; // No flags in 3.0 path

        rayQuery.candidate.rayTCurrent    = tHit;
        rayQuery.candidate.barycentrics.x = abs(asfloat(result.z) / tDenom);
        rayQuery.candidate.barycentrics.y = abs(asfloat(result.w) / tDenom);

        // Positive result.y indicates front facing triangle hit
        rayQuery.candidate.frontFace      = (result.y >> 31) ^ 1;

        // Sign bit of J (result.w) is set for non-opaque primitives
        const bool isOpaque   = (haveNonOpaquePrimitives == false) || ((result.w & 0x80000000) == 0);

        const bool isCandidate = (!isOpaque || !isTriangle);

        if (isCandidate == false)
        {
            status = PRIM_STATUS_COMMIT;

            // Let the app know an opaque triangle hit was detected. The primitive index and geometry
            // index are loaded after the traversal loop.
            rayQuery.committed.currNodePtr          = rayQuery.candidate.currNodePtr;
            rayQuery.committed.instNodePtr          = rayQuery.candidate.instNodePtr;
            rayQuery.committed.instanceContribution = rayQuery.candidate.instanceContribution;
            rayQuery.committed.rayTCurrent          = rayQuery.candidate.rayTCurrent;
            rayQuery.committed.barycentrics.x       = rayQuery.candidate.barycentrics.x;
            rayQuery.committed.barycentrics.y       = rayQuery.candidate.barycentrics.y;
            rayQuery.committed.frontFace            = rayQuery.candidate.frontFace;
            rayQuery.committed.origin               = rayQuery.candidate.origin;
            rayQuery.committed.direction            = rayQuery.candidate.direction;

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

            const PrimitiveData primitiveData =
                FetchPrimitiveDataAddr3_0(nodePointer, GetNodeAddr(currBasePtr, nodePointer));

            rayQuery.candidate.primitiveIndex = primitiveData.primitiveIndex;
            rayQuery.candidate.geometryIndex  = (geometryIndex & 0xFFFFF);
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
// RayQuery::Proceed() implementation for IP level 3.0
static bool RayQueryProceedImpl3_0(
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

    // BLAS root node is always fp32 regardless of mode for fp16 box nodes
    const uint blasRootNodePtr = CreateRootNodePointer1_1();

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

    uint64_t committedBlasBasePtr = 0;

    // prevNodePtr field is used for lastNodePointer in RTIP3 traversal.
    uint lastNodePointer = rayQuery.prevNodePtr;

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
        }

#if DEVELOPER
        if (EnableTraversalCounter())
        {
            rayQuery.numIterations++;

            WriteRayHistoryTokenNodePtr(rayId, rayQuery.currNodePtr);
            UpdateWaveTraversalStatistics(GPURT_RTIP3_0, rayQuery.currNodePtr);

            if (rayQuery.currNodePtr2 != INVALID_NODE)
            {
                WriteRayHistoryTokenNodePtr(rayId, rayQuery.currNodePtr2);
                UpdateWaveTraversalStatistics(GPURT_RTIP3_0, rayQuery.currNodePtr2);
            }
        }
#endif

        DualIntersectResult result =
            image_bvh_dual_intersect_ray(currBasePtr,
                                         GetNodePointerExclMsbFlag(rayQuery.currNodePtr),
                                         GetNodePointerExclMsbFlag(rayQuery.currNodePtr2),
                                         rayQuery.candidate.origin,
                                         rayQuery.candidate.direction,
                                         rayQuery.committed.rayTCurrent,
                                         rayQuery.instanceInclusionMask,
                                         boxHeuristicMode);

        if (IsUserNodeInstance(rayQuery.currNodePtr))
        {
            currBasePtr = PackUint64(result.first.z, result.first.w);
            rayQuery.instanceHitContributionAndFlags = result.second.z;
            rayQuery.instNodePtr = rayQuery.currNodePtr;
            rayQuery.candidate.instNodePtr = rayQuery.currNodePtr;
            if (IsBvhRebraid())
            {
                rayQuery.lastInstanceNode = result.second.w;
            }
            // result.second.w contains the root node of the instance or INVALID_NODE if the instance was culled. Set
            // result.second.z to SKIP_0_7 in order to skip the remaining inputs to ds_stack_push.
            result.second.z = SKIP_0_7;
        }
        else
        {
            // The node getting processed first only needs to check for node type
            // since there should be no previous hits
            if (IsTriangleNode1_1(rayQuery.currNodePtr2))
            {
                const uint status =
                    ProcessPrimitiveNode(rayQuery,
                                         currBasePtr,
                                         rayQuery.currNodePtr2,
                                         result.second,
                                         result.geometryId[1],
                                         haveNonOpaquePrimitives,
                                         haveProceduralNodes);

                result.second = GetResultVecForStackPush(result.second, rayQuery.currNodePtr2, SKIP_4_7);

                if (status == PRIM_STATUS_COMMIT)
                {
                    committedBlasBasePtr = currBasePtr;
                    if (rayQuery.rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)
                    {
                        break;
                    }
                }
                else if (status == PRIM_STATUS_CANDIDATE)
                {
                    continueTraversal = 1;

                    // currNodePtr2 is processed first in current cycle and it's a hit, push currNodePtr back to
                    // stack and process it in next iteration if currNodePtr is also triangle
                    if (IsTriangleNode1_1(rayQuery.currNodePtr))
                    {
                        // If currNodePtr2 is a tri1, include the generated tri0 flag for stackless walkback to read.
                        const uint newNodePointer = (GetNodeType(rayQuery.currNodePtr2) == NODE_TYPE_TRIANGLE_1) ?
                            (rayQuery.currNodePtr | GENERATED_TRI0_NODE_FLAG) : rayQuery.currNodePtr;

                        // Save the other triangle node to process on the next Proceed()
                        result.first.zw = uint2(SKIP_0_3, newNodePointer);
                    }
                }
            }

            // currNodePtr2 is not a hit, process currNodePtr in current iteration
            if (IsTriangleNode1_1(rayQuery.currNodePtr) &&
                ((haveCandidatePrims == false) ||
                (continueTraversal == 0)))
            {
                const uint status =
                    ProcessPrimitiveNode(rayQuery,
                                         currBasePtr,
                                         rayQuery.currNodePtr,
                                         result.first,
                                         result.geometryId[0],
                                         haveNonOpaquePrimitives,
                                         haveProceduralNodes);

                result.first = GetResultVecForStackPush(result.first, rayQuery.currNodePtr, SKIP_0_3);

                if (status == PRIM_STATUS_COMMIT)
                {
                    committedBlasBasePtr = currBasePtr;
                    if (rayQuery.rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)
                    {
                        break;
                    }
                }
                else if (status == PRIM_STATUS_CANDIDATE)
                {
                    continueTraversal = 1;
                }
            }
        }

        uint2 returnNodes = ds_stack_push8_pop2(rayQuery.stackPtr, lastNodePointer, result.first, result.second);
        uint prevNodePtr  = (rayQuery.currNodePtr2 != INVALID_NODE) ? rayQuery.currNodePtr2 : rayQuery.currNodePtr;

        rayQuery.currNodePtr  = returnNodes.x;
        rayQuery.currNodePtr2 = returnNodes.y;

        lastNodePointer = INVALID_NODE;

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

            // A generated tri0 node pointer from triangle pair compression does not really exist in the BVH tree, so
            // override the lastNodePointer to the corresponding tri1.
            const bool intersectedGeneratedTri0 = (prevNodePtr & GENERATED_TRI0_NODE_FLAG) != 0;

            prevNodePtr     = GetNodePointerExclMsbFlag(prevNodePtr);
            lastNodePointer = intersectedGeneratedTri0 ? (prevNodePtr + 1) : prevNodePtr;

            rayQuery.currNodePtr = FetchParentNodePointer(
                ExtractInstanceAddr(currBasePtr),
                prevNodePtr);
        }

        // haveCandidatePrims is typically a compile time constant. It ensures that continueTraversal is not read and
        // the loop exit logic is eliminated unless there is a need for candidate processing.
        if (haveCandidatePrims && continueTraversal)
        {
            break;
        }
    }

    if (committedBlasBasePtr != 0)
    {
        const uint nodePointer = rayQuery.committed.currNodePtr;

        const PrimitiveData primitiveData =
            FetchPrimitiveDataAddr3_0(nodePointer, GetNodeAddr(committedBlasBasePtr, nodePointer));

        rayQuery.committed.primitiveIndex = primitiveData.primitiveIndex;
        rayQuery.committed.geometryIndex  = primitiveData.geometryIndex;
    }

    rayQuery.bvhLo = LowPart(currBasePtr);
    rayQuery.bvhHi = HighPart(currBasePtr);

    // Save lastNodePointer in unused prevNodePtr field
    rayQuery.prevNodePtr = lastNodePointer;

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        uint64_t timerEnd = AmdTraceRaySampleGpuTimer();
        rayQuery.clocks += (uint)(timerEnd - timerBegin);
    }
#endif

    return continueTraversal;
}

//======================================================================================================================
// RayQuery::Proceed() implementation for IP level 3.0 with BVH8
static bool RayQueryProceedImpl3_0BVH8(
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

    const uint blasRootNodePtr = CreateRootNodePointer1_1();

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

    uint64_t committedBlasBasePtr = 0;

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
        }

#if DEVELOPER
        if (EnableTraversalCounter())
        {
            rayQuery.numIterations++;

            WriteRayHistoryTokenNodePtr(rayId, rayQuery.currNodePtr);
            UpdateWaveTraversalStatistics(GPURT_RTIP3_0, rayQuery.currNodePtr);
        }
#endif

        DualIntersectResult result =
                image_bvh8_intersect_ray(currBasePtr,
                                         rayQuery.currNodePtr,
                                         rayQuery.candidate.origin,
                                         rayQuery.candidate.direction,
                                         rayQuery.committed.rayTCurrent,
                                         rayQuery.instanceInclusionMask,
                                         boxHeuristicMode);

        if (IsUserNodeInstance(rayQuery.currNodePtr))
        {
            rayQuery.instNodePtr = rayQuery.currNodePtr;
            rayQuery.candidate.instNodePtr = rayQuery.currNodePtr;
            currBasePtr = PackUint64(result.first.z, result.first.w);
            rayQuery.instanceHitContributionAndFlags = result.second.z;
            if (IsBvhRebraid())
            {
                rayQuery.lastInstanceNode = result.second.w;
            }
            // result.second.w contains the root node of the instance or INVALID_NODE if the instance was culled. Set
            // result.second.z to SKIP_0_7 in order to skip the remaining inputs to ds_stack_push.
            result.second.z = SKIP_0_7;
        }
        else if (IsTriangleNode1_1(rayQuery.currNodePtr))
        {
            const bool isPairCompressed =
                ((AmdTraceRayGetTriangleCompressionMode() == PAIR_TRIANGLE_COMPRESSION) ||
                 (AmdTraceRayGetTriangleCompressionMode() == AUTO_TRIANGLE_COMPRESSION)) &&
                (GetNodeType(rayQuery.currNodePtr) == NODE_TYPE_TRIANGLE_1);

            if (isPairCompressed)
            {
                const uint status = ProcessPrimitiveNode(rayQuery,
                                                         currBasePtr,
                                                         rayQuery.currNodePtr,
                                                         result.second,
                                                         result.geometryId[1],
                                                         haveCandidatePrims,
                                                         haveProceduralNodes);

                // Skip inputs to ds_stack_push
                result.second.w = SKIP_0_7;

                if (status == PRIM_STATUS_COMMIT)
                {
                    committedBlasBasePtr = currBasePtr;
                    if (rayQuery.rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)
                    {
                        break;
                    }
                }
                else if (status == PRIM_STATUS_CANDIDATE)
                {
                    continueTraversal = 1;

                    // Save tri 0 to process on the next Proceed()
                    result.second.zw = uint2(SKIP_0_7, ClearNodeType(rayQuery.currNodePtr));
                }
            }

            // Skip processing tri 0 if we need to leave traversal to allow candidate processing.
            if ((haveCandidatePrims == false) || (continueTraversal == 0))
            {
                const uint tri0NodePtr = ClearNodeType(rayQuery.currNodePtr);

                const uint status = ProcessPrimitiveNode(rayQuery,
                                                         currBasePtr,
                                                         tri0NodePtr,
                                                         result.first,
                                                         result.geometryId[0],
                                                         haveCandidatePrims,
                                                         haveProceduralNodes);

                // Skip inputs to ds_stack_push
                result.second.w = SKIP_0_7;

                if (status == PRIM_STATUS_COMMIT)
                {
                    committedBlasBasePtr = currBasePtr;
                    if (rayQuery.rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)
                    {
                        break;
                    }
                }
                else if (status == PRIM_STATUS_CANDIDATE)
                {
                    continueTraversal = 1;
                }
            }
        }

        uint prevNodePtr = rayQuery.currNodePtr;
        rayQuery.currNodePtr =
            ds_stack_push8_pop1(rayQuery.stackPtr, lastNodePtr, result.first, result.second, GPURT_RTIP3_0);
        lastNodePtr = INVALID_NODE;

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

            rayQuery.currNodePtr = FetchParentNodePointer(ExtractInstanceAddr(currBasePtr),
                                                          prevNodePtr);

            lastNodePtr = prevNodePtr;
        }

        // haveCandidatePrims is typically a compile time constant. It ensures that continueTraversal is not read and
        // the loop exit logic is eliminated unless there is a need for candidate processing.
        if (haveCandidatePrims && continueTraversal)
        {
            break;
        }
    }

    if (committedBlasBasePtr != 0)
    {
        const uint nodePointer = rayQuery.committed.currNodePtr;

        const PrimitiveData primitiveData =
            FetchPrimitiveDataAddr3_0(nodePointer, GetNodeAddr(committedBlasBasePtr, nodePointer));

        rayQuery.committed.primitiveIndex = primitiveData.primitiveIndex;
        rayQuery.committed.geometryIndex  = primitiveData.geometryIndex;
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
