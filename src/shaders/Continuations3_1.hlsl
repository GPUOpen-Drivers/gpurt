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
#include "rtip3_1.hlsli"

//=====================================================================================================================
static void InitCommittedState(
    inout_param(_AmdPrimitiveSystemState) committed,
    inout_param(float2) committedBarycentrics,
    float tMin,
    float tMax)
{
    committed.rayTCurrent = tMax - tMin;
    committed.instNodePtr = INVALID_NODE;
    committed.primitiveIndex = INVALID_NODE;
    committed.PackInstanceContribution(0);

    committed.PackGeometryIndex(0, false, TRAVERSAL_STATE_COMMITTED_NOTHING, false);
    committed.SetCurrNodePtr(INVALID_NODE);

#if GPURT_DEBUG_CONTINUATION_TRAVERSAL
    committed.PackAnyHitCallType(0);
    committed.PackIsProcedural(0);
#endif

    committedBarycentrics = float2(0.0, 0.0);
}

//=====================================================================================================================
static _AmdTraversalState InitTraversalState3_1(
    uint     instanceInclusionMask,
    RayDesc  ray,
    bool     isValid)
{
    // Initialise traversal state to initial state
    _AmdTraversalState traversal = (_AmdTraversalState)0;

#ifndef REMAT_INSTANCE_RAY
    traversal.candidateRayOrigin    = ray.Origin + (ray.TMin * ray.Direction);
    traversal.candidateRayDirection = ray.Direction;
#endif

    InitCommittedState(traversal.committed, traversal.committedBarycentrics, ray.TMin, ray.TMax);

    traversal.instNodePtr             = INVALID_NODE;
    traversal.reservedNodePtr         = INVALID_NODE;
    traversal.lastInstanceRootNodePtr = INVALID_NODE;

    traversal.PackParentPointer(INVALID_NODE);
    traversal.PackInstanceContributionAndSkipTri0(0, false);

    // Initialise traversal stack pointers
    traversal.stackPtr = RtIp3LdsStackInit();
    return traversal;
}

//=====================================================================================================================
// Helper function allowing to apply the same condition to dynamic and compile-time flags.
static bool AlwaysOpaque(uint flags)
{
    return (flags & (RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_CULL_NON_OPAQUE));
}

//=====================================================================================================================
static bool SkipProcedural(uint flags)
{
    return (flags & RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES);
}

//=====================================================================================================================
// Fixed function traversal loop
static void TraversalInternal3_1(
    inout_param(_AmdSystemData)           data,
    inout_param(uint)                     state,
    inout_param(_AmdPrimitiveSystemState) candidate,
    inout_param(float2)                   candidateBarycentrics)
{
    // Start from root node which follows acceleration structure header
    const uint rootNodePtr  = CreateRootNodePointer3_1();
    const uint instanceMask = data.hitObject.ray.traceParameters & 0xff;

    const uint rayFlags         = data.hitObject.ray.Flags();
    const uint boxHeuristicMode = GetBoxHeuristicMode();

    // "RtIp3LdsStackInit" is stored in "data.traversal.stackPtr"
    uint stackPtr = data.traversal.stackPtr;
    uint instNodePtr      = INVALID_NODE;
    uint lastInstanceRootNodePtr = data.traversal.lastInstanceRootNodePtr;

    // load parentNodePointer and instanceHitContribution from save traversal state
    // when initializing the traversal, those will be set to default vaule;
    // when re-entering the traversal, it will load from saved state
    // Note:
    //     should not set "parentNodePtr" to INVALID_NODE, or "instanceHitContribution" to 0
    //     it need to reload the previous traversal state
    //     parentNodePtr could be used for stackless walkback
    //     instanceHitContribution could be used for triangle / aabb process (see processContinuation)
    uint parentNodePtr    = data.traversal.ParentPointer();
    uint instanceHitContribution = data.traversal.InstanceContribution();

    // Initialise transient traversal state
    uint lastNodePtr = data.traversal.reservedNodePtr;

    // Initialise local registers from traversal state. Note, using the large _AmdTraversalState register directly results
    // in spilling to scratch memory. To avoid this, copy traversal state into temporary registers
    _AmdPrimitiveSystemState committed = data.traversal.committed;

    const bool useDeferMode            = (Options::getCpsCandidatePrimitiveMode() == CpsCandidatePrimitiveMode::DeferFirst);
    bool haveDeferredCandidate         = false;
    bool deferredCandidateIsProcedural = false;

    candidate                          = (_AmdPrimitiveSystemState)0;

    float2 committedBarycentrics       = data.traversal.committedBarycentrics;
    candidateBarycentrics              = float2(0.0f, 0.0f);

    uint   nextNodePtr             = data.traversal.nextNodePtr;   // starting with rootNodePtr
    float3 candidateRayOrigin      = data.hitObject.ray.origin  + (data.hitObject.ray.tMin * data.hitObject.ray.direction);
    float3 candidateRayDirection   = data.hitObject.ray.direction;
    state                          = TRAVERSAL_STATE_COMMITTED_NOTHING;

    // The ray flag bools prefixed with static.. only test whether flags are known at compile time.
    // These are intended to be used for compile-time optimizations, e.g. switching to a more optimized
    // function variant without the overhead of a run-time check and having to compile both variants.
    // Even if the static flags are unset, the dynamic ones might be.
    const bool skipProcedural       = SkipProcedural(rayFlags);
    const bool staticSkipProcedural = SkipProcedural(AmdTraceRayGetAllKnownStaticallySetFlags());
    const bool alwaysOpaque         = AlwaysOpaque(rayFlags);
    const bool staticAlwaysOpaque   = AlwaysOpaque(AmdTraceRayGetAllKnownStaticallySetFlags());

    const bool staticMayHaveCandidatePrims = (!staticSkipProcedural || !staticAlwaysOpaque);

    // Indicates whether we just processed a non-opaque tri0 hit, and thus when re-processing the same
    // triangle pair should skip the first triangle.
    bool skipTri0 = staticMayHaveCandidatePrims ? data.traversal.SkipTri0() : false;

    // Root bvh address for reuse
    const GpuVirtualAddress tlasBaseAddr = data.hitObject.ray.AccelStruct();
    const uint64_t tlasBasePtr           = EncodeBasePointer(tlasBaseAddr, rayFlags);
    // Updateable bottom level bvh for reuse
    GpuVirtualAddress currBasePtr        = tlasBasePtr;

    const float tMax       = data.hitObject.ray.tMax;
    // Restore after AnyHit or Intersection was called
    if (staticMayHaveCandidatePrims && (data.traversal.instNodePtr != INVALID_NODE))
    {
#ifdef REMAT_INSTANCE_RAY
        Bvh8IntersectResult result =
            image_bvh8_intersect_ray_3_1(currBasePtr,
                                         data.traversal.instNodePtr,
                                         candidateRayOrigin,
                                         candidateRayDirection,
                                         committed.rayTCurrent,
                                         instanceMask,
                                         boxHeuristicMode);

        currBasePtr             = result.InstanceChildBasePtr();
        instanceHitContribution = result.InstanceHitContribution();
#else
        const uint64_t instanceAddr = data.hitObject.ray.AccelStruct() +
                                      ExtractNodePointerOffset(data.traversal.instNodePtr);
        // node.childBasePtr stored in the 4th DW
        uint dw3 = LoadDwordAtAddrx4(instanceAddr + 0x30);
        currBasePtr = PackUint64(dw3.x, dw3.y);

        candidateRayOrigin    = data.traversal.candidateRayOrigin;
        candidateRayDirection = data.traversal.candidateRayDirection;
#endif
    }

    // This if guards all lanes that will do actual BVH traversal work
    if (IsValidNode(nextNodePtr))
    {
        if (AmdTraceRayGetAllKnownStaticallySetFlags() & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)
        {
            // If we statically know the first hit will always be accepted, and we know that we don't have a hit
            // yet because we are about to do BVH traversal, then reset the committed hit so the compiler
            // doesn't have to preserve it in case we miss.
            InitCommittedState(committed, committedBarycentrics, data.hitObject.ray.tMin, data.hitObject.ray.tMax);
        }

        // Traverse acceleration structure while we have valid nodes to intersect. Note, the traversal routine is
        // re-entrant and AnyHit shaders may set nextNodePtr to invalid when AcceptHitAndEndSearch() is called.
        do
        {
            RayHistoryHandleIteration(data, nextNodePtr);

            const uint nodePtr = nextNodePtr;

            const float rayExtent = (lastNodePtr == INVALID_NODE) ? committed.rayTCurrent : tMax;

            // Perform ray-bvh node intersection
            Bvh8IntersectResult result =
                image_bvh8_intersect_ray_3_1(currBasePtr,
                                             nodePtr,
                                             candidateRayOrigin,
                                             candidateRayDirection,
                                             rayExtent,
                                             instanceMask,
                                             boxHeuristicMode);

            // Trigger node search for next iteration
            nextNodePtr = INVALID_NODE;

            uint nodeType = GetNodeType(nodePtr);
            if (NodeTypeIsUserNodeInstance(nodeType))
            {
                RayHistoryIncInstanceIntersections(data);

                data.traversal.instNodePtr = nodePtr;

                currBasePtr = result.InstanceChildBasePtr();

                // Instance contribution is stored in the 24 bit user data field of the instance node
                instanceHitContribution = result.InstanceHitContribution();

                result.slot1.w = DecodeBlasRootFromInstanceResult(result.slot1.w);
                lastInstanceRootNodePtr = result.slot1.w;

                // result.slot1.z to SKIP_0_7 in order to skip the remaining inputs to ds_stack_push.
                result.slot1.z = SKIP_0_7;

                if (result.slot1.w != INVALID_NODE)
                {
                    RayHistoryWriteBottomLevel(data, ExtractInstanceAddr(currBasePtr));
                }
            }
            else if (NodeTypeIsTriangleNode3_1(nodeType))
            {
                RayHistoryIncNumRayTriangleTest(data);

                // This is a compile-time if:
                if (staticMayHaveCandidatePrims)
                {
                    bool skipTri1 = false;

                    // Process Tri0 first
                    // use "instanceHitContribution" here instead of candidate.instanceContribution
                    // because in AnyHit, "instanceHitContribution" could be reset when REMAT_INSTANCE_RAY
                    if (skipTri0 == false)
                    {
                        bool isTri0 = true;

                        const bool isProcedural = !skipProcedural && result.IsProcedural(isTri0);
                        const bool hitIsNonOpaque = result.IsNonOpaque(isTri0);
                        const bool processNonOpaque = !alwaysOpaque && hitIsNonOpaque;
                        float tHit = result.T(isTri0);

                        // For procedural nodes, the HW returns tHit = 0, and the Intersection shader could report a hit at tHit = 0.
                        // Procedural hits with matching tHit must be processed, and thus we can't check (tHit < committed.rayTCurrent)
                        // for procedural hits.
                        if ((tHit < committed.rayTCurrent) || isProcedural)
                        {
                            RayHistorySetCandidateTCurrent(data, tHit);

                            uint hitKind        = result.HitKind(isTri0);
                            uint primitiveIndex = result.PrimitiveIndex(isTri0);
                            uint geometryIndex  = result.GeometryIndex(isTri0);

                            bool hasAnyHit = false;
                            if (processNonOpaque)
                            {
                                hasAnyHit = AnyHitIsNonNull(data.hitObject.ray.traceParameters,
                                                            geometryIndex,
                                                            instanceHitContribution);
                            }

                            // DXR spec: if there is no any hit shader, the geometry is considered opaque.
                            if (!isProcedural && (!processNonOpaque || (!hasAnyHit)))
                            {
                                // For opaque hits, directly access the committed hit, instead of first setting the candidate,
                                // and then copying over the candidate to the committed hit.
                                committed.PackGeometryIndex(result.GeometryIndex(isTri0), isTri0, TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT, true);
                                committed.primitiveIndex = result.PrimitiveIndex(isTri0);
                                committed.rayTCurrent = tHit;
                                committed.instNodePtr = data.traversal.instNodePtr;
                                committed.PackInstanceContribution(instanceHitContribution, result.HitKind(isTri0));
                                committed.SetCurrNodePtr(nodePtr);

                                committedBarycentrics.x = result.BaryI(isTri0);
                                committedBarycentrics.y = result.BaryJ(isTri0);

                                state = TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT;
                                if (rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)
                                {
                                    if (useDeferMode)
                                    {
                                        haveDeferredCandidate = false;
                                    }
                                    break;
                                }

                                if (useDeferMode && haveDeferredCandidate && candidate.rayTCurrent > tHit)
                                {
                                    // There is a pending candidate, but the new committed hit is closer, so we can discard it.
                                    haveDeferredCandidate = false;
                                }
                            }
                            else
                            {
                                if (useDeferMode && haveDeferredCandidate)
                                {
                                    // We already have a deferred candidate, so we can't process the current one here.
                                    // immediately break to first process the deferred candidate and come back to this
                                    // candidate here on Traversal resume.
                                    nextNodePtr = nodePtr;
                                    break;
                                }

                                // Use COMMITTED_TRIANGLE_HIT for the read back in a successive _cont_Traversal():
                                //   "state = data.traversal.CommittedState();"
                                // Using COMMITTED_PROCEDURAL_PRIMITIVE_HIT is also fine.
                                candidate.PackGeometryIndex(geometryIndex, isTri0, TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT, !processNonOpaque);
                                candidate.primitiveIndex = primitiveIndex;
                                candidate.rayTCurrent = tHit;
                                candidate.instNodePtr = data.traversal.instNodePtr;
                                candidate.PackInstanceContribution(instanceHitContribution, hitKind);
                                candidate.SetCurrNodePtr(nodePtr);

                                if (!isProcedural)
                                {
                                    candidateBarycentrics.x = result.BaryI(isTri0);;
                                    candidateBarycentrics.y = result.BaryJ(isTri0);;
                                }

#if GPURT_DEBUG_CONTINUATION_TRAVERSAL
                                candidate.PackIsProcedural(isProcedural);

                                const uint anyHitCallType = isProcedural ?
                                                            (processNonOpaque ? ANYHIT_CALLTYPE_DUPLICATE : ANYHIT_CALLTYPE_SKIP) :
                                                            ANYHIT_CALLTYPE_NO_DUPLICATE;
                                candidate.PackAnyHitCallType(anyHitCallType);
#endif

                                if (useDeferMode)
                                {
                                    // just record the candidate for later processing.
                                    GPU_ASSERT(!haveDeferredCandidate);
                                    haveDeferredCandidate = true;
                                    deferredCandidateIsProcedural = isProcedural;
                                }
                                else
                                {
                                    // process the candidate now. Set the state and prepare to suspend Traversal.
                                    if (isProcedural)
                                    {
                                        state = processNonOpaque ? TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_PROCEDURAL_PRIMITIVE :
                                                                   TRAVERSAL_STATE_CANDIDATE_PROCEDURAL_PRIMITIVE;
                                    }
                                    else
                                    {
                                        state = TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_TRIANGLE;
                                    }

                                    skipTri1 = true; // skip tri1 in this iteration to process the tri0 candidate now
                                    skipTri0 = true; // skip tri0 when resuming Traversal at the same node

                                    // Save to process tri 1 on the next traverse
                                    result.slot1.zw = uint2(SKIP_0_7, nodePtr);
                                } // end of process non-deferred candidate
                            } // end of process candidate
                        } // end of process non-ignored hit (opaque or candidate)
                    } // end of process tri0

                    // Skip processing tri 1 if we need to leave traversal to allow candidate processing.
                    if (skipTri1 == false)
                    {
                        // Process Tri1

                        bool isTri0 = false;

                        const bool isProcedural = !skipProcedural && result.IsProcedural(isTri0);
                        const bool hitIsNonOpaque = result.IsNonOpaque(isTri0);
                        const bool processNonOpaque = !alwaysOpaque && hitIsNonOpaque;
                        float tHit = result.T(isTri0);

                        // For procedural nodes, the HW returns tHit = 0, and the Intersection shader could report a hit at tHit = 0.
                        // Procedural hits with matching tHit must be processed, and thus we can't check (tHit < committed.rayTCurrent)
                        // for procedural hits.
                        if ((tHit < committed.rayTCurrent) || isProcedural)
                        {
                            RayHistorySetCandidateTCurrent(data, tHit);

                            uint hitKind        = result.HitKind(isTri0);
                            uint primitiveIndex = result.PrimitiveIndex(isTri0);
                            uint geometryIndex  = result.GeometryIndex(isTri0);

                            bool hasAnyHit = false;
                            if (processNonOpaque)
                            {
                                hasAnyHit = AnyHitIsNonNull(data.hitObject.ray.traceParameters,
                                                            geometryIndex,
                                                            instanceHitContribution);
                            }

                            // DXR spec: if there is no any hit shader, the geometry is considered opaque.
                            if (!isProcedural && (!processNonOpaque || (!hasAnyHit)))
                            {
                                // For opaque hits, directly access the committed hit, instead of first setting the candidate,
                                // and then copying over the candidate to the committed hit.
                                committed.PackGeometryIndex(result.GeometryIndex(isTri0), isTri0, TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT, true);
                                committed.primitiveIndex = result.PrimitiveIndex(isTri0);
                                committed.rayTCurrent = tHit;
                                committed.instNodePtr = data.traversal.instNodePtr;
                                committed.PackInstanceContribution(instanceHitContribution, result.HitKind(isTri0));
                                committed.SetCurrNodePtr(nodePtr);

                                committedBarycentrics.x = result.BaryI(isTri0);
                                committedBarycentrics.y = result.BaryJ(isTri0);

                                state = TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT;
                                if (rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)
                                {
                                    if (useDeferMode)
                                    {
                                        haveDeferredCandidate = false;
                                    }
                                    break;
                                }

                                if (useDeferMode && haveDeferredCandidate && candidate.rayTCurrent > tHit)
                                {
                                    // There is a pending candidate, but the new committed hit is closer, so we can discard it.
                                    haveDeferredCandidate = false;
                                }
                            }
                            else
                            {
                                if (useDeferMode && haveDeferredCandidate)
                                {
                                    // We already have a deferred candidate, so we can't process the current one here.
                                    // immediately break to first process the deferred candidate and come back to this
                                    // candidate here on Traversal resume.
                                    nextNodePtr = nodePtr;
                                    skipTri0    = true;
                                    break;
                                }

                                // Use COMMITTED_TRIANGLE_HIT for the read back in a successive _cont_Traversal():
                                //   "state = data.traversal.CommittedState();"
                                // Using COMMITTED_PROCEDURAL_PRIMITIVE_HIT is also fine.
                                candidate.PackGeometryIndex(geometryIndex, isTri0, TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT, !processNonOpaque);
                                candidate.primitiveIndex = primitiveIndex;
                                candidate.rayTCurrent = tHit;
                                candidate.instNodePtr = data.traversal.instNodePtr;
                                candidate.PackInstanceContribution(instanceHitContribution, hitKind);
                                candidate.SetCurrNodePtr(nodePtr);

                                if (!isProcedural)
                                {
                                    candidateBarycentrics.x = result.BaryI(isTri0);;
                                    candidateBarycentrics.y = result.BaryJ(isTri0);;
                                }

#if GPURT_DEBUG_CONTINUATION_TRAVERSAL
                                candidate.PackIsProcedural(isProcedural);

                                const uint anyHitCallType = isProcedural ?
                                                            (processNonOpaque ? ANYHIT_CALLTYPE_DUPLICATE : ANYHIT_CALLTYPE_SKIP) :
                                                            ANYHIT_CALLTYPE_NO_DUPLICATE;
                                candidate.PackAnyHitCallType(anyHitCallType);
#endif

                                if (useDeferMode)
                                {
                                    // just record the candidate for later processing.
                                    GPU_ASSERT(!haveDeferredCandidate);
                                    haveDeferredCandidate = true;
                                    deferredCandidateIsProcedural = isProcedural;
                                }
                                else
                                {
                                    if (isProcedural)
                                    {
                                        state = processNonOpaque ? TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_PROCEDURAL_PRIMITIVE :
                                                                   TRAVERSAL_STATE_CANDIDATE_PROCEDURAL_PRIMITIVE;
                                    }
                                    else
                                    {
                                        state = TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_TRIANGLE;
                                    }
                                } // end of process non-deferred candidate
                            } // end of process candidate
                        } // end of process non-ignored hit (opaque or candidate)

                        result.slot1.w = nodePtr;
                        // NavigationBits is only encoded in tri0's return data.
                        const uint navigationBits = result.NavigationBits(true);
                        lastNodePtr = PRIM_RANGE_UPDATE | navigationBits;
                        // skipTri0 is both used as incoming argument to Traversal, to indicate we should skip tri0 in
                        // the first iteration, as well as outgoing, to indicate we should skip tri0 in the first
                        // iteration of resumed traversal. We tried splitting it, but that produces worse ISA.
                        // Here, we reset skipTri0 after potentially having skipped tri0 in the first iteration.
                        // We rely on that this code here is unreachable from all places that set the outgoing value.
                        skipTri0 = false;
                    } // end of process tri0
                } // end of process triangle node with compile-time-support for candidates
                else
                {
                    // Alternate implementation of the above specialized for opaque-only, triangle-only hits
                    bool isTri0 = true;
                    float tHit = result.T(true); //tri0

                    if (tHit > result.T(false)) // compare tri0 with tri1 and pick the closer.
                    {
                        tHit = result.T(false);
                        isTri0 = false;
                    }

                    if (tHit < committed.rayTCurrent)
                    {
                        committed.instNodePtr = data.traversal.instNodePtr;
                        committed.primitiveIndex = result.PrimitiveIndex(isTri0);
                        committed.rayTCurrent = tHit;
                        committed.PackInstanceContribution(instanceHitContribution, result.HitKind(isTri0));
                        committed.PackGeometryIndex(result.GeometryIndex(isTri0), isTri0, TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT, true);
                        committed.SetCurrNodePtr(nodePtr);

                        committedBarycentrics.x = result.BaryI(isTri0);
                        committedBarycentrics.y = result.BaryJ(isTri0);

                        state = TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT;

                        if (rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)
                        {
                            if (useDeferMode)
                            {
                                haveDeferredCandidate = false;
                            }
                            break;
                        }

                        if (useDeferMode && haveDeferredCandidate && candidate.rayTCurrent > tHit)
                        {
                            // There is a pending candidate, but the new committed hit is closer, so we can discard it.
                            haveDeferredCandidate = false;
                        }
                    }

                    result.slot1.w = nodePtr;

                    // NavigationBits is only encoded in tri0's return data.
                    const uint navigationBits = result.NavigationBits(true);
                    lastNodePtr = PRIM_RANGE_UPDATE | navigationBits;
                }
            }
            else if (NodeTypeIsBoxNode3_1(nodeType))
            {
                RayHistoryIncNumRayBoxTest(data);
                parentNodePtr = nodePtr;
            }

            // backup previous node pointer
            uint prevNodePtr = nodePtr;

            nextNodePtr = ds_stack_push8_pop1(stackPtr,
                                              lastNodePtr,
                                              result.slot0,
                                              result.slot1,
                                              GPURT_RTIP3_1);
            lastNodePtr = INVALID_NODE;

            // Fallback to stackless walk
            if (nextNodePtr == INVALID_NODE)
            {
                const uint lastRootNode = IsBvhRebraid() ? lastInstanceRootNodePtr : rootNodePtr;

                if ((prevNodePtr == lastRootNode) && (currBasePtr != tlasBasePtr))
                {
                    prevNodePtr = data.traversal.instNodePtr;
                }

                if (prevNodePtr == data.traversal.instNodePtr)
                {
                    currBasePtr = tlasBasePtr;
                    stackPtr |= STACK_ADDR_BLAS_TO_TLAS_MASK;
                }

                lastNodePtr = prevNodePtr;

                if (IsTriangleNode3_1(prevNodePtr))
                {
                    nextNodePtr = parentNodePtr;
                }
                else
                {
                    nextNodePtr = FetchParentNodePointer3_1(ExtractInstanceAddr(currBasePtr), prevNodePtr);
                }
            }

            // Reset ray on transition from BLAS to TLAS
            if (stackPtr & STACK_ADDR_BLAS_TO_TLAS_MASK)
            {
                // Intentionally recompute the original origin, instead of computing it once before the loop and reusing.
                // This recomputation is quite cheap, only occurs on a BLAS to TLAS transition, and can save 3 VGPRs.
                candidateRayDirection = data.hitObject.ray.direction;
                candidateRayOrigin    = data.hitObject.ray.origin + (data.hitObject.ray.tMin * candidateRayDirection);
                currBasePtr           = tlasBasePtr;

                data.traversal.instNodePtr   = INVALID_NODE;
                parentNodePtr                = INVALID_NODE;

                RayHistorySetWriteTopLevel(data);
                if (state >= TRAVERSAL_STATE_COMMITTED_NOTHING)
                {
                    // Write the top level when resumes for AHS and IS cases, otherwise write it immediately
                    RayHistoryWriteTopLevel(data);
                }
            }

            if (!useDeferMode && (state < TRAVERSAL_STATE_COMMITTED_NOTHING))
            {
                // Break out of traversal so that caller can process non-opaque triangle hits
                break;
            }
            if (useDeferMode && staticMayHaveCandidatePrims && WaveActiveAllTrue(haveDeferredCandidate))
            {
                // All lanes have a pending candidate, so we can uniformly break for candidate processing.
                // This is a trade-off: Continuing traversal has the potential to find opaque hit that are
                // closer than the pending candidate, allowing us to prune the candidate.
                // However, continuing does not have the benefit of improving Traversal coherency,
                // and risks running into another candidate, at which point we need to abort,
                // discarding the result of an expensive BVH instruction.
                break;
            }
        }
        while (IsValidNode(nextNodePtr));
    }

    if (useDeferMode && haveDeferredCandidate)
    {
        // This ignores TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_PROCEDURAL_PRIMITIVE -- nothing seems to depend on it.
        state = (deferredCandidateIsProcedural) ? TRAVERSAL_STATE_CANDIDATE_PROCEDURAL_PRIMITIVE
                                                : TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_TRIANGLE;
    }

    // Pack traversal results back into traversal state structure
    data.traversal.nextNodePtr             = nextNodePtr;
    data.traversal.committed               = committed;
    data.traversal.committedBarycentrics   = committedBarycentrics;
#ifndef REMAT_INSTANCE_RAY
    data.traversal.candidateRayOrigin      = candidateRayOrigin;
    data.traversal.candidateRayDirection   = candidateRayDirection;
#endif
    data.traversal.stackPtr                = stackPtr;
    data.traversal.reservedNodePtr         = lastNodePtr;
    if (IsBvhRebraid())
    {
        data.traversal.lastInstanceRootNodePtr = lastInstanceRootNodePtr;
    }
    data.traversal.PackParentPointer(parentNodePtr);
    data.traversal.PackInstanceContributionAndSkipTri0(instanceHitContribution, skipTri0);
}
