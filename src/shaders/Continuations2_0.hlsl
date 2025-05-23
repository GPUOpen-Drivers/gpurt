/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2020-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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

#include "bvh_stack_dxr_prototype.hlsl"

//=====================================================================================================================
static _AmdTraversalState InitTraversalState2_0(
    uint     instanceInclusionMask,
    RayDesc  ray,
    bool     isValid)
{
    // Initialise traversal state to initial state
    _AmdTraversalState traversal    = (_AmdTraversalState)0;

#if REMAT_INSTANCE_RAY == 0
    traversal.candidateRayOrigin    = ray.Origin + (ApplyTMinBias(ray.TMin) * ray.Direction);
    traversal.candidateRayDirection = ray.Direction;
#endif
    traversal.committed.instNodePtr = INVALID_NODE;
    traversal.committed.rayTCurrent = ray.TMax - ApplyTMinBias(ray.TMin);

    uint schedulerState = TRAVERSAL_STATE_COMMITTED_NOTHING;
    traversal.committed.PackState(schedulerState);
    traversal.committed.SetCurrNodePtr(INVALID_NODE);

    // Start traversing from root node
    traversal.reservedNodePtr         = INVALID_NODE;
    traversal.lastInstanceRootNodePtr = INVALID_NODE;

    // Initialise traversal stack pointers
#if USE_HW_INTRINSIC
    traversal.stackPtr = AmdTraceRayLdsStackInit();
#else
    traversal.stackPtr = init_ds_store_stack();
#endif

    traversal.PackStackPtrTop(0);

#if GPURT_DEBUG_CONTINUATION_TRAVERSAL
    traversal.committed.PackAnyHitCallType(0);
#endif

    return traversal;
}

//=====================================================================================================================
static void TraversalInternal2_0(
    inout_param(_AmdSystemData) data,
    inout_param(uint) state,
    inout_param(_AmdPrimitiveSystemState) candidate,
    inout_param(float2) candidateBarycentrics)
{
    const uint rayFlags         = data.hitObject.ray.Flags();
    const uint boxHeuristicMode = GetBoxHeuristicMode();

    // Root bvh address for reuse
    const GpuVirtualAddress topBvhAddress = data.hitObject.ray.AccelStruct();
    // Updateable bottom level bvh for reuse
    GpuVirtualAddress bvhAddress;

    const uint blasRootNodePtr = CreateRootNodePointer1_1();

    const bool rayForceOpaque        = (rayFlags & RAY_FLAG_FORCE_OPAQUE);
    const bool rayForceNonOpaque     = (rayFlags & RAY_FLAG_FORCE_NON_OPAQUE);
    const bool rayCullOpaque         = (rayFlags & RAY_FLAG_CULL_OPAQUE);
    const bool rayCullNonOpaque      = (rayFlags & RAY_FLAG_CULL_NON_OPAQUE);
    const bool raySkipProcedural     = (rayFlags & RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES);
    const bool triangleCullFrontFace = (rayFlags & RAY_FLAG_CULL_FRONT_FACING_TRIANGLES);
    const bool triangleCullBackFace  = (rayFlags & RAY_FLAG_CULL_BACK_FACING_TRIANGLES);
    const bool triangleCullEnable    = triangleCullFrontFace || triangleCullBackFace;

    // Regenerate top-level ray for traversal from ray system state
    float3 topLevelRayOrigin = data.hitObject.ray.origin + (ApplyTMinBias(data.hitObject.ray.tMin) * data.hitObject.ray.direction);
    float3 topLevelRayDirection = data.hitObject.ray.direction;

    // Initialise transient traversal state
    uint lastNodePtr   = data.traversal.reservedNodePtr;
    uint instanceFlags = 0;
    uint instanceContributionToHitGroupIndex = 0;

    // Initialise local registers from traversal state. Note, using the large _AmdTraversalState register directly results
    // in spilling to scratch memory. To avoid this, copy traversal state into temporary registers

    _AmdPrimitiveSystemState committed = data.traversal.committed;
    candidate = (_AmdPrimitiveSystemState)0;
    const bool useDeferMode = (Options::getCpsCandidatePrimitiveMode() == CpsCandidatePrimitiveMode::DeferFirst);
    bool haveDeferredCandidate = false;
    float2 committedBarycentrics = data.traversal.committedBarycentrics;
    candidateBarycentrics = float2(0.0f, 0.0f);

    // Encode api ray flags into pointer flags resetting accept_first_hit & skip_closest_hit which do not apply here.
    const uint rayFlagsSetBits   = (rayFlags & (~RAY_FLAG_EXCLUDE_MASK)) << POINTER_FLAGS_HIDWORD_SHIFT;

    // Handles the overriding of (rayFlags.ForceOpaque && instanceFlags.ForceNonOpaque) or
    //                           (rayFlags.ForceNonOpaque && instanceFlags.ForceOpaque)
    // Other flags can just be or-ed together
    uint instanceFlagsPreserveBits =
        (rayFlags & RAY_FLAG_OVERRIDE_MASK) ? RAY_FLAG_PRESERVE_MASK : RAY_FLAG_VALID_MASK;

    instanceFlagsPreserveBits <<= POINTER_FLAGS_HIDWORD_SHIFT;

    uint   nextNodePtr             = data.traversal.nextNodePtr;
    float3 candidateRayOrigin      = topLevelRayOrigin;
    float3 candidateRayDirection   = topLevelRayDirection;
    state                          = TRAVERSAL_STATE_COMMITTED_NOTHING;
    uint   stackPtr                = data.traversal.stackPtr;
    uint   stackPtrTop             = data.traversal.StackPtrTop();
    uint   lastInstanceRootNodePtr = data.traversal.lastInstanceRootNodePtr;
    uint   pointerFlags            = rayFlagsSetBits;

    if (data.traversal.instNodePtr != 0)
    {
        // Restore after AnyHit or Intersection was called
        const GpuVirtualAddress nodeAddr64  = data.hitObject.ray.AccelStruct() + ExtractNodePointerOffset(data.traversal.instNodePtr);
        InstanceDesc desc                   = FetchInstanceDescAddr(nodeAddr64);
        bvhAddress                          = GetInstanceAddr(desc);
        instanceFlags                       = desc.InstanceContributionToHitGroupIndex_and_Flags >> 24;
        instanceContributionToHitGroupIndex = desc.InstanceContributionToHitGroupIndex_and_Flags & 0x00ffffff;

        const uint instanceFlagsSetBits = (desc.accelStructureAddressHiAndFlags & POINTER_FLAGS_VALID_MASK);
        pointerFlags = (instanceFlagsSetBits & instanceFlagsPreserveBits) | rayFlagsSetBits;

#if REMAT_INSTANCE_RAY
        InstanceTransform(desc,
                          topLevelRayOrigin,
                          topLevelRayDirection,
                          candidateRayOrigin,
                          candidateRayDirection);
#else
        candidateRayOrigin    = data.traversal.candidateRayOrigin;
        candidateRayDirection = data.traversal.candidateRayDirection;
#endif
    }
    else
    {
        bvhAddress = data.hitObject.ray.AccelStruct();
    }

    // Shift 8-bit instance mask into upper 8-bits to align with instance mask from instance node
    const uint instanceInclusionMask = ExtractInstanceInclusionMask(data.hitObject.ray.traceParameters) << 24;

    // Traverse acceleration structure while we have valid nodes to intersect. Note, the traversal routine is
    // re-entrant and AnyHit shaders may set nextNodePtr to invalid when AcceptHitAndEndSearch() is called.
    while (IsValidNode(nextNodePtr))
    {
        RayHistoryHandleIteration(data, nextNodePtr);

        // Backup last traversed node pointer
        uint nodePtr = nextNodePtr;

        // Using a boolean here generates a scalar mask of lanes which intersected a triangle pair node.
        // Note, we evaluate the compile time flag first to avoid redundant code in stackless path when
        // triangle compression is disabled.
        const bool trianglePairIntersected =
            (AmdTraceRayGetTriangleCompressionMode() != NO_TRIANGLE_COMPRESSION) &&
                (GetNodeType(nodePtr) == NODE_TYPE_TRIANGLE_1);

        // Pre-calculate 64-bit node address. Avoids recomputation inside the instance node block.
        const GpuVirtualAddress nodeAddr64 = bvhAddress + ExtractNodePointerOffset(nodePtr);

        // Backup node pointer for performing ray-bvh node intersection
        uint nodePointerForIntersect = nextNodePtr;

        // Convert fused instance node pointer to Float32BoxNode
        if (EnableFusedInstanceNodes())
        {
            // The nodePointer consists of a packed 64-byte aligned offset and 3-bit node type. Adding 0xF
            // to the node pointer increments this aligned offset by 2 (i.e. 128 bytes); 1 from the 4th bit
            // from LSB and another 1 from the carry (0x6 + 0x7). The node type as a result is set to 0x5
            // i.e. NODE_TYPE_BOX_FLOAT32.
            const bool isUserNodeInstance = IsUserNodeInstance(nodePtr);
            nodePointerForIntersect += isUserNodeInstance ? 0xF : 0;
        }

        // Perform ray-bvh node intersection
        uint4 intersectionResult =
            image_bvh64_intersect_ray_2_0(bvhAddress,
                                          nodePointerForIntersect,
                                          pointerFlags,
                                          boxHeuristicMode,
                                          committed.rayTCurrent,
                                          candidateRayOrigin,
                                          candidateRayDirection,
                                          rcp(candidateRayDirection));

        if (IsBoxNode1_1(nodePtr))
        {
            RayHistoryIncNumRayBoxTest(data);
        }

        if (IsUserNodeInstance(nodePtr))
        {
            RayHistoryIncInstanceIntersections(data);

            InstanceDesc desc = FetchInstanceDescAddr(nodeAddr64);

            data.traversal.instNodePtr = nodePtr;
            // Fetch instance node pointer (for rebraid) here to allow for the global loads to be
            // coalesced into a single loadx2
            const uint instanceNodePtr          = FetchInstanceNodePointerAddr(nodeAddr64);
            instanceFlags                       = desc.InstanceContributionToHitGroupIndex_and_Flags >> 24;
            instanceContributionToHitGroupIndex = desc.InstanceContributionToHitGroupIndex_and_Flags & 0x00ffffff;

            InstanceTransform(desc,
                              topLevelRayOrigin,
                              topLevelRayDirection,
                              candidateRayOrigin,
                              candidateRayDirection);

            const uint instanceFlagsSetBits = (desc.accelStructureAddressHiAndFlags & POINTER_FLAGS_VALID_MASK);
            pointerFlags = (instanceFlagsSetBits & instanceFlagsPreserveBits) | rayFlagsSetBits;

            bvhAddress = GetInstanceAddr(desc);

            bool isInstanceCulled = ((desc.InstanceID_and_Mask & instanceInclusionMask) == 0);

            if (EnableFusedInstanceNodes())
            {
                // Intersection would return [-1, -1, -1, -1] for a miss. Mark instance as culled if that occurs.
                isInstanceCulled |= (intersectionResult.x == INVALID_NODE);
                lastNodePtr = isInstanceCulled ? TERMINAL_NODE : lastNodePtr;

                if (IsBvhRebraid())
                {
                    lastInstanceRootNodePtr = instanceNodePtr;
                }
            }
            else
            {
                if (IsBvhRebraid())
                {
                    lastInstanceRootNodePtr = instanceNodePtr;
                    intersectionResult.x = isInstanceCulled ? INVALID_NODE : instanceNodePtr;
                }
                else
                {
                    intersectionResult.x = isInstanceCulled ? INVALID_NODE : blasRootNodePtr;
                }
            }

            // Setting the stackPtrTop to INVALID_NODE forces an immediate BLAS->TLAS transition
            // Reverting the transforms done above when an instance is culled.
            // Setting to stackAddr postpones the BLAS->TLAS transition until we pop a BLAS node from the stack.
            stackPtrTop = isInstanceCulled ? INVALID_NODE : stackPtr;

            if (isInstanceCulled == false)
            {
                RayHistoryWriteBottomLevel(data, bvhAddress);
            }
        }

        // Check if it is an triangle node
        if (CheckHandleTriangleNode(nodePtr))
        {
            RayHistoryIncNumRayTriangleTest(data);

            const float t_num   = asfloat(intersectionResult.x);
            const float t_denom = asfloat(intersectionResult.y);

            // Determine triangle hit or miss
            const float candidateT = (t_num / t_denom);
            if (EvaluateTriangleHit(data.hitObject.ray.tMin, candidateT, committed.rayTCurrent))
            {
                RayHistorySetCandidateTCurrent(data, candidateT);

                const PrimitiveData primitiveData = FetchPrimitiveDataAddr(nodePtr, nodeAddr64);
                const uint geometryFlags = primitiveData.geometryFlags;

                const bool isOpaque = !(((pointerFlags >> POINTER_FLAGS_HIDWORD_SHIFT) & RAY_FLAG_FORCE_NON_OPAQUE) ||
                                        ((((pointerFlags >> POINTER_FLAGS_HIDWORD_SHIFT) & RAY_FLAG_FORCE_OPAQUE) == 0) &&
                                         ((geometryFlags & D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE) == 0)));

                bool hasAnyHit = false;
                if ((rayForceOpaque == false) && (isOpaque == false))
                {
                    hasAnyHit = AnyHitIsNonNull(data.hitObject.ray.traceParameters,
                                                primitiveData.geometryIndex,
                                                instanceContributionToHitGroupIndex);
                }

                uint hitKind        = (intersectionResult.y >> 31) | HIT_KIND_TRIANGLE_FRONT_FACE;
                float2 barycentrics = { asfloat(intersectionResult.z) / asfloat(intersectionResult.y),
                                        asfloat(intersectionResult.w) / asfloat(intersectionResult.y) };

                // DXR spec: if there is no any hit shader, the geometry is considered opaque.
                if (rayForceOpaque || isOpaque || (!hasAnyHit))
                {
                    // Commit opaque triangle hit
                    RayHistoryWriteTriangleHitResult(data, true);

                    committed.instNodePtr    = data.traversal.instNodePtr;
                    committed.rayTCurrent    = candidateT;
                    committedBarycentrics    = barycentrics;
                    committed.primitiveIndex = primitiveData.primitiveIndex;
                    committed.PackInstanceContribution(instanceContributionToHitGroupIndex, hitKind);
                    committed.PackGeometryIndex(primitiveData.geometryIndex,
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
                        /* isTri0 */ 0,
#endif
                        TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT, false);
                    committed.SetCurrNodePtr(nodePtr);

                    state = TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT;

                    // Exit traversal early if ray flags indicate end search after first hit
                    if (rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)
                    {
                        break;
                    }

                    if (useDeferMode && haveDeferredCandidate && candidate.rayTCurrent > candidateT)
                    {
                        // There is a pending candidate, but the new committed hit is closer, so we can discard it.
                        candidate = (_AmdPrimitiveSystemState)0;
                        haveDeferredCandidate = false;
                        candidateBarycentrics = float2(0.0f, 0.0f);
                    }
                }
                else
                {
                    // Need to run AHS on triangle hit
                    if (useDeferMode && haveDeferredCandidate)
                    {
                       // There already is a pending candidate. Need to break out of Traversal to process the pending hit,
                       // so we can process this hit here on Traversal resume.
                       break;
                    }

                    candidate.instNodePtr    = data.traversal.instNodePtr;
                    candidate.rayTCurrent    = candidateT;
                    candidateBarycentrics    = barycentrics;
                    candidate.primitiveIndex = primitiveData.primitiveIndex;
                    candidate.PackInstanceContribution(instanceContributionToHitGroupIndex, hitKind);
                    candidate.PackGeometryIndex(primitiveData.geometryIndex,
                    // This #ifdef is required until the legacy GPURT_RTIP_LEVEL == 0 lib has been removed:
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
                        /* isTri0 */ 0,
#endif
                        TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT, isOpaque);
                    candidate.SetCurrNodePtr(nodePtr);

                    if (useDeferMode)
                    {
                        haveDeferredCandidate = true;
                    }
                    else
                    {
                        state = TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_TRIANGLE;
                    }
                }
            }

            if ((AmdTraceRayGetTriangleCompressionMode() == PAIR_TRIANGLE_COMPRESSION) ||
                (AmdTraceRayGetTriangleCompressionMode() == AUTO_TRIANGLE_COMPRESSION))
            {
                if (GetNodeType(nodePtr) == NODE_TYPE_TRIANGLE_1)
                {
                    // Intersect with triangle 0 next
                    const uint nextPairNodePtr = ClearNodeType(nodePtr);

                    intersectionResult = uint4(nextPairNodePtr, INVALID_NODE, INVALID_NODE, INVALID_NODE);
                }
                else
                {
                    lastNodePtr = TERMINAL_NODE;
                }
            }
            else
            {
                lastNodePtr = TERMINAL_NODE;
            }
        }
        else if (CheckHandleProceduralUserNode(nodePtr))
        {
            const PrimitiveData primitiveData = FetchPrimitiveDataAddr(nodePtr, nodeAddr64);

            const uint geometryFlags = primitiveData.geometryFlags;
            const uint isOpaque      = IsOpaque(geometryFlags, instanceFlags, rayFlags);
            const bool isCulled      = (isOpaque ? rayCullOpaque : rayCullNonOpaque);

            if (isCulled == false)
            {

                if (useDeferMode && haveDeferredCandidate)
                {
                   // There already is a pending candidate. Need to break out of Traversal to process the pending hit,
                   // so we can process this hit here on Traversal resume.
                   break;
                }

                candidate.PackState(TRAVERSAL_STATE_COMMITTED_PROCEDURAL_PRIMITIVE_HIT);
                candidate.primitiveIndex = primitiveData.primitiveIndex;
                candidate.PackGeometryIndex(primitiveData.geometryIndex);
                candidate.PackIsOpaque(isOpaque);
                candidate.PackInstanceContribution(instanceContributionToHitGroupIndex);
                candidate.SetCurrNodePtr(nodePtr);
                candidate.instNodePtr = data.traversal.instNodePtr;

#if GPURT_DEBUG_CONTINUATION_TRAVERSAL
                // Determine anyHit shader call type
                uint anyHitCallType = rayForceOpaque ? ANYHIT_CALLTYPE_SKIP : ANYHIT_CALLTYPE_DUPLICATE;

                const bool noDuplicateAnyHit = (geometryFlags & D3D12_RAYTRACING_GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION);
                anyHitCallType = noDuplicateAnyHit ? ANYHIT_CALLTYPE_NO_DUPLICATE : anyHitCallType;
                if (isOpaque)
                {
                    // Skip calling any hit shader for opaque nodes
                    anyHitCallType = ANYHIT_CALLTYPE_SKIP;
                }
                candidate.PackAnyHitCallType(anyHitCallType);
#endif

                // Handle intersection shader and anyhit (if exists in hitgroup)
                // Let the app know a procedural primitive hit was detected
                state = (isOpaque == false) ? TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_PROCEDURAL_PRIMITIVE :
                                              TRAVERSAL_STATE_CANDIDATE_PROCEDURAL_PRIMITIVE;
            }
        }

#if USE_HW_INTRINSIC
        nextNodePtr = AmdTraceRayLdsStackStore(stackPtr, lastNodePtr, intersectionResult);
#else
        // SW stack emulation
        nextNodePtr = ds_store_stack(stackPtr, lastNodePtr, intersectionResult);
#endif

        lastNodePtr = INVALID_NODE;

        // BLAS->TLAS Transition condition
        bool resetRay = (stackPtr < stackPtrTop);

        RayHistorySetMaxStackDepth(data, stackPtr);

        // Can possibly compute next node to traverse just before exiting the traversal loop (similar to 1.0)
        if (resetRay || (nextNodePtr == INVALID_NODE))
        {
            if (nextNodePtr == INVALID_NODE)
            {
                // Add one to the stackAddr, to reset it back up from the stackless pop
                stackPtr++;

                // Recompute the resetRay using the new stackAddr
                resetRay = (stackPtr < stackPtrTop);

                const uint lastRootNode = IsBvhRebraid() ? lastInstanceRootNodePtr : blasRootNodePtr;

                // Switch the node pointer to the instance node in the TLAS if the last tested node was a BLAS root node.
                if ((nodePtr == lastRootNode) && (bvhAddress != topBvhAddress))
                {
                    nodePtr = data.traversal.instNodePtr;
                }

                // data.traversal.instNodePtr will still point to the TLAS instance node if an instance is culled. We also need to
                // switch back to the TLAS in this case.
                if (nodePtr == data.traversal.instNodePtr)
                {
                    bvhAddress = topBvhAddress;
                    resetRay = true;
                }

                nextNodePtr = FetchParentNodePointer(bvhAddress, nodePtr);

                if (trianglePairIntersected)
                {
                    lastNodePtr = nodePtr + 1;
                }
                else
                {
                    lastNodePtr = nodePtr;
                }
            }

            if (resetRay)
            {
                bvhAddress  = topBvhAddress;
                stackPtrTop = 0;

                pointerFlags               = rayFlagsSetBits;
                candidateRayOrigin         = topLevelRayOrigin;
                candidateRayDirection      = topLevelRayDirection;
                data.traversal.instNodePtr = 0;

                RayHistorySetWriteTopLevel(data);
                if (state >= TRAVERSAL_STATE_COMMITTED_NOTHING)
                {
                    // Write the top level when resumes for AHS and IS cases, otherwise write it immediately
                    RayHistoryWriteTopLevel(data);
                }
            }
        }

        bool laneHasCandidate = (state < TRAVERSAL_STATE_COMMITTED_NOTHING);
        if (laneHasCandidate)
        {
            // Break out of traversal to run AHS/IS
            break;
        }
        if (useDeferMode && WaveActiveAllTrue(haveDeferredCandidate))
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

    if (useDeferMode && haveDeferredCandidate)
    {
        state = TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_TRIANGLE;
    }

    // Pack traversal results back into traversal state structure
    data.traversal.nextNodePtr             = nextNodePtr;
    data.traversal.committed               = committed;
    data.traversal.committedBarycentrics   = committedBarycentrics;
#if REMAT_INSTANCE_RAY == 0
    data.traversal.candidateRayOrigin      = candidateRayOrigin;
    data.traversal.candidateRayDirection   = candidateRayDirection;
#endif
    data.traversal.stackPtr                = stackPtr;
    data.traversal.PackStackPtrTop(stackPtrTop);
    data.traversal.lastInstanceRootNodePtr = lastInstanceRootNodePtr;
    // Last node pointer intersected for handling pair compression stackless bug
    data.traversal.reservedNodePtr         = lastNodePtr;
}
