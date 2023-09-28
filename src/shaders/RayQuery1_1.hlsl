/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
static GpuVirtualAddress GetRayQueryTopBvhAddress(in RayQueryInternal rayQuery)
{
    return MakeGpuVirtualAddress(rayQuery.topLevelBvhLo, rayQuery.topLevelBvhHi);
}

//=====================================================================================================================
static GpuVirtualAddress GetRayQueryBvhAddress(in RayQueryInternal rayQuery)
{
    return MakeGpuVirtualAddress(rayQuery.bvhLo, rayQuery.bvhHi);
}

//=====================================================================================================================
static void PushNodes(inout_param(RayQueryInternal) rayQuery, in uint4 nodes, in uint4 valid)
{
    // Push nodes unconditionally. The valid bits passed in are
    //  x : not used
    //  y : is y valid (stackStride or 0)
    //  z : is z valid (stackStride or 0)
    //  w : is w valid (stackStride or 0)

    rayQuery.stackNumEntries -= rayQuery.stackPtr;

    AmdTraceRayLdsWrite(rayQuery.stackPtr % AmdTraceRayGetStackSize(), nodes.w);
    rayQuery.stackPtr += valid.w;
    AmdTraceRayLdsWrite(rayQuery.stackPtr % AmdTraceRayGetStackSize(), nodes.z);
    rayQuery.stackPtr += valid.z;
    AmdTraceRayLdsWrite(rayQuery.stackPtr % AmdTraceRayGetStackSize(), nodes.y);
    rayQuery.stackPtr += valid.y;

    // update stack entry count (stack.numEntries += stackPtrNew - stackPtrOld)
    // Min is handled in pop path
    rayQuery.stackNumEntries += rayQuery.stackPtr;

#if DEVELOPER
    const uint maxStackDepth = GetRayQueryMaxStackDepth(rayQuery);
    SetRayQueryMaxStackDepth(rayQuery, max(maxStackDepth, rayQuery.stackPtr));
#endif
}

//=====================================================================================================================
// PopStack()
static uint PopStack(inout_param(RayQueryInternal) rayQuery)
{
    // Update stack entry count
    rayQuery.stackNumEntries = min(AmdTraceRayGetStackSize(), rayQuery.stackNumEntries);

    rayQuery.stackPtr        -= AmdTraceRayGetStackStride();
    rayQuery.stackNumEntries -= AmdTraceRayGetStackStride();

    uint node = AmdTraceRayLdsRead(rayQuery.stackPtr % AmdTraceRayGetStackSize());

    return node;
}

//=====================================================================================================================
// Process box intersection
static void ProcessBoxIntersection(
    inout_param(RayQueryInternal) rayQuery,
    in    uint4                   intersectionResult,
    in    uint                    lastNodePtr)
{
    const uint stackStride = AmdTraceRayGetStackStride();
    if (rayQuery.isGoingDown)
    {
        // Determine nodes to push to stack
        uint4 isValid;
        isValid.y = (intersectionResult.y != INVALID_NODE) ? stackStride : 0;

        // Push all nodes if we have at least one valid node
        // Sorting causes z and w to be invalid if y is invalid
        if (isValid.y)
        {
            isValid.w = (intersectionResult.w != INVALID_NODE) ? stackStride : 0;
            isValid.z = (intersectionResult.z != INVALID_NODE) ? stackStride : 0;
            PushNodes(rayQuery, intersectionResult, isValid);
        }

        // Traverse next node
        rayQuery.currNodePtr = intersectionResult.x;
    }
    else
    {
        // Determine sibling node to traverse if stackless
        // Don't need to check for valid here, since invalid nodes will cause a pop anyways
        // and there are no holes in the return list due to sorting.
        const bool gotoY = ComparePointers(intersectionResult.x, lastNodePtr);
        const bool gotoZ = ComparePointers(intersectionResult.y, lastNodePtr);
        const bool gotoW = ComparePointers(intersectionResult.z, lastNodePtr);

        // Switch to sibling node
        // Note, exactly one of the goto flags is set or none. If the last node is not a
        // traversal candidate we have to go search for a new node.
        rayQuery.currNodePtr = gotoW ? intersectionResult.w : INVALID_NODE;
        rayQuery.currNodePtr = gotoZ ? intersectionResult.z : rayQuery.currNodePtr;
        rayQuery.currNodePtr = gotoY ? intersectionResult.y : rayQuery.currNodePtr;

        // Update traversal direction
        rayQuery.isGoingDown = 1;
    }
}

//=====================================================================================================================
// TraceRayInline() implementation for ray tracing IP level 1.1
static void TraceRayInlineImpl1_1(
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

    const GpuVirtualAddress accelStruct = FetchAccelStructBaseAddr(accelStructLo, accelStructHi);

    rayQuery.bvhLo                 = LowPart(accelStruct);
    rayQuery.bvhHi                 = HighPart(accelStruct);
    rayQuery.topLevelBvhLo         = LowPart(accelStruct);
    rayQuery.topLevelBvhHi         = HighPart(accelStruct);

    // OR the templated ray flag with the dynamic ray flag function parameter
    rayQuery.rayFlags              = constRayFlags | rayFlags;
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
    rayQuery.isGoingDown           = 1;
    rayQuery.stackPtr              = AmdTraceRayGetStackBase();
    rayQuery.stackNumEntries       = 0;
    rayQuery.stackPtrTop           = INVALID_NODE;

    // Shift 8-bit instance mask into upper 8-bits to align with instance mask from instance node
    rayQuery.instanceInclusionMask = (instanceMask << 24);

    if (IsValidTrace(rayQuery.rayDesc, accelStruct, instanceMask, rayQuery.rayFlags, 0))
    {
        LogAccelStruct(accelStruct);
        rayQuery.currNodePtr = CreateRootNodePointer1_1();
    }
    else
    {
        // If acceleration structure is null or ray descriptor has nan values in it then set
        // prevNodePtr to invalid node which will quit traversal right away
        rayQuery.currNodePtr = INVALID_NODE;
    }

    // Initialise prevNodePtr to root node
    rayQuery.prevNodePtr = rayQuery.currNodePtr;

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        const uint rayId = GetRayId(dispatchThreadId);
        SetRayQueryDynamicId(rayQuery, AllocateRayHistoryDynamicId());
        WriteRayHistoryTokenBegin(rayId,
                                  dispatchThreadId,
                                  accelStruct,
                                  rayQuery.rayFlags,
                                  instanceMask,
                                  rayDesc,
                                  AmdTraceRayGetStaticId(),
                                  GetRayQueryDynamicId(rayQuery),
                                  -1);
    }
#endif
}

//=====================================================================================================================
// RayQuery::Proceed() implementation for IP level 1.1
static bool RayQueryProceedImpl1_1(
    inout_param(RayQueryInternal) rayQuery,
    in    uint                    constRayFlags,
    in    uint3                   dispatchThreadId)
{
#if DEVELOPER
    uint64_t timerBegin = SampleGpuTimer();
    uint rayId = 0;
    if (EnableTraversalCounter())
    {
        rayId = GetRayId(dispatchThreadId);
    }
#endif

    // Terminate traversal if RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH is set
    // and if a procedural primitive or triangle is already committed.
    if ((rayQuery.rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)    &&
        ((rayQuery.committedStatus == COMMITTED_PROCEDURAL_PRIMITIVE_HIT) ||
         (rayQuery.committedStatus == COMMITTED_TRIANGLE_HIT)))
    {
        return false;
    }

    // box sort mode
    uint boxSortMode = AmdTraceRayGetBoxSortHeuristicMode();
    if (boxSortMode == BoxSortHeuristic::DisabledOnAcceptFirstHit)
    {
        boxSortMode = GetBoxSortingHeuristicFromRayFlags(rayQuery.rayFlags, boxSortMode);
    }
    else
    {
        boxSortMode = (boxSortMode == BoxSortHeuristic::Disabled) ?
            BoxSortHeuristic::Disabled : BoxSortHeuristic::Closest;
    }

    uint continueTraversal = 0;

    // Root bvh address for reuse
    const GpuVirtualAddress topBvhAddress = GetRayQueryTopBvhAddress(rayQuery);
    // Updateable bottom level bvh for reuse
    GpuVirtualAddress bvhAddress          = GetRayQueryBvhAddress(rayQuery);

    // BLAS root node is always fp32 regardless of mode for fp16 box nodes
    const uint blasRootNodePtr = CreateRootNodePointer1_1();

    const bool rayForceOpaque    = (rayQuery.rayFlags & RAY_FLAG_FORCE_OPAQUE);
    const bool rayForceNonOpaque = (rayQuery.rayFlags & RAY_FLAG_FORCE_NON_OPAQUE);
    const bool rayCullOpaque     = (rayQuery.rayFlags & RAY_FLAG_CULL_OPAQUE);
    const bool rayCullNonOpaque  = (rayQuery.rayFlags & RAY_FLAG_CULL_NON_OPAQUE);
    const bool raySkipProcedural = (rayQuery.rayFlags & RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES);
    const bool triangleCullFrontFace = (rayQuery.rayFlags & RAY_FLAG_CULL_FRONT_FACING_TRIANGLES);
    const bool triangleCullBackFace  = (rayQuery.rayFlags & RAY_FLAG_CULL_BACK_FACING_TRIANGLES);
    const bool triangleCullEnable    = triangleCullFrontFace || triangleCullBackFace;

    const bool haveProceduralNodes     = !raySkipProcedural;
    const bool alwaysOpaque            = (rayForceOpaque || rayCullNonOpaque);
    const bool haveNonOpaquePrimitives = !alwaysOpaque;

    // If the app rejected a candidate then revert back to the current committed rayT
    rayQuery.candidate.rayTCurrent = rayQuery.committed.rayTCurrent;

    GpuVirtualAddress committedBvhAddress = 0;
    uint lastNodePtr = INVALID_NODE;
    // Note, when returning from user shader currNodePtr is INVALID_NODE. When a shader calls Abort(), the driver
    // sets stackNumEntries = 0, stackPtr = AmdTraceRayGetStackBase() to exit loop on next call to Proceed().
    // Otherwise, the traversal continues as normally.

    // Note trying to use any member variable of rayQuery structure in the exit condition results in a massive amount
    // of v_mov instructions generated by SC which results in a big performance drop. Needs further investigation.
    while (1)
    {
        if (rayQuery.currNodePtr == INVALID_NODE)
        {
            // IsStackEmpty(), traverse to parent
            if (rayQuery.stackNumEntries == 0)
            {
                // IsStackOverflowed()
                if (rayQuery.stackPtr != AmdTraceRayGetStackBase())
                {
                    uint instanceRootNodePtr = blasRootNodePtr;

                    if (IsBvhRebraid())
                    {
                        instanceRootNodePtr = rayQuery.lastInstanceNode;
                    }

                    // bottom->top tree. traverse back up to find new node
                    if ((rayQuery.prevNodePtr == instanceRootNodePtr) && (bvhAddress != topBvhAddress))
                    {
                        bvhAddress           = topBvhAddress;
                        rayQuery.prevNodePtr = rayQuery.instNodePtr;

                        rayQuery.candidate.origin    = rayQuery.rayDesc.Origin;
                        rayQuery.candidate.direction = rayQuery.rayDesc.Direction;
#if DEVELOPER
                        if (EnableTraversalCounter())
                        {
                            WriteRayHistoryTokenTopLevel(rayId, bvhAddress);
                        }
#endif
                    }

                    rayQuery.currNodePtr = FetchParentNodePointer(bvhAddress,
                                                                  rayQuery.prevNodePtr);
                    rayQuery.isGoingDown = 0;
                    lastNodePtr = rayQuery.prevNodePtr;
                } // else traversal is complete, packedNodePointer stays invalid and the loop does not continue
            }
            else
            {
                if (rayQuery.stackPtrTop == rayQuery.stackPtr)
                {
                    bvhAddress = topBvhAddress;

                    rayQuery.candidate.origin    = rayQuery.rayDesc.Origin;
                    rayQuery.candidate.direction = rayQuery.rayDesc.Direction;
#if DEVELOPER
                    if (EnableTraversalCounter())
                    {
                        WriteRayHistoryTokenTopLevel(rayId, bvhAddress);
                    }
#endif
                }

                rayQuery.currNodePtr = PopStack(rayQuery);
            }
        }

        if (rayQuery.currNodePtr == INVALID_NODE)
        {
            break;
        }

#if DEVELOPER
        if (EnableTraversalCounter())
        {
            rayQuery.numIterations++;
            WriteRayHistoryTokenNodePtr(rayId, rayQuery.currNodePtr);
            UpdateWaveTraversalStatistics(GPURT_RTIP1_1, rayQuery.currNodePtr);
        }
#endif
        // Backup last traversed node pointer
        rayQuery.prevNodePtr = rayQuery.currNodePtr;

        const uint nodePointer = ExtractNodePointer(rayQuery.currNodePtr);

        // pre-calculate node address
        const GpuVirtualAddress nodeAddr64 = bvhAddress + ExtractNodePointerOffset(nodePointer);

        uint4 intersectionResult =
            image_bvh64_intersect_ray(bvhAddress,
                                      nodePointer,
                                      boxSortMode,
                                      rayQuery.committed.rayTCurrent,
                                      rayQuery.candidate.origin,
                                      rayQuery.candidate.direction,
                                      rcp(rayQuery.candidate.direction));

        rayQuery.currNodePtr = INVALID_NODE;

        // Check if it is an internal node
        if (IsBoxNode1_1(rayQuery.prevNodePtr))
        {
#if DEVELOPER
            if (EnableTraversalCounter())
            {
                rayQuery.numRayBoxTest++;
            }
#endif
            ProcessBoxIntersection(rayQuery, intersectionResult, lastNodePtr);
        }
        else if (IsTriangleNode1_1(rayQuery.prevNodePtr))
        {
#if DEVELOPER
            if (EnableTraversalCounter())
            {
                rayQuery.numRayTriangleTest++;
            }
#endif

            const float t_num = asfloat(intersectionResult.x);
            const float t_denom = asfloat(intersectionResult.y);
            const float candidateT = (t_num / t_denom);

            if (candidateT < rayQuery.committed.rayTCurrent)
            {
                uint status = HIT_STATUS_ACCEPT;

                // Evaluate triangle culling
                // Extract instance flags
                const uint instanceFlags      = rayQuery.instanceHitContributionAndFlags >> 24;
                const bool instanceCullEnable = ((instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_CULL_DISABLE) == 0);
                const bool instanceFrontCCW   = (instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_FRONT_COUNTERCLOCKWISE);

                // If determinant is positive and front face winding is counterclockwise or vice versa, the triangle is back facing
                const uint backFacingTriangle = ((intersectionResult.y >> 31) ^ instanceFrontCCW);
                uint hitKind = backFacingTriangle | HIT_KIND_TRIANGLE_FRONT_FACE;

                // Most likely a compile time constant. Evaluate first to allow entire branch to get compiled out
                if (triangleCullEnable)
                {
                    if (instanceCullEnable)
                    {
                        // cull back/front facing triangle
                        const bool cullTriangle = backFacingTriangle ? triangleCullBackFace : triangleCullFrontFace;

                        // update triangle hit status
                        status = cullTriangle ? HIT_STATUS_IGNORE : status;
                    }
                }

                if (status != HIT_STATUS_IGNORE)
                {
                    const PrimitiveData primitiveData = FetchPrimitiveDataAddr(nodePointer, nodeAddr64);

                    const uint geometryFlags          = primitiveData.geometryFlags;
                    const uint isOpaque               = IsOpaque(geometryFlags, instanceFlags, rayQuery.rayFlags);

                    const bool isCulled = (isOpaque ? rayCullOpaque : rayCullNonOpaque);

                    if (isCulled == false)
                    {
                        rayQuery.candidate.currNodePtr          = nodePointer;
                        rayQuery.candidate.instanceContribution = rayQuery.instanceHitContributionAndFlags & 0x00FFFFFF;

                        // Execute anyhit shader for built-in triangle intersection
                        rayQuery.candidate.rayTCurrent    = candidateT;
                        rayQuery.candidate.barycentrics.x = asfloat(intersectionResult.z) / asfloat(intersectionResult.y);
                        rayQuery.candidate.barycentrics.y = asfloat(intersectionResult.w) / asfloat(intersectionResult.y);
                        rayQuery.candidate.frontFace      = backFacingTriangle ^ 1;

                        if (isOpaque)
                        {
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

                            committedBvhAddress = bvhAddress;

                            // Commit an opaque triangle hit
                            rayQuery.committedStatus = COMMITTED_TRIANGLE_HIT;

                            // Exit traversal early if ray flags indicate end search after first hit
                            if (rayQuery.rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)
                            {
                                break;
                            }
                        }
                        else
                        {
                            // Let the app know a non opaque triangle hit was detected
                            rayQuery.candidateType   = CANDIDATE_NON_OPAQUE_TRIANGLE;

                            rayQuery.candidate.primitiveIndex = primitiveData.primitiveIndex;
                            rayQuery.candidate.geometryIndex  = primitiveData.geometryIndex;

                            // Leave traversal loop and notify app a candidate is ready
                            continueTraversal        = 1;
                        }

#if DEVELOPER
                        if (EnableTraversalCounter() && (status != HIT_STATUS_IGNORE))
                        {
                            rayQuery.numCandidateHits++;
                        }
#endif
                    }
                }
            }

            if (IsBvhCollapse())
            {
                if (ExtractPrimitiveCount(rayQuery.prevNodePtr) != 0)
                {
                    rayQuery.currNodePtr = IncrementNodePointer(rayQuery.prevNodePtr);
                }
            }
            else if ((AmdTraceRayGetTriangleCompressionMode() == PAIR_TRIANGLE_COMPRESSION) ||
                     (AmdTraceRayGetTriangleCompressionMode() == AUTO_TRIANGLE_COMPRESSION))
            {
                if (GetNodeType(rayQuery.prevNodePtr) == NODE_TYPE_TRIANGLE_1)
                {
                    // Intersect with triangle 0 next
                    rayQuery.currNodePtr = ClearNodeType(rayQuery.prevNodePtr);
                }
            }
        }
        else if (haveProceduralNodes && IsUserNodeProcedural(rayQuery.prevNodePtr))
        {
            // Extract instance flags
            const uint instanceFlags = rayQuery.instanceHitContributionAndFlags >> 24;

            const PrimitiveData primitiveData = FetchPrimitiveDataAddr(nodePointer, nodeAddr64);

            const uint geometryFlags = primitiveData.geometryFlags;
            const uint isOpaque      = IsOpaque(geometryFlags, instanceFlags, rayQuery.rayFlags);
            const bool isCulled      = (isOpaque ? rayCullOpaque : rayCullNonOpaque);

            if (isCulled == false)
            {
                rayQuery.candidate.primitiveIndex       = primitiveData.primitiveIndex;
                rayQuery.candidate.geometryIndex        = primitiveData.geometryIndex;
                rayQuery.candidate.currNodePtr          = nodePointer;
                rayQuery.candidate.instanceContribution = rayQuery.instanceHitContributionAndFlags & 0x00FFFFFF;

                // Handle intersection shader and anyhit (if exists in hitgroup)
                // Let the app know a procedural primitive hit was detected
                rayQuery.candidateType = (isOpaque == false) ?
                                         CANDIDATE_NON_OPAQUE_PROCEDURAL_PRIMITIVE :
                                         CANDIDATE_PROCEDURAL_PRIMITIVE;

                // Leave traversal loop and notify app a candidate is ready
                continueTraversal                  = 1;

#if DEVELOPER
                if (EnableTraversalCounter())
                {
                    rayQuery.numCandidateHits++;
                }
#endif
            }
        }
        else  // NODE_TYPE_USER_NODE_INSTANCE
        {
#if DEVELOPER
            if (EnableTraversalCounter())
            {
                rayQuery.instanceIntersections++;
            }
#endif
            InstanceDesc desc = FetchInstanceDescAddr(nodeAddr64);

            // Avoid branching here to delay the wait on the fetched data
            rayQuery.candidate.instNodePtr = rayQuery.prevNodePtr;
            // Fetch instance node pointer (for rebraid) here to allow for the global loads to be
            // coalesced into a single loadx2
            const uint instanceNodePtr               = FetchInstanceNodePointerAddr(nodeAddr64);
            rayQuery.instanceHitContributionAndFlags = desc.InstanceContributionToHitGroupIndex_and_Flags;

            InstanceTransform(desc,
                              rayQuery.rayDesc.Origin,
                              rayQuery.rayDesc.Direction,
                              rayQuery.candidate.origin,
                              rayQuery.candidate.direction);

            bool isInstanceCulled = true;

            /// @note: Instance mask is 0 for null BLAS and it would be ignored
            if (desc.InstanceID_and_Mask & rayQuery.instanceInclusionMask)
            {
                isInstanceCulled = CheckInstanceCulling(desc, rayQuery.rayFlags, constRayFlags);
            }

            if (!isInstanceCulled)
            {
                bvhAddress           = GetInstanceAddr(desc);
                rayQuery.instNodePtr = rayQuery.prevNodePtr;
                rayQuery.stackPtrTop = rayQuery.stackPtr;

                if (IsBvhRebraid())
                {
                    rayQuery.lastInstanceNode = instanceNodePtr;
                    rayQuery.currNodePtr      = instanceNodePtr;
                }
                else
                {
                    rayQuery.currNodePtr = blasRootNodePtr;
                }

#if DEVELOPER
                if (EnableTraversalCounter())
                {
                    WriteRayHistoryTokenBottomLevel(rayId, bvhAddress);
                }
#endif
            }
            else
            {
                rayQuery.candidate.origin    = rayQuery.rayDesc.Origin;
                rayQuery.candidate.direction = rayQuery.rayDesc.Direction;
            }
        }

        // RayFlag check ensures that continueTraversal will never be read as long as the following RayFlags are set:
        // Early return is possible only if there are procedural or non opaque primitives to process.
        if ((haveProceduralNodes || haveNonOpaquePrimitives) &&
             continueTraversal)
        {
            break;
        }
    }

    if (committedBvhAddress != 0)
    {
        const PrimitiveData primitiveData = FetchPrimitiveData(committedBvhAddress, rayQuery.committed.currNodePtr);

        rayQuery.committed.primitiveIndex = primitiveData.primitiveIndex;
        rayQuery.committed.geometryIndex  = primitiveData.geometryIndex;
    }

    rayQuery.bvhLo = LowPart(bvhAddress);
    rayQuery.bvhHi = HighPart(bvhAddress);

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        uint64_t timerEnd = SampleGpuTimer();
        rayQuery.clocks += (uint)(timerEnd - timerBegin);
    }
#endif

    return continueTraversal;
}
