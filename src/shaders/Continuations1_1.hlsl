/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2020-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

//=====================================================================================================================
// RTIPv1.1 traversal stack implementation
//
struct TraversalStack
{
    uint ptr;
    uint numEntries;
    uint isGoingDown;

    void SetIsGoingDown(uint val)
    {
        isGoingDown = val;
    }

    bool IsEmpty()
    {
        return (numEntries == 0);
    }

    bool IsOverflowed()
    {
        return (ptr != AmdTraceRayGetStackBase());
    }

    void PushNodes(in uint4 nodes, in uint4 valid)
    {
        // Push nodes unconditionally. The valid bits passed in are
        //  x : not used
        //  y : is y valid (stackStride or 0)
        //  z : is z valid (stackStride or 0)
        //  w : is w valid (stackStride or 0)

        numEntries -= ptr;

        AmdTraceRayLdsWrite(ptr % AmdTraceRayGetStackSize(), nodes.w);
        ptr += valid.w;
        AmdTraceRayLdsWrite(ptr % AmdTraceRayGetStackSize(), nodes.z);
        ptr += valid.z;
        AmdTraceRayLdsWrite(ptr % AmdTraceRayGetStackSize(), nodes.y);
        ptr += valid.y;

        // update stack entry count (stack.numEntries += stack.ptrNew - stack.ptrOld)
        // Min is handled in pop path
        numEntries += ptr;
    }

    void Push(
        inout_param(uint) nextNodePtr,
        in    uint        lastNodePtr,
        in    uint4       intersectionResult)
    {
        const uint stackStride = AmdTraceRayGetStackStride();
        if (isGoingDown)
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
                PushNodes(intersectionResult, isValid);
            }

            // Traverse next node
            nextNodePtr = intersectionResult.x;
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
            nextNodePtr = gotoW ? intersectionResult.w : INVALID_NODE;
            nextNodePtr = gotoZ ? intersectionResult.z : nextNodePtr;
            nextNodePtr = gotoY ? intersectionResult.y : nextNodePtr;

            // Update traversal direction
            isGoingDown = 1;
        }
    }

    uint Pop()
    {
        // Update stack entry count
        numEntries = min(AmdTraceRayGetStackSize(), numEntries);

        ptr -= AmdTraceRayGetStackStride();
        numEntries -= AmdTraceRayGetStackStride();

        uint node = AmdTraceRayLdsRead(ptr % AmdTraceRayGetStackSize());

        return node;
    }

    uint Pack()
    {
        return (ptr | (numEntries << 16) | (isGoingDown << 31));
    }

    void Unpack(in uint packedStackPtr)
    {
        ptr         = (packedStackPtr >> 0)  & 0xffff;
        numEntries  = (packedStackPtr >> 16) & 0x7fff;
        isGoingDown = (packedStackPtr >> 31);
    }
};

//=====================================================================================================================
static _AmdTraversalState InitTraversalState1_1(
    uint     instanceInclusionMask,
    RayDesc  ray,
    bool     isValid)
{
    // Initialise traversal state to initial state
    _AmdTraversalState traversal = (_AmdTraversalState)0;

#if REMAT_INSTANCE_RAY == 0
    traversal.candidateRayOrigin    = ray.Origin + (ray.TMin * ray.Direction);
    traversal.candidateRayDirection = ray.Direction;
#endif
    traversal.committed.instNodePtr = INVALID_NODE;
    traversal.committed.rayTCurrent = ray.TMax - ray.TMin;

    uint schedulerState = TRAVERSAL_STATE_COMMITTED_NOTHING;
    traversal.committed.PackState(schedulerState);
#if AMD_VULKAN || GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP
    traversal.committed.currNodePtr = INVALID_NODE;
#endif

    // Start traversing from root node
    traversal.nextNodePtr = isValid ? CreateRootNodePointer1_1() : INVALID_NODE;
    traversal.reservedNodePtr         = INVALID_NODE;
    traversal.lastInstanceRootNodePtr = INVALID_NODE;

    // Initialise traversal stack pointers
    TraversalStack stack;
    stack.ptr         = AmdTraceRayGetStackBase();
    stack.isGoingDown = 1;
    stack.numEntries  = 0;

    traversal.stackPtr = stack.Pack();
    traversal.PackStackPtrTop(INVALID_NODE);

#if GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP
    traversal.committed.PackAnyHitCallType(0);
#endif

    return traversal;
}

//=====================================================================================================================
static void TraversalInternal1_1(
    inout_param(_AmdSystemData) data,
    inout_param(uint) state,
    inout_param(_AmdPrimitiveSystemState) candidate,
    inout_param(float2) candidateBarycentrics
)
{
    uint rayFlags = data.ray.Flags();

    // Initialise box sort mode based on ray flags and compile time constant flags (constRayFlags)
    uint boxSortMode = AmdTraceRayGetBoxSortHeuristicMode();
    if (boxSortMode == BoxSortHeuristic::DisabledOnAcceptFirstHit)
    {
        boxSortMode = GetBoxSortingHeuristicFromRayFlags(rayFlags, boxSortMode);
    }
    else
    {
        boxSortMode = (boxSortMode == BoxSortHeuristic::Disabled) ? BoxSortHeuristic::Disabled : BoxSortHeuristic::Closest;
    }

    // Root bvh address for reuse
    const GpuVirtualAddress topBvhAddress = data.ray.AccelStruct();
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
    float3 topLevelRayOrigin = data.ray.origin + (data.ray.tMin * data.ray.direction);
    float3 topLevelRayDirection = data.ray.direction;

    // Initialise transient traversal state
    uint lastNodePtr   = INVALID_NODE;
    uint instanceFlags = 0;
    uint instanceContributionToHitGroupIndex = 0;

    // Initialise local registers from traversal state. Note, using the large _AmdTraversalState register directly results
    // in spilling to scratch memory. To avoid this, copy traversal state into temporary registers

    // Unpack traversal stack pointer
    TraversalStack stack;
    stack.Unpack(data.traversal.stackPtr);

    _AmdPrimitiveSystemState committed = data.traversal.committed;
    candidate = (_AmdPrimitiveSystemState)0;
    float2 committedBarycentrics = data.traversal.committedBarycentrics;
    candidateBarycentrics = float2(0.0f, 0.0f);

    uint   nextNodePtr             = data.traversal.nextNodePtr;
    float3 candidateRayOrigin      = topLevelRayOrigin;
    float3 candidateRayDirection   = topLevelRayDirection;
    state                          = TRAVERSAL_STATE_COMMITTED_NOTHING;
    uint   stackPtrTop             = data.traversal.StackPtrTop();
    uint   lastInstanceRootNodePtr = data.traversal.lastInstanceRootNodePtr;

    if (data.traversal.instNodePtr != 0)
    {
        candidate.instNodePtr = data.traversal.instNodePtr;
        // Restore after AnyHit or Intersection was called
        const GpuVirtualAddress nodeAddr64  = data.ray.AccelStruct() + ExtractNodePointerOffset(data.traversal.instNodePtr);
        InstanceDesc desc                   = FetchInstanceDescAddr(nodeAddr64);
        bvhAddress                          = GetInstanceAddr(desc);
        instanceFlags                       = desc.InstanceContributionToHitGroupIndex_and_Flags >> 24;
        instanceContributionToHitGroupIndex = desc.InstanceContributionToHitGroupIndex_and_Flags & 0x00ffffff;

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
        bvhAddress = data.ray.AccelStruct();
    }

    // Traverse acceleration structure while we have valid nodes to intersect. Note, the traversal routine is
    // re-entrant and AnyHit shaders may set nextNodePtr to invalid when AcceptHitAndEndSearch() is called.
    while (IsValidNode(nextNodePtr))
    {
        // Backup last traversed node pointer
        uint nodePtr = nextNodePtr;

        // Pre-calculate 64-bit node address. Avoids recomputation inside the instance node block.
        const GpuVirtualAddress nodeAddr64 = bvhAddress + ExtractNodePointerOffset(nodePtr);

        // Perform ray-bvh node intersection
        uint4 intersectionResult =
            image_bvh64_intersect_ray(bvhAddress,
                                      nodePtr,
                                      boxSortMode,
                                      committed.rayTCurrent,
                                      candidateRayOrigin,
                                      candidateRayDirection,
                                      rcp(candidateRayDirection));

        // Trigger node search for next iteration
        nextNodePtr = INVALID_NODE;

        // Check if it is an internal node
        if (IsBoxNode1_1(nodePtr))
        {
            stack.Push(nextNodePtr, lastNodePtr, intersectionResult);
        }
        else if (CheckHandleTriangleNode(nodePtr))
        {
            const float t_num   = asfloat(intersectionResult.x);
            const float t_denom = asfloat(intersectionResult.y);

            // Determine triangle hit or miss
            const float candidateT = (t_num / t_denom);
            uint status = (candidateT < committed.rayTCurrent) ? HIT_STATUS_ACCEPT : HIT_STATUS_IGNORE;

            if (status == HIT_STATUS_ACCEPT)
            {
                // Evaluate triangle culling
                // Extract instance flags
                const bool instanceCullEnable = ((instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_CULL_DISABLE) == 0);
                const bool instanceFrontCW    = ((instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_FRONT_COUNTERCLOCKWISE) == 0);

                // If determinant is positive and front face winding is counterclockwise or vice versa, the triangle is back facing
                const bool backFacingTriangle = ((t_denom > 0.0f) ^ instanceFrontCW);

                // cull back/front facing triangle
                // computing these in the form of boolean expressions instead of ? :
                // makes it clearer to the compiler what is going on, and allows optimizing
                // the code to scalar and SGPRs.
                const bool cullBack = backFacingTriangle && triangleCullBackFace;
                const bool cullFront = !backFacingTriangle && triangleCullFrontFace;
                const bool cullTriangle = triangleCullEnable && instanceCullEnable && (cullBack || cullFront);

                // update triangle hit status
                status = cullTriangle ? HIT_STATUS_IGNORE : status;

                if (status != HIT_STATUS_IGNORE)
                {
                    const PrimitiveData primitiveData = FetchPrimitiveDataAddr(nodePtr, nodeAddr64);

                    const uint geometryFlags = primitiveData.geometryFlags;
                    const bool isOpaque      = IsOpaque(geometryFlags, instanceFlags, rayFlags);

                    // Computing in terms of logical expressions helps the compiler
                    // convert to scalar computaitons.
                    const bool cullOpaque = isOpaque && rayCullOpaque;
                    const bool cullNonOpaque = !isOpaque && rayCullNonOpaque;
                    const bool isCulled = cullOpaque || cullNonOpaque;

                    if (isCulled == false)
                    {
                        candidate.PackState(TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT);

                        candidate.rayTCurrent    = candidateT;
                        candidateBarycentrics.x  = asfloat(intersectionResult.z) / asfloat(intersectionResult.y);
                        candidateBarycentrics.y  = asfloat(intersectionResult.w) / asfloat(intersectionResult.y);
                        uint hitKind        = backFacingTriangle ? HIT_KIND_TRIANGLE_BACK_FACE : HIT_KIND_TRIANGLE_FRONT_FACE;
                        candidate.primitiveIndex = primitiveData.primitiveIndex;
                        candidate.PackHitKind(hitKind);
                        candidate.PackGeometryIndex(primitiveData.geometryIndex);
                        candidate.PackIsOpaque(isOpaque);
#if AMD_VULKAN || GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP
                        candidate.currNodePtr = nodePtr;
#endif

                        bool hasAnyHit = false;
                        if ((rayForceOpaque == false) && (isOpaque == false))
                        {
                            hasAnyHit = AnyHitIsNonNull(data.ray.traceParameters,
                                                        primitiveData.geometryIndex,
                                                        instanceContributionToHitGroupIndex);
                        }

                        // DXR spec: if there is no any hit shader, the geometry is considered opaque.
                        if (rayForceOpaque || isOpaque || (!hasAnyHit))
                        {
                            // Let the app know an opaque triangle hit was detected. The primitive index and geometry
                            // index are loaded after the traversal loop.
                            committed = candidate;
                            committedBarycentrics = candidateBarycentrics;
                            committed.PackState(TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT);

                            // Commit an opaque triangle hit
                            state = TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT;

                            // Exit traversal early if ray flags indicate end search after first hit
                            if (rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)
                            {
                                break;
                            }
                        }
                        else
                        {
                            // Let the app know a non opaque triangle hit was detected
                            state = TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_TRIANGLE;
                        }
                    }
                }
            }

           if ((AmdTraceRayGetTriangleCompressionMode() == PAIR_TRIANGLE_COMPRESSION) ||
               (AmdTraceRayGetTriangleCompressionMode() == AUTO_TRIANGLE_COMPRESSION))
            {
                if (GetNodeType(nodePtr) == NODE_TYPE_TRIANGLE_1)
                {
                    // Intersect with triangle 0 next
                    nextNodePtr = ClearNodeType(nodePtr);
                }
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
                candidate.PackState(TRAVERSAL_STATE_COMMITTED_PROCEDURAL_PRIMITIVE_HIT);
                candidate.primitiveIndex = primitiveData.primitiveIndex;
                candidate.PackGeometryIndex(primitiveData.geometryIndex);
                candidate.PackIsOpaque(isOpaque);
#if AMD_VULKAN || GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP
                candidate.currNodePtr = nodePtr;
#endif

#if GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP
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
        else  // NODE_TYPE_USER_NODE_INSTANCE
        {
            InstanceDesc desc = FetchInstanceDescAddr(nodeAddr64);

            // Avoid branching here to delay the wait on the fetched data
            candidate.instNodePtr      = nodePtr;
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

            bool isInstanceCulled = true;

            /// @note: Instance mask is 0 for null BLAS and it would be ignored
            const uint instanceInclusionMask = ExtractInstanceInclusionMask(data.ray.traceParameters);
            if ((desc.InstanceID_and_Mask >> 24) & instanceInclusionMask)
            {
                isInstanceCulled = CheckInstanceCulling(desc, rayFlags, AmdTraceRayGetStaticFlags());
            }

            if (!isInstanceCulled)
            {
                bvhAddress            = GetInstanceAddr(desc);
                candidate.instNodePtr = nodePtr;
                stackPtrTop           = stack.ptr;

                if (IsBvhRebraid())
                {
                    lastInstanceRootNodePtr = instanceNodePtr;
                    nextNodePtr             = instanceNodePtr;
                }
                else
                {
                    nextNodePtr = blasRootNodePtr;
                }
            }
            else
            {
                candidateRayOrigin    = topLevelRayOrigin;
                candidateRayDirection = topLevelRayDirection;
            }
        }

        // Can possibly compute next node to traverse just before exiting the traversal loop (similar to 1.0)
        if (nextNodePtr == INVALID_NODE)
        {
            // IsStackEmpty(), traverse to parent
            if (stack.IsEmpty())
            {
                // IsStackOverflowed()
                if (stack.IsOverflowed())
                {
                    uint instanceRootNodePtr = blasRootNodePtr;

                    if (IsBvhRebraid())
                    {
                        instanceRootNodePtr = lastInstanceRootNodePtr;
                    }

                    // bottom->top tree. traverse back up to find new node
                    if ((nodePtr == instanceRootNodePtr) && (bvhAddress != topBvhAddress))
                    {
                        bvhAddress = topBvhAddress;
                        nodePtr = candidate.instNodePtr;
                    }

                    nextNodePtr = FetchParentNodePointer(bvhAddress,nodePtr);
                    stack.SetIsGoingDown(0);
                    lastNodePtr = nodePtr;
                } // else traversal is complete, packedNodePointer stays invalid and the loop does not continue
            }
            else
            {
                if (stackPtrTop == stack.ptr)
                {
                    bvhAddress = topBvhAddress;
                }

                nextNodePtr = stack.Pop();
            }

            // Select ray based on tlas vs blas
            if (bvhAddress == topBvhAddress)
            {
                candidateRayOrigin = topLevelRayOrigin;
                candidateRayDirection = topLevelRayDirection;
                data.traversal.instNodePtr = 0;
            }
        }

        if (state < TRAVERSAL_STATE_COMMITTED_NOTHING)
        {
            // Break out of traversal so that caller can process non-opaque triangle hits
            break;
        }
    }

    // Repack the stack for continuation
    data.traversal.stackPtr = stack.Pack();

    // Pack traversal results back into traversal state structure
    data.traversal.nextNodePtr             = nextNodePtr;
    data.traversal.committed               = committed;
    data.traversal.committedBarycentrics   = committedBarycentrics;
#if REMAT_INSTANCE_RAY == 0
    data.traversal.candidateRayOrigin      = candidateRayOrigin;
    data.traversal.candidateRayDirection   = candidateRayDirection;
#endif
    data.traversal.PackStackPtrTop(stackPtrTop);
    data.traversal.lastInstanceRootNodePtr = lastInstanceRootNodePtr;
}
