/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
// TraceRayInline() implementation for ray tracing IP level 2.0
static void TraceRayInlineImpl2_0(
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

    rayQuery.stackPtr = 0;
#if USE_HW_INTRINSIC
    rayQuery.stackPtr = AmdTraceRayLdsStackInit();
#else
    // SW stack emulation
    rayQuery.stackPtr = init_ds_store_stack();
#endif

    rayQuery.stackPtrTop           = 0;

    // Shift 8-bit instance mask into upper 8-bits to align with instance mask from instance node
    rayQuery.instanceInclusionMask = (instanceMask << 24);

    // Initialize pointerFlags, and store it at the bvhHi
    // Encode api ray flags into pointer flags resetting accept_first_hit & skip_closest_hit which do not apply here.
    const uint rayFlagsSetBits = (rayQuery.rayFlags & (~RAY_FLAG_EXCLUDE_MASK)) << POINTER_FLAGS_HIDWORD_SHIFT;
    uint pointerFlags = rayFlagsSetBits;
    rayQuery.bvhHi |= pointerFlags;

    if (IsValidTrace(rayQuery.rayDesc, accelStruct, instanceMask, rayQuery.rayFlags, 0))
    {
        LogAccelStruct(accelStruct);
        rayQuery.currNodePtr = CreateRootNodePointer1_1();
    }
    else
    {
        // If acceleration structure is null or ray descriptor has nan values in it then set
        // prevNodePtr to TERMINAL_NODE which will quit traversal right away
        rayQuery.currNodePtr = TERMINAL_NODE;
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
// RayQuery::Proceed() implementation for IP level 2.0
static bool RayQueryProceedImpl2_0(
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

    uint boxHeuristicMode = AmdTraceRayGetBoxSortHeuristicMode();
    if ((boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosest) ||
        (boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosestMidPoint))
    {
        boxHeuristicMode = GetBoxSortingHeuristicFromRayFlags(rayQuery.rayFlags, boxHeuristicMode);
    }

    uint continueTraversal = 0;

    // Root bvh address for reuse
    const GpuVirtualAddress topBvhAddress = GetRayQueryTopBvhAddress(rayQuery);
    // Updateable bottom level bvh for reuse
    GpuVirtualAddress bvhAddress          =
        MakeGpuVirtualAddress(rayQuery.bvhLo, (rayQuery.bvhHi & POINTER_FLAGS_EXCLUDED_MASK));

    // BLAS root node is always fp32 regardless of mode for fp16 box nodes
    const uint blasRootNodePtr = CreateRootNodePointer1_1();

    const bool rayForceOpaque           = (rayQuery.rayFlags & RAY_FLAG_FORCE_OPAQUE);
    const bool rayCullOpaque            = (rayQuery.rayFlags & RAY_FLAG_CULL_OPAQUE);
    const bool rayCullNonOpaque         = (rayQuery.rayFlags & RAY_FLAG_CULL_NON_OPAQUE);
    const bool raySkipProcedural        = (rayQuery.rayFlags & RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES);
    const bool triangleCullFrontFace    = (rayQuery.rayFlags & RAY_FLAG_CULL_FRONT_FACING_TRIANGLES);
    const bool triangleCullBackFace     = (rayQuery.rayFlags & RAY_FLAG_CULL_BACK_FACING_TRIANGLES);

    const bool haveProceduralNodes      = !raySkipProcedural;
    bool alwaysOpaque                   = (rayForceOpaque || rayCullNonOpaque);
    bool haveNonOpaquePrimitives        = !alwaysOpaque;

    // Encode api ray flags into pointer flags resetting accept_first_hit & skip_closest_hit which do not apply here.
    const uint rayFlagsSetBits = (rayQuery.rayFlags & (~RAY_FLAG_EXCLUDE_MASK)) << POINTER_FLAGS_HIDWORD_SHIFT;

    // Handles the overriding of (rayFlags.ForceOpaque && instanceFlags.ForceNonOpaque) or
    //                           (rayFlags.ForceNonOpaque && instanceFlags.ForceOpaque)
    // Other flags can just be or-ed together
    uint instanceFlagsPreserveBits =
        (rayQuery.rayFlags & RAY_FLAG_OVERRIDE_MASK) ? RAY_FLAG_PRESERVE_MASK : RAY_FLAG_VALID_MASK;

    instanceFlagsPreserveBits <<= POINTER_FLAGS_HIDWORD_SHIFT;

    // If the app rejected a candidate then revert back to the current committed rayT
    rayQuery.candidate.rayTCurrent = rayQuery.committed.rayTCurrent;

    GpuVirtualAddress committedBvhAddress = 0;
    uint lastNodePtr  = INVALID_NODE;

    uint pointerFlags = rayQuery.bvhHi & POINTER_FLAGS_VALID_MASK;

    // Note trying to use any member variable of rayQuery structure in the exit condition results in a massive amount
    // of v_mov instructions generated by SC which results in a big performance drop. Needs further investigation.
    while (1)
    {
        // BLAS->TLAS transition condition
        bool resetRay = (rayQuery.stackPtr < rayQuery.stackPtrTop);

        // Combine all low frequency conditions in one path to reduce branches on typical loop iterations
        if (resetRay || (rayQuery.currNodePtr >= TERMINAL_NODE))
        {
            // TERMINAL_NODE is the traversal end condition. It is returned from LDS when the stack is empty or set
            // by the driver when the shader calls Abort().
            if (rayQuery.currNodePtr == TERMINAL_NODE)
            {
                break;
            }

            // Perform stackless walkback to find a valid node
            if (rayQuery.currNodePtr == INVALID_NODE)
            {
                // Add one to the stackAddr, to reset it back up from the stackless pop
                rayQuery.stackPtr++;

                // Recompute the resetRay using the new stackAddr
                resetRay = (rayQuery.stackPtr < rayQuery.stackPtrTop);

                const uint lastRootNode = IsBvhRebraid() ? rayQuery.lastInstanceNode : blasRootNodePtr;

                // Switch prevNodePtr to the instance node in the TLAS if the last tested node was a BLAS root node
                if ((rayQuery.prevNodePtr == lastRootNode) && (bvhAddress != topBvhAddress))
                {
                    rayQuery.prevNodePtr = rayQuery.instNodePtr;
                }

                // prevNodePtr will still point to the TLAS instance node if an instance is culled. We also need to
                // switch back to the TLAS in this case.
                if (rayQuery.prevNodePtr == rayQuery.instNodePtr)
                {
                    resetRay   = true;
                    bvhAddress = topBvhAddress;
                }

                rayQuery.currNodePtr = FetchParentNodePointer(bvhAddress, rayQuery.prevNodePtr);

                if (rayQuery.currNodePtr == INVALID_NODE)
                {
                    break;
                }

                // Note, we evaluate the compile time flag first to avoid redundant code in stackless path when
                // triangle compression is disabled.
                const bool trianglePairIntersected =
                    (AmdTraceRayGetTriangleCompressionMode() != NO_TRIANGLE_COMPRESSION) &&
                    (GetNodeType(rayQuery.prevNodePtr) == NODE_TYPE_TRIANGLE_1);

                lastNodePtr = trianglePairIntersected ? (rayQuery.prevNodePtr + 1) : rayQuery.prevNodePtr;
            }

            if (resetRay)
            {
                bvhAddress = topBvhAddress;

                rayQuery.candidate.origin    = rayQuery.rayDesc.Origin;
                rayQuery.candidate.direction = rayQuery.rayDesc.Direction;

                rayQuery.stackPtrTop = 0;
                pointerFlags         = rayFlagsSetBits;

                resetRay = false;
#if DEVELOPER
                if (EnableTraversalCounter())
                {
                    WriteRayHistoryTokenTopLevel(rayId, bvhAddress);
                }
#endif
            }
        }

#if DEVELOPER
        if (EnableTraversalCounter())
        {
            rayQuery.numIterations++;
            WriteRayHistoryTokenNodePtr(rayId, rayQuery.currNodePtr);
            UpdateWaveTraversalStatistics(GPURT_RTIP2_0, rayQuery.currNodePtr);
        }
#endif
        // Backup last traversed node pointer
        rayQuery.prevNodePtr = rayQuery.currNodePtr;

        uint nodePointer = ExtractNodePointer(rayQuery.currNodePtr);

        // pre-calculate node address
        const GpuVirtualAddress nodeAddr64 = bvhAddress + ExtractNodePointerOffset(nodePointer);

        // Convert fused instance node pointer to Float32BoxNode
        if (EnableFusedInstanceNodes())
        {
            // The nodePointer consists of a packed 64-byte aligned offset and 3-bit node type. Adding 0xF
            // to the node pointer increments this aligned offset by 2 (i.e. 128 bytes); 1 from the 4th bit
            // from LSB and another 1 from the carry (0x6 + 0x7). The node type as a result is set to 0x5
            // i.e. NODE_TYPE_BOX_FLOAT32.
            const bool isUserNodeInstance = IsUserNodeInstance(rayQuery.prevNodePtr);
            nodePointer += isUserNodeInstance ? 0xF : 0;
        }

        uint4 intersectionResult =
            image_bvh64_intersect_ray_2_0(bvhAddress,
                                          nodePointer,
                                          pointerFlags,
                                          boxHeuristicMode,
                                          rayQuery.committed.rayTCurrent,
                                          rayQuery.candidate.origin,
                                          rayQuery.candidate.direction,
                                          rcp(rayQuery.candidate.direction));

        rayQuery.currNodePtr = INVALID_NODE;

        // Check if it is an internal node
        if (IsTriangleNode1_1(rayQuery.prevNodePtr))
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
                // Extract instance flags
                const uint instanceFlags = rayQuery.instanceHitContributionAndFlags >> 24;

                const PrimitiveData primitiveData = FetchPrimitiveDataAddr(nodePointer, nodeAddr64);

                const uint geometryFlags = primitiveData.geometryFlags;

                const bool isNonOpaque =
                    ((pointerFlags >> POINTER_FLAGS_HIDWORD_SHIFT) & RAY_FLAG_FORCE_NON_OPAQUE) ||
                    ((((pointerFlags >> POINTER_FLAGS_HIDWORD_SHIFT) & RAY_FLAG_FORCE_OPAQUE) == 0) &&
                     ((geometryFlags & D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE) == 0));

                rayQuery.candidate.currNodePtr          = nodePointer;
                rayQuery.candidate.instanceContribution = rayQuery.instanceHitContributionAndFlags & 0x00FFFFFF;

                rayQuery.candidate.rayTCurrent    = candidateT;
                rayQuery.candidate.barycentrics.x = asfloat(intersectionResult.z) / asfloat(intersectionResult.y);
                rayQuery.candidate.barycentrics.y = asfloat(intersectionResult.w) / asfloat(intersectionResult.y);
                rayQuery.candidate.frontFace      = (intersectionResult.y >> 31) ^ 1;

                if (alwaysOpaque || (isNonOpaque == false))
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
                    rayQuery.candidateType = CANDIDATE_NON_OPAQUE_TRIANGLE;

                    rayQuery.candidate.primitiveIndex = primitiveData.primitiveIndex;
                    rayQuery.candidate.geometryIndex  = primitiveData.geometryIndex;

                    // Leave traversal loop and notify app a candidate is ready
                    continueTraversal = 1;
                }
#if DEVELOPER
                if (EnableTraversalCounter())
                {
                    rayQuery.numCandidateHits++;
                }
#endif
            }

            if (IsBvhCollapse())
            {
                if (ExtractPrimitiveCount(rayQuery.prevNodePtr) != 0)
                {
                    const uint nextNodePtr = IncrementNodePointer(rayQuery.prevNodePtr);

                    // Manually update the next node pointer, and then set up so that it will pass through the
                    // ds_store_stack instruction without modifying the stack (assumes lastNodePtr == INVALID_NODE).
                    intersectionResult = uint4(nextNodePtr, INVALID_NODE, INVALID_NODE, INVALID_NODE);
                }
                else
                {
                    lastNodePtr = TERMINAL_NODE;
                }
            }
            else if ((AmdTraceRayGetTriangleCompressionMode() == PAIR_TRIANGLE_COMPRESSION) ||
                     (AmdTraceRayGetTriangleCompressionMode() == AUTO_TRIANGLE_COMPRESSION))
            {
                // prevNodePtr is guaranteed to be NODE_TYPE_TRIANGLE_0 or NODE_TYPE_TRIANGLE_1
                if (rayQuery.prevNodePtr & 0x1)
                {
                    // Intersect with triangle 0 next
                    const uint nextNodePtr = ClearNodeType(rayQuery.prevNodePtr);

                    // Manually update the next node pointer, and then set up so that it will pass through the
                    // ds_store_stack instruction without modifying the stack (assumes lastNodePtr == INVALID_NODE).
                    intersectionResult = uint4(nextNodePtr, INVALID_NODE, INVALID_NODE, INVALID_NODE);
                }
                else
                {
                    lastNodePtr = TERMINAL_NODE;
                }
            }
            else
            {
                // Disallow any nodes from being pushed into the stack when set as the lastNodePtr
                // This works because the ds_store_stack opcode will discard all pushes if none of
                // the intersectionResults match the lastNodePtr and lastNodePtr is not INVALID_NODE.
                // The value TERMINAL_NODE(0xfffffffe) is guaranteed to never be in the intersection results
                // Because as a floating point value it encodes a type of signaling NaN that is not generated
                // by our FP units and does not encode a possible nodePointer, hitStatus or triangleId return value.

                lastNodePtr = TERMINAL_NODE;
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

            // Setting lastNodePtr to TERMINAL_NODE prevents any of the intersection_results being pushed onto the stack.
            lastNodePtr = TERMINAL_NODE;
        }
        else if (IsUserNodeInstance(rayQuery.prevNodePtr)) // NODE_TYPE_USER_NODE_INSTANCE
        {
#if DEVELOPER
            if (EnableTraversalCounter())
            {
                rayQuery.instanceIntersections++;
            }
#endif
            const InstanceDesc desc = FetchInstanceDescAddr(nodeAddr64);

            rayQuery.candidate.instNodePtr = rayQuery.prevNodePtr;
            rayQuery.instNodePtr = rayQuery.prevNodePtr;

            // Fetch instance node pointer (for rebraid) here to allow for the global loads to be coalesced into a
            // single loadx2
            const uint instanceNodePtr               = FetchInstanceNodePointerAddr(nodeAddr64);
            rayQuery.instanceHitContributionAndFlags = desc.InstanceContributionToHitGroupIndex_and_Flags;

            InstanceTransform(desc,
                              rayQuery.rayDesc.Origin,
                              rayQuery.rayDesc.Direction,
                              rayQuery.candidate.origin,
                              rayQuery.candidate.direction);

            const uint instanceFlagsSetBits = (desc.accelStructureAddressHiAndFlags & POINTER_FLAGS_VALID_MASK);
            pointerFlags = (instanceFlagsSetBits & instanceFlagsPreserveBits) | rayFlagsSetBits;

            bvhAddress = GetInstanceAddr(desc);

            // Instance mask is 0 for null BLAS and it would be ignored
            bool isInstanceCulled = ((desc.InstanceID_and_Mask & rayQuery.instanceInclusionMask) == 0);

            if (EnableFusedInstanceNodes())
            {
                // Intersection would return [-1, -1, -1, -1] for a miss. Mark instance as culled if that occurs.
                isInstanceCulled |= (intersectionResult.x == INVALID_NODE);
                lastNodePtr = isInstanceCulled ? TERMINAL_NODE : lastNodePtr;

                if (IsBvhRebraid())
                {
                    rayQuery.lastInstanceNode = instanceNodePtr;
                }
            }
            else
            {
                if (IsBvhRebraid())
                {
                    rayQuery.lastInstanceNode = instanceNodePtr;
                    intersectionResult.x      = isInstanceCulled ? INVALID_NODE : instanceNodePtr;
                }
                else
                {
                    intersectionResult.x = isInstanceCulled ? INVALID_NODE : blasRootNodePtr;
                }
            }

            // Setting stackPtrTop to INVALID_IDX forces an immediate BLAS->TLAS transition when an instance is culled,
            // reverting the updates above.
            // Setting it to stackAddr postpones the BLAS->TLAS transition until we pop a BLAS node from the stack.
            rayQuery.stackPtrTop = isInstanceCulled ? INVALID_IDX : rayQuery.stackPtr;

#if DEVELOPER
            if (EnableTraversalCounter() && (isInstanceCulled == false))
            {
                WriteRayHistoryTokenBottomLevel(rayId, bvhAddress);
            }
#endif
        }

#if DEVELOPER
        else if (EnableTraversalCounter() && IsBoxNode1_1(rayQuery.prevNodePtr))
        {
            rayQuery.numRayBoxTest++;
        }
#endif

#if USE_HW_INTRINSIC
        rayQuery.currNodePtr = AmdTraceRayLdsStackStore(rayQuery.stackPtr, lastNodePtr, intersectionResult);
#else
        rayQuery.currNodePtr = ds_store_stack(rayQuery.stackPtr, lastNodePtr, intersectionResult);
#endif

        lastNodePtr = INVALID_NODE;

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
    rayQuery.bvhHi = HighPart(bvhAddress) | pointerFlags;

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        uint64_t timerEnd = SampleGpuTimer();
        rayQuery.clocks += (uint)(timerEnd - timerBegin);
    }
#endif

    return continueTraversal;
}
