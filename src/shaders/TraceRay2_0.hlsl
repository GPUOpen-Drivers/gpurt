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
#include "bvh_stack_dxr_prototype.hlsl"

//=====================================================================================================================
static IntersectionResult TraceRayImpl2_0(
    in GpuVirtualAddress topLevelBvh,               ///< Top-level acceleration structure to use
    in uint              rayFlags,                  ///< Ray flags
    in uint              traceRayParameters,        ///< Packed trace ray parameters
    in RayDesc           ray,                       ///< Ray to be traced
    in uint              rayId                      ///< Ray ID for profiling
)
{
#if DEVELOPER
    const uint64_t rayTraceStartTime = SampleGpuTimer();
#endif

    uint boxHeuristicMode = AmdTraceRayGetBoxSortHeuristicMode();
    if ((boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosest) ||
        (boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosestMidPoint))
    {
        boxHeuristicMode = GetBoxSortingHeuristicFromRayFlags(rayFlags, boxHeuristicMode);
    }

    // Shift 8-bit instance mask into upper 8-bits to align with instance mask from instance node
    const uint instanceInclusionMask = (ExtractInstanceInclusionMask(traceRayParameters) << 24);

    // Temporary locals related to intersection result
    uint instanceHitContributionAndFlags;

    // Temporary locals for traversal
    RayDesc localRay = ray;
    localRay.Origin += localRay.TMin * localRay.Direction;
    RayDesc topLevelRay = localRay;
    GpuVirtualAddress currentBvh = topLevelBvh;

    // Initialise intersection result
    IntersectionResult intersection = (IntersectionResult)0;
    intersection.t                  = ray.TMax - ray.TMin;
    intersection.nodeIndex          = INVALID_IDX;

    // Start from root node which follows acceleration structure header
    // BLAS root node is always fp32 regardless of mode for fp16 box nodes
    const uint blasRootNodePtr   = CreateRootNodePointer1_1();
    uint       packedNodePointer = blasRootNodePtr;
    uint       tlasNodePtr       = INVALID_IDX;

    uint prevNodePtr   = INVALID_IDX;
    uint stackPtrTop   = 0;
    uint traversalMode = 0;
    uint instNodePtr;

    uint lastInstanceNode = INVALID_IDX;

    // OR compile time pipeline flags into the ray flags so they are handled by hardware culling
    rayFlags |=
        (AmdTraceRayGetStaticFlags() & (PIPELINE_FLAG_SKIP_PROCEDURAL_PRIMITIVES | PIPELINE_FLAG_SKIP_TRIANGLES));

    const bool rayForceOpaque        = (rayFlags & RAY_FLAG_FORCE_OPAQUE);
    const bool cullOpaque            = (rayFlags & RAY_FLAG_CULL_OPAQUE);
    const bool cullNonOpaque         = (rayFlags & RAY_FLAG_CULL_NON_OPAQUE);
    const bool triangleCullFrontFace = (rayFlags & RAY_FLAG_CULL_FRONT_FACING_TRIANGLES);
    const bool triangleCullBackFace  = (rayFlags & RAY_FLAG_CULL_BACK_FACING_TRIANGLES);

    // Encode api ray flags into pointer flags resetting accept_first_hit & skip_closest_hit which do not apply here.
    const uint rayFlagsSetBits = (rayFlags & (~RAY_FLAG_EXCLUDE_MASK)) << POINTER_FLAGS_HIDWORD_SHIFT;

    // Handles the overriding of (rayFlags.ForceOpaque && instanceFlags.ForceNonOpaque) or
    //                           (rayFlags.ForceNonOpaque && instanceFlags.ForceOpaque)
    // Other flags can just be or-ed together
    uint instanceFlagsPreserveBits =
        (rayFlags & RAY_FLAG_OVERRIDE_MASK) ? RAY_FLAG_PRESERVE_MASK : RAY_FLAG_VALID_MASK;

    instanceFlagsPreserveBits <<= POINTER_FLAGS_HIDWORD_SHIFT;

    // Last visited node pointer used to record which node was prior for stackless back tracking
    uint lastNodePtr = INVALID_NODE;

    // stack marker for top level node
    uint stackAddr = 0;
#if USE_HW_INTRINSIC
    stackAddr = AmdTraceRayLdsStackInit();
#else
    // SW stack emulation
   stackAddr = init_ds_store_stack();
#endif

    uint pointerFlags = rayFlagsSetBits;

#if DEVELOPER
    while ((packedNodePointer < TERMINAL_NODE) && (intersection.numIterations < DispatchRaysConstBuf.profileMaxIterations))
#else
    while (packedNodePointer < TERMINAL_NODE)
#endif
    {
#if DEVELOPER
        if (EnableTraversalCounter())
        {
            WriteRayHistoryTokenNodePtr(rayId, packedNodePointer);
            UpdateWaveTraversalStatistics(GPURT_RTIP2_0, packedNodePointer);
        }
#endif

#if DEVELOPER
        intersection.numIterations++;
#endif

        // Using a boolean here generates a scalar mask of lanes which intersected a triangle pair node.
        // Note, we evaluate the compile time flag first to avoid redundant code in stackless path when
        // triangle compression is disabled.
        const bool trianglePairIntersected =
            (AmdTraceRayGetTriangleCompressionMode() != NO_TRIANGLE_COMPRESSION) &&
                (GetNodeType(prevNodePtr) == NODE_TYPE_TRIANGLE_1);

        // Backup last traversed node pointer
        prevNodePtr = packedNodePointer;

        uint nodePointer = ExtractNodePointer(packedNodePointer);

        // pre-calculate node address
        const GpuVirtualAddress nodeAddr64 = currentBvh + ExtractNodePointerOffset(nodePointer);

        // Convert fused instance node pointer to Float32BoxNode
        if (EnableFusedInstanceNodes())
        {
            // The nodePointer consists of a packed 64-byte aligned offset and 3-bit node type. Adding 0xF
            // to the node pointer increments this aligned offset by 2 (i.e. 128 bytes); 1 from the 4th bit
            // from LSB and another 1 from the carry (0x6 + 0x7). The node type as a result is set to 0x5
            // i.e. NODE_TYPE_BOX_FLOAT32.
            const bool isUserNodeInstance = IsUserNodeInstance(prevNodePtr);
            nodePointer += isUserNodeInstance ? 0xF : 0;
        }

        uint4 intersectionResult =
            image_bvh64_intersect_ray_2_0(currentBvh,
                                          nodePointer,
                                          pointerFlags,
                                          boxHeuristicMode,
                                          intersection.t,
                                          localRay.Origin,
                                          localRay.Direction,
                                          rcp(localRay.Direction));
#if DEVELOPER
        if (EnableTraversalCounter() && IsBoxNode1_1(prevNodePtr))
        {
            intersection.numRayBoxTest++;
        }
#endif

        if (IsUserNodeInstance(prevNodePtr)) // NODE_TYPE_USER_NODE_INSTANCE
        {
#if DEVELOPER
            if (EnableTraversalCounter())
            {
                intersection.instanceIntersections++;
            }
#endif
            const InstanceDesc desc = FetchInstanceDescAddr(nodeAddr64);

            instNodePtr = prevNodePtr;

            const uint instanceNodePtr = FetchInstanceNodePointerAddr(nodeAddr64);
            instanceHitContributionAndFlags = desc.InstanceContributionToHitGroupIndex_and_Flags;

            InstanceTransform(desc,
                              topLevelRay.Origin,
                              topLevelRay.Direction,
                              localRay.Origin,
                              localRay.Direction);

            const uint instanceFlagsSetBits = (desc.accelStructureAddressHiAndFlags & POINTER_FLAGS_VALID_MASK);
            pointerFlags = (instanceFlagsSetBits & instanceFlagsPreserveBits) | rayFlagsSetBits;

            currentBvh = GetInstanceAddr(desc);

            bool isInstanceCulled = ((desc.InstanceID_and_Mask & instanceInclusionMask) == 0);

            if (EnableFusedInstanceNodes())
            {
                // Intersection would return [-1, -1, -1, -1] for a miss. Mark instance as culled if that occurs.
                isInstanceCulled |= (intersectionResult.x == INVALID_NODE);
                lastNodePtr = isInstanceCulled ? TERMINAL_NODE : lastNodePtr;

                if (IsBvhRebraid())
                {
                    lastInstanceNode = instanceNodePtr;
                }
            }
            else
            {
                if (IsBvhRebraid())
                {
                    lastInstanceNode     = instanceNodePtr;
                    intersectionResult.x = isInstanceCulled ? INVALID_NODE : instanceNodePtr;
                }
                else
                {
                    intersectionResult.x = isInstanceCulled ? INVALID_NODE : blasRootNodePtr;
                }
            }

            // Setting stackPtrTop to INVALID_IDX forces an immediate BLAS->TLAS transition when an instance is culled,
            // reverting the updates above.
            // Setting it to stackAddr postpones the BLAS->TLAS transition until we pop a BLAS node from the stack.
            stackPtrTop = isInstanceCulled ? INVALID_IDX : stackAddr;

#if DEVELOPER
            if (EnableTraversalCounter() && (isInstanceCulled == false))
            {
                WriteRayHistoryTokenBottomLevel(rayId, currentBvh);
            }
#endif
        }

        if (CheckHandleTriangleNode(prevNodePtr))
        {
#if DEVELOPER
            if (EnableTraversalCounter())
            {
                intersection.numRayTriangleTest++;
            }
#endif
            const float t_num = asfloat(intersectionResult.x);
            const float t_denom = asfloat(intersectionResult.y);
            float candidateT = (t_num / t_denom);

            if (candidateT < intersection.t)
            {
                uint status = HIT_STATUS_ACCEPT;

                // A negative t_denom denotes back face hit (0xFF), front face otherwise (0xFE).
                uint hitKind = (intersectionResult.y >> 31) | HIT_KIND_TRIANGLE_FRONT_FACE;

                // Check for ray flags first. If forced opaque (known at compile time), this entire block is skipped by the compiler
                if (rayForceOpaque == false)
                {
                    // load primitive data
                    const PrimitiveData primitiveData = FetchPrimitiveDataAddr(nodePointer, nodeAddr64);

                    const uint primitiveIndex = primitiveData.primitiveIndex;
                    const uint geometryIndex  = primitiveData.geometryIndex;
                    const uint geometryFlags  = primitiveData.geometryFlags;

                    const bool doAnyHit =
                        ((pointerFlags >> POINTER_FLAGS_HIDWORD_SHIFT) & RAY_FLAG_FORCE_NON_OPAQUE) ||
                        ((((pointerFlags >> POINTER_FLAGS_HIDWORD_SHIFT) & RAY_FLAG_FORCE_OPAQUE) == 0) &&
                            ((geometryFlags & D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE) == 0));

                    // call anyhit shader for non-opaque triangles
                    if (doAnyHit)
                    {
                        const uint instanceContribution = (instanceHitContributionAndFlags & 0x00FFFFFF);
                        HitGroupInfo hitInfo = GetHitGroupInfo(ExtractRayContributionToHitIndex(traceRayParameters),
                                                                ExtractMultiplierForGeometryContributionToHitIndex(traceRayParameters),
                                                                geometryIndex,
                                                                instanceContribution);

                        const uint64_t instNodePtr64 =
                            CalculateInstanceNodePtr64(GPURT_RTIP2_0, topLevelBvh, instNodePtr);
#if DEVELOPER
                        if (EnableTraversalCounter())
                        {
                            WriteRayHistoryTokenFunctionCall(rayId,
                                                             hitInfo.anyHitId,
                                                             hitInfo.tableIndex,
                                                             RAY_HISTORY_FUNC_CALL_TYPE_ANY_HIT);
                            intersection.numAnyHitInvocation++;
                        }
#endif
                        // Set intersection attributes
                        AmdTraceRaySetHitAttributes(candidateT,
                                                    hitKind,
                                                    HIT_STATUS_ACCEPT,
                                                    LowPart(instNodePtr64),
                                                    HighPart(instNodePtr64),
                                                    primitiveIndex,
                                                    ANYHIT_CALLTYPE_NO_DUPLICATE,
                                                    geometryIndex);

                        // Set hit triangle information
                        AmdTraceRaySetHitTriangleNodePointer(currentBvh, nodePointer);

                        // get barycentrics
                        float2 barycentrics;
                        barycentrics.x = asfloat(intersectionResult.z) / asfloat(intersectionResult.y);
                        barycentrics.y = asfloat(intersectionResult.w) / asfloat(intersectionResult.y);

                        // Call triangle anyhit shader.
                        BuiltInTriangleIntersectionAttributes attr = { barycentrics };
                        AmdTraceRayCallTriangleAnyHitShader(hitInfo.anyHitId, hitInfo.tableIndex, attr);

                        // Returns hit attributes post intersection/anyhit shader call. "candidateT" and "hitKind" are only
                        // updated if the hit was accepted and "status" reflects that decision
                        AmdTraceRayGetHitAttributes(candidateT, hitKind, status);
#if DEVELOPER
                        if (EnableTraversalCounter())
                        {
                            WriteRayHistoryTokenAnyHitStatus(rayId, status);
                        }
#endif
                    }
                }

                if (status != HIT_STATUS_IGNORE)
                {
                    intersection.barycentrics.x    = asfloat(intersectionResult.z) / asfloat(intersectionResult.y);
                    intersection.barycentrics.y    = asfloat(intersectionResult.w) / asfloat(intersectionResult.y);
                    intersection.t                 = candidateT;
                    intersection.nodeIndex         = nodePointer;
                    intersection.hitkind           = hitKind;
                    tlasNodePtr                    = instNodePtr;

                    if ((rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH) ||
                        (status == HIT_STATUS_ACCEPT_AND_END_SEARCH))
                    {
                        break;
                    }
                }
            }

            if (IsBvhCollapse())
            {
                if (ExtractPrimitiveCount(prevNodePtr) != 0)
                {
                    packedNodePointer = IncrementNodePointer(prevNodePtr);

                    // Manually update the next node pointer, and then set up so that it will pass through the
                    // ds_store_stack instruction without modifying the stack (assumes lastNodePtr == INVALID_NODE).
                    intersectionResult = uint4(packedNodePointer, INVALID_NODE, INVALID_NODE, INVALID_NODE);
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
                if (prevNodePtr & 0x1)
                {
                    // Intersect with triangle 0 next
                    packedNodePointer = ClearNodeType(prevNodePtr);

                    // Manually update the next node pointer, and then set up so that it will pass through the
                    // ds_store_stack instruction without modifying the stack (assumes lastNodePtr == INVALID_NODE).
                    intersectionResult = uint4(packedNodePointer, INVALID_NODE, INVALID_NODE, INVALID_NODE);
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

        if (CheckHandleProceduralUserNode(prevNodePtr))
        {
            // Load primitive data
            const PrimitiveData primitiveData = FetchPrimitiveDataAddr(nodePointer, nodeAddr64);

            const uint primitiveIndex = primitiveData.primitiveIndex;
            const uint geometryIndex  = primitiveData.geometryIndex;
            const uint geometryFlags  = primitiveData.geometryFlags;

            // Fetch hitgroup information
            const uint instanceContribution = instanceHitContributionAndFlags & 0x00FFFFFF;
            HitGroupInfo hitInfo = GetHitGroupInfo(ExtractRayContributionToHitIndex(traceRayParameters),
                                                   ExtractMultiplierForGeometryContributionToHitIndex(traceRayParameters),
                                                   geometryIndex,
                                                   instanceContribution);

            // Determine anyHit shader call type
            uint anyHitCallType = rayForceOpaque ? ANYHIT_CALLTYPE_SKIP : ANYHIT_CALLTYPE_DUPLICATE;

            const bool noDuplicateAnyHit = (geometryFlags & D3D12_RAYTRACING_GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION);
            anyHitCallType = noDuplicateAnyHit ? ANYHIT_CALLTYPE_NO_DUPLICATE : anyHitCallType;

            // Check if this is an opaque intersection or not
            const uint instanceFlags = instanceHitContributionAndFlags >> 24;
            bool isOpaque = IsOpaque(geometryFlags, instanceFlags, rayFlags);

            bool isCulled = false;
            if (isOpaque)
            {
                // Skip calling any hit shader for opaque nodes
                anyHitCallType = ANYHIT_CALLTYPE_SKIP;
                isCulled = cullOpaque;
            }
            else
            {
                isCulled = cullNonOpaque;
            }

            if (isCulled == false)
            {
                const uint64_t instNodePtr64 = CalculateInstanceNodePtr64(GPURT_RTIP2_0, topLevelBvh, instNodePtr);
#if DEVELOPER
                if (EnableTraversalCounter())
                {
                    WriteRayHistoryTokenFunctionCall(rayId,
                                                     hitInfo.intersectionId,
                                                     hitInfo.tableIndex,
                                                     RAY_HISTORY_FUNC_CALL_TYPE_INTERSECTION);
                }
#endif
                // Set intersection attributes
                AmdTraceRaySetHitAttributes(intersection.t,
                                            0,
                                            HIT_STATUS_IGNORE,
                                            LowPart(instNodePtr64),
                                            HighPart(instNodePtr64),
                                            primitiveIndex,
                                            anyHitCallType,
                                            geometryIndex);

                // Call intersection shader
                AmdTraceRayCallIntersectionShader(hitInfo.intersectionId, hitInfo.anyHitId, hitInfo.tableIndex);

                // Returns hit attributes post intersection/anyhit shader call. "intersection.t" and "hitKind" are only
                // updated if the hit was accepted and "status" reflects that decision
                uint status = HIT_STATUS_IGNORE;
                AmdTraceRayGetHitAttributes(intersection.t, intersection.hitkind, status);

#if DEVELOPER
                if (EnableTraversalCounter())
                {
                    WriteRayHistoryTokenProceduralIntersectionStatus(rayId, status, intersection.t, intersection.hitkind);
                }
#endif

                // If hit is accepted update intersection result
                if (status != HIT_STATUS_IGNORE)
                {
#if DEVELOPER
                    if (EnableTraversalCounter())
                    {
                        intersection.numCandidateHits++;
                    }
#endif
                    intersection.nodeIndex = nodePointer;
                    tlasNodePtr = instNodePtr;

                    if ((rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH) ||
                        (status == HIT_STATUS_ACCEPT_AND_END_SEARCH))
                    {
                        break;
                    }
                }
            }
            // Setting lastNodePtr to TERMINAL_NODE prevents any of the intersection_results being pushed onto the stack.
            lastNodePtr = TERMINAL_NODE;
        }

#if USE_HW_INTRINSIC
        packedNodePointer = AmdTraceRayLdsStackStore(stackAddr, lastNodePtr, intersectionResult);
#else
        // SW stack emulation
        packedNodePointer = ds_store_stack(stackAddr, lastNodePtr, intersectionResult);
#endif

        lastNodePtr = INVALID_NODE;

        // BLAS->TLAS Transition condition
        bool resetRay = (stackAddr < stackPtrTop);

#if DEVELOPER
        if (EnableTraversalCounter())
        {
            intersection.maxStackDepth = max(intersection.maxStackDepth, stackAddr & 0xffff);
        }
#endif
        if (resetRay || (packedNodePointer == INVALID_NODE))
        {
            if (packedNodePointer == INVALID_NODE)
            {
                // Add one to the stackAddr, to reset it back up from the stackless pop
                stackAddr++;

                // Recompute the resetRay using the new stackAddr
                resetRay = (stackAddr < stackPtrTop);

                const uint lastRootNode = IsBvhRebraid() ? lastInstanceNode : blasRootNodePtr;

                // Switch prevNodePtr to the instance node in the TLAS if the last tested node was a BLAS root node.
                if ((prevNodePtr == lastRootNode) && (currentBvh != topLevelBvh))
                {
                    prevNodePtr = instNodePtr;
                }

                // prevNodePtr will still point to the TLAS instance node if an instance is culled. We also need to
                // switch back to the TLAS in this case.
                if (prevNodePtr == instNodePtr)
                {
                    currentBvh = topLevelBvh;
                    resetRay   = true;
                }

                packedNodePointer = FetchParentNodePointer(currentBvh,
                                                           prevNodePtr);

                lastNodePtr = trianglePairIntersected ? (prevNodePtr + 1) : prevNodePtr;
            }

            if (resetRay)
            {
                currentBvh   = topLevelBvh;
                localRay     = topLevelRay;
                stackPtrTop  = 0;
                pointerFlags = rayFlagsSetBits;
#if DEVELOPER
                if (EnableTraversalCounter())
                {
                    WriteRayHistoryTokenTopLevel(rayId, currentBvh);
                }
#endif
            }
        }

    }

    if (intersection.nodeIndex != INVALID_NODE)
    {
        const InstanceDesc      desc = FetchInstanceDesc(topLevelBvh, tlasNodePtr);
        const GpuVirtualAddress blas = GetInstanceAddr(desc);

        const PrimitiveData primitiveData = FetchPrimitiveData(blas, intersection.nodeIndex);

        intersection.instNodePtr          = tlasNodePtr;
        intersection.instanceContribution = desc.InstanceContributionToHitGroupIndex_and_Flags & 0x00FFFFFF;
        intersection.geometryIndex        = primitiveData.geometryIndex;
        intersection.primitiveIndex       = primitiveData.primitiveIndex;

        // Set hit token blas and tlas values
        AmdTraceRaySetHitTokenData(intersection.nodeIndex, tlasNodePtr);

        if (CheckHandleTriangleNode(intersection.nodeIndex))
        {
            AmdTraceRaySetTriangleIntersectionAttributes(intersection.barycentrics);
        }
    }
    else
    {
        // Set hit token blas and tlas values to invalid for miss
        AmdTraceRaySetHitTokenData(INVALID_NODE, INVALID_NODE);
    }

    return intersection;
}
