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
#ifndef USE_TEMP_ARRAY_STACK
#define USE_TEMP_ARRAY_STACK 0
#endif

//=====================================================================================================================
// RTIP 1.1 traversal stack
struct TraversalStackInfo
{
    uint stackPtr;
    uint numEntries;
#if DEVELOPER
    uint maxStackDepth;
#endif
#if USE_TEMP_ARRAY_STACK
    int data[SHORT_STACK_SIZE];
#endif
};

//=====================================================================================================================
static void InitStack(inout_param(TraversalStackInfo) stack)
{
    stack.stackPtr   = AmdTraceRayGetStackBase();
    stack.numEntries = 0;

#if DEVELOPER
    stack.maxStackDepth = 0;
#endif
}

//=====================================================================================================================
static void WriteStackEntry(inout_param(TraversalStackInfo) stack, uint value)
{
#if USE_TEMP_ARRAY_STACK
    stack.data[stack.stackPtr % SHORT_STACK_SIZE] = value;
#else
    AmdTraceRayLdsWrite(stack.stackPtr % AmdTraceRayGetStackSize(), value);
#endif
}

//=====================================================================================================================
static void PushNodes(inout_param(TraversalStackInfo) stack, in uint4 nodes, in uint4 valid)
{
    // Push nodes unconditionally. The valid bits passed in are
    //  x : not used
    //  y : is y valid (stackStride or 0)
    //  z : is z valid (stackStride or 0)
    //  w : is w valid (stackStride or 0)

    stack.numEntries-= stack.stackPtr;

    WriteStackEntry(stack, nodes.w);
    stack.stackPtr += valid.w;
    WriteStackEntry(stack, nodes.z);
    stack.stackPtr += valid.z;
    WriteStackEntry(stack, nodes.y);
    stack.stackPtr += valid.y;

    // update stack entry count (stack.numEntries += stackPtrNew - stackPtrOld)
    // Min is handled in pop path
    stack.numEntries += stack.stackPtr;

#if DEVELOPER
    stack.maxStackDepth = max(stack.maxStackDepth, stack.stackPtr);
#endif
}

//=====================================================================================================================
static uint PopStack(inout_param(TraversalStackInfo) stack)
{
    // Update stack entry count
    stack.numEntries = min(AmdTraceRayGetStackSize(), stack.numEntries);

    stack.stackPtr   -= AmdTraceRayGetStackStride();
    stack.numEntries -= AmdTraceRayGetStackStride();

#if USE_TEMP_ARRAY_STACK
    return stack.data[stack.stackPtr % SHORT_STACK_SIZE];
#else
    return AmdTraceRayLdsRead(stack.stackPtr % AmdTraceRayGetStackSize());
#endif
}

//=====================================================================================================================
static bool IsStackEmpty(TraversalStackInfo stack)
{
    return (stack.numEntries == 0);
}

//=====================================================================================================================
static bool IsStackOverflowed(TraversalStackInfo stack)
{
    // Assumes stack is empty. I.e. stack.numEntries = 0
    return (stack.stackPtr != AmdTraceRayGetStackBase());
}

//=====================================================================================================================
// Main TraceRays implementation
static IntersectionResult TraceRayImpl1_1(
    in GpuVirtualAddress topLevelBvh,               ///< Top-level acceleration structure to use
    in uint              rayFlags,                  ///< Ray flags
    in uint              traceRayParameters,        ///< Packed trace ray parameters
    in RayDesc           ray,                       ///< Ray to be traced
    in uint              rayId                      ///< Ray ID for profiling
)
{
    TraversalStackInfo stack;
    InitStack(stack);

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

    bool isGoingDown   = true;
    uint prevNodePtr   = INVALID_IDX;
    uint stackPtrTop   = INVALID_IDX;
    uint traversalMode = 0;
    uint instNodePtr;

    uint lastInstanceNode = INVALID_IDX;

    const bool rayForceOpaque        = (rayFlags & RAY_FLAG_FORCE_OPAQUE);
    const bool rayForceNonOpaque     = (rayFlags & RAY_FLAG_FORCE_NON_OPAQUE);
    const bool cullOpaque            = (rayFlags & RAY_FLAG_CULL_OPAQUE);
    const bool cullNonOpaque         = (rayFlags & RAY_FLAG_CULL_NON_OPAQUE);
    const bool triangleCullFrontFace = (rayFlags & RAY_FLAG_CULL_FRONT_FACING_TRIANGLES);
    const bool triangleCullBackFace  = (rayFlags & RAY_FLAG_CULL_BACK_FACING_TRIANGLES);
    const bool triangleCullEnable    = triangleCullFrontFace || triangleCullBackFace;

    // Last visited node pointer used to record which node was prior for stackless back tracking
    uint lastNodePtr = INVALID_NODE;

    // LDS stack stride
    const uint stackStride = AmdTraceRayGetStackStride();
    // box sort mode
    uint boxSortMode = AmdTraceRayGetBoxSortHeuristicMode();
    if (boxSortMode == BoxSortHeuristic::DisabledOnAcceptFirstHit)
    {
        boxSortMode = GetBoxSortingHeuristicFromRayFlags(rayFlags, boxSortMode);
    }
    else
    {
        boxSortMode = (boxSortMode == BoxSortHeuristic::Disabled) ?
            BoxSortHeuristic::Disabled : BoxSortHeuristic::Closest;
    }

#if DEVELOPER
    while ((packedNodePointer != INVALID_IDX) && (intersection.numIterations < DispatchRaysConstBuf.profileMaxIterations))
#else
    while (packedNodePointer != INVALID_IDX)
#endif
    {
#if DEVELOPER
        if (EnableTraversalCounter())
        {
            WriteRayHistoryTokenNodePtr(rayId, packedNodePointer);

            intersection.numIterations++;

            if (IsBoxNode1_1(packedNodePointer))
            {
                intersection.numRayBoxTest++;
            }
            else if (IsTriangleNode1_1(packedNodePointer))
            {
                intersection.numRayTriangleTest++;
            }
            else if (IsUserNodeInstance(packedNodePointer))
            {
                intersection.instanceIntersections++;
            }

            UpdateWaveTraversalStatistics(GPURT_RTIP1_1, packedNodePointer);
        }
#endif

        // Backup last traversed node pointer
        prevNodePtr = packedNodePointer;

        const uint nodePointer = ExtractNodePointer(packedNodePointer);

        // pre-calculate node address
        const GpuVirtualAddress nodeAddr64 = currentBvh + ExtractNodePointerOffset(nodePointer);

        uint4 intersectionResult = image_bvh64_intersect_ray(currentBvh,
                                                             nodePointer,
                                                             boxSortMode,
                                                             intersection.t,
                                                             localRay.Origin,
                                                             localRay.Direction,
                                                             rcp(localRay.Direction));

        // Set next node pointer to invalid to trigger node search
        packedNodePointer = INVALID_NODE;

        // Check if it is an internal node
        if (IsBoxNode1_1(prevNodePtr))
        {
            // Determine nodes to push to stack
            if (isGoingDown)
            {
                uint4 isValid;

                isValid.y = (intersectionResult.y != INVALID_NODE) ? stackStride : 0;

                // Push all nodes if we have at least one valid node
                // Sorting causes z and w to be invalid if y is invalid
                if (isValid.y)
                {
                    isValid.w = (intersectionResult.w != INVALID_NODE) ? stackStride : 0;
                    isValid.z = (intersectionResult.z != INVALID_NODE) ? stackStride : 0;
                    PushNodes(stack, intersectionResult, isValid);
                }

                // Traverse next valid node
                packedNodePointer = intersectionResult.x;
            }
            else
            {
                // Determine sibling node to traverse if stackless
                // Don't need to check for valid here, since invalid nodes will cause a pop anyways
                // and there is no holes in the return list due to sorting.
                const bool gotoY = ComparePointers(intersectionResult.x, lastNodePtr);
                const bool gotoZ = ComparePointers(intersectionResult.y, lastNodePtr);
                const bool gotoW = ComparePointers(intersectionResult.z, lastNodePtr);

                // Switch to sibling node
                // Note, exactly one of the goto flags is set or none. If the last node is not a
                // traversal candidate we have to go search for a new node.
                packedNodePointer = gotoW ? intersectionResult.w : INVALID_NODE;
                packedNodePointer = gotoZ ? intersectionResult.z : packedNodePointer;
                packedNodePointer = gotoY ? intersectionResult.y : packedNodePointer;

                // Update traversal direction
                isGoingDown = true;
            }
        }
        else if (CheckHandleTriangleNode(prevNodePtr))
        {
            const float t_num = asfloat(intersectionResult.x);
            const float t_denom = asfloat(intersectionResult.y);
            float candidateT  = (t_num / t_denom);

            if (candidateT < intersection.t)
            {
                uint status = HIT_STATUS_ACCEPT;

                // Evaluate triangle culling
                const uint instanceFlags      = (instanceHitContributionAndFlags >> 24);
                const bool instanceCullEnable = ((instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_CULL_DISABLE) == 0);
                const bool instanceFrontCCW    = (instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_FRONT_COUNTERCLOCKWISE);

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

                // Check for ray flags first. If forced opaque (known at compile time), this entire block is skipped by the compiler
                if (rayForceOpaque == false)
                {
                    // handle opaque hits
                    if (status != HIT_STATUS_IGNORE)
                    {
                        // load primitive data
                        const PrimitiveData primitiveData = FetchPrimitiveDataAddr(nodePointer, nodeAddr64);

                        const uint primitiveIndex = primitiveData.primitiveIndex;
                        const uint geometryIndex  = primitiveData.geometryIndex;
                        const uint geometryFlags  = primitiveData.geometryFlags;

                        // Check if this is an opaque intersection or not
                        const uint instanceFlags = (instanceHitContributionAndFlags >> 24);
                        bool isOpaque = IsOpaque(geometryFlags, instanceFlags, rayFlags);

                        bool isCulled = false;
                        if (isOpaque)
                        {
                            status = cullOpaque ? HIT_STATUS_IGNORE : status;
                        }
                        else
                        {
                            status = cullNonOpaque ? HIT_STATUS_IGNORE : status;
                        }

                        // call anyhit shader for non-opaque triangles
                        if ((status != HIT_STATUS_IGNORE) && (!isOpaque))
                        {
                            const uint instanceContribution = (instanceHitContributionAndFlags & 0x00FFFFFF);
                            HitGroupInfo hitInfo = GetHitGroupInfo(ExtractRayContributionToHitIndex(traceRayParameters),
                                                                   ExtractMultiplierForGeometryContributionToHitIndex(traceRayParameters),
                                                                   geometryIndex,
                                                                   instanceContribution);

                            const uint64_t instNodePtr64 =
                                CalculateInstanceNodePtr64(GPURT_RTIP1_1, topLevelBvh, instNodePtr);
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
                }
            }
            else if ((AmdTraceRayGetTriangleCompressionMode() == PAIR_TRIANGLE_COMPRESSION) ||
                     (AmdTraceRayGetTriangleCompressionMode() == AUTO_TRIANGLE_COMPRESSION))
            {
                if (GetNodeType(prevNodePtr) == NODE_TYPE_TRIANGLE_1)
                {
                    // Intersect with triangle 0 next
                    packedNodePointer = ClearNodeType(prevNodePtr);
                }
            }
        }
        else if (CheckHandleProceduralUserNode(prevNodePtr))
        {
            // Load primitive data
            const PrimitiveData primitiveData = FetchPrimitiveData(currentBvh, nodePointer);

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
                const uint64_t instNodePtr64 = CalculateInstanceNodePtr64(GPURT_RTIP1_1, topLevelBvh, instNodePtr);
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
                    intersection.nodeIndex         = nodePointer;
                    tlasNodePtr                    = instNodePtr;

                    if ((rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH) ||
                        (status == HIT_STATUS_ACCEPT_AND_END_SEARCH))
                    {
                        break;
                    }
               }
            }
        }
        else  // NODE_TYPE_USER_NODE_INSTANCE
        {
            const InstanceDesc desc = FetchInstanceDescAddr(nodeAddr64);

            // Avoid branching here to delay the wait on the fetched data
            // Fetch instance node pointer (for rebraid) here to allow for the global loads to be
            // coalesced into a single loadx2
            const uint instanceNodePtr      = FetchInstanceNodePointerAddr(nodeAddr64);
            instanceHitContributionAndFlags = desc.InstanceContributionToHitGroupIndex_and_Flags;

            InstanceTransform(desc,
                              topLevelRay.Origin,
                              topLevelRay.Direction,
                              localRay.Origin,
                              localRay.Direction);

            bool isInstanceCulled = true;

            /// @note: Instance mask is 0 for null BLAS and it would be ignored
            if (desc.InstanceID_and_Mask & instanceInclusionMask)
            {
                isInstanceCulled = CheckInstanceCulling(desc, rayFlags, AmdTraceRayGetStaticFlags());
            }

            if (!isInstanceCulled)
            {
                currentBvh        = GetInstanceAddr(desc);
                instNodePtr       = prevNodePtr;
                stackPtrTop       = stack.stackPtr;

                if (IsBvhRebraid())
                {
                    lastInstanceNode  = instanceNodePtr;
                    packedNodePointer = instanceNodePtr;
                }
                else
                {
                    packedNodePointer = blasRootNodePtr;
                }

#if DEVELOPER
                if (EnableTraversalCounter())
                {
                    WriteRayHistoryTokenBottomLevel(rayId, currentBvh);
                }
#endif
            }
        }

        // If invalid, search for next node to traverse
        if (packedNodePointer == INVALID_NODE)
        {
            // stack is empty, traverse to parent
            if (IsStackEmpty(stack))
            {
                if (IsStackOverflowed(stack))
                {
                    if (IsBvhRebraid())
                    {
                        // bottom->top currentBvh. traverse back up to find new node
                        if ((prevNodePtr == lastInstanceNode) && (currentBvh != topLevelBvh))
                        {
                            currentBvh  = topLevelBvh;
                            prevNodePtr = instNodePtr;
#if DEVELOPER
                            if (EnableTraversalCounter())
                            {
                                WriteRayHistoryTokenTopLevel(rayId, currentBvh);
                            }
#endif
                        }
                    }
                    else
                    {
                        // bottom->top currentBvh. traverse back up to find new node
                        if ((prevNodePtr == blasRootNodePtr) && (currentBvh != topLevelBvh))
                        {
                            currentBvh  = topLevelBvh;
                            prevNodePtr = instNodePtr;
#if DEVELOPER
                            if (EnableTraversalCounter())
                            {
                                WriteRayHistoryTokenTopLevel(rayId, currentBvh);
                            }
#endif
                        }
                    }

                    packedNodePointer = FetchParentNodePointer(currentBvh,
                                                               prevNodePtr);
                    isGoingDown = false;
                    lastNodePtr = prevNodePtr;
                } // else traversal is complete, packedNodePointer stays invalid and the loop does not continue
            }
            else
            {
                if (stackPtrTop == stack.stackPtr)
                {
                    currentBvh = topLevelBvh;
#if DEVELOPER
                    if (EnableTraversalCounter())
                    {
                        WriteRayHistoryTokenTopLevel(rayId, currentBvh);
                    }
#endif
                }

                packedNodePointer = PopStack(stack);
            }

            // Select ray based on tlas vs blas
            if (currentBvh == topLevelBvh)
            {
                localRay = topLevelRay;
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

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        AmdTraceRaySetParentId(rayId);
        intersection.maxStackDepth = stack.maxStackDepth;
    }
#endif

    return intersection;
}

//=====================================================================================================================
static IntersectionResult IntersectRayImpl1_1(
    in GpuVirtualAddress topLevelBvh,   ///< Top-level acceleration structure to use
    in RayDesc           ray,           ///< Ray to be traced
    in uint              blasPointer,   ///< bottom level node pointer
    in uint              tlasPointer,   ///< top level node pointer
    in uint              rayId)         ///< Ray ID for profiling
{
    RayDesc localRay = ray;
    localRay.Origin += localRay.TMin * localRay.Direction;
    RayDesc topLevelRay = localRay;

    IntersectionResult result = (IntersectionResult)0;
    result.t         = ray.TMax - ray.TMin;
    result.nodeIndex = INVALID_IDX;

#if DEVELOPER
    result.numIterations = 0;

    result.numRayBoxTest         = 0;
    result.numRayTriangleTest    = 0;
    result.maxStackDepth         = 0;
    result.numAnyHitInvocation   = 0;
    result.numCandidateHits      = 0;
    result.instanceIntersections = 0;
#endif

    if ((blasPointer != INVALID_NODE) && (tlasPointer != INVALID_NODE))
    {
        const InstanceDesc      desc            = FetchInstanceDesc(topLevelBvh, tlasPointer);
        const GpuVirtualAddress blasAddress     = GetInstanceAddr(desc);

#if DEVELOPER
        if (EnableTraversalCounter())
        {
            WriteRayHistoryTokenBottomLevel(rayId, blasAddress);
            WriteRayHistoryTokenNodePtr(rayId, blasPointer);
        }
#endif

        InstanceTransform(desc,
                          topLevelRay.Origin,
                          topLevelRay.Direction,
                          localRay.Origin,
                          localRay.Direction);

        uint nodePointer = ExtractNodePointer(blasPointer);

        uint4 intersectionResult = image_bvh64_intersect_ray(blasAddress,
                                                             nodePointer,
                                                             BoxSortHeuristic::Closest,
                                                             result.t,
                                                             localRay.Origin,
                                                             localRay.Direction,
                                                             rcp(localRay.Direction));

        const PrimitiveData primitiveData = FetchPrimitiveData(blasAddress, nodePointer);
        result.primitiveIndex             = primitiveData.primitiveIndex;
        result.geometryIndex              = primitiveData.geometryIndex;
        result.t                          = (asfloat(intersectionResult.x) / asfloat(intersectionResult.y));
        result.barycentrics.x             = asfloat(intersectionResult.z) / asfloat(intersectionResult.y);
        result.barycentrics.y             = asfloat(intersectionResult.w) / asfloat(intersectionResult.y);
        result.nodeIndex                  = nodePointer;
        result.instNodePtr                = tlasPointer;
        result.instanceContribution       = desc.InstanceContributionToHitGroupIndex_and_Flags & 0x00FFFFFF;

        const bool instanceFrontCCW = ((desc.InstanceContributionToHitGroupIndex_and_Flags >> 24) &
                                        D3D12_RAYTRACING_INSTANCE_FLAG_TRIANGLE_FRONT_COUNTERCLOCKWISE);

        // If determinant is positive and front face winding is counterclockwise or vice versa, the triangle is back facing
        const uint backFacingTriangle = ((intersectionResult.y >> 31) ^ instanceFrontCCW);
        result.hitkind = backFacingTriangle | HIT_KIND_TRIANGLE_FRONT_FACE;

        AmdTraceRaySetTriangleIntersectionAttributes(result.barycentrics);
    }

    return result;
}
