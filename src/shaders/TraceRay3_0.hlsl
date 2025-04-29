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

#include "rtip3_ops.hlsl"

//=====================================================================================================================
static uint ProcessTriangleNode(
    uint                           rayId,
    inout_param(IntersectionState) intersection,
    uint4                          result,
    uint                           geometryIndex,
    uint                           instanceContribution,
    uint                           packedTraceRayParameters,
    uint64_t                       tlasBaseAddr,
    uint64_t                       blasBaseAddr,
    uint                           tlasNodePtr,
    uint                           blasNodePtr,
    uint                           rayFlags,
    float                          tMin)
{
    const bool rayForceOpaque    = (rayFlags & RAY_FLAG_FORCE_OPAQUE);
    const bool raySkipProcedural = (rayFlags & RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES);

    const bool isProcedural = !raySkipProcedural && ((result.z & 0x80000000) != 0);
    const bool runAnyHit    = !rayForceOpaque && ((result.w & 0x80000000) != 0);

    const float tNum   = asfloat(result.x);
    const float tDenom = asfloat(result.y);
    float tHit = tNum / tDenom;

    BuiltInTriangleIntersectionAttributes attr;
    attr.barycentrics.x = abs(asfloat(result.z) / tDenom);
    attr.barycentrics.y = abs(asfloat(result.w) / tDenom);

    bool triangleHit = EvaluateTriangleHit(tMin, tHit, intersection.t);
    uint status = triangleHit ? HIT_STATUS_ACCEPT : HIT_STATUS_IGNORE;

    uint kind = 0;
    if (triangleHit || isProcedural)
    {
        // On real hardware, we'd pass in the intersection parameters to the HitGroup shader and have it
        // determine the hitKind from tDenom as below
        kind = (result.y >> 31) | HIT_KIND_TRIANGLE_FRONT_FACE;

        if (!raySkipProcedural || !rayForceOpaque)
        {
            if (isProcedural || runAnyHit)
            {
                const uint64_t tlasNodePtr64 = CalculateInstanceNodePtr64(GPURT_RTIP3_0,
                                                                          ExtractInstanceAddr(tlasBaseAddr),
                                                                          tlasNodePtr);
                const uint64_t blasNodeAddr = GetNodeAddr(blasBaseAddr, blasNodePtr);

                // fetch primitive data from triangle node
                PrimitiveData primitiveData = FetchPrimitiveDataAddr3_0(blasNodePtr, blasNodeAddr);

                const uint anyHitCallType = isProcedural ?
                                            (runAnyHit ? ANYHIT_CALLTYPE_DUPLICATE : ANYHIT_CALLTYPE_SKIP) :
                                            ANYHIT_CALLTYPE_NO_DUPLICATE;

                AmdTraceRaySetHitAttributes((isProcedural ? intersection.t : tHit),
                                            kind,
                                            (isProcedural ? HIT_STATUS_IGNORE : HIT_STATUS_ACCEPT),
                                            LowPart(tlasNodePtr64),
                                            HighPart(tlasNodePtr64),
                                            primitiveData.primitiveIndex,
                                            anyHitCallType,
                                            (geometryIndex & 0xFFFFF));

                // Set hit triangle information
                AmdTraceRaySetHitTriangleNodePointer(blasBaseAddr, blasNodePtr);

                HitGroupInfo hitInfo = GetHitGroupInfo(ExtractRayContributionToHitIndex(packedTraceRayParameters),
                                                       ExtractMultiplierForGeometryContributionToHitIndex(packedTraceRayParameters),
                                                       geometryIndex,
                                                       instanceContribution);

                if (isProcedural)
                {
                    AmdTraceRayCallIntersectionShader(hitInfo.intersectionId, hitInfo.anyHitId, hitInfo.tableIndex);
                }
                else if (runAnyHit)
                {
                    AmdTraceRayCallTriangleAnyHitShader(hitInfo.anyHitId, hitInfo.tableIndex, attr);
                }

                // Read back attributes that may have been modified by the Intersection / AnyHit shader
                AmdTraceRayGetHitAttributes(tHit, kind, status);

#if DEVELOPER
                if (EnableTraversalCounter())
                {
                    if (isProcedural)
                    {
                        WriteRayHistoryTokenProceduralIntersectionStatus(rayId, status, tHit, kind);
                    }
                    else if (runAnyHit && (PackUint64(hitInfo.anyHitId) != 0))
                    {
                        WriteRayHistoryTokenAnyHitStatus(rayId, status);
                    }
                }
#endif
            }
        }

#if DEVELOPER
        if (EnableTraversalCounter())
        {
            WriteRayHistoryTokenTriangleHitResult(rayId,
                                                  uint(status > HIT_STATUS_IGNORE),
                                                  tHit);
        }
#endif

        if (status != HIT_STATUS_IGNORE)
        {
            intersection.t                    = tHit;
            intersection.barycentrics.x       = asfloat(result.z) / asfloat(result.y);
            intersection.barycentrics.y       = asfloat(result.w) / asfloat(result.y);
            intersection.hitKind              = kind;
            intersection.instanceContribution = instanceContribution;
        }
    }

    return status;
}

//=====================================================================================================================
static IntersectionResult TraceRayImpl3_0(
    in GpuVirtualAddress tlasBaseAddr,              ///< Top-level acceleration structure to use
    in uint              rayFlags,                  ///< Ray flags
    in uint              traceRayParameters,        ///< Packed trace ray parameters
    in RayDesc           ray,                       ///< Ray to be traced
    in uint              rayId                      ///< Ray ID for profiling
)
{
    // Start from root node which follows acceleration structure header
    const uint rootNodePtr  = CreateRootNodePointer1_1();
    const uint instanceMask = (traceRayParameters & 0xff);

    uint boxHeuristicMode = AmdTraceRayGetBoxSortHeuristicMode();
    if ((boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosest) ||
        (boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosestMidPoint))
    {
        boxHeuristicMode = GetBoxSortingHeuristicFromRayFlags(rayFlags, boxHeuristicMode);
    }

    // Temporary locals for traversal
    RayDesc localRay = ray;
    localRay.Origin += ApplyTMinBias(localRay.TMin) * localRay.Direction;
    RayDesc topLevelRay = localRay;

    uint stackAddr = RtIp3LdsStackInit();

    // Dual node pointers for traversal
    uint2 nodePtr;
    nodePtr.x = rootNodePtr;
    nodePtr.y = INVALID_NODE;

    uint instNodePtr      = INVALID_NODE;
    uint lastNodePtr      = INVALID_NODE;
    uint lastInstanceNode = INVALID_NODE;

    // Committed traversal state
    IntersectionState intersection;
    intersection.t              = ray.TMax - ApplyTMinBias(ray.TMin);
    intersection.barycentrics.x = 0.0f;
    intersection.barycentrics.y = 0.0f;
    intersection.hitKind        = 0u;

    uint blasNodePtr          = INVALID_NODE;
    uint tlasNodePtr          = INVALID_NODE;
    uint instanceContribution = 0;

    IntersectionResult iResult = (IntersectionResult)0;

    tlasBaseAddr = EncodeBasePointer(tlasBaseAddr, rayFlags);

    uint64_t currBaseAddr = tlasBaseAddr;
    uint64_t blasBaseAddr = 0;

    while (IsValidNode(nodePtr.x))
    {
#if DEVELOPER
        if (EnableTraversalCounter())
        {
            WriteRayHistoryTokenNodePtr(rayId, nodePtr.x);

            iResult.numIterations++;

            if (IsUserNodeInstance(nodePtr.x))
            {
                iResult.instanceIntersections++;
            }
            else if (IsTriangleNode1_1(nodePtr.x))
            {
                iResult.numRayTriangleTest++;
            }
            else if (IsBoxNode1_1(nodePtr.x))
            {
                iResult.numRayBoxTest++;
            }

            UpdateWaveTraversalStatistics(GPURT_RTIP3_0, nodePtr.x);

            if (nodePtr.y != INVALID_NODE)
            {
                WriteRayHistoryTokenNodePtr(rayId, nodePtr.y);

                if (IsUserNodeInstance(nodePtr.y))
                {
                    iResult.instanceIntersections++;
                }
                else if (IsTriangleNode1_1(nodePtr.y))
                {
                    iResult.numRayTriangleTest++;
                }
                else if (IsBoxNode1_1(nodePtr.y))
                {
                    iResult.numRayBoxTest++;
                }

                UpdateWaveTraversalStatistics(GPURT_RTIP3_0, nodePtr.y);
            }
        }
#endif

        DualIntersectResult result =
            image_bvh_dual_intersect_ray(currBaseAddr,
                                         GetNodePointerExclMsbFlag(nodePtr.x),
                                         GetNodePointerExclMsbFlag(nodePtr.y),
                                         localRay.Origin,
                                         localRay.Direction,
                                         intersection.t,
                                         instanceMask,
                                         boxHeuristicMode);

        if (IsUserNodeInstance(nodePtr.x))
        {
            instNodePtr = nodePtr.x;
            currBaseAddr = PackUint64(result.first.z, result.first.w);
            // Instance contribution is stored in the 24 bit user data field of the instance node
            instanceContribution = result.second.z;
            if (IsBvhRebraid())
            {
                lastInstanceNode = result.second.w;
            }
            // result.second.w contains the root node of the instance or INVALID_NODE if the instance was culled. Set
            // result.second.z to SKIP_0_7 in order to skip the remaining inputs to ds_stack_push.
            result.second.z = SKIP_0_7;
        }
        else
        {
            if (IsTriangleNode1_1(nodePtr.y))
            {
                const uint status = ProcessTriangleNode(rayId,
                                                        intersection,
                                                        result.second,
                                                        result.geometryId[1],
                                                        instanceContribution,
                                                        traceRayParameters,
                                                        tlasBaseAddr,
                                                        currBaseAddr,
                                                        instNodePtr,
                                                        nodePtr.y,
                                                        rayFlags,
                                                        ray.TMin);

                if (status != HIT_STATUS_IGNORE)
                {
                    tlasNodePtr  = instNodePtr;
                    blasNodePtr  = nodePtr.y;
                    blasBaseAddr = currBaseAddr;

                    if ((status == HIT_STATUS_ACCEPT_AND_END_SEARCH) ||
                        (rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH))
                    {
                        break;
                    }
                }

                result.second = GetResultVecForStackPush(result.second, nodePtr.y, SKIP_4_7);
            }
            else if (IsValidNode(nodePtr.y) == false)
            {
                // When nodePtr.y is invalid, result.second is also invalid. On some GPUs, result.second may
                // not be updated in this case, retaining its value from the previous iteration. Experimental
                // results show that SKIP_4_7 fails to skip these invalid values, it is safer to set all values to
                // invalid.
                result.second = uint4(INVALID_NODE, INVALID_NODE, INVALID_NODE, INVALID_NODE);
            }

            if (IsTriangleNode1_1(nodePtr.x))
            {
                const uint status = ProcessTriangleNode(rayId,
                                                        intersection,
                                                        result.first,
                                                        result.geometryId[0],
                                                        instanceContribution,
                                                        traceRayParameters,
                                                        tlasBaseAddr,
                                                        currBaseAddr,
                                                        instNodePtr,
                                                        nodePtr.x,
                                                        rayFlags,
                                                        ray.TMin);

                if (status != HIT_STATUS_IGNORE)
                {
                    tlasNodePtr   = instNodePtr;
                    blasNodePtr   = nodePtr.x;
                    blasBaseAddr  = currBaseAddr;

                    if ((status == HIT_STATUS_ACCEPT_AND_END_SEARCH) ||
                        (rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH))
                    {
                        break;
                    }
                }

                result.first = GetResultVecForStackPush(result.first, nodePtr.x, SKIP_0_3);
            }
        }

        // backup previous node pointer
        uint prevNodePtr = (nodePtr.y != INVALID_NODE) ? nodePtr.y : nodePtr.x;
        nodePtr          = ds_stack_push8_pop2(stackAddr, lastNodePtr, result.first, result.second);
        lastNodePtr      = INVALID_NODE;

        // Fallback to stackless walk
        if (nodePtr.x == INVALID_NODE)
        {
            const uint lastRootNode = IsBvhRebraid() ? lastInstanceNode : rootNodePtr;

            lastNodePtr = prevNodePtr;
            // In triangle pair compression mode, only the tri1 node is stored in the BVH tree, not the tri0 node.
            // Therefore, when comparing with lastRootNode and identifying the parent node, prevNodePtr must be
            // converted to the corresponding tri1 node.
            if ((prevNodePtr & GENERATED_TRI0_NODE_FLAG) != 0)
            {
                prevNodePtr = GetNodePointerExclMsbFlag(prevNodePtr) + 1;
                lastNodePtr = prevNodePtr;
            }

            if ((prevNodePtr == lastRootNode) && (currBaseAddr != tlasBaseAddr))
            {
                prevNodePtr = instNodePtr;
            }

            if (prevNodePtr == instNodePtr)
            {
                currBaseAddr = tlasBaseAddr;
                stackAddr   |= STACK_ADDR_BLAS_TO_TLAS_MASK;
            }

            nodePtr.x = FetchParentNodePointer(ExtractInstanceAddr(currBaseAddr), prevNodePtr);
        }

        // Reset ray on transition from BLAS to TLAS
        if (stackAddr & STACK_ADDR_BLAS_TO_TLAS_MASK)
        {
            localRay.Direction = topLevelRay.Direction;
            localRay.Origin    = topLevelRay.Origin;
            currBaseAddr       = tlasBaseAddr;
        }

    }

    iResult.t            = intersection.t;
    iResult.barycentrics = intersection.barycentrics;
    iResult.nodeIndex    = blasNodePtr;

    // The signs of I, J, and t_denom are used to communicate other information so we need to take the absolute value
    // to get the final barycentrics.
    iResult.barycentrics.x = abs(iResult.barycentrics.x);
    iResult.barycentrics.y = abs(iResult.barycentrics.y);

    // Fetch additional data for closest hit
    if (blasNodePtr != INVALID_NODE)
    {
        const uint64_t tlasNodeAddr = GetNodeAddr(tlasBaseAddr, tlasNodePtr);
        const uint64_t blasNodeAddr = GetNodeAddr(blasBaseAddr, blasNodePtr);

        // fetch primitive data from triangle node
        const PrimitiveData primitiveData = FetchPrimitiveDataAddr3_0(blasNodePtr, blasNodeAddr);

        iResult.geometryIndex        = primitiveData.geometryIndex;
        iResult.primitiveIndex       = primitiveData.primitiveIndex;
        iResult.instNodePtr          = tlasNodePtr;
        iResult.hitkind              = intersection.hitKind;
        iResult.instanceContribution = intersection.instanceContribution;

        // Set hit token blas and tlas values
        AmdTraceRaySetHitTokenData(blasNodePtr, tlasNodePtr);

        // Both barycentrics = 1.0 only for procedural primitives
        if ((iResult.barycentrics.x != 1.0f) || (iResult.barycentrics.y != 1.0f))
        {
            AmdTraceRaySetTriangleIntersectionAttributes(iResult.barycentrics.xy);
        }
    }
    else
    {
        // Set hit token blas and tlas values to invalid for miss
        AmdTraceRaySetHitTokenData(INVALID_NODE, INVALID_NODE);
    }

    return iResult;
}

//=====================================================================================================================
static IntersectionResult TraceRayImpl3_0BVH8(
    in GpuVirtualAddress tlasBaseAddr,              ///< Top-level acceleration structure to use
    in uint              rayFlags,                  ///< Ray flags
    in uint              traceRayParameters,        ///< Packed trace ray parameters
    in RayDesc           ray,                       ///< Ray to be traced
    in uint              rayId                      ///< Ray ID for profiling
)
{
    // Start from root node which follows acceleration structure header
    const uint rootNodePtr  = CreateRootNodePointer1_1();
    const uint instanceMask = (traceRayParameters & 0xff);

    uint boxHeuristicMode = AmdTraceRayGetBoxSortHeuristicMode();
    if ((boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosest) ||
        (boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosestMidPoint))
    {
        boxHeuristicMode = GetBoxSortingHeuristicFromRayFlags(rayFlags, boxHeuristicMode);
    }

    // Temporary locals for traversal
    RayDesc localRay = ray;
    localRay.Origin += ApplyTMinBias(localRay.TMin) * localRay.Direction;
    RayDesc topLevelRay = localRay;

    uint stackAddr = RtIp3LdsStackInit();

    // Dual node pointers for traversal
    uint nodePtr = rootNodePtr;

    uint instNodePtr = INVALID_NODE;
    uint lastNodePtr = INVALID_NODE;
    uint lastInstanceNode = INVALID_NODE;

    // Committed traversal state
    IntersectionState intersection;
    intersection.t = ray.TMax - ApplyTMinBias(ray.TMin);
    intersection.barycentrics.x = 0.0f;
    intersection.barycentrics.y = 0.0f;
    intersection.hitKind = 0u;

    uint blasNodePtr          = INVALID_NODE;
    uint tlasNodePtr          = INVALID_NODE;
    uint instanceContribution = 0;

    IntersectionResult iResult = (IntersectionResult) 0;

    tlasBaseAddr = EncodeBasePointer(tlasBaseAddr, rayFlags);

    uint64_t currBaseAddr = tlasBaseAddr;
    uint64_t blasBaseAddr = 0;

    while (IsValidNode(nodePtr))
    {
#if DEVELOPER
        if (EnableTraversalCounter())
        {
            WriteRayHistoryTokenNodePtr(rayId, nodePtr);

            iResult.numIterations++;

            if (IsUserNodeInstance(nodePtr))
            {
                iResult.instanceIntersections++;
            }
            else if (IsTriangleNode1_1(nodePtr))
            {
                iResult.numRayTriangleTest++;
            }
            else if (IsBoxNode1_1(nodePtr))
            {
                iResult.numRayBoxTest++;
            }

            UpdateWaveTraversalStatistics(GPURT_RTIP3_0, nodePtr);
        }
#endif

        DualIntersectResult result =
                image_bvh8_intersect_ray(currBaseAddr,
                                         nodePtr,
                                         localRay.Origin,
                                         localRay.Direction,
                                         intersection.t,
                                         instanceMask,
                                         boxHeuristicMode);

        if (IsUserNodeInstance(nodePtr))
        {
            instNodePtr = nodePtr;
            currBaseAddr = PackUint64(result.first.z, result.first.w);
            // Instance contribution is stored in the 24 bit user data field of the instance node
            instanceContribution = result.second.z;
            if (IsBvhRebraid())
            {
                lastInstanceNode = result.second.w;
            }
            // result.second.w contains the root node of the instance or INVALID_NODE if the instance was culled. Set
            // result.second.z to SKIP_0_7 in order to skip the remaining inputs to ds_stack_push.
            result.second.z = SKIP_0_7;
        }
        else
        {
            if (IsTriangleNode1_1(nodePtr))
            {
                // Process tri 0.
                const uint tri0NodePtr = ClearNodeType(nodePtr);

                uint status = ProcessTriangleNode(rayId,
                                                  intersection,
                                                  result.first,
                                                  result.geometryId[0],
                                                  instanceContribution,
                                                  traceRayParameters,
                                                  tlasBaseAddr,
                                                  currBaseAddr,
                                                  instNodePtr,
                                                  tri0NodePtr,
                                                  rayFlags,
                                                  ray.TMin);

                if (status != HIT_STATUS_IGNORE)
                {
                    blasNodePtr   = tri0NodePtr;
                }

                // If pair compression is enabled and the node type is 1, process tri 1.
                const bool isPairCompressed =
                    ((AmdTraceRayGetTriangleCompressionMode() == PAIR_TRIANGLE_COMPRESSION) ||
                     (AmdTraceRayGetTriangleCompressionMode() == AUTO_TRIANGLE_COMPRESSION)) &&
                    (GetNodeType(nodePtr) == NODE_TYPE_TRIANGLE_1);

                if (isPairCompressed &&
                    (status != HIT_STATUS_ACCEPT_AND_END_SEARCH) &&
                    ((status == HIT_STATUS_IGNORE) ||
                     ((rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH) == false)))
                {
                    const uint status2 = ProcessTriangleNode(rayId,
                                                             intersection,
                                                             result.second,
                                                             result.geometryId[1],
                                                             instanceContribution,
                                                             traceRayParameters,
                                                             tlasBaseAddr,
                                                             currBaseAddr,
                                                             instNodePtr,
                                                             nodePtr,
                                                             rayFlags,
                                                             ray.TMin);

                    if (status2 != HIT_STATUS_IGNORE)
                    {
                        blasNodePtr   = nodePtr;
                        status       |= status2;
                    }
                }

                // We have ORed both the statuses above for the following
                if (status != HIT_STATUS_IGNORE)
                {
                    tlasNodePtr  = instNodePtr;
                    blasBaseAddr = currBaseAddr;

                    if ((status & HIT_STATUS_ACCEPT_AND_END_SEARCH) ||
                        (rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH))
                    {
                        break;
                    }
                }

                // Skip inputs to ds_stack_push
                result.second.w = SKIP_0_7;
            }
        }

        // backup previous node pointer
        uint prevNodePtr = nodePtr;

        nodePtr = ds_stack_push8_pop1(stackAddr, lastNodePtr, result.first, result.second, GPURT_RTIP3_0);
        lastNodePtr = INVALID_NODE;

        // Fallback to stackless walk
        if (nodePtr == INVALID_NODE)
        {
            const uint lastRootNode = IsBvhRebraid() ? lastInstanceNode : rootNodePtr;

            if ((prevNodePtr == lastRootNode) && (currBaseAddr != tlasBaseAddr))
            {
                prevNodePtr = instNodePtr;
            }

            if (prevNodePtr == instNodePtr)
            {
                currBaseAddr = tlasBaseAddr;
                stackAddr   |= STACK_ADDR_BLAS_TO_TLAS_MASK;
            }

            nodePtr = FetchParentNodePointer(ExtractInstanceAddr(currBaseAddr), prevNodePtr);

            lastNodePtr = prevNodePtr;
        }

        // Reset ray on transition from BLAS to TLAS
        if (stackAddr & STACK_ADDR_BLAS_TO_TLAS_MASK)
        {
            localRay.Direction = topLevelRay.Direction;
            localRay.Origin    = topLevelRay.Origin;
            currBaseAddr       = tlasBaseAddr;
        }

    }

    iResult.t            = intersection.t;
    iResult.barycentrics = intersection.barycentrics;
    iResult.nodeIndex    = blasNodePtr;

    // The signs of I, J, and t_denom are used to communicate other information so we need to take the absolute value
    // to get the final barycentrics.
    iResult.barycentrics.x = abs(iResult.barycentrics.x);
    iResult.barycentrics.y = abs(iResult.barycentrics.y);

    // Fetch additional data for closest hit
    if (blasNodePtr != INVALID_NODE)
    {
        const uint64_t tlasNodeAddr = GetNodeAddr(tlasBaseAddr, tlasNodePtr);
        const uint64_t blasNodeAddr = GetNodeAddr(blasBaseAddr, blasNodePtr);

        // fetch primitive data from triangle node
        PrimitiveData primitiveData = FetchPrimitiveDataAddr3_0(blasNodePtr, blasNodeAddr);

        iResult.geometryIndex        = primitiveData.geometryIndex;
        iResult.primitiveIndex       = primitiveData.primitiveIndex;
        iResult.instNodePtr          = tlasNodePtr;
        iResult.hitkind              = intersection.hitKind;
        iResult.instanceContribution = intersection.instanceContribution;

        // Set hit token blas and tlas values
        AmdTraceRaySetHitTokenData(blasNodePtr, tlasNodePtr);

        // Both barycentrics = 1.0 only for procedural primitives
        if ((iResult.barycentrics.x != 1.0f) || (iResult.barycentrics.y != 1.0f))
        {
            AmdTraceRaySetTriangleIntersectionAttributes(iResult.barycentrics.xy);
        }
    }
    else
    {
        // Set hit token blas and tlas values to invalid for miss
        AmdTraceRaySetHitTokenData(INVALID_NODE, INVALID_NODE);
    }

    return iResult;
}

//=====================================================================================================================
static IntersectionResult IntersectRayImpl3_0(
    in GpuVirtualAddress tlasBaseAddr,  ///< Top-level acceleration structure to use
    in RayDesc           ray,           ///< Ray to be traced
    in uint              blasNodePtr,   ///< bottom level node pointer
    in uint              tlasNodePtr,   ///< top level node pointer
    in uint              rayId)         ///< Ray ID for profiling
{
    IntersectionResult result = (IntersectionResult)0;
    result.t           = ray.TMax - ApplyTMinBias(ray.TMin);
    result.nodeIndex   = blasNodePtr;
    result.instNodePtr = tlasNodePtr;

    if ((blasNodePtr != INVALID_NODE) && (tlasNodePtr != INVALID_NODE))
    {
        RayDesc localRay = ray;
        localRay.Origin += ApplyTMinBias(localRay.TMin) * localRay.Direction;
        RayDesc topLevelRay = localRay;

        // Initialise hardware node pointer. Note, we do not need to encode flags here since all flags have been
        // evaluated in the first pass to obtain the hit tokens
        const uint64_t tlasBasePtr = (tlasBaseAddr >> 3);

        // Perform hardware intersection with TLAS to transform the ray
        DualIntersectResult tlasIntersectionResult =
            image_bvh_dual_intersect_ray(tlasBasePtr, tlasNodePtr, INVALID_NODE,
                                         localRay.Origin,
                                         localRay.Direction,
                                         result.t,
                                         0xff,
                                         BoxSortHeuristic::Closest);

        // Extract base address from intersection result vector
        const uint64_t blasBasePtr = PackUint64(tlasIntersectionResult.first.z, tlasIntersectionResult.first.w);

#if DEVELOPER
        if (EnableTraversalCounter())
        {
            WriteRayHistoryTokenBottomLevel(rayId, ExtractInstanceAddr(blasBasePtr));
            WriteRayHistoryTokenNodePtr(rayId, blasNodePtr);
        }
#endif

        DualIntersectResult triangleIntersectionResult =
            image_bvh_dual_intersect_ray(blasBasePtr, blasNodePtr, INVALID_NODE,
                                         localRay.Origin,
                                         localRay.Direction,
                                         result.t,
                                         0xff,
                                         BoxSortHeuristic::Closest);

        const float tNum   = asfloat(triangleIntersectionResult.first.x);
        const float tDenom = asfloat(triangleIntersectionResult.first.y);
        const float tHit   = tNum / tDenom;

        const uint64_t tlasNodeAddr = GetNodeAddr(tlasBasePtr, tlasNodePtr);
        const uint64_t blasNodeAddr = GetNodeAddr(blasBasePtr, blasNodePtr);

        // fetch primitive data from triangle node
        const PrimitiveData primitiveData = FetchPrimitiveDataAddr3_0(blasNodePtr, blasNodeAddr);

        result.primitiveIndex       = primitiveData.primitiveIndex;
        result.geometryIndex        = triangleIntersectionResult.geometryId[0];
        result.t                    = tHit;
        result.barycentrics.x       = abs(asfloat(triangleIntersectionResult.first.z) / tDenom);
        result.barycentrics.y       = abs(asfloat(triangleIntersectionResult.first.w) / tDenom);
        // Instance contribution is stored in the 24 bit user data field of the instance node
        result.instanceContribution = tlasIntersectionResult.second.z;

        result.hitkind = (triangleIntersectionResult.first.y >> 31) | HIT_KIND_TRIANGLE_FRONT_FACE;

        AmdTraceRaySetTriangleIntersectionAttributes(result.barycentrics);
    }

    return result;
}
