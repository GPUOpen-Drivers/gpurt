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

// Set in the BLAS node pointer to indicate which triangle was hit. This is required for the hit token extension.
#define TRI1_NODE_FLAG NODE_POINTER_MASK_MSB

//=====================================================================================================================
static uint ProcessTriangleNode(
    in uint                        rayId,
    inout_param(IntersectionState) intersection,
    Bvh8IntersectResult            result,
    uint                           instanceContribution,
    uint                           packedTraceRayParameters,
    uint64_t                       tlasBaseAddr,
    uint64_t                       blasBaseAddr,
    uint                           tlasNodePtr,
    uint                           blasNodePtr,
    uint                           rayFlags,
    bool                           tri0)
{
    const bool rayForceOpaque    = (rayFlags & RAY_FLAG_FORCE_OPAQUE);
    const bool raySkipProcedural = (rayFlags & RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES);

    const bool isProcedural = !raySkipProcedural && result.IsProcedural(tri0);
    const bool runAnyHit    = !rayForceOpaque && result.IsNonOpaque(tri0);

    float tHit = result.T(tri0);

    uint status = (tHit < intersection.t) ? HIT_STATUS_ACCEPT : HIT_STATUS_IGNORE;

    if ((tHit < intersection.t) || isProcedural)
    {
        uint hitKind        = result.HitKind(tri0);
        uint primitiveIndex = result.PrimitiveIndex(tri0);
        uint geometryIndex  = result.GeometryIndex(tri0);

        if (!raySkipProcedural || !rayForceOpaque)
        {
            if (isProcedural || runAnyHit)
            {
                const uint64_t tlasNodePtr64 = CalculateInstanceNodePtr64(GPURT_RTIP3_1,
                                                                          ExtractInstanceAddr(tlasBaseAddr),
                                                                          tlasNodePtr);

                const uint anyHitCallType = isProcedural ?
                                            (runAnyHit ? ANYHIT_CALLTYPE_DUPLICATE : ANYHIT_CALLTYPE_SKIP) :
                                            ANYHIT_CALLTYPE_NO_DUPLICATE;

                AmdTraceRaySetHitAttributes((isProcedural ? intersection.t : tHit),
                                            hitKind,
                                            (isProcedural ? HIT_STATUS_IGNORE : HIT_STATUS_ACCEPT),
                                            LowPart(tlasNodePtr64),
                                            HighPart(tlasNodePtr64),
                                            primitiveIndex,
                                            anyHitCallType,
                                            geometryIndex);

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
                    BuiltInTriangleIntersectionAttributes attr;
                    attr.barycentrics.x = result.BaryI(tri0);
                    attr.barycentrics.y = result.BaryJ(tri0);
                    AmdTraceRayCallTriangleAnyHitShader(hitInfo.anyHitId, hitInfo.tableIndex, attr);
                }

                // Read back attributes that may have been modified by the Intersection / AnyHit shader
                AmdTraceRayGetHitAttributes(tHit, hitKind, status);

#if DEVELOPER
                if (EnableTraversalCounter())
                {
                    if (isProcedural)
                    {
                        WriteRayHistoryTokenFunctionCall(rayId,
                                                         hitInfo.intersectionId,
                                                         hitInfo.tableIndex,
                                                         RAY_HISTORY_FUNC_CALL_TYPE_INTERSECTION);

                        WriteRayHistoryTokenProceduralIntersectionStatus(rayId, status, tHit, hitKind);
                    }
                    else if (runAnyHit && (PackUint64(hitInfo.anyHitId) != 0))
                    {
                        WriteRayHistoryTokenFunctionCall(rayId,
                                                         hitInfo.anyHitId,
                                                         hitInfo.tableIndex,
                                                         RAY_HISTORY_FUNC_CALL_TYPE_ANY_HIT);
                        WriteRayHistoryTokenAnyHitStatus(rayId, status);
                    }
                }
#endif
            }
        }

#if DEVELOPER
        if (EnableTraversalCounter() && (isProcedural == false))
        {
            WriteRayHistoryTokenTriangleHitResult(rayId,
                                                  uint(status > HIT_STATUS_IGNORE),
                                                  tHit);
        }
#endif

        if (status != HIT_STATUS_IGNORE)
        {
            intersection.t                     = tHit;
            intersection.barycentrics.x        = result.BaryI(tri0);
            intersection.barycentrics.y        = result.BaryJ(tri0);
            intersection.hitKind               = hitKind;
            intersection.instanceContribution  = instanceContribution;
            intersection.primitiveIndex        = primitiveIndex;
            intersection.geometryIndex         = geometryIndex;
        }
    }

    return status;
}

//=====================================================================================================================
static IntersectionResult TraceRayImpl3_1(
    in GpuVirtualAddress tlasBaseAddr,              ///< Top-level acceleration structure to use
    in uint              rayFlags,                  ///< Ray flags
    in uint              traceRayParameters,        ///< Packed trace ray parameters
    in RayDesc           ray,                       ///< Ray to be traced
    in uint              rayId                      ///< Ray ID for profiling
)
{
    // Start from root node which follows acceleration structure header
    const uint rootNodePtr  = CreateRootNodePointer3_1();
    const uint instanceMask = (traceRayParameters & 0xff);

    uint boxHeuristicMode = AmdTraceRayGetBoxSortHeuristicMode();
    if ((boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosest) ||
        (boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosestMidPoint))
    {
        boxHeuristicMode = GetBoxSortingHeuristicFromRayFlags(rayFlags, boxHeuristicMode);
    }

    // Temporary locals for traversal
    RayDesc localRay = ray;
    localRay.Origin += localRay.TMin * localRay.Direction;
    RayDesc topLevelRay = localRay;

    uint stackAddr = RtIp3LdsStackInit();

    uint nodePtr = rootNodePtr;

    uint instNodePtr      = INVALID_NODE;
    uint lastNodePtr      = INVALID_NODE;
    uint lastInstanceNode = INVALID_NODE;
    uint parentNodePtr    = INVALID_NODE;

    const float tMax = ray.TMax - ray.TMin;

    // Committed traversal state
    IntersectionState intersection;
    intersection.t                    = tMax;
    intersection.barycentrics.x       = 0.0f;
    intersection.barycentrics.y       = 0.0f;
    intersection.hitKind              = 0u;
    intersection.instanceContribution = 0u;
    intersection.primitiveIndex       = 0xFFFFFFFFu;
    intersection.geometryIndex        = 0xFFFFFFFFu;

    uint instanceContribution = 0;
    uint blasNodePtr          = INVALID_NODE;
    uint tlasNodePtr          = INVALID_NODE;

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
            else if (IsTriangleNode3_1(nodePtr))
            {
                iResult.numRayTriangleTest++;
            }
            else if (IsBoxNode3_1(nodePtr))
            {
                iResult.numRayBoxTest++;
            }

            UpdateWaveTraversalStatistics(GPURT_RTIP3_1, nodePtr);
        }
#endif
        const float rayExtent = (lastNodePtr == INVALID_NODE) ? intersection.t : tMax;

        Bvh8IntersectResult result = image_bvh8_intersect_ray_3_1(currBaseAddr,
                                                                  nodePtr,
                                                                  localRay.Origin,
                                                                  localRay.Direction,
                                                                  rayExtent,
                                                                  instanceMask,
                                                                  boxHeuristicMode);

#if GPURT_ENABLE_GPU_DEBUG
        // Check if the child pointers returned are valid.
        if (IsBoxNode3_1(nodePtr))
        {
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot0.x, currBaseAddr, tlasBaseAddr);
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot0.y, currBaseAddr, tlasBaseAddr);
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot0.z, currBaseAddr, tlasBaseAddr);
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot0.w, currBaseAddr, tlasBaseAddr);

            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot1.x, currBaseAddr, tlasBaseAddr);
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot1.y, currBaseAddr, tlasBaseAddr);
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot1.z, currBaseAddr, tlasBaseAddr);
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1, result.slot1.w, currBaseAddr, tlasBaseAddr);
        }
#endif

        if (IsUserNodeInstance(nodePtr))
        {
            instNodePtr = nodePtr;
            currBaseAddr = result.InstanceChildBasePtr();
            // Instance contribution is stored in the 24 bit user data field of the instance node
            instanceContribution = result.InstanceHitContribution();
            result.slot1.w = DecodeBlasRootFromInstanceResult(result.slot1.w);
            lastInstanceNode = result.slot1.w;
            // result.slot1.z to SKIP_0_7 in order to skip the remaining inputs to ds_stack_push.
            result.slot1.z = SKIP_0_7;

#if DEVELOPER
            if (EnableTraversalCounter() && (result.slot1.w != INVALID_NODE))
            {
                WriteRayHistoryTokenBottomLevel(rayId, ExtractInstanceAddr(currBaseAddr));
            }
#endif
        }
        else if (IsTriangleNode3_1(nodePtr))
        {
            const bool rayForceOpaque = (rayFlags & RAY_FLAG_FORCE_OPAQUE);
            const bool raySkipProcedural = (rayFlags & RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES);

            const float t0 = result.T(true);
            const float t1 = result.T(false);
            const bool closestTri0 = (t0 <= t1);

            // Special case opaque triangle rays. We only need to process one of the results.
            // This avoid branching in the core of the loop and reduces overall VALU when a pair.
            // of triangles is intersected.
            if (rayForceOpaque && raySkipProcedural)
            {
                uint status = ProcessTriangleNode(rayId,
                                                  intersection,
                                                  result,
                                                  instanceContribution,
                                                  traceRayParameters,
                                                  tlasBaseAddr,
                                                  currBaseAddr,
                                                  instNodePtr,
                                                  nodePtr,
                                                  rayFlags,
                                                  closestTri0);

                if (status != HIT_STATUS_IGNORE)
                {
                    tlasNodePtr = instNodePtr;
                    blasBaseAddr = currBaseAddr;
                    blasNodePtr = nodePtr;

                    if (rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH)
                    {
                        break;
                    }
                }
            }
            else
            {
                // Process Tri0 first
                uint status = ProcessTriangleNode(rayId,
                                                  intersection,
                                                  result,
                                                  instanceContribution,
                                                  traceRayParameters,
                                                  tlasBaseAddr,
                                                  currBaseAddr,
                                                  instNodePtr,
                                                  nodePtr,
                                                  rayFlags,
                                                  closestTri0);

                if (status != HIT_STATUS_IGNORE)
                {
                    tlasNodePtr = instNodePtr;
                    blasBaseAddr = currBaseAddr;
                    blasNodePtr = nodePtr;

                    if ((status == HIT_STATUS_ACCEPT_AND_END_SEARCH) ||
                        (rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH))
                    {
                        break;
                    }
                }

                // Process Tri1
                status = ProcessTriangleNode(rayId,
                                             intersection,
                                             result,
                                             instanceContribution,
                                             traceRayParameters,
                                             tlasBaseAddr,
                                             currBaseAddr,
                                             instNodePtr,
                                             nodePtr,
                                             rayFlags,
                                             !closestTri0);

                if (status != HIT_STATUS_IGNORE)
                {
                    tlasNodePtr  = instNodePtr;
                    blasBaseAddr = currBaseAddr;
                    blasNodePtr  = nodePtr | TRI1_NODE_FLAG;

                    if ((status == HIT_STATUS_ACCEPT_AND_END_SEARCH) ||
                        (rayFlags & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH))
                    {
                        break;
                    }
                }
            }

            result.slot1.w = nodePtr;

            // NavigationBits is only encoded in tri0's return data.
            const uint navigationBits = result.NavigationBits(true);
            lastNodePtr = PRIM_RANGE_UPDATE | navigationBits;
        }
        else
        {
#if GPURT_ENABLE_GPU_DEBUG
            GPU_ASSERT(IsBoxNode3_1(nodePtr));
#endif
            parentNodePtr = nodePtr;
        }

        // backup previous node pointer
        uint prevNodePtr = nodePtr;

        nodePtr = ds_stack_push8_pop1(stackAddr, lastNodePtr, result.slot0, result.slot1, GPURT_RTIP3_1);
        lastNodePtr = INVALID_NODE;

#if GPURT_ENABLE_GPU_DEBUG
        // Check if the node pointer popped from stack is valid.
        OutOfRangeNodePointerAssert(GPURT_RTIP3_1,
                                    nodePtr,
                                    (stackAddr & STACK_ADDR_BLAS_TO_TLAS_MASK) ? tlasBaseAddr : currBaseAddr,
                                    tlasBaseAddr);
#endif

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

            lastNodePtr = prevNodePtr;

            if (IsTriangleNode3_1(prevNodePtr))
            {
                nodePtr = parentNodePtr;
            }
            else
            {
                nodePtr = FetchParentNodePointer3_1(ExtractInstanceAddr(currBaseAddr), prevNodePtr);
            }

#if GPURT_ENABLE_GPU_DEBUG
            // Check if parent pointer fetched is valid.
            OutOfRangeNodePointerAssert(GPURT_RTIP3_1,
                                        nodePtr,
                                        (stackAddr & STACK_ADDR_BLAS_TO_TLAS_MASK) ? tlasBaseAddr : currBaseAddr,
                                        tlasBaseAddr);
#endif
        }

        // Reset ray on transition from BLAS to TLAS
        if (stackAddr & STACK_ADDR_BLAS_TO_TLAS_MASK)
        {
            localRay.Direction = topLevelRay.Direction;
            localRay.Origin    = topLevelRay.Origin;
            currBaseAddr       = tlasBaseAddr;

#if DEVELOPER
            if (EnableTraversalCounter())
            {
                WriteRayHistoryTokenTopLevel(rayId, ExtractInstanceAddr(currBaseAddr));
            }
#endif
        }

    }

    iResult.t            = intersection.t;
    iResult.barycentrics = intersection.barycentrics;
    iResult.nodeIndex    = blasNodePtr;

    // Fetch additional data for closest hit
    if (blasNodePtr != INVALID_NODE)
    {
        iResult.geometryIndex        = intersection.geometryIndex;
        iResult.primitiveIndex       = intersection.primitiveIndex;
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
static IntersectionResult IntersectRayImpl3_1(
    in GpuVirtualAddress tlasBaseAddr,  ///< Top-level acceleration structure to use
    in RayDesc           ray,           ///< Ray to be traced
    in uint              blasNodePtr,   ///< bottom level node pointer
    in uint              tlasNodePtr,   ///< top level node pointer
    in uint              rayId)         ///< Ray ID for profiling
{
    IntersectionResult result = (IntersectionResult)0;
    result.t           = ray.TMax - ray.TMin;
    result.nodeIndex   = blasNodePtr;
    result.instNodePtr = tlasNodePtr;

    if ((blasNodePtr != INVALID_NODE) && (tlasNodePtr != INVALID_NODE))
    {
        RayDesc localRay = ray;
        localRay.Origin += localRay.TMin * localRay.Direction;
        RayDesc topLevelRay = localRay;

        // Initialise hardware node pointer. Note, we do not need to encode flags here since all flags have been
        // evaluated in the first pass to obtain the hit tokens
        const uint64_t tlasBasePtr = (tlasBaseAddr >> 3);

        // Perform hardware intersection with TLAS to transform the ray
        Bvh8IntersectResult tlasIntersectionResult =
            image_bvh8_intersect_ray_3_1(tlasBasePtr,
                                         tlasNodePtr,
                                         localRay.Origin,
                                         localRay.Direction,
                                         result.t,
                                         0xff,
                                         BoxSortHeuristic::Closest);

        // Extract base address from intersection result vector
        const uint64_t blasBasePtr = tlasIntersectionResult.InstanceChildBasePtr();

#if DEVELOPER
        if (EnableTraversalCounter())
        {
            WriteRayHistoryTokenBottomLevel(rayId, ExtractInstanceAddr(blasBasePtr));
            WriteRayHistoryTokenNodePtr(rayId, blasNodePtr);
        }
#endif

        const bool tri0 = (blasNodePtr & TRI1_NODE_FLAG) == 0;
        blasNodePtr &= ~TRI1_NODE_FLAG;

        Bvh8IntersectResult triangleIntersectionResult =
            image_bvh8_intersect_ray_3_1(blasBasePtr,
                                         blasNodePtr,
                                         localRay.Origin,
                                         localRay.Direction,
                                         result.t,
                                         0xff,
                                         BoxSortHeuristic::Closest);

        const bool isProcedural     = triangleIntersectionResult.IsProcedural(tri0);
        result.primitiveIndex       = triangleIntersectionResult.PrimitiveIndex(tri0);
        result.geometryIndex        = triangleIntersectionResult.GeometryIndex(tri0);
        result.t                    = triangleIntersectionResult.T(tri0);
        result.barycentrics.x       = triangleIntersectionResult.BaryI(tri0);
        result.barycentrics.y       = triangleIntersectionResult.BaryJ(tri0);

        result.instanceContribution = tlasIntersectionResult.InstanceHitContribution();

        result.hitkind = triangleIntersectionResult.HitKind(tri0);

        AmdTraceRaySetTriangleIntersectionAttributes(result.barycentrics);
    }

    return result;
}
