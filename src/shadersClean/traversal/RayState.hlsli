/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef RAYSTATE_HLSLI
#define RAYSTATE_HLSLI

#include "../common/Bits.hlsli"
#include "../common/Common.hlsli"
#include "../common/ShaderDefs.hlsli"

#include "DispatchRaysConstants.hlsli"
#include "RayPipelineFlags.hlsli"
#include "Traits.hlsli"
#include "TraversalDefs.hlsli"
#include "Vpc.hlsli"

#define REMAT_INSTANCE_RAY 1

//=====================================================================================================================
// Dispatch system data. This data is shared between all shader stages and is initialized at the raygeneration shader
// stage e.g. remapping rays to threads
struct _AmdDispatchSystemData
{
#if defined(__cplusplus)
    _AmdDispatchSystemData(int val) : dispatchLinearId(val)
    {
    }

    _AmdDispatchSystemData() : _AmdDispatchSystemData(0)
    {
    }
#endif

    // Pack dispatch id (x, y) into the linear id (1 DWORD)
    void PackDispatchId(in uint3 dispatchId)
    {
        const uint3 dims            = GetDispatchRaysDimensions();
        const uint widthBitOffsets  = firstbithigh(dims.x - 1) + 1;
        const uint heightBitOffsets = firstbithigh(dims.y - 1) + 1;

        // Explicit bit operation is more straight forward, and produces better ISA code than bitFieldInsert.
        dispatchLinearId =
            dispatchId.x | (dispatchId.y << widthBitOffsets) | (dispatchId.z << (widthBitOffsets + heightBitOffsets));

        GPU_ASSERT(dispatchLinearId != DISPATCHID_DEAD_STACKFUL && dispatchLinearId != DISPATCHID_DEAD_STACKLESS);
        GPU_ASSERT(all(DispatchId() == dispatchId));
    }

    // Extract (x, y) of dispatch id from the linear id, and (z) from the y of DispatchDims.
    uint3 DispatchId()
    {
        const uint3 dims            = GetDispatchRaysDimensions();
        const uint widthBitOffsets  = firstbithigh(dims.x - 1) + 1;
        const uint heightBitOffsets = firstbithigh(dims.y - 1) + 1;

        uint3 dispatchId;
        dispatchId.x = dispatchLinearId & ((1u << widthBitOffsets) - 1);
        dispatchId.y = (dispatchLinearId >> widthBitOffsets) & ((1u << heightBitOffsets) - 1);
        dispatchId.z = dispatchLinearId >> (widthBitOffsets + heightBitOffsets);

        return dispatchId;
    }

    void SetDead(bool withStack)
    {
        dispatchLinearId = withStack ? DISPATCHID_DEAD_STACKFUL : DISPATCHID_DEAD_STACKLESS;
    }

    bool IsDeadStackless()
    {
        return Traits::HasStacklessDeadLanes() && dispatchLinearId == DISPATCHID_DEAD_STACKLESS;
    }

    bool IsDeadStackful()
    {
        return Traits::HasStackfulDeadLanes() && dispatchLinearId == DISPATCHID_DEAD_STACKFUL;
    }

    bool IsDead()
    {
        // Note: this saves quite a few instructions compared to IsDeadStackless() || IsDeadStackful(),
        // however it relies on DISPATCHID_DEAD_STACKLESS being the lowest of all sentinel values, and
        // these being consecutive (which is not necessarily the case if more sentinel values are added).
        return (Traits::HasStacklessDeadLanes() || Traits::HasStackfulDeadLanes()) && dispatchLinearId >= DISPATCHID_DEAD_STACKLESS;
    }

    uint  dispatchLinearId;   // Packed dispatch linear id. Combine x/y/z into 1 DWORD.

#if DEVELOPER
    uint  parentId;     // Record the parent's dynamic Id for ray history counter, -1 for RayGen shader.
    uint  staticId;     // Record the static Id of current trace ray call site.
#endif
};

//=====================================================================================================================
// Ray system state required for HLSL intrinsics
//
struct _AmdRaySystemState
{
#if defined(__cplusplus)
    _AmdRaySystemState(int val)
    {
        memset(this, val, sizeof(_AmdRaySystemState));
    }
#endif

    void PackAccelStructAndRayflags(in uint64_t accelStruct, in uint rayFlags)
    {
        GPU_ASSERT((accelStruct & ~((1u << 48) - 1)) == 0);
        packedAccelStruct = bitFieldInsert64(packedAccelStruct, 0, 48, accelStruct);
        GPU_ASSERT((rayFlags & ~((1u << 12) - 1)) == 0);
        packedAccelStruct = bitFieldInsert64(packedAccelStruct, 48, 12, rayFlags);
    }

    uint64_t AccelStruct()
    {
        return bitFieldExtract64(packedAccelStruct, 0, 48);
    }

    // Incoming flags are the flags passed by TraceRay call
    uint IncomingFlags()
    {
        return uint(bitFieldExtract64(packedAccelStruct, 48, 12));
    }

    uint Flags()
    {
        return ApplyAllStaticallyKnownFlags(IncomingFlags());
    }

    void SetAnyHitDidAccept(bool value)
    {
        packedAccelStruct = bitFieldInsert64(packedAccelStruct, 60, 1, value);
    }

    bool AnyHitDidAccept()
    {
        return bitFieldExtract64(packedAccelStruct, 60, 1) != 0;
    }

    uint64_t packedAccelStruct;      // Packed accel struct with ray flags
                                     // bits format:
                                     //  0 - 47 : Base address of the top level acceleration structure
                                     // 48 - 59 : Ray flags, only 12-bits are valid.
                                     // 60 - 60 : AnyHitDidAccept. This is used to communicate between IS and AHS.
                                     // 61 - 63 : Reserved
    uint     traceParameters;        // Packed instanceMask, rayContribution, geometryMultiplier, missShaderIndex
    float3   origin;                 // World space ray origin
    float3   direction;              // World space ray direction
    float    tMin;                   // Minimum ray bounds
    float    tMax;
};

//=====================================================================================================================
struct _AmdHitObject
{
#if defined(__cplusplus)
    _AmdHitObject() : ray(val)
    {
    }
#endif

    _AmdRaySystemState ray;
};

//=====================================================================================================================
// Data required for primitive system value intrinsics
//
struct _AmdPrimitiveSystemState
{
#if defined(__cplusplus)
    _AmdPrimitiveSystemState()
    {
    }

    _AmdPrimitiveSystemState(int val) :
        rayTCurrent(val),
        instNodePtr(val),
        primitiveIndex(INVALID_IDX),
        packedGeometryIndex(0),
        packedInstanceContribution(0)
      , currNodePtr(INVALID_IDX)
#if GPURT_DEBUG_CONTINUATION_TRAVERSAL
      , packedType(0)
#endif
    {
    }
#endif
    float  rayTCurrent;   // AMD Gpu shifts the origin, so 0 <= rayTCurrent <= (tMaxApp - tMinApp).
    uint   instNodePtr;   // Current node pointer in top level acceleration structure

    uint primitiveIndex;
    uint packedGeometryIndex;           // geometryIndex:        [23 : 0]
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
                                        // isTri0:               [27]
#endif
                                        // State                 [30 : 28]
                                        // IsOpaque              [31]

    // For procedural primitives, it is a user-defined 8-bit value.
    // For triangles, it is HIT_KIND_TRIANGLE_FRONT_FACE (254) or HIT_KIND_TRIANGLE_BACK_FACE (255)
    uint packedInstanceContribution;    // instanceContribution [23 :  0]
                                        // hitKind              [31 : 24]

    uint currNodePtr;
    void SetCurrNodePtr(uint p)
    {
        currNodePtr = p;
    }

    uint GeometryIndex()
    {
        // To extract "GeometryIndex" - (packedGeometryIndex >> 0) & bits(24);
        return bitFieldExtract(packedGeometryIndex, 0, 24);
    }

    void PackGeometryIndex(uint geometryIndex)
    {
        // To pack "GeometryIndex" - packedGeometryIndex |= (geometryIndex & 0x00FFFFFF);
        packedGeometryIndex = bitFieldInsert(packedGeometryIndex, 0, 24, geometryIndex);
    }

    // Directly construct packedGeometryIndex when all bit-fields are available
    void PackGeometryIndex(
        uint geometryIndex,
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
        bool isTri0,
#endif
        uint state,
        bool isOpaque)
    {
        packedGeometryIndex = geometryIndex |
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
                              (isTri0 << 27) |
#endif
                              (state << 28) |
                              (isOpaque << 31);
    }

    bool IsOpaque()
    {
        // To extract "IsOpaque" flag - (packedGeometryIndex >> 31) & bits(1);
        return bitFieldExtract(packedGeometryIndex, 31, 1);
    }

    void PackIsOpaque(uint val)
    {
        // To pack "IsOpaque" flag - packedGeometryIndex |= val << 31;
        packedGeometryIndex = bitFieldInsert(packedGeometryIndex, 31, 1, val);
    }

    uint State()
    {
        // To extract "State" - (packedGeometryIndex >> 28) & 7;
        return bitFieldExtract(packedGeometryIndex, 28, 3);
    }

    void PackState(uint state)
    {
        // To pack "State" - packedGeometryIndex |= (state & 7) << 28;
        packedGeometryIndex = bitFieldInsert(packedGeometryIndex, 28, 3, state);
    }

#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
    bool IsTri0()
    {
        // To extract "isTri0" flag - (packedGeometryIndex >> 27) & bits(1);
        return bitFieldExtract(packedGeometryIndex, 27, 1);
    }

    void PackIsTri0(uint val)
    {
        // To pack "isTri0" flag - packedGeometryIndex |= val << 27;
        packedGeometryIndex = bitFieldInsert(packedGeometryIndex, 27, 1, val);
    }
#endif

    uint InstanceContribution()
    {
        // To extract "instanceContribution" - (packedInstanceContribution & 0x00FFFFFF)
        return bitFieldExtract(packedInstanceContribution, 0, 24);
    }

    void PackInstanceContribution(uint instContribution)
    {
        // To pack "instanceContribution" - packedInstanceContribution |= (instContribution & 0x00FFFFFF);
        packedInstanceContribution = bitFieldInsert(packedInstanceContribution, 0, 24, instContribution);
    }

    // Directly construct packedInstanceContribution when all bit-fields are available
    void PackInstanceContribution(uint instContribution, uint hitKind)
    {
        packedInstanceContribution = instContribution | (hitKind << 24);
    }

    uint HitKind()
    {
        // To extract -  (packedInstanceContribution >> 24);
        return bitFieldExtract(packedInstanceContribution, 24, 8);
    }

    void PackHitKind(uint hitKind)
    {
        // To Pack - packedInstanceContribution |= ((hitKind & 0xFF) << 24);
        packedInstanceContribution = bitFieldInsert(packedInstanceContribution, 24, 8, hitKind);
    }

#if GPURT_DEBUG_CONTINUATION_TRAVERSAL
    // The following member data are only used in DEBUG
    uint packedType;        // IsProcedural:   [31]    - 1 bit
                            // AnyhitCallType: [1 : 0] - 2 bits

    bool IsProcedural()
    {
        // To extract -  (packedType >> 31) & bits(1);
        return bitFieldExtract(packedType, 31, 1);
    }

    void PackIsProcedural(bool isProceralNode)
    {
        // To Pack - packedType |= ((isProceralNode & bits(1)) << 31);
        packedType = bitFieldInsert(packedType, 31, 1, isProceralNode);
    }

    uint AnyHitCallType()
    {
        // To extract -  (packedType >> 0) & bits(2);
        return bitFieldExtract(packedType, 0, 2);
    }

    void PackAnyHitCallType(uint anyHitType)
    {
        // To Pack - packedType |= (anyHitType & bits(2));
        packedType = bitFieldInsert(packedType, 0, 2, anyHitType);
    }
#endif
};

//=====================================================================================================================
// Traversal state required for resumable traversal loop implementation and for various HLSL system/ray/primitive
// intrinsics
struct _AmdTraversalState
{
#if defined(__cplusplus)
    _AmdTraversalState(int val)
    {
        memset(this, val, sizeof(_AmdTraversalState));
    }
#endif

    // The committed state stores the final hit information for closestHit/Miss shaders.
    _AmdPrimitiveSystemState committed;

    // The following state is used for triangle hits and procedural geometry. Note, this state can be moved into the
    // register space reserved for ray attributes in general
    float2 committedBarycentrics;

    uint nextNodePtr;   // Next node pointer
    uint instNodePtr;

    // Traversal stack state. Note, on some hardware this data represents a packed stack pointer that will
    // be unpacked at the beginning of the traversal routine

    // In RTIP2_0, stackPtr is packed with both stack ptr and stack ptr top.
    // bits format:
    // 0 - 15:  stack_base
    // 16 - 23: stack_index
    // 24 - 31: stack_index_top
    uint stackPtr;
    uint packedStackTopOrParentPointer;     // stackPtrTop (RTIP1.1, 2.0)

#if REMAT_INSTANCE_RAY == 0
    // Following state represents the transformed ray in object space for resuming from a candidate
    // hit inside a bottom level acceleration structure. This state can be in local registers if
    // no anyHit/intersection shaders are involved. Optionally, this can be rematerialized from the
    // instance node by retransforming the world space ray on every resumption. This can be quite expensive
    // on existing hardware.
    float3 candidateRayOrigin;
    float3 candidateRayDirection;
#endif

    // Optional state for advanced features/optimizations
    uint lastInstanceRootNodePtr; // Only for rebraid mode.
                                  // This field currently discarded in non-rebraid mode. Ensure to update that if the
                                  // field becomes re-used for something else in non-rebraid mode.
    uint reservedNodePtr;         // RTIPv2.0 (lastNodePtr)
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
    uint packedInstanceContribution;    // instanceContribution  [23 :  0] RTIP3.1 ( instanceContribution )
                                        // skipTri0              [31]
#endif

    uint32_t packedReturnAddr; // The address of the function to return to, packed into 32 bits.

    uint InstanceContribution()
    {
        uint ret = 0;
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
        GPU_ASSERT(GetRtIpLevel() == RayTracingIpLevel::RtIp3_1);

        // To extract - (packedInstanceContribution & 0x00FFFFFF);
        ret =  bitFieldExtract(packedInstanceContribution, 0, 24);
#endif
        return ret;
    }

    void PackInstanceContribution(uint instContribution)
    {
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
        GPU_ASSERT(GetRtIpLevel() == RayTracingIpLevel::RtIp3_1);

        // To pack - packedInstanceContribution |= (instContribution & 0x00FFFFFF);
        packedInstanceContribution = bitFieldInsert(packedInstanceContribution, 0, 24, instContribution);
#endif
    }

#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
#endif
    bool SkipTri0()
    {
        uint ret = 0;
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
        GPU_ASSERT(GetRtIpLevel() == RayTracingIpLevel::RtIp3_1);

        // To extract - (packedInstanceContribution >> 31);
        ret = bitFieldExtract(packedInstanceContribution, 31, 1);
#endif
        return ret;
    }

    void PackSkipTri0(bool val)
    {
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
        GPU_ASSERT(GetRtIpLevel() == RayTracingIpLevel::RtIp3_1);

        // To pack - packedInstanceContribution |= (val & bits(1)) << 31;
        packedInstanceContribution = bitFieldInsert(packedInstanceContribution, 31, 1, val);
#endif
    }

    // Combined setter for instContribution and skipTri0 that prevents a dependency on the old value.
    void PackInstanceContributionAndSkipTri0(uint instContribution, bool skipTri0)
    {
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
        GPU_ASSERT(GetRtIpLevel() == RayTracingIpLevel::RtIp3_1);
        packedInstanceContribution = instContribution | (uint(skipTri0) << 31);
#endif
    }

    void PackStackPtrTop(uint ptr)
    {
        GPU_ASSERT((GetRtIpLevel() == RayTracingIpLevel::RtIp1_1) ||
                   (GetRtIpLevel() == RayTracingIpLevel::RtIp2_0));

        packedStackTopOrParentPointer = ptr;
    }

    uint StackPtrTop()
    {
        GPU_ASSERT((GetRtIpLevel() == RayTracingIpLevel::RtIp1_1) ||
                   (GetRtIpLevel() == RayTracingIpLevel::RtIp2_0));
        return packedStackTopOrParentPointer;
    }

    void PackParentPointer(uint ptr)
    {
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
        GPU_ASSERT(GetRtIpLevel() == RayTracingIpLevel::RtIp3_1);
#endif
        packedStackTopOrParentPointer = ptr;
    }

    uint ParentPointer()
    {
        uint ptr = 0;
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
        GPU_ASSERT(GetRtIpLevel() == RayTracingIpLevel::RtIp3_1);
#endif
        ptr = packedStackTopOrParentPointer;
        return ptr;
    }

    uint CommittedState()
    {
        return committed.State();
    }

    void SetReturnAddress(Vpc32 returnAddr)
    {
        packedReturnAddr = returnAddr.GetU32();
    }

    Vpc32 ReturnAddress()
    {
        return Vpc32(packedReturnAddr);
    }
};

#if DEVELOPER
//=====================================================================================================================
struct _AmdRayHistoryCounter
{
    void SetCallerShaderType(DXILShaderKind type)
    {
        packedCallerShaderType = bitFieldInsert(packedCallerShaderType, 0, 31, (uint)type);
    }

    DXILShaderKind CallerShaderType()
    {
        return (DXILShaderKind)bitFieldExtract(packedCallerShaderType, 0, 31);
    }

    void SetWriteTokenTopLevel(bool value)
    {
        packedCallerShaderType = bitFieldInsert(packedCallerShaderType, 31, 1, value);
    }

    bool WriteTokenTopLevel()
    {
        return bitFieldExtract(packedCallerShaderType, 31, 1) != 0;
    }

    uint64_t timerBegin;
    uint     dynamicId;
    float    candidateTCurrent;
    uint     packedCallerShaderType;    // bit 0~30: caller shader type
                                        // bit 31: flag to write TLAS
    uint     numRayBoxTest;
    uint     numRayTriangleTest;
    uint     numIterations;
    uint     maxStackDepth;
    uint     numAnyHitInvocation;
    uint     shaderIdLow;
    uint     shaderRecIdx;
    uint     timer;
    uint     numCandidateHits;
    uint     instanceIntersections;
};
#endif

//=====================================================================================================================
struct _AmdSystemData
{
#if defined(__cplusplus)
    _AmdSystemData(int val) : dispatch(val), hitObject(val), traversal(val)
    {
    }
#endif

    bool IsTraversal()
    {
        GPU_ASSERT(!dispatch.IsDead());
        return IsValidNode(traversal.nextNodePtr);
    }

    bool IsChsOrMiss(in uint state)
    {
        GPU_ASSERT(!dispatch.IsDead());
        return (state >= TRAVERSAL_STATE_COMMITTED_NOTHING);
    }

    bool IsMiss(in uint state)
    {
        GPU_ASSERT(!dispatch.IsDead());
        return IsChsOrMiss(state) && !IsValidNode(traversal.committed.instNodePtr);
    }

    bool IsAhs(in uint state)
    {
        GPU_ASSERT(!dispatch.IsDead());
        return (state == TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_TRIANGLE);
    }

    bool IsIs(in uint state)
    {
        GPU_ASSERT(!dispatch.IsDead());
        return ((state == TRAVERSAL_STATE_CANDIDATE_PROCEDURAL_PRIMITIVE) ||
                (state == TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_PROCEDURAL_PRIMITIVE));
    }

    bool IsChs(in uint state)
    {
        GPU_ASSERT(!dispatch.IsDead());
        return IsChsOrMiss(state) && IsValidNode(traversal.committed.instNodePtr);
    }

    // Note: _AmdDispatchSystemData must be the first member of _AmdSystemData. This allows us to save some VGPRs if
    //       we need to call a function that takes _AmdSystemData but doesn't actually need ray or traversal data.
    _AmdDispatchSystemData dispatch;
    _AmdHitObject          hitObject;
    _AmdTraversalState     traversal;
#if DEVELOPER
    _AmdRayHistoryCounter  counter;
#endif
};

//=====================================================================================================================
struct _AmdAnyHitSystemData
{
#if defined(__cplusplus)
    _AmdAnyHitSystemData(int val) : base(val), candidate(val)
    {
    }
#endif

    _AmdSystemData base;

    // The candidate state holds temporary candidate hit information for AnyHit/Intersection shaders. Required for
    // resuming traversal from AnyHit/Intersection shader calls. Also holds intermediate traversal data
    _AmdPrimitiveSystemState candidate;
};

#undef GPURT_HAS_SRID

#endif
