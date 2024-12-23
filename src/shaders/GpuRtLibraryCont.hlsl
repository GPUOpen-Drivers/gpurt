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

// Include intrinsics and defines from the compiler
#include "llpc/GpurtIntrinsics.h"
#if DEVELOPER
#include "../../gpurt/gpurtCounter.h"
#endif

#include "../shadersClean/common/Math.hlsli"
#include "../shadersClean/common/InstanceDesc.hlsli"

// Do not use ~0 as an invalid stack pointer, to leave it free to use as a sentinel value
#define CPS_STACK_PTR_STACKLESS_DEAD_LANE (~uint32_t(1))
// CPS Stack pointers are dword-aligned, so we can use up to 2 bits. Use the second bit
// to flag a dead lane, so in particular CPS_STACK_PTR_STACKLESS_DEAD_LANE identifies a dead lane
#define CPS_STACK_PTR_DEAD_LANE_FLAG (2)
#define CPS_STACK_PTR_INVALID (CPS_STACK_PTR_STACKLESS_DEAD_LANE & ~CPS_STACK_PTR_DEAD_LANE_FLAG)

#define DEAD_SHADER_ADDR (~uint32_t(0))

// The total w * h * d of the dispatch must be <= 2^30.
// For DX, this is explicitly stated in the documentation of ID3D12GraphicsCommandList4::DispatchRays.
// For Vulkan, it is set in VkPhysicalDeviceRayTracingPipelinePropertiesKHR, which itself is set
// to RayTraceRayGenShaderThreads = 0x40000000, defined in gpurt.h.
// We bit-pack the dispatch id into a 32 bit integer. It can be shown that all 32-bit integers with one or
// less not-set bits do not yield a valid dispatch id (i.e. they imply a dispatch size > 2^30).
// So ~0 and ~0 - (1 << n) are all free to use.
//
// Note: when adding/changing sentinel values, make sure to update _AmdDispatchSystemData::IsDead.
#define DISPATCHID_DEAD_STACKFUL  0xffffffff
#define DISPATCHID_DEAD_STACKLESS 0xfffffffe

static bool RtIpIsAtLeast(RayTracingIpLevel level)
{
    return ((uint32_t)GetRtIpLevel()) >= ((uint32_t)level);
}

#ifndef __cplusplus
#endif

#define REMAT_INSTANCE_RAY 1

//=====================================================================================================================
// Alternate encoding of state bits (TODO: See impact on generated ISA)
//
// 0: procedural
// 1: opaque
// 2: committed
// 3: no-duplicate anyhit
//
// 0: non_opaque_triangle
// 1: non_opaque_procedural
// 2: opaque_triangle
// 3: opaque_procedural
// 4: committed_non_opaque_triangle
// 5: committed_non_opaque_procedural
// 6: committed_opaque_triangle
// 7: committed_opaque_procedural
//
//=====================================================================================================================
// CANDIDATE_STATUS = (TRAVERSAL_STATE - 1)
#define TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_TRIANGLE 0
#define TRAVERSAL_STATE_CANDIDATE_PROCEDURAL_PRIMITIVE 1
#define TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_PROCEDURAL_PRIMITIVE 2
#define TRAVERSAL_STATE_CANDIDATE_NO_DUPLICATE_ANYHIT_PROCEDURAL_PRIMITIVE 3

// COMMITTED_STATUS = (TRAVERSAL_STATE - 4)
#define TRAVERSAL_STATE_COMMITTED_NOTHING 4
#define TRAVERSAL_STATE_COMMITTED_TRIANGLE_HIT 5
#define TRAVERSAL_STATE_COMMITTED_PROCEDURAL_PRIMITIVE_HIT 6

// Forward declaration for _AmdDispatchSystemData.PackDispatchId() and _AmdDispatchSystemData.DispatchId()
static uint3 GetDispatchRaysDimensions();

//=====================================================================================================================
// Apply the known set/unset bits
static uint ApplyKnownFlags(
    uint incomingFlags)
{
    uint flags = incomingFlags;

#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION  >= 41
    // Apply known bits common to all TraceRay calls
    flags = ((flags & ~AmdTraceRayGetKnownUnsetRayFlags()) | AmdTraceRayGetKnownSetRayFlags());
#endif

    // Apply options overrides
    flags &= ~Options::getRayFlagsOverrideForceDisableMask();
    flags |=  Options::getRayFlagsOverrideForceEnableMask();

    return flags;
}

//=====================================================================================================================
// Apply compile time pipeline config flags only, it does not apply known common flags from TraceRay call sites
static uint ApplyCompileTimePipelineConfigFlags(
    uint incomingFlags)
{
    uint flags = incomingFlags;

    flags |= (AmdTraceRayGetStaticFlags() & (PIPELINE_FLAG_SKIP_PROCEDURAL_PRIMITIVES | PIPELINE_FLAG_SKIP_TRIANGLES));
#if DEVELOPER
    flags |= DispatchRaysConstBuf.profileRayFlags;
#endif

    return flags;
}

//=====================================================================================================================
// Apply all static known flags, include both compile time pipeline config flags and known set/unset bits
static uint ApplyAllStaticallyKnownFlags(
    uint incomingFlags)     // The flags from TraceRay call sites,
                            // 0 means get Pipeline flags for all shaders in this pipeline
{
    return ApplyCompileTimePipelineConfigFlags(ApplyKnownFlags(incomingFlags));
}

//=====================================================================================================================
// Get the box sort heuristic mode according to the pipeline flags
static uint GetBoxHeuristicMode()
{
    uint boxHeuristicMode = AmdTraceRayGetBoxSortHeuristicMode();
    if ((boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosest) ||
        (boxHeuristicMode == BoxSortHeuristic::LargestFirstOrClosestMidPoint))
    {
        boxHeuristicMode = GetBoxSortingHeuristicFromRayFlags(ApplyAllStaticallyKnownFlags(0), boxHeuristicMode);
    }

    return boxHeuristicMode;
}

//=====================================================================================================================
// The VPC (vector program counter) is an opaque compiler-controlled 32-bit value.
// The low 6 bits are used for storing metadata, the high 26 bits store a function address.
struct Vpc32 {
    uint32_t vpc;

#if defined(__cplusplus)
    Vpc32(uint32_t value) : vpc(value) {}
#endif

    uint32_t GetU32()
    {
        return vpc;
    }

    uint32_t GetFunctionAddr()
    {
        return (uint32_t)(vpc & 0xFFFFFFC0);
    }

    bool IsValid()
    {
        return vpc != 0;
    }
};

//=====================================================================================================================
// Traits used to abstract away the combination of flags that toggle a feature. Currently used for dead lanes.
namespace Traits
{

static bool HasStacklessDeadLanes()
{
    return false;
}

static bool HasStackfulDeadLanes()
{
    return Options::getPersistentLaunchEnabled();
}

static bool HasDeadLanes()
{
    return HasStackfulDeadLanes() || HasStacklessDeadLanes();
}

} // namespace Traits

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
        uint state,
        bool isOpaque)
    {
        packedGeometryIndex = geometryIndex |
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

    uint32_t packedReturnAddr; // The address of the function to return to, packed into 32 bits.

    uint InstanceContribution()
    {
        uint ret = 0;
        return ret;
    }

    void PackInstanceContribution(uint instContribution)
    {
    }

    bool SkipTri0()
    {
        uint ret = 0;
        return ret;
    }

    void PackSkipTri0(bool val)
    {
    }

    // Combined setter for instContribution and skipTri0 that prevents a dependency on the old value.
    void PackInstanceContributionAndSkipTri0(uint instContribution, bool skipTri0)
    {
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
        packedStackTopOrParentPointer = ptr;
    }

    uint ParentPointer()
    {
        uint ptr = 0;
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
    _AmdSystemData(int val) : dispatch(val), ray(val), traversal(val)
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

    _AmdRaySystemState     ray;
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

//=====================================================================================================================
// Used in the debug path. Contains the minimum data not covered by _AmdSystemData.
struct _AmdTraversalResultData
{
#if defined(__cplusplus)
    _AmdTraversalResultData(int val) : candidate(val), candidateBarycentrics(val), state(val)
    {
    }
#endif

    _AmdPrimitiveSystemState candidate;

    float2 candidateBarycentrics;

    uint   state; // state of a single traversal call:
                  // 1) if all primitives are opaque, the state when the entire BVH traversal is done.
                  // 2) otherwise the first hitted non-opaque primitive.
};

#if ((GPURT_DEBUG_CONTINUATION_TRAVERSAL == 0) && (!defined(__cplusplus)))
// Define specialized intrinsics.
// We use macros because HLSL does not have varargs or generics.
// The macros and intrinsics are defined by llpc.
DECLARE_ENQUEUE(, uint32_t shaderRecIdx, uint32_t returnAddr, _AmdSystemData data)

DECLARE_ENQUEUE(Traversal, uint32_t shaderRecIdx, uint32_t dummyReturnAddr, _AmdSystemData data)
DECLARE_ENQUEUE(TraversalDead, uint32_t shaderRecIdx, uint32_t dummyReturnAddr, _AmdDispatchSystemData data)
DECLARE_ENQUEUE(RayGen, uint32_t shaderRecIdx, uint32_t dummyReturnAddr, _AmdDispatchSystemData data)

DECLARE_ENQUEUE(AnyHit, uint32_t shaderRecIdx, uint32_t returnAddr, _AmdAnyHitSystemData data, float2 candidateBarycentrics)
DECLARE_ENQUEUE(Intersection, uint32_t shaderRecIdx, uint32_t returnAddr, _AmdAnyHitSystemData data)

DECLARE_AWAIT(AnyHit, _AmdAnyHitSystemData, uint32_t shaderRecIdx, uint32_t returnAddr, _AmdAnyHitSystemData data)
DECLARE_AWAIT(CallShader, _AmdDispatchSystemData, uint32_t shaderRecIdx, uint32_t returnAddr, _AmdDispatchSystemData data)

// Pass a dummy return address for consistency reasons.
// The actual return address is included in the passed system data.
DECLARE_AWAIT(Traversal, _AmdDispatchSystemData, uint32_t shaderRecIdx, uint32_t dummyReturnAddr, _AmdSystemData data)

DECLARE_ACCEPT_HIT_ATTRIBUTES(, inout_param(_AmdAnyHitSystemData) data)

DECLARE_GET_UNINITIALIZED(F32, float)
DECLARE_GET_UNINITIALIZED(I32, uint32_t)
DECLARE_GET_UNINITIALIZED(I64, uint64_t)
DECLARE_GET_UNINITIALIZED(SystemData, _AmdSystemData)
DECLARE_GET_UNINITIALIZED(DispatchSystemData, _AmdDispatchSystemData)

DECLARE_CONT_STACK_LOAD_LAST_USE(U32, uint32_t)
DECLARE_CONT_STACK_STORE(U32, uint32_t value)
DECLARE_CONT_STACK_LOAD_LAST_USE(U64, uint64_t)
DECLARE_CONT_STACK_STORE(U64, uint64_t value)
#else // ((GPURT_DEBUG_CONTINUATION_TRAVERSAL == 0) && (!defined(__cplusplus)))
//=====================================================================================================================
inline _AmdDispatchSystemData _AmdGetUninitializedDispatchSystemData()
{
    return (_AmdDispatchSystemData)0;
}

//=====================================================================================================================
inline _AmdSystemData _AmdGetUninitializedSystemData()
{
    return (_AmdSystemData)0;
}
#endif

//=====================================================================================================================
static Vpc32 GetVpcFromShaderIdAddr(GpuVirtualAddress addr)
{
#ifdef __cplusplus
    return 1;
#else
    return Vpc32(ConstantLoadDwordAtAddr(addr));
#endif
}

//=====================================================================================================================
static Vpc32 GetVpcFromShaderIdTable(
    GpuVirtualAddress tableAddress,
    uint index,
    uint stride)
{
    return GetVpcFromShaderIdAddr(tableAddress + stride * index);
}

//=====================================================================================================================
// Returns the 32-bit part of the hit group shader id containing the AHS shader id.
static Vpc32 GetAnyHit32BitShaderId(
    uint hitGroupRecordIndex)
{
    const uint offset = DispatchRaysConstBuf.hitGroupTableStrideInBytes * hitGroupRecordIndex;

    const GpuVirtualAddress tableVa =
        PackUint64(DispatchRaysConstBuf.hitGroupTableBaseAddressLo, DispatchRaysConstBuf.hitGroupTableBaseAddressHi);
    if (tableVa == 0)
    {
       return Vpc32(0);
    }
    return Vpc32(ConstantLoadDwordAtAddr(tableVa + offset + 8));
}

//=====================================================================================================================
// Returns whether the corresponding AHS is non-null.
static bool AnyHitIsNonNull(
    in uint traceParameters,
    in uint geometryContributionToHitGroupIndex,
    in uint instanceContributionToHitGroupIndex)
{
    const uint hitGroupRecordIndex =
        CalculateHitGroupRecordAddress(ExtractRayContributionToHitIndex(traceParameters),
                                       ExtractMultiplierForGeometryContributionToHitIndex(traceParameters),
                                       geometryContributionToHitGroupIndex,
                                       instanceContributionToHitGroupIndex);

    return GetAnyHit32BitShaderId(hitGroupRecordIndex).IsValid();
}

//=====================================================================================================================
// Internal intrinsic function
static uint64_t GetInstanceNodeAddr(in uint64_t tlasBaseAddr, in uint nodePtr)
{
    const uint64_t instNodeAddr = tlasBaseAddr + ExtractNodePointerOffset(nodePtr);
    return instNodeAddr;
}

//=====================================================================================================================
static float3x4 ObjectToWorld3x4(in uint64_t tlasBaseAddr, in uint instNodePtr)
{
    float3x4 transform;

    {
        const uint offset = RTIP1_1_INSTANCE_SIDEBAND_OBJECT2WORLD_OFFSET + INSTANCE_NODE_EXTRA_OFFSET;

        transform[0] = asfloat(ConstantLoadDwordAtAddrx4(GetInstanceNodeAddr(tlasBaseAddr, instNodePtr) + offset + 0));
        transform[1] = asfloat(ConstantLoadDwordAtAddrx4(GetInstanceNodeAddr(tlasBaseAddr, instNodePtr) + offset + 16));
        transform[2] = asfloat(ConstantLoadDwordAtAddrx4(GetInstanceNodeAddr(tlasBaseAddr, instNodePtr) + offset + 32));
    }
    return transform;
}

//=====================================================================================================================
static float4x3 ObjectToWorld4x3(in uint64_t tlasBaseAddr, in uint instNodePtr)
{
    return transpose(ObjectToWorld3x4(tlasBaseAddr, instNodePtr));
}

//=====================================================================================================================
static float3x4 WorldToObject3x4(in uint64_t tlasBaseAddr, in uint instNodePtr)
{
    const uint offset = INSTANCE_DESC_WORLD_TO_OBJECT_XFORM_OFFSET;

    float3x4 transform;
    transform[0] = asfloat(ConstantLoadDwordAtAddrx4(GetInstanceNodeAddr(tlasBaseAddr, instNodePtr) + offset + 0));
    transform[1] = asfloat(ConstantLoadDwordAtAddrx4(GetInstanceNodeAddr(tlasBaseAddr, instNodePtr) + offset + 16));
    transform[2] = asfloat(ConstantLoadDwordAtAddrx4(GetInstanceNodeAddr(tlasBaseAddr, instNodePtr) + offset + 32));

    return transform;
}

//=====================================================================================================================
static float4x3 WorldToObject4x3(in uint64_t tlasBaseAddr, in uint instNodePtr)
{
    return transpose(WorldToObject3x4(tlasBaseAddr, instNodePtr));
}

//=====================================================================================================================
// Load dispatch dimensions from constant buffer.
static uint3 GetDispatchRaysDimensions()
{
    const uint width  = DispatchRaysConstBuf.rayDispatchWidth;
    const uint height = DispatchRaysConstBuf.rayDispatchHeight;
    const uint depth  = DispatchRaysConstBuf.rayDispatchDepth;

    return uint3(width, height, depth);
}

//=====================================================================================================================
// Persistent dispatch size (1D).
static uint GetPersistentDispatchSize()
{
    // Groups needed to cover the dispatch if each thread only processes 1 ray
    const uint3 rayDispatch   = GetDispatchRaysDimensions();
    const uint  threadsNeeded = rayDispatch.x * rayDispatch.y * rayDispatch.z;
    const uint3 groupDim      = AmdExtGroupDimCompute();
    const uint  groupsNeeded  = RoundUpQuotient(threadsNeeded, groupDim.x * groupDim.y * groupDim.z);

    // Dispatch size is the lesser of rayDispatchMaxGroups and groupsNeeded
    // rayDispatchMaxGroups would mean threads handle >= 1 ray, groupsNeeded would mean threads handle <= 1 ray
    return min(DispatchRaysConstBuf.rayDispatchMaxGroups, groupsNeeded);
}

//=====================================================================================================================
// Map a thread to a ray, some threads could end up with non-existent (invalid) rays.
// Note D3D12_DISPATCH_RAYS_DESC::(w x h x d) are organized to DispatchDims = (?, d, 1).
static uint3 GetDispatchId()
{
    const uint3 threadIdInGroup = AmdExtThreadIdInGroupCompute();
    const uint3 groupId         = AmdExtGroupIdCompute();
    const uint3 dims            = GetDispatchRaysDimensions();
    const uint  threadGroupSize = AmdExtGroupDimCompute().x * AmdExtGroupDimCompute().y * AmdExtGroupDimCompute().z;

    uint3 dispatchId;
    dispatchId.z = groupId.y;
    if ((dims.x > 1) && (dims.y > 1))
    {
        // Use 8 x (threadGroupSize / 8) tiles.
        /*
        Sample: D3D12_DISPATCH_RAYS_DESC::(w x h x d) = (18, 6, 1). Divided into 8x4 tiles(boxes).
        A number in a box is the group id.
        ___________________________
        |   0    |   1    |   2    |
        |________|________|________|
        |   3    |   4    |   5    |
        |________|________|________|
        */

        const uint wTile = (dims.x + 7) / 8;
        const uint xTile = groupId.x % wTile;
        const uint yTile = groupId.x / wTile;

        dispatchId.x = xTile * 8 + (threadIdInGroup.x % 8);
        dispatchId.y = yTile * (threadGroupSize / 8) + (threadIdInGroup.x / 8);
    }
    else
    {
        // Do a naive 1:1 simple map.
        const uint id = threadIdInGroup.x + threadGroupSize * groupId.x;
        const uint gridSize = dims.x * dims.y; // width x height
        dispatchId.y = id / dims.x;
        dispatchId.x = id - (dispatchId.y * dims.x);
    }

    return dispatchId;
}

//=====================================================================================================================
// Compute the X/Y/Z ray index based on the dispatch dimensions and a 32-bit dispatch ID
static uint3 GetDispatchId(uint width, uint height, uint dispatchId)
{
    // Progressively work from Z to Y to X, subtracting as we go along

    // Determine the Z index - divide by size of the 2D plane
    const uint planeSize = width * height;
    const uint z = dispatchId / planeSize;
    dispatchId -= z * planeSize;

    // Split the 2D plane into 8 x 64 tiles
    const uint TileWidth  = 8;
    const uint TileHeight = 64;

    // Determine which tile along the Y axis - divide by size of the 2D strip
    const uint yTile = dispatchId / TileHeight / width;
    dispatchId      -= yTile * TileHeight * width;

    // Determine which tile along the X axis - divide by size of the 2D strip
    // Take care in case the dispatch height is not a multiple of TileHeight
    const uint xStripHeight = min(TileHeight, height - (yTile * TileHeight));
    const uint xStripSize   = TileWidth * xStripHeight;
    const uint xTile        = dispatchId / xStripSize;
    dispatchId             -= xTile * xStripSize;

    // Determine Y position within the tile - divide by width of the 2D strip
    // Take care in case the dispatch width is not a multiple of TileWidth
    const uint xStripWidth = min(TileWidth, width - xTile * TileWidth);
    const uint y           = dispatchId / xStripWidth;
    dispatchId            -= y * xStripWidth;

    // Remainder is the X position within the tile
    const uint x = dispatchId;

    // Return ray index - X/Y based on their respective tiles and position within
    return uint3(xTile * TileWidth + x, yTile * TileHeight + y, z);
}

#ifdef __cplusplus
//=====================================================================================================================
// Helper function for cpp only
static float3 mul(in float3 v, in float4x3 m)
{
    float3 r;
    r.x = dot(m[0], v);
    r.y = dot(m[1], v);
    r.z = dot(m[2], v);
    return r;
}
#endif

#if ((GPURT_DEBUG_CONTINUATION_TRAVERSAL == 0) && (!defined(__cplusplus)))
//=====================================================================================================================
// Implementation of DispatchRaysIndex.
export uint3 _cont_DispatchRaysIndex3(in _AmdDispatchSystemData data)
{
    return data.DispatchId();
}

//=====================================================================================================================
// Implementation of DispatchRaysDimensions().
export uint3 _cont_DispatchRaysDimensions3(in _AmdDispatchSystemData data)
{
    return GetDispatchRaysDimensions();
}

//=====================================================================================================================
// Return the hit state for AnyHit and Intersection
export _AmdPrimitiveSystemState _cont_GetCandidateState(in _AmdAnyHitSystemData data)
{
    return data.candidate;
}

//=====================================================================================================================
// Return the hit state for ClosestHit
export _AmdPrimitiveSystemState _cont_GetCommittedState(in _AmdSystemData data)
{
    return data.traversal.committed;
}

//=====================================================================================================================
export float3 _cont_WorldRayOrigin3(in _AmdSystemData state)
{
    return state.ray.origin;
}

//=====================================================================================================================
export float3 _cont_WorldRayDirection3(in _AmdSystemData state)
{
    return state.ray.direction;
}

//=====================================================================================================================
export float _cont_RayTMin(in _AmdSystemData state)
{
    return state.ray.tMin;
}

//=====================================================================================================================
export uint _cont_RayFlags(in _AmdSystemData state)
{
    // Get the flags passed by TraceRay call and apply the known set/unset bits.
    return ApplyKnownFlags(state.ray.IncomingFlags());
}

//=====================================================================================================================
export uint _cont_InstanceInclusionMask(in _AmdSystemData data)
{
    return ExtractInstanceInclusionMask(data.ray.traceParameters);
}

//=====================================================================================================================
export float _cont_RayTCurrent(in _AmdSystemData data, in _AmdPrimitiveSystemState primitive)
{
    if (_AmdGetShaderKind() == DXILShaderKind::Intersection)
    {
        // The intersection shader is an exception. While the system data is usually about the candidate hit, the
        // current t must be from the committed hit.
        primitive = _cont_GetCommittedState(data);
    }

    float tCurrentHw = primitive.rayTCurrent;

    // AMD Gpu shifts the origin, so rayTCurrent is between 0 and (tMaxApp - tMinApp). Add tMinApp back for App's use.
    return tCurrentHw + data.ray.tMin;
}

//=====================================================================================================================
export uint _cont_InstanceIndex(in _AmdSystemData data, in _AmdPrimitiveSystemState primitive)
{

    return ConstantLoadDwordAtAddr(
        GetInstanceNodeAddr(data.ray.AccelStruct(), primitive.instNodePtr) +
        INSTANCE_NODE_EXTRA_OFFSET + RTIP1_1_INSTANCE_SIDEBAND_INSTANCE_INDEX_OFFSET);
}

//=====================================================================================================================
export uint _cont_InstanceID(in _AmdSystemData data, in _AmdPrimitiveSystemState primitive)
{

    return ConstantLoadDwordAtAddr(
        GetInstanceNodeAddr(data.ray.AccelStruct(), primitive.instNodePtr) + INSTANCE_DESC_ID_AND_MASK_OFFSET) & 0x00ffffff;
}

//=====================================================================================================================
export uint _cont_GeometryIndex(in _AmdSystemData data, in _AmdPrimitiveSystemState primitive)
{
    return primitive.GeometryIndex();
}

//=====================================================================================================================
export uint _cont_PrimitiveIndex(in _AmdSystemData data, in _AmdPrimitiveSystemState primitive)
{
    return primitive.primitiveIndex;
}

//=====================================================================================================================
export float4x3 _cont_ObjectToWorld4x3(in _AmdSystemData data, in _AmdPrimitiveSystemState primitive)
{
    return ObjectToWorld4x3(data.ray.AccelStruct(), primitive.instNodePtr);
}

//=====================================================================================================================
export float4x3 _cont_WorldToObject4x3(in _AmdSystemData data, in _AmdPrimitiveSystemState primitive)
{
    return WorldToObject4x3(data.ray.AccelStruct(), primitive.instNodePtr);
}

//=====================================================================================================================
export TriangleData _cont_TriangleVertexPositions(in _AmdSystemData data, in _AmdPrimitiveSystemState primitive)
{
    const GpuVirtualAddress instanceAddr = GetInstanceNodeAddr(data.ray.AccelStruct(), primitive.instNodePtr);
    {
        return FetchTriangleFromNode(GetInstanceAddr(FetchInstanceDescAddr(instanceAddr)), primitive.currNodePtr);
    }
}

//=====================================================================================================================
export float3 _cont_ObjectRayOrigin3(in _AmdSystemData data, in _AmdPrimitiveSystemState primitive)
{
    return mul(float4(data.ray.origin, 1.0), WorldToObject4x3(data.ray.AccelStruct(), primitive.instNodePtr));
}

//=====================================================================================================================
export float3 _cont_ObjectRayDirection3(in _AmdSystemData data, in _AmdPrimitiveSystemState primitive)
{
    return mul(float4(data.ray.direction, 0.0), WorldToObject4x3(data.ray.AccelStruct(), primitive.instNodePtr));
}

//=====================================================================================================================
// This intrinsic is called in response to HitKind() intrinsic called from an AnytHit or ClosestHit shader.
export uint _cont_HitKind(in _AmdSystemData data, in _AmdPrimitiveSystemState primitive)
{
    return primitive.HitKind();
}

//=====================================================================================================================
// Get the triangle hit attributes (barycentrics) when calling a ClosestHit shader for triangle geometry.
// Get the first 8 bytes of custom hit attributes when calling the ClosestHit shader for procedural geometry.
export BuiltInTriangleIntersectionAttributes _cont_GetTriangleHitAttributes(in _AmdSystemData data)
{
    BuiltInTriangleIntersectionAttributes attr;
    attr.barycentrics.x = data.traversal.committedBarycentrics.x;
    attr.barycentrics.y = data.traversal.committedBarycentrics.y;

    return attr;
}

//=====================================================================================================================
// Store triangle or procedural hit attributes (first 8 bytes) in system data when a hit is accepted.
export void _cont_SetTriangleHitAttributes(inout_param(_AmdSystemData) data, BuiltInTriangleIntersectionAttributes attr)
{
    data.traversal.committedBarycentrics = attr.barycentrics;
}

//=====================================================================================================================
// Common helper for _cont_AcceptHit and _cont_AcceptHitAndEndSearch
static void AcceptHit(inout_param(_AmdAnyHitSystemData) data, bool endSearch)
{
    {
        data.base.ray.SetAnyHitDidAccept(true);
        data.base.traversal.committed = data.candidate;
        if (endSearch)
        {
            data.base.traversal.nextNodePtr = END_SEARCH;     // End search
        }
    }
}

//=====================================================================================================================
// AcceptFirstHitAndSearch call from an AnyHit shader
export void _cont_AcceptHit(inout_param(_AmdAnyHitSystemData) data)
{
    AcceptHit(data, (data.base.ray.Flags() & RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH) != 0);
}

//=====================================================================================================================
// AcceptFirstHitAndSearch call from an AnyHit shader
export void _cont_AcceptHitAndEndSearch(inout_param(_AmdAnyHitSystemData) data)
{
    AcceptHit(data, true);
}

//=====================================================================================================================
// IgnoreHit call from an AnyHit shader
export void _cont_IgnoreHit(inout_param(_AmdAnyHitSystemData) data)
{
    {
        // Do nothing means we ignore the hit
        data.base.ray.SetAnyHitDidAccept(false);
    }
}

//=====================================================================================================================
// Returns if the Intersection shader should return immediately because AnyHit called AcceptHitAndEndSearch.
export bool _cont_IsEndSearch(in _AmdAnyHitSystemData data)
{
    // If AnyHit shader called AcceptHitAndEndSearch, or RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH was set, nextNodePtr
    // is END_SEARCH.
    // On the other side, the values Traversal function may set to traversal.nextNodePtr on its exit are different:
    // normal pointers, TERMINAL_NODE or INVALID_NODE.
    return (data.base.traversal.nextNodePtr == END_SEARCH);
}

//=====================================================================================================================
// Continuation stack sits in the scratch memory
export uint _cont_GetContinuationStackAddr()
{
    uint offset = 0;

    if (_AmdContinuationStackIsGlobal())
    {
        const uint3 threadIdInGroup = AmdExtThreadIdInGroupCompute();
        const uint3 groupId         = AmdExtGroupIdCompute();
        const uint  threadGroupSize = AmdExtGroupDimCompute().x * AmdExtGroupDimCompute().y * AmdExtGroupDimCompute().z;

        // Do a naive 1:1 simple map.
        const uint id = threadIdInGroup.x + threadGroupSize * groupId.x;

        offset = id * DispatchRaysConstBuf.cpsFrontendStackSize;
    }
    else
    {
        offset =
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 36
            DispatchRaysConstBuf.cpsStackOffsetInBytes;
#else
            DispatchRaysConstBuf.cpsBackendStackSize;
#endif
    }

    return offset;
}

//=====================================================================================================================
// Continuation stack sits in a global memory
export uint64_t _cont_GetContinuationStackGlobalMemBase()
{
    return PackUint64(DispatchRaysConstBuf.cpsGlobalMemoryAddressLo, DispatchRaysConstBuf.cpsGlobalMemoryAddressHi);
}

//=====================================================================================================================
static Vpc32 GetTraversalVpc32()
{
    // NOTE: DXCP uses a table for TraceRay, thus a load to traceRayGpuVa retrieves the actual traversal function
    // address. But Vulkan does not use the table so far, traceRayGpuVa is already the traversal function address.
    return Vpc32(DispatchRaysConstBuf.traceRayGpuVaLo);
}

//=====================================================================================================================
static Vpc32 GetRayGenVpc32()
{
    return GetVpcFromShaderIdAddr(PackUint64(DispatchRaysConstBuf.rayGenerationTableAddressLo,
                                             DispatchRaysConstBuf.rayGenerationTableAddressHi));
}

//=====================================================================================================================
// Returns shader binding table address for the current shader stage
export uint64_t _cont_GetSbtAddress()
{
    switch (_AmdGetShaderKind())
    {
    case DXILShaderKind::RayGeneration:
        return PackUint64(DispatchRaysConstBuf.rayGenerationTableAddressLo,
                          DispatchRaysConstBuf.rayGenerationTableAddressHi);
    case DXILShaderKind::ClosestHit:
    case DXILShaderKind::AnyHit:
    case DXILShaderKind::Intersection:
        return PackUint64(DispatchRaysConstBuf.hitGroupTableBaseAddressLo,
                          DispatchRaysConstBuf.hitGroupTableBaseAddressHi);
    case DXILShaderKind::Miss:
        return PackUint64(DispatchRaysConstBuf.missTableBaseAddressLo,
                          DispatchRaysConstBuf.missTableBaseAddressHi);
    case DXILShaderKind::Callable:
        return PackUint64(DispatchRaysConstBuf.callableTableBaseAddressLo,
                          DispatchRaysConstBuf.callableTableBaseAddressHi);
    default:
        // Should never be called
        GPU_ASSERT(false);
        return 0;
    }
}

//=====================================================================================================================
// Return shader binding table stride for the current shader stage
export uint _cont_GetSbtStride()
{
    switch (_AmdGetShaderKind())
    {
    case DXILShaderKind::RayGeneration:
        return 0;
    case DXILShaderKind::ClosestHit:
    case DXILShaderKind::AnyHit:
    case DXILShaderKind::Intersection:
        return DispatchRaysConstBuf.hitGroupTableStrideInBytes;
    case DXILShaderKind::Miss:
        return DispatchRaysConstBuf.missTableStrideInBytes;
    case DXILShaderKind::Callable:
        return DispatchRaysConstBuf.callableTableStrideInBytes;
    default:
        // Should never be called
        GPU_ASSERT(false);
        return 0;
    }
}

//=====================================================================================================================
// ReportHit implementation that is called from the intersection shader.
// May call the AnyHit shader.
export bool _cont_ReportHit(inout_param(_AmdAnyHitSystemData) data, float THit, uint HitKind)
{
    // TODO Reuse shader record index computed in Traversal
    // TODO Check for closest hit and duplicate anyHit calling

    THit -= data.base.ray.tMin;
    float tCurrentCommitted = 0.f;
    {
        tCurrentCommitted = data.base.traversal.committed.rayTCurrent;
    }

    if ((THit < 0.f) || (THit > tCurrentCommitted))
    {
        // Discard the hit candidate and hint the compiler to not keep the
        // values alive, which will remove redundant moves.
        data.candidate.rayTCurrent = _AmdGetUninitializedF32();
        // Don't discard the hit kind as it is bit packed and cannot be discarded partially.
        return false;
    }

    data.candidate.rayTCurrent = THit;
    data.candidate.PackHitKind(HitKind);

    uint isOpaque = true;
    {
        PrimitiveData primitiveData;
        InstanceDesc desc;

        {
            // Get primitive nodes to process based on candidate or committed hit
            const uint tlasNodePtr = data.candidate.instNodePtr;

            const GpuVirtualAddress tlasAddr = data.base.ray.AccelStruct() + ExtractNodePointerOffset(tlasNodePtr);
            desc = FetchInstanceDescAddr(tlasAddr);
            isOpaque = data.candidate.IsOpaque();
        }
    }

    if (!isOpaque)
    {
        uint hitGroupRecordIndex = 0;
        {
            hitGroupRecordIndex = _AmdGetShaderRecordIndex();
        }
        // Compute hit group address and fetch shader identifiers
        const Vpc32 anyHitAddr = GetAnyHit32BitShaderId(hitGroupRecordIndex);

        if (anyHitAddr.IsValid())
        {
            // Call AnyHit
            // Hit attributes are added as an additional argument by the compiler
            data = _AmdAwaitAnyHit(anyHitAddr.GetU32(), hitGroupRecordIndex, _AmdGetResumePointAddr(), data);
            return data.base.ray.AnyHitDidAccept();
        }
        else
        {
            _cont_AcceptHit(data);
            _AmdAcceptHitAttributes(data); // changes data.base.traversal.committedBarycentrics plus up-to-6 DW data in payload
            return true;
        }
    }
    else
    {
        _cont_AcceptHit(data);
        _AmdAcceptHitAttributes(data);
        return true;
    }
}

//=====================================================================================================================
// CallShader implementation
export void _cont_CallShader(inout_param(_AmdDispatchSystemData) data, uint index)
{
    const uint64_t callableTableBaseAddress =
        PackUint64(DispatchRaysConstBuf.callableTableBaseAddressLo, DispatchRaysConstBuf.callableTableBaseAddressHi);

    if (callableTableBaseAddress == 0)
    {
        // TODO: It might be better to AwaitSelf here, adding an artificial suspend point.
        //       For the common case of non-null callable shaders, this would reduce
        //       the size of compiled shaders, as the post-CallShader() part is unreachable,
        //       also simplifying manual testing with suspend points.
        //       For null callable shaders, it has the advantage of allowing
        //       to reconverge on the resume function if implemented in a way that yields only
        //       a single resume function.
        return;
    }

    const Vpc32 addr = GetVpcFromShaderIdTable(callableTableBaseAddress,
                                               index,
                                               DispatchRaysConstBuf.callableTableStrideInBytes);

    if (!addr.IsValid())
    {
        // See TODO above on how to handle this case better.
        return;
    }

    data = _AmdAwaitCallShader(addr.GetU32(), index, _AmdGetResumePointAddr(), data);
}

//=====================================================================================================================
// Returns the low part of the miss shader address and sets up the dispatch data to have the correct shader record
// index.
static Vpc32 SetupMissShader(inout_param(_AmdSystemData) data, out_param(uint) shaderRecIdx)
{
    const uint64_t missTableBaseAddress =
        PackUint64(DispatchRaysConstBuf.missTableBaseAddressLo, DispatchRaysConstBuf.missTableBaseAddressHi);
    if (missTableBaseAddress == 0)
    {
        shaderRecIdx = 0;
        return Vpc32(0);
    }

    shaderRecIdx = ExtractMissShaderIndex(data.ray.traceParameters);

    // Calculate miss shader record address
    return GetVpcFromShaderIdTable(missTableBaseAddress,
                                   shaderRecIdx,
                                   DispatchRaysConstBuf.missTableStrideInBytes);
}

//=====================================================================================================================
static HitGroupInfo GetHitGroupInfo(
    in _AmdSystemData           data,
    in uint                     state,
    in _AmdPrimitiveSystemState candidate)
{
    uint geometryIndex = (state < TRAVERSAL_STATE_COMMITTED_NOTHING) ?
            candidate.GeometryIndex() : data.traversal.committed.GeometryIndex();
    uint instanceContribution = (state < TRAVERSAL_STATE_COMMITTED_NOTHING) ?
            candidate.InstanceContribution() : data.traversal.committed.InstanceContribution();

    return GetHitGroupInfo(ExtractRayContributionToHitIndex(data.ray.traceParameters),
                           ExtractMultiplierForGeometryContributionToHitIndex(data.ray.traceParameters),
                           geometryIndex,
                           instanceContribution);
}
#endif

//=====================================================================================================================
// Ray History helper functions
//=====================================================================================================================
static void RayHistoryIncNumRayBoxTest(inout_param(_AmdSystemData) data)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        data.counter.numRayBoxTest++;
    }
#endif
}

//=====================================================================================================================
static void RayHistoryIncNumRayTriangleTest(inout_param(_AmdSystemData) data)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        data.counter.numRayTriangleTest++;
    }
#endif
}

//=====================================================================================================================
static void RayHistorySetMaxStackDepth(inout_param(_AmdSystemData) data, uint stackPtr)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        data.counter.maxStackDepth = max(data.counter.maxStackDepth, stackPtr & 0xffff);
    }
#endif
}

//=====================================================================================================================
static void RayHistoryIncInstanceIntersections(inout_param(_AmdSystemData) data)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        data.counter.instanceIntersections++;
    }
#endif
}

//=====================================================================================================================
static void RayHistorySetWriteTopLevel(inout_param(_AmdSystemData) data)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        data.counter.SetWriteTokenTopLevel(true);
    }
#endif
}

//=====================================================================================================================
static void RayHistoryWriteTopLevel(inout_param(_AmdSystemData) data)
{
#if DEVELOPER
    if (EnableTraversalCounter() && data.counter.WriteTokenTopLevel())
    {
        WriteRayHistoryTokenTopLevel(GetRayId(data.dispatch.DispatchId()), data.ray.AccelStruct());
        data.counter.SetWriteTokenTopLevel(false);
    }
#endif
}

//=====================================================================================================================
static void RayHistorySetCandidateTCurrent(inout_param(_AmdSystemData) data, float rayTCurrent)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        data.counter.candidateTCurrent = rayTCurrent;
    }
#endif
}

//=====================================================================================================================
static void RayHistoryInitStaticId(inout_param(_AmdSystemData) data)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION >= 50
        data.dispatch.staticId = AmdTraceRayInitStaticId();
#else
        AmdTraceRayInitStaticId();
        data.dispatch.staticId = AmdTraceRayGetStaticId();
#endif
    }
#endif
}

//=====================================================================================================================
static uint RayHistoryGetParentId(_AmdDispatchSystemData data)
{
    uint parentId = -1;

#if DEVELOPER
    if (EnableTraversalCounter())
    {
        parentId = data.parentId;
    }
#endif

    return parentId;
}

//=====================================================================================================================
static void RayHistorySetParentId(inout_param(_AmdDispatchSystemData) data, uint parentId)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        data.parentId = parentId;
    }
#endif
}

//=====================================================================================================================
static void RayHistoryWriteBegin(inout_param(_AmdSystemData) data)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        const uint rayId  = GetRayId(data.dispatch.DispatchId());
        RayDesc rayDesc   = (RayDesc)0;
        rayDesc.Origin    = data.ray.origin;
        rayDesc.Direction = data.ray.direction;
        rayDesc.TMin      = data.ray.tMin;
        rayDesc.TMax      = data.ray.tMax;

        data.counter.timerBegin = AmdTraceRaySampleGpuTimer();
        data.counter.dynamicId  = AllocateRayHistoryDynamicId();
        data.counter.SetCallerShaderType(_AmdGetShaderKind());

        WriteRayHistoryTokenBegin(rayId,
                                  data.dispatch.DispatchId(),
                                  data.ray.AccelStruct(),
                                  data.ray.Flags(),
                                  data.ray.traceParameters,
                                  rayDesc,
                                  data.dispatch.staticId,
                                  data.counter.dynamicId,
                                  data.dispatch.parentId);
        WriteRayHistoryTokenTimeStamp(rayId, data.counter.timerBegin);
    }
#endif
}

//=====================================================================================================================
static void RayHistoryWriteEnd(inout_param(_AmdSystemData) data, uint state)
{
#if DEVELOPER
    WriteDispatchCounters(data.counter.numIterations);

    const uint     rayId    = GetRayId(data.dispatch.DispatchId());
    const uint64_t timerEnd = AmdTraceRaySampleGpuTimer();
    WriteRayHistoryTokenTimeStamp(rayId, timerEnd);

    if (timerEnd > data.counter.timerBegin)
    {
        data.counter.timer = (uint)(timerEnd - data.counter.timerBegin);
    }
    else
    {
        data.counter.timer = 0;
    }

    if (data.IsChs(state))
    {
        // For CHS, get candidate and barycentrics from traversal.
        const uint instNodeIndex = FetchInstanceIdx(ConvertRtIpLevel(GetRtIpLevel()),
                                                    data.ray.AccelStruct(),
                                                    data.traversal.committed.instNodePtr);
        WriteRayHistoryTokenEnd(rayId,
                                uint2(data.traversal.committed.primitiveIndex,
                                      data.traversal.committed.GeometryIndex()),
                                instNodeIndex,
                                data.counter.numIterations,
                                data.counter.instanceIntersections,
                                data.traversal.committed.HitKind(),
                                data.traversal.committed.rayTCurrent);
    }
    else
    {
        WriteRayHistoryTokenEnd(rayId,
                                uint2(~0, ~0),
                                uint(~0),
                                data.counter.numIterations,
                                data.counter.instanceIntersections,
                                uint(~0),
                                (data.ray.tMax - data.ray.tMin));
    }
#endif
}

//=====================================================================================================================
static uint2 RayHistoryGetIdentifierFromVPC(Vpc32 vpc)
{
    // Zero out the metadata bits
    return uint2(vpc.GetFunctionAddr(), 0);
}

//=====================================================================================================================
static void RayHistoryWriteTriangleHitResult(_AmdSystemData data, bool accept)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        WriteRayHistoryTokenTriangleHitResult(GetRayId(data.dispatch.DispatchId()),
                                              uint(accept),
                                              data.counter.candidateTCurrent);
    }
#endif
}

//=====================================================================================================================
static void RayHistoryWriteFunctionCall(inout_param(_AmdSystemData) data,
                                        uint2                       shaderId,
                                        uint                        tableIndex,
                                        DXILShaderKind              shaderKind)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        const uint rayId = GetRayId(data.dispatch.DispatchId());

        switch(shaderKind)
        {
        case DXILShaderKind::Miss:
            WriteRayHistoryTokenFunctionCall(rayId,
                                             shaderId,
                                             tableIndex,
                                             RAY_HISTORY_FUNC_CALL_TYPE_MISS);
            data.counter.shaderIdLow  = shaderId.x;
            data.counter.shaderRecIdx = tableIndex | TCID_SHADER_RECORD_INDEX_MISS;
            RayHistorySetParentId(data.dispatch, data.counter.dynamicId);
            break;

        case DXILShaderKind::ClosestHit:
            WriteRayHistoryTokenFunctionCall(rayId,
                                             shaderId,
                                             tableIndex,
                                             RAY_HISTORY_FUNC_CALL_TYPE_CLOSEST);
            data.counter.shaderIdLow  = shaderId.x;
            data.counter.shaderRecIdx = tableIndex;
            RayHistorySetParentId(data.dispatch, data.counter.dynamicId);
            break;

        case DXILShaderKind::AnyHit:
            WriteRayHistoryTokenFunctionCall(rayId,
                                             shaderId,
                                             tableIndex,
                                             RAY_HISTORY_FUNC_CALL_TYPE_ANY_HIT);
            data.counter.numAnyHitInvocation++;
            data.counter.SetCallerShaderType(shaderKind);
            break;

        case DXILShaderKind::Intersection:
            WriteRayHistoryTokenFunctionCall(rayId,
                                             shaderId,
                                             tableIndex,
                                             RAY_HISTORY_FUNC_CALL_TYPE_INTERSECTION);
            data.counter.SetCallerShaderType(shaderKind);
            break;

        default:
            break;
        }
    }
#endif
}

//=====================================================================================================================
static void RayHistoryWriteAnyHitOrProceduralStatus(inout_param(_AmdSystemData) data)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        const uint rayId  = GetRayId(data.dispatch.DispatchId());
        const uint status = (data.traversal.nextNodePtr == END_SEARCH)
                            ? HIT_STATUS_ACCEPT_AND_END_SEARCH
                            : (data.ray.AnyHitDidAccept() ? HIT_STATUS_ACCEPT : HIT_STATUS_IGNORE);

        switch (data.counter.CallerShaderType())
        {
        case DXILShaderKind::AnyHit:
            WriteRayHistoryTokenAnyHitStatus(rayId, status);
            RayHistoryWriteTriangleHitResult(data, (status > HIT_STATUS_IGNORE));
            if (status != HIT_STATUS_ACCEPT_AND_END_SEARCH)
            {
                RayHistoryWriteTopLevel(data);
            }
            break;

        case DXILShaderKind::Intersection:
            WriteRayHistoryTokenProceduralIntersectionStatus(rayId,
                                                             status,
                                                             data.traversal.committed.rayTCurrent,
                                                             data.traversal.committed.HitKind());
            if (status != HIT_STATUS_ACCEPT_AND_END_SEARCH)
            {
                RayHistoryWriteTopLevel(data);
            }

            if (status != HIT_STATUS_IGNORE)
            {
                data.counter.numCandidateHits++;
            }
            break;

        default:
            break;
        }
        data.counter.SetCallerShaderType(DXILShaderKind::Invalid);
    }
#endif
}

//=====================================================================================================================
static void RayHistoryHandleIteration(inout_param(_AmdSystemData) data, uint nextNodePtr)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        WriteRayHistoryTokenNodePtr(GetRayId(data.dispatch.DispatchId()), nextNodePtr);
        UpdateWaveTraversalStatistics(ConvertRtIpLevel(GetRtIpLevel()), nextNodePtr);

        data.counter.numIterations++;
    }
#endif
}

//=====================================================================================================================
static void RayHistoryWriteBottomLevel(_AmdSystemData data, GpuVirtualAddress bvhAddress)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        WriteRayHistoryTokenBottomLevel(GetRayId(data.dispatch.DispatchId()), bvhAddress);
    }
#endif
}

//=====================================================================================================================
static void TraversalCounterWriteCounter(_AmdSystemData data)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        TraversalCounter counter = (TraversalCounter)0;
        counter.data[TCID_NUM_RAY_BOX_TEST]       = data.counter.numRayBoxTest;
        counter.data[TCID_NUM_RAY_TRIANGLE_TEST]  = data.counter.numRayTriangleTest;
        counter.data[TCID_NUM_ITERATION]          = data.counter.numIterations;
        counter.data[TCID_MAX_TRAVERSAL_DEPTH]    = data.counter.maxStackDepth;
        counter.data[TCID_NUM_ANYHIT_INVOCATION]  = data.counter.numAnyHitInvocation;
        counter.data[TCID_SHADER_ID]              = data.counter.shaderIdLow;
        counter.data[TCID_SHADER_RECORD_INDEX]    = data.counter.shaderRecIdx;
        counter.data[TCID_TIMING_DATA]            = data.counter.timer;
        counter.data[TCID_WAVE_ID]                = AmdTraceRayGetHwWaveId();
        counter.data[TCID_NUM_CANDIDATE_HITS]     = data.counter.numCandidateHits;
        counter.data[TCID_INSTANCE_INTERSECTIONS] = data.counter.instanceIntersections;

        WriteTraversalCounter(GetRayId(data.dispatch.DispatchId()), counter);
    }
#endif
}

//=====================================================================================================================
// Order matters, the following HLSL reference the functions and structs defined above. TODO: refactor these into a
// separate HLSL
#include "Continuations1_1.hlsl"
#include "Continuations2_0.hlsl"

//=====================================================================================================================
// Calls traversal for the current rtip.
static void TraversalInternal(
    inout_param(_AmdSystemData) data,
    inout_param(uint) state,
    inout_param(_AmdPrimitiveSystemState) candidate,
    inout_param(float2) candidateBarycentrics)
{
    switch (GetRtIpLevel())
    {
#if (GPURT_RTIP_LEVEL == GPURT_RTIP_LEGACY_LEVEL) || (GPURT_RTIP_LEVEL == 0)
    case RayTracingIpLevel::RtIp1_1:
        TraversalInternal1_1(data, state, candidate, candidateBarycentrics);
        break;
    case RayTracingIpLevel::RtIp2_0:
        TraversalInternal2_0(data, state, candidate, candidateBarycentrics);
        break;
#endif
    default:
        break;
    }
}

#if ((GPURT_DEBUG_CONTINUATION_TRAVERSAL == 0) && (!defined(__cplusplus)))
static uint64_t GetDispatchIdAddr()
{
    return PackUint64(DispatchRaysConstBuf.cpsDispatchIdAddressLo, DispatchRaysConstBuf.cpsDispatchIdAddressHi);
}

//=====================================================================================================================
static void LaunchRayGen(bool setupStack)
{
    uint3 dispatchId;
    bool  valid;

    if (Options::getPersistentLaunchEnabled() == false)
    {
        // Each thread will process <= 1 ray. No need for extra counter logic.
        dispatchId = GetDispatchId();
        valid      = (dispatchId.x < DispatchRaysConstBuf.rayDispatchWidth &&
                      dispatchId.y < DispatchRaysConstBuf.rayDispatchHeight);
    }
    else
    {
        // This is a persistent launch where each thread will process >= 1 ray.

        // This is written in a way that is intended to be correct even if threads don't reconverge after calling into
        // the ray generation shader.
        const uint popCount = WaveActiveCountBits(true);

        uint flatDispatchId = 0;
        if (WaveIsFirstLane())
        {
            flatDispatchId = AmdExtAtomicAddAtAddr(GetDispatchIdAddr(), 0, popCount);
        }
        flatDispatchId = WaveReadLaneFirst(flatDispatchId) + WavePrefixCountBits(true);

        const uint3 rayDims = GetDispatchRaysDimensions();

        dispatchId = GetDispatchId(rayDims.x, rayDims.y, flatDispatchId);
        valid      = flatDispatchId < (rayDims.x * rayDims.y * rayDims.z);
    }

    // With persistent launch every lane gets a stack
    if (setupStack)
    {
        _AmdContStackSetPtr(_cont_GetContinuationStackAddr());
    }

    if (WaveActiveAllTrue(!valid))
    {
        // This wave is done.
        _AmdComplete();
    }

    // But only lanes that have a valid dispatch id execute RGS, the others stay dead:
    if (valid)
    {
        _AmdDispatchSystemData systemData;
        systemData.PackDispatchId(dispatchId);
#if DEVELOPER
        systemData.parentId = -1;
#endif
        _AmdEnqueueRayGen(GetRayGenVpc32().GetU32(), 0, _AmdGetUninitializedI32(), systemData);
    }
    else if (Options::getPersistentLaunchEnabled())
    {
        _AmdDispatchSystemData systemData = _AmdGetUninitializedDispatchSystemData();
        systemData.SetDead(true);
        _AmdEnqueueTraversalDead(GetTraversalVpc32().GetU32(), _AmdGetUninitializedI32(), _AmdGetUninitializedI32(), systemData);
    }
}

//=====================================================================================================================
// KernelEntry is entry function of the RayTracing continuation mode
export void _cont_KernelEntry()
{
    LaunchRayGen(true);
}

//=====================================================================================================================
// Preamble function with overloading that is called at the beginning of each shader
export void _cont_ShaderStart(inout_param(_AmdDispatchSystemData) data)
{
    // TODO: Currently this is an interface that allows compiler to use, add more functionality for specific case later.
    return;
}

export void _cont_ShaderStart(inout_param(_AmdSystemData) data)
{
    // TODO: Currently this is an interface that allows compiler to use, add more functionality for specific case later.
    return;
}

export void _cont_ShaderStart(inout_param(_AmdAnyHitSystemData) data)
{
    // TODO: Currently this is an interface that allows compiler to use, add more functionality for specific case later.
    return;
}

//=====================================================================================================================
// TraceRays implementation using the continuations programming model
//
export void _cont_TraceRay(
    inout_param(_AmdDispatchSystemData) dispatch,
    uint64_t accelStruct,
    uint     rayFlags,
    uint     instanceInclusionMask,
    uint     rayContributionToHitGroupIndex,
    uint     multiplierForGeometryContributionToShaderIndex,
    uint     missShaderIndex,
    float    originX,
    float    originY,
    float    originZ,
    float    tMin,
    float    dirX,
    float    dirY,
    float    dirZ,
    float    tMax)
{
    accelStruct = FetchAccelStructBaseAddr(accelStruct); //AccelStructMetadataHeader* --> AccelStructHeader*

    RayDesc rayDesc = (RayDesc)0;
    rayDesc.Origin = float3(originX, originY, originZ);
    rayDesc.Direction = float3(dirX, dirY, dirZ);
    rayDesc.TMin = tMin;
    rayDesc.TMax = tMax;

    // Initialise ray system state from TraceRay parameters
    _AmdRaySystemState ray = (_AmdRaySystemState)0;
    ray.PackAccelStructAndRayflags(accelStruct, rayFlags);
    ray.direction       = float3(dirX, dirY, dirZ);
    ray.origin          = float3(originX, originY, originZ);
    ray.tMin            = tMin;
    ray.tMax            = tMax;
    ray.traceParameters =
        ((instanceInclusionMask                          & 0x00FF) << 0)  |
        ((rayContributionToHitGroupIndex                 & 0x000F) << 8)  |
        ((multiplierForGeometryContributionToShaderIndex & 0x000F) << 12) |
        ((missShaderIndex                                & 0xFFFF) << 16);

    // Really only needs to take in ray origin, direction and acceleration structure address
    const bool isValid = IsValidTrace(rayDesc, accelStruct, instanceInclusionMask, rayFlags, AmdTraceRayGetStaticFlags());

    if (isValid)
    {
        LogAccelStruct(accelStruct);
    }
    // Initialise traversal system state
    _AmdTraversalState traversal = (_AmdTraversalState)0;
    switch (GetRtIpLevel())
    {
    case RayTracingIpLevel::RtIp1_1:
        traversal = InitTraversalState1_1(instanceInclusionMask, rayDesc, isValid);
        traversal.nextNodePtr = isValid ? CreateRootNodePointer1_1() : INVALID_NODE;
        break;
    case RayTracingIpLevel::RtIp2_0:
        traversal = InitTraversalState2_0(instanceInclusionMask, rayDesc, isValid);
        traversal.nextNodePtr = isValid ? CreateRootNodePointer1_1() : TERMINAL_NODE;
        break;
    default:
        break;
    }

    _AmdSystemData data = (_AmdSystemData) 0;
    data.dispatch  = dispatch;
    data.ray       = ray;
    data.traversal = traversal;

    RayHistoryInitStaticId(data);
    RayHistoryWriteBegin(data);

    const uint     parentId              = RayHistoryGetParentId(dispatch);

    // NO control flow is allowed between _AmdGetResumePointAddr() and _AmdAwaitTraversal().
    data.traversal.SetReturnAddress(Vpc32(_AmdGetResumePointAddr()));
    dispatch = _AmdAwaitTraversal(GetTraversalVpc32().GetU32(), _AmdGetUninitializedI32(), _AmdGetUninitializedI32(), data);

    RayHistorySetParentId(dispatch, parentId);  // restore parentId

    TraversalCounterWriteCounter(data);
}

//=====================================================================================================================
// Get the address of the function that should be called next, either a closest hit or a miss shader.
// If no hit or miss shader should be called, this method returns DEAD_SHADER_ADDR.
static Vpc32 GetNextHitMissPc(
    inout_param(_AmdSystemData) data,
    uint state,
    _AmdPrimitiveSystemState candidate,
    out_param(uint32_t) nextShaderRecIdx
)
{
    // MS
    if (data.IsMiss(state))
    {
        uint shaderRecIdx;
        const Vpc32 missShaderAddr = SetupMissShader(data, shaderRecIdx);
        if (missShaderAddr.IsValid())
        {
            // Valid MS
            nextShaderRecIdx = shaderRecIdx;

            return missShaderAddr;
        }
    }

    // CHS
    else if (data.IsChs(state))
    {
        HitGroupInfo hitInfo = (HitGroupInfo)0;
        {
            hitInfo = GetHitGroupInfo(data, state, candidate);
        }

        nextShaderRecIdx = hitInfo.tableIndex;

        if ((data.ray.Flags() & RAY_FLAG_SKIP_CLOSEST_HIT_SHADER) == 0)
        {
            Vpc32 closestHitId = Vpc32(hitInfo.closestHitId.x);
            if (closestHitId.IsValid())
            {
                return closestHitId;
            }
        }
    }
    return Vpc32(DEAD_SHADER_ADDR);
}

//=====================================================================================================================
// Helper to handle enqueueing CHS, MS.
static void EnqueueHitMiss(_AmdSystemData data, Vpc32 nextShaderAddr, uint32_t nextShaderRecIdx)
{
    GPU_ASSERT(nextShaderAddr.GetU32() != DEAD_SHADER_ADDR && !data.dispatch.IsDead());
    const uint state = data.traversal.committed.State();
    RayHistoryWriteEnd(data, state);

    const Vpc32 returnAddr = data.traversal.ReturnAddress();

    if (nextShaderAddr.GetU32() == DEAD_SHADER_ADDR)
    {
        // We do not have an address to jump to, retrieve the return address and return to RGS
        _AmdEnqueueRayGen(returnAddr.GetU32(), _AmdGetUninitializedI32(), _AmdGetUninitializedI32(), data.dispatch);
    }

    // Enqueue the selected shader
    const DXILShaderKind shaderKind = (DXILShaderKind)(data.IsMiss(state)
        ? (int)DXILShaderKind::Miss // convert to int to fix linux build error
        : (int)DXILShaderKind::ClosestHit
    );

    RayHistoryWriteFunctionCall(data,
                                RayHistoryGetIdentifierFromVPC(nextShaderAddr),
                                nextShaderRecIdx,
                                shaderKind);

    _AmdEnqueue(nextShaderAddr.GetU32(), nextShaderRecIdx, returnAddr.GetU32(), data);
}

//=====================================================================================================================

export void _cont_ExitRayGen(in _AmdDispatchSystemData data)
{
    if (Options::getPersistentLaunchEnabled()
    ) {
        // Lanes that exit raygen own a stack. Return them to traversal for scheduling
        _AmdDispatchSystemData sysData = _AmdGetUninitializedDispatchSystemData();
        sysData.SetDead(true);
        _AmdEnqueueTraversalDead(GetTraversalVpc32().GetU32(), _AmdGetUninitializedI32(), _AmdGetUninitializedI32(), sysData);
    }
    // In all other cases, exit the wave
    _AmdComplete();
}

//=====================================================================================================================

//=====================================================================================================================

namespace ThreadTrace
{

enum struct Section
{
    Scheduler = 8,
    Traversal = 6
};

//=================================================================================================================
// Wrapper to ensure the subsequent shader section is correctly identified in TTV.
// If thread traces are disabled, this does nothing. Otherwise, it issues a return token and a new shader data token
// of the type specified by `section`.
static void EnterSection(uint section)
{
    if (Options::getThreadTraceEnabled())
    {
        // Rest of the function is scheduler.
        // Emit a function return token to end the current function.
        AmdExtD3DShaderIntrinsics_ShaderMarker(0x10);

        // Emit a function call token to start the scheduler function.
        AmdExtD3DShaderIntrinsics_ShaderMarker(0x11 |
            (/* section */ section << 8) |
            (/* exec    */ WaveActiveCountBits(true) << 13));
    }
}

} // namespace ThreadTrace

//=====================================================================================================================
// Scheduler for dead lanes.
// Some lanes may return from this function. All lanes that return are guaranteed to be dead and are supposed to enqueue
// traversal for subsequent processing. If the full wave is dead and persistent launch is on, new work will be started.
// If persistent work is off, and all lanes are dead (potentially less than a full wave), and no work could be obtained,
// then the lanes are terminated.
static void ScheduleDeadWave(_AmdSystemData data, Vpc32 traversalAddr)
{
    GPU_ASSERT(WaveActiveAllTrue(data.dispatch.IsDead()));

    if (Options::getPersistentLaunchEnabled())
    {
        if (data.dispatch.IsDeadStackful())
        {
            if (WaveActiveCountBits(true) == AmdExtLaneCount())
            {
                // If the whole wave is dead, get ready to start a new dispatch
                LaunchRayGen(false);
            }
            // Passthrough these stackful dead lanes
        }
        _AmdEnqueueTraversalDead(traversalAddr.GetU32(), _AmdGetUninitializedI32(), _AmdGetUninitializedI32(), data.dispatch);
    }

    _AmdComplete();
}

//=====================================================================================================================
// Re-entrant traversal loop.
export void _cont_Traversal(
    inout_param(_AmdSystemData) data)
{
    bool IsDead = data.dispatch.IsDead();
    const bool IsTraversal = !IsDead && data.IsTraversal();

    // In assertion-enabled builds explicitly check that all ways to obtain a traversal address yield the same value
    GPU_ASSERT(GetTraversalVpc32().GetU32() == _AmdGetCurrentFuncAddr());

    // TRAVERSAL: BVH -------------------------------------------------------------------------------------------------
    uint state                          = TRAVERSAL_STATE_COMMITTED_NOTHING;
    _AmdPrimitiveSystemState candidate  = (_AmdPrimitiveSystemState)0;
    float2 candidateBarycentrics        = float2(0.0f, 0.0f);
    // Discard data that doesn't need to be kept alive during Traversal
    if (!IsBvhRebraid())
    {
        // lastInstanceRootNodePtr is only used for rebraid
        data.traversal.lastInstanceRootNodePtr = _AmdGetUninitializedI32();
    }

    if (!IsDead)
    {
        // Write AHS/IS returned status
        RayHistoryWriteAnyHitOrProceduralStatus(data);
    }

    // Execute traversal for active lanes.
    if (IsTraversal)
    {
        TraversalInternal(data, state, candidate, candidateBarycentrics);
    }
    else if (!IsDead)
    {
        // This branch is hit when the traversal for a lane is done:
        // a) AHS/IS enqueued _cont_Traversal(), for the very last time.
        // b) _cont_Traversal() enqueued _cont_Traversal() for a lane which is waiting to execute CHS/Miss, but
        //    scheduled again.

        // Retrieve lane state. Note, the state in here is only used to determine
        // whether this lane needs to run CHS or not. It does not matter if the state
        // is a committed triangle or procedural primitive.
        state = data.traversal.CommittedState();

        // For CHS, get candidate and barycentrics from traversal.
        if (data.IsChs(state))
        {
            candidate             = data.traversal.committed;
            candidateBarycentrics = data.traversal.committedBarycentrics;
        }
    }

    // ALIASES AND CACHED VARIABLES -----------------------------------------------------------------------------------

    // Some aliases for variable state. Help the compiler figure out these are mutually exclusive in all modes.
    bool IsChsOrMiss = false;
    bool IsAhsOrIs = false;
    if (!IsDead)
    {
        IsChsOrMiss = data.IsChsOrMiss(state);
        IsAhsOrIs = (data.IsAhs(state) || data.IsIs(state));
    }
    bool AllDead = Traits::HasDeadLanes() && WaveActiveAllTrue(IsDead);
    bool AnyIsAhsOrIs = WaveActiveAnyTrue(IsAhsOrIs);

    // TRAVERSAL: AHS AND IS ------------------------------------------------------------------------------------------

    if (AnyIsAhsOrIs)
    {
        if (IsAhsOrIs)
        {
            HitGroupInfo hitInfo = (HitGroupInfo)0;
            {
                hitInfo = GetHitGroupInfo(data, state, candidate);
            }

            _AmdAnyHitSystemData anyHitData = (_AmdAnyHitSystemData)0;
            anyHitData.base = data;
            anyHitData.candidate = candidate;

            // AHS and IS re-enqueue SchedulerInternal when finished.
            if (data.IsAhs(state))
            {
                const Vpc32 anyHitAddr = Vpc32(hitInfo.anyHitId.x);
                RayHistoryWriteFunctionCall(anyHitData.base,
                                            RayHistoryGetIdentifierFromVPC(anyHitAddr),
                                            hitInfo.tableIndex,
                                            DXILShaderKind::AnyHit);

                _AmdEnqueueAnyHit(anyHitAddr.GetU32(), hitInfo.tableIndex, _AmdGetCurrentFuncAddr(), anyHitData, candidateBarycentrics);
            }
            else
            {
                // Intersection shader
                GPU_ASSERT(data.IsIs(state));

                const Vpc32 isAddr = Vpc32(hitInfo.intersectionId.x);
                RayHistoryWriteFunctionCall(anyHitData.base,
                                            RayHistoryGetIdentifierFromVPC(isAddr),
                                            hitInfo.tableIndex,
                                            DXILShaderKind::Intersection);

                _AmdEnqueueIntersection(isAddr.GetU32(), hitInfo.tableIndex, _AmdGetCurrentFuncAddr(), anyHitData);
            }
        }
        _AmdEnqueueTraversal(_AmdGetCurrentFuncAddr(), _AmdGetUninitializedI32(), _AmdGetUninitializedI32(), data);
    }

    // FULL WAVE OF DEAD LANES ----------------------------------------------------------------------------------------
    else if (AllDead)
    {
        ScheduleDeadWave(data, Vpc32(_AmdGetCurrentFuncAddr()));
        // this is unreachable, ScheduleDeadWave guarantees to end with an enqueue
    }

    // CHS, MISS AND POSSIBLY DEAD LANES ------------------------------------------------------------------------------
    else
    {
        GPU_ASSERT(IsChsOrMiss || IsDead);
        ThreadTrace::EnterSection(ThreadTrace::Section::Scheduler);

        uint32_t nextShaderRecIdx = _AmdGetUninitializedI32();

        Vpc32 nextShaderAddr = Vpc32(DEAD_SHADER_ADDR);
        if (!IsDead)
        {
            nextShaderAddr = GetNextHitMissPc(data, state, candidate, nextShaderRecIdx);
        }

        if (!IsDead)
        {
            EnqueueHitMiss(data, nextShaderAddr, nextShaderRecIdx);
        }
        _AmdEnqueueTraversalDead(_AmdGetCurrentFuncAddr(), _AmdGetUninitializedI32(), _AmdGetUninitializedI32(), data.dispatch);
    }
}

#elif GPURT_DEBUG_CONTINUATION_TRAVERSAL // ((GPURT_DEBUG_CONTINUATION_TRAVERSAL == 0) && (!defined(__cplusplus)))

//=====================================================================================================================
// For debug. Convenience helper calling Traversal on the debug/emulation path that returns _AmdTraversalResultData.
static _AmdTraversalResultData TraversalInternalDebugWrapper(
    inout_param(_AmdSystemData) data)
{
    uint state                          = TRAVERSAL_STATE_COMMITTED_NOTHING;
    _AmdPrimitiveSystemState candidate  = (_AmdPrimitiveSystemState)0;
    float2 candidateBarycentrics        = float2(0.0f, 0.0f);

    TraversalInternal(data, state, candidate, candidateBarycentrics);

    _AmdTraversalResultData result = (_AmdTraversalResultData)0;
    result.state = state;
    result.candidate = candidate;
    result.candidateBarycentrics = candidateBarycentrics;

    return result;
}

//=====================================================================================================================
// For debug. Support DxcpRt (non-continuation) to use Continuation traversal.
static IntersectionResult TraceRayInternalCPSDebug(
    in GpuVirtualAddress topLevelBvh,             // Top-level acceleration structure to use
    in uint              rayFlags,                // Ray flags
    in uint              traceRayParameters,      // Packed trace ray parameters
    in RayDesc           rayDesc,                 // Ray to be traced
    in uint              rayId,                   // Ray ID for profiling
    in uint              rtIpLevel                // HW version to determine TraceRay implementation
#if DEVELOPER
    , in uint            dynamicId                // dynamic ID
#endif
)
{
    // Initialise ray system state from TraceRay parameters
    _AmdRaySystemState ray = (_AmdRaySystemState)0;
    ray.PackAccelStructAndRayflags(topLevelBvh, rayFlags);
    ray.direction          = rayDesc.Direction;
    ray.origin             = rayDesc.Origin;
    ray.tMin               = rayDesc.TMin;
    ray.tMax               = rayDesc.TMax;
    ray.traceParameters    = traceRayParameters;

    const bool isValid = true; // already verified in the caller

    _AmdDispatchSystemData dispatch = (_AmdDispatchSystemData)0;
    dispatch.PackDispatchId(AmdTraceRayDispatchRaysIndex());
#if DEVELOPER
    dispatch.parentId = -1;
#endif

    // Initialise traversal system state from driver intrinsic
    _AmdTraversalState traversal = (_AmdTraversalState)0;
    switch (rtIpLevel)
    {
    case GPURT_RTIP1_1:
        traversal = InitTraversalState1_1(0,
                                          rayDesc,
                                          isValid);
        traversal.nextNodePtr = isValid ? CreateRootNodePointer1_1() : INVALID_NODE;
        break;
    case GPURT_RTIP2_0:
        traversal = InitTraversalState2_0(0,
                                          rayDesc,
                                          isValid);
        traversal.nextNodePtr = isValid ? CreateRootNodePointer1_1() : TERMINAL_NODE;
        break;
    default:
        break;
    }

    _AmdSystemData sysData = (_AmdSystemData)0;
    sysData.dispatch  = dispatch;
    sysData.ray       = ray;
    sysData.traversal = traversal;

#if DEVELOPER
    sysData.counter.dynamicId = dynamicId;
#endif

    // Begin outer while loop
    while (sysData.traversal.nextNodePtr < TERMINAL_NODE)
    {
        _AmdTraversalResultData ret = TraversalInternalDebugWrapper(sysData);
        uint state = ret.state;

        // Process candidate hits
        if (state < TRAVERSAL_STATE_COMMITTED_NOTHING)
        {
            // Get primitive nodes to process based on candidate or committed hit
            const uint tlasNodePtr = ret.candidate.instNodePtr;

            PrimitiveData primitiveData;
            InstanceDesc desc;
            uint primitiveIndex       = ret.candidate.primitiveIndex;
            uint geometryIndex        = INVALID_IDX;
            uint instanceContribution = 0;
            uint anyHitCallType       = 0;

            {
                // Fetch primitive node addresses
                const GpuVirtualAddress tlasAddr = topLevelBvh + ExtractNodePointerOffset(tlasNodePtr);
                InstanceDesc desc = FetchInstanceDescAddr(tlasAddr);

                geometryIndex = ret.candidate.GeometryIndex();

                // Compute hit group address and fetch shader identifiers
                instanceContribution = desc.InstanceContributionToHitGroupIndex_and_Flags & 0x00ffffff;
                anyHitCallType       = ret.candidate.AnyHitCallType();
            }

            const HitGroupInfo hitInfo = GetHitGroupInfo(ExtractRayContributionToHitIndex(traceRayParameters),
                                         ExtractMultiplierForGeometryContributionToHitIndex(traceRayParameters),
                                         geometryIndex,
                                         instanceContribution);

            const uint64_t instNodePtr64 = CalculateInstanceNodePtr64(rtIpLevel, topLevelBvh, tlasNodePtr);
            if (state == TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_TRIANGLE)
            {
                uint status = HIT_STATUS_ACCEPT;

                // This test reduces sp3 instructions, when rayFlags is a const containing RAY_FLAG_FORCE_OPAQUE. Note
                // in this case, this branch is not executed w/wo this test, but simpler sp3 boosts performance.
                if ((ray.Flags() & RAY_FLAG_FORCE_OPAQUE) == 0)
                {
                    if (PackUint64(hitInfo.anyHitId) != 0)
                    {
                        RayHistoryWriteFunctionCall(sysData,
                                                    hitInfo.anyHitId,
                                                    hitInfo.tableIndex,
                                                    DXILShaderKind::AnyHit);
                    }

                    uint hitKind = ret.candidate.HitKind();
                    // Set intersection attributes
                    AmdTraceRaySetHitAttributes(ret.candidate.rayTCurrent,
                                                hitKind,
                                                HIT_STATUS_ACCEPT,
                                                LowPart(instNodePtr64),
                                                HighPart(instNodePtr64),
                                                primitiveIndex,
                                                ANYHIT_CALLTYPE_NO_DUPLICATE,
                                                geometryIndex);

                    BuiltInTriangleIntersectionAttributes attr = { ret.candidateBarycentrics };
                    AmdTraceRayCallTriangleAnyHitShader(hitInfo.anyHitId, hitInfo.tableIndex, attr);

                    AmdTraceRayGetHitAttributes(ret.candidate.rayTCurrent, hitKind, status);

                    if (PackUint64(hitInfo.anyHitId) != 0)
                    {
                        RayHistoryWriteAnyHitOrProceduralStatus(sysData);
                    }

                    if (status != HIT_STATUS_IGNORE)
                    {
                        sysData.traversal.committed = ret.candidate;
                        sysData.traversal.committedBarycentrics = ret.candidateBarycentrics;

                        if (status == HIT_STATUS_ACCEPT_AND_END_SEARCH)
                        {
                            sysData.traversal.nextNodePtr = INVALID_NODE;
                        }
                    }
                }

                RayHistoryWriteTriangleHitResult(sysData, status > HIT_STATUS_IGNORE);
            }
            else
            {
                // Intersection requires the currently committed hit as RayTCurrent()
                ret.candidate.rayTCurrent = sysData.traversal.committed.rayTCurrent;

                RayHistoryWriteFunctionCall(sysData,
                                            hitInfo.intersectionId,
                                            hitInfo.tableIndex,
                                            DXILShaderKind::Intersection);

                // Set intersection attributes
                AmdTraceRaySetHitAttributes(sysData.traversal.committed.rayTCurrent,
                                            0,
                                            HIT_STATUS_IGNORE,
                                            LowPart(instNodePtr64),
                                            HighPart(instNodePtr64),
                                            primitiveIndex,
                                            anyHitCallType,
                                            geometryIndex);

                AmdTraceRayCallIntersectionShader(hitInfo.intersectionId, hitInfo.anyHitId, hitInfo.tableIndex);

                // Retrieve post-intersection hit attributes
                uint status = HIT_STATUS_IGNORE;
                uint hitKind = ret.candidate.HitKind();

                AmdTraceRayGetHitAttributes(ret.candidate.rayTCurrent,
                                            hitKind,
                                            status);
                RayHistoryWriteAnyHitOrProceduralStatus(sysData);

                if (status != HIT_STATUS_IGNORE)
                {
                    sysData.traversal.committed = ret.candidate;
                    if (status == HIT_STATUS_ACCEPT_AND_END_SEARCH)
                    {
                        sysData.traversal.nextNodePtr = INVALID_NODE;
                    }
                }
            }
        }
    }

    IntersectionResult result   = (IntersectionResult)0;
    result.t                    = sysData.traversal.committed.rayTCurrent;
    result.nodeIndex            = sysData.traversal.committed.currNodePtr;
    result.barycentrics         = sysData.traversal.committedBarycentrics;
    result.instNodePtr          = sysData.traversal.committed.instNodePtr;
    result.hitkind              = sysData.traversal.committed.HitKind();
    result.instanceContribution = 0;
    result.geometryIndex        = INVALID_IDX;
    result.primitiveIndex       = INVALID_IDX;

    if (result.nodeIndex != INVALID_NODE)
    {
        // Fetch primitive node addresses
        uint primitiveIndex = INVALID_IDX;
        uint geometryIndex = INVALID_IDX;
        uint instanceContribution = 0;

        {
            const GpuVirtualAddress tlasAddr = topLevelBvh + ExtractNodePointerOffset(sysData.traversal.committed.instNodePtr);
            InstanceDesc desc = FetchInstanceDescAddr(tlasAddr);

            const GpuVirtualAddress  blasAddr = GetInstanceAddr(desc) + ExtractNodePointerOffset(sysData.traversal.committed.currNodePtr);
            const PrimitiveData primitiveData = FetchPrimitiveDataAddr(sysData.traversal.committed.currNodePtr, blasAddr);

            instanceContribution = desc.InstanceContributionToHitGroupIndex_and_Flags & 0x00ffffff;
            geometryIndex        = primitiveData.geometryIndex;
            primitiveIndex       = primitiveData.primitiveIndex;
        }

        result.instanceContribution = instanceContribution;
        result.geometryIndex = geometryIndex;
        result.primitiveIndex = primitiveIndex;

        AmdTraceRaySetHitTokenData(INVALID_NODE, INVALID_NODE);

        AmdTraceRaySetHitTokenData(sysData.traversal.committed.currNodePtr, sysData.traversal.committed.instNodePtr);

        bool handleTriangleNode = false;
        {
            handleTriangleNode = CheckHandleTriangleNode(sysData.traversal.committed.currNodePtr);
        }

        if (handleTriangleNode)
        {
            AmdTraceRaySetTriangleIntersectionAttributes(result.barycentrics);
        }
    }
    else
    {
        // Set hit token blas and tlas values to invalid for miss
        AmdTraceRaySetHitTokenData(INVALID_NODE, INVALID_NODE);
    }

#if DEVELOPER
    result.numRayBoxTest         = sysData.counter.numRayBoxTest;
    result.numRayTriangleTest    = sysData.counter.numRayTriangleTest;
    result.numIterations         = sysData.counter.numIterations;
    result.maxStackDepth         = sysData.counter.maxStackDepth;
    result.numAnyHitInvocation   = sysData.counter.numAnyHitInvocation;
    result.numCandidateHits      = sysData.counter.numCandidateHits;
    result.instanceIntersections = sysData.counter.instanceIntersections;
#endif

    return result;
}
#endif
