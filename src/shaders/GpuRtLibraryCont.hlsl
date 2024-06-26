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
#ifndef __cplusplus
#endif
#if DEVELOPER
#include "../../gpurt/gpurtCounter.h"
#endif

// By default, Gpurt exports both non-continuation and continuation traversal functions. Dxcp picks one based on panel
// setting.
// GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP = GPURT_RTIP1_1/GPURT_RTIP2_0
// is only used for a debug purpose.
// It supports DxcpRt (non-continuation) to use Continuation traversal. In this config, the pure continuation model does
// not work.
#ifndef GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP
#define GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP 0
#endif

#if ((GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP == 0) && (!defined(__cplusplus)))
#define CONTINUATION_ON_GPU 1
#else
#define CONTINUATION_ON_GPU 0
#endif

#define REMAT_INSTANCE_RAY 1

// If this experimental define is set, we support suspending the Traversal loop early to allow execution of IS/AHS
// for other lanes to allow these lanes to re-join Traversal. This could be beneficial in cases where these
// other lanes would otherwise idle for a long time until all other lanes have finished Traversal.
// After having executed IS/AHS, these lanes potentially need to run more Traversal iterations, which can be possibly
// avoided by scheduling IS/AHS early. Indirect mode also runs AHS/IS immediately.
// If AHS is expensive however, then waiting improves AHS execution coherency.
//
// This mode potentially has partial support only, e.g. only certain RTIPs.
#ifndef GPURT_CONT_TRAVERSAL_EARLY_IS_AHS
#define GPURT_CONT_TRAVERSAL_EARLY_IS_AHS 0
#endif

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

#if GPURT_CONT_TRAVERSAL_EARLY_IS_AHS
// This state implies Traversal was stopped to run AHS/IS for other lanes. This lane wants to resume Traversal.
#define TRAVERSAL_STATE_SUSPEND_TRAVERSAL 7
#endif

// Shader priorities for continuation scheduling. Higher values mean higher scheduling precedence.
// Reserve priority 0 as invalid value. This way, 0-initialized priorities in metadata-annotated
// function pointers (e.g. from relocations) can be detected.
// Note: For 32-bit packing of function pointers, we require the scheduling priority to fit into 3 bits.
#define SCHEDULING_PRIORITY_INVALID   0
#define SCHEDULING_PRIORITY_RGS       1
#define SCHEDULING_PRIORITY_CHS       2
#define SCHEDULING_PRIORITY_MISS      2
#define SCHEDULING_PRIORITY_TRAVERSAL 3
// Give IS higher prio than AHS so AHS called by ReportHit
// have a chance to run together with AHS called by Traversal.
#define SCHEDULING_PRIORITY_AHS       4
#define SCHEDULING_PRIORITY_IS        5
// TODO Decide on the callable shader priority
#define SCHEDULING_PRIORITY_CALLABLE  6
// Maximum supported value (3 bits): 7

#if CONTINUATION_ON_GPU == 0
#ifdef __cplusplus
extern uint g_rtIpLevel;          // defined in cputraversal
void _AmdSetRtip(uint rtIpLevel); // defined in cputraversal
#endif
static RayTracingIpLevel _AmdGetRtip()
{
    RayTracingIpLevel rtIpLevel = RayTracingIpLevel::_None;
#ifdef __cplusplus
    switch (g_rtIpLevel)
#else
    switch (GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP)
#endif
    {
    case GPURT_RTIP1_1:
        rtIpLevel = RayTracingIpLevel::RtIp1_1;
        break;
    case GPURT_RTIP2_0:
        rtIpLevel = RayTracingIpLevel::RtIp2_0;
        break;
    }

    return rtIpLevel;
}
#endif

static bool RtIpIsAtLeast(RayTracingIpLevel level)
{
    return ((uint32_t)_AmdGetRtip()) >= ((uint32_t)level);
}

//=====================================================================================================================
static uint ConvertRtIpLevel(RayTracingIpLevel rtIpLevel)
{
    uint level = 0;

    switch (rtIpLevel)
    {
    case RayTracingIpLevel::RtIp1_1:
        level = GPURT_RTIP1_1;
        break;
    case RayTracingIpLevel::RtIp2_0:
        level = GPURT_RTIP2_0;
        break;
    default:
        break;
    }

    return level;
}

//=====================================================================================================================
static uint GetPriorityForShaderType(
    DXILShaderKind shaderKind)
{
    switch (shaderKind)
    {
    case DXILShaderKind::Callable:       return SCHEDULING_PRIORITY_CALLABLE;
    case DXILShaderKind::Miss:           return SCHEDULING_PRIORITY_MISS;
    case DXILShaderKind::ClosestHit:     return SCHEDULING_PRIORITY_CHS;
    case DXILShaderKind::AnyHit:         return SCHEDULING_PRIORITY_AHS;
    case DXILShaderKind::Intersection:   return SCHEDULING_PRIORITY_IS;
    case DXILShaderKind::RayGeneration:  return SCHEDULING_PRIORITY_RGS;
    default:                             return SCHEDULING_PRIORITY_INVALID;
    }
}

// Forward declaration for _AmdDispatchSystemData.PackDispatchId() and _AmdDispatchSystemData.DispatchId()
static uint3 GetDispatchRaysDimensions();

#ifdef AMD_VULKAN

static uint64_t GetVpcWithPriority(uint64_t vpc, uint priority)
{
    return vpc;
}

static uint64_t Unpack32BitVpcTo64BitVpc(uint32_t vpc32, bool /*unpackPriority*/)
{
    return vpc32;
}

static uint32_t Pack64BitVpcTo32Bits(uint64_t vpc)
{
    return (vpc & 0xFFFFFFFF);
}

#else // AMD_VULKAN

//=====================================================================================================================
static uint64_t GetVpcWithPriority(uint64_t vpc, uint priority)
{
    const uint64_t prio64 = priority;
    const uint firstMetadataBit = 32;
    const uint firstPriorityBitInMetadata = 16;
    GPU_ASSERT((vpc & 0xFFFF000000000000) == 0);
    return vpc | (prio64 << (firstMetadataBit + firstPriorityBitInMetadata));
}

//=====================================================================================================================
// 32-bit function pointer packing/unpacking
//
static uint64_t Unpack32BitVpcTo64BitVpc(uint32_t vpc32, bool unpackPriority)
{
    uint64_t vpc = (vpc32 & 0xFFFFFFC0);

    if (unpackPriority)
    {
       // The priority is stored in bits 0..2.
       uint32_t priority = (vpc32 & 0x7);
       vpc = GetVpcWithPriority(vpc, priority);
    }

    return vpc;
}

static uint32_t Pack64BitVpcTo32Bits(uint64_t vpc)
{
    // Incoming metadata is in the high dword
    uint32_t inMetadata = (uint32_t)(vpc >> 32);
    uint32_t prio = (inMetadata >> 16);
    // We only have three bits for the priority:
    GPU_ASSERT(prio <= 7);

    // Outgoing metadata is in the low 6 bits
    uint32_t outMetadata = prio;

    GPU_ASSERT((vpc & 0x2F) == 0);
    return SplitUint64(vpc).x | outMetadata;
}

#endif

//=====================================================================================================================
// Dispatch system data. This data is shared between all shader stages and is initialized at the raygeneration shader
// stage e.g. remapping rays to threads
struct _AmdDispatchSystemData
{
#if defined(__cplusplus)
    _AmdDispatchSystemData(int val) : dispatchLinearId(val), shaderRecIdx(val)
    {
    }

    _AmdDispatchSystemData() : _AmdDispatchSystemData(0) {
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

    uint  dispatchLinearId;   // Packed dispatch linear id. Combine x/y/z into 1 DWORD.

    uint  shaderRecIdx; // Record index for local root parameters.
#if DEVELOPER
    uint  parentId;     // Record the parent Id for ray history counter, -1 for RayGen shader.
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
        uint incomingFlags = uint(bitFieldExtract64(packedAccelStruct, 48, 12));
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION  >= 41
        // Apply known bits common to all TraceRay calls
        incomingFlags = ((incomingFlags & ~AmdTraceRayGetKnownUnsetRayFlags()) | AmdTraceRayGetKnownSetRayFlags());
#endif
        return incomingFlags;
    }

    uint Flags()
    {
        uint rayFlags = IncomingFlags();
        // Apply compile time pipeline config flags into the ray flags
        rayFlags |= (AmdTraceRayGetStaticFlags() & (PIPELINE_FLAG_SKIP_PROCEDURAL_PRIMITIVES | PIPELINE_FLAG_SKIP_TRIANGLES));
#if DEVELOPER
        rayFlags |= DispatchRaysConstBuf.profileRayFlags;
#endif
        return rayFlags;
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
#if AMD_VULKAN || GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP
      , currNodePtr(INVALID_IDX)
#endif
#if GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP
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

#if AMD_VULKAN || GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP
    uint currNodePtr;
#endif

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

#if GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP
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

    uint nextNodePtr;
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

#if GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP == 0
    uint32_t packedReturnAddr; // The address of the function to return to, packed into 32 bits.
#endif

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
        GPU_ASSERT((_AmdGetRtip() == RayTracingIpLevel::RtIp1_1) ||
                   (_AmdGetRtip() == RayTracingIpLevel::RtIp2_0));

        packedStackTopOrParentPointer = ptr;
    }

    uint StackPtrTop()
    {
        GPU_ASSERT((_AmdGetRtip() == RayTracingIpLevel::RtIp1_1) ||
                   (_AmdGetRtip() == RayTracingIpLevel::RtIp2_0));
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

    void PackReturnAddress(uint64_t returnAddr)
    {
        packedReturnAddr = Pack64BitVpcTo32Bits(returnAddr);
    }

    uint64_t ReturnAddress()
    {
        return Unpack32BitVpcTo64BitVpc(packedReturnAddr, true);
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

    bool IsDeadLane()
    {
        return traversal.nextNodePtr == DEAD_LANE;
    }

    bool IsTraversal()
    {
        return IsValidNode(traversal.nextNodePtr);
    }

    bool IsChsOrMiss(in uint state)
    {
        return (state >= TRAVERSAL_STATE_COMMITTED_NOTHING) &&
#if GPURT_CONT_TRAVERSAL_EARLY_IS_AHS
               (state < TRAVERSAL_STATE_SUSPEND_TRAVERSAL);
#else
               true;
#endif
    }

    bool IsMiss(in uint state)
    {
        return IsChsOrMiss(state) && !IsValidNode(traversal.committed.instNodePtr);
    }

    bool IsAhs(in uint state)
    {
        return (state == TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_TRIANGLE);
    }

    bool IsIs(in uint state)
    {
        return ((state == TRAVERSAL_STATE_CANDIDATE_PROCEDURAL_PRIMITIVE) ||
            (state == TRAVERSAL_STATE_CANDIDATE_NON_OPAQUE_PROCEDURAL_PRIMITIVE));
    }

    bool IsChs(in uint state)
    {
        return IsChsOrMiss(state) && IsValidNode(traversal.committed.instNodePtr);
    }

    static _AmdSystemData MakeDeadLane();

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

#if GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP == 0
// Define specialized intrinsics.
// We use macros because HLSL does not have varargs or generics.
// The macros and intrinsics are defined by llpc.
DECLARE_ENQUEUE(, uint64_t returnAddr, _AmdSystemData data)

DECLARE_ENQUEUE(Traversal, uint64_t dummyReturnAddr, _AmdSystemData data)
DECLARE_ENQUEUE(RayGen, uint64_t dummyReturnAddr, _AmdDispatchSystemData data)
DECLARE_WAIT_ENQUEUE(Traversal, uint64_t dummyReturnAddr, _AmdSystemData data)

DECLARE_ENQUEUE(AnyHit, uint64_t returnAddr, _AmdAnyHitSystemData data, float2 candidateBarycentrics)
DECLARE_ENQUEUE(Intersection, uint64_t returnAddr, _AmdAnyHitSystemData data)
DECLARE_WAIT_ENQUEUE(, uint64_t returnAddr, _AmdSystemData data)

DECLARE_AWAIT(AnyHit, _AmdAnyHitSystemData, uint64_t returnAddr, _AmdAnyHitSystemData data)
DECLARE_AWAIT(CallShader, _AmdDispatchSystemData, uint64_t returnAddr, _AmdDispatchSystemData data)

// No returnAddr argument. The return address is instead included in the passed system data.
DECLARE_WAIT_AWAIT(Traversal, _AmdDispatchSystemData, _AmdSystemData data)

DECLARE_RESTORE_SYSTEM_DATA(, _AmdDispatchSystemData data)
DECLARE_RESTORE_SYSTEM_DATA(AnyHit, _AmdAnyHitSystemData data)
DECLARE_ACCEPT_HIT_ATTRIBUTES(, inout_param(_AmdAnyHitSystemData) data)

DECLARE_VALUE_I32_COUNT(, _AmdSystemData data)
DECLARE_VALUE_GET_I32(, _AmdSystemData data)
DECLARE_VALUE_SET_I32(, _AmdSystemData data)

DECLARE_GET_UNINITIALIZED(F32, float)
DECLARE_GET_UNINITIALIZED(I32, uint32_t)
DECLARE_GET_UNINITIALIZED(I64, uint64_t)
DECLARE_GET_UNINITIALIZED(SystemData, _AmdSystemData)

#if CONTINUATIONS_LGC_STACK_LOWERING
DECLARE_CONT_STACK_LOAD_LAST_USE(U32, uint32_t)
DECLARE_CONT_STACK_STORE(U32, uint32_t value)
DECLARE_CONT_STACK_LOAD_LAST_USE(U64, uint64_t)
DECLARE_CONT_STACK_STORE(U64, uint64_t value)
#endif
#endif

static _AmdSystemData _AmdSystemData::MakeDeadLane()
{
    _AmdSystemData data = _AmdGetUninitializedSystemData();
    data.traversal.nextNodePtr = DEAD_LANE;
    return data;
}

//=====================================================================================================================
#ifdef AMD_VULKAN
// Return the argument.
#else
#endif
static uint64_t GetVpcFromShaderId(uint32_t shaderId, uint priority)
{
    uint64_t vpc = Unpack32BitVpcTo64BitVpc(shaderId, /* unpackPriority = */ false);
    return GetVpcWithPriority(vpc, priority);
}

//=====================================================================================================================
static uint64_t GetVpcFromShaderIdAddr(GpuVirtualAddress addr, uint priority)
{
#ifdef __cplusplus
    return 1;
#else
    uint32_t shaderId = ConstantLoadDwordAtAddr(addr);
    return GetVpcFromShaderId(shaderId, priority);
#endif
}

//=====================================================================================================================
static uint64_t GetVpcFromShaderIdTable(
    GpuVirtualAddress tableAddress,
    uint index,
    uint stride,
    uint priority)
{
    return GetVpcFromShaderIdAddr(tableAddress + stride * index, priority);
}

//=====================================================================================================================
// Returns the 32-bit part of the hit group shader id containing the AHS shader id.
static uint32_t GetAnyHit32BitShaderId(
    uint hitGroupRecordIndex)
{
    const uint offset = DispatchRaysConstBuf.hitGroupTableStrideInBytes * hitGroupRecordIndex;

    const GpuVirtualAddress tableVa =
        PackUint64(DispatchRaysConstBuf.hitGroupTableBaseAddressLo, DispatchRaysConstBuf.hitGroupTableBaseAddressHi);
    if (tableVa == 0)
    {
       return 0;
    }
    return ConstantLoadDwordAtAddr(tableVa + offset + 8);
}

//=====================================================================================================================
// Returns the 64-bit VPC for the given AHS by loading its shader address, and setting the AHS priority.
static uint64_t GetAnyHitAddr(
    uint hitGroupRecordIndex)
{
    uint32_t shaderId = GetAnyHit32BitShaderId(hitGroupRecordIndex);
    return GetVpcFromShaderId(shaderId, SCHEDULING_PRIORITY_AHS);
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

    return GetAnyHit32BitShaderId(hitGroupRecordIndex) != 0;
}

//=====================================================================================================================
// Internal intrinsic function
static uint64_t GetInstanceNodeAddr(in uint64_t tlasBaseAddr, in uint nodePtr)
{
    const uint64_t instNodeAddr = tlasBaseAddr + ExtractNodePointerOffset(nodePtr);
    return instNodeAddr;
}

//=====================================================================================================================
// Driver internal intrinsic function
static uint64_t GetPrimtiveNodeAddr(in uint64_t tlasBaseAddr, in uint instNodePtr, in uint nodePtr)
{
    uint64_t instNodeAddr = GetInstanceNodeAddr(tlasBaseAddr, instNodePtr);
    return GetInstanceAddr(FetchInstanceDescAddr(instNodeAddr)) + ExtractNodePointerOffset(nodePtr);
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
// Implementation of DispatchRaysIndex.
export uint3 _cont_DispatchRaysIndex3(in _AmdDispatchSystemData data)
{
    return data.DispatchId();
}

//=====================================================================================================================
// Load dispatch dimensions from constant buffer.
static uint3 GetDispatchRaysDimensions()
{
    const uint  width  = DispatchRaysConstBuf.rayDispatchWidth;
    const uint  height = DispatchRaysConstBuf.rayDispatchHeight;
    const uint  depth  = DispatchRaysConstBuf.rayDispatchDepth;

    return uint3(width, height, depth);
}

//=====================================================================================================================
// Implementation of DispatchRaysDimensions().
export uint3 _cont_DispatchRaysDimensions3(in _AmdDispatchSystemData data)
{
    return GetDispatchRaysDimensions();
}

#if CONTINUATION_ON_GPU
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
    return state.ray.IncomingFlags();
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

    float tCurrentHw = 0.f;
    {
        tCurrentHw = primitive.rayTCurrent;
    }

    // AMD Gpu shifts the origin, so rayTCurrent is between 0 and (tMaxApp - tMinApp). Add tMinApp back for App's use.
    return tCurrentHw + data.ray.tMin;
}
#endif

//=====================================================================================================================
__decl uint3 AmdExtThreadIdInGroupCompute() DUMMY_UINT3_FUNC
__decl uint  AmdExtLoadDwordAtAddr(uint64_t addr, uint offset) DUMMY_UINT_FUNC
__decl void  AmdExtStoreDwordAtAddr(uint64_t addr, uint offset, uint value) DUMMY_VOID_FUNC

//=====================================================================================================================
// Map a thread to a ray, some threads could end up with non-existent (invalid) rays. Assuming numthreads(32, 1, 1).
// Note D3D12_DISPATCH_RAYS_DESC::(w x h x d) are organized to DispatchDims = (?, d, 1).
static uint3 GetDispatchId()
{
    const uint3 threadIdInGroup = AmdExtThreadIdInGroupCompute();
    const uint3 groupId         = AmdExtGroupIdCompute();
    const uint3 dims            = GetDispatchRaysDimensions();

    uint3 dispatchId;
    dispatchId.z = groupId.y;
    if ((dims.x > 1) && (dims.y > 1))
    {
        // Use 8 x 4 tiles.
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
        dispatchId.y = yTile * 4 + (threadIdInGroup.x / 8);
    }
    else
    {
        // Do a naive 1:1 simple map.
        const uint id = threadIdInGroup.x + 32 * groupId.x;
        const uint gridSize = dims.x * dims.y; // width x height
        dispatchId.y = id / dims.x;
        dispatchId.x = id - (dispatchId.y * dims.x);
    }

    return dispatchId;
}

//=====================================================================================================================
export _AmdDispatchSystemData _cont_SetupRayGen()
{
    // Driver code for swizzling dispatch rays indices. This function would be properly implemented inside
    // GPURT library
    _AmdDispatchSystemData data = (_AmdDispatchSystemData)0;
    uint3 dispatchId = GetDispatchId();
    data.PackDispatchId(dispatchId);
#if DEVELOPER
    data.parentId = -1;
#endif

    return data;
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

#if AMD_VULKAN

//=====================================================================================================================
export TriangleData _cont_TriangleVertexPositions(in _AmdSystemData data, in _AmdPrimitiveSystemState primitive)
{
    const GpuVirtualAddress instanceAddr = GetInstanceNodeAddr(data.ray.AccelStruct(), primitive.instNodePtr);
    {
        return FetchTriangleFromNode(GetInstanceAddr(FetchInstanceDescAddr(instanceAddr)), primitive.currNodePtr);
    }
}
#endif

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
// Returns the current local root index
export uint _cont_GetLocalRootIndex(in _AmdDispatchSystemData data)
{
    return data.shaderRecIdx;
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

#if GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP == 0
    if (_AmdContinuationStackIsGlobal())
    {
        const uint3 threadIdInGroup = AmdExtThreadIdInGroupCompute();
        const uint3 groupId         = AmdExtGroupIdCompute();

        // Do a naive 1:1 simple map. Also for now, assume numthreads(32, 1, 1)
        const uint id = threadIdInGroup.x + 32 * groupId.x;

        offset = id * DispatchRaysConstBuf.cpsFrontendStackSize;
    }
    else
#endif
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
static uint64_t GetTraversalVpc()
{
    // NOTE: DXCP uses a table for TraceRay, thus a load to traceRayGpuVa retrieves the actual traversal function
    // address. But Vulkan does not use the table so far, traceRayGpuVa is already the traversal function address.
#if AMD_VULKAN
    return PackUint64(DispatchRaysConstBuf.traceRayGpuVaLo,
                      DispatchRaysConstBuf.traceRayGpuVaHi);
#else
    return GetVpcFromShaderIdAddr(PackUint64(DispatchRaysConstBuf.traceRayGpuVaLo,
                                             DispatchRaysConstBuf.traceRayGpuVaHi),
                                  SCHEDULING_PRIORITY_TRAVERSAL);
#endif
}

//=====================================================================================================================
static uint64_t GetRayGenVpc()
{
    return GetVpcFromShaderIdAddr(PackUint64(DispatchRaysConstBuf.rayGenerationTableAddressLo,
                                             DispatchRaysConstBuf.rayGenerationTableAddressHi),
                                  SCHEDULING_PRIORITY_RGS);
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
        WriteRayHistoryTokenTopLevel(GetRayId(_cont_DispatchRaysIndex3(data.dispatch)), data.ray.AccelStruct());
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
static void RayHistoryInitStaticId()
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        AmdTraceRayInitStaticId();
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
        const uint rayId  = GetRayId(_cont_DispatchRaysIndex3(data.dispatch));
        RayDesc rayDesc   = (RayDesc)0;
        rayDesc.Origin    = data.ray.origin;
        rayDesc.Direction = data.ray.direction;
        rayDesc.TMin      = data.ray.tMin;
        rayDesc.TMax      = data.ray.tMax;

        data.counter.timerBegin = AmdTraceRaySampleGpuTimer();
        data.counter.dynamicId  = AllocateRayHistoryDynamicId();
        data.counter.SetCallerShaderType(_AmdGetShaderKind());

        WriteRayHistoryTokenBegin(rayId,
                                  _cont_DispatchRaysIndex3(data.dispatch),
                                  data.ray.AccelStruct(),
                                  data.ray.Flags(),
                                  data.ray.traceParameters,
                                  rayDesc,
                                  AmdTraceRayGetStaticId(),
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

    const uint     rayId    = GetRayId(_cont_DispatchRaysIndex3(data.dispatch));
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
        const uint instNodeIndex = FetchInstanceIdx(ConvertRtIpLevel(_AmdGetRtip()),
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
static uint2 RayHistoryGetIdentifierFromVPC(uint64_t vpc)
{
    // Zero out the metadata bits
    return uint2(SplitUint64(vpc).x & 0xFFFFFFC0, 0);
}

//=====================================================================================================================
static uint2 RayHistoryGetIdentifierFromShaderId(uint2 shaderId)
{
    // Zero out the dVGPR bits and the higher dWord
    return uint2(shaderId.x & 0xFFFFFFC0, 0);
}

//=====================================================================================================================
static void RayHistoryWriteTriangleHitResult(_AmdSystemData data, bool accept)
{
#if DEVELOPER
    if (EnableTraversalCounter())
    {
        WriteRayHistoryTokenTriangleHitResult(GetRayId(_cont_DispatchRaysIndex3(data.dispatch)),
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
        const uint rayId = GetRayId(_cont_DispatchRaysIndex3(data.dispatch));

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
        const uint rayId  = GetRayId(_cont_DispatchRaysIndex3(data.dispatch));
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
        WriteRayHistoryTokenNodePtr(GetRayId(_cont_DispatchRaysIndex3(data.dispatch)), nextNodePtr);
        UpdateWaveTraversalStatistics(ConvertRtIpLevel(_AmdGetRtip()), nextNodePtr);

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
        WriteRayHistoryTokenBottomLevel(GetRayId(_cont_DispatchRaysIndex3(data.dispatch)), bvhAddress);
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

        WriteTraversalCounter(GetRayId(_cont_DispatchRaysIndex3(data.dispatch)), counter);
    }
#endif
}

#if CONTINUATION_ON_GPU
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
            hitGroupRecordIndex = data.base.dispatch.shaderRecIdx;
        }
        // Compute hit group address and fetch shader identifiers
        const uint64_t anyHitAddr = GetAnyHitAddr(hitGroupRecordIndex);

        if (SplitUint64(anyHitAddr).x != 0)
        {
            // Call AnyHit
            // Hit attributes are added as an additional argument by the compiler
            const uint64_t resumeAddr = _AmdGetResumePointAddr();
            const uint64_t resumeAddrWithPrio = GetVpcWithPriority(resumeAddr, SCHEDULING_PRIORITY_IS);
            data = _AmdAwaitAnyHit(anyHitAddr, resumeAddrWithPrio, data);
            _AmdRestoreSystemDataAnyHit(data);
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

    const uint64_t addr = GetVpcFromShaderIdTable(callableTableBaseAddress,
                                                  index,
                                                  DispatchRaysConstBuf.callableTableStrideInBytes,
                                                  SCHEDULING_PRIORITY_CALLABLE);

    if (SplitUint64(addr).x == 0)
    {
        // See TODO above on how to handle this case better.
        return;
    }

    const uint callerShaderRecIdx = data.shaderRecIdx;
    data.shaderRecIdx = index; // the record index used by the callable shader

    const DXILShaderKind enclosingShaderType = _AmdGetShaderKind();
    const uint resumePrio = GetPriorityForShaderType(enclosingShaderType);
    const uint64_t resumeAddr = _AmdGetResumePointAddr();
    const uint64_t resumeAddrWithPrio = GetVpcWithPriority(resumeAddr, resumePrio);

    data = _AmdAwaitCallShader(addr, resumeAddrWithPrio, data);

    // for the resume part.
    data.shaderRecIdx = callerShaderRecIdx; // restores callerShaderRecIdx
    _AmdRestoreSystemData(data); // llvm inserts amd.dx.setLocalRootIndex(data.shaderRecIdx)
}

//=====================================================================================================================
// Returns the low part of the miss shader address and sets up the dispatch data to have the correct shader record
// index.
static uint64_t SetupMissShader(inout_param(_AmdSystemData) data, out_param(uint) shaderRecIdx)
{
    const uint64_t missTableBaseAddress =
        PackUint64(DispatchRaysConstBuf.missTableBaseAddressLo, DispatchRaysConstBuf.missTableBaseAddressHi);
    if (missTableBaseAddress == 0)
    {
        shaderRecIdx = 0;
        return 0;
    }

    shaderRecIdx = ExtractMissShaderIndex(data.ray.traceParameters);

    // Calculate miss shader record address
    const uint64_t shaderAddr = GetVpcFromShaderIdTable(missTableBaseAddress,
                                                        shaderRecIdx,
                                                        DispatchRaysConstBuf.missTableStrideInBytes,
                                                        SCHEDULING_PRIORITY_MISS);

    return shaderAddr;
}

//=====================================================================================================================
static HitGroupInfo GetHitGroupInfo(
    in _AmdSystemData           data,
    in uint                     state,
    in _AmdPrimitiveSystemState candidate)
{
    uint instanceContribution = 0;
    uint geometryIndex        = INVALID_IDX;

    {
        // Get primitive nodes to process based on candidate or committed hit
        const uint tlasNodePtr = (state < TRAVERSAL_STATE_COMMITTED_NOTHING) ?
            candidate.instNodePtr : data.traversal.committed.instNodePtr;

        // Fetch primitive node addresses
        const GpuVirtualAddress tlasAddr = data.ray.AccelStruct() + ExtractNodePointerOffset(tlasNodePtr);
        InstanceDesc desc = FetchInstanceDescAddr(tlasAddr);
        geometryIndex = (state < TRAVERSAL_STATE_COMMITTED_NOTHING) ?
            candidate.GeometryIndex() : data.traversal.committed.GeometryIndex();

        // Compute hit group address and fetch shader identifiers
        instanceContribution = desc.InstanceContributionToHitGroupIndex_and_Flags & 0x00ffffff;
    }

    return GetHitGroupInfo(ExtractRayContributionToHitIndex(data.ray.traceParameters),
                           ExtractMultiplierForGeometryContributionToHitIndex(data.ray.traceParameters),
                           geometryIndex,
                           instanceContribution);
}
#endif

//=====================================================================================================================
// Order matters, the following HLSL reference the functions and structs defined above. TODO: refactor these into a
// separate HLSL
#include "Continuations1_1.hlsl"
#include "Continuations2_0.hlsl"

#if CONTINUATION_ON_GPU
//=====================================================================================================================
// KernelEntry is entry function of the RayTracing continuation mode
export void _cont_KernelEntry()
{
    _AmdDispatchSystemData systemData;
    uint3 dispatchId = GetDispatchId();
    systemData.PackDispatchId(dispatchId);
    systemData.shaderRecIdx = _AmdGetUninitializedI32();
    GPU_ASSERT(dispatchId.z < DispatchRaysConstBuf.rayDispatchDepth);
    if (dispatchId.x >= DispatchRaysConstBuf.rayDispatchWidth ||
        dispatchId.y >= DispatchRaysConstBuf.rayDispatchHeight)
    {
        return;
    }

    _AmdContStackSetPtr(_cont_GetContinuationStackAddr());

    _AmdEnqueueRayGen(GetRayGenVpc(), _AmdGetUninitializedI64(), systemData);
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
    RayTracingIpLevel rtIpLevel = _AmdGetRtip();
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
    switch (rtIpLevel)
    {
    case RayTracingIpLevel::RtIp1_1:
        traversal = InitTraversalState1_1(instanceInclusionMask,
                                          rayDesc,
                                          isValid);
        break;
    case RayTracingIpLevel::RtIp2_0:
        traversal = InitTraversalState2_0(instanceInclusionMask,
                                          rayDesc,
                                          isValid);
        break;
    default:
        break;
    }

    _AmdSystemData data = (_AmdSystemData) 0;
    data.dispatch  = dispatch;
    data.ray       = ray;
    data.traversal = traversal;

    RayHistoryInitStaticId();
    RayHistoryWriteBegin(data);

    const uint     callerShaderRecIdx    = dispatch.shaderRecIdx; // 0 if from RayGen.
    const uint     parentId              = RayHistoryGetParentId(dispatch);
    const uint64_t traversalAddrWithPrio = GetTraversalVpc();

    // The type of the shader containing this TraceRay call, i.e. the shader we are inlined into.
    const DXILShaderKind enclosingShaderType = _AmdGetShaderKind();
    const uint           resumePrio          = GetPriorityForShaderType(enclosingShaderType);

    // NO control flow is allowed between _AmdGetResumePointAddr() and _AmdWaitAwaitTraversal().
    const uint64_t resumeAddr         = _AmdGetResumePointAddr();
    const uint64_t resumeAddrWithPrio = GetVpcWithPriority(resumeAddr, resumePrio);
    data.traversal.PackReturnAddress(resumeAddrWithPrio);
    dispatch = _AmdWaitAwaitTraversal(traversalAddrWithPrio, -1, data);

    // for the resume part.
    dispatch.shaderRecIdx = callerShaderRecIdx; // restores callerShaderRecIdx
    RayHistorySetParentId(dispatch, parentId);  // restore parentId
    _AmdRestoreSystemData(dispatch); // llvm inserts amd.dx.setLocalRootIndex(dispatch.shaderRecIdx)

    TraversalCounterWriteCounter(data);
}

//=====================================================================================================================
// Get the address of the function that should be called next, either a closest hit or a miss shader. If no hit or miss
// shader should be called, this method returns false (and in that case it should return to
// data.traversal.ReturnAddress()), otherwise it returns true.
static bool GetNextHitMissPc(
    inout_param(_AmdSystemData) data,
    uint state,
    _AmdPrimitiveSystemState candidate,
    out_param(uint64_t) nextShaderAddr)
{
    // MS
    if (data.IsMiss(state))
    {
        uint shaderRecIdx;
        const uint64_t missShaderAddr = SetupMissShader(data, shaderRecIdx);
        if (SplitUint64(missShaderAddr).x != 0)
        {
            // Valid MS
            data.dispatch.shaderRecIdx = shaderRecIdx;
            nextShaderAddr = missShaderAddr;
            return true;
        }
    }

    // CHS
    else if (data.IsChs(state))
    {
        HitGroupInfo hitInfo = (HitGroupInfo)0;
        {
            hitInfo = GetHitGroupInfo(data, state, candidate);
        }
        data.dispatch.shaderRecIdx = hitInfo.tableIndex;

        if ((data.ray.Flags() & RAY_FLAG_SKIP_CLOSEST_HIT_SHADER) == 0)
        {
            if (hitInfo.closestHitId.x != 0)
            {
                // Valid CHS
                nextShaderAddr = GetVpcFromShaderId(hitInfo.closestHitId.x, SCHEDULING_PRIORITY_CHS);
                return true;
            }
        }
    }
    return false;
}

//=====================================================================================================================
// Calls traversal for the current rtip.
static void TraversalInternal(
    inout_param(_AmdSystemData) data,
    inout_param(uint) state,
    inout_param(_AmdPrimitiveSystemState) candidate,
    inout_param(float2) candidateBarycentrics)
{
    switch (_AmdGetRtip())
    {
    case RayTracingIpLevel::RtIp1_1:
        TraversalInternal1_1(data, state, candidate, candidateBarycentrics);
        break;
    case RayTracingIpLevel::RtIp2_0:
        TraversalInternal2_0(data, state, candidate, candidateBarycentrics);
        break;
    default:
        break;
    }
}

static void EnqueueNextShader(bool hasWorkToDo, uint64_t nextShaderAddr, uint64_t returnAddr, _AmdSystemData data)
{
    if (!hasWorkToDo)
    {
        if (_AmdContinuationStackIsGlobal())
        {
            // No work to do = dead lane, jump to traversal as a synchronization point with an empty system data
            _AmdWaitEnqueueTraversal(GetTraversalVpc(), -1, _AmdGetUninitializedI64(), _AmdSystemData::MakeDeadLane());
        }
        else
        {
            // This case should only occur in sorting mode.
            GPU_ASSERT(false);
        }
    }

    const uint newState = data.traversal.committed.State();
    RayHistoryWriteEnd(data, newState);

    // Finished sorting, previously dead lanes may now have CHS|MS to execute and vice-versa
    if (nextShaderAddr != returnAddr)
    {
        const DXILShaderKind shaderKind = (DXILShaderKind)(data.IsMiss(newState) ?
                                          (int)DXILShaderKind::Miss : // convert to int to fix linux build error
                                          (int)DXILShaderKind::ClosestHit);
        RayHistoryWriteFunctionCall(data,
                                    RayHistoryGetIdentifierFromVPC(nextShaderAddr),
                                    data.dispatch.shaderRecIdx,
                                    shaderKind);

        _AmdEnqueue(nextShaderAddr, returnAddr, data);
    }

    // Return to RayGen. No need to set a priority, as it is already set in the stored return address.
    _AmdEnqueueRayGen(returnAddr, _AmdGetUninitializedI64(), data.dispatch);
}

//=====================================================================================================================
// Convenience helper calling Traversal on the debug/emulation path that returns _AmdTraversalResultData.
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
// Re-entrant traversal loop.
export void _cont_Traversal(
    inout_param(_AmdSystemData) data)
{
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION  >= 41
    data.ray.PackAccelStructAndRayflags(
        data.ray.AccelStruct(),
        (data.ray.IncomingFlags() & ~AmdTraceRayGetKnownUnsetRayFlags()) | AmdTraceRayGetKnownSetRayFlags());
#endif
    // Discard data that doesn't need to be kept alive during Traversal
    data.dispatch.shaderRecIdx = _AmdGetUninitializedI32();
    if (!IsBvhRebraid())
    {
        // lastInstanceRootNodePtr is only used for rebraid
        data.traversal.lastInstanceRootNodePtr = _AmdGetUninitializedI32();
    }

    // Write AHS/IS returned status
    if (!data.IsDeadLane())
    {
        RayHistoryWriteAnyHitOrProceduralStatus(data);
    }

    // Handle reordering of rays/threads before processing since dead lanes may become alive after sorting.
    // Execute traversal for active lanes.
    uint state                          = TRAVERSAL_STATE_COMMITTED_NOTHING;
    _AmdPrimitiveSystemState candidate  = (_AmdPrimitiveSystemState)0;
    float2 candidateBarycentrics        = float2(0.0f, 0.0f);

    if (data.IsTraversal())
    {
        TraversalInternal(data, state, candidate, candidateBarycentrics);
    }
    else
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
            candidate                   = data.traversal.committed;
            candidateBarycentrics       = data.traversal.committedBarycentrics;
        }
    }

    // Result used on the CPU path. This is an unused dummy return value on the GPU path.
    _AmdTraversalResultData result = (_AmdTraversalResultData)0;

    bool IsChsOrMiss = data.IsChsOrMiss(state);
    // For sorting-enabled global mem mode, we only enqueue CHS/Miss once all
    // lanes have arrived in this state.
    // In non-sorting mode, we immediately enqueue CHS/Miss. This is mostly
    // to replicate the old ProcessContinuation() behavior for now.
    // We might want to consider also waiting for all lanes here in the non-global
    // mem mode for consistency, and potentially also to have a common place
    // in between Traversal and CHS/Miss where extra work can be done just once
    // for all lanes, e.g. preparing system data for CHS/Miss.
    if ((_AmdContinuationStackIsGlobal() && WaveActiveAllTrue(IsChsOrMiss)) ||
        (!_AmdContinuationStackIsGlobal() && IsChsOrMiss))
    {
        if (Options::getThreadTraceEnabled())
        {
            // Rest of the function is scheduler.
            // Emit a function return token to end the current function.
            AmdExtD3DShaderIntrinsics_ShaderMarker(0x10);

            // Emit a function call token to start the scheduler function.
            AmdExtD3DShaderIntrinsics_ShaderMarker(0x11 |
                (/* scheduler */ 8 << 8) |
                (/* exec      */ WaveActiveCountBits(true) << 13));
        }

        uint64_t nextShaderAddr = 0;
        GetNextHitMissPc(data, state, candidate, nextShaderAddr);

        bool hasWorkToDo = true;
        // Avoid sorting on return addresses to RayGen (the case nextShaderValid == false), as it may create
        // unexpected behavior and might increase execution divergence. For example, we might have multiple resume
        // points due to divergent control flow in the TraceRay call, but those resume points are all copies of the same
        // code. If we sort and re-read only from one bin, we might prevent future TraceRay calls from reconverging
        // on traversal.
        if (_AmdContinuationStackIsGlobal() && (nextShaderAddr != 0))
        {
        }

        const uint64_t returnAddr = data.traversal.ReturnAddress();
        if (nextShaderAddr == 0)
        {
            nextShaderAddr = returnAddr;
        }
        EnqueueNextShader(hasWorkToDo, nextShaderAddr, returnAddr, data);
    }
    else
    {
        bool earlyIsAhsModeEnabled =
#if GPURT_CONT_TRAVERSAL_EARLY_IS_AHS
            true;
#else
            false;
#endif
        bool mayEnqueueTraversal = (_AmdContinuationStackIsGlobal() || earlyIsAhsModeEnabled);
        // If we cannot re-enqueue Traversal, then we already know that we are in AHS or IS state.
        if (!mayEnqueueTraversal || data.IsAhs(state) || data.IsIs(state))
        {
            HitGroupInfo hitInfo = (HitGroupInfo)0;
            {
                hitInfo = GetHitGroupInfo(data, state, candidate);
            }
            data.dispatch.shaderRecIdx = hitInfo.tableIndex;

            _AmdAnyHitSystemData anyHitData = (_AmdAnyHitSystemData)0;
            anyHitData.base = data;
            anyHitData.candidate = candidate;

            // AHS and IS re-enqueue SchedulerInternal when finished.
            if (data.IsAhs(state))
            {
                RayHistoryWriteFunctionCall(anyHitData.base,
                                            RayHistoryGetIdentifierFromShaderId(hitInfo.anyHitId),
                                            hitInfo.tableIndex,
                                            DXILShaderKind::AnyHit);

                const uint64_t addr = GetVpcFromShaderId(hitInfo.anyHitId.x, SCHEDULING_PRIORITY_AHS);
                const uint64_t returnAddr = _AmdGetCurrentFuncAddr();
                const uint64_t returnAddrWithPrio = GetVpcWithPriority(returnAddr, SCHEDULING_PRIORITY_TRAVERSAL);
                _AmdEnqueueAnyHit(addr, returnAddrWithPrio, anyHitData, candidateBarycentrics);
            }
            else
            {
                // Intersection shader
                GPU_ASSERT(data.IsIs(state));

                RayHistoryWriteFunctionCall(anyHitData.base,
                                            RayHistoryGetIdentifierFromShaderId(hitInfo.intersectionId),
                                            hitInfo.tableIndex,
                                            DXILShaderKind::Intersection);

                const uint64_t addr = GetVpcFromShaderId(hitInfo.intersectionId.x, SCHEDULING_PRIORITY_IS);
                const uint64_t returnAddr = _AmdGetCurrentFuncAddr();
                const uint64_t returnAddrWithPrio = GetVpcWithPriority(returnAddr, SCHEDULING_PRIORITY_TRAVERSAL);
                _AmdEnqueueIntersection(addr, returnAddrWithPrio, anyHitData);
            }
        }
        else
        {
            // The last remaining case is that we need to re-enqueue Traversal, because we are waiting for
            // other lanes to finish BVH traversal before sorting, or to resume suspended lanes that wait for
            // other lanes to run IS/AHS in early-is-ahs mode.
            //
            // Everything else needs to go back through scheduling/traversal, regardless of state
            // Note we don't need "Wait" here because priorities run AHS and IS first
            const uint64_t traversalAddr = _AmdGetCurrentFuncAddr();
            const uint64_t traversalAddrWithPrio = GetVpcWithPriority(traversalAddr, SCHEDULING_PRIORITY_TRAVERSAL);
            _AmdEnqueueTraversal(traversalAddrWithPrio, _AmdGetUninitializedI64(), data);
        }
    }
    // This is unreachable
}
#endif

//=====================================================================================================================
#if GPURT_DEBUG_CONTINUATION_TRAVERSAL_RTIP
// For debug. Support DxcpRt (non-continuation) to use Continuation traversal.
static IntersectionResult TraceRayInternalCPSDebug(
    in GpuVirtualAddress topLevelBvh,             // Top-level acceleration structure to use
    in uint              rayFlags,                // Ray flags
    in uint              traceRayParameters,      // Packed trace ray parameters
    in RayDesc           rayDesc,                 // Ray to be traced
    in uint              rayId,                   // Ray ID for profiling
    in uint              rtIpLevel                // HW version to determine TraceRay implementation
)
{
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION  >= 41
    rayFlags = (rayFlags & ~AmdTraceRayGetKnownUnsetRayFlags()) | AmdTraceRayGetKnownSetRayFlags();
#endif

    // Initialise ray system state from TraceRay parameters
    _AmdRaySystemState ray = (_AmdRaySystemState)0;
    ray.accelStruct        = topLevelBvh;
    ray.direction          = rayDesc.Direction;
    ray.origin             = rayDesc.Origin;
    ray.tMin               = rayDesc.TMin;
    ray.tMax               = rayDesc.TMax;
    ray.flags              = rayFlags;
    ray.traceParameters    = traceRayParameters;

    const bool isValid = true; // already verified in the caller

    // Initialise traversal system state from driver intrinsic
    _AmdTraversalState traversal = (_AmdTraversalState)0;
    switch (rtIpLevel)
    {
    case GPURT_RTIP1_1:
        traversal = InitTraversalState1_1(0,
                                          rayDesc,
                                          isValid);
        break;
    case GPURT_RTIP2_0:
        traversal = InitTraversalState2_0(0,
                                          rayDesc,
                                          isValid);
        break;
    default:
        break;
    }

    _AmdSystemData sysData = (_AmdSystemData)0;
    sysData.dispatch       = _cont_SetupRayGen();
    sysData.ray            = ray;
    sysData.traversal      = traversal;

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
                // This test reduces sp3 instructions, when rayFlags is a const containing RAY_FLAG_FORCE_OPAQUE. Note
                // in this case, this branch is not executed w/wo this test, but simpler sp3 boosts performance.
                if ((rayFlags & RAY_FLAG_FORCE_OPAQUE) == 0)
                {
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

                    uint status = HIT_STATUS_ACCEPT;
                    AmdTraceRayGetHitAttributes(ret.candidate.rayTCurrent, hitKind, status);

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
            }
            else
            {
                // Intersection requires the currently committed hit as RayTCurrent()
                ret.candidate.rayTCurrent = sysData.traversal.committed.rayTCurrent;

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

    return result;
}
#endif
