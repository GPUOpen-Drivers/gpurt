/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
//

#include "IntersectCommon.hlsl"
#if DEVELOPER
#include "../../gpurt/gpurtCounter.h"
#endif
#include "../shadersClean/traversal/DispatchRaysConstants.hlsli"
#include "../shadersClean/traversal/TraversalCounter.hlsli"

#include "llpc/GpurtIntrinsics.h"

static float ApplyTMinBias(float originalTMin)
{
    return UseUnbiasedOrigin() ? 0.0f : originalTMin;
}

static bool EvaluateTriangleHit(float tMin, float tHit, float tCurrent)
{
    return (tHit < tCurrent) && (UseUnbiasedOrigin() ? (tHit > tMin) : true);
}

//=====================================================================================================================
static uint ExtractInstanceInclusionMask(in uint traceRayParameters)
{
    return (traceRayParameters & 0xFF);
}

//=====================================================================================================================
static uint ExtractRayContributionToHitIndex(in uint traceRayParameters)
{
    return ((traceRayParameters >> 8) & 0xF);
}

//=====================================================================================================================
static uint ExtractMultiplierForGeometryContributionToHitIndex(in uint traceRayParameters)
{
    return ((traceRayParameters >> 12) & 0xF);
}

//=====================================================================================================================
static uint ExtractMissShaderIndex(in uint traceRayParameters)
{
    return ((traceRayParameters >> 16) & 0xFFFF);
}

//=====================================================================================================================
static uint2 GetShaderId(GpuVirtualAddress tableAddress, uint index, uint stride)
{
    return LoadDwordAtAddrx2(tableAddress + stride * index);
}

//=====================================================================================================================
static uint CalculateHitGroupRecordAddress(
    uint RayContributionToHitGroupIndex,
    uint MultiplierForGeometryContributionToHitGroupIndex,
    uint GeometryContributionToHitGroupIndex,
    uint InstanceContributionToHitGroupIndex)
{
    return (RayContributionToHitGroupIndex +                   // from shader: TraceRay()
           (MultiplierForGeometryContributionToHitGroupIndex * // from shader: TraceRay()
            GeometryContributionToHitGroupIndex) +             // system generated index of geometry in
                                                               // bottom level acceleration structure (0,1,2,3..)
            InstanceContributionToHitGroupIndex                // from instance
           );
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
#if GPURT_BUILD_RTIP3 && ((GPURT_RTIP_LEVEL == 30) || (GPURT_RTIP_LEVEL == 0))
    case RayTracingIpLevel::RtIp3_0:
        level = GPURT_RTIP3_0;
        break;
#endif
#if GPURT_BUILD_RTIP3_1 && ((GPURT_RTIP_LEVEL == 31) || (GPURT_RTIP_LEVEL == 0))
    case RayTracingIpLevel::RtIp3_1:
        level = GPURT_RTIP3_1;
        break;
#endif
    default:
        break;
    }

    return level;
}

//=====================================================================================================================
static HitGroupInfo GetHitGroupInfoFromRecordIndex(
    uint hitGroupRecordIndex)
{
#if __cplusplus
    HitGroupInfo hitInfo;
    hitInfo.tableIndex = hitGroupRecordIndex;
#else
    const uint offset = DispatchRaysConstBuf.hitGroupTableStrideInBytes * hitGroupRecordIndex;

    const GpuVirtualAddress tableVa =
        PackUint64(DispatchRaysConstBuf.hitGroupTableBaseAddressLo, DispatchRaysConstBuf.hitGroupTableBaseAddressHi);

    HitGroupInfo hitInfo = (HitGroupInfo)0;

    if (tableVa != 0)
    {
        const uint4 d0 = LoadDwordAtAddrx4(tableVa + offset);
        const uint2 d1 = LoadDwordAtAddrx4(tableVa + offset + 0x10).xy;
        hitInfo.closestHitId   = d0.xy;
        hitInfo.anyHitId       = d0.zw;
        hitInfo.intersectionId = d1.xy;
    }
    hitInfo.tableIndex     = hitGroupRecordIndex;
#endif

    return hitInfo;
}

//=====================================================================================================================
static HitGroupInfo GetHitGroupInfo(
    uint RayContributionToHitGroupIndex,
    uint MultiplierForGeometryContributionToHitGroupIndex,
    uint GeometryContributionToHitGroupIndex,
    uint InstanceContributionToHitGroupIndex)
{
    const uint hitGroupRecordIndex = CalculateHitGroupRecordAddress(RayContributionToHitGroupIndex,
                                                                    MultiplierForGeometryContributionToHitGroupIndex,
                                                                    GeometryContributionToHitGroupIndex,
                                                                    InstanceContributionToHitGroupIndex);

    return GetHitGroupInfoFromRecordIndex(hitGroupRecordIndex);
}

#if GPURT_BUILD_RTIP3_1
//=====================================================================================================================
static uint64_t UnpackBaseAddrFromInstanceNodePtr3_1(
    in uint64_t instanceNodePtr)
{
    return (instanceNodePtr >> 24) << 7;
}

//=====================================================================================================================
static uint64_t GetInstanceNodeBaseAddr3_1(
    in uint64_t instanceNodePtr)
{
    // Unpack 64-bit instance node pointer into the 64-bit base address and 32-bit node offset
    // See PackInstanceNodePtr3_1().
    //

    const uint64_t tlasBaseAddr = UnpackBaseAddrFromInstanceNodePtr3_1(instanceNodePtr);
    const uint32_t nodeOffset = (uint32_t(instanceNodePtr) & 0x00ffffff) << 6;

    return tlasBaseAddr + nodeOffset;
}

//=====================================================================================================================
static uint64_t CalculateInstanceSidebandBaseAddr3_1(
    in uint64_t tlasBaseAddr,
    const uint32_t nodeOffset
)
{
    // Fetch acceleration structure offsets from header
    const uint32_t leafNodesOffset =
        FetchHeaderOffsetField(tlasBaseAddr, ACCEL_STRUCT_OFFSETS_LEAF_NODES_OFFSET);

    const uint32_t sidebandDataOffset =
        FetchHeaderOffsetField(tlasBaseAddr, ACCEL_STRUCT_OFFSETS_GEOMETRY_INFO_OFFSET);

    const uint32_t sidebandOffset = ComputeInstanceSidebandOffset(nodeOffset, leafNodesOffset, sidebandDataOffset);
    return tlasBaseAddr + sidebandOffset;
}

//=====================================================================================================================
static uint64_t GetInstanceSidebandBaseAddr3_1(
    in uint64_t instanceNodePtr)
{
    // Unpack 64-bit instance node pointer into the 64-bit base address and 32-bit node offset
    // See PackInstanceNodePtr3_1().
    //
    const uint64_t tlasBaseAddr = UnpackBaseAddrFromInstanceNodePtr3_1(instanceNodePtr);
    const uint32_t nodeOffset = (uint32_t(instanceNodePtr) & 0x00ffffff) << 6;

    return CalculateInstanceSidebandBaseAddr3_1(tlasBaseAddr, nodeOffset);
}

//=====================================================================================================================
// unpack instance sideband address given tlas and nodePointer
static uint64_t GetInstanceSidebandBaseAddr3_1(
    in uint64_t tlasBaseAddr,
    in uint     instanceNodePtr)
{
    const uint32_t nodeOffset = ExtractNodePointerOffset3_1(instanceNodePtr);
    return CalculateInstanceSidebandBaseAddr3_1(tlasBaseAddr, nodeOffset);
}

//=====================================================================================================================
static uint64_t PackInstanceNodePtr3_1(
    in uint64_t instanceBaseAddr,
    in uint32_t instanceNodePtr)
{
    // Unpack 64-bit instance node pointer into the 64-bit base address and 32-bit node offset
    // Our hardware supports 48-bit virtual addressing. The current layout of the acceleration structure ensures that
    // the base pointer is 128-byte aligned which allows us to pack the node address into 41 bits.
    //
    // The ray tracing APIs (DXR/VulkanRT) enforce a 256 byte alignment on the base address of the acceleration
    // structure memory address. On RTIP3.1+ we do not require metadata section, we should simply change the
    // data layout such that the acceleration structure header is at the base of the gpu memory. Thus, the base pointer
    // itself will be 256-aligned and we can pack into 40 bits.
    //
    // struct PackedInstanceNodePtr
    // {
    //     uint64_t tlasBaseAddr   : 40; // 128-byte aligned acceleration structure base address
    //     uint64_t instNodeOffset : 24; // 64-byte aligned instance node offset
    // };

    uint64_t packedNodePtr = (instanceBaseAddr >> 7) << 24;
    packedNodePtr |= (instanceNodePtr >> 3) & 0x00ffffff;

    return packedNodePtr;
}
#endif

//=====================================================================================================================
static uint64_t CalculateInstanceNodePtr64(
    in uint32_t rtIpLevel,
    in uint64_t instanceBaseAddr,
    in uint32_t instanceNodePtr)
{
#if GPURT_BUILD_RTIP3_1
    if (rtIpLevel >= GPURT_RTIP3_1)
    {
        return PackInstanceNodePtr3_1(instanceBaseAddr, instanceNodePtr);
    }
#endif
    return CalculateNodeAddr64(instanceBaseAddr, instanceNodePtr);
}

//=====================================================================================================================
static uint FetchInstanceIdx(
    in uint     rtIpLevel,
    in uint64_t accelStruct,
    in uint     instNodePtr)
{
    uint instNodeIndex = 0;

#if GPURT_BUILD_RTIP3_1
    if (rtIpLevel == GPURT_RTIP3_1)
    {
        GpuVirtualAddress sidebandBaseAddr = GetInstanceSidebandBaseAddr3_1(accelStruct, instNodePtr);
        instNodeIndex = LoadDwordAtAddr(sidebandBaseAddr + RTIP3_INSTANCE_SIDEBAND_INSTANCE_INDEX_OFFSET);
    }
    else
#endif
    {
        GpuVirtualAddress instanceNodePtr = accelStruct + ExtractNodePointerOffset(instNodePtr);
        instNodeIndex = LoadDwordAtAddr(instanceNodePtr +
                                        INSTANCE_DESC_SIZE +
                                        RTIP1_1_INSTANCE_SIDEBAND_INSTANCE_INDEX_OFFSET);
    }
    return instNodeIndex;
}

#if DEVELOPER|| (0 && !defined(__cplusplus))
//=====================================================================================================================
static uint GetRayId(in uint3 dispatchRaysIndex)
{
    const uint flatRayIndex = dispatchRaysIndex.x +
        (dispatchRaysIndex.y * DispatchRaysConstBuf.rayDispatchWidth) +
        (dispatchRaysIndex.z * DispatchRaysConstBuf.rayDispatchWidth * DispatchRaysConstBuf.rayDispatchHeight);

    return flatRayIndex;
}
#endif
