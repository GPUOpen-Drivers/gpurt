/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "../shadersClean/common/ShaderDefs.hlsli"

#include "../shadersClean/build/BuildRootSignature.hlsli"

template<typename T>
T LoadInstanceDescBuffer(uint offset)
{
    return InstanceDescBuffer.Load<T>(offset);
}
#include "IndirectArgBufferUtils.hlsl"
#include "IntersectCommon.hlsl"
#include "BuildCommonScratch.hlsl"
#include "EncodeTopLevelCommon.hlsl"
#include "EncodeTopLevelBuild.hlsl"
#include "EncodeTopLevelUpdate.hlsl"

//=====================================================================================================================
uint64_t GetAccelStructBaseAddr(
    in  InstanceDesc desc,
    in  bool         allowUpdate,
    out uint         blasMetadataSize)
{
    blasMetadataSize = 0;

    bool forceInactive = false;
    if (allowUpdate == false)
    {
        const bool isTransformZero = !any(desc.Transform[0].xyz) &&
                                     !any(desc.Transform[1].xyz) &&
                                     !any(desc.Transform[2].xyz);
        if (isTransformZero || (GetInstanceMask(desc) == 0))
        {
            // Mark instance inactive with a NULL address
            forceInactive = true;
        }
    }

    uint64_t baseAddrAccelStructHeader = 0;
    if (forceInactive == false)
    {
        GpuVirtualAddress baseAddr = MakeGpuVirtualAddress(desc.accelStructureAddressLo,
                                                           desc.accelStructureAddressHiAndFlags);
        if (baseAddr != 0)
        {
            baseAddrAccelStructHeader = MakeGpuVirtualAddress(
                LoadDwordAtAddr(baseAddr + ACCEL_STRUCT_METADATA_VA_LO_OFFSET),
                LoadDwordAtAddr(baseAddr + ACCEL_STRUCT_METADATA_VA_HI_OFFSET));

            // In some odd cases where the BLAS memory gets trampled, this is a failsafe to skip such BLAS
            // and treat them as inactive rather than causing a page fault.
            blasMetadataSize = LoadDwordAtAddr(baseAddr + ACCEL_STRUCT_METADATA_SIZE_OFFSET);
            const uint64_t expectedBaseAddr = baseAddr + blasMetadataSize;

            if (baseAddrAccelStructHeader != expectedBaseAddr)
            {
                baseAddrAccelStructHeader = 0;
            }
        }
    }

    return baseAddrAccelStructHeader;
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void EncodeInstances(
    in uint3 globalThreadId : SV_DispatchThreadID)
{
    const bool allowUpdate = IsUpdateAllowed();

    uint index = globalThreadId.x;

    const NumPrimAndOffset numPrimAndOffset = LoadNumPrimAndOffset();
    const uint MaxNumPrimitives = ShaderConstants.numPrimitives;

    if (index < numPrimAndOffset.numPrimitives)
    {
        const InstanceDesc desc = LoadInstanceDesc(index, numPrimAndOffset.primitiveOffset);

        const uint tlasMetadataSize =
            IsUpdate() ? SrcBuffer.Load(ACCEL_STRUCT_METADATA_SIZE_OFFSET) : ShaderConstants.header.metadataSizeInBytes;

        // In Parallel Builds, Header is initialized after Encode, therefore, we can only use this var for updates
        AccelStructOffsets offsets = ShaderConstants.header.offsets;
        if (IsUpdate())
        {
            offsets = SrcBuffer.Load<AccelStructOffsets>(tlasMetadataSize + ACCEL_STRUCT_HEADER_OFFSETS_OFFSET);
        }

        const uint basePrimNodePointersOffset = offsets.primNodePtrs;
        const uint primNodePointerOffset = tlasMetadataSize + offsets.primNodePtrs + (index * sizeof(uint));
        const uint destScratchNodeOffset = (index * ByteStrideScratchNode) + ShaderConstants.offsets.bvhLeafNodeData;

        uint blasMetadataSize = 0;
        uint64_t baseAddrAccelStructHeader = GetAccelStructBaseAddr(desc, allowUpdate, blasMetadataSize);

        uint numActivePrims = 0;
        if (baseAddrAccelStructHeader != 0)
        {
            numActivePrims = FetchHeaderField(baseAddrAccelStructHeader, ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
            if ((allowUpdate == false) && (numActivePrims == 0))
            {
                // Mark instance inactive with a NULL address
                baseAddrAccelStructHeader = 0;
            }
        }

        if (IsUpdate())
        {
            EncodeInstancesUpdate(index,
                                  desc,
                                  tlasMetadataSize,
                                  offsets,
                                  primNodePointerOffset,
                                  baseAddrAccelStructHeader,
                                  numActivePrims,
                                  blasMetadataSize);
        }
        else
        {
            EncodeInstancesBuild(index,
                                 desc,
                                 primNodePointerOffset,
                                 destScratchNodeOffset,
                                 baseAddrAccelStructHeader,
                                 numActivePrims,
                                 allowUpdate,
                                 blasMetadataSize);
        }
    }

    IncrementPrimitiveTaskCounters(index,
                                   numPrimAndOffset.numPrimitives,
                                   MaxNumPrimitives);
}

//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
//=====================================================================================================================
void RefitInstanceBounds(
    in uint3 globalThreadId : SV_DispatchThreadID)
{
}
