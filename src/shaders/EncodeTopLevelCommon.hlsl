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
#ifndef _ENCODETOPLEVELCOMMON_HLSL_
#define _ENCODETOPLEVELCOMMON_HLSL_

//=====================================================================================================================
bool IsUpdateInPlace()
{
    return Settings.isUpdateInPlace;
}

//=====================================================================================================================
bool IsRebraidEnabled()
{
    return (Settings.rebraidType != RebraidType::Off);
}

//=====================================================================================================================
bool IsFusedInstanceNode()
{
    return Settings.enableFusedInstanceNode;
}

//=====================================================================================================================
void WriteScratchInstanceNode(
    uint                offset,
    uint                instanceIndex,
    in BoundingBox      bbox,
    uint                boxNodeFlags,
    uint                instanceBasePointerLo,
    uint                instanceBasePointerHi,
    uint                instanceMask,
    uint                numActivePrims,
    float               cost)
{
    uint4 data;

    // LeafNode.bbox_min_or_v0, instanceIndex
    data = uint4(asuint(bbox.min), instanceIndex);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_V0_OFFSET, data);

    // LeafNode.bbox_max_or_v1, padding
    data = uint4(asuint(bbox.max), 0);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_V1_OFFSET, data);

    // LeafNode.instanceNodeBasePointerLo, instanceNodeBasePointerHi, numActivePrims, parent
    data = uint4(instanceBasePointerLo, instanceBasePointerHi, numActivePrims, 0xffffffff);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_INSTANCE_BASE_PTR_OFFSET, data);

    // When rebraid might occur, the traversal shader will read the root node pointer of the bottom level from the
    // instance extra data. If we actually perform rebraid during the build, the node pointer must be set to 0 to
    // indicate the leaf node is unused. The correct pointer will be filled in later. If we have rebraid support
    // enabled in traversal but not during this build, we need to set the pointer to the true root of the bottom level.
    const uint rootNodePointer = IsRebraidEnabled() ? 0 : CreateRootNodePointer();

    const uint packedFlags = PackInstanceMaskAndNodeFlags(instanceMask, boxNodeFlags);

    // type, flags, nodePointer, numPrimitivesAndDoCollapse
    data = uint4(NODE_TYPE_USER_NODE_INSTANCE, packedFlags, rootNodePointer, asuint(cost));
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_TYPE_OFFSET, data);
}

//=====================================================================================================================
uint CalcTopLevelBoxNodeFlags(
    uint geometryType,
    uint instanceFlags,
    uint blasNodeFlags)
{
    uint nodeFlags = 0;

    nodeFlags |= (instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_FORCE_OPAQUE)
                 ? 1 << BOX_NODE_FLAGS_ONLY_OPAQUE_SHIFT : 0;
    nodeFlags |= (instanceFlags & D3D12_RAYTRACING_INSTANCE_FLAG_FORCE_NON_OPAQUE)
                 ? 1 << BOX_NODE_FLAGS_ONLY_NON_OPAQUE_SHIFT : 0;

    // Propagate BLAS root node flags if instance does not override them.
    nodeFlags = (nodeFlags == 0) ? blasNodeFlags : nodeFlags;

    nodeFlags |= (geometryType == GEOMETRY_TYPE_TRIANGLES) ? 1 << BOX_NODE_FLAGS_ONLY_TRIANGLES_SHIFT
                                                           : 1 << BOX_NODE_FLAGS_ONLY_PROCEDURAL_SHIFT;

    return nodeFlags;
}

//=====================================================================================================================
uint GetInstanceMask(InstanceDesc desc)
{
    return desc.InstanceID_and_Mask >> 24;
}

//=====================================================================================================================
uint64_t GetAccelStructBaseAddr(InstanceDesc desc, bool allowUpdate)
{
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
            const uint metadataSizeInBytes = LoadDwordAtAddr(baseAddr + ACCEL_STRUCT_METADATA_SIZE_OFFSET);
            const uint64_t expectedBaseAddr = baseAddr + metadataSizeInBytes;

            if (baseAddrAccelStructHeader != expectedBaseAddr)
            {
                baseAddrAccelStructHeader = 0;
            }
        }
    }

    return baseAddrAccelStructHeader;
}

//=====================================================================================================================
float CalculateSAHCost(uint64_t baseAddrAccelStructHeader, BoundingBox rootBbox, BoundingBox instanceBbox)
{
    float cost = 0;

    float origSA = ComputeBoxSurfaceArea(rootBbox);
    float transformedSA = ComputeBoxSurfaceArea(instanceBbox);

    if (IsRebraidEnabled())
    {
        cost = transformedSA / origSA;
    }
    else
    {
        for (uint c = 0; c < 4; c++)
        {
            cost += asfloat(FetchHeaderField(baseAddrAccelStructHeader,
                                             ACCEL_STRUCT_HEADER_NUM_CHILD_PRIMS_OFFSET + (4 * c)));
        }

        cost = (transformedSA / origSA) * cost + (transformedSA * SAH_COST_AABBB_INTERSECTION);
    }

    return cost;
}

#endif
