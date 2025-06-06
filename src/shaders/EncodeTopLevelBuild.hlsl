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
#include "../shadersClean/common/ShaderDefs.hlsli"
#include "../shadersClean/common/ScratchNode.hlsli"

//=====================================================================================================================
void WriteScratchInstanceNode(
    uint                offset,
    uint                instanceIndex,
    in BoundingBox      bbox,
    uint                boxNodeFlags,
    uint64_t            instanceBasePointer,
    uint                instanceMask,
    uint                numActivePrims,
    uint                blasMetadataSize,
    uint                rebraidBLAS)
{
    uint4 data;

    // LeafNode.bbox_min_or_v0, instanceIndex
    data = uint4(asuint(bbox.min), instanceIndex);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_V0_OFFSET, data);

    // LeafNode.bbox_max_or_v1, padding
    data = uint4(asuint(bbox.max), 0);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_V1_OFFSET, data);

    // LeafNode.instanceNodeBasePointerLo, instanceNodeBasePointerHi, numActivePrims, parent
    data = uint4(LowPart(instanceBasePointer), HighPart(instanceBasePointer), numActivePrims, 0xffffffff);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_INSTANCE_BASE_PTR_OFFSET, data);

    // When rebraid might occur, the traversal shader will read the root node pointer of the bottom level from the
    // instance extra data. If we actually perform rebraid during the build, the node pointer must be set to 0 to
    // indicate the leaf node is unused. The correct pointer will be filled in later. If we have rebraid support
    // enabled in traversal but not during this build, we need to set the pointer to the true root of the bottom level.
    const uint rootNodePointer = rebraidBLAS ? 0 : CreateRootNodePointer();

    const uint packedFlags = PackScratchNodeFlags(instanceMask, boxNodeFlags, 0);

    // BLAS node pointer, BLAS metadata size, unused, packed flags
    data = uint4(rootNodePointer, blasMetadataSize, 0, packedFlags);
    WriteScratchNodeDataAtOffset(offset, SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET, data);
}

//=====================================================================================================================
void EncodeInstancesBuild(
    uint         index,
    InstanceDesc desc,
    uint         primNodePointerOffset,
    uint         destScratchNodeOffset,
    uint64_t     baseAddrAccelStructHeader,
    uint         numActivePrims,
    bool         allowUpdate,
    uint         blasMetadataSize)
{
    // If the BLAS address is NULL, the instance is inactive. Inactive instances cannot be activated during updates
    // so we always exclude them from the build.
    if (baseAddrAccelStructHeader != 0)
    {
        const uint geometryType =
            FetchHeaderField(baseAddrAccelStructHeader, ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET);

        // Initialise to invalid bounds
        BoundingBox boundingBox = InvalidBoundingBox;
        if (numActivePrims != 0)
        {
#if GPURT_BUILD_RTIP3_1
            if ((Settings.tlasRefittingMode == TlasRefittingMode::Early) && (geometryType != GEOMETRY_TYPE_AABBS))
            {
                // Compute instance bounds using BLAS KDOP
                const uint64_t blasBaseAddr = PackUint64(desc.accelStructureAddressLo, desc.accelStructureAddressHiAndFlags);
                const uint64_t kdopAddress = blasBaseAddr + ACCEL_STRUCT_METADATA_KDOP_OFFSET;
                const BoundingBox apexBoundingBox = ComputeInstanceBoundsFromKdop(kdopAddress, desc.Transform);
                if ((IsCorruptBox(apexBoundingBox) == false) && (IsInvalidBoundingBox(apexBoundingBox) == false))
                {
                    boundingBox = apexBoundingBox;
                }
                else
                {
                    // Fetch root bounds from BLAS header
                    const BoundingBox rootBbox = FetchHeaderRootBoundingBox(baseAddrAccelStructHeader);
                    // Compute transformed bounding box
                    boundingBox = GenerateInstanceBoundingBox(desc.Transform, rootBbox);
                }
            }
            else
#endif
            {
                // Fetch root bounds from BLAS header
                const BoundingBox rootBbox = FetchHeaderRootBoundingBox(baseAddrAccelStructHeader);
                // Compute transformed bounding box
                boundingBox = GenerateInstanceBoundingBox(desc.Transform, rootBbox);
            }
        }

        if ((allowUpdate == false) && (IsCorruptBox(boundingBox) || IsInvalidBoundingBox(boundingBox)))
        {
            // Deactivate instance permanently by setting bbox_min_or_v0.x to NaN
            WriteScratchNodeDataAtOffset(destScratchNodeOffset, 0, asuint(NaN));
        }
        else
        {
            const uint64_t instanceBasePointer =
                PackInstanceBasePointer(baseAddrAccelStructHeader,
                                        desc.InstanceContributionToHitGroupIndex_and_Flags >> 24,
                                        geometryType);

            const uint packedFlags = FetchHeaderField(baseAddrAccelStructHeader, ACCEL_STRUCT_HEADER_PACKED_FLAGS_OFFSET);
            const uint boxNodeFlags = CalcTopLevelBoxNodeFlags(geometryType,
                                                               desc.InstanceContributionToHitGroupIndex_and_Flags >> 24,
                                                               ExtractScratchNodeBoxFlags(packedFlags));

            // Propagate the instance mask from the BLAS by fetching it from the header.
            // Note, the header contains the exlusion mask so we take its bitwise complement to get the inclusion mask.
            // Set the instance inclusion mask to 0 for degenerate instances so that they are culled out.
            const uint instanceMask = (boundingBox.min.x > boundingBox.max.x) ?
                                      0 :
                                      ((desc.InstanceID_and_Mask >> 24) & ExtractScratchNodeInstanceMask(packedFlags));

            uint rebraidBLAS = 0;

            if (IsRebraidEnabled())
            {
#if GPURT_BUILD_RTIP3_1
                if (EnableNonPrioritySortingRebraid())
                {
                    const uint tempInfo = FetchHeaderField(baseAddrAccelStructHeader, ACCEL_STRUCT_HEADER_INFO_OFFSET);
                    rebraidBLAS = (tempInfo >> ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_SHIFT) &
                                  ACCEL_STRUCT_HEADER_INFO_REBRAID_FLAGS_MASK;
                }
                else
#endif
                {
                    rebraidBLAS = true;
                }
            }

            // Write scratch node
            WriteScratchInstanceNode(destScratchNodeOffset,
                                     index,
                                     boundingBox,
                                     boxNodeFlags,
                                     instanceBasePointer,
                                     instanceMask,
                                     numActivePrims,
                                     blasMetadataSize,
                                     rebraidBLAS);

            if (numActivePrims != 0)
            {
                // Update scene bounding box
                // Always encode the scene bounds
                // When rebraid is enabled the scene bounds won't change when nodes are opened
                UpdateSceneBounds(ShaderConstants.offsets.sceneBounds, boundingBox);

                // Rebraid handles size and centroids, so no need to update them here
                if (IsRebraidEnabled() == false)
                {
                    // Only when size bits are enabled, update the scene size
                    if (IsMortonSizeBitsEnabled(ShaderConstants.numMortonSizeBits))
                    {
                        UpdateSceneSize(ShaderConstants.offsets.sceneBounds, boundingBox);
                    }

                    // Only if the centroid bounds are required, update the centroid bounds
                    if (IsCentroidMortonBoundsEnabled() || IsConciseMortonBoundsEnabled())
                    {
                        UpdateCentroidBounds(ShaderConstants.offsets.sceneBounds, boundingBox);
                    }
                }
            }
        }

        {
            // Store invalid prim node pointer for now during first time builds.
            // If the instance is active, EncodeHwBvh will write it in.
            DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
        }
    }
    else
    {
        // Deactivate instance permanently by setting bbox_min_or_v0.x to NaN
        WriteScratchNodeDataAtOffset(destScratchNodeOffset, 0, asuint(NaN));

        {
            DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
        }
    }
}
