/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "EncodeTopLevelCommon.hlsl"

//=====================================================================================================================
void EncodeInstancesBuild(
    uint         index,
    InstanceDesc desc,
    uint         primNodePointerOffset,
    uint         destScratchNodeOffset,
    uint64_t     baseAddrAccelStructHeader,
    uint         numActivePrims,
    bool         allowUpdate)
{
    // If the BLAS address is NULL, the instance is inactive. Inactive instances cannot be activated during updates
    // so we always exclude them from the build.
    if (baseAddrAccelStructHeader != 0)
    {
        const uint geometryType =
            FetchHeaderField(baseAddrAccelStructHeader, ACCEL_STRUCT_HEADER_GEOMETRY_TYPE_OFFSET);

        const uint64_t instanceBasePointer =
            PackInstanceBasePointer(baseAddrAccelStructHeader,
                                    desc.InstanceContributionToHitGroupIndex_and_Flags >> 24,
                                    geometryType);

        desc.accelStructureAddressLo         = LowPart(instanceBasePointer);
        desc.accelStructureAddressHiAndFlags = HighPart(instanceBasePointer);

        // calc transformed AABB
        BoundingBox boundingBox;

        if (numActivePrims != 0)
        {
            // Fetch root bounds from BLAS header
            const BoundingBox rootBbox = FetchHeaderRootBoundingBox(baseAddrAccelStructHeader);

            boundingBox = GenerateInstanceBoundingBox(desc.Transform, rootBbox);

        }
        else
        {
            // Generate a invalid dummy bounding box
            boundingBox = InvalidBoundingBox;
        }

        if ((allowUpdate == false) && (IsCorruptBox(boundingBox) || IsInvalidBoundingBox(boundingBox)))
        {
            // Deactivate instance permanently by setting bbox_min_or_v0.x to NaN
            WriteScratchNodeDataAtOffset(destScratchNodeOffset, 0, asuint(NaN));
        }
        else
        {
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

            // Write scratch node
            WriteScratchInstanceNode(destScratchNodeOffset,
                                     index,
                                     boundingBox,
                                     boxNodeFlags,
                                     desc.accelStructureAddressLo,
                                     desc.accelStructureAddressHiAndFlags,
                                     instanceMask,
                                     numActivePrims);

            if (numActivePrims != 0)
            {
                if (IsRebraidEnabled() == false)
                {
                    // Update scene bounding box
                    if (Settings.sceneBoundsCalculationType == SceneBoundsBasedOnGeometryWithSize)
                    {
                        UpdateSceneBoundsWithSize(ShaderConstants.sceneBoundsByteOffset, boundingBox);
                    }
                    else
                    {
                        UpdateSceneBounds(ShaderConstants.sceneBoundsByteOffset, boundingBox);
                    }
                }
            }
        }

        // Store invalid prim node pointer for now during first time builds.
        // If the instance is active, BuildQBVH will write it in.
        DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
    }
    else
    {
        // Deactivate instance permanently by setting bbox_min_or_v0.x to NaN
        WriteScratchNodeDataAtOffset(destScratchNodeOffset, 0, asuint(NaN));

        DstMetadata.Store(primNodePointerOffset, INVALID_IDX);
    }
}
