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
#ifndef _COMPACTCOMMON_HLSL
#define _COMPACTCOMMON_HLSL

#if GPURT_BUILD_RTIP3_1
#include "../shadersClean/common/gfx12/QuantizedBVH8BoxNode.hlsli"
#include "rtip3_1.hlsli"
#endif

//=====================================================================================================================
uint CalcCompactedSize(
    in  AccelStructHeader  srcHeader,
    in  bool               topLevelBuild,
    out AccelStructOffsets dstOffsets,
    out uint               metadataSizeInBytes)
{
    // Acceleration structure data starts with the header (not including the metadata)
    uint runningOffset = sizeof(AccelStructHeader);

    AccelStructOffsets offsets = (AccelStructOffsets)0;
    offsets.internalNodes = runningOffset;

    uint internalNodeSize = 0;
    uint leafNodeSize     = 0;

#if GPURT_BUILD_RTIP3_1
    if (Settings.rtIpLevel == GPURT_RTIP3_1)
    {
        internalNodeSize = sizeof(QuantizedBVH8BoxNode);
        leafNodeSize = PRIMITIVE_STRUCT_SIZE_IN_BYTE;
    }
    else
#endif
#if GPURT_BUILD_RTIP3
    if (Settings.highPrecisionBoxNodeEnable)
    {
        leafNodeSize = GetBvhNodeSizeLeaf(srcHeader.geometryType, 0);
        internalNodeSize = sizeof(HighPrecisionBoxNode);
    }
    else
#endif
    {
        leafNodeSize = GetBvhNodeSizeLeaf(srcHeader.geometryType, 0);
        internalNodeSize = sizeof(Float32BoxNode);
    }

    if (topLevelBuild == false)
    {
        internalNodeSize = srcHeader.numInternalNodesFp32 * internalNodeSize;
#if GPURT_BUILD_RTIP3_1
        if (Settings.rtIpLevel < GPURT_RTIP3_1)
#endif
        {
            internalNodeSize += srcHeader.numInternalNodesFp16 * sizeof(Float16BoxNode);
        }
        runningOffset += internalNodeSize;

        offsets.leafNodes = runningOffset;
#if GPURT_BUILD_RTIP3_1
        if (srcHeader.UsesLegacyLeafCompression())
        {
            // In RTIP 3.1, legacy compression leaves gaps in the AS, so we take numActivePrims as worst case.
            leafNodeSize = srcHeader.numActivePrims * leafNodeSize;
        }
        else
#endif
        {
            leafNodeSize = srcHeader.numLeafNodes * leafNodeSize;
        }
        runningOffset += leafNodeSize;

        offsets.geometryInfo = runningOffset;
        runningOffset += srcHeader.numDescs * sizeof(GeometryInfo);

        {
            offsets.primNodePtrs = runningOffset;
            runningOffset += srcHeader.numPrimitives * sizeof(uint);

#if GPURT_BUILD_RTIP3_1
            if (Settings.rtIpLevel == GPURT_RTIP3_1)
            {
                // We are using numInternalNodesFp16 to store fat leaf offsets for update.
                runningOffset += (srcHeader.numInternalNodesFp16) * sizeof(uint);
            }
#endif
        }

#if GPURT_BUILD_RTIP3_1
        if (srcHeader.obbBlasMetadataOffset != 0)
        {
            runningOffset += BLAS_OBB_METADATA_SIZE;
        }
#endif
    }
    else
    {
        // TLAS always uses 32-bit internal nodes
        internalNodeSize = srcHeader.numInternalNodesFp32 * internalNodeSize;
        runningOffset += internalNodeSize;

        offsets.leafNodes = runningOffset;
        leafNodeSize
            = srcHeader.numLeafNodes * GetBvhNodeSizeLeaf(PrimitiveType::Instance, Settings.enableFusedInstanceNode);
        runningOffset += leafNodeSize;

#if GPURT_BUILD_RTIP3_1
        if (Settings.rtIpLevel == GPURT_RTIP3_1)
        {
            // Instance sideband data is stored in geometryInfo offset in RTIP 3.1
            offsets.geometryInfo = runningOffset;
            runningOffset += srcHeader.numLeafNodes * sizeof(InstanceSidebandData);
        }
        else
#endif
        {
            // Top level acceleration structures do not have geometry info.
            offsets.geometryInfo = 0;
        }

        {
            offsets.primNodePtrs = runningOffset;
            runningOffset += srcHeader.numPrimitives * sizeof(uint);

#if GPURT_BUILD_RTIP3_1
            if (Settings.rtIpLevel == GPURT_RTIP3_1)
            {
                // We are using numInternalNodesFp16 to store fat leaf offsets for update.
                runningOffset += (srcHeader.numInternalNodesFp16) * sizeof(uint);
            }
#endif
        }
    }

#if GPURT_BUILD_RTIP3_1
    if (Settings.rtIpLevel >= GPURT_RTIP3_1)
    {
        // RTIP 3.1 does not store parent pointers in metadata.
        // Maintain the original metadata size which includes variable padding to avoid channel imbalance.
        metadataSizeInBytes = srcHeader.metadataSizeInBytes;
    }
    else
#endif
    {
        metadataSizeInBytes = CalcMetadataSizeInBytes(internalNodeSize, leafNodeSize);
        metadataSizeInBytes = Pow2Align(metadataSizeInBytes, 128);
    }

    uint totalSizeInBytes = runningOffset + metadataSizeInBytes;

#if GPURT_BUILD_RTIP3_1
    if ((Settings.rtIpLevel >= GPURT_RTIP3_1) && Settings.enableBvhChannelBalancing)
    {
        const uint cacheLineSize = 256;
        const uint metadataSizeWithoutPadding = Pow2Align(ACCEL_STRUCT_METADATA_HEADER_SIZE, cacheLineSize);
        const uint maxPaddedMetadataSize = metadataSizeWithoutPadding + ACCEL_STRUCT_METADATA_MAX_PADDING;

        // Max padding so that compacted size is deterministic.
        totalSizeInBytes += (maxPaddedMetadataSize - metadataSizeInBytes);
    }
#endif

    dstOffsets = offsets;

    // Return total size including metadata
    return totalSizeInBytes;
}

#endif
