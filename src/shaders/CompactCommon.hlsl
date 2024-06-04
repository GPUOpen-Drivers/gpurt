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
#ifndef _COMPACTCOMMON_HLSL
#define _COMPACTCOMMON_HLSL

//=====================================================================================================================
uint CalcCompactedSize(
    in  AccelStructHeader  srcHeader,
    in  bool               topLevelBuild,
    out AccelStructOffsets dstOffsets,
    out uint               metadataSizeInBytes)
{
    // Acceleration structure data starts with the header (not including the metadata)
    uint runningOffset = sizeof(AccelStructHeader);

    AccelStructOffsets offsets;
    offsets.internalNodes = runningOffset;

    uint internalNodeSize = 0;
    uint leafNodeSize     = 0;

    {
        leafNodeSize = GetBvhNodeSizeLeaf(srcHeader.geometryType, 0);
        internalNodeSize = sizeof(Float32BoxNode);
    }

    if (topLevelBuild == false)
    {
        internalNodeSize = srcHeader.numInternalNodesFp32 * internalNodeSize +
                           srcHeader.numInternalNodesFp16 * sizeof(Float16BoxNode);
        runningOffset += internalNodeSize;

        offsets.leafNodes = runningOffset;
        {
            leafNodeSize = srcHeader.numLeafNodes * leafNodeSize;
        }
        runningOffset += leafNodeSize;

        offsets.geometryInfo = runningOffset;
        runningOffset += srcHeader.numDescs * sizeof(GeometryInfo);

        offsets.primNodePtrs = runningOffset;
        runningOffset += srcHeader.numPrimitives * sizeof(uint);
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

        {
            // Top level acceleration structures do not have geometry info.
            offsets.geometryInfo = 0;
        }

        offsets.primNodePtrs = runningOffset;
        runningOffset += srcHeader.numPrimitives * sizeof(uint);
    }

    {
        metadataSizeInBytes = CalcMetadataSizeInBytes(internalNodeSize, leafNodeSize);
        metadataSizeInBytes = Pow2Align(metadataSizeInBytes, 128);
    }

    uint totalSizeInBytes = runningOffset + metadataSizeInBytes;

    dstOffsets = offsets;

    // Return total size including metadata
    return totalSizeInBytes;
}

#endif
