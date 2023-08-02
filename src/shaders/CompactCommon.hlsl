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
#ifndef _COMPACTCOMMON_HLSL
#define _COMPACTCOMMON_HLSL

//=====================================================================================================================
uint CalcCompactedSize(
    in  AccelStructHeader  srcHeader,
    in  uint               accelStructType,
    out AccelStructOffsets dstOffsets,
    out uint               metadataSizeInBytes)
{
    // Acceleration structure data starts with the header (not including the metadata)
    uint runningOffset = sizeof(AccelStructHeader);

    AccelStructOffsets offsets;
    offsets.internalNodes = runningOffset;

    uint internalNodeSize = 0;
    uint leafNodeSize     = GetBvhNodeSizeLeaf(srcHeader.geometryType, 0);
    uint boxNodeSize      = sizeof(Float32BoxNode);

    if (accelStructType == BOTTOM_LEVEL)
    {
        internalNodeSize = srcHeader.numInternalNodesFp32 * boxNodeSize +
                           srcHeader.numInternalNodesFp16 * sizeof(Float16BoxNode);
        runningOffset += internalNodeSize;

        offsets.leafNodes = runningOffset;
        leafNodeSize = srcHeader.numLeafNodes * leafNodeSize;
        runningOffset += leafNodeSize;

        offsets.geometryInfo = runningOffset;
        runningOffset += srcHeader.numDescs * sizeof(GeometryInfo);

        offsets.primNodePtrs = runningOffset;
        runningOffset += srcHeader.numPrimitives * sizeof(uint);
    }
    else
    {
        const uint enableFusedInstanceNode = srcHeader.UsesFusedInstanceNode();

        // TLAS always uses 32-bit internal nodes
        internalNodeSize = srcHeader.numInternalNodesFp32 * boxNodeSize;
        runningOffset += internalNodeSize;

        offsets.leafNodes = runningOffset;
        leafNodeSize = srcHeader.numLeafNodes * GetBvhNodeSizeLeaf(PrimitiveType::Instance, enableFusedInstanceNode);
        runningOffset += leafNodeSize;

        // Top level acceleration structures do not have geometry info.
        offsets.geometryInfo = 0;

        offsets.primNodePtrs = runningOffset;
        runningOffset += srcHeader.numPrimitives * sizeof(uint);
    }

    // Align metadata size to 128B cache line boundary
    metadataSizeInBytes = Pow2Align(CalcMetadataSizeInBytes(internalNodeSize,
                                                            leafNodeSize),
                                    128);

    dstOffsets = offsets;

    // Return total size including metadata
    return runningOffset + metadataSizeInBytes;
}

//=====================================================================================================================
void WriteCompactedSize(
    in uint              emitCompactSize,
    in uint              accelStructType)
{
    const AccelStructHeader header = DstBuffer.Load<AccelStructHeader>(0);

    // Unused
    uint metadataSizeInBytes = 0;

    // Unused
    AccelStructOffsets offsets;

    // Calculate compacted size
    const uint compactedSize = CalcCompactedSize(header,
                                                 accelStructType,
                                                 offsets,
                                                 metadataSizeInBytes);

    WriteAccelStructHeaderField(ACCEL_STRUCT_HEADER_COMPACTED_BYTE_SIZE_OFFSET, compactedSize);

    if (emitCompactSize != 0)
    {
        EmitBuffer.Store2(0, uint2(compactedSize, 0));
    }
}
#endif
