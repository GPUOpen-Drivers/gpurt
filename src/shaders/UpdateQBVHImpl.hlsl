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
// Signal parent node
uint SignalParentNode(
    uint baseFlagsOffset,
    uint parentNodePointer,
    uint decodeNodePtrAsFp16)
{
    const uint parentNodeOffset = ExtractNodePointerOffset(parentNodePointer);
    const uint parentNodeIndex = CalcQbvhInternalNodeIndex(parentNodeOffset, decodeNodePtrAsFp16);
    const uint flagOffset = baseFlagsOffset + (parentNodeIndex * sizeof(uint));

    uint originalFlagValue = 0;
    ScratchBuffer.InterlockedAdd(flagOffset, 1, originalFlagValue);

    return originalFlagValue;
}

//=====================================================================================================================
// Computes box node flags by merging individual child flags
uint ComputeMergedFloat32BoxNodeFlags(
    in uint flags)
{
    const uint flags0 = ExtractBoxNodeFlagsField(flags, 0);
    const uint flags1 = ExtractBoxNodeFlagsField(flags, 1);
    const uint flags2 = ExtractBoxNodeFlagsField(flags, 2);
    const uint flags3 = ExtractBoxNodeFlagsField(flags, 3);

    return (flags0 & flags1 & flags2 & flags3);
}

//=====================================================================================================================
void UpdateNodeFlagsTopLevel(
    uint metadataSize,
    uint parentNodeOffset)
{
    // Read child pointers from source buffer
    const uint childPointers[4] = SrcBuffer.Load<uint[4]>(parentNodeOffset);

    // Read node flags from parent. Note, instance nodes update their relevant flag bits at EncodeTopLevel
    uint parentNodeFlags = DstMetadata.Load(parentNodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET);

    for (uint i = 0; i < 4; i++)
    {
        // Note this also returns false for INVALID_IDX
        if (IsBoxNode32(childPointers[i]))
        {
            const uint childOffset = metadataSize + ExtractNodePointerOffset(childPointers[i]);
            const uint childFlags = DstMetadata.Load(childOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET);

            const uint flags0 = ExtractBoxNodeFlagsField(childFlags, 0);
            const uint flags1 = ExtractBoxNodeFlagsField(childFlags, 1);
            const uint flags2 = ExtractBoxNodeFlagsField(childFlags, 2);
            const uint flags3 = ExtractBoxNodeFlagsField(childFlags, 3);

            const uint mergedNodeFlags = flags0 & flags1 & flags2 & flags3;

            parentNodeFlags = ClearBoxNodeFlagsField(parentNodeFlags, i);
            parentNodeFlags = SetBoxNodeFlagsField(parentNodeFlags, mergedNodeFlags, i);
        }
    }

    DstMetadata.Store(parentNodeOffset + FLOAT32_BOX_NODE_FLAGS_OFFSET, parentNodeFlags);
}

#include "UpdateQBVH1_1.hlsl"

//=====================================================================================================================
void UpdateQBVHImpl(
    uint globalID,
    uint numWorkItems,
    uint numThreads)
{
    switch (Settings.rtIpLevel)
    {
        default:
            UpdateQBVHImpl1_1(globalID, numWorkItems, numThreads);
            return;
    }
}
