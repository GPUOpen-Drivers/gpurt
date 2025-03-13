/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
// This function indicates a distance metric between the two keys where each internal node splits the hierarchy
// Optionally, we can use the squared distance to compute the distance between two centroids
uint64_t Delta64(
    uint id)
{
    const uint left = id;
    const uint right = id + 1;

    // Special handling of duplicated codes: use their indices as a fallback
    const uint64_t leftCode = LDS.ReadMortonCode(left);
    const uint64_t rightCode = LDS.ReadMortonCode(right);

    // logical xor can be used instead of finding the index of the highest differing bit as we can compare the numbers.
    // The higher the index of the differing bit, the larger the number
    return (leftCode != rightCode) ? (leftCode ^ rightCode) : (left ^ right);
}

//=====================================================================================================================
bool IsSplitRight(
    uint left,
    uint right)
{
    return (Delta64(right) < Delta64(left - 1));
}

//======================================================================================================================
void BuildLBVH(
    uint threadId, uint numPrimRefs)
{
    if (threadId >= numPrimRefs)
    {
        return;
    }

    // Leaf nodes cover exactly one range of keys indexed by the primitive index
    const uint primRefIndex = LDS.ReadPrimRefIdx(threadId);

    uint left = threadId;
    uint right = threadId;

    const uint numInternalNodes = (numPrimRefs - 1);

    // Initialise current node index to leaf node
    uint currNodePtr = MakePrimRefChildPtr(primRefIndex, numPrimRefs);

    while (1)
    {
        if ((left == 0) && (right == numInternalNodes))
        {
            // Store the index of the root node
            WriteRootNodeIndex(currNodePtr);
            break;
        }

        // we look at the internal nodes with the index i-1 and i and compare the values returned by the
        // delta function. The one with the lowest value will be the parent because it splits the hierarchy
        // between two more similar clusters (subtrees) than the other node.
        const bool isRight = ((left == 0) || ((right != numInternalNodes) && IsSplitRight(left, right)));
        uint parentNodeIndex = isRight ? right : left - 1;

        WriteBvh2ChildPtr(parentNodeIndex, currNodePtr, (isRight == false));

        // The atomic exchange below guarantees a child has written its addresses to its parent before the parent tries
        // to read the address, but a memory barrier is needed to ensure the write is ordered before the atomic
        // operation in other threads.
        DeviceMemoryBarrier();

        // ... and pass the opposite range of keys to the parent
        const uint rangeLimit = isRight ? left : right;

        const uint previous = LDS.InterlockedXchg(parentNodeIndex, 3, rangeLimit);

        if (previous != 0xffffffff)
        {
            if (isRight)
            {
                right = previous;
            }
            else
            {
                left = previous;
            }

            // Both child nodes have been processed and the current thread will write the parent node
            uint2 childPtrs = ReadBvh2ChildPtrs(parentNodeIndex);

            const BoundingBox bbox0 = ReadBvh2NodeBounds(GetBvh2NodeIdx(childPtrs.x));
            const BoundingBox bbox1 = ReadBvh2NodeBounds(GetBvh2NodeIdx(childPtrs.y));

            MergeBvh2Nodes(parentNodeIndex, childPtrs.x, childPtrs.y, bbox0, bbox1);

            // Traverse up to parent node
            currNodePtr = parentNodeIndex;
        }
        else
        {
            break;
        }
    }
}
