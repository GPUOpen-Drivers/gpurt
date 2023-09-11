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
#ifndef _SCRATCHNODE_HLSL
#define _SCRATCHNODE_HLSL

#include "rayTracingDefs.h"

//=====================================================================================================================
// The structure is 64-byte aligned
struct ScratchNode
{
    float3 bbox_min_or_v0;
    uint   left_or_primIndex_or_instIndex; // left child for internal nodes /
                                           // primitive index for primitive nodes /
                                           // instance index for instance nodes
    float3 bbox_max_or_v1;
    uint   right_or_geometryIndex;         // right child for internal nodes /
                                           // geometryIndex if it's a leaf node
    float3 sah_or_v2_or_instBasePtr;       // SAH cost, area and density for internal nodes /
                                           // vertex2 for triangle nodes /
                                           // instanceNodeBasePointerLo, instanceNodeBasePointerHi for instance node
    uint   parent;

    uint   type;                           // type [0:2], triangle id [3:18] for max of 2 compressed triangles
    uint   flags_and_instanceMask;         // flags [0:7], instanceMask [8:15]
    uint   splitBox_or_nodePointer;        // TriangleSplitBox index for triangle nodes /
                                           // BLAS node pointer for instance nodes
    uint   numPrimitivesAndDoCollapse;     // number of tris collapsed, doCollapse is boolean bit in the LSB /
                                           // scratch node index of the tri in the pair in PAIR_TRIANGLE_COMPRESSION
};

#define SCRATCH_NODE_FLAGS_DISABLE_TRIANGLE_SPLIT_SHIFT 31
#define SCRATCH_NODE_FLAGS_DISABLE_TRIANGLE_SPLIT_MASK (1 << SCRATCH_NODE_FLAGS_DISABLE_TRIANGLE_SPLIT_SHIFT)

#define SCRATCH_NODE_BBOX_MIN_OFFSET                  0
#define SCRATCH_NODE_V0_OFFSET                        SCRATCH_NODE_BBOX_MIN_OFFSET
#define SCRATCH_NODE_LEFT_OFFSET                      12
#define SCRATCH_NODE_PRIMITIVE_ID_OFFSET              SCRATCH_NODE_LEFT_OFFSET
#define SCRATCH_NODE_INSTANCE_ID_OFFSET               SCRATCH_NODE_PRIMITIVE_ID_OFFSET
#define SCRATCH_NODE_BBOX_MAX_OFFSET                  16
#define SCRATCH_NODE_V1_OFFSET                        SCRATCH_NODE_BBOX_MAX_OFFSET
#define SCRATCH_NODE_RIGHT_OFFSET                     28
#define SCRATCH_NODE_GEOMETRY_INDEX_OFFSET            SCRATCH_NODE_RIGHT_OFFSET
#define SCRATCH_NODE_COST_OFFSET                      32
#define SCRATCH_NODE_SA_OFFSET                        36
#define SCRATCH_NODE_DENSITY_OFFSET                   40
#define SCRATCH_NODE_V2_OFFSET                        SCRATCH_NODE_COST_OFFSET
#define SCRATCH_NODE_INSTANCE_BASE_PTR_OFFSET         SCRATCH_NODE_V2_OFFSET
#define SCRATCH_NODE_INSTANCE_NUM_PRIMS_OFFSET        SCRATCH_NODE_DENSITY_OFFSET
#define SCRATCH_NODE_PARENT_OFFSET                    44
#define SCRATCH_NODE_TYPE_OFFSET                      48
#define SCRATCH_NODE_FLAGS_AND_INSTANCE_MASK_OFFSET   52
#define SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET           56
#define SCRATCH_NODE_NODE_POINTER_OFFSET              SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET
#define SCRATCH_NODE_NUM_MORTON_CELLS_OFFSET          SCRATCH_NODE_SPLIT_BOX_INDEX_OFFSET
#define SCRATCH_NODE_NUM_PRIMS_AND_DO_COLLAPSE_OFFSET 60
#define SCRATCH_NODE_SIZE                             64

//=====================================================================================================================
static bool IsLeafNode(in uint x, uint numActivePrims)
{
    return (x >= (numActivePrims - 1));
}

//=====================================================================================================================
static uint CalcScratchNodeOffset(
    uint baseScratchNodesOffset,
    uint nodeIndex)
{
    return baseScratchNodesOffset + (nodeIndex * SCRATCH_NODE_SIZE);
}

//======================================================================================================================
static uint CalculateScratchBvhNodesOffset(
    in uint numActivePrims,
    in uint numLeafNodes,
    in uint bvhNodesOffset,
    in bool noCopySortedNodes)
{
    const uint offset = noCopySortedNodes ? (numLeafNodes - numActivePrims) * SCRATCH_NODE_SIZE : 0;
    return bvhNodesOffset + offset;
}

//=====================================================================================================================
// Extract node flags from scratch node
static uint ExtractScratchNodeFlags(
    in uint flagsAndInstanceMask)
{
    return (flagsAndInstanceMask & 0xff);
}

//=====================================================================================================================
// Extract instance mask from scratch node
static uint ExtractScratchNodeInstanceMask(
    in uint flagsAndInstanceMask)
{
    // Note, we store the instance exclusion mask (instead of inclusion) so that we can combine the
    // masks together using a AND operation similar to boxNodeFlags.
    return (~flagsAndInstanceMask >> 8) & 0xff;
}

//=====================================================================================================================
static uint FetchScratchNodeNumPrimitives(
    ScratchNode node,
    bool        isLeaf)
{
    if (isLeaf)
    {
        return 1;
    }
    else
    {
        return node.numPrimitivesAndDoCollapse >> 1;
    }
}

//=====================================================================================================================
static float FetchScratchInternalNodeCost(ScratchNode node)
{
    return node.sah_or_v2_or_instBasePtr.x;
}

//=====================================================================================================================
static float FetchScratchLeafNodeCost(ScratchNode node)
{
    return asfloat(node.numPrimitivesAndDoCollapse);
}

//=====================================================================================================================
static bool IsNodeActive(ScratchNode node)
{
    // Inactive nodes force v0.x to NaN during encode
    return !isnan(node.bbox_min_or_v0.x);
}

//=====================================================================================================================
static bool IsNodeLinkedOnly(ScratchNode node)
{
    // if this the "2nd" triangle of a set of paired triangle
    return (node.splitBox_or_nodePointer == PAIRED_TRI_LINKONLY);
}

#endif
