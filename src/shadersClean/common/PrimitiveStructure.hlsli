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
#ifndef PRIMITIVE_STRUCTURE_HLSLI
#define PRIMITIVE_STRUCTURE_HLSLI

#include "LaneGroup.hlsli"
#include "Common.hlsli"
#include "../debug/Debug.hlsli"

uint GetSharedMem(uint index);
void SetSharedMem(uint index, uint value);

static LaneGroup g_laneGroup;

// =====================================================================================================================
struct VertexMask
{
    uint64_t mask;

    void Init()
    {
        mask = 0;
    }

    void SetBit(uint index)
    {
        mask |= bit64(index);
    }

    bool TestBit(uint index)
    {
        return mask & bit64(index);
    }

    void PostfixOr()
    {
        mask = g_laneGroup.PostfixOr(mask);
    }

    uint CountBits()
    {
        return countbits64(mask);
    }

    VertexMask ReadFromLane(uint lane)
    {
        VertexMask newVertexMask;
        newVertexMask.mask = g_laneGroup.Broadcast(mask, lane);
        return newVertexMask;
    }

    uint CompactIndex(uint index)
    {
        return countbits64(mask & bits64(index));
    }

    uint3 CompactIndices(uint3 indices)
    {
        return uint3(CompactIndex(indices[0]), CompactIndex(indices[1]), CompactIndex(indices[2]));
    }
};

// =====================================================================================================================
uint GetLdsOffsetVertex(uint baseLdsOffset);

// =====================================================================================================================
void StoreVert(
    uint   baseLdsOffset,
    uint   index,
    float3 vert)
{
    const uint ldsOffset = GetLdsOffsetVertex(baseLdsOffset);
    SetSharedMem(ldsOffset + index * 3 + 0, asuint(vert.x));
    SetSharedMem(ldsOffset + index * 3 + 1, asuint(vert.y));
    SetSharedMem(ldsOffset + index * 3 + 2, asuint(vert.z));
}

// =====================================================================================================================
float3 ReadVert(
    uint baseLdsOffset,
    uint index)
{
    float3 vert;

    const uint ldsOffset = GetLdsOffsetVertex(baseLdsOffset);
    vert.x = asfloat(GetSharedMem(ldsOffset + index * 3 + 0));
    vert.y = asfloat(GetSharedMem(ldsOffset + index * 3 + 1));
    vert.z = asfloat(GetSharedMem(ldsOffset + index * 3 + 2));

    return vert;
}

// =====================================================================================================================
// Find the indices of the specified vertices if they already exist in LDS
uint3 FindVerts(
    uint   baseLdsOffset,
    uint   rangeStart,
    uint   rangeEnd,
    float3 v0,
    float3 v1,
    float3 v2)
{
    GPU_ASSERT(rangeStart <= rangeEnd);

    uint3 index = uint3(INVALID_IDX, INVALID_IDX, INVALID_IDX);

    const uint startVertex = rangeStart * 3;
    const uint endVertex = (rangeEnd + 1) * 3;

    for (uint i = startVertex; i < endVertex ; i++)
    {
        const float3 vert = ReadVert(baseLdsOffset, i);

        index.x = ((index.x == INVALID_IDX) && all(v0 == vert)) ? i : index.x;
        index.y = ((index.y == INVALID_IDX) && all(v1 == vert)) ? i : index.y;
        index.z = ((index.z == INVALID_IDX) && all(v2 == vert)) ? i : index.z;

        if (all(index != INVALID_IDX))
        {
            break;
        }
    }

    GPU_ASSERT(all(index != INVALID_IDX));

    return index - (rangeStart * 3).xxx;
}

#endif
