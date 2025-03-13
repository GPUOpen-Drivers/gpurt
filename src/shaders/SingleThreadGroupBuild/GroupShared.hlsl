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
#define MAX_ELEMENT_PER_THREAD 16
groupshared uint SharedMem[MAX_ELEMENT_PER_THREAD * STGB_THREADGROUP_SIZE];
uint GetSharedMem(uint index)
{
    return SharedMem[index];
}
void SetSharedMem(uint index, uint value)
{
    SharedMem[index] = value;
}

//=====================================================================================================================
// LDS layout by phases
//
// 'x' denotes unused slots
//
// Encode      : 0: [bbox_min.xyz, bbox_max.xyz, sa_min, sa_max], 1: [active_prim_count, x, x, x, x, x, x, x]
//             : [2:NumThreads] is used for storing KDOP min/max
// Morton      : [morton_lo, morton_hi, primitive_index_0, x, x, x, x, x] x numPrimitives
// Sort        : [morton_lo, morton_hi, primitive_index_0, x, x, x, x, x] x pow2Align(numPrimitives)
// LBVH        : [morton_lo, morton_hi, primitive_index_0, r, x, x, x, x] x numPrimitives
// PLOC        : [bbox_min.xyz, bbox_max.xyz, neighbor, prefixSum] x numActivePrims (numActiveWaves for prefix sum)
// EncodeHwBvh : [child_node_array_0, 1, 2, 3, 4, 5, 6, 7]         x maxNumInternalNodes
//             : group_size-1: [box_ref_count, prim_ref_count, x, x, x, x, x, x]
//======================================================================================================================
struct GroupSharedMemLayout
{
    uint GetOffset(uint threadId, uint elementId)
    {
        return (threadId * MAX_ELEMENT_PER_THREAD) + elementId;
    }

    void Write(uint threadId, uint elementId, uint value)
    {
        SharedMem[GetOffset(threadId, elementId)] = value;
    }

    uint Read(uint threadId, uint elementId)
    {
        return SharedMem[GetOffset(threadId, elementId)];
    }

    uint InterlockedXchg(uint threadId, uint elementId, uint value)
    {
        uint orig;
        InterlockedExchange(SharedMem[GetOffset(threadId, elementId)], value, orig);
        return orig;
    }

    uint InterlockedIncr(uint threadId, uint elementId, uint value)
    {
        uint orig;
        InterlockedAdd(SharedMem[GetOffset(threadId, elementId)], value, orig);
        return orig;
    }

    void InitSceneBounds()
    {
        Write(0, 0, FloatToUint(+FLT_MAX));
        Write(0, 1, FloatToUint(+FLT_MAX));
        Write(0, 2, FloatToUint(+FLT_MAX));
        Write(0, 3, FloatToUint(-FLT_MAX));
        Write(0, 4, FloatToUint(-FLT_MAX));
        Write(0, 5, FloatToUint(-FLT_MAX));
    }

    BoundingBox ReadSceneBounds()
    {
        BoundingBox bbox;
        bbox.min.x = UintToFloat(Read(0, 0));
        bbox.min.y = UintToFloat(Read(0, 1));
        bbox.min.z = UintToFloat(Read(0, 2));
        bbox.max.x = UintToFloat(Read(0, 3));
        bbox.max.y = UintToFloat(Read(0, 4));
        bbox.max.z = UintToFloat(Read(0, 5));
        return bbox;
    }

    void UpdateSceneBounds(float3 boundsMin, float3 boundsMax)
    {
        InterlockedMin(SharedMem[GetOffset(0, 0)], FloatToUint(boundsMin.x));
        InterlockedMin(SharedMem[GetOffset(0, 1)], FloatToUint(boundsMin.y));
        InterlockedMin(SharedMem[GetOffset(0, 2)], FloatToUint(boundsMin.z));
        InterlockedMax(SharedMem[GetOffset(0, 3)], FloatToUint(boundsMax.x));
        InterlockedMax(SharedMem[GetOffset(0, 4)], FloatToUint(boundsMax.y));
        InterlockedMax(SharedMem[GetOffset(0, 5)], FloatToUint(boundsMax.z));
    }

    float2 ReadSceneSizeMinMax()
    {
        return float2(UintToFloat(Read(0, 6)), UintToFloat(Read(0, 7)));
    }

    void UpdateSceneSize(float surfaceAreaMin, float surfaceAreaMax)
    {
        InterlockedMin(SharedMem[GetOffset(0, 6)], FloatToUint(surfaceAreaMin));
        InterlockedMax(SharedMem[GetOffset(0, 7)], FloatToUint(surfaceAreaMax));
    }

    void InitSceneSizeMinMax()
    {
        Write(0, 6, FloatToUint(+FLT_MAX));
        Write(0, 7, FloatToUint(-FLT_MAX));
    }

    void WriteWaveActivePrimRefCount(uint waveId, uint value)
    {
        Write(1 + waveId, 0, value);
    }

    uint ReadWaveActivePrimRefCount(uint waveId)
    {
        return Read(1 + waveId, 0);
    }

    void UpdateKDopMin(uint planeIndex, float value)
    {
        InterlockedMin(SharedMem[GetOffset(2 + planeIndex, 0)], FloatToUint(value));
    }

    void UpdateKDopMax(uint planeIndex, float value)
    {
        InterlockedMax(SharedMem[GetOffset(2 + planeIndex, 1)], FloatToUint(value));
    }

    void WriteKDopMin(uint planeIndex, float value)
    {
        SharedMem[GetOffset(2 + planeIndex, 0)] = FloatToUint(value);
    }

    void WriteKDopMax(uint planeIndex, float value)
    {
        SharedMem[GetOffset(2 + planeIndex, 1)] = FloatToUint(value);
    }

    uint ReadKDopMinUint(uint planeIndex)
    {
        return SharedMem[GetOffset(2 + planeIndex, 0)];
    }

    uint ReadKDopMaxUint(uint planeIndex)
    {
        return SharedMem[GetOffset(2 + planeIndex, 1)];
    }

    // Phase 2: Morton codes, primitive reference indices, BVH2 range flags
    void WriteMortonCode(uint threadId, uint64_t value)
    {
        Write(threadId, 0, LowPart(value));
        Write(threadId, 1, HighPart(value));
    }

    void WritePrimRefIdx(uint threadId, uint value)
    {
        Write(threadId, 2, value);
    }

    void WriteBvh2RangeFlag(uint threadId, uint value)
    {
        Write(threadId, 3, value);
    }

    uint64_t ReadMortonCode(uint threadId)
    {
        return PackUint64(Read(threadId, 0), Read(threadId, 1));
    }

    uint ReadPrimRefIdx(uint threadId)
    {
        return Read(threadId, 2);
    }

    // PLOC
    void WriteCachedBounds(uint threadId, BoundingBox bbox)
    {
        Write(threadId, 0, asuint(bbox.min.x));
        Write(threadId, 1, asuint(bbox.min.y));
        Write(threadId, 2, asuint(bbox.min.z));
        Write(threadId, 3, asuint(bbox.max.x));
        Write(threadId, 4, asuint(bbox.max.y));
        Write(threadId, 5, asuint(bbox.max.z));
    }

    void WriteNeighborIndex(uint threadId, uint value)
    {
        Write(threadId, 6, value);
    }

    void WriteClusterId(uint threadId, uint value)
    {
        Write(threadId, 7, value);
    }

    BoundingBox ReadCachedBounds(uint threadId)
    {
        BoundingBox bbox;
        bbox.min.x = asfloat(Read(threadId, 0));
        bbox.min.y = asfloat(Read(threadId, 1));
        bbox.min.z = asfloat(Read(threadId, 2));
        bbox.max.x = asfloat(Read(threadId, 3));
        bbox.max.y = asfloat(Read(threadId, 4));
        bbox.max.z = asfloat(Read(threadId, 5));
        return bbox;
    }

    uint ReadNeighborIndex(uint threadId)
    {
        return Read(threadId, 6);
    }

    uint ReadClusterId(uint threadId)
    {
        return Read(threadId, 7);
    }

    void UpdateNeighborIndex(uint threadId, uint value)
    {
        InterlockedMin(SharedMem[GetOffset(threadId, 6)], value);
    }
};
