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
#ifndef _GFX12_CHILD_INFO_H
#define _GFX12_CHILD_INFO_H

#include "../BoundingBox.hlsli"
#include "../Bits.hlsli"
#include "../Math.hlsli"

//=====================================================================================================================
// Quantized child node information
#if __cplusplus
#pragma pack(push, 4)
#endif
struct ChildInfo
{
#if __cplusplus
    union
    {
        struct
        {
            uint32_t minX         : 12;
            uint32_t minY         : 12;
            uint32_t cullingFlags : 4;
            uint32_t unused       : 4;
        };

        uint32_t minXMinYAndCullingFlags;
    };

    union
    {
        struct
        {
            uint32_t minZ         : 12;
            uint32_t maxX         : 12;
            uint32_t instanceMask : 8;
        };

        uint32_t minZMaxXAndInstanceMask;
    };

    union
    {
        struct
        {
            uint32_t maxY      : 12;
            uint32_t maxZ      : 12;
            uint32_t nodeType  : 4;
            uint32_t nodeRange : 4;
        };

        uint32_t maxYMaxZNodeTypeAndNodeRange;
    };

    ChildInfo() : ChildInfo(0)
    {
    }

    ChildInfo(uint val)
    {
        memset(this, val, sizeof(ChildInfo));
    }
#else
    uint32_t minXMinYAndCullingFlags;
    uint32_t minZMaxXAndInstanceMask;
    uint32_t maxYMaxZNodeTypeAndNodeRange;
#endif

    static const uint3 Invalid;

    static uint3 BuildPacked(uint3 quantMin,
                             uint3 quantMax,
                             uint  boxNodeFlags,
                             uint  instanceMask,
                             uint  nodeType,
                             uint  nodeRangeLength);

    void Load(uint3 packedData);
    void Init();
    void Invalidate();
    uint3 Min();
    uint3 Max();
    void SetMin(uint3 min);
    void SetMax(uint3 max);
    uint CullingFlags();
    void SetCullingFlags(uint cullingFlags);
    uint InstanceMask();
    void SetInstanceMask(uint instanceMask);
    uint NodeType();
    void SetNodeType(uint nodeType);
    uint NodeRangeLength();
    void SetNodeRangeLength(uint nodeBoundaries);
    bool Valid();
    BoundingBox DecodeBounds(float3 origin, uint3 exponents);
};
#if __cplusplus
#pragma pack(pop)
#endif

#define QUANTIZED_NODE_CHILD_INFO_OFFSET_MINX_MINY_CULLING_FLAGS        0  // offsetof(ChildInfo, minXMinYAndCullingFlags);
#define QUANTIZED_NODE_CHILD_INFO_OFFSET_MINZ_MAXX_INSTANCE_MASK        4  // offsetof(ChildInfo, minZMaxXAndInstanceMask);
#define QUANTIZED_NODE_CHILD_INFO_OFFSET_MAXY_MAXZ_NODE_TYPE_AND_RANGE  8  // offsetof(ChildInfo, maxYMaxZNodeTypeAndNodeRange);
#define QUANTIZED_NODE_CHILD_INFO_STRIDE                                12 // sizeof(ChildInfo) / sizeof(uint32_t);

#if __cplusplus
static_assert(sizeof(ChildInfo) == 12);
#endif

//=====================================================================================================================
#if __cplusplus
inline const uint3 ChildInfo::Invalid = uint3(0xFFFFFFFF, 0x00000FFF, 0);
#else
const uint3 ChildInfo::Invalid = uint3(0xFFFFFFFF, 0x00000FFF, 0);
#endif

//=====================================================================================================================
inline uint3 ChildInfo::BuildPacked(
    uint3 quantMin,
    uint3 quantMax,
    uint  boxNodeFlags,
    uint  instanceMask,
    uint  nodeType,
    uint  nodeRangeLength)
{
    uint3 childInfo = uint3(0, 0, 0);
    childInfo.x = bitFieldInsert(childInfo.x,  0, 12, quantMin.x);
    childInfo.x = bitFieldInsert(childInfo.x, 12, 12, quantMin.y);
    childInfo.x = bitFieldInsert(childInfo.x, 24,  4, boxNodeFlags);
    childInfo.y = bitFieldInsert(childInfo.y,  0, 12, quantMin.z);
    childInfo.y = bitFieldInsert(childInfo.y, 12, 12, quantMax.x);
    childInfo.y = bitFieldInsert(childInfo.y, 24,  8, instanceMask);
    childInfo.z = bitFieldInsert(childInfo.z,  0, 12, quantMax.y);
    childInfo.z = bitFieldInsert(childInfo.z, 12, 12, quantMax.z);
    childInfo.z = bitFieldInsert(childInfo.z, 24,  4, nodeType);
    childInfo.z = bitFieldInsert(childInfo.z, 28,  4, nodeRangeLength);

    return childInfo;
}

//=====================================================================================================================
inline void ChildInfo::Load(
    uint3 packedData)
{
    minXMinYAndCullingFlags = packedData.x;
    minZMaxXAndInstanceMask = packedData.y;
    maxYMaxZNodeTypeAndNodeRange = packedData.z;
}

//=====================================================================================================================
inline void ChildInfo::Init()
{
    minXMinYAndCullingFlags = 0;
    minZMaxXAndInstanceMask = 0;
    maxYMaxZNodeTypeAndNodeRange = 0;
}

//=====================================================================================================================
inline void ChildInfo::Invalidate()
{
    minXMinYAndCullingFlags = 0xffffffff;
    minZMaxXAndInstanceMask = 0x00000fff;
    maxYMaxZNodeTypeAndNodeRange = 0;
}

//=====================================================================================================================
inline uint3 ChildInfo::Min()
{
    uint3 min;
    min.x = bitFieldExtract(minXMinYAndCullingFlags, 0, 12);
    min.y = bitFieldExtract(minXMinYAndCullingFlags, 12, 12);
    min.z = bitFieldExtract(minZMaxXAndInstanceMask, 0, 12);
    return min;
}

//=====================================================================================================================
inline uint3 ChildInfo::Max()
{
    uint3 max;
    max.x = bitFieldExtract(minZMaxXAndInstanceMask, 12, 12);
    max.y = bitFieldExtract(maxYMaxZNodeTypeAndNodeRange, 0, 12);
    max.z = bitFieldExtract(maxYMaxZNodeTypeAndNodeRange, 12, 12);
    return max;
}

//=====================================================================================================================
inline void ChildInfo::SetMin(uint3 min)
{
    minXMinYAndCullingFlags = bitFieldInsert(minXMinYAndCullingFlags, 0, 12, min.x);
    minXMinYAndCullingFlags = bitFieldInsert(minXMinYAndCullingFlags, 12, 12, min.y);
    minZMaxXAndInstanceMask = bitFieldInsert(minZMaxXAndInstanceMask, 0, 12, min.z);
}

//=====================================================================================================================
inline void ChildInfo::SetMax(uint3 max)
{
    minZMaxXAndInstanceMask = bitFieldInsert(minZMaxXAndInstanceMask, 12, 12, max.x);
    maxYMaxZNodeTypeAndNodeRange = bitFieldInsert(maxYMaxZNodeTypeAndNodeRange, 0, 12, max.y);
    maxYMaxZNodeTypeAndNodeRange = bitFieldInsert(maxYMaxZNodeTypeAndNodeRange, 12, 12, max.z);
}

//=====================================================================================================================
inline uint ChildInfo::CullingFlags()
{
    return bitFieldExtract(minXMinYAndCullingFlags, 24, 4);
}

//=====================================================================================================================
inline void ChildInfo::SetCullingFlags(uint cullingFlags)
{
    minXMinYAndCullingFlags = bitFieldInsert(minXMinYAndCullingFlags, 24, 4, cullingFlags);
}

//=====================================================================================================================
inline uint ChildInfo::InstanceMask()
{
    return bitFieldExtract(minZMaxXAndInstanceMask, 24, 8);
}

//=====================================================================================================================
inline void ChildInfo::SetInstanceMask(uint instanceMask)
{
    minZMaxXAndInstanceMask = bitFieldInsert(minZMaxXAndInstanceMask, 24, 8, instanceMask);
}

//=====================================================================================================================
inline uint ChildInfo::NodeType()
{
    return bitFieldExtract(maxYMaxZNodeTypeAndNodeRange, 24, 4);
}

//=====================================================================================================================
inline void ChildInfo::SetNodeType(uint nodeType)
{
    maxYMaxZNodeTypeAndNodeRange = bitFieldInsert(maxYMaxZNodeTypeAndNodeRange, 24, 4, nodeType);
}

//=====================================================================================================================
inline uint ChildInfo::NodeRangeLength()
{
    return bitFieldExtract(maxYMaxZNodeTypeAndNodeRange, 28, 4);
}

//=====================================================================================================================
inline void ChildInfo::SetNodeRangeLength(uint nodeBoundaries)
{
    maxYMaxZNodeTypeAndNodeRange = bitFieldInsert(maxYMaxZNodeTypeAndNodeRange, 28, 4, nodeBoundaries);
}

//=====================================================================================================================
inline bool ChildInfo::Valid()
{
    uint3 min = Min();
    uint3 max = Max();
    return !(min.y == 0xfff && min.x == 0xfff && max.y == 0 && max.z == 0);
}

//=====================================================================================================================
inline BoundingBox ChildInfo::DecodeBounds(float3 origin, uint3 exponents)
{
    uint3 qmin = Min();
    uint3 qmax = Max();
    BoundingBox result;
    result.min = float3(Dequantize(origin.x, exponents.x, qmin.x, 12),
                        Dequantize(origin.y, exponents.y, qmin.y, 12),
                        Dequantize(origin.z, exponents.z, qmin.z, 12));
    result.max = float3(Dequantize(origin.x, exponents.x, qmax.x + 1, 12),
                        Dequantize(origin.y, exponents.y, qmax.y + 1, 12),
                        Dequantize(origin.z, exponents.z, qmax.z + 1, 12));
    return result;
}

#endif
