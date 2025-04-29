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
const uint3 ChildInfo::Invalid = uint3(0xFFFFFFFF, 0x00000FFF, 0);
#else
const uint3 ChildInfo::Invalid = uint3(0xFFFFFFFF, 0x00000FFF, 0);
#endif

#ifndef LIBRARY_COMPILATION
#include "childInfo.hlsl"
#endif

#endif
