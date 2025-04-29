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

#include "../BoundingBox.hlsli"
#include "../Bits.hlsli"
#include "../Math.hlsli"

#include "childInfo.hlsli"

//=====================================================================================================================
uint3 ChildInfo::BuildPacked(
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
void ChildInfo::Load(
    uint3 packedData)
{
    minXMinYAndCullingFlags = packedData.x;
    minZMaxXAndInstanceMask = packedData.y;
    maxYMaxZNodeTypeAndNodeRange = packedData.z;
}

//=====================================================================================================================
void ChildInfo::Init()
{
    minXMinYAndCullingFlags = 0;
    minZMaxXAndInstanceMask = 0;
    maxYMaxZNodeTypeAndNodeRange = 0;
}

//=====================================================================================================================
void ChildInfo::Invalidate()
{
    minXMinYAndCullingFlags = 0xffffffff;
    minZMaxXAndInstanceMask = 0x00000fff;
    maxYMaxZNodeTypeAndNodeRange = 0;
}

//=====================================================================================================================
uint3 ChildInfo::Min()
{
    uint3 min;
    min.x = bitFieldExtract(minXMinYAndCullingFlags, 0, 12);
    min.y = bitFieldExtract(minXMinYAndCullingFlags, 12, 12);
    min.z = bitFieldExtract(minZMaxXAndInstanceMask, 0, 12);
    return min;
}

//=====================================================================================================================
uint3 ChildInfo::Max()
{
    uint3 max;
    max.x = bitFieldExtract(minZMaxXAndInstanceMask, 12, 12);
    max.y = bitFieldExtract(maxYMaxZNodeTypeAndNodeRange, 0, 12);
    max.z = bitFieldExtract(maxYMaxZNodeTypeAndNodeRange, 12, 12);
    return max;
}

//=====================================================================================================================
void ChildInfo::SetMin(uint3 min)
{
    minXMinYAndCullingFlags = bitFieldInsert(minXMinYAndCullingFlags, 0, 12, min.x);
    minXMinYAndCullingFlags = bitFieldInsert(minXMinYAndCullingFlags, 12, 12, min.y);
    minZMaxXAndInstanceMask = bitFieldInsert(minZMaxXAndInstanceMask, 0, 12, min.z);
}

//=====================================================================================================================
void ChildInfo::SetMax(uint3 max)
{
    minZMaxXAndInstanceMask = bitFieldInsert(minZMaxXAndInstanceMask, 12, 12, max.x);
    maxYMaxZNodeTypeAndNodeRange = bitFieldInsert(maxYMaxZNodeTypeAndNodeRange, 0, 12, max.y);
    maxYMaxZNodeTypeAndNodeRange = bitFieldInsert(maxYMaxZNodeTypeAndNodeRange, 12, 12, max.z);
}

//=====================================================================================================================
uint ChildInfo::CullingFlags()
{
    return bitFieldExtract(minXMinYAndCullingFlags, 24, 4);
}

//=====================================================================================================================
void ChildInfo::SetCullingFlags(uint cullingFlags)
{
    minXMinYAndCullingFlags = bitFieldInsert(minXMinYAndCullingFlags, 24, 4, cullingFlags);
}

//=====================================================================================================================
uint ChildInfo::InstanceMask()
{
    return bitFieldExtract(minZMaxXAndInstanceMask, 24, 8);
}

//=====================================================================================================================
void ChildInfo::SetInstanceMask(uint instanceMask)
{
    minZMaxXAndInstanceMask = bitFieldInsert(minZMaxXAndInstanceMask, 24, 8, instanceMask);
}

//=====================================================================================================================
uint ChildInfo::NodeType()
{
    return bitFieldExtract(maxYMaxZNodeTypeAndNodeRange, 24, 4);
}

//=====================================================================================================================
void ChildInfo::SetNodeType(uint nodeType)
{
    maxYMaxZNodeTypeAndNodeRange = bitFieldInsert(maxYMaxZNodeTypeAndNodeRange, 24, 4, nodeType);
}

//=====================================================================================================================
uint ChildInfo::NodeRangeLength()
{
    return bitFieldExtract(maxYMaxZNodeTypeAndNodeRange, 28, 4);
}

//=====================================================================================================================
void ChildInfo::SetNodeRangeLength(uint nodeBoundaries)
{
    maxYMaxZNodeTypeAndNodeRange = bitFieldInsert(maxYMaxZNodeTypeAndNodeRange, 28, 4, nodeBoundaries);
}

//=====================================================================================================================
bool ChildInfo::Valid()
{
    uint3 min = Min();
    uint3 max = Max();
    return !(min.y == 0xfff && min.x == 0xfff && max.y == 0 && max.z == 0);
}

//=====================================================================================================================
BoundingBox ChildInfo::DecodeBounds(float3 origin, uint3 exponents)
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
