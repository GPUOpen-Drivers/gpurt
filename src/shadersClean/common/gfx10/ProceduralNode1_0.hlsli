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
#ifndef PROCEDURAL_NODE_1_1_HLSLI
#define PROCEDURAL_NODE_1_1_HLSLI

#include "../../../shared/assert.h"

//=====================================================================================================================
#define USER_NODE_PROCEDURAL_MIN_OFFSET 0
#define USER_NODE_PROCEDURAL_MAX_OFFSET 12
#define USER_NODE_PROCEDURAL_SIZE       64

//=====================================================================================================================
// Procedural node primitive data offsets
#if GPURT_BUILD_RTIP3
#define RTIP3_USER_NODE_PROCEDURAL_PRIMITIVE_INDEX_OFFSET      RTIP3_TRIANGLE_NODE_PRIMITIVE_INDEX0_OFFSET
#define RTIP3_USER_NODE_PROCEDURAL_GEOMETRY_INDEX_OFFSET       RTIP3_TRIANGLE_NODE_GEOMETRY_INDEX_OFFSET
#endif
#define USER_NODE_PROCEDURAL_PRIMITIVE_INDEX_OFFSET            TRIANGLE_NODE_PRIMITIVE_INDEX1_OFFSET
#define USER_NODE_PROCEDURAL_GEOMETRY_INDEX_AND_FLAGS_OFFSET   TRIANGLE_NODE_GEOMETRY_INDEX_AND_FLAGS_OFFSET
#define USER_NODE_PROCEDURAL_TRIANGLE_ID_OFFSET                TRIANGLE_NODE_ID_OFFSET

//=====================================================================================================================
// User defined procedural node format
struct ProceduralNode
{
    float3 bbox_min;
    float3 bbox_max;
    uint   padding1[6];
    uint   geometryIndexAndFlags;
    uint   reserved;
    uint   primitiveIndex;
    uint   triangleId;
};

GPURT_STATIC_ASSERT(USER_NODE_PROCEDURAL_SIZE == sizeof(ProceduralNode), "ProceduralNode structure mismatch");

#if GPURT_BUILD_RTIP3
//=====================================================================================================================
// User defined procedural node format
struct ProceduralNode3_0
{
    float3 bbox_min;
    float3 bbox_max;
    uint   padding1[6];
    uint   primitiveIndex;
    uint   reserved;
    uint   geometryIndex;
    uint   triangleId;
};

GPURT_STATIC_ASSERT(USER_NODE_PROCEDURAL_SIZE == sizeof(ProceduralNode3_0), "ProceduralNode structure mismatch");
#endif

#endif
