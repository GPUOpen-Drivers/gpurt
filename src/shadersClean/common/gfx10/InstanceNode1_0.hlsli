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
#ifndef INSTANCE_NODE_1_1_HLSLI
#define INSTANCE_NODE_1_1_HLSLI

#include "BoxNode1_0.hlsli"
#include "../InstanceDesc.hlsli"
#include "../../../shared/assert.h"

//=====================================================================================================================
struct InstanceSidebandData1_1
{
    uint   instanceIndex;
    uint   blasNodePointer; // might not point to root
    uint   blasMetadataSize;
    uint   padding0;
    float4 Transform[3]; // Non-inverse (original D3D12_RAYTRACING_INSTANCE_DESC.Transform)
};

#define RTIP1_1_INSTANCE_SIDEBAND_INSTANCE_INDEX_OFFSET        0
#define RTIP1_1_INSTANCE_SIDEBAND_CHILD_POINTER_OFFSET         4
#define RTIP1_1_INSTANCE_SIDEBAND_CHILD_METADATA_SIZE_OFFSET   8
#define RTIP1_1_INSTANCE_SIDEBAND_OBJECT2WORLD_OFFSET         16
#define RTIP1_1_INSTANCE_SIDEBAND_SIZE                        64

GPURT_STATIC_ASSERT(RTIP1_1_INSTANCE_SIDEBAND_SIZE == sizeof(InstanceSidebandData1_1), "Instance sideband structure mismatch");

//=====================================================================================================================
struct FusedInstanceNode
{
    InstanceDesc            desc;
    InstanceSidebandData1_1 sideband;
    Float32BoxNode          blasRootNode;
};

//=====================================================================================================================
struct InstanceNode
{
    InstanceDesc            desc;
    InstanceSidebandData1_1 sideband;
};

#define INSTANCE_NODE_DESC_OFFSET       0
#define INSTANCE_NODE_EXTRA_OFFSET      64
#define INSTANCE_NODE_SIZE              128
#define FUSED_INSTANCE_NODE_ROOT_OFFSET INSTANCE_NODE_SIZE
#define FUSED_INSTANCE_NODE_SIZE        256
GPURT_STATIC_ASSERT(INSTANCE_NODE_SIZE == sizeof(InstanceNode), "InstanceNode structure mismatch");

//=====================================================================================================================
// Get leaf instance node size in bytes
static uint GetBvhNodeSizeInstance(uint enableFusedInstanceNode);

#ifndef LIBRARY_COMPILATION
#include "InstanceNode1_0.hlsl"
#endif

#endif
