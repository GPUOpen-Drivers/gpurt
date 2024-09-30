/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef INSTANCE_DESC_HLSLI
#define INSTANCE_DESC_HLSLI

#include "TempAssert.hlsli"

//=====================================================================================================================
// 64-byte aligned structure matching D3D12_RAYTRACING_INSTANCE_DESC
struct InstanceDesc
{
    float4 Transform[3];                                    // Inverse transform for traversal
    uint   InstanceID_and_Mask;                             // 24-bit instance ID and 8-bit mask
    uint   InstanceContributionToHitGroupIndex_and_Flags;   // 24-bit instance contribution and 8-bit flags
    uint   accelStructureAddressLo;                         // Lower part of acceleration structure base address
    uint   accelStructureAddressHiAndFlags;                 // Upper part of acceleration structure base address and
                                                            // HW raytracing IP 2.0 flags
};

#define INSTANCE_DESC_SIZE                          64
#define INSTANCE_DESC_WORLD_TO_OBJECT_XFORM_OFFSET   0
#define INSTANCE_DESC_ID_AND_MASK_OFFSET            48
#define INSTANCE_DESC_CONTRIBUTION_AND_FLAGS_OFFSET 52
#define INSTANCE_DESC_VA_LO_OFFSET                  56
#define INSTANCE_DESC_VA_HI_OFFSET                  60

GPURT_STATIC_ASSERT(INSTANCE_DESC_SIZE == sizeof(InstanceDesc), "InstanceDesc structure mismatch");

#endif
