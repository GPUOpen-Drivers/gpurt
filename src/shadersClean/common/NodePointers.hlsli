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

#ifndef NODE_POINTERS_HLSLI
#define NODE_POINTERS_HLSLI

#include "../common/TempAssert.hlsli"

//=====================================================================================================================
// Node pointer size in bytes
#define NODE_PTR_SIZE 4
GPURT_STATIC_ASSERT(NODE_PTR_SIZE == sizeof(uint32_t), "NODE_PTR_SIZE size mismatch");

//=====================================================================================================================
// Instance base pointer layout from the HW raytracing IP 2.0 spec:
// Zero                         [ 2: 0]
// Tree Base Address (64B index)[53: 3]
// Force Opaque                 [   54]
// Force Non-Opaque             [   55]
// Disable Triangle Cull        [   56]
// Flip Facedness               [   57]
// Cull Back Facing Triangles   [   58]
// Cull Front Facing Triangles  [   59]
// Cull Opaque                  [   60]
// Cull Non-Opaque              [   61]
// Skip Triangles               [   62]
// Skip Procedural              [   63]
//
// Since GPU VAs can only be 48 bits, only 42 bits of the Tree Base Address field are used:
// Used Address                 [44: 3]
// Unused Address               [53:45]
//
#define INSTANCE_BASE_POINTER_ZERO_MASK                          0x7ull
#define INSTANCE_BASE_POINTER_ADDRESS_USED_MASK       0x1FFFFFFFFFF8ull
#define INSTANCE_BASE_POINTER_ADDRESS_UNUSED_MASK   0x3FE00000000000ull
#define INSTANCE_BASE_POINTER_ADDRESS_MASK          0x3FFFFFFFFFFFF8ull
#define INSTANCE_BASE_POINTER_FLAGS_MASK          0xFFC0000000000000ull

#define NODE_POINTER_FLAGS_SHIFT                 54
#define NODE_POINTER_FORCE_OPAQUE_SHIFT          54
#define NODE_POINTER_FORCE_NON_OPAQUE_SHIFT      55
#define NODE_POINTER_DISABLE_TRIANGLE_CULL_SHIFT 56
#define NODE_POINTER_FLIP_FACEDNESS_SHIFT        57
#define NODE_POINTER_CULL_BACK_FACING_SHIFT      58
#define NODE_POINTER_CULL_FRONT_FACING_SHIFT     59
#define NODE_POINTER_CULL_OPAQUE_SHIFT           60
#define NODE_POINTER_CULL_NON_OPAQUE_SHIFT       61
#define NODE_POINTER_SKIP_TRIANGLES_SHIFT        62
#define NODE_POINTER_SKIP_PROCEDURAL_SHIFT       63

#define RAY_FLAG_VALID_MASK         0x3ffu
#define RAY_FLAG_EXCLUDE_MASK       (RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER)
#define RAY_FLAG_OVERRIDE_MASK      (RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_FORCE_NON_OPAQUE)   // 0x3
#define RAY_FLAG_PRESERVE_MASK      (RAY_FLAG_VALID_MASK & (~RAY_FLAG_OVERRIDE_MASK))     // 0x3fc

#define POINTER_FLAGS_HIDWORD_SHIFT (NODE_POINTER_FORCE_OPAQUE_SHIFT - 32)                  // 22
#define POINTER_FLAGS_VALID_MASK    (RAY_FLAG_VALID_MASK << POINTER_FLAGS_HIDWORD_SHIFT)    // 0x3ff << 22
#define POINTER_FLAGS_EXCLUDED_MASK  ~(POINTER_FLAGS_VALID_MASK)                            // 0xFFC00000

#endif
