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
#include "../shared/rayTracingDefs.h"

// DebugBuffer
#if GPURT_ENABLE_GPU_DEBUG
#define str(a)  #a
#define xstr(a) str(a)

#define DEBUG_BUFFER_SLOT u8
#define DEBUG_BUFFER "UAV(" xstr(DEBUG_BUFFER_SLOT) ", visibility=SHADER_VISIBILITY_ALL),"
#else
#define DEBUG_BUFFER
#endif

// Note, CBV(b255) must be the last used binding in the root signature.
#define RootSig "RootConstants(num32BitConstants=1, b0, visibility=SHADER_VISIBILITY_ALL),"\
                "CBV(b1),"\
                "CBV(b2),"\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u6, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u7, visibility=SHADER_VISIBILITY_ALL),"\
                DEBUG_BUFFER\
                "DescriptorTable(CBV(b0, numDescriptors = 4294967295, space = 1)),"\
                "DescriptorTable(UAV(u0, numDescriptors = 4294967295, space = 1)),"\
                "CBV(b255),"\
                "DescriptorTable(UAV(u0, numDescriptors = 1, space = 2147420894)),"\

[[vk::push_constant]] ConstantBuffer<BuildShaderRootConstants> ShaderRootConstants     : register(b0);

[[vk::binding(1, 1)]] ConstantBuffer<BuildShaderConstants> ShaderConstants             : register(b1);

[[vk::binding(2, 1)]] ConstantBuffer<LutData>                      LutBuffer           : register(b2);

[[vk::binding(0, 0)]] RWByteAddressBuffer                          SrcBuffer           : register(u0);

#ifdef GC_DSTBUFFER
[[vk::binding(1, 0)]] globallycoherent RWByteAddressBuffer         DstBuffer           : register(u1);
#else
[[vk::binding(1, 0)]] RWByteAddressBuffer                          DstBuffer           : register(u1);
#endif

#ifdef GC_DSTMETADATA
[[vk::binding(2, 0)]] globallycoherent RWByteAddressBuffer         DstMetadata         : register(u2);
#else
[[vk::binding(2, 0)]] RWByteAddressBuffer                          DstMetadata         : register(u2);
#endif

#ifdef GC_SCRATCHBUFFER
[[vk::binding(3, 0)]] globallycoherent RWByteAddressBuffer         ScratchBuffer       : register(u3);
[[vk::binding(4, 0)]] globallycoherent RWByteAddressBuffer         ScratchGlobal       : register(u4);
#else
[[vk::binding(3, 0)]] RWByteAddressBuffer                          ScratchBuffer       : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer                          ScratchGlobal       : register(u4);
#endif

[[vk::binding(5, 0)]] RWByteAddressBuffer                          InstanceDescBuffer  : register(u5);
[[vk::binding(6, 0)]] RWByteAddressBuffer                          EmitBuffer          : register(u6);
[[vk::binding(7, 0)]] RWByteAddressBuffer                          IndirectArgBuffer   : register(u7);

// Debug Buffer

[[vk::binding(0, 3)]] ConstantBuffer<BuildShaderGeometryConstants> GeometryConstants[] : register(b0, space1);
[[vk::binding(0, 4)]] RWBuffer<float3>                             GeometryBuffer[]    : register(u0, space1);
