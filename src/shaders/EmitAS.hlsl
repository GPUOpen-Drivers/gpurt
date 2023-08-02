/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
#define RootSig "RootConstants(num32BitConstants=2, b0, visibility=SHADER_VISIBILITY_ALL), "\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL)"

//=====================================================================================================================
// 32 bit constants
struct InputArgs
{
    uint offset;
    uint fp16BoxNodesInBlasMode;
};

[[vk::push_constant]] ConstantBuffer<InputArgs> ShaderConstants : register(b0);

//=====================================================================================================================
[[vk::binding(0, 0)]] RWByteAddressBuffer DstBuffer   : register(u0);
[[vk::binding(1, 0)]] RWByteAddressBuffer SrcBuffer   : register(u1);

// unused buffer
[[vk::binding(2, 0)]] RWByteAddressBuffer EmitBuffer     : register(u2);
[[vk::binding(3, 0)]] RWByteAddressBuffer ScratchBuffer  : register(u3);
[[vk::binding(4, 0)]] RWByteAddressBuffer DstMetadata    : register(u4);

// The emit path uses DstBuffer as the true acceleration structure base (same as DstMetdata).
#define DstMetadata DstBuffer
#include "BuildCommon.hlsl"
#undef DstMetadata

#include "Common.hlsl"
#include "CompactCommon.hlsl"
#include "DecodeCommon.hlsl"
#include "SerializeCommon.hlsl"

//=====================================================================================================================
AccelStructHeader FetchAccelStructHeader()
{
    // Fetch acceleration structure metadata size
    const uint32_t metadataSizeInBytes = SrcBuffer.Load<uint32_t>(ACCEL_STRUCT_METADATA_SIZE_OFFSET);

    // Acceleration structure header follows metadata
    return SrcBuffer.Load<AccelStructHeader>(metadataSizeInBytes);
}

//=====================================================================================================================
// EmitCurrentSize
// output matches  D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_COMPACTED_SIZE_DESC
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(1, 1, 1)]
void EmitCurrentSize(in uint3 globalThreadId : SV_DispatchThreadID)
{
    // Load acceleration structure header
    const AccelStructHeader header = FetchAccelStructHeader();

    // Store size in destination buffer
    DstBuffer.Store2(ShaderConstants.offset, uint2(header.sizeInBytes, 0));
}

//=====================================================================================================================
// EmitCompactSize
// output matches  D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_COMPACTED_SIZE_DESC
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(1, 1, 1)]
void EmitCompactSize(in uint3 globalThreadId : SV_DispatchThreadID)
{
    // Load acceleration structure header
    const AccelStructHeader header = FetchAccelStructHeader();

    DstBuffer.Store2(ShaderConstants.offset, uint2(header.compactedSizeInBytes, 0));
}

//=====================================================================================================================
// EmitSerializeDesc
// output matches D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_SERIALIZATION_DESC
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(1, 1, 1)]
void EmitSerializeDesc(in uint3 globalThreadId : SV_DispatchThreadID)
{
    // Load acceleration structure header
    const AccelStructHeader header = FetchAccelStructHeader();

    const uint type = (header.info & ACCEL_STRUCT_HEADER_INFO_TYPE_MASK);

    const uint numBlasPointers = (type == TOP_LEVEL ? header.numDescs : 0);

    const uint serializedSizeInBytes = SERIALIZED_AS_HEADER_SIZE + header.sizeInBytes + GPUVA_SIZE * numBlasPointers;

    // SerializedSizeInBytes
    DstBuffer.Store2(ShaderConstants.offset, uint2(serializedSizeInBytes, 0));

    // NumBottomLevelAccelerationStructurePointers
    DstBuffer.Store2(ShaderConstants.offset + 8, uint2(numBlasPointers, 0));
}

//=====================================================================================================================
// EmitToolVisDesc
// output matches D3D12_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_TOOLS_VISUALIZATION_DESC
// Not performance optimal, should be used for debug only
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(1, 1, 1)]
void EmitToolVisDesc(in uint3 globalThreadId : SV_DispatchThreadID)
{
    // Load acceleration structure header
    const AccelStructHeader header = FetchAccelStructHeader();

    const uint type = (header.info & ACCEL_STRUCT_HEADER_INFO_TYPE_MASK);

    uint decodedSizeInBytes = 0;

    if (type == TOP_LEVEL)
    {
        // Visualization Header + D3D12_RAYTRACING_INSTANCE_DESC * numDescs
        decodedSizeInBytes = VISUALIZATION_HEADER_SIZE + (INSTANCE_DESC_SIZE * header.numDescs);

    }
    else
    {
        // Visualization Header + (D3D12_RAYTRACING_GEOMETRY_DESC * numDescs) + vertices/AABBs
        decodedSizeInBytes = VISUALIZATION_HEADER_SIZE              +
                             (GEOMETRY_DESC_SIZE * header.numDescs) +
                             (header.numPrimitives * 36);

        ///@todo The per-primitive size reported above is an overestimate i.e. maxsize(triangle, AABB).
        ///      However, according to the DXR spec, a bottom level acceleration structure can only contain
        ///      one type of geometry (triangle or AABB). Check the geometryType and use the appropriate
        ///      primitive stride
    }

    // Additional logging for driver decode operations. Note, in order to avoid adding a additional Emit flag, we're
    // simply padding the decoded size since it is a driver defined size.
    decodedSizeInBytes += GetDriverDecodeHeaderSize();

    // DecodedSizeInBytes
    DstBuffer.Store2(ShaderConstants.offset, uint2(decodedSizeInBytes, 0));
}
