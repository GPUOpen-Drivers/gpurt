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
#include "Common.hlsl"
#include "../../gpurt/gpurtDispatch.h"

#define RootSig "CBV(b0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u0, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u1, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u2, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u3, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u4, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u5, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u6, visibility=SHADER_VISIBILITY_ALL),"\
                "UAV(u0, space=2147420894, visibility=SHADER_VISIBILITY_ALL)"

#if AMD_VULKAN
#define DISPATCH_DIM_SWIZZLE_MODE_NATIVE                0
#define DISPATCH_DIM_SWIZZLE_MODE_FLATTEN_WIDTH_HEIGHT  1
#endif

#if AMD_VULKAN_GLSLANG
namespace ExecuteIndirectArgType
{
    static const uint DispatchDimensions = 0x0;
    static const uint DispatchDimenionsAndShaderTable = 0x1;
}
#else
enum ExecuteIndirectArgType : uint
{
    DispatchDimensions              = 0x0,
    DispatchDimenionsAndShaderTable = 0x1
};
#endif

#define MAX_SUPPORTED_INDIRECT_COUNTERS 8
#define MAX_SUPPORTED_BUFFER_SRD_DWORDS 4

struct AddressRange
{
    uint64_t address;
    uint64_t stride;
};

struct AddressRangeAndStride
{
    AddressRange range;
    uint64_t     stride;
};

#if AMD_VULKAN
// DispatchRays() indirect argument. This is the same struct passed to normal DispatchRays().
struct DispatchRaysDimensions
{
    uint width;
    uint height;
    uint depth;
};
#endif

// DispatchRays() indirect argument. This is the same struct passed to normal DispatchRays().
struct DispatchRaysDesc
{
    AddressRange          rayGenShaderTable;
    AddressRangeAndStride missShaderTable;
    AddressRangeAndStride hitGroupTable;
    AddressRangeAndStride callableShaderTable;

    uint width;
    uint height;
    uint depth;
};

struct DispatchRaysDescriptorTable
{
    uint64_t staticConstants;
    uint32_t internalUavSrd0[MAX_SUPPORTED_BUFFER_SRD_DWORDS];
    uint32_t accelStructTrackerSrd[MAX_SUPPORTED_BUFFER_SRD_DWORDS];
};

struct IndirectCounterMetadata
{
    uint32_t rayGenShaderIdLo;
    uint32_t rayGenShaderIdHi;
    uint32_t dispatchDimensionX;
    uint32_t dispatchDimensionY;
    uint32_t dispatchDimensionZ;
};

[[vk::binding(0, 1)]] ConstantBuffer<InitExecuteIndirectConstants> Constants : register(b0);

[[vk::binding(0, 0)]] RWStructuredBuffer<uint> DispatchCount   : register(u0);
[[vk::binding(1, 0)]] RWStructuredBuffer<uint> RayGenIds       : register(u1);
[[vk::binding(2, 0)]] RWByteAddressBuffer      InputArgBuffer  : register(u2);
[[vk::binding(3, 0)]] RWByteAddressBuffer      OutputArgBuffer : register(u3);

[[vk::binding(4, 0)]] RWStructuredBuffer<DispatchRaysConstantData>    OutputConstants       : register(u4);
[[vk::binding(5, 0)]] RWStructuredBuffer<DispatchRaysDescriptorTable> OutputDescriptorTable : register(u5);
[[vk::binding(6, 0)]] RWStructuredBuffer<IndirectCounterMetadata>     CounterMetadata       : register(u6);

uint CalcDispatchDim(uint threadCount, uint tgSize)
{
    return ((threadCount + (tgSize - 1)) / tgSize);
}

#if AMD_VULKAN
[RootSignature(RootSig)]
[numthreads(8, 8, 1)]
void InitExecuteIndirect(
    uint3 threadId : SV_DispatchThreadId)
{
    const uint dispatchIdx = threadId.x; // Dispatch index: 0 to the app's MaxCount
    const uint pipelineIdx = threadId.y; // Pipeline index: 0 to the number of compiled pipelines to dispatch

    // Only 1 dispatch is supported
    if ((dispatchIdx == 0) &&
        (pipelineIdx == 0))
    {
        DispatchRaysDimensions dispatchRaysDescDim;

        if (Constants.indirectMode == DispatchDimensions)
        {
            // vkCmdTraceRaysIndirectKHR - ray trace query dimensions
            const DispatchRaysDimensions dispatchRaysDesc = InputArgBuffer.Load<DispatchRaysDimensions>(0);

            dispatchRaysDescDim = dispatchRaysDesc;

            // Note there is one internal constant buffer per app dispatch
            OutputConstants[dispatchIdx].rayDispatchWidth  = dispatchRaysDesc.width;
            OutputConstants[dispatchIdx].rayDispatchHeight = dispatchRaysDesc.height;
            OutputConstants[dispatchIdx].rayDispatchDepth  = dispatchRaysDesc.depth;
        }
        else
        {
            // vkCmdTraceRaysIndirect2KHR- shaderTable + ray trace query dimensions
            const DispatchRaysDesc dispatchRaysDesc = InputArgBuffer.Load<DispatchRaysDesc>(0);

            dispatchRaysDescDim.width  = dispatchRaysDesc.width;
            dispatchRaysDescDim.height = dispatchRaysDesc.height;
            dispatchRaysDescDim.depth  = dispatchRaysDesc.depth;

            // Note there is one internal constant buffer per app dispatch
            // Save dispatch dims
            OutputConstants[dispatchIdx].rayDispatchWidth  = dispatchRaysDesc.width;
            OutputConstants[dispatchIdx].rayDispatchHeight = dispatchRaysDesc.height;
            OutputConstants[dispatchIdx].rayDispatchDepth  = dispatchRaysDesc.depth;

            // Save shader table
            OutputConstants[dispatchIdx].rayGenerationTableAddressLo = LowPart(dispatchRaysDesc.rayGenShaderTable.address);
            OutputConstants[dispatchIdx].rayGenerationTableAddressHi = HighPart(dispatchRaysDesc.rayGenShaderTable.address);
            OutputConstants[dispatchIdx].missTableBaseAddressLo      = LowPart(dispatchRaysDesc.missShaderTable.range.address);
            OutputConstants[dispatchIdx].missTableBaseAddressHi      = HighPart(dispatchRaysDesc.missShaderTable.range.address);
            OutputConstants[dispatchIdx].missTableStrideInBytes      = uint(dispatchRaysDesc.missShaderTable.stride);
            OutputConstants[dispatchIdx].hitGroupTableBaseAddressLo  = LowPart(dispatchRaysDesc.hitGroupTable.range.address);
            OutputConstants[dispatchIdx].hitGroupTableBaseAddressHi  = HighPart(dispatchRaysDesc.hitGroupTable.range.address);
            OutputConstants[dispatchIdx].hitGroupTableStrideInBytes  = uint(dispatchRaysDesc.hitGroupTable.stride);
            OutputConstants[dispatchIdx].callableTableBaseAddressLo  = LowPart(dispatchRaysDesc.callableShaderTable.range.address);
            OutputConstants[dispatchIdx].callableTableBaseAddressHi  = HighPart(dispatchRaysDesc.callableShaderTable.range.address);
            OutputConstants[dispatchIdx].callableTableStrideInBytes  = uint(dispatchRaysDesc.callableShaderTable.stride);
        }

        uint outputOffset = 0;
        uint3 dispatchDim = uint3(0, 0, 0);

        switch (Constants.dispatchDimSwizzleMode)
        {
        case DISPATCH_DIM_SWIZZLE_MODE_NATIVE:
            dispatchDim.x = CalcDispatchDim(dispatchRaysDescDim.width,  Constants.rtThreadGroupSizeX);
            dispatchDim.y = CalcDispatchDim(dispatchRaysDescDim.height, Constants.rtThreadGroupSizeY);
            dispatchDim.z = CalcDispatchDim(dispatchRaysDescDim.depth,  Constants.rtThreadGroupSizeZ);
            break;
        case DISPATCH_DIM_SWIZZLE_MODE_FLATTEN_WIDTH_HEIGHT:
            {
                uint dispatchSize = 0;
                if ((dispatchRaysDescDim.width > 1) && (dispatchRaysDescDim.height > 1))
                {
                    const uint paddedWidth  = Pow2Align(dispatchRaysDescDim.width,  8);
                    const uint paddedHeight = Pow2Align(dispatchRaysDescDim.height, Constants.rtThreadGroupSizeX / 8);
                    dispatchSize = paddedWidth * paddedHeight;
                }
                else
                {
                    dispatchSize = dispatchRaysDescDim.width * dispatchRaysDescDim.height;
                }
                dispatchDim.x = CalcDispatchDim(dispatchSize,  Constants.rtThreadGroupSizeY);
                dispatchDim.y = dispatchRaysDescDim.depth;
                dispatchDim.z = 1;
            }
            break;
        default:
            break;
        }

        // Indirect pipelines only need one dispatch
        OutputArgBuffer.Store3(outputOffset, dispatchDim);
    }
}
#else
[RootSignature(RootSig)]
[numthreads(8, 8, 1)]
void InitExecuteIndirect(
    uint3 threadId : SV_DispatchThreadId)
{
    const uint dispatchIdx = threadId.x; // Dispatch index: 0 to the app's MaxCount
    const uint pipelineIdx = threadId.y; // Pipeline index: 0 to the number of compiled pipelines to dispatch

    if ((dispatchIdx < Constants.maxDispatchCount) &&
        (pipelineIdx < Constants.pipelineCount))
    {
        // The DispatchRays indirect argument struct follows any resource bindings
        const uint dispatchRaysDescOffset = (dispatchIdx * Constants.inputBytesPerDispatch) + Constants.bindingArgsSize;

        const DispatchRaysDesc dispatchRaysDesc = InputArgBuffer.Load<DispatchRaysDesc>(dispatchRaysDescOffset);

        // Note there is one internal constant buffer per app dispatch
        if (pipelineIdx == 0)
        {
            DispatchRaysConstantData info = (DispatchRaysConstantData)0;
            info.rayDispatchWidth            = dispatchRaysDesc.width;
            info.rayDispatchHeight           = dispatchRaysDesc.height;
            info.rayDispatchDepth            = dispatchRaysDesc.depth;
            info.rayGenerationTableAddressLo = LowPart(dispatchRaysDesc.rayGenShaderTable.address);
            info.rayGenerationTableAddressHi = HighPart(dispatchRaysDesc.rayGenShaderTable.address);
            info.missTableBaseAddressLo      = LowPart(dispatchRaysDesc.missShaderTable.range.address);
            info.missTableBaseAddressHi      = HighPart(dispatchRaysDesc.missShaderTable.range.address);
            info.missTableStrideInBytes      = uint(dispatchRaysDesc.missShaderTable.stride);
            info.hitGroupTableBaseAddressLo  = LowPart(dispatchRaysDesc.hitGroupTable.range.address);
            info.hitGroupTableBaseAddressHi  = HighPart(dispatchRaysDesc.hitGroupTable.range.address);
            info.hitGroupTableStrideInBytes  = uint(dispatchRaysDesc.hitGroupTable.stride);
            info.callableTableBaseAddressLo  = LowPart(dispatchRaysDesc.callableShaderTable.range.address);
            info.callableTableBaseAddressHi  = HighPart(dispatchRaysDesc.callableShaderTable.range.address);
            info.callableTableStrideInBytes  = uint(dispatchRaysDesc.callableShaderTable.stride);
            info.profileRayFlags             = Constants.profileRayFlags;
            info.profileMaxIterations        = Constants.maxIterations;
            info.traceRayGpuVaLo             = Constants.traceRayFunctionAddrLo;
            info.traceRayGpuVaHi             = Constants.traceRayFunctionAddrHi;

            info.counterMode                 = Constants.counterMode;
            info.counterRayIdRangeBegin      = Constants.counterRayIdRangeBegin;
            info.counterRayIdRangeEnd        = Constants.counterRayIdRangeEnd;
            info.counterMask                 = Constants.counterMask;

            OutputConstants[dispatchIdx] = info;

            // Setup constant buffer pointing to the constants filled in above
            GpuVirtualAddress constantsVa = MakeGpuVirtualAddress(Constants.outputConstantsVaLo, Constants.outputConstantsVaHi);
            constantsVa += dispatchIdx * sizeof(DispatchRaysConstantData);

            DispatchRaysDescriptorTable table = (DispatchRaysDescriptorTable)0;
            table.staticConstants = constantsVa;

            // Bind internal UAV for counters etc.
            if ((Constants.counterMode != 0) && (dispatchIdx < MAX_SUPPORTED_INDIRECT_COUNTERS))
            {
                table.internalUavSrd0[0] = Constants.internalUavSrd[dispatchIdx][0];
                table.internalUavSrd0[1] = Constants.internalUavSrd[dispatchIdx][1];
                table.internalUavSrd0[2] = Constants.internalUavSrd[dispatchIdx][2];
                table.internalUavSrd0[3] = Constants.internalUavSrd[dispatchIdx][3];

                // Write counter metadata
                const uint2 rayGenShaderId = LoadDwordAtAddrx2(dispatchRaysDesc.rayGenShaderTable.address);
                CounterMetadata[dispatchIdx].rayGenShaderIdLo   = rayGenShaderId.x;
                CounterMetadata[dispatchIdx].rayGenShaderIdHi   = rayGenShaderId.y;
                CounterMetadata[dispatchIdx].dispatchDimensionX = dispatchRaysDesc.width;
                CounterMetadata[dispatchIdx].dispatchDimensionY = dispatchRaysDesc.height;
                CounterMetadata[dispatchIdx].dispatchDimensionZ = dispatchRaysDesc.depth;
            }

            table.accelStructTrackerSrd[0] = Constants.accelStructTrackerSrd[0];
            table.accelStructTrackerSrd[1] = Constants.accelStructTrackerSrd[1];
            table.accelStructTrackerSrd[2] = Constants.accelStructTrackerSrd[2];
            table.accelStructTrackerSrd[3] = Constants.accelStructTrackerSrd[3];

            OutputDescriptorTable[dispatchIdx] = table;
        }

        uint inputOffset  = dispatchIdx * Constants.inputBytesPerDispatch;
        uint outputOffset = (pipelineIdx * Constants.maxDispatchCount + dispatchIdx) * Constants.outputBytesPerDispatch;

        // Directly copy all indirect binding args from the app buffer to our temp internal buffer
        for (uint i = 0; i < Constants.bindingArgsSize; i += sizeof(uint))
        {
            const uint data = InputArgBuffer.Load(inputOffset);
            OutputArgBuffer.Store(outputOffset, data);
            outputOffset += sizeof(uint);
            inputOffset  += sizeof(uint);
        }

        // The indirect command generator path has an extra internal binding argument. This is a single dword
        // descriptor table pointer.
        if (Constants.bindingArgsSize > 0)
        {
            const uint internalTableSize    = sizeof(DispatchRaysDescriptorTable);
            const uint perDispatchTableVaLo = Constants.internalTableVaLo + dispatchIdx * internalTableSize;

            OutputArgBuffer.Store(outputOffset, perDispatchTableVaLo);
            outputOffset += sizeof(uint);
        }

        uint3 dispatchDim = uint3(0, 0, 0);

        // Skip dispatches between the actual count and max count
        if (dispatchIdx < DispatchCount[0])
        {
            bool skipDispatch = false;

            // Output the indirect dispatch size for each pipeline we will dispatch
            if (Constants.pipelineCount > 1)
            {
                const uint rayGenIdentifier = LoadDwordAtAddr(dispatchRaysDesc.rayGenShaderTable.address);

                // Setup indirect dispatch arguments for all raygen shaders in the pipeline. Only the raygen with the
                // matching ID will be launched.
                if (RayGenIds[pipelineIdx] != rayGenIdentifier)
                {
                    skipDispatch = true;
                }
            }

            if (skipDispatch == false)
            {
                uint threadCountX;

                // Pad out 2D dispatches to wave tiles sized with 8 rays in a row. Thread group size determines the number of rows.
                const bool is2D = ((dispatchRaysDesc.width > 1) && (dispatchRaysDesc.height > 1));
                if (is2D)
                {
                    const uint paddedWidth  = Pow2Align(dispatchRaysDesc.width,  8);
                    const uint paddedHeight = Pow2Align(dispatchRaysDesc.height, Constants.rtThreadGroupSize / 8);

                    threadCountX = paddedWidth * paddedHeight;
                }
                else
                {
                    // Note height could potentially be zero here
                    threadCountX = dispatchRaysDesc.width * dispatchRaysDesc.height;
                }

                dispatchDim.x = CalcDispatchDim(threadCountX, Constants.rtThreadGroupSize);
                dispatchDim.y = dispatchRaysDesc.depth;
                dispatchDim.z = 1;
            }
        }

        OutputArgBuffer.Store3(outputOffset, dispatchDim);
    }
}
#endif
