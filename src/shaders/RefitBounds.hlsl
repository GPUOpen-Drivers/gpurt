/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "../shadersClean/common/ShaderDefs.hlsli"

#define GC_DSTBUFFER
#define GC_SCRATCHBUFFER
#include "../shadersClean/build/BuildRootSignature.hlsli"

#include "IntersectCommon.hlsl"
#include "RefitBoundsImpl.hlsl"

//=====================================================================================================================
// Main Function : RefitBounds
//=====================================================================================================================
[RootSignature(RootSig)]
[numthreads(BUILD_THREADGROUP_SIZE, 1, 1)]
void RefitBounds(
    in uint globalId : SV_DispatchThreadID)
{
    const uint numActivePrims = ReadAccelStructHeaderField(ACCEL_STRUCT_HEADER_NUM_ACTIVE_PRIMS_OFFSET);
    const uint bvhNodes = CalculateBvhNodesOffset(ShaderConstants, numActivePrims);
    if (globalId < numActivePrims)
    {
        RefitBoundsImpl(globalId,
                        numActivePrims,
                        ShaderConstants.offsets.propagationFlags,
                        bvhNodes,
                        ShaderConstants.offsets.bvhLeafNodeData,
                        ShaderConstants.offsets.primIndicesSorted,
                        Settings.doTriangleSplitting,
                        Settings.enableEarlyPairCompression,
                        EnableLatePairCompression(),
                        Settings.enablePairCostCheck,
                        ShaderConstants.offsets.triangleSplitBoxes,
                        ShaderConstants.offsets.numBatches,
                        ShaderConstants.offsets.batchIndices,
                        Settings.fp16BoxNodesMode,
                        Settings.fp16BoxModeMixedSaThreshold);
    }
}
