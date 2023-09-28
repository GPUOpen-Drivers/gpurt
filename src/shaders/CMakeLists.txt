##
 #######################################################################################################################
 #
 #  Copyright (c) 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
 #
 #  Permission is hereby granted, free of charge, to any person obtaining a copy
 #  of this software and associated documentation files (the "Software"), to deal
 #  in the Software without restriction, including without limitation the rights
 #  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 #  copies of the Software, and to permit persons to whom the Software is
 #  furnished to do so, subject to the following conditions:
 #
 #  The above copyright notice and this permission notice shall be included in all
 #  copies or substantial portions of the Software.
 #
 #  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 #  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 #  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 #  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 #  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 #  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 #  SOFTWARE.
 #
 #######################################################################################################################

target_include_directories(gpurt PRIVATE .)

set(gpurtHlsl
    RadixSort/BitHistogram.hlsl
    RadixSort/DistributePartSumInt4.hlsl
    RadixSort/RadixSortParallel.hlsl
    RadixSort/ScanCommon.hlsli
    RadixSort/ScanExclusiveInt4.hlsl
    RadixSort/ScanExclusiveInt4DLB.hlsl
    RadixSort/ScanExclusiveInt4DLBCommon.hlsl
    RadixSort/ScanExclusivePartInt4.hlsl
    RadixSort/ScatterKeysAndValues.hlsl
    AccelStructTracker.hlsl
    BuildBVH.hlsl
    BuildBVHPLOC.hlsl
    BuildBVHTDTR.hlsl
    BuildCommon.hlsl
    BuildCommonScratch.hlsl
    Debug.hlsl
    TaskQueueCounter.hlsl
    BuildSettings.hlsli
    BuildParallel.hlsl
    BuildQBVH.hlsl
#if GPURT_BUILD_RTIP2
    bvh_stack_dxr_prototype.hlsl
#endif
    ClearBuffer.hlsl
    Common.hlsl
    CompactAS.hlsl
    CompactCommon.hlsl
    CopyAS.hlsl
    CopyBufferRaw.hlsl
    DecodeAS.hlsl
    DecodeCommon.hlsl
    DeserializeAS.hlsl
    EmitAS.hlsl
    EncodeCommon.hlsl
    EncodeNodes.hlsl
    EncodePairedTriangle.hlsl
    EncodePairedTriangleImpl.hlsl
    EncodeTopLevel.hlsl
    Extensions.hlsl
    GenerateMortonCodes.hlsl
    GpuRtLibrary.hlsl
    InitAccelerationStructure.hlsl
    InitExecuteIndirect.hlsl
    IntersectCommon.hlsl
    MergeSort.hlsl
    MortonCodes.hlsl
    PairCompression.hlsl
    RayQuery.hlsl
    RayQuery1_1.hlsl
#if GPURT_BUILD_RTIP2
    RayQuery2_0.hlsl
#endif
    Rebraid.hlsl
    RefitBounds.hlsl
    RefitBoundsImpl.hlsl
    SerializeAS.hlsl
    SerializeCommon.hlsl
    TraceRay.hlsl
    TraceRay1_1.hlsl
#if GPURT_BUILD_RTIP2
    TraceRay2_0.hlsl
#endif
    TraceRayCommon.hlsl
    TriangleSplitting.hlsl
    Update.hlsl
    UpdateParallel.hlsl
    UpdateQBVH.hlsl
    UpdateQBVHImpl.hlsl
)
set(otherDeps
    ../../gpurt/gpurtAccelStruct.h
    ../../gpurt/gpurtBuildSettings.h
    ../../gpurt/gpurtCounter.h
    ../../gpurt/gpurtDispatch.h
    ../shared/accelStruct.h
    ../shared/debugBuffer.h
    ../shared/gpurtBuildConstants.h
    ../shared/math.h
    ../shared/rayTracingDefs.h
    ../shared/scratchNode.h
)

set(GPURT_SHADER_SOURCE_FILES "${gpurtHlsl}" "${otherDeps}" PARENT_SCOPE)

list(TRANSFORM gpurtHlsl PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

