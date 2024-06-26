##
 #######################################################################################################################
 #
 #  Copyright (c) 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

target_include_directories(gpurt_internal PRIVATE .)

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
    IndirectArgBufferUtils.hlsl
    AccelStructTracker.hlsl
    BuildBVH.hlsl
    BuildBVHPLOC.hlsl
    BuildBVHTDTR.hlsl
    BuildCommon.hlsl
    BuildCommonScratch.hlsl
    BuildCommonScratchGlobal.hlsl
    BuildFastAgglomerativeLbvh.hlsl
    Debug.hlsl
    TaskCounter.hlsl
    TaskMacros.hlsl
    TaskQueueCounter.hlsl
    EncodeHwBvhCommon.hlsl
    BuildSettings.hlsli
    BuildParallel.hlsl
    BuildQBVH.hlsl
    bvh_stack_dxr_prototype.hlsl
    ClearBuffer.hlsl
    Common.hlsl
    CompactAS.hlsl
    CompactAS1_1.hlsl
    CompactCommon.hlsl
    CopyAS.hlsl
    CopyBufferRaw.hlsl
    DecodeAS.hlsl
    DecodeCommon.hlsl
    DeserializeAS.hlsl
    EmitAS.hlsl
    EncodeCommon.hlsl
    EncodeNodes.hlsl
    EncodePairedTriangleImpl.hlsl
    EncodeTopLevel.hlsl
    EncodeTopLevelBuild.hlsl
    EncodeTopLevelCommon.hlsl
    EncodeTopLevelUpdate.hlsl
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
    RayQuery2_0.hlsl
    Rebraid.hlsl
    RefitBounds.hlsl
    RefitBoundsImpl.hlsl
    SerializeAS.hlsl
    SerializeCommon.hlsl
    TraceRay.hlsl
    TraceRay1_1.hlsl
    TraceRay2_0.hlsl
    TraceRayCommon.hlsl
    TriangleSplitting.hlsl
    TrianglePrimitive.hlsl
    Update.hlsl
    UpdateParallel.hlsl
    UpdateQBVH.hlsl
    UpdateQBVH1_1.hlsl
    UpdateQBVHImpl.hlsl
#if GPURT_BUILD_CONTINUATION
    GpuRtLibraryCont.hlsl
    Continuations1_1.hlsl
    Continuations2_0.hlsl
#endif
)
set(otherDeps
    ../../gpurt/gpurtAccelStruct.h
    ../../gpurt/gpurtBuildSettings.h
    ../../gpurt/gpurtCounter.h
    ../../gpurt/gpurtDispatch.h
    ../../gpurt/gpurtTraceSettings.h
    ../shared/accelStruct.h
    ../shared/debugBuffer.h
    ../shared/gpurtBuildConstants.h
    ../shared/hlslTypes.h
    ../shared/math.h
    ../shared/rayTracingDefs.h
    ../shared/scratchNode.h
)

set(GPURT_SHADER_SOURCE_FILES "${gpurtHlsl}" "${otherDeps}" PARENT_SCOPE)

list(TRANSFORM gpurtHlsl PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

