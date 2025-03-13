##
 #######################################################################################################################
 #
 #  Copyright (c) 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
    ../shadersClean/build/BuildRootSignature.hlsli
    BuildPLOC.hlsl
    BuildCommon.hlsl
    BuildCommonScratch.hlsl
    BuildCommonScratchGlobal.hlsl
    BuildFastAgglomerativeLbvh.hlsl
    ../shadersClean/debug/Debug.hlsli
    TaskCounter.hlsl
    TaskMacros.hlsl
    TaskQueueCounter.hlsl
    EncodeHwBvhCommon.hlsl
#if GPURT_BUILD_RTIP3_1
    EncodeHwBvh3_1.hlsl
    QBVH8Common.hlsl
    ../shadersClean/common/LaneGroup.hlsli
#endif
    ../shadersClean/build/BuildSettings.hlsli
    BuildParallel.hlsl
#if GPURT_BUILD_RTIP3_1
    BuildTrivialBvh.hlsl
    BuildSingleThreadGroup.hlsl
    SingleThreadGroupBuild/GroupShared.hlsl
    SingleThreadGroupBuild/ScratchBuffer.hlsl
    SingleThreadGroupBuild/MergeSortLocal.hlsl
    SingleThreadGroupBuild/LBVH.hlsl
    SingleThreadGroupBuild/PLOC.hlsl
    SingleThreadGroupBuild/EncodeHwBvhCounters.hlsl
    SingleThreadGroupBuild/EncodeHwBvh3_1_SingleTG.hlsl
    SingleThreadGroupBuild/PrimCompress3_1_SingleTG.hlsl
#endif
    BuildQBVH.hlsl
    bvh_stack_dxr_prototype.hlsl
    ../shadersClean/debug/ClearBuffer.hlsl
    ../shadersClean/common/Common.hlsli
    CompactAS.hlsl
    CompactAS1_1.hlsl
#if GPURT_BUILD_RTIP3_1
    CompactAS3_1.hlsl
#endif
    CompactCommon.hlsl
    ../shadersClean/build/CopyAS.hlsl
    ../shadersClean/build/CopyBufferRaw.hlsl
    DecodeAS.hlsl
    DecodeCommon.hlsl
    DeserializeAS.hlsl
    ../shadersClean/build/EmitAS.hlsl
    EncodeCommon.hlsl
    EncodeNodes.hlsl
    EncodePairedTriangleImpl.hlsl
    EncodeTopLevel.hlsl
    EncodeTopLevelBuild.hlsl
    EncodeTopLevelCommon.hlsl
    EncodeTopLevelUpdate.hlsl
    GenerateMortonCodes.hlsl
    GpuRtLibrary.hlsl
#if GPURT_BUILD_RTIP3
    HighPrecisionBoxNode.hlsl
#endif
    InitAccelerationStructure.hlsl
    InitExecuteIndirect.hlsl
    IntersectCommon.hlsl
    MergeSort.hlsl
    MortonCodes.hlsl
    PairCompression.hlsl
#if GPURT_BUILD_RTIP3_1
    PrimitiveStructure3_1.hlsl
    ../shadersClean/common/PrimitiveStructure.hlsli
    PrimitiveStructureEncoder3_1.hlsl
    QuantizedBVH8BoxNode.hlsl
    ObbCommon.hlsl
    KDOP.hlsl
    OrientedBoundingBoxes.hlsl
    RefitOrientedBounds3_1.hlsl
    RefitOrientedBoundsTopLevel3_1.hlsl
#endif
    RayQuery.hlsl
    RayQuery1_1.hlsl
    RayQuery2_0.hlsl
#if GPURT_BUILD_RTIP3
    RayQuery3_0.hlsl
#endif
#if GPURT_BUILD_RTIP3_1
    RayQuery3_1.hlsl
#endif
    Rebraid.hlsl
#if GPURT_BUILD_RTIP3
    rtip3_ds_stack_emulation.hlsl
    rtip3_x.h
    rtip3_opcode_emulation.hlsl
    rtip3_ops.hlsl
#endif
#if GPURT_BUILD_RTIP3_1
    rtip3_1.hlsli
#endif
    SerializeAS.hlsl
    ../shadersClean/common/SerializeDefs.hlsli
    TraceRay.hlsl
    TraceRay1_1.hlsl
    TraceRay2_0.hlsl
#if GPURT_BUILD_RTIP3
    TraceRay3_0.hlsl
#endif
#if GPURT_BUILD_RTIP3_1
    TraceRay3_1.hlsl
#endif
    TraceRayCommon.hlsl
    TrianglePrimitive.hlsl
    UpdateCommon.hlsl
    Update.hlsl
#if GPURT_BUILD_RTIP3_1
    Update3_1.hlsl
#endif
    UpdateParallel.hlsl
    UpdateQBVH.hlsl
    UpdateQBVH1_1.hlsl
#if GPURT_BUILD_RTIP3
    UpdateQBVH3_0.hlsl
#endif
    UpdateQBVHImpl.hlsl
#if GPURT_BUILD_CONTINUATION
    GpuRtLibraryCont.hlsl
    Continuations1_1.hlsl
    Continuations2_0.hlsl
    ../shadersClean/traversal/StackCommon.hlsli
#if GPURT_BUILD_RTIP3_1
    Continuations3_1.hlsl
#endif
#endif
)
set(otherDeps
    ../../gpurt/gpurtAccelStruct.h
    ../../gpurt/gpurtBuildSettings.h
    ../../gpurt/gpurtCounter.h
    ../../gpurt/gpurtDispatch.h
    ../../gpurt/gpurtTraceSettings.h
    ../shared/accelStruct.h
    ../shared/assert.h
    ../shared/debugBuffer.h
    ../shared/gpurtBuildConstants.h
    ../shared/hlslTypes.h
    ../shared/rayTracingDefs.h
#if GPURT_BUILD_RTIP3_1
    ../shadersClean/common/gfx12/childInfo.hlsli
    ../shadersClean/common/gfx12/internalNode.hlsli
    ../shadersClean/common/gfx12/primitiveNode.hlsli
    ../shared/gpurtShaderConstants.h
#endif
    ../shadersClean/common/Bits.hlsli
    ../shadersClean/common/Math.hlsli
    ../shadersClean/common/Math.hlsl
    ../shadersClean/common/Extensions.hlsli
    ../shadersClean/common/Extensions.hlsl
    ../shadersClean/common/ShaderDefs.hlsli
    ../shadersClean/common/BoundingBox.hlsli
    ../shadersClean/common/InstanceDesc.hlsli
    ../shadersClean/common/NodePointers.hlsli
    ../shadersClean/common/ScratchNode.hlsli
    ../shadersClean/traversal/TraversalDefs.hlsli
    ../shadersClean/common/gfx10/BoxNode1_0.hlsli
    ../shadersClean/common/gfx10/InstanceNode1_0.hlsli
    ../shadersClean/common/gfx10/ProceduralNode1_0.hlsli
    ../shadersClean/common/gfx10/TriangleNode1_0.hlsli
)

set(GPURT_SHADER_SOURCE_FILES "${gpurtHlsl}" "${otherDeps}" PARENT_SCOPE)

list(TRANSFORM gpurtHlsl PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

