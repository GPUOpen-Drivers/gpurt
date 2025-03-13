/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2019-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#include "gpurtInternal.h"
#include "gpurtInternalShaderBindings.h"

namespace GpuRt
{

#include "pipelines/g_internal_shaders.h"

#define ArraySize(x) (sizeof(x) / sizeof((x)[0]))

#if GPURT_DEVELOPER
#define COMPILER_OPTION_INIT nullptr,nullptr,0
#else // GPURT_DEVELOPER
#define COMPILER_OPTION_INIT nullptr,0
#endif

// Helper macro used to set up the pipeline build info array
#if GPURT_GENERATED_AMDIL_AVAILABLE
#define PIPELINE_BUILD_MAP_INFO(m, x) { 0, \
                                 m ## Mapping, ArraySize(m ## Mapping), \
                                 PipelineShaderCode{ Cs ## x, sizeof(Cs ## x), nullptr, 0, Cs ## x ## _spv, sizeof(Cs ## x ## _spv) }, \
                                 InternalRayTracingCsType::x, \
                                 COMPILER_OPTION_INIT, \
                                 #x }
#else // GPURT_GENERATED_AMDIL_AVAILABLE
#define PIPELINE_BUILD_MAP_INFO(m, x) { 0, \
                                 m ## Mapping, ArraySize(m ## Mapping), \
                                 PipelineShaderCode{ nullptr, 0, nullptr, 0, Cs ## x ## _spv, sizeof(Cs ## x ## _spv) }, \
                                 InternalRayTracingCsType::x, \
                                 COMPILER_OPTION_INIT, \
                                 #x }
#endif

#define PIPELINE_BUILD_INFO(x) PIPELINE_BUILD_MAP_INFO(x, x)
#define PIPELINE_BUILD_BVH_INFO(x) PIPELINE_BUILD_MAP_INFO(BuildBVH, x)

// Array of structs which contain the information required to build their associated raytracing pipeline
const PipelineBuildInfo InternalPipelineBuildInfo[size_t(InternalRayTracingCsType::Count)] =
{
    PIPELINE_BUILD_BVH_INFO(BuildParallel),
    PIPELINE_BUILD_BVH_INFO(EncodeTriangleNodes),
    PIPELINE_BUILD_BVH_INFO(EncodeAABBNodes),
    PIPELINE_BUILD_BVH_INFO(EncodeInstances),
    PIPELINE_BUILD_BVH_INFO(Rebraid),
    PIPELINE_BUILD_BVH_INFO(GenerateMortonCodes),
    PIPELINE_BUILD_BVH_INFO(BuildPLOC),
    PIPELINE_BUILD_INFO(UpdateQBVH),
    PIPELINE_BUILD_INFO(UpdateParallel),
    PIPELINE_BUILD_INFO(ClearBuffer),
    PIPELINE_BUILD_INFO(CopyBufferRaw),
    PIPELINE_BUILD_BVH_INFO(BuildQBVH),
    PIPELINE_BUILD_BVH_INFO(BitHistogram),
    PIPELINE_BUILD_BVH_INFO(ScatterKeysAndValues),
    PIPELINE_BUILD_BVH_INFO(ScanExclusiveInt4),
    PIPELINE_BUILD_BVH_INFO(ScanExclusivePartInt4),
    PIPELINE_BUILD_BVH_INFO(ScanExclusiveInt4DLB),
    PIPELINE_BUILD_BVH_INFO(InitScanExclusiveInt4DLB),
    PIPELINE_BUILD_BVH_INFO(DistributePartSumInt4),
    PIPELINE_BUILD_INFO(EmitCurrentSize),
    PIPELINE_BUILD_INFO(EmitCompactSize),
    PIPELINE_BUILD_INFO(EmitSerializeDesc),
    PIPELINE_BUILD_INFO(EmitToolVisDesc),
    PIPELINE_BUILD_INFO(CopyAS),
    PIPELINE_BUILD_INFO(CompactAS),
    PIPELINE_BUILD_INFO(DecodeAS),
    PIPELINE_BUILD_INFO(SerializeAS),
    PIPELINE_BUILD_INFO(DeserializeAS),
    PIPELINE_BUILD_INFO(InitExecuteIndirect),
    PIPELINE_BUILD_BVH_INFO(PairCompression),
    PIPELINE_BUILD_BVH_INFO(MergeSort),
    PIPELINE_BUILD_BVH_INFO(MergeSortLocal),
    PIPELINE_BUILD_BVH_INFO(MergeSortGlobalIteration),
    PIPELINE_BUILD_BVH_INFO(MergeSortCopyLastLevel),
    PIPELINE_BUILD_INFO(UpdateTriangles),
    PIPELINE_BUILD_INFO(UpdateAabbs),
    PIPELINE_BUILD_INFO(InitAccelerationStructure),
    PIPELINE_BUILD_INFO(InitUpdateAccelerationStructure),
#if GPURT_BUILD_RTIP3_1
    PIPELINE_BUILD_BVH_INFO(RefitOrientedBounds),
    PIPELINE_BUILD_BVH_INFO(RefitOrientedBoundsTopLevel),
    PIPELINE_BUILD_BVH_INFO(CompressPrims),
#endif
#if GPURT_BUILD_RTIP3|| GPURT_BUILD_RTIP3_1
    PIPELINE_BUILD_BVH_INFO(BuildParallelRtip3x),
#endif
    PIPELINE_BUILD_BVH_INFO(BuildFastAgglomerativeLbvh),
    PIPELINE_BUILD_BVH_INFO(EncodeQuadNodes),
#if GPURT_BUILD_RTIP3_1
    PIPELINE_BUILD_INFO(BuildTrivialBvh),
    PIPELINE_BUILD_INFO(BuildSingleThreadGroup32),
    PIPELINE_BUILD_INFO(BuildSingleThreadGroup64),
    PIPELINE_BUILD_INFO(BuildSingleThreadGroup128),
    PIPELINE_BUILD_INFO(BuildSingleThreadGroup256),
    PIPELINE_BUILD_INFO(BuildSingleThreadGroup512),
    PIPELINE_BUILD_INFO(BuildSingleThreadGroup1024),
    PIPELINE_BUILD_BVH_INFO(EncodeHwBvh3_1),
    PIPELINE_BUILD_BVH_INFO(Update3_1),
    PIPELINE_BUILD_BVH_INFO(RefitInstanceBounds),
#endif
};

#undef PIPELINE_BUILD_INFO
#undef ArraySize

};
