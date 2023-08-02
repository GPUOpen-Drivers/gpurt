/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2021-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
// NOTE: This header file is shared by HLSL

#ifndef GPURT_COUNTER_H
#define GPURT_COUNTER_H

// Major version changes: The major version below must be incremented in the event that an incompatible change is
// required. An incompatible change means that consumers of the data are required to make changes in order to interpret
// the data structures correctly.
//
// Minor version changes: The minor version below must be incremented when fields are added or modified in any
// structures. Fields may be added to the end of structures, and reserved fields may be changed to meaningful values.
// Reserved fields are zeroed by GPURT until they have a purpose. If a structure is smaller than the size of the most
// recent version, tools may assume the non-present fields are zero.

#define GPURT_COUNTER_MAJOR_VERSION 1
#define GPURT_COUNTER_MINOR_VERSION 0
#define GPURT_COUNTER_VERSION       ((GPURT_COUNTER_MAJOR_VERSION << 16) | GPURT_COUNTER_MINOR_VERSION)

#include "gpurtAccelStruct.h"

#ifdef __cplusplus
namespace GpuRt
{

typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

#endif

// ====================================================================================================================
#define TRACERAY_COUNTER_MODE_DISABLE          0
#define TRACERAY_COUNTER_MODE_RAYHISTORY_LIGHT 1
#define TRACERAY_COUNTER_MODE_RAYHISTORY_FULL  2
#define TRACERAY_COUNTER_MODE_TRAVERSAL        3
#define TRACERAY_COUNTER_MODE_CUSTOM           4
#define TRACERAY_COUNTER_MODE_DISPATCH         5

#ifdef __cplusplus
#pragma pack(push, 4)
#endif

// ====================================================================================================================
struct IndirectCounterMetadata
{
    uint64 rayGenShaderId;        // Raygeneration shader identifier
    uint32 dispatchRayDimensionX; // DispatchRayDimension X
    uint32 dispatchRayDimensionY; // DispatchRayDimension Y
    uint32 dispatchRayDimensionZ; // DispatchRayDimension Z
};

// =====================================================================================================================
// Per-dispatch ray tracing counter data
struct DispatchCounterData
{
    uint32 numActiveRays;                  // Total number of rays that invoked traversal.
    uint32 numIterations;                  // Total number of iterations
    uint32 minIterations;                  // Minimum iterations amongst all active rays
    uint32 maxIterations;                  // Maximum iterations amongst all active rays
    uint32 activeLaneCountPerIteration;    // Active lane count within a wave per traversal iteration.
    uint32 numWaveIterations;              // Number of wave iterations.
    uint32 maxActiveLaneCountPerIteration; // Maximum active lane count amongst unique node types for a wave iteration.
};

// ====================================================================================================================
// Ray tracing counter information
struct CounterInfo
{
    uint32 dispatchRayDimensionX;        // DispatchRayDimension X
    uint32 dispatchRayDimensionY;        // DispatchRayDimension Y
    uint32 dispatchRayDimensionZ;        // DispatchRayDimension Z
    uint32 hitGroupShaderRecordCount;    // Hit-group shader record count
    uint32 missShaderRecordCount;        // Miss shader record count
    uint32 pipelineShaderCount;          // Pipeline per-shader count
    uint64 stateObjectHash;              // State object hash
    uint32 counterMode;                  // Counter mode
    uint32 counterMask;                  // Traversal counter mask
    uint32 counterStride;                // Ray tracing counter stride
    uint32 rayCounterDataSize;           // Per-ray counter data
    uint32 lostTokenBytes;               // Total lost token bytes
    uint32 counterRayIdRangeBegin;       // Partial rayID range begin
    uint32 counterRayIdRangeEnd;         // Partial rayID range end
    uint32 pipelineType;                 // Pipeline type (native RT or RayQuery). RayTracing=0, Compute=1, Graphics=2
};
#ifdef __cplusplus
#pragma pack(pop)
#endif

// ====================================================================================================================
#define TCID_NUM_RAY_BOX_TEST        0       // Number of ray-box tests
#define TCID_NUM_RAY_TRIANGLE_TEST   1       // Number of ray-triangle tests
#define TCID_NUM_ITERATION           2       // Traversal loop iteration count
#define TCID_MAX_TRAVERSAL_DEPTH     3       // Maximum traversal stack depth
#define TCID_NUM_ANYHIT_INVOCATION   4       // Number of anyhit shader invocations
#define TCID_SHADER_ID               5       // Hit/Miss shader ID
#define TCID_SHADER_RECORD_INDEX     6       // Shader record index
#define TCID_TIMING_DATA             7       // Realtime timer data
#define TCID_WAVE_ID                 8       // Wave ID
#define TCID_NUM_CANDIDATE_HITS      9       // Number of candidate leaf node hits
#define TCID_INSTANCE_INTERSECTIONS 10       // Instance nodes intersected
#define TCID_COUNT                  11       // Number of traversal counter IDs
#define TCID_STRIDE                  4       // Counter data stride
#define TCD_SIZE (TCID_STRIDE * TCID_COUNT)  // Counter size per-ray

#define TCID_SHADER_RECORD_INDEX_MISS      0x80000000
#define TCID_SHADER_RECORD_INDEX_DATA_MASK 0x7FFFFFFF

// ====================================================================================================================
// Per-ray traversal counter data
struct TraversalCounter
{
    uint32 data[TCID_COUNT];  // Counter data per-ray
};

// ====================================================================================================================
// Ray ID
#define RAYID_CONTROL_MASK 0x80000000
#define RAYID_ID_MASK      0x3FFFFFFF

// ====================================================================================================================
// Ray history token type
#define RAY_HISTORY_TOKEN_TYPE_BEGIN                                 0
#define RAY_HISTORY_TOKEN_TYPE_TOP_LEVEL                             1
#define RAY_HISTORY_TOKEN_TYPE_BOTTOM_LEVEL                          2
#define RAY_HISTORY_TOKEN_TYPE_INTERSECTION_RESULT                   3
#define RAY_HISTORY_TOKEN_TYPE_FUNC_CALL                             4
#define RAY_HISTORY_TOKEN_TYPE_GPU_TIME                              5
#define RAY_HISTORY_TOKEN_TYPE_ANYHIT_STATUS                         6
#define RAY_HISTORY_TOKEN_TYPE_FUNC_CALL_V2                          7
#define RAY_HISTORY_TOKEN_TYPE_CANDIDATE_INTERSECTION_RESULT         8
#define RAY_HISTORY_TOKEN_TYPE_INTERSECTION_RESULT_V2                9
#define RAY_HISTORY_TOKEN_TYPE_BEGIN_V2                              10
#define RAY_HISTORY_TOKEN_TYPE_RESERVED                              0x8000
#define RAY_HISTORY_TOKEN_TYPE_UNKNOWN                               0xffff

#define RAY_HISTORY_TOKEN_BEGIN_SIZE                                 16
#define RAY_HISTORY_TOKEN_TOP_LEVEL_SIZE                             2
#define RAY_HISTORY_TOKEN_BOTTOM_LEVEL_SIZE                          2
#define RAY_HISTORY_TOKEN_INTERSECTION_RESULT_SIZE                   2
#define RAY_HISTORY_TOKEN_FUNC_CALL_SIZE                             2
#define RAY_HISTORY_TOKEN_GPU_TIME_SIZE                              2
#define RAY_HISTORY_TOKEN_CONTROL_SIZE                               2
#define RAY_HISTORY_TOKEN_FUNC_CALL_V2_SIZE                          3
#define RAY_HISTORY_TOKEN_CANDIDATE_INTERSECTION_RESULT_SIZE         2
#define RAY_HISTORY_TOKEN_INTERSECTION_RESULT_V2_SIZE                6
#define RAY_HISTORY_TOKEN_BEGIN_V2_SIZE                              19

#define RAY_HISTORY_CONTROL_TOKEN_TYPE_MASK    0xFFFF
#define RAY_HISTORY_CONTROL_TOKEN_LENGTH_MASK  0xFF
#define RAY_HISTORY_CONTROL_TOKEN_DATA_MASK    0xFF
#define RAY_HISTORY_CONTROL_TOKEN_DATA_SHIFT   0
#define RAY_HISTORY_CONTROL_TOKEN_LENGTH_SHIFT 16
#define RAY_HISTORY_CONTROL_TOKEN_TYPE_SHIFT   24

// ====================================================================================================================
// Types of ray history DXR shader function calls
// NOTE: Type is stored within RayHistoryControlToken.data field, so value 0 is reserved
#define RAY_HISTORY_FUNC_CALL_TYPE_MISS         1
#define RAY_HISTORY_FUNC_CALL_TYPE_CLOSEST      2
#define RAY_HISTORY_FUNC_CALL_TYPE_ANY_HIT      3
#define RAY_HISTORY_FUNC_CALL_TYPE_INTERSECTION 4

#define RAY_TRACING_COUNTER_REQUEST_BYTE_OFFSET 0
#define RAY_TRACING_COUNTER_RAY_ID_BYTE_OFFSET  4
#define RAY_TRACING_COUNTER_RESERVED_BYTE_SIZE  8

#ifdef __cplusplus
// ====================================================================================================================
// Ray tracing binary file type enumeration
enum class RayTracingBinaryFileType : uint32
{
    Unknown = 0,
    RayHistory,
    TraversalCounter,
    BvhRaw,
    BvhDecoded,
};

// ====================================================================================================================
// Flags indicating various traversal features used for ray history capture
union RayHistoryTraversalFlags
{
    struct
    {
        uint32 boxSortMode      : 1;    /// Indicates box sort mode used by traversal
        uint32 usesNodePtrFlags : 1;    /// Indicates node pointer flag usage for culling
        uint32 reserved         : 30;
    };

    uint32 u32All;
};

// ====================================================================================================================
// Common header for ray tracing binary files
struct RayTracingBinaryHeader
{
    uint32                   identifier; // Must be GpuRt::RayTracingFileIdentifier
    uint32                   version;    // Binary file format version
    RayTracingBinaryFileType fileType;   // Binary file type
    uint32                   ipLevel;    // Raytracing IP level (Pal::RayTracingIpLevel)
    uint32                   headerSize; // Header size including RayTracingBinaryHeader
};

// ====================================================================================================================
// Header for decoded acceleration structure binary files
struct RayTracingBinaryHeaderDecoded : RayTracingBinaryHeader
{
    uint32             accelStructType; // GpuRt::AccelStructType
    uint32             buildFlags;      // API build flags
    DriverDecodeHeader decodeHeader;    // Driver decoded header
};

// ====================================================================================================================
// Header for raw acceleration structure binary files
struct RayTracingBinaryHeaderRaw : RayTracingBinaryHeader
{
};

// ====================================================================================================================
// Header for ray history binary files
struct RayTracingBinaryHeaderRayHistory : RayTracingBinaryHeader
{
    CounterInfo              counterInfo;
    RayHistoryTraversalFlags flags;
};

// ====================================================================================================================
// Header for traversal counter binary files
struct RayTracingBinaryHeaderTraversalCounter : RayTracingBinaryHeader
{
    CounterInfo counterInfo;
};

// ====================================================================================================================
// 32-bit unique ray identifier calculated as below.
//      uint32_t id = threadID.x + (threadID.y * dim.x) + (threadID.z * dim.x * dim.y);
struct RayID
{
    uint32 id       : 30; // Unique identifier
    uint32 unused   : 1;  // Reserved for future uses
    uint32 control  : 1;  // Indicates that a control DWORD follows the RayID in token stream
};

// ====================================================================================================================
// Ray history token type enumeration
enum RayHistoryTokenType : uint16
{
    RayHistoryTokenBegin,
    RayHistoryTokenTopLevel,
    RayHistoryTokenBottomLevel,
    RayHistoryTokenIntersectionResult,
    RayHistoryTokenFunctionCall,
    RayHistoryTokenGpuTimestamp,
    RayHistoryTokenAnyHitStatus,
    RayHistoryTokenFunctionCall_v2,
    RayHistoryTokenCandidateIntersectionResult,
    RayHistoryTokenIntersectionResult_v2,
    RayHistoryTokenBegin_v2,

    // Anything with the top bit set is reserved
    RayHistoryTokenReserved = 0x8000,
    RayHistoryTokenUnknown = 0xffff,
};

// ====================================================================================================================
// Ray history token begin data format
struct RayHistoryTokenBeginData
{
    uint32 hwWaveId;              // Unique hardware wave identifier (SQ_WAVE_HW_ID_LEGACY)
    uint32 dispatchRaysIndex[3];  // Dispatch rays index in 3-dimensional ray grid

    uint32 accelStructAddrLo; // API top-level acceleration structure base address (lower 32-bits)
    uint32 accelStructAddrHi; // API top-level acceleration structure base address (upper 32-bits)
    uint32 rayFlags;          // API ray flags (see API documentation for further details)

    union
    {
        struct
        {
            uint32 instanceInclusionMask          : 8;  // API instance inclusion mask (see API documentation for further details)
            uint32 rayContributionToHitGroupIndex : 4;  // API ray hit group offset (see API documentation for further details)
            uint32 geometryMultiplier             : 4;  // API ray geometry stride (see API documentation for further details)
            uint32 missShaderIndex                : 16; // API ray miss shader index (see API documentation for further details)
        };

        uint32 packedTraceRayParams;  // Packed parameters
    };

    struct
    {
        float  origin[3];   // Ray origin
        float  tMin;        // Ray time minimum bounds
        float  direction[3];// Ray direction
        float  tMax;        // Ray time maximum bounds
    } rayDesc;
};

// ====================================================================================================================
struct HwWaveIdGfx10
{
    uint32 wave_id : 5;
    uint32 simd_id : 2;
    uint32 wgp_id : 4;
    uint32 sa_id : 1;
    uint32 se_id : 3;
    uint32 padding : 17;
};

// ====================================================================================================================
// Ray history token begin data v2 format
struct RayHistoryTokenBeginData_v2 : RayHistoryTokenBeginData
{
    uint32 staticId;  // 32-bit unique identifier, unique to a shader call site (TraceRay/RayQuery.TraceRayInline()
    uint32 dynamicId; // 32-bit unique identifier generated on traversal begin that is uniform across the wave.
    uint32 parentId;  // 32-bit unique identifier for parent traversal that invoked recursive traversal.
};

// ====================================================================================================================
struct RayHistoryTokenNodePtrData
{
    uint32 nodePtr;
};

// ====================================================================================================================
// Ray history token top level data format
struct RayHistoryTokenTopLevelData
{
    uint64 baseAddr;
};

// ====================================================================================================================
// Ray history token bottom level data format
struct RayHistoryTokenBottomLevelData
{
    uint64 baseAddr;
};

// ====================================================================================================================
// Ray history token end data format
struct RayHistoryTokenIntersectionResultData
{
    uint32 primitiveIndex; // Primitive index of the geometry hit by this ray (-1 for a miss)
    uint32 geometryIndex;  // Geometry index of the geometry hit by this ray (-1 for a miss)
};

// ====================================================================================================================
// Ray history extended token end data format
struct RayHistoryTokenIntersectionResultData_v2 : RayHistoryTokenIntersectionResultData
{
    uint32 instanceIndexAndHitKind;  // Instance index [0:23] and hit kind [24:31]
    uint32 numIterations;            // Traversal iterations
    uint32 numInstanceIntersections; // Number of instance intersections
    float  hitT;                     // Closest intersection distance on ray
};

// ====================================================================================================================
// Ray history token function call data format
struct RayHistoryTokenFunctionCallData
{
    uint64 shaderId;
};

// ====================================================================================================================
// Ray history extended token function call data format
struct RayHistoryTokenFunctionCallData_v2 : RayHistoryTokenFunctionCallData
{
    uint32 shaderRecordIndex;
};

// ====================================================================================================================
// Ray history token candidate intersection data format
struct RayHistoryTokenCandidateIntersectionResultData
{
    float  hitT;     // Hit T as reported by the intersection shader or determined by fixed function triangle intersector
    uint32 hitKind;  // Hit kind (only lower 8-bits are valid)
};

// ====================================================================================================================
// Ray history token gpu timestamp data format
struct RayHistoryTokenGpuTimestampData
{
    uint64_t clocks;
};

// ====================================================================================================================
// Ray history control token
union RayHistoryControlToken
{
    struct
    {
        RayHistoryTokenType type;        // Token type
        uint8               tokenLength; // Length of tokens of this type, in DWORDS
        uint8               data;        // Additional data (optional)
    };

    uint32 u32;
};

// ====================================================================================================================
// Ray history trace header format
struct RayHistoryRdfChunkHeader
{
    uint32 dispatchID;
};

// ====================================================================================================================
// Ray history trace metadata kind
enum class RayHistoryMetadataKind : uint32
{
    CounterInfo        = 1,
    DispatchDimensions = 2,
    TraversalFlags     = 3,
};

// ====================================================================================================================
// Ray history trace metadata chunk format
struct RayHistoryMetadataInfo
{
    RayHistoryMetadataKind kind;
    uint64                 sizeInByte;
};

// ====================================================================================================================
// Ray history trace shader table type
enum class ShaderTableType : uint32
{
    RayGen,
    Miss,
    HitGroup,
    Callable,
};

// ====================================================================================================================
// Ray history trace shader table chunk format
struct ShaderTableInfo
{
    ShaderTableType type;
    uint32          reserved;
    uint64          stride;
    uint64          sizeInBytes;
};
} // namespace GpuRt
#endif
#endif
