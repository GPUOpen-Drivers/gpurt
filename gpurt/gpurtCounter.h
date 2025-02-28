/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2021-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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

#define GPURT_COUNTER_MAJOR_VERSION 2
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

enum class IndexFormat : uint32;
enum class VertexFormat : uint32;

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

#ifdef __cplusplus
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
    struct
    {
        uint32 isIndirect   : 1;         // Execute indirect
        uint32 reserved     : 31;
    };
};
#else
typedef uint32 CounterInfo;
#endif

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
#ifdef __cplusplus
    TraversalCounter(int val)
    {
        memset(this, val, sizeof(TraversalCounter));
    }

    TraversalCounter() : TraversalCounter(0)
    {
    }
#endif
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
#define RAY_HISTORY_TOKEN_TYPE_WAVE_BEGIN                            11
#define RAY_HISTORY_TOKEN_TYPE_TRIANGLE_HIT_RESULT                   12
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
#define RAY_HISTORY_WAVE_BEGIN_PACKET_HEADER_SIZE                    6
#define RAY_HISTORY_WAVE_BEGIN_PACKET_DATA_SIZE                      7
#define RAY_HISTORY_TOKEN_TRIANGLE_HIT_RESULT_SIZE                   1

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

// ====================================================================================================================
// Each bit in the uniformVarBitMask corresponds to one entry in the payload type enumeration.
enum RayHistoryTokenWaveBeginTokenPayloadType
{
    DispatchRaysIndexX,
    DispatchRaysIndexY,
    DispatchRaysIndexZ,
    AccelStructAddrLo,
    AccelStructAddrHi,
    RayFlags,
    PackedTraceRayParams,
    TMin,
    TMax
};

#define WAVE_UNIFORM_MASK_DISPATCH_X 0x0001
#define WAVE_UNIFORM_MASK_DISPATCH_Y 0x0002
#define WAVE_UNIFORM_MASK_DISPATCH_Z 0x0004
#define WAVE_UNIFORM_MASK_ADDR_LO    0x0008
#define WAVE_UNIFORM_MASK_ADDR_HI    0x0010
#define WAVE_UNIFORM_MASK_RAY_FLAGS  0x0020
#define WAVE_UNIFORM_MASK_PARAMS     0x0040
#define WAVE_UNIFORM_MASK_TMIN       0x0080
#define WAVE_UNIFORM_MASK_TMAX       0x0100
#define WAVE_UNIFORM_BITS_VALID_MASK 0x01ff

// ====================================================================================================================
struct RayHistoryTokenWaveBeginPacketHeader
{
#ifdef __cplusplus
    uint32_t hwWaveId          : 16; // Unique hardware wave identifier (SQ_WAVE_HW_ID_LEGACY), low 16-bits only
    uint32_t uniformVarBitMask : 16; // Bit mask of uniform payload data
#else
    uint32_t packedHwWaveIdAndMask;  // See above.
#endif
    uint32_t activeLaneMaskLo;       // Lower 32-bits of active lane mask
    uint32_t activeLaneMaskHi;       // Uppper 32-bits of active lane mask
    uint32_t staticId;
    uint32_t dynamicId;
    uint32_t parentId;

    // variable data follows (size = bitcount(uniformVarBitMask); max size of 24 bytes)
    //
};

// ====================================================================================================================
struct RayHistoryTokenWaveBeginPacketData
{
    uint32 rayId;
    float origin[3];
    float direction[3];

    // variable data follows (size = bitcount(~uniformVarBitMask))
    //
};

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
    BuildInput,
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
enum class CmdQueueType : uint32
{
    Graphics = 0,
    AsyncCompute,
    Count
};

// ====================================================================================================================
// Header for build acceleration structure input binary files
struct RayTracingBinaryHeaderBuild : RayTracingBinaryHeader
{
    CmdQueueType queueType;      // Queue Type (async compute, graphics)
    uint32       commandListId;  // Command list ID
    uint32       frameNumber;    // Frame ID
    uint64       accelAddr;      // Acceleration structure address
    uint32       updateCount;    // Number of updates
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
struct TriangleInputHeader
{
    gpusize      transform3x4; // Offset to transform data in the chunk (can be 0)
                               // Count: 1, Stride: 36 bytes
    IndexFormat  indexFormat;  // Index format (unknown when non-indexed)
    VertexFormat vertexFormat; // Vertex position format
    uint32       indexCount;   // Number of vertex indices
    uint32       vertexCount;  // Number of vertex in the vertex buffer
    gpusize      indexBuffer;  // Offset to index buffer data in the chunk (can be 0)
                               // Count: indexCount, Stride: based on indexFormat
    gpusize      vertexBuffer; // Offset to vertex buffer data in the chunk
                               // Count: vertexCount, Stride: vertexStride
    uint32       vertexStride; // Stride in bytes between vertex indices
    uint32       reserved;
};

#define TRIANGLE_INPUTS_TRANSFORM_OFFSET                0
#define TRIANGLE_INPUTS_INDEX_FORMAT_OFFSET             8
#define TRIANGLE_INPUTS_VERTEX_FORMAT_OFFSET           12
#define TRIANGLE_INPUTS_INDEX_COUNT_OFFSET             16
#define TRIANGLE_INPUTS_VERTEX_COUNT_OFFSET            20
#define TRIANGLE_INPUTS_INDEX_DATA_OFFSET              24
#define TRIANGLE_INPUTS_VERTEX_DATA_OFFSET             32
#define TRIANGLE_INPUTS_VERTEX_STRIDE_OFFSET           40
#define TRIANGLE_INPUTS_HEADER_SIZE                    48

GPURT_STATIC_ASSERT(TRIANGLE_INPUTS_TRANSFORM_OFFSET == offsetof(TriangleInputHeader, transform3x4), "");
GPURT_STATIC_ASSERT(TRIANGLE_INPUTS_INDEX_FORMAT_OFFSET == offsetof(TriangleInputHeader, indexFormat), "");
GPURT_STATIC_ASSERT(TRIANGLE_INPUTS_VERTEX_FORMAT_OFFSET == offsetof(TriangleInputHeader, vertexFormat), "");
GPURT_STATIC_ASSERT(TRIANGLE_INPUTS_INDEX_COUNT_OFFSET == offsetof(TriangleInputHeader, indexCount), "");
GPURT_STATIC_ASSERT(TRIANGLE_INPUTS_VERTEX_COUNT_OFFSET == offsetof(TriangleInputHeader, vertexCount), "");
GPURT_STATIC_ASSERT(TRIANGLE_INPUTS_VERTEX_DATA_OFFSET == offsetof(TriangleInputHeader, vertexBuffer), "");
GPURT_STATIC_ASSERT(TRIANGLE_INPUTS_VERTEX_STRIDE_OFFSET == offsetof(TriangleInputHeader, vertexStride), "");
GPURT_STATIC_ASSERT(sizeof(TriangleInputHeader) == TRIANGLE_INPUTS_HEADER_SIZE, "Triangle inputs header mismatch.");

// ====================================================================================================================
struct AABBInputHeader
{
    uint32  aabbCount;          // Number of bounding boxes
    uint32  aabbStride;         // Stride in bytes between consecutive bounding boxes
    gpusize aabbBuffer;         // Offset to AABB data in AabbBuffer chunk
};

#define AABB_INPUTS_COUNT_OFFSET                0
#define AABB_INPUTS_STRIDE_OFFSET               4
#define AABB_INPUTS_BUFFER_OFFSET               8
#define AABB_INPUTS_HEADER_SIZE                16

GPURT_STATIC_ASSERT(AABB_INPUTS_COUNT_OFFSET == offsetof(AABBInputHeader, aabbCount), "");
GPURT_STATIC_ASSERT(AABB_INPUTS_STRIDE_OFFSET == offsetof(AABBInputHeader, aabbStride), "");
GPURT_STATIC_ASSERT(AABB_INPUTS_BUFFER_OFFSET == offsetof(AABBInputHeader, aabbBuffer), "");
GPURT_STATIC_ASSERT(sizeof(AABBInputHeader) == AABB_INPUTS_HEADER_SIZE, "AABB inputs header mismatch.");

// ====================================================================================================================
struct InstanceInputHeader
{
    uint32  instanceCount;         // Number of instance nodes
    uint32  instanceStride;        // Stride in bytes between instances
    gpusize instances;             // Offset to instance desc data in the chunk
};

#define INSTANCE_INPUTS_COUNT_OFFSET                0
#define INSTANCE_INPUTS_STRIDE_OFFSET               4
#define INSTANCE_INPUTS_INSTANCES_OFFSET            8
#define INSTANCE_INPUTS_HEADER_SIZE                16

GPURT_STATIC_ASSERT(INSTANCE_INPUTS_COUNT_OFFSET == offsetof(InstanceInputHeader, instanceCount), "");
GPURT_STATIC_ASSERT(INSTANCE_INPUTS_STRIDE_OFFSET == offsetof(InstanceInputHeader, instanceStride), "");
GPURT_STATIC_ASSERT(INSTANCE_INPUTS_INSTANCES_OFFSET == offsetof(InstanceInputHeader, instances), "");
GPURT_STATIC_ASSERT(sizeof(InstanceInputHeader) == INSTANCE_INPUTS_HEADER_SIZE, "Instance inputs header mismatch.");

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
    RayHistoryTokenWaveBegin,
    RayHistoryTokenTriangleHitResult,

    // Custom application tokens starting here
    RayHistoryTokenCustom = 0x4000,

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
    float  hitT;     // Hit T as reported by the intersection shader
    uint32 hitKind;  // Hit kind (only lower 8-bits are valid)
};

// ====================================================================================================================
// Ray history token for triangle intersection data format
struct RayHistoryTokenTriangleHitResultData
{
    float  hitT;   // Closest intersection distance on ray
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

#ifdef __cplusplus
#pragma pack(push, 4)
#endif
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
    Unused             = 2,
    TraversalFlags     = 3,
    UserMarkerInfo     = 4
};

// ====================================================================================================================
// Ray history trace metadata chunk format
struct RayHistoryMetadataInfo
{
    RayHistoryMetadataKind kind;
    // Older versions of this struct had padding between kind and sizeOfBytes keep a reserved value for compatibility.
    uint32                 reserved;
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
#ifdef __cplusplus
#pragma pack(pop)
#endif

} // namespace GpuRt
#endif
#endif
