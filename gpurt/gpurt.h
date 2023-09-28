/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2019-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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

#pragma once
#include <stdint.h>
#include "pal.h"
#include "palSysMemory.h"
#include "palDevice.h"
#include "palPipeline.h"
#include "palMutex.h"
#include "palHashMap.h"
#include "palVector.h"
#include "src/gpurtTraceSource.h"
#include "gpurtDispatch.h"

#define GPURT_API_ENTRY PAL_STDCALL

using ClientCmdContextHandle = void*;
using ClientGpuMemHandle = void*;

namespace Pal
{
class IDevice;
class IPipeline;
class ICmdBuffer;
class IPlatform;
struct DeviceProperties;
};

namespace GpuUtil
{

class ITraceController;
class ITraceSource;

};

namespace GpuRt
{
using uint8  = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;
};

namespace GpuRt
{

// Individual bit values for static traversal flags returned by AmdTraceRayGetStaticFlags(), in case
// clients need to make decisions based on individual bits during code generation.
enum class StaticPipelineFlag : uint32
{
    SkipTriangles                  = 0x100,        // Always skip triangle node intersections
    SkipProceduralPrims            = 0x200,        // Always skip procedural node intersections
    BvhCollapse                    = (1u << 31),   // Enable BVH collapse
    UseTreeRebraid                 = (1u << 30),   // Use Tree Rebraid for TraceRays
    EnableAccelStructTracking      = (1u << 29),   // Enable logging of TLAS addresses using AccelStructTracker
    EnableTraversalCounter         = (1u << 28),   // Enable Traversal counters
    Reserved                       = (1u << 27),
    EnableFusedInstanceNodes       = (1u << 26),   // Enable fused instance nodes
    Reserved2                      = (1u << 25),
    Reserved3                      = (1u << 24),
    Reserved4                      = (1u << 23),
    Reserved6                      = (1u << 22),
};

// TODO #gpurt: Abstract these?  Some of these probably should come from PAL device properties

constexpr uint32 RayTracingBoxGrowthNumUlpsDefault = 6;

constexpr uint64 RayTracingMaxNumNodes = 0x40000000000;

// Byte size of a BVH2 node
constexpr size_t RayTracingBVHNodeSize  = 64;

// Byte size of a BVH4 node, each AABB encoded using 32bit floats
constexpr size_t RayTracingQBVH32NodeSize = 128;

// Byte size of a BVH4 node, each AABB encoded using 16bit floats
constexpr size_t RayTracingQBVH16NodeSize = 64;

// Byte size of a instance node
constexpr size_t RayTracingInstanceNodeSize = 128;

// Byte size of a fused instance node
constexpr size_t RayTracingFusedInstanceNodeSize = 256;

// Max stride between shader records
constexpr uint32 RayTraceMaxShaderRecordByteStride   = 4096;

// Base address alignment of a shader record buffer
constexpr uint32 RayTraceShaderRecordBaseAlignment   = 64;

// Max number of instances in a top-level acceleration structure
constexpr uint32 RayTraceMaxTopLevelInstanceCount    = (1u << 24);

// Max number of triangles in a bottom-level acceleration structure
constexpr uint32 RayTraceMaxBottomLevelTriangleCount = (1u << 29);

// Max number of geometry records in a bottom-level acceleration structure
constexpr uint32 RayTraceMaxBottomLevelGeometryCount = (1u << 24);

// Byte size of a shader identifier structure
constexpr uint32 RayTraceShaderIdentifierByteSize    = 32;

// Byte size of an acceleration structure handle
constexpr uint32 RayTraceAccelHandleByteSize         = 8;

// Acceleration structure base address memory alignment
constexpr uint32 RayTraceAccelMemoryBaseAlignment    = 256;

// Acceleration structure Gpu Va Size
constexpr uint32 RayTracingGpuVaSize                 = 8;

// Visualisation header size in bytes
constexpr uint32 RayTracingVisualisationHeaderSize   = 8;

// Visualisation header size in bytes
constexpr uint32 RayTracingGeometryDescSize          = 56;

// Visualisation header size in bytes
constexpr uint32 RayTracingDecodedLeafDataSize       = 36;

// Acceleration structure Guid Size
constexpr uint32 RayTracingGuidSize                  = 32;

// Serialized acceleration structure header size
constexpr uint32 RayTracingSerializedAsHeaderSize    = (RayTracingGuidSize + (8 * 3));

// Serialized acceleration structure header size offset
constexpr uint32 RayTracingSerializedAsHeaderSizeOffset = RayTracingGuidSize;

// DeSerialized acceleration structure header size offset
constexpr uint32 RayTracingDeSerializedAsHeaderSizeOffset = RayTracingGuidSize + 8;

// Serialized acceleration structure header number of blas ptrs offset
constexpr uint32 RayTracingSerializedAsHeaderNumBlasPtrsOffset = RayTracingGuidSize + 16;

// Instance desc Va Lo offset
constexpr uint32 RayTracingInstanceDescVaLoOffset    = 56;

// Instance desc Va Hi offset
constexpr uint32 RayTracingInstanceDescVaHiOffset    = 60;

// Acceleration structure triangle node size
constexpr uint32 RayTracingTriangleNodeSize = 64;

// Acceleration structure user node procedural size
constexpr uint32 RayTracingUserNodeProceduralSize = 64;

// BVH SRD Box Sort Shift constant
constexpr uint32 RayTracingBvhSrdBoxSortShift = 31;

// BVH SRD Box Sort Expansion Shift constant
constexpr uint32 RayTracingBvhSrdBoxExpansionShift = 23;

// BVH SRD Disable Box Sort constant
constexpr uint32 RayTracingBvhSrdBoxSortDisable = 3;

constexpr uint32 RayTraceAccelerationStructureByteAlignment = 256;

// Maximum number of geometries in a bottom level acceleration structure: 2^24
constexpr uint64 RayTraceBLASMaxGeometries = 0x1000000;

// Maximum number of primitives in a bottom level acceleration structure (sum across all geometries): 2^29
constexpr uint64 RayTraceBLASMaxPrimitiveCount = 0x20000000;

// Maximum instance count in a top level acceleration structure: 2^24
constexpr uint64 RayTraceTLASMaxInstanceCount = 0x1000000;

// Maximum ray generation threads/invocations in one dispatch 1073741824 (0x4000,0000)
constexpr uint32 RayTraceRayGenShaderThreads = 0x40000000;

// Identifier for GPURT file dumps
constexpr uint32 RayTracingFileIdentifier = 0x46425452; // 'R''T''B''F'

// Either a GPU or a CPU address that may be written to
union RwGpuCpuAddr
{
    gpusize gpu;
    void*   pCpu;
};

// Either a GPU or a CPU address that may only be read from
union GpuCpuAddr
{
    gpusize     gpu;
    const void* pCpu;
};

// Type of internal pipeline resource mapping node
enum class NodeType : uint32
{
    Constant,            // Inline/root constant
    ConstantBuffer,      // Constant buffer VA
    Uav,                 // Raw or structured buffer UAV VA
    ConstantBufferTable, // Single DWORD constant buffer table VA
    UavTable,            // Single DWORD UAV table VA (raw or structured)
    TypedUavTable,       // Single DWORD UAV table VA (typed)
    Count
};

// Internal pipeline resource mapping node
struct NodeMapping
{
    NodeType type;
    uint32   dwSize;

    uint32   binding;
    uint32   descSet;

    uint32   logicalId;
    uint32   dwOffset;
    uint32   srdStartIndex;
    uint32   srdStride;
};

// Structure containing shader code in various intermediate forms.
struct PipelineShaderCode
{
    const void* pAmdilCode; // Code in AMD intermediate language form
    size_t      amdilSize;  // Size in bytes of AMDIL code

    const void* pDxilCode;  // Code in DXIL form
    size_t      dxilSize;   // Size in bytes of DXIL code

    const void* pSpvCode;   // Code in SPIR-V form
    size_t      spvSize;    // Size in bytes of SPIR-V code

    // TODO #gpurt: Fields for SPIRV and DXIL here
};

// Maximum size of a NodeMapping array for any internal pipeline
constexpr uint32 MaxInternalPipelineNodes = 16;

// Enum for internal raytracing compute shader types
enum class InternalRayTracingCsType : uint32
{
    BuildParallel,
    EncodeTriangleNodes,
    EncodeTriangleNodesIndirect,
    EncodeAABBNodes,
    EncodeInstances,
    Rebraid,
    GenerateMortonCodes,
    BuildBVH,
    BuildBVHSortLeaves,
    BuildBVHTD,
    BuildBVHTDTR,
    BuildBVHPLOC,
    UpdateQBVH,
    UpdateParallel,
    RefitBounds,
    ClearBuffer,
    CopyBufferRaw,
    InitBuildQBVH,
    BuildQBVH,
    BuildQBVHCollapse,
    BitHistogram,
    ScatterKeysAndValues,
    ScanExclusiveInt4,
    ScanExclusivePartInt4,
    ScanExclusiveInt4DLB,
    InitScanExclusiveInt4DLB,
    DistributePartSumInt4,
    EmitCurrentSize,
    EmitCompactSize,
    EmitSerializeDesc,
    EmitToolVisDesc,
    CopyAS,
    CompactAS,
    DecodeAS,
    DecodeCollapse,
    SerializeAS,
    DeserializeAS,
    InitExecuteIndirect,
    PairCompression,
    MergeSort,
    Update,
    InitAccelerationStructure,
    Count
};

// Hashed name/value pairs to pass to pipeline compiler
struct PipelineCompilerOption
{
    uint32 hashedOptionName; // hash of associated option name, see the PipelineOptionName namespace
    uint32 value;            // associated option value
};

// Contains information to build an internal gpurt pipeline
struct PipelineBuildInfo
{
    uint64                        apiPsoHash;
    const NodeMapping*            pNodes;
    uint32                        nodeCount;
    PipelineShaderCode            code;
    InternalRayTracingCsType      shaderType;
#if GPURT_DEVELOPER
    const char*                   pExperimentalCompilerOptions; // Optional null-terminated string to pass to client
                                                                // pipeline compiler
#endif
    const PipelineCompilerOption* pHashedCompilerOptions;
    uint32                        hashedCompilerOptionCount;
    const char*                   pPipelineName;
};

// Specifies the compile time constant buffer for internal pipelines
struct CompileTimeConstants
{
    const uint32* pConstants;
    uint32        numConstants;
    uint32        logicalId;
    uint32        constantBufferIndex;
};

// Acceleration structure type (analogous to e.g. D3D12DDI_RAYTRACING_ACCELERATION_STRUCTURE_TYPE)
enum class AccelStructType : uint32
{
    TopLevel    = 0, // Top-level acceleration structure
    BottomLevel = 1  // Bottom-level acceleration structure
};

static_assert(uint32(AccelStructType::TopLevel)    == 0, "Enums encoded in the acceleration structure must not change.");
static_assert(uint32(AccelStructType::BottomLevel) == 1, "Enums encoded in the acceleration structure must not change.");

// Acceleration structure builder type
enum class AccelStructBuilderType : uint32
{
    Gpu = 0, // Acceleration structure built on GPU
    Cpu = 1  // Acceleration structure built on CPU
};

static_assert(uint32(AccelStructBuilderType::Gpu) == 0, "Enums encoded in the acceleration structure must not change.");
static_assert(uint32(AccelStructBuilderType::Cpu) == 1, "Enums encoded in the acceleration structure must not change.");

// Modes for which interior box nodes in BLAS are written as fp16
enum class Fp16BoxNodesInBlasMode : uint32
{
    NoNodes,    // No interior box nodes in BLAS are fp16 (so all interior box nodes are fp32)
    LeafNodes,  // Leaf nodes (parents of triangles) are fp16, all other box nodes are fp32
    MixedNodes, // TODO: Mix of fp16 and fp32 nodes based on thresholded increase in surface area from conversion
    AllNodes,   // All interior box nodes in BLAS are fp16
    Count
};

static_assert(uint32(Fp16BoxNodesInBlasMode::NoNodes)    == 0, "Enums encoded in the acceleration structure must not change.");
static_assert(uint32(Fp16BoxNodesInBlasMode::LeafNodes)  == 1, "Enums encoded in the acceleration structure must not change.");
static_assert(uint32(Fp16BoxNodesInBlasMode::MixedNodes) == 2, "Enums encoded in the acceleration structure must not change.");
static_assert(uint32(Fp16BoxNodesInBlasMode::AllNodes)   == 3, "Enums encoded in the acceleration structure must not change.");

enum class RebraidType : uint32
{
    Off = 0, // No Rebraid
    V1  = 1, // First version of Rebraid
    V2  = 2, // Second version of Rebraid
    Count
};

// Modes for which indirect arguments
enum class ExecuteIndirectArgType : uint32
{
    DispatchDimensions              = 0,  // ray trace query dimensions
    DispatchDimenionsAndShaderTable = 1,  // ray trace query dimensions + shaderTable
};

// Triangle Compression modes
enum class TriangleCompressionMode : uint32
{
    None,  // NoTriangleCompression is defined as storing a single triangle along with the triangle
           // sideband data (geometryIndex, primitiveIndex, geometryFlags) inside the triangle node.

    Reserved,

    Pair,  // Pair compression stores up to two triangles per node, stores their sideband data (shared geometryIndex,
           // primitiveIndex0, primitiveIndex1, shared geometryFlags) inside the triangle nodes.
           // The triangles are in a node sharing a bounding box in the parent box node.

    Count
};

static_assert(uint32(TriangleCompressionMode::None)     == 0, "Enums encoded in the acceleration structure must not change.");
static_assert(uint32(TriangleCompressionMode::Reserved) == 1, "Enums encoded in the acceleration structure must not change.");
static_assert(uint32(TriangleCompressionMode::Pair)     == 2, "Enums encoded in the acceleration structure must not change.");

// Controls when pair compression is enabled based on build flags.
enum class TriangleCompressionAutoMode : uint32
{
    Disabled                   = 0, // Disables triangle compression
    AlwaysEnabled              = 1, // Enables triangle compression
    DefaultBuild               = 2, // Checks if default or fast trace is set in flags
    FastTrace                  = 3, // Checks if fast trace is set in flags
    Compaction                 = 4, // Checks if compaction is set in flags
    DefaultBuildWithCompaction = 5, // Checks if compaction and either default or fast trace is set in flags
    FastTraceWithCompaction    = 6, // Checks if compaction and fast trace is set in flags
    DefaultBuildOrCompaction   = 7, // Checks if compaction or default or fast trace is set in flags
    FastTraceOrCompaction      = 8, // Checks if compaction or fast trace is set in flags
};

// Bottom-level geometry node flags.  (analogous to e.g. D3D12DDI_RAYTRACING_GEOMETRY_FLAGS)
enum class GeometryFlag : uint32
{
    None                        = 0x0, // Dummy value
    Opaque                      = 0x1, // Geometry is opaque.  Implies no any-hit shader.
    NoDuplicateAnyHitInvocation = 0x2  // Require that an any-hit shader is invoked only once per ray-prim intersect.
};

typedef uint32 GeometryFlags;

// Type of geometry node
enum class GeometryType : uint32
{
    Triangles = 0,      // Triangle geometry.  Geometry::triangles is valid.
    Aabbs               // Procedural bounding box geometry.  Geometry::aabbs is valid.
};

// Index format for triangle geometry.
enum class IndexFormat : uint32
{
    Unknown = 0,        // Unknown (non-indexed) format
    R32_Uint,           // 32-bit unsigned int indices
    R16_Uint            // 16-bit unsigned int indices
};

// Vertex format for triangle geometry position data
enum class VertexFormat : uint32
{
    Invalid = 0,        // Invalid (illegal) value
    R32G32B32_Float,    // 32-bit floating point R32G32B32 X,Y,Z format
    R32G32_Float,       // 32-bit floating point R32G32 X,Y,0 format
    R16G16B16A16_Float, // 16-bit floating point R16G16B16X16 X,Y,Z format
    R16G16_Float,       // 16-bit floating point R16G16 X,Y,0 format
    R16G16B16A16_Snorm, // 16-bit fixed-point signed normalized R16G16B16X16 X,Y,Z format
    R16G16_Snorm,       // 16-bit fixed-point signed normalized R16G16 X,Y,0 format
    R16G16B16A16_Unorm, // 16-bit fixed-point unsigned normalized R16G16B16X16 X,Y,Z format
    R16G16_Unorm,       // 16-bit fixed-point unsigned normalized R16G16 X,Y,0 format
    R10G10B10A2_Unorm,  // 10-bit fixed-point unsigned normalized R10G10B10X2 X,Y,Z format
    R8G8B8A8_Snorm,     // 8-bit fixed-point signed normalized R8G8B8X8 X,Y,Z format
    R8G8_Snorm,         // 8-bit fixed-point signed normalized R8G8 X,Y,0 format
    R8G8B8A8_Unorm,     // 8-bit fixed-point unsigned normalized R8G8B8X8 X,Y,Z format
    R8G8_Unorm          // 8-bit fixed-point unsigned normalized R8G8 X,Y,0 format
};

// Geometry node triangle data
struct GeometryTriangles
{
    GpuCpuAddr      columnMajorTransform3x4;   // GPU/CPU address to a 3x4 matrix to be applied on source vertices during build
    IndexFormat     indexFormat;               // Index format (unknown when non-indexed)
    VertexFormat    vertexFormat;              // Vertex position format
    uint32          indexCount;                // Number of vertex indices
    uint32          vertexCount;               // Number of vertices in the vertex buffer
    GpuCpuAddr      indexBufferAddr;           // GPU/CPU index buffer base address (0 when non-indexed)
    GpuCpuAddr      vertexBufferAddr;          // GPU/CPU vertex buffer base address
    gpusize         vertexBufferByteStride;    // Stride in bytes between vertex indices
};

// Geometry node AABB data
struct GeometryAabbs
{
    uint64          aabbCount;      // Number of bounding boxes
    GpuCpuAddr      aabbAddr;       // GPU/CPU address to first bounding box
    gpusize         aabbByteStride; // Stride in bytes between consecutive bounding boxes
};

// Bottom-level geometry node information (analogous to D3D12DDI_RAYTRACING_GEOMETRY_DESC)
struct Geometry
{
    GeometryType  type;                 // Geometry type
    GeometryFlags flags;                // Geometry flags
    union
    {
        GeometryTriangles triangles;    // Triangle geometry.  Valid if type is Triangles.
        GeometryAabbs     aabbs;        // Procedural AABB geometry.  Valid if type is Aabbs.
    };
};

// Flags for building an acceleration structure (analogous to D3D12DDI_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS)
enum AccelStructBuildFlag : uint32
{
    AccelStructBuildFlagNone            = 0x00,
    AccelStructBuildFlagAllowUpdate     = 0x01,
    AccelStructBuildFlagAllowCompaction = 0x02,
    AccelStructBuildFlagPreferFastTrace = 0x04,
    AccelStructBuildFlagPreferFastBuild = 0x08,
    AccelStructBuildFlagMinimizeMemory  = 0x10,
    AccelStructBuildFlagPerformUpdate   = 0x20,
};

typedef uint32 AccelStructBuildFlags;

// Organization of input element descriptions (geometries, instances) in acceleration structure build inputs
enum class InputElementLayout : uint32
{
    Array           = 0, // Inputs are organized as a flat tightly-packed array of elements
    ArrayOfPointers = 1  // Inputs are organized as an array of pointers to elements
};

// BVH builder modes
enum class BvhBuildMode : uint32
{
    Linear   = 0, // Linear BVH builder
    Reserved = 1, // Formerly agglomerative clustering BVH builder
    PLOC     = 2, // Parallel locally-ordered clustering BVH builder
    Auto     = 4, // Used in override build to fall back to regular build options
    Count
};

static_assert(uint32(BvhBuildMode::Linear)   == 0, "Enums encoded in the acceleration structure must not change.");
static_assert(uint32(BvhBuildMode::Reserved) == 1, "Enums encoded in the acceleration structure must not change.");
static_assert(uint32(BvhBuildMode::PLOC)     == 2, "Enums encoded in the acceleration structure must not change.");

// BVH CPU builder modes
enum class BvhCpuBuildMode : uint32
{
    RecursiveSAH = 0,       // Recursive partitioning by optimal SAH cost
    RecursiveLargestExtent, // Recursive by largest centroid bound extent
};

static_assert(uint32(BvhCpuBuildMode::RecursiveSAH)           == 0,
    "Enums encoded in the acceleration structure must not change.");
static_assert(uint32(BvhCpuBuildMode::RecursiveLargestExtent) == 1,
    "Enums encoded in the acceleration structure must not change.");

// Ray tracing counter mode for a particular TraceRay dispatch, written as part of DispatchRaysConstants
enum TraceRayCounterMode : uint32
{
    TraceRayCounterDisable         = 0, // Disable counters
    TraceRayCounterRayHistoryLight = 1, // Write ray traversal history (selective)
    TraceRayCounterRayHistoryFull  = 2, // Write ray traversal history (full)
    TraceRayCounterTraversal       = 3, // Write traversal statistics counters
    TraceRayCounterCustom          = 4, // Custom logging
    TraceRayCounterDispatch        = 5, // Traversal counters accumulated across entire dispatch
};

// Acceleration structure build-time information about the acceleration structure's indirect buffers
struct AccelStructBuildInputsIndirect
{
    gpusize indirectGpuAddr;
    uint32  indirectStride;
};

// Instance node flags (analogous to D3D12_RAYTRACING_INSTANCE_FLAGS)
enum class InstanceFlag : uint32
{
    None                          = 0x0,
    TriangleCullDisable           = 0x1,
    TriangleFrontCounterclockwise = 0x2,
    ForceOpaque                   = 0x4,
    ForceNonOpaque                = 0x8
};

// GPURT version of the API instance description in GPU memory.  It should match e.g. D3D12DDI_RAYTRACING_INSTANCE_DESC
struct InstanceDesc
{
    float  transform[3][4];
    uint32 instanceID : 24;
    uint32 instanceMask : 8;
    uint32 instanceContributionToHitGroupIndex : 24;
    uint32 flags : 8;
    uint64 accelerationStructure;
};

// Information about a bottom-level AS reference that is required during host TLAS builds.  Clients provide this
// either explicitly or via a callback converter.
struct InstanceBottomLevelInfo
{
    InstanceDesc desc;      // Instance description.  The accelerationStructure field should be the GPU address
                            // to BLAS data.
    const void*  pCpuAddr;  // Host-visible CPU address to start of BLAS data
};

// Acceleration structure build-time information about the acceleration structure's inputs
struct AccelStructBuildInputs
{
    AccelStructType       type;            // Acceleration structure type (top- or bottom-level)
    AccelStructBuildFlags flags;           // Build-time flags
    uint32                inputElemCount;  // Number of input elements (geometries, instances)
    InputElementLayout    inputElemLayout; // Organization of input elements

    union
    {
        GpuCpuAddr        instances;       // GPU/CPU address to raytracing InstanceDesc elements in memory
        const void*       pGeometries;     // Client-provided opaque data pointer that will be decoded during
                                           // build by calling the client-provided function
                                           // ClientConvertAccelStructBuildGeometry().
    };

    void*                 pClientData;     // Arbitrary client data context that is passed back to during geometry
                                           // conversion and dumping events.
};

// This enum describes what kind of postbuild information to emit.
enum class AccelStructPostBuildInfoType : uint32
{
    CompactedSize               = 0x0, // Size of the accel struct after compaction.
    ToolsVisualization          = 0x1, // Space required to decode the acceleration structure into tools visualized form.
    Serialization               = 0x2, // Space required to serialize an acceleration structure
    CurrentSize                 = 0x3, // Current space required by an acceleration structure
    BottomLevelASPointerCount   = 0x4  // Count of bottom-level acceleration structure pointers
};

// Description for a post build info event emission.
struct AccelStructPostBuildInfoDesc
{
    RwGpuCpuAddr                 postBuildBufferAddr; // GPU/CPU postbuild info address
    AccelStructPostBuildInfoType infoType;            // Type of information to be written
};

// Acceleration structure build information
struct AccelStructBuildInfo
{
    gpusize                        dstAccelStructGpuAddr;  // GPU address of destination acceleration structure
    void*                          pDstAccelStructCpuAddr; // CPU address of dest. acceleration structure (CPU build only)
    AccelStructBuildInputs         inputs;                 // Acceleration structure inputs
    AccelStructBuildInputsIndirect indirect;               // Indirect Arguments
    gpusize                        srcAccelStructGpuAddr;  // Source acceleration structure if updating
    const void*                    pSrcAccelStructCpuAddr; // CPU address of src. acceleration structure (CPU build only)
    RwGpuCpuAddr                   scratchAddr;            // GPU/CPU acceleration structure scratch space
    uint32                         postBuildInfoDescCount; // Number of postbuild information entries
    const void*                    pPostBuildInfoDescs;    // Client-provided opaque data pointer to postbuild emit requests in
                                                           // client API form. These will be translated at build time by
                                                           // calling the client-provided function
                                                           // ClientConvertAccelStructPostBuildInfo().
};

// GPURT version of the API AABB description in GPU memory.  It should match e.g. D3D12DDI_RAYTRACING_AABB.
struct Aabb
{
    float minX;
    float minY;
    float minZ;
    float maxX;
    float maxY;
    float maxZ;
};

// Struct containing various relevant prebuild information for an acceleration structure built from
// some given inputs.
struct AccelStructPrebuildInfo
{
    uint64 resultDataMaxSizeInBytes;      // Video memory required to build an acceleration structure
    uint64 scratchDataSizeInBytes;        // Amount of temporary memory required to build an acceleration structure
    uint64 updateScratchDataSizeInBytes;  // Amount of temporary memory required for future updates of that struct

    uint32 maxPrimitiveCount;             // Max primitive count
};

// Device - wide settings for GPURT
struct DeviceSettings
{
    uint32                      rgpBarrierReason;

    BvhBuildMode                bvhBuildModeDefault;                  // Bvh build mode for default builds.
    BvhBuildMode                bvhBuildModeFastTrace;                // Bvh build mode for fast trace builds.
    BvhBuildMode                bvhBuildModeFastBuild;                // Bvh build mode for fast builds.
    BvhCpuBuildMode             bvhCpuBuildModeDefault;               // Bvh CPU build mode for default builds.
    BvhCpuBuildMode             bvhCpuBuildModeFastTrace;             // Bvh CPU build mode for fast trace builds.
    BvhCpuBuildMode             bvhCpuBuildModeFastBuild;             // Bvh CPU build mode for fast builds.

    BvhBuildMode                bvhBuildModeOverrideBLAS;             // Bvh build mode for BLAS if override is enabled.
    BvhBuildMode                bvhBuildModeOverrideTLAS;             // Bvh build mode for TLAS if override is enabled.

    TriangleCompressionAutoMode triangleCompressionAutoMode;
    float                       triangleSplittingFactor;
    uint32                      tsBudgetPerTriangle;
    float                       tsPriority;

    RebraidType                 rebraidType;                          // Tree rebraid in TLAS
    uint32                      rebraidFactor;                        // Rebraid factor
    float                       rebraidLengthPercentage;              // Rebraid length percentage
    uint32                      plocRadius;                           // PLOC Radius
    uint32                      maxTopDownBuildInstances;             // Max instances allowed for top down build
    uint32                      parallelBuildWavesPerSimd;            // Waves per SIMD to launch for parallel build

    uint32                      fastBuildThreshold;                   // Apply Fast Build for BVHs with primitives <= Threshold.
    uint32                      lbvhBuildThreshold;                   // Apply LBVH for BVHs with primitives <= Threshold.

    Fp16BoxNodesInBlasMode      fp16BoxNodesInBlasMode;               // Mode for which interior nodes in BLAS are FP16
    float                       fp16BoxModeMixedSaThresh;             // For fp16 mode "mixed", surface area threshold

    Pal::RayTracingIpLevel      emulatedRtIpLevel;                    // Client request RTIP level, used to override IP level related GPURT settings.

    struct
    {
        uint32 enableFusedInstanceNode : 1;
        uint32 enableMortonCode30 : 1;
        uint32 enableVariableBitsMortonCodes : 1;
        uint32 enableParallelUpdate : 1;
        uint32 enableParallelBuild : 1;
        uint32 enablePrefixScanDLB : 1;
        uint32 enableAcquireReleaseInterface : 1;
        uint32 enableBuildAccelStructDumping : 1;
        uint32 enableBuildAccelStructScratchDumping : 1;
        uint32 enableBuildAccelStructStats : 1;
        uint32 enableTriangleSplitting : 1;
        uint32 enableBVHBuildDebugCounters : 1;
        uint32 enableInsertBarriersInBuildAS : 1;
        uint32 enablePairCompressionCostCheck : 1;
        uint32 enableMergeSort : 1;
        uint32 bvhCollapse : 1;                             // Collapse individual geometry leaf nodes into multi-geometry leaves
        uint32 topDownBuild : 1;                            // Top down build in TLAS
        uint32 allowFp16BoxNodesInUpdatableBvh : 1;         // Allow box node in updatable bvh.
        uint32 fp16BoxNodesRequireCompaction : 1;           // Compaction is set or not.
        uint32 noCopySortedNodes : 1;                       // Disable CopyUnsortedScratchLeafNode()
        uint32 enableSAHCost : 1;                           // Use more accurate SAH cost
        uint32 enableMergedEncodeUpdate : 1;                // Combine encode and update in one dispatch
        uint32 enableMergedEncodeBuild : 1;                 // Combine encode and build in one dispatch
        uint32 enableEarlyPairCompression : 1;              // Enable pair triangle compression in early (Encode) phase

        uint32 enableFastLBVH : 1;                          // Enable the Fast LBVH path
    };

    uint64                      accelerationStructureUUID;  // Acceleration Structure UUID

    uint32                      numMortonSizeBits;
    uint32                      trianglePairingSearchRadius; // The search radius for paired triangle, used by EarlyCompression

    uint32                      gpuDebugFlags;

};

// Describes a postbuild info write request from some acceleration structures to some location.
struct AccelStructPostBuildInfo
{
    AccelStructPostBuildInfoDesc desc;                      // Description of the postbuild info
    uint32                       srcAccelStructCount;       // Number of accel structs for which info is written
    const gpusize*               pSrcAccelStructGpuAddrs;   // GPU addresses of accel structs
    const void*                  pSrcAccelStructCpuAddrs;   // CPU addresses of accel structs (CPU build only)
};

// GPU memory layout of a AccelStructPostBuildInfoType::CompactedSize postbuild emit.  Should match
// D3D12DDI_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_COMPACTED_SIZE_DESC
struct AccelStructPostBuildInfoCompactedSizeDesc
{
    uint64 compactedSizeInBytes;
};

// GPU memory layout of a AccelStructPostBuildInfoType::ToolsVisualization postbuild emit.  Should match
// D3D12DDI_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_TOOLS_VISUALIZATION_DESC
struct AccelStructPostBuildInfoToolsVisualizationDesc
{
    uint64 decodedSizeInBytes;
};

// GPU memory layout of a AccelStructPostBuildInfoType::Serialization postbuild emit.  Should match
// D3D12DDI_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_SERIALIZATION_DESC
struct AccelStructPostBuildInfoSerializationDesc
{
    uint64 serializedSizeInBytes;
    uint64 numBottomLevelAccelerationStructurePointers;
};

// GPU memory layout of a AccelStructPostBuildInfoType::Serialization postbuild emit.  Should match
// D3D12DDI_RAYTRACING_ACCELERATION_STRUCTURE_POSTBUILD_INFO_CURRENT_DESC
struct AccelStructPostBuildInfoCurrentSizeDesc
{
    uint64 currentSizeInBytes;
};

// Copy mode for acceleration structures
enum class AccelStructCopyMode : uint32
{
    Clone                       = 0x0, // Clone the acceleration structure
    Compact                     = 0x1, // Create a compacted version of an acceleration structure
    VisualizationDecodeForTools = 0x2, // Decode a PIX-visualizable version of the acceleration structure
    Serialize                   = 0x3, // Serialize the acceleration structure
    Deserialize                 = 0x4, // Deserialize the acceleration structure
    DriverDecode                = 0x5, // Same as VisualizationDecodeForTools with extended header
};

// Info structure for acceleration structure copy operations
struct AccelStructCopyInfo
{
    RwGpuCpuAddr        dstAccelStructAddr;     // Destination accel struct GPU/CPU address
    GpuCpuAddr          srcAccelStructAddr;     // Source accel struct GPU/CPU address
    AccelStructCopyMode mode;                   // Copy operation mode
};

// Information about acceleration structure operations
struct AccelStructInfo
{
    AccelStructType         type;                    // Top or Bottom
    uint32                  numDesc;                 // Number of input elements (geometries, instances)
    uint32                  numPrimitives;           // Num Primitives in the acceleration structure
    AccelStructBuildFlags   buildFlags;              // Build Flags
    AccelStructBuilderType  buildType;               // Acceleration structure builder type (CPU or GPU)
    uint32                  buildMode;               // What mode the acceleration structure was built with
    TriangleCompressionMode triangleCompressionMode; // What mode triangles are being compressed with
    Fp16BoxNodesInBlasMode  fp16BoxNodesInBlasMode;  // What fp16BoxNodes mode is in blas
    uint64                  gpuVa;                   // Result buffer GPU VA
    uint32                  sizeInBytes;             // Result buffer size in bytes
    uint64                  scratchGpuVa;            // Scratch buffer GPU VA
    uint32                  scratchSizeInBytes;      // Scratch buffer size in bytes
    Pal::IGpuMemory*        pTimeStampVidMem;        // Pointer to time stamp memory
    uint64                  timeStampVidMemoffset;   // Offset into time stamp memory
};

// AMD GUID
#define GPURT_AMD_GUID_0    0x445D18EA
#define GPURT_AMD_GUID_1    0xB42547D8
#define GPURT_AMD_GUID_2    0x867BA9A4
#define GPURT_AMD_GUID_3    0x496A1A2E

struct DataDriverMatchingIdentifier
{
    uint8 driverOpaqueGUID[16];
    uint8 driverOpaqueVersioningData[16];
};

struct SerializedAccelStructHeader
{
    DataDriverMatchingIdentifier driverMatchingIdentifier;
    uint64 serializedSizeInBytesIncludingHeader;
    uint64 deserializedSizeInBytes;
    uint64 numBottomLevelAccelerationStructurePointersAfterHeader;
};

enum class DataDriverMatchingIdentifierStatus : uint32
{
    CompatibleWithDevice = 0,    // The data is compatible with the current device/driver.
    Unrecognized = 0x1,          // The data may be corrupt or produced by a different hardware vendor.
    IncompatibleVersion = 0x2,   // The data may be produced by an incompatible version of driver.
};

// Entry function table containing entry symbols. This structure is filled out by GPURT in response
// to a call to QueryRayTracingEntryFunctionTable
struct EntryFunctionTable
{
    struct
    {
        const char* pProceed;
        const char* pTraceRayInline;
        const char* pGet64BitInstanceNodePtr;
    } rayQuery;

    struct
    {
        const char* pTraceRay;
        const char* pTraceRayUsingHitToken;
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 37
        const char* pTraceRayUsingRayQuery;
#endif
    } traceRay;

    struct
    {
        const char* pGetInstanceID;
        const char* pGetInstanceIndex;
        const char* pGetObjectToWorldTransform;
        const char* pGetWorldToObjectTransform;
        const char* pFetchTrianglePositionFromNodePointer;
        const char* pFetchTrianglePositionFromRayQuery;
    } intrinsic;
};

// Input flags to enable/disable GPURT shader library features
enum class ShaderLibraryFeatureFlag : uint32
{
    Developer         = 0x1,
    SoftwareTraversal = 0x2,
};

typedef uint32 ShaderLibraryFeatureFlags;

// Data structure used by the traversal shader to log used acceleration structures for dumping purposes.
struct AccelStructTracker
{
    uint32 count;                // Number of addresses logged so far
    uint32 lock;                 // Spin lock protecting the list of addresses
    uint32 enabled;              // Non-zero if tracking is enabled
    uint32 outOfSpace;           // Non-zero if the tracker ran out of space to store an acceleration structure
    uint64 accelStructAddrs[16]; // List of unique acceleration structures
};

// Information to initialize a gpurt device
struct DeviceInitInfo
{
    uint32            gpuIdx;                       // Client GPU index associated with this gpurt device
    void*             pClientUserData;              // User data pointer passed to internal pipeline create/destroy callbacks

    const Pal::DeviceProperties* pDeviceProperties; // Pointer to host PAL device properties
                                                    // (this pointer is retained).
    DeviceSettings    deviceSettings;               // Settings Used to initialize parts of GlobalSettings

    union
    {
        struct
        {
            uint32 reserved              : 32;
        };
        uint32 u32All;
    } flags;

    Pal::IDevice* pPalDevice;
    Pal::IPlatform* pPalPlatform;

    AccelStructTracker* pAccelStructTracker; // Acceleration structure tracker memory allocated by the client
    gpusize      accelStructTrackerGpuAddr;  // Address of tracker from gpu memory.
};

// Raytracing shader identifier (GPU structure)
struct ShaderIdentifier
{
    uint64 shaderId;       // Generic shader ID for RayGen, ClosestHit, Miss, and Callable
    uint64 anyHitId;       // AnyHit ID for hit groups
    uint64 intersectionId; // Intersection shader ID for hit groups
    uint64 padding;        // Padding to meet 32-byte api requirement and 8-byte alignment for descriptor table offset
};

static_assert(sizeof(ShaderIdentifier) == RayTraceShaderIdentifierByteSize, "");

// Map key for map of internal pipelines
struct InternalPipelineKey
{
    InternalRayTracingCsType shaderType;
    uint32                   settingsHash;
};

struct InternalPipelineMemoryPair
{
    Pal::IPipeline* pPipeline;
    void*           pMemory;
};

// Box sorting heuristic
enum class BoxSortHeuristic : uint32
{
    Closest = 0x0,
#if GPURT_BUILD_RTIP2
    Largest = 0x1,
    MidPoint = 0x2,
#endif
    Disabled = 0x3,
#if GPURT_BUILD_RTIP2
    LargestFirstOrClosest = 0x4,
    LargestFirstOrClosestMidPoint = 0x5,
#endif
    DisabledOnAcceptFirstHit = 0x6
};

using InternalPipelineMap = Util::HashMap<
    InternalPipelineKey,
    InternalPipelineMemoryPair,
    Util::GenericAllocatorTracked,
    Util::JenkinsHashFunc,
    Util::DefaultEqualFunc,
    Util::HashAllocator<Util::GenericAllocatorTracked>,
    sizeof(InternalPipelineMemoryPair) * 16>;

// Ray history trace
enum class RtPipelineType
{
    RayTracing,
    Compute,
    Graphics, // Not currently supported for ray history capture
};

struct ShaderTable
{
    gpusize addr;
    gpusize size;
    gpusize stride;
};

struct RtDispatchInfo
{
    uint32 dimX;
    uint32 dimY;
    uint32 dimZ;

#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION >= 38
    uint32 threadGroupSizeX;
    uint32 threadGroupSizeY;
    uint32 threadGroupSizeZ;
#endif
    uint32 pipelineShaderCount;
    uint64 stateObjectHash;

    uint32 boxSortMode;
    uint32 usesNodePtrFlags;

    ShaderTable raygenShaderTable;
    ShaderTable missShaderTable;
    ShaderTable hitGroupTable;
    ShaderTable callableShaderTable;
};

#if GPURT_DEVELOPER
// Client-defined callback to push or pop an RGP marker through the given command buffer.  These are used to
// annotate GPURT operations.
//
// @param pCmdBuffer [in] PAL command buffer passed to BVH operation
// @param pMarker    [in] Marker string when isPush is true, null otherwise
// @param isPush     [in] Whether this is a marker push or a pop operation.
extern void ClientInsertRGPMarker(
    Pal::ICmdBuffer* pCmdBuffer,
    const char*      pMarker,
    bool             isPush);
#endif

// Client-provided callback to convert some i-th bottom-level geometry description from client API-specific struct
// to the GPURT struct. This function is called during acceleration struct builds.
//
// Setting this can be useful for clients that want to avoid temporary memory allocation just to rename arbitrary
// enums.
//
// @param inputs        [in] The main build input structure
// @param geometryIndex [in] The geometry index whose description should be converted
//
// @returns A GPURT Geometry struct containing the converted data
extern Geometry ClientConvertAccelStructBuildGeometry(
    const AccelStructBuildInputs& inputs,
    uint32                        geometryIndex);

// Client-provided callback to convert some i-th entry in the set of top-level instances from client API-specific
// format to relevant GPURT data.
//
// This function is only used by host builds and clients that do not support host builds can simply implement a
// stub function.
//
// When called, the instanceIndex identifies the instance corresponding with the same element of the
// AccelStructBuildInputs::instances instance description array.  The client needs to return information about
// the referenced BLAS, such as the BLAS GPU address and the host-mapped CPU address to that data (the client-
// specific acceleration structure handle may not be the BLAS GPU address during host builds).
//
// @param inputs        [in] The main build input structure
// @param instanceIndex [in] The instance index whose description should be converted
//
// @returns A GPURT InstanceBottomLevelInfo struct containing the converted data
extern InstanceBottomLevelInfo ClientConvertAccelStructBuildInstanceBottomLevel(
    const AccelStructBuildInputs& inputs,
    uint32                        instanceIndex);

// Client-provided callback to convert some i-th postbuild info description from client API-specific struct
// to the GPURT struct. This function is called during acceleration struct builds.
//
// Setting this can be useful for clients that want to avoid temporary memory allocation just to rename arbitrary
// enums.
//
// @param buildInfo     [in] The main build info structure
// @param postBuildndex [in] The postbuild index whose description should be converted
//
// @returns A GPURT Geometry struct containing the converted data
extern AccelStructPostBuildInfo ClientConvertAccelStructPostBuildInfo(
    const AccelStructBuildInfo& buildInfo,
    uint32                      postBuildIndex);

// Client-provided optional function that is called preceding an acceleration structure build-time dump operation.
//
// The client must allocate video memory that meets the size needs given in the info structure and return the GPU VA
// of that memory.  It can assume that the relevant dump commands are inserted into the given command buffer by GPURT
// if this function returns successfully.  The client can then retain this video memory and process the dump in
// a way it sees fit.
//
// @param pCmdBuffer       [in] PAL command buffer that will contain the dump commands
// @param info             [in] Information about the acceleration structure being dumped
// @param pDumpGpuVirtAddr [out] GPU virtual address of memory allocated by this function to store the dumped BVH
//
// @returns Pal::Result::Success if allocation succeeded and the dump should be handled; any error otherwise.
extern Pal::Result ClientAccelStructBuildDumpEvent(
    Pal::ICmdBuffer*            pCmdBuffer,
    const AccelStructInfo&      info,
    const AccelStructBuildInfo& buildInfo,
    gpusize*                    pDumpGpuVirtAddr);

// Client-provided optional function that is called preceding an acceleration structure build-time dump operation.
//
// The client must allocate video memory that meets the size needs given in the info structure and return the GPU VA
// of that memory.  It can assume that the relevant dump commands are inserted into the given command buffer by GPURT
// if this function returns successfully.  The client can then retain this video memory and process the dump in
// a way it sees fit.
//
// @param pCmdBuffer       [in] PAL command buffer that will contain the dump commands
// @param info             [in] Information about the acceleration structure being dumped
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 39
// @param gpuMem          [out] GPU memory allocated by this function to store the timestamps
// @param offset           [out] offset into GPU memory to store the timestamps
#endif
//
// @returns Pal::Result::Success if allocation succeeded and the dump should be handled; any error otherwise.
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION >= 39
extern Pal::Result ClientAccelStatsBuildDumpEvent(
    Pal::ICmdBuffer*        pCmdbuf, // PAL command buffer that will handle the dump
    GpuRt::AccelStructInfo* pInfo);  // Information about the accel struct being dumped
#else
extern Pal::Result ClientAccelStatsBuildDumpEvent(
    Pal::ICmdBuffer*              pCmdbuf,               // PAL command buffer that will handle the dump
    const GpuRt::AccelStructInfo& info,                  // Information about the accel struct being dumped
    Pal::IGpuMemory**             ppGpuMem,              // Pointer time stamp memory
    uint64*                       pOffset);              // Offset into time stamp memory
#endif

// Client-provided callback to build an internal compute pipeline.  This is called by gpurt during initialization
// of a gpurt device.
//
// The client must implement this function to successfully initialize gpurt.
//
// @param initInfo         [in]  Information about the host device
// @param buildInfo        [in]  Information about the pipeline to be built
// @param ppResultPipeline [out] Result PAL pipeline object pointer
// @param ppResultMemory   [out] Result PAL pipeline memory if different from pipeline pointer.  Optional.
//
// @returns Compilation success result.
extern Pal::Result ClientCreateInternalComputePipeline(
    const DeviceInitInfo&       initInfo,          // Information about the host device
    const PipelineBuildInfo&    buildInfo,         // Information about the pipeline to be built
    const CompileTimeConstants& constantInfo,      // Compile time constants
    Pal::IPipeline**            ppResultPipeline,  // Result PAL pipeline object pointer
    void**                      ppResultMemory);   // (Optional) Result PAL pipeline memory, if different from obj

// Client-provided callback to destroy an internal compute pipeline.  This is called by gpurt during device destroy.
//
// The client must implement this function to successfully initialize gpurt.
//
// @param initInfo  [in] Information about the host device
// @param pPipeline [in] Pipeline to be destroyed
// @param pMemory   [in] Memory previously allocated to the pipeline (may be different based on client needs)
extern void ClientDestroyInternalComputePipeline(
    const DeviceInitInfo& initInfo,
    Pal::IPipeline*       pPipeline,
    void*                 pMemory);

// Acquires a command context for use by GPURT. The same context may be returned on each call,
// so it must be used in a thread safe manner.
extern Pal::Result ClientAcquireCmdContext(
    const DeviceInitInfo&   initInfo,      // GpuRt device info
    ClientCmdContextHandle* pContext,      // (out) Opaque command context handle
    Pal::ICmdBuffer**       ppCmdBuffer);  // (out) Command buffer for GPURT to fill

// Client-provided function to submit the context's command buffer and wait for completion.
extern Pal::Result ClientFlushCmdContext(
    ClientCmdContextHandle context);

// Client-provided function to allocate gpu memory
extern Pal::Result ClientAllocateGpuMemory(
    const DeviceInitInfo& initInfo,                 // GpuRt device info
    uint64                sizeInBytes,              // Buffer size in bytes
    ClientGpuMemHandle*   pGpuMem,                  // (out) GPU video memory
    gpusize*              pDestGpuVa,               // (out) Buffer GPU VA
    void**                ppMappedData = nullptr);  // (out) Map data

// Free Gpu Memory
extern void ClientFreeGpuMem(
    const DeviceInitInfo& initInfo,
    ClientGpuMemHandle    gpuMem);

#if GPURT_DEVELOPER
// Client-defined callback to push or pop an RGP marker through the given command buffer.  These are used to
// annotate GPURT operations.
//
// @param pCmdBuffer [in] PAL command buffer passed to BVH operation
// @param pMarker    [in] Marker string when isPush is true, null otherwise
// @param isPush     [in] Whether this is a marker push or a pop operation.
typedef void (*FnClientInsertRGPMarker)(
    Pal::ICmdBuffer* pCmdBuffer,
    const char*      pMarker,
    bool             isPush);
#endif

// Client-provided callback to convert some i-th bottom-level geometry description from client API-specific struct
// to the GPURT struct. This function is called during acceleration struct builds.
//
// Setting this can be useful for clients that want to avoid temporary memory allocation just to rename arbitrary
// enums.
//
// @param inputs        [in] The main build input structure
// @param geometryIndex [in] The geometry index whose description should be converted
//
// @returns A GPURT Geometry struct containing the converted data
typedef Geometry (*FnClientConvertAccelStructBuildGeometry)(
    const AccelStructBuildInputs& inputs,
    uint32                        geometryIndex);

// Client-provided callback to convert some i-th entry in the set of top-level instances from client API-specific
// format to relevant GPURT data.
//
// This function is only used by host builds and clients that do not support host builds can simply implement a
// stub function.
//
// When called, the instanceIndex identifies the instance corresponding with the same element of the
// AccelStructBuildInputs::instances instance description array.  The client needs to return information about
// the referenced BLAS, such as the BLAS GPU address and the host-mapped CPU address to that data (the client-
// specific acceleration structure handle may not be the BLAS GPU address during host builds).
//
// @param inputs        [in] The main build input structure
// @param instanceIndex [in] The instance index whose description should be converted
//
// @returns A GPURT InstanceBottomLevelInfo struct containing the converted data
typedef InstanceBottomLevelInfo (*FnClientConvertAccelStructBuildInstanceBottomLevel)(
    const AccelStructBuildInputs& inputs,
    uint32                        instanceIndex);

// Client-provided callback to convert some i-th postbuild info description from client API-specific struct
// to the GPURT struct. This function is called during acceleration struct builds.
//
// Setting this can be useful for clients that want to avoid temporary memory allocation just to rename arbitrary
// enums.
//
// @param buildInfo     [in] The main build info structure
// @param postBuildndex [in] The postbuild index whose description should be converted
//
// @returns A GPUR Geometry struct containing the converted data
typedef AccelStructPostBuildInfo (*FnClientConvertAccelStructPostBuildInfo)(
    const AccelStructBuildInfo& buildInfo,
    uint32                      postBuildIndex);

// Client-provided optional function that is called preceding an acceleration structure build-time dump operation.
//
// The client must allocate video memory that meets the size needs given in the info structure and return the GPU VA
// of that memory.  It can assume that the relevant dump commands are inserted into the given command buffer by GPURT
// if this function returns successfully.  The client can then retain this video memory and process the dump in
// a way it sees fit.
//
// @param pCmdBuffer       [in] PAL command buffer that will contain the dump commands
// @param info             [in] Information about the acceleration structure being dumped
// @param pDumpGpuVirtAddr [out] GPU virtual address of memory allocated by this function to store the dumped BVH
//
// @returns Pal::Result::Success if allocation succeeded and the dump should be handled; any error otherwise.
typedef Pal::Result (*FnClientAccelStructBuildDumpEvent)(
    Pal::ICmdBuffer*            pCmdBuffer,
    const AccelStructInfo&      info,
    const AccelStructBuildInfo& buildInfo,
    gpusize*                    pDumpGpuVirtAddr);

// Client-provided optional function that is called preceding an acceleration structure build-time dump operation.
//
// The client must allocate video memory that meets the size needs given in the info structure and return the GPU VA
// of that memory.  It can assume that the relevant dump commands are inserted into the given command buffer by GPURT
// if this function returns successfully.  The client can then retain this video memory and process the dump in
// a way it sees fit.
//
// @param pCmdBuffer       [in] PAL command buffer that will contain the dump commands
// @param info             [in] Information about the acceleration structure being dumped
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 39
// @param gpuMem          [out] GPU memory allocated by this function to store the timestamps
// @param offset           [out] offset into GPU memory to store the timestamps
#endif
//
// @returns Pal::Result::Success if allocation succeeded and the dump should be handled; any error otherwise.
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION >= 39
typedef Pal::Result (*FnClientAccelStatsBuildDumpEvent)(
    Pal::ICmdBuffer*        pCmdbuf, // PAL command buffer that will handle the dump
    GpuRt::AccelStructInfo* pInfo);  // Information about the accel struct being dumped
#else
typedef Pal::Result (*FnClientAccelStatsBuildDumpEvent)(
    Pal::ICmdBuffer*              pCmdbuf,               // PAL command buffer that will handle the dump
    const GpuRt::AccelStructInfo& info,                  // Information about the accel struct being dumped
    Pal::IGpuMemory**             ppGpuMem,              // Pointer time stamp memory
    uint64*                       pOffset);              // Offset into time stamp memory
#endif

// Client-provided callback to build an internal compute pipeline.  This is called by gpurt during initialization
// of a gpurt device.
//
// The client must implement this function to successfully initialize gpurt.
//
// @param initInfo         [in]  Information about the host device
// @param buildInfo        [in]  Information about the pipeline to be built
// @param ppResultPipeline [out] Result PAL pipeline object pointer
// @param ppResultMemory   [out] Result PAL pipeline memory if different from pipeline pointer.  Optional.
//
// @returns Compilation success result.
typedef Pal::Result (*FnClientCreateInternalComputePipeline)(
    const DeviceInitInfo&       initInfo,          // Information about the host device
    const PipelineBuildInfo&    buildInfo,         // Information about the pipeline to be built
    const CompileTimeConstants& constantInfo,      // Compile time constants
    Pal::IPipeline**            ppResultPipeline,  // Result PAL pipeline object pointer
    void**                      ppResultMemory);   // (Optional) Result PAL pipeline memory, if different from obj

// Client-provided callback to destroy an internal compute pipeline.  This is called by gpurt during device destroy.
//
// The client must implement this function to successfully initialize gpurt.
//
// @param initInfo  [in] Information about the host device
// @param pPipeline [in] Pipeline to be destroyed
// @param pMemory   [in] Memory previously allocated to the pipeline (may be different based on client needs)
typedef void (*FnClientDestroyInternalComputePipeline)(
    const DeviceInitInfo& initInfo,
    Pal::IPipeline*       pPipeline,
    void*                 pMemory);

// Acquires a command context for use by GPURT. The same context may be returned on each call,
// so it must be used in a thread safe manner.
typedef Pal::Result (*FnClientAcquireCmdContext)(
    const DeviceInitInfo&   initInfo,      // GpuRt device info
    ClientCmdContextHandle* pContext,      // (out) Opaque command context handle
    Pal::ICmdBuffer**       ppCmdBuffer);  // (out) Command buffer for GPURT to fill

// Client-provided function to submit the context's command buffer and wait for completion.
typedef Pal::Result (*FnClientFlushCmdContext)(
    ClientCmdContextHandle context);

// Client-provided function to allocate gpu memory
typedef Pal::Result (*FnClientAllocateGpuMemory)(
    const DeviceInitInfo& initInfo,                 // GpuRt device info
    uint64                sizeInBytes,              // Buffer size in bytes
    ClientGpuMemHandle*   pGpuMem,                  // (out) GPU video memory
    gpusize*              pDestGpuVa,               // (out) Buffer GPU VA
    void**                ppMappedData);            // (out) Map data

// Free Gpu Memory
typedef void (*FnClientFreeGpuMem)(
    const DeviceInitInfo& initInfo,
    ClientGpuMemHandle    gpuMem);

// =====================================================================================================================
// Client callback function pointers
struct ClientCallbacks
{
#if GPURT_DEVELOPER
    FnClientInsertRGPMarker pfnInsertRGPMarker;
#endif

    FnClientConvertAccelStructBuildGeometry pfnConvertAccelStructBuildGeometry;
    FnClientConvertAccelStructBuildInstanceBottomLevel pfnConvertAccelStructBuildInstanceBottomLevel;
    FnClientConvertAccelStructPostBuildInfo pfnConvertAccelStructPostBuildInfo;
    FnClientAccelStructBuildDumpEvent pfnAccelStructBuildDumpEvent;
    FnClientAccelStatsBuildDumpEvent pfnAccelStatsBuildDumpEvent;
    FnClientCreateInternalComputePipeline pfnCreateInternalComputePipeline;
    FnClientDestroyInternalComputePipeline pfnDestroyInternalComputePipeline;
    FnClientAcquireCmdContext pfnAcquireCmdContext;
    FnClientFlushCmdContext pfnFlushCmdContext;
    FnClientAllocateGpuMemory pfnAllocateGpuMemory;
    FnClientFreeGpuMem pfnFreeGpuMem;
};

class IDevice;

// =====================================================================================================================
// Create GPURT device
//
Pal::Result GPURT_API_ENTRY CreateDevice(
    const DeviceInitInfo&  info,
    const ClientCallbacks& callbacks,
    void*            const pMemory,
    IDevice**        const ppDevice);

// =====================================================================================================================
size_t GPURT_API_ENTRY GetDeviceSize();

// =====================================================================================================================
// Get GPURT shader library data
//
// @param flags [in] Feature flags to enable in GPURT library
//
// @return Shader code for the shader library
//
PipelineShaderCode GPURT_API_ENTRY GetShaderLibraryCode(
    ShaderLibraryFeatureFlags flags);

// =====================================================================================================================
// Returns GPURT shader library function table for input ray tracing IP level.
//
// @param rayTracingIpLevel   [in]  Pal IP level
// @param pEntryFunctionTable [out] Requested function table, if found
//
// @return whether the function table was found successfully
Pal::Result GPURT_API_ENTRY QueryRayTracingEntryFunctionTable(
    const Pal::RayTracingIpLevel   rayTracingIpLevel,
    EntryFunctionTable* const      pEntryFunctionTable);

// =====================================================================================================================
// GPURT device
//
// =====================================================================================================================
class IDevice
{
public:

    // Initializes the device
    //
    // @returns Initialization success status
    virtual Pal::Result Init() = 0;

    // Destroy device
    //
    virtual void Destroy() = 0;

    // Returns GPURT shader library function table for input ray tracing IP level.
    //
    // @param rayTracingIpLevel   [in]  Pal IP level
    // @param pEntryFunctionTable [out] Requested function table, if found
    //
    // @return whether the function table was found successfully
    virtual Pal::Result QueryRayTracingEntryFunctionTable(
        const Pal::RayTracingIpLevel   rayTracingIpLevel,
        EntryFunctionTable* const      pEntryFunctionTable) = 0;

    // Returns the static pipeline mask shader constant values for a particular pipeline given a compatible
    // AS memory layout parameters.
    //
    // @param skipTriangles             (in) Force skip all triangle intersections
    // @param skipProceduralPrims       (in) Force skip all procedural AABB intersections
    // @param usePointerFlags           (in) Node pointer contains embedded flags
    // @param enableAccelStructTracking (in) Enable AccelStruct tracking
    // @param enableTraversalCounter    (in) Enable Traversal Counter
    //
    // @return Static pipeline mask literal
    //
    // This is the value that drivers must return back to the shader by implementing the AmdTraceRayGetStaticFlags()
    // driver stub.
    virtual uint32 GetStaticPipelineFlags(
        bool  skipTriangles,
        bool  skipProceduralPrims,
#if GPURT_CLIENT_INTERFACE_MAJOR_VERSION < 37
        bool  unused0,
#endif
        bool  enableAccelStructTracking,
        bool  enableTraversalCounter) = 0;

    // Writes commands into a command buffer to build an acceleration structure
    //
    // @param pCmdBuffer           [in] Command buffer where commands will be written
    // @param buildInfo            [in] Acceleration structure build info
    virtual void BuildAccelStruct(
        Pal::ICmdBuffer*              pCmdBuffer,
        const AccelStructBuildInfo&   buildInfo
    )= 0;

    // Writes commands into a command buffer to build multiple acceleration structures
    //
    // @param pCmdBuffer           [in] Command buffer where commands will be written
    // @param buildInfo            [in] Acceleration structure build info
    virtual void BuildAccelStructs(
        Pal::ICmdBuffer*                       pCmdBuffer,
        Util::Span<const AccelStructBuildInfo> buildInfo
    )= 0;

    // Writes commands into a command buffer to emit post-build information about an acceleration structure
    //
    // @param pCmdBuffer           [in] Command buffer where commands will be written
    // @param postBuildInfo        [in] Post-build event info
    virtual void EmitAccelStructPostBuildInfo(
        Pal::ICmdBuffer*                pCmdBuffer,
        const AccelStructPostBuildInfo& postBuildInfo
    ) = 0;

    // Writes commands into a command buffer to execute an acceleration structure copy/update/compress operation
    //
    // @param pCmdBuffer           [in] Command buffer where commands will be written
    // @param copyInfo             [in] Copy operation info
    virtual void CopyAccelStruct(
        Pal::ICmdBuffer*              pCmdBuffer,
        const AccelStructCopyInfo&    copyInfo
    ) = 0;

    // Prepares the input buffer (indirect arguments, bindings, constants) for an indirect raytracing dispatch
    //
    // @param pCmdBuffer           [in/out] Command buffer where commands will be written
    // @param userData             [in] Addresses of input/output buffers
    // @param maxDispatchCount     Max indirect dispatches
    // @param pipelineCount        Number of pipelines to dispatch
    virtual void InitExecuteIndirect(
        Pal::ICmdBuffer*                   pCmdBuffer,
        const InitExecuteIndirectUserData& userData,
        uint32                             maxDispatchCount,
        uint32                             pipelineCount) const = 0;

    // Calculates and returns prebuild information about some given future acceleration structure.
    //
    // @param inputs          [in]  Struct describing the inputs for building an acceleration structure
    // @param pPrebuildInfo   [out] Resulting prebuild information
    virtual void GetAccelStructPrebuildInfo(
        const AccelStructBuildInputs& inputs,
        AccelStructPrebuildInfo*      pPrebuildInfo) = 0;

    // Decodes the acceleration structure memory and returns information about the built acceleration structure.
    // @param pAccelStructData [in]  CPU pointer to acceleration structure data
    // @param pAccelStructInfo [out] Result acceleration structure info
    virtual void GetAccelStructInfo(
        const void*            pAccelStructData,
        AccelStructInfo* const pAccelStructInfo) = 0;

    // Returns true if the acceleration structure trace source is currently enabled.
    virtual bool AccelStructTraceEnabled() const = 0;

    // Parses acceleration structure header to read acceleration structure UUID
    // @param pData        [in]  Serialized acceleration structure pointer
    // @param pVersion     [out] Result UUID
    virtual void GetSerializedAccelStructVersion(
        const void*     pData,
        uint64_t*       pVersion) = 0;

    // Check serialized acceleration structure GUID and version
    // @param identifier [in] Serialized acceleration structure identifier
    // Returns the matching status
    virtual DataDriverMatchingIdentifierStatus CheckSerializedAccelStructVersion(
        const DataDriverMatchingIdentifier* identifier) = 0;

    // Returns true if the ray history trace source is currently available for use.
    virtual bool RayHistoryTraceAvailable() const = 0;

    // Returns true if a trace is active. If true, the client should call TraceRtDispatch() for each direct dispatch
    // and TraceIndirectRtDispatch() for each indirect dispatch.
    virtual bool RayHistoryTraceActive() const = 0;

    // Notifies GPURT about an RT dispatch, triggers a trace buffer allocation, and outputs a buffer view for the trace
    // buffer.
    virtual void TraceRtDispatch(
        Pal::ICmdBuffer*       pCmdBuffer,
        RtPipelineType         pipelineType,
        RtDispatchInfo         dispatchInfo,
        DispatchRaysConstants* pConstants) = 0;

    // Notifies GPURT about an indirect RT dispatch similar to TraceRtDispatch(). Multiple disptches may occur in
    // one invocation, and the max count must be provided. At most MaxSupportedIndirectCounters will be traced.
    //
    // @param pCounterMetadataVa [out] GPUVA pointing to an array of maxDispatchCount IndirectCounterMetadata structs.
    //                                 This GPUVA must be used to initialize InitExecuteIndirectUserData.outputCounterMetaVa.
    //
    // @param pIndirectConstants [out] InitExecuteIndirectConstants for RT pipelines and DispatchRaysConstants otherwise.
    virtual void TraceIndirectRtDispatch(
        RtPipelineType                pipelineType,
        RtDispatchInfo                dispatchInfo,
        uint32                        maxDispatchCount,
        gpusize*                      pCounterMetadataVa,
        void*                         pIndirectConstants) = 0;

protected:

    /// Client must create objects by explicitly calling CreateDevice method
    IDevice() { }

    /// Disallow use of delete operator on this interface. Client must destroy objects by explicitly calling
    /// IDevice::Destroy() method.
    virtual ~IDevice() { }
};
}; // namespace GpuRt
