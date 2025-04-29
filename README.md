# GPU Ray Tracing Library

The GPU Ray Tracing (GPURT) library is a static library (source deliverable) that provides ray tracing related functionalities for AMD drivers supporting DXR (DirectX 12&reg;) and the Vulkan&reg; RT API. The GPURT library is built on top of AMD's Platform Abstraction Library (PAL). Refer to PAL documentation for information regarding interfaces used by GPURT for various operations including command generation, memory allocation etc.

GPURT provides the majority of non-compiler functionality required to support DX12 and Vulkan raytracing APIs.

The primary functionality provided by GPURT includes the following:

* Building acceleration structures.
* Traversal loop and associated intrinsic functions written in HLSL.
  * Ray tracing traversal counter and ray history data capture.
* Acceleration structure capture and decode.

## Code Style

GPURT follows [PAL Coding Standards](https://github.com/GPUOpen-Drivers/pal/blob/dev/doc/process/palCodingStandards.md) whenever possible.

## Code Organization

We are moving code from the unorganized src/shaders/ directory to the src/shadersClean/ directory. Within src/shadersClean the code is organized as follows:

| Directory | Description |
| --------- | ----------- |
| _build_ | Code related to BuildRaytracingAccelerationStructure and vkCmdBuildAccelerationStructuresKHR |
| _traversal_ | Code related to DispatchRays & vkCmdTraceRaysKHR. |
| _common_ | Code common to build and traversal. |
| _debug_ | Code related to implementing GPU_ASSERT. |

Code in the src/shadersClean directory undergoes validation. Before compiling shader entrypoints, each HLSL file in src/shadersClean is compiled separately (results are discarded) to check if it has no implicit dependencies. HLSL files can only include HLSLI files (or C++ Interfaces) and HLSLI files can only include other HLSLI files (or C++ Interfaces).

### C++ Interfaces

GPURT uses a C++ interface. The public interface is defined in .../gpurt/gpurt, and clients must only include headers from that directory. The interface is divided into multiple header files based on their dependencies and usage.

* gpurt/gpurt.h
  * Provides definition of the public GPURT device interface used to perform various ray tracing operations and querying traversal shader code.
* gpurt/gpurtAccelStruct.h
  * Shared header file between C++ and HLSL that provides the definition of the GPURT acceleration structure produced by the GPURT acceleration structure builder.
* gpurt/gpurtCounter.h
  * Shared header file between C++ and HLSL code that provides the definitions of GPURT counters and related data structures.

## Acceleration Structure

GPURT interfaces support various acceleration structure operations including:

* Querying acceleration structure memory and associated scratch memory requirements.
* Building top-level and bottom-level acceleration structures.
* Cloning or relocating acceleration structure data.
* Serializing and Deserializing acceleration structure data.
* Decoding acceleration structures.

GPURT supports various algorithms for building efficient acceleration structures. The build algorithm implementations are located in the src/shaders folder.

## Traversal Loop

The GPURT library implements various traversal loops for API intrinsics such as DXR's TraceRay() & RayQuery and Vulkan RT's TraceRayKHR & RayQueryKHR. The HLSL implementation of these traversal loops and associated intrinsics is located in the src/shaders folder with the main entry points defined in src/shader/GpuRtLibrary.hlsl

## Dependencies

GPURT has a direct dependency on AMD's PAL library as it relies on the Utility library and uses PAL interfaces for command generation, memory allocation etc. Client drivers must already have PAL sources and library specified in their include paths.

## How does the shader compilation work?

A CMake script is used to get all the executable dependencies (mainly compilers) and check for file changes. After it gets everything it needs, it invokes the `tools/CompileRTShaders.py` script, which will do the actual compilation by calling tools such as [DirectXShaderCompiler](https://github.com/microsoft/DirectXShaderCompiler) and [glslang](https://github.com/KhronosGroup/glslang). This produces a bunch of header files which hardcode the compiled shaders into arrays of 32-bit unsigned integers, as well as a header file `g_internal_shaders.h` which just has generated includes for all the other headers that were created. After all this is done, proper source files just include `g_internal_shaders.h` to reference the compiled shaders.
