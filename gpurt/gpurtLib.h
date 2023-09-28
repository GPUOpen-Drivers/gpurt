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

#pragma once

#include <stdint.h>
namespace GpuRt
{
    using uint16 = uint16_t;
    using uint32 = uint32_t;
};

// Major interface version. This number must be incremented when GPURT's interface to the driver changes in an
// incompatible way. An incompatible interface change requires updates in the driver to build or function correctly.
// GPURT maintains source level compatibility with old interface versions for a period of time. Client drivers must
// update their definition of GPURT_CLIENT_INTERFACE_MAJOR_VERSION to indicate that they have made the required changes
// to support a new version. When the client version is updated, the old interface will be compiled out and only the
// new one will remain.
#define GPURT_INTERFACE_MAJOR_VERSION 39

// Minor interface version. This number is incrememnted when a compatible interface change is made. Compatible changes
// do not require client driver changes to maintain existing functionality. GPURT assumes all structures are zero
// initialized by the client, therefore new fields may be added in minor versions as long as zero is an acceptable
// default value.
#define GPURT_INTERFACE_MINOR_VERSION 0

// Minimum major interface version. This is the minimum interface version GPURT supports in order to support backward
// compatibility. When it is equal to GPURT_INTERFACE_MAJOR_VERSION, only the latest interface version is supported.
#define GPURT_MINIMUM_INTERFACE_MAJOR_VERSION 33

// Current GPURT interface version packed into a 32-bit unsigned integer
#define GPURT_INTERFACE_VERSION ((GPURT_INTERFACE_MAJOR_VERSION << 16) | GPURT_INTERFACE_MINOR_VERSION)

// Static asserts to ensure clients have defined GPURT_CLIENT_INTERFACE_MAJOR_VERSION and that it falls in the supported
// range.
static_assert((GPURT_CLIENT_INTERFACE_MAJOR_VERSION >= GPURT_MINIMUM_INTERFACE_MAJOR_VERSION) &&
    (GPURT_CLIENT_INTERFACE_MAJOR_VERSION <= GPURT_INTERFACE_MAJOR_VERSION),
    "The specified GPURT_CLIENT_INTERFACE_MAJOR_VERSION is not supported.");

namespace GpuRt
{

// Ray tracing binary file version. This number must be incremented whenever the BVH, traversal counter or ray history
// data format changes or the associated header changes. This version is used for legacy dump files only. It is not
// used for RDF.
constexpr uint16 RayTracingBinaryFileVersionMajor = 8;
constexpr uint16 RayTracingBinaryFileVersionMinor = 1;
constexpr uint32 RayTracingBinaryFileVersion =
    (RayTracingBinaryFileVersionMajor << 16) | RayTracingBinaryFileVersionMinor;

// ====================================================================================================================
// Ray Tracing Binary File Version History
// ---------------------------------------
//
// | Version | Change Description                                                                                     |
// | ------- | ------------------------------------------------------------------------------------------------------ |
// |    8.1  | Move decoded header to the front of the decoded dump.                                                  |
// |    8.0  | Add decoded header to decoded BVH dumps.                                                               |
// |    7.2  | BLAS header stores the instance mask as well so that it can be propagated up to TLAS.                  |
// |    7.1  | Add pipeline type to counter info                                                                      |
// |    7.0  | Add new ray history tokens replacing old one according to version changes (v3.0), and added a new      |
// |         | Offset for the TraceRay Index to ray history.                                                          |
// |    6.3  | BLAS header stores the root node's bounding box and is loaded in EncodeInstances() when building the   |
// |         | TLAS, instead of fetching the node and generating the bounding box.                                    |
// |    6.2  | Procedural Intersection shader call status added to light mode logging.                                |
// |    6.1  | Add counter for half f32 box node.                                                                     |
// |    6.0  | Add shader table index to ray history data.                                                            |
// |    5.1  | Add prim node pointers for TLAS.                                                                       |
// |    5.0  | Add RTIP version information to binary file headers and traversal flags for ray history capture.       |
// |    4.0  | Remove four triangle compression mode and triangle sideband data.                                      |
// |    3.0  | Added pair triangle compression mode, where each triangle node has only one parent.                    |
// |    2.1  | AnyHit call status added to light mode logging.                                                        |
// |    2.0  | Task queue counters moved to scratch.                                                                  |
// |    1.0  | Initial version.                                                                                       |
// ====================================================================================================================

} // GpuRt
