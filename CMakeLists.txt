##
 #######################################################################################################################
 #
 #  Copyright (c) 2020-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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

cmake_minimum_required(VERSION 3.13...3.20)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

project(GPU_RAY_TRACING LANGUAGES "CXX")

add_library(gpurt STATIC)

add_library(AMD::gpurt ALIAS gpurt)

set_target_properties(gpurt PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

option(GPURT_ENABLE_GPU_DEBUG "Enable GPU debugging features (assert, debug prints)." OFF)

if (GPURT_ENABLE_GPU_DEBUG)
    target_compile_definitions(gpurt PRIVATE GPURT_ENABLE_GPU_DEBUG=1)
endif()

# Force client to specify developer mode
if (NOT DEFINED GPURT_DEVELOPER_MODE)
    message(FATAL_ERROR "Client must specify developer mode")
endif()

# Force client to specify the api. It doesn't make sense not to.
if (NOT DEFINED GPURT_CLIENT_API)
    message(FATAL_ERROR "Client must specify API")
endif()

# GPU Ray Tracing requires PAL as a dependency
target_link_libraries(gpurt PUBLIC pal)

#if GPURT_BUILD_RTIP2
# Enable GpuRT defines for Gfx11 builds
target_compile_definitions(gpurt PUBLIC GPURT_BUILD_RTIP2=1)
#endif

# Enable gpu developer mode if the client wants it.
if (GPURT_DEVELOPER_MODE)
    target_compile_definitions(gpurt PUBLIC GPURT_DEVELOPER=1)
endif()

# Set the client API, error checking done via static_asserts.
target_compile_definitions(gpurt PUBLIC GPURT_CLIENT_API_${GPURT_CLIENT_API}=1)

# Disable run time type information
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(gpurt PRIVATE -fno-rtti)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    target_compile_options(gpurt PRIVATE
        # Disable run time type information
        /GR-
    )
endif()

if (DEFINED GPURT_CLIENT_INTERFACE_MAJOR_VERSION)
    target_compile_definitions(gpurt PUBLIC GPURT_CLIENT_INTERFACE_MAJOR_VERSION=${GPURT_CLIENT_INTERFACE_MAJOR_VERSION})
endif()

### Add Source Directories
target_include_directories(gpurt PUBLIC .)
add_subdirectory(gpurt)
add_subdirectory(src)
add_subdirectory(tools)
add_subdirectory(cmake)

