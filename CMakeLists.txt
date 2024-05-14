##
 #######################################################################################################################
 #
 #  Copyright (c) 2020-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

cmake_minimum_required(VERSION 3.21)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

project(GPU_RAY_TRACING LANGUAGES "CXX")

add_library(gpurt STATIC)

add_library(gpurt_internal STATIC)

target_link_libraries(gpurt PUBLIC gpurt_internal)

add_library(AMD::gpurt ALIAS gpurt)

function(gpurt_add_compile_definitions)
    target_compile_definitions(gpurt          PUBLIC ${ARGV} NOMINMAX WIN32_LEAN_AND_MEAN)
    target_compile_definitions(gpurt_internal PUBLIC ${ARGV} NOMINMAX WIN32_LEAN_AND_MEAN)
endfunction()

function(gpurt_add_compile_options)
    target_compile_options(gpurt          PRIVATE ${ARGV})
    target_compile_options(gpurt_internal PRIVATE ${ARGV})
endfunction()

function(gpurt_set_properties)
    set_target_properties(gpurt          PROPERTIES ${ARGV})
    set_target_properties(gpurt_internal PROPERTIES ${ARGV})
endfunction()

gpurt_set_properties(
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# Set output filenames for targets, i.e. gpurt_backend.lib, gpurt_backend.pdb, gpurt.lib, ...
set_target_properties(gpurt          PROPERTIES OUTPUT_NAME "gpurt_backend")
set_target_properties(gpurt_internal PROPERTIES OUTPUT_NAME "gpurt")

option(GPURT_ENABLE_GPU_DEBUG "Enable GPU debugging features (assert, debug prints)." OFF)

if (GPURT_ENABLE_GPU_DEBUG)
    gpurt_add_compile_definitions(GPURT_ENABLE_GPU_DEBUG=1)
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
target_link_libraries(gpurt_internal PUBLIC pal)

# Default PAL variables are not visible here, but sometimes they are set explicitly by the user or build configuration.
# Implicitly enable the related RTIPs in this case.

# Enable gpu developer mode if the client wants it.
if (GPURT_DEVELOPER_MODE)
    gpurt_add_compile_definitions(GPURT_DEVELOPER=1)
endif()

# Set the client API, error checking done via static_asserts.
gpurt_add_compile_definitions(GPURT_CLIENT_API_${GPURT_CLIENT_API}=1)

#if GPURT_BUILD_CONTINUATION
option(GPURT_BUILD_CONTINUATION "GpuRt uses continuation traversal" ON)

if (GPURT_BUILD_CONTINUATION)
    gpurt_add_compile_definitions(GPURT_BUILD_CONTINUATION=1)
endif()
#endif

# Disable run time type information
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    gpurt_add_compile_options(-fno-rtti)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    gpurt_add_compile_options(
        # Disable run time type information
        /GR-
    )
endif()

if (DEFINED GPURT_CLIENT_INTERFACE_MAJOR_VERSION)
    gpurt_add_compile_definitions(GPURT_CLIENT_INTERFACE_MAJOR_VERSION=${GPURT_CLIENT_INTERFACE_MAJOR_VERSION})
endif()

### Add Source Directories
target_include_directories(gpurt PUBLIC .)
target_include_directories(gpurt_internal PUBLIC .)
add_subdirectory(gpurt)
add_subdirectory(src)
add_subdirectory(tools)
add_subdirectory(cmake)
add_subdirectory(backends/pal)

