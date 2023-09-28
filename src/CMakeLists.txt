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

add_subdirectory(shaders)

if (CMAKE_HOST_SYSTEM_NAME MATCHES "Linux" AND GPURT_CLIENT_API STREQUAL "VULKAN")
    find_program(DXC_PATH dxc)
    if ("${DXC_PATH}" STREQUAL "DXC_PATH-NOTFOUND")
        message(WARNING "Could not find dxc.")
        if (EXISTS ${GPU_RAY_TRACING_SOURCE_DIR}/tools/lnx)
            include(GpuRtGenerateShaders)
        else()
            message(FATAL_ERROR "Could not find shader compiler tool dxc.")
        endif()
    else()
        include(GpuRtGenerateShaders)
    endif()
else()
    include(GpuRtGenerateShaders)
endif()

target_include_directories(gpurt PRIVATE .)

target_sources(gpurt PRIVATE
    CMakeLists.txt
    gpurtBvhBatcher.cpp
    gpurtBvhBuilder.cpp
    gpurtDebugMonitor.cpp
    gpurtDevice.cpp
    gpurtInternalShaders.cpp
    gpurtTraceSource.cpp
)

