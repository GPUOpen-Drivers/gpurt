##
 #######################################################################################################################
 #
 #  Copyright (c) 2022-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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

# No particular version of python3 is neccessary
find_package(Python3
    # Disable information messages
    QUIET
    # Python3 is required to run the shader compile script
    REQUIRED
    # Only the interpreter is required, we don't need the Compiler, Development, or NumPy
    COMPONENTS Interpreter
)

set(gpurtToolsDir         "${GPU_RAY_TRACING_SOURCE_DIR}/tools")
set(gpurtShadersSourceDir "${GPU_RAY_TRACING_SOURCE_DIR}/src/shaders")

# Shared tools
set(gpurtCompileScript    "${gpurtToolsDir}/CompileRTShaders.py")
set(gpurtCompileConfig    "${gpurtToolsDir}/RTShaders.xml")
set(gpurtStripWhitelist   "${gpurtToolsDir}/strip_whitelist.txt")

# Headers that are shared between HLSL and other things
set(gpurtSharedLanguageHeaders "${GPU_RAY_TRACING_SOURCE_DIR}/gpurt/gpurtCounter.h")

# Outputs
set(gpurtOutputDir "${CMAKE_CURRENT_BINARY_DIR}/pipelines")
set(gpurtShaders
    "${gpurtOutputDir}/g_internal_shaders.h"
    "${gpurtOutputDir}/g_GpuRtLibrary.h"
)

# Make the outputs accessible in the source code.
target_include_directories(gpurt PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

set(gpurtSharedDependencies
    ${GPURT_GEN_SHADERS_SOURCE}
    ${gpurtShaderHeaders}
    ${gpurtSharedLanguageHeaders}
    ${gpurtCompileScript}
    ${gpurtCompileConfig}
)

# Create custom command that outputs the generated shaders
# The generated shaders depend on all the above mentioned files
if(GPURT_CLIENT_API STREQUAL "VULKAN")
    set(SPIRV_FLAG "--spirv")

    if (CMAKE_HOST_SYSTEM_NAME MATCHES "Linux")
        # Find other executable paths so we can check if they updated in the DEPENDS.
        set(gpurtSpirvRemap "")
        find_program(gpurtSpirvRemap spirv-remap REQUIRED)
        set(gpurtDxcCompiler "")
        find_program(gpurtDxcCompiler dxc REQUIRED)

        if (EXISTS ${GPU_RAY_TRACING_SOURCE_DIR}/tools/lnx)
            set(SPIRV_COMPILER_ARGUMENT "--spirvCompilerPath" "${GPU_RAY_TRACING_SOURCE_DIR}/tools/lnx")
            set(SPIRV_REMAP_ARGUMENT    "--spirvRemapPath"    "${GPU_RAY_TRACING_SOURCE_DIR}/tools/lnx")
        else()
            # Adjust arguments to remove the unnecessary ones. These binaries should be found on the system PATH.
            set(SPIRV_COMPILER_ARGUMENT "")
            set(SPIRV_REMAP_ARGUMENT "")
        endif()
    endif()

    add_custom_command(
        OUTPUT
            ${gpurtShaders}

        DEPENDS
            ${gpurtSharedDependencies}
            ${gpurtStripWhitelist}
            ${gpurtDxcCompiler}
            ${gpurtSpirvRemap}

        COMMAND Python3::Interpreter "${gpurtCompileScript}"
            --vulkan
            "${SPIRV_FLAG}"
            --outputDir "${gpurtOutputDir}"
            ${SPIRV_COMPILER_ARGUMENT}
            ${SPIRV_REMAP_ARGUMENT}
            --shaderConfig "${gpurtCompileConfig}"
            --whiteListPath "${gpurtStripWhitelist}"
            "${gpurtShadersSourceDir}"
    )
else()
    message(FATAL_ERROR "Unknown graphics API: ${GPURT_CLIENT_API}")
endif()

# Create the custom target
# Have it depend on the above custom commands' output to establish a dependency
add_custom_target(GpuRtGenerateShaders DEPENDS ${gpurtShaders})

# Make gpurt dependent on RT shader generation
add_dependencies(gpurt GpuRtGenerateShaders)
