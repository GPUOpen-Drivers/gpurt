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

get_target_property(COMPILE_DEFINITIONS gpurt COMPILE_DEFINITIONS)
set(gpurtDefines "${COMPILE_DEFINITIONS}")

set(gpurtToolsDir "${GPU_RAY_TRACING_SOURCE_DIR}/tools")

# Shared tools
set(gpurtCompileScript    "${gpurtToolsDir}/CompileRTShaders.py")
set(gpurtCompileConfig    "${gpurtToolsDir}/RTShaders.xml")
set(gpurtStripWhitelist   "${gpurtToolsDir}/strip_whitelist.txt")

# Outputs
set(gpurtOutputDir "${CMAKE_CURRENT_BINARY_DIR}/pipelines")
set(gpurtShaders
    "${gpurtOutputDir}/g_internal_shaders.h"
    "${gpurtOutputDir}/g_GpuRtLibrary.h"
)

set(gpurtDebugInfoFile "${CMAKE_CURRENT_BINARY_DIR}/g_gpurtDebugInfo.h")

# Make the outputs accessible in the source code.
target_include_directories(gpurt PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

set(originalShaderSourceDir "${GPU_RAY_TRACING_SOURCE_DIR}/src/shaders/")
set(originalShaderSource ${GPURT_SHADER_SOURCE_FILES})
list(TRANSFORM originalShaderSource PREPEND "${originalShaderSourceDir}")

if (GPURT_ENABLE_GPU_DEBUG)
    # Mimic the directory structure of the source tree so relative paths to shared headers work
    set(debugShaderDirectory "${CMAKE_CURRENT_BINARY_DIR}/debugShaders/src/shaders/")
    set(gpurtShaderSource ${GPURT_SHADER_SOURCE_FILES})
    set(gpurtShadersSourceDir ${debugShaderDirectory})
    list(TRANSFORM gpurtShaderSource PREPEND "${debugShaderDirectory}")
    set(preprocessArgs "")
    foreach(originalSourceFile ${GPURT_SHADER_SOURCE_FILES})
        set(originalSourcePath "${originalShaderSourceDir}${originalSourceFile}")
        set(newSourceFilePath "${debugShaderDirectory}${originalSourceFile}")
        list(APPEND preprocessArgs "${originalSourcePath}" "${newSourceFilePath}")
    endforeach()
    set(gpurtDebugPreprocessorScript "${gpurtToolsDir}/DebugPreprocessShaders.py")
    add_custom_command(
        OUTPUT  ${gpurtShaderSource} ${gpurtDebugInfoFile}
        DEPENDS ${originalShaderSource} ${gpurtDebugPreprocessorScript}
        COMMAND Python3::Interpreter ${gpurtDebugPreprocessorScript} ${preprocessArgs} ${gpurtDebugInfoFile}
    )
else()
    set(gpurtShaderSource "${originalShaderSource}")
    set(gpurtShadersSourceDir "${originalShaderSourceDir}")
endif()

set(gpurtSharedDependencies
    ${gpurtShaderSource}
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
            --defines "\"${gpurtDefines}\""
            --shaderConfig "${gpurtCompileConfig}"
            --whiteListPath "${gpurtStripWhitelist}"
            "${gpurtShadersSourceDir}"
            --strict
    )
else()
    message(FATAL_ERROR "Unknown graphics API: ${GPURT_CLIENT_API}")
endif()

# Create the custom target
# Have it depend on the above custom commands' output to establish a dependency
add_custom_target(GpuRtGenerateShaders DEPENDS ${gpurtShaders})

# Make gpurt dependent on RT shader generation
add_dependencies(gpurt GpuRtGenerateShaders)
