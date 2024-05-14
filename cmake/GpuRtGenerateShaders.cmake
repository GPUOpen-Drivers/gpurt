##
 #######################################################################################################################
 #
 #  Copyright (c) 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
# For including generated headers
set(gpurtIncludeDirectories "${CMAKE_CURRENT_BINARY_DIR}")
# In CMake, we have to add target AND file dependencies here.
# See also https://gitlab.kitware.com/cmake/cmake/-/issues/20828
# or https://samthursfield.wordpress.com/2015/11/21/cmake-dependencies-between-targets-and-files-and-custom-commands/
set(gpurtSharedDependencies generate_gpurtOptions_h ${GPURTOPTIONS_OUTPUT})

if (TARGET llpc_version)
    # Propagate include directories and defines from llpc_version into the HLSL code
    get_target_property(LLPC_VERSION_INCLUDE_DIRS llpc_version INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(LLPC_VERSION_DEFS llpc_version INTERFACE_COMPILE_DEFINITIONS)
    get_target_property(LLPC_VERSION_SOURCE llpc_version SOURCE_DIR)

    list(APPEND gpurtDefines "${LLPC_VERSION_DEFS}")
    list(APPEND gpurtIncludeDirectories "${LLPC_VERSION_INCLUDE_DIRS}")
    # Need to add target AND file dependencies. See comment on gpurtSharedDependencies.
    list(APPEND gpurtSharedDependencies llpc_version "${LLPC_VERSION_SOURCE}/include/llpc/GpurtIntrinsics.h")
endif()

set(gpurtToolsDir "${GPU_RAY_TRACING_SOURCE_DIR}/tools")

# Shared tools
set(gpurtCompileScript  "${gpurtToolsDir}/CompileRTShaders.py")
set(gpurtStripWhitelist "${gpurtToolsDir}/strip_whitelist.txt")

# Outputs
set(gpurtOutputDir "${CMAKE_CURRENT_BINARY_DIR}/pipelines")
set(gpurtBvhShaders "${gpurtOutputDir}/g_internal_shaders.h")
set(gpurtTraceShadersSpirv "${gpurtOutputDir}/g_GpuRtLibrary_spv.h")

set(gpurtDebugInfoFile "${CMAKE_CURRENT_BINARY_DIR}/g_gpurtDebugInfo.h")

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

list(APPEND gpurtSharedDependencies
    ${gpurtShaderSource}
    ${gpurtCompileScript}
)

# Create custom command that outputs the generated BVH shaders
# The generated shaders depend on all the above mentioned files
if(GPURT_CLIENT_API STREQUAL "VULKAN")
    set(SPIRV_FLAG "--spirv")

    if (NOT CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
        # Find other executable paths so we can check if they updated in the DEPENDS.
        set(gpurtSpirvRemap "")
        find_program(gpurtSpirvRemap spirv-remap REQUIRED)
        set(gpurtDxcCompiler "")
        find_program(gpurtDxcCompiler dxc REQUIRED)

        if (EXISTS ${GPU_RAY_TRACING_SOURCE_DIR}/tools/lnx)
            set(COMPILER_ARGUMENT    "--compilerPath"   "${GPU_RAY_TRACING_SOURCE_DIR}/tools/lnx")
            set(SPIRV_REMAP_ARGUMENT "--spirvRemapPath" "${GPU_RAY_TRACING_SOURCE_DIR}/tools/lnx")
        else()
            # Adjust arguments to remove the unnecessary ones. These binaries should be found on the system PATH.
            set(COMPILER_ARGUMENT "")
            set(SPIRV_REMAP_ARGUMENT "")
        endif()
    endif()

    add_custom_command(
        OUTPUT
            ${gpurtBvhShaders}

        DEPENDS
            ${gpurtSharedDependencies}
            ${gpurtStripWhitelist}
            ${gpurtDxcCompiler}
            ${gpurtSpirvRemap}

        COMMAND Python3::Interpreter "${gpurtCompileScript}"
            --vulkan
            "${SPIRV_FLAG}"
            --outputDir "${gpurtOutputDir}"
            --skip-trace
            ${COMPILER_ARGUMENT}
            ${SPIRV_REMAP_ARGUMENT}
            --defines "\"${gpurtDefines}\""
            --includePaths "\"${gpurtIncludeDirectories}\""
            --whiteListPath "${gpurtStripWhitelist}"
            "${gpurtShadersSourceDir}"
            --strict
    )
else()
    message(FATAL_ERROR "Unknown graphics API: ${GPURT_CLIENT_API}")
endif()

# For trace shaders, remove the GPURT_CLIENT_API_* definition.
# The command below adds the appropriate GPURT_CLIENT_API_* definition for the
# trace shaders being built.
list(FILTER gpurtDefines EXCLUDE REGEX "GPURT_CLIENT_API_.*")

# Create custom command that outputs the generated trace shaders as SPIR-V.
# The generated shaders depend on all the above mentioned files.
add_custom_command(
    OUTPUT
        ${gpurtTraceShadersSpirv}

    DEPENDS
        ${gpurtSharedDependencies}
        ${gpurtStripWhitelist}
        ${gpurtDxcCompiler}
        ${gpurtSpirvRemap}

    COMMAND Python3::Interpreter "${gpurtCompileScript}"
        --vulkan
        "${SPIRV_FLAG}"
        --outputDir "${gpurtOutputDir}"
        --skip-bvh
        --spirv
        ${COMPILER_ARGUMENT}
        ${SPIRV_REMAP_ARGUMENT}
        --defines "\"${gpurtDefines};GPURT_CLIENT_API_VULKAN=1\""
        --includePaths "\"${gpurtIncludeDirectories}\""
        --whiteListPath "${gpurtStripWhitelist}"
        "${gpurtShadersSourceDir}"
        --strict
)

# Create the custom targets for generating the header files.
# Have them depend on the above custom commands' output to establish a dependency.
add_custom_target(GpuRtGenerateBvhShaders DEPENDS ${gpurtBvhShaders})
set_target_properties(GpuRtGenerateBvhShaders PROPERTIES EXCLUDE_FROM_ALL TRUE)
add_custom_target(GpuRtGenerateTraceShadersSpirv DEPENDS ${gpurtTraceShadersSpirv})
set_target_properties(GpuRtGenerateTraceShadersSpirv PROPERTIES EXCLUDE_FROM_ALL TRUE)

# Create interface targets for using the generated header files. The interface targets encode
# the include directory, which is inherited by anything using the interface.
add_library(GpuRtBvhShaders INTERFACE)
add_dependencies(GpuRtBvhShaders GpuRtGenerateBvhShaders)
target_include_directories(GpuRtBvhShaders INTERFACE ${CMAKE_CURRENT_BINARY_DIR})

add_library(GpuRtTraceShadersSpirv INTERFACE)
add_dependencies(GpuRtTraceShadersSpirv GpuRtGenerateTraceShadersSpirv)
target_include_directories(GpuRtTraceShadersSpirv INTERFACE ${CMAKE_CURRENT_BINARY_DIR})
target_compile_definitions(GpuRtTraceShadersSpirv INTERFACE HAVE_GPURT_TRACE_SHADERS_SPIRV=1)

# Make gpurt dependent on RT shader generation and include directory.
target_link_libraries(gpurt_internal PRIVATE GpuRtBvhShaders)
if(GPURT_CLIENT_API STREQUAL "VULKAN")
    target_link_libraries(gpurt_internal PRIVATE GpuRtTraceShadersSpirv)
else()
    message(FATAL_ERROR "Unknown graphics API: ${GPURT_CLIENT_API}")
endif()

