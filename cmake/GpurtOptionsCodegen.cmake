##
 #######################################################################################################################
 #
 #  Copyright (c) 2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

# CMake file for autogenerating options code.

# No particular version of python3 is neccessary
find_package(Python3
    # Disable information messages
    QUIET
    # Python3 is required to run the generate script
    REQUIRED
    # Only the interpreter is required, we don't need the Compiler, Development, or NumPy
    COMPONENTS Interpreter
)

set(OUTDIR "${CMAKE_CURRENT_BINARY_DIR}")

set(GPURTOPTIONS_SCRIPT "${GPU_RAY_TRACING_SOURCE_DIR}/tools/genOptions.py")
set(GPURTOPTIONS_INPUT  "${GPU_RAY_TRACING_SOURCE_DIR}/src/options.yaml")
set(GPURTOPTIONS_OUTPUT "${OUTDIR}/g_gpurtOptions.h")
set(GPURTOPTIONS_OUTPUT "${GPURTOPTIONS_OUTPUT}" PARENT_SCOPE)
add_custom_command(
    OUTPUT ${GPURTOPTIONS_OUTPUT}
    COMMAND ${Python3_EXECUTABLE} ${GPURTOPTIONS_SCRIPT} -o ${GPURTOPTIONS_OUTPUT} ${GPURTOPTIONS_INPUT}
    DEPENDS ${GPURTOPTIONS_SCRIPT} ${GPURTOPTIONS_INPUT}
    COMMENT "Generating GPURT options tables"
)

add_custom_target(generate_gpurtOptions_h
    DEPENDS ${GPURTOPTIONS_OUTPUT}
)

target_include_directories(gpurt PUBLIC ${OUTDIR})

