##
 #######################################################################################################################
 #
 #  Copyright (c) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
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


import sys
import os
import re
from xml.etree.ElementTree import ElementTree
import subprocess
import argparse
from concurrent.futures import ThreadPoolExecutor
import time
import enum
import traceback
import struct
import shutil

DWORDS_PER_LINE = 8

FILE_STANDARD_HEADER = """
/* Copyright (c) 2022 Advanced Micro Devices, Inc. All rights reserved. */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!
//
// This code has been generated automatically. Do not hand-modify this code.
//
// WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING! WARNING!  WARNING!  WARNING!  WARNING!
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "gpurt/gpurt.h"

"""
FILE_HEADER_FORMAT_STRING = "static constexpr uint32 Cs%s[] =\n{\n"
FILE_FOOTER_STRING = "\n};\n"

SHADER_PROFILE = "cs_6_2"
SHADER_PROFILE_LIB = "lib_6_3"
DEFAULT_SHADER_CONFIG = "RTShaders.xml"
DEFAULT_INTERNAL_SHADERS_HEADER_NAME = "g_internal_shaders"
DEFAULT_OUTPUT_DIR = "CompiledShaders"
SPIRV_WHITELIST = "strip_whitelist.txt"
SPV_REMAP_EXECUTABLE = "spirv-remap"
DXC_EXECUTABLE = "dxc"
GLSLANG_EXECUTABLE = "glslangValidator"

class ShaderConfig:
    def __init__(self, inPath, inEntryPoint, inOutputName, inBaseLogicalId, inRootSignaturePath, inDefines, inGroupTag):
        self.path = inPath
        self.entryPoint = inEntryPoint
        self.outputName = inOutputName
        self.baseLogicalId = inBaseLogicalId
        self.rootSignaturePath = inRootSignaturePath
        self.defines = inDefines
        self.groupTag = inGroupTag

    def __str__(self):
        return "ShaderConfig< Path: %s, EntryPoint: %s, OutputName: %s, BaseLogicalId: %s, RootSignaturePath: %s, Defines: %s, GroupTag: %s >" % (self.path, self.entryPoint, self.outputName, self.baseLogicalId, self.rootSignaturePath, self.defines, self.groupTag)

    def isLibrary(self):
        return self.entryPoint is None

    # The output filename is either explicitly specified or we default to the entry point
    def getName(self):
        return self.outputName or self.entryPoint

    # Only trace shaders are libraries
    def isBVH(self):
        return not self.isLibrary()

def isSpirvShader(shaderConfig, args):
    return args.vulkan and (shaderConfig.isLibrary() or (shaderConfig.isBVH() and args.spirv))

def ParseShaderConfig(args, inFilename):
    root = ElementTree().parse(inFilename)

    shaders = []

    for group in root.iter():
        if (group.tag == "Trace" and not args.skip_trace) or (group.tag == "BVH" and not args.skip_bvh):
            for shader in group.iter('Shader'):
                path = shader.get('path').replace('\\', '/')
                entryPoint = shader.get('entryPoint')
                outputName = shader.get('outputName')
                baseLogicalId = shader.get('baseLogicalId')
                rootSignaturePath = shader.get('rootSignaturePath')
                defines = shader.get('defines')

                shaders.append(ShaderConfig(path, entryPoint, outputName, baseLogicalId, rootSignaturePath, defines, group.tag))

    return shaders

class CompilationType(enum.Enum):
    Dxil = 0
    VulkanRtSpirv = 1

def FixExePath(exePath):
    if os.path.exists(exePath + ".exe"):
        # Use local Windows Path
        return exePath + ".exe"
    elif not os.path.exists(exePath):
        # Use system PATH on Linux
        return os.path.basename(exePath)
    return exePath

def RemoveFile(path):
    # Workaround for file handles sometimes not closing correctly when just using os.remove()
    tmpPath = path + "tmp"
    os.rename(path, tmpPath)
    os.remove(tmpPath)

def RemoveFolder(path):
    shutil.rmtree(path)

def InvokeSubprocess(commandArgs, workingDirectory, threadOutput, linuxLibraryPath=None, expectNoOutput=False) -> int:
    compileResult = 0
    # `communicate` returns byte strings
    compileOutput = (b"", b"")

    newEnv = os.environ.copy()

    # Windows loads DLLs from the same directory as the executable.
    # This will replicate that behavior on Linux for shared objects.
    # (needed so that it can access libdxcompiler.so)
    if sys.platform == "linux":
        if linuxLibraryPath:
            newEnv["LD_LIBRARY_PATH"] = (os.environ["LD_LIBRARY_PATH"] + ':' if "LD_LIBRARY_PATH" in os.environ else '') + os.path.dirname(linuxLibraryPath)
        elif "LD_LIBRARY_PATH" in os.environ:
            newEnv["LD_LIBRARY_PATH"] = os.environ["LD_LIBRARY_PATH"]

    try:
        compileProcess = subprocess.Popen(commandArgs, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=newEnv, cwd=workingDirectory)
        compileOutput = compileProcess.communicate()
        if (expectNoOutput == True) and ((len(compileOutput[0]) + len(compileOutput[1])) > 0):
            compileResult = -1
        else:
            compileResult = compileProcess.returncode
    except Exception as error:
        threadOutput.append("Encountered Python exception when running subprocess. Error follows this message, followed by information about the subprocess:")
        threadOutput.append(f"{error}")
        threadOutput.append(traceback.format_exc())
        compileResult = -1

    if compileResult != 0:
        threadOutput.append(f"Subprocess invocation failed. Error code {compileResult}. Additional details follow:")
        threadOutput.append(f"Environment: {newEnv}")
        threadOutput.append(f"Command arguments: {commandArgs}")
        threadOutput.append(f"Expected empty output: {expectNoOutput}")
        threadOutput.append(f"Working directory: {workingDirectory}")
        threadOutput.append("Stdout:")
        threadOutput.append(compileOutput[0].decode(sys.stdout.encoding).strip())
        threadOutput.append("Stderr:")
        threadOutput.append(compileOutput[1].decode(sys.stdout.encoding).strip())

    return compileResult

def SpvRemap(spvRemap, whiteListFile, inSpvFilename, outputDir, threadOutput) -> int:
    spvRemap = FixExePath(spvRemap)
    cmdArgs = [
        spvRemap,
        '--strip-white-list', whiteListFile,
        '--strip-all',
        '-i', inSpvFilename,
        '-o', '.'
    ]
    return InvokeSubprocess(cmdArgs, outputDir, threadOutput)

def ConvertSpvFile(disableDebugStripping, spvRemap, whiteListFile, inSpvFilename, inOutputName, isBVH, outputDir, threadOutput):
    try:
        # Generate file name with *_spv.h and file header name with Cs*_spv[] for SPIR-V BVH shaders to distinguish them with the AMDIL header
        outputFileName = "g_" + inOutputName + '.h' if not isBVH else "g_" + inOutputName + '_spv.h'
        conversionOutputFilename = inOutputName if not isBVH else inOutputName + '_spv'

        if not disableDebugStripping:
            remapResult = SpvRemap(spvRemap, whiteListFile, inSpvFilename, outputDir, threadOutput)
            if remapResult != 0:
                # SpvRemap should already have filled out debug info in `threadOutput`
                return False

        spvBinaryFile = open(f"{outputDir}/{inSpvFilename}", "rb")
        spvBinData = spvBinaryFile.read()
        spvBinaryFile.close()

        i = 0
        spvHexText = ""
        while i < len(spvBinData):
            binWord = spvBinData[i:i+4]
            intWord = struct.unpack('I', binWord)[0]
            hexWord = "{0:#010x}".format(intWord)
            spvHexText += hexWord

            i += 4

            if (i != len(spvBinData)):
                spvHexText += ","
            if (i % 32 == 0):
                spvHexText += "\n"
            else:
                spvHexText += " "

        outputFile = open(outputFileName, "w")
        outputFile.write(FILE_STANDARD_HEADER)
        outputFile.write(FILE_HEADER_FORMAT_STRING % conversionOutputFilename)
        outputFile.write(spvHexText)
        outputFile.write(FILE_FOOTER_STRING)
        outputFile.close()

        return True
    except Exception as e:
        threadOutput.append(f"Error: {e}")
        return False

def RunSpirv(outputDir, compilerPath, inShaderConfig, inShaderBasePath, inDXC, inVerbose, threadOutput):
    shaderPath = os.path.join(inShaderBasePath, inShaderConfig.path).replace('\\', '/')
    shaderFilename = os.path.basename(inShaderConfig.path)

    detailString = ":" + inShaderConfig.entryPoint if inShaderConfig.entryPoint is not None else "<Library>"
    ext = ".h"
    if inDXC:
        ext = "_spv" + ext
    genFileName = inShaderConfig.getName() + ext
    compilationString = "Compiling %s%s -> %s..." % (shaderFilename, detailString, genFileName)
    threadOutput.append(compilationString)

    shaderDefines = []
    if inShaderConfig.defines:
        defines = inShaderConfig.defines.split(',')
        for define in defines:
            shaderDefines.append(f"-D{define}")

    hlslFile = shaderPath
    spvFile  = inShaderConfig.getName() + ".spv"
    entryPoint = inShaderConfig.entryPoint

    # Get proper file extension for executable
    compilerPath = FixExePath(compilerPath)

    commandArgs = [compilerPath]

    if inDXC:
        isLibrary = inShaderConfig.isLibrary()
        shaderProfileString = SHADER_PROFILE_LIB if isLibrary else SHADER_PROFILE
        target_env = 'universal1.5' if isLibrary else 'vulkan1.1'

        commandArgs += [f'-fspv-target-env={target_env}']
        commandArgs += ['-spirv']
        if entryPoint:
            commandArgs += ['-E', entryPoint]
        commandArgs += ['-DAMD_VULKAN', '-DAMD_VULKAN_DXC', '-DAMD_VULKAN_SPV']
        commandArgs += shaderDefines
        commandArgs += ['-fvk-use-scalar-layout']
        if isLibrary:
            commandArgs += ['-fcgl']
        commandArgs += ['-Od']
        commandArgs += ['-Vd']
        commandArgs += ['-T', shaderProfileString]
        commandArgs += ['-Wno-ignored-attributes']
        commandArgs += ['-Wno-parentheses-equality']
        commandArgs += ['-Fo', spvFile]
        commandArgs += [hlslFile]
    else:
        commandArgs += ['-V', hlslFile]
        commandArgs += ['-o', spvFile]
        commandArgs += ['-D']
        commandArgs += ['-S', 'comp']
        commandArgs += ['--entry-point', 'TraceRaysRTPSO']
        commandArgs += ['--keep-uncalled']
        commandArgs += ['-Od']
        commandArgs += ['-DDEFINE_RAYDESC=1', '-DAMD_VULKAN', '-DAMD_VULKAN_GLSLANG', '-DAMD_VULKAN_SPV']
#if GPURT_BUILD_RTIP2
        commandArgs += ['-DGPURT_BUILD_RTIP2']
#endif
        commandArgs += shaderDefines

    compileResult = InvokeSubprocess(commandArgs, outputDir, threadOutput, linuxLibraryPath=(compilerPath if inDXC else ''))

    if compileResult != 0:
        return False

    if inVerbose:
        # non-essential command, so return code can be ignored
        InvokeSubprocess(['spirv-dis', spvFile, '-o', spvFile + 'as'], outputDir, threadOutput)

    return True

# returns true if compiled successfully, false if failed.
def CompileShaderConfig(shaderConfig, args, shadersOutputDir,
    dxcompilerLibPath, spirvCompilerPath, spvRemap, whiteListPath, internalShadersHeader, shadersBasePath):
    # Output for this thread if we need to report errors
    threadOutput = []

    # Shader name
    conversionOutputFilename = shaderConfig.getName()

    # Identify if we're compiling a shader or library
    isLibrary = shaderConfig.isLibrary()

    # Is it a BVH shader rather than a trace shader
    isBVH = shaderConfig.isBVH()

    # Attempt to compile the shader. Exit the loop if we fail.
    isSpirv = isSpirvShader(shaderConfig, args)

    # Add Defines passed from CMAKE
    shaderConfig.defines = args.defines.replace(";",",") + ("" if shaderConfig.defines is None else str("," + shaderConfig.defines))

    if isBVH:
        shaderConfig.defines += ",GPURT_BVH_BUILD_SHADER=1"

    # Create a temporary working directory for this shader
    tempDirPath = shadersOutputDir + '/' + conversionOutputFilename

    # Get rid of temp dir that can be left over from an unclean build.
    if os.path.exists(tempDirPath):
        RemoveFolder(tempDirPath)

    os.mkdir(tempDirPath)

    # Get time for profiling
    startTime = time.time()

    try:
        if not isSpirv:
            pass
        else:
            if not RunSpirv(tempDirPath, spirvCompilerPath, shaderConfig, shadersBasePath, (not args.glslang or isBVH), args.verbose, threadOutput):
                threadOutput.append("Failed to compile Vulkan shader config %s" % shaderConfig)
                return (False, os.linesep.join(threadOutput))

        # Make sure we got a valid output filename
        if conversionOutputFilename is None:
            return (False, os.linesep.join(threadOutput))

        compilationString = "Converting compiled output..."

        conversionResult = False

        # Convert the file to a header
        compiledDxilFileName = None
        textDumpFileName = None
        compiledSpvFilename = None

        if not args.interm_only:
            if isSpirv:
                compiledSpvFilename = f"{conversionOutputFilename}.spv"
                conversionResult = ConvertSpvFile(args.disableDebugStripping, spvRemap, whiteListPath, compiledSpvFilename, conversionOutputFilename, isBVH, tempDirPath, threadOutput)

        if not args.interm_only:
            if compiledSpvFilename:
                RemoveFile(f"{tempDirPath}/{compiledSpvFilename}")

            if compiledDxilFileName:
                RemoveFile(compiledDxilFileName)

        if textDumpFileName:
            RemoveFile(textDumpFileName)

        if conversionResult == False and args.interm_only == False:
            compilationString += "Failure"
            threadOutput.append(compilationString)
            return (False, os.linesep.join(threadOutput))

        compilationString += "Success"
        threadOutput.append(compilationString)
        threadOutput.append("Compilation took %sms." % int((time.time() - startTime) * 1_000))

        return (True, os.linesep.join(threadOutput))
    except Exception as e:
        threadOutput.append(f"Error: {e}")
        return (False, os.linesep.join(threadOutput))
    finally:
        if not (args.interm_only or args.verbose):
            RemoveFolder(tempDirPath)

def FixInputPath(path) -> str:
    return os.path.abspath(path).replace('\\\\', '\\').replace('\\', '/')

def CompileShaders(args, internalShadersHeader, compileType) -> int:
    shaderConfigPath = FixInputPath(args.shaderConfig)
    shadersBasePath = FixInputPath(args.basepath)
    shadersOutputDir = FixInputPath(args.outputDir)

    spirvCompilerPath = FixInputPath(args.spirvCompilerPath) + '/'
    if args.glslang:
        spirvCompilerPath += GLSLANG_EXECUTABLE
    else:
        spirvCompilerPath += DXC_EXECUTABLE
    spirvCompilerPath = FixExePath(spirvCompilerPath)

    dxcompilerLibPath = FixInputPath(args.dxcompilerLibPath)

    spvRemap = FixExePath(FixInputPath(args.spirvRemapPath) + '/' + SPV_REMAP_EXECUTABLE)

    whiteListPath = FixInputPath(args.whiteListPath)

    # Parse the shader config file
    shadersConfigs = ParseShaderConfig(args, shaderConfigPath)

    # Make sure the output directory exists
    if not os.path.exists(shadersOutputDir):
        os.makedirs(shadersOutputDir)

    # Change our directory to the output directory
    os.chdir(shadersOutputDir)

    print("Launching threads for raytracing shader compilation...")
    compileResults = [None] * len(shadersConfigs)
    with ThreadPoolExecutor(max_workers=int(args.jobs)) as exe:
        for idx, shaderConfig in enumerate(shadersConfigs):
            compileResults[idx] = exe.submit(CompileShaderConfig, shaderConfig, args, shadersOutputDir,
                dxcompilerLibPath, spirvCompilerPath, spvRemap, whiteListPath, internalShadersHeader, shadersBasePath)

    successfulCompilation = True
    for idx, future in enumerate(compileResults):
        result, outputLog = future.result()
        shaderConfig = shadersConfigs[idx]
        shaderName = shaderConfig.getName()
        if result == False:
            print(f"Compilation failed for shader {shaderName}", file=sys.stderr)
            print("Output log:", file=sys.stderr)
            print(outputLog, file=sys.stderr)
            successfulCompilation = False
        elif args.verbose:
            print(f"Output log for shader {shaderName}:")
            print(outputLog)

    # Append `#include`s into the internal_shaders header
    for shaderConfig in shadersConfigs:
        shaderName = shaderConfig.getName()
        if isSpirvShader(shaderConfig, args):
            shaderName = shaderName if not shaderConfig.isBVH() else shaderName + '_spv'

        dir = ""
        if compileType == CompilationType.VulkanRtSpirv:
            dir = "spv/"
        elif compileType == CompilationType.Dxil:
            # empty dir
            pass
        else:
            print(f"Unhandled compilation type: {compileType}")
            return 1
        internalShadersHeader.write(f"#include \"{dir}g_{shaderName}.h\"\n")

    if successfulCompilation:
        print("Raytracing shader compilation completed with no errors.")

    return 0 if successfulCompilation else 1

def main() -> int:
    result = 0

    parser = argparse.ArgumentParser(description='RT Shader Compilation Script')
    parser.add_argument('basepath', help='base path of the directory that contains the raytracing shaders')
    parser.add_argument('--shaderConfig', help='XML file that contains information about the shaders that need to be compiled', default=DEFAULT_SHADER_CONFIG)
    parser.add_argument('--outputDir', help='Output directory for compiled shaders', default=DEFAULT_OUTPUT_DIR)
    parser.add_argument('--vulkan', action='store_const', const=True, help='Output Vulkan shaders', default=False)
    parser.add_argument('--glslang', action='store_const', const=True, help='Use glslangValidator (experimental) for Vulkan SPIRV generation instead of DXC', default=False)
    parser.add_argument('--disableDebugStripping', action='store_const', const=True, help='Disable Vulkan SPIRV debug symbol stripping', default=False)
    parser.add_argument('--interm-only', action='store_true', help="Generate intermediate SPIRV/DXIL instead of headers", default=False)
    parser.add_argument('--skip-bvh', action='store_true', help='Skip updating BVH shaders', default=False)
    parser.add_argument('--skip-trace', action='store_true', help='Skip updating traversal shaders', default=False)
    parser.add_argument('--verbose', action='store_true', help='Output verbose inforation', default=False)
    parser.add_argument('--defines', help='Defines for the shader compiler.', default="")
    parser.add_argument('--spirvCompilerPath', help='Path to SPIR-V compiler (either dxc or glslangValidator).', default='./dxc.exe')
    parser.add_argument('--dxcompilerLibPath', help='Path to dxcompiler.dll/libdxcompiler.so', default='./dxcompiler.dll')
    parser.add_argument('--spirvRemapPath', help='Path to spirv-remap executable', default='./spirv-remap.exe')
    parser.add_argument('--whiteListPath', help='Path to SPIRV whitelist file', default=SPIRV_WHITELIST)
    parser.add_argument('--jobs', help='Number of job threads to compile with', default=os.cpu_count())
    parser.add_argument('--spirv', action='store_true', help='Output SPIR-V for Vulkan for BVH shaders, need to be used with --vulkan', default=False)
    parser.add_argument('--strict', action='store_true', help='Require SSC invocations to finish cleanly without output or warnings.')

    originalPath = os.getcwd()

    args = parser.parse_args()
    outputDir = os.path.abspath(args.outputDir).replace('\\', '/')

    # Make sure the output directory exists
    if not os.path.exists(outputDir):
        os.mkdir(outputDir)

    # Generate the internal_shaders header, containing includes for all the generated headers
    internalShadersHeader = open(outputDir + '/' + DEFAULT_INTERNAL_SHADERS_HEADER_NAME + ".h", "w")
    internalShadersHeader.write(FILE_STANDARD_HEADER)

    compileType = CompilationType.Dxil

    if args.vulkan:
        # Compile traversal shaders first
        args.skip_bvh = True
        args.outputDir = outputDir # vk root dir
        result |= CompileShaders(args, internalShadersHeader, CompilationType.Dxil)
        # Compile BVH shaders later
        args.skip_bvh = False
        args.skip_trace = True
        if args.spirv:
            # For option --vulkan --spirv, only compile spv format
            args.outputDir = os.path.join(outputDir,"spv") # vk spv dir
            compileType = CompilationType.VulkanRtSpirv
        os.chdir(originalPath)

    result |= CompileShaders(args, internalShadersHeader, compileType)

    internalShadersHeader.flush()
    internalShadersHeader.close()

    return result

if __name__ == '__main__':
    sys.exit(main())
