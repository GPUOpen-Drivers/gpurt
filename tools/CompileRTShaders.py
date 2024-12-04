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


import cmd
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
import glob
import pathlib

DWORDS_PER_LINE = 8

FILE_STANDARD_HEADER = """
/* Copyright (c) 2022-2024 Advanced Micro Devices, Inc. All rights reserved. */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!
//
// This code has been generated automatically. Do not hand-modify this code.
//
// WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING! WARNING!  WARNING!  WARNING!  WARNING!
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

"""
FILE_HEADER_FORMAT_STRING = "static constexpr uint32 Cs%s[] =\n{\n"
FILE_FOOTER_STRING = "\n};\n"

SHADER_PROFILE = "cs_6_2"
SHADER_PROFILE_LIB = "lib_6_3"
DEFAULT_INTERNAL_SHADERS_HEADER_NAME = "g_internal_shaders"
DEFAULT_OUTPUT_DIR = "CompiledShaders"
SPIRV_WHITELIST = "strip_whitelist.txt"
SPV_REMAP_EXECUTABLE = "spirv-remap"
DXC_EXECUTABLE = "dxc"
maxLegacyRtIpLevel = 20

class ShaderConfig:
    def __init__(self, path, entryPoint=None, outputName=None, defines=None, includePaths=None):
        self.path = path
        self.entryPoint = entryPoint
        self.outputName = outputName
        self.defines = defines
        self.includePaths = includePaths

    def __str__(self):
        return f"ShaderConfig< Path: {self.path}, EntryPoint: {self.entryPoint}, OutputName: {self.outputName}, Defines: {self.defines}, isBVH: {self.isBVH()} >"

    def isLibrary(self):
        return self.entryPoint is None

    # The output filename is either explicitly specified or we default to the entry point
    def getName(self):
        return self.outputName or self.entryPoint

    # Only trace shaders are libraries
    def isBVH(self):
        return not self.isLibrary()

# Explicitly pass the legacy RtIp level as a separate define so HLSL code can determine whether its GPURT_RTIP_LEVEL is the legacy one.
commonTraceDefines = f"GPURT_RTIP_LEGACY_LEVEL={maxLegacyRtIpLevel}"

traceShaderConfigs = [
    ShaderConfig(path="GpuRtLibrary.hlsl", outputName="GpuRtLibrarySw", defines=f"GPURT_RTIP_LEVEL=0,{commonTraceDefines}"),
    ShaderConfig(path="GpuRtLibrary.hlsl", outputName="GpuRtLibrarySwDev", defines=f"DEVELOPER=1,GPURT_RTIP_LEVEL=0,{commonTraceDefines}"),
    # Below 2 lines will be removed after GPURT_MINIMUM_INTERFACE_MAJOR_VERSION is bumped to 48
    ShaderConfig(path="GpuRtLibrary.hlsl", outputName="GpuRtLibrary", defines=f"USE_HW_INTRINSIC=1,GPURT_RTIP_LEVEL=0,{commonTraceDefines}"),
    ShaderConfig(path="GpuRtLibrary.hlsl", outputName="GpuRtLibraryDev", defines=f"USE_HW_INTRINSIC=1,DEVELOPER=1,GPURT_RTIP_LEVEL=0,{commonTraceDefines}"),

    ShaderConfig(path="GpuRtLibrary.hlsl", outputName="GpuRtLibraryLegacy", defines=f"USE_HW_INTRINSIC=1,GPURT_RTIP_LEVEL={maxLegacyRtIpLevel},{commonTraceDefines}"),
    ShaderConfig(path="GpuRtLibrary.hlsl", outputName="GpuRtLibraryDevLegacy", defines=f"USE_HW_INTRINSIC=1,DEVELOPER=1,GPURT_RTIP_LEVEL={maxLegacyRtIpLevel},{commonTraceDefines}"),
]

bvhShaderConfigs = [
    ShaderConfig(path="Update.hlsl", entryPoint="UpdateTriangles"),
    ShaderConfig(path="Update.hlsl", entryPoint="UpdateAabbs"),
    ShaderConfig(path="EncodeNodes.hlsl", entryPoint="EncodeTriangleNodes"),
    ShaderConfig(path="EncodeNodes.hlsl", entryPoint="EncodeAABBNodes"),
    ShaderConfig(path="EncodeNodes.hlsl", entryPoint="EncodeQuadNodes"),
    ShaderConfig(path="EncodeTopLevel.hlsl", entryPoint="EncodeInstances"),
    ShaderConfig(path="BuildParallel.hlsl", entryPoint="BuildBvh", outputName="BuildParallel"),
    ShaderConfig(path="UpdateParallel.hlsl", entryPoint="UpdateParallel"),
    ShaderConfig(path="BuildBVHTDTR.hlsl", entryPoint="BuildBVHTD"),
    ShaderConfig(path="BuildBVHTDTR.hlsl", entryPoint="BuildBVHTD", outputName="BuildBVHTDTR", defines="USE_BVH_REBRAID=1"),
    ShaderConfig(path="GenerateMortonCodes.hlsl", entryPoint="GenerateMortonCodes"),
    ShaderConfig(path="Rebraid.hlsl", entryPoint="Rebraid"),
    ShaderConfig(path="BuildBVH.hlsl", entryPoint="BuildBVH", defines="USE_BUILD_LBVH=1"),
    ShaderConfig(path="BuildPLOC.hlsl", entryPoint="BuildPLOC"),
    ShaderConfig(path="UpdateQBVH.hlsl", entryPoint="UpdateQBVH"),
    ShaderConfig(path="RefitBounds.hlsl", entryPoint="RefitBounds"),
    ShaderConfig(path="ClearBuffer.hlsl", entryPoint="ClearBuffer"),
    ShaderConfig(path="../shadersClean/build/CopyBufferRaw.hlsl", entryPoint="CopyBufferRaw"),
    ShaderConfig(path="BuildQBVH.hlsl", entryPoint="BuildQBVH"),
    ShaderConfig(path="RadixSort/BitHistogram.hlsl", entryPoint="BitHistogram"),
    ShaderConfig(path="RadixSort/ScatterKeysAndValues.hlsl", entryPoint="ScatterKeysAndValues"),
    ShaderConfig(path="RadixSort/ScanExclusiveInt4.hlsl", entryPoint="ScanExclusiveInt4"),
    ShaderConfig(path="RadixSort/ScanExclusivePartInt4.hlsl", entryPoint="ScanExclusivePartInt4"),
    ShaderConfig(path="RadixSort/ScanExclusiveInt4DLB.hlsl", entryPoint="ScanExclusiveInt4DLB"),
    ShaderConfig(path="RadixSort/ScanExclusiveInt4DLB.hlsl", entryPoint="InitScanExclusiveInt4DLB"),
    ShaderConfig(path="RadixSort/DistributePartSumInt4.hlsl", entryPoint="DistributePartSumInt4"),
    ShaderConfig(path="EmitAS.hlsl", entryPoint="EmitCurrentSize"),
    ShaderConfig(path="EmitAS.hlsl", entryPoint="EmitCompactSize"),
    ShaderConfig(path="EmitAS.hlsl", entryPoint="EmitSerializeDesc"),
    ShaderConfig(path="EmitAS.hlsl", entryPoint="EmitToolVisDesc"),
    ShaderConfig(path="../shadersClean/build/CopyAS.hlsl", entryPoint="CopyAS"),
    ShaderConfig(path="CompactAS.hlsl", entryPoint="CompactAS"),
    ShaderConfig(path="DecodeAS.hlsl", entryPoint="DecodeAS"),
    ShaderConfig(path="SerializeAS.hlsl", entryPoint="SerializeAS"),
    ShaderConfig(path="DeserializeAS.hlsl", entryPoint="DeserializeAS"),
    ShaderConfig(path="InitExecuteIndirect.hlsl", entryPoint="InitExecuteIndirect", outputName="InitExecuteIndirect"),
    ShaderConfig(path="PairCompression.hlsl", entryPoint="PairCompression"),
    ShaderConfig(path="MergeSort.hlsl", entryPoint="MergeSort"),
    ShaderConfig(path="MergeSort.hlsl", entryPoint="MergeSortLocal"),
    ShaderConfig(path="MergeSort.hlsl", entryPoint="MergeSortGlobalIteration"),
    ShaderConfig(path="MergeSort.hlsl", entryPoint="MergeSortCopyLastLevel"),
    ShaderConfig(path="InitAccelerationStructure.hlsl", entryPoint="InitAccelerationStructure"),
    ShaderConfig(path="InitAccelerationStructure.hlsl", entryPoint="InitAccelerationStructure", defines="IS_UPDATE=1", outputName="InitUpdateAccelerationStructure"),
    ShaderConfig(path="BuildFastAgglomerativeLbvh.hlsl", entryPoint="BuildFastAgglomerativeLbvh"),
]

def getBaseDxcCommandArgs(isBvh:bool, isLibrary:bool, isSpirv:bool):
    dxcOptions = []
    dxcOptions += ["-Vd"]
    dxcOptions += ["-O3"]
    if isSpirv:
        dxcOptions += ["-spirv"]
        dxcOptions += ["-fspv-target-env=universal1.5", "-fcgl"] if isLibrary else ["-fspv-target-env=vulkan1.1"]
        dxcOptions += ["-DAMD_VULKAN", "-DAMD_VULKAN_DXC", "-DAMD_VULKAN_SPV"]
        dxcOptions += ["-fvk-use-scalar-layout"]

    dxcOptions += ["-Wall", "-Wextra"]

    dxcOptions +=  ["-Wno-ignored-attributes",
                    "-Wno-parentheses-equality",
                    "-Wno-parameter-usage",
                    "-Wno-unused-variable",
                    "-Wno-unused-function",
                    "-Wno-unused-parameter",
                    "-Wno-unknown-pragmas",
                    "-Wno-sometimes-uninitialized",
                    "-Wno-uninitialized",
                    "-Wno-conversion",
                    "-Wno-parameter-usage",
                    ]

    dxcOptions += ["-enable-16bit-types"]
    dxcOptions += ["-T", SHADER_PROFILE_LIB] if isLibrary else  ["-T", SHADER_PROFILE]
    # Currently rayquery takes a 8% perf hit in *only* gankino when traversal is compiled with hlsl 2021
    dxcOptions += ["-HV", "2021"] if isBvh else ["-HV", "2018"]

    return dxcOptions

"""
Combines args into an array of strings that can be used as compilation command by InvokeSubprocess.
Output command lacks: filename, -M flag for listing includes and entrypoint-specific defines like USE_HW_INTRINSIC
"""
def getValidationCmdArgs(args) -> [str]:

    compilerPath = FixInputPath(args.compilerPath) + '/'
    compilerPath += DXC_EXECUTABLE
    compilerPath = FixExePath(compilerPath)

    validateCommand =  [compilerPath]
    validateCommand += getBaseDxcCommandArgs(True, True, args.spirv)
    validateCommand += ["-Wno-misplaced-attributes"] # -Wmisplaced-attributes is triggered by [RootSignature()]
                                                     # used by entrypoint code and compiled as library
    validateCommand += ['-Fo', 'temp.bin']

    validateCommand += ['-DLIBRARY_COMPILATION']

    #use defines from cmake
    for d in args.defines.split(' '):
        d = d.strip()
        if d != '':
            validateCommand += ['-D' + d]
    if args.vulkan:
        validateCommand += ['-DAMD_VULKAN']
    #developer only ever adds, so we enable it globally for validation
    validateCommand += ['-DDEVELOPER']
    validateCommand += ['-DUSE_HW_INTRINSIC=1']

    #use include pathes from cmake
    for p in args.includePaths.split(' '):
        p = p.strip()
        if p != '':
            validateCommand += ['-I', p]

    return validateCommand

def removeSuffix(path: pathlib.Path, suffix: str) -> pathlib.Path:
    return pathlib.Path(path.as_posix()[:-len(suffix)])

"""
Finds all implementation-interface pairs of files under basePath (recursively).
Outputs dict of filenames (without extension) to pair of bools meaning (has_implementation, has_interface)
"""
def getImplInterfacePairs(directory: pathlib.Path, implementationSuffix: str, interfaceSuffix: str) -> {pathlib.Path, (bool, bool)}:
    # pairs -> {pair_path_without_extension: (has_implementation, has_interface)}
    pairs = {}
    # insert implementation part of pairs
    for implPath in directory.rglob("*" + implementationSuffix):
        pairs[removeSuffix(implPath.resolve(), implementationSuffix)] = (True, False)

    # insert interface part of pairs
    for interfacePath in directory.rglob("*" + interfaceSuffix):
        withoutSuffix = removeSuffix(interfacePath.resolve(), interfaceSuffix)
        hasImplFile = pairs.get(withoutSuffix, (False, False))[0]
        pairs[withoutSuffix] = (hasImplFile, True)

    return pairs

validationSpecialCaseDefines = {x.path:x.defines for x in list(filter(lambda a : a.defines is not None, bvhShaderConfigs))}

"""
Some files/functions can be included conditionally behind ifdefs.
This function combines defines, so that we can test compilation with different combinations of defines.
"""
def getDefineCombos(path: pathlib.Path) -> [[str]]:
    # required to test all combos as we cannot know how these interact
    # it's only partial compilation, so it should not take a lot of time
    # tests 1 or 2 combos (only 1 if there are no file specific defines)
    # - default defines
    # - default defines + file specific defines

    defineCombos = [[]]
    filename = path.name
    if filename in validationSpecialCaseDefines.keys():
        temp = validationSpecialCaseDefines[filename].split(',')
        temp = list(map(lambda a : "-D"+a, temp))
        defineCombos += [temp]

    return defineCombos

"""
shaderClean's hlsl-hlsli pair is considered clean when:
1. let [(dir, suffix)] = allowedDirSuffix, it includes only -suffix files from dir/ directory, and
2. it does not include any other files except its own .hlsl file.
"""
def validateIncludes(cmd: [str], path: pathlib.Path, implSuffix: str, interfaceSuffix: str,
                     allowedDirSuffix: [(pathlib.Path, str)]) -> bool:
    allowedDirSuffix = [(dirPath.as_posix(), suffix) for (dirPath, suffix) in allowedDirSuffix]
    listIncludesCmd = cmd + ["-M"]
    threadOutput = []
    retVal = InvokeSubprocess(listIncludesCmd, None, threadOutput, linuxLibraryPath=listIncludesCmd[0], expectNoOutput=False)
    assert retVal == 0, "Could not list includes of {0} with cmd {1} because:\n {2}".format(path, listIncludesCmd, "\n".join(threadOutput))

    includedFilesStr = set()
    for line in threadOutput[0].split("\n")[1:]:
        # use resolve() + as_posix() to avoid path mismatches when using drive mapping
        includedFilesStr |= {pathlib.Path(line.strip(" \n\r\t\\/")).resolve().as_posix()}
    includedFilesStr -= {path.as_posix() + implSuffix}
    includedFilesStr -= {path.as_posix() + interfaceSuffix}

    for includedFileStr in includedFilesStr:
        isAllowed = False
        for (dirStr, suffix) in allowedDirSuffix:
            if (includedFileStr.endswith(suffix)) and (dirStr in includedFileStr):
                isAllowed = True
                break

        if not isAllowed:
            print("GPURT clean shader validation failed:")
            print("\t{0} includes {1} which is not allowed.".format(path, includedFileStr))
            return False

    return True

"""
hlsl-hlsli pairs must compile on its own. It tests whether pairs contain or include everything needed.
If they do it allows including them anywhere in any order, except for some macros.
"""
def validateCompilation(cmd: [str], path: pathlib.Path) -> bool:
    threadOutput = []
    retVal = InvokeSubprocess(cmd, None, threadOutput, linuxLibraryPath=cmd[0], expectNoOutput=False)
    if retVal != 0:
        print("GPURT clean shader validation failed:")
        print("\tCould not compile {0} as library with cmd {1} because:\n {2}".format(path, cmd, threadOutput[-1]))
        return False

    return True

"""
Validates the organization of files in the shared folder to enforce cpp/h a src/header sort of structure
This helps keep the shader library untangled and easier to maintain.
#define'ing LIBRARY_COMPILATION enables including files in any order and does not include implementation dependencies.
"""
def validateShared(args) -> bool:
    cmdBase = getValidationCmdArgs(args)
    # use resolve() + as_posix() to avoid path mismatches when using drive mapping
    srcPath = pathlib.Path(FixInputPath(args.basepath)).parent.resolve()

    gpurtInterfacePath = (srcPath / "../gpurt").resolve()
    sharedPath = srcPath / "shared"
    generatedFilepath = pathlib.Path(args.g_FilePath)
    implExt = "._unused_"
    headerExt = ".h"

    # shared files need to be able to include the gpurt interface files due to the requirements of the interface
    # we treat this as an exception for rules about what files can be included

    for path, (hasImpl, hasHeader) in getImplInterfacePairs(sharedPath, implExt, headerExt).items():
        assert (hasHeader and not hasImpl), "Shared files should be header only."
        fullPath = path.with_suffix(path.suffix + (implExt if hasImpl else headerExt))
        for defines in getDefineCombos(fullPath):
            compileCmd = cmdBase + defines + [fullPath.as_posix()]
            if not validateIncludes(compileCmd, path, implExt, headerExt, [(sharedPath, headerExt), (gpurtInterfacePath, ".h"), (generatedFilepath, ".h")]):
                return False
            if not validateCompilation(compileCmd, fullPath):
                return False

    return True

"""
Validates the organization of shaders to enforce cpp/h a src/header sort of structure
This helps keep the shader library untangled and easier to maintain.
#define'ing LIBRARY_COMPILATION enables including files in any order and does not include implementation dependencies.
"""
def validateShadersClean(args) -> bool:
    cmdBase = getValidationCmdArgs(args)
    # use resolve() + as_posix() to avoid path mismatches when using drive mapping
    srcPath = pathlib.Path(FixInputPath(args.basepath)).parent.resolve()

    gpurtInterfacePath = (srcPath / "../gpurt").resolve()
    sharedPath = srcPath / "shared"
    generatedFilepath = pathlib.Path(args.g_FilePath)
# Validation of the shadersClean folder
    shadersCleanPath = srcPath / "shadersClean"

    implExt = ".hlsl"
    headerExt = ".hlsli"
    for path, (hasImpl, hasHeader) in getImplInterfacePairs(shadersCleanPath, implExt, headerExt).items():
        assert (hasImpl or hasHeader), "There should not be files without impl nor header."
        fullPath = path.with_suffix(path.suffix + (implExt if hasImpl else headerExt))
        for defines in getDefineCombos(fullPath):
            compileCmd = cmdBase + defines + [fullPath.as_posix()]
            if not validateIncludes(compileCmd, path, implExt, headerExt, [(shadersCleanPath, headerExt), (sharedPath, ".hlsli"), (gpurtInterfacePath, ".h"), (generatedFilepath, ".h")]):
                return False
            if not validateCompilation(compileCmd, fullPath):
                return False
    return True

def isSpirvShader(shaderConfig, args):
    return args.vulkan and (shaderConfig.isLibrary() or (shaderConfig.isBVH() and args.spirv))

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
    else:
        threadOutput.append(compileOutput[0].decode(sys.stdout.encoding).strip())
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
        # Generate file name with *_spv.h and file header name with Cs*_spv[] for SPIR-V shaders to distinguish
        # them with the AMDIL header
        outputFileName = "g_" + inOutputName + '_spv.h'
        conversionOutputFilename = inOutputName + '_spv'

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

def ConvertDxilFile(inDxilFilename, inOutputName, threadOutput, addDxilPostfix):
    try:
        postfix = ''
        # Generate file name with *_dxil.h and file header name with Cs*_dxil[] for DXIL shaders to distinguish
        # them with the AMDIL header
        if (addDxilPostfix):
            postfix = '_dxil'

        outputFileName = "g_" + inOutputName + postfix + '.h'
        conversionOutputFilename = inOutputName + postfix

        dxilBinaryFile = open(f"{inDxilFilename}", "rb")
        dxilBinData = dxilBinaryFile.read()
        dxilBinaryFile.close()

        i = 0
        dxilHexText = ""
        while i < len(dxilBinData):
            binWord = dxilBinData[i:i+4]
            intWord = struct.unpack('I', binWord)[0]
            hexWord = "{0:#010x}".format(intWord)
            dxilHexText += hexWord

            i += 4

            if (i != len(dxilBinData)):
                dxilHexText += ","
            if (i % 32 == 0):
                dxilHexText += "\n"
            else:
                dxilHexText += " "

        outputFile = open(outputFileName, "w")
        outputFile.write(FILE_STANDARD_HEADER)
        outputFile.write(FILE_HEADER_FORMAT_STRING % conversionOutputFilename)
        outputFile.write(dxilHexText)
        outputFile.write(FILE_FOOTER_STRING)
        outputFile.close()

        return True
    except Exception as e:
        threadOutput.append(f"Error: {e}")
        return False

def RunCompiler(outputDir, compilerPath, inShaderConfig, inShaderBasePath, inVerbose, threadOutput, spirv, dxilPostfix):
    shaderPath = os.path.join(inShaderBasePath, inShaderConfig.path).replace('\\', '/')
    shaderFilename = os.path.basename(inShaderConfig.path)

    detailString = ":" + inShaderConfig.entryPoint if inShaderConfig.entryPoint is not None else "<Library>"
    ext = ".h"
    if spirv:
        ext = "_spv" + ext
    elif dxilPostfix:
        ext = "_dxil" + ext
    genFileName = inShaderConfig.getName() + ext
    compilerName = os.path.split(compilerPath)[1]
    compilationString = "Compiling (%s spirv=%s) %s%s -> %s..." % (compilerName, spirv, shaderFilename, detailString, genFileName)

    shaderDefines = []
    if inShaderConfig.defines:
        defines = inShaderConfig.defines.split(',')
        for define in defines:
            shaderDefines.append(f"-D{define}")

    hlslFile = shaderPath
    if spirv:
        binFile  = inShaderConfig.getName() + ".spv"
    else:
        binFile  = inShaderConfig.getName() + ".dxil"
    entryPoint = inShaderConfig.entryPoint

    # Get proper file extension for executable
    compilerPath = FixExePath(compilerPath)

    commandArgs = [compilerPath]

    for p in inShaderConfig.includePaths.split(','):
        commandArgs += ['-I', p]

    commandArgs += getBaseDxcCommandArgs(inShaderConfig.isBVH(), inShaderConfig.isLibrary(), spirv)
    commandArgs += shaderDefines
    commandArgs += ['-Fo', binFile]
    commandArgs += [hlslFile]
    if entryPoint:
        commandArgs += ['-E', entryPoint]

    compileResult = InvokeSubprocess(commandArgs, outputDir, threadOutput, linuxLibraryPath=compilerPath)

    compilationString += "Success" if (compileResult == 0) else "Failure"
    threadOutput.append(compilationString)
    print(compilationString)

    if compileResult != 0:
        return False

    if inVerbose and spirv:
        # non-essential command, so return code can be ignored
        InvokeSubprocess(['spirv-dis', binFile, '-o', binFile + 'as'], outputDir, threadOutput)

    return True

# returns true if compiled successfully, false if failed.
def CompileShaderConfig(shaderConfig, args, shadersOutputDir,
    dxcompilerLibPath, compilerPath, spvRemap, whiteListPath, shadersBasePath):
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
    shaderConfig.includePaths = args.includePaths.replace(";",",") + ("" if shaderConfig.includePaths is None else str("," + shaderConfig.includePaths))

    if isBVH:
        shaderConfig.defines += ",GPURT_BVH_BUILD_SHADER=1"

    # Create a temporary working directory for this shader.
    tempDirPath = shadersOutputDir + '/' + conversionOutputFilename

    # Get rid of temp dir that can be left over from an unclean build.
    if os.path.exists(tempDirPath):
        RemoveFolder(tempDirPath)

    os.mkdir(tempDirPath)

    # Get time for profiling
    startTime = time.time()

    try:
        if False:
            pass
        else:
            addDxilPostfix = True
            if not RunCompiler(tempDirPath, compilerPath, shaderConfig, shadersBasePath, args.verbose, threadOutput, args.spirv, addDxilPostfix):
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
            else:
                compiledDxilFileName = f"{tempDirPath}/{conversionOutputFilename}.dxil"
                addDxilPostfix = True
                conversionResult = ConvertDxilFile(compiledDxilFileName, conversionOutputFilename, threadOutput, addDxilPostfix)

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
    shadersBasePath = FixInputPath(args.basepath)
    shadersOutputDir = FixInputPath(args.outputDir)

    compilerPath = FixInputPath(args.compilerPath) + '/'
    compilerPath += DXC_EXECUTABLE
    compilerPath = FixExePath(compilerPath)

    dxcompilerLibPath = FixInputPath(args.dxcompilerLibPath)

    spvRemap = FixExePath(FixInputPath(args.spirvRemapPath) + '/' + SPV_REMAP_EXECUTABLE)

    whiteListPath = FixInputPath(args.whiteListPath)

    # Parse the shader config file
    shadersConfigs = []

    if not args.skip_bvh:
        shadersConfigs.extend(bvhShaderConfigs)
    if not args.skip_trace:
        shadersConfigs.extend(traceShaderConfigs)

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
                dxcompilerLibPath, compilerPath, spvRemap, whiteListPath, shadersBasePath)

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
        if internalShadersHeader:
            internalShadersHeader.write(f"#include \"{dir}g_{shaderName}.h\"\n")

    if successfulCompilation:
        print("Raytracing shader compilation completed with no errors.")

    return 0 if successfulCompilation else 1

def main() -> int:
    result = 0

    parser = argparse.ArgumentParser(description='RT Shader Compilation Script')
    parser.add_argument('basepath', help='base path of the directory that contains the raytracing shaders')
    parser.add_argument('--outputDir', help='Output directory for compiled shaders', default=DEFAULT_OUTPUT_DIR)
    parser.add_argument('--vulkan', action='store_const', const=True, help='Output Vulkan shaders', default=False)
    parser.add_argument('--disableDebugStripping', action='store_const', const=True, help='Disable Vulkan SPIRV debug symbol stripping', default=False)
    parser.add_argument('--interm-only', action='store_true', help="Generate intermediate SPIRV/DXIL instead of headers", default=False)
    parser.add_argument('--skip-bvh', action='store_true', help='Skip updating BVH shaders', default=False)
    parser.add_argument('--skip-trace', action='store_true', help='Skip updating traversal shaders', default=False)
    parser.add_argument('--verbose', action='store_true', help='Output verbose inforation', default=False)
    parser.add_argument('--defines', help='Defines for the shader compiler, separated by ; or ,.', default="")
    parser.add_argument('--includePaths', help='Include paths for the shader compiler, separated by ; or ,.', default="")
    parser.add_argument('--g_FilePath', help='Path to the build destination where generated headers are written', default="")
    parser.add_argument('--compilerPath', help='Path to standalone compiler.', default='./dxc.exe')
    parser.add_argument('--dxcompilerLibPath', help='Path to dxcompiler.dll/libdxcompiler.so', default='./dxcompiler.dll')
    parser.add_argument('--spirvRemapPath', help='Path to spirv-remap executable', default='./spirv-remap.exe')
    parser.add_argument('--whiteListPath', help='Path to SPIRV whitelist file', default=SPIRV_WHITELIST)
    parser.add_argument('--jobs', help='Number of job threads to compile with', default=os.cpu_count())
    parser.add_argument('--spirv', action='store_true', help='Output SPIR-V for Vulkan for BVH shaders, need to be used with --vulkan', default=False)
    parser.add_argument('--strict', action='store_true', help='Require SSC invocations to finish cleanly without output or warnings.')
    parser.add_argument('--validateShadersClean', action='store_const',const=True, help='Run #include validation on shaders for style enforcement.', default=False)

    originalPath = os.getcwd()

    args = parser.parse_args()

    if args.validateShadersClean:
        print("Validating shadersClean directory.")
        tBegin = time.perf_counter()

        validIncludes = validateShadersClean(args)
        validIncludes &= validateShared(args)

        # For vulkan, we validate SPIR-V shaders in the same run instead of running the script again.
        if args.vulkan and not args.spirv:
            print("Now doing SPIR-V validation...")
            args.spirv = True
            validIncludes &= validateShadersClean(args)
            validIncludes &= validateShared(args)

        tDuration = time.perf_counter() - tBegin
        if validIncludes:
            tDuration = round(time.perf_counter() - tBegin, 4)
            print(f"Validated shadersClean directory in {tDuration}s.")
        else:
            print("Some files are not clean. See errors above.")
        return 0 if validIncludes else -1

    outputDir = os.path.abspath(args.outputDir).replace('\\', '/')

    # Make sure the output directory exists
    if not os.path.exists(outputDir):
        os.mkdir(outputDir)

    # Generate the internal_shaders header, containing includes for all the generated headers
    internalShadersHeader = None
    if not args.skip_bvh:
        internalShadersHeader = open(outputDir + '/' + DEFAULT_INTERNAL_SHADERS_HEADER_NAME + ".h", "w")
        internalShadersHeader.write(FILE_STANDARD_HEADER)

    compileType = CompilationType.Dxil

    if args.vulkan and not args.skip_bvh:
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

    if internalShadersHeader:
        internalShadersHeader.flush()
        internalShadersHeader.close()

    return result

if __name__ == '__main__':
    sys.exit(main())
