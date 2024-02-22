##
 #######################################################################################################################
 #
 #  Copyright (c) 2019-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

# This is a modified copy of timingReport.py, which is came from the PAL repo
# pal/tools/gpuProfilerTools/timingReport.py

import collections
import csv
import glob
import os
import re
import sys

#to prevent bad printing when "python script.py > outputfile.txt" is used
sys.stdout.reconfigure(encoding='utf-8')
class TH():
    QueueCallCol     = 0
    CmdBufIndexCol   = QueueCallCol + 1
    CmdBufCallCol    = CmdBufIndexCol + 1
    SubQueueIdxCol   = CmdBufCallCol + 1
    StartClockCol    = SubQueueIdxCol + 1
    EndClockCol      = StartClockCol + 1
    TimeCol          = EndClockCol + 1
    PipelineHashCol  = TimeCol + 1
    CompilerHashCol  = PipelineHashCol + 1
    VsCsCol          = CompilerHashCol + 1
    HsCol            = VsCsCol + 1
    DsCol            = HsCol + 1
    GsCol            = DsCol + 1
    PsCol            = GsCol + 1
    VertsThdGrpsCol  = PsCol + 1
    InstancesCol     = VertsThdGrpsCol + 1
    CommentsCol      = InstancesCol + 1

def decode_file_name(file_name) -> dict:
    search_obj  = re.search(r"frame([0-9]*)Dev([0-9]*)Eng(\D*)([0-9]*)-([0-9]*)\.csv", file_name)

    results_dict = {}
    results_dict["frameNum"]   = int(search_obj.group(1))
    results_dict["deviceNum"]  = int(search_obj.group(2))
    results_dict["engineType"] = search_obj.group(3)
    results_dict["engineId"]   = int(search_obj.group(4))
    results_dict["queueId"]    = int(search_obj.group(5))

    return results_dict

def itervalues(d):
    return iter(d.values())
def iteritems(d):
    return iter(d.items())

from enum import Enum
import re

enumName = "InternalRayTracingCsType"
enumVals = {}
enumRegex = "enum class InternalRayTracingCsType : uint32\n\{[.\S\s]*?\}."
try:
    gpurt_h = open("../../gpurt/gpurt.h", "r+")

    fileString = gpurt_h.read()
    original = re.findall(enumRegex, fileString)[0]
    possibleVals = list(map(lambda v: v.strip(" ,") , original.split("\n")[2:-2]))
    #print(possibleVals)
    unknownFromHere = False
    enumIter = 0
    for possibleVal in possibleVals:
        if possibleVal[0] == '#': #once we hit a #define, we can't confidently parse
            unknownFromHere = True#since we don't have the #defines from DXCP's build
            continue
        if unknownFromHere:
            enumVals["maybe"+(possibleVal)] = enumIter
        else:
            enumVals[possibleVal] = enumIter
        enumIter+=1

    enumVals["Unknown"] =enumIter
    enumIter+=1
    enumVals["TraceRay"] =enumIter
    enumIter+=1
    enumVals["RayQuery"] =enumIter
    enumIter+=1

    gpurt_h.close()
except:
    print("Error populating pipeline enum. Make sure you're running this file from the scope of the 'gpurt\\tools\\PALProfiler' folder")
    exit(-1)
InternalRayTracingCsType = Enum(enumName, enumVals)

def decode_file_name(file_name) -> dict:
    search_obj  = re.search(r"frame([0-9]*)Dev([0-9]*)Eng(\D*)([0-9]*)-([0-9]*)\.csv", file_name)

    results_dict = {}
    results_dict["frameNum"]   = int(search_obj.group(1))
    results_dict["deviceNum"]  = int(search_obj.group(2))
    results_dict["engineType"] = search_obj.group(3)
    results_dict["engineId"]   = int(search_obj.group(4))
    results_dict["queueId"]    = int(search_obj.group(5))

    return results_dict

def isValidHash(string):
    # A valid hash is a non-empty string that represents a non-zero hex value.
    return string and (int(string, 16) != 0)

rtBuildIds = [InternalRayTracingCsType.BuildParallel,
              InternalRayTracingCsType.GenerateMortonCodes,
              InternalRayTracingCsType.BuildBVHTD]
rtUpdateIds = [InternalRayTracingCsType.UpdateTriangles,
               InternalRayTracingCsType.UpdateAabbs,
               InternalRayTracingCsType.UpdateParallel,
               InternalRayTracingCsType.UpdateQBVH]

NextDispatchIsBottomLevel = True

TRACE_RAY_HASH_TOP = "0xbbb2ccc3"
RAY_QUERY_HASH_TOP = "0xaaa1bbb2"
BUILD_BVH_HASH_TOP = "0xeee5fff6"

def isRTPipeline(row):
    return BUILD_BVH_HASH_TOP in row[TH.PipelineHashCol]\
        or TRACE_RAY_HASH_TOP in row[TH.VsCsCol]\
        or RAY_QUERY_HASH_TOP in row[TH.PipelineHashCol]

def toInternalRayTracingCsType(row):
    if TRACE_RAY_HASH_TOP in row[TH.VsCsCol]: #traceray
        return InternalRayTracingCsType.TraceRay
    if RAY_QUERY_HASH_TOP in row[TH.PipelineHashCol]: #rayquery
        return InternalRayTracingCsType.RayQuery

    try:
        return InternalRayTracingCsType(int(row[TH.PipelineHashCol][-4:], 16))
    except:
        print("bad hash:\n", row, "\n")
        return InternalRayTracingCsType.Unknown

def DeterminePipelineType(row):
    if not row[TH.CompilerHashCol]:
        return "No Pipeline (BLT, Barrier, etc.)"
    if re.search("Dispatch", row[TH.CmdBufCallCol]):
        if TRACE_RAY_HASH_TOP in row[TH.VsCsCol]:
            return "TraceRayCs"
        if RAY_QUERY_HASH_TOP in row[TH.PipelineHashCol]:
            return "RayQueryCs"
        if BUILD_BVH_HASH_TOP in row[TH.PipelineHashCol]:
            prefix = "Blas" if NextDispatchIsBottomLevel else "Tlas"
            if toInternalRayTracingCsType(row) in rtBuildIds:
                return prefix+"BuildCs"
            if toInternalRayTracingCsType(row) in rtUpdateIds:
                return prefix+"UpdateCs"
            return prefix+"BuildCs2"
        return "Cs"
    if isValidHash(row[TH.HsCol]) and isValidHash(row[TH.GsCol]):
        return "VsHsDsGsPs"
    if isValidHash(row[TH.HsCol]):
        return "VsHsDsPs"
    if isValidHash(row[TH.GsCol]):
        return "VsGsPs"

    return "VsPs"

outputFile = None
origStdout = sys.stdout
def setupPrintToFile(filename):
    try:
        outputFile = open(filename, 'w')
        sys.stdout = outputFile
        return True
    except:
        return False

if len(sys.argv) > 3 or len(sys.argv) < 2:
    sys.exit("Usage: timingReport.py <full path to log folder> <optional output file>")
elif len(sys.argv) == 3:
    if not setupPrintToFile(sys.argv[2]):
        sys.exit("Usage: timingReport.py <full path to log folder> <optional output file>")

gpuFrameTime = 0

os.chdir(sys.argv[1])
files = glob.glob("frame*.csv")

if (len(files) == 0):
    sys.exit(f"ERROR: Looking at directory <{os.getcwd()}> \
             but cannot find any files that match the \"frame*.csv\" pattern.")

frames               = { }  # Frame num -> [ tsFreq, cmdBufClockPairs, total barrier time ]
rtFrames             = { }  # Frame num -> [ tsFreq, cmdBufClockPairs, total barrier time ]
perCallTable         = { }  # Device -> Engine -> QueueId -> Call -> [ count, totalTime ]
perPipelineTypeTable = { }  # PipelineType -> [ count, totalTime ]
perPipelineTable     = { }  # Pipeline Hash -> [ type, count, totalTime, vs/csHash, hsHash, dsHash, gsHash, psHash ]
perPsTable           = { }  # PS Hash -> [ count, totalTime ]
pipelineRangeTable   = { }  # Frame num -> EngineType -> PipelineHash ->
                            #     [(startClock1, endClock1, time1), (startClock2, endClock2, time2), ...]
perPipelinePerEngineTypeTable = { } # Engine -> PipelineType -> [ count, totalTime ]
perRTPipelinePerEngineTypeTable = { }
perRTPipelineTable = { }

frameCount           = 0
submitCount          = 0
cmdBufCount          = 0

filesProcessedSoFar  = 0 # For printing parsing progress.

for file in files:
    if sys.stdout.isatty():
        sys.stdout.write(f"Parsing input files. {((filesProcessedSoFar / len(files)) * 100):.0f}% Complete.\r")

    filesProcessedSoFar += 1

    # Decode file name.
    decoded_file_name = decode_file_name(file)
    frameNum   = decoded_file_name["frameNum"]
    deviceNum  = decoded_file_name["deviceNum"]
    engineType = decoded_file_name["engineType"]
    engineId   = decoded_file_name["engineId"]
    queueId    = decoded_file_name["queueId"]

    # Track the fact we've never seen this frame before:
    # - Zero out the time spend in barriers for it.
    # - Place some empty maps in the pipelineRangeTable.
    if not frameNum in frames:
        frames[frameNum] = [0, [], 0]
        rtFrames[frameNum] = [0, [], 0]
        pipelineRangeTable[frameNum] = { "Ace" : {}, "Dma" : {}, "Gfx" : {} }

    # Expand C,D,U to full engine type name.
    if engineType == "Ace":
        engineKey = "Compute"
    elif engineType == "Dma":
        engineKey = "DMA"
    elif engineType == "Gfx":
        engineKey = "Universal"
    else:
        continue

    # Create readable keys for perCallTable, will be displayed later.
    deviceKey = "Device " + str(deviceNum)
    engineKey = engineKey + " Engine " + str(engineId)
    queueKey  = "Queue " + str(queueId)

    if deviceKey not in perCallTable.keys():
        perCallTable[deviceKey] = { }
    if engineKey not in perCallTable[deviceKey].keys():
        perCallTable[deviceKey][engineKey] = { }
    if queueKey not in perCallTable[deviceKey][engineKey].keys():
        perCallTable[deviceKey][engineKey][queueKey] = { }

    with open(file) as csvFile:
        reader = csv.reader(csvFile, skipinitialspace=True)
        headers = next(reader)

        tsFreqSearch        = re.search(".*Frequency: (\d+).*", headers[TH.TimeCol])
        frames[frameNum][0] = int(tsFreqSearch.group(1))
        rtFrames[frameNum][0] = int(tsFreqSearch.group(1))

        for row in reader:
            if "BVH Build Settings:" in row[TH.CommentsCol]:
                if "BottomLevel" in row[TH.CommentsCol]:
                    NextDispatchIsBottomLevel = True
                elif "TopLevel" in row[TH.CommentsCol]:
                    NextDispatchIsBottomLevel = False

            if row[TH.QueueCallCol] == "Submit()":
                submitCount += 1
            if row[TH.CmdBufCallCol] == "Begin()" and row[TH.StartClockCol]:
                frames[frameNum][1].append((int(row[TH.StartClockCol]), int(row[TH.EndClockCol])))
                cmdBufCount += 1
            if row[TH.TimeCol]:

                #timing stuff I think?
                if row[TH.CmdBufCallCol] in perCallTable[deviceKey][engineKey][queueKey].keys():
                    perCallTable[deviceKey][engineKey][queueKey][row[TH.CmdBufCallCol]][0] += 1
                    perCallTable[deviceKey][engineKey][queueKey][row[TH.CmdBufCallCol]][1] += float(row[TH.TimeCol])
                else:
                    perCallTable[deviceKey][engineKey][queueKey][row[TH.CmdBufCallCol]] = [ 1, float(row[TH.TimeCol]) ]

                pipelineType = DeterminePipelineType(row)

                #Per Pipeline Stuff
                if pipelineType in perPipelineTypeTable:
                    perPipelineTypeTable[pipelineType][0] += 1
                    perPipelineTypeTable[pipelineType][1] += float(row[TH.TimeCol])
                else:
                    perPipelineTypeTable[pipelineType] = [ 1, float(row[TH.TimeCol]) ]

                #Per Pipline Per Engine Stuff
                if not engineKey in perPipelinePerEngineTypeTable:
                    perPipelinePerEngineTypeTable[engineKey] = {}
                if pipelineType in perPipelinePerEngineTypeTable[engineKey]:
                    perPipelinePerEngineTypeTable[engineKey][pipelineType][0] += 1
                    perPipelinePerEngineTypeTable[engineKey][pipelineType][1] += float(row[TH.TimeCol])
                else:
                    perPipelinePerEngineTypeTable[engineKey][pipelineType] = [ 1, float(row[TH.TimeCol]) ]

                if row[TH.CompilerHashCol]:
                    # Update the perPipelineTable totals.
                    # Note that in practice the compiler hash is most useful because it's in all of the pipeline dumps.
                    if row[TH.CompilerHashCol] in perPipelineTable:
                        perPipelineTable[row[TH.CompilerHashCol]][1] += 1
                        perPipelineTable[row[TH.CompilerHashCol]][2] += float(row[TH.TimeCol])
                    else:
                        perPipelineTable[row[TH.CompilerHashCol]] = [ pipelineType, 1, float(row[TH.TimeCol]), row[TH.VsCsCol],\
                                                                   row[TH.HsCol], row[TH.DsCol], row[TH.GsCol], row[TH.PsCol] ]
                    #RT specific
                    if isRTPipeline(row):
                        rtFrames[frameNum][1].append((int(row[TH.StartClockCol]), int(row[TH.EndClockCol])))

                        if not engineKey in perRTPipelinePerEngineTypeTable:
                            perRTPipelinePerEngineTypeTable[engineKey] = {}

                        #Per Pipeline Stuff
                        if row[TH.CompilerHashCol] in perRTPipelineTable:
                            perRTPipelineTable[row[TH.CompilerHashCol]][1] += 1
                            perRTPipelineTable[row[TH.CompilerHashCol]][2] += float(row[TH.TimeCol])
                        else:
                            perRTPipelineTable[row[TH.CompilerHashCol]] = [ toInternalRayTracingCsType(row).name, 1, float(row[TH.TimeCol]), row[TH.VsCsCol],\
                                                                   row[TH.HsCol], row[TH.DsCol], row[TH.GsCol], row[TH.PsCol] ]

                        #Per Pipeline Per Engine Stuf
                        if row[TH.CompilerHashCol] in perRTPipelinePerEngineTypeTable[engineKey]:
                            perRTPipelinePerEngineTypeTable[engineKey][row[TH.CompilerHashCol]][1] += 1
                            perRTPipelinePerEngineTypeTable[engineKey][row[TH.CompilerHashCol]][2] += float(row[TH.TimeCol])
                        else:
                            perRTPipelinePerEngineTypeTable[engineKey][row[TH.CompilerHashCol]] = [ toInternalRayTracingCsType(row).name, 1, float(row[TH.TimeCol]), row[TH.VsCsCol],\
                                                                   row[TH.HsCol], row[TH.DsCol], row[TH.GsCol], row[TH.PsCol] ]

                    # Record the start and end clocks and the time of this shader work in the pipelineRangeTable.
                    # Note that we may divide by zero later unless we exclude rows with identical start and end clocks.
                    startClock = int(row[TH.StartClockCol])
                    endClock   = int(row[TH.EndClockCol])
                    if (endClock - startClock) > 0:
                        if row[TH.CompilerHashCol] in pipelineRangeTable[frameNum][engineType]:
                            pipelineRangeTable[frameNum][engineType][row[TH.CompilerHashCol]].append((startClock,\
                                                                                                   endClock,\
                                                                                                   float(row[TH.TimeCol])))
                        else:
                            pipelineRangeTable[frameNum][engineType][row[TH.CompilerHashCol]] = [(startClock,\
                                                                                               endClock,\
                                                                                               float(row[TH.TimeCol]))]

                if row[TH.PsCol]:
                    if row[TH.PsCol] in perPsTable:
                        perPsTable[row[TH.PsCol]][0] += 1
                        perPsTable[row[TH.PsCol]][1] += float(row[TH.TimeCol])
                    else:
                        perPsTable[row[TH.PsCol]] = [ 1, float(row[TH.TimeCol]) ]

                if row[TH.CmdBufCallCol] == "CmdBarrier()":
                    frames[frameNum][2] += float(row[TH.TimeCol])

        csvFile.close

# Compute the sum of all GPU frame times, where the time of a single frame is the amount of time the GPU spent being
# busy. We can do this by creating a list of all GPU clock ranges when the GPU was busy from the list of all command
# buffer clock ranges like so:
# - For the current frame, sort the list of top-level command buffer (begin, end) clocks by increasing begin time.
# - Pop the top (begin, end) pair and use it to start a new GPU busy range.
# - While the top pair overlaps with the busy range, update the busy range with the latest ending time and pop
#   the top pair.
# - Once there are no more overlapping ranges, push the current range as a complete busy range and repeat.
# Once that is done we can simply sum the busy ranges to get the ammount of time the GPU was busy for the current frame.
gpuFrameTime = 0
for frame in frames.keys():
    tsFreq        = frames[frame][0]
    orderedRanges = sorted(frames[frame][1], key=lambda x: x[0])
    busyRanges    = []
    while orderedRanges:
        (curBegin, curEnd) = orderedRanges.pop(0)
        while orderedRanges and orderedRanges[0][0] <= curEnd:
            curEnd = max(curEnd, orderedRanges[0][1])
            orderedRanges.pop(0)
        busyRanges.append((curBegin, curEnd))
    for (begin, end) in busyRanges:
        gpuFrameTime += ((1000000 * (end - begin)) / tsFreq)

frameCount    = int(len(frames))
gpuFrameTime /= frameCount

rtGpuFrameTime = 0
for frame in rtFrames.keys():
    tsFreq        = rtFrames[frame][0]
    orderedRanges = sorted(rtFrames[frame][1], key=lambda x: x[0])
    busyRanges    = []
    while orderedRanges:
        (curBegin, curEnd) = orderedRanges.pop(0)
        while orderedRanges and orderedRanges[0][0] <= curEnd:
            curEnd = max(curEnd, orderedRanges[0][1])
            orderedRanges.pop(0)
        busyRanges.append((curBegin, curEnd))
    for (begin, end) in busyRanges:
        rtGpuFrameTime += ((1000000 * (end - begin)) / tsFreq)

rtFrameCount    = int(len(rtFrames))
rtGpuFrameTime /= rtFrameCount
rtFramePercent = (rtGpuFrameTime/gpuFrameTime)*100

print("== Frame Timing Summary ==============\n")
print("Average GPU busy time per frame:    {0:.3f}ms ({1:,d} frames)".format(gpuFrameTime / 1000.0, frameCount))
print("Average RT GPU busy time per frame: {0:.3f}ms ({1:.3g}%)".format(rtGpuFrameTime / 1000.0, rtFramePercent))
print("Average submits per frame:          " + str(submitCount / frameCount))
print("Average command buffers per frame:  " + str(cmdBufCount / frameCount))
print("")

def printPerPypelineTypeTable(pptTable):
    print("   Pipeline Type                        | Avg. Call Count | Avg. GPU Time [us] | Avg. Frame %")
    print("  --------------------------------------+-----------------+--------------------|--------------")
    for pipelineType in collections.OrderedDict(sorted(pptTable.items(), key=lambda x: x[1][1], reverse=True)):
        timePerFrame = pptTable[pipelineType][1] / frameCount
        pctOfFrame   = (timePerFrame / gpuFrameTime) * 100
        print("  {0:37s} |    {1:12,.2f} |       {2:>12,.2f} |      {3:5.2f} %".
            format(pipelineType,
                pptTable[pipelineType][0] / frameCount,
                timePerFrame,
                pctOfFrame))
    print("\n")

print("== Frame Breakdown By Pipeline Type ==============\n")
printPerPypelineTypeTable(perPipelineTypeTable)

print("== Frame Breakdown By Pipeline Type Per Engine ==============\n")
for engineKey in perPipelinePerEngineTypeTable:
    print("engine:", engineKey)
    printPerPypelineTypeTable(perPipelinePerEngineTypeTable[engineKey])

def printPerRTPypelineTypeTable(prtpt):
    pipelineNum = 0
    print("   Compiler Hash         | Type                   | Avg. Call Count | Avg. GPU Time [us] | Avg. Frame %")
    print("  -----------------------+------------------------+-----------------+--------------------|--------------")
    for pipeline in collections.OrderedDict(sorted(prtpt.items(), key=lambda x: x[1][2], reverse=True)):
        pipelineNum += 1
        timePerFrame = prtpt[pipeline][2] / frameCount
        pctOfFrame   = (timePerFrame / gpuFrameTime) * 100

        print("  {0:2d}. {1:s} | {2:20s}   |    {3:12,.2f} |       {4:>12,.2f} |      {5:5.2f} %".
            format(pipelineNum,
                    pipeline,
                    prtpt[pipeline][0],
                    prtpt[pipeline][1] / frameCount,
                    timePerFrame,
                    pctOfFrame))
    print("\n")

print("== Frame Breakdown By RT Pipeline Type Per Engine ==============\n")
for engineKey in perRTPipelinePerEngineTypeTable:
    print("engine:", engineKey)
    printPerRTPypelineTypeTable(perRTPipelinePerEngineTypeTable[engineKey])

#cleanup stdout redirect
if outputFile:
    sys.stdout = origStdout
    outputFile.close()
