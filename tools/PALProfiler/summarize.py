##
 #######################################################################################################################
 #
 #  Copyright (c) 2023-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

import pathlib
import sys

if not len(sys.argv) == 2:
    sys.exit("Usage: summarize.py <full path to data folder>")

sys.stdout.reconfigure(encoding='utf-8')

baseDir = None

try:
    baseDir = pathlib.Path(sys.argv[1])
except:
    sys.exit("Bad data folder path")

dataFiles = list(baseDir.rglob("*.txt"))

fileNames = []

Calls = {}
Times = {}
Percs = {}

typicalEngineNames = ["Compute Engine 0", "DMA Engine 0", "Universal Engine 0"]
for engineName in typicalEngineNames:
    Calls[engineName] = []
    Times[engineName] = []
    Percs[engineName] = []

rtPipelineCategories = [
    "TlasBuildCs", "TlasBuildCs2", "TlasUpdateCs",
    "BlasBuildCs", "BlasBuildCs2", "BlasUpdateCs",
    "TraceRayCs", "RayQueryCs"]

for f in dataFiles:
    print("on file", f.name)
    with f.open() as openFile:
        lines = list(map(lambda x : x.strip(), openFile.readlines()))
        pipelineDict = {}
        lineIndex = 0
        while not "Frame Breakdown By Pipeline Type Per Engine" in lines[lineIndex]:
            lineIndex +=1
        lineIndex+=2
        while (lineIndex <= (len(lines)-1)) and "engine:" in lines[lineIndex]:
            engineName = lines[lineIndex].split(":")[1].strip()
            if not engineName in Calls.keys():
                Calls[engineName] = []
                Times[engineName] = []
                Percs[engineName] = []
            pipelineDict[engineName] = {}
            lineIndex += 3
            while "|" in lines[lineIndex]:
                lineData = lines[lineIndex].split("|")
                lineData = list(map(lambda x : x.strip("\n\r %"), lineData))
                pipelineDict[engineName][lineData[0]] = (lineData[1:4])
                lineIndex += 1
            lineIndex+=2

        #fudge 0 data for good csv data shape
        for engine in typicalEngineNames:
            if not engine in pipelineDict.keys():
                pipelineDict[engine] = {}
            for cat in rtPipelineCategories:
                if not cat in pipelineDict[engine].keys():
                    pipelineDict[engine][cat] = ["0", "0", "0"]

        for engineKey in Calls.keys():
            if engineKey in pipelineDict.keys():
                currCalls = [f.stem]
                currTimes = [f.stem]
                currPercs = [f.stem]
                for cat in rtPipelineCategories:
                    if cat in pipelineDict[engineKey].keys():
                        currCalls.append(pipelineDict[engineKey][cat][0])
                        currTimes.append(pipelineDict[engineKey][cat][1])
                        currPercs.append(pipelineDict[engineKey][cat][2])
                    else:
                        currCalls.append("0")
                        currTimes.append("0")
                        currPercs.append("0")

                Calls[engineKey].append(currCalls)
                Times[engineKey].append(currTimes)
                Percs[engineKey].append(currPercs)

headers = ["App"] + rtPipelineCategories

files = ["Calls", "Times", "Percs"]
for engineKey in Calls.keys():
    engineFileName = engineKey.replace(" ", "_")
    for csvFileName in files:
        with open(engineFileName + csvFileName + ".csv", "w+") as csvFile:
            csvFile.write((",".join(headers)) + "\n")
            currArr = globals()[csvFileName][engineKey] #eh
            for entry in currArr:
                entry = map(lambda x : x.replace(",", ""), entry)
                csvFile.write((",".join(entry)) + "\n")
