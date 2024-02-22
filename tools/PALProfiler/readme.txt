What:
    The PALProfiler folder is a collection of python scripts that parse the output of the PALProfiler.
    There is an existing script that does this in PAL, called, "timingReport.py", but this version is
    tuned specifically to output details about raytracing, both the bvh build and traversal. It takes
    the recording fo the dispatches, uses the hashes to determine which ones are which RT dispatches,
    and catagorizes/sums the timing for those catagories.

    rtTimingReport.py - this is the primary script that parses the CSV files the PAL profiler throws out;
        It outputs its data in the form of text to the console, but can also be redirected to a text files
        with an argument for use with summarize.py and summarizePerStage.py.

        Also, ANY TIME the output of this file changes, summarize.py and summarizePerStage.py have to
        account for it. I have plans to change that, but only plans.

    summarize.py - this takes a folder of multiple outputs of rtTimingReport.py. Its output is 9 files, a pairing of 3x3 categories.

        Engines: Graphics, Compute, DMA
        Data Type: Calls per frame, Time per frame, Percent of time per frame

        *For each of these, it has the data for a Pipeline Type (ie, Cs, VsPc, TlasBuildCs, TraceRayCs, ect)

        The point of this is to be able to more easily copy this data into an excel sheet for presenting it.
        Improvements are possible, and definitely are planned, but again, only planned.

    summarizePerStage.py - this takes a folder of multiple outputs of rtTimingReport.py. Its output is 9 files, a pairing of 3x3 categories.

        Engines: Graphics, Compute, DMA
        Data Type: Calls per frame, Time per frame, Percent of time per frame

        *For each of these, it has the data for a Specific Pipeline (ie BuildParallel, EncodeInstances, TraceRay, RayQuery, ect)

        The point of this is to be able to more easily copy this data into an excel sheet for presenting it.
        Improvements are possible, and definitely are planned, but again, only planned.

How:
    For individual script usage, see individual scripts
    The flow is -
    1. PAL profiler with mode=1 and granularity = 0
    2. Point rtTimingReport.py at that folder, and it will output the summary data
    3. Put all summary data files in a folder, and point either/both summarize.py and summarizePerStage.py at it.
