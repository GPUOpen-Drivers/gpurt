/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 **********************************************************************************************************************/

#ifndef __cplusplus

#define ACCEL_STRUCT_TRACKER_LIST_SIZE 16

struct AccelStructTracker
{
    uint count;      // Number of addresses logged so far
    uint lock;       // Spin lock protecting the list of addresses
    uint enable;     // Non-zero enables tracking
    uint outOfSpace; // Non-zero indicates the tracker ran out of slots

    // List of unique acceleration structures
    uint64_t accelStructAddrs[ACCEL_STRUCT_TRACKER_LIST_SIZE];
};

globallycoherent RWStructuredBuffer<AccelStructTracker> AccelStructTrackerBuf : register(u1, SPACEID);

//======================================================================================================================
static bool SearchAccelStructTrackerBufList(
    uint64_t vaToFind,
    out uint currentCount)
{
    currentCount = AccelStructTrackerBuf[0].count;

    bool found = false;

    for (uint i = 0; i < currentCount; i++)
    {
        const uint64_t accelStructVa = AccelStructTrackerBuf[0].accelStructAddrs[i];
        if (accelStructVa == vaToFind)
        {
            found = true;
            break;
        }
    }

    return found;
}

//======================================================================================================================
// Log the specified acceleration structure address in the tracker if it has not been logged yet.
static void LogOneAccelStruct(
    uint64_t accelStructVa)
{
    const uint metadataSize = FetchHeaderField(accelStructVa, ACCEL_STRUCT_HEADER_METADATA_SIZE_OFFSET);
    const uint64_t baseVa = accelStructVa - metadataSize;

    bool done = false;

    uint currentCount = 0;

    while (true)
    {
        DeviceMemoryBarrier();

        if (SearchAccelStructTrackerBufList(baseVa, currentCount))
        {
            // Found the current accel struct already in the list.
            done = true;
            break;
        }

        // List is full. We can't add a new accel struct.
        if (currentCount >= ACCEL_STRUCT_TRACKER_LIST_SIZE)
        {
            break;
        }

        uint orig = 1;
        InterlockedCompareExchange(AccelStructTrackerBuf[0].lock, 0, 1, orig);
        if (orig == 0)
        {
            // Only search after the lock has been acquired.
            DeviceMemoryBarrier();

            // Acquired lock. Check the list again.
            if (SearchAccelStructTrackerBufList(baseVa, currentCount))
            {
                // Already in the list since the last time we checked.
                done = true;
            }
            else if (currentCount < ACCEL_STRUCT_TRACKER_LIST_SIZE)
            {
                // Add it to the list.
                AccelStructTrackerBuf[0].accelStructAddrs[currentCount] = baseVa;
                AccelStructTrackerBuf[0].count = currentCount + 1;
                // Don't release the lock until the value is stored in the list.
                DeviceMemoryBarrier();
                done = true;
            }
            // Release the lock.
            AccelStructTrackerBuf[0].lock = 0;
            break;
        }
    }

    if (!done)
    {
        AccelStructTrackerBuf[0].outOfSpace = 1;
    }
}

//======================================================================================================================
// Log all acceleration structures in use by the current wave if logging is enabled.
static void LogAccelStruct(
    uint64_t accelStructVa)
{
    // Skip logging if tracking is disabled at compile time, at run time, or we've run out of space already.
    if (EnableAccelStructTracking() && AccelStructTrackerBuf[0].enable && (AccelStructTrackerBuf[0].outOfSpace == 0))
    {
        while (true)
        {
            if (WaveIsFirstLane())
            {
                LogOneAccelStruct(accelStructVa);
            }

            if (accelStructVa == WaveReadLaneFirst(accelStructVa))
            {
                break;
            }
        }
    }
}
#else
//======================================================================================================================
// Log all acceleration structures in use by the current wave if logging is enabled.
static void LogAccelStruct(
    uint64_t accelStructVa)
{
    // Dummy function for cpp compatibility
}
#endif
