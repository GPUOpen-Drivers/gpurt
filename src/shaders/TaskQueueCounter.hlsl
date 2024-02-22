/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2023-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
void AllocTasks(const uint numTasks, const uint phase, uint taskQueueOffset)
{
    // start = end
    const uint end = ScratchGlobal.Load(taskQueueOffset + STATE_TASK_QUEUE_END_PHASE_INDEX_OFFSET);

    ScratchGlobal.Store(taskQueueOffset + STATE_TASK_QUEUE_START_PHASE_INDEX_OFFSET, end);

    ScratchGlobal.Store(taskQueueOffset + STATE_TASK_QUEUE_PHASE_OFFSET, phase);

    DeviceMemoryBarrier();

    ScratchGlobal.Store(taskQueueOffset + STATE_TASK_QUEUE_END_PHASE_INDEX_OFFSET, end + numTasks);
}

//=====================================================================================================================
uint2 BeginTask(const uint localId, uint taskQueueOffset)
{
    AllMemoryBarrierWithGroupSync();
    uint index = 0xFFFFFFFF;
    if (localId == 0)
    {
        ScratchGlobal.InterlockedAdd(taskQueueOffset + STATE_TASK_QUEUE_TASK_COUNTER_OFFSET, 1, index);
        // wait until there are valid tasks to do
        do
        {
            DeviceMemoryBarrier();
        } while (index >= ScratchGlobal.Load(taskQueueOffset + STATE_TASK_QUEUE_END_PHASE_INDEX_OFFSET));

        SharedMem[0] = index;
    }
    GroupMemoryBarrierWithGroupSync();
    index = SharedMem[0];
    GroupMemoryBarrierWithGroupSync();

    const uint phase = ScratchGlobal.Load(taskQueueOffset + STATE_TASK_QUEUE_PHASE_OFFSET);

    const uint startPhaseIndex = ScratchGlobal.Load(taskQueueOffset + STATE_TASK_QUEUE_START_PHASE_INDEX_OFFSET);

    return uint2(index - startPhaseIndex, phase);
}

//=====================================================================================================================
bool EndTask(const uint localId, uint taskQueueOffset)
{
    bool returnValue = false;

    AllMemoryBarrierWithGroupSync();

    if (localId == 0)
    {
        const uint endPhaseIndex = ScratchGlobal.Load(taskQueueOffset + STATE_TASK_QUEUE_END_PHASE_INDEX_OFFSET);

        uint orig;
        ScratchGlobal.InterlockedAdd(taskQueueOffset + STATE_TASK_QUEUE_NUM_TASKS_DONE_OFFSET, 1, orig);

        // current phase is done
        if (orig == (endPhaseIndex - 1))
        {
            returnValue = true;
        }
    }

    return returnValue;
}

//======================================================================================================================
// Set a scratch buffer counter to 0 if it has a valid index
void InitScratchCounter(uint offset)
{
    if (offset != INVALID_IDX)
    {
        ScratchGlobal.Store(offset, 0);
    }
}
