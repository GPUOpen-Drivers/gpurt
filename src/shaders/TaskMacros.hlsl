
/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _TASKMACROS_HLSL
#define _TASKMACROS_HLSL

//=====================================================================================================================
// Task Counter Macros
// INIT_TASK is used at the beginning of a shader to allow for waves to atomically fetch and task index
// BEGIN_TASK is used at the start of a pass that needs to be completed before the next pass
// END_TASK is used at the end of of a pass that makes sure the task is completed before moving onto the next pass
//=====================================================================================================================
#ifndef TASK_COUNTER_BUFFER
#define TASK_COUNTER_BUFFER DstMetadata
#endif
#ifndef TASK_COUNTER_OFFSET
#define TASK_COUNTER_OFFSET ACCEL_STRUCT_METADATA_TASK_COUNTER_OFFSET
#endif
#ifndef NUM_TASKS_DONE_OFFSET
#define NUM_TASKS_DONE_OFFSET ACCEL_STRUCT_METADATA_NUM_TASKS_DONE_OFFSET
#endif

#define INIT_TASK           if(localId == 0)\
                            {\
                                TASK_COUNTER_BUFFER.InterlockedAdd(TASK_COUNTER_OFFSET, 1, SharedMem[0]);\
                            }\
                            GroupMemoryBarrierWithGroupSync();\
                            waveId = SharedMem[0];\
                            GroupMemoryBarrierWithGroupSync();

#define BEGIN_TASK(n)       while((waveId >= numTasksWait) && (waveId < (numTasksWait + n)))\
                            {\
                                uint globalId = (waveId - numTasksWait) * BUILD_THREADGROUP_SIZE + localId;\
                                uint groupId = (waveId - numTasksWait);

#define END_TASK(n)             DeviceMemoryBarrierWithGroupSync();\
                                if(localId == 0)\
                                {\
                                    TASK_COUNTER_BUFFER.InterlockedAdd(TASK_COUNTER_OFFSET, 1, SharedMem[0]);\
                                    TASK_COUNTER_BUFFER.InterlockedAdd(NUM_TASKS_DONE_OFFSET, 1);\
                                }\
                                GroupMemoryBarrierWithGroupSync();\
                                waveId = SharedMem[0];\
                                GroupMemoryBarrierWithGroupSync();\
                            }\
                            numTasksWait += n;\
                            do\
                            {\
                                DeviceMemoryBarrier();\
                            } while (TASK_COUNTER_BUFFER.Load(NUM_TASKS_DONE_OFFSET) < numTasksWait);

#endif
