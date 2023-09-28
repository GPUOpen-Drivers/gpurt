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

#ifndef DEBUG_BUFFER_H
#define DEBUG_BUFFER_H

#if __cplusplus
union DebugPacket
{
    struct
    {
        uint32_t messageId : 21; // Index into the message string array
        uint32_t argCount  :  4; // Number of uint arguments for printf (0 for assert)
        uint32_t laneId    :  6; // Source lane index
        uint32_t assert    :  1; // Trigger assert
    };
    uint32_t u32All;
};
#endif

#define DEBUG_BUFFER_SIZE 1024

#define DEBUG_PACKET_ARG_COUNT_SHIFT  21
#define DEBUG_PACKET_LANE_ID_SHIFT    25
#define DEBUG_PACKET_ASSERT_SHIFT     31

#define DEBUG_CONTROL_WRITE_OFFSET   0
#define DEBUG_CONTROL_READ_OFFSET    4
#define DEBUG_CONTROL_LOCK_OFFSET    8
#define DEBUG_CONTROL_DWORDS         3

#endif
