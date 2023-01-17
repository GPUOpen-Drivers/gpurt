/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2018-2023 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef _BVH_STACK_DXR_HLSL
#define _BVH_STACK_DXR_HLSL

#define STACK_ADDR_INDEX_MASK   0x3ffff
#define STACK_INDEX_MAX_VALUE   STACK_ADDR_INDEX_MASK

static uint AmdTraceRayLdsSwap(
    uint address,
    uint new_value)
{
    uint old_value = AmdTraceRayLdsRead(address);
    AmdTraceRayLdsWrite(address, new_value);
    return old_value;
}

static uint init_ds_store_stack()
{
    uint start_index = 0;
    uint stack_base_dwords = AmdTraceRayGetStackBase() >> 2u;

    // {stack_base[15:2] , stackIndex[17:0]}
    uint stack_addr = (stack_base_dwords << 18u) | start_index;
    return stack_addr;
}

static uint ds_store_stack(
    inout_param(uint) stackAddr, // combined array base and byte offset
    uint last_visited,    // aka prevNodePtr
    uint4 data)
{
    uint addr = INVALID_NODE;  // default return value (hardware can assume this is already in the dst)

    // stackBase, stackSize, and stackIndex are packed in stackAddr VGPR as follows:
    // {stack_base[15:2] , stackIndex[17:0]}
    uint stackIndex = stackAddr & STACK_ADDR_INDEX_MASK;
    //Extract the stackbase and convert to a byte address
    uint stackBase = ((stackAddr >> 18u) & 0x3fff) << 2u;
    uint stack_size = 1;  // 0: 8 DWORDs, 1: 16 DWORDs, 2: 32 DWORDs 3: 64DWORDs
    uint index_mask = (1u << (stack_size + 3)) - 1;

    uint stride_log2 = 7;
    uint stride = 1u << stride_log2;
    bool lv = 1;  // lane valid
    // last_visited may be invalid
    bool last_visited_invalid = last_visited == INVALID_NODE;  // AND

    // first data cycle (note the order matters here)
    lv = lv &&
        ((data.x == last_visited) || (data.y == last_visited) || (data.z == last_visited) ||( data.w == last_visited) ||
         (last_visited == INVALID_NODE));

    lv = lv && (last_visited_invalid || (data.w != last_visited));

    if (lv && (data.w != INVALID_NODE))
    {
        uint stackOffset = (stackIndex & index_mask) << stride_log2;
        AmdTraceRayLdsWrite(stackBase + stackOffset, data.w);
        //Do not overflow when pushing nodes
        if (stackIndex != STACK_INDEX_MAX_VALUE)
        {
            stackIndex++;
        }
    }
    // second data cycle
    lv = lv && (last_visited_invalid || (data.z != last_visited));
    if (lv && (data.z != INVALID_NODE))
    {
        uint stackOffset = (stackIndex & index_mask) << stride_log2;
        AmdTraceRayLdsWrite(stackBase + stackOffset, data.z);
        //Do not overflow when pushing nodes
        if (stackIndex != STACK_INDEX_MAX_VALUE)
        {
            stackIndex++;
        }
    }

    // third data cycle
    lv = lv && (last_visited_invalid || (data.y != last_visited));
    if (lv && (data.y != INVALID_NODE))
    {
        uint stackOffset = (stackIndex & index_mask) << stride_log2;
        AmdTraceRayLdsWrite(stackBase + stackOffset, data.y);
        //Do not overflow when pushing nodes
        if (stackIndex != STACK_INDEX_MAX_VALUE)
        {
            stackIndex++;
        }
    }
    lv = lv && (last_visited_invalid || (data.x != last_visited));
    if (lv && (data.x != INVALID_NODE))
    {
        addr = data.x;
    }

    if (addr == INVALID_NODE)
    {
        if (stackIndex == 0)
        {
            addr = TERMINAL_NODE;
        }
        else
        {
            stackIndex = (stackIndex - 1);
            uint stackOffset = (stackIndex & index_mask) << stride_log2;
            addr = AmdTraceRayLdsSwap(stackBase + stackOffset, INVALID_NODE);
        }
    }
    //Repack the stackAddr returned to instruction
    // {stack_base[15:2] , stackIndex[17:0]}
    stackAddr = ((stackBase >> 2u) << 18u) | (stackIndex & STACK_ADDR_INDEX_MASK);

    return addr;  // returned to DST VGPR
}

#endif
