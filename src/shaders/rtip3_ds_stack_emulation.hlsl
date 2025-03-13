/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2020-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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

#ifndef RTIP3_DS_STACK_EMULATION_H
#define RTIP3_DS_STACK_EMULATION_H

//=====================================================================================================================
// RTIP3.x packed stack pointer
struct StackAddr
{
#if defined(__cplusplus)
    StackAddr(int val)
    {
        memset(this, val, sizeof(StackAddr));
    }
#endif
    bool blas2tlasPop;
    bool hasOverflowed;
    bool hasTlasInStack;
    uint base; // Base offset in bytes
    uint addr; // Current address (DWORD index)
    int entriesToTlas;
    int validEntries;
};

//=====================================================================================================================
static StackAddr unpack_stack_addr(uint packedStackAddr)
{
    StackAddr result = (StackAddr)0;

    result.blas2tlasPop = (packedStackAddr & bit(31)) != 0;
    result.hasOverflowed = (packedStackAddr & bit(30)) != 0;
    result.hasTlasInStack = (packedStackAddr & bit(29)) != 0;
    result.base = (packedStackAddr >> 15) & bits(14);
    result.addr = (packedStackAddr >> 10) & bits(5);
    result.entriesToTlas = (packedStackAddr >> 5) & bits(5);
    result.validEntries = packedStackAddr & bits(5);

    return result;
}

//=====================================================================================================================
static uint pack_stack_addr(StackAddr stackAddr)
{
    uint result = 0;

    result |= uint(stackAddr.blas2tlasPop) << 31;
    result |= uint(stackAddr.hasOverflowed) << 30;
    result |= uint(stackAddr.hasTlasInStack) << 29;
    result |= (stackAddr.base & bits(14)) << 15;
    result |= (stackAddr.addr & bits(5)) << 10;
    result |= (stackAddr.entriesToTlas & bits(5)) << 5;
    result |= stackAddr.validEntries & bits(5);

    return result;
}

//=====================================================================================================================
static void push(inout_param(StackAddr) stackAddr, in uint node, in int stackSize)
{
    if (stackAddr.validEntries == stackSize) stackAddr.hasOverflowed = true;
    AmdTraceRayLdsWrite(stackAddr.base + stackAddr.addr * AmdTraceRayGetStackStride(), node);

    if(stackAddr.addr == stackSize-1)
    {
        stackAddr.addr = 0;
    }
    else {
        stackAddr.addr = stackAddr.addr + 1;
    }
    if (stackAddr.entriesToTlas == stackSize)
    {
        stackAddr.hasTlasInStack = false;
    }
    stackAddr.entriesToTlas = min(stackAddr.entriesToTlas + 1, stackSize);
    stackAddr.validEntries = min(stackAddr.validEntries + 1, stackSize);
}

//=====================================================================================================================
static uint pop(inout_param(StackAddr) stackAddr, in uint stackSize)
{
    stackAddr.blas2tlasPop = stackAddr.entriesToTlas == 0 && stackAddr.hasTlasInStack;

    if (stackAddr.blas2tlasPop)stackAddr.hasTlasInStack = false;

    if (stackAddr.validEntries == 0)
    {

        stackAddr.blas2tlasPop = false;
        stackAddr.hasTlasInStack = false;
        return stackAddr.hasOverflowed ? INVALID_NODE : TERMINAL_NODE;
    }

    stackAddr.entriesToTlas = max(stackAddr.entriesToTlas - 1, 0);
    stackAddr.validEntries = max(stackAddr.validEntries - 1, 0);

    if (stackAddr.addr == 0)
    {
        stackAddr.addr = stackSize - 1;
    }
    else
    {
        stackAddr.addr = stackAddr.addr - 1;
    }

    uint result = AmdTraceRayLdsRead(stackAddr.base + stackAddr.addr * AmdTraceRayGetStackStride());

    if (GetNodeType(result) == NODE_TYPE_USER_NODE_INSTANCE)
    {
        // If the stack was already tracking data from another parent BVH, discard all
        // data in the stack from the parent BVH and set the overflowed bit to enable
        // stackless traversal to recover it.
        if (stackAddr.hasTlasInStack)
        {
            stackAddr.validEntries = stackAddr.entriesToTlas;
            stackAddr.hasOverflowed = true;
        }

        stackAddr.hasTlasInStack = true;
        stackAddr.entriesToTlas = 0;
    }

    return result;
}

//=====================================================================================================================
static uint try_pop(inout_param(StackAddr) stackAddr, in uint stackSize)
{
    if (stackAddr.entriesToTlas == 0 && stackAddr.hasTlasInStack)
    {
        return INVALID_NODE;
    }

    if (stackAddr.validEntries == 0)
    {
        return INVALID_NODE;
    }

    uint entriesToTlasTemp = max(stackAddr.entriesToTlas - 1, 0);
    uint validEntriesTemp = max(stackAddr.validEntries - 1, 0);

    uint stackAddrTemp;
    if (stackAddr.addr == 0)
    {
        stackAddrTemp = stackSize - 1;
    }
    else
    {
        stackAddrTemp = stackAddr.addr - 1;
    }

    uint result = AmdTraceRayLdsRead(stackAddr.base + stackAddrTemp * AmdTraceRayGetStackStride());

    if (GetNodeType(result) == NODE_TYPE_USER_NODE_INSTANCE)
    {
        return INVALID_NODE;
    }

    stackAddr.addr = stackAddrTemp;
    stackAddr.entriesToTlas = entriesToTlasTemp;
    stackAddr.validEntries = validEntriesTemp;

    return result;
}

//=====================================================================================================================
#if GPURT_BUILD_RTIP3_1
static uint update_primitive_range(in uint node, in bool end_of_node)
{
    if (end_of_node)
    {
        // Clear bottom 4 bits and add 1 to offset (represents striding to next 128B node).
        return ((node & ~0xf) + 0x10);
    }
    else // END_OF_PAIR
    {
        // Decode the node type, which is 4-bits for triangle nodes (bits 0, 1, and 3 encode the pair).
        uint node_type = node & 0xF;
        // Update node type in bottom 4 bits of node.
        // Only tricky case is when the node type is TRI3 (3) and must become TRI4 (8).
        return (node_type == 0x3) ? (node ^ 0xb) : (node + 1);
    }
}
#endif

//=====================================================================================================================
static StackAddr ds_stack_push8_impl(
    uint                    packedStackAddr,
    uint                    lastNodePtr,
    uint                    rtipLevel,
    inout_param(uint4)      nextNodes0,
    inout_param(uint4)      nextNodes1)
{
    // The stack size is a modifier on the instruction. The driver configures it internally for the hardware path.
    const uint stackSize = 16;

    StackAddr stackAddr = unpack_stack_addr(packedStackAddr);

    // First it determines if the current stack push is the result of stackless walkback as indicated by the node
    // pointers address pointing to the INVALID_NODE (0x1fffffff).
    const bool stackless_walkback = (lastNodePtr >> 3) != 0x1ffffffful;

    // Then it looks for which DWORDs of the data pushed onto the stack (if any) are equal to the last_node_ptr from
    // the previous traversal step.
    bool d7 = (nextNodes1.w == lastNodePtr);
    bool d6 = (nextNodes1.z == lastNodePtr);
    bool d5 = (nextNodes1.y == lastNodePtr);
    bool d4 = (nextNodes1.x == lastNodePtr);
    bool d3 = (nextNodes0.w == lastNodePtr);
    bool d2 = (nextNodes0.z == lastNodePtr);
    bool d1 = (nextNodes0.y == lastNodePtr);
    bool d0 = (nextNodes0.x == lastNodePtr);

#if GPURT_BUILD_RTIP3_1
    if (rtipLevel >= GPURT_RTIP3_1)
    {
        d7 |= (GetNodeType(nextNodes1.w) <= 3);
        d6 |= (GetNodeType(nextNodes1.z) <= 3);
        d5 |= (GetNodeType(nextNodes1.y) <= 3);
        d4 |= (GetNodeType(nextNodes1.x) <= 3);
        d3 |= (GetNodeType(nextNodes0.w) <= 3);
        d2 |= (GetNodeType(nextNodes0.z) <= 3);
        d1 |= (GetNodeType(nextNodes0.y) <= 3);
        d0 |= (GetNodeType(nextNodes0.x) <= 3);
    }
#endif

    bool dataValid = !stackless_walkback || (d7 || d6 || d5 || d4 || d3 || d2 || d1 || d0);

#if GPURT_BUILD_RTIP3_1
    bool is_primitive_range_update = false;

    if (rtipLevel >= GPURT_RTIP3_1)
    {
        // If this is not a stackless walkback operation (indicated by last_node_ptr[31:3]==0x1fffffff) then it could be a
        // primitive range update operation (indicated by last_node_ptr[2]==0) so node pointer 7 will need to be modified
        // prior to the push.
        is_primitive_range_update = ((lastNodePtr & bit(2)) == 0) && !stackless_walkback;
        // However, if the navigation state is END_OF_RANGE (indicated by last_node_ptr[1]==1), then the node will be
        // updated to INVALID_NODE, so the push should be skipped entirely.
        const bool is_primitive_range_end = is_primitive_range_update && ((lastNodePtr & bit(1)));
        // As primitives are sorted first, there is no need to store a 32-bit last pointer value.
        // Instead it is sufficient to invalidate all primitive pointers and push all non-primitive pointers.
        const bool last_node_ptr_is_prim = (GetNodeType(lastNodePtr) <= 3);

        // Additionally, if a primitive range update is requested but the navigation state indicates END_OF_RANGE then all
        // pushes are also skipped.
        dataValid = (dataValid || last_node_ptr_is_prim) && !is_primitive_range_end;
    }
#endif

    uint node;
    bool skip_4_7 = false;
    bool skip_0_3 = false;

    node = nextNodes1.w;
    skip_0_3 |= (node >> 3u) == 0x1ffffffful && (node & bit(1)) == 0 && dataValid && !skip_4_7;
    skip_4_7 |= (node >> 3u) == 0x1ffffffful && (node & bit(2)) == 0 && dataValid && !skip_4_7;
    if (stackless_walkback && d7) dataValid = false;
    if (dataValid && (node >> 3u) != 0x1ffffffful && !skip_4_7)
    {
#if GPURT_BUILD_RTIP3_1
        // Check if a primitive range update is required. Do not need to consider END_OF_RANGE case as data will have
        // already been invalidated.
        if ((rtipLevel >= GPURT_RTIP3_1) && is_primitive_range_update)
        {
            // For a primitive range update the driver will encode the required navigation state in last_node_ptr[1:0].
            // However, the END_OF_RANGE case has already been handled above, so only END_OF_PAIR and END_OF_NODE need
            // to be handled, which can be differentiated solely by last_node_ptr[0].
            const uint navigation_state = (lastNodePtr & bit(0));
            node = update_primitive_range(node, navigation_state);
        }
#endif
        push(stackAddr, node, stackSize);
    }

    node = nextNodes1.z;
    skip_0_3 |= (node >> 3u) == 0x1ffffffful && (node & bit(1)) == 0 && dataValid && !skip_4_7;
    skip_4_7 |= (node >> 3u) == 0x1ffffffful && (node & bit(2)) == 0 && dataValid && !skip_4_7;
#if GPURT_BUILD_RTIP3_1
    if (rtipLevel >= GPURT_RTIP3_1)
    {
        // Infer the skip flags following a primitive range update.
        skip_0_3 |= is_primitive_range_update;
        skip_4_7 |= is_primitive_range_update;
    }
#endif
    if (stackless_walkback && d6) dataValid = false;
    if (dataValid && (node >> 3u) != 0x1ffffffful && !skip_4_7) push(stackAddr, node, stackSize);

    node = nextNodes1.y;
    skip_0_3 |= (node >> 3u) == 0x1ffffffful && (node & bit(1)) == 0 && dataValid && !skip_4_7;
    skip_4_7 |= (node >> 3u) == 0x1ffffffful && (node & bit(2)) == 0 && dataValid && !skip_4_7;
    if (stackless_walkback && d5) dataValid = false;
    if (dataValid && (node >> 3u) != 0x1ffffffful && !skip_4_7) push(stackAddr, node, stackSize);

    node = nextNodes1.x;
    skip_0_3 |= (node >> 3u) == 0x1ffffffful && (node & bit(1)) == 0 && dataValid && !skip_4_7;
    skip_4_7 |= (node >> 3u) == 0x1ffffffful && (node & bit(2)) == 0 && dataValid && !skip_4_7;
    if (stackless_walkback && d4) dataValid = false;
    if (dataValid && (node >> 3u) != 0x1ffffffful && !skip_4_7) push(stackAddr, node, stackSize);

    node = nextNodes0.w;
    skip_0_3 |= (node >> 3u) == 0x1ffffffful && (node & bit(1)) == 0 && dataValid && !skip_0_3;
    if (stackless_walkback && d3) dataValid = false;
    if (dataValid && (node >> 3u) != 0x1ffffffful && !skip_0_3) push(stackAddr, node, stackSize);

    node = nextNodes0.z;
    skip_0_3 |= (node >> 3u) == 0x1ffffffful && (node & bit(1)) == 0 && dataValid && !skip_0_3;
    if (stackless_walkback && d2) dataValid = false;
    if (dataValid && (node >> 3u) != 0x1ffffffful && !skip_0_3) push(stackAddr, node, stackSize);

    node = nextNodes0.y;
    skip_0_3 |= (node >> 3u) == 0x1ffffffful && (node & bit(1)) == 0 && dataValid && !skip_0_3;
    if (stackless_walkback && d1) dataValid = false;
    if (dataValid && (node >> 3u) != 0x1ffffffful && !skip_0_3) push(stackAddr, node, stackSize);

    node = nextNodes0.x;
    skip_0_3 |= (node >> 3u) == 0x1ffffffful && (node & bit(1)) == 0 && dataValid && !skip_0_3;
    if (stackless_walkback && d0) dataValid = false;
    if (dataValid && (node >> 3u) != 0x1ffffffful && !skip_0_3) push(stackAddr, node, stackSize);

    return stackAddr;
}

//=====================================================================================================================
static uint ds_stack_push8_pop1_emulation(
    inout_param(uint) packedStackAddr,
    in uint           lastNodePtr,
    in uint4          nextNodes0,
    in uint4          nextNodes1,
    in uint           rtipLevel)
{
    // The stack size is a modifier on the instruction. The driver configures it internally for the hardware path.
    const uint stackSize = 16;

    StackAddr stackAddr = ds_stack_push8_impl(packedStackAddr, lastNodePtr, rtipLevel, nextNodes0, nextNodes1);

    const uint result = pop(stackAddr, stackSize);

    packedStackAddr = pack_stack_addr(stackAddr);
    return result;
}

//=====================================================================================================================
static uint2 ds_stack_push8_pop2_emulation(
    inout_param(uint) packedStackAddr,
    in uint           lastNodePtr,
    in uint4          nextNodes0,
    in uint4          nextNodes1)
{
    // The stack size is a modifier on the instruction. The driver configures it internally for the hardware path.
    const uint stackSize = 16;

    StackAddr stackAddr = ds_stack_push8_impl(packedStackAddr, lastNodePtr, GPURT_RTIP3_0, nextNodes0, nextNodes1);

    uint res1;
    uint res2 = pop(stackAddr, stackSize);

    if (GetNodeType(res2) == NODE_TYPE_TRIANGLE_1)
    {
        // If slot1 has a tri1, expand it to fill slot0 with tri0. A tri1 in slot0 (from try_pop below) is fine as it is
        // expanded during traversal instead.
        res1 = ClearNodeType(res2);
    }
    else
    {
        res1 = try_pop(stackAddr, stackSize);

        if (res1 == INVALID_NODE)
        {
            // Swap these to avoid an invalid node in slot0 and a valid one in slot1.
            res1 = res2;
            res2 = INVALID_NODE;
        }
    }

    packedStackAddr = pack_stack_addr(stackAddr);
    return uint2(res1, res2);
}

//=====================================================================================================================
static uint RtIp3LdsStackInit_emulation()
{
    StackAddr initStackAddr = (StackAddr)0;
    initStackAddr.base = AmdTraceRayGetStackBase();

    return pack_stack_addr(initStackAddr);
}

#endif
