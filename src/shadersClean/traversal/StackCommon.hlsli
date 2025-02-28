/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2025 Advanced Micro Devices, Inc. All Rights Reserved.
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
#ifndef STACK_COMMON_HLSL
#define STACK_COMMON_HLSL

#include "llpc/GpurtIntrinsics.h"

DECLARE_CONT_STACK_LOAD(U32, uint32_t)
DECLARE_CONT_STACK_STORE(U32, uint32_t value)

//=================================================================================================================
// Allocates memory on the continuation stack and pushes the payload onto it.
static void ContStackPushPayload()
{
    const uint32_t payloadDwordCount = _AmdContPayloadRegistersI32Count();
    const uint32_t payloadByteCount = payloadDwordCount * sizeof(uint32_t);
    const uint32_t payloadCsp = _AmdContStackAlloc(payloadByteCount);
    for (uint32_t i = 0; i < payloadDwordCount; i++)
    {
        const uint32_t addr = payloadCsp + i * sizeof(uint32_t);
        _AmdContStackStoreU32(addr, _AmdContPayloadRegistersGetI32(i));
    }
}

//=================================================================================================================
// Pops the payload from the continuation stack, and then frees the associated memory.
// This method does not check whether the stack actually contains payload data, the caller is responsible for
// matching ContStackPushPayload calls with ContStackPopPayload calls.
static void ContStackPopPayload()
{
    const uint32_t payloadDwordCount = _AmdContPayloadRegistersI32Count();
    const uint32_t payloadByteCount = payloadDwordCount * sizeof(uint32_t);
    const uint32_t cpsStackPtr = _AmdContStackGetPtr() - payloadByteCount;
    for (uint32_t i = 0; i < payloadDwordCount; i++)
    {
        const uint32_t addr = cpsStackPtr + i * sizeof(uint32_t);
        _AmdContPayloadRegistersSetI32(i, _AmdContStackLoadU32(addr));
    }
    _AmdContStackFree(payloadByteCount);
}

//=================================================================================================================
// Allocates and pushes a single dword onto the continuation stack.
static void ContStackPushDword(uint32_t value)
{
    _AmdContStackStoreU32(_AmdContStackAlloc(sizeof(uint32_t)), value);
}

//=================================================================================================================
// Pops a single dword from the continuation stack and frees one dword of memory.
static uint32_t ContStackPopDword()
{
    const uint32_t value = _AmdContStackLoadU32(_AmdContStackGetPtr() - sizeof(uint32_t));
    _AmdContStackFree(sizeof(uint32_t));
    return value;
}

//=================================================================================================================
// Defines a `ContStackPushStructTy` method (e.g. for `Foo`, `ContStackPushFoo`) that takes an instance of StructTy
// as unique argument. This method allocates enough memory on the continuation stack to accommodate the full
// structure, and then pushes the structure onto the stack.
//
// Note: this method needs _AmdValueI32CountStructTy and _AmdValueGetI32StructTy, which are defined by macros
// DECLARE_VALUE_I32_COUNT and DECLARE_VALUE_GET_I32, respectively. Alternatively, use DECLARE_PUSHPOP_STRUCT.
#define DECLARE_PUSH_STRUCT(StructTy)                                      \
static void ContStackPush##StructTy(inout_param(StructTy) object)          \
{                                                                          \
    const uint32_t dwordCount = _AmdValueI32Count##StructTy(object);       \
    const uint32_t byteCount = dwordCount * sizeof(uint32_t);              \
    const uint32_t cpsStackPtr = _AmdContStackAlloc(byteCount);            \
    for (uint32_t i = 0; i < dwordCount; i++)                              \
    {                                                                      \
        const uint32_t addr = cpsStackPtr + i * sizeof(uint32_t);          \
        _AmdContStackStoreU32(addr, _AmdValueGetI32##StructTy(object, i)); \
    }                                                                      \
}

//=================================================================================================================
// Defines a `ContStackPopStructTy` method (e.g. for `Foo`, `ContStackPopFoo`) that takes an instance of StructTy
// as unique argument (as in-out parameter). This method fills the instance with the data stored on the
// continuation stack, and then frees the memory.
// This method does not check whether the data in the continuation stack actually contains StructTy data, therefore
// the caller is responsible for matching ContStackPushStructTy with ContStackPopStructTy calls.
//
// Note: this method needs _AmdValueI32CountStructTy and _AmdValueSetI32StructTy, which are defined by macros
// DECLARE_VALUE_I32_COUNT and DECLARE_VALUE_SET_I32, respectively. Alternatively, use DECLARE_PUSHPOP_STRUCT.
#define DECLARE_POP_STRUCT(StructTy)                                       \
static void ContStackPop##StructTy(inout_param(StructTy) object)           \
{                                                                          \
    const uint32_t dwordCount = _AmdValueI32Count##StructTy(object);       \
    const uint32_t byteCount = dwordCount * sizeof(uint32_t);              \
    const uint32_t cpsStackPtr = _AmdContStackGetPtr() - byteCount;        \
    for (uint32_t i = 0; i < dwordCount; i++)                              \
    {                                                                      \
        const uint32_t addr = cpsStackPtr + i * sizeof(uint32_t);          \
        _AmdValueSetI32##StructTy(object, i, _AmdContStackLoadU32(addr));  \
    }                                                                      \
    _AmdContStackFree(byteCount);                                          \
}

//=================================================================================================================
// Defines a pair of `ContStackPush/PopStructTy` methods (e.g. for `Foo`, `ContStackPushFoo` and `ContStackPopFoo`)
// that take an instance of StructTy as unique argument (as in-out parameter). The push variant allocates memory
// onto the continuation stack and pushes the content of the StructTy instance onto it. The pop variant does the
// opposite, fills the instance with the data from the continuation stack and the frees the memory.
// The caller is responsible for matching Push call with Pop calls.
// This macro should not be used inside HLSL namespaces.
//
// Note: this macro also defines the auxiliary methods _AmdValueI32CountStructTy, _AmdValueGetI32StructTy and
// _AmdValueSetI32StructTy via the DECLARE_VALUE_I32_COUNT, DECLARE_VALUE_GET_I32 and DECLARE_VALUE_SET_I32 macros.
#define DECLARE_PUSHPOP_STRUCT(StructTy)               \
    DECLARE_VALUE_I32_COUNT(StructTy, StructTy object) \
    DECLARE_VALUE_GET_I32(StructTy, StructTy object)   \
    DECLARE_VALUE_SET_I32(StructTy, StructTy object)   \
    DECLARE_PUSH_STRUCT(StructTy)                      \
    DECLARE_POP_STRUCT(StructTy)                       \

#endif
