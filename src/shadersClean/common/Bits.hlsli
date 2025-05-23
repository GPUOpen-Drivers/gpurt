/*
 ***********************************************************************************************************************
 *
 *  Copyright (c) 2024-2025 Advanced Micro Devices, Inc. All Rights Reserved.
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

#ifndef BITS_HLSLI
#define BITS_HLSLI

//=====================================================================================================================
static uint LowPart(uint64_t value)
{
    return uint(value);
}

//=====================================================================================================================
static uint HighPart(uint64_t value)
{
    return uint(value >> 32);
}

//=====================================================================================================================
// Helper function for producing a 32 bit mask of one bit
inline uint32_t bit(uint32_t index)
{
    return 1u << index;
}

//=====================================================================================================================
// Helper function for producing a 16 bit mask of one bit
inline uint16_t bit16(uint16_t index)
{
    return uint16_t(1u << index);
}

//=====================================================================================================================
// Helper function for producing a 64 bit mask of one bit
inline uint64_t bit64(uint32_t index)
{
    return 1ull << index;
}

//=====================================================================================================================
// Helper function for generating a 32-bit bit mask
inline uint32_t bits(uint32_t bitcount)
{
    return (bitcount == 32) ? 0xFFFFFFFF : ((1u << bitcount) - 1);
}

//=====================================================================================================================
// Helper function for generating a 16-bit bit mask
inline uint16_t bits16(uint16_t bitcount)
{
    return (bitcount == 16) ? uint16_t(0xFFFFu) : uint16_t((1u << bitcount) - 1);
}

//=====================================================================================================================
// Helper function for generating a 32-bit bit mask
inline uint64_t bits64(uint64_t bitcount)
{
    return (bitcount == 64) ? 0xFFFFFFFFFFFFFFFFull : ((1ull << bitcount) - 1ull);
}

//=====================================================================================================================
// Helper function for inserting data into a src bitfield and returning the output
static uint32_t bitFieldInsert(
    in uint32_t src,
    in uint32_t bitOffset,
    in uint32_t numBits,
    in uint32_t data)
{
    const uint32_t mask = bits(numBits);
    src &= ~(mask << bitOffset);
    return (src | ((data & mask) << bitOffset));
}

//=====================================================================================================================
// Helper function for inserting data into a uint16_t src bitfield and returning the output
static uint16_t bitFieldInsert16(
    in uint16_t src,
    in uint16_t bitOffset,
    in uint16_t numBits,
    in uint16_t data)
{
    const uint16_t mask = bits16(numBits);
    src &= ~(mask << bitOffset);
    return (src | ((data & mask) << bitOffset));
}

//=====================================================================================================================
// Helper function for inserting data into a uint64_t src bitfield and returning the output
static uint64_t bitFieldInsert64(
    in uint64_t src,
    in uint64_t bitOffset,
    in uint64_t numBits,
    in uint64_t data)
{
    const uint64_t mask = bits64(numBits);
    src &= ~(mask << bitOffset);
    return (src | ((data & mask) << bitOffset));
}

//=====================================================================================================================
// Helper function for extracting data from a src bitfield
static uint32_t bitFieldExtract(
    in uint32_t src,
    in uint32_t bitOffset,
    in uint32_t numBits)
{
    return (src >> bitOffset) & bits(numBits);
}

//=====================================================================================================================
// Helper function for extracting data from a src bitfield
static uint16_t bitFieldExtract16(
    in uint16_t src,
    in uint16_t bitOffset,
    in uint16_t numBits)
{
    return (src >> bitOffset) & bits16(numBits);
}

//=====================================================================================================================
// Helper function for extracting data from a uint64_t src bitfield
static uint64_t bitFieldExtract64(
    in uint64_t src,
    in uint64_t bitOffset,
    in uint64_t numBits)
{
    return (src >> bitOffset) & bits64(numBits);
}

//=====================================================================================================================
static uint32_t Pow2Align(
    uint32_t value,      ///< Value to align.
    uint32_t alignment)  ///< Desired alignment (must be a power of 2).
{
    return ((value + alignment - 1) & ~(alignment - 1));
}

//=====================================================================================================================
// The uint64 overload for countbits() as DXC does not generate spirv's OpBitCount with uint64
inline uint countbits64(uint64_t val)
{
    return countbits(LowPart(val)) + countbits(HighPart(val));
}

//=====================================================================================================================
static uint FloatToUint(float v)
{
    const uint bitShift = 31;
    const uint bitMask = 0x80000000;

    uint ui = uint(asuint(v));
    ui ^= (1 + ~(ui >> bitShift) | bitMask);

    return ui;
}

#endif
