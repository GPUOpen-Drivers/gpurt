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
//
#ifndef _MATH_H
#define _MATH_H

//=====================================================================================================================
static BoundingBox CombineAABB(
    BoundingBox b0,
    BoundingBox b1)
{
    BoundingBox bbox;

    bbox.min = min(b0.min, b1.min);
    bbox.max = max(b0.max, b1.max);

    return bbox;
}

//=====================================================================================================================
// Helper function for extracting a single bit from a 32-bit field
inline uint32_t bit(uint32_t index)
{
    return 1u << index;
}

//=====================================================================================================================
// Helper function for generating a 32-bit bit mask
inline uint32_t bits(uint32_t bitcount)
{
    return (1u << bitcount) - 1;
}

//=====================================================================================================================
// Helper function for generating a 32-bit bit mask
inline uint64_t bits64(uint64_t bitcount)
{
    return (1ull << bitcount) - 1ull;
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
static uint32_t Pow2Align(
    uint32_t value,      ///< Value to align.
    uint32_t alignment)  ///< Desired alignment (must be a power of 2).
{
    return ((value + alignment - 1) & ~(alignment - 1));
}

#endif
