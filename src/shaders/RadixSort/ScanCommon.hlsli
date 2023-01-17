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
#include "../RayTracingDefs.h"

#define NUMBER_OF_BLOCKS_PER_GROUP 1
#define NUM_BINS                   16
#define GROUP_SIZE                 BUILD_THREADGROUP_SIZE
#define INT_MAX                    0x7FFFFFFF
#define SHARED_KEYS_OFFSET         (GROUP_SIZE * 4 + NUM_BINS)

//=====================================================================================================================
int4 safe_load_int4(
    RWByteAddressBuffer source,
    uint                baseOffset,
    uint                idx,
    uint                sizeInTypeUnits)
{
    const uint offset = baseOffset + (idx * sizeof(int4));

    int4 res = int4(0, 0, 0, 0);
    if (((idx + 1) << 2) <= sizeInTypeUnits)
    {
        res = source.Load<int4>(offset);
    }
    else
    {
        if ((idx << 2) < sizeInTypeUnits)
        {
            res.x = source.Load<int4>(offset).x;
        }
        if ((idx << 2) + 1 < sizeInTypeUnits)
        {
            res.y = source.Load<int4>(offset).y;
        }
        if ((idx << 2) + 2 < sizeInTypeUnits)
        {
            res.z = source.Load<int4>(offset).z;
        }
    }

    return res;
}

//=====================================================================================================================
void safe_store_int4(
    int4                val,
    RWByteAddressBuffer dest,
    uint                baseOffset,
    uint                idx,
    uint                sizeInTypeUnits)
{
    const uint offset = baseOffset + (idx * sizeof(int4));

    if ((idx + 1) * 4 <= sizeInTypeUnits)
    {
        dest.Store<int4>(offset, val);
    }
    else
    {
        if (idx * 4 < sizeInTypeUnits)
        {
            dest.Store(offset, val.x);
        }
        if (idx * 4 + 1 < sizeInTypeUnits)
        {
            dest.Store(offset + sizeof(int), val.y);
        }
        if (idx * 4 + 2 < sizeInTypeUnits)
        {
            dest.Store(offset + (2 * sizeof(int)), val.z);
        }
    }
}

//=====================================================================================================================
int4 safe_load_int4_intmax(
    RWByteAddressBuffer source,
    uint                baseOffset,
    uint                idx,
    uint                sizeInInts)
{
    const uint offset = baseOffset + (idx * sizeof(int4));

    int4 res = int4(INT_MAX, INT_MAX, INT_MAX, INT_MAX);
    if (((idx + 1) << 2) <= sizeInInts)
    {
        res = source.Load<int4>(offset);
    }
    else
    {
        if ((idx << 2) < sizeInInts)
        {
            res.x = source.Load<int4>(offset).x;
        }
        if ((idx << 2) + 1 < sizeInInts)
        {
            res.y = source.Load<int4>(offset).y;
        }
        if ((idx << 2) + 2 < sizeInInts)
        {
            res.z = source.Load<int4>(offset).z;
        }
    }

    return res;
}

uint64_t4 safe_load_int64_4_intmax(
    RWByteAddressBuffer source,
    uint                baseOffset,
    uint                idx,
    uint                sizeInInts)
{
    const uint offset = baseOffset + (idx * sizeof(uint64_t4));

    uint64_t  temp = 0xffffffffffffffffull;
    uint64_t4 res  = uint64_t4(temp, temp, temp, temp);
    if (((idx + 1) << 2) <= sizeInInts)
    {
        res = source.Load<uint64_t4>(offset);
    }
    else
    {
        if ((idx << 2) < sizeInInts)
        {
            res.x = source.Load<uint64_t4>(offset).x;
        }
        if ((idx << 2) + 1 < sizeInInts)
        {
            res.y = source.Load<uint64_t4>(offset).y;
        }
        if ((idx << 2) + 2 < sizeInInts)
        {
            res.z = source.Load<uint64_t4>(offset).z;
        }
    }

    return res;
}
