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
#ifndef _SERIALIZECOMMON_HLSL
#define _SERIALIZECOMMON_HLSL

// D3D12DDI_SERIALIZED_DATA_DRIVER_MATCHING_IDENTIFIER AMD GUID
#define GPURT_AMD_GUID_0    0x445D18EA
#define GPURT_AMD_GUID_1    0xB42547D8
#define GPURT_AMD_GUID_2    0x867BA9A4
#define GPURT_AMD_GUID_3    0x496A1A2E

// sizeof(D3D12DDI_GPU_VIRTUAL_ADDRESS)
#define GPUVA_SIZE 8

// sizeof(D3D12DDI_SERIALIZED_DATA_DRIVER_MATCHING_IDENTIFIER)
#define GUID_SIZE 32

// D3D12_SERIALIZED_ACCELERATION_STRUCTURE_HEADER
#define SERIALIZED_AS_HEADER_SIZE (GUID_SIZE + (8 * 3))

#define SERIALIZED_AS_HEADER_SERIALIZED_SIZE_OFFSET   (GUID_SIZE)
#define SERIALIZED_AS_HEADER_DESERIALIZED_SIZE_OFFSET (GUID_SIZE + 8)
#define SERIALIZED_AS_HEADER_NUM_BLAS_PTRS_OFFSET     (GUID_SIZE + 16)

#endif
