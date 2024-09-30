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

// TODO: this is a temporary assert file to allow files with asserts to be "clean"
// while the assert file itself cannot be. We need this as we have to move files out of "shared"
// which use assert.h, but cannot then include assert.h as "clean" inclusion of shared files isn't set up yet,
// *because* there are too many files in shared, and they can't be moved out because
// they use assert.h and... (cyclical issue)

#ifndef ASSERT_HLSLI
#define ASSERT_HLSLI
#ifndef GPURT_STATIC_ASSERT
// _Static_assert is not supported with -spirv: https://github.com/microsoft/DirectXShaderCompiler/issues/5750
#define GPURT_STATIC_ASSERT(condition, message)
#endif
#endif
