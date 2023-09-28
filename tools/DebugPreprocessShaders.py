##
 #######################################################################################################################
 #
 #  Copyright (c) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
 #
 #  Permission is hereby granted, free of charge, to any person obtaining a copy
 #  of this software and associated documentation files (the "Software"), to deal
 #  in the Software without restriction, including without limitation the rights
 #  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 #  copies of the Software, and to permit persons to whom the Software is
 #  furnished to do so, subject to the following conditions:
 #
 #  The above copyright notice and this permission notice shall be included in all
 #  copies or substantial portions of the Software.
 #
 #  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 #  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 #  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 #  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 #  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 #  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 #  SOFTWARE.
 #
 #######################################################################################################################

import sys
import os
import re

cpp_file_header = """
/* Copyright (c) 2023 Advanced Micro Devices, Inc. All rights reserved. */

namespace GpuRt
{

const char* GpuDebugMessages[] =
{
"""

cpp_file_footer = """};

}
"""

assert_lines = []

def add_assert(file_name, line_num, line_str):
    line_str = line_str.strip()
    assert_lines.append(f'GPU assertion failed: {file_name}:{line_num} lane:%02u {line_str}')
    return len(assert_lines) - 1

def add_print_msg(file_name, line_num, msg):
    assert_lines.append(f'GPU debug: {file_name}:{line_num} lane:%02u {msg}')
    return len(assert_lines) - 1

# Preprocess a single HLSL source file. Replaces GPU_ASSERT with GPU_ASSERT_IMPL and an assert ID.
def process_file(src_path, dst_path):
    with open(src_path, 'r') as src_file, open(dst_path, 'w') as dst_file:
        src_name = os.path.basename(src_path)
        line_num = 1
        output_str = ''
        for line in src_file:
            # Find something that looks like a GPU_ASSERT macro invocation (GPU_ASSERT + '(' or space)
            if line.find('#define') == -1:
                m = re.match('.*GPU_ASSERT\s*(\()', line)
                if m is not None:
                    open_paren = m.start(1)
                    assert_id = add_assert(src_name, line_num, line)
                    # Insert the assert ID as an argument to the assert macro and rename the macro
                    line = line[:open_paren+1] + str(assert_id) + ', ' + line[open_paren+1:]
                    line = line.replace('GPU_ASSERT', 'GPU_ASSERT_IMPL', 1)
                else:
                    # Find something that looks like a GPU_DPF macro invocation (GPU_DPF + '(' or space)
                    m = re.match('.*GPU_DPF\s*(\().*"(.*)"', line)
                    if m is not None:
                        open_paren = m.start(1)
                        msg_id = add_print_msg(src_name, line_num, m.group(2))
                        # Insert the assert ID as an argument to the assert macro and rename the macro
                        line = line[:open_paren+1] + str(msg_id) + ', ' + line[open_paren+1:]
                        line = line.replace('GPU_DPF', 'GPU_DPF_IMPL', 1)
            output_str += line
            line_num += 1
        dst_file.write(output_str)

# Generate a CPP file containing all assert messages gathered from the HLSL
def generate_cpp_file(output_file_path):
    with open(output_file_path, 'w') as output_file:
        output_str = cpp_file_header
        for assert_line in assert_lines:
            output_str += f'"{assert_line}",\n'
        output_str += cpp_file_footer
        output_file.write(output_str)

def main():
    # Process each file in the argument list
    # The argments are pairs of input and ouput files then the path to the output file
    for i in range(1, len(sys.argv) - 1, 2):
        process_file(sys.argv[i], sys.argv[i+1])
    generate_cpp_file(sys.argv[-1])
    return 0

if __name__ == '__main__':
    sys.exit(main())
