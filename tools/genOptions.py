##
 #######################################################################################################################
 #
 #  Copyright (c) 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
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

# Equivalent of pal/tools/generate/genScpcOptions.py

import os
import sys
import collections
import collections.abc
import datetime
from ruamel.yaml import YAML
import argparse

Header = f"""/* Copyright (c) {datetime.date.today().year} Advanced Micro Devices, Inc. All rights reserved. */"""
Header += """
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!
//
// This code has been generated automatically. Do not hand-modify this code.
//
// When changes are needed, modify the tools generating this module in the tools directory OR
// the options.yaml file.
//
// WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING!  WARNING! WARNING!  WARNING!  WARNING!  WARNING!
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Auto-generated tables for options.

#ifndef _GPURT_GENERATED_OPTIONS_H
#define _GPURT_GENERATED_OPTIONS_H

#ifdef __cplusplus
namespace GpuRt
{
#endif

typedef uint32_t uint32;

#ifdef AMD_VULKAN
#define DUMMY_OPTION_FUNC(value) { return value; }
#else // AMD_VULKAN
#define DUMMY_OPTION_FUNC(value) ;
#endif

#ifdef __cplusplus
#define GPURT_OPTION_DECL extern
#else // __cplusplus
#ifdef AMD_VULKAN
#define GPURT_OPTION_DECL [noinline]
#else // AMD_VULKAN
#define GPURT_OPTION_DECL
#endif
#endif

"""

Tail = """
#ifdef __cplusplus
} // namespace GpuRt
#endif

#endif
"""

WrapperHeader = """

class Options
{
"""

WrapperTail = """
};
"""

OptionDefaultStructHeader = """
#ifdef __cplusplus
struct Option {
  uint32 nameHash;
  uint64_t value;
};

constexpr Option OptionDefaults[] = {
"""

OptionDefaultStructTail = """
};
#endif
"""

########################################################################################################################
def IsSequence(obj):
    return obj and isinstance(obj, collections.abc.Sequence) and (not any(isinstance(obj, t) for t in (str, bytes)))

########################################################################################################################
# Get comment associated with item in YAML list. Returns a list of strings, with each comment line in a separate
# string.
def GetYamlItemComment(
    optionData,
    optionName
):
    comments = optionData.ca.items.get(optionName)
    comment = []
    if comments:
        for i, section in enumerate(comments):
            if section:
                if IsSequence(section):
                    for j, commentInfo in enumerate(section):
                        # Process Comments above option definition.
                        for thisComment in commentInfo.value.splitlines():
                            thisComment = thisComment.strip()
                            if thisComment:
                                if thisComment.startswith('#'):
                                    thisComment = thisComment[1:].strip()
                                comment.append(thisComment)
                else:
                    # Process comments fall after option definition.
                    for thisComment in section.value.splitlines():
                        thisComment = thisComment.strip()
                        if thisComment:
                            if thisComment.startswith('#'):
                                thisComment = thisComment[1:].strip()
                            comment.append(thisComment)
    return comment

########################################################################################################################
# Output a list of strings as a comment, one string per line. Returns the output.
def OutputComment(
    comment, # Comment as list of strings
    indent   # Indent string
):
    output = ''
    for line in comment:
        output += indent + '// ' + line + '\n'
    return output

########################################################################################################################
# Process an enum, returning the output
def ProcessEnum(
    container, # Dict containing the key
    key        # Key containing the enum (has "enum " at the start)
):
    comment = GetYamlItemComment(container, key)
    data = container[key]
    enumName = key.split(None, 1)[1]

    enumOutput = ''
    enumOutput += OutputComment(comment, '')
    enumOutput += 'enum class ' + enumName + ' : uint32\n{\n'

    # Process enumerators.
    for enumerator in data.keys():
        if IsSequence(data[enumerator]):
            errorexit("Unexpected contents in enumerator '" + enumerator + "' in enumeration '" + enumName + "'")
        comment = GetYamlItemComment(data, enumerator)
        enumOutput += OutputComment(comment, '    ')
        hash = Fnv1aHash(enumerator)
        if enumerator[0].isdigit():
            # Enumerator starts with a digit (e.g. '8x8'). Prepend an underscore for the enumerator name in
            # C++, but not the name accepted by options processing.
            enumerator = '_' + enumerator
        enumOutput += str.format('    {} = 0x{:08X},\n', enumerator, hash)

    enumOutput += '};\n\n'
    return enumOutput

########################################################################################################################
# Process an option, returning the intrinsic and wrapper output
def ProcessOption(
    container, # Dict containing the key
    key        # Key containing the enum (has "enum " at the start)
):
    comment = GetYamlItemComment(container, key)
    data = container[key]
    name = key[0:1].upper() + key[1:]
    nameHash = Fnv1aHash(key)

    intrinsicOutput = ''
    wrapperOutput = ''

    optionDefault = ''

    if 'Type' in data.keys() or 'Default' in data.keys():
        assert('Type' in data.keys())
        typ = data['Type']
        default = "{}"
        if 'Default' in data.keys():
            default = data['Default']

        # Output name hash and default value in case client may want to use it.
        intrinsicOutput += f'constexpr uint32 {name}OptionNameHash = {nameHash};\n'
        intrinsicOutput += f'constexpr {typ} {name}OptionDefault = {default};\n'

        intrinsicOutput += f'GPURT_OPTION_DECL {typ} _AmdGetSetting_{nameHash}() DUMMY_OPTION_FUNC({default})\n'

        optionDefault += f'    {{ {name}OptionNameHash, static_cast<uint64_t>({name}OptionDefault) }},\n'

        wrapperOutput += OutputComment(comment, '    ')
        wrapperOutput += f'    static {typ} get{name}() {{ return _AmdGetSetting_{nameHash}(); }}\n'
    else:
        sys.exit(f"Unknown option type for '{key}'")

    return intrinsicOutput, wrapperOutput, optionDefault

########################################################################################################################
# Calculate the FNV-1a hash of a string, like Util::HashLiteralString
def Fnv1aHash(
    string
):
    Fnv1aOffset = 2166136261
    Fnv1aPrime  = 16777619
    hash = Fnv1aOffset
    for ch in string:
        hash = (hash ^ ord(ch)) * Fnv1aPrime % 0x100000000
    return hash

########################################################################################################################
# Process options struct, returning the output for the actual struct declaration, and stashing other
# information in global variables.
def ProcessStruct(
    container, # Dict containing the key
    key,       # Key containing the struct
    nesting    # Empty or '.'-separated-and-terminated names of nested struct elements
):
    data = container[key]
    structName = key[0:1].upper() + key[1:]
    intrinsicOutput = ''
    wrapperOutput = ''
    optionDefaults = ''

    # Output declarations of nested structs.
    for key in data.keys():
        i, w, o = ProcessOption(data, key)
        intrinsicOutput += i
        wrapperOutput += w
        optionDefaults += o

    intrinsicOutput += OptionDefaultStructHeader
    intrinsicOutput += optionDefaults
    intrinsicOutput += OptionDefaultStructTail

    return intrinsicOutput, wrapperOutput

########################################################################################################################
# Entry function
def main():
    parser = argparse.ArgumentParser( \
    'Option code generation script.\n\
    Sample usage: `python genOptions.py options.yaml` to load YAML file and output generated header to stdout.\n\
                  Use -o option to specify an output filename.')

    parser.add_argument('optionsFilename', type=str, help='YAML file containing option data.')

    parser.add_argument('-outFilename', '-o', type=str,
                        help='Output filename (default stdout).')

    args = parser.parse_args()

    # Try to open and parse the yaml file of options data.
    if not os.path.exists(args.optionsFilename):
        sys.exit("YAML file '" + args.optionsFilename + "' not found")

    yaml = YAML(typ = 'rt', pure = True)
    with open(args.optionsFilename) as f:
        optionsData = yaml.load(f.read())

    # Process top-level items (enums, and the top-level options struct).
    enumOutput = ''
    intrinsicOutput = ''
    wrapperOutput = ''
    for key in optionsData.keys():
        if key.startswith('enum '):
            # Process an enum
            enumOutput += ProcessEnum(optionsData, key)
        else:
            # Process top-level options struct
            i, w = ProcessStruct(optionsData, key, "")
            intrinsicOutput += i
            wrapperOutput += w

    # Gather all output
    output = Header
    output += enumOutput
    output += intrinsicOutput
    output += WrapperHeader
    output += wrapperOutput
    output += WrapperTail
    output += Tail

    # Output the finished auto-generated text.
    if args.outFilename:
        with open(args.outFilename, 'w') as headerFile:
            headerFile.write(output)
    else:
        print(output)

if __name__ == '__main__':
    main()
