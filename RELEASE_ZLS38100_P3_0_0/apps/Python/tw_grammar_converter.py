#!/usr/bin/env python

# Functions designed for programmatic interface:
# fw_bin = GetFirmwareBinFile(in_path, fw_opn, block_size = 16)
#     Returns a buffer containing an SDK compatible firmware

import sys
import os
import struct
import argparse
from array import array

# Globals
programmatic = True

# ****************************************************************************
def CreateHeader(trigger_so_size, trigger_am_size, command_so_size, command_am_size, description, version):
#typedef struct
#{
#   const u16 *trigger_grammar_ptr;
#   const u16 *trigger_net_ptr;
#   const u16 *command_grammar_ptr;
#   const u16 *command_net_ptr;
#   char  description[32];
#   u16 version;
#   char  rsrvd[14];
# } GRAMMAR_DESCRIPTOR;

    desc_bytes = bytearray(64)

    # Note: "!=" is effectively an XOR
    if (trigger_so_size == 0) != (trigger_am_size == 0):
        raise ValueError("Error - CreateHeader(): Part of the Trigger grammar is missing")

    if (command_so_size == 0) != (command_am_size == 0):
        raise ValueError("Error - CreateHeader(): Part of the Command grammar is missing")

    if (trigger_so_size == 0) and (trigger_am_size == 0) and (command_so_size == 0) and (command_am_size == 0):
        raise ValueError("Error - CreateHeader(): No grammar to load")

    trigger_so_offset = 0
    trigger_am_offset = 0
    command_so_offset = 0
    command_am_offset = 0

    # Note: The offset of 0x80 is to store two copies of the GRAMMAR_DESCRIPTOR structure
    if (trigger_so_size != 0):
        trigger_so_offset = 0x80
        trigger_am_offset = trigger_so_offset + trigger_so_size
    else:
        command_so_offset = 0x80

    if (command_so_size != 0):
        # Check if a Trigger grammar exists
        if (command_so_offset != 0x80):
            command_so_offset = trigger_am_offset + trigger_am_size
        command_am_offset = command_so_offset + command_so_size

    struct.pack_into(">IIII", desc_bytes, 0, trigger_so_offset, trigger_am_offset, command_so_offset, command_am_offset)

    # Store the grammar description (max 32 char)
    gram_desc_bytes = bytes(description)
    endindex = len(gram_desc_bytes)
    if(endindex >= 32):
        endindex = 31
    endindex = endindex + 16
    desc_bytes[16: endindex] = gram_desc_bytes;

    # Store the version
    struct.pack_into(">H", desc_bytes, 48, version)

    return desc_bytes

# ****************************************************************************
def ParseGrammarBinFile(path):
    # Note: "b" in "rb" is only required for Windows to read in binary but doesn't hurt on Linux
    with open(path, "rb") as f:
        buf = f.read()

    return buf, len(buf)

# ****************************************************************************
def CombineGrammar(trigger_am_path, trigger_so_path, command_am_path, command_so_path, description):
    global programmatic

    # Parse all the grammar files
    if ((trigger_am_path != None) and os.path.isfile(trigger_am_path)):
        trigger_am_buf, trigger_am_size = ParseGrammarBinFile(trigger_am_path)
    else:
        if not programmatic:
            print "Info - No Trigger acoustic model"
        trigger_am_buf = ""
        trigger_am_size = 0

    if ((trigger_so_path != None) and os.path.isfile(trigger_so_path)):
        trigger_so_buf, trigger_so_size = ParseGrammarBinFile(trigger_so_path)
    else:
        if not programmatic:
            print "Info - No Trigger search object"
        trigger_so_buf = ""
        trigger_so_size = 0

    if ((command_am_path != None) and os.path.isfile(command_am_path)):
        command_am_buf, command_am_size = ParseGrammarBinFile(command_am_path)
    else:
        if not programmatic:
            print "Info - No Command acoustic model"
        command_am_buf = ""
        command_am_size = 0

    if ((command_so_path != None) and os.path.isfile(command_so_path)):
        command_so_buf, command_so_size = ParseGrammarBinFile(command_so_path)
    else:
        if not programmatic:
            print "Info - No Command search object"
        command_so_buf = ""
        command_so_size = 0

    # Create the description stucture (version 1)
    structure = CreateHeader(trigger_so_size, trigger_am_size, command_so_size, command_am_size, description, 1)

    # Merge all the pieces of the grammar inluding two copies of the description structure
    return structure + structure + trigger_so_buf + trigger_am_buf + command_so_buf + command_am_buf

# ****************************************************************************
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        formatter_class = argparse.RawDescriptionHelpFormatter,
        description = "Timberwolf ASR Grammar Converter",
        epilog = ("""
This tool combines all the pieces (bin files) of a Sensory (TM)
Automatic Speech Recognition (ASR) and generates a Bin file (*.bin)
for dynamic loading with the Grammar loader example code

ex: Convert a full grammar with a trigger word and cammand set
    %s -tam trigger_am.bin -tso trigger_so.bin -cam command_am.bin -cso command_so -d "my full grammar"
ex: Load a trigger only grammar
    %s -tam trigger_am.bin -tso trigger_so.bin -d "my trigger only grammar"
""" % (sys.argv[0], sys.argv[0])))
    parser.add_argument("outputPath", help = "output combined grammar image path (*.bin)")
    parser.add_argument("-tam", "--triggerAM", help = "trigger acoustic model (*.bin)")
    parser.add_argument("-tso", "--triggerSO", help = "trigger search object (*.bin)")
    parser.add_argument("-cam", "--commandAM", help = "command acoustic model (*.bin)")
    parser.add_argument("-cso", "--commandSO", help = "command search object (*.bin)")
    parser.add_argument("-d", "--description", help = "grammar decription string (up to 32 characters)", default = "")

    # Print the help if no arguments are passed
    if (len(sys.argv[1:]) == 0):
        parser.print_help()
        parser.exit()

    # Parse the input arguments
    args = parser.parse_args()

    programmatic = False

    try:
        grammar = CombineGrammar(args.triggerAM, args.triggerSO, args.commandAM, args.commandSO, args.description)

        # Save the grammar to a bin file
        outputPath = args.outputPath
        filename, file_extension = os.path.splitext(outputPath)
        if file_extension.lower != ".bin":
            outputPath += ".bin"

        with open(outputPath, "wb") as out_file:
            out_file.write(grammar)

        print 'Info - Successfully generated: "%s"' % outputPath
    except ValueError as err:
        print err
