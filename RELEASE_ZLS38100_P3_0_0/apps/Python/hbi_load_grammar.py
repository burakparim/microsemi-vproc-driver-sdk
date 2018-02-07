#!/usr/bin/env python

# Functions designed for programmatic interface:
# grammar = CombineGrammar(trigger_am_path, trigger_so_path, command_am_path, command_so_path, description)
#     Returns a buffer containing the combined grammar
#     Use the keyword “None” when an input path is missing, description is mandatory
#
# CheckGrammarSize(handle, grammar_size)
#     Generates an error if the grammar size exceeds the size specified in the configuration record
#
# LoadGrammar(handle, grammar)
#     Loads the previously combined grammar in the Timberwolf memory
#
# grm_flash_slot = SaveGrammarToFlash(handle)
#     Saves the grammar from RAM to FLASH and returns the grammar flash index

from os.path import dirname, realpath, isfile
import sys
import struct
import argparse
sys.path.append(dirname(realpath(__file__)) + "/../../../libs")
from hbi import *

# Globals
programmatic = True

# ****************************************************************************
def FormatNumber(res_list):
    number = 0

    for byteNum in res_list:
        number = (number << 8) + byteNum

    return number

# ****************************************************************************
def BusySpinWait(handle):
    # The register 0x032 reads 0xFFFF while the Timberwolf is processing a command
    while (FormatNumber(HBI_read(handle, 0x032, 2)) == 0xFFFF):
        pass

# ****************************************************************************
def LoadGrammar(handle, grammar):
    global programmatic

    # Turn off the ASR
    HBI_write(handle, 0x032, (0x80, 0x0D))
    HBI_write(handle, 0x006, (0x00, 0x04))
    BusySpinWait(handle)

    # Grammar start address
    start_add = FormatNumber(HBI_read(handle, 0x4B8, 4))

    # Copy the grammar in memory (64 bytes block or less)
    grammar_size = len(grammar)
    block_num = grammar_size // 64
    block_rem = grammar_size % 64
    if (block_rem != 0):
        block_num += 1

    for idx in xrange(0, block_num):
        address = start_add + idx * 64
        address_seq = struct.unpack("4B", struct.pack(">I", address))
        hbi_offset = address_seq[3]
        HBI_write(handle, 0x00C, address_seq)

        if ((idx == (block_num - 1)) and (block_rem != 0)):
            block_size = block_rem
        else:
            block_size = 64
        grammar_block = struct.unpack(str(block_size) + "B", grammar[idx * 64:idx * 64 + block_size])
        HBI_write(handle, 0xFF00 + hbi_offset, grammar_block)

    # Update the segment table
    idx_last_segment = FormatNumber(HBI_read(handle, 0x13E, 2)) - 1

    # Check if the last segment is already an ASR segment (if so, start_add should match that segment address)
    last_segment_add = FormatNumber(HBI_read(handle, 0x144 + 8 * idx_last_segment, 4))
    grammar_size_seq = struct.unpack("4B", struct.pack(">I", grammar_size))
    if (start_add == last_segment_add):
        # Only update the segment size
        HBI_write(handle, 0x140 + 8 * idx_last_segment, grammar_size_seq)
    else:
        # Add a new segment to the table for the ASR and update the segment counter
        if not programmatic:
            print "Info - Create a new segment"
        idx_last_segment += 1
        last_segment_add_seq = struct.unpack("4B", struct.pack(">I", start_add))
        HBI_write(handle, 0x144 + 8 * idx_last_segment, last_segment_add_seq)
        HBI_write(handle, 0x140 + 8 * idx_last_segment, grammar_size_seq)
        HBI_write(handle, 0x13E, (0x00, idx_last_segment + 1))

    # Start the ASR
    HBI_write(handle, 0x032, (0x80, 0x0E))
    HBI_write(handle, 0x006, (0x00, 0x04))
    BusySpinWait(handle)

    if not programmatic:
        print "Info - Grammar successfully loaded to RAM"

# ****************************************************************************
def SaveGrammarToFlash(handle):
    # Read the number of grammars in flash
    images_flash = FormatNumber(HBI_read(handle, 0x04E, 2))

    # Save the current image in the next slot (slot indexes start at 1)
    HBI_write(handle, 0x032, (0x80, 0x0B))
    HBI_write(handle, 0x034, (0x00, images_flash + 1))
    HBI_write(handle, 0x006, (0x00, 0x04))
    BusySpinWait(handle)

    # Check the flash operation status
    status = FormatNumber(HBI_read(handle, 0x034, 2)) & 0x000F
    if (status != 0):
        raise ValueError("Error - SaveGrammarToFlash(): Grammar could not be saved to flash (error 0x%X)" % status)

    return images_flash + 1

# ****************************************************************************
def CheckGrammarSize(handle, grammar_size):
    start_add = FormatNumber(HBI_read(handle, 0x4B8, 4))
    endAdd = FormatNumber(HBI_read(handle, 0x4BC, 4))
    if (grammar_size > (endAdd - start_add + 1)):
        raise ValueError("Error - CheckGrammarSize(): The grammar exceeds the memory size")

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
    desc_bytes[16:endindex] = gram_desc_bytes;

    # Store the version
    struct.pack_into(">H", desc_bytes, 48, version)

    return desc_bytes

# ****************************************************************************
def ParseGrammarBinFile(path):
    # Note: "b" in "rb" is only required for Windows to read in binary but doesn't hurt on Linux
    f = open(path, "rb")
    buf = f.read()
    f.close()
    return buf, len(buf)

# ****************************************************************************
def CombineGrammar(trigger_am_path, trigger_so_path, command_am_path, command_so_path, description):
    global programmatic

    # Parse all the grammar files
    if ((trigger_am_path != None) and isfile(trigger_am_path)):
        trigger_am_buf, trigger_am_size = ParseGrammarBinFile(trigger_am_path)
    else:
        if not programmatic:
            print "Info - No Trigger acoustic model"
        trigger_am_buf = ""
        trigger_am_size = 0

    if ((trigger_so_path != None) and isfile(trigger_so_path)):
        trigger_so_buf, trigger_so_size = ParseGrammarBinFile(trigger_so_path)
    else:
        if not programmatic:
            print "Info - No Trigger search object"
        trigger_so_buf = ""
        trigger_so_size = 0

    if ((command_am_path != None) and isfile(command_am_path)):
        command_am_buf, command_am_size = ParseGrammarBinFile(command_am_path)
    else:
        if not programmatic:
            print "Info - No Command acoustic model"
        command_am_buf = ""
        command_am_size = 0

    if ((command_so_path != None) and isfile(command_so_path)):
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
        description = "Timberwolf ASR Grammar loader",
        epilog = ("""
This tool combines all the pieces (bin files) of a Sensory (TM)
Automatic Speech Recognition (ASR) model and loads them in
an ASR enabled Timberwolf device

ex: Load a full grammar with a trigger word and cammand set then save it to flash
    %s -tam trigger_am.bin -tso trigger_so.bin -cam command_am.bin -cso command_so -d "my full grammar" -s
ex: Load a trigger only grammar then save it to flash
    %s -tam trigger_am.bin -tso trigger_so.bin -d "my trigger only grammar" -s
""" % (sys.argv[0], sys.argv[0])))
    parser.add_argument("-tam", "--triggerAM", help = "trigger acoustic model (*.bin)")
    parser.add_argument("-tso", "--triggerSO", help = "trigger search object (*.bin)")
    parser.add_argument("-cam", "--commandAM", help = "command acoustic model (*.bin)")
    parser.add_argument("-cso", "--commandSO", help = "command search object (*.bin)")
    parser.add_argument("-d", "--description", help = "grammar decription string (up to 32 characters)", default = "")
    parser.add_argument("-s", "--saveToFlash", help = "save the grammar to flash", action = "store_true")

    # Print the help if no arguments are passed
    if (len(sys.argv[1:]) == 0):
        parser.print_help()
        parser.exit()

    # Parse the input arguments
    args = parser.parse_args()

    programmatic = False

    try:
        grammar = CombineGrammar(args.triggerAM, args.triggerSO, args.commandAM, args.commandSO, args.description)

    except ValueError as err:
        print err
        sys.exit()

    # Init the HBI driver
    cfg = hbi_dev_cfg_t();
    handle = HBI_open(cfg)

    try:
        # Check if the grammar fits in the Timberwolf
        CheckGrammarSize(handle, len(grammar))

        # Load the grammar in memory
        LoadGrammar(handle, grammar)

        if args.saveToFlash:
            slot = SaveGrammarToFlash(handle)
            print "Info - Grammar successfully saved to flash slot %d" % slot

    except ValueError as err:
        print err

    # Close HBI driver
    HBI_close(handle)
