#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "typedefs.h"
#include "chip.h"
#include "hbi.h"

/* size of buffer for HBI read/write*/
#define MAX_RW_SIZE 64

/* ------------------------------------------------------------ */
void HbiRead(hbi_handle_t handle, reg_addr_t address, user_buffer_t *pData, size_t length) {
    hbi_status_t status;

    status = HBI_read(handle, address, pData, length);
    if(status != HBI_STATUS_SUCCESS) {
        printf("Error - HbiRead(): HBI_read failed (%d)\n", status);
        HBI_close(handle);
        exit(-1);
    }
}

/* ------------------------------------------------------------ */
void HbiWrite(hbi_handle_t handle, reg_addr_t address, user_buffer_t *pData, size_t length) {
    hbi_status_t status;

    status = HBI_write(handle, address, pData, length);
    if(status != HBI_STATUS_SUCCESS) {
        printf("Error - HbiWrite(): HBI_write failed (%d)\n", status);
        HBI_close(handle);
        exit(-1);
    }
}
/* ------------------------------------------------------------ */
unsigned short HbiReadWord(hbi_handle_t handle, reg_addr_t address) {
    user_buffer_t buf[2];

    HbiRead(handle, address, buf, 2);

    return (((unsigned short)buf[0] << 8) + (unsigned short)buf[1]);
}

/* ------------------------------------------------------------ */
void HbiWriteWord(hbi_handle_t handle, reg_addr_t address, unsigned short data) {
    user_buffer_t buf[2];

    buf[0] = (user_buffer_t)(data >> 8);
    buf[1] = (user_buffer_t)(data & 0x00FF);
    HbiWrite(handle, address, buf, 2);
}

/* ------------------------------------------------------------ */
void BusySpinWait(hbi_handle_t handle) {
    while (HbiReadWord(handle, 0x032) == 0xFFFF);
}

/* ------------------------------------------------------------ */
unsigned int Buffer2Int(user_buffer_t *pData) {
    unsigned int integer;

    integer = (unsigned int)pData[0];
    integer = (integer << 8) + (unsigned int)pData[1];
    integer = (integer << 8) + (unsigned int)pData[2];
    integer = (integer << 8) + (unsigned int)pData[3];

    return integer;
}

/* ------------------------------------------------------------ */
void Int2Buffer(unsigned int integer, user_buffer_t *pData) {

    pData[0] = (user_buffer_t)((integer >> 24) & 0x000000FF);
    pData[1] = (user_buffer_t)((integer >> 16) & 0x000000FF);
    pData[2] = (user_buffer_t)((integer >> 8) & 0x000000FF);
    pData[3] = (user_buffer_t)(integer & 0x000000FF);
}

/* ------------------------------------------------------------ */
void CheckHostCmdStatus(hbi_handle_t handle) {
    unsigned short flashStat;

    flashStat = HbiReadWord(handle, 0x034) & 0x000F;
    if (flashStat != 0) {
        switch (flashStat) {
            case 0x1:
                printf("Error - Invalid flash image\n");
                break;

            case 0x2:
                printf("Error - Checksum error during load\n");
                break;

            case 0x3:
                printf("Error - No space in flash for application image size\n");
                break;

            case 0x4:
                printf("Error - Configuration record size doesn’t match application specified\n");
                break;

            case 0x5:
                printf("Error - Invalid flash header data\n");
                break;

            case 0x6:
                printf("Error - No flash present\n");
                break;

            case 0x7:
                printf("Error - Flash failure\n");
                break;

            case 0x8:
                printf("Error - Command failure\n");
                break;

            case 0x9:
                printf("Error - No configuration record available\n");
                break;

            case 0xA:
                printf("Error - Invalid Command - Application running\n");
                break;

            default:
                printf("Error - Unknown flash error\n");
                break;
        }
    }

    if (flashStat != 0) {
        HBI_close(handle);
        exit(-1);
    }
}

/* ------------------------------------------------------------ */
void LoadGrammarFile(hbi_handle_t handle, char * binGrammarPath) {
    size_t byteCount;
    FILE *file;
    user_buffer_t  buf[MAX_RW_SIZE];
    hbi_status_t status;
    user_buffer_t offset, segAddress[4], segAddressTemp[4], segSize[4];
    unsigned short lastSegIndex;
    unsigned int maxSize, grammarSize = 0;

    file = fopen(binGrammarPath, "rb");
    if (file == NULL) {
        printf("Error - LoadGrammarFile(): Cannot open the grammar input file\n");
        HBI_close(handle);
        exit(-1);
    }

    /* Read the ASR segment address */
    HbiRead(handle, 0x4B8, segAddress, 4);
    offset = segAddress[3];

    /* Read the ASR max address */
    HbiRead(handle, 0x4BC, segAddressTemp, 4);

    /* Get the grammar max size */
    maxSize = Buffer2Int(segAddressTemp) - Buffer2Int(segAddress) - 1;

    /* Get the file size */
    fseek(file, 0L, SEEK_END);
    grammarSize = ftell(file);
    fseek(file, 0L, SEEK_SET);
    if (grammarSize > maxSize) {
        printf("Error - LoadGrammarFile(): Grammar file to large (exceeds %d bytes)\n", maxSize);
        HBI_close(handle);
        exit(-1);
    }

    /* Store the start address for later use */
    memcpy(segAddressTemp, segAddress, 4);

    /* Disable the ASR */
    HbiWriteWord(handle, 0x032, 0x800D);
    HbiWriteWord(handle, 0x006, 4);
    BusySpinWait(handle);

    byteCount = MAX_RW_SIZE;
    while (byteCount == MAX_RW_SIZE) {
        /* Read a block of data from the bin file */
        byteCount = fread(buf, 1, MAX_RW_SIZE, file);

        /* Write the address for a page 255 type access */
        HbiWrite(handle, 0x00C, segAddress, 4);

        /* Write the block */
        HbiWrite(handle, 0xFF00 | (reg_addr_t)offset, buf, byteCount);

        /* increment the address pointer */
        Int2Buffer(Buffer2Int(segAddress) + MAX_RW_SIZE, segAddress);
        offset = segAddress[3];
    }

    fclose(file);

    /* Update the segment table */
    /* Recover the start address */
    memcpy(segAddress, segAddressTemp, 4);

    /* get the number of segments */
    lastSegIndex = HbiReadWord(handle, 0x13E) - 1;

    /* Read the load address of the last segment */
    HbiRead(handle, 0x144 + 8 * lastSegIndex, segAddressTemp, 4);

    /* Convert the grammar size in a buffer */
    Int2Buffer(grammarSize, segSize);

    /* If the last segment address is an ASR segment, update the size otherwise create a new segment */
    if (Buffer2Int(segAddressTemp) == Buffer2Int(segAddress)) {
        /* Update the last segment size */
        HbiWrite(handle, 0x140 + 8 * lastSegIndex, segSize, 4);
    } else {
        /* Create a new segment */
        lastSegIndex++;
        HbiWrite(handle, 0x140 + 8 * lastSegIndex, segSize, 4);
        HbiWrite(handle, 0x144 + 8 * lastSegIndex, segAddress, 4);
        HbiWriteWord(handle, 0x13E, lastSegIndex + 1);
    }

    /* Enable the ASR */
    HbiWriteWord(handle, 0x032, 0x800E);
    HbiWriteWord(handle, 0x006, 4);
    BusySpinWait(handle);

    printf("Info - Grammar successfully loaded to RAM\n");
}

/* ------------------------------------------------------------ */
void printHelp(char * exeName) {
    printf("\n-= ASR Grammar Manager =-\n");
    printf("Usage: %s -[d deviceId] [-q] [-l binPath] [-f] [-s flashSlot] [-h]\n", exeName);
    printf("    - \"-q\":\n");
    printf("        Query the number of grammars stored in flash\n");
    printf("    - \"-l binPath\":\n");
    printf("        Load a grammar from a bin file to RAM\n");
    printf("        Note: generate the bin file using \"tw_grammar_converter.py\"\n");
    printf("    - \"-f\":\n");
    printf("        Save the grammar from RAM to flash\n");
    printf("    - \"-s flashSlot\":\n");
    printf("        Swap grammars from flash\n");
    printf("    - \"-h\":\n");
    printf("        Prints that help\n\n");
}

/* ------------------------------------------------------------ */
int main(int argc, char** argv) {
    int c, numGrammars, queryNumGrammar = 0, saveToFlash = 0, grammarIdx = -1;
    char *binGrammarPath = NULL;
    hbi_status_t status;
    hbi_dev_cfg_t devConfig;
    hbi_handle_t handle;
    hbi_device_id_t deviceId = -1;

    /* Optional arguments */
    while((c = getopt(argc,argv,"hql:fs:d:")) != -1) {
        switch(c) {
            case 'd':
                deviceId= strtol(optarg,NULL,16);
                break;
            case 'h':
                printHelp(argv[0]);
                return 0;
                break;

            case 'q':
                /* Query the number of grammars stored in flash */
                queryNumGrammar = 1;
                break;

            case 'l':
                /* Load a grammar from a bin file to RAM */
                binGrammarPath = optarg;
                break;

            case 'f':
                /* Save the grammar from RAM to flash */
                saveToFlash = 1;
                break;

            case 's':
                /* Swap grammars from flash */
                grammarIdx = atoi(optarg);
                break;

            case '?':
                return -1;
                break;

            default:
                abort();
        }
    }

    /* No arguments */
    if (argc == 1) {
        printHelp(argv[0]);
        return 0;
    }

    /* Open the HBI driver */
    devConfig.deviceId = deviceId;
    devConfig.pDevName = NULL;


    status = HBI_open(&handle, &devConfig);
    if(status != HBI_STATUS_SUCCESS) {
        printf("Error - Main(): HBI_open failed (%d)\n", status);
        return -1;
    }

    /* The firmware needs to be running in order to manage grammars */
    if ((HbiReadWord(handle, 0x028) & 0x8000) == 0) {
        printf("Error - Main(): Application firmware stopped\n");
        HBI_close(handle);
        exit(-1);
    }

    /* Get the number of images in flash */
    numGrammars = HbiReadWord(handle, 0x04E);

    if (queryNumGrammar) {
        printf("Info - Number of grammars in flash: %d\n", numGrammars);
    } else if (binGrammarPath != NULL) {
        LoadGrammarFile(handle, binGrammarPath);
        if (saveToFlash) {
            /* Save the grammar from RAM to the next flash slot */
            HbiWriteWord(handle, 0x034, numGrammars + 1);
            HbiWriteWord(handle, 0x032, 0x800B);
            HbiWriteWord(handle, 0x006, 4);
            BusySpinWait(handle);
            CheckHostCmdStatus(handle);
            printf("Info - Grammar successfully saved to flash slot: %d\n", numGrammars + 1);
        }
    } else if (grammarIdx != -1) {
        if ((grammarIdx == 0) || (grammarIdx > numGrammars)) {
            printf("Error - Illegal flash index, should be: 1 <= N <= %d\n", numGrammars);
        } else {
            /* Load the specified "grammarIdx" from flash */
            HbiWriteWord(handle, 0x034, grammarIdx);
            HbiWriteWord(handle, 0x032, 0x800C);
            HbiWriteWord(handle, 0x006, 4);
            BusySpinWait(handle);
            CheckHostCmdStatus(handle);
            printf("Info - Grammar successfully loaded from flash slot: %d\n", grammarIdx);
        }
    }

    /* Close the HBI driver */
    HBI_close(handle);

    return 0;
}
