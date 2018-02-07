#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include "typedefs.h"
#include "chip.h"
#include "hbi.h"

/*! \file hbi_test.c */

/********************************************/
/*! \mainpage 
*
* \section intro_sec Introduction
*
* This is an document summarizes test suite for verifying Microsemi Voice 
* Device SDK. 
*
* \section BI Build Instructions
*
* Sample Makefile available with apps that build sample applicatons for
* linux platform as userspace apps. User can modify Makefile as per their 
* platform.
* In exisiting Makefile,all sample applications are built as an independent 
* execuatable binaries.hbi_test sample app is built by default.
* To enable compilation of other sample, app following TEST Macros are defined 
*
* TEST_DOA - Builds Direction-of-arrival sample app
*
* Example Make Command: 
*
* make apps - This build only hbi_test sample app
*
* make apps TEST_DOA=1 -  Builds both hbi_test and test_doa
*
*
* \defgroup hbi_test hbi_test
* hbi_test is a simple application to test HBI driver functions and demonstrate
* its usage.
*
* Test is based on various TEST macros and command line options
* User can enable/disable them at compile-time to validate specific set of 
* functions.
*
* Test Macros to test various hbi driver feature 
*                                                       
* TEST_RST - Resets the device. A successful reset 
*                                          verify HBI_read and HBI_write function 
*                                          calls 
*                                                               
* TEST_LOAD_FWRCFG_FROM_FLASH - Test firmware loading from flash. 
*                                Load image number 1 from flash. 
*
*                                                                   
* TEST_ERASE_IMAGE -            Test Erasing a particular image from flash.
*                                Erase image number 1 from flash 
*
* TEST_ERASE_FLASH -            Test Erasing whole flash
*
*
* To Execute test case, simply run:
*
* ./hbi_test 
*
* To read specific register
*
* ./hbi_test -r <reg address in hex> <number of bytes to read in decimal>, example
*
* To read 10 bytes from address 0x200
*
* ./hbi_test -r 0x200 10
*
* To write single word at specific register
*
* hbi_test -w <reg address in hex> <16-bit length word>
*
* Example, to write 0xDEAD at register 0x200
*
* hbi_test -w 0x200 0xDEAD
*
* To write multiple words starting register 0x200
*
* hbi_test -w 0x200 0xDEAD 0xBEEF
*
* To Display help menu
*
* ./hbi_test -h
*
********************************************************************************/


/*! \ingroup hbi_test
 * \{
 *
*/

/* size of buffer for HBI read/write*/
#define MAX_RW_SIZE 256

/* Uncomment to enable any test option */
//#define TEST_RST                    
         
int hbi_dump_reg(void);
static hbi_handle_t handle;

static void usage(void)
{
   printf("Description: Test is based on various test macros\n");

   printf("To verify specific TEST, compile test application with TEST Macro enabled." \
          " if not enabled, test will simply does HBI driver basic API call testing\n");

   printf("usage: ./hbi_test <options>\n");

   printf("Options: -d <deviceID> -r <reg> <len> - read length bytes from register reg." \
          " reg value is hex and len is input in bytes in decimal\n");

   printf("Example to read 10 bytes from reg 0x200 of device 0\n");
   printf("hbi_test -d 0 -r 0x200 10 \n");

   printf(" -w <reg in hex> <16-bit length reg value in hex> - writes words in to" \
          "reg \n");

   printf("Example: to write 0x0102 0x0304 at register 0x200 of device 0\n");   
   printf("hbi_test -d 0 -w 0x200 0x0102 0x0304\n");   
   printf(" Example: to read 2 bytes and write reg 0x200 and 0x100\n");

   printf("hbi_test -d 0 -r 0x200 2 -w 0x100 0xDEAD\n");
   printf(" -lf <image number>  :to load a firmware number specified by image number from flash \n");
   printf(" Example: to load firmware 1 from flash of device 0\n");
   printf("hbi_test -d 0 -lf 1\n");

   printf(" -e <image number>   :to erase a firmware specified by image number from flash \n");
   printf(" Example: to erase firmware 1 from flash of device 0\n");
   printf("hbi_test -d 0 -e 1\n");

   printf(" -ef  :to erase all firmware on the flash \n");
   printf(" Example: to erase all firmware from the flash of device 0\n");
   printf("hbi_test -d 0 -ef\n");

   printf(" -rst  :to reset the device to boot mode\n");
   printf(" Example: to reset the device 0\n");
   printf("hbi_test -d 0 -rst\n");


   return;
}


/*
 Internal function to convert string to hex value. 
*/
char * strtoh (char *str,int len,unsigned char *val)
{
   int i=0;

   if(*str == '0' && (*(str+1) == 'x' || *(str+1) == 'X'))
   {
      str+=2;
   }

   while(*str!='\0' && i<len)
   {

      if(*str >= 'a' && *str <= 'f' )
      {
         *val <<= 4;
         *val |= (*str-'a')+0xa;
         i++;
      }
      if(*str >= 'A' && *str <= 'F')
      {
         *val <<= 4;
         *val |= (*str-'A')+0xa;
         i++;
      }
      if(*str >= '0' && *str <= '9')
      {
         *val <<= 4;
         *val |= (*str-'0');
         i++;
      }
      str++;
   }
    return str;
}
int hbi_test(int argc, char** argv) {

   hbi_status_t status = HBI_STATUS_SUCCESS;
   hbi_dev_cfg_t devcfg;
   reg_addr_t    reg;
   user_buffer_t buf[MAX_RW_SIZE];
   size_t        len;
   int           i,j=0;
   char           *in;
   hbi_device_id_t deviceId = -1;


   i=1;
   if(strcmp(argv[i],"-d")==0) {
        deviceId = strtol(argv[++i],NULL,16);	  
   } else {
        printf("Invalid device number !!!\n");
        return;   
   }

   if ((deviceId < 0) || (deviceId >=VPROC_MAX_NUM_DEVS))
   {
       printf("Incorrect deviceId - deviceID must be within 0 to %d \n", VPROC_MAX_NUM_DEVS-1);
       return;
   }

#if 0
   status = HBI_init(NULL);
   if (status != HBI_STATUS_SUCCESS)
   {
     printf("HBI_init failed\n");
     return -1;
   }
#endif

   devcfg.deviceId = deviceId;
   devcfg.pDevName = NULL;

   status = HBI_open(&handle,&devcfg);
   if(status != HBI_STATUS_SUCCESS)
   {
     printf("dev open error\n");
     return -1;
   }

   i=3;
   while(i<argc)
   {
      if(strcmp(argv[i],"-h")==0)
      {
         usage();
         HBI_close(handle);
         return 1;
      }
      if(strcmp(argv[i],"-r")==0)
      {
         i++;
         
         reg=strtol(argv[i++],NULL,16);
         len=strtol(argv[i++],NULL,10);

         if (len % 2)
         {
            printf("Length of data should be in multiple of 2\n");
            HBI_close(handle);
            return -1;
         }


         if(len >= MAX_RW_SIZE)
         {
            printf("Max supported r/w size is %d\n",MAX_RW_SIZE);
            len = MAX_RW_SIZE;
         }   
       
         status = HBI_read(handle, reg, buf, len);
         if(status == HBI_STATUS_SUCCESS)
         {
            printf("Read %d bytes\n",len);
            for (j = 0; j < len; ) {
                printf("RD: addr 0x%04x = 0x%02x%02x\n", (reg+j), buf[j++],buf[j++]);
            }
         }
         else
         {
            printf("HBI_read failed. Error 0x%x\n",status);
            HBI_close(handle);
         }
      }
      if((i<argc) && strcmp(argv[i],"-w")==0)
      {
         j=0;
         i++;

         reg=strtol(argv[i++],NULL,16);

         memset(buf,0,sizeof(buf));

         while( (i<argc) && j<sizeof(buf))
         {  
            in = argv[i];

            /* store byte-by-byte */
            in=strtoh(in,2,&(buf[j]));

            j++;

            in=strtoh(in,2,&(buf[j]));

            j++;
            i++;
         }
         
         len=j;
         for(j=0;j<len;)
         {
            printf("wr: addr 0x%04X = 0x%02X%02X\n", reg+j, buf[j++],buf[j++]);
         }
         status = HBI_write(handle,reg,buf,len);
         if(status != HBI_STATUS_SUCCESS)
         {
            printf("HBI_write() Failed\n");
            HBI_close(handle);
         }
      }
      if((i<argc) && strcmp(argv[i],"-lf")==0)
      {    
            /*Load ZL380x0 firmware + related config record from flash*/
            unsigned short image_num = strtol(argv[++i],NULL,16);

            status  = HBI_set_command(handle,HBI_CMD_LOAD_FWRCFG_FROM_FLASH,&image_num);
            if (status != HBI_STATUS_SUCCESS) {
                printf("Error %d:HBI_set_command(HBI_CMD_LOAD_FWRCFG_FROM_FLASH)\n", status);
                HBI_close(handle);
                return -1;
            }
           printf("\nDevice boot loading from flash completed successfully...\n");
        
           printf("\n Start Firmware\n");
        
           status  = HBI_set_command(handle,HBI_CMD_START_FWR,NULL);
           if (status != HBI_STATUS_SUCCESS) {
               printf("Error %d:HBI_set_command(HBI_CMD_START_FWR)\n", status);
               HBI_close(handle);
               return -1;
           }
        
        } 
        if((i<argc) && strcmp(argv[i],"-e")==0)
        {
           int32_t image_num= strtol(argv[++i],NULL,16);
           
           /*Erase the full content of the ZL380x0 controlled slave flash*/              
           status  = HBI_set_command(handle, HBI_CMD_ERASE_FWRCFG_FROM_FLASH,&image_num);        
           if (status != HBI_STATUS_SUCCESS) {
               printf("Error %d:HBI_set_command(HBI_CMD_ERASE_FWRCFG_FROM_FLASH)\n", status);
               HBI_close(handle);
               return -1;
           }
           printf("Erased image %d completed successfully...\n",image_num>>8);         
        }
        if((i<argc) && strcmp(argv[i],"-dump")==0)
        {          
           /*dump all registers*/ 
           status  = hbi_dump_reg();        
           if (status != HBI_STATUS_SUCCESS) {
               printf("Error %d:HBI_set_command(HBI_CMD_ERASE_FWRCFG_FROM_FLASH)\n", status);
               HBI_close(handle);
               return -1;
           }         
        }        
                
        if((i<argc) && strcmp(argv[i],"-ef")==0)
        { 
            /*Erase the full content of the ZL380x0 controlled slave flash*/              
            status  = HBI_set_command(handle, HBI_CMD_ERASE_WHOLE_FLASH, NULL);        
            if (status != HBI_STATUS_SUCCESS) {
                printf("Error %d:HBI_set_command(HBI_CMD_ERASE_WHOLE_FLASH)\n", status);
                HBI_close(handle);
                return -1;
            }
            printf("flash erasing completed successfully...\n");         
        }                      
        if((i<argc) && strcmp(argv[i],"-rst")==0)
        { 
        #define TEST_BUF_SIZE 12
            uint8_t val[TEST_BUF_SIZE];
            reg_addr_t reg=0x0020;
            int i;
           
            printf("value @ 0x%04x before...\n",reg);
            
            status = HBI_read(handle,reg,(user_buffer_t *)val,sizeof(val));
            if(status == HBI_STATUS_SUCCESS)
            {
                for(i=0;i<sizeof(val);i++)
                    printf("0x%02x\t", val[i]);
            }
            val[0] = ZL380xx_CLK_STATUS_HWRST >> 8;
            val[1] = ZL380xx_CLK_STATUS_HWRST & 0xFF;
            
            status  = HBI_write(handle,ZL380xx_CLK_STATUS_REG,(user_buffer_t *)val,2);        
            if (status != HBI_STATUS_SUCCESS) {
                printf("Error %d:HBI_reset()\n", status);
                HBI_close(handle);
                return -1;
            }
            printf("\nvalue @ 0x%04x after ....\n",reg);
            
            for(i=0;i<500;i++) /*delay*/
            
            status = HBI_read(handle,reg,(user_buffer_t *)val,sizeof(val));
            if(status == HBI_STATUS_SUCCESS)
            {
                for(i=0;i<sizeof(val);i++)
                    printf("0x%02x\t", val[i]);
            }            
            printf("\nDevice reset completed successfully...\n");

        }                      
      i++;
   }

   status = HBI_close(handle);
   if(status != HBI_STATUS_SUCCESS)
   {
      printf("dev close error\n");
      return -1;
   }

   return 0;

}

static void prntdata(user_buffer_t *buf, reg_addr_t start_reg, int len)
{
   int j, n=0;
   #define NUM_BLOCKS 14

   printf("0x%04x-0x%04x: ",start_reg, start_reg+NUM_BLOCKS); 
   for(j=0;j<len; ) {
      printf("0x%02X%02X ", buf[j++],buf[j++]);
      ++n;
      
         if (n > 7) {
            printf("\n");  
            start_reg +=(NUM_BLOCKS+2); 
            if (j < len-1)         
               printf("0x%04x-0x%04x: ",start_reg, start_reg+NUM_BLOCKS);  
            n=0;
                           
         }
   }
   printf("\n\n");
   return;
}

int hbi_dump_reg(void)
{
    int i;
    int num_pages_to_read = 13;
    reg_addr_t start_reg = 0x0000; 
    user_buffer_t val[MAX_RW_SIZE];
    hbi_status_t status = HBI_STATUS_SUCCESS;

    printf("ZL380xx Register Dump:\n"); 
    
    for (i=0;i < num_pages_to_read; i++) {  
        status = HBI_read(handle,start_reg,(user_buffer_t *)val, sizeof(val));   
        if (status != HBI_STATUS_SUCCESS) {
            printf("Error %d:VprocTwolfHbiRead()\n", status);
            return -1;
        }
        prntdata(val, start_reg, MAX_RW_SIZE);          
        start_reg +=0x0100;
    }
    return status;   
}

void main (int argc, char** argv)
{
   hbi_test(argc,argv);
   return;
}


