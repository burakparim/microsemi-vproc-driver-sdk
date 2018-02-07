#include <stdio.h>
#include <stdint.h>

/*VPROC SDK mandatory includes*/
#include "typedefs.h"
#include "chip.h"
#include "hbi.h"

#define VPD1_DEVICE_ID 0 /*device ID of that Timberwolf*/
void main (void)
{
    hbi_dev_cfg_t devcfg;
    hbi_handle_t handle;
    reg_addr_t reg = 0x00E;
    user_buffer_t buf[] = {0x12, 0x34};
    hbi_status_t status = HBI_STATUS_SUCCESS; 
    int i = 0;
   
    devcfg.deviceId = VPD1_DEVICE_ID;
    devcfg.pDevName = NULL;
    /*Open one instance of the Timberwolf identified by VPD1_DEVICE_ID */ 
    status = HBI_open(&handle,&devcfg);
    if(status != HBI_STATUS_SUCCESS)
    {
         printf("HBI ERROR: %d, HBI_open() \n", status);
         return;
    }
   /*Write the content of buf into the Timberwolf register defined in reg*/
    status  = HBI_write(handle,  reg, buf, sizeof(buf));
    if (status != HBI_STATUS_SUCCESS) {
        printf("HBI ERROR: %d, HBI_write() failed\n", status);
        HBI_close(handle);
        return;
    }
   /*Read the Timberwolf register defined in reg and store the result in buf*/ 
    status  = HBI_read(handle, reg, buf, sizeof(buf));
    if (status != HBI_STATUS_SUCCESS) {
            printf("HBI ERROR: %d, HBI_read() failed\n", status);
        HBI_close(handle);
        return;
        
    }
    /*Print the read results stored in buf*/
    printf("Results:\t");
    for(i=0;i<sizeof(buf);i++)
        printf("0x%02x\t", buf[i]);

    printf("\n");
    /*Close the opened instance*/
    HBI_close(handle);
    return; 
}
