/*
* hbi_tw.c - Private file with internal function defined for use by hbi user space driver
*
*
* Copyright 2016 Microsemi Inc.
*
*/

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#include "typedefs.h"
#include "chip.h"
#include "hbi.h"
#include "hbi_prv.h"
#include "vproc_u_dbg.h"


/* TW firmware bin image header description */
/* header field width in bytes */
#define VER_WIDTH          1
#define FORMAT_WIDTH       1
#define OPN_WIDTH          2
#define BLOCK_SIZE_WIDTH   2
#define TOTAL_LEN_WIDTH    4
#define RESERVE_LEN_WIDTH  2
/* unused right now */
#define FWR_CHKSUM_LEN     1
#define IMG_HDR_LEN    \
   (VER_WIDTH + FORMAT_WIDTH +  \
   OPN_WIDTH + BLOCK_SIZE_WIDTH + TOTAL_LEN_WIDTH + RESERVE_LEN_WIDTH)

/* field index */
#define VER_INDX        0
#define FORMAT_INDX     (VER_INDX+VER_WIDTH)
#define FWR_OPN_INDX    (FORMAT_INDX+FORMAT_WIDTH)
#define BLOCK_SIZE_INDX (FWR_OPN_INDX + OPN_WIDTH)
#define TOTAL_LEN_INDX  (BLOCK_SIZE_INDX + BLOCK_SIZE_WIDTH)

/* Image Version Info */
#define IMG_VERSION_MAJOR_SHIFT 6
#define IMG_VERSION_MINOR_SHIFT 4

/* image type fields */
#define IMG_HDR_TYPE_SHIFT    6
#define IMG_HDR_ENDIAN_SHIFT  5


hbi_status_t internal_hbi_get_hdr(hbi_data_t *pImg,hbi_img_hdr_t *pHdr)
{
   
   if(pImg == NULL || (pImg->size < IMG_HDR_LEN) || pHdr==NULL)
   {
      VPROC_U_DBG_PRINT(VPROC_DBG_LVL_ERR, "Invalid arguments passed\n:");
      return HBI_STATUS_INVALID_ARG;
   }

   pHdr->major_ver = pImg->pData[VER_INDX] >> IMG_VERSION_MAJOR_SHIFT;
   pHdr->minor_ver = pImg->pData[VER_INDX] >> IMG_VERSION_MINOR_SHIFT;
   pHdr->image_type = pImg->pData[FORMAT_INDX] >> IMG_HDR_TYPE_SHIFT;

   if(pHdr->image_type == HBI_IMG_TYPE_FWR)
   {
      pHdr->endianness = pImg->pData[FORMAT_INDX] >> IMG_HDR_ENDIAN_SHIFT;
      pHdr->fwr_code   = (pImg->pData[FWR_OPN_INDX] << 8) | pImg->pData[FWR_OPN_INDX+1];
   }
   pHdr->block_size = (pImg->pData[BLOCK_SIZE_INDX] << 8) | pImg->pData[BLOCK_SIZE_INDX+1];

   pHdr->img_len = pImg->pData[TOTAL_LEN_INDX] << 24; 
   pHdr->img_len |= pImg->pData[TOTAL_LEN_INDX+1] << 16; 
   pHdr->img_len |= pImg->pData[TOTAL_LEN_INDX+2] <<8; 
   pHdr->img_len |= pImg->pData[TOTAL_LEN_INDX+3];

   pHdr->hdr_len = IMG_HDR_LEN;
   VPROC_U_DBG_PRINT(VPROC_DBG_LVL_INFO, "image_type %d, major %d minor %d, block size %d," \
                  "total len %d, code %d, endian %d\n",
                  pHdr->image_type, 
                  pHdr->major_ver,
                  pHdr->minor_ver,
                  pHdr->block_size,
                  pHdr->img_len,
                  pHdr->fwr_code,
                  pHdr->endianness);
   return HBI_STATUS_SUCCESS;
}

