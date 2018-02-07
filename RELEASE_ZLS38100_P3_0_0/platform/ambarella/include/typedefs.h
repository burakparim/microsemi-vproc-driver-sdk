/*
* typedefs.h  --  Header file for debugging macros/functions
*
* Copyright 2016 Microsemi Inc.
*/

#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__

/**
 * \file typedefs.h
 * \brief Host Platform Specific Typedefine File.
 *
 * Defines the datatypes to host-specific supported types.
 * This file should be defined or ported by developer responsible
 * for porting System Service Layer over host platform.
 *
 * Following are typical typedefs that must be defined by developer
 *
 * dev_addr_t - device address type. an identifier for device over bus.
 *              bus number, address or chip id.
 *
 * TRUE / FALSE - Boolean value indicating truthfulness of expression
 *
 * ssl_port_handle_t - Device Communication Port Handle. Defined and used by
 *                     System Service Layer.
 *
 * ssl_dev_cfg_t - Device Configuration Type as supported by implementation of
 *                 System Service Layer
 *
 * ssl_drv_cfg_t - Driver initialisation configuration. Defined as per the
 *                 implementation.
 *
*/
#include <linux/string.h>
#include <linux/types.h>
#include <stddef.h>

/*
* Handle the extern differences between c and c++
*/
#ifdef EXTERN
  #undef EXTERN
  #error EXTERN was redefined!
#endif /* undef EXTERN */
#ifdef __cplusplus
  #define EXTERN extern "C"
#else
  #define EXTERN extern
#endif /* __cplusplus */


#ifndef NULL
    #define NULL (0)
#endif

typedef uint16_t dev_addr_t;
typedef int tw_device_id_t;

#ifndef __bool_true_and_false_are_defined
#define __bool_true_and_false_are_defined

#define TRUE        1
#define FALSE      (!TRUE)
#endif

/* typedef for SSL port and lock handle. User can redefine to any as per their system need */
typedef uint32_t ssl_port_handle_t;
typedef uint32_t ssl_lock_handle_t;

/* structure defining device configuration */
typedef struct
{
    tw_device_id_t deviceId;
    dev_addr_t   dev_addr; /* device address */
    uint8_t     *pDevName; /* null terminated  device name as passed by user*/
    uint8_t      bus_num; /* bus id device is connected on */
}ssl_dev_cfg_t;

typedef void ssl_drv_cfg_t;
#endif /*__TYPEDEF_H__ */
