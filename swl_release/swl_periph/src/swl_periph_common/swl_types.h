// File: swl_types.h 
// Release: Release 2022-05-27-A
// Copyright (c) 2021-2022, SwaraLink Technologies 
// All Rights Reserved 
// This software is licensed for Production by SwaraLink Technologies to Brilliant Labs Limited 
// Subject to all terms of Software License Agreement SLA-SWL-BRL-2021-07-20 
#ifndef SWL_TYPES_H__
#define SWL_TYPES_H__
    

// Includes
#if     (SWL_DEVICE_FAMILY == DEV_FAMILY_NRF)
    #include "swl_nrf_52_types.h"
#elif   (SWL_DEVICE_FAMILY == DEV_FAMILY_TI)
    #include "swl_ti_types.h"
#elif   (SWL_DEVICE_FAMILY == DEV_FAMILY_SILABS)
    #include "swl_SILABS_types.h"
#elif   (SWL_DEVICE_FAMILY == DEV_FAMILY_DIALOG)
    #include "swl_dialog_types.h"
#elif   (SWL_DEVICE_FAMILY == DEV_FAMILY_CYPRESS)
    #include "swl_cypress_types.h"
#endif

#define EXCEPTION_FILENAME_CHAR_LEN    6

typedef uint16_t  swl_err_t;

// Error Codes

// 0x000 indicates success (consistent with other APIs)
#define SWL_SUCCESS                           0x0000

// SWL_DEV error codes all start with 0xD00 to attempt to avoid conflict with SDK-defined error codes
#define SWL_ERR_INVALID_STATE                 0xDA00
#define SWL_ERR_INVALID_PARAM                 0xDA01
#define SWL_ERR_INSUFFICIENT_MEMORY           0xDA02
#define SWL_ERR_INSUFFICIENT_BUFFERS          0xDA03
#define SWL_ERR_BUSY                          0xDA04
#define SWL_ERR_NO_DATA_FOUND                 0xDA05
#define SWL_ERR_FLASH_OP_FAILED               0xDA06

#define SWL_ERR_INVALID_LICENSE               0xDAFD
#define SWL_ERR_CRITICAL                      0xDAFE
#define SWL_ERR_UNKNOWN                       0xDAFF


#endif // SWL_TYPES_H__
