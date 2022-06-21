// File: swl_nrf_52_types.h 
// Release: Release 2022-05-27-A
// Copyright (c) 2021-2022, SwaraLink Technologies 
// All Rights Reserved 
// This software is licensed for Production by SwaraLink Technologies to Brilliant Labs Limited 
// Subject to all terms of Software License Agreement SLA-SWL-BRL-2021-07-20 
#ifndef SWL_NRF_52_TYPES_H__

#define SWL_NRF_52_TYPES_H__
#include "app_util_platform.h"
#include "app_error.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

typedef uint8_t     uint8;
typedef int8_t      int8;
typedef uint16_t    uint16;
typedef int16_t     int16;
typedef uint32_t    uint32;
typedef int32_t     int32;
typedef uint64_t    uint64;
typedef int64_t     int64;

// Programatic Breakpoint
#define SWL_BKPT                    NRF_BREAKPOINT_COND

// Error check macro
#define SWL_ERROR_CHECK(ERR_CODE)   APP_ERROR_CHECK(ERR_CODE)

#endif /* SWL_NRF_52_TYPES_H_ */

