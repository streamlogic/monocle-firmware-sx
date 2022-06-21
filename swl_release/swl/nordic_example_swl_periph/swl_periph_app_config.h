// File: swl_periph_app_config.h
// Copyright (c) 2021, SwaraLink Technologies
// All Rights Reserved
// Licensed by SwaraLink Technologies, subject to terms of Software License Agreement

#ifndef SWL_PERIPH_APP_CONFIG_H__
#define SWL_PERIPH_APP_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "swl_types.h"
#include "swl_periph.h"


// Application Configuration Parameters
#define TX_OP_QUEUE_SIZE                100
#define TX_BY_COPY_BUFFER_SIZE          1024
#define MAX_RX_PAYLOAD_SIZE             1024
#define INIT_PWR_MODE                   SWL_PWR_MODE_SPM4
#define MAX_NUM_PAIRED_DEVICES          5  //DO NOT modify. Feature is still under development!
#define DIAGNOSTICS_LOG_SIZE            256

/*
// Advanced Configuration Parameters
#define ADV_INT_SPM2A               874     //Units of 0.625 ms. 546.25 ms.
#define CONN_INT_SPM2A              792     //Units of 1.25 ms. 990 ms.
#define PERIPH_LATENCY_SPM2A        0
#define SUPER_TIMEOUT_SPM2A         600 //In terms of 10 ms. 6000 ms
#define ADV_INT_SPM2B               338 //Uints of 0.625 ms. 211.25 ms.
#define CONN_INT_SPM2B              72  //Uints of 1.25 ms. 90 ms
#define PERIPH_LATENCY_SPM2B        9
#define SUPER_TIMEOUT_SPM2B         400 //In terms of 10 ms. 4000 ms
#define ADV_INT_SPM2C               338 //Uints of 0.625 ms. 211.25 ms.
#define CONN_INT_SPM2C              72  //Uints of 1.25 ms. 90 ms.
#define PERIPH_LATENCY_SPM2C        0
#define SUPER_TIMEOUT_SPM2C         200 //In terms of 10 ms. 2000 ms
*/


#ifdef __cplusplus
}
#endif

#endif // SWL_PERIPH_APP_CONFIG_H__

// End of file swl_periph_app_config.h


