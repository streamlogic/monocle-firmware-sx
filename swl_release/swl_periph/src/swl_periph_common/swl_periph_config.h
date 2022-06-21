// File: swl_periph_config.h 
// Release: Release 2022-05-27-A
// Copyright (c) 2021-2022, SwaraLink Technologies 
// All Rights Reserved 
// This software is licensed for Production by SwaraLink Technologies to Brilliant Labs Limited 
// Subject to all terms of Software License Agreement SLA-SWL-BRL-2021-07-20 
#ifndef SWL_PERIPH_CONFIG_H__
#define SWL_PERIPH_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/************** DO NOT MODIFY THE CONTENTS OF THIS FILE!!! ************/

// SWL Peripheral Application Configuration - Modify this file for build-time configuration
#include "swl_periph_app_config.h"

// SWL Peripheral Library Includes
#include "swl_types.h"
#include "swl_periph.h"


/************** DO NOT MODIFY ANY OF THE VALUES BELOW **************/

#define TX_OP_QUEUE_BUFFER_ELEMENT_SIZE           14
#define PAIRED_DEVICES_BUFFER_ELEMENT_SIZE        12
#define DIAGNOSTICS_LOG_BUFFER_ELEMENT_SIZE       3

/************** DO NOT MODIFY ANY OF THE VALUES BELOW **************/

#define SWL_PERIPH_TX_OP_QUEUE_SIZE               (TX_OP_QUEUE_SIZE + 1)
#define SWL_PERIPH_TX_OP_QUEUE_BUFFER_SIZE        (SWL_PERIPH_TX_OP_QUEUE_SIZE * TX_OP_QUEUE_BUFFER_ELEMENT_SIZE)
#define SWL_PERIPH_TX_BY_COPY_BUFFER_SIZE         (TX_BY_COPY_BUFFER_SIZE + 1)
#define SWL_PERIPH_MAX_RX_PAYLOAD_SIZE            (MAX_RX_PAYLOAD_SIZE)
#define SWL_PERIPH_MAX_NUM_PAIRED_DEVICES         (MAX_NUM_PAIRED_DEVICES + 1)
#define SWL_PERIPH_PAIRED_DEVICES_BUFFER          (SWL_PERIPH_MAX_NUM_PAIRED_DEVICES * PAIRED_DEVICES_BUFFER_ELEMENT_SIZE) 
#define SWL_PERIPH_DIAGNOSTICS_LOG_BUFFER_SIZE    (DIAGNOSTICS_LOG_SIZE * DIAGNOSTICS_LOG_BUFFER_ELEMENT_SIZE)

/************** DO NOT MODIFY THE CONTENTS OF THIS FILE!!! ************/

// Config Function Prototype
swl_err_t swl_periph_config(void);


#ifdef __cplusplus
}
#endif

#endif // SWL_PERIPH_CONFIG_H__

// End of file swl_periph_config.h


