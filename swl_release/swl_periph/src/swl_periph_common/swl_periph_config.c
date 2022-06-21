// File: swl_periph_config.c 
// Release: Release 2022-05-27-A
// Copyright (c) 2021-2022, SwaraLink Technologies 
// All Rights Reserved 
// This software is licensed for Production by SwaraLink Technologies to Brilliant Labs Limited 
// Subject to all terms of Software License Agreement SLA-SWL-BRL-2021-07-20 
/************** DO NOT MODIFY THE CONTENTS OF THIS FILE!!! ************/

#include "swl_periph_config.h"

/*********** DO NOT MODIFY ANY OF THE BELOW STATIC VARIABLE DECLARATIONS!!! ********/

static uint8 swl_periph_tx_op_queue_buffer[SWL_PERIPH_TX_OP_QUEUE_BUFFER_SIZE] = {0};
static uint8 swl_periph_tx_by_copy_buffer[SWL_PERIPH_TX_BY_COPY_BUFFER_SIZE] = {0};
static uint8 swl_periph_rx_payload_buffer[SWL_PERIPH_MAX_RX_PAYLOAD_SIZE] = {0};
static uint8 swl_periph_paired_devices_buffer[SWL_PERIPH_PAIRED_DEVICES_BUFFER] = {0};
static uint8 swl_periph_diagnostics_log_buffer[SWL_PERIPH_DIAGNOSTICS_LOG_BUFFER_SIZE] __attribute__ (( section(".non_init") ));

/*********** DO NOT MODIFY ANY OF THE BELOW STRUCTURES!!! ********/

static const swl_periph_config_const_t swl_periph_config_const =
{
  .tx_op_queue_size             = SWL_PERIPH_TX_OP_QUEUE_SIZE,
  .tx_by_copy_buffer_size       = SWL_PERIPH_TX_BY_COPY_BUFFER_SIZE,
  .max_rx_payload_size          = SWL_PERIPH_MAX_RX_PAYLOAD_SIZE,
  .init_pwr_mode                = INIT_PWR_MODE,
  .max_num_paired_devices       = SWL_PERIPH_MAX_NUM_PAIRED_DEVICES,
  .diagnostics_log_size         = DIAGNOSTICS_LOG_SIZE,
};

static const swl_periph_config_buffers_t swl_periph_config_buffers =
{
  .p_tx_op_queue_buffer       = swl_periph_tx_op_queue_buffer,
  .p_tx_by_copy_buffer        = swl_periph_tx_by_copy_buffer,
  .p_rx_payload_buffer        = swl_periph_rx_payload_buffer,
  .p_paired_devices_buffer    = swl_periph_paired_devices_buffer,
  .p_diagnostics_log_buffer   = swl_periph_diagnostics_log_buffer,
};


/************** DO NOT MODIFY THE CONTENTS OF THIS FILE!!! ************/

swl_err_t swl_periph_config(void)
{
  /************** DO NOT MODIFY THIS FUNCTION!!! ************/

  return swl_periph_register_config(&swl_periph_config_const, &swl_periph_config_buffers);
}

/************** DO NOT MODIFY THE CONTENTS OF THIS FILE!!! ************/

// End of file swl_periph_config.c




