// File: swl_periph_nrf_52.h 
// Release: Release 2022-05-27-A
// Copyright (c) 2021-2022, SwaraLink Technologies 
// All Rights Reserved 
// This software is licensed for Production by SwaraLink Technologies to Brilliant Labs Limited 
// Subject to all terms of Software License Agreement SLA-SWL-BRL-2021-07-20 
/**
 * @file swl_periph_nrf_52.h
 *
 * @{
 * @brief Module for application layer functions and variables
 *
 * @details The API consists of functions for passing stack events to the lower layers.
 */


#ifndef SWL_PERIPH_NRF_52_H__

#define SWL_PERIPH_NRF_52_H__

#include "ble.h"
         
/**@brief Used to pass events and data to the lower layers.
  *
  *@param[in]  p_ble_evt     Bluetooth stack event.
  *@param[in]  p_context     Not used.
  */                                                                    
void swl_periph_nrf_process_stack_evt(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Used in the main loop to process events.
  *
  */
void swl_periph_nrf_process_idle_state(void);

#endif // SWL_PERIPH_NRF_52_H__


// End of file swl_periph_nrf_52.h
/** @} */