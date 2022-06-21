// File: main.c
// Copyright (c) 2021, SwaraLink Technologies
// All Rights Reserved
// Licensed by SwaraLink Technologies, subject to terms of Software License Agreement

// Note: the contents of this file are derived from source code written by
// Nordic Semiconductor. The original Nordic license header and copyright notice
// are included below



/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @brief Nordic Example SWL Peripheral Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

// C standard library header files
#include <stdint.h>
#include <string.h>

// Board file
#include "boards.h"

// Nordic libraries
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_crypto.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_error.h"
#include "nordic_common.h"
#include "sdk_errors.h"
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#endif

// SWL Peripheral Application Configuration - Modify this file for build-time configuration
#include "swl_periph_app_config.h"

// SWL Peripheral Library Includes
#include "swl_periph.h"
#include "swl_periph_config.h"
#include "swl_periph_nrf_52.h"
#include "swl_periph_demo.h"

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SWL_PAIRING_TIMEOUT_DEFAULT     120         // 120 seconds pairing timeout

#define SWL_BUTTON_1                     BSP_BUTTON_0   
#define SWL_BUTTON_2                     BSP_BUTTON_1
#define SWL_BUTTON_3                     BSP_BUTTON_2
#define SWL_BUTTON_4                     BSP_BUTTON_3

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

//Records the # of times the demo 4k file needs to be sent.
//Wait until full 4k file is sent to another.
static uint8 data_tx_count = 0;

// Bool to track whether data is being sent by copy or reference.
static bool data_sent_by_copy;

// Bool to track whether data being sent requires an acknowledgement
static bool ack_required = false;

// Array for pass by copy file transfer demo.
const uint8 test_array[64] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,
                               23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,
                               42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,
                               61,62,63,64 };

static bool ble_connection_established = false;

//Used to tell whether we are in pairing mode
static bool in_pairing_mode = false;

//Tracks the current power mode.
static swl_pwr_mode_t current_spm;

static void handle_rx_data(const swl_c2p_data_t * p_data);
void dump_diagnostic_log(void);

uint8 test_c2p_array[1000] = {0};

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


// Custom declaration of app_error_fault_handler, overwrites the default declaration.
// Function is called due to APP_ERROR_CHECK() not due to SWL Library
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    // Error has occurred
    NRF_LOG_INFO("Error has occurred!");
    NRF_LOG_INFO("Error ID: %u", id);
    NRF_LOG_INFO("Error PC: %u", pc);
    NRF_LOG_INFO("Error Info: %u", info);

    __disable_irq();
    NRF_LOG_FINAL_FLUSH();

    switch (id)
    {
        case NRF_FAULT_ID_SD_ASSERT:
            NRF_LOG_ERROR("SOFTDEVICE: ASSERTION FAILED");
            break;

        case NRF_FAULT_ID_APP_MEMACC:
            NRF_LOG_ERROR("SOFTDEVICE: INVALID MEMORY ACCESS");
            break;

        case NRF_FAULT_ID_SDK_ASSERT:
            {
                assert_info_t * p_info = (assert_info_t *)info;
                NRF_LOG_ERROR("ASSERTION FAILED at %s:%u",
                              p_info->p_file_name,
                              p_info->line_num);
            }
            break;

        case NRF_FAULT_ID_SDK_ERROR:
            {
                error_info_t * p_info = (error_info_t *)info;
                NRF_LOG_ERROR("ERROR 0x%08x [%s] at %s:%u\r\nPC at: 0x%08x",
                              p_info->err_code,
                              nrf_strerror_get(p_info->err_code),
                              p_info->p_file_name,
                              p_info->line_num,
                              pc);
                 NRF_LOG_ERROR("End of error report");
            }
            break;

        default:
            NRF_LOG_ERROR("UNKNOWN FAULT at 0x%08X", pc);
            break;
    }

    // retrieve and print out swl_periph diagnostic log raw data
    dump_diagnostic_log();

    // Trigger breakpoint if debugger is connected
    SWL_BKPT;

    // it is recommend to soft reset the system as it may be in an unreliable state due to the unknown error
    sd_nvic_SystemReset();

    // infinite loop; code should never reach here
    for(;;)
    {}
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    // forward BLE stack events to the swl_periph event handler
    swl_periph_nrf_process_stack_evt(p_ble_evt, p_context);
}

static void periph_evt_handler(swl_periph_evt_t evt, const swl_periph_evt_data_t * p_evt_data)
{
  swl_err_t err_code;

  NRF_LOG_INFO("App Event: %u", (uint8)evt);

  // process events
  switch(evt)
  {
    case SWL_PERIPH_EVT_CONN_ESTABLISHED:
      NRF_LOG_INFO("Device ID: %u", p_evt_data->conn_establish.device_id);
      bsp_board_led_on(BSP_BOARD_LED_0);
      ble_connection_established = true;
      break;

    case SWL_PERIPH_EVT_CONN_TERMINATED:
      NRF_LOG_INFO("Device ID: %u", p_evt_data->conn_terminate.device_id);
      ble_connection_established = false;

      bsp_board_led_off(BSP_BOARD_LED_0);
      break;

    case SWL_PERIPH_EVT_RX_DATA:
      // send back whatever was received
      handle_rx_data((swl_c2p_data_t*)p_evt_data);
      break;

    case SWL_PERIPH_EVT_DATA_ACK:
      //data was acknowledged by peer device.
      NRF_LOG_INFO("Major: %u Minor: %u", p_evt_data->data_info.id_hdr_major, p_evt_data->data_info.id_hdr_minor);
      break;

    case SWL_PERIPH_EVT_DATA_SENT:
      //Event generated when data transfer has been completely queued up by the Bluetooth stack
      //Additional data can be queued up once this event is recieved.

      NRF_LOG_INFO("Major: %u Minor: %u", p_evt_data->data_info.id_hdr_major, p_evt_data->data_info.id_hdr_minor);

      // are there any pending transmits?
      if(data_tx_count > 0)
      {
        data_tx_count--;

        //Check whether data is being sent by copy or reference.
        if(data_sent_by_copy)
        {
          err_code = swl_periph_p2c_tx_by_copy(test_array, sizeof(test_array), ack_required, ID_HDR_MAJ_P2C_DEMO_RSP, ID_HDR_MIN_P2C_DEMO_RSP_TX_BY_COPY_NO_ACK);
          SWL_ERROR_CHECK(err_code);
        }
        else
        {
          err_code = swl_periph_p2c_tx_by_ref(demofile_array, DEMOFILE_SIZE, ack_required, ID_HDR_MAJ_P2C_DEMO_RSP, ID_HDR_MIN_P2C_DEMO_RSP_TX_BY_REF_NO_ACK);
          SWL_ERROR_CHECK(err_code);
        } 
       
        
      }
      break;

    case SWL_PERIPH_EVT_PWR_MODE_UPDATE:
      //Power mode has been successfully changed
      break;

    case SWL_PERIPH_EVT_PWR_MODE_UPDATE_REQUEST:
      //Request to change power mode has be recieved.
      break;

    case SWL_PERIPH_EVT_PAIRING_MODE_EXITED:
      //Event generated when pairing mode has exited
       NRF_LOG_INFO("Device ID: %u Reason: %u", p_evt_data->pairing_mode_exited.device_id, p_evt_data->pairing_mode_exited.swl_pairing_mode);
       
       //Check whether this event was generated because the pairing mode timer expired.
       if((p_evt_data->pairing_mode_exited.swl_pairing_mode == SWL_PAIRING_MODE_TIMER_EXPIRED) &&
          (ble_connection_established == false) )
       {
       }

       in_pairing_mode = false;

       bsp_board_led_off(BSP_BOARD_LED_1);

      break;

    default:
      // unexpected event;
      break;
  }
}

static void handle_rx_data(const swl_c2p_data_t * p_rx_data)
{
  swl_err_t err_code;
  uint8 id_hdr_maj = p_rx_data->id_hdr_major;
  uint16 rx_data_len = p_rx_data->len;

  memcpy(test_c2p_array, p_rx_data->p_data, p_rx_data->len);

  // SWL demo commands all have id_hdr_maj value of ID_HDR_MAJ_P2C_DEMO_REQ
  // and have a 1-byte payload

  if((p_rx_data->id_hdr_major == ID_HDR_MAJ_P2C_DEMO_REQ) && (rx_data_len == 1))
  {
    // this was a demo req; process based on id_hdr_min value
    uint8 req_type = p_rx_data->id_hdr_minor;

    switch(req_type)
    {
      case ID_HDR_MIN_P2C_DEMO_REQ_TX_BY_REF_NO_ACK:        
        {
          // queue up data by reference with no ack(use 4096 byte demofile_array)
          
          // byte 0 of payload is the number of times to repeat the transfer
          uint8 repeat_ct = p_rx_data->p_data[0];
          
          // valid range is 1-100 (if out of range, then ignore command)
          if((repeat_ct >= 1) && (repeat_ct <= 100))
          {
            // valid count
            data_tx_count = repeat_ct;

            data_sent_by_copy = false;

            ack_required = false;

            // decrement count since first transmit has been queued
            data_tx_count--;
            
            // queue up first transfer
            err_code = swl_periph_p2c_tx_by_ref(demofile_array, DEMOFILE_SIZE, false, ID_HDR_MAJ_P2C_DEMO_RSP, ID_HDR_MIN_P2C_DEMO_RSP_TX_BY_REF_NO_ACK);
            SWL_ERROR_CHECK(err_code);
          }
        }
        break;

      case ID_HDR_MIN_P2C_DEMO_REQ_TX_BY_COPY_NO_ACK:
        {
          // queue up data by copy with no ack (use 64-byte data packet)
          
          // byte 0 of payload is the number of times to repeat the transfer
          uint8 repeat_ct = p_rx_data->p_data[0];

          // valid range is 1-100 (if out of range, then ignore command)
          if((repeat_ct >= 1) && (repeat_ct <= 100))
          {
            // valid count
            data_tx_count = repeat_ct;

            data_sent_by_copy = true;

            ack_required = false;

            // decrement count since first transmit has been queued
            data_tx_count--;

            // queue up first transfer
            err_code = swl_periph_p2c_tx_by_copy(test_array, sizeof(test_array), false, ID_HDR_MAJ_P2C_DEMO_RSP, ID_HDR_MIN_P2C_DEMO_RSP_TX_BY_COPY_NO_ACK);
            SWL_ERROR_CHECK(err_code);
          }
        }
        break;

      case ID_HDR_MIN_P2C_DEMO_REQ_TX_BY_REF_ACK:        
        {
          // queue up data by reference with ack (use 4096 byte demofile_array)
          
          // byte 0 of payload is the number of times to repeat the transfer
          uint8 repeat_ct = p_rx_data->p_data[0];
          
          // valid range is 1-100 (if out of range, then ignore command)
          if((repeat_ct >= 1) && (repeat_ct <= 100))
          {
            // valid count
            data_tx_count = repeat_ct;

            data_sent_by_copy = false;

            ack_required = true;

            // decrement count since first transmit has been queued
            data_tx_count--;
            
            // queue up first transfer
            err_code = swl_periph_p2c_tx_by_ref(demofile_array, DEMOFILE_SIZE, true, ID_HDR_MAJ_P2C_DEMO_RSP, ID_HDR_MIN_P2C_DEMO_RSP_TX_BY_REF_ACK);
            SWL_ERROR_CHECK(err_code); 
          }
        }
        break;

      case ID_HDR_MIN_P2C_DEMO_REQ_TX_BY_COPY_ACK:
        {
          // queue up data by copy with ack (use 64-byte data packet)
          
          // byte 0 of payload is the number of times to repeat the transfer
          uint8 repeat_ct = p_rx_data->p_data[0];

          // valid range is 1-100 (if out of range, then ignore command)
          if((repeat_ct >= 1) && (repeat_ct <= 100))
          {
            // valid count
            data_tx_count = repeat_ct;

            data_sent_by_copy = true;

            ack_required = true;

            // decrement count since first transmit has been queued
            data_tx_count--;

            // queue up first transfer
            err_code = swl_periph_p2c_tx_by_copy(test_array, sizeof(test_array), true, ID_HDR_MAJ_P2C_DEMO_RSP, ID_HDR_MIN_P2C_DEMO_RSP_TX_BY_COPY_ACK);
            SWL_ERROR_CHECK(err_code);
          }
        }
        break;

      case ID_HDR_MIN_P2C_DEMO_REQ_SET_PWR_MODE:
        {
          // request to update power mode
          swl_pwr_mode_t new_pwr_mode = (swl_pwr_mode_t)(p_rx_data->p_data[0]);

          // switch power mode
          err_code = swl_periph_set_power_mode(new_pwr_mode);
          SWL_ERROR_CHECK(err_code);
        }
        break;

      default:
        // do nothing
        break;
    }
  }
}


void dump_diagnostic_log(void)
{
    uint8 * p_diag_log;

    // retrieve diagnostic log
    swl_periph_get_diagnostic_log(&p_diag_log);
    
    if(p_diag_log != NULL)
    {
        NRF_LOG_INFO("SwaraLink Peripheral Middleware - Diagnostic log dump:");

        // print out diagnostic log raw data
        for(uint16 i = 0; i < SWL_PERIPH_DIAGNOSTICS_LOG_BUFFER_SIZE; i++)
        {
            // print byte from diagnostic log
            NRF_LOG_INFO("index: %04d, data (hex): %02x", i, *p_diag_log);
        
            // increment pointer by 1 byte
            p_diag_log++;
        }
    }
    else
    {
        NRF_LOG_INFO("SwaraLink Peripheral Middleware - Diagnostic log not found!");
    }
}

void exception_handler(const swl_periph_exception_data_t * p_exception_data)
{
    // exception occurred in swl_periph library 
    
    if(p_exception_data != NULL)
    {    
        NRF_LOG_INFO("swl_periph exception occurred! Please take note of the following information and report to SwaraLink Technologies:");
        NRF_LOG_INFO("Exception filename code:      %u", p_exception_data->filename_code);
        NRF_LOG_INFO("Exception line number:        %u", p_exception_data->line_number);
        NRF_LOG_INFO("Exception metadata A:         %u", p_exception_data->metadata_a);
        NRF_LOG_INFO("Exception metadata B:         %u", p_exception_data->metadata_b);
    }

    // retrieve and print out diagnostic log raw data
    dump_diagnostic_log();

    // Trigger breakpoint if debugger is connected
    SWL_BKPT;

    // for in-field exceptions, it is recommend to soft reset the system as the swl_periph library may be in an unreliable state
    sd_nvic_SystemReset();

    // infinite loop; code should never reach here
    for(;;)
    {}
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{   
    swl_periph_nrf_process_idle_state();

    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    swl_err_t err_code;

    switch (pin_no)
    {
        case SWL_BUTTON_1:
        {
          //Check whether button_action has been pressed down
          if(button_action)
          {
            //Check whether we are in pairing mode.
            if(!in_pairing_mode)
            {
              NRF_LOG_INFO("Entering SWL Pairing Mode!!!");

              //We are not in pairing mode. Enter pairing mode.
              err_code = swl_periph_enter_pairing_mode(SWL_PAIRING_TIMEOUT_DEFAULT);
              SWL_ERROR_CHECK(err_code);

              in_pairing_mode = true;

              bsp_board_led_on(BSP_BOARD_LED_1);
            }

            else
            {
              NRF_LOG_INFO("Exiting SWL Pairing Mode!!!");

              //We are currently in pairing mode. Exit pairing mode.
              err_code = swl_periph_exit_pairing_mode();
              SWL_ERROR_CHECK(err_code);
            }
          }
        }
            break;

        case SWL_BUTTON_2:
          if(button_action)
          {
            NRF_LOG_INFO("Factory Reset!");

            err_code = swl_periph_perform_factory_reset();
            SWL_ERROR_CHECK(err_code);
          }
            break;
         
         case SWL_BUTTON_3:
          if(button_action)
          {
            NRF_LOG_INFO("SPM change!");

            switch(current_spm)
            {
              case SWL_PWR_MODE_SPM2C:
                current_spm = SWL_PWR_MODE_SPM4;
                break;

              case SWL_PWR_MODE_SPM4:
                current_spm = SWL_PWR_MODE_SPM2C;
                break;

              default:
                current_spm = SWL_PWR_MODE_SPM4;
                break;
            }
            
            //Update power mode.
            err_code = swl_periph_set_power_mode(current_spm);
            SWL_ERROR_CHECK(err_code);
          }
            break;

         case SWL_BUTTON_4:
          if(button_action)
          {
            //Check if connection has been established.
            if(ble_connection_established)
            {

              //Send a 4k file
              err_code = swl_periph_p2c_tx_by_ref(demofile_array, DEMOFILE_SIZE, ack_required, ID_HDR_MAJ_P2C_DEMO_RSP, ID_HDR_MIN_P2C_DEMO_RSP_TX_BY_REF_NO_ACK);
              SWL_ERROR_CHECK(err_code);
            }
          }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    SWL_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

//Optional. Used for demo purposes.
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}

//Optional. Function for initializing the button handler module.
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {BUTTON_1, false, BUTTON_PULL, button_event_handler},
        {BUTTON_2, false, BUTTON_PULL, button_event_handler},
        {BUTTON_3, false, BUTTON_PULL, button_event_handler},
        {BUTTON_4, false, BUTTON_PULL, button_event_handler}

    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}

static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


static void initialize_swl_periph_middleware(void)
{
    ret_code_t err_code;
    swl_err_t  swl_err_code;

    // Initialize app_timer module
    // The app_timer module is required by the SwaraLink Peripheral middleware. It can 
    // also be used by the application; however, the application must ensure that it
    // does not interfere with the operation of any timers that it did not create.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Initialize Nordic's crypto library
    // The nrf_crypto module is required by the SwaraLink Peripheral middleware. It can
    // also can be used by the application; however the application must not 
    // uninitialize it at any time.
    err_code = nrf_crypto_init();
    APP_ERROR_CHECK(err_code);

    // Configure buffers for the SwaraLink Peripheral middleware
    swl_err_code = swl_periph_config();
    SWL_ERROR_CHECK(swl_err_code);

    // Initialize SwaraLink peripheral module; register event handlers
    swl_err_code = swl_periph_init(periph_evt_handler, exception_handler);
    SWL_ERROR_CHECK(swl_err_code);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    log_init();
    leds_init();
    power_management_init();
    buttons_init();

    app_button_enable();

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

    initialize_swl_periph_middleware();

    // Start execution.
    NRF_LOG_INFO("SWL Periph Example started!");

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
