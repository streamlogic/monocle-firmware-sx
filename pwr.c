/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file pwr.c
 *	@brief Code for Power Management interface. MCU hardware dependent.
 *
 *	@author Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */

#include "pwr.h"
#include "nrf_power.h" // HAL layer, for nrf_power_resetreas_get(), _clear()
#include "nrf_log.h"
#include "sdk_config.h"

// ===== hardware dependent data =====



// ===== pwr configuration =====



// ===== internal =====

// RESETREAS register is documented in Product Specification here:
//   https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.nrf52832.ps.v1.1%2Fpower.html&anchor=register.RESETREAS
uint8_t* reason_string(uint32_t code)
{
    switch(code) {
        case 0: return "POR - Power On Reset or brownout";
        case 1: return "RESETPIN - reset pin";
        case 2: return "DOG - watchdog";
        case 4: return "SREQ - soft reset";
        case 8: return "LOCKUP - CPU lockup detected";
        case 0x10000: return "OFF - wake up from system off by GPIO";
        case 0x20000: return "LPCOMP - wake up from system off by LPCOMP";
        case 0x40000: return "DIF - wake up from system off by Debug Interface Mode";
        case 0x80000: return "NFC - wake up from system off by NFC field detect";
        default: return "UNKNOWN";
    }
}

// ===== public functions =====

/**@brief Function for initializing power management.
 */
void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

void power_log_reset_reason(void)
{
    uint32_t reason = 0;
    reason = nrf_power_resetreas_get();
    NRF_LOG_INFO("Reset reason: 0x%x = %s", reason, reason_string(reason));
    nrf_power_resetreas_clear(0xFFFFFFFF);
}