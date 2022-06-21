/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file i2c.h
 *	@brief Function prototypes for I2C interface. MCU hardware dependent.
 *
 *	@author Nathan Ashelman for Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */

#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include "nrf_drv_saadc.h"
#include "board.h"

// ===== Debug Logging for this module =====
//#define ADC_LOG_INFO_ON
//#define ADC_LOG_DEBUG_ON

// For use in early initialization, single-shot ADC to check voltage value; should de-initialize before using adc_init() in main code
void adc_quick_init(void);
float adc_quick_get_batt_voltage(void);
void adc_quick_uninit(void);

/* Requires: adc_start_sample() must be called periodically within the main application
 * every 5 samples will be averaged, and the battery voltage & SoC updated
 */
void adc_init(void);
void adc_uninit(void);
//void adc_calibrate(void);
void adc_start_sample(void);
float adc_get_batt_voltage(void);
uint8_t adc_get_batt_soc(void);

#endif // ADC_H
