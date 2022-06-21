/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file max77654.h
 *	@brief Function prototypes for MAX77654 PMIC interface. MCU hardware dependent (I2C & logging).
 *
 *	@author Nathan Ashelman for Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */

// Code written from scratch based on datasheet

#ifndef MAX77654_H
#define MAX77654_H

#include <stdbool.h>
#include <stdint.h>

// ===== Debug Logging for this module =====
#define MAX77654_LOG_INFO_ON
//#define MAX77654_LOG_DEBUG_ON

// return values of max77654_charging_status()
typedef enum MAX77654_charging_status {
    MAX77654_READ_ERROR = -1,// I2C error on read of status register
    MAX77654_READY = 0,      // ready
    MAX77654_CHARGING,       // charging
    MAX77654_CHARGE_DONE,    // charge done
    MAX77654_FAULT,          // fault
} max77654_status;

typedef enum MAX77654_charge_fault {
    MAX77654_NORMAL = 0,     // no faults
    MAX77654_FAULT_PRE_Q,    // Prequalification timer fault
    MAX77654_FAULT_TIME,     // Fast-charge timer fault
    MAX77654_FAULT_TEMP,     // Battery temperature fault
} max77654_fault;

bool max77654_init(void);
bool max77654_rail_1v8sw_on(bool on);
bool max77654_rail_2v7_on(bool on);
bool max77654_rail_1v2_on(bool on);
bool max77654_rail_10v_on(bool on);
bool max77654_rail_vled_on(bool on);
bool max77654_led_red_on(bool on);
bool max77654_led_green_on(bool on);
max77654_status max77654_charging_status(void);
max77654_fault max77654_faults_status(void);
bool max77654_set_charge_current(uint16_t current);
bool max77654_set_charge_voltage(uint16_t voltage);
bool max77654_set_current_limit(uint16_t current);
bool max77564_factory_ship_mode(void);

#endif // MAX77654_H
