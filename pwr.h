/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file pwr.h
 *	@brief Function prototypes for Power Management interface. MCU hardware dependent.
 *
 *	@author Nathan Ashelman for Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */

#ifndef PWR_H
#define PWR_H

#include "board.h"
#include "nrf.h" // for NRF_POWER->DCDCEN
#include "nrf_pwr_mgmt.h"

void power_management_init(void);
void power_log_reset_reason(void);

#endif // PWR_H
