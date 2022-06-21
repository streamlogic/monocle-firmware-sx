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

#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h> // for memset()
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "board.h"

//NOTE can support one or two I2C busses, per mkXX_board.h file

void i2c_init(void);
void i2c_uninit(void);
bool i2c_write(uint8_t slaveAddress, uint8_t *writeBuffer, uint8_t numberOfBytes);
bool i2c_write_no_stop(uint8_t slaveAddress, uint8_t *writeBuffer, uint8_t numberOfBytes);
bool i2c_read(uint8_t slaveAddress, uint8_t *readBuffer, uint8_t numberOfBytes);

#ifdef TWI_SW_INSTANCE_ID
void i2c_sw_init(void);
void i2c_sw_uninit(void);
bool i2c_sw_write(uint8_t slaveAddress, uint8_t *writeBuffer, uint8_t numberOfBytes);
bool i2c_sw_read(uint8_t slaveAddress, uint8_t *readBuffer, uint8_t numberOfBytes);
#endif

#endif // I2C_H
