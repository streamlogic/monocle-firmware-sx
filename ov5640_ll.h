/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file ov5640_ll.h
 *	@brief Low-level, hardware-specific functions to interface OV5640 to I2C and GPIO pins.
 *
 *	@author Nathan Ashelman for Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */

#ifndef _OV5640_LL_H
#define _OV5640_LL_H

#include "board.h"

void ov5640_ll_RST(uint8_t n);
void ov5640_ll_PWDN(uint8_t n);
void ov5640_ll_2V8EN(uint8_t n);

void ov5640_ll_delay_ms(uint32_t ms);
uint8_t ov5640_ll_init(void);
uint8_t OV5640_WR_Reg(uint16_t reg, uint8_t data);
uint8_t OV5640_RD_Reg(uint16_t reg);

#define TRANSFER_CMPLT (0x00u)
#define TRANSFER_ERROR (0x01u)
#define I2C_SLAVE_ADDR OV5640_ADDR

#endif	// _OV5640_LL_H
