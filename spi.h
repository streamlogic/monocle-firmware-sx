/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file spi.h
 *	@brief Function prototypes for dual SPI interface. MCU hardware dependent.
 *
 *	@author Nathan Ashelman for Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */

#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h> // for memset()
#include "nrfx_spim.h"
#include "nrf_delay.h"
#include "board.h" // TODO: this is maybe not right; is required to work here, but broken when used in hrs project

#define SPI_MAX_BURST_LENGTH 254 // maximum length of burst write or read (Bytes)
#if(SPI_MAX_BURST_LENGTH >= UINT16_MAX)
#error "Reduce SPI_MAX_BURST_LENGTH or redefine length type."
#elif(SPI_MAX_BURST_LENGTH > 1024)
#error "Current FPGA FIFO limit is 1024 bytes."
#endif

void spi_init(void);
void spi_uninit(void);
void spi_set_cs_pin(uint8_t cs_pin);
void spi_write_byte(uint8_t addr, uint8_t data);
void spi_write_burst(uint8_t addr, const uint8_t *data, uint16_t length);
uint8_t spi_read_byte(uint8_t addr);
uint8_t *spi_read_burst(uint8_t addr, uint16_t length);

// for unit testing
bool spi_exercise_register(uint8_t addr);

#endif // SPI_H
