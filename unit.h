/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file unit.h
 *	@brief Function prototypes for unit test subroutines.
 *
 *	@author Nathan Ashelman for Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */

#ifndef UNIT_H
#define UNIT_H

#include "board.h"
#include "sdk_config.h"
#include <stdint.h>

// See mkxx_board.h for list of unit tests & brief description of each
#if defined TEST1 || defined TEST1b || defined TEST2 || defined TEST3 || defined TEST4 || defined TEST5a || defined TEST5b || defined TEST5c || defined TEST6

//tests 1, 2, 3 integrated from FPGA-Test-Live_Video project
bool unit_test_1(uint8_t num_leds, uint32_t counts);
bool unit_test_1b(uint32_t counts);
bool unit_test_2(uint8_t num_leds, uint32_t counts);
bool unit_test_3(uint32_t counts);

bool unit_test_4(uint32_t counts);  // OLED exercise of SPI
bool unit_test_5a(void);  // OLED burst read
bool unit_test_5b(uint32_t counts);  // FPGA burst read/write test (integrated from FPGA-Test-SPI_burst)
bool unit_test_5c(uint32_t counts, uint16_t read_burst_length_long, uint16_t read_burst_length_short);  // FPGA burst read/write test
bool unit_test_6a(void); // checksum calculation unit tests

#endif // TEST1, ...

#if defined TEST_BLE_DATA
#define BLE_DATA_CAPT_BLACK       1 // single frame image capture, all black
#define BLE_DATA_CAPT_WHITE       2 // single frame image capture, all white
#define BLE_DATA_CAPT_BARS        3 // single frame image capture, color bars
#define BLE_DATA_CAPT_CROSS_WHITE 4 // single frame image capture, white cross on black background
#define BLE_DATA_CAPT_CROSS_RED   5 // single frame image capture, white cross on black background
#define BLE_DATA_CAPT_CROSS_BLUE  6 // single frame image capture, white cross on black background
#define BLE_DATA_CAPT_CROSS_GREEN 7 // single frame image capture, white cross on black background

uint32_t test_get_capture_size(void);
uint8_t *test_read_burst(uint16_t length);

#endif

#endif // UNIT_H
