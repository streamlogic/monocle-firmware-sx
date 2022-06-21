/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file custom_board.h
 *	@brief Defines to support MK9B V1.2 board
 *
 *	@author Nathan Ashelman for Brilliant Labs Limited (Hong Kong)
 *
 *	@bug No known bugs.
 */

#ifndef EBSLSN_H
#define EBSLSN_H

#ifdef __cplusplus
extern "C" {
#endif

// ===== MK series hardware definitions, used in ov5640 code =====
#define MK_HW_UNDEFINED   0
#define MK_HW_DEV_BRD     1
#define MK_HW_MK3B_V0_0   2
#define MK_HW_6_TEST_V0_0 3
#define MK_HW_MK6_V0_0    4
#define MK_HW_MK8_V0_3    5   // MK8 V0.3 assembled July 2020
#define MK_HW_MK9_V1_1    6   // MK9 V1.1 assembled November 2020
// Nordic MCU boards start here
#define MK_HW_MK9B_V1_1   7   // MK9B V1.1 assembled December 2020 with nRF52810 MCU
#define MK_HW_MK9B_V1_2   8   // MK9B V1.2 modified February 2020 to upgrade MCU to nRF52832
#define MK_HW_MK10_V2_1   9   // MK10 assembled January 2021
// selected hardware to build for: UPDATE THIS for your target!
#define MK_HW_DEVICE     MK_HW_MK9B_V1_2
#ifndef MK_HW_DEVICE
#error "Hardware target must be specified."
#endif

// ===== Options for testing =====
// define this if the CapSense dev board is connected to MK9B
// otherwise, the on-board button S1 can be used as a substitute
//#define TOUCH_CAPSENSE
// comment thsese lines out to disable configuration & use of the peripheral
#define CAMERA_ON
#define DISPLAY_ON
#define FPGA_ON
#if (defined CAMERA_ON && !(defined FPGA_ON)) || (defined DISPLAY_ON && !(defined FPGA_ON))
  #error "Camera & Display depend on FPGA."
#endif
#define IGNORE_BATT_LEVEL // for testing: never power off due to low battery or battery charging (use when running on 5V power)
#define IGNORE_STANDBY_TIMEOUT // for testing: never power down from STANDBY state
#define VCAL_AS_LED // use disp_VCAL pin as an output to control LED (requires jumper wire on MK9B)

// unit testing of SPI & FPGA
//#define TEST1     // TODO: import from earlier project
//#define TEST2     // TODO: import from earlier project
//#define TEST3     // TODO: import from earlier project
//#define TEST4     // tests SPI write/read to the OLED
//#define TEST5     // tests SPI burst read from the OLED (TODO & burst write to OLED & FPGA burst read/write)
//#define TEST6     // test of fpga_checksum() function (TODO checksum over SPI, see project ___)

#include "nrf_gpio.h"

// ===== LEDs definitions =====
#ifdef VCAL_AS_LED
#define LEDS_NUMBER    1

#define LED_START       3
#define LED_1           3   // P0.03, disp_VCAL (repurpose as LED, jumper to J8)
#define LED_STOP        3

#define LEDS_ACTIVE_STATE 0 // active low: 0V = ON, 1.8V = OFF

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1 }

#define BSP_LED_0      LED_1
#endif

// ===== Recovery pin =====
// hold this pin to GND on boot to enter busy loop & guarantee debugger/programming accesss
#define IO_RECOVERY_PIN           7   // has pull-up resistor; after boot redefined as SCL

// ===== Output IO pin definitions =====
#define IO_EN_PWR                20   // P0.20, en_pwr
#define IO_N_CAM_RESET           00   // P0.00, n_cam_RESET
#define IO_CAM_PWDN              01   // P0.01, cam_PWDN
#define IO_DISP_XCLR             18   // P0.18, disp_XCLR

#define EN_PWR_DELAY              3   // ms delay until power good; MK9B measured: 1.8V=2.8ms, TODO others?

// ===== Input IO pin definitions =====
// configured in touch.c
#define IO_TOUCHED_PIN           21   // P0.21 = touched_N
#define IO_TOUCHED_PIN_ACTIVE    0    // active low
#define IO_TOUCHED_PIN_PULL      NRF_GPIO_PIN_PULLUP
// confifured in fpga.c
#define IO_FPGA_INT_PIN          6    // P0.6 = fpga_int_N
#define IO_FPGA_INT_PIN_ACTIVE   0    // active low
#define IO_FPGA_INT_PIN_PULL     NRF_GPIO_PIN_PULLUP
// configured in adc.c
#define IO_ADC_VBATT             NRF_SAADC_INPUT_AIN0    // P0.02/AIN0 = vbatt_meas

#ifndef VCAL_AS_LED
#error "VCAL ADC input not yet implemented."
#endif

// ===== SPI related =====
#define SPIM0_SCK_PIN    9  // SPI clock GPIO pin number.
#define SPIM0_MOSI_PIN  10  // SPI Master Out Slave In GPIO pin number.
#define SPIM0_MISO_PIN  28  // SPI Master In Slave Out GPIO pin number.
#define SPIM0_SS0_PIN    8  // SPI Slave Select 0 GPIO pin number, for display (spi_disp_CS)
#define SPIM0_SS1_PIN    4  // SPI Slave Select 1 GPIO pin number, for FPGA (spi_fpga_CS)

#define SPI_INSTANCE     0  // SPI instance index

// ===== I2C related =====
#define TWI_SCL_PIN      7  // I2C clock GPIO pin
#define TWI_SDA_PIN      5  // I2C data GPIO pin

#define TWI_INSTANCE_ID  1  // TWI instance ID

#define OV5640_ADDR     0X3C // OV5640 address on I2C bus

#define CY8CMBR3_ADDR   0X37 // touch sensor I2C address, set in config
#define CY8CMBR3_BOOT_DELAY 1000 // ms to wait before attempt to initialize
#define CY8CMBR3_GPIO_LED   0x01 // GP0 connected to PROXIMITY LED on dev brd



#ifdef __cplusplus
}
#endif

#endif // EBSLSN_H
