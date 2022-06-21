/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file mk11_board.h
 *	@brief Defines to support MK11 V1.0 board (& MK11 TEST V1.0)
 *
 *	@author Nathan Ashelman for Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */

#ifndef MK11_BOARD_H
#define MK11_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

// ========== Options for released builds ==========
// unless compiling test code, select one to build for
//#define BUILD_PCBA_TEST // factory use to flash & test PCBA without camera or OLED
#define BUILD_PRODUCTION // factory to flash for use with assembled monocle
// ========== END Options for released builds ==========



// ========== Options for testing ==========
// --- normally ON ---
// comment these lines out to disable configuration & use of the peripheral (for code testing)
#define CAMERA_ON
#define DISPLAY_ON
//#define MIC_ON
#define FPGA_ON
#define WATCHDOG_ON // enable watchdog timer
#define BLE_ON // enable Swaralink Bluetooth library (note: then any pause in debugger will crash)

// --- normally OFF = commented out ---
// state machine control
//#define IGNORE_BATT_LEVEL // for testing: never power off due to low battery or battery charging (use when running on 5V power)
// turn on to hold FPGA is a state ready for programming (then after programming turn off again to allow booting)
//#define FPGA_HOLD
// turn on to keep the OLED always on (useful for testing); requires DISPLAY_ON above
//#define DISPLAY_ALWAYS_ON
// turn on to simulate Bluetooth transfer, even if BLE_ON is turned off above
//#define BLE_TEST_MODE

// --- unit tests of FPGA, SPI, checksum calculation ---
// disable CAMERA_ON above & enable _ONE_ of these:
//#define TEST1     // just writes to LED CONTROL register, counting from 0x00 to 0x0F in a loop
                  // initial test result: Will work correctly for many cycles, but eventually the LED register will get stuck on a value, even though SPI writes continue.
                  //              After ~5 minutes, the OLED brightness once began to pulse (probably some refreshes are being missed, but others get through).
                  //              Eventually (after 10~20 minutes), the FPGA signals to the OLED will stop & it goes completely dark.
                  //              It was also once observed (after brightness pulsing for ~7 minutes) that LED count resumed, and OLED brightness stabilized,
                  //              but then about 4 mins later the LEDs stopped again.
                  // 2021-03-05 result: Counted to >15,000 cycles (twice), LEDs continue to count (quickly, 10ms delay), OLED color bars remain on
//#define TEST1b    // alternates reads from LED CONTROL & FPGA_DISPLAY_CONTROL registers, checking they have (their different) default values
                  // test result: would get errors on both, sometimes just a dozen, sometimes hundreds (usually observed to be in a row, every other count
                  //              meaning a given register was stuck on the wrong value
                  // 2021-03-05 result: >30,000 cycles (twice), no errors, OLED color bars remain on
//#define TEST2     // as in TEST1, + reads back LED register value; if different from expected, MCU logs an error message 
                  // test result: Fails after some time (may be 1st cycle, may be >200), with LEDs becoming stuck & error logging messages by MCU;
                  //              OLED goes off immediately on failure.
                  // 2021-03-05 result: Counted to >15,000 cycles (twice), LEDs continue to count (quickly, 10ms delay), OLED color bars remain on
//#define TEST3     // exercises SPI write & read to FPGA Memory Control register, by writing values from 0x00 to 0xFF & back down, and reading to check result; on failure, stops SPI write/read
                  // test result: fails during count up, sometimes OLED will go off, sometimes it stays on (probably indefinitely, since SPI activity is stopped by MCU)
                  // 2021-03-05 result: >4,000 cycles = 2M write/reads (twice), no errors, OLED good
//#define TEST4     // tests SPI write/read to the OLED
                  // test result: will PASS if the FPGA is not programmed (all pins in input, weak pull-high state)
                  //              when FPGA _is_ programmed, will usually pass, but rarely it will fail in the OLED SPI write/read
                  //                                         and rarely will pass one cycle with SPI but the OLED color bars will go orange/green
//#define TEST5a     // tests SPI burst read from the OLED, outputs to log
//#define TEST5b     // FPGA burst write/read: W:36*7B+2B, R:1*254B [REQUIRES FPGA to enable test FIFO]
//#define TEST5c     // FPGA burst write/read: W:146*7B+2B(=1024B), R:2*(1*252B, 1*120B)+1*252B+1*28B [REQUIRES FPGA to enable test FIFO]
//#define TEST6     // test of fpga_checksum() function
//(TODO checksum over SPI, see project ___)
// tests for Bluetooth -- transmit known image (single frame) over Bluetooth instead of actually captured data
//#define TEST_BLE_DATA BLE_DATA_CAPT_BARS
//#define TEST_FLICKER_RB_SHIFT

// ========== END Options for testing ==========



// ========== Hardware definitions ==========

// --- board definition, used in ov5640 code ---
// see board.h file
// selected hardware to build for: UPDATE THIS for your target!
#define MK_HW_DEVICE     MK_HW_MK11_V1_0
#define MK_HW_TEXT       "MK11 V1.0"

// --- touch sensor ---
#define CY8CMBR3 1
#define IQS620   2
// define TOUCH_CAPSENSE as one of above to set CapSense IC
#define TOUCH_CAPSENSE IQS620

// --- camera ---
// define this if the camera module has an auto-focus motor (posssible in MK9B, not in MK10, MK11)
//#define AUTO_FOCUS

// -- LEDs ---
// no MCU controlled LEDs on MK11

// --- Recovery pin ---
// hold this pin to GND on boot to enter busy loop & guarantee debugger/programming accesss
#define IO_RECOVERY_PIN          17   // has pull-up resistor; after boot redefined as SCL

// --- Output IO pin definitions ---
//#define IO_DISP_VCAL_PIN         XX   // P0.XX, DISP_VCAL -- NOT connected ON MK11
//#define IO_EN_PWR                XX   // P0.XX, en_pwr -- NOT ON MK11
#define IO_N_CAM_RESET           20   // P0.20, cam_RESET_N (n_cam_RESET) = CAM_RST
#define IO_CAM_PWDN              29   // P0.29, cam_PWDN = CAM_PWRDN
#define IO_DISP_XCLR             15   // P0.15, disp_XCLR = DISP_XCLR

#define EN_PWR_DELAY              3   // ms delay until power good; MK9B measured: 1.8V=2.8ms, TODO others?

// --- Input IO pin definitions ---
#include "nrf_gpio.h"
// configured in touch.c
#define IO_TOUCHED_PIN           2    // P0.2 = touched_N on MK9B/MK10, TOUCH_RDY on MK11
#define IO_TOUCHED_PIN_ACTIVE    0    // active low NOTE NOT USED
#define IO_TOUCHED_PIN_PULL      NRF_GPIO_PIN_PULLUP // NOTE maybe hard code this?
// confifured in fpga.c (not yet used)
#define IO_FPGA_INT_PIN          5    // P0.5 = fpga_int_N
#define IO_FPGA_INT_PIN_ACTIVE   0    // active low
#define IO_FPGA_INT_PIN_PULL     NRF_GPIO_PIN_PULLUP
// configured in adc.c
//TODO in MK9B/MK10 goes to resistor divider. in MK11 goes to PMIC AMUX pin
//WARNING need code changes to work on MK11
#define IO_ADC_VBATT             NRF_SAADC_INPUT_AIN1    // P0.03/AIN1 = vbatt_meas

// --- SPI related ---
#define SPIM_SCK_PIN             7  // SPI clock GPIO pin number.
#define SPIM_MOSI_PIN            9  // SPI Master Out Slave In GPIO pin number.
#define SPIM_MISO_PIN           10  // SPI Master In Slave Out GPIO pin number.
#define SPIM_SS0_PIN             6  // SPI Slave Select 0 GPIO pin number, for display (spi_disp_CS)
#define SPIM_SS1_PIN             8  // SPI Slave Select 1 GPIO pin number, for FPGA (spi_fpga_CS)
#define SPIM_SS2_PIN             4  // SPI Slave Select 2, for external flash (SPI_FLASH_CS)

#define SPI_INSTANCE             2  // SPI instance index (SPIM0 and SPIM1 driver memory conflicts with TWI0 and TWI1)

// --- I2C related ---
#define TWI_SCL_PIN             17  // I2C clock GPIO pin
#define TWI_SDA_PIN             13  // I2C data GPIO pin

#define TWI_INSTANCE_ID          0  // TWI instance ID (main I2C bus)

// - Dual I2C -
#define TWI_SW_INSTANCE_ID       1  // TWI instance ID (second I2C bus, if used)
#define TWI_SW_SCL_PIN          18  // SW_I2C_SCL
#define TWI_SW_SDA_PIN          16  // SW_I2C_SDA pin number

#define OV5640_ADDR              0X3C // OV5640 address on I2C bus

#define CY8CMBR3_ADDR            0X37 // Cypress touch sensor I2C address, set in config
#define CY8CMBR3_BOOT_DELAY      1000 // ms to wait before attempt to initialize
#define CY8CMBR3_GPIO_LED        0x01 // GP0 connected to PROXIMITY LED on dev brd

#define IQS620_ADDR              0x44 // Azotek IQS620A touch sensor I2C address

//      MAX77654_ADDR            0x48 is defined in max77654.c

//NOTE
// pins not yet used: PMIC_IRQ=P0.14; P0.21=RESET_L; SPI_FLASH_CS=P0.04

// ========== END Hardware definitions ==========



// ========== Sanity checks ==========
#if ((defined CAMERA_ON || defined DISPLAY_ON || defined MIC_ON) && !(defined FPGA_ON))
    #error "Camera, Display & Mic depend on FPGA."
#endif

//NOTE: MIC currently untested/unsupported; turn on later
//#if ((defined BUILD_PRODUCTION) && !( (defined CAMERA_ON) && (defined DISPLAY_ON) && (defined MIC_ON) && (defined FPGA_ON) ))
//    #error "Production build requires: CAMERA_ON, DISPLAY_ON,  MIC_ON."
#if ((defined BUILD_PRODUCTION) && !( (defined CAMERA_ON) && (defined DISPLAY_ON) && (defined FPGA_ON) ))
    #error "Production build requires: CAMERA_ON, DISPLAY_ON."
#elif ((defined BUILD_PRODUCTION) && !( (defined TOUCH_CAPSENSE) && (defined WATCHDOG_ON) && (defined BLE_ON) ))
    #error "Production build requires: TOUCH_CAPSENSE, WATCHDOG_ON, BLE_ON."
#elif ((defined BUILD_PRODUCTION) && (defined IGNORE_BATT_LEVEL))
    #error "Production build must disable: IGNORE_BATT_LEVEL"
#elif ((defined BUILD_PRODUCTION) && (defined DEBUG))
    #error "Production build must be in Release configuration, not Debug."
#endif

#if ((defined BUILD_PCBA_TEST) && !(defined FPGA_ON && defined TOUCH_CAPSENSE))
    #error "PCBA-Test build requires: FPGA_ON, TOUCH_CAPSENSE"
#elif ((defined BUILD_PCBA_TEST) && !( (defined IGNORE_BATT_LEVEL) ))
    #error "PCBA-Test build requires: IGNORE_BATT_LEVEL"
#elif ((defined BUILD_PCBA_TEST) && (defined CAMERA_ON || defined DISPLAY_ON || defined WATCHDOG_ON))
    #error "PCBA-Test build must disable: CAMERA_ON, DISPLAY_ON, WATCHDOG_ON"
// currently don't care, may check in future: MIC_ON, BLE_ON, DEBUG
#endif

// ========== END Sanity checks ==========

#ifdef __cplusplus
}
#endif

#endif // MK11_BOARD_H
