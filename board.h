/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file board.h
 *	@brief Function prototypes for Board Support Package. MCU hardware dependent.
 *
 *	@author Nathan Ashelman for Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */
#ifndef BOARD_H
#define BOARD_H

#include "nrf_gpio.h"
#include "nordic_common.h"

// ===== MK series hardware definitions, used in ov5640 code =====
#define MK_HW_UNDEFINED   0
#define MK_HW_DEV_BRD     1
#define MK_HW_MK3B_V0_0   2
#define MK_HW_6_TEST_V0_0 3
#define MK_HW_MK6_V0_0    4
#define MK_HW_MK8_V0_3    5   // MK8 V0.3 assembled July 2020
#define MK_HW_MK9_V1_1    6   // MK9 V1.1 assembled November 2020
// Nordic MCU boards start here
#define MK_HW_MK9B_V1_1   7   // MK9B V1.1 assembled December 2020 with nRF52810 MCU (all boards converted to V1.2)
#define MK_HW_MK9B_V1_2   8   // MK9B V1.2 modified February 2021 to upgrade MCU to nRF52832
#define MK_HW_MK10_V2_2   9   // MK10 assembled January 2021
#define MK_HW_MK11_V1_0  10   // MK11 and MK11 TEST assembled October 2021
// selected hardware to build for: UPDATE THIS for your target!
// use #define MK_HW_DEVICE in the xxx_board.h file


#if defined(BOARD_MK9B)
  #include "mk9b_board.h"
#elif defined(BOARD_MK10)
  #include "mk10_board.h"
#elif defined(BOARD_MK11)
  #include "mk11_board.h"
#elif defined(BOARD_CUSTOM)
  #include "custom_board.h"
  #error "This is the old config file."
#else
  #error "Board is not defined"
#endif

#ifndef MK_HW_DEVICE
#error "Hardware target must be specified."
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if LEDS_NUMBER > 0
/**
 * Function for returning the state of an LED.
 *
 * @param led_idx LED index (starting from 0), as defined in the board-specific header.
 *
 * @return True if the LED is turned on.
 */
bool bsp_board_led_state_get(uint32_t led_idx);

/**
 * Function for turning on an LED.
 *
 * @param led_idx LED index (starting from 0), as defined in the board-specific header.
 */
void bsp_board_led_on(uint32_t led_idx);

/**
 * Function for turning off an LED.
 *
 * @param led_idx LED index (starting from 0), as defined in the board-specific header.
 */
void bsp_board_led_off(uint32_t led_idx);

/**
 * Function for inverting the state of an LED.
 *
 * @param led_idx LED index (starting from 0), as defined in the board-specific header.
 */
void bsp_board_led_invert(uint32_t led_idx);
/**
 * Function for turning off all LEDs.
 */
void bsp_board_leds_off(void);

/**
 * Function for turning on all LEDs.
 */
void bsp_board_leds_on(void);

/**
 * Function for converting pin number to LED index.
 *
 * @param pin_number Pin number.
 *
 * @return LED index of the given pin or 0xFFFFFFFF if invalid pin provided.
 */
uint32_t bsp_board_pin_to_led_idx(uint32_t pin_number);

/**
 * Function for converting LED index to pin number.
 *
 * @param led_idx LED index.
 *
 * @return Pin number.
 */
uint32_t bsp_board_led_idx_to_pin(uint32_t led_idx);
#endif //LEDS_NUMBER > 0

/**
 * Function for initializing the BSP handling for the board.
 *
 * @param[in]  init_flags  Flags specifying what to initialize (LEDs/buttons).
 *                         See @ref BSP_BOARD_INIT_FLAGS.
 */
void bsp_board_init(void);
void bsp_board_uninit(void);

void bsp_board_check_recovery(void);
void bsp_board_io_off(uint8_t io_pin);
void bsp_board_io_on(uint8_t io_pin);
void bsp_board_aux_power_on(void); // only for MK9B, MK10
void bsp_board_aux_power_off(void); // only for MK9B, MK10

#ifdef __cplusplus
}
#endif

#endif // BOARD_H
