/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file led.h
 *	@brief Function prototypes for LED interface. Abstracts differences between hardware versions.
 *
 *	@author Nathan Ashelman for Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */

#ifndef LED_H
#define LED_H

#include "board.h"

void led_init(void); // NOTE app_timer_init() must have been called prior to calling this
void led_red_on(void);
void led_red_off(void);
void led_green_on(void);
void led_green_off(void);
void led_green_blink_now(uint8_t count); // blocking call, blinks count times
void led_green_blink_timer(uint8_t count, bool repeat); // non-blocking; requires prior call to led_init()

#endif // LED_H
