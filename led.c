/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file led.c
 *	@brief Code for LED interface.
 *
 *	@author Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug none
 */

#include "led.h"
#include "nrf_log.h" //NOTE testing
#include "nrf_assert.h"
#include "app_timer.h"
#include "nrf_delay.h"

// blink timing
#define LED_TIME_ON      100 // ms on
#define LED_TIME_OFF     400 // ms off
#define LED_TIME_PAUSE  2000 // time to stay off between cycles of blinks

// on MK9B & MK10, LEDs are connected to FPGA
#if ((defined BOARD_MK9B) || (defined BOARD_MK10))
    #if !(defined FPGA_ON)
        #define LEDS_SUPPORTED (0)
        #define LED_GREEN_ON  {}
        #define LED_GREEN_OFF {}
        #define LED_RED_ON    {}
        #define LED_RED_OFF   {}
    #else
        #define LEDS_SUPPORTED (2)
        #define LED_GREEN (0) // red on MK9B, green on MK10
        #define LED_RED (1) // red on both
        #include "fpga.h" // interface to FPGA controlled LEDs
        #define LED_GREEN_ON  {fpga_led_on(LED_GREEN);}
        #define LED_GREEN_OFF {fpga_led_off(LED_GREEN);}
        #define LED_RED_ON    {fpga_led_on(LED_RED);}
        #define LED_RED_OFF   {fpga_led_off(LED_RED);}
    #endif
#elif (defined BOARD_MK11)
    #define LEDS_SUPPORTED (2)
    #include "max77654.h"
    #define LED_GREEN_ON  {max77654_led_green_on(true);}
    #define LED_GREEN_OFF {max77654_led_green_on(false);}
    #define LED_RED_ON    {max77654_led_red_on(true);}
    #define LED_RED_OFF   {max77654_led_red_on(false);}
#else
    #error "LED: board not defined"
#endif

APP_TIMER_DEF(m_led_timer_id);                                  /**< LED timer; repeated. */
bool    m_led_repeat = false;
uint8_t m_led_count  = 0;
bool    m_led_on     = false;
uint8_t m_led_blink  = 0;

static void led_timer_start(uint32_t ms)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_led_timer_id, APP_TIMER_TICKS(ms), NULL);
    APP_ERROR_CHECK(err_code);
}

static void led_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    if(m_led_on)
    {
        LED_GREEN_OFF;
        m_led_on = false;
        led_timer_start(LED_TIME_OFF);
    }
    else // LED off
    {
        m_led_blink++; // a blink completed
        if(m_led_blink < m_led_count) // still need to blink
        {
            LED_GREEN_ON;
            m_led_on = true;
            led_timer_start(LED_TIME_ON);
        }
        else if(m_led_blink == m_led_count) // that was last blink
        {
            if(m_led_repeat) // keep off for PAUSE time before restarting blink cycle
            {
                led_timer_start(LED_TIME_PAUSE);
            }
            else // no more timer, we are done
            {
                m_led_blink = 0; // reset for next time
            }
        }
        else // (m_led_blink > m_led_count); finished the PAUSE
        {
            ASSERT(m_led_repeat); // should only happen if in repeat mode
            m_led_blink = 0; // restart next cycle
            LED_GREEN_ON;
            m_led_on = true;
            led_timer_start(LED_TIME_ON);
        }
    }
}

void led_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    // NOTE app_timer_init() must have been called prior to calling this

    // Create timer
    err_code = app_timer_create(&m_led_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                led_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

void led_red_on(void)
{
#if LEDS_SUPPORTED > 0
    LED_RED_ON;
#endif
}

void led_red_off(void)
{
#if LEDS_SUPPORTED > 0
    LED_RED_OFF;
#endif
}

void led_green_on(void)
{
#if LEDS_SUPPORTED > 0
    LED_GREEN_ON;
#endif
}

void led_green_off(void)
{
#if LEDS_SUPPORTED > 0
    LED_GREEN_OFF;
#endif
}

//NOTE: this blocks for 150ms*times while waiting for completion; careful in use (eg interrupt context)
void led_green_blink_now(uint8_t count)
{
#if LEDS_SUPPORTED > 0
    for(uint8_t i=0; i<count; i++)
    {
        LED_GREEN_ON;
        nrf_delay_ms(LED_TIME_ON);
        LED_GREEN_OFF;
        nrf_delay_ms(LED_TIME_OFF);
    }
#endif
}

// non-blocking call, requires led_init() to have been called
void led_green_blink_timer(uint8_t count, bool repeat)
{
#if LEDS_SUPPORTED > 0
    LED_GREEN_ON;
    m_led_on = true;
    m_led_count = count;
    m_led_repeat = repeat;
    led_timer_start(LED_TIME_ON);
    // led_timeout_handler() takes it from here
#endif
}
