/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file touch.c
 *	@brief Function implementation for touch interface.
 *
 *	@author Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */

#include "touch.h"

#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "app_timer.h"

#ifdef TOUCH_CAPSENSE
#define TOUCH_NUM_SENSORS (2) // number of touch sensors configured
#include "i2c.h" //NOTE TESTING

#if (TOUCH_CAPSENSE == CY8CMBR3)
#include "cy8cmbr3.h"
#elif (TOUCH_CAPSENSE == IQS620)
#include "iqs620.h"
#else
#error "Touch sensor type is not defined"
#endif // TOUCH_CAPSENSE == XXX

#else // no TOUCH_CAPSENSE, on MK9B use button S1 to simulate HI signal; can't distinguish between multiple buttons
#define TOUCH_NUM_SENSORS (1)
#include "app_button.h" // for debounce of push-button switch on MK9B
#endif // TOUCH_CAPSENSE

/*
 * This state machine can distinguish between the following gestures: 
 * Tap: push & quick release
 * Slide LR or RL +: tap on one button followed by tap on other
 * DoubleTap: Tap, followed quickly by another Tap
 * Press: push one for >0.5s & <10s then release
 * LongPress: push for >10s then release
 * LongBoth +: push both buttons for >10s then release
 * (+ these 3 gestures require TOUCH_CAPSENSE to be enabled)
 *
 * Transition to new state is triggered by a timeout or push/release event.
 * Timer, of given duration, is started when entering state that has a timeout.
 * Transitions back to IDLE will generate a gesture, this happens on release
 * for most gestures, but after TAP_INTERVAL for Tap (i.e. some delay).
 *
 * State machine transitions:
 *
 * -----------------------------------------------------------------------------
 * | current state   | event/timeout | new state      | gesture    | _CAPSENSE |
 * |---------------------------------------------------------------------------|
 * | IDLE            | push[one]     | TRIGGERED      |            |           |
 * | IDLE            | push[both]    | BOTH_PRESSED   |            | *                  
 * | TRIGGERED       | 0.5s          | PRESSED        |            |           |
 * | TRIGGERED       | release       | TAPPED         |            |           |
 * | TRIGGERED       | push[other]   | BOTH_PRESSED   |            | *         |
 * | TAPPED          | 0.25s         | IDLE           | Tap        |           |
 * | TAPPED          | push[same]    | TAPPED2        |            |           |
 * | TAPPED          | push[other]   | SLID           |            | *         |
 * | TAPPED2         | (infinite)    | (no change)    |            |           |
 * | TAPPED2         | release       | IDLE           | DoubleTap  |           |
 * | SLID            | (infinite)    | (no change)    |            | *         |
 * | SLID            | release[LR]   | IDLE           | SlideLR    | *         |
 * | SLID            | release[RL]   | IDLE           | SlideRL    | *         |
 * | PRESSED         | 9.5s          | LONG           |            |           |
 * | PRESSED         | release       | IDLE           | Press      |           |
 * | PRESSED         | push[other]   | BOTH_PRESSED   |            | *         |
 * | LONG            | (infinite)    | (no change)    |            |           |
 * | LONG            | release       | IDLE           | LongPress  |           |
 * | BOTH_PRESSED    | 9.5s          | BOTH_LONG      |            | *         |
 * | BOTH_PRESSED    | release[one]  | (no change)+   |            | *         | + and stop timer
 * | BOTH_PRESSED    | release[all]  | IDLE           | PressBoth  | *         |
 * | BOTH_LONG       | (infinite)    | (no change)    |            | *         |
 * | BOTH_LONG       | release[one]  | (no change)    |            | *         |
 * | BOTH_LONG       | release[all]  | IDLE           | LongBoth   | *         |
 * -----------------------------------------------------------------------------
 *
 * See diagram at: https://drive.google.com/file/d/1nErJZ_vvQBIfS90sQ9oX6CoDodD6m3_0/view?usp=sharing
 *
 */

#define TOUCH_TAP_INTERVAL     APP_TIMER_TICKS(250)   /**< Timeout for button tap (ticks) = 0.25 second */
#define TOUCH_PRESS_INTERVAL   APP_TIMER_TICKS(500)   /**< Timeout for button press (ticks) = 0.5 second */
#define TOUCH_LONG_INTERVAL    APP_TIMER_TICKS(9500)  /**< Timeout for long button press (ticks) = 9.5 seconds + PRESS_INTERVAL = 10 sec */

typedef enum {
#ifdef TOUCH_CAPSENSE
    TOUCH_SLID,
    TOUCH_BOTH_PRESSED,
    TOUCH_BOTH_LONG,
#endif
    TOUCH_IDLE,
    TOUCH_TRIGGERED,
    TOUCH_TAPPED,
    TOUCH_TAPPED2,
    TOUCH_PRESSED,
    TOUCH_LONG
} touch_state_t;

// ===== private data =====

static touch_state_t touch_state = TOUCH_IDLE; // current state machine state
static touch_gesture_handler_t touch_gesture_handler = NULL; // gesture handler registered by init()
APP_TIMER_DEF(delay_timer_id);  /**< state machine timer id. */

#ifdef TOUCH_CAPSENSE
static uint16_t first_push_status = 0;
static uint16_t second_push_status = 0;
static uint16_t release_status = 0;
#endif

#if (TOUCH_CAPSENSE == CY8CMBR3)
static void touch_pin_handler(void *cy8cmbr3); // forward declaration
static cy8cmbr3_t sensor = 
{
    .touched_pin = IO_TOUCHED_PIN,
    .addr        = CY8CMBR3_ADDR,
    .callback    = touch_pin_handler
};

#elif (TOUCH_CAPSENSE == IQS620)
static void touch_pin_handler(void *iqs620, iqs620_button_t button, iqs620_event_t event); // forward declaration
static iqs620_t sensor =
{
    .rdy_pin         = IO_TOUCHED_PIN,
    .addr            = IQS620_ADDR,
    .callback        = touch_pin_handler,
    .prox_threshold  = 0, // 0=most sensitive, 255=least sensitive
    .touch_threshold = 0,
    .ati_target      = 0x1E // target = 0x1E * 32 = 960, gives good results on MK11 Flex through 1mm plastic (higher value slow to react)
};

static const char *buttonNames[] =
{
    [IQS620_BUTTON_B0]      = "B0",
    [IQS620_BUTTON_B1]      = "B1"
};

static const char *eventNames[] =
{
    [IQS620_BUTTON_UP]      = "UP",
    [IQS620_BUTTON_PROX]    = "PROX",
    [IQS620_BUTTON_DOWN]    = "DOWN",
};
#endif

// ===== private functions =====

// for logging
#ifdef TOUCH_LOG_INFO_ON
#define TOUCH_LOG_INFO(...) NRF_LOG_INFO(__VA_ARGS__)
#else
#define TOUCH_LOG_INFO(...) 
#endif

#ifdef TOUCH_LOG_DEBUG_ON
#define TOUCH_LOG_DEBUG(...) NRF_LOG_DEBUG(__VA_ARGS__)
#else
#define TOUCH_LOG_DEBUG(...) 
#endif

// forward declarations
static void touch_timer_start(uint32_t ticks);
static void touch_timer_restart(uint32_t ticks);
static void touch_timer_stop(void);
#ifdef TOUCH_CAPSENSE
static void touch_store_button_status(uint16_t *status);
#endif

// send gesture to the handler (if any) registered by touch_init()
static void generate_gesture(touch_gesture_t gesture)
{
#ifdef TOUCH_LOG_INFO_ON
    switch(gesture)
    {
    case TOUCH_GESTURE_TAP:
        TOUCH_LOG_INFO("Touch gesture Tap.");
        break;
    case TOUCH_GESTURE_DOUBLETAP:
        TOUCH_LOG_INFO("Touch gesture DoubleTap.");
        break;
    case TOUCH_GESTURE_PRESS:
        TOUCH_LOG_INFO("Touch gesture Press.");
        break;
    case TOUCH_GESTURE_LONGPRESS:
        TOUCH_LOG_INFO("Touch gesture LongPress.");
        break;
#ifdef TOUCH_CAPSENSE
    case TOUCH_GESTURE_PRESSBOTH:
        TOUCH_LOG_INFO("Touch gesture PressBoth.");
        break;
    case TOUCH_GESTURE_LONGBOTH:
        TOUCH_LOG_INFO("Touch gesture LongBoth.");
        break;
    case TOUCH_GESTURE_SLIDELR:
        TOUCH_LOG_INFO("Touch gesture SlideLR.");
        break;
    case TOUCH_GESTURE_SLIDERL:
        TOUCH_LOG_INFO("Touch gesture SlideRL.");
        break;
#endif
    }
#endif

    if (touch_gesture_handler)
    {
        touch_gesture_handler(gesture);
    }
}

uint8_t bit_count (uint16_t value) {
    uint8_t count = 0;
    while (value > 0) {              // until all bits are zero
        if ((value & 0x0001) == 1)   // check lower bit
            count++;
        value >>= 1;                 // shift bits, removing lower bit
    }
    return count;
}

// handle events, which can be button push/release (coming from touch_pin_handler()) or timer timeout
// update state machine accordingly
static void touch_event_handler(bool istimer)
{
    switch(touch_state)
    {
    case TOUCH_IDLE:
        if (istimer)
        {
            // stay in IDLE
            NRF_LOG_ERROR("Touch IDLE: Timer was not stopped!");
        } else {
#ifdef TOUCH_CAPSENSE
            // record first button touched
            touch_store_button_status(&first_push_status);
            // check whether one button pushed, or two
            if(first_push_status == 0) { // no buttons pushed, so an unexpected release event
                NRF_LOG_ERROR("Touch IDLE: release event, but expected push");
                return;
            } else if (bit_count(first_push_status) == 2) { // two buttons pushed simultaneously
                touch_state = TOUCH_BOTH_PRESSED;
                TOUCH_LOG_DEBUG("Touch IDLE->BOTH_PRESSED");
                touch_timer_restart(TOUCH_LONG_INTERVAL);
            } else { // one button pushed
#endif
                touch_state = TOUCH_TRIGGERED;
                TOUCH_LOG_DEBUG("Touch IDLE->TRIGGERED");
                touch_timer_start(TOUCH_PRESS_INTERVAL);
#ifdef TOUCH_CAPSENSE
            }
#endif
        }
        break;
    case TOUCH_TRIGGERED:
        if (istimer) {
            touch_state = TOUCH_PRESSED;
            TOUCH_LOG_DEBUG("Touch TRIGGERED->PRESSED");
            touch_timer_restart(TOUCH_LONG_INTERVAL);
        } else {
#ifdef TOUCH_CAPSENSE
            touch_store_button_status(&second_push_status);
            if(second_push_status) { // non-zero -> another push
                touch_state = TOUCH_BOTH_PRESSED;
                TOUCH_LOG_DEBUG("Touch TRIGGERED->BOTH_PRESSED");
                touch_timer_restart(TOUCH_LONG_INTERVAL);
            } else { // all zeros -> release
#endif
                touch_state = TOUCH_TAPPED;
                TOUCH_LOG_DEBUG("Touch TRIGGERED->TAPPED");
                touch_timer_restart(TOUCH_TAP_INTERVAL);
#ifdef TOUCH_CAPSENSE
            }
#endif
        }
        break;
    case TOUCH_TAPPED:
        if (istimer) {
            touch_state = TOUCH_IDLE;
            TOUCH_LOG_DEBUG("Touch TAPPED->IDLE");
            // timer already stopped
            generate_gesture(TOUCH_GESTURE_TAP);
        } else {
            touch_timer_stop();
            // no timer in TAPPED2 or SLID; wait indefinitely for release
#ifdef TOUCH_CAPSENSE
            touch_store_button_status(&second_push_status);
            if(second_push_status != first_push_status) { // pushed different button
                touch_state = TOUCH_SLID;
                TOUCH_LOG_DEBUG("Touch TAPPED->SLID");
            } else { // pushed same button
#endif
                touch_state = TOUCH_TAPPED2;
                TOUCH_LOG_DEBUG("Touch TAPPED->TAPPED2");
#ifdef TOUCH_CAPSENSE
            }
#endif
        }
        break;
    case TOUCH_TAPPED2:
        if (istimer) {
            // stay in TAPPED2
            NRF_LOG_ERROR("Touch TAPPED2: should not be a timer here!");
        } else {
            touch_state = TOUCH_IDLE;
            TOUCH_LOG_DEBUG("Touch TAPPED2->IDLE");
            touch_timer_stop();
            generate_gesture(TOUCH_GESTURE_DOUBLETAP);
        }
        break;
    case TOUCH_PRESSED:
        if (istimer) {
            touch_state = TOUCH_LONG;
            TOUCH_LOG_DEBUG("Touch PRESSED->LONG");
            // timer already stopped
            // no timer in LONG; wait indefinitely for release
        } else {
#ifdef TOUCH_CAPSENSE
            touch_store_button_status(&second_push_status);
            if(second_push_status) { // non-zero -> another push
                touch_state = TOUCH_BOTH_PRESSED;
                TOUCH_LOG_DEBUG("Touch PRESSED->BOTH_PRESSED");
                touch_timer_restart(TOUCH_LONG_INTERVAL);
            } else { // all zeros -> release
#endif
                touch_state = TOUCH_IDLE;
                TOUCH_LOG_DEBUG("Touch PRESSED->IDLE");
                touch_timer_stop();
                // no timer in IDLE
                generate_gesture(TOUCH_GESTURE_PRESS);
#ifdef TOUCH_CAPSENSE
            }
#endif
        }
        break;
    case TOUCH_LONG:
        if (istimer) {
            // stay in LONG;
            NRF_LOG_ERROR("Touch LONG: should not be a timer here!");
        } else {
            touch_state = TOUCH_IDLE;
            TOUCH_LOG_DEBUG("Touch LONG->IDLE");
            touch_timer_stop();
            generate_gesture(TOUCH_GESTURE_LONGPRESS);
        }
        break;
#ifdef TOUCH_CAPSENSE
    case TOUCH_SLID:
        if (istimer) {
            // stay in SLID;
            NRF_LOG_ERROR("Touch SLID: should not be a timer here!");
        } else { // must be a release
            touch_state = TOUCH_IDLE;
            TOUCH_LOG_DEBUG("Touch SLID->IDLE");
#if defined(BOARD_MK9B)
            if(first_push_status > second_push_status) { //NOTE tested OK for MK9B
#else
            if(first_push_status < second_push_status) { //NOTE believe this is correct for MK10
#endif
                generate_gesture(TOUCH_GESTURE_SLIDELR);
            } else {
                generate_gesture(TOUCH_GESTURE_SLIDERL);
            }
        }
        break;
    case TOUCH_BOTH_PRESSED:
        if (istimer) {
            touch_state = TOUCH_BOTH_LONG;
            TOUCH_LOG_DEBUG("Touch BOTH_PRESSED->BOTH_LONG");
            // timer already stopped
            // no timer in BOTH_LONG; wait indefinitely for release
        } else {
            touch_timer_stop();
            // no timer in either case
            // check whether both are released, or just one
            touch_store_button_status(&release_status);
            if(release_status == 0) { // both buttons released
                touch_state = TOUCH_IDLE;
                TOUCH_LOG_DEBUG("Touch BOTH_PRESSED->IDLE");
                generate_gesture(TOUCH_GESTURE_PRESSBOTH);
            } else { // one button released
                TOUCH_LOG_DEBUG("Touch BOTH_PRESSED: one button released");
            }
        }
        break;
    case TOUCH_BOTH_LONG:
        if (istimer) {
            // stay in BOTH_LONG;
            NRF_LOG_ERROR("Touch BOTH_LONG: should not be a timer here!");
        } else {
            // check whether both are released, or just one
            touch_store_button_status(&release_status);
            if(release_status == 0) { // both buttons released
                touch_state = TOUCH_IDLE;
                TOUCH_LOG_DEBUG("Touch BOTH_LONG->IDLE");
                generate_gesture(TOUCH_GESTURE_LONGBOTH);
            } else { // one button released
                TOUCH_LOG_DEBUG("Touch BOTH_LONG: one button released");
            }
        }
        break;
#endif
    }
}

#ifdef TOUCH_CAPSENSE
#if (TOUCH_CAPSENSE == CY8CMBR3)
static void touch_pin_handler(void *cy8cmbr3)
{
    ASSERT(cy8cmbr3 != NULL);
    TOUCH_LOG_DEBUG("Touch pin handler.");
    touch_event_handler(false); // process state machine change, from button push/release (not timer, so false)
}
#elif (TOUCH_CAPSENSE == IQS620)
static void touch_pin_handler(void *iqs620, iqs620_button_t button, iqs620_event_t event)
{
    ASSERT(iqs620 != NULL);
    TOUCH_LOG_DEBUG("Touch pin handler: IQS620 %s %s", buttonNames[button], eventNames[event]);
    if(event == IQS620_BUTTON_DOWN || event == IQS620_BUTTON_UP) // ignore IQS620_BUTTON_PROX event
    {
        touch_event_handler(false); // process state machine change, from button push/release (not timer, so false)
    }
}
#endif
#else
// handle debouced switch with app_button library
// scope measurement results:
//   from MK9B on-board button: bounce very rarely observed when pressing button (only observed once, 1 bounce within 5us)
//                              bounce more common when releasing button (observed 1~3 bounces within 42~198us)
//                              bounces are <5us in duration
static void touch_button_handler(uint8_t pin_no, uint8_t button_action)
{
    TOUCH_LOG_DEBUG("Debounced Touch button %s.", (button_action == APP_BUTTON_PUSH) ? "Push" : "Release" );
    touch_event_handler(false); // process state machine change, from button push/release (not timer, so false)
}
#endif

static void delay_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    TOUCH_LOG_DEBUG("Touch timeout.");
    touch_event_handler(true); // process state machine change, from timer (so true)
}

static void touch_timer_start(uint32_t ticks)
{
    ret_code_t err_code;

    // Start application timer
    err_code = app_timer_start(delay_timer_id, ticks, NULL);
    APP_ERROR_CHECK(err_code);
}

static void touch_timer_restart(uint32_t ticks)
{
    ret_code_t err_code;

    // Stop the timer
    err_code = app_timer_stop(delay_timer_id);
    APP_ERROR_CHECK(err_code);

    // Re-initialize the timer (per app_timer.h: "can be called again ... and will re-initialize ... if the timer is not running.")
    err_code = app_timer_create(&delay_timer_id, APP_TIMER_MODE_SINGLE_SHOT, delay_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // Start application timer
    err_code = app_timer_start(delay_timer_id, ticks, NULL);
    APP_ERROR_CHECK(err_code);
}

static void touch_timer_stop(void)
{
    ret_code_t err_code;

    // Stop the timer
    err_code = app_timer_stop(delay_timer_id);
    APP_ERROR_CHECK(err_code);

    // Re-initialize the timer (per app_timer.h: "can be called again ... and will re-initialize ... if the timer is not running.")
    err_code = app_timer_create(&delay_timer_id, APP_TIMER_MODE_SINGLE_SHOT, delay_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // Ready for next call to touch_timer_start()
}

#ifdef TOUCH_CAPSENSE
static void touch_store_button_status(uint16_t *status)
{
#if (TOUCH_CAPSENSE == CY8CMBR3)
    cy8cmbr3_get_button_status(&sensor, status);
#elif (TOUCH_CAPSENSE == IQS620)
    iqs620_get_button_status(&sensor, status);
#endif
}
#endif

// ====== public function implementations =====

// used in early init, for wakeup if put to sleep immediately due to low or charging battery
void touch_quick_init(void)
{
    nrf_gpio_cfg(
        IO_TOUCHED_PIN,
        NRF_GPIO_PIN_DIR_INPUT,
        NRF_GPIO_PIN_INPUT_CONNECT,
        IO_TOUCHED_PIN_PULL,
        NRF_GPIO_PIN_S0S1,
        NRF_GPIO_PIN_NOSENSE); // NOTE sense is set in shutdown_handler()
}

// used in full init
bool touch_init(touch_gesture_handler_t handler)
{
    ret_code_t err_code;
    bool success = false;

    // configure touched_N pin
#ifdef TOUCH_CAPSENSE
#if (TOUCH_CAPSENSE == CY8CMBR3)
    success = cy8cmbr3_init(&sensor);
#elif (TOUCH_CAPSENSE == IQS620)
    success = iqs620_init(&sensor);
#endif
#else // no TOUCH_CAPSENSE, use push button
    // OLD, witouth debounce using app_button
    //nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);

    // handle button debounce using app_button library
    static const app_button_cfg_t app_buttons[1] =
    {
        {IO_TOUCHED_PIN, APP_BUTTON_ACTIVE_LOW, IO_TOUCHED_PIN_PULL, touch_button_handler},
    };
    app_button_init((app_button_cfg_t *)app_buttons, 1, APP_TIMER_TICKS(30));
    app_button_enable();
    success = true; // nothing else to set without CapSense
#endif

    err_code = app_timer_create(&delay_timer_id, APP_TIMER_MODE_SINGLE_SHOT, delay_timeout_handler);
    APP_ERROR_CHECK(err_code);

    touch_gesture_handler = handler; // resgister the handler
    touch_state = TOUCH_IDLE; // start in IDLE state & wait for button push interrupt
    return(success);
}

bool touch_reprogram(void)
{
#if (TOUCH_CAPSENSE == CY8CMBR3)
    bool success = cy8cmbr3_reprogram(&sensor);
#elif (TOUCH_CAPSENSE == IQS620)
    bool success = iqs620_reset(&sensor);
#endif
    return(success);
}