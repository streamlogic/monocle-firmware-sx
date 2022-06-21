/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License

  Contains BLE library code:
  --------------------------
  Copyright (c) 2021, SwaraLink Technologies
  All Rights Reserved
  Licensed by SwaraLink Technologies, subject to terms of Software License Agreement
  Refer swl_release/LICENSE.txt for the license information.

*******************************************************************************/

/** @file
 *
 * @brief Brilliant Monocle with BLE running on MK9B, MK10, MK11
 *
 * This file contains the source code for Brilliant Monocle with BLE running on MK9B, MK10, MK11
 *
 */

 // MCU firmware version number; up this for each release to match the tag on main branch
#define MCU_VER_MAJOR 1
#define MCU_VER_MINOR 6

// Interrupt Map
// see: https://infocenter.nordicsemi.com/topic/sds_s132/SDS/s1xx/processor_avail_interrupt_latency/exception_mgmt_sd.html?cp=4_7_3_0_15_1
//
// 0 Softdevice: timing-critical
// 1 Softdevice: memory protection
// 2 App: 
// 3 App: 
// 4 Softdevice: API & non-time-critical
// 5 App: NRFX_SPIM
// 6 App: GPIOTE, NRFX_CLOCK, NRFX_GPIOTE, NRFX_RNG, NRFX_SAADC, NRF_CLOCK, RNG, TWI, WDT, APP_TIMER
// 7 App: 
// - main()

 // C standard library header files
#include <stdbool.h>
#include <stdint.h>
#include <string.h> // from SWL, needed?

// Board file
#include "board.h"  // BSP file(s), see also board.c, mkXX_board.h

// Nordic libraries
//#include "nrf.h" // SWL: in pwr.h
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
//#include "nrf_pwr_mgmt.h" // SWL: in pwr.h
#include "nrf_delay.h"

// Nordic logging
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// for SPI & peripherals
#include "spi.h" // TODO this should be commented out & all interaction should be through the following, to ensure correct CS
#include "oled.h"
#include "fpga.h"

// for I2C & peripherals
#include "i2c.h"
#include "ov5640.h"
// PMIC on I2C
#ifdef BOARD_MK10
#include "bq25125.h"
#endif
#ifdef BOARD_MK11
#include "max77654.h"
#endif

// for ADC & battery
#include "adc.h"

// for touch sensor
#include "touch.h"

// for timer
#include "app_timer.h"
#include "nrf_drv_clock.h"

// for LEDs (either on MCU, on FPGA, or on PMIC)
#include "led.h"

// for power management
//TODO not much here, makes more sense to bring back into main.c
#include "pwr.h"

#ifdef WATCHDOG_ON
// for watchdog
#include "nrf_drv_wdt.h"
#endif

// for unit tests
#include "unit.h"

#ifdef BLE_ON
#include "nrf_crypto.h"
#include "app_error.h"
#include "nordic_common.h"
#include "sdk_errors.h"
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#endif

// SWL Peripheral Application Configuration - Modify this file for build-time configuration
#include "swl_periph_app_config.h"

// SWL Peripheral Library Includes
#include "swl_periph.h"
#include "swl_periph_config.h"
#include "swl_periph_nrf_52.h"
#include "swl_periph_demo.h"

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define SWL_PAIRING_TIMEOUT_DEFAULT     120         // 120 seconds pairing timeout

static const char * swl_periph_evt_desc[] = {
  [SWL_PERIPH_EVT_CONN_ESTABLISHED] = "SWL_PERIPH_EVT_CONN_ESTABLISHED",
  [SWL_PERIPH_EVT_CONN_TERMINATED] = "SWL_PERIPH_EVT_CONN_TERMINATED",
  [SWL_PERIPH_EVT_RX_DATA] = "SWL_PERIPH_EVT_RX_DATA",
  [SWL_PERIPH_EVT_DATA_ACK] = "SWL_PERIPH_EVT_DATA_ACK",
  [SWL_PERIPH_EVT_DATA_SENT] = "SWL_PERIPH_EVT_DATA_SENT",
  [SWL_PERIPH_EVT_PWR_MODE_UPDATE] = "SWL_PERIPH_EVT_PWR_MODE_UPDATE",
  [SWL_PERIPH_EVT_PWR_MODE_UPDATE_REQUEST] = "SWL_PERIPH_EVT_PWR_MODE_UPDATE_REQUEST",
  [SWL_PERIPH_EVT_PAIRING_MODE_EXITED] = "SWL_PERIPH_EVT_PAIRING_MODE_EXITED"
};

//Used to tell whether we are in pairing mode
static bool swl_in_pairing_mode = false;

#endif // BLE_ON

#if defined(BLE_ON) || defined(BLE_TEST_MODE)
// tx module, supports bulk data transfers from Monocle over Bluetooth (can also run in "simulation" with BLE_TEST_MODE)
#include "tx.h"
#else
#define tx_is_full(x) false // buffer never full if no BLE transmission
#define tx_is_transferring(x) false // never transmitting if no BLE
#endif

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_HALT_ON_ERROR(err, msg) if(err) {NRF_LOG_ERROR(msg); \
                                             led_green_off(); \
                                             led_red_on();  \
                                             led_green_blink_timer(self_test_pass, true); while(1){NRF_LOG_PROCESS();};}

// === global version information ===
uint8_t g_mcu_ver_major = MCU_VER_MAJOR;
uint8_t g_mcu_ver_minor = MCU_VER_MINOR;
uint8_t g_fpga_ver_major = 0; // zero indicates unknown, until set after FPGA is booted
uint8_t g_fpga_ver_minor = 0; // zero indicates unknown, until set after FPGA is booted

// === State Machine ===
typedef enum {
    MONOCLE_EARLY_INIT,
    MONOCLE_FULL_INIT,
    MONOCLE_POWERED_DOWN,
    MONOCLE_SHUTDOWN_PENDING,
    MONOCLE_RECORD_D1,
    MONOCLE_RECORD_D0,
    MONOCLE_PHOTO_D1,
    MONOCLE_PHOTO_D0,
    MONOCLE_REPLAY_D1,
    MONOCLE_REPLAY_D0,
    MONOCLE_STANDBY_WARN,
    MONOCLE_STANDBY
} monocle_state_t;      // main state machine

static const char * monocle_state_desc[] = {
    [MONOCLE_EARLY_INIT] = "MONOCLE_EARLY_INIT",
    [MONOCLE_FULL_INIT] = "MONOCLE_FULL_INIT",
    [MONOCLE_POWERED_DOWN] = "MONOCLE_POWERED_DOWN",
    [MONOCLE_SHUTDOWN_PENDING] = "MONOCLE_SHUTDOWN_PENDING",
    [MONOCLE_RECORD_D1] = "MONOCLE_RECORD_D1",
    [MONOCLE_RECORD_D0] = "MONOCLE_RECORD_D0",
    [MONOCLE_PHOTO_D1] = "MONOCLE_PHOTO_D1",
    [MONOCLE_PHOTO_D0] = "MONOCLE_PHOTO_D0",
    [MONOCLE_REPLAY_D1] = "MONOCLE_REPLAY_D1",
    [MONOCLE_REPLAY_D0] = "MONOCLE_REPLAY_D0",
    [MONOCLE_STANDBY_WARN] = "MONOCLE_STANDBY_WARN",
    [MONOCLE_STANDBY] = "MONOCLE_STANDBY",
};                      // for debug logging

typedef enum {
    MONOCLE_ZOOM_1X,
    MONOCLE_ZOOM_4X,
    MONOCLE_ZOOM_16X
} monocle_zoom_t;       // digital zoom level

typedef enum {
    MONOCLE_SPEED_1_4X, // quarter speed
    MONOCLE_SPEED_1X    // normal speed
} monocle_speed_t;      // determines frame rate in Replay mode

typedef enum {
    MONOCLE_DISP_CAM,   // default: display on (live from camera, image capture, replay)
    MONOCLE_DISP_BUSY,  // display busy screen (grey screen)
    MONOCLE_DISP_BARS,  // display vertical color bars
    MONOCLE_DISP_OFF    // display off
} monocle_display_state_t;      // display state

static monocle_state_t monocle_state = MONOCLE_EARLY_INIT; // current state machine state
static monocle_zoom_t monocle_zoom = MONOCLE_ZOOM_1X; // current zoom level
static monocle_speed_t monocle_speed = MONOCLE_SPEED_1X; // current replay speed
static monocle_display_state_t monocle_display_state = MONOCLE_DISP_CAM; // current display state
// for Bluetooth state
static bool monocle_is_ble_connected = false; // if BLE connection is established
static bool monocle_discard_on_resume = true; // send dispose command after resuming (i.e. if no tx_en_queue())

typedef enum {
    MONOCLE_EVENT_G_TAP, // event generated by tap gesture
    MONOCLE_EVENT_G_DOUBLE_TAP,
    MONOCLE_EVENT_G_PRESS,
    MONOCLE_EVENT_G_LONGPRESS, // not in official UX, but used for testing display modes
#ifdef TOUCH_CAPSENSE
    MONOCLE_EVENT_G_LONGBOTH,
#endif
    MONOCLE_EVENT_TIMEOUT,
    MONOCLE_EVENT_BATT_LOW,
    MONOCLE_EVENT_BATT_CHARGE,
    MONOCLE_EVENT_BLE_CONNECT,
    MONOCLE_EVENT_BLE_DISCONNECT,
    MONOCLE_EVENT_BLE_TRANSFER_DONE // TODO: event is not yet generated (but processing by state machine is done)
} monocle_event_t;      // events to be handled by state machine

static const char * monocle_event_desc[] = {
    [MONOCLE_EVENT_G_TAP] = "MONOCLE_EVENT_G_TAP",
    [MONOCLE_EVENT_G_DOUBLE_TAP] = "MONOCLE_EVENT_G_DOUBLE_TAP",
    [MONOCLE_EVENT_G_PRESS] = "MONOCLE_EVENT_G_PRESS",
    [MONOCLE_EVENT_G_LONGPRESS] = "MONOCLE_EVENT_G_LONGPRESS",
#ifdef TOUCH_CAPSENSE
    [MONOCLE_EVENT_G_LONGBOTH] = "MONOCLE_EVENT_G_LONGBOTH",
#endif
    [MONOCLE_EVENT_TIMEOUT] = "MONOCLE_EVENT_TIMEOUT",
    [MONOCLE_EVENT_BATT_LOW] = "MONOCLE_EVENT_BATT_LOW",
    [MONOCLE_EVENT_BATT_CHARGE] = "MONOCLE_EVENT_BATT_CHARGE",
    [MONOCLE_EVENT_BLE_CONNECT] = "MONOCLE_EVENT_BLE_CONNECT",
    [MONOCLE_EVENT_BLE_DISCONNECT] = "MONOCLE_EVENT_BLE_DISCONNECT",
    [MONOCLE_EVENT_BLE_TRANSFER_DONE] = "MONOCLE_EVENT_BLE_TRANSFER_DONE"
};

#ifdef TOUCH_CAPSENSE
#define EVENT_IS_GESTURE(event) ( (event==MONOCLE_EVENT_G_TAP)     || (event==MONOCLE_EVENT_G_DOUBLE_TAP) \
                               || (event==MONOCLE_EVENT_G_PRESS)   || (event==MONOCLE_EVENT_G_LONGPRESS)  \
                               || (event==MONOCLE_EVENT_G_LONGBOTH)                                       \
                                )
#else
#define EVENT_IS_GESTURE(event) ( (event==MONOCLE_EVENT_G_TAP)   || (event==MONOCLE_EVENT_G_DOUBLE_TAP) \
                               || (event==MONOCLE_EVENT_G_PRESS) || (event==MONOCLE_EVENT_G_LONGPRESS)  \
                                )
#endif

 // forward declarations
void event_handler(monocle_event_t event);
static void idle_state_handle(void);

// === Start Timers code ===
#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(1000)                   /**< Interval between Battery level measurements (ticks) = 1 second */
#define KEEP_ALIVE_INTERVAL              APP_TIMER_TICKS(5000)                   /**< Interval for watchdog keep-alive & LED blinks (ticks) = 5 seconds */

#define STATE_TIMEOUT_DISPLAYING_SHORT   APP_TIMER_TICKS(5000) // 5 seconds --> OLED displying for short time
#define STATE_TIMEOUT_DISPLAYING_LONG    APP_TIMER_TICKS(10000) // 10 seconds --> OLED displying for longer time
#define STATE_TIMEOUT_DISPLAYING_REPLAY  APP_TIMER_TICKS(20000) // 20 seconds --> OLED displying for replay
#define STATE_TIMEOUT_RECORDING          APP_TIMER_TICKS(300000) // 5 minutes --> from recording (Record_D1, Record_D0) to standby_warn
#define STATE_TIMEOUT_STANDBY_WARN       APP_TIMER_TICKS(3000) // 3 seconds --> from standby_warn to standby
#define STATE_TIMEOUT_STANDBY            APP_TIMER_TICKS(60000) // 1 minute --> from standby to shutdown

APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer; repeated. */
APP_TIMER_DEF(m_keepalive_timer_id);                                /**< Sleep timer; repeated. */
APP_TIMER_DEF(m_state_timer_id);                                    /**< State timeout timer; single-shot. */
#ifdef WATCHDOG_ON
nrf_drv_wdt_channel_id m_channel_id;                                /**< Watchdog timer channel. */
#endif

// === Battery level definitions ===
#define BATTERY_LEVEL_CHARGING      4.40 // if greater than this is considered to be in charging case
#define BATTERY_LEVEL_EMERGENCY_OFF 3.0  // (0% SoC) emergency cut-off: turn off immediately if battery reaches this level, or lower
#define BATTERY_LEVEL_TURN_OFF      3.3  // (4% SoC) initiate clean shut-down when battery drops below this level (includes completing pending data transfer)
#define BATTERY_BOOT_HYSTERESIS     0.15 // (20% SoC) prevent re-boot loops by requiring battery to be above TURN_OFF + BOOT_HYSTERESIS to allow boot

// Global variables
bool shut_me_down = false;

/**@brief Handler for shutdown preparation.
 */
bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    uint32_t err_code;

    if (false) //TODO return false to prevent shutdown (ie if BLE data transfer still in progress)
    {
        return false;
    }

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_SYSOFF: // nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_STAY_IN_SYSOFF); was called
            NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_SYSOFF: shutting down until external reset.");
            // the application will require an external reset to restart because the wakeup mechanism is not set up
            // do something
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP: // nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF); was called
            NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_WAKEUP: shutting down until next touched.");
            // set wakeup pin, active low
            // NOTE How does this work?
            nrf_gpio_cfg_sense_set(IO_TOUCHED_PIN, NRF_GPIO_PIN_SENSE_LOW);
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_ERROR("NRF_PWR_MGMT_EVT_PREPARE_DFU: Entering DFU is not supported by this example.");
            APP_ERROR_HANDLER(NRF_ERROR_API_NOT_IMPLEMENTED);
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_RESET: // nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET); was called
            NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_RESET: restarting app.");
            // do something
            break;
    }

    if(monocle_state == MONOCLE_EARLY_INIT) // can happen for MK9B, MK10, which check boot voltage here
    {
        adc_quick_uninit();
    }
    else if (monocle_state == MONOCLE_FULL_INIT) // can only happen for MK11, because of delayed boot voltage check
    {
        adc_uninit();
        i2c_uninit();
    }
    else // fully powered up, timers were started
    {

        if (shut_me_down == true) // Activate Factory Mode
        {
            NRF_LOG_INFO("Going to factory ship mode");
            max77564_factory_ship_mode();
        }
        //TODO anything else?
        err_code = app_timer_stop_all();
        APP_ERROR_CHECK(err_code);
        i2c_sw_uninit();
        spi_uninit();
        bsp_board_aux_power_off(); // turn off auxiliary power
        i2c_uninit();
        adc_uninit();
    }
    bsp_board_uninit();
#ifdef BOARD_MK10
    bsp_board_io_off(IO_PMIC_ACTIVE); // active mode=LOW so that charging will be enabled
#endif
    return true;
}

/**@brief Register application shutdown handler with priority 0. */
NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, 0);


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    ret_code_t err_code;
    uint8_t reg_value = 0;
    static bool RB_enabled = false;

    UNUSED_PARAMETER(p_context);
    //battery_level_update();
    // trigger taking one sample, non-blocking, result will be stored in buffer
    adc_start_sample();

#ifdef TEST_FLICKER_RB_SHIFT
    fpga_disp_RB_shift(RB_enabled);
    NRF_LOG_INFO("RB Shift set %s", RB_enabled?"ON":"OFF");
    RB_enabled = !RB_enabled;
#endif
}


/**@brief Function for handling the Keepalive timer timeout.
 *
 * @details This function will be called each time the keep alive timer expires to reset the watchdog.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void keepalive_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

#if LEDS_NUMBER > 0 // for MK9B
    bsp_board_led_invert(0);
//    NRF_LOG_INFO("Batt: "NRF_LOG_FLOAT_MARKER"V, SoC=%d%%", NRF_LOG_FLOAT(adc_get_batt_voltage()), adc_get_batt_soc());
#endif

//WARNING turn this back on when not testing!!
#if 0 // turn off, interferes with SPI testing (incl. checksum testing)
//#if defined (FPGA_ON) && !(defined(TEST1) || defined(TEST3)) // blinking LED with SPI write in interrupt context breaks unit tests!
//    fpga_led_toggle(0); // green LED on MK10
    led_green_blink_now(1); // NOTE blocks for 150ms; green LED on MK10
#endif

#ifdef BOARD_MK10
    NRF_LOG_INFO("Batt: "NRF_LOG_FLOAT_MARKER"V, SoC=%d%%", NRF_LOG_FLOAT(adc_get_batt_voltage()), adc_get_batt_soc());
    bq25125_charging_status();
#endif

    // NOTE battery level updated avery 5 samples (5s) in saadc_callback() in adc.c
//    NRF_LOG_INFO("Batt: "NRF_LOG_FLOAT_MARKER"V, SoC=%d%%", NRF_LOG_FLOAT(adc_get_batt_voltage()), adc_get_batt_soc());
    if(adc_get_batt_voltage() < BATTERY_LEVEL_TURN_OFF) { // battery low: generate event
        event_handler(MONOCLE_EVENT_BATT_LOW);
    } else if (adc_get_batt_voltage() > BATTERY_LEVEL_CHARGING) {
        event_handler(MONOCLE_EVENT_BATT_CHARGE);         // battery charging: generate event
    }

#ifdef WATCHDOG_ON
    //feed watchdog timer; guarantee will be fed at least every KEEP_ALIVE_INTERVAL seconds
    nrf_drv_wdt_channel_feed(m_channel_id);
#endif

}

static void state_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    event_handler(MONOCLE_EVENT_TIMEOUT);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
#ifdef BLE_ON
    // Initialization is now done by the SWL library, which must be called before this
#else
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
#endif

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_keepalive_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                keepalive_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_state_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                state_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    //NOTE battery timer off for TESTING on MK9B
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_keepalive_timer_id, KEEP_ALIVE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_state_timer_id, STATE_TIMEOUT_DISPLAYING_SHORT, NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Monocle Repeated timers started.");
}


static void state_timer_restart(uint32_t ticks)
{
    ret_code_t err_code;

    // Stop the timer
    err_code = app_timer_stop(m_state_timer_id);
    APP_ERROR_CHECK(err_code);

    // Re-initialize the timer (per app_timer.h: "can be called again ... and will re-initialize ... if the timer is not running.")
    err_code = app_timer_create(&m_state_timer_id, APP_TIMER_MODE_SINGLE_SHOT, state_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // Start application timer
    err_code = app_timer_start(m_state_timer_id, ticks, NULL);
    APP_ERROR_CHECK(err_code);
//    NRF_LOG_DEBUG("Monocle State timer re-started, %d ticks.", ticks);
}

static void state_timer_stop(void)
{
    ret_code_t err_code;

    // Stop the timer
    err_code = app_timer_stop(m_state_timer_id);
    APP_ERROR_CHECK(err_code);

    // Re-initialize the timer (per app_timer.h: "can be called again ... and will re-initialize ... if the timer is not running.")
    err_code = app_timer_create(&m_state_timer_id, APP_TIMER_MODE_SINGLE_SHOT, state_timeout_handler);
    APP_ERROR_CHECK(err_code);

//     NRF_LOG_DEBUG("Monocle State timer stopped.");
   // Ready for next call to state_timer_start()
}

// === Start ADC code ===
// moved to adc.c
// === end ADC ===

// Softdevices will start the LF clock, but in case of no softdevice, start it manually
static void lfclk_start(void)
{
    if(!nrf_drv_clock_init_check()) // check whether it was already initialized (eg by BLE)
    {
        ret_code_t err_code = nrf_drv_clock_init();
        APP_ERROR_CHECK(err_code);
        nrf_drv_clock_lfclk_request(NULL);
    }
}

// for logging timestamp
uint32_t get_rtc_counter(void)
{
    return NRF_RTC1->COUNTER;
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(get_rtc_counter);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void zoom_in(void)
{
    switch(monocle_zoom)
    {
        case MONOCLE_ZOOM_1X: // go to 4x zoom
            ov5640_mode_2x(); // camera ISP pre-scaling size 1280x800
            nrf_delay_ms(70); // ensure previous command completes (OV5640 will execute it at end of frame)
            ov5640_outsize_set(8+320, 24+200, 640, 400); // camera ISP pre-scaling size 640x400
            monocle_zoom = MONOCLE_ZOOM_4X;
            NRF_LOG_INFO("Monocle Zoom In 1X -> 4X (camera 4x).");
            break;
        case MONOCLE_ZOOM_4X: // go to 16x zoom
            fpga_set_zoom(4);
            monocle_zoom = MONOCLE_ZOOM_16X;
            NRF_LOG_INFO("Monocle Zoom In 4X -> 16X (camera 4x, FPGA 4x).");
            break;
        case MONOCLE_ZOOM_16X: // go back to 1x zoom
            ov5640_mode_1x(); // camera ISP pre-scaling size 2560x1600
            fpga_set_zoom(1);
            monocle_zoom = MONOCLE_ZOOM_1X;
            NRF_LOG_INFO("Monocle Zoom In 16X -> 1X (camera 1x, FPGA 1x).");
            break;
        default:
            NRF_LOG_ERROR("Zoom level not supported.");
    }
}

// toggle (+1) or reset (0) replay speed
void replay_speed(int8_t change_speed)
{
    //TODO
    if(+1 == change_speed) { // toggle speed
        switch(monocle_speed)
        {
            case MONOCLE_SPEED_1_4X: // quarter speed to normal speed
                fpga_replay_rate(3);
                monocle_speed = MONOCLE_SPEED_1X;
                NRF_LOG_INFO("Replay speed increased to 1x speed.");
                break;
            case MONOCLE_SPEED_1X:   // normal speed to quarter speed
                fpga_replay_rate(13);
                monocle_speed = MONOCLE_SPEED_1_4X;
                NRF_LOG_INFO("Replay speed decreased to 1/4 speed.");
                break;
        }
    } else if(0 == change_speed) { // reset speed
                fpga_replay_rate(3);
                monocle_speed = MONOCLE_SPEED_1X;
                NRF_LOG_INFO("Replay speed reset to 1x speed.");
    } else {
        NRF_LOG_ERROR("replay_speed() invalid parameter.");
    }
}

void sleep_camera(void)
{
    NRF_LOG_INFO("Sleeping camera.");
    fpga_camera_off(); // turn off en_cam, allow last frame to finish entering video buffer to avoid split screen
    ov5640_pwr_sleep();
}

void wakeup_camera(void)
{
    NRF_LOG_INFO("Waking up camera.");
    ov5640_pwr_wake();
    fpga_camera_on(); // delay 4 frames to discard AWB adjustments, turn on en_cam
}

// update main state machine based on received event
void event_handler(monocle_event_t event)
{
    //NRF_LOG_INFO("event_handler(%s) in %s", monocle_event_desc[event], monocle_state_desc[monocle_state]);
    // handle battery power events, which apply to almost every state
#ifndef IGNORE_BATT_LEVEL
    if((MONOCLE_EVENT_BATT_CHARGE == event) && (MONOCLE_SHUTDOWN_PENDING != monocle_state)) {
        if(tx_is_transferring()) {
            NRF_LOG_INFO("State: (ANY) -> SHUTDOWN_PENDING due to charging battery.");
            monocle_state = MONOCLE_SHUTDOWN_PENDING;
        } else {
            NRF_LOG_INFO("State: (ANY) -> POWERED_DOWN due to charging battery.");
            monocle_state = MONOCLE_POWERED_DOWN;
            // bsp_board_aux_power_off() will be called by shutdown handler
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF); // shut down until touched
        }
        return; // returns if gone to SHUTDOWN_PENDING; will not return if shutdown above
    }
    if((MONOCLE_EVENT_BATT_LOW == event) && (MONOCLE_SHUTDOWN_PENDING != monocle_state)) {
                                                                                          
        /* Flagging to Shutdown */
        shut_me_down = true;

        if(MONOCLE_STANDBY == monocle_state) {
            if(tx_is_transferring()) {
                NRF_LOG_INFO("State: STANDBY -> SHUTDOWN_PENDING due to low battery.");
                monocle_state = MONOCLE_SHUTDOWN_PENDING;
            } else {
                NRF_LOG_INFO("State: STANDBY -> POWERED_DOWN due to low battery.");
                monocle_state = MONOCLE_POWERED_DOWN;
                // bsp_board_aux_power_off() will be called by shutdown handler
                nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF); // shut down until touched
                // execution will not reach here
            }
        } else { // any other state except SHUTDOWN_PENDING and STANDBY
            NRF_LOG_INFO("State: (ANY) -> STANDBY_WARN due to low battery.");
            state_timer_restart(STATE_TIMEOUT_STANDBY_WARN);
            led_red_on();
            //fpga_disp_bars(); // Battery voltage is already low, cannot turn on the display!
#if defined(DISPLAY_ON) && !defined(DISPLAY_ALWAYS_ON)
            //oled_pwr_wake();
#endif

            monocle_state = MONOCLE_STANDBY_WARN;
        }
        return;
    }
    else
    {
        /* Do not Shutdown */
        shut_me_down = false;
    }
#endif
    // Bluetooth connection events update state, and event processing continues below depending on current state
    if(MONOCLE_EVENT_BLE_CONNECT == event)
    {
        monocle_is_ble_connected = true;
    }
    if(MONOCLE_EVENT_BLE_DISCONNECT == event)
    {
        monocle_is_ble_connected = false;
    }
    // handle non-battery events
#ifdef TOUCH_CAPSENSE
    if(MONOCLE_EVENT_G_LONGBOTH == event) // reset to factory defaults & reboot, regardless of current state
    {
        NRF_LOG_INFO("State: (ANY) -> POWERED_DOWN & reboot due to LongPress of both buttons.");
        NRF_LOG_INFO("===== PERFORMING FACTORY RESET =====");
        // clear touch IC configuration to factory default
        // TODO MK9B: There does not appear to be a simple way to do this. Maybe write an invalid config & chip will boot to defaults?
        //            for now, reprogram the configuration (this provides a way to force an upgrade to new touch config code)
        //      MK11: Since configuration is not stored in flash, this is not required. Can be removed when MK9B support discontinued.
        touch_reprogram();
        NRF_LOG_INFO("Touch IC reset performed.");

#ifdef BLE_ON
        // reset SWL Bluetooth stack
        swl_err_t err_code;
        err_code = swl_periph_perform_factory_reset();
        SWL_ERROR_CHECK(err_code);
        NRF_LOG_INFO("SWL Bluetooth factory reset performed.");
#endif

        // reboot
        monocle_state = MONOCLE_POWERED_DOWN;
        // bsp_board_aux_power_off() will be called by shutdown handler
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET); // trigger reboot
        return; // will not return because of shutdown above
    }
#endif
    switch(monocle_state)
    {
        case MONOCLE_EARLY_INIT:
        case MONOCLE_FULL_INIT:
        case MONOCLE_POWERED_DOWN:
            NRF_LOG_ERROR("No events expected in EARLY/FULL_INIT or POWERED_DOWN states.");
            break;
        case MONOCLE_SHUTDOWN_PENDING:
            if(MONOCLE_EVENT_BLE_TRANSFER_DONE == event) {
                NRF_LOG_INFO("State: SHUTDOWN_PENDING -> POWERED_DOWN.");
                bsp_board_aux_power_off(); // turn off aux power
                monocle_state = MONOCLE_POWERED_DOWN; 
                nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF); // shut down until touched
            } else {
                // ignore event
            }
            break;
        case MONOCLE_RECORD_D1: // ==================== in MONOCLE_RECORD_D1 ====================
            if(MONOCLE_EVENT_TIMEOUT == event) { // turn OLED off, reset timer
                NRF_LOG_INFO("State: RECORD_D1 -> RECORD_D0.");
                state_timer_restart(STATE_TIMEOUT_RECORDING);
#if defined(DISPLAY_ON) && !defined(DISPLAY_ALWAYS_ON)
                oled_pwr_sleep();
#endif
                monocle_state = MONOCLE_RECORD_D0;
            } else if(MONOCLE_EVENT_G_TAP == event) { // zoom and keep OLED on for another 10s
                NRF_LOG_INFO("State: RECORD_D1, Zoom In.");
                state_timer_restart(STATE_TIMEOUT_DISPLAYING_LONG);
                zoom_in();
                // stay in this state
            } else if(MONOCLE_EVENT_G_PRESS == event) { // keep OLED on for another 3s, image capture
                NRF_LOG_INFO("State: RECORD_D1 -> PHOTO_D1.");
                state_timer_restart(STATE_TIMEOUT_DISPLAYING_SHORT);
                fpga_image_capture();
                sleep_camera();
#if defined(BLE_ON) || defined(BLE_TEST_MODE)
                if(monocle_is_ble_connected) 
                {
#ifdef BLE_ON
                    tx_en_queue(false);
#endif
                    monocle_discard_on_resume = false; // will be discarded by tx_continue()
                }
#endif
                monocle_state = MONOCLE_PHOTO_D1;
            } else if(MONOCLE_EVENT_G_DOUBLE_TAP == event) { // keep OLED on for another 20s, replay
                NRF_LOG_INFO("State: RECORD_D1 -> REPLAY_D1.");
                state_timer_restart(STATE_TIMEOUT_DISPLAYING_REPLAY);
                fpga_video_capture();
                sleep_camera();
//#ifdef MIC_ON
//                fpga_mic_off();
//#endif
#if defined(BLE_ON) || defined(BLE_TEST_MODE)
                if(monocle_is_ble_connected) 
                {
                    tx_en_queue(false);
#ifdef BLE_ON
                    monocle_discard_on_resume = false; // will be discarded by tx_continue()
#endif
                }
#endif
                monocle_state = MONOCLE_REPLAY_D1;
            } else {
                //ignore event
            }
            break;
        case MONOCLE_RECORD_D0: // ==================== in MONOCLE_RECORD_D0 ====================
            if(MONOCLE_EVENT_TIMEOUT == event) { // begin standby
                NRF_LOG_INFO("State: RECORD_D0 -> STANDBY_WARN.");
                state_timer_restart(STATE_TIMEOUT_STANDBY_WARN);
                fpga_disp_busy();
#if defined(DISPLAY_ON) && !defined(DISPLAY_ALWAYS_ON)
                oled_pwr_wake();
#endif
                monocle_state = MONOCLE_STANDBY_WARN;
            } else if(MONOCLE_EVENT_G_TAP == event) { // turn OLED on for 3s
                NRF_LOG_INFO("State: RECORD_D0 -> RECORD_D1.");
                state_timer_restart(STATE_TIMEOUT_DISPLAYING_SHORT);
#if defined(DISPLAY_ON) && !defined(DISPLAY_ALWAYS_ON)
                oled_pwr_wake();
#endif
                monocle_state = MONOCLE_RECORD_D1;
            } else if(MONOCLE_EVENT_G_PRESS == event) { // turn OLED on for 3s, image capture
                NRF_LOG_INFO("State: RECORD_D0 -> PHOTO_D1.");
                state_timer_restart(STATE_TIMEOUT_DISPLAYING_SHORT);
                fpga_image_capture();
#if defined(DISPLAY_ON) && !defined(DISPLAY_ALWAYS_ON)
                oled_pwr_wake();
#endif
                sleep_camera();
#if defined(BLE_ON) || defined(BLE_TEST_MODE)
                if(monocle_is_ble_connected) 
                {
                    tx_en_queue(false);
#ifdef BLE_ON
                    monocle_discard_on_resume = false; // with real BLE tranfer, will be discarded by tx_continue()
#endif
                }
#endif
                monocle_state = MONOCLE_PHOTO_D1;
            } else if(MONOCLE_EVENT_G_DOUBLE_TAP == event) { // turn OLED on for 20s, replay
                NRF_LOG_INFO("State: RECORD_D1 -> REPLAY_D1.");
                state_timer_restart(STATE_TIMEOUT_DISPLAYING_REPLAY);
                fpga_video_capture();
#if defined(DISPLAY_ON) && !defined(DISPLAY_ALWAYS_ON)
                oled_pwr_wake();
#endif
                sleep_camera();
//#ifdef MIC_ON
//                fpga_mic_off();
//#endif
#if defined(BLE_ON) || defined(BLE_TEST_MODE)
                if(monocle_is_ble_connected) 
                {
                    tx_en_queue(false);
#ifdef BLE_ON
                    monocle_discard_on_resume = false; // will be discarded by tx_continue()
#endif
                }
#endif
                monocle_state = MONOCLE_REPLAY_D1;
            } else {
                //ignore event
            }
            break;
        case MONOCLE_PHOTO_D1: // ==================== in MONOCLE_PHOTO_D1 ====================
            if((MONOCLE_EVENT_G_TAP == event || MONOCLE_EVENT_TIMEOUT == event) && (!tx_is_full())) {
                NRF_LOG_INFO("State: PHOTO_D1 -> RECORD_D0 (home).");
                state_timer_restart(STATE_TIMEOUT_RECORDING);
                if(monocle_display_state != MONOCLE_DISP_CAM) { // restore display mode
                    monocle_display_state = MONOCLE_DISP_CAM;
                    fpga_disp_live();
                }
#ifdef CAMERA_ON
                wakeup_camera();
#endif
#if defined(DISPLAY_ON) && !defined(DISPLAY_ALWAYS_ON)
                oled_pwr_sleep();
#endif
                fpga_resume_live_video();
                monocle_state = MONOCLE_RECORD_D0;
            } else if ((MONOCLE_EVENT_TIMEOUT == event) && (tx_is_full())) {
                NRF_LOG_INFO("State: PHOTO_D1 -> PHOTO_D0.");
#ifdef DISPLAY_ON
                oled_pwr_sleep();
#endif                
                monocle_state = MONOCLE_PHOTO_D0;
            } else if (MONOCLE_EVENT_G_LONGPRESS == event) { // for testing, cycle through display modes
                switch(monocle_display_state)
                {
                    case MONOCLE_DISP_CAM:
                        monocle_display_state = MONOCLE_DISP_BARS;
                        fpga_disp_bars();
                        NRF_LOG_INFO("State: IMAGE_CAPTURE, display = color bars.");
                        break;
                    case MONOCLE_DISP_BARS:
                        monocle_display_state = MONOCLE_DISP_BUSY;
                        fpga_disp_busy();
                        NRF_LOG_INFO("State: IMAGE_CAPTURE, display = busy screen (grey).");
                        break;
                    case MONOCLE_DISP_BUSY:
                        monocle_display_state = MONOCLE_DISP_CAM;
                        fpga_disp_live();
                        NRF_LOG_INFO("State: IMAGE_CAPTURE, display = normal.");
                        break;
                }
            } else if (MONOCLE_EVENT_BLE_CONNECT == event) {
                // TODO begin transfer
            } else {
                // ingnore event
            }
            break;
        case MONOCLE_PHOTO_D0: // ==================== in MONOCLE_PHOTO_D0 ====================
            if(EVENT_IS_GESTURE(event)) {
                NRF_LOG_INFO("State: PHOTO_D0 -> PHOTO_D1.");
#ifdef DISPLAY_ON
                oled_pwr_wake();
#endif
                state_timer_restart(STATE_TIMEOUT_DISPLAYING_SHORT);
                monocle_state = MONOCLE_PHOTO_D1;
            } else if(MONOCLE_EVENT_BLE_TRANSFER_DONE == event) {
                NRF_LOG_INFO("State: PHOTO_D0 -> RECORD_D0 (home).");
                state_timer_restart(STATE_TIMEOUT_RECORDING);
                if(monocle_display_state != MONOCLE_DISP_CAM) { // restore display mode
                    monocle_display_state = MONOCLE_DISP_CAM;
                    fpga_disp_live();
                }
#ifdef CAMERA_ON
                wakeup_camera();
#endif
                fpga_resume_live_video();
                monocle_state = MONOCLE_RECORD_D0;
            } else {
                // ignore event
            }
            break;
        case MONOCLE_REPLAY_D1: // ==================== in MONOCLE_REPLAY_D1 ====================
            if (((MONOCLE_EVENT_G_TAP == event || MONOCLE_EVENT_TIMEOUT == event) && (!tx_is_full())) || (MONOCLE_EVENT_BLE_TRANSFER_DONE == event)) {
                NRF_LOG_INFO("State: REPLAY_D1 -> RECORD_D0 (home).");
                state_timer_restart(STATE_TIMEOUT_RECORDING);
#ifdef CAMERA_ON
                wakeup_camera();
#endif
#if defined(DISPLAY_ON) && !defined(DISPLAY_ALWAYS_ON)
                oled_pwr_sleep();
#endif
                fpga_resume_live_video();
                replay_speed(0); // reset to default 1x speed for next time we enter Replay
                monocle_state = MONOCLE_RECORD_D0;
            } else if ((MONOCLE_EVENT_TIMEOUT == event) && (tx_is_full())) {
                NRF_LOG_INFO("State: REPLAY_D1 -> REPLAY_D0.");
#ifdef DISPLAY_ON
                oled_pwr_sleep();
#endif                
                monocle_state = MONOCLE_REPLAY_D0;
            } else if (MONOCLE_EVENT_G_PRESS == event) {
                replay_speed(+1); //toggle replay speed
            } else if (MONOCLE_EVENT_BLE_CONNECT == event) {
                // TODO begin transfer
            } else {
                // ingnore event
            }
            break;
        case MONOCLE_REPLAY_D0: // ==================== in MONOCLE_REPLAY_D0 ====================
            if (EVENT_IS_GESTURE(event)) {
                NRF_LOG_INFO("State: REPLAY_D0 -> REPLAY_D1.");
#if defined(DISPLAY_ON) && !defined(DISPLAY_ALWAYS_ON)
                oled_pwr_wake();
#endif
                state_timer_restart(STATE_TIMEOUT_DISPLAYING_REPLAY);
                monocle_state = MONOCLE_REPLAY_D1;
            } else if (MONOCLE_EVENT_BLE_TRANSFER_DONE == event) {
                NRF_LOG_INFO("State: REPLAY_D0 -> RECORD_D0 (home).");
                state_timer_restart(STATE_TIMEOUT_RECORDING);
#ifdef CAMERA_ON
                wakeup_camera();
#endif
                fpga_resume_live_video();
                monocle_state = MONOCLE_RECORD_D0;
            } else {
                // ignore event
            }
            break;

        case MONOCLE_STANDBY_WARN: // ==================== in MONOCLE_STANDBY_WARN ====================
            if (EVENT_IS_GESTURE(event)) {
                NRF_LOG_INFO("State: STANDBY_WARN -> RECORD_D1.");
                state_timer_restart(STATE_TIMEOUT_DISPLAYING_SHORT);
                fpga_disp_live();
                monocle_state = MONOCLE_RECORD_D1;
            } else if (MONOCLE_EVENT_TIMEOUT == event) {
                NRF_LOG_INFO("State: STANDBY_WARN -> STANDBY.");
                state_timer_restart(STATE_TIMEOUT_STANDBY);

                if (shut_me_down == false)
                {
#ifdef DISPLAY_ON
                    oled_pwr_sleep();
#endif
                    fpga_disp_live();
#ifdef CAMERA_ON
                    sleep_camera();
#endif
                }

                monocle_state = MONOCLE_STANDBY;
            } else {
                // ignore event
            }
            break;

        case MONOCLE_STANDBY: // ==================== in MONOCLE_STANDBY ====================
            if (EVENT_IS_GESTURE(event)) {
                NRF_LOG_INFO("State: STANDBY -> RECORD_D1.");
                state_timer_restart(STATE_TIMEOUT_DISPLAYING_SHORT);
#ifdef CAMERA_ON
                wakeup_camera();
#endif
                fpga_resume_live_video();
#ifdef DISPLAY_ON
                oled_pwr_wake();
#endif
                monocle_state = MONOCLE_RECORD_D1;
            } else if (MONOCLE_EVENT_TIMEOUT == event) {
                if ((shut_me_down == true) || (!tx_is_transferring())) {
                    NRF_LOG_INFO("State: STANDBY -> POWERED_DOWN due to standby timeout.");
                    monocle_state = MONOCLE_POWERED_DOWN;
                    // bsp_board_aux_power_off() will be called by shutdown handler
                    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF); // shut down until touched
                    // execution will not reach here
                }
                else {
                    NRF_LOG_INFO("State: STANDBY -> SHUTDOWN_PENDING due to standby timeout.");
                    monocle_state = MONOCLE_SHUTDOWN_PENDING;
                }
            } else {
                // ignore event
            }
            break;

        default:
            NRF_LOG_ERROR("event_handler(): undefined state.");
    }
}

// send gesture to event_handler()
void gesture_handler(touch_gesture_t gesture)
{
    switch(gesture)
    {
        case TOUCH_GESTURE_TAP:
            event_handler(MONOCLE_EVENT_G_TAP);
            break;
        case TOUCH_GESTURE_DOUBLETAP:
            event_handler(MONOCLE_EVENT_G_DOUBLE_TAP);
            break;
        case TOUCH_GESTURE_PRESS:
            event_handler(MONOCLE_EVENT_G_PRESS);
            break;
        case TOUCH_GESTURE_LONGPRESS:
            event_handler(MONOCLE_EVENT_G_LONGPRESS);
            break;
#ifdef TOUCH_CAPSENSE
        case TOUCH_GESTURE_LONGBOTH:
            event_handler(MONOCLE_EVENT_G_LONGBOTH);
            break;
#endif
        default :
            ; // ignore other gestures
    }
}

// ===== BLE related functions =====
#ifdef BLE_ON
void dump_diagnostic_log(void);

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


// Custom declaration of app_error_fault_handler, overwrites the default declaration.
// Function is called due to APP_ERROR_CHECK() not due to SWL Library
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    // Error has occurred
    NRF_LOG_INFO("Error has occurred!");
    NRF_LOG_INFO("Error ID: %u", id);
    NRF_LOG_INFO("Error PC: %u", pc);
    NRF_LOG_INFO("Error Info: %u", info);

    __disable_irq();
    NRF_LOG_FINAL_FLUSH();

    switch (id)
    {
        case NRF_FAULT_ID_SD_ASSERT:
            NRF_LOG_ERROR("SOFTDEVICE: ASSERTION FAILED");
            break;

        case NRF_FAULT_ID_APP_MEMACC:
            NRF_LOG_ERROR("SOFTDEVICE: INVALID MEMORY ACCESS");
            break;

        case NRF_FAULT_ID_SDK_ASSERT:
            {
                assert_info_t * p_info = (assert_info_t *)info;
                NRF_LOG_ERROR("ASSERTION FAILED at %s:%u",
                              p_info->p_file_name,
                              p_info->line_num);
            }
            break;

        case NRF_FAULT_ID_SDK_ERROR:
            {
                error_info_t * p_info = (error_info_t *)info;
                NRF_LOG_ERROR("ERROR 0x%08x [%s] at %s:%u\r\nPC at: 0x%08x",
                              p_info->err_code,
                              nrf_strerror_get(p_info->err_code),
                              p_info->p_file_name,
                              p_info->line_num,
                              pc);
                 NRF_LOG_ERROR("End of error report");
            }
            break;

        default:
            NRF_LOG_ERROR("UNKNOWN FAULT at 0x%08X", pc);
            break;
    }

    // retrieve and print out swl_periph diagnostic log raw data
    dump_diagnostic_log();

    // Trigger breakpoint if debugger is connected
    SWL_BKPT;

    // it is recommend to soft reset the system as it may be in an unreliable state due to the unknown error
    sd_nvic_SystemReset();

    // infinite loop; code should never reach here
    for(;;)
    {}
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    // forward BLE stack events to the swl_periph event handler
    swl_periph_nrf_process_stack_evt(p_ble_evt, p_context);
}

static void ble_periph_evt_handler(swl_periph_evt_t evt, const swl_periph_evt_data_t * p_evt_data)
{
  swl_err_t err_code;

  if( !(evt == SWL_PERIPH_EVT_DATA_SENT) )
  {
      NRF_LOG_INFO("SWL Event: %s", swl_periph_evt_desc[(uint8)evt]);
  }

  // process events
  switch(evt)
  {
    case SWL_PERIPH_EVT_CONN_ESTABLISHED:
      NRF_LOG_INFO("Device ID: %u", p_evt_data->conn_establish.device_id);
      event_handler(MONOCLE_EVENT_BLE_CONNECT);
      break;

    case SWL_PERIPH_EVT_CONN_TERMINATED:
      NRF_LOG_INFO("Device ID: %u", p_evt_data->conn_terminate.device_id);

      if(!swl_in_pairing_mode)
      {
        err_code = swl_periph_enter_pairing_mode(SWL_PAIRING_TIMEOUT_DEFAULT);
        SWL_ERROR_CHECK(err_code);

        swl_in_pairing_mode = true;
      }

      //TODO clean up bluetooth transmission state variables, in case terminated in middle of transfer
      event_handler(MONOCLE_EVENT_BLE_DISCONNECT);
      break;

    case SWL_PERIPH_EVT_RX_DATA:
      //TODO in future: handle data sent from phone App
      break;

    case SWL_PERIPH_EVT_DATA_ACK:
      //data was acknowledged by peer device.
      NRF_LOG_INFO("Major: %u Minor: %u", p_evt_data->data_info.id_hdr_major, p_evt_data->data_info.id_hdr_minor);
      break;

    case SWL_PERIPH_EVT_DATA_SENT:
      //Event generated when data transfer has been completely queued up by the Bluetooth stack
      //Additional data can be queued up once this event is recieved.
      if(tx_is_transferring()) tx_continue();
      break;

    case SWL_PERIPH_EVT_PWR_MODE_UPDATE:
      //Power mode has been successfully changed
      break;

    case SWL_PERIPH_EVT_PWR_MODE_UPDATE_REQUEST:
      //Request to change power mode has be recieved.
      break;

    case SWL_PERIPH_EVT_PAIRING_MODE_EXITED:
      //Event generated when pairing mode has exited
       NRF_LOG_INFO("Device ID: %u Reason: %u", p_evt_data->pairing_mode_exited.device_id, p_evt_data->pairing_mode_exited.swl_pairing_mode);

       //Check whether this event was generated because the pairing mode timer expired.
       if((p_evt_data->pairing_mode_exited.swl_pairing_mode == SWL_PAIRING_MODE_TIMER_EXPIRED) &&
          (monocle_is_ble_connected == false) )
       {
          //Enter pairing mode. Temp. workaround.
          err_code = swl_periph_enter_pairing_mode(SWL_PAIRING_TIMEOUT_DEFAULT);
          SWL_ERROR_CHECK(err_code);

          swl_in_pairing_mode = true;
       }
       else
       {
          swl_in_pairing_mode = false;
       }
      break;

    default:
      // unexpected event;
      break;
  }
}

void dump_diagnostic_log(void)
{
    uint8 * p_diag_log;

    // retrieve diagnostic log
    swl_periph_get_diagnostic_log(&p_diag_log);
    
    if(p_diag_log != NULL)
    {
        NRF_LOG_INFO("SwaraLink Peripheral Middleware - Diagnostic log dump:");

        // print out diagnostic log raw data
        for(uint16 i = 0; i < SWL_PERIPH_DIAGNOSTICS_LOG_BUFFER_SIZE; i++)
        {
            // print byte from diagnostic log
            NRF_LOG_INFO("index: %04d, data (hex): %02x", i, *p_diag_log);
        
            // increment pointer by 1 byte
            p_diag_log++;
        }
    }
    else
    {
        NRF_LOG_INFO("SwaraLink Peripheral Middleware - Diagnostic log not found!");
    }
}

void ble_exception_handler(const swl_periph_exception_data_t * p_exception_data)
{
    // exception occurred in swl_periph library 
    
    if(p_exception_data != NULL)
    {    
        NRF_LOG_INFO("swl_periph exception occurred! Please take note of the following information and report to SwaraLink Technologies:");
        NRF_LOG_INFO("Exception filename code:      %u", p_exception_data->filename_code);
        NRF_LOG_INFO("Exception line number:        %u", p_exception_data->line_number);
        NRF_LOG_INFO("Exception metadata A:         %u", p_exception_data->metadata_a);
        NRF_LOG_INFO("Exception metadata B:         %u", p_exception_data->metadata_b);
    }

    // retrieve and print out diagnostic log raw data
    dump_diagnostic_log();

    // Trigger breakpoint if debugger is connected
    SWL_BKPT;

    // for in-field exceptions, it is recommend to soft reset the system as the swl_periph library may be in an unreliable state
    sd_nvic_SystemReset();

    // infinite loop; code should never reach here
    for(;;)
    {}
}

static void ble_initialize_swl_periph_middleware(void)
{
    ret_code_t err_code;
    swl_err_t  swl_err_code;

    // Initialize app_timer module
    // The app_timer module is required by the SwaraLink Peripheral middleware. It can 
    // also be used by the application; however, the application must ensure that it
    // does not interfere with the operation of any timers that it did not create.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Initialize Nordic's crypto library
    // The nrf_crypto module is required by the SwaraLink Peripheral middleware. It can
    // also can be used by the application; however the application must not 
    // uninitialize it at any time.
    err_code = nrf_crypto_init();
    APP_ERROR_CHECK(err_code);

    // Configure buffers for the SwaraLink Peripheral middleware
    swl_err_code = swl_periph_config();
    SWL_ERROR_CHECK(swl_err_code);

    // Initialize SwaraLink peripheral module; register event handlers
    swl_err_code = swl_periph_init(ble_periph_evt_handler, ble_exception_handler);
    SWL_ERROR_CHECK(swl_err_code);
    
    // Enter pairing mode. Must be called after swl_periph_init().
    // Peer must successfully connect within the given time frame.
    swl_err_code = swl_periph_enter_pairing_mode(SWL_PAIRING_TIMEOUT_DEFAULT);
    SWL_ERROR_CHECK(swl_err_code);    

    swl_in_pairing_mode = true;
}
#endif
// ===== (end) BLE related functions (end) =====

/**
 * @brief Watch Dog Timer events handler.
 */
void wdt_event_handler(void)
{
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
    bsp_board_aux_power_off(); // turn off aux power
    monocle_state = MONOCLE_POWERED_DOWN;
    //NRF_LOG_ERROR("Watchdog Timer triggered. Rebooting..."); // with deferred logging, this won't get executed
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
#ifdef BLE_ON
    // process BLE pairing request using SWL stack
    // this calls nrf_ble_lesc_request_handler() internally
    swl_periph_nrf_process_idle_state();
#endif

#ifdef WATCHDOG_ON
    //feed watchdog timer
    nrf_drv_wdt_channel_feed(m_channel_id);
#endif

    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
        //NRF_LOG_FLUSH(); // for ADC data NOTE: this can only be called from main context (not interrupt handler)
    }
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint8_t reg_value = 0;
    uint16_t reg = 0;
    bool success = false;
    uint32_t err_code = NRF_SUCCESS;
    uint8_t retry = 0;
    uint8_t self_test_pass = 0; // count how many stages of the power-on self-test have passed

    // check recovery pin, busy loop if activated to guarantee debugging access
    bsp_board_check_recovery();

    // ===== Early Initialization =====
    //TODO Haven't been able to measure a difference between enabled & not -- measure if necessary (ie external inductors)
    //WARNING if external inductors are not present, this could hang, and not be recoverable
    //NRF_POWER->DCDCEN = 1; // enable DC/DC power supply (requires external inductors, which are present in MK9B, MK10)
    log_init();

#ifdef TEST6
    success = unit_test_6a();
    if(success) NRF_LOG_INFO("TEST6: Checksums passed.");
    APP_HALT_ON_ERROR(!success, "TEST6: Checksums did not match expected values."); 
#endif

    NRF_LOG_INFO("===== Monocle booting =====");
    NRF_LOG_INFO("Hardware: %s", MK_HW_TEXT);
    NRF_LOG_INFO("MCU Firmware: %d.%d", g_mcu_ver_major, g_mcu_ver_minor);
    bsp_board_init(); // initialize the GPIOs not initialized elsewhere
        // sets en_pwr low (0V) so peripheral power is off
        // sets disp_XCLR low (0V) before display is powered on
        // MK10: IO_PMIC_ACTIVE=HI so I2C communication will succeed; MUST turn off later to enable charging
        // MK11: set spi_fpga_CS = MODE1 LOW for AUTO BOOT (from FPGA internal flash)
    ov5640_init(); // set camera control pins to initial state
    touch_quick_init();  // quick setup of touched_N pin, to be overwritten by touch_init() later
    adc_quick_init();
    power_management_init();

    // log the reason for this wakeup
    power_log_reset_reason();

    // check battery status, sleep if low
    float boot_voltage = 0;
#ifndef BOARD_MK11 // for MK9B & MK10, which require only GPIO to control aux power
    boot_voltage = adc_quick_get_batt_voltage();
    NRF_LOG_INFO("Boot voltage: " NRF_LOG_FLOAT_MARKER ".", NRF_LOG_FLOAT(boot_voltage));
#ifndef IGNORE_BATT_LEVEL
    if ( (boot_voltage <= (BATTERY_LEVEL_TURN_OFF + BATTERY_BOOT_HYSTERESIS)) || (boot_voltage > BATTERY_LEVEL_CHARGING) ) // go back to sleep
    {
        NRF_LOG_INFO("Battery %s, power down until next touch.", (boot_voltage <= BATTERY_LEVEL_EMERGENCY_OFF)?"LOW":"CHARGING");
        // go to sleep until touched
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
#endif
// for MK11 must do later, because I2C is not initialized to talk to PMIC in order to turn off aux power
#endif
    NRF_LOG_INFO("===== Monocle Early Init complete =====");

    // ===== Full Initialization =====
    monocle_state = MONOCLE_FULL_INIT;

#ifdef BLE_ON
    // initialize BLE stack using SwaraLink library
    //NOTE in previous SWL release, must do this first, since it initialized app_timer internally
    //TODO in this SWL release, app_timer config is done in ble_initialize_swl_periph_middleware(), so it could be moved
    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    ble_initialize_swl_periph_middleware();
    NRF_LOG_INFO("SWL BLE stack started.");
#endif

#if defined(BLE_ON) || defined(BLE_TEST_MODE)
    // initialize tx module
    tx_init();
#endif

    lfclk_start();
    timers_init(); // BLE, if on, must be initialized before this

#ifndef BOARD_MK11
    adc_quick_uninit();
    adc_init();
#endif
    i2c_init(); // init main I2C bus; secondary bus will be initialized after auxiliary power turned on
    // spi will be initialized after auxiliary power turned on
    led_init();

//NOTE: for power TESTING pause here (to catch reboots)
    //bool debugger_pause = true;
    //while(debugger_pause);

#ifdef WATCHDOG_ON
    // initialize watchdog. Per sdk_config.h WDT_CONFIG_RELOAD_VALUE, must be fed within 20 seconds (this limits boot time)
    nrf_drv_wdt_config_t wdt_config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&wdt_config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
    NRF_LOG_INFO("Watchdog timer initialized, %d seconds.", WDT_CONFIG_RELOAD_VALUE/1000);
#endif


    // === PMIC ===
    // --- init PMIC ---
#ifdef BOARD_MK10
    // initialize BQ25125, set to load-switch mode
    // NOTE if running on battery, will be in active mode & I2C communication enabled
    // NOTE with 5V power, by default I2C is in Hi-Z mode, bsp_board_init() sets pin IO_PMIC_ACTIVE to High to force active
    success = bq25125_init();
    APP_HALT_ON_ERROR(!success, "PMIC BQ25125 init failed.");
    success = bq25125_set_charge_current(50); // increase charging current to 50mA (0.8C)
    APP_HALT_ON_ERROR(!success, "BQ25125 charge current FAILED to set.");
    bq25125_set_charge_voltage(4300); // increase charging voltage to 4.3V = 4300mV (Grepow battery supports 4.35 +/- 0.03V)
    APP_HALT_ON_ERROR(!success, "BQ25125 charge voltage FAILED to set.");
    bq25125_set_current_limit(200); // increase input current limit to 200mA
    APP_HALT_ON_ERROR(!success, "BQ25125 input current limit FAILED to set.");
    bsp_board_io_off(IO_PMIC_ACTIVE); // turn off active mode so that charging will be enabled
#endif
#ifdef BOARD_MK11
    success = max77654_init();
    APP_HALT_ON_ERROR(!success, "PMIC MAX77654 init failed.");

    // check power status, as is done earlier for MK9B & MK10
    boot_voltage = adc_quick_get_batt_voltage();
    NRF_LOG_INFO("Boot voltage: " NRF_LOG_FLOAT_MARKER ".", NRF_LOG_FLOAT(boot_voltage));
    adc_quick_uninit();
    adc_init();
#ifndef IGNORE_BATT_LEVEL
    if ( (boot_voltage <= (BATTERY_LEVEL_TURN_OFF + BATTERY_BOOT_HYSTERESIS)) || (boot_voltage > BATTERY_LEVEL_CHARGING) ) // go back to sleep
    {
        NRF_LOG_INFO("Battery %s, power down until next touch.", (boot_voltage <= BATTERY_LEVEL_EMERGENCY_OFF)?"LOW":"CHARGING");
        // go to sleep until touched
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
#endif

    // force power cycle, in case rails were on (possible in testing scenarios)
    bsp_board_aux_power_off();
//    //NOTE TESTING: KEEP AUX POWER OFF
//    while(1);
    // enable LEDs to indicate self-test status
    max77654_rail_vled_on(true);
    led_green_on(); // Green LED: keep on until end of initialization
    // NOTE MK9B and MK10 turn on LED later, since it depends on FPGA
#endif
    self_test_pass++;
    NRF_LOG_INFO("=== Self-Test %d PASS: PMIC ===", self_test_pass);


    // --- turn on auxiliary power rails ---
#ifdef BOARD_MK11
    bsp_board_aux_power_on(); // power-on sequence
#else // MK9B and MK10
    // power cycle was forced by initialization of EN_PWR pin in ???.c
    bsp_board_aux_power_on(); // turn on 1.2V, 1.8V_s, 2.8V, 10V power
    nrf_delay_ms(2); // TODO check rise time // necessary? there is already a delay in _aux_power_on()
#endif
#if LEDS_NUMBER > 0
    bsp_board_led_on(0); // turn on LED1 (testing)
#endif
    NRF_LOG_INFO("Monocle peripheral power on.");


//NOTE: for power TESTING pause here (to catch reboots)
    //bool debugger_pause = true;
    //while(debugger_pause);
#if defined(BOARD_MK11) && defined(CAMERA_ON)
    //NOTE Hack to allow MK11 to boot through camera power surge; hope to figure out a way to eliminate this
    NRF_LOG_INFO("MK11 POWER HACK - Delay 1Sec"); // power surge when camera turned on causes brownout reset
    nrf_delay_ms(1000); // from testing seems minimum for 90% reliable boot (& longer will hit watchdog); reason this works is unknown
    NRF_LOG_INFO("POWER - Resume");
#ifdef WATCHDOG_ON
    //feed watchdog timer
    nrf_drv_wdt_channel_feed(m_channel_id);
#endif
#endif

    // initialize the MCU pins on busses connected to auxiliary power
    spi_init(); // in MK11, this pulls MODE1 high (since it is also spi_fpga_CS)
#ifdef TWI_SW_INSTANCE_ID
    i2c_sw_init();
#endif


    // === Touch ===
    // initialize touch, requires I2C & timers to be initialized
    // does not require auxiliary power, but place after to allow power rail testing even if touch fails
    success = touch_init(gesture_handler);
    APP_HALT_ON_ERROR(!success, "Touch init failed.");
    self_test_pass++;
    NRF_LOG_INFO("=== Self-Test %d PASS: Touch ===", self_test_pass);


    // === FPGA ===
#ifdef FPGA_ON
#if defined(BOARD_MK11) && defined(FPGA_HOLD)
    nrf_gpio_pin_clear(SPIM_SS1_PIN); //spi_fpga_CS = MODE1, set back to LOW to allow programming & AUTO BOOT (from FPGA internal flash)
    success = false;
    APP_HALT_ON_ERROR(!success, "FPGA hold for programming.");
#endif
    // software reset of FPGA to put into known state
    fpga_soft_reset();
    // test that reset works as expected (side effects: will turn on XCLK, and then soft-reset)
    success = fpga_test_reset();
    if(!success)
    {
        nrf_gpio_pin_clear(SPIM_SS1_PIN); //spi_fpga_CS = MODE1, set back to LOW to allow programming & AUTO BOOT (from FPGA internal flash)
    }
    APP_HALT_ON_ERROR(!success, "FPGA reset failed.");
#if BOARD_MK10
    fpga_led_off_all(); // handle active low LEDs on MK10
    nrf_delay_ms(350); // so can see following blink
    led_red_on(); // MK10 LED depends on FPGA; red LED: keep on until end of initialization
#elif BOARD_MK9B
    led_red_on(); // MK9B LED depends on FPGA; red LED: keep on until end of initialization
#endif
    self_test_pass++;
    NRF_LOG_INFO("=== Self-Test %d PASS: FPGA ===", self_test_pass);
    fpga_get_version(&g_fpga_ver_major, &g_fpga_ver_minor);
    //TODO add FPGA support for this feature; for now returns version = 0.0
    NRF_LOG_INFO("FPGA Firmware: %d.%d", g_fpga_ver_major, g_fpga_ver_minor);
#endif


    // === RAM Memory ===
    success = fpga_ram_check(); //NOTE for now, only checks for proper initialization
    APP_HALT_ON_ERROR(!success, "RAM initialization failed.");
    self_test_pass++;
    NRF_LOG_INFO("=== Self-Test %d PASS: RAM memory ===", self_test_pass);


    // === Microphone ===
#ifdef MIC_ON
    //TODO add self-test code to FPGA
    success = fpga_mic_on(); // enable mic
    APP_HALT_ON_ERROR(!success, "FPGA mic enable failed.");
    NRF_LOG_INFO("Monocle FPGA mic enabled.");
    self_test_pass++;
    NRF_LOG_INFO("=== Self-Test %d PASS: Mic on ===", self_test_pass);
#endif

    // === Camera ===
    //feed watchdog timer
//    nrf_drv_wdt_channel_feed(m_channel_id);
    // == XCLK should be OFF here ==
#ifdef CAMERA_ON
    success = fpga_xclk_on(); // turn on XCLK
    APP_HALT_ON_ERROR(!success, "Failed to turn on XCLK.");
    // power on camera
    retry = 3;
    success = false;
    do {
        success = ov5640_pwr_on();
        if(!success)
        {
            NRF_LOG_ERROR("Camera power-on failed, retrying...");
            nrf_delay_ms(1000);
        }
        retry--;
    } while(!success && retry>0);
    APP_HALT_ON_ERROR(!success, "Failed to turn on Camera.");

    // PCLK will fail to come up here
//    ov5640_YUV422_mode(); // PCLK will be 10.5MHz after this call
//    ov5640_mode_1x(); // PCLK will be 42MHz after this call
//#ifdef AUTO_FOCUS
//NOTE Strangely, even with a fixed-focus lens, this call seems required.
//     Without it, the image appears ghostly, and possibly with slower frame rate.
//TODO Investigate, check with OV FAE.
    ov5640_focus_init(); //TODO check return value NOTE: this is the slowest init routine
//#endif
    ov5640_light_mode(0);
    ov5640_color_saturation(3);
    ov5640_brightness(4);
    ov5640_contrast(3);
    ov5640_sharpness(33);
#ifdef AUTO_FOCUS
    ov5640_focus_constant(); // continuous auto-focus
    //ov5640_focus_single(); // focus only once (now)
#endif
#if defined(BOARD_MK9B) || defined(BOARD_MK10) || defined (BOARD_MK11)
    ov5640_flip(1);
    // do not mirror
#else
    #error "New hardware: define optics requirements"
    //ov5640_flip(1); // do optics require a flip?
    //ov5640_mirror(1); // do optics require mirroring?
#endif
    //ov5640_test_pattern(1); // generate color bar test pattern
    self_test_pass++;
    NRF_LOG_INFO("=== Self-Test %d PASS: Camera on ===", self_test_pass);
#endif

#ifdef WATCHDOG_ON
    //feed watchdog timer
    nrf_drv_wdt_channel_feed(m_channel_id);
#endif

#ifdef DISPLAY_ON
    // SONY ECX336-CN OLED power-on sequence, see Datasheet section 9
    // 1ms after 1.8V on, device has finished initializing
    nrf_delay_ms(1);
    // set XCLR to high to change to power-saving mode
    bsp_board_io_on(IO_DISP_XCLR); // pin 15, disp_XCLR -- sets high 1.8V
    // MK9B measured: 9ms after en_pwr on, longer than expected! TODO: may want to adjust delays above
    // later register settings can be made, and power-saving turned off via SPI interface

    // NOTE: MCLK must be running at this point, or OLED will not respond to SPI

    // Turn on OLED
    retry = 3;
    success = false;
    do {
        oled_config(); // known to work
//        oled_config_burst(); // 4x faster, TODO: seems to not work correctly
        // verify that OLED is connected & configured
//        success = oled_verify_config_full(); //TODO won't return a consistent result
        success = oled_verify_config();
        if(!success)
        {
            NRF_LOG_ERROR("OLED config failed, retrying...");
            nrf_delay_ms(1000);
        }
        retry--;
    } while (!success && retry>0);
    APP_HALT_ON_ERROR(!success, "Failed to turn on OLED.");

#ifdef DEBUG
    oled_set_luminance(OLED_DIM); //increases display lifetime & easier to take photos during testing
#else
    oled_set_luminance(OLED_HIGH); // 3000 nits for release (NOTE: possible to go higher)
#endif

    self_test_pass++;
    NRF_LOG_INFO("=== Self-Test %d PASS: OLED on ===", self_test_pass);
#endif


#ifdef CAMERA_ON
    success = fpga_camera_on(); // enable camera interface (& keep XCLK enabled!)
    APP_HALT_ON_ERROR(!success, "FPGA camera enable failed.");
    NRF_LOG_INFO("Monocle FPGA camera interface enabled.");
#endif

    NRF_LOG_INFO("===== Monocle Full Init complete =====");
//    //WARNING TESTING
//    fpga_disp_bars();
    monocle_state = MONOCLE_RECORD_D1;
    NRF_LOG_INFO("State: MONOCLE_FULL_INIT -> RECORD_D1.");
    // start the battery, keepalive, and state timers
    // state timer initializes as 3s, to briefly show live video on OLED at startup
    application_timers_start();

//NOTE testing variables, to clean up
    uint32_t total_count = 0;
    bool inject_event = false;
//    monocle_event_t event_to_inject = MONOCLE_EVENT_G_SLIDELR; // for zoom in: 1x->2x->4x->8x->16x->1x
//    monocle_event_t event_to_inject = MONOCLE_EVENT_G_SLIDERL; // for zoom out: 1x->16x->8x->4x->2x->1x
//    monocle_event_t event_to_inject = MONOCLE_EVENT_G_PRESS; // for image capture
    monocle_event_t event_to_inject = MONOCLE_EVENT_G_DOUBLE_TAP; // for video capture

    bool turn_luma_off = false;
    bool force_fpga_zoom = false;
    uint8_t fpga_zoom_level = 8; // valid levels: 1, 2, 4, 8
    bool force_display = false;
    uint8_t display_mode = 1; // 0=off; 1=disp_cam, 2=disp_busy, 3=disp_bars
#ifdef BLE_ON
    bool force_swl_dump = false;
    uint8 * p_diag_log;
#endif
#ifdef BLE_TEST_MODE
    NRF_LOG_INFO("BLE_TEST_MODE set, simulating BLE connection");
    event_handler(MONOCLE_EVENT_BLE_CONNECT);
#endif

    led_green_off(); // turn off red LED to indicate initialization success
    nrf_delay_ms(1000); // turn off so that green blinks can be counted
    led_green_blink_timer(self_test_pass, false); // blink to indicate # of self-tests passed (one blink cycle only)

#ifdef BUILD_PRODUCTION
    max77654_rail_vled_on(false);
#endif

    /* main loop: print any logs; sleep until interrupt */
    while (true)
    {
        idle_state_handle();

        total_count++;

        if(force_fpga_zoom)
        {
            fpga_set_zoom(fpga_zoom_level);  // valid levels: 1, 2, 4, 8
            force_fpga_zoom = false;
        }

        if(turn_luma_off)
        {
            fpga_set_luma(false);
            turn_luma_off = false;
        }

        if(force_display)
        {
            fpga_set_display(display_mode); // 0=off; 1=disp_cam, 2=disp_busy, 3=disp_bars
            force_display = false;
        }

        if(inject_event)
        {
            event_handler(event_to_inject);
            inject_event = false;
        }

#ifdef BLE_ON
        if(force_swl_dump)
        {
            // retrieve and print out the diagnostic log
            swl_periph_get_diagnostic_log(&p_diag_log);

            NRF_LOG_INFO("Diagnostic log dump:");

            for(uint16 i=0; i < SWL_PERIPH_DIAGNOSTICS_LOG_BUFFER_SIZE; i++)
            {
                // print bytes from diagnostic log
                uint8 data = *p_diag_log;
                NRF_LOG_INFO("%02x", data);
        
                // increment pointer by 1
                p_diag_log = p_diag_log + 1;
            }
            force_swl_dump = false;
        }
#endif

#ifdef TEST1
        if (total_count == 30)
        {
            success = unit_test_1(4, 1000); // 4 LEDs, count to 1000
            NRF_LOG_INFO("TEST1: counted to 1000.";
        }
#endif

#ifdef TEST1b
        if ((total_count >= 30) && (total_count <50))
        {
            success = unit_test_1b(1000);
            if (success) NRF_LOG_INFO("TEST1b: SUCCESS: 1000 SPI reads.");
        }
#endif

#ifdef TEST2
        if ((total_count >= 30) && (total_count <40))
        {
            success = unit_test_2(4, 1000); // 4 LEDs, count to 1000
            if (success) NRF_LOG_INFO("TEST2: SUCCESS: 1000 SPI writes & reads.");
        }
#endif

#ifdef TEST3
        if ((total_count >= 30) && (total_count <40))
        {
            success = unit_test_3(100);
            if (success) NRF_LOG_INFO("TEST3: SUCCESS: 100 cycles of 512 SPI read/writes to FPGA.");
        }
#endif

#ifdef TEST4
        if ((total_count >= 30) && (total_count <40))
        {
            success = unit_test_4(100);
            if(success) NRF_LOG_INFO("TEST4: SUCCESS: 100 cycles of 512 SPI read/writes to OLED.", total_count);
            APP_HALT_ON_ERROR(!success, "TEST4: OLED exercise of SPI failed. OLED reconfigured; halting MCU execution.");
            nrf_delay_ms(100);
        }
#endif

#ifdef TEST5a
        if (total_count == 30) success = unit_test_5a();
#endif

#ifdef TEST5b
        if (total_count == 20) success = unit_test_5b(10);
#endif

#ifdef TEST5c
        if (total_count == 20) success = unit_test_5c(20, 252, 120); // parameters: # cycles, long burst size, short burst size
        //if (total_count == 40) success = unit_test_5c(20, 250, 140);
#endif


    }
}

/**
 *@}
 **/
