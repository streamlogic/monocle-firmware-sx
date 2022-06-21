/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file adc.c
 *	@brief Code for ADC interface. MCU hardware dependent.
 *
 *	@author Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug When built in "Release" configuration, SoC is always 0% (in "Debug" configuration it works as expected).
 */

#include "adc.h"
#include "math.h"
#include "nrf_log.h"
#include "sdk_config.h"

// ===== hardware dependent data =====

// Lithium battery discharge curve, modeled from Grepow data for 1C discharge rate
// Requirement: x-values (i.e. voltage) must be stricly increasing
#if defined(BOARD_MK9B) || defined(BOARD_MK10) // for 2 * Grepow GRP2410020
const uint8_t batt_points = 15;
static float batt_v[]   = {3.0, 3.2, 3.3, 3.4,  3.5, 3.55, 3.60, 3.65, 3.70, 3.80, 3.90, 4.05, 4.10, 4.15,  4.20};
static float batt_soc[] = {0.0, 2.0, 3.7, 6.2, 12.0, 17.7, 25.5, 37.0, 48.5, 65.0, 76.0, 89.3, 93.0, 97.0, 100.0};
#elif defined(BOARD_MK11) // for 1 * Varta CP1254A4
const uint8_t batt_points = 13;
static float batt_v[]   = {3.0, 3.3, 3.35,  3.4, 3.43, 3.48, 3.54, 3.64, 3.76, 3.90, 4.02, 4.13,  4.25};
static float batt_soc[] = {0.0, 4.0,  6.0, 12.0, 17.0, 28.0, 39.0, 51.0, 62.0, 74.0, 85.0, 96.0, 100.0};
#else
#error "Define battery discharge curve"
#endif

// Input resistor divider
#if defined(BOARD_MK9B) || defined(BOARD_MK10)
// Assumes: (V measured) |--- R_HI --- (ADC input) --- R_LO ---> GND
#define R_HI 100.0 //kOhm, MK9B R2
#define R_LO  47.0 //kOhm, MK9B R3
#elif defined(BOARD_MK11)
// VSYS full scale is 4.8V, resulting in 1.25V on ADC, so factor is 4.8/1.25
#define R_HI (4.8-1.25)
#define R_LO 1.25 
#else
#error "Define ADC input factor"
#endif

// ===== ADC configuration =====
#if (NRFX_SAADC_CONFIG_RESOLUTION == 1) // 10-bit resolution
#define ADC_RESOLUTION 1024
#else
#error "Define ADC resolution"
#endif

// Depends on R_HI, reference:
// https://infocenter.nordicsemi.com/topic/com.nordic.infocenter.nrf52832.ps.v1.1/saadc.html?cp=4_2_0_36_8#concept_qh3_spp_qr
#define ADC_CONFIG_ACQTIME NRF_SAADC_ACQTIME_10US  // for 100k input resistance (MK9B R2); seems OK for MK11

#define ADC_CONFIG_REFERENCE NRF_SAADC_REFERENCE_VDD4 // VDD=1.8V divided by 4 as reference
#define ADC_REFERENCE (1.8 / 4.0)

#define ADC_CONFIG_GAIN NRF_SAADC_GAIN1_4 // gain 1/4, so input range = VDD (full range)
#define ADC_GAIN (1.0/4.0)

// ===== internal =====

// NOTE: these values are averages, taken over time determined by sampling time (in main code) and SAMPLES_IN_BUFFER
//       they will only be valid after an initial time (currently 5 seconds)
static float battery_voltage = 0; // stores battery voltage, expressed in Volts
static uint8_t battery_soc = 0; // Stores battery state-of-charge, expressed in percent (0-100)

#define SAMPLES_IN_BUFFER 5  // number of ADC samples to average
static nrf_saadc_value_t     adc_single_sample; // for quick mode
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static uint32_t              m_adc_evt_counter;

// for logging
#ifdef ADC_LOG_INFO_ON
#define ADC_LOG_INFO(...) NRF_LOG_INFO(__VA_ARGS__)
#else
#define ADC_LOG_INFO(...) 
#endif

#ifdef ADC_LOG_DEBUG_ON
#define ADC_LOG_DEBUG(...) NRF_LOG_DEBUG(__VA_ARGS__)
#else
#define ADC_LOG_DEBUG(...) 
#endif

// linear interpolation functions from: https://www.electro-tech-online.com/threads/linear-interpolation-and-lookup-tables-c.147507/

/* Structure definition */
struct table_1d {
    uint8_t x_length;

    float *x_values;
    float *y_values;
};

/* Declare variable using above structure and the battery discharge function datapoints */
static struct table_1d batt_table = {
    batt_points, /* Number of data points */
    batt_v,      /* Array of x-coordinates */
    batt_soc     /* Array of y-coordinates */
};

static float interpolate_segment(float x0, float y0, float x1, float y1, float x)
{
    float t = 0;

    if (x <= x0) { return y0; }
    if (x >= x1) { return y1; }

    t =  (x-x0);
    t /= (x1-x0);

    return y0 + t*(y1-y0);
}

static float interpolate_table_1d(struct table_1d *table, float x)
/* 1D Table lookup with interpolation */
{
    uint8_t segment;

    /* Check input bounds and saturate if out-of-bounds */
    if (x > (table->x_values[table->x_length-1])) {
       /* x-value too large, saturate to max y-value */
        return table->y_values[table->x_length-1];
    }
    else if (x < (table->x_values[0])) {
       /* x-value too small, saturate to min y-value */
        return table->y_values[0];
    }

    /* Find the segment that holds x */
    for (segment = 0; segment<(table->x_length-1); segment++)
    {
        if ((table->x_values[segment]   <= x) &&
            (table->x_values[segment+1] >= x))
        {
            /* Found the correct segment */
            /* Interpolate */
            return interpolate_segment(table->x_values[segment],   /* x0 */
                                       table->y_values[segment],   /* y0 */
                                       table->x_values[segment+1], /* x1 */
                                       table->y_values[segment+1], /* y1 */
                                       x);                         /* x  */
        }
    }

    /* Something with the data was wrong if we get here */
    /* Saturate to the max value */
    return table->y_values[table->x_length-1];
}

static float adc_batt_v_to_soc(float voltage)
{
    interpolate_table_1d(&batt_table, voltage);
}

// see: https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.nrf52832.ps.v1.1%2Fsaadc.html
static float adc_convert_to_voltage(nrf_saadc_value_t reading)
{
    float voltage = 0;
    float factor = 0;

    if (reading < 0) reading = 0; // observe -1 to -4 raw ADC readings when input grounded
    factor = ((R_HI + R_LO)/(R_LO)) * (ADC_REFERENCE / (ADC_GAIN * ADC_RESOLUTION));
    voltage = (float) reading * factor;
    return voltage;
}


static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) // sampling buffer is full
    {
        ret_code_t err_code;
        nrf_saadc_value_t average_level= 0;

        // put buffer back into the sampling queue
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        //NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter); // NOT IMPLEMENTED

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            //NRF_LOG_INFO("%d", p_event->data.done.p_buffer[i]);
            average_level += p_event->data.done.p_buffer[i];
        }
        average_level = average_level / (nrf_saadc_value_t) SAMPLES_IN_BUFFER; // this truncates, doesn't round up

        battery_voltage = adc_convert_to_voltage(average_level);
        battery_soc = round(adc_batt_v_to_soc(battery_voltage));
        ADC_LOG_DEBUG("Batt average (ADC raw): %d", average_level);
        ADC_LOG_DEBUG("Batt average (voltage): " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(battery_voltage));
        //NRF_LOG_INFO("Batt SoC: " NRF_LOG_FLOAT_MARKER "%%", NRF_LOG_FLOAT(adc_batt_v_to_soc(battery_voltage)));
        ADC_LOG_DEBUG("Batt SoC: %d%%", battery_soc);
        m_adc_evt_counter++;

        // update Bluetooth BAS
        //battery_level_update(battery_soc);
    }
}

static void saadc_quick_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) // sampling is done
    {
        ret_code_t err_code;
        ADC_LOG_INFO("saadc_quick_callback() called.");
        // nothing to do?
    }
}

// ===== public functions =====

float adc_get_batt_voltage(void)
{
    float ret_value = battery_voltage;
    return ret_value;
}

uint8_t adc_get_batt_soc(void)
{
    uint8_t ret_value = battery_soc;
    return ret_value;
}

void adc_start_sample(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing ADC.
 */
void adc_init(void)
{
    // Note: ADC resolution set in sdk_config.h as 10-bit = 1024

    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(IO_ADC_VBATT);
    channel_config.acq_time = ADC_CONFIG_ACQTIME;
    channel_config.reference = ADC_CONFIG_REFERENCE;
    channel_config.gain = ADC_CONFIG_GAIN;

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

//    adc_calibrate();

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

/*
// code from: https://devzone.nordicsemi.com/f/nordic-q-a/48462/questions-on-saadc-calibration-gain-best-use/208563#208563
nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
while(NRF_SAADC->EVENTS_STOPPED == 0);
NRF_SAADC->EVENTS_STOPPED = 0;
*/

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}

void adc_uninit(void)
{
    nrf_drv_saadc_uninit();
}

/* BROKEN!
// taken from: https://devzone.nordicsemi.com/f/nordic-q-a/39454/calibrating-saadc-on-nrf52832
#define HW_TIMEOUT 10000
void adc_calibrate(void)
{
    nrfx_err_t nrfx_err_code = NRFX_SUCCESS;

    NRF_LOG_INFO("Starting ADC calibration...");

    // Stop ADC
    nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
    NRFX_IRQ_DISABLE(SAADC_IRQn);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);

    // Wait for ADC being stopped.
    bool result;
    NRFX_WAIT_FOR(nrf_saadc_event_check(NRF_SAADC_EVENT_STOPPED), HW_TIMEOUT, 0, result);
    NRFX_ASSERT(result);

    // Start calibration
    NRFX_IRQ_ENABLE(SAADC_IRQn);
    nrfx_err_code = nrfx_saadc_calibrate_offset();
    APP_ERROR_CHECK(nrfx_err_code);
    while(nrfx_saadc_is_busy()){};
    
    // Stop ADC
    nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
    NRFX_IRQ_DISABLE(SAADC_IRQn);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);

    // Wait for ADC being stopped. 
    NRFX_WAIT_FOR(nrf_saadc_event_check(NRF_SAADC_EVENT_STOPPED), HW_TIMEOUT, 0, result);
    NRFX_ASSERT(result);
    
    // Enable IRQ
    NRFX_IRQ_ENABLE(SAADC_IRQn);

// code from: https://devzone.nordicsemi.com/f/nordic-q-a/48462/questions-on-saadc-calibration-gain-best-use/208563#208563
//TODO: should this be here, or in init()? A second call to calibration later will crash!!!
nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
while(NRF_SAADC->EVENTS_STOPPED == 0);
NRF_SAADC->EVENTS_STOPPED = 0;

    NRF_LOG_INFO("ADC calibrated.");
}
*/

void adc_quick_init(void)
{
    // Note: ADC resolution set in sdk_config.h as 10-bit = 1024
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(IO_ADC_VBATT);
    channel_config.acq_time = ADC_CONFIG_ACQTIME;
    channel_config.reference = ADC_CONFIG_REFERENCE;
    channel_config.gain = ADC_CONFIG_GAIN;

    err_code = nrf_drv_saadc_init(NULL, saadc_quick_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
}

void adc_quick_uninit(void)
{
    nrf_drv_saadc_uninit();
}

float adc_quick_get_batt_voltage(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_saadc_sample_convert(0, &adc_single_sample); // blocking ADC call
    APP_ERROR_CHECK(err_code);

    battery_voltage = adc_convert_to_voltage(adc_single_sample);
    battery_soc = round(adc_batt_v_to_soc(battery_voltage));

    return battery_voltage;
}
