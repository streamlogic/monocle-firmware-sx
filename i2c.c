/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file i2c.c
 *	@brief Code for I2C interface. Supports 1 or 2 interfaces. MCU hardware dependent.
 *
 *	@author Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug i2c_scan_bus() doesn't work (but is not used, so not urgent)
 */

#include "i2c.h"
#include "nrf_log.h" //NOTE testing
#include "nrf_assert.h"
#include "nrf_gpio.h"

// internal

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false; // ended, may have been successful, may have been NACK
static volatile bool m_xfer_nack = false; // NACK returned, operation was unsuccessful

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

#ifdef TWI_SW_INSTANCE_ID
static volatile bool m_sw_xfer_done = false;
static volatile bool m_sw_xfer_nack = false;
static const nrf_drv_twi_t m_sw_twi = NRF_DRV_TWI_INSTANCE(TWI_SW_INSTANCE_ID);
#endif

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler();
            }
            m_xfer_nack = false;
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
        case NRF_DRV_TWI_EVT_DATA_NACK:
            m_xfer_nack = true;
            break;
        default:
            break;
    }
    m_xfer_done = true;
}

#ifdef TWI_SW_INSTANCE_ID
void twi_sw_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //data_handler();
            }
            m_sw_xfer_nack = false;
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
        case NRF_DRV_TWI_EVT_DATA_NACK:
            m_sw_xfer_nack = true;
            break;
        default:
            break;
    }
    m_sw_xfer_done = true;
}
#endif

// externally visible functions

// configure I2C driver and return a handle to it
// TODO validate that 400kH speed works & increase to that
void i2c_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_main_config = {
       .scl                = TWI_SCL_PIN,
       .sda                = TWI_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_main_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);

#ifdef TWI_SW_INSTANCE_ID
    // set second I2C bus pins to default state (input, hi-z)
    nrf_gpio_cfg_default(TWI_SW_SCL_PIN);
    nrf_gpio_cfg_default(TWI_SW_SDA_PIN);
    // these pins will later be initialized after auxiliary power is on, by calling i2c_sw_init()
#endif
}

void i2c_uninit(void)
{
    nrf_drv_twi_disable(&m_twi);
    nrf_drv_twi_uninit(&m_twi);

    // SDK docs unclear on whether this is done by twi_uninit(); can't hurt (but did not make a measurable difference)
    nrf_gpio_cfg_default(TWI_SCL_PIN);
    nrf_gpio_cfg_default(TWI_SDA_PIN);

    // errata 89, see https://infocenter.nordicsemi.com/index.jsp?topic=%2Ferrata_nRF52832_Rev3%2FERR%2FnRF52832%2FRev3%2Flatest%2Fanomaly_832_89.html&cp=4_2_1_0_1_26
    ASSERT(TWI_INSTANCE_ID == 0);
    *(volatile uint32_t *)0x40003FFC = 0;
    *(volatile uint32_t *)0x40003FFC;
    *(volatile uint32_t *)0x40003FFC = 1;
    // NOTE this did not make a measurable difference
}


// for debugging, expect MBR3@55=0x37 & OV5640@60=0x3C
// this generates a lot of false positives, and didn't detect @55...
// TODO debug later
bool i2c_scan_bus(void)
{
    ret_code_t err_code;
    uint8_t address;
    uint8_t sample_data;
    bool detected_device = false;

    for (address = 1; address <= 127; address++)
    {
        err_code = nrf_drv_twi_rx(&m_twi, address, &sample_data, sizeof(sample_data));
        if (err_code == NRF_SUCCESS)
        {
            detected_device = true;
            NRF_LOG_INFO("I2C device detected at address 0x%x.", address);
        }
        //NRF_LOG_FLUSH();
    }
    return detected_device;
}


bool i2c_write(uint8_t slaveAddress, uint8_t *writeBuffer, uint8_t numberOfBytes)
{
    ret_code_t err_code;

    m_xfer_done = false;
    // Issue a non-blocking write, with STOP condition at end
    err_code = nrf_drv_twi_tx(&m_twi, slaveAddress, writeBuffer, numberOfBytes, false);
    APP_ERROR_CHECK(err_code);
    // wait until transfer complete or NACK received
    while (m_xfer_done == false);
    if(m_xfer_nack) {
        return false;
    } else {
        return (err_code == NRF_SUCCESS);
    }
}


bool i2c_write_no_stop(uint8_t slaveAddress, uint8_t *writeBuffer, uint8_t numberOfBytes)
{
    ret_code_t err_code;

    m_xfer_done = false;
    // Issue a non-blocking write, without STOP condition at end
    err_code = nrf_drv_twi_tx(&m_twi, slaveAddress, writeBuffer, numberOfBytes, true);
    APP_ERROR_CHECK(err_code);
    // wait until transfer complete or NACK received
    while (m_xfer_done == false);
    if(m_xfer_nack) {
        return false;
    } else {
        return (err_code == NRF_SUCCESS);
    }
}


bool i2c_read(uint8_t slaveAddress, uint8_t *readBuffer, uint8_t numberOfBytes)
{
    ret_code_t err_code;

    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, slaveAddress, readBuffer, numberOfBytes);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    if(m_xfer_nack) {
        return false;
    } else {
        return (err_code == NRF_SUCCESS);
    }
}


#ifdef TWI_SW_INSTANCE_ID
void i2c_sw_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_sw_config = {
       .scl                = TWI_SW_SCL_PIN,
       .sda                = TWI_SW_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_sw_twi, &twi_sw_config, twi_sw_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_sw_twi);
}


void i2c_sw_uninit(void)
{
    nrf_drv_twi_disable(&m_sw_twi);
    nrf_drv_twi_uninit(&m_sw_twi);

    // SDK docs unclear on whether this is done by twi_uninit(); can't hurt (but did not make a measurable difference)
    nrf_gpio_cfg_default(TWI_SW_SCL_PIN);
    nrf_gpio_cfg_default(TWI_SW_SDA_PIN);

    // errata 89, see https://infocenter.nordicsemi.com/index.jsp?topic=%2Ferrata_nRF52832_Rev3%2FERR%2FnRF52832%2FRev3%2Flatest%2Fanomaly_832_89.html&cp=4_2_1_0_1_26
    ASSERT(TWI_SW_INSTANCE_ID == 1);
    *(volatile uint32_t *)0x40004FFC = 0;
    *(volatile uint32_t *)0x40004FFC;
    *(volatile uint32_t *)0x40004FFC = 1;
    // NOTE this did not make a measurable difference
}


bool i2c_sw_write(uint8_t slaveAddress, uint8_t *writeBuffer, uint8_t numberOfBytes)
{
    ret_code_t err_code;

    m_sw_xfer_done = false;
    // Issue a non-blocking write, with STOP condition at end
    err_code = nrf_drv_twi_tx(&m_sw_twi, slaveAddress, writeBuffer, numberOfBytes, false);
    APP_ERROR_CHECK(err_code);
    // wait until transfer complete or NACK received
    while (m_sw_xfer_done == false);
    if(m_sw_xfer_nack) {
        return false;
    } else {
        return (err_code == NRF_SUCCESS);
    }
}


bool i2c_sw_read(uint8_t slaveAddress, uint8_t *readBuffer, uint8_t numberOfBytes)
{
    ret_code_t err_code;

    m_sw_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_sw_twi, slaveAddress, readBuffer, numberOfBytes);
    APP_ERROR_CHECK(err_code);
    while (m_sw_xfer_done == false);
    if(m_sw_xfer_nack) {
        return false;
    } else {
        return (err_code == NRF_SUCCESS);
    }
}
#endif