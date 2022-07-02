/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file spi.c
 *	@brief Code for dual SPI interface. MCU hardware dependent.
 *
 *	@author Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */

#include "spi.h"
#include "nrf_log.h"

// internal

static const nrfx_spim_t m_spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static uint8_t m_spi_cs_pin; // keep track of current CS pin
static volatile bool m_spi_xfer_done = true;  /**< Flag used to indicate that SPI instance completed the transfer. */

//static uint8_t       m_tx_buf[sizeof(SPI_MAX_BURST_LENGTH) + 1];  /**< TX buffer. */
//static uint8_t       m_rx_buf[sizeof(SPI_MAX_BURST_LENGTH) + 1];  /**< RX buffer. */
static uint8_t       m_tx_buf[SPI_MAX_BURST_LENGTH + 1];  /**< TX buffer. */
static uint8_t       m_rx_buf[SPI_MAX_BURST_LENGTH + 1];  /**< RX buffer. */

/**
 * @brief SPI event handler
 */
void spim_event_handler(nrfx_spim_evt_t const * p_event,
                       void *                  p_context)
{
    // NOTE: there is only one event type: NRFX_SPIM_EVENT_DONE, so no need for case statement
    nrf_gpio_pin_set(m_spi_cs_pin);
    m_spi_xfer_done = true;
}

// externally visible functions

void spi_init(void)
{
    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_1M;
    //spi_config.ss_pin         = SPIM0_SS_PIN;
    spi_config.ss_pin         = NRFX_SPIM_PIN_NOT_USED;
    spi_config.miso_pin       = SPIM_MISO_PIN;
    spi_config.mosi_pin       = SPIM_MOSI_PIN;
    spi_config.sck_pin        = SPIM_SCK_PIN;
    spi_config.mode           = NRF_SPIM_MODE_3;
    spi_config.orc            = 0xFF; // Over Read Character: if RX length > TX length, extra transmitted bytes will be ORC
                                      // if TX length > RX length, extra received bytes ignored
    spi_config.bit_order      = NRF_SPIM_BIT_ORDER_LSB_FIRST;
    spi_config.ss_active_high = false;
    APP_ERROR_CHECK(nrfx_spim_init(&m_spi, &spi_config, spim_event_handler, NULL));

    // configure CS pins (for active low)
    nrf_gpio_pin_set(SPIM_SS0_PIN); //spi_disp_CS
    nrf_gpio_cfg_output(SPIM_SS0_PIN);
#ifdef BOARD_MK11
    nrf_gpio_pin_set(SPIM_SS1_PIN); //spi_fpga_CS = MODE1, now set HIGH for use as SPI_CS
    //  nrf_gpio_cfg_output() already called in board.c, bsp_board_init() 
    nrf_gpio_pin_set(SPIM_SS2_PIN); //SPI_FLASH_CS
    nrf_gpio_cfg_output(SPIM_SS2_PIN); // for now, pull high to disable external flash chip
#else // for MK9B & MK10, FPGA_CS should be set high from the beginning
    nrf_gpio_pin_set(SPIM_SS1_PIN); //spi_fpga_CS = HIGH for use as SPI_CS
    nrf_gpio_cfg_output(SPIM_SS1_PIN);
#endif
    m_spi_cs_pin = SPIM_SS0_PIN; //default to this
    // initialze xfer state (needed for init/uninit cycles)
    m_spi_xfer_done = true;
}

void spi_uninit(void)
{
    // return pins to default state (input, hi-z)
#ifdef BOARD_MK11
    nrf_gpio_cfg_default(SPIM_SS2_PIN);
#endif
    nrf_gpio_cfg_default(SPIM_SS1_PIN);
    nrf_gpio_cfg_default(SPIM_SS0_PIN);
    // uninitialize the SPIM driver instance
    nrfx_spim_uninit(&m_spi);

    // errata 89, see https://infocenter.nordicsemi.com/index.jsp?topic=%2Ferrata_nRF52832_Rev3%2FERR%2FnRF52832%2FRev3%2Flatest%2Fanomaly_832_89.html&cp=4_2_1_0_1_26
    ASSERT(SPI_INSTANCE == 2);
    *(volatile uint32_t *)0x40023FFC = 0;
    *(volatile uint32_t *)0x40023FFC;
    *(volatile uint32_t *)0x40023FFC = 1;
    // NOTE this did not make a measurable difference
}

void spi_set_cs_pin(uint8_t cs_pin)
{
    ASSERT((cs_pin == SPIM_SS0_PIN) || (cs_pin == SPIM_SS1_PIN));
    m_spi_cs_pin = cs_pin;
}

uint8_t *spi_write_burst(uint8_t addr, const uint8_t *data, uint16_t length)
{
    ASSERT(length <= SPI_MAX_BURST_LENGTH);

    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(m_tx_buf, length+1, m_rx_buf, length+1);
    m_tx_buf[0]=addr;
    mempcpy(&m_tx_buf[1], data, length+1);
    //m_tx_buf[1]=data;

    //wait for any pending SPI operation to complete
    while (!m_spi_xfer_done)
    {
        __WFE();
    }
    // Reset rx buffer and transfer done flag
    memset(m_rx_buf, 0, length+1);
    m_spi_xfer_done = false;

    // set the current CS pin low (will be set high by event handler)
    nrf_gpio_pin_clear(m_spi_cs_pin); 

    //CS must remain asserted for both bytes (verified with scope)
    APP_ERROR_CHECK(nrfx_spim_xfer(&m_spi, &xfer_desc, 0));
    
    // Wait until SPI Master completes transfer data
    while (!m_spi_xfer_done)
    {
        __WFE();
    }

    return &m_rx_buf[1];
}

void spi_write_byte(uint8_t addr, uint8_t data)
{
    spi_write_burst(addr, &data, 1);
}

// NOTE: returns pointer to data, but data will be overwritten by the next SPI read or write
//       so caller must guarantee processing of the data before the next SPI transaction
uint8_t *spi_read_burst(uint8_t addr, uint16_t length)
{
    ASSERT(length > 0);
    ASSERT(length <= SPI_MAX_BURST_LENGTH);

    // set RD_ON register to 1; CS should go high after
    spi_write_byte(0x80, 0x01);
    
    // write address of target register to RD_ADDR register; CS should go high after
    spi_write_byte(0x81, addr);
    
    // clear receive buffer in case there is anything left from earlier activity
    memset(m_rx_buf, 0, length+1);

    // set TX buffer with don't cares (we will write 00) to write out as we read
    memset(m_tx_buf, 0, length+1);
    
    // read data from target register by:
    // write RD_ADDR address to SI pin
    // write bytes, value don't care
    // SO pin will active during this second write, and data will go into Rx buffer
    spi_write_burst(0x81, m_tx_buf, length);
    
    // address contents should have been sent on MISO pin during second byte; read from Rx buffer
    return &m_rx_buf[1];
}

uint8_t spi_read_byte(uint8_t addr)
{
    uint8_t ReadData;
    
    // set RD_ON register to 1; CS should go high after
    spi_write_byte(0x80, 0x01);
    
    // write address of target register to RD_ADDR register; CS should go high after
    spi_write_byte(0x81, addr);
    
    // clear receive buffer in case there is anything left from earlier activity
    memset(m_rx_buf, 0, 2);
    
    // read data from target register by:
    // write RD_ADDR address to SI pin
    // write a second byte, value don't care (we will write 00)
    // SO pin will active during this second write, and data will go into Rx buffer
    spi_write_byte(0x81, 0x00);
    
    // address contents should have been sent on MISO pin during second byte; read from Rx buffer
    ReadData = m_rx_buf[1];
    
    return ReadData;
}

// PRECONDITION: correct CS pin must be set (for OLED or FPGA)
bool spi_exercise_register(uint8_t addr)
{
    bool success = true;
    uint8_t original_value = spi_read_byte(addr);
    uint8_t write_value = 0x00;
    uint8_t read_value = 0x00;

    while (success && (write_value < 0xFF)) { // count up
        spi_write_byte(addr, write_value);
        read_value = spi_read_byte(addr);
        success = (read_value == write_value);
        if (success) {
            write_value++;
        } else {
            NRF_LOG_ERROR("SPI ERROR: register(0x%x) = 0x%x, expected 0x%x", addr, read_value, write_value);
        }
    }
    if (success) { // count down
        //NRF_LOG_INFO("SPI test: register (0x%x) count up passed.", addr);
        while (success && (write_value > 0x00)) { // count up
            spi_write_byte(addr, write_value);
            read_value = spi_read_byte(addr);
            success = (read_value == write_value);
            if (success) {
                write_value--;
            } else {
                NRF_LOG_ERROR("SPI ERROR: register(0x%x) = 0x%x, expected 0x%x", addr, read_value, write_value);
            }
        }
    }
    if (success) {
        //NRF_LOG_INFO("SPI test: register (0x%x) count down passed.", addr);
    }

    // attempt to restore original value, but this might fail if !success
    spi_write_byte(addr, original_value);

    return(success);
}