/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file board.c
 *	@brief Code for Board Support Package. MCU hardware dependent.
 *
 *	@author Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */
#include "board.h"
#include "nrf_delay.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef BOARD_MK11
#include "max77654.h"
#endif

#if LEDS_NUMBER > 0
static const uint8_t m_board_led_list[LEDS_NUMBER] = LEDS_LIST;
#endif

#if LEDS_NUMBER > 0
bool bsp_board_led_state_get(uint32_t led_idx)
{
    ASSERT(led_idx < LEDS_NUMBER);
    bool pin_set = nrf_gpio_pin_out_read(m_board_led_list[led_idx]) ? true : false;
    return (pin_set == (LEDS_ACTIVE_STATE ? true : false));
}

void bsp_board_led_on(uint32_t led_idx)
{
        ASSERT(led_idx < LEDS_NUMBER);
        nrf_gpio_pin_write(m_board_led_list[led_idx], LEDS_ACTIVE_STATE ? 1 : 0);
}

void bsp_board_led_off(uint32_t led_idx)
{
    ASSERT(led_idx < LEDS_NUMBER);
    nrf_gpio_pin_write(m_board_led_list[led_idx], LEDS_ACTIVE_STATE ? 0 : 1);
}

void bsp_board_leds_off(void)
{
    uint32_t i;
    for (i = 0; i < LEDS_NUMBER; ++i)
    {
        bsp_board_led_off(i);
    }
}

void bsp_board_leds_on(void)
{
    uint32_t i;
    for (i = 0; i < LEDS_NUMBER; ++i)
    {
        bsp_board_led_on(i);
    }
}

void bsp_board_led_invert(uint32_t led_idx)
{
    ASSERT(led_idx < LEDS_NUMBER);
    nrf_gpio_pin_toggle(m_board_led_list[led_idx]);
}

static void bsp_board_leds_init(void)
{
    uint32_t i;
    for (i = 0; i < LEDS_NUMBER; ++i)
    {
        nrf_gpio_cfg_output(m_board_led_list[i]);
    }
    bsp_board_leds_off();
}

uint32_t bsp_board_led_idx_to_pin(uint32_t led_idx)
{
    ASSERT(led_idx < LEDS_NUMBER);
    return m_board_led_list[led_idx];
}

uint32_t bsp_board_pin_to_led_idx(uint32_t pin_number)
{
    uint32_t ret = 0xFFFFFFFF;
    uint32_t i;
    for (i = 0; i < LEDS_NUMBER; ++i)
    {
        if (m_board_led_list[i] == pin_number)
        {
            ret = i;
            break;
        }
    }
    return ret;
}
#endif //LEDS_NUMBER > 0

void bsp_board_io_off(uint8_t io_pin)
{
    nrf_gpio_pin_write(io_pin, 0);
}

void bsp_board_io_on(uint8_t io_pin)
{
    nrf_gpio_pin_write(io_pin, 1);
}

void bsp_board_aux_power_on(void)
{
#ifdef IO_EN_PWR // for MK9B, MK10
    nrf_gpio_pin_write(IO_EN_PWR, 1);
    nrf_delay_ms(EN_PWR_DELAY);
#endif
#ifdef BOARD_MK11
    //WARNING -- don't pause the debugger in this section to preserve power-on timing requirements --
    max77654_rail_1v2_on(true); // FPGA requires VCC(1.2V) before VCCX(2.7V) or VCCO(1.8V)
    max77654_rail_1v8sw_on(true); // camera requires 1.8V before 2.7V
    nrf_delay_ms(1); // 1.8V ramp rate is slower than 2.7V; without this, 1.8V reaches target voltage 0.2s _after_ 2.7V
    max77654_rail_2v7_on(true);
    nrf_delay_ms(2); // rise time measured as 1.2ms; give FPGA time to boot, while MODE1 is still held low
    //-- end WARNING --
    max77654_rail_10v_on(true);
#endif
}

void bsp_board_aux_power_off(void)
{
#ifdef IO_EN_PWR // for MK9B, MK10
    nrf_gpio_pin_write(IO_EN_PWR, 0);
#endif
#ifdef BOARD_MK11
    max77654_rail_vled_on(false);
    max77654_rail_10v_on(false);
    max77654_rail_2v7_on(false);
    max77654_rail_1v8sw_on(false);
    max77654_rail_1v2_on(false);
#endif
}

void bsp_board_init(void)
{
    #if LEDS_NUMBER > 0
        bsp_board_leds_init();
    #endif //LEDS_NUMBER > 0

    // configure output pins & set to initial state
#ifdef IO_EN_PWR // for MK9B, MK10
    nrf_gpio_pin_write(IO_EN_PWR, 0); // set to 0V = power off
    nrf_gpio_cfg_output(IO_EN_PWR);
#endif
    nrf_gpio_pin_write(IO_N_CAM_RESET, 0); // set to 0V = hold camera in reset
    nrf_gpio_cfg_output(IO_N_CAM_RESET);
    nrf_gpio_pin_write(IO_CAM_PWDN, 0); // set to 0V = not asserted
    nrf_gpio_cfg_output(IO_CAM_PWDN);
    nrf_gpio_pin_write(IO_DISP_XCLR, 0); // set to 0V on boot (datasheet p.11)
    nrf_gpio_cfg_output(IO_DISP_XCLR);
#ifdef BOARD_MK10
// in testing, this was found not to work: turning it on later did not make the PMIC contactable via I2C!
//    nrf_gpio_pin_write(IO_PMIC_ACTIVE, 0); // set to 0V = not active, PMIC in Hi-Z state unless 5V power supplied
//NOTE workaround: set active now; remember to set to 0V later, in order to enable battery charging!
    nrf_gpio_pin_write(IO_PMIC_ACTIVE, 1); // set to 1.8V = active, PMIC active, battery charging disabled
    nrf_gpio_cfg_output(IO_PMIC_ACTIVE);
#endif
#ifdef BOARD_MK11
    nrf_gpio_pin_clear(SPIM_SS1_PIN); //spi_fpga_CS = MODE1, for now set LOW for AUTO BOOT (from FPGA internal flash)
    nrf_gpio_cfg_output(SPIM_SS1_PIN);
#endif
}

void bsp_board_uninit(void)
{
    // return all pins configured by bsp_board_init() to default (input/hi-Z)
#ifdef IO_EN_PWR // for MK9B, MK10
    nrf_gpio_cfg_default(IO_EN_PWR);
#endif
    nrf_gpio_cfg_default(IO_N_CAM_RESET);
    nrf_gpio_cfg_default(IO_CAM_PWDN);
    nrf_gpio_cfg_default(IO_DISP_XCLR);
#ifdef BOARD_MK10
    nrf_gpio_cfg_default(IO_PMIC_ACTIVE);
#endif
#ifdef BOARD_MK11
    nrf_gpio_cfg_default(SPIM_SS1_PIN);
#endif
}

void bsp_board_check_recovery(void)
{
    // configure recovery pin
    nrf_gpio_cfg_input(IO_RECOVERY_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_delay_ms(2); // withought this, in Release build, reads as 0 (OK in Debug build)?
    // check if held to GND
    if(nrf_gpio_pin_read(IO_RECOVERY_PIN) == 0)
    {
        while(1); //busy loop here
    }
}