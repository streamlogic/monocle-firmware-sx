/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file unit.c
 *	@brief Code for unit test subroutines.
 *
 *	@author Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */

#include "unit.h"
#include "nrf_log.h"
#include "oled.h"
#include "fpga.h"

// ===== internal =====




// ===== public functions =====
bool unit_test_1(uint8_t num_leds, uint32_t counts)
{
    uint8_t max_count = 1 << num_leds; // max number to count on LEDs
    uint8_t count = 0; // value displayed in binary on LEDs
    uint32_t i = 0;

    for(i=0; i<counts; i++)
    {
        if (max_count == count) count = 0;
        fpga_write_byte(FPGA_LED_CONTROL, count);
        nrf_delay_ms(60);
        count++;
    }
//    fpga_led_off_all();
    return true;
}

bool unit_test_1b(uint32_t counts)
{
    ASSERT(FPGA_DISPLAY_CONTROL_DEFAULT != FPGA_LED_CONTROL_DEFAULT); // so we alternate between different values
    uint32_t i = 0;
    uint8_t reg_value = 0;

    fpga_write_byte(FPGA_LED_CONTROL, FPGA_LED_CONTROL_DEFAULT);
    for(i=0; i<counts; i++)
    {
        if (0 == (i % 2)) {
            reg_value = fpga_read_byte(FPGA_LED_CONTROL);
            if (FPGA_LED_CONTROL_DEFAULT != reg_value)
            {
                NRF_LOG_ERROR("TEST1b: ERROR detected on count %li: LED_CONTROL = 0x%x, expected 0x%x", i, reg_value, FPGA_LED_CONTROL_DEFAULT);
                return false;
            }
        } else {
            reg_value = fpga_read_byte(FPGA_DISPLAY_CONTROL);
            if (FPGA_DISPLAY_CONTROL_DEFAULT != reg_value)
            {
                NRF_LOG_ERROR("TEST1b: ERROR detected on count %li: DISPLAY_CONTROL = 0x%x, expected 0x%x", i, reg_value, FPGA_DISPLAY_CONTROL_DEFAULT);
                return false;
            }
        }
    }
    return true;
}

bool unit_test_2(uint8_t num_leds, uint32_t counts)
{
    uint8_t max_count = 1 << num_leds; // max number to count on LEDs
    uint8_t count = 0; // value written to register & displayed in binary on LEDs
    uint8_t led_control_reg_value = 0; // value read from register
    uint32_t i = 0;

    for(i=0; i<counts; i++)
    {
        if (max_count == count) count = 0;
        fpga_write_byte(FPGA_LED_CONTROL, count);
        led_control_reg_value = fpga_read_byte(FPGA_LED_CONTROL);
        if (count != led_control_reg_value)
        {
            NRF_LOG_ERROR("TEST2: ERROR detected on cycle %li: LED_CONTROL = 0x%x, expected 0x%x", i, led_control_reg_value, count);
            return false;
        }
        nrf_delay_ms(25);
        count++;
    }
//    fpga_led_off_all();
    return true;
}

bool unit_test_3(uint32_t counts)
{
    uint32_t i = 0;
    bool success = false;

    for(i=0; i<counts; i++)
    {
        success = fpga_spi_exercise_register(FPGA_MEMORY_CONTROL); // test SPI write&read to this register
        if(!success)
        {
            NRF_LOG_ERROR("TEST3: ERROR on cycle %li.", i);
            return false;
        }
    }
    return true;
}

bool unit_test_4(uint32_t counts)
{
    uint32_t i = 0;
    bool success = false;
    bool config_success = false;

    for(i=0; i<counts; i++)
    {
        success = oled_spi_exercise_register();
        if(!success)
        {
            do {
                oled_config();
                config_success = oled_verify_config();
            } while (!config_success);
            oled_set_luminance(OLED_DIM);
            NRF_LOG_INFO("TEST4: ERROR on cycle %li; OLED reconfigured.", i);
            return false;
        }
    }
    return true;
}

bool unit_test_5a(void)
{
    bool success = false;
    //uint8_t burst_write_data[] = {0xAA, 0xAA, 0xAA, 0xAA};
    uint8_t *burst_read_data = NULL;

    oled_verify_config(); // to set the CS to OLED
    burst_read_data = spi_read_burst(0x00, 72);
    NRF_LOG_INFO("TEST5a: OLED config data burst read @0x00");
    NRF_LOG_HEXDUMP_INFO(burst_read_data, 72);
    //NOTE further logging after this is broken, reason unknown

    //TODO add burst write test to register ??? (what was it?)

    success = true;
    return success;
}

// FPGA burst write/read: W:36*7B+2B, R:1*254B [REQUIRES FPGA to enable test FIFO]
bool unit_test_5b(uint32_t counts)
{
    //from project FPGA-Test-SPI_burst
    uint8_t burst_write_data[] = {0xAA, 0x00, 0x0F, 0xFF, 0xAA, 0x06, 0x07};
    const uint8_t write_burst_length = 7;
    const uint8_t num_bursts = 36;
    const uint16_t read_burst_length = num_bursts * write_burst_length; // 252 bytes
    uint8_t *burst_read_data = NULL;
    uint8_t burst_num = 0;
    uint8_t byte_num = 0;
    uint8_t read_byte = 0;
    uint8_t write_byte = 0;
    bool success = false;
    uint32_t i = 0;

    for(i=0; i<counts; i++)
    {
        //NRF_LOG_INFO("NRF_SPIM0->RXD.MAXCNT = %d", NRF_SPIM0->RXD.MAXCNT);
        // fill the FPGA with multiple writes (MCU limitation of max. 7-byte write burst)
        for(burst_num=0; burst_num<num_bursts; burst_num++)
        {
            fpga_write_burst(burst_write_data, write_burst_length);
        }
        fpga_write_burst(burst_write_data, 2); // 252+2=254
        NRF_LOG_INFO("TEST5b: cycle %li write complete ...", i);

        burst_read_data = fpga_read_burst(read_burst_length+2); // 255-byte burst is the maximum supported by SPIM driver
        // validate read results
        success = true;
        for(burst_num=0; burst_num<num_bursts; burst_num++)
        {
            for(byte_num=0; byte_num<write_burst_length; byte_num++)
            {
                read_byte = burst_read_data[burst_num*write_burst_length + byte_num];
                write_byte = burst_write_data[byte_num];
                if(read_byte != write_byte)
                {
                    success = false;
                    NRF_LOG_ERROR("TEST5b: error in cycle %li: byte[%d] = 0x%x, expected 0x%x ", counts, burst_num*write_burst_length + byte_num, read_byte, write_byte);
                }
            }
        }
        if(success) {
            NRF_LOG_INFO("TEST5b: Burst read cycle %li data OK!", i);
        } else {
            return false;
        }
    }
    return success;
}
/* BACKUP -- THIS WORKS
//FPGA burst write/read: W:146*7B+2B(=1024B), R:2*(1*252B, 1*120B)+1*252B+1*28B [REQUIRES FPGA to enable test FIFO]
bool unit_test_5c(uint32_t counts)
{
    uint8_t burst_write_data[] = {0xAA, 0x00, 0x0F, 0xFF, 0xAA, 0x06, 0x07}; // 7 bytes
    const uint8_t write_burst_length = 7;
    const uint8_t write_num_bursts = 146;
    const uint16_t read_burst_length_long = 252; // 255-byte burst is max. supported by SPIM driver
    const uint16_t read_burst_length_short = 120;
    uint8_t *burst_read_data = NULL;
    uint8_t burst_num = 0;
    uint16_t burst_byte_num = 0;
    uint16_t buffer_byte_num = 0; // track location in FPGA FIFO buffer
    uint8_t read_byte = 0;
    uint8_t write_byte = 0;
    bool success = true;
    uint32_t i = 0;

    for(i=0; i<counts; i++)
    {
        buffer_byte_num = 0; // reset the buffer pointer

        // fill the FPGA with multiple writes (MCU limitation of max. 7-byte write burst)
        for(burst_num=0; burst_num<write_num_bursts; burst_num++)
        {
            fpga_write_burst(burst_write_data, write_burst_length);
        }
        fpga_write_burst(burst_write_data, 2); // (146*7)+2=1024
        //NRF_LOG_INFO("TEST5c: cycle %li write complete ...", i);

        for(burst_num=0; burst_num<2; burst_num++)
        {
            // long read
            burst_read_data = fpga_read_burst(read_burst_length_long);
            // validate read results
            for(burst_byte_num=0; burst_byte_num<read_burst_length_long; burst_byte_num++)
            {
                read_byte = burst_read_data[burst_byte_num];
                write_byte = burst_write_data[buffer_byte_num % write_burst_length];
                if(read_byte != write_byte)
                {
                    success = false;
                    NRF_LOG_ERROR("TEST5c: error in cycle %li: byte[%d] = 0x%x, expected 0x%x ", counts, buffer_byte_num, read_byte, write_byte);
                }
                buffer_byte_num++;
            }
 
            // short read
            burst_read_data = fpga_read_burst(read_burst_length_short);
            // validate read results
            for(burst_byte_num=0; burst_byte_num<read_burst_length_short; burst_byte_num++)
            {
                read_byte = burst_read_data[burst_byte_num];
                write_byte = burst_write_data[buffer_byte_num % write_burst_length];
                if(read_byte != write_byte)
                {
                    success = false;
                    NRF_LOG_ERROR("TEST5c: error in cycle %li: byte[%d] = 0x%x, expected 0x%x ", counts, buffer_byte_num, read_byte, write_byte);
                }
                buffer_byte_num++;
            }
        }
        // final sub-cycle, which is shorter
        // long read
        burst_read_data = fpga_read_burst(read_burst_length_long);
        // validate read results
        for(burst_byte_num=0; burst_byte_num<read_burst_length_long; burst_byte_num++)
        {
            read_byte = burst_read_data[burst_byte_num];
            write_byte = burst_write_data[buffer_byte_num % write_burst_length];
            if(read_byte != write_byte)
            {
                success = false;
                NRF_LOG_ERROR("TEST5c: error in cycle %li: byte[%d] = 0x%x, expected 0x%x ", counts, buffer_byte_num, read_byte, write_byte);
            }
            buffer_byte_num++;
        }
        // final read, extra < short
        burst_read_data = fpga_read_burst(28);
        // validate read results
        for(burst_byte_num=0; burst_byte_num<28; burst_byte_num++)
        {
            read_byte = burst_read_data[burst_byte_num];
            write_byte = burst_write_data[buffer_byte_num % write_burst_length];
            if(read_byte != write_byte)
            {
                success = false;
                NRF_LOG_ERROR("TEST5c: error in cycle %li: byte[%d] = 0x%x, expected 0x%x ", counts, buffer_byte_num, read_byte, write_byte);
            }
            buffer_byte_num++;
        }
        ASSERT(buffer_byte_num == 1024);

        if(success) {
            NRF_LOG_INFO("TEST5c: Burst read cycle %li data OK!", i);
        } else {
            return false;
        }
    }
    return success;
}
*/

//FPGA burst write/read: W:146*7B+2B(=1024B), R:2*(1*252B, 1*120B)+1*252B+1*28B [REQUIRES FPGA to enable test FIFO]
bool unit_test_5c(uint32_t counts, uint16_t read_burst_length_long, uint16_t read_burst_length_short)
{
    ASSERT(read_burst_length_long < 256);
    ASSERT(read_burst_length_long > read_burst_length_short);

    const uint16_t FPGA_FIFO_size = 1024; // FPGA FIFO size in bytes, we will fill it completely
    uint8_t burst_write_data[] = {0xAA, 0x00, 0x0F, 0xFF, 0xAA, 0x06, 0x07}; // 7 bytes
    const uint8_t write_burst_length = 7;
    const uint8_t write_num_bursts = FPGA_FIFO_size / write_burst_length;
    ASSERT(write_num_bursts == 146);
    const uint8_t write_extra_bytes = FPGA_FIFO_size - (write_num_bursts * write_burst_length);
    ASSERT(write_extra_bytes == 2);

    //const uint16_t read_burst_length_long = 250; // 255-byte burst is max. supported by SPIM driver
    //const uint16_t read_burst_length_short = 140;
    const uint16_t mini_cycle_length = read_burst_length_long + read_burst_length_short;
    const uint8_t num_mini_cycles = FPGA_FIFO_size / mini_cycle_length;
    uint16_t read_extra_bytes = FPGA_FIFO_size - (num_mini_cycles * mini_cycle_length);
    bool read_extra_long = false;
    if(read_extra_bytes > read_burst_length_long)
    {
        read_extra_long = true;
        read_extra_bytes = read_extra_bytes - read_burst_length_long;
    }

    uint8_t *burst_read_data = NULL;
    uint8_t burst_num = 0;
    uint16_t burst_byte_num = 0;
    uint16_t buffer_byte_num = 0; // track location in FPGA FIFO buffer
    uint8_t read_byte = 0;
    uint8_t write_byte = 0;
    bool success = true;
    uint32_t i = 0;

    NRF_LOG_INFO("TEST5c: one cycle is:");
    NRF_LOG_INFO("   Burst Writes: %d*%dB+%dB = %d Bytes", write_num_bursts, write_burst_length, write_extra_bytes, FPGA_FIFO_size);
    if(read_extra_long) {
      NRF_LOG_INFO("   Burst Reads : %d*(%dB+%dB)+%dB+%dB", num_mini_cycles, read_burst_length_long, read_burst_length_short, read_burst_length_long, read_extra_bytes);
    } else {
      NRF_LOG_INFO("   Burst Reads : %d*(%dB+%dB)+%dB", num_mini_cycles, read_burst_length_long, read_burst_length_short, read_extra_bytes);
    }
    for(i=0; i<counts; i++)
    {
        buffer_byte_num = 0; // reset the buffer pointer

        // fill the FPGA with multiple writes (MCU limitation of max. 7-byte write burst)
        for(burst_num=0; burst_num<write_num_bursts; burst_num++)
        {
            fpga_write_burst(burst_write_data, write_burst_length);
        }
        fpga_write_burst(burst_write_data, write_extra_bytes); // (146*7)+2=1024
        //NRF_LOG_INFO("TEST5c: cycle %li write complete ...", i);

        for(burst_num=0; burst_num<num_mini_cycles; burst_num++)
        {
            // long read
            burst_read_data = fpga_read_burst(read_burst_length_long);
            // validate read results
            for(burst_byte_num=0; burst_byte_num<read_burst_length_long; burst_byte_num++)
            {
                read_byte = burst_read_data[burst_byte_num];
                write_byte = burst_write_data[buffer_byte_num % write_burst_length];
                if(read_byte != write_byte)
                {
                    success = false;
                    NRF_LOG_ERROR("TEST5c: error in cycle %li: byte[%d] = 0x%x, expected 0x%x ", counts, buffer_byte_num, read_byte, write_byte);
                }
                buffer_byte_num++;
            }
 
            // short read
            burst_read_data = fpga_read_burst(read_burst_length_short);
            // validate read results
            for(burst_byte_num=0; burst_byte_num<read_burst_length_short; burst_byte_num++)
            {
                read_byte = burst_read_data[burst_byte_num];
                write_byte = burst_write_data[buffer_byte_num % write_burst_length];
                if(read_byte != write_byte)
                {
                    success = false;
                    NRF_LOG_ERROR("TEST5c: error in cycle %li: byte[%d] = 0x%x, expected 0x%x ", counts, buffer_byte_num, read_byte, write_byte);
                }
                buffer_byte_num++;
            }
        }
        // final sub-cycle, which is shorter
        // long read, if sufficent bytes left
        if(read_extra_long)
        {
            burst_read_data = fpga_read_burst(read_burst_length_long);
            // validate read results
            for(burst_byte_num=0; burst_byte_num<read_burst_length_long; burst_byte_num++)
            {
                read_byte = burst_read_data[burst_byte_num];
                write_byte = burst_write_data[buffer_byte_num % write_burst_length];
                if(read_byte != write_byte)
                {
                    success = false;
                    NRF_LOG_ERROR("TEST5c: error in cycle %li: byte[%d] = 0x%x, expected 0x%x ", counts, buffer_byte_num, read_byte, write_byte);
                }
                buffer_byte_num++;
            }
        }
        // final read, extra < short
        burst_read_data = fpga_read_burst(read_extra_bytes);
        // validate read results
        for(burst_byte_num=0; burst_byte_num<read_extra_bytes; burst_byte_num++)
        {
            read_byte = burst_read_data[burst_byte_num];
            write_byte = burst_write_data[buffer_byte_num % write_burst_length];
            if(read_byte != write_byte)
            {
                success = false;
                NRF_LOG_ERROR("TEST5c: error in cycle %li: byte[%d] = 0x%x, expected 0x%x ", counts, buffer_byte_num, read_byte, write_byte);
            }
            buffer_byte_num++;
        }
        ASSERT(buffer_byte_num == FPGA_FIFO_size);

        if(success) {
            NRF_LOG_INFO("TEST5c: Burst read cycle %li data OK!", i);
        } else {
            return false;
        }
    }
    return success;
}

bool unit_test_6a(void)
{
    bool success = false;
    uint16_t checksum = 0;

    uint8_t cs0[] = {0xFF, 0xFE, 0x00, 0x02};
    checksum = fpga_calc_checksum(cs0, 4);
    NRF_LOG_INFO("Checksum 0: 0x%x", checksum);
    success = (checksum == 0x100);

    uint8_t cs1[] = {0xFE, 0xFF, 0x02, 0x00};
    checksum = fpga_calc_checksum(cs1, 4);
    NRF_LOG_INFO("Checksum 1: 0x%x", checksum);
    success = success && (checksum == 0x1);

    uint8_t cs2[] = {0xAA, 0x55, 0x55, 0xAA};
    checksum = fpga_calc_checksum(cs2, 4);
    NRF_LOG_INFO("Checksum 2: 0x%x", checksum);
    success = success && (checksum == 0xFFFF);

    uint8_t cs3[] = {0xFF, 0xFF, 0xFF, 0xFF};
    checksum = fpga_calc_checksum(cs3, 4);
    NRF_LOG_INFO("Checksum 3: 0x%x", checksum);
    success = success && (checksum == 0xFFFF);

    uint8_t cs4[] = {0xFF, 0xF0, 0x00, 0xBD, 0xBE, 0x00};
    checksum = fpga_calc_checksum(cs4, 6);
    NRF_LOG_INFO("Checksum 4: 0x%x", checksum);
    success = success && (checksum == 0xAEBE);

    uint8_t cs5[] = {0xBE, 0x00};
    checksum = fpga_checksum_add(fpga_calc_checksum(cs4, 4), fpga_calc_checksum(cs5, 2));
    NRF_LOG_INFO("Checksum 5: 0x%x", checksum);
    success = success && (checksum == 0xAEBE);

    uint8_t cs6[] = {0x00, 0x00, 0xBE, 0xF7, 0xF5, 0xF7, 0xE3, 0xEF, 0xE4, 0xEF, 0xFF, 0x2F, 0xFF, 0x1F, 0xFA, 0x1F, \
                     0xE0, 0x17, 0x1E, 0xC8, 0x1F, 0xD8, 0x1F, 0xE0, 0x00, 0xD8, 0x02, 0xD8, 0x1D, 0x50, 0x1B, 0x40, \
                     0x1B, 0x08, 0x1A, 0x00, 0x03, 0x00, 0x00, 0x00};
    checksum = fpga_calc_checksum(cs6, 40);
    NRF_LOG_INFO("Checksum 6: 0x%x", checksum);
    success = success && (checksum == 0x2029);

    uint8_t cs7[] = {0x28, 0xEF, 0x28, 0x6E};
    NRF_LOG_INFO("Blue screen, 2 pixels checksum = 0x%x", fpga_calc_checksum(cs7, 4));
    checksum = 0;
    uint32_t counter = 0;
    for(counter = 0; counter < 320; counter++)
    {
        checksum = fpga_checksum_add(checksum, fpga_calc_checksum(cs7, 4));
    }
    NRF_LOG_INFO("Blue screen, 1 line checksum = 0x%x", checksum);
    success = success && (checksum == 0xA5B4);
    checksum = 0;
    for(counter=0; counter<(320*400); counter++)
    {
        checksum = fpga_checksum_add(checksum, fpga_calc_checksum(cs7, 4));
    }
    NRF_LOG_INFO("Checksum 7 (full blue frame): 0x%x", checksum);
    success = success && (checksum == 0xEA42);
    return success;
}

#if TEST_BLE_DATA == BLE_DATA_CAPT_BLACK
    uint8_t pixel_data[] = {0x10, 0x80, 0x10, 0x80}; // Y0, Cb0, Y1, Cr0
#elif TEST_BLE_DATA == BLE_DATA_CAPT_WHITE
    uint8_t pixel_data[] = {0xEB, 0x80, 0xEB, 0x80}; // Y0, Cb0, Y1, Cr0
#elif TEST_BLE_DATA == BLE_DATA_CAPT_BARS
    // TODO
#elif TEST_BLE_DATA == BLE_DATA_CAPT_CROSS_WHITE

#elif TEST_BLE_DATA == BLE_DATA_CAPT_CROSS_RED

#elif TEST_BLE_DATA == BLE_DATA_CAPT_CROSS_BLUE

#elif TEST_BLE_DATA == BLE_DATA_CAPT_CROSS_GREEN

#endif

// array for holding burst data
uint8_t test_data[256] = {0x00};
uint8_t line_data[640*2] = {0x00};
const uint8_t bar_data[8][4] = { {0xEB, 0x80, 0xEB, 0x80},   // White
                                 {0xD2, 0x10, 0xD2, 0x91},   // Yellow
                                 {0xA9, 0xA5, 0xA9, 0x10},   // Cyan
                                 {0x90, 0x36, 0x90, 0x22},   // Green
                                 {0x6C, 0xC9, 0x6A, 0x99},   // Magenta
                                 {0x51, 0x5A, 0x51, 0xEF},   // Red
                                 {0x28, 0xEF, 0x28, 0x6E},   // Blue
                                 {0x10, 0x80, 0x10, 0x80} }; // Black
uint16_t current_line = 0; // count of line within the frame, 0..399
uint16_t current_byte = 0; // count of byte within the line, 0..639*2

uint32_t test_get_capture_size(void)
{
    // reset counters
    current_line = 0;
    current_byte = 0;
    // currently only supports image capture simulation (single frame)
    return(512000); // 640x400 pixels * 2 bytes/pixel
}

uint8_t *test_read_burst(uint16_t length)
{
    ASSERT(length <= 256); // SPIM buffer max size
    ASSERT(length % 4 == 0); // must be in multiples of 4 bytes (2 pixels)
#if (TEST_BLE_DATA == BLE_DATA_CAPT_BLACK) || (TEST_BLE_DATA == BLE_DATA_CAPT_WHITE) // simple case, just repeat 2 pixels (4 bytes)
    for(uint16_t i = 0; i < length; i+=4)
    {
        memcpy(&test_data[i], pixel_data, 4);
    }
#elif TEST_BLE_DATA == BLE_DATA_CAPT_BARS // more complex case: generate bytes based on location in the frame
    uint8_t col = 0;
    uint16_t x = 0;
    // generate line
    for(col = 0; col < 8; col++)
    {
        for(x = 0; x < 80*2; x+=4)
        {
            memcpy(&line_data[col*80*2+x], bar_data[col], 4);
        }
    }
    ASSERT(col*80*2 == 640*2);
    // copy relevant section into test_data[]
    if(current_byte + length <= 640*2) { // all within one line
        memcpy(test_data, &line_data[current_byte], length);
        current_byte += length;
    } else { // spans a line
        uint16_t length_2 = current_byte + length - 640*2;
        uint16_t length_1 = length - length_2;
        memcpy(test_data, &line_data[current_byte], length_1);
        current_byte = 0; // reset to next line
        current_line++;
        memcpy(&test_data[length_1], &line_data[current_byte], length_2);
        current_byte += length_2;
    }
    ASSERT(current_byte <= 640*2);
#else
#error "Other BLE tests not yet implemented"
#endif
    return test_data;
}
