#include "sxfpga.h"
#include "spi.h"
#include "nrf_assert.h"
#include "nrf_delay.h"  // for nrf_delay_ms()
#include "nrf_log.h"
#include "ov5640.h" // for OV5640_FPS

#include "pipeline_api.h" // StreamLogic FPGA API defintions

#ifdef FPGA_LOG_INFO_ON
#define FPGA_LOG_INFO(...) NRF_LOG_INFO(__VA_ARGS__)
#else
#define FPGA_LOG_INFO(...) 
#endif

#ifdef FPGA_LOG_DEBUG_ON
#define FPGA_LOG_DEBUG(...) NRF_LOG_DEBUG(__VA_ARGS__)
#else
#define FPGA_LOG_DEBUG(...) 
#endif

#define BIGLITTLESWAP32(A) ((((uint32_t) (A) & 0xff000000) >> 24) | (((uint32_t) (A) & 0x00ff0000) >> 8) | (((uint32_t) (A) & 0x0000ff00) << 8) | (((uint32_t) (A) & 0x000000ff) << 24))


// SPI module functions

extern uint8_t *spi_write_burst(uint8_t addr, const uint8_t *data, uint16_t length);


//// FPGA SPI API functions


void fpga_write_burst(const uint8_t *data, uint16_t length)
{
    // select FPGA on SPI bus
    spi_set_cs_pin(SPIM_SS1_PIN);
    spi_write_burst(data[0], data+1, length-1);
}

void fpga_read_burst(uint8_t *data, uint16_t length)
{
    uint8_t *rxbuf;

    // select FPGA on SPI bus
    spi_set_cs_pin(SPIM_SS1_PIN);
    rxbuf = spi_write_burst(data[0], data+1, length-1);
    // rxbuf is offset +1
    memcpy(data+2, rxbuf+1, length-2);
}

static uint8_t fpga_read_reg8(uint8_t addr, uint8_t regno) {
    uint8_t buf[3];
    buf[0] = addr;
    buf[1] = regno;
    fpga_read_burst(buf, 3);
    return buf[2];
}

static uint8_t fpga_read_reg32(uint8_t addr, uint8_t regno) {
    uint8_t buf[6];
    uint32_t value;

    buf[0] = addr;
    buf[1] = regno;
    fpga_read_burst(buf, 6);

    value = *(uint32_t *)(buf+2);
    return BIGLITTLESWAP32(value);
}

static uint8_t streambuf[252];

static uint8_t *fpga_read_stream(uint8_t addr, uint8_t regno, uint16_t length) {
    streambuf[0] = addr;
    streambuf[1] = regno;
    fpga_read_burst(streambuf, 2+length);
    return streambuf+2;
}

bool fpga_xclk_on(void)
{
    const uint8_t cmd[2] = {OVCAM_ADDR, OVCAM_XCLK_ON_REQ};
    fpga_write_burst(cmd, 2);
    nrf_delay_ms(300);
    return true;
}

void fpga_status() {
    uint8_t buf[3];

    buf[0] = OVCAM_ADDR;
    buf[1] = OVCAM_GET_STATUS8;
    fpga_read_burst(buf, 3);
    NRF_LOG_INFO("OVCAM status %08x", buf[2]);

    
    buf[0] = FRAMEBUF_ADDR;
    buf[1] = FRAMEBUF_GET_STATUS8;
    fpga_read_burst(buf, 3);
    NRF_LOG_INFO("FRAMEBUF status %08x", buf[2]);
}

bool fpga_camera_on(void) {
    const uint8_t cmd[2] = {OVCAM_ADDR, OVCAM_START_REQ};
    fpga_write_burst(cmd, 2);
    FPGA_LOG_INFO("fpga_camera_on() sent OVCAM_START_REQ");
    nrf_delay_ms(1*(1000/OV5640_FPS) + 1); // allow last frame to finish entering video buffer to avoid split screen
    fpga_status();
    return true;
}

bool fpga_camera_off(void) {
    const uint8_t cmd[2] = {OVCAM_ADDR, OVCAM_PAUSE_REQ};
    fpga_write_burst(cmd, 2);
    nrf_delay_ms(2*(1000/OV5640_FPS) + 1); // allow last frame to finish entering video buffer to avoid split screen
    FPGA_LOG_INFO("fpga_camera_off() sent OVCAM_PAUSE_REQ, waited 2 frames");
    fpga_status();
    return true;
}

bool fpga_framebuf_range(uint32_t *first_p, uint32_t *last_p) {
    uint32_t first, last;
    uint8_t buf[6];
    buf[0] = FRAMEBUF_ADDR;
    buf[1] = FRAMEBUF_GET_OLDEST32;
    fpga_read_burst(buf, 6);
    first = *(uint32_t *)(buf+2);

    buf[0] = FRAMEBUF_ADDR;
    buf[1] = FRAMEBUF_GET_NEWEST32;
    fpga_read_burst(buf, 6);
    last = *(uint32_t *)(buf+2);

    *first_p = BIGLITTLESWAP32(first);
    *last_p = BIGLITTLESWAP32(last);

    return true;
}

void fpga_replay(uint8_t speed) {
    const uint8_t cmd[2] = {FBOUT_ADDR, FBOUT_REPLAY_REQ};
    uint8_t buf[6];
/*
    uint32_t x,y;
    fpga_framebuf_range(&x,&y);
    NRF_LOG_INFO("Framebuffer %d -> %d (%08x, %08x)", x/640/400, y/640/400, x, y);
*/
    // capture oldest available frame
    buf[0] = FRAMEBUF_ADDR;
    buf[1] = FRAMEBUF_GET_OLDEST32;
    fpga_read_burst(buf, 6);

    // write that as replay start frame
    buf[0] = FBOUT_ADDR;
    buf[1] = FBOUT_SET_START32;
    //NRF_LOG_INFO("start %02x%02x%02x%02x", buf[2], buf[3], buf[4], buf[5]);
    fpga_write_burst(buf, 6);

    // set speed
    if (speed > 0)
      speed -= 1;
    buf[0] = FBOUT_ADDR; // fb out
    buf[1] = FBOUT_SET_SPEED8;
    buf[2] = speed;
    fpga_write_burst(buf, 3);

    // send cmd
    fpga_write_burst(cmd, 2);

    FPGA_LOG_INFO("fpga_replay() sent FBOUT_REPLAY_REQ, repeat=%d", speed);
}

void fpga_resume() {
    const uint8_t cmd[2] = {FBOUT_ADDR, FBOUT_PLAY_REQ};
    fpga_write_burst(cmd, 2);
    FPGA_LOG_INFO("fpga_resume() sent FBOUT_PLAY_REQ");
}

void fpga_begin_readout() {
  const uint8_t cmd[2] = {FBREADOUT_ADDR, FBREADOUT_READOUT_REQ};
  fpga_write_burst(cmd, 2);
  NRF_LOG_INFO("fpga_begin_readout() sent FBREADOUT_READOUT_REQ");
}

uint8_t *fpga_readout_next(uint16_t *len_p) {
  uint8_t status = fpga_read_reg8(FBREADOUT_ADDR, FBREADOUT_GET_STATUS8);
  if ((status & 0x2) == 0) {
    NRF_LOG_INFO("fpga_readout_next() end (%x)", status);
    *len_p = 0;
    return NULL;
  }

  uint8_t *raw = fpga_read_stream(FBREADOUT_ADDR, FBREADOUT_IMAGE_STRM, 250);
  *len_p = 250;
  return raw;
}

// Simple checksum calculation; matching the algorithm used on FPGA
// Sums a byte stream as 16-bit words (with first byte being least significant), adding carry back in, to return 16-bit result
// Precondition: length must be a multiple of 2 (bytes)
uint16_t fpga_calc_checksum(uint8_t *bytearray, uint32_t length)
{
    if((bytearray == NULL) || (length == 0)) return 0;
    if((length % 2) != 0) return 0;

    uint32_t checksum = 0;
    uint32_t carry = 0;
    uint32_t loop = 0;
    for(loop=0; loop<length; loop=loop+2)
    {
        checksum = fpga_checksum_add(checksum, ((bytearray[loop+1]<<8) + bytearray[loop]));
    }
    return (uint16_t) checksum;
}
