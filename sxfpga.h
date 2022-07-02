#ifndef FPGA_H
#define FPGA_H

#include <stdint.h>
#include <stdbool.h>
#include "spi.h"
#include "board.h"

#define STREAMLOGIC_FPGA 1

#define FPGA_LOG_INFO_ON

#define FPGA_BUFFERS_SUPPORTED 1

void fpga_status();
bool fpga_framebuf_range(uint32_t *first, uint32_t *last);
void fpga_begin_readout();
uint8_t *fpga_readout_next(uint16_t *len_p);
void fpga_replay(uint8_t speed);
void fpga_resume();
uint16_t fpga_calc_checksum(uint8_t *bytearray, uint32_t length);

#endif // FPGA_H
