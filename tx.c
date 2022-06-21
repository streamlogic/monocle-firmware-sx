/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file tx.c
 *	@brief Function implementation for "bulk transfer" over Bluetooth.
 *
 *	@author Brilliant Labs Limited (Hong Kong)
 *	
 *	@bug No known bugs.
 */

#include "tx.h"
#include "board.h"
#include "fpga.h"
#if !(FPGA_BUFFERS_SUPPORTED == 1)
#error "tx module does not yet support multiple buffers"
#endif
#include "unit.h"

#ifdef BLE_ON
// SWL Peripheral Library Includes
#include "swl_periph.h"
#endif

#include <stdint.h>
#include <stdbool.h>
#include "nrf_log.h"
#include "nrf_queue.h"
#include "protocol.h"
#include "media.h"

// ===== private data =====

typedef struct {
    int32_t     tx_bytes_transfer;
    int32_t     tx_bytes_pending;
    uint16_t    checksum_on_MCU;
    uint16_t    checksum_on_FPGA;
    bool        tx_audio_pending;
    structMediaPkt sMediaHdr;

    /* For Logging */
    uint32_t    uiTotalPkts;
    uint32_t    uiPktOffset;  /* Every PktOffset value - print */
    uint32_t    uiPktCount;
} buffer_info_t;

static bool tx_is_ble_transferring = false; // if BLE transfer is in progress
static const uint16_t spi_burst_length = 252; // size of burst read from FPGA (limited to <255 by SPIM driver)
static buffer_info_t buffer_last_captured;
//static buffer_info_t buffer_now_sending; //TODO turn on for MRB support

// ===== private functions =====

// for logging
#ifdef TX_LOG_INFO_ON
#define TX_LOG_INFO(...) NRF_LOG_INFO(__VA_ARGS__)
#else
#define TX_LOG_INFO(...)
#endif

#ifdef TX_LOG_DEBUG_ON
#define TX_LOG_DEBUG(...) NRF_LOG_DEBUG(__VA_ARGS__)
#else
#define TX_LOG_DEBUG(...)
#endif

// forward declarations
#ifdef BLE_TEST_MODE
void tx_continue(void);
#endif

// initialize buffer to default (new) state
void buffer_init(buffer_info_t *buf)
{
    buf->tx_bytes_transfer = 0;
    buf->tx_bytes_pending = 0;
    buf->checksum_on_MCU = 0;
    buf->checksum_on_FPGA = 0;
    buf->tx_audio_pending = false;

    /* Clear the Media Header */
    memset (&buf->sMediaHdr, 0, sizeof(structMediaPkt));

    /* For Logging */
    buf->uiTotalPkts = 0;
    buf->uiPktOffset = 0;
    buf->uiPktCount  = 0;
}

// FOR UNIT TESTING, check that data matches what is expected for blue screen
bool check_data_bluescreen(uint8_t *data, uint32_t length)
{
    if((data == NULL) || (length == 0)) return false;
    if((length % 4) != 0) return false;

    uint32_t loop = 0;
    for(loop=0; loop<length; loop=loop+4)
    {
        if((data[loop+0] != 0x28) || (data[loop+1] != 0xEF) || \
           (data[loop+2] != 0x28) || (data[loop+3] != 0x6E))
        {
            return false;
        }
    }
    return true;
}



// ====== public function implementations =====

bool tx_init(void)
{
    buffer_init(&buffer_last_captured);
//    buffer_init(&buffer_now_sending);

}

// ==== for managing what is on the transfer queue ====

void tx_en_queue(bool is_video)
{
    bool matching = false;
    bool num_bytes_match = false;
    bool success = false;
    pstructMediaPkt psMediaHdr = &buffer_last_captured.sMediaHdr;

#ifdef BLE_ON
    swl_err_t err_code;
#endif
    // reset data
    buffer_init(&buffer_last_captured);

    // wait for data to be available, max 1/15s = 67ms?
    //TODO change this to be interrupt driven, when implemented on FPGA
    while(!fpga_capture_done())
    {
        nrf_delay_ms(30);
    }

    // always start by reading video data
    success = fpga_is_buffer_at_start(); // double-check
    if(!success) NRF_LOG_ERROR("FAILED fpga_is_buffer_at_start()"); // this may be recoverable; continue

    /* Store Media parameters to Header
     */
    psMediaHdr->uiMediaType   = (is_video) ? VIDEO_WITHOUT_AUDIO: PHOTO;
    psMediaHdr->uiMediaLength = fpga_get_capture_size();
    psMediaHdr->uiFrameSize   = FPGA_FRAME_SIZE;
    psMediaHdr->uiNumOfFrames = (psMediaHdr->uiMediaLength / FPGA_FRAME_SIZE);

    /* Store Context variables to Transfer Frame Packets.
     */
    // get length of bytes to read over SPI
    buffer_last_captured.tx_bytes_transfer = psMediaHdr->uiMediaLength;
    buffer_last_captured.tx_bytes_pending  = buffer_last_captured.tx_bytes_transfer;
    if(0 == buffer_last_captured.tx_bytes_pending)
    {
        NRF_LOG_ERROR("tx_en_queue() failed: fpga_get_capture_size() = 0");
        return;
    }
    if(!((FPGA_FRAME_SIZE == buffer_last_captured.tx_bytes_pending) ||
         ((FPGA_FRAME_SIZE * FPGA_NUM_VIDEO_FRAMES) == buffer_last_captured.tx_bytes_pending)))
    {
        NRF_LOG_ERROR("tx_en_queue() failed: fpga_get_capture_size() = %d. expected %d or %d",
                          buffer_last_captured.tx_bytes_pending, FPGA_FRAME_SIZE,
                          (FPGA_FRAME_SIZE * FPGA_NUM_VIDEO_FRAMES));
        return;
    }

#ifdef TEST_BLE_DATA
    ASSERT(buffer_last_captured.tx_bytes_pending == test_get_capture_size());
#endif

    tx_is_ble_transferring = true;
    NRF_LOG_INFO("BLE: MEDIA Header Xfer: HdrSize:%d MediaSize:%d Initiated, ",
                        sizeof(buffer_last_captured.sMediaHdr),
                        buffer_last_captured.tx_bytes_pending);
    
    /* For Logging */
    buffer_last_captured.uiTotalPkts = buffer_last_captured.tx_bytes_pending / spi_burst_length;
    buffer_last_captured.uiPktOffset = (buffer_last_captured.uiTotalPkts * 10) / 100; /* Every 10% */

    // read by bursts, calculating checksum as we go
    //uint8_t *burst_data; // pointer to data, which is stored in SPI driver code
    //uint16_t bytes_to_tx = 0;
    //if(buffer_last_captured.tx_bytes_pending >= spi_burst_length) {
    //    bytes_to_tx = spi_burst_length;
    //} else {
    //    bytes_to_tx = buffer_last_captured.tx_bytes_pending;
    //}
    // read data from SPI & update local checksum
    // TODO more efficient transfer if we do 4x burst reads: 4*252 = 1008 < 1024 (max by_copy size)
#ifdef FPGA_RELEASE_MRB
    fpga_get_frame_info();
#endif
    //burst_data = fpga_read_burst(bytes_to_tx);
    //buffer_last_captured.checksum_on_MCU = 0; // reset for this new transfer
    //buffer_last_captured.checksum_on_MCU = fpga_checksum_add(buffer_last_captured.checksum_on_MCU, fpga_calc_checksum(burst_data, bytes_to_tx));

#ifdef TEST_BLE_DATA // replace data with known test pattern
    burst_data = test_read_burst(bytes_to_tx);
#endif

#ifdef BLE_ON
    // initiate sending over Bluetooth
    err_code = swl_periph_p2c_tx_by_copy((uint8_t *)psMediaHdr, sizeof(structMediaPkt), false, (PCMD_MEDIA | MONOCLE_TO_APP), SCMD_MEDIA_DATA_HDR);
    SWL_ERROR_CHECK(err_code);
#endif

    // update remaining number of bytes to TX
    //buffer_last_captured.tx_bytes_pending = buffer_last_captured.tx_bytes_pending - bytes_to_tx;

#ifdef BLE_ON
    // remaining data will continue to be sent by DATA_SENT event in ble_periph_dev_evt_handler()
    if(!is_video) return; // no audio data for image capture, done
#elif defined(BLE_TEST_MODE)
    // simulate sending of data by completing all the reads from FPGA now, until all sent
    while(tx_is_ble_transferring) {
        tx_continue();
    }
#endif

#ifdef MIC_ON
    ble_tx_audio_pending = true; // is video transfer, and mic enabled; flag audio as pending
    NRF_LOG_INFO("BLE: audio data transfer pending.");
#endif
    return;
}

#if defined(BLE_ON) || defined(BLE_TEST_MODE)
void tx_continue(void)
{
    bool checksums_match = false;
    bool num_bytes_match = false;
    bool success = false;
    uint8_t *burst_data; // pointer to data, which is stored in SPI driver code
    uint16_t bytes_to_tx = 0;
#ifdef BLE_ON
    swl_err_t err_code;
#endif

    if(buffer_last_captured.tx_bytes_pending > 0) { // continue transmitting pending data
        if(buffer_last_captured.tx_bytes_pending >= spi_burst_length) {
            bytes_to_tx = spi_burst_length;
        } else {
            bytes_to_tx = buffer_last_captured.tx_bytes_pending;
        }

        /* Log The Percentage of Transfer */
        {
            if ((buffer_last_captured.uiPktCount % buffer_last_captured.uiPktOffset) == 0)
            {
                NRF_LOG_INFO("BLE: Xfer Status: %d/%d",
                              buffer_last_captured.tx_bytes_pending,
                              buffer_last_captured.tx_bytes_transfer);
            }
            buffer_last_captured.uiPktCount++;
        }

        // read data from SPI & update local checksum
        // TODO maybe more efficient transfer if we do 4x burst reads: 4*252 = 1008 < 1024 (max by_copy size)
        burst_data = fpga_read_burst(bytes_to_tx);
        buffer_last_captured.checksum_on_MCU = fpga_checksum_add(buffer_last_captured.checksum_on_MCU, fpga_calc_checksum(burst_data, bytes_to_tx));
#ifdef TEST_BLE_DATA // replace data with known test pattern
        burst_data = test_read_burst(bytes_to_tx);
#endif
#ifdef BLE_ON
        // send next chunk over Bluetooth
        err_code = swl_periph_p2c_tx_by_copy(burst_data, bytes_to_tx, false, (PCMD_MEDIA | MONOCLE_TO_APP), SCMD_MEDIA_DATA);
        SWL_ERROR_CHECK(err_code);
#endif
        // update remaining number of bytes to TX
        buffer_last_captured.tx_bytes_pending -= bytes_to_tx;
        // remaining data will continue to be sent by next call to this function (from DATA_SENT event in ble_periph_dev_evt_handler())
    } else { // ble_tx_bytes_pending == 0, last tx was sent
        success = fpga_is_buffer_read_done(); // double-check
        if(!success) NRF_LOG_ERROR("FAILED fpga_is_buffer_read_done()");

        // read checksum calculated by FPGA
        buffer_last_captured.checksum_on_FPGA = fpga_get_checksum();

        // compare checksums & lengths
        checksums_match = (buffer_last_captured.checksum_on_FPGA == buffer_last_captured.checksum_on_MCU);
        num_bytes_match = (buffer_last_captured.tx_bytes_transfer == fpga_get_bytes_read());
        if(checksums_match) {
            NRF_LOG_INFO("Checksums match (0x%x)", buffer_last_captured.checksum_on_FPGA);
        } else {
            NRF_LOG_ERROR("Checksums MISMATCH: FPGA=0x%x, MCU=0x%x", buffer_last_captured.checksum_on_FPGA, buffer_last_captured.checksum_on_MCU);
        }
        if(num_bytes_match) {
            NRF_LOG_INFO("FPGA_CAPTURE_SIZE and FPGA_CAPT_BYTE_COUNT match (0x%x)", buffer_last_captured.tx_bytes_transfer);
        } else {
            NRF_LOG_ERROR("Length MISMATCH: CAPTURE_SIZE=0x%x, CAPT_BYTE_COUNT=0x%x", buffer_last_captured.tx_bytes_transfer, fpga_get_bytes_read());
        }
        if(!buffer_last_captured.tx_audio_pending) { // done; clean up state
            tx_is_ble_transferring = false;
#ifdef BLE_ON // only with real bluetooth transfer will we discard here; with BLE_TEST_MODE, it should be discarded on resume
            fpga_discard_buffer(); // disard last buffer
#endif
            //TODO once we support multiple buffers, this needs to happen at the end of each, not just end of data transfer
            NRF_LOG_INFO("BLE: data transfer complete.");
        } else { // initiate audio data tx
#ifdef MIC_ON
            fpga_prep_read_audio(); // clear checksum & set audio read bit
            buffer_last_captured.checksum_on_MCU = 0;

            // this will be audio data
            success = fpga_is_buffer_at_start(); // double-check
            if(!success) NRF_LOG_ERROR("FAILED fpga_is_buffer_at_start() for audio");

            // get length of bytes to read over SPI
            ble_tx_bytes_pending = fpga_get_capture_size();
            if(0 == ble_tx_bytes_pending)
            {
                NRF_LOG_ERROR("ble_initiate_transfer() for audio failed: fpga_get_capture_size() = 0");
                buffer_last_captured.tx_audio_pending = false;
                return;
            }

            // read by bursts, calculating checksum as we go
            if(ble_tx_bytes_pending >= spi_burst_length) {
                bytes_to_tx = spi_burst_length;
            } else {
                bytes_to_tx = ble_tx_bytes_pending;
            }
            // read data from SPI & update local checksum
            // TODO maybe more efficient transfer if we do 4x burst reads: 4*252 = 1008 < 1024 (max by_copy size)
            burst_data = fpga_read_burst(bytes_to_tx);
            checksum_on_MCU = fpga_checksum_add(checksum_on_MCU, fpga_calc_checksum(burst_data, bytes_to_tx));
#ifdef BLE_ON
            // initiate sending over Bluetooth
            err_code = swl_periph_p2c_tx_by_copy(burst_data, bytes_to_tx, false, 0x00, 0x00);
            SWL_ERROR_CHECK(err_code);
#endif
            // remaining data will continue to be sent by DATA_SENT event in ble_periph_dev_evt_handler()
            NRF_LOG_INFO("BLE: audio data transfer of %d bytes initiated.", ble_tx_bytes_pending);
            // update remaining number of bytes to TX
            ble_tx_bytes_pending = ble_tx_bytes_pending - bytes_to_tx;
#endif
            buffer_last_captured.tx_audio_pending = false;
        }
    }
    return;
}
#endif

#ifdef BLE_ON
void tx_abort(void)
{
//TODO
}
#endif // BLE_ON

// ==== for checking status ====
bool tx_is_transferring(void)
{
    return tx_is_ble_transferring;
}

// checks whether buffers or memory is full (so no futher captures should be allowed)
bool tx_is_full(void)
{
//NOTE for now, only handles single buffer
    return tx_is_ble_transferring; // if transferring, the buffer is in use
//TODO support multiple buffers
}
