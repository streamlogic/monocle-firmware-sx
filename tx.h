/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

/** @file tx.h
 *	@brief Function prototypes for "bulk transfer" over Bluetooth.
 *
 *	@author Nathan Ashelman for Brilliant Labs Limited
 *	
 *	@bug No known bugs.
 */

#ifndef TX_H
#define TX_H

#include <stdint.h>
#include <stdbool.h>
#include "board.h"

// ===== Logging for this module =====
#define TX_LOG_INFO_ON
#define TX_LOG_DEBUG_ON

// ==== initialization, must be called before use ====
bool tx_init(void); // initialize queue & internal data
// ==== for managing what is on the transfer queue ====
void tx_en_queue(bool is_video); // add a transfer to the queue, indicate if video (ie expecting audio data)
// NOTE dequeue is internal to tx module
#if defined(BLE_ON)
void tx_continue(void); // continue with Bluetooth transfer (call when swl_periph_evt = SWL_PERIPH_EVT_DATA_SENT)
void tx_abort(void); // cease transferring data & flush the queue
#endif
// ==== for checking status ====
bool tx_is_transferring(void); // checks whether there is data on the queue (to still be transferred)
bool tx_is_full(void); // checks whether buffers or memory is full (so no futher captures should be allowed)

#endif // TX_H
