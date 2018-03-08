/**
 * Copyright (c) 2015 Digi International Inc.,
 * All rights not expressly granted are reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Digi International Inc. 11001 Bren Road East, Minnetonka, MN 55343
 * =======================================================================
 */

#include "802_Frames.h"

#define FRAME_ID_LEN              1
#define ADDR64_LEN                8
#define ADDR16_LEN                2
#define OPTIONS_LEN               1
#define TX_REQUEST_OVERHEAD       (FRAME_ID_LEN + ADDR64_LEN + OPTIONS_LEN)
#define TX_REQUEST_OVERHEAD2      (FRAME_ID_LEN + ADDR16_LEN + OPTIONS_LEN)

/** Class constructor */
TxFrame802::TxFrame802(uint64_t addr, uint8_t tx_options,
                const uint8_t *const data, uint16_t len)
{
    uint8_t frame_data[TX_REQUEST_OVERHEAD + len];

    _frame_id = get_next_frame_id();

    /* copy the frame id, the 64bit remote address, the tx options byte
     * and the frame data */

    frame_data[0] = _frame_id;
    rmemcpy(&frame_data[1], (const uint8_t *)&addr, sizeof addr);
    frame_data[9] = tx_options;

    if (len) {
        memcpy(&frame_data[10], data, len);
    }

    set_api_frame(TxReq64Bit, frame_data, TX_REQUEST_OVERHEAD + len);
}

/** Class constructor */
TxFrame802::TxFrame802(uint16_t addr16, uint8_t tx_options,
                const uint8_t *const data, uint16_t len)
{
    uint8_t frame_data[TX_REQUEST_OVERHEAD2 + len];

    _frame_id = get_next_frame_id();

    /* copy the frame id, the 16bit remote address, the tx options byte
     * and the frame data */

    frame_data[0] = _frame_id;
    frame_data[1] = (uint8_t)(addr16 >> 8);
    frame_data[2] = (uint8_t)addr16;
    frame_data[3] = tx_options;

    if (len) {
        memcpy(&frame_data[4], data, len);
    }

    set_api_frame(TxReq16Bit, frame_data, TX_REQUEST_OVERHEAD2 + len);
}
