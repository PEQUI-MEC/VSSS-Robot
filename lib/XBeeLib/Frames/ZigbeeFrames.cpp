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

#include "ZigbeeFrames.h"

#define FRAME_ID_LEN              1
#define ADDR64_LEN                8
#define ADDR16_LEN                2
#define BROADCAST_RADIOUS_LEN     1
#define OPTIONS_LEN               1
#define TX_REQUEST_OVERHEAD       (FRAME_ID_LEN + ADDR64_LEN + \
                                   ADDR16_LEN + BROADCAST_RADIOUS_LEN + \
                                   OPTIONS_LEN)
#define SOURCE_EP_LEN             1
#define DEST_EP_LEN               1
#define CLUSTER_ID_LEN            2
#define PROFILE_ID_LEN            2

#define EXP_ADDR_OVERHEAD         (TX_REQUEST_OVERHEAD + SOURCE_EP_LEN + \
                                   DEST_EP_LEN + CLUSTER_ID_LEN + \
                                   PROFILE_ID_LEN)

/** Class constructor */
TxFrameZB::TxFrameZB(uint64_t addr, uint16_t addr16, uint8_t broadcast_rad, uint8_t tx_opt,
                const uint8_t *const data, uint16_t len)
{
    uint8_t frame_data[TX_REQUEST_OVERHEAD + len];

    _frame_id = get_next_frame_id();

    /* copy the frame id, the 64bit remote address, the 16bit network address,
     * the broad cast radious, the options byte and the frame data */

    frame_data[0] = _frame_id;
    rmemcpy(&frame_data[1], (const uint8_t *)&addr, sizeof addr);
    frame_data[9] = (uint8_t)(addr16 >> 8);
    frame_data[10] = (uint8_t)addr16;
    frame_data[11] = broadcast_rad;
    frame_data[12] = tx_opt;

    if (len) {
        memcpy(&frame_data[13], data, len);
    }

    set_api_frame(TxReqZBDM, frame_data, TX_REQUEST_OVERHEAD + len);
}

/** Class constructor */
TxFrameZB::TxFrameZB(uint64_t addr, uint16_t addr16, uint8_t source_ep, uint8_t dest_ep,
                  uint16_t cluster_id, uint16_t profile_id, uint8_t broadcast_rad,
                  uint8_t tx_opt, const uint8_t *const data, uint16_t len)
{
    uint8_t frame_data[EXP_ADDR_OVERHEAD + len];

    _frame_id = get_next_frame_id();

    /* copy the frame id, the 64bit remote address, the 16bit network address,
     * the end point source and destination addresses, the cluster and profile IDs,
     * the broad cast radious, the options byte and the frame data */

    frame_data[0] = _frame_id;
    rmemcpy(&frame_data[1], (const uint8_t *)&addr, sizeof addr);
    frame_data[9] = (uint8_t)(addr16 >> 8);
    frame_data[10] = (uint8_t)addr16;
    frame_data[11] = source_ep;
    frame_data[12] = dest_ep;
    frame_data[13] = (uint8_t)(cluster_id >> 8);
    frame_data[14] = (uint8_t)cluster_id;
    frame_data[15] = (uint8_t)(profile_id >> 8);
    frame_data[16] = (uint8_t)profile_id;
    frame_data[17] = broadcast_rad;
    frame_data[18] = tx_opt;

    if (len) {
        memcpy(&frame_data[19], data, len);
    }

    set_api_frame(ExpAddrCmd, frame_data, EXP_ADDR_OVERHEAD + len);
}
