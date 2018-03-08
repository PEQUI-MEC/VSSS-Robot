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

#include "FH_RxPacketZB.h"

using namespace XBeeLib;

/** Class constructor */
FH_RxPacketZB::FH_RxPacketZB() : FrameHandler(ApiFrame::RxPacketAO0), receive_cb(NULL)
{
}

/** Class destructor */
FH_RxPacketZB::~FH_RxPacketZB()
{
}

void FH_RxPacketZB::register_receive_cb(receive_zb_cb_t function)
{
    receive_cb = function;
}

void FH_RxPacketZB::unregister_receive_cb(void)
{
    receive_cb = NULL;
}

/* ZB RX packet offsets */
#define ZB_RX_ADDR16_MSB_OFFSET     8
#define ZB_RX_ADDR16_LSB_OFFSET     9
#define ZB_RX_OPTIONS_OFFSET        10
#define ZB_RX_DATA_OFFSET           11
#define ZB_RX_OVERHEAD              (8+2+1)

#define BROADCAST_PACKET            0x02

void FH_RxPacketZB::process_frame_data(const ApiFrame *const frame)
{
    /* The caller checks that the type matches, so no need to check it here again */

    if (receive_cb == NULL) {
        return;
    }

    /* We got a rx packet, decode it... */
    const uint8_t *datap = frame->get_data();
    const uint64_t sender64 = addr64_from_uint8_t(datap);
    const uint16_t sender16 = ADDR16(datap[ZB_RX_ADDR16_MSB_OFFSET], datap[ZB_RX_ADDR16_LSB_OFFSET]);
    const uint8_t rx_options = datap[ZB_RX_OPTIONS_OFFSET];
    const RemoteXBeeZB sender = RemoteXBeeZB(sender64, sender16);

    receive_cb(sender, rx_options & BROADCAST_PACKET, &datap[ZB_RX_DATA_OFFSET], frame->get_data_len() - ZB_RX_OVERHEAD);
}
