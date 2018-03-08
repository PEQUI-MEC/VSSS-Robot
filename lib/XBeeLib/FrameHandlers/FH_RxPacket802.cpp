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

#include "FH_RxPacket802.h"

using namespace XBeeLib;

/** Class constructor */
FH_RxPacket64b802::FH_RxPacket64b802() : FrameHandler(ApiFrame::RxPacket64Bit),
    receive_cb(NULL)
{
}

/** Class destructor */
FH_RxPacket64b802::~FH_RxPacket64b802()
{
}

void FH_RxPacket64b802::register_receive_cb(receive_802_cb_t function)
{
    receive_cb = function;
}

void FH_RxPacket64b802::unregister_receive_cb(void)
{
    receive_cb = NULL;
}

/* 802.15.4 RX packet offsets */
#define RX_802_RSSI_OFFSET          8
#define RX_802_OPTIONS_OFFSET       9
#define RX_802_DATA_OFFSET          10
#define RX_802_OVERHEAD             (8+1+1)

#define BROADCAST_PACKET            0x02

void FH_RxPacket64b802::process_frame_data(const ApiFrame *const frame)
{
    /* The caller checks that the type matches, so no need to check it here again */

    if (receive_cb == NULL) {
        return;
    }

    /* We got a rx packet, decode it... */
    const uint8_t *datap = frame->get_data();
    const uint64_t sender64 = addr64_from_uint8_t(datap);
    const uint8_t rx_options = datap[RX_802_OPTIONS_OFFSET];
    const RemoteXBee802 sender = RemoteXBee802(sender64);

    receive_cb(sender, rx_options & BROADCAST_PACKET, &datap[RX_802_DATA_OFFSET], frame->get_data_len() - RX_802_OVERHEAD);
}


/** Class constructor */
FH_RxPacket16b802::FH_RxPacket16b802() : FrameHandler(ApiFrame::RxPacket16Bit),
    receive_cb(NULL)
{
}

/** Class destructor */
FH_RxPacket16b802::~FH_RxPacket16b802()
{
}

void FH_RxPacket16b802::register_receive_cb(receive_802_cb_t function)
{
    receive_cb = function;
}

void FH_RxPacket16b802::unregister_receive_cb(void)
{
    receive_cb = NULL;
}

/* 802.15.4 RX packet offsets */
#define RX_802_ADDR16_MSB_OFFSET    0
#define RX_802_ADDR16_LSB_OFFSET    1
#define RX_802_RSSI_OFFSET2         2
#define RX_802_OPTIONS_OFFSET2      3
#define RX_802_DATA_OFFSET2         4
#define RX_802_OVERHEAD2            (2+1+1)

void FH_RxPacket16b802::process_frame_data(const ApiFrame *const frame)
{
    /* The caller checks that the type matches, so no need to check it here again */

    if (receive_cb == NULL) {
        return;
    }

    /* We got a rx packet, decode it... */
    const uint8_t *datap = frame->get_data();
    const uint16_t sender16 = ADDR16(datap[RX_802_ADDR16_MSB_OFFSET], datap[RX_802_ADDR16_LSB_OFFSET]);
    const uint8_t rx_options = datap[RX_802_OPTIONS_OFFSET2];
    const RemoteXBee802 sender = RemoteXBee802(sender16);

    receive_cb(sender, rx_options & BROADCAST_PACKET, &datap[RX_802_DATA_OFFSET2], frame->get_data_len() - RX_802_OVERHEAD2);
}
