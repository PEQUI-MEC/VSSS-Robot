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

#include "FH_RxPacketDM.h"

using namespace XBeeLib;

/** Class constructor */
FH_RxPacketDM::FH_RxPacketDM() : FrameHandler(ApiFrame::RxPacketAO0), receive_cb(NULL)
{
}

/** Class destructor */
FH_RxPacketDM::~FH_RxPacketDM()
{
}

void FH_RxPacketDM::register_receive_cb(receive_dm_cb_t function)
{
    receive_cb = function;
}

void FH_RxPacketDM::unregister_receive_cb(void)
{
    receive_cb = NULL;
}

/* DM RX packet offsets */
#define DM_RX_OPTIONS_OFFSET        10
#define DM_RX_DATA_OFFSET           11
#define DM_RX_OVERHEAD              (8+2+1)

#define BROADCAST_PACKET            0x02

void FH_RxPacketDM::process_frame_data(const ApiFrame *const frame)
{
    /* The caller checks that the type matches, so no need to check it here again */

    if (receive_cb == NULL) {
        return;
    }

    /* We got a rx packet, decode it... */
    const uint8_t *datap = frame->get_data();
    const uint64_t sender64 = addr64_from_uint8_t(datap);
    const uint8_t rx_options = datap[DM_RX_OPTIONS_OFFSET];
    const RemoteXBeeDM sender = RemoteXBeeDM(sender64);

    receive_cb(sender, rx_options & BROADCAST_PACKET, &datap[DM_RX_DATA_OFFSET], frame->get_data_len() - DM_RX_OVERHEAD);
}
