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

#include "XBeeLib.h"

using namespace XBeeLib;

/** Class constructor */
FH_IoDataSampeZB::FH_IoDataSampeZB() : FrameHandler(ApiFrame::IoSampleRxZBDM),
    io_data_cb(NULL)
{
}

/** Class destructor */
FH_IoDataSampeZB::~FH_IoDataSampeZB()
{
}

void FH_IoDataSampeZB::register_io_data_cb(io_data_cb_zb_t function)
{
    io_data_cb = function;
}

void FH_IoDataSampeZB::unregister_io_data_cb()
{
    io_data_cb = NULL;
}

/* ZB RX packet offsets */
#define ZB_IO_SAMPLE_ADDR16_MSB_OFFSET      8
#define ZB_IO_SAMPLE_ADDR16_LSB_OFFSET      9
#define ZB_IO_SAMPLE_DATA_OFFSET            11

void FH_IoDataSampeZB::process_frame_data(const ApiFrame *const frame)
{
    const uint8_t * const datap = frame->get_data();;

    /* The caller checks that the type matches, so no need to check it here again */
    if (io_data_cb == NULL) {
        return;
    }

    /* We got an IO packet, decode it... */
    const uint64_t sender64 = addr64_from_uint8_t(datap);
    const uint16_t sender16 = ADDR16(datap[ZB_IO_SAMPLE_ADDR16_MSB_OFFSET], datap[ZB_IO_SAMPLE_ADDR16_LSB_OFFSET]);
    const RemoteXBeeZB sender = RemoteXBeeZB(sender64, sender16);
    const IOSampleZB ioSample = IOSampleZB(&datap[ZB_IO_SAMPLE_DATA_OFFSET], frame->get_data_len() - ZB_IO_SAMPLE_DATA_OFFSET);

    io_data_cb(sender, ioSample);
}
