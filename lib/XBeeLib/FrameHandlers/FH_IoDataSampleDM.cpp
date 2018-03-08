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
FH_IoDataSampeDM::FH_IoDataSampeDM() : FrameHandler(ApiFrame::IoSampleRxZBDM),
    io_data_cb(NULL)
{
}

/** Class destructor */
FH_IoDataSampeDM::~FH_IoDataSampeDM()
{
}

void FH_IoDataSampeDM::register_io_data_cb(io_data_cb_dm_t function)
{
    io_data_cb = function;
}

void FH_IoDataSampeDM::unregister_io_data_cb()
{
    io_data_cb = NULL;
}

/* DM RX packet offsets */
#define DM_IO_SAMPLE_DATA_OFFSET            11

void FH_IoDataSampeDM::process_frame_data(const ApiFrame *const frame)
{
    const uint8_t * const datap = frame->get_data();;

    /* The caller checks that the type matches, so no need to check it here again */
    if (io_data_cb == NULL) {
        return;
    }

    /* We got an IO packet, decode it... */
    const uint64_t sender64 = addr64_from_uint8_t(datap);
    const RemoteXBeeDM sender = RemoteXBeeDM(sender64);
    const IOSampleDM ioSample = IOSampleDM(&datap[DM_IO_SAMPLE_DATA_OFFSET], frame->get_data_len() - DM_IO_SAMPLE_DATA_OFFSET);

    io_data_cb(sender, ioSample);
}
