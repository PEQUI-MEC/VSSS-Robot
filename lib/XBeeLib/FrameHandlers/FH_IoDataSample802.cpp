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
FH_IoDataSampe64b802::FH_IoDataSampe64b802() : FrameHandler(ApiFrame::Io64Bit),
    io_data_cb(NULL)
{
}

/** Class destructor */
FH_IoDataSampe64b802::~FH_IoDataSampe64b802()
{

}

void FH_IoDataSampe64b802::register_io_data_cb(io_data_cb_802_t function)
{
    io_data_cb = function;
}

void FH_IoDataSampe64b802::unregister_io_data_cb()
{
    io_data_cb = NULL;
}

#define IO_SAMPLE_64_802_DATA_OFFSET        10

void FH_IoDataSampe64b802::process_frame_data(const ApiFrame *const frame)
{
    const uint8_t * const datap = frame->get_data();

    /* The caller checks that the type matches, so no need to check it here again */
    if (io_data_cb == NULL) {
        return;
    }

    /* We got an IO packet, decode it... */
    const uint64_t sender64 = addr64_from_uint8_t(datap);
    const RemoteXBee802 sender = RemoteXBee802(sender64);
    const IOSample802 ioSample = IOSample802(&datap[IO_SAMPLE_64_802_DATA_OFFSET], frame->get_data_len() - IO_SAMPLE_64_802_DATA_OFFSET);

    io_data_cb(sender, ioSample);
}

/** Class constructor */
FH_IoDataSampe16b802::FH_IoDataSampe16b802() : FrameHandler(ApiFrame::Io16Bit),
    io_data_cb(NULL)
{
}

/** Class destructor */
FH_IoDataSampe16b802::~FH_IoDataSampe16b802()
{
}

void FH_IoDataSampe16b802::register_io_data_cb(io_data_cb_802_t function)
{
    io_data_cb = function;
}

void FH_IoDataSampe16b802::unregister_io_data_cb()
{
    io_data_cb = NULL;
}

#define IO_SAMPLE_16_802_ADDR16_MSB_OFFSET  0
#define IO_SAMPLE_16_802_ADDR16_LSB_OFFSET  1
#define IO_SAMPLE_16_802_DATA_OFFSET        4

void FH_IoDataSampe16b802::process_frame_data(const ApiFrame *const frame)
{
    const uint8_t * const datap = frame->get_data();;

    /* The caller checks that the type matches, so no need to check it here again */
    if (io_data_cb == NULL) {
        return;
    }

    /* We got an IO packet, decode it... */
    const uint16_t sender16 = ADDR16(datap[IO_SAMPLE_16_802_ADDR16_MSB_OFFSET], datap[IO_SAMPLE_16_802_ADDR16_LSB_OFFSET]);
    const RemoteXBee802 sender = RemoteXBee802(sender16);

    const IOSample802 ioSample = IOSample802(&datap[IO_SAMPLE_16_802_DATA_OFFSET], frame->get_data_len() - IO_SAMPLE_16_802_DATA_OFFSET);

    io_data_cb(sender, ioSample);
}
