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
#include "IO/IOSample802.h"
#include "Utils/Debug.h"

#define IO_SAMPLE_802_DIGITAL_INPUTS_MASK   0x01FF
#define IO_SAMPLE_802_DIGITAL_INPUTS_COUNT  9
#define IO_SAMPLE_802_MIN_SIZE              (2 + 2)

using namespace XBeeLib;

IOSample802::IOSample802(const uint8_t* const raw_data, size_t size)
{
    if (raw_data == NULL || size == 0) {
        _channel_mask = 0;
        return;
    }
    assert(size >= IO_SAMPLE_802_MIN_SIZE);
    assert(size <= sizeof _sampled_data);

    _channel_mask = UINT16(raw_data[1], raw_data[2]);
    _sampled_data_size = size - 3;
    memcpy(&_sampled_data[0], &raw_data[3], _sampled_data_size);
}

IOSample802::~IOSample802()
{

}

RadioStatus IOSample802::get_dio(XBee802::IoLine line, DioVal* const dio_value) const
{
    if (line > XBee802::DI8) {
        digi_log(LogLevelError, "get_dio: Pin %d not supported as IO\r\n", line);
        return Failure;
    }

    const uint16_t mask = 1 << line;
    if (mask & _channel_mask) {
        const uint8_t digital_channels = get_dio_channels();

        *dio_value = digital_channels & mask ? High : Low;
        return Success;
    }
    return Failure;
}

RadioStatus IOSample802::get_adc(XBee802::IoLine line, uint16_t* const val) const
{
    if (line > XBee802::DIO5_AD5) {
        digi_log(LogLevelError, "get_adc: Pin %d not supported as ADC\r\n", line);
        return Failure;
    }
    const uint8_t analog_mask = _channel_mask >> IO_SAMPLE_802_DIGITAL_INPUTS_COUNT;
    const uint8_t line_mask = 1 << line;
    const bool adc_present = line_mask & analog_mask;
    if (!adc_present) {
        return Failure;
    }

    uint8_t analog_data_idx = dio_channels_present() == 0 ? 0 : 2;
    uint8_t line_sample = 0;

    while (analog_data_idx < _sampled_data_size) {
        if (analog_mask & (1 << line_sample)) {
            if (line_sample == line) {
                /* Write the analog value */
                *val = UINT16(_sampled_data[analog_data_idx], _sampled_data[analog_data_idx + 1]);
                break;
            }
            analog_data_idx += 2;
        }
        line_sample++;
    }

    return Success;
}

inline bool IOSample802::dio_channels_present(void) const
{
    return _channel_mask & IO_SAMPLE_802_DIGITAL_INPUTS_MASK;
}

inline uint8_t IOSample802::get_dio_channels(void) const
{
    if (dio_channels_present()) {
        return UINT16(_sampled_data[0], _sampled_data[1]);
    } else {
        return 0;
    }
}
