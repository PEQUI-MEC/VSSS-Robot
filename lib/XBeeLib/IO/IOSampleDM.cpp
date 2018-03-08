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
#include "IO/IOSampleDM.h"

#define IO_SAMPLE_DM_MIN_SIZE              (2 + 1 + 2)

using namespace XBeeLib;

IOSampleDM::IOSampleDM(const uint8_t* const raw_data, size_t size)
{
    if (raw_data == NULL || size == 0) {
        _digital_mask = 0;
        _analog_mask = 0;
        return;
    }
    assert(size >= IO_SAMPLE_DM_MIN_SIZE);
    assert(size <= sizeof _sampled_data);

    _digital_mask = UINT16(raw_data[1], raw_data[2]);
    _analog_mask = raw_data[3];
    _sampled_data_size = size - 4;
    memcpy(&_sampled_data[0], &raw_data[4], _sampled_data_size);
}

IOSampleDM::~IOSampleDM()
{

}

RadioStatus IOSampleDM::get_dio(XBeeDM::IoLine line, DioVal* const dio_value) const
{
    const uint16_t mask = 1 << line;
    if (mask & _digital_mask) {
        const uint16_t digital_channels = get_digital_channels();

        *dio_value = digital_channels & mask ? High : Low;
        return Success;
    }
    return Failure;
}

RadioStatus IOSampleDM::get_adc(XBeeDM::IoLine line, uint16_t* const val) const
{
    const uint8_t line_mask = 1 << line;
    const bool adc_present = line_mask & _analog_mask;
    if (!adc_present) {
        return Failure;
    }

    uint8_t analog_data_idx = _digital_mask == 0 ? 0 : 2;
    uint8_t line_sample = 0;

    while (analog_data_idx < _sampled_data_size) {
        if (_analog_mask & (1 << line_sample)) {
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

inline uint16_t IOSampleDM::get_digital_channels(void) const
{
    if (_digital_mask == 0) {
        return 0;
    }
    return UINT16(_sampled_data[0], _sampled_data[1]);
}

