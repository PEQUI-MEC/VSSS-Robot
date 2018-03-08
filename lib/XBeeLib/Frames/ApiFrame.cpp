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

#include "mbed.h"
#include "XBee/XBee.h"
#include "ApiFrame.h"

using namespace XBeeLib;

uint8_t ApiFrame::last_frame_id = 0;

ApiFrame::ApiFrame(void)
{
    this->_type = Invalid;
    this->_data = NULL;
    this->_data_frame_len = 0;
    this->_alloc_data = false;
    _frame_id = get_next_frame_id();
}

ApiFrame::ApiFrame(uint16_t len)
{
    this->_type = Invalid;
    this->_data = new uint8_t[len];
    this->_alloc_data = true;
    this->_data_frame_len = len;
    this->_frame_id = get_next_frame_id();
}

uint8_t ApiFrame::get_next_frame_id(void)
{
    last_frame_id++;
    if (last_frame_id == 0) {
        last_frame_id++;
    }

    return last_frame_id;
}

ApiFrame::ApiFrame(ApiFrameType type, const uint8_t *data, uint16_t len)
{
    this->_data = NULL;
    set_api_frame(type, data, len);
}

void ApiFrame::set_api_frame(ApiFrameType type, const uint8_t *data, uint16_t len)
{
    this->_type = type;
    this->_data_frame_len = len;
    if (this->_data) {
        delete _data;
    }
    this->_data = new uint8_t[len];
    this->_alloc_data = true;
    assert(this->_data != NULL);
    memcpy((void *)this->_data, data, len);
}

ApiFrame::~ApiFrame()
{
    if (this->_data != NULL && this->_alloc_data) {
        delete[] this->_data;
    }
}

void ApiFrame::dump(void) const
{
#if defined(ENABLE_LOGGING)
    digi_log(LogLevelFrameData, "API frame: type %02x, len %d\r\n", this->_type, this->_data_frame_len);
    for (int i = 0; i < this->_data_frame_len; i++)
        digi_log(LogLevelFrameData, "%02x ", this->_data[i]);
    digi_log(LogLevelFrameData, "\r\n");
#endif
}

void ApiFrame::dump_if(ApiFrameType type)
{
    if (_type != type) {
        return;
    }
    dump();
}

ApiFrame::ApiFrameType ApiFrame::get_frame_type() const
{
    return _type;
}

void ApiFrame::set_frame_type(ApiFrameType type)
{
    _type = type;
}

uint16_t ApiFrame::get_data_len() const
{
    return _data_frame_len;
}

void ApiFrame::set_data_len(uint16_t len)
{
    _data_frame_len = len;
}

const uint8_t *ApiFrame::get_data() const
{
    return _data;
}

uint8_t ApiFrame::get_data_at(uint16_t index) const
{
    return *(_data + index);
}

void ApiFrame::set_data(uint8_t d, uint16_t index)
{
    *(_data + index) = d;
}

/* Returns the frame_id of this frame */
uint8_t ApiFrame::get_frame_id() const
{
    return _frame_id;
}
