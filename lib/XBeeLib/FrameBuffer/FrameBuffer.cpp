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

#include "FrameBuffer.h"
#include "Utils/Debug.h"

#if !(defined AVOID_DISABLE_IRQS)
#define disable_irq() __disable_irq()
#define enable_irq() __enable_irq()
#else
#define disable_irq()
#define enable_irq()
#endif

FrameBuffer::FrameBuffer(uint8_t size, uint16_t max_payload_len) : _size(size), _head(0), _tail(0), _dropped_frames(0)
{
    _frm_buf = new buf_element_t[_size];

    assert(_frm_buf != NULL);

    for (int i = 0; i < _size; i++) {
        _frm_buf[i].frame = new ApiFrame(max_payload_len - 1);
        _frm_buf[i].status = FrameStatusFree;
    }
}

FrameBuffer::~FrameBuffer()
{
    for (int i = 0; i < _size; i++) {
        delete _frm_buf[i].frame;
    }

    delete _frm_buf;
}

ApiFrame *FrameBuffer::get_next_free_frame(void)
{
    uint8_t i = _head;
    ApiFrame *ret = NULL;

    do {
        if (_frm_buf[i].status == FrameStatusFree || _frm_buf[i].status == FrameStatusComplete) {
            if (_frm_buf[i].status == FrameStatusComplete) {
                _dropped_frames++;
            }
            _frm_buf[i].status = FrameStatusAssigned;
            ret = _frm_buf[i].frame;
            _head = ++i % _size;
            break;
        }
        i++;
        i = i % _size;
    } while (i != _head);

    return ret;
}

bool FrameBuffer::complete_frame(ApiFrame *frame)
{
    bool ret = false;

    for (int i = 0; i < _size; i++) {
        if (_frm_buf[i].frame == frame) {
            _frm_buf[i].status = FrameStatusComplete;
            ret = true;
            break;
        }
    }

    return ret;
}

ApiFrame *FrameBuffer::get_next_complete_frame(void)
{
    uint8_t i = _tail;
    ApiFrame *ret = NULL;

    do {
        disable_irq();
        if (_frm_buf[i].status == FrameStatusComplete) {
            _frm_buf[i].status = FrameStatusAssigned;
            enable_irq();
            ret = _frm_buf[i].frame;
            _tail = ++i % _size;
            break;
        }
        enable_irq();
        i++;
        i = i % _size;
    } while (i != _tail);

    return ret;
}

bool FrameBuffer::free_frame(ApiFrame *frame)
{
    bool ret = false;

    for (int i = 0; i < _size; i++) {
        if (_frm_buf[i].frame == frame) {
            _frm_buf[i].status = FrameStatusFree;
            ret = true;
            break;
        }
    }

    return ret;
}

uint32_t FrameBuffer::get_dropped_frames_count(void)
{
    const uint32_t dropped_frames = _dropped_frames;

    _dropped_frames = 0;

    return dropped_frames;
}
