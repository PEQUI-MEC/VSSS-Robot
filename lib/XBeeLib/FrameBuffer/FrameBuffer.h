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

#if !defined(__FRAME_BUFFER_H_)
#define __FRAME_BUFFER_H_

#include "config.h"
#include "mbed.h"
#include "Frames/ApiFrame.h"

#if FRAME_BUFFER_SIZE > 255
# error "FRAME_BUFFER_SIZE must be lower than 256"
#endif

typedef struct element {
    ApiFrame    *frame;
    uint8_t     status;
} buf_element_t;

/**
 *  @class FrameBuffer
 *  @brief storage class for incoming frames
 */
class FrameBuffer
{
    public:
        /** Constructor */
        FrameBuffer(uint8_t size, uint16_t max_payload_len);

        FrameBuffer(const FrameBuffer& other); /* Intentionally not implemented */
        /** Destructor */
        ~FrameBuffer();

        /** get_next_free_frame returns the next free frame
         *
         * @returns a pointer to the next free frame */
        ApiFrame *get_next_free_frame();

        /** complete_frame sets the status of the frame to complete once the
         *                      data has been set in the buffer.
         *
         * @param pointer to the buffer we want to set as complete
         * @returns true on success, false otherwise
         */
        bool complete_frame(ApiFrame *frame);

        /** free_frame makes the frame available to be reused in future
         *
         * @param frame to release */
        bool free_frame(ApiFrame *frame);

        /** get_next_complete_frame returns the pointer to the next complete frame
         *
         * @returns the pointer to the selected buffer
         */
        ApiFrame *get_next_complete_frame();

        /** get_dropped_frames_count returns the number of dropped frames since latest call to this method
         *
         * @returns the number of dropped frames since latest call to this method
         */
        uint32_t get_dropped_frames_count();

protected:

        /** frame status */
        enum FrameStatus {
            FrameStatusFree = 0,   /**< Free */
            FrameStatusAssigned,   /**< Assigned */
            FrameStatusComplete    /**< Complete */
        };

        /** buffer array */
        buf_element_t   * _frm_buf;

        uint8_t          _size;
        

        /** head frame index */
        uint8_t         _head;

        /** tail frame index for application */
        uint8_t         _tail;

        /** dropped frames */
        uint32_t        _dropped_frames;
};

#endif /* __FRAME_BUFFER_H_ */
