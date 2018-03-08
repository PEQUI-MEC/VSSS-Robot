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

#if !defined(__FRAME_HANDLER_H_)
#define __FRAME_HANDLER_H_

#include "Frames/ApiFrame.h"

/** Class for the frame handlers */
class FrameHandler
{
    friend class ApiFrame;

    public:
        /** Class constructor
         *
         * @param type frame type handled by this frame handler
         */
        FrameHandler(ApiFrame::ApiFrameType type);

        FrameHandler(const FrameHandler& other); /* Intentionally not implemented */
        /** Class destructor */
        virtual ~FrameHandler();

        /** get_type returns the type of frames handled by this handler
         *
         * @returns the frame type handled by the handler
         */
        ApiFrame::ApiFrameType get_type() const;

        /** process_frame_data method called by the library to process the
         *                     the incoming frames if the type matches.
         *
         * @param frame pointer pointing to the api frame that must be processed
         */
        virtual void process_frame_data(const ApiFrame *const frame) = 0;

    protected:
        /** frame type handled by this handler */
        ApiFrame::ApiFrameType   _type;
};

#endif /* defined(__FRAME_HANDLER_H_) */
