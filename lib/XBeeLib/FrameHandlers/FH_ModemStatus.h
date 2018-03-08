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

#if !defined(__FH_MODEM_STATUS_H_)
#define __FH_MODEM_STATUS_H_

#include "Frames/AtCmdFrame.h"
#include "FrameHandler.h"

typedef void (*modem_status_cb_t)(AtCmdFrame::ModemStatus status);

class FH_ModemStatus : public FrameHandler
{
    private:
        /** Callback function, invoked (if registered) when a modem status packet is received */
        modem_status_cb_t modem_status_cb;

    public:
        /** Class constructor */
        FH_ModemStatus();

        /** Class destructor */
        virtual ~FH_ModemStatus();

         /** Method called by the stack to process the modem status frame data

             \param frame pointer pointing to api frame that must be processed */
        virtual void process_frame_data(const ApiFrame *const frame);

        virtual void register_modem_status_cb(modem_status_cb_t function);

        virtual void unregister_modem_status_cb(void);
};

#endif /* __FH_MODEM_STATUS_H_ */
