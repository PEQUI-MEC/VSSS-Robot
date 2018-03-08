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

#include "FH_ModemStatus.h"

/** Class constructor */
FH_ModemStatus::FH_ModemStatus() : FrameHandler(ApiFrame::AtModemStatus), modem_status_cb(NULL)
{
}

/** Class destructor */
FH_ModemStatus::~FH_ModemStatus()
{
}

void FH_ModemStatus::register_modem_status_cb(modem_status_cb_t function)
{
    modem_status_cb = function;
}

void FH_ModemStatus::unregister_modem_status_cb(void)
{
    modem_status_cb = NULL;
}

void FH_ModemStatus::process_frame_data(const ApiFrame *const frame)
{
    /* The caller checks that the type matches, so no need to check it here again */

    if (modem_status_cb == NULL) {
        return;
    }

    modem_status_cb((AtCmdFrame::ModemStatus)frame->get_data_at(0));
}
