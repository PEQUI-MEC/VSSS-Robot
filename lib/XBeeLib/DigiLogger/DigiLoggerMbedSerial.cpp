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

#include "DigiLoggerMbedSerial.h"

using namespace DigiLog;

Serial *DigiLoggerMbedSerial::_log_serial;

/* Class constructor when using a serial port as logging channel */
DigiLoggerMbedSerial::DigiLoggerMbedSerial(Serial * log_serial, LogLevel log_level)
{
    _log_serial = log_serial;

    _log_level = log_level;

    DigiLogger::current_logger = this;
}

/* Class destructor */
DigiLoggerMbedSerial::~DigiLoggerMbedSerial()
{
    _log_serial = NULL;
    DigiLogger::current_logger = NULL;
}

void DigiLoggerMbedSerial::log_buffer(char const * const buffer)
{
    if (_log_serial == NULL) {
        return;
    }

    _log_serial->printf("%s", buffer);
    fflush(*_log_serial);
}



