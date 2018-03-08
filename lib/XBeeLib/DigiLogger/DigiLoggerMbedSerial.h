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

#if !defined(__DIGI_LOGGER_MBED_SERIAL_H_)
#define __DIGI_LOGGER_MBED_SERIAL_H_

#include "mbed.h"
#include "DigiLogger.h"

namespace DigiLog {

class DigiLoggerMbedSerial : public DigiLogger
{
    protected:

        /** serial port for debugging */
        static Serial *_log_serial;

        /** log_buffer - logs a buffer through the configured serial port.
         *
         *  @param buffer ... buffer to log
         */
        virtual void log_buffer(char const * const buffer);

    public:

        /** Class constructor */
        DigiLoggerMbedSerial(Serial * log_serial, LogLevel log_level = LogLevelInfo);

        /** Class destructor */
        virtual ~DigiLoggerMbedSerial();
};

}   /* namespace DigiLog */

#endif /* defined(__DIGI_LOGGER_MBED_SERIAL_H_) */



