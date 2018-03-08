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

#include "DigiLogger.h"

#include <stdarg.h>

using namespace DigiLog;

LogLevel DigiLogger::_log_level;

DigiLogger* DigiLogger::current_logger;

/* Base Class constructor */
DigiLogger::DigiLogger()
{
    _log_level = LogLevelNone;

    current_logger = NULL;
}

/* Class destructor */
DigiLogger::~DigiLogger()
{
    current_logger = NULL;
}

void DigiLogger::set_level(LogLevel log_level)
{
    _log_level = log_level;
}

LogLevel DigiLogger::get_level()
{
    return _log_level;
}

void DigiLogger::log_format(LogLevel log_level, const char *format, ...)
{
    static char buffer[DEBUG_BUFFER_LEN];
    va_list argp;

    if (current_logger == NULL) {
        return;
    }

    if (_log_level < log_level) {
        return;
    }

    va_start(argp, format);
    vsnprintf(buffer, DEBUG_BUFFER_LEN, format, argp);
    va_end(argp);

    current_logger->log_buffer(buffer);
}

void DigiLogger::log_buffer(char const * const buffer)
{
    (void)(buffer);
}
