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

#if !defined(__DIGI_LOGGER_H_)
#define __DIGI_LOGGER_H_

#include <cstdlib>
#include <stdio.h>

/**
 * @defgroup LogLevel
 * @{
 */
/**
 * Library Logging level.
 */
enum LogLevel {
    LogLevelNone,      /** Level None */
    LogLevelError,     /** Level Error */
    LogLevelWarning,   /** Level Warning */
    LogLevelInfo,      /** Level Info */
    LogLevelDebug,     /** Level Debug */
    LogLevelFrameData, /** Level Frame Data */
    LogLevelAll        /** Level All */
};
/**
 * @}
 */

#define DEBUG_BUFFER_LEN    200

namespace DigiLog {

class DigiLogger
{
    protected:

        /** module log level */
        static LogLevel _log_level;

        static DigiLogger* current_logger;

        /* Not implemented for base class */
        virtual void log_buffer(char const * const buffer);

    public:

        /** Class constructor */
        DigiLogger();

        /** Class destructor */
        virtual ~DigiLogger();

        /** set_level - set logging level.
         *
         *  @param log_level desired overall logging level
         */
        static void set_level(LogLevel log_level);

        /** get_level - get logging level.
         *
         *  @returns current overall logging level
         */
        static LogLevel get_level();

        /** log_format - logs a printf-like message.
         *
         *  @param log_level logging level
         *  @param format ... printf-like message
         */
        static void log_format(LogLevel log_level, const char *format, ...);

};

}   /* namespace DigiLog */

#endif /* defined(__DIGI_LOGGER_H_) */



