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

#ifndef __CONFIG_H_
#define __CONFIG_H_

/** Library configuration options */
//#define ENABLE_LOGGING
#define ENABLE_ASSERTIONS
#define FRAME_BUFFER_SIZE           10
#define MAX_FRAME_PAYLOAD_LEN       128

#define SYNC_OPS_TIMEOUT_MS         2000

#define RADIO_TX                p13 /* TODO: specify your setup's Serial TX pin connected to the XBee module DIN pin */
#define RADIO_RX                p14 /* TODO: specify your setup's Serial RX pin connected to the XBee module DOUT pin */
//#define RADIO_RTS               NC /* TODO: specify your setup's Serial RTS# pin connected to the XBee module RTS# pin */
//#define RADIO_CTS               NC /* TODO: specify your setup's Serial CTS# pin connected to the XBee module CTS# pin */
#define RADIO_RESET             NC /* TODO: specify your setup's GPIO (output) connected to the XBee module's reset pin */
//#define RADIO_SLEEP_REQ         NC /* TODO: specify your setup's GPIO (output) connected to the XBee module's SLEEP_RQ pin */
//#define RADIO_ON_SLEEP          NC /* TODO: specify your setup's GPIO (input) connected to the XBee module's ON_SLEEP pin */
#define DEBUG_TX                USBTX /* TODO: specify your setup's Serial TX for debugging */
#define DEBUG_RX                USBRX /* TODO: specify your setup's Serial RX for debugging (optional) */

#if !defined(RADIO_TX)
    #error "Please define RADIO_TX pin"
#endif

#if !defined(RADIO_RX)
    #error "Please define RADIO_RX pin"
#endif

#if !defined(RADIO_RESET)
    #define RADIO_RESET             NC
    #warning "RADIO_RESET not defined, defaulted to 'NC'"
#endif

#if defined(ENABLE_LOGGING)
    #if !defined(DEBUG_TX)
        #error "Please define DEBUG_TX"
    #endif
    #if !defined(DEBUG_RX)
        #define DEBUG_RX                NC
        #warning "DEBUG_RX not defined, defaulted to 'NC'"
    #endif
#endif

#endif /* __CONFIG_H_ */
