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

#if !defined(__FH_RX_PACKET_802_H_)
#define __FH_RX_PACKET_802_H_

#include "FrameHandler.h"
#include "RemoteXBee/RemoteXBee.h"

namespace XBeeLib {

/**
 * @defgroup callback_types "Callback types declaration"
 * @{
 */
/** receive callback type declaration
  * @param remote the remote module that sent the data
  * @param broadcast a boolean to tell if the message was broadcast (true) or unicast (false)
  * @param data a pointer to data sent by @b remote.
  * @param len length (in bytes) of @b data buffer
  */
typedef void (*receive_802_cb_t)(const RemoteXBee802& remote, bool broadcast, const uint8_t *const data, uint16_t len);
/**
 * @}
 */

class FH_RxPacket64b802 : public FrameHandler
{
    private:
        /** Callback function, invoked if registered */
        receive_802_cb_t receive_cb;

    public:
        /** Class constructor */
        FH_RxPacket64b802();

        /** Class destructor */
        virtual ~FH_RxPacket64b802();

        virtual void process_frame_data(const ApiFrame* const frame);

        virtual void register_receive_cb(receive_802_cb_t function);

        virtual void unregister_receive_cb();
};

class FH_RxPacket16b802 : public FrameHandler
{
    private:
        /** Callback function, invoked if registered */
        receive_802_cb_t receive_cb;

    public:
        /** Class constructor */
        FH_RxPacket16b802();

        /** Class destructor */
        virtual ~FH_RxPacket16b802();

        virtual void process_frame_data(const ApiFrame *const frame);

        virtual void register_receive_cb(receive_802_cb_t function);

        virtual void unregister_receive_cb();
};

}   /* namespace XBeeLib */

#endif /* __FH_RX_PACKET_802_H_ */
