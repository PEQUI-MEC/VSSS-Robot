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

#if !defined(__FH_IO_DATA_SAMPLE_ZB_H_)
#define __FH_IO_DATA_SAMPLE_ZB_H_

#include "FrameHandler.h"
#include "RemoteXBee/RemoteXBee.h"

namespace XBeeLib {

class IOSampleZB;

/**
 * @defgroup callback_types "Callback types declaration"
 * @{
 */
/** IO Data Sample reception (ZigBee modules) callback type declaration
  * @param remote the remote module that sent the data
  * @param sample_data a referece to an @ref IOSampleZB that can be queried for the IoLines' values
  */
typedef void (*io_data_cb_zb_t)(const RemoteXBeeZB& remote, const IOSampleZB& sample_data);
/**
 * @}
 */

class FH_IoDataSampeZB : public FrameHandler
{
    public:
        /** Class constructor */
        FH_IoDataSampeZB();

        /** Class destructor */
        virtual ~FH_IoDataSampeZB();

        virtual void process_frame_data(const ApiFrame *const frame);

        void register_io_data_cb(io_data_cb_zb_t function);

        void unregister_io_data_cb();

    private:
        /** Callback function, invoked if registered */
        io_data_cb_zb_t io_data_cb;
};

}   /* namespace XBeeLib */

#endif /* __FH_IO_DATA_SAMPLE_ZB_H_ */
