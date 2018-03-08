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

#if !defined(__802_FRAMES_H_)
#define __802_FRAMES_H_

#include "ApiFrame.h"

class TxFrame802 : public ApiFrame
{
    public:
        /** Class constructor */
        TxFrame802(uint64_t addr, uint8_t tx_options, const uint8_t *const data, uint16_t len);
        TxFrame802(uint16_t addr16, uint8_t tx_options, const uint8_t *const data, uint16_t len);

    protected:
};

#endif /* __802_FRAMES_H_ */
