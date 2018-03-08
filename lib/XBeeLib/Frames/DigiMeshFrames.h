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

#if !defined(__DIGIMESH_FRAME_H_)
#define __DIGIMESH_FRAME_H_

#include "ApiFrame.h"

class TxFrameDM : public ApiFrame
{
    public:
        /** Class constructor */
        TxFrameDM(uint64_t addr, uint16_t addr16, uint8_t broadcast_rad,
                    uint8_t tx_opt, const uint8_t *const data, uint16_t len);

        /** Class constructor */
        TxFrameDM(uint64_t addr, uint16_t addr16, uint8_t source_ep, uint8_t dest_ep,
                  uint16_t cluster_id, uint16_t profile_id, uint8_t broadcast_rad,
                  uint8_t tx_opt, const uint8_t *const data, uint16_t len);
};

#endif /* __DIGIMESH_FRAME_H_ */
