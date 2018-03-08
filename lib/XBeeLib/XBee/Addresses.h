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

#ifndef __ADDRESSES_H_
#define __ADDRESSES_H_

#include <stdint.h>
#include <string.h>

#include "Utils/Utils.h"

/* Some commonly used addresses */
#define ADDR64_BROADCAST         ((uint64_t)0x000000000000FFFF)
#define ADDR64_COORDINATOR       ((uint64_t)0x0000000000000000)
#define ADDR64_UNASSIGNED        ((uint64_t)0xFFFFFFFFFFFFFFFF)

#define ADDR16_UNKNOWN           ((uint16_t)0xFFFE)
#define ADDR16_BROADCAST         ((uint16_t)0xFFFF)

/** Macro used to create a 16bit data type from 2 bytes */
#define ADDR16(msb,lsb)             UINT16(msb,lsb)

uint64_t addr64_from_uint8_t(const uint8_t * const data, bool big_endian = true);

#endif /* __ADDRESSES_H_ */
