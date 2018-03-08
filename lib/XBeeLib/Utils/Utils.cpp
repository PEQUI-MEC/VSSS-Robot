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

#include "Utils.h"
#include <string.h>

void rmemcpy(uint8_t * const dest, const uint8_t * const src, uint16_t bytes)
{
    uint8_t *destp = dest + bytes - 1;
    uint8_t *srcp = (uint8_t *)src;

    while (destp >= dest)
        *destp-- = *srcp++;
}

uint64_t addr64_from_uint8_t(const uint8_t * const data, bool big_endian = true)
{
    int64_t addr64;
    if (big_endian) {
        rmemcpy((uint8_t *)&addr64, data, 8);
    } else {
        memcpy((uint8_t *)&addr64, data, 8);
    }
    return addr64;
}
