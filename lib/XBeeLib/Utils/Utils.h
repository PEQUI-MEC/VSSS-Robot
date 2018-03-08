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

#if !defined(__XB_UTILS_H_)
#define __XB_UTILS_H_

#include <stdint.h>

/** Buils an uint16_t out of 2 single bytes */
#define UINT16(msb,lsb)     (uint16_t)(((msb) << 8) | (lsb))
/** Buils an uint64_t out of 2 uint32_t */
#define UINT64(msb,lsb)     (uint64_t)(((uint64_t)(msb) << 32) | (lsb))

#define UINT64_HI32(u64)     (uint32_t)((u64) >> 32)
#define UINT64_LO32(u64)     (uint32_t)((u64) & 0xFFFFFFFF)

#define UNUSED_PARAMETER(a)  ((void)(a))

/** rmemcpy - like memcpy but copies the bytes in reverse order
 *
 *  @param dest pointer with the destination address
 *  @param src pointer with the source address
 *  @param bytes number of bytes that will be copied
 */
void rmemcpy(uint8_t * const dest, const uint8_t * const src, uint16_t bytes);

#endif /* __XB_UTILS_H_ */
