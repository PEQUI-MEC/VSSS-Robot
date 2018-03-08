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

#include "RemoteXBee.h"

using namespace XBeeLib;

RemoteXBee::RemoteXBee()
{
    _dev_addr64 = ADDR64_UNASSIGNED;
    _dev_addr16 = ADDR16_UNKNOWN;
}

RemoteXBee::RemoteXBee(uint64_t remote64) : _dev_addr64(remote64)
{
    _dev_addr16 = ADDR16_UNKNOWN;
}

RemoteXBee::~RemoteXBee()
{
}

uint64_t RemoteXBee::get_addr64() const
{
    return _dev_addr64;
}

uint16_t RemoteXBee::get_addr16() const
{
    return _dev_addr16;
}

RemoteXBee802::RemoteXBee802() : RemoteXBee()
{
}

RemoteXBee802::RemoteXBee802(uint64_t remote64) : RemoteXBee(remote64)
{
}

RemoteXBee802::RemoteXBee802(uint16_t remote16) : RemoteXBee()
{
    _dev_addr16 = remote16;
}

RemoteXBee802::RemoteXBee802(uint64_t remote64, uint16_t remote16) : RemoteXBee(remote64)
{
    _dev_addr16 = remote16;
}

RemoteXBee802::~RemoteXBee802()
{
}

RemoteXBeeZB::RemoteXBeeZB() : RemoteXBee()
{
}

RemoteXBeeZB::RemoteXBeeZB(uint64_t remote64) : RemoteXBee(remote64)
{
}

RemoteXBeeZB::RemoteXBeeZB(uint64_t remote64, uint16_t remote16) : RemoteXBee(remote64)
{
    _dev_addr16 = remote16;
}

RemoteXBeeZB::~RemoteXBeeZB()
{
}

RemoteXBeeDM::RemoteXBeeDM() : RemoteXBee()
{
}

RemoteXBeeDM::RemoteXBeeDM(uint64_t remote64) : RemoteXBee(remote64)
{
}

RemoteXBeeDM::~RemoteXBeeDM()
{
}
