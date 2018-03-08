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

#include "FrameHandler.h"

FrameHandler::FrameHandler(ApiFrame::ApiFrameType t) : _type(t)
{
}

FrameHandler::~FrameHandler()
{
}

ApiFrame::ApiFrameType FrameHandler::get_type() const
{
    return _type;
}
