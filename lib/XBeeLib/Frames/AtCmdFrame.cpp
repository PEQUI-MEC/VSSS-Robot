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

#include "mbed.h"
#include "Utils/Debug.h"
#include "AtCmdFrame.h"

#define AT_CMD_LEN              2
#define AT_CMD_ID_LEN           1

void AtCmdFrame::build_at_cmd_frame(const char *cmd, const uint8_t *cmd_params, uint8_t payload_len, bool reverse)
{
    uint8_t frame_data[AT_CMD_LEN + AT_CMD_ID_LEN + payload_len];

    frame_data[0] = _frame_id;
    frame_data[1] = cmd[0];
    frame_data[2] = cmd[1];
    if (payload_len) {
        if (reverse) {
            rmemcpy(&frame_data[3], cmd_params, payload_len);
        } else {
            memcpy(&frame_data[3], cmd_params, payload_len);
        }
    }

    set_api_frame(AtCmd, frame_data, AT_CMD_LEN + AT_CMD_ID_LEN + payload_len);
}

AtCmdFrame::AtCmdFrame(const char * const cmd, uint32_t cmd_param)
{
    assert(cmd != NULL);
    assert(strlen(cmd) == AT_CMD_LEN);

    uint8_t len;
    if (cmd_param <= 0xFF) {
        len = 1;
    } else if (cmd_param <= 0xFFFF) {
        len = 2;
    } else if (cmd_param <= 0xFFFFFF) {
        len = 3;
    } else {
        len = 4;
    }
    build_at_cmd_frame(cmd, (uint8_t *)&cmd_param, len);
}

AtCmdFrame::AtCmdFrame(const char * const cmd, const uint8_t * cmd_param, uint16_t param_len)
{
    assert(cmd != NULL);
    assert(strlen(cmd) == AT_CMD_LEN);

    build_at_cmd_frame(cmd, cmd_param, param_len, false);
}

AtCmdFrame::AtCmdFrame(uint64_t remote, const char * const cmd, uint32_t cmd_param)
{
    assert(cmd != NULL);
    assert(strlen(cmd) == AT_CMD_LEN);

    build_at_cmd_remote_frame(remote, ADDR16_UNKNOWN, cmd, (uint8_t *)&cmd_param, 4);
}

AtCmdFrame::AtCmdFrame(uint64_t remote, const char * const cmd, const uint8_t * cmd_param, uint16_t param_len)
{
    assert(cmd != NULL);
    assert(strlen(cmd) == AT_CMD_LEN);

    build_at_cmd_remote_frame(remote, ADDR16_UNKNOWN, cmd, cmd_param, param_len, false);
}

AtCmdFrame::AtCmdFrame(uint16_t remote, const char * const cmd, uint32_t cmd_param)
{
    assert(cmd != NULL);
    assert(strlen(cmd) == AT_CMD_LEN);

    build_at_cmd_remote_frame(ADDR64_UNASSIGNED, remote, cmd, (uint8_t *)&cmd_param, 4);
}

AtCmdFrame::AtCmdFrame(uint16_t remote, const char * const cmd, const uint8_t * cmd_param, uint16_t param_len)
{
    assert(cmd != NULL);
    assert(strlen(cmd) == AT_CMD_LEN);

    build_at_cmd_remote_frame(ADDR64_UNASSIGNED, remote, cmd, cmd_param, param_len, false);
}

AtCmdFrame::AtCmdFrame(uint64_t remote64, uint16_t remote16, const char * const cmd, uint32_t cmd_param)
{
    assert(cmd != NULL);
    assert(strlen(cmd) == AT_CMD_LEN);

    build_at_cmd_remote_frame(remote64, remote16, cmd, (uint8_t *)&cmd_param, 4);
}

AtCmdFrame::AtCmdFrame(uint64_t remote64, uint16_t remote16, const char * const cmd,
                           const uint8_t * cmd_param, uint16_t param_len)
{
    assert(cmd != NULL);
    assert(strlen(cmd) == AT_CMD_LEN);

    build_at_cmd_remote_frame(remote64, remote16, cmd, cmd_param, param_len, false);
}

#define FRAME_ID_LEN              1
#define ADDR64_LEN                8
#define ADDR16_LEN                2
#define OPTIONS_LEN               1
#define AT_CMD_LEN                2
#define REM_AT_CMD_OVERHEAD       (FRAME_ID_LEN + ADDR64_LEN + \
                                   ADDR16_LEN + OPTIONS_LEN + \
                                   AT_CMD_LEN)

void AtCmdFrame::build_at_cmd_remote_frame(uint64_t remote64, uint16_t remote16,
                const char *const cmd, const uint8_t *const cmd_params, uint8_t params_len, bool reverse)
{
    uint8_t frame_data[REM_AT_CMD_OVERHEAD + params_len];

    /* copy the frame id, the 64bit remote address, the 16bit network address,
     *  the options byte, the command and the command params */

    frame_data[0] = _frame_id;
    rmemcpy(&frame_data[1], (const uint8_t *)&remote64, sizeof remote64);
    frame_data[9] = (uint8_t)(remote16 >> 8);
    frame_data[10] = (uint8_t)remote16;
    frame_data[11] = 0x02; /* TODO Options */
    frame_data[12] = cmd[0];
    frame_data[13] = cmd[1];

    if (params_len) {
        if (reverse) {
            rmemcpy(&frame_data[14], cmd_params, params_len);
        } else {
            memcpy(&frame_data[14], cmd_params, params_len);
        }
    }

    set_api_frame(RemoteCmdReq, frame_data, REM_AT_CMD_OVERHEAD + params_len);
}
