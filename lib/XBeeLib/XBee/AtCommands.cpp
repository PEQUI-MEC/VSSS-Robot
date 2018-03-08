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

#include "XBeeLib.h"

#define GET_CMD_RESP(fr, radio_location)    (radio_location == RadioRemote ? fr->get_data_at(REM_AT_CMD_RESP_STATUS_OFFSET) \
                                                                   : fr->get_data_at(ATCMD_RESP_STATUS_OFFSET))

#define GET_DATA_LEN(fr, radio_location)    (radio_location == RadioRemote ? (fr->get_data_len() - REM_AT_CMD_RESP_OVERHEAD) \
                                                                   : (fr->get_data_len() - ATCMD_RESP_OVERHEAD))

#define GET_DATA_OFF(radio_location)        (radio_location == RadioRemote ? REM_AT_CMD_RESP_CMD_DATA_OFFSET \
                                                                   : ATCMD_RESP_DATA_OFFSET)

using namespace XBeeLib;

/** Method that sends an AT command to the module and waits for the command response.
 *  @returns the AT command response */
AtCmdFrame::AtCmdResp XBee::send_at_cmd(AtCmdFrame *frame,
     uint8_t *const buf, uint16_t *const len, RadioLocation radio_location, bool reverse)
{
    AtCmdFrame::AtCmdResp resp = AtCmdFrame::AtCmdRespTimeout;
    ApiFrame *resp_frame;
    ApiFrame::ApiFrameType expected_type =
            (frame->get_frame_type() == ApiFrame::AtCmd) ?
            ApiFrame::AtCmdResp : ApiFrame::RemoteCmdResp;

    send_api_frame(frame);

    /* Wait for the AT command response packet */
    resp_frame = get_this_api_frame(frame->get_frame_id(), expected_type);
    if (resp_frame == NULL) {
        return resp;
    }

    resp = (AtCmdFrame::AtCmdResp)GET_CMD_RESP(resp_frame, radio_location);
    if (resp == AtCmdFrame::AtCmdRespOk) {
        if (buf != NULL && len != NULL) {

            /* Copy the command response data */
            uint16_t new_len = GET_DATA_LEN(resp_frame, radio_location);

            *len = (*len < new_len) ? *len : new_len;

            /* rmemcpy makes the endian change */
            if (reverse) {
                rmemcpy(buf, resp_frame->get_data() + GET_DATA_OFF(radio_location), *len);
            } else {
                memcpy(buf, resp_frame->get_data() + GET_DATA_OFF(radio_location), *len);
            }
        }
    } else {
        digi_log(LogLevelWarning, "send_at_cmd bad response: 0x%x\r\n", resp);
    }

    /* Once processed, remove the frame from the buffer */
    _framebuf_syncr.free_frame(resp_frame);
    return resp;
}

/** Method that sends an AT command to the module and waits for the command response.
 *  @returns the AT command response */
AtCmdFrame::AtCmdResp XBee::send_at_cmd(AtCmdFrame *frame)
{
    return send_at_cmd(frame, NULL, NULL);
}

AtCmdFrame::AtCmdResp XBee::send_at_cmd(AtCmdFrame *frame, uint8_t *data)
{
    uint16_t len = sizeof *data;
    AtCmdFrame::AtCmdResp atCmdResponse = send_at_cmd(frame, data, &len);

    if (atCmdResponse == AtCmdFrame::AtCmdRespOk && len != sizeof *data) {
        atCmdResponse = AtCmdFrame::AtCmdRespLenMismatch;
    }

    return atCmdResponse;
}

AtCmdFrame::AtCmdResp XBee::send_at_cmd(AtCmdFrame *frame, uint16_t *data)
{
    uint16_t len = sizeof *data;
    AtCmdFrame::AtCmdResp atCmdResponse = send_at_cmd(frame, (uint8_t *)data, &len);

    if (atCmdResponse == AtCmdFrame::AtCmdRespOk && len != sizeof *data) {
        atCmdResponse = AtCmdFrame::AtCmdRespLenMismatch;
    }

    return atCmdResponse;
}

AtCmdFrame::AtCmdResp XBee::send_at_cmd(AtCmdFrame *frame, uint32_t *data)
{
    uint16_t len = sizeof *data;
    AtCmdFrame::AtCmdResp atCmdResponse = send_at_cmd(frame, (uint8_t *)data, &len);

    if (atCmdResponse == AtCmdFrame::AtCmdRespOk && len != sizeof *data) {
        atCmdResponse = AtCmdFrame::AtCmdRespLenMismatch;
    }

    return atCmdResponse;
}

AtCmdFrame::AtCmdResp XBee::get_param(const char * const param, uint32_t * const data)
{
    uint16_t len = sizeof *data;
    AtCmdFrame cmd_frame = AtCmdFrame(param);

    *data = 0; /* Set to zero, send_at_cmd() only writes the necessary bytes, so if only 1 is written all the remaining 3 should be 0. */
    AtCmdFrame::AtCmdResp atCmdResponse = send_at_cmd(&cmd_frame, (uint8_t *)data, &len);

    if (atCmdResponse == AtCmdFrame::AtCmdRespOk && len > sizeof *data) {
        atCmdResponse = AtCmdFrame::AtCmdRespLenMismatch;
    }

    return atCmdResponse;
}

AtCmdFrame::AtCmdResp XBee::set_param(const char * const param, uint32_t data)
{
    AtCmdFrame cmd_frame = AtCmdFrame(param, data);
    return send_at_cmd(&cmd_frame, NULL, NULL);
}

AtCmdFrame::AtCmdResp XBee::set_param(const char * const param, const uint8_t * data, uint16_t len)
{
    AtCmdFrame cmd_frame = AtCmdFrame(param, data, len);
    return send_at_cmd(&cmd_frame, NULL, NULL);
}

AtCmdFrame::AtCmdResp XBee::get_param(const char * const param, uint8_t * const data, uint16_t * const len)
{
    AtCmdFrame cmd_frame = AtCmdFrame(param);
    return send_at_cmd(&cmd_frame, data, len, RadioLocal, false);
}
