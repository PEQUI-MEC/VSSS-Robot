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
#include "Frames/ApiFrame.h"

using namespace XBeeLib;

RadioStatus XBee::write_config(void)
{
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("WR");
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    return Success;
}

RadioStatus XBee::set_power_level(uint8_t  level)
{
    AtCmdFrame::AtCmdResp cmdresp;

    if (level > 4) {
        return Failure;
    }

    cmdresp = set_param("PL", level);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }

    return Success;
}

RadioStatus XBee::get_power_level(uint8_t * const  level)
{
    if (level == NULL) {
        return Failure;
    }
    AtCmdFrame::AtCmdResp cmdresp;

    uint32_t var32;
    cmdresp = get_param("PL", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *level = var32;
    return Success;
}

RadioStatus XBee::software_reset(void)
{
    volatile uint16_t * const rst_cnt_p = &_wd_reset_cnt;
    const uint16_t init_rst_cnt = *rst_cnt_p;

    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("FR");
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "software_reset failed!\r\n");
        return Failure;
    }

    return wait_for_module_to_reset(rst_cnt_p, init_rst_cnt);
}

RadioStatus XBee::set_node_identifier(const char * const node_id)
{
    if (node_id == NULL) {
        return Failure;
    }

    AtCmdFrame::AtCmdResp cmdresp;
    const size_t str_len = strlen(node_id);

    if(str_len > 20 || str_len < 1) {
        return Failure;
    }

    cmdresp = set_param("NI", (const uint8_t *)node_id, str_len);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    return Success;
}

RadioStatus XBee::get_node_identifier(char * const node_id)
{
    if (node_id == NULL) {
        return Failure;
    }

    uint16_t max_ni_length = 20;
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = get_param("NI", (uint8_t *)node_id, &max_ni_length);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    node_id[max_ni_length] = '\0';
    return Success;
}

RadioStatus XBee::enable_network_encryption(bool enable)
{
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("EE", enable);
    return cmdresp == AtCmdFrame::AtCmdRespOk ? Success : Failure;
}

RadioStatus XBee::set_network_encryption_key(const uint8_t * const key, const uint16_t length)
{
    if (key == NULL || length == 0 || length > 16) {
        return Failure;
    }
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("KY", key, length);
    return cmdresp == AtCmdFrame::AtCmdRespOk ? Success : Failure;
}

uint16_t XBee::get_hw_version() const
{
    return _hw_version;
}

uint16_t XBee::get_fw_version() const
{
    return _fw_version;
}

void XBee::set_tx_options(uint8_t options)
{
    _tx_options = options;
}

uint8_t XBee::get_tx_options() const
{
    return _tx_options;
}

RadioStatus XBee::start_node_discovery()
{
    RadioStatus status;
    uint16_t nd_timeout;

    status = get_node_discovery_timeout(&nd_timeout);
    if (status != Success) {
	    return status;
    }

    _nd_timeout = nd_timeout;

    _nd_timer.start();

    AtCmdFrame cmd_frame = AtCmdFrame("ND");
    send_api_frame(&cmd_frame);

    return Success;
}

bool XBee::is_node_discovery_in_progress()
{
    const int nd_timer = _nd_timer.read_ms();

    if (nd_timer == 0)
        return false;

    if (nd_timer > _nd_timeout) {
        _nd_timer.stop();
        _nd_timer.reset();
    }

    return true;
}

void XBee::_get_remote_node_by_id(const char * const node_id, uint64_t * const addr64, uint16_t * const addr16)
{
    *addr64 = ADDR64_UNASSIGNED;
    *addr16 = ADDR16_UNKNOWN;
    if (node_id == NULL) {
        return;
    }
    const size_t node_id_len = strlen(node_id);
    if (node_id_len == 0 || node_id_len > MAX_NI_PARAM_LEN) {
        return;
    }

    const uint16_t old_timeout = _timeout_ms;

    RadioStatus status;
    uint16_t nd_timeout;
	bool wait_for_complete_timeout;

    status = get_node_discovery_timeout(&nd_timeout, &wait_for_complete_timeout);
    if (status != Success) {
	    return;
    }
	_timeout_ms = nd_timeout;

    Timer nd_timer;

    nd_timer.start();

    AtCmdFrame atnd_frame = AtCmdFrame("ND", (const uint8_t *)node_id, strlen(node_id));
    const uint8_t frame_id = atnd_frame.get_frame_id();
    _node_by_ni_frame_id = frame_id;
    send_api_frame(&atnd_frame);

    ApiFrame * const resp_frame = get_this_api_frame(frame_id, ApiFrame::AtCmdResp);
    _timeout_ms = old_timeout;

    _node_by_ni_frame_id = 0;

    if (resp_frame == NULL) {
        digi_log(LogLevelWarning, "_get_remote_node_by_id: timeout when waiting for ATND response");
        return;
    }

    if (resp_frame->get_data_len() < sizeof (uint16_t) + sizeof (uint64_t)) {
        /* In 802.15.4 this might be the OK or Timeout message with no information */
        digi_log(LogLevelInfo, "_get_remote_node_by_id: node not found\r\n", __FUNCTION__, node_id);
        _framebuf_syncr.free_frame(resp_frame);
        return;
    }

    const AtCmdFrame::AtCmdResp resp = (AtCmdFrame::AtCmdResp)resp_frame->get_data_at(ATCMD_RESP_STATUS_OFFSET);
    if (resp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelWarning, "_get_remote_node_by_id: send_at_cmd bad response: 0x%x\r\n", resp);
        _framebuf_syncr.free_frame(resp_frame);
        return;
    }

    rmemcpy((uint8_t *)addr16, resp_frame->get_data() + ATCMD_RESP_DATA_OFFSET, sizeof *addr16);
    rmemcpy((uint8_t *)addr64, resp_frame->get_data() + ATCMD_RESP_DATA_OFFSET + sizeof *addr16, sizeof *addr64);
    _framebuf_syncr.free_frame(resp_frame);

    if (wait_for_complete_timeout) {
        while (nd_timer.read_ms() < nd_timeout) {
            wait_ms(10);
        }
    }

    return;
}

RadioStatus XBee::config_node_discovery(uint16_t backoff_ms, uint8_t options)
{
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("NT", (uint8_t)(backoff_ms / 100));
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }

    cmdresp = set_param("NO", (uint8_t)options);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    cmdresp = set_param("AC");
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }

    return Success;
}

RadioStatus XBee::get_config_node_discovery(uint16_t * const backoff_ms, uint8_t * const options)
{
    AtCmdFrame::AtCmdResp cmdresp;
    uint32_t var32;

    cmdresp = get_param("NT", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *backoff_ms = var32;

    cmdresp = get_param("NO", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *options = var32;
    return Success;
}

RadioStatus XBee::_get_iosample(const RemoteXBee& remote, uint8_t * const io_sample, uint16_t * const len)
{
    AtCmdFrame::AtCmdResp cmdresp;

    /* Force a sample read */
    cmdresp = get_param(remote, "IS", io_sample, len);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "_get_iosample error %d:\r\n", cmdresp);
        return Failure;
    }

    return Success;
}

RadioStatus XBee::config_io_sample_destination(const RemoteXBee& remote, const RemoteXBee& destination)
{
    uint32_t dh;
    uint32_t dl;

    if (destination.is_valid_addr64b()) {
        const uint64_t dest64 = destination.get_addr64();
        dh = (uint32_t)((dest64 >> 32) & 0xFFFFFFFF);
        dl = (uint32_t)((dest64 & 0xFFFFFFFF));
    } else if (destination.is_valid_addr16b()) {
        const uint16_t destAddr16 = destination.get_addr16();
        dh = 0;
        dl = destAddr16;
    } else {
        digi_log(LogLevelError, "send_io_sample_to: Invalid destination");
        return Failure;
    }

    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param(remote, "DH", dh);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "send_io_sample_to error %d:\r\n", cmdresp);
        return Failure;
    }

    cmdresp = set_param(remote, "DL", dl);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "send_io_sample_to error %d:\r\n", cmdresp);
        return Failure;
    }

    return Success;
}

RadioStatus XBee::set_io_sample_rate(const RemoteXBee& remote, const float seconds)
{
    const float max_seconds = 65.535;

    if (seconds > max_seconds) {
        digi_log(LogLevelError, "XBee::set_io_sample_rate error seconds rate exceeds maximum %d:\r\n", max_seconds);
        return Failure;
    }

    AtCmdFrame::AtCmdResp cmdresp;
    const uint16_t milliseconds = seconds * 1000;

    cmdresp = set_param(remote, "IR", milliseconds);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "XBee::set_io_sample_rate error %d:\r\n", cmdresp);
        return Failure;
    }

    return Success;
}
