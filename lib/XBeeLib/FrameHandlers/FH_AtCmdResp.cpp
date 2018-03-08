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

#include "FrameHandlers/FH_AtCmdResp.h"
#include "Frames/ApiFrame.h"

using namespace XBeeLib;

/** Class constructor */
FH_AtCmdResp::FH_AtCmdResp() :
    FrameHandler(ApiFrame::AtCmdResp), at_cmd_resp_cb(NULL)
{
}

FH_AtCmdResp::FH_AtCmdResp(ApiFrame::ApiFrameType type) :
    FrameHandler(type), at_cmd_resp_cb(NULL)
{
}

/** Class destructor */
FH_AtCmdResp::~FH_AtCmdResp()
{
}

void FH_AtCmdResp::register_at_cmd_resp_cb(at_cmd_resp_cb_t function)
{
    at_cmd_resp_cb = function;
}

void FH_AtCmdResp::unregister_at_cmd_resp_cb()
{
    at_cmd_resp_cb = NULL;
}

void FH_AtCmdResp::process_frame_data(const ApiFrame * const frame)
{
    /* The caller checks that the type matches, so no need to check it here again */

    if (at_cmd_resp_cb == NULL) {
        return;
    }

    at_cmd_resp_cb(frame->get_data(), frame->get_data_len());
}


/** Class constructor */
FH_NodeDiscoveryZB::FH_NodeDiscoveryZB() :
    FH_AtCmdResp(ApiFrame::AtCmdResp), node_discovery_cb(NULL)
{
}

/** Class destructor */
FH_NodeDiscoveryZB::~FH_NodeDiscoveryZB()
{
}

void FH_NodeDiscoveryZB::register_node_discovery_cb(node_discovery_zb_cb_t function)
{
    node_discovery_cb = function;
}

void FH_NodeDiscoveryZB::unregister_node_discovery_cb()
{
    node_discovery_cb = NULL;
}


void FH_NodeDiscoveryZB::process_frame_data(const ApiFrame *const frame)
{
    /* The caller checks that the type matches, so no need to check it here again */

    if (node_discovery_cb == NULL) {
        return;
    }

    if (frame->get_data_at(ATCMD_RESP_CMD_LOW_OFFSET) != 'N' ||
        frame->get_data_at(ATCMD_RESP_CMD_HIGH_OFFSET) != 'D') {
        return;
    }

    if (frame->get_data_at(ATCMD_RESP_STATUS_OFFSET) != AtCmdFrame::AtCmdRespOk) {
        return;
    }

    const uint8_t * const data = frame->get_data(); /* The data payload we get here is the full AT command response payload, excluding the frameid. Keep that in mind for the offsets */
    const uint16_t addr16 = UINT16(data[ATCMD_RESP_NW_ADDR_H_OFFSET], data[ATCMD_RESP_NW_ADDR_L_OFFSET]);
    const uint64_t addr64 = addr64_from_uint8_t(&data[ATCMD_RESP_SH_ADDR_L_OFFSET]);
    const char * const node_id = (const char *)&data[ATCMD_RESP_NI_OFFSET];
    RemoteXBeeZB remote = RemoteXBeeZB(addr64, addr16);

    node_discovery_cb(remote, node_id);
}



/** Class constructor */
FH_NodeDiscovery802::FH_NodeDiscovery802() :
    FH_AtCmdResp(ApiFrame::AtCmdResp), node_discovery_cb(NULL)
{
}

/** Class destructor */
FH_NodeDiscovery802::~FH_NodeDiscovery802()
{
}

void FH_NodeDiscovery802::register_node_discovery_cb(node_discovery_802_cb_t function)
{
    node_discovery_cb = function;
}

void FH_NodeDiscovery802::unregister_node_discovery_cb()
{
    node_discovery_cb = NULL;
}


void FH_NodeDiscovery802::process_frame_data(const ApiFrame *const frame)
{
    /* The caller checks that the type matches, so no need to check it here again */

    if (node_discovery_cb == NULL) {
        return;
    }

    if (frame->get_data_at(ATCMD_RESP_CMD_LOW_OFFSET) != 'N' ||
        frame->get_data_at(ATCMD_RESP_CMD_HIGH_OFFSET) != 'D') {
        return;
    }

    if (frame->get_data_at(ATCMD_RESP_STATUS_OFFSET) != AtCmdFrame::AtCmdRespOk) {
        return;
    }

    const uint16_t min_atnd_response_with_data = sizeof (uint16_t) + sizeof(uint64_t);
    if (frame->get_data_len() < min_atnd_response_with_data) {
        /* Do not process the ATND "OK" response */
        return;
    }

    const uint8_t * const data = frame->get_data();
    const uint16_t addr16 = UINT16(data[ATCMD_RESP_NW_ADDR_H_OFFSET], data[ATCMD_RESP_NW_ADDR_L_OFFSET]);
    const uint64_t addr64 = addr64_from_uint8_t(&data[ATCMD_RESP_SH_ADDR_L_OFFSET]);
#if 0
    const uint8_t signal_strength = data[ATCMD_802_RESP_SIGN_STR_OFFSET];
#endif
    const RemoteXBee802 remote = RemoteXBee802(addr64, addr16);
    const char * const nodeid = (const char *)(&data[ATCMD_802_RESP_NI_OFFSET]);

    node_discovery_cb(remote, nodeid);
}

/** Class constructor */
FH_NodeDiscoveryDM::FH_NodeDiscoveryDM() :
    FH_AtCmdResp(ApiFrame::AtCmdResp), node_discovery_cb(NULL)
{
}

/** Class destructor */
FH_NodeDiscoveryDM::~FH_NodeDiscoveryDM()
{
}

void FH_NodeDiscoveryDM::register_node_discovery_cb(node_discovery_dm_cb_t function)
{
    node_discovery_cb = function;
}

void FH_NodeDiscoveryDM::unregister_node_discovery_cb()
{
    node_discovery_cb = NULL;
}


void FH_NodeDiscoveryDM::process_frame_data(const ApiFrame *const frame)
{
    /* The caller checks that the type matches, so no need to check it here again */

    if (node_discovery_cb == NULL) {
        return;
    }

    if (frame->get_data_at(ATCMD_RESP_CMD_LOW_OFFSET) != 'N' ||
        frame->get_data_at(ATCMD_RESP_CMD_HIGH_OFFSET) != 'D') {
        return;
    }

    if (frame->get_data_at(ATCMD_RESP_STATUS_OFFSET) != AtCmdFrame::AtCmdRespOk) {
        return;
    }

    const uint8_t * const data = frame->get_data(); /* The data payload we get here is the full AT command response payload, excluding the frameid. Keep that in mind for the offsets */
    const uint64_t addr64 = addr64_from_uint8_t(&data[ATCMD_RESP_SH_ADDR_L_OFFSET]);
    const char * const node_id = (const char *)&data[ATCMD_RESP_NI_OFFSET];
    RemoteXBeeDM remote = RemoteXBeeDM(addr64);

    node_discovery_cb(remote, node_id);
}