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

#include "XBeeDM.h"
#include "IO/IOSampleDM.h"
#include "Frames/DigiMeshFrames.h"

using namespace XBeeLib;

#define BROADCAST_RADIUS_USE_NH 0x00

/* Class constructor */
XBeeDM::XBeeDM(PinName tx, PinName rx, PinName reset, PinName rts, PinName cts, int baud) :
         XBee(tx, rx, reset, rts, cts, baud), _nd_handler(NULL), _rx_pkt_handler(NULL), _io_data_handler(NULL)
{
}

RadioStatus XBeeDM::init()
{
    RadioStatus retval = XBee::init();

    const RadioProtocol radioProtocol = get_radio_protocol();
    if (radioProtocol != DigiMesh) {
        digi_log(LogLevelError, "Radio protocol does not match, needed a %d got a %d\r\n", DigiMesh, radioProtocol);
        retval = Failure;
    }
    assert(radioProtocol == DigiMesh);

    return retval;
}

/* Class destructor */
XBeeDM::~XBeeDM()
{
    unregister_node_discovery_cb();
    unregister_receive_cb();
    unregister_io_sample_cb();
}

RadioStatus XBeeDM::set_channel(uint8_t  channel)
{
    AtCmdFrame::AtCmdResp cmdresp;

    /* Pro and Non-Pro modules have different channels available. The at 
       command will return an error if the selected channel is not available */
    cmdresp = set_param("CH", channel);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    return Success;
}

RadioStatus XBeeDM::get_channel(uint8_t * const  channel)
{
    if (channel == NULL) {
        return Failure;
    }
    AtCmdFrame::AtCmdResp cmdresp;

    uint32_t var32;
    cmdresp = get_param("CH", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *channel = var32;
    return Success;
}

RadioStatus XBeeDM::set_network_id(uint16_t network_id)
{
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("ID", network_id);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    return Success;
}

RadioStatus XBeeDM::get_network_id(uint16_t * const network_id)
{
    if (network_id == NULL) {
        return Failure;
    }
    AtCmdFrame::AtCmdResp cmdresp;

    uint32_t var32;
    cmdresp = get_param("ID", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *network_id = var32;
    return Success;
}

RadioStatus XBeeDM::get_node_discovery_timeout(uint16_t * const timeout_ms)
{
    AtCmdFrame::AtCmdResp cmdresp;
    uint32_t var32;

    cmdresp = get_param("N?", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *timeout_ms = (uint16_t)var32;

    return Success;
}

RadioStatus XBeeDM::get_node_discovery_timeout(uint16_t * const timeout_ms, bool * const wait_for_complete_timeout)
{
    const RadioStatus status = get_node_discovery_timeout(timeout_ms);

    *wait_for_complete_timeout = false;

    return status;
}

void XBeeDM::radio_status_update(AtCmdFrame::ModemStatus modem_status)
{
    /* Update the radio status variables */
    if (modem_status == AtCmdFrame::HwReset) {
        _hw_reset_cnt++;
    } else if (modem_status == AtCmdFrame::WdReset) {
        _wd_reset_cnt++;
    }

    _modem_status = modem_status;

    digi_log(LogLevelDebug, "\r\nUpdating radio status: %02x\r\n", modem_status);
}

TxStatus XBeeDM::send_data(const RemoteXBee& remote, const uint8_t *const data, uint16_t len, bool syncr)
{
    if (!remote.is_valid_addr64b()) {
        return TxStatusInvalidAddr;
    }

    const uint64_t remote64 = remote.get_addr64();
    const uint16_t remote16 = remote.get_addr16();

    TxFrameDM frame = TxFrameDM(remote64, remote16, BROADCAST_RADIUS_USE_NH,
                                _tx_options, data, len);
    if (syncr) {
        return send_data(&frame);
    } else {
        frame.set_data(0, 0); /* Set frame id to 0 so there is no answer */
        send_api_frame(&frame);
        return TxStatusSuccess;
    }
}

TxStatus XBeeDM::send_data(const RemoteXBee& remote, uint8_t source_ep,
                                uint8_t dest_ep, uint16_t cluster_id, uint16_t profile_id,
                                const uint8_t *const data, uint16_t len, bool syncr)
{
    if (!remote.is_valid_addr64b()) {
        return TxStatusInvalidAddr;
    }

    const uint64_t remote64 = remote.get_addr64();
    const uint16_t remote16 = remote.get_addr16();

    TxFrameDM frame = TxFrameDM(remote64, remote16, source_ep, dest_ep,
                                cluster_id, profile_id, BROADCAST_RADIUS_USE_NH,
                                _tx_options, data, len);
    if (syncr) {
        return send_data(&frame);
    } else {
        frame.set_data(0, 0); /* Set frame id to 0 so there is no answer */
        send_api_frame(&frame);
        return TxStatusSuccess;
    }
}

TxStatus XBeeDM::send_data_to_coordinator(const uint8_t *const data, uint16_t len, bool syncr)
{
    const uint64_t remaddr = ADDR64_COORDINATOR;

    TxFrameDM frame = TxFrameDM(remaddr, ADDR16_UNKNOWN, BROADCAST_RADIUS_USE_NH, _tx_options, data, len);
    if (syncr) {
        return send_data(&frame);
    } else {
        frame.set_data(0, 0); /* Set frame id to 0 so there is no answer */
        send_api_frame(&frame);
        return TxStatusSuccess;
    }
}

RemoteXBeeDM XBeeDM::get_remote_node_by_id(const char * const node_id)
{
    uint64_t addr64;
    uint16_t addr16;
    _get_remote_node_by_id(node_id, &addr64, &addr16);
    return RemoteXBeeDM(addr64);
}

void XBeeDM::register_node_discovery_cb(node_discovery_dm_cb_t function)
{
    if (_nd_handler == NULL) {
        _nd_handler = new FH_NodeDiscoveryDM();
        register_frame_handler(_nd_handler);
    }
    _nd_handler->register_node_discovery_cb(function);
}

void XBeeDM::unregister_node_discovery_cb()
{
    if (_nd_handler != NULL) {
        _nd_handler->unregister_node_discovery_cb();
        unregister_frame_handler(_nd_handler);
        delete _nd_handler;
        _nd_handler = NULL; /* as delete does not set to NULL */
    }
}

void XBeeDM::register_receive_cb(receive_dm_cb_t function)
{
    if (_rx_pkt_handler == NULL) {
        _rx_pkt_handler = new FH_RxPacketDM();
        register_frame_handler(_rx_pkt_handler);
    }
    _rx_pkt_handler->register_receive_cb(function);
}

void XBeeDM::unregister_receive_cb()
{
    if (_rx_pkt_handler != NULL) {
        _rx_pkt_handler->unregister_receive_cb();
        unregister_frame_handler(_rx_pkt_handler);
        delete _rx_pkt_handler;
        _rx_pkt_handler = NULL; /* as delete does not set to NULL */
    }
}

void XBeeDM::register_io_sample_cb(io_data_cb_dm_t function)
{
    if (_io_data_handler == NULL) {
        _io_data_handler = new FH_IoDataSampeDM();
        register_frame_handler(_io_data_handler);
    }
    _io_data_handler->register_io_data_cb(function);
}

void XBeeDM::unregister_io_sample_cb()
{
    if (_io_data_handler != NULL) {
        _io_data_handler->unregister_io_data_cb();
        unregister_frame_handler(_io_data_handler);
        delete _io_data_handler;
        _io_data_handler = NULL; /* as delete does not set to NULL */
    }
}

AtCmdFrame::AtCmdResp XBeeDM::get_param(const RemoteXBee& remote, const char * const param, uint32_t * const data)
{
    if (!remote.is_valid_addr64b()) {
        return AtCmdFrame::AtCmdRespInvalidAddr;
    }

    const uint64_t remote64 = remote.get_addr64();
    const uint16_t remote16 = remote.get_addr16();
    uint16_t len = sizeof *data;
    AtCmdFrame::AtCmdResp atCmdResponse;

    AtCmdFrame cmd_frame = AtCmdFrame(remote64, remote16, param);
    atCmdResponse = send_at_cmd(&cmd_frame, (uint8_t *)data, &len, RadioRemote);

    if (atCmdResponse == AtCmdFrame::AtCmdRespOk && len > sizeof *data) {
        atCmdResponse = AtCmdFrame::AtCmdRespLenMismatch;
    }

    return atCmdResponse;
}

AtCmdFrame::AtCmdResp XBeeDM::set_param(const RemoteXBee& remote, const char * const param, uint32_t data)
{
    if (!remote.is_valid_addr64b()) {
        return AtCmdFrame::AtCmdRespInvalidAddr;
    }

    const uint64_t remote64 = remote.get_addr64();
    const uint16_t remote16 = remote.get_addr16();

    AtCmdFrame cmd_frame = AtCmdFrame(remote64, remote16, param, data);
    return send_at_cmd(&cmd_frame, NULL, NULL, RadioRemote);
}

AtCmdFrame::AtCmdResp XBeeDM::set_param(const RemoteXBee& remote, const char * const param, const uint8_t * data, uint16_t len)
{
    if (!remote.is_valid_addr64b()) {
        return AtCmdFrame::AtCmdRespInvalidAddr;
    }

    const uint64_t remote64 = remote.get_addr64();
    const uint16_t remote16 = remote.get_addr16();

    AtCmdFrame cmd_frame = AtCmdFrame(remote64, remote16, param, data, len);
    return send_at_cmd(&cmd_frame, NULL, NULL, RadioRemote);
}

AtCmdFrame::AtCmdResp XBeeDM::get_param(const RemoteXBee& remote, const char * const param, uint8_t * const data, uint16_t * const len)
{

    if (!remote.is_valid_addr64b()) {
        return AtCmdFrame::AtCmdRespInvalidAddr;
    }

    const uint64_t remote64 = remote.get_addr64();
    const uint16_t remote16 = remote.get_addr16();

    AtCmdFrame cmd_frame = AtCmdFrame(remote64, remote16, param);
    return send_at_cmd(&cmd_frame, data, len, RadioRemote, false);
}

static void get_dio_cmd(XBeeDM::IoLine line, char * const iocmd)
{
    if (line >= XBeeDM::DIO10_PWM0) {
        iocmd[0] = 'P';
        iocmd[1] = '0' + line - XBeeDM::DIO10_PWM0;
    } else {
        iocmd[0] = 'D';
        iocmd[1] = '0' + line;
    }
    iocmd[2] = '\0';
}

RadioStatus XBeeDM::set_pin_config(const RemoteXBee& remote, IoLine line, IoMode mode)
{
    AtCmdFrame::AtCmdResp cmdresp;
    char iocmd[3];

    get_dio_cmd(line, iocmd);

    cmdresp = set_param(remote, iocmd, (uint8_t)mode);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "set_pin_config: set_param returned %d\r\n", cmdresp);
        return Failure;
    }

    return Success;
}

RadioStatus XBeeDM::get_pin_config(const RemoteXBee& remote, IoLine line, IoMode * const mode)
{
    AtCmdFrame::AtCmdResp cmdresp;
    char iocmd[3];

    get_dio_cmd(line, iocmd);

    uint32_t var32;
    cmdresp = get_param(remote, iocmd, &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *mode = (IoMode)var32;

    return Success;
}

RadioStatus XBeeDM::set_dio(const RemoteXBee& remote, IoLine line, DioVal val)
{
    return set_pin_config(remote, line, val == Low ? DigitalOutLow : DigitalOutHigh);
}

RadioStatus XBeeDM::get_dio(const RemoteXBee& remote, IoLine line, DioVal * const val)
{
    return get_iosample(remote).get_dio(line, val);
}

RadioStatus XBeeDM::get_adc(const RemoteXBee& remote, IoLine line, uint16_t * const val)
{
    return get_iosample(remote).get_adc(line, val);
}

RadioStatus XBeeDM::set_pwm(const RemoteXBee& remote, IoLine line, float duty_cycle)
{
    AtCmdFrame::AtCmdResp cmdresp;
    char iocmd[3] = { 'M', '0', '\0' };

    if (line != DIO10_PWM0 && line != DIO11_PWM1) {
        return Failure;
    }
    if (line == DIO11_PWM1) {
        iocmd[1] = '1';
    }

    uint16_t pwm_val = (uint16_t)(duty_cycle * DR_PWM_MAX_VAL / 100);

    cmdresp = set_param(remote, iocmd, pwm_val);
    return cmdresp == AtCmdFrame::AtCmdRespOk ? Success : Failure;
}

IOSampleDM XBeeDM::get_iosample(const RemoteXBee& remote)
{
    uint8_t io_sample[MAX_IO_SAMPLE_DM_LEN];
    uint16_t len = sizeof io_sample;

    RadioStatus resp = _get_iosample(remote, io_sample, &len);
    if (resp != Success) {
        digi_log(LogLevelError, "XBeeDM::get_iosample failed to get an IOSample\r\n");
        len = 0;
    }

    return IOSampleDM(io_sample, len);
}

static uint16_t get_dio_pr_mask(XBeeDM::IoLine line)
{
    switch (line) {
        case XBeeDM::DIO4:
            return (1 << 0);
        case XBeeDM::DIO3_AD3:
            return (1 << 1);
        case XBeeDM::DIO2_AD2:
            return (1 << 2);
        case XBeeDM::DIO1_AD1:
            return (1 << 3);
        case XBeeDM::DIO0_AD0:
            return (1 << 4);
        case XBeeDM::DIO6:
            return (1 << 5);
        case XBeeDM::DIO8:
            return (1 << 6);
        case XBeeDM::DIO5:
            return (1 << 8);
        case XBeeDM::DIO9:
            return (1 << 9);
        case XBeeDM::DIO12:
            return (1 << 10);
        case XBeeDM::DIO10_PWM0:
            return (1 << 11);
        case XBeeDM::DIO11_PWM1:
            return (1 << 12);
        case XBeeDM::DIO7:
            return (1 << 13);
        default:
            return 0;
    }
}

RadioStatus XBeeDM::set_pin_pull_up(const RemoteXBee& remote, IoLine line, bool enable)
{
    AtCmdFrame::AtCmdResp cmdresp;
    uint32_t var32;
    uint16_t pr;

    cmdresp = get_param(remote, "PR", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    pr = var32;

    const uint16_t dio_mask = get_dio_pr_mask(line);
    if (dio_mask == 0) {
        digi_log(LogLevelError, "XBeeDM::set_pin_pull_up: invalid pin %d\r\n", line);
        return Failure;
    }

    if (enable) {
        pr |= dio_mask;
    } else {
        pr &= ~dio_mask;
    }

    cmdresp = set_param(remote, "PR", pr);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }

    return Success;
}

static uint16_t get_dio_ic_mask(XBeeDM::IoLine line)
{
    if (line <= XBeeDM::DIO12) {
        return (1 << line);
    }
    return 0;
}

RadioStatus XBeeDM::enable_dio_change_detection(const RemoteXBee& remote, IoLine line, bool enable)
{
    if (line > DIO12) {
        digi_log(LogLevelError, "XBeeDM::enable_dio_change_detection: pin not supported (%d)\r\n", line);
        return Failure;
    }

    AtCmdFrame::AtCmdResp cmdresp;
    uint32_t var32;
    uint16_t ic;

    cmdresp = get_param(remote, "IC", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    ic = var32;

    const uint16_t dio_mask = get_dio_ic_mask(line);
    if (dio_mask == 0) {
        digi_log(LogLevelError, "XBeeDM::enable_dio_change_detection: invalid pin %d\r\n", line);
        return Failure;
    }

    if (enable) {
        ic |= dio_mask;
    } else {
        ic &= ~dio_mask;
    }

    cmdresp = set_param(remote, "IC", ic);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }

    return Success;
}

RadioStatus XBeeDM::config_poll_destination(const RemoteXBee& destination)
{
    uint32_t dh;
    uint32_t dl;

    if (destination.is_valid_addr64b()) {
        const uint64_t dest64 = destination.get_addr64();
        dh = (uint32_t)((dest64 >> 32) & 0xFFFFFFFF);
        dl = (uint32_t)((dest64 & 0xFFFFFFFF));
    } else {
        digi_log(LogLevelError, "config_poll_destination: Invalid destination");
        return Failure;
    }

    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("DH", dh);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "config_poll_destination: error %d:\r\n", cmdresp);
        return Failure;
    }

    cmdresp = set_param("DL", dl);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "config_poll_destination: error %d:\r\n", cmdresp);
        return Failure;
    }

    return Success;
}
