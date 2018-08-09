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
#include "XBee802.h"
#include "IO/IOSample802.h"
#include "Frames/802_Frames.h"
#include "FrameHandlers/FH_ModemStatus.h"

using namespace XBeeLib;

/* Class constructor */
XBee802::XBee802(PinName tx, PinName rx, PinName reset, PinName rts, PinName cts, int baud) :
        XBee(tx, rx, reset, rts, cts, baud),
        _nd_handler(NULL), _rx_64b_handler(NULL), _rx_16b_handler(NULL),
        _io_data_64b_handler(NULL), _io_data_16b_handler(NULL)
{

}

/* Class destructor */
XBee802::~XBee802()
{
    unregister_node_discovery_cb();
    unregister_receive_cb();
    unregister_io_sample_cb();
}

RadioStatus XBee802::init()
{
    RadioStatus retval = XBee::init();
    uint16_t addr16;
    RadioStatus error = get_network_address(&addr16);
    if (error == Success) {
        digi_log(LogLevelInfo, "ADDR16: %04x\r\n", addr16);
    } else {
        digi_log(LogLevelInfo, "ADDR16: UNKNOWN\r\n");
    }

    const RadioProtocol radioProtocol = get_radio_protocol();
    if (radioProtocol != Raw_802_15_4) {
        digi_log(LogLevelError, "Radio protocol does not match, needed a %d got a %d\r\n", Raw_802_15_4, radioProtocol);
        retval = Failure;
    }
    assert(radioProtocol == Raw_802_15_4);

    return retval;
}

RadioStatus XBee802::set_channel(uint8_t  channel)
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

RadioStatus XBee802::get_channel(uint8_t * const  channel)
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

RadioStatus XBee802::set_panid(uint16_t  panid)
{
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("ID", panid);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    return Success;
}

RadioStatus XBee802::get_panid(uint16_t * const  panid)
{
    if (panid == NULL) {
        return Failure;
    }
    AtCmdFrame::AtCmdResp cmdresp;

    uint32_t var32;
    cmdresp = get_param("ID", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *panid = var32;
    return Success;
}

RadioStatus XBee802::get_network_address(uint16_t * const  addr16)
{
    if (addr16 == NULL) {
        return Failure;
    }
    AtCmdFrame::AtCmdResp cmdresp;

    uint32_t var32;
    cmdresp = get_param("MY", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *addr16 = var32;
    return Success;
}

RadioStatus XBee802::set_network_address(uint16_t  addr16)
{
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("MY", addr16);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    return Success;
}

RadioStatus XBee802::get_node_discovery_timeout(uint16_t * const timeout_ms)
{
    AtCmdFrame::AtCmdResp cmdresp;
    uint32_t var32;

    cmdresp = get_param("NT", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *timeout_ms = (uint16_t)var32;

    /* No N? command available for this protocol. Add a fix 1s guard time */
    *timeout_ms += 1000;

    return Success;
}

RadioStatus XBee802::get_node_discovery_timeout(uint16_t * const timeout_ms, bool * const wait_for_complete_timeout)
{
    const RadioStatus status = get_node_discovery_timeout(timeout_ms);

    /* This protocol requires to wait for the complete timeout before attempting
       to execute other commands */
    *wait_for_complete_timeout = true;

    return status;
}

void XBee802::radio_status_update(AtCmdFrame::ModemStatus modem_status)
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

TxStatus XBee802::send_data(const RemoteXBee& remote, const uint8_t *const data, uint16_t len, bool syncr)
{
    if (remote.is_valid_addr64b()) {
        const uint64_t remote64 =  remote.get_addr64();

        digi_log(LogLevelDebug, "send_data ADDR64: %08x:%08x\r\n", UINT64_HI32(remote64), UINT64_LO32(remote64));

        TxFrame802 frame = TxFrame802(remote64, _tx_options, data, len);

        if (syncr) {
            return send_data(&frame);
        } else {
            frame.set_data(0, 0); /* Set frame id to 0 so there is no answer */
            send_api_frame(&frame);
            return TxStatusSuccess;
        }
    }

    if (remote.is_valid_addr16b()) {
        const uint16_t remote16 = remote.get_addr16();

        digi_log(LogLevelDebug, "send_data ADDR16: %04x\r\n", remote16);

        TxFrame802 frame = TxFrame802(remote16, _tx_options, data, len);

        if (syncr) {
            return send_data(&frame);
        } else {
            frame.set_data(0, 0); /* Set frame id to 0 so there is no answer */
            send_api_frame(&frame);
            return TxStatusSuccess;
        }
    }

    return TxStatusInvalidAddr;
}

XBee802::AssocStatus XBee802::get_assoc_status(void)
{
    return (AssocStatus)get_AI();
}

RemoteXBee802 XBee802::get_remote_node_by_id(const char * const node_id)
{
    uint64_t addr64;
    uint16_t addr16;

    _get_remote_node_by_id(node_id, &addr64, &addr16);
    return RemoteXBee802(addr64, addr16);
}

void XBee802::register_node_discovery_cb(node_discovery_802_cb_t function)
{
    if (_nd_handler == NULL) {
        _nd_handler = new FH_NodeDiscovery802();
        register_frame_handler(_nd_handler);
    }
    _nd_handler->register_node_discovery_cb(function);
}

void XBee802::unregister_node_discovery_cb()
{
    if (_nd_handler != NULL) {
        _nd_handler->unregister_node_discovery_cb();
        unregister_frame_handler(_nd_handler);
        delete _nd_handler;
        _nd_handler = NULL; /* as delete does not set to NULL */
    }
}

void XBee802::set_on_complete_callback(void (*cb)()) {
    _framebuf_app.cb_complete = cb;
}

void XBee802::register_receive_cb(receive_802_cb_t function)
{
    if (_rx_64b_handler == NULL) {
        _rx_64b_handler = new FH_RxPacket64b802();
        register_frame_handler(_rx_64b_handler);
    }
    _rx_64b_handler->register_receive_cb(function);

    if (_rx_16b_handler == NULL) {
        _rx_16b_handler = new FH_RxPacket16b802();
        register_frame_handler(_rx_16b_handler);
    }
    _rx_16b_handler->register_receive_cb(function);
}

void XBee802::unregister_receive_cb()
{
    if (_rx_64b_handler != NULL) {
        _rx_64b_handler->unregister_receive_cb();
        unregister_frame_handler(_rx_64b_handler);
        delete _rx_64b_handler;
        _rx_64b_handler = NULL; /* as delete does not set to NULL */
    }

    if (_rx_16b_handler != NULL) {
        _rx_16b_handler->unregister_receive_cb();
        unregister_frame_handler(_rx_16b_handler);
        delete _rx_16b_handler;
        _rx_16b_handler = NULL; /* as delete does not set to NULL */
    }
}

void XBee802::register_io_sample_cb(io_data_cb_802_t function)
{
    if (_io_data_64b_handler == NULL) {
        _io_data_64b_handler = new FH_IoDataSampe64b802();
        register_frame_handler(_io_data_64b_handler);
    }
    _io_data_64b_handler->register_io_data_cb(function);

    if (_io_data_16b_handler == NULL) {
        _io_data_16b_handler = new FH_IoDataSampe16b802();
        register_frame_handler(_io_data_16b_handler);
    }
    _io_data_16b_handler->register_io_data_cb(function);
}

void XBee802::unregister_io_sample_cb()
{
    if (_io_data_64b_handler != NULL) {
        _io_data_64b_handler->unregister_io_data_cb();
        unregister_frame_handler(_io_data_64b_handler);
        delete _io_data_64b_handler;
        _io_data_64b_handler = NULL; /* as delete does not set to NULL */
    }

    if (_io_data_16b_handler != NULL) {
        _io_data_16b_handler->unregister_io_data_cb();
        unregister_frame_handler(_io_data_16b_handler);
        delete _io_data_16b_handler;
        _io_data_16b_handler = NULL; /* as delete does not set to NULL */
    }
}

AtCmdFrame::AtCmdResp XBee802::get_param(const RemoteXBee& remote, const char * const param, uint32_t * const data)
{
    uint16_t len = sizeof *data;
    AtCmdFrame::AtCmdResp atCmdResponse;

    if (remote.is_valid_addr64b()) {
        const uint64_t dev_addr64 =  remote.get_addr64();

        AtCmdFrame cmd_frame = AtCmdFrame(dev_addr64, param);
        atCmdResponse = send_at_cmd(&cmd_frame, (uint8_t *)data, &len, RadioRemote);
    } else if (remote.is_valid_addr16b()) {
        const uint16_t dev_addr16 = remote.get_addr16();

        AtCmdFrame cmd_frame = AtCmdFrame(dev_addr16, param);
        atCmdResponse = send_at_cmd(&cmd_frame, (uint8_t *)data, &len, RadioRemote);
    } else {
        return AtCmdFrame::AtCmdRespInvalidAddr;
    }

    if (atCmdResponse == AtCmdFrame::AtCmdRespOk && len > sizeof *data) {
        atCmdResponse = AtCmdFrame::AtCmdRespLenMismatch;
    }

    return atCmdResponse;
}

AtCmdFrame::AtCmdResp XBee802::set_param(const RemoteXBee& remote, const char * const param, uint32_t data)
{
    if (remote.is_valid_addr64b()) {
        const uint64_t dev_addr64 =  remote.get_addr64();

        AtCmdFrame cmd_frame = AtCmdFrame(dev_addr64, param, data);
        return send_at_cmd(&cmd_frame, NULL, NULL, RadioRemote);
    }

    if (remote.is_valid_addr16b()) {
        const uint16_t dev_addr16 = remote.get_addr16();

        AtCmdFrame cmd_frame = AtCmdFrame(dev_addr16, param, data);
        return send_at_cmd(&cmd_frame, NULL, NULL, RadioRemote);
    }

    return AtCmdFrame::AtCmdRespInvalidAddr;
}

AtCmdFrame::AtCmdResp XBee802::set_param(const RemoteXBee& remote, const char * const param, const uint8_t * data, uint16_t len)
{
    if (remote.is_valid_addr64b()) {
        const uint64_t dev_addr64 =  remote.get_addr64();

        AtCmdFrame cmd_frame = AtCmdFrame(dev_addr64, param, data, len);
        return send_at_cmd(&cmd_frame, NULL, NULL, RadioRemote);
    }

    if (remote.is_valid_addr16b()) {
        const uint16_t dev_addr16 = remote.get_addr16();

        AtCmdFrame cmd_frame = AtCmdFrame(dev_addr16, param, data, len);
        return send_at_cmd(&cmd_frame, NULL, NULL, RadioRemote);
    }

    return AtCmdFrame::AtCmdRespInvalidAddr;
}

AtCmdFrame::AtCmdResp XBee802::get_param(const RemoteXBee& remote, const char * const param, uint8_t * const data, uint16_t * const len)
{
    if (remote.is_valid_addr64b()) {
        uint64_t dev_addr64 = remote.get_addr64();

        AtCmdFrame cmd_frame = AtCmdFrame(dev_addr64, param);
        return send_at_cmd(&cmd_frame, data, len, RadioRemote, false);
    }

    if (remote.is_valid_addr16b()) {
        uint16_t dev_addr16 = remote.get_addr16();

        AtCmdFrame cmd_frame = AtCmdFrame(dev_addr16, param);
        return send_at_cmd(&cmd_frame, data, len, RadioRemote, false);
    }

    return AtCmdFrame::AtCmdRespInvalidAddr;
}

static void get_dio_cmd(XBee802::IoLine line, char * const iocmd)
{
    if (line >= XBee802::PWM0) {
        iocmd[0] = 'P';
        iocmd[1] = '0' + line - XBee802::PWM0;
    } else {
        iocmd[0] = 'D';
        iocmd[1] = '0' + line;
    }
    iocmd[2] = '\0';
}

RadioStatus XBee802::set_pin_config(const RemoteXBee& remote, IoLine line, IoMode mode)
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

RadioStatus XBee802::get_pin_config(const RemoteXBee& remote, IoLine line, IoMode * const mode)
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

RadioStatus XBee802::set_dio(const RemoteXBee& remote, IoLine line, DioVal val)
{
    if (line > DI8) {
        digi_log(LogLevelError, "set_dio: Pin %d not supported as IO\r\n", line);
        return Failure;
    }

    if (val == Low) {
        return set_pin_config(remote, line, DigitalOutLow);
    } else {
        return set_pin_config(remote, line, DigitalOutHigh);
    }
}

RadioStatus XBee802::get_dio(const RemoteXBee& remote, IoLine line, DioVal * const val)
{
    return get_iosample(remote).get_dio(line, val);
}

RadioStatus XBee802::get_adc(const RemoteXBee& remote, IoLine line, uint16_t * const val)
{
    return get_iosample(remote).get_adc(line, val);
}

RadioStatus XBee802::set_pwm(const RemoteXBee& remote, IoLine line, float duty_cycle)
{
    AtCmdFrame::AtCmdResp cmdresp;
    char iocmd[3] = { 'M', '0', '\0' };

    if (line != PWM0 && line != PWM1) {
        return Failure;
    }
    if (line == PWM1) {
        iocmd[1] = '1';
    }

    uint16_t pwm_val = (uint16_t)(duty_cycle * DR_PWM_MAX_VAL / 100);

    cmdresp = set_param(remote, iocmd, pwm_val);
    return cmdresp == AtCmdFrame::AtCmdRespOk ? Success : Failure;
}

IOSample802 XBee802::get_iosample(const RemoteXBee& remote)
{
    uint8_t io_sample[MAX_IO_SAMPLE_802_LEN];
    uint16_t len = sizeof io_sample;

    RadioStatus resp = _get_iosample(remote, io_sample, &len);
    if (resp != Success) {
        digi_log(LogLevelError, "XBee802::get_iosample failed to get an IOSample\r\n");
        len = 0;
    }
    return IOSample802(io_sample, len);
}

static uint8_t get_dio_mask(XBee802::IoLine line)
{
    switch (line) {
        case XBee802::DIO4_AD4:
            return (1 << 0);
        case XBee802::DIO3_AD3:
            return (1 << 1);
        case XBee802::DIO2_AD2:
            return (1 << 2);
        case XBee802::DIO1_AD1:
            return (1 << 3);
        case XBee802::DIO0_AD0:
            return (1 << 4);
        case XBee802::DIO6:
            return (1 << 5);
        case XBee802::DI8:
            return (1 << 6);
        default:
            return 0;
    }
}

RadioStatus XBee802::set_pin_pull_up(const RemoteXBee& remote, IoLine line, bool enable)
{
    AtCmdFrame::AtCmdResp cmdresp;
    uint32_t var32;
    uint8_t pr;

    cmdresp = get_param(remote, "PR", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    pr = var32;

    const uint8_t dio_mask = get_dio_mask(line);
    if (dio_mask == 0) {
        digi_log(LogLevelError, "XBee802::set_pin_pull_up: invalid pin %d\r\n", line);
        return Failure;
    }

    if (enable) {
        pr |= dio_mask;
    } else {
        pr &= ~dio_mask;
    }

    cmdresp = set_param(remote, "PR", pr);
    return cmdresp == AtCmdFrame::AtCmdRespOk ? Success : Failure;
}

static uint8_t get_dio_ic_mask(XBee802::IoLine line)
{
    if (line < XBee802::DI8) {
        return (1 << line);
    }
    return 0;
}

RadioStatus XBee802::enable_dio_change_detection(const RemoteXBee& remote, IoLine line, bool enable)
{
    if (line > DIO7) {
        digi_log(LogLevelError, "XBee802::enable_dio_change_detection: pin not supported (%d)\r\n", line);
        return Failure;
    }

    AtCmdFrame::AtCmdResp cmdresp;
    uint32_t var32;
    uint8_t ic;

    cmdresp = get_param(remote, "IC", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    ic = var32;

    const uint8_t dio_mask = get_dio_ic_mask(line);
    if (dio_mask == 0) {
        digi_log(LogLevelError, "XBeeZB::enable_dio_change_detection: invalid pin %d\r\n", line);
        return Failure;
    }

    if (enable) {
        ic |= dio_mask;
    } else {
        ic &= ~dio_mask;
    }

    cmdresp = set_param(remote, "IC", ic);
    return cmdresp == AtCmdFrame::AtCmdRespOk ? Success : Failure;
}

#ifdef GET_PWM_AVAILABLE
RadioStatus XBee802::get_pwm(const RemoteXBee& remote, IoLine line, float * const duty_cycle)
{
    AtCmdFrame::AtCmdResp cmdresp;
    char iocmd[3] = { 'M', '0', '\0' };

    if (line != PWM0 && line != PWM1) {
        return Failure;
    }

    if (line == PWM1) {
        iocmd[1] = '1';
    }

    uint16_t pwm_val;

    cmdresp = get_param(remote, iocmd, &pwm_val);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }

    *duty_cycle = (float)(pwm_val * 100 / DR_PWM_MAX_VAL);

    return Success;
}
#endif
