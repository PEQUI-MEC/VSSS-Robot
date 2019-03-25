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
#include "FrameHandlers/FH_ModemStatus.h"

/* States for the state machine that processes incoming data on the serial port */
#define WAITING_FOR_START_FRAME (0)
#define WAITING_FOR_LENGTH_MSB  (1)
#define WAITING_FOR_LENGTH_LSB  (2)
#define WAITING_FOR_PAYLOAD     (3)
#define WAITING_FOR_CHECKSUM    (4)

#define IS_API2()               (_mode == ModeAPI2)
#define IS_API_MODE()           (_mode == ModeAPI1 || _mode == ModeAPI2)

using namespace XBeeLib;

#if defined(FRAME_BUFFER_SIZE_SYNCR)
#if FRAME_BUFFER_SIZE_SYNCR < 2
#error "FRAME_BUFFER_SIZE_SYNCR must be at least 2"
#endif
#else
#define FRAME_BUFFER_SIZE_SYNCR     1
#endif

#define MAX_FRAME_PAYLOAD_LEN_SYNCR (1  /* type */         + 1  /* id */       + 2 /* at cmd*/    + 1 /* status */    + 2 /* MY sender */ + \
                                     8  /* 64b sender */   + 20 /* max id */   + 1 /* null ter */ + 2 /* MY parent */ + 1  /* dev type */ + \
                                     1  /* source event */ + 2  /* prof. id */ + 2 /* man. id */)

FrameBuffer XBee::_framebuf_app(FRAME_BUFFER_SIZE, MAX_FRAME_PAYLOAD_LEN);
FrameBuffer XBee::_framebuf_syncr(FRAME_BUFFER_SIZE_SYNCR, MAX_FRAME_PAYLOAD_LEN_SYNCR);

#if defined(DEVICE_SERIAL_FC)
bool XBee::check_radio_flow_control()
{
    AtCmdFrame::AtCmdResp cmdresp;
    uint32_t value;

    if (_serial_flow_type == SerialBase::RTSCTS || _serial_flow_type == SerialBase::CTS) {
        cmdresp = get_param("D7", &value);
        if (cmdresp != AtCmdFrame::AtCmdRespOk) {
            digi_log(LogLevelError, "Could not read CTS configuration. Error %d\r\n", cmdresp);
            return false;
        } else if (value != 1) {
            digi_log(LogLevelError, "Bad CTS configuration. Radio 'D7' param is %d and should be 1\r\n", value);
            return false;
        }
    }

    if (_serial_flow_type == SerialBase::RTSCTS || _serial_flow_type == SerialBase::RTS) {
        cmdresp = get_param("D6", &value);
        if (cmdresp != AtCmdFrame::AtCmdRespOk) {
            digi_log(LogLevelError, "Could not read RTS configuration. Error %d\r\n", cmdresp);
            return false;
        } else if (value != 1) {
            digi_log(LogLevelError, "Bad RTS configuration. Radio 'D6' param is %d and should be 1\r\n", value);
            return false;
        }
    }

    return true;
}
#endif

/* Class constructor */
XBee::XBee(PinName tx, PinName rx, PinName reset, PinName rts, PinName cts, int baud) :
    _mode(ModeUnknown), _hw_version(0), _fw_version(0), _timeout_ms(SYNC_OPS_TIMEOUT_MS), _dev_addr64(ADDR64_UNASSIGNED),
    _reset(NULL), _tx_options(0), _hw_reset_cnt(0), _wd_reset_cnt(0), _modem_status_handler(NULL), _modem_status(AtCmdFrame::HwReset), _initializing(true), _node_by_ni_frame_id(0)
{

    if (reset != NC) {
        _reset = new DigitalOut(reset, 1);
    }

    _uart = new RawSerial(tx, rx);
    _uart->baud(baud);

    _serial_flow_type = SerialBase::Disabled;
#if defined(DEVICE_SERIAL_FC)
    if (rts != NC && cts != NC) {
        _serial_flow_type = SerialBase::RTSCTS;
        _uart->set_flow_control(_serial_flow_type, rts, cts);
    } else if (rts != NC && cts == NC) {
        _serial_flow_type = SerialBase::RTS;
        _uart->set_flow_control(_serial_flow_type, rts);
    } else if (rts == NC && cts != NC) {
        _serial_flow_type = SerialBase::CTS;
        _uart->set_flow_control(_serial_flow_type, cts);
    }
#endif
    /* Enable the reception of bytes on the serial interface by providing a cb */
    _uart->attach(callback(this, &XBee::uart_read_cb), Serial::RxIrq);

    for (int i = 0; i < MAX_FRAME_HANDLERS; i++) {
        _fhandlers[i] = NULL;
    }
}

/* Class destructor */
XBee::~XBee()
{
    unregister_modem_status_cb();

    if (_uart != NULL) {
        delete _uart;
    }
    if (_reset != NULL) {
        delete _reset;
    }
}

#include <inttypes.h>

RadioStatus XBee::init(void)
{
    AtCmdFrame::AtCmdResp cmd_resp;
    uint32_t var32;

    _initializing = true;

    const unsigned int max_reset_retries = 3;
    RadioStatus reset_status;
    for (unsigned int i = 0; i < max_reset_retries; i++) {
        reset_status = device_reset();
        if (reset_status == Success) {
            break;
        }
    }
    if (reset_status != Success) {
        return reset_status;
    }

    /* Check if radio is in API1 or API2 _mode */
    cmd_resp = get_param("AP", &var32);
    if (cmd_resp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    _mode = (RadioMode)var32;

    /* Read the device unique 64b address */
    uint32_t serialn_high, serialn_low;
    cmd_resp = get_param("SH", &serialn_high);
    if (cmd_resp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }

    cmd_resp = get_param("SL", &serialn_low);
    if (cmd_resp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }

    _dev_addr64 = ((uint64_t)serialn_high << 32) | serialn_low;

    /* Read some important parameters */
    cmd_resp = get_param("HV", &var32);
    if (cmd_resp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    _hw_version = var32;

    cmd_resp = get_param("VR", &var32);
    if (cmd_resp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    _fw_version = var32;

    digi_log(LogLevelInfo, "mode:   %02x\r\n", (uint8_t)_mode);
    digi_log(LogLevelInfo, "HV:     %04x\r\n", _hw_version);
    digi_log(LogLevelInfo, "VR:     %04x\r\n", _fw_version);
    digi_log(LogLevelInfo, "ADDR64: %08x:%08x\r\n", UINT64_HI32(_dev_addr64), UINT64_LO32(_dev_addr64));

#if defined(DEVICE_SERIAL_FC)
    bool valid_radio_fc = check_radio_flow_control();
    assert(valid_radio_fc == true);
#endif

    _initializing = false;
    if (_modem_status_handler != NULL) {
        const ApiFrame frame = ApiFrame(ApiFrame::AtModemStatus, (uint8_t *)&_modem_status, sizeof(_modem_status));
        _modem_status_handler->process_frame_data(&frame);
    }

    return Success;
}

uint64_t XBee::get_addr64() const
{
    return _dev_addr64;
}

RadioStatus XBee::hardware_reset()
{
    if (_reset != NULL) {
        volatile uint16_t * const rst_cnt_p = &_hw_reset_cnt;
        const uint16_t init_rst_cnt = *rst_cnt_p;
        *_reset = 0;
        wait_ms(10);
        *_reset = 1;
        return wait_for_module_to_reset(rst_cnt_p, init_rst_cnt);
    }

    return Failure;
}

RadioStatus XBee::device_reset()
{
    if (hardware_reset() == Success) {
        return Success;
    }

    return software_reset();
}

RadioStatus XBee::wait_for_module_to_reset(volatile uint16_t *rst_cnt_p, uint16_t init_rst_cnt)
{
    Timer timer;
    timer.start();

    while (*rst_cnt_p == init_rst_cnt && timer.read_ms() < RESET_TIMEOUT_MS) {
        wait_ms(100);
    }

    if (*rst_cnt_p == init_rst_cnt) {
        digi_log(LogLevelWarning, "Reset Timeout\r\n");
        return Failure;
    }
    return Success;
}

/** Callback function called when data is received on the serial port */
void XBee::uart_read_cb(void)
{
    static uint8_t rxstate = WAITING_FOR_START_FRAME;
    static uint16_t framelen = 0;
    static uint16_t bytes_read;
    static uint8_t chksum;
    static ApiFrame *frame = NULL;
    static bool last_byte_escaped = false;
    static FrameBuffer * framebuf = NULL;

    while (_uart->readable()) {
        uint8_t data = _uart->getc();

        if (IS_API2() && rxstate != WAITING_FOR_START_FRAME) {
            if (last_byte_escaped) {
                data = data ^ DR_ESCAPE_XOR_BYTE;
                last_byte_escaped = false;
            } else if (data == DR_ESCAPE_BYTE) {
                last_byte_escaped = true;
                continue;
            }
        }

        switch (rxstate) {
            case WAITING_FOR_START_FRAME:
                if (data == DR_START_OF_FRAME) {
                    rxstate = WAITING_FOR_LENGTH_MSB;
                }
                break;

            case WAITING_FOR_LENGTH_MSB:
                framelen = data << 8;
                rxstate = WAITING_FOR_LENGTH_LSB;
                break;

            case WAITING_FOR_LENGTH_LSB:
                framelen |= data;
                rxstate = WAITING_FOR_PAYLOAD;
                bytes_read = 0;
                chksum = 0;
                /* Sanity check that the frame is smaller than... */
                if (framelen > MAX_FRAME_PAYLOAD_LEN) {
                    digi_log(LogLevelDebug, "framelen=%d too long\r\n", framelen);
                    digi_log(LogLevelWarning, "Frame dropped, frame too long. Increase MAX_FRAME_PAYLOAD_LEN define\r\n");
                    rxstate = WAITING_FOR_START_FRAME;
                }
                break;

            case WAITING_FOR_PAYLOAD:
                #define CACHED_SIZE 3
                static uint8_t frame_cached[CACHED_SIZE];

                if (framelen <= CACHED_SIZE) {
                    if (!bytes_read) {
                        const ApiFrame::ApiFrameType frame_type = (ApiFrame::ApiFrameType)data;
                        switch (frame_type)
                        {
                            case ApiFrame::AtCmdResp:
                            case ApiFrame::RemoteCmdResp:
                            case ApiFrame::TxStatusZBDM:
                            case ApiFrame::TxStatus:
                                framebuf = &_framebuf_syncr;
                                break;

                            case ApiFrame::RxPacket64Bit:
                            case ApiFrame::RxPacket16Bit:
                            case ApiFrame::Io64Bit:
                            case ApiFrame::Io16Bit:
                            case ApiFrame::AtModemStatus:
                            case ApiFrame::RxPacketAO0:
                            case ApiFrame::IoSampleRxZBDM:
                                framebuf = &_framebuf_app;
                                break;

                            case ApiFrame::RxPacketAO1:
                            case ApiFrame::SensorRxIndAO0:
                            case ApiFrame::NodeIdentIndAO0:
                            case ApiFrame::OtaFwUpStatus:
                            case ApiFrame::RouteRecInd:
                            case ApiFrame::Many2OneRRInd:
                            case ApiFrame::TxReq64Bit:
                            case ApiFrame::TxReq16Bit:
                            case ApiFrame::AtCmd:
                            case ApiFrame::AtCmdQueuePV:
                            case ApiFrame::TxReqZBDM:
                            case ApiFrame::ExpAddrCmd:
                            case ApiFrame::RemoteCmdReq:
                            case ApiFrame::CreateSrcRoute:
                            case ApiFrame::Invalid:
                            case ApiFrame::RouteInfo:
                            case ApiFrame::AggregateAddr:
                                framebuf = NULL;
                                break;
                        }

                        if (framebuf == NULL) {
                            digi_log(LogLevelWarning, "Discarding not supported frame type %02x\r\n", frame_type);
                            rxstate = WAITING_FOR_START_FRAME;
                        } else {
                            frame = framebuf->get_next_free_frame();
                            if (frame == NULL) {
                                /* It's not possible to achive this condition as we discard older frames and only one frame can be used by syncr. commands */
                                assert(frame != NULL);
                                rxstate = WAITING_FOR_START_FRAME;
                            } else {
                                frame->set_data_len(framelen - 1);
                            }

                            frame->set_frame_type(frame_type);
                        }
                    } else {
                        frame->set_data(data, bytes_read - 1);
                    }
                    chksum += data;
                    bytes_read++;
                    if (bytes_read == framelen) {
                        rxstate = WAITING_FOR_CHECKSUM;
                    }
                    break;
                }


                if (bytes_read < CACHED_SIZE) {
                    frame_cached[bytes_read] = data;
                }
                else if (bytes_read == CACHED_SIZE) {
                    const ApiFrame::ApiFrameType frame_type = (ApiFrame::ApiFrameType)frame_cached[0];
                    switch (frame_type)
                    {
                        case ApiFrame::RemoteCmdResp:
                        case ApiFrame::TxStatusZBDM:
                        case ApiFrame::TxStatus:
                            framebuf = &_framebuf_syncr;
                            break;

                        case ApiFrame::AtCmdResp:
                            if ((frame_cached[1] != _node_by_ni_frame_id ) && (frame_cached[2] == 'N') && (data == 'D'))
                            {
                                framebuf = &_framebuf_app;
                            } else {
                                framebuf = &_framebuf_syncr;
                            }
                            break;

                        case ApiFrame::RxPacket64Bit:
                        case ApiFrame::RxPacket16Bit:
                        case ApiFrame::Io64Bit:
                        case ApiFrame::Io16Bit:
                        case ApiFrame::AtModemStatus:
                        case ApiFrame::RxPacketAO0:
                        case ApiFrame::IoSampleRxZBDM:
                            framebuf = &_framebuf_app;
                            break;

                        case ApiFrame::RxPacketAO1:
                        case ApiFrame::SensorRxIndAO0:
                        case ApiFrame::NodeIdentIndAO0:
                        case ApiFrame::OtaFwUpStatus:
                        case ApiFrame::RouteRecInd:
                        case ApiFrame::Many2OneRRInd:
                        case ApiFrame::TxReq64Bit:
                        case ApiFrame::TxReq16Bit:
                        case ApiFrame::AtCmd:
                        case ApiFrame::AtCmdQueuePV:
                        case ApiFrame::TxReqZBDM:
                        case ApiFrame::ExpAddrCmd:
                        case ApiFrame::RemoteCmdReq:
                        case ApiFrame::CreateSrcRoute:
                        case ApiFrame::Invalid:
                        case ApiFrame::RouteInfo:
                        case ApiFrame::AggregateAddr:
                            framebuf = NULL;
                            break;
                    }

                    if (framebuf == NULL) {
                        digi_log(LogLevelWarning, "Discarding not supported frame type %02x\r\n", frame_type);
                        rxstate = WAITING_FOR_START_FRAME;
                    } else {
                        frame = framebuf->get_next_free_frame();
                        if (frame == NULL) {
                            /* It's not possible to achive this condition as we discard older frames and only one frame can be used by syncr. commands */
                            assert(frame != NULL);
                            rxstate = WAITING_FOR_START_FRAME;
                        } else {
                            frame->set_data_len(framelen - 1);
                        }

                        frame->set_frame_type(frame_type);
                        frame->set_data(frame_cached[1], 0);
                        frame->set_data(frame_cached[2], 1);
                        frame->set_data(data, 2);
                    }
                } else {
                    frame->set_data(data, bytes_read - 1);
                }
                chksum += data;
                bytes_read++;
                if (bytes_read == framelen) {
                    rxstate = WAITING_FOR_CHECKSUM;
                }
                break;

            case WAITING_FOR_CHECKSUM:
                chksum += data;
                if (chksum == 0xFF) {
                    /* We got a valid frame!! */
                    frame->dump();

                    /* If its a _modem status frame, process it to update the status info of the library.
                     * The frame is also queued to allow processing it other handlers registered.
                     * Note that radio_status_update() has to be fast to minimize the impact of processing
                     * the funcion here */
                    if (frame->get_frame_type() == ApiFrame::AtModemStatus) {
                        radio_status_update((AtCmdFrame::ModemStatus)frame->get_data_at(0));
                        if (_initializing) {
                            framebuf->free_frame(frame);
                        } else {
                            framebuf->complete_frame(frame);
                        }
                    } else {
                        framebuf->complete_frame(frame);
                        /* Note, the frame will be released elsewhere, once it has been processed */
                    }
                } else {
                    framebuf->free_frame(frame);
                    digi_log(LogLevelWarning, "Checksum error, got %02x, %02x\r\n", data, chksum);
                }
                /* Intentional fall-through */
            default:
                rxstate = WAITING_FOR_START_FRAME;
                break;
        }
    }
    /* TODO, signal the thread processing incoming frames */
}

/* This is a pure virtual function, but exists here because its called from this class to
 * to update the status of the object, and can be called before the construction of the
 * object has been completed and the virtual functions filled */
void XBee::radio_status_update(AtCmdFrame::ModemStatus modem_status)
{
    UNUSED_PARAMETER(modem_status);
}

void XBee::set_timeout(uint16_t timeout_ms)
{
    this->_timeout_ms = timeout_ms;
}

uint16_t XBee::get_timeout(void) const
{
    return _timeout_ms;
}

ApiFrame * XBee::get_this_api_frame(uint8_t id, ApiFrame::ApiFrameType type,
                                          ApiFrame::ApiFrameType type2)
{
    Timer timer;
    timer.start();

    while (timer.read_ms() < _timeout_ms) {
        ApiFrame * frame = _framebuf_syncr.get_next_complete_frame();
        if (frame == NULL) {
            Thread::wait(1);
            continue;
        }

        if ((frame->get_frame_type() != type) &&
            (frame->get_frame_type() != type2)) {
            _framebuf_syncr.complete_frame(frame);
            Thread::wait(1);
            continue;
        }

        if (frame->get_data_at(ATCMD_RESP_FRAME_ID_OFFSET) != id) {
            _framebuf_syncr.complete_frame(frame);
            Thread::wait(1);
            continue;
        }

        /* frame found */
        return frame;
    }

    digi_log(LogLevelWarning, "Frame type: %02x, id: %02x, timeout\r\n", (uint8_t)type, id);

    return NULL;
}

void XBee::send_byte_escaping_if(uint8_t data)
{
    if (IS_API2()) {
        switch (data) {
            case DR_START_OF_FRAME:
            case DR_ESCAPE_BYTE:
            case DR_XON_BYTE:
            case DR_XOFF_BYTE:
                _uart->putc(DR_ESCAPE_BYTE);
                _uart->putc(data ^ DR_ESCAPE_XOR_BYTE);
                break;
            default:
                _uart->putc(data);
        }
    } else {
        _uart->putc(data);
    }
}

void XBee::send_api_frame(ApiFrame *frame)
{
    uint8_t chksum;
    const uint8_t *data;
    uint16_t bytes_sent = 0, frame_len;

    frame->dump();

    frame_len = 1 + frame->get_data_len(); /* frame type + frame payload */
    data = frame->get_data();

    /* Send the start of frame delimiter */
    _uart->putc(DR_START_OF_FRAME);

    /* Now the length */
    send_byte_escaping_if((uint8_t)(frame_len >> 8));
    send_byte_escaping_if((uint8_t)frame_len);

    /* Send the Frame type and then the payload */
    chksum = (uint8_t)frame->get_frame_type();
    send_byte_escaping_if(chksum);
    bytes_sent++;

    /* And now, send the packet payload */
    while (bytes_sent++ < frame_len) {
        chksum += *data;
        send_byte_escaping_if(*data++);
    }

    /* And finally send the checksum */
    send_byte_escaping_if(~chksum);
}

RadioStatus XBee::register_frame_handler(FrameHandler *const handler)
{
    if (handler != NULL) {
        for (int i = 0; i < MAX_FRAME_HANDLERS; i++) {
            if (_fhandlers[i] != NULL) {
                continue;
            }
            _fhandlers[i] = handler;
            return Success;
        }
    }

    digi_log(LogLevelError, "No more Frame Handlers available. Increase MAX_FRAME_HANDLERS define\r\n");

    return Failure;
}

RadioStatus XBee::unregister_frame_handler(FrameHandler *const handler)
{
    int i;

    if (handler != NULL) {
        for (i = 0; i < MAX_FRAME_HANDLERS; i++) {
            if (_fhandlers[i] == handler) {
                break;
            }
        }

        if (i == MAX_FRAME_HANDLERS) {
            return Failure;
        }

        do {
            if (i == MAX_FRAME_HANDLERS - 1) {
                _fhandlers[i] = NULL;
            } else {
                _fhandlers[i] = _fhandlers[i + 1];
            }
        } while (++i < MAX_FRAME_HANDLERS);
    }

    return Success;
}

XBee::RadioProtocol XBee::get_radio_protocol(void) const
{
    enum HardwareVersion {
#ifdef EXTRA_XBEE_PROTOCOLS
        X09_009 = 0x01,
        X09_019 = 0x02,
        XH9_009 = 0x03,
        XH9_019 = 0x04,
        X24_009 = 0x05,
        X24_019 = 0x06,
        X09_001 = 0x07,
        XH9_001 = 0x08,
        X08_004 = 0x09,
        XC09_009 = 0x0A,
        XC09_038 = 0x0B,
        X24_038 = 0x0C,
        X09_009_TX = 0x0D,
        X09_019_TX = 0x0E,
        XH9_009_TX = 0x0F,
        XH9_019_TX = 0x10,
        X09_001_TX = 0x11,
        XH9_001_TX = 0x12,
        XT09B_XXX = 0x13,
        XT09_XXX = 0x14,
        XC08_009 = 0x15,
        XC08_038 = 0x16,
#endif
        XB24_AXX_XX = 0x17,
        XBP24_AXX_XX = 0x18,
        XB24_BXIX_XXX = 0x19,
        XBP24_BXIX_XXX = 0x1A,
#ifdef EXTRA_XBEE_PROTOCOLS
        XBP09_DXIX_XXX = 0x1B,
        XBP09_XCXX_XXX = 0x1C,
        XBP08_DXXX_XXX = 0x1D,
#endif
        XBP24B = 0x1E,
#ifdef EXTRA_XBEE_PROTOCOLS
        XB24_WF = 0x1F,
        AMBER_MBUS = 0x20,
#endif
        XBP24C = 0x21,
        XB24C = 0x22,
#ifdef EXTRA_XBEE_PROTOCOLS
        XSC_GEN3 = 0x23,
        SRD_868_GEN3 = 0x24,
        ABANDONATED = 0x25,
        SMT_900LP = 0x26,
        WIFI_ATHEROS = 0x27,
        SMT_WIFI_ATHEROS = 0x28,
        SMT_475LP = 0x29,
        XBEE_CELL_TH = 0x2A,
        XLR_MODULE = 0x2B,
        XB900HP_NZ = 0x2C,
        XBP24C_TH_DIP = 0x2D,
        XB24C_TH_DIP = 0x2E,
        XLR_BASEBOARD = 0x2F,
        XBP24C_S2C_SMT = 0x30
#endif
    };
    const bool fw_4_bytes_len = _fw_version > 0x0FFF && _fw_version < 0xFFFF;
    const uint8_t fw_nibble_3 = (_fw_version >> (4 * 3)) & 0x000F;
    const uint8_t fw_nibble_1 = (_fw_version >> (4 * 1)) & 0x000F;
    const uint8_t fw_nibble_0 = (_fw_version >> (4 * 0)) & 0x000F;
    const uint8_t hw_version_msb = _hw_version >> 8;

    if (hw_version_msb == XB24_AXX_XX || hw_version_msb == XBP24_AXX_XX) {
#ifdef EXTRA_XBEE_PROTOCOLS
        if (fw_4_bytes_len && fw_nibble_3 == 8) {
            return DigiMesh;
        }
        return Raw_802_15_4;
#else
        if (!(fw_4_bytes_len && fw_nibble_3 == 8)) {
            return Raw_802_15_4;
        }
#endif
    } else if (hw_version_msb == XB24_BXIX_XXX || hw_version_msb == XBP24_BXIX_XXX) {
        if (fw_4_bytes_len && ((fw_nibble_3 == 1 && fw_nibble_1 == 2 && fw_nibble_0 == 0) || fw_nibble_3 == 2)) {
            return ZigBee;
        }
#ifdef EXTRA_XBEE_PROTOCOLS
        if (fw_4_bytes_len && fw_nibble_3 == 3) {
            return SmartEnergy;
        }
        return ZNet;
    } else if (hw_version_msb == XBP09_DXIX_XXX) {
        if (fw_4_bytes_len && (fw_nibble_3 == 8 || fw_nibble_1 == 8))  {
            return DigiMesh;
        }
        return DigiPoint;
    } else if (hw_version_msb == XBP08_DXXX_XXX) {
        return DigiPoint;
#endif
    } else if (hw_version_msb == XBP24B) {
#ifdef EXTRA_XBEE_PROTOCOLS
        if (fw_4_bytes_len && fw_nibble_3 == 3) {
            return SmartEnergy;
        }
        return ZigBee;
#else
        if (!(fw_4_bytes_len && fw_nibble_3 == 3)) {
            return ZigBee;
        }
#endif
#ifdef EXTRA_XBEE_PROTOCOLS
    } else if (hw_version_msb == XB24_WF || hw_version_msb == WIFI_ATHEROS || hw_version_msb == SMT_WIFI_ATHEROS) {
        return XBeeWiFi;
#endif
    } else if (hw_version_msb == XBP24C || hw_version_msb == XB24C) {
        if (fw_4_bytes_len && fw_nibble_3 == 2) {
            return Raw_802_15_4;
        }
#ifdef EXTRA_XBEE_PROTOCOLS
        if (fw_4_bytes_len && fw_nibble_3 == 5) {
            return SmartEnergy;
        }
        return ZigBee;
#else
        if (!(fw_4_bytes_len && fw_nibble_3 == 5)) {
            return ZigBee;
        }
#endif
#ifdef EXTRA_XBEE_PROTOCOLS
    } else if (hw_version_msb == XSC_GEN3 || hw_version_msb == SRD_868_GEN3) {
        if (fw_4_bytes_len && fw_nibble_3 == 8) {
            return DigiMesh;
        } else if (fw_4_bytes_len && fw_nibble_3 == 1) {
            return DigiPoint;
        }
        return None;
    } else if (hw_version_msb == XBEE_CELL_TH) {
        return None;
    } else if (hw_version_msb == XLR_MODULE) {
        return None;
    } else if (hw_version_msb == XLR_BASEBOARD) {
        return None;
    } else if (hw_version_msb == XB900HP_NZ) {
        return DigiPoint;
    } else if (hw_version_msb == XBP24C_TH_DIP || hw_version_msb == XB24C_TH_DIP || hw_version_msb == XBP24C_S2C_SMT) {
        if (fw_4_bytes_len && fw_nibble_3 == 9) {
            return DigiMesh;
        }
        if (fw_4_bytes_len && fw_nibble_3 == 5) {
            return SmartEnergy;
        }
        if (fw_4_bytes_len && fw_nibble_3 == 2) {
            return Raw_802_15_4;
        }
        return ZigBee;
    }
#else
    }
#endif

    return None;
}

#define TX_STATUS_OFFSET_ZB     4
#define TX_STATUS_OFFSET_802    1

TxStatus XBee::send_data(ApiFrame *frame)
{
    TxStatus resp = TxStatusTimeout;
    ApiFrame *resp_frame;

    send_api_frame(frame);

    /* Wait for the transmit status response packet */
    resp_frame = get_this_api_frame(frame->get_frame_id(),
                    ApiFrame::TxStatusZBDM, ApiFrame::TxStatus);
    if (resp_frame == NULL) {
        return resp;
    }

    uint8_t index = resp_frame->get_frame_type() == ApiFrame::TxStatusZBDM ?
            TX_STATUS_OFFSET_ZB : TX_STATUS_OFFSET_802;

    resp = (TxStatus)resp_frame->get_data_at(index);

    /* Once processed, remove the frame from the buffer */
    _framebuf_syncr.free_frame(resp_frame);

    return resp;
}

TxStatus XBee::send_data_broadcast(const uint8_t *const data, uint16_t len, bool syncr)
{
    const RemoteXBee remoteDevice = RemoteXBee(ADDR64_BROADCAST);
    return send_data(remoteDevice, data, len, syncr);
}

uint32_t XBee::process_rx_frames()
{
    ApiFrame *frame = NULL;

    while ((frame = _framebuf_app.get_next_complete_frame()) != NULL) {
        for (int i = 0; i < MAX_FRAME_HANDLERS; i++) {

            if (_fhandlers[i] == NULL) {
                /* No more handlers, break here */
                break;
            }

            /* Check if frame and handler match, if not... go for the next one */
            if (frame->get_frame_type() != _fhandlers[i]->get_type()) {
                continue;
            }

            _fhandlers[i]->process_frame_data(frame);
        }

        /* Once processed, remove the frame from the buffer */
        _framebuf_app.free_frame(frame);
    }

    const uint32_t dropped_frames = _framebuf_app.get_dropped_frames_count();
    if (dropped_frames != 0) {
        digi_log(LogLevelWarning, "process_rx_frames: %d frames dropped!!!\r\n", dropped_frames);
    }

    return dropped_frames;
}

void XBee::register_modem_status_cb(modem_status_cb_t function)
{
    if (_modem_status_handler == NULL) {
        _modem_status_handler = new FH_ModemStatus();
        register_frame_handler(_modem_status_handler);
    }
    _modem_status_handler->register_modem_status_cb(function);
}

void XBee::unregister_modem_status_cb()
{
    if (_modem_status_handler != NULL) {
        _modem_status_handler->unregister_modem_status_cb();
        unregister_frame_handler(_modem_status_handler);
        delete _modem_status_handler;
        _modem_status_handler = NULL; /* as delete does not set to NULL */
    }
}

int XBee::get_AI(void)
{
    uint32_t atai;
    const AtCmdFrame::AtCmdResp status = get_param("AI", &atai);

    if (status != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "get_association_indication() failed with %d\r\n", status);
        return -1;
    }
    return atai;
}
