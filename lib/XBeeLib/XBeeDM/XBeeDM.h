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

#if !defined(__XBEE_DM_H_)
#define __XBEE_DM_H_

#include "XBee/XBee.h"
#include "FrameHandlers/FH_AtCmdResp.h"
#include "FrameHandlers/FH_RxPacketDM.h"
#include "FrameHandlers/FH_IoDataSampleDM.h"
#include "RemoteXBee/RemoteXBee.h"

namespace XBeeLib {

/** Class for XBee ZigBee modules, derived from XBee */
class XBeeDM : public XBee
{
    public:

        /**
         * IoLine for XBeeDM Modules
         */
        enum IoLine {
            DIO0_AD0   = 0,  /**< DIO0_AD0 pin */
            DIO1_AD1   = 1,  /**< DIO1_AD1 pin */
            DIO2_AD2   = 2,  /**< DIO2_AD2 pin */
            DIO3_AD3   = 3,  /**< DIO3_AD3 pin */
            DIO4       = 4,  /**< DIO4 pin */
            DIO5       = 5,  /**< DIO5 pin */
            DIO6       = 6,  /**< DIO6 pin */
            DIO7       = 7,  /**< DIO7 pin */
            DIO8       = 8,  /**< DIO8 pin */
            DIO9       = 9,  /**< DIO9 pin */
            DIO10_PWM0 = 10, /**< DIO10_PWM0 pin */
            DIO11_PWM1 = 11, /**< DIO11_PWM1 pin */
            DIO12      = 12  /**< DIO12 pin */
        };

        /** Class constructor
         * @param tx the TX pin of the UART that will interface the XBee module
         * @param rx the RX pin of the UART that will interface the XBee module
         * @param reset the pin to which the XBee's reset line is attached to, use NC if not available
         * @param rts the RTS pin for the UART that will interface the XBee module, use NC if not available
         * @param cts the CTS pin for the UART that will interface the XBee module, use NC if not available
         * @param baud the baudrate for the UART that will interface the XBee module. Note that the module has to be already configured
         * to this baud rate (ATBD parameter). By default it is configured to 9600 bps
         */
        XBeeDM(PinName tx, PinName rx, PinName reset = NC, PinName rts = NC, PinName cts = NC, int baud = 9600);

        /** Class destructor */
        virtual ~XBeeDM();

        /** init-  initializes object
         * This function must be called just after creating the object so it initializes internal data.
         * @returns
         *         Success if the module has been properly initialized and is ready to process data.
         *         Failure otherwise.
         */
        RadioStatus init();

        /** set_channel - sets the channel number
         *
         *  @param channel the channel in which the radio operates. Range is 0x0B - 0x1A for XBee and 0x0C - 0x17 for XBee-PRO.
         *  The Center Frequency = 2.405 + (CH - 11) * 5 MHz
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus set_channel(uint8_t channel);

        /** get_channel - gets the channel number
         *
         *  @param channel pointer where the channel value will be stored.
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus get_channel(uint8_t * const channel);

        /** set_network_id - sets the Network ID.
         *
         *  @param network_id the Network ID value that will be set on the radio
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
         RadioStatus set_network_id(uint16_t network_id);

        /** get_network_id - gets the Network ID, as it was set by @ref set_network_id().
         *
         *  @param network_id pointer where the Network ID will be stored
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus get_network_id(uint16_t * const network_id);

        /** register_node_discovery_cb - registers the callback function that will be called
         * when the responses to the node discovery command arrive
         *
         *  @param function function pointer with the callback function
         */
        void register_node_discovery_cb(node_discovery_dm_cb_t function);

        /** unregister_node_discovery_cb - removes the node discovery callback */
        void unregister_node_discovery_cb();

        /** register_receive_cb - registers the callback function that will be called
         * when a data packet is received
         *
         *  @param function function pointer with the callback function
         */
        void register_receive_cb(receive_dm_cb_t function);

        /** unregister_receive_cb - removes the rx packet callback */
        void unregister_receive_cb();

        /** register_io_sample_cb - registers the callback function that will be called
         * when a IO Sample Data packet is received
         *
         *  @param function function pointer with the callback function
         */
        void register_io_sample_cb(io_data_cb_dm_t function);

        /** unregister_io_sample_cb - removes the IO Sample Data reception callback */
        void unregister_io_sample_cb();

        /*********************** send_data member methods ************************/
        /** send_data - sends data to a remote device
         *
         *  @param remote remote device
         *  @param data pointer to the data that will be sent
         *  @param len number of bytes that will be transmitted
         *  @param syncr if true, method waits for the packet answer with the result of the operation
         *  @returns the result of the data transfer
         *     TxStatusSuccess if the operation was successful,
         *     the error code otherwise
         */
        virtual TxStatus send_data(const RemoteXBee& remote, const uint8_t *const data, uint16_t len, bool syncr = true);

        /** send_data - sends data to a remote device. This method uses
         *                   the explicit addressing frame, allowing to use source and
         *                   destination end points and cluster and profile IDs
         *
         *  @param remote remote device
         *  @param source_ep source end point
         *  @param dest_ep destination end point
         *  @param cluster_id cluster ID
         *  @param profile_id profile ID
         *  @param data pointer to the data that will be sent
         *  @param len number of bytes that will be transmitted
         *  @param syncr if true, method waits for the packet answer with the result of the operation
         *  @returns the result of the data transfer
         *     TxStatusSuccess if the operation was successful,
         *     the error code otherwise
         */
        TxStatus send_data(const RemoteXBee& remote, uint8_t source_ep,
                                uint8_t dest_ep, uint16_t cluster_id, uint16_t profile_id,
                                const uint8_t *const data, uint16_t len, bool syncr = true);

        /** send_data_to_coordinator - sends data to the ZigBee coordinator
         *
         *  @param data pointer to the data that will be sent
         *  @param len number of bytes that will be transmitted
         *  @param syncr if true, method waits for the packet answer with the result of the operation
         *  @returns the result of the data transfer
         *     TxStatusSuccess if the operation was successful,
         *     the error code otherwise
         */
        TxStatus send_data_to_coordinator(const uint8_t *const data, uint16_t len, bool syncr = true);

        /** get_remote_node_by_id - searches for a device in the network with the specified Node Identifier.
         *
         *  @param node_id node id of the device we are looking for
         *  @returns a RemoteXBeeDM with the 16-bit and 64-bit address of the remote device whose node id matches with the parameter.
         *  If node is not found, the returned object will have invalid addresses (RemoteXBeeDM::is_valid() will return false).
         */
        RemoteXBeeDM get_remote_node_by_id(const char * const node_id);

        /* Allow using XBee::set_param() methods for local radio from this class */
        using XBee::set_param;

        /** set_param - sets a parameter in a remote radio by sending an AT command and waiting for the response.
         *
         *  @param remote remote device
         *  @param param parameter to be set.
         *  @param data the parameter value (4 bytes) to be set.
         *  @returns the command response status.
         */
        virtual AtCmdFrame::AtCmdResp set_param(const RemoteXBee& remote, const char * const param, uint32_t data);

        /** set_param - sets a parameter in a remote radio by sending an AT command and waiting for the response.
         *
         *  @param remote remote device
         *  @param param parameter to be set.
         *  @param data the parameter value byte array (len bytes) to be set.
         *  @param len number of bytes of the parameter value.
         *  @returns the command response status.
         */
        virtual AtCmdFrame::AtCmdResp set_param(const RemoteXBee& remote, const char * const param, const uint8_t * data = NULL, uint16_t len = 0);

        /* Allow using XBee::get_param() methods for local radio from this class */
        using XBee::get_param;

        /** get_param - gets a parameter from a remote radio by sending an AT command and waiting for the response.
         *
         *  @param remote remote device
         *  @param param parameter to be get.
         *  @param data pointer where the param value (4 bytes) will be stored.
         *  @returns the command response status.
         */
        virtual AtCmdFrame::AtCmdResp get_param(const RemoteXBee& remote, const char * const param, uint32_t * const data);

        /** get_param - gets a parameter from a remote radio by sending an AT command and waiting for the response.
         *
         *  @param remote remote device
         *  @param param parameter to be get.
         *  @param data pointer where the param value (n bytes) will be stored.
         *  @param len pointer where the number of bytes of the param value will be stored.
         *  @returns the command response status.
         */
        virtual AtCmdFrame::AtCmdResp get_param(const RemoteXBee& remote, const char * const param, uint8_t * const data, uint16_t * const len);

        /************************* IO member methods **************************/
        /** set_pin_config - configures a radio IO line
         *
         *  @param remote remote device
         *  @param line IO line being configured
         *  @param mode configuration mode for the selected line
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus set_pin_config(const RemoteXBee& remote, IoLine line, IoMode mode);

        /** get_pin_config - gets the configuration of a radio IO line
         *
         *  @param remote remote device
         *  @param line IO line being read to get its configuration
         *  @param mode pointer where the configuration will be stored
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus get_pin_config(const RemoteXBee& remote, IoLine line, IoMode * const mode);

        /** set_dio - sets to low/high a DIO line
         *
         *  @param remote remote device
         *  @param line DIO line being set
         *  @param val value that will be set in the DIO line
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus set_dio(const RemoteXBee& remote, IoLine line, DioVal val);

        /** get_dio - read the value of a DIO configured as digital input
         *
         *  @param remote remote device
         *  @param line DIO line being read
         *  @param val pointer where the DIO value read will be stored
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus get_dio(const RemoteXBee& remote, IoLine line, DioVal * const val);

        /** get_adc - read the value of the espcified ADC line
         *
         *  @param remote remote device
         *  @param line ADC line being read
         *  @param val pointer where the value read from hte ADC will be stored
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus get_adc(const RemoteXBee& remote, IoLine line, uint16_t * const val);

        /** get_iosample - retrieves an @ref IOSampleDM from a remote node. This object can be used to get the remote node's ADC and DIO values.
         *
         *  @param remote remote device
         *  @returns IOSampleDM object with the remote node's DIO and ADC values.
         */
        IOSampleDM get_iosample(const RemoteXBee& remote);

        /** set_pwm - sets the duty cycle of a PWM line
         *
         *  @param remote remote device
         *  @param line PWM line being set
         *  @param duty_cycle duty cycle that will be set in the PWM line
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus set_pwm(const RemoteXBee& remote, IoLine line, float duty_cycle);

        /** set_pin_pull_up - enables or disables the internal pull-up resistor of a line
         *
         *  @param remote remote device
         *  @param line line being configured for pull-up
         *  @param enable whether to enable the internal pull-up resistor.
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus set_pin_pull_up(const RemoteXBee& remote, IoLine line, bool enable);

        /** enable_dio_change_detection - enables or disables the notification when a change is detected in a digital input line.
         * In other words, it will force an IO Sample transmission when the DIO state changes. Only for DIO0 to DIO11.
         *
         *  @param remote remote device
         *  @param line line being configured for pull-up
         *  @param enable whether to enable the internal pull-up resistor.
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus enable_dio_change_detection(const RemoteXBee& remote, IoLine line, bool enable);

        /** config_poll_destination - configures to which node poll messages will be sent once module awakes if configured as
         *                            an Indirect Msg Poller (CE==0x4).
         * Consult the module's reference manual for more information.
         *
         *  @param destination remote device where the poll messages will be sent
         *  @returns the result of the data transfer
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus config_poll_destination(const RemoteXBee& destination);

    protected:

        /** Frame handler used for the node discovery. Registered when a callback function
         * is registered */
        FH_NodeDiscoveryDM  *_nd_handler;

        /** Frame handler used for the rx packets. Automatically registered when a callback
         *  function is registered */
        FH_RxPacketDM  *_rx_pkt_handler;

        /** Frame handler used for the IO Data Sample packets. Automatically registered when a callback
         *  function is registered */
        FH_IoDataSampeDM  *_io_data_handler;

        /** Method called directly by the library when a modem status frame is received to
         * update the internal status variables */
        virtual void radio_status_update(AtCmdFrame::ModemStatus modem_status);

        /* Allow using XBee::send_data() methods from this class */
        using XBee::send_data;

        /** get_node_discovery_timeout - gets the node discovery timeout
          *
          *  @param timeout_ms pointer where the node discovery timeout value will be stored
          *  @param wait_for_complete_timeout pointer where the function will store if the operator
          *                                   has to wait for the complete nd timeout after issuing 
          *                                   a directed nd request
          *  @returns
          *     Success if the operation was successful,
          *     Failure otherwise
          */
        virtual RadioStatus get_node_discovery_timeout(uint16_t * const timeout_ms);
        virtual RadioStatus get_node_discovery_timeout(uint16_t * const timeout_ms, bool * const wait_for_complete_timeout);
};

}   /* namespace XBeeLib */

#endif /* __XBEE_DM_H_ */
