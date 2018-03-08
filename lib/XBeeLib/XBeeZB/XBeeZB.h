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

#if !defined(__XBEE_ZB_H_)
#define __XBEE_ZB_H_

#include "XBee/XBee.h"
#include "FrameHandlers/FH_AtCmdResp.h"
#include "FrameHandlers/FH_RxPacketZB.h"
#include "FrameHandlers/FH_IoDataSampleZB.h"
#include "RemoteXBee/RemoteXBee.h"

namespace XBeeLib {

/** Class for XBee ZigBee modules, derived from XBee */
class XBeeZB : public XBee
{
    public:

        /**
         * IoLine for XBeeZB Modules
         */
        enum IoLine {
            DIO0_AD0 = 0,  /**< DIO0_AD0 pin */
            DIO1_AD1 = 1,  /**< DIO1_AD1 pin */
            DIO2_AD2 = 2,  /**< DIO2_AD2 pin */
            DIO3_AD3 = 3,  /**< DIO3_AD3 pin */
            DIO4     = 4,  /**< DIO4 pin */
            DIO5     = 5,  /**< DIO5 pin */
            DIO6     = 6,  /**< DIO6 pin */
            DIO7     = 7,  /**< DIO7 pin */
            DIO10    = 10, /**< DIO10 pin */
            DIO11    = 11, /**< DIO11 pin */
            DIO12    = 12, /**< DIO12 pin */
            SUPPLY_VOLTAGE = 7, /**< SUPPLY_VOLTAGE is not a real pin */
        };

        enum AssocStatus {
            ErrorReading    = -1,       /**< Error occurred when reading parameter. */
            Joined          = 0x00,     /**< Successfully formed or joined a network. (Coordinators form a network, routers and end devices join a network.) */
            NoPANs          = 0x21,     /**< Scan found no PANs */
            NoValidPAN      = 0x22,     /**< Scan found no valid PANs based on current SC and ID settings */
            JoinNotAllowed  = 0x23,     /**< Valid Coordinator or Routers found, but they are not allowing joining (NJ expired). */
            NoBeacons       = 0x24,     /**< No joinable beacons were found. */
            Unexpected      = 0x25,     /**< Unexpected state, node should not be attempting to join at this time. */
            JoinFailed      = 0x27,     /**< Node Joining attempt failed (typically due to incompatible security settings). */
            CoordStartFail  = 0x2A,     /**< Coordinator start attempt failed */
            CheckingCoord   = 0x2B,     /**< Checking for an existing coordinator. */
            LeaveFail       = 0x2C,     /**< Attempt to leave the network failed. */
            JoinNoResponse  = 0xAB,     /**< Attempted to join a device that did not respond. */
            SecKeyUnsec     = 0xAC,     /**< Secure join error - network security key received unsecured. */
            SecKeyNotRec    = 0xAD,     /**< Secure join error - network security key not received. */
            SecBadKey       = 0xAF,     /**< Secure join error - joining device does not have the right preconfigured link key. */
            Scanning        = 0xFF      /**< Scanning for a ZigBee network (routers and end devices). */
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
        XBeeZB(PinName tx, PinName rx, PinName reset = NC, PinName rts = NC, PinName cts = NC, int baud = 9600);

        /** Class destructor */
        virtual ~XBeeZB();

        /** init-  initializes object
         * This function must be called just after creating the object so it initializes internal data.
         * @returns
         *         Success if the module has been properly initialized and is ready to process data.
         *         Failure otherwise.
         */
        RadioStatus init();

        /** set_panid - sets the 64bit extended PAN ID.
         *
         *  @note on ZigBee devices, if set to 0, the coordinator will select a random PAN ID
         *           and the routers will join any extended PAN ID
         *  @param panid the PAN ID value that will be set on the radio
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus set_panid(uint64_t panid);

        /** get_configured_panid - gets the configured PAN ID, as it was set by @ref set_panid().
         *
         *  @note on ZigBee devices, if set to 0, the coordinator will select a random PAN ID
         *           and the routers will join any extended PAN ID
         *  @param panid pointer where the configured PAN ID will be stored
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus get_configured_panid(uint64_t * const panid);

        /** get_operating_panid - gets the operating 64bit extended PAN ID the module is running on. This is useful to determine the PAN ID when the ID parameter (@ref set_panid) is set to 0x00.
         *
         *  @param panid pointer where the operating PAN ID will be stored
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus get_operating_panid(uint64_t * const panid);

        /** set_panid - sets the 64bit extended PAN ID.
         *
         *  @note on ZigBee devices, if set to 0, the coordinator will select a random PAN ID
         *           and the routers will join any extended PAN ID
         *  @param remote remote device
         *  @param panid the PAN ID value that will be set on the radio
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus set_panid(const RemoteXBee& remote, uint64_t panid);

        /** get_configured_panid - gets the configured PAN ID in a remote node, as it was set by @ref set_panid()
         *
         *  @note on ZigBee devices, if set to 0, the coordinator will select a random PAN ID
         *           and the routers will join any extended PAN ID
         *
         *  @param remote remote device
         *  @param panid pointer where the configured PAN ID will be stored
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus get_configured_panid(const RemoteXBee& remote, uint64_t * const panid);

        /** get_operating_panid - gets the operating 64bit extended PAN ID in which a remote node is running on. This is useful to determine the PAN ID when the ID parameter (@ref set_panid) is set to 0x00.
         *
         *  @param remote remote device
         *  @param panid pointer where the operating PAN ID will be stored
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus get_operating_panid(const RemoteXBee& remote, uint64_t * const panid);

        /** set_channel_mask - sets the channel mask in which the module will scan for the PAN ID (if it is a router or end-device) or start the network (if it is a coordinator).
         * It should be set to the minimum available set of channels of all nodes in the network. Refer to "SC" parameter in the product manual for more information.
         *
         *  @param chmask bit field list of channels to scan (router/end-devices) or to choose when starting a network (coordinator). Bit 0 is for channel 0x0B and bit 15 for channel 0x1A.
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus set_channel_mask(uint16_t const chmask);

        /** get_channel_mask - gets the channel mask in which the module will scan for the PAN ID (if it is a router or end-device) or start the network (if it is a coordinator).
         *
         *  @param chmask pointer to where the configured channel mask will be stored
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus get_channel_mask(uint16_t * const chmask);

        /** get_network_address - gets the 16bit network address of the device
         *
         *  @param addr pointer where the device 16bit network address will be stored
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus get_network_address(uint16_t * const addr);

        /** check_for_coordinator_at_start - (Routers only) If enabled, a router will verify the coordinator is on its operating channel when joining or coming up from a power cycle.
         * If a coordinator is not detected, the router will leave its current channel and attempt to join a new PAN. If JV=0, the router will continue operating on its current channel even if a coordinator is not detected.
         *
         *  @param enable whether to enable this feature or not
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus check_for_coordinator_at_start(bool enable);

        /** set_network_security_key - (Coordinator only) Set the 128-bit AES network encryption key. If set to 0 (default), the module will select a random network key.
         *  It is not recommended to set the key programmatically, because it could be read through the raw serial port bits.
         *  @param key pointer to the 128-bit AES key
         *  @param length size of the buffer pointed by 'key'
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus set_network_security_key(const uint8_t * const key, const uint16_t length);

#define XBEE_ZB_ENC_OPT_SEND_KEY_ON_JOIN    0x01
#define XBEE_ZB_ENC_OPT_USE_TRUST_CENTER    0x02
        /** set_encryption_options - Configure options for encryption. Unused option bits should be set to 0. Options include:
         *  - XBEE_ZB_ENC_OPT_SEND_KEY_ON_JOIN - Send the security key unsecured over-the-air during joins
         *  - XBEE_ZB_ENC_OPT_USE_TRUST_CENTER - Use trust center (coordinator only)
         *  @param options bit mask with the encryption options
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus set_encryption_options(const uint8_t options);

        /** register_node_discovery_cb - registers the callback function that will be called
         * when the responses to the node discovery command arrive
         *
         *  @param function function pointer with the callback function
         */
        void register_node_discovery_cb(node_discovery_zb_cb_t function);

        /** unregister_node_discovery_cb - removes the node discovery callback */
        void unregister_node_discovery_cb();

        /** register_receive_cb - registers the callback function that will be called
         * when a data packet is received
         *
         *  @param function function pointer with the callback function
         */
        void register_receive_cb(receive_zb_cb_t function);

        /** unregister_receive_cb - removes the rx packet callback */
        void unregister_receive_cb();

        /** register_io_sample_cb - registers the callback function that will be called
         * when a IO Sample Data packet is received
         *
         *  @param function function pointer with the callback function
         */
        void register_io_sample_cb(io_data_cb_zb_t function);

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

        /** get_assoc_status - returns current network association status. This wraps AI parameter, for more information refer to moudle's Reference Manual.
         *
         *  @returns an AssocStatus with current network association status.
         */
        AssocStatus get_assoc_status(void);

        /** is_joined - checks if the device is joined to ZigBee network
         *  @returns true if joined, false otherwise
         */
        bool is_joined();

        /** get_remote_node_by_id - searches for a device in the network with the specified Node Identifier.
         *
         *  @param node_id node id of the device we are looking for
         *  @returns a RemoteXBeeZB with the 16-bit and 64-bit address of the remote device whose node id matches with the parameter.
         *  If node is not found, the returned object will have invalid addresses (RemoteXBeeZB::is_valid() will return false).
         */
        RemoteXBeeZB get_remote_node_by_id(const char * const node_id);

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

        /** get_iosample - retrieves an @ref IOSampleZB from a remote node. This object can be used to get the remote node's ADC and DIO values.
         *
         *  @param remote remote device
         *  @returns IOSampleZB object with the remote node's DIO and ADC values.
         */
        IOSampleZB get_iosample(const RemoteXBee& remote);

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

    protected:

        /** Frame handler used for the node discovery. Registered when a callback function
         * is registered */
        FH_NodeDiscoveryZB  *_nd_handler;

        /** Frame handler used for the rx packets. Automatically registered when a callback
         *  function is registered */
        FH_RxPacketZB  *_rx_pkt_handler;

        /** Frame handler used for the IO Data Sample packets. Automatically registered when a callback
         *  function is registered */
        FH_IoDataSampeZB  *_io_data_handler;

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

#endif /* __XBEE_ZB_H_ */
