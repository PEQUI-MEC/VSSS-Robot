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

#if !defined(__API_FRAME_H_)
#define __API_FRAME_H_

#include "XBee/Addresses.h"

/** Class for XBee API frames */
class ApiFrame
{
    /** Static variable that contains the last frame ID value assigned */
    static uint8_t      last_frame_id;

        public:
        /** List of API frames. Note that not all frames are supported by all radios */
        enum ApiFrameType {

            TxReq64Bit      = 0x00,     /**< TxReq64Bit: Only for 802.15.4 modules */
            TxReq16Bit      = 0x01,     /**< TxReq16Bit: Only for 802.15.4 modules */
            AtCmd           = 0x08,     /**< AtCmd */
            AtCmdQueuePV    = 0x09,     /**< AtCmdQueuePV */
            TxReqZBDM       = 0x10,     /**< TxReqZBDM: Only for ZigBee and DigiMesh modules */
            ExpAddrCmd      = 0x11,     /**< ExpAddrCmd: Only for ZigBee modules and DigiMesh */
            RemoteCmdReq    = 0x17,     /**< RemoteCmdReq */
            CreateSrcRoute  = 0x21,     /**< CreateSrcRoute */
            RxPacket64Bit   = 0x80,     /**< RxPacket64Bit: Only for 802.15.4 modules */
            RxPacket16Bit   = 0x81,     /**< RxPacket16Bit: Only for 802.15.4 modules */
            Io64Bit         = 0x82,     /**< Io64Bit: Only for 802.15.4 modules */
            Io16Bit         = 0x83,     /**< Io16Bit */
            AtCmdResp       = 0x88,     /**< AtCmdResp */
            TxStatus        = 0x89,     /**< TxStatus */
            AtModemStatus   = 0x8A,     /**< AtModemStatus */
            TxStatusZBDM    = 0x8B,     /**< TxStatusZBDM: Only for ZigBee and DigiMesh modules */
            RouteInfo       = 0x8D,     /**< RouteInfo: Only for DigiMesh modules */
            AggregateAddr   = 0x8E,     /**< AggregateAddr: Only for DigiMesh modules */
            RxPacketAO0     = 0x90,     /**< RxPacketAO0: Only for ZigBee and DigiMesh modules */
            RxPacketAO1     = 0x91,     /**< RxPacketAO1: Only for ZigBee and DigiMesh modules */
            IoSampleRxZBDM  = 0x92,     /**< IoSampleRxZBDM: Only for ZigBee and DigiMesh modules */
            SensorRxIndAO0  = 0x94,     /**< SensorRxIndAO0: Only for ZigBee modules */
            NodeIdentIndAO0 = 0x95,     /**< NodeIdentIndAO0: Only for ZigBee and DigiMesh modules */
            RemoteCmdResp   = 0x97,     /**< RemoteCmdResp */
            OtaFwUpStatus   = 0xA0,     /**< OtaFwUpStatus */
            RouteRecInd     = 0xA1,     /**< RouteRecInd */
            Many2OneRRInd   = 0xA3,     /**< Many2OneRRInd */
            Invalid         = ~0,       /**< Invalid */
        };

        /** Default constructor */
        ApiFrame();

        /** Constructor
         *
         * @param len length of the API frame (will allocate len bytes).
         */
        ApiFrame(uint16_t len);

        /** Constructor
         *
         * @param type frame type of this api frame.
         * @param data pointer to frame data payload.
         * @param len length of the payload.
         */
        ApiFrame(ApiFrameType type, const uint8_t *data, uint16_t len);

        /** Destructor */
        ~ApiFrame();

        ApiFrame(const ApiFrame& other); /* Intentionally not implemented */

        /** get_frame_type gets the type of the frame
         *
         * @returns the type of this frame.
         */
        ApiFrameType get_frame_type() const;

        /** dump dumps the information of this frame */
        void dump() const;

        /** dump_if dumps the information of the frame if the frame type matches
         *          with the parameter.
         *
         * @param type dump the frame info/data if the frame type matches with type.
         */
        void dump_if(ApiFrameType type);


        /** set_frame_type sets the type of the frame to type.
         *
         * @param type the type we want to set on the frame.
         */
        void set_frame_type(ApiFrameType type);

        /** get_data_len gets the length of the frame data payload.
         *
         * @returns the length of the data payload.
         */
        uint16_t get_data_len() const;

        /** set_data_len sets the length of the frame data payload.
         *
         * @param len the length of the data payload will be set on this frame.
         */
        void set_data_len(uint16_t len);

        /** get_data returns a pointer to the frame data payload.
         *
         * @returns a pointer to the frame data payload.
         */
        const uint8_t *get_data() const;

        /** get_data_at returns the byte at index offset.
         *
         * @param index offset of the byte we want to get.
         * @returns the byte at index offset.
         */
        uint8_t get_data_at(uint16_t index) const;

        /** set_data sets data byte at the specified index or offset.
         *
         * @param data byte that will be set at index position.
         * @param index offset of the byte we want to set.
         */
        void set_data(uint8_t data, uint16_t index);

        /** get_frame_id returns the frame id of this frame.
         *
         * @returns the frame id of this frame.
         */
        uint8_t get_frame_id() const;

        static uint8_t get_current_frame_id()
        {
            return last_frame_id;
        }

    protected:
        /** Type of this frame */
        ApiFrameType        _type;

        /** length of the payload, excluding the frame type */
        uint16_t            _data_frame_len;

        /** pointer to the frame data */
        uint8_t             *_data;

        /** True if the constructor allocates the data. Needed to delete it on the destructor */
        bool                _alloc_data;

        /** Frame ID of this frame */
        uint8_t             _frame_id;

        /** get_next_frame_id - returns the next frame ID secuentially, skipping the value 0
         *
         *  @returns the next frame ID that should be assigned to a frame
         */
        uint8_t get_next_frame_id();

        /** set_api_frame sets several members
         *
         * @param type frame type of this api frame.
         * @param data pointer to frame data payload.
         * @param len length of the payload.
         */
        void set_api_frame(ApiFrameType type, const uint8_t *data, uint16_t len);
};

#endif /* __API_FRAME_H_ */
