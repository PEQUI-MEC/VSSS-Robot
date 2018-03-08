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

#ifndef _IO_IOSAMPLE802_H_
#define _IO_IOSAMPLE802_H_

#define MAX_IO_SAMPLE_802_LEN   (2 + 2 * 6)

namespace XBeeLib {

/** Class to handle the incoming IO Data Samples in 802.15.4 modules */
class IOSample802 {
    public:
        /** Class constructor
         *  @param raw_data The IO Sample data, as returned by an "IS" command response or in the Io16Bit (0x83) or Io64Bit (0x82) frames
         *  @param size size (in bytes) of raw_data
         */
        IOSample802(const uint8_t* const raw_data = NULL, size_t size = 0);

        /** Class destructor */
        ~IOSample802();

        /** get_dio - read the value of a DIO configured as digital input
         *
         *  @param line DIO line being read
         *  @param val pointer where the DIO value read will be stored
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus get_dio(XBee802::IoLine line, DioVal* const dio_value) const;

        /** get_adc - read the value of the espcified ADC line
         *
         *  @param line ADC line being read
         *  @param val pointer where the value read from the ADC will be stored
         *  @returns
         *     Success if the operation was successful,
         *     Failure otherwise
         */
        RadioStatus get_adc(XBee802::IoLine line, uint16_t* const val) const;

        /** is_valid - checks if the IOSample802 object has at least one DIO or ADC sample.
         *  @returns true if valid, false otherwise
         */
        inline bool is_valid()
        {
            return _channel_mask != 0;
        }

    protected:
        uint16_t _channel_mask;
        uint8_t _sampled_data[MAX_IO_SAMPLE_802_LEN];
        uint8_t _sampled_data_size;

        inline bool dio_channels_present(void) const;
        inline uint8_t get_dio_channels(void) const;
};

}   /* namespace XBeeLib */

#endif /* _IO_IOSAMPLE802_H_ */
