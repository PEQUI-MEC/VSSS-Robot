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

#if !defined(__XBEE_REMOTE_H_)
#define __XBEE_REMOTE_H_

#include "XBee/Addresses.h"

namespace XBeeLib {

/** Class for Remote XBee modules. Not to be used directly. */
class RemoteXBee
{
    public:

         /** Default Class constructor for a remote device (connected wirelessly). No address set.
         */
        RemoteXBee();

        /** Class constructor for a remote device (connected wirelessly) using 64bit addressing
         * @param remote64 the 64-bit address (ATSH and ATSL parameters) of the remote XBee module
         */
        RemoteXBee(uint64_t remote64);

        /** Class destructor */
        ~RemoteXBee();

        /** get_addr64 - returns the 64bit address of the remote device
         *
         *  @returns the 64bit address of the remote device
         */
        uint64_t get_addr64() const;

        /** get_addr16 - returns the 16bit address of the remote device
         *
         *  @returns the 16bit address of the remote device
         */
        uint16_t get_addr16() const;

        /** operator == overload so the object can be compared to equal */
        inline bool operator == (const RemoteXBee &b) const
        {
            return ((b._dev_addr16 == _dev_addr16) &&
                    (b._dev_addr64 == _dev_addr64));
        }

        /** operator != overload so the object can be compared to not equal */
        inline bool operator != (const RemoteXBee &b) const
        {
            return !(this == &b);
        }

        /** is_valid_addr16b - checks if the RemoteXBee object has a valid 16b address
         *  @returns true if valid, false otherwise
         */
        inline bool is_valid_addr16b() const
        {
            return (_dev_addr16 != ADDR16_UNKNOWN);
        }

        /** is_valid_addr64b - checks if the RemoteXBee object has a valid 64b address
         *  @returns true if valid, false otherwise
         */
        inline bool is_valid_addr64b() const
        {
            return !(_dev_addr64 == ADDR64_UNASSIGNED);
        }


    protected:
        /** Remote Device 64 bit address */
        uint64_t      _dev_addr64;

        /** Remote Device 16 bit address */
        uint16_t    _dev_addr16;
};

class FH_NodeDiscovery802;
/** Class for 802.15.4 Remote XBee modules */
class RemoteXBee802 : public RemoteXBee
{
    public:

         /** Default Class constructor for a 802.15.4 remote device (connected wirelessly). No address set.
         */
        RemoteXBee802();

        /** Class constructor for a 802.15.4 remote device (connected wirelessly) using 64bit addressing
         * @param remote64 the 64-bit address (ATSH and ATSL parameters) of the remote XBee module
         */
        RemoteXBee802(uint64_t remote64);

        /** Class constructor for a 802.15.4 remote device (connected wirelessly) using 16bit addressing
         * @param remote16 the 16-bit address (ATMY parameter) of the remote XBee module
         */
        RemoteXBee802(uint16_t remote16);

        /** Class destructor */
        ~RemoteXBee802();

        inline bool is_valid(void)
        {
            return is_valid_addr64b() || is_valid_addr16b();
        }

    protected:

        friend FH_NodeDiscovery802;
        friend class XBee802;

        /** Class constructor for a 802.15.4 remote device (connected wirelessly) for which both the 64-bit and 16-bit addresses are known.
         * This constructor is only used by FH_NodeDiscovery802 class.
         * @param remote64 the 64-bit address (ATSH and ATSL parameters) of the remote XBee module
         * @param remote16 the 16-bit address (ATMY parameter) of the remote XBee module
         */
        RemoteXBee802(uint64_t remote64, uint16_t remote16);
};

/** Class for ZigBee Remote XBee modules */
class RemoteXBeeZB : public RemoteXBee
{
    public:

         /** Default Class constructor for a ZigBee remote device (connected wirelessly). No address set.
         */
        RemoteXBeeZB();

       /** Class constructor for a ZigBee remote device (connected wirelessly) using 64bit addressing
         * @param remote64 the 64-bit address (ATSH and ATSL parameters) of the remote XBee module
         */
        RemoteXBeeZB(uint64_t remote64);

        /** Class constructor for a ZigBee remote device (connected wirelessly) using 64bit and 16b addressing
         * @param remote64 the 64-bit address (ATSH and ATSL parameters) of the remote XBee module
         * @param remote16 the 16-bit address (ATMY parameter) of the remote XBee module
         */
        RemoteXBeeZB(uint64_t remote64, uint16_t remote16);

        /** Class destructor */
        ~RemoteXBeeZB();

        inline bool is_valid(void)
        {
            return is_valid_addr64b();
        }
};

/** Class for DigiMesh Remote XBee modules */
class RemoteXBeeDM : public RemoteXBee
{
    public:

         /** Default Class constructor for a DigiMesh remote device (connected wirelessly). No address set.
         */
        RemoteXBeeDM();

       /** Class constructor for a DigiMesh remote device (connected wirelessly) using 64bit addressing
         * @param remote64 the 64-bit address (ATSH and ATSL parameters) of the remote XBee module
         */
        RemoteXBeeDM(uint64_t remote64);

        /** Class destructor */
        ~RemoteXBeeDM();

        inline bool is_valid(void)
        {
            return is_valid_addr64b();
        }
};

}   /* namespace XBeeLib */

#endif /* defined(__XBEE_REMOTE_H_) */
