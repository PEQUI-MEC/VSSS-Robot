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

#if !defined(__FH_AT_CMD_RESP_H_)
#define __FH_AT_CMD_RESP_H_

#include "FrameHandler.h"
#include "XBee/XBee.h"

namespace XBeeLib {
typedef void (*at_cmd_resp_cb_t)(const uint8_t * data, uint16_t len);

class FH_AtCmdResp : public FrameHandler
{
    private:
        /** Callback function, invoked (if registered) when an at command response packet is received */
        at_cmd_resp_cb_t at_cmd_resp_cb;

    public:

        /** Class constructor */
        FH_AtCmdResp();

        /** Class constructor */
        FH_AtCmdResp(ApiFrame::ApiFrameType type);


        /** Class destructor */
        virtual ~FH_AtCmdResp();

        virtual void process_frame_data(const ApiFrame *const frame);

        virtual void register_at_cmd_resp_cb(at_cmd_resp_cb_t function);

        virtual void unregister_at_cmd_resp_cb();
};

/**
 * @defgroup callback_types "Callback types declaration"
 * @{
 */
/** Node Discovery Response callback type declaration for ZigBee
  * @param remote discovered remote node.
  * @param node_id Node Identifier (NI parameter) of remote.
  */
typedef void (*node_discovery_zb_cb_t)(const RemoteXBeeZB& remote, char const * const node_id);
/**
 * @}
 */

class FH_NodeDiscoveryZB : public FH_AtCmdResp
{
    private:
        /** Callback function, invoked (if registered) when an at command response packet is received */
        node_discovery_zb_cb_t node_discovery_cb;

    public:

        /** Class constructor */
        FH_NodeDiscoveryZB();

        /** Class destructor */
        virtual ~FH_NodeDiscoveryZB();

        virtual void process_frame_data(const ApiFrame *const frame);

        virtual void register_node_discovery_cb(node_discovery_zb_cb_t function);

        virtual void unregister_node_discovery_cb();
};

/**
 * @defgroup callback_types "Callback types declaration"
 * @{
 */
/** Node Discovery Response callback type declaration for ZigBee
  * @param remote discovered remote node.
  * @param node_id Node Identifier (NI parameter) of remote.
  */
typedef void (*node_discovery_dm_cb_t)(const RemoteXBeeDM& remote, char const * const node_id);
/**
 * @}
 */

class FH_NodeDiscoveryDM : public FH_AtCmdResp
{
    private:
        /** Callback function, invoked (if registered) when an at command response packet is received */
        node_discovery_dm_cb_t node_discovery_cb;

    public:

        /** Class constructor */
        FH_NodeDiscoveryDM();

        /** Class destructor */
        virtual ~FH_NodeDiscoveryDM();

        virtual void process_frame_data(const ApiFrame *const frame);

        virtual void register_node_discovery_cb(node_discovery_dm_cb_t function);

        virtual void unregister_node_discovery_cb();
};

/**
 * @defgroup callback_types "Callback types declaration"
 * @{
 */
/** Node Discovery Response callback type declaration for 802.15.4
  * @param remote discovered remote node.
  * @param node_id Node Identifier (NI parameter) of remote.
  */
typedef void (*node_discovery_802_cb_t)(const RemoteXBee802& remote, char const * const node_id);
/**
 * @}
 */

class FH_NodeDiscovery802 : public FH_AtCmdResp
{
    private:
        /** Callback function, invoked (if registered) when an at command response packet is received */
        node_discovery_802_cb_t node_discovery_cb;

    public:

        /** Class constructor */
        FH_NodeDiscovery802();

        /** Class destructor */
        virtual ~FH_NodeDiscovery802();

        virtual void process_frame_data(const ApiFrame *const frame);

        virtual void register_node_discovery_cb(node_discovery_802_cb_t function);

        virtual void unregister_node_discovery_cb();
};

} /* namespace XBeeLib */

#endif /* __FH_AT_CMD_RESP_H_ */
