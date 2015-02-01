/*
 * DiscoveryIpcTypes.hpp
 *
 *  Created on: Jan 10, 2015
 *      Author: willy
 */

#ifndef DISCOVERYIPCTYPES_HPP_
#define DISCOVERYIPCTYPES_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    MSG_RESERVED = 0,
    MSG_VERSION = 1,
    MSG_STATUS = 2,
    MSG_LOG = 3,
    MSG_FAULT = 4,
    MSG_EVENT = 5,
    MSG_OPP_LIST = 6
} DiscoveryMsgType;

typedef enum
{
    EVT_RESERVED = 0,
    EVT_START_MATCH = 1,
    EVT_REQUEST_VERSION = 2,
    EVT_REBOOT = 3,
    EVT_LIST_TASKS = 4,
} EventId;

#ifdef __cplusplus
}
#endif

#endif /* DISCOVERYIPCTYPES_HPP_ */
