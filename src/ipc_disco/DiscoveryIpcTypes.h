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
    MSG_RESERVED = 0,		//not used, reserved for coherency checks (0 is default memory value)
    MSG_VERSION = 1,		//sended by stm32, publish the runtime version
    MSG_STATUS = 2,			//sended by stm32, see control.cxx
    MSG_LOG = 3,			//sended by stm32, log to be appended by x86 log system
    MSG_FAULT = 4,			//sended by stm32, publish an hardware fault, see fault.cxx
    MSG_EVENT = 5,			//simple message with no payload, see EventId below
    MSG_OPP_LIST = 6,		//sended by stm32, list of detected opponents in hokuyo scans, see detection.cxx
    MSG_HOKUYO_SCAN = 7,	//sended by stm32, hokuyo raw scan, for debug purposes, see hokuyo.cxx
} DiscoveryMsgType;

typedef enum
{
    EVT_RESERVED = 0,			//not used, reserved for coherency checks (0 is default memory value)
    EVT_START_MATCH = 1,		//sended by stm32, match beginning event, see end.cxx
    EVT_REQUEST_VERSION = 2,	//sended by x86, request stm32 runtime version and implicitly let usb communication begin
    EVT_REBOOT = 3,				//sended by x86, request stm32 soft reboot
    EVT_LIST_TASKS = 4,			//sended by x86, for debug purpose, display tasks data
    EVT_END_MATCH = 5,			//sended by stm32, match end event, see end.cxx
} EventId;

#ifdef __cplusplus
}
#endif

#endif /* DISCOVERYIPCTYPES_HPP_ */
