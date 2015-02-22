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
    MSG_RESERVED,			//not used, reserved for coherency checks (0 is default memory value)
    MSG_VERSION,			//sended by stm32, publish the runtime version
    MSG_STATUS,				//sended by stm32, see control.cxx
    MSG_LOG,				//sended by stm32, log to be appended by x86 log system
    MSG_EVENT,				//sended by both, simple message with no payload, see EventId below
    MSG_FAULT,				//sended by stm32, publish an hardware fault, see fault.cxx
    MSG_OPP_LIST,			//sended by stm32, list of detected opponents in hokuyo scans, see detection.cxx
    MSG_HOKUYO_SCAN,		//sended by stm32, hokuyo raw scan, for debug purposes, see hokuyo.cxx
    MSG_X86_CMD,			//sended by x86, publish a consolidated list of commands for stm32 actuators
    MSG_CONFIGURATION,		//sended by x86, send configuration parameters to the stm32
    MSG_GYRO_CMD,			//sended by x86, gyrometer related commands (calibration, config, set pose)
    MSG_NB
} DiscoveryMsgType;

typedef enum
{
    EVT_RESERVED,			//not used, reserved for coherency checks (0 is default memory value)
    EVT_INFORM_READY,       //sended by stm32, to inform it's ready to operate
    EVT_INFORM_START_MATCH,	//sended by stm32, match beginning event, see end.cxx
    EVT_INFORM_END_MATCH,	//sended by stm32, match end event, see end.cxx
    EVT_REBOOT,				//sended by x86, request stm32 soft reboot
    EVT_ENABLE_HEARTBEAT,	//sended by x86, to enable the x86 life check
    EVT_LIST_TASKS,			//sended by x86, for debug purpose, display tasks data
    EVT_X86_INIT_DONE,		//sended by x86, to inform strat is deployed and ready
    EVT_SCAN_DYNAMIXELS, 	//sended by x86, debug only
    EVT_REQUEST_END_MATCH,  //sended by x86, request the end of match
    EVT_X86_READY_FOR_MATCH,//sended by x86, when the strat is ready for the match
    EVT_NB
} EventId;

#ifdef __cplusplus
}
#endif

#endif /* DISCOVERYIPCTYPES_HPP_ */
