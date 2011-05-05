
/*********************************************************************
 *  This file contains canFestival callbacks. As we wish to have them
 *  in C++, we need this wrapper
 **********************************************************************/

#ifndef CAN_FESTIVAL_ARD_MASTER_WRAPPER_HPP_
#define CAN_FESTIVAL_ARD_MASTER_WRAPPER_HPP_

#include <rtt/Port.hpp>
#include "hml/can/dictionnary/CanARD.h"
#include "hml/can/ard_can_types.hpp"

extern string                          canFestivalWrapperName;
extern RTT::OutputPort<e_nodeState>    canFestival_outNMTState;
extern RTT::OutputPort<nodeID_t>            canFestival_outBootUpReceived;

bool initWrapper();
void startTimerLoopCallback     (CO_Data* d, UNS32 id);
void exitTimerLoopCallback      (CO_Data* d, UNS32 id);
void initialisationCallback     (CO_Data* d);
void preOperationalCallback     (CO_Data* d);
void operationalCallback        (CO_Data* d);
void stoppedCallback            (CO_Data* d);
void heartbeatErrorCallback     (CO_Data* d, UNS8 heartbeatID);
void postSyncCallback           (CO_Data* d);
void postTPDOCallback           (CO_Data* d);
void postEmcy                   (CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg);
void postSlaveBootup            (CO_Data* d, UNS8 nodeId);

#endif /* CAN_FESTIVAL_ARD_MASTER_WRAPPER_HPP_ */
