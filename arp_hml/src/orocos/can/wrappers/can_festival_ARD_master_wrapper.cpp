/*********************************************************************
 *  This file contains canFestival callbacks. As we wish to have them
 *  in C++, we need this wrapper
 **********************************************************************/

using namespace std;

#include "orocos/can/wrappers/can_festival_ARD_master_wrapper.hpp"

//#define DEBUG_ON


string canFestivalWrapperName;
RTT::OutputPort<e_nodeState> canFestival_outNMTState;
RTT::OutputPort<nodeID_t> canFestival_outBootUpReceived;
RTT::OutputPort<timespec> canFestival_outSyncSent;

bool initWrapper()
{
    canFestivalWrapperName = "ARD Master";
    canFestival_outNMTState.write(Unknown_state);
    return true;
}

void startTimerLoopCallback(CO_Data* d, UNS32 id)
{
    //it seems there is a bug into the canFestival, d is always null... please do not use it

#ifdef DEBUG_ON
    cerr << "CERR : [INFO] startTimerLoopCallback(" << d << " , " << id << " )" << endl;
#endif

    canFestival_outNMTState.write(Initialisation);
    setState(&CanARD_Data, Initialisation);
}

void exitTimerLoopCallback(CO_Data* d, UNS32 id)
{
#ifdef DEBUG_ON
    cerr << "CERR : [INFO] exitTimerLoopCallback(" << d << " , " << id << " )" << endl;
#endif

    masterSendNMTstateChange(&CanARD_Data, *(CanARD_Data.bDeviceNodeId),
            NMT_Reset_Node);
    canFestival_outNMTState.write(Stopped);
    setState(&CanARD_Data, Stopped);
}

void initialisationCallback(CO_Data* d)
{
#ifdef DEBUG_ON
    cerr << "CERR : [INFO] initialisationCallback(" << d << ")" << endl;
#endif

    canFestival_outNMTState.write(Initialisation);

    //this is automatically done by CanFestival, keep this line to remind
    //setState(&CanARD_Data, Pre_operational);
}

void preOperationalCallback(CO_Data* d)
{
#ifdef DEBUG_ON
    cerr << "CERR : [INFO] preOperationalCallback(" << d << ")" << endl;
#endif

    canFestival_outNMTState.write(Pre_operational);
}

void operationalCallback(CO_Data* d)
{
#ifdef DEBUG_ON
    cerr << "CERR : [INFO] operationalCallback(" << d << ")" << endl;
#endif

    canFestival_outNMTState.write(Operational);
}

void stoppedCallback(CO_Data* d)
{
#ifdef DEBUG_ON
    cerr << "CERR : [INFO] stoppedCallback(" << d << ")" << endl;
#endif

    canFestival_outNMTState.write(Stopped);
}

void heartbeatErrorCallback(CO_Data* d, UNS8 heartbeatID)
{
#ifdef DEBUG_ON
    cerr << "CERR : [INFO] heartbeatErrorCallback(" << d << " , " << heartbeatID << " )" << endl;
#endif
}

void postSyncCallback(CO_Data* d)
{
#ifdef DEBUG_ON
    cerr << "CERR : [INFO] postSyncCallback(" << d << ")" << endl;
#endif
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    canFestival_outSyncSent.write(now);
}

void postTPDOCallback(CO_Data* d)
{
#ifdef DEBUG_ON
    cerr << "CERR : [INFO] postTPDOCallback(" << d << ")" << endl;
#endif
}

void postEmcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg)
{
#ifdef DEBUG_ON
    printf("CERR : [INFO] postEmcy : received EMCY message :\nNode: %2.2x\nErrorCode: %4.4x\nErrorRegister: %2.2x\n", nodeID, errCode, errReg);
#endif
}

void postSlaveBootup(CO_Data* d, UNS8 nodeId)
{
#ifdef DEBUG_ON
    printf("CERR : [INFO] postSlaveBootup(d=%x, nodeId=%2.2x)\n", d, nodeId);
#endif

    canFestival_outBootUpReceived.write((int) nodeId);
}
