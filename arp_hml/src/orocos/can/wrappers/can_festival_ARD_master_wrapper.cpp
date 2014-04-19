/*********************************************************************
 *  This file contains canFestival callbacks. As we wish to have them
 *  in C++, we need this wrapper
 **********************************************************************/

#include "orocos/can/wrappers/can_festival_ARD_master_wrapper.hpp"

using namespace std;
using namespace arp_hml;
using namespace arp_time;

//#define DEBUG_ON


string canFestivalWrapperName;
RTT::OutputPort<e_nodeState> canFestival_outNMTState;
RTT::OutputPort<nodeID_t> canFestival_outBootUpReceived;
RTT::OutputPort<ArdAbsoluteTime> canFestival_outSyncSent;

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
}

void exitTimerLoopCallback(CO_Data* d, UNS32 id)
{
#ifdef DEBUG_ON
    cerr << "CERR : [INFO] exitTimerLoopCallback(" << d << " , " << id << " )" << endl;
#endif

    EnterMutex();
    masterSendNMTstateChange(&CanARD_Data, *(CanARD_Data.bDeviceNodeId),
            NMT_Reset_Node);
    canFestival_outNMTState.write(Stopped);
    setState(&CanARD_Data, Stopped);
    LeaveMutex();
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
    //canFestival_outSyncSent.write(getAbsoluteTime());
#ifdef DEBUG_ON
    cerr << "CERR : [INFO] postSyncCallback(" << d << ") " << now.tv_sec << "s " << now.tv_nsec << "ns"<< endl;
#endif
}

void postTPDOCallback(CO_Data* d)
{
#ifdef DEBUG_ON
    ArdAbsoluteTime now = getAbsoluteTime();
    cerr << "CERR : [INFO] postTPDOCallback(" << d << ") " << now. << "s."<< endl;
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
