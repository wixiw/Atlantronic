/*
 * CanOpenController.cpp
 *
 *  Created on: 10 févr. 2011
 *      Author: ard
 */
#include <stdio.h>
#include <string.h>
#include <canfestival/canfestival.h>
#include "orocos/can/dictionnary/CanARD.h"
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <iostream>
#include <math/core>
#include <time/ArdTime.hpp>
using namespace std;
using namespace arp_math;
using namespace arp_time;

std::string propBusName("can0");
std::string propBaudRate("1000K");
int propNodeId(0);
double propSyncPeriod(0.010);
std::string propCanFestivalDriverName("/opt/ard/can_festival/lib/libcanfestival_can_socket.so");
/** This is a temporary buffer to pass the propBusName string to the CanFestival process */
char m_busNameLocalCopy[100];
/** This is a temporary buffer to pass the propBaudRate string to the CanFestival process */
char m_baurateLocalCopy[6];

#define LOG(truc) cout
#define endlog() endl

int MY_PRIORITY;

#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */

ArdAbsoluteTime last;
double max_period = 0;
bool first_time = true;

void postSyncCallback(CO_Data* d)
{
    ArdAbsoluteTime now = getAbsoluteTime();
    if( first_time )
    {
        first_time = false;
        last = now;
    }
    ArdTimeDelta period = getTimeDelta(last,now);
    if( period > max_period)
    {
        max_period = period;
    }
    //cout << "[INFO] postSyncCallback = " << period << "s (max=" << max_period << ")"<< endl;
    last = now;
}

void startTimerLoopCallback(CO_Data* d, UNS32 id)
{
    cout << "[INFO] startTimerLoopCallback(" << d << " , " << id << " )" << endl;
}

void exitTimerLoopCallback(CO_Data* d, UNS32 id)
{
    cout << "[INFO] exitTimerLoopCallback(" << d << " , " << id << " )" << endl;
}

void initialisationCallback(CO_Data* d)
{
    cout << "[INFO] initialisationCallback(" << d << ")" << endl;
}

void preOperationalCallback(CO_Data* d)
{
}

void operationalCallback(CO_Data* d)
{
}

void stoppedCallback(CO_Data* d)
{
    cout << "[INFO] stoppedCallback(" << d << ")" << endl;
}

void heartbeatErrorCallback(CO_Data* d, UNS8 heartbeatID)
{
}

void postTPDOCallback(CO_Data* d)
{
}

void postEmcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg)
{
}

void postSlaveBootup(CO_Data* d, UNS8 nodeId)
{
}


bool initialiazeCanFestivalDatas()
{
    bool res = true;

    //dictionnary initialization
    setNodeId(&CanARD_Data, propNodeId);
    CanARD_Data.heartbeatError = heartbeatErrorCallback;
    CanARD_Data.initialisation = initialisationCallback;
    CanARD_Data.preOperational = preOperationalCallback;
    CanARD_Data.operational = operationalCallback;
    CanARD_Data.stopped = stoppedCallback;
    CanARD_Data.post_sync = postSyncCallback;
    CanARD_Data.post_TPDO = postTPDOCallback;
    CanARD_Data.post_emcy = postEmcy;
    CanARD_Data.post_SlaveBootup = postSlaveBootup;

    return res;
}

bool initializeCanFestival()
{
    bool res = true;

    // Initializes CANFestival shared datas
    res &= initialiazeCanFestivalDatas();
    if (res == false)
    {
        LOG(Error) << "failed to configure : initialiazeCanFestivalDatas"
                << endlog();
    }

    // Open the CAN driver (such as socketCan,rtCan, ...)
    if (LoadCanDriver(propCanFestivalDriverName.c_str()) == NULL)
    {
        LOG(Error) << "failed to configure : unable to load library: "
                << propCanFestivalDriverName << endlog();
        res &= false;
    }

    // Start periodic CANFestival Task, since now, every call to CanFestival stack should be surrounded by mutex locks
    // Initializes CANFestival timers
    TimerInit();
    StartTimerLoop(NULL);

    return res;
}

bool openCanBus()
{
    bool res = true;

    // Board description
    s_BOARD canChannel;
    strcpy(m_busNameLocalCopy, propBusName.c_str());
    strcpy(m_baurateLocalCopy, propBaudRate.c_str());
    canChannel.busname = m_busNameLocalCopy;
    canChannel.baudrate = m_baurateLocalCopy;

    /* Open the bus for communication. */
    CAN_PORT canPort = canOpen(&canChannel, &CanARD_Data);
    if (canPort == NULL)
    {
        LOG(Error) << "openCanBus() : failed to configure :  canOpen("
                << canChannel.busname << "," << canChannel.baudrate
                << ") failed." << endlog();
        TimerCleanup();
        res &= false;
    }
    else
    {
        EnterMutex();
        UNS8 cmdResult = setState(&CanARD_Data, Initialisation);
        LeaveMutex();

        if( cmdResult != Initialisation && cmdResult != Pre_operational )
        {
            LOG(Error) << "openCanBus() : bus is not in init state. Current state=0x" << std::hex << (int) cmdResult << endlog();
            res &= false;
        }
    }

    return res;
}

void goRt()
{
    struct sched_param param;

    /* Declare ourself as a real time task */

    param.sched_priority = MY_PRIORITY;
    if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
            perror("sched_setscheduler failed");
            exit(-1);
    }

    /* Lock memory */

    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
            perror("mlockall failed");
            exit(-2);
    }

    /* Pre-fault our stack */
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

int main(int argc, char* argv[])
{
    bool res = true;

    int c;
    opterr = 0;
    char *pvalue = NULL;

    while ((c = getopt(argc, argv, "p:")) != -1)
        switch (c)
        {
            case 'p':
                pvalue = optarg;
                sscanf(pvalue,"%d", &MY_PRIORITY);
                goRt();
                break;
            case '?':
                if (optopt == 'p')
                    fprintf(stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint(optopt))
                    fprintf(stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
                return 1;
            default:
                abort();
        }


    //Initialize all CanFestival related stuff (shared datas, wrappers, timers loop, loading drivers,...)
    res &= initializeCanFestival();
    if (res)
    {
        LOG(Info) << "CanOpenController initialization succeed" << endlog();
    }

    //Open the propBusName CAN bus
    res &= openCanBus();
    if (res == false)
    {
        LOG(Error) << "failed to configure : openCanBus" << endlog();
    }

    cout << "Timer started" << endl;

    sleep(30);

    cout << "[INFO] postSyncCallback (max=" << max_period << ")"<< endl;

    StopTimerLoop(&exitTimerLoopCallback);
    TimerCleanup();
}
