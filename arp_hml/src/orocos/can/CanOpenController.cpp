/*
 * CanOpenController.cpp
 *
 *  Created on: 10 févr. 2011
 *      Author: ard
 */
#include <stdio.h>
#include <string.h>
#include <rtt/Component.hpp>

#include "CanOpenController.hpp"
#include "orocos/can/wrappers/can_festival_ARD_master_wrapper.hpp"
#include <math/core>

using namespace arp_hml;
using namespace arp_core;
using namespace std;
using namespace Eigen;

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE( arp_hml::CanOpenController)

CanOpenController::CanOpenController(const std::string& name) :
            HmlTaskContext(name),
            attrCurrentNMTState(Unknown_state),
            propCanFestivalDriverName(
                    "/opt/ard/can_festival/lib/libcanfestival_can_socket.so"),
            propBusName("can0"), 
            propBaudRate("1000K"), 
            propNodeId(0),
            propMasterMaxBootDelay(0.500), 
            propSyncPeriod(0.010),
            m_dispatcher(*this),
            m_canPort(NULL)
{
    clock_gettime(CLOCK_MONOTONIC, &m_lastSyncTime);
    propTimeReporting = true;
    m_timer.SetMaxBufferSize(3000);
    createOrocosInterface();
}

CanOpenController::~CanOpenController()
{
    StopTimerLoop(&exitTimerLoopCallback);
    TimerCleanup();
}

bool CanOpenController::checkInputsPorts()
{
    return true;
}

bool CanOpenController::configureHook()
{
    bool res = HmlTaskContext::configureHook();

    //Initialize all CanFestival related stuff (shared datas, wrappers, timers loop, loading drivers,...)
    res &= initializeCanFestival();
    if (res)
    {
        LOG(Info) << "CanFestival initialization succeed" << endlog();
    }

    //on donne propMasterMaxBootDelay ms aux devices pour booter
    //à l'issue de ce temps on doit être passé en pre-op
    if (res)
    {
        usleep(propMasterMaxBootDelay * 1E6);
        inControllerNmtState.read(attrCurrentNMTState);
        if (attrCurrentNMTState == Pre_operational)
        {
            LOG(Info) << propBusName
                    << " NMT Master in Pre-Operationnal state" << endlog();
        }
        else
        {
            LOG(Error)
                    << "****************    !!!!!       ************************"
                    << endlog();
            LOG(Error)
                    << "failed to Configure : took too much time to transit into Pre-Operationnal state"
                    << endlog();
            LOG(Error)
                    << "It is possible that you forgot to reset the AU button. For an unknown reason it is blocking everything under wenomai"
                    << endlog();
            LOG(Error)
                    << "****************    !!!!!       ************************"
                    << endlog();
            res = false;
        }
    }

    return res;
}

bool CanOpenController::startHook()
{
    bool res = HmlTaskContext::startHook();

    EnterMutex();
    UNS8 cmdResult = setState(&CanARD_Data, Operational);
    LeaveMutex();

    if (cmdResult != Operational)
    {
        LOG(Error) << "startHook: failed to switch to Operationl state"
                << endlog();
        res = false;
    }
    else
    {
        LOG(Info) << propBusName << " NMT Master in Operationnal state" << endlog();
        //on laisse du temps parce qu'un reset node general a ete envoye et il faut leur laisser le temps de finir
        //pour ne pas partir avec des devices en vrac
        usleep(1.000 * 1E6);
    }

    if( propTimeReporting )
    {
        m_timer.Start();
        m_timer.Stop();
        m_timer.ResetStat();
    }


    return res;
}

/**
 * Overloaded to update the attrCurrentNMTState value
 */
void CanOpenController::updateHook()
{
    //tic
    if( propTimeReporting )
        m_timer.Start();

    //Récupération de la date du cycle CAN
    inSync.readNewest(attrSyncTime);
    //on se permet le double parce qu'on sait que la periode est petite.
    double period = arp_math::delta_t(m_lastSyncTime, attrSyncTime);
    m_lastSyncTime = attrSyncTime;

    HmlTaskContext::updateHook();

    //synchronize local attribute with CanFestival state
    inControllerNmtState.read(attrCurrentNMTState);

    //dispatch bootup frame to the rigth output port
    m_dispatcher.dispatchBootUp(propNodeId, inBootUpReceived);

    outPeriod.write(period);
    outClock.write(attrSyncTime);

    //scheduling de tous les peers
    m_dispatcher.triggerAllRead();

    //actions de fin de scheduling pour les devices qui vont ecrire sur le bus
    m_dispatcher.triggerAllWrite();

    //envoit des PDO manuels de fin de cycle
    sendPDOevent(&CanARD_Data);

    //tac
    if( propTimeReporting )
    {
        m_timer.Stop();
		LOG(Info) << "Period = " << period*1000<< "ms" << endlog();
	}
}

void CanOpenController::stopHook()
{
}


void CanOpenController::cleanupHook()
{
    m_dispatcher.unRegisterAll();

    StopTimerLoop(&exitTimerLoopCallback);
    TimerCleanup();

    HmlTaskContext::cleanupHook();
}

bool CanOpenController::initializeCanFestival()
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

    // Initializes CANFestival timers
    TimerInit();

    //Initializes CANFestival wrappers
    res &= initialiazeCanFestivalWrappers();
    if (res == false)
    {
        LOG(Error) << "failed to configure : initialiazeCanFestivalDatas"
                << endlog();
    }

    //Open the propBusName CAN bus
    res &= openCanBus();
    if (res == false)
    {
        LOG(Error) << "failed to configure : openCanBus" << endlog();
    }

    return res;
}

bool CanOpenController::initialiazeCanFestivalDatas()
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
    //synchronization de la période can_festival et de la période du composant
    ooSetSyncPeriod(propSyncPeriod);

    return res;
}

bool CanOpenController::initialiazeCanFestivalWrappers()
{
    bool res = true;

    if (!initWrapper())
    {
        LOG(Error) << "failed to configure : initWrapper failed"
                << endlog();
        res &= false;
    }

    if (!inControllerNmtState.connectTo(&canFestival_outNMTState))
    {
        LOG(Error)
                << "failed to configure : inControllerNmtState failed to connect to canFestival_outNMTState"
                << endlog();
        res &= false;
    }

    if (!inBootUpReceived.connectTo(&canFestival_outBootUpReceived,
            ConnPolicy::buffer(20, ConnPolicy::LOCK_FREE, true, false)))
    {
        LOG(Error)
                << "failed to configure : inBootUpReceived failed to connect to canFestival_outBootUpReceived"
                << endlog();
        res &= false;
    }

    if (!inSync.connectTo(&canFestival_outSyncSent))
    {
        LOG(Error)
                << "failed to configure : inSync failed to connect to canFestival_outSyncSent"
                << endlog();
        res &= false;
    }

    return res;
}

bool CanOpenController::openCanBus()
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
        LOG(Error) << "failed to configure :  canOpen("
                << canChannel.busname << "," << canChannel.baudrate
                << ") failed." << endlog();
        TimerCleanup();
        res &= false;
    }
    else
    {
        m_canPort = canPort;
        // Start periodic CANFestival Task, since now, every call to CanFestival stack should be surrounded by mutex locks
        StartTimerLoop(startTimerLoopCallback);
    }

    return res;
}

bool CanOpenController::coWriteInLocalDico(CanDicoEntry dicoEntry)
{
    int writeResult;
    UNS32 size = dicoEntry.size;
    //EnterMutex();
    writeResult
            = writeLocalDict( &CanARD_Data, dicoEntry.index, dicoEntry.subindex, &dicoEntry.value, &size, 0);
    //LeaveMutex();
    if (OD_SUCCESSFUL != writeResult)
    {
        LOG(Error) << "Write failed in 0x" << std::hex << dicoEntry.index
                << ":0x" << std::hex << dicoEntry.subindex
                << " with errorcode " << std::hex << writeResult << endlog();
        goto failed;
    }

    LOG(Info) << "Write success in 0x" << std::hex << dicoEntry.index
            << ":0x" << std::hex << dicoEntry.subindex << endlog();
    goto success;

    failed: return false;
    success: return true;
}

bool CanOpenController::coReadInLocalDico(CanDicoEntry& dicoEntry)
{
    int readResult;
    UNS32 size;
    UNS8 dataType;
    EnterMutex();
    readResult
            = readLocalDict( &CanARD_Data, dicoEntry.index, dicoEntry.subindex, &dicoEntry.value, &size, &dataType, 0);
    LeaveMutex();

    dicoEntry.size = size;
    dicoEntry.dataType = dataType;

    if (OD_SUCCESSFUL != readResult)
    {
        LOG(Error) << "Read failed in 0x" << std::hex << dicoEntry.index
                << ":0x" << std::hex << dicoEntry.subindex
                << " with errorcode " << std::hex << readResult << endlog();
        goto failed;
    }

    LOG(Info) << "Read success in 0x" << std::hex << dicoEntry.index
            << ":0x" << std::hex << dicoEntry.subindex << endlog();
    goto success;

    failed: return false;
    success: return true;
}

bool CanOpenController::coWriteInRemoteDico(CanDicoEntry dicoEntry)
{
    bool res = true;

    //send SDO to remote node
    EnterMutex();
    int writeResult = writeNetworkDict(&CanARD_Data, (UNS8) (dicoEntry.nodeId),
            (UNS16) (dicoEntry.index), (UNS8) (dicoEntry.subindex),
            dicoEntry.size, dicoEntry.dataType, &(dicoEntry.value),0);
    LeaveMutex();

    if (writeResult == 0xFF)
    {
        LOG(Error) << "Failed to send write SDO request : writeResult="
                << writeResult << endl;
        res &= false;
    }
    else
    {
        //on attend pour laisser le CAN respirer
        usleep(1000);

        //check received SDO
        UNS32 abortCode;
        EnterMutex();
        timespec begin, end;
        clock_gettime(CLOCK_MONOTONIC, &begin);
        writeResult = getWriteResultNetworkDict(&CanARD_Data, dicoEntry.nodeId,
                &abortCode);
        LeaveMutex();
        while (writeResult == SDO_DOWNLOAD_IN_PROGRESS)
        {
            EnterMutex();
            writeResult = getWriteResultNetworkDict(&CanARD_Data,
                    dicoEntry.nodeId, &abortCode);
            LeaveMutex();
            usleep(1000);
        }

        if (writeResult == SDO_FINISHED)
        {
            /* Finalise last SDO transfer with this node */
            EnterMutex();
            closeSDOtransfer(&CanARD_Data, dicoEntry.nodeId, SDO_CLIENT);
            clock_gettime(CLOCK_MONOTONIC, &end);
            LeaveMutex();
        }
        else
        {
            LOG(Error) << "Failed to send SDO 0x"<< std::hex << dicoEntry.index<< ":" << std::hex << dicoEntry.subindex << " to node 0x" << std::hex << dicoEntry.nodeId << ": writeResult="
                    << writeResult << " abortCode=0x" << std::hex << (unsigned int) abortCode
                    << endlog();
            res = false;
        }
    }

    return res;
}

bool CanOpenController::coReadInRemoteDico(CanDicoEntry dicoEntry,
        int* receivedData)
{
    bool res = true;
    uint32_t dataSize;

    EnterMutex();
    int readResult = readNetworkDict(&CanARD_Data, (UNS8) dicoEntry.nodeId,
            (UNS16) dicoEntry.index, (UNS8) dicoEntry.subindex,
            (UNS8) dicoEntry.dataType,0);
    LeaveMutex();

    if (readResult == 0xFF)
    {
        LOG(Error) << "Failed to send SDO read request : readResult="
                << readResult << endl;
        res &= false;
    }
    else
    {
        //check received SDO
        UNS32 abortCode;
        EnterMutex();
        readResult = getReadResultNetworkDict(&CanARD_Data, dicoEntry.nodeId,
                receivedData, &dataSize, &abortCode);
        LeaveMutex();
        while (readResult == SDO_UPLOAD_IN_PROGRESS)
        {
            EnterMutex();
            readResult = getReadResultNetworkDict(&CanARD_Data,
                    dicoEntry.nodeId, receivedData, &dataSize, &abortCode);
            LeaveMutex();
            usleep(1000);
        }

        if (readResult == SDO_FINISHED)
        {
            /* Finalise last SDO transfer with this node */
            EnterMutex();
            closeSDOtransfer(&CanARD_Data, dicoEntry.nodeId, SDO_CLIENT);
            LeaveMutex();
        }
        else
        {
            LOG(Error) << "Failed to read SDO : readResult=" << readResult
                    << " abortCode=" << (unsigned int) abortCode << endlog();
            res = false;
        }
    }

    return res;
}

void CanOpenController::ooResetSdoBuffers()
{
    EnterMutex();
    resetSDO(&CanARD_Data);
    LeaveMutex();
}

bool CanOpenController::ooSetSyncPeriod(double period)
{
    //propSyncPeriod = period;
    CanDicoEntry dicoEntry(0xFF, 0x1006, 0x00, (int) (period * 1E6), 0, 4);
    return coWriteInLocalDico(dicoEntry);
}

void CanOpenController::createOrocosInterface()
{
    //TODO makes the "ls" in orocos deployer buggy, the typekit is certainly broken"
    //addAttribute("attrCurrentNMTState", attrCurrentNMTState);
      addAttribute("attrSyncTime", attrSyncTime);
      addAttribute("sdo", attrTestingSdo);

      addProperty("propCanFestivalDriverName", propCanFestivalDriverName) .doc(
              "contains the name of the can driver library that will be loaded dynamically");
      addProperty("propBusName", propBusName) .doc(
              "contains the name of the bus attached to the CanController");
      addProperty("propBaudRate", propBaudRate) .doc(
              "contains the baudrate of the attached bus (10K,250K,500K,1000K, ...)");
      addProperty("propNodeId", propNodeId) .doc(
              "contains the nodeID of the Controller node on the attached bus (in decimal)");
      addProperty("propMasterMaxBootDelay", propMasterMaxBootDelay) .doc(
              "defines the maximal allowed duration for master bootup (in ms)");
      addProperty("propSyncPeriod", propSyncPeriod) .doc(
              "delay between 2 SYNC messages in s ");
      addProperty("propTimeReporting",propTimeReporting).doc(""
              "Set this to true to activate timing reporting");



      addPort("inControllerNmtState", inControllerNmtState) .doc(
              "This port is connected to the CanFestival thread to populate attrCurrentNMTState");
      addPort("inBootUpReceived", inBootUpReceived) .doc(
              "his port is connected to the CanFestival thread to dispatch the boot event to registred Device Components");
      addPort("outClock", outClock) .doc("");
      addPort("outPeriod", outPeriod) .doc("");
      addEventPort("inSync", inSync) .doc(
              "wakes up the component on SYNC message");

      /**
       * Register/Unregister
       */
      addOperation("ooRegisterNewNode", &CanOpenDispatcher::ooRegisterNewNode,
              &m_dispatcher, OwnThread) .doc(
              "This operation allows a Device Component to register in the CanController. Returns true if the node has been registred successfully") .arg(
              "node", "the 'idCard' of the node who wants to be added");
      addOperation("ooUnregisterNode", &CanOpenDispatcher::ooUnregisterNode,
              &m_dispatcher, OwnThread) .doc(
              "This operation allows a Device Component to unregister from the CanController. Returns true if the node has been unregistred successfully") .arg(
              "node", "the nodeId of the node who wants to be removed");

      /**
       * SDO and dictionnary Access
       */
      addOperation("coWriteInLocalDico", &CanOpenController::coWriteInLocalDico,
              this, ClientThread) .doc(
              "This operation allows anyone in the application to write in the local dictionnary") .arg(
              "dicoEntry",
              "a structure containing the information to write in the local dictionnary");
      addOperation("coReadInLocalDico", &CanOpenController::coReadInLocalDico,
              this, ClientThread) .doc(
              "This operation allows anyone in the application to read the value of the local dictionnary entry") .arg(
              "dicoEntry",
              "a structure containing the information to read from the local dictionnary. The result value is written in the receivedData param.");
      addOperation("coWriteInRemoteDico",
              &CanOpenController::coWriteInRemoteDico, this, ClientThread) .doc(
              "This operation allows anyone in the application to write in a remote dictionnary") .arg(
              "dicoEntry",
              "a structure containing the information to write in a remote dictionnary this is done via sdo. ");
      addOperation("coReadInRemoteDico", &CanOpenController::coReadInRemoteDico,
              this, ClientThread) .doc(
              "This operation allows anyone in the application to read from a remote dictionnary. It returns an int you have to convert depending of the type value") .arg(
              "dicoEntry",
              "a structure containing the information to read from the remote dictionnary. The result value is written in the dicoEntry.value entry.") .arg(
              "receivedData",
              "the value read from Can, it is the caller responsibility to conver it into the right format");
      addOperation("ooResetSdoBuffers", &CanOpenController::ooResetSdoBuffers,
              this, OwnThread) .doc(
              "This operation allows to reset all the SDO emission lines. It should be use with care as it is a very intrusive behavior.");
      /**
       * Others
       */
      addOperation("ooSetSyncPeriod", &CanOpenController::ooSetSyncPeriod, this,
              OwnThread) .doc("define a new period for SYNC object") .arg(
              "period", "in s.");

      /**
       * Debug Operations
       */
      addOperation("ooPrintRegisteredNodes",
              &CanOpenDispatcher::ooPrintRegisteredNodes, &m_dispatcher,
              OwnThread) .doc(
              "DEBUG purposes : this operation prints in the console the registred nodes.");

}
