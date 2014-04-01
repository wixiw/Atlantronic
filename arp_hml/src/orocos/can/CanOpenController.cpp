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
using namespace arp_time;
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
            propSyncPeriod(0.010),
            propTimeReporting(false),
            m_dispatcher(*this),
            m_canPort(NULL),
            m_RunningState(UNKNOWN),
            m_timer(0)
{
    attrLastSyncTime = getAbsoluteTime();
    attrSyncTime = getAbsoluteTime();
    createOrocosInterface();

    outBusConnected.write(false);
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
        LOG(Info) << "CanOpenController initialization succeed" << endlog();
    }

    return res;
}

bool CanOpenController::startHook()
{
    bool res = HmlTaskContext::startHook();

    //Open the propBusName CAN bus
    res &= openCanBus();
    if (res == false)
    {
        LOG(Error) << "failed to configure : openCanBus" << endlog();
    }

    m_RunningState = SETTING_UP_DEVICES;
    LOG(Info) << "CanOpenController started." << endlog();

    UNS8 cmdResult = setState(&CanARD_Data, Operational);
    if (cmdResult != Operational)
    {
        LOG(Error) << "startHook: failed to switch to Operationl state"
                << endlog();
        res &= false;
    }

    //reset du bus.
    ooResetCanBus();

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
    ArdTimeDelta period = getTimeDelta(attrLastSyncTime, attrSyncTime);
    attrLastSyncTime = attrSyncTime;

    outPeriod.write(period);
    outClock.write(attrSyncTime);

    HmlTaskContext::updateHook();

    //synchronize local attribute with CanFestival state
    inControllerNmtState.read(attrCurrentNMTState);

    //dispatch bootup frame to the rigth output port
    m_dispatcher.dispatchBootUp(propNodeId, inBootUpReceived);

    switch (m_RunningState)
    {
        case SETTING_UP_DEVICES:
            settingUpDeviceHook();
            break;

        case RUNNING:
            runningHook();
            break;

        default:
            LOG(Error) << "CanOpenController::updateHook(): unknown m_RunningState=" << m_RunningState << endlog();
            break;
    }

    //tac
    if( propTimeReporting )
    {
        m_timer.Stop();
        //LOG(Info) << "Period = " << period*1000<< "ms" << endlog();
    }
}

void CanOpenController::settingUpDeviceHook()
{
    bool res = true;

    res &= m_dispatcher.configureAll();
    res &= m_dispatcher.waitSlavesState(OPERATIONAL, 3.0);

    if( res == false )
    {
        LOG(Error) << "CanOpenController settingUpDeviceHook failed." << endlog();
        return;
    }

    //reactivate SYNC message
    if( false == ooSetSyncPeriod(propSyncPeriod) )
    {
        return;
    }

    m_RunningState = RUNNING;
    LOG(Info) << "CanOpenController settingUpDeviceHook success goigt into RUNNING state." << endlog();
}

void CanOpenController::runningHook()
{
    //scheduling de tous les peers
    m_dispatcher.triggerAllRead();

    //actions de fin de scheduling pour les devices qui vont ecrire sur le bus
    m_dispatcher.triggerAllWrite();

    //envoit des PDO manuels de fin de cycle
    sendPDOevent(&CanARD_Data);
}

void CanOpenController::stopHook()
{
    //desactivate SYNC msg
    ooSetSyncPeriod(0);
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

    //Initializes CANFestival wrappers
    res &= initialiazeCanFestivalWrappers();
    if (res == false)
    {
        LOG(Error) << "failed to configure : initialiazeCanFestivalDatas"
                << endlog();
    }

    // Start periodic CANFestival Task, since now, every call to CanFestival stack should be surrounded by mutex locks
    // Initializes CANFestival timers
    TimerInit();
    StartTimerLoop(startTimerLoopCallback);

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
    //low frequency to initialise PDO tables => cracra mais c'est la faute à CanFestival
    ooSetSyncPeriod(0);

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
        LOG(Error) << "openCanBus() : failed to configure :  canOpen("
                << canChannel.busname << "," << canChannel.baudrate
                << ") failed." << endlog();
        TimerCleanup();
        res &= false;
    }
    else
    {
        m_canPort = canPort;

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

    LOG(Info) << "Write value=0x" << std::hex << dicoEntry.value << " successfully in 0x" << std::hex << dicoEntry.index
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
        ArdAbsoluteTime begin, end;
        begin = getAbsoluteTime();
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
            end = getAbsoluteTime();
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

bool CanOpenController::coSendNmtCmd(nodeID_t nodeId, enum_DS301_nmtStateRequest nmtStateCmd)
{
    int cmdResult = 0;

    if( nodeId < 0 || nodeId >= 0xFF )
    {
        LOG(Error) << "dispatchNmtState : Node 0x" <<  std::hex << nodeId  << " is out of Node ID bounds " << endlog();
        goto failed;
    }

    if( nmtStateCmd == UnknownRequest )
    {
        LOG(Error) << "coSendNmtCmd :  failed you can not ask for the UnknownRequest" << endlog();
        goto failed;
    }

    //send NMT cmd
    EnterMutex();
    cmdResult = masterSendNMTstateChange(&CanARD_Data, (UNS8) nodeId, (UNS8) nmtStateCmd);
    LeaveMutex();
    if( cmdResult )
    {
        LOG(Error) << "coSendNmtCmd : failed to send NMT command to node 0x" << std::hex << nodeId << "; for command " << nmtStateCmd << endlog();
        goto failed;
    }

    //everything wnet ok
    goto success;

    failed:
        return false;
    success:
        return true;
}

bool CanOpenController::coRequestNmtChange(nodeID_t nodeId, enum_DS301_nmtStateRequest nmtStateCmd, ArdTimeDelta timeout)
{
    int cmdResult = 0;
    ArdTimeDelta chrono = 0;

    //send NMT cmd
    if( !coSendNmtCmd(nodeId,nmtStateCmd) )
    {
        LOG(Error) << "CanOpenController::coRequestNmtChange : failed to send NMT command " << nmtStateCmd  << " to node 0x" << std::hex << nodeId << endlog();
        goto failed;
    }

    //send the NMT state request
    EnterMutex();
    cmdResult = masterRequestNodeState (&CanARD_Data, (UNS8) nodeId);
    LeaveMutex();
    if( cmdResult )
    {
        LOG(Error) << "CanOpenController::coRequestNmtChange : failed to send NMT state request " << nmtStateCmd  << " to node 0x" << std::hex << nodeId << endlog();
        goto failed;
    }

   //polling on the NMT state because CAN Festival is not doing node guarding properly ...
   //chrono is not reseted on purpose to continue the counting if we are reseting a node
   whileTimeout( !isNmtStateChangeDone(nmtStateCmd, nodeId) , timeout, 0.010 )
   //si le timeout est explosé c'est que ça a foiré
   IfWhileTimeoutExpired(timeout)
   {
        LOG(Error) << "CanOpenController::coRequestNmtChange : timeout expired 0x" << std::hex << nodeId << " for command " << nmtStateCmd << endlog();
        goto failed;
   }

   //here everything is OK.
   goto success;

    failed:
        return false;
    success:
        return true;

}

bool CanOpenController::isNmtStateChangeDone(enum_DS301_nmtStateRequest nmtCmd, nodeID_t nodeId)
{
    bool res = false;
    enum_nodeState nmtState;

    EnterMutex();
    nmtState = CanARD_Data.NMTable[nodeId];
    LeaveMutex();

    switch( nmtCmd )
    {
        case StartNode:
            if( nmtState == ::Operational)
                res = true;
            break;
        case StopNode:
            if( nmtState == ::Stopped)
                res = true;
            break;
        case EnterPreOp:
            if( nmtState == ::Pre_operational)
                res = true;
            break;
        case ResetNode:
        case ResetComunication:
            if( nmtState == ::Initialisation ||
                nmtState == ::Disconnected ||
                nmtState == ::Connecting ||
                nmtState == ::Preparing ||
                nmtState == ::Pre_operational
                    )
                res = true;
            break;
        case UnknownRequest:
            break;
    }

    return res;
}

void CanOpenController::ooResetSdoBuffers()
{
    EnterMutex();
    resetSDO(&CanARD_Data);
    LeaveMutex();
}

bool CanOpenController::ooSetSyncPeriod(ArdTimeDelta period)
{
    CanDicoEntry dicoEntry(0xFF, 0x1006, 0x00, (int) (period * 1E6), 0, 4);
    return coWriteInLocalDico(dicoEntry);
}

void CanOpenController::ooResetCanBus()
{
    LOG(Info) << "CanOpen bus RESET request." << endlog();

    //deactivate SYNC message
    if( false == ooSetSyncPeriod(0) )
    {
        LOG(Error) << "CanOpenController::ooResetCanBus failed to mute sync" << endlog();
        return;
    }

    if( false == coSendNmtCmd(0x0, ResetNode) )
    {
        LOG(Error) << "CanOpenController::ooResetCanBus failed to send reset cmd" << endlog();
        return;
    }

    //TODO faire mieux pour attendre les bootups
    sleep(2);

    //changing state
    m_RunningState = SETTING_UP_DEVICES;

    LOG(Info) << "CanOpen bus reseted, will now configure nodes." << endlog();

    update();
}


void CanOpenController::ooGetPerformanceReport()
{
    if( !isRunning() || !propTimeReporting )
        cout << "Time Stats are disabled. The component must be in running state with propTimereporting=true." << endl;
    else
        cout << m_timer.GetReport() << endl;
}

void CanOpenController::ooSetMaxBufferSize(unsigned int size)
{
    m_timer.SetMaxBufferSize(size);
}


void CanOpenController::createOrocosInterface()
{
    //TODO makes the "ls" in orocos deployer buggy, the typekit is certainly broken"
    //addAttribute("attrCurrentNMTState", attrCurrentNMTState);
      addAttribute("attrSyncTime", attrSyncTime);
      addAttribute("attrLastSyncTime", attrLastSyncTime);
      addAttribute("sdo", attrTestingSdo);

      addProperty("propCanFestivalDriverName", propCanFestivalDriverName) .doc(
              "contains the name of the can driver library that will be loaded dynamically");
      addProperty("propBusName", propBusName) .doc(
              "contains the name of the bus attached to the CanController");
      addProperty("propBaudRate", propBaudRate) .doc(
              "contains the baudrate of the attached bus (10K,250K,500K,1000K, ...)");
      addProperty("propNodeId", propNodeId) .doc(
              "contains the nodeID of the Controller node on the attached bus (in decimal)");
      addProperty("propSyncPeriod", propSyncPeriod) .doc(
              "delay between 2 SYNC messages in s ");
      addProperty("propTimeReporting",propTimeReporting).doc(""
              "Set this to true to activate timing reporting");


      addEventPort("inSync", inSync) .doc(
              "wakes up the component on SYNC message");
      addPort("inControllerNmtState", inControllerNmtState) .doc(
              "This port is connected to the CanFestival thread to populate attrCurrentNMTState");
      addEventPort("inBootUpReceived", inBootUpReceived) .doc(
              "his port is connected to the CanFestival thread to dispatch the boot event to registred Device Components");
      addPort("outClock", outClock) .doc("It contains the SYNC CAN message date");
      addPort("outPeriod", outPeriod) .doc("Delay beetween inSync and last cycle inSync in s");
      addPort("outBusConnected", outBusConnected) .doc("Is true when the bus is electronically up. Typically it's down when emergency stop is on");

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
      addOperation("coSendNmtCmd", &CanOpenController::coSendNmtCmd,
              this, ClientThread) .doc("This operation allows anyone in the application to request a new slave node NMT state. Result is *NOT* checked")
              .arg("nodeId","CanOpen ID of the targeted node")
              .arg("requestedNmtState","target NMT state command");
      addOperation("coRequestNmtChange", &CanOpenController::coRequestNmtChange,
              this, ClientThread) .doc("This operation allows anyone in the application to request a new slave node NMT state. Result is checked.")
              .arg("nodeId","CanOpen ID of the targeted node")
              .arg("requestedNmtState","target NMT state command")
              .arg("timeout","Delay to wait before considering the device did not transit");

      addOperation("ooResetSdoBuffers", &CanOpenController::ooResetSdoBuffers,
              this, OwnThread) .doc(
              "This operation allows to reset all the SDO emission lines. It should be use with care as it is a very intrusive behavior.");

      /**
       * Can BUS management
       */
      addOperation("ooSetSyncPeriod", &CanOpenController::ooSetSyncPeriod, this,
              OwnThread) .doc("define a new period for SYNC object") .arg(
              "period", "in s.");
      addOperation("ooResetCanBus", &CanOpenController::ooResetCanBus, this,
              OwnThread) .doc("Restart the can bus and all connected devices.");

      /**
       * Debug Operations
       */
      addOperation("ooPrintRegisteredNodes",
              &CanOpenDispatcher::ooPrintRegisteredNodes, &m_dispatcher,
              OwnThread) .doc(
              "DEBUG purposes : this operation prints in the console the registred nodes.");


      addOperation("ooGetPerformanceReport", &CanOpenController::ooGetPerformanceReport, this, OwnThread).doc("Get Performance report about timing, already formated in readable string");
      addOperation("ooSetMaxBufferSize", &CanOpenController::ooSetMaxBufferSize, this, OwnThread)
              .arg(
                        "bufSize",
                        "New number of period that will be logged.")
              .doc("Change the timing log buffer size.");

}
