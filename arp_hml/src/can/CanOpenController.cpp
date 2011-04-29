/*
 * CanOpenController.cpp
 *
 *  Created on: 10 févr. 2011
 *      Author: ard
 */
#include <stdio.h>
#include <string.h>
#include <ocl/Component.hpp>

#include "can/CanOpenController.hpp"
#include "can/wrappers/can_festival_ARD_master_wrapper.hpp"

using namespace arp_hml;
using namespace arp_core;

ORO_LIST_COMPONENT_TYPE( arp_hml::CanOpenController )

CanOpenController::CanOpenController(const std::string& name):
    ARDTaskContext(name),
    attrCurrentNMTState(Unknown_state),
    propCanFestivalDriverName("/opt/ros/ard/can_festival/lib/libcanfestival_can_socket.so"),
    propBusName("can1"),
    propBaudRate("250K"),
    propNodeId(0),
    propMasterMaxBootDelay(50),
    m_dispatcher(this),
    m_canPort(NULL)

{
    addAttribute("attrCurrentNMTState",attrCurrentNMTState);

    addProperty("propCanFestivalDriverName",propCanFestivalDriverName)
        .doc("contains the name of the can driver library that will be loaded dynamically");
    addProperty("propBusName",propBusName)
        .doc("contains the name of the bus attached to the CanController");
    addProperty("propBaudRate",propBaudRate)
        .doc("contains the baudrate of the attached bus (10K,250K,500K,1000K, ...)");
    addProperty("propNodeId",propNodeId)
        .doc("contains the nodeID of the Controller node on the attached bus (in decimal)");
    addProperty("propMasterMaxBootDelay",propMasterMaxBootDelay)
        .doc("defines the maximal allowed duration for master bootup (in ms)");

    addPort("inControllerNmtState",inControllerNmtState)
        .doc("This port is connected to the CanFestival thread to populate attrCurrentNMTState");
    addPort("inBootUpReceived",inBootUpReceived)
        .doc("his port is connected to the CanFestival thread to dispatch the boot event to registred Device Components");

    /**
     * Register/Unregister
     */
    addOperation("ooRegisterNewNode", &CanOpenDispatcher::ooRegisterNewNode, &m_dispatcher, OwnThread )
        .doc("This operation allows a Device Component to register in the CanController. Returns true if the node has been registred successfully")
        .arg("node","the 'idCard' of the node who wants to be added");
    addOperation("ooUnregisterNode", &CanOpenDispatcher::ooUnregisterNode, &m_dispatcher, OwnThread )
        .doc("This operation allows a Device Component to unregister from the CanController. Returns true if the node has been unregistred successfully")
        .arg("node","the nodeId of the node who wants to be removed");


    /**
     * SDO and dictionnary Access
     */
    addOperation("coWriteInLocalDico", &CanOpenController::coWriteInLocalDico, this, ClientThread )
        .doc("This operation allows anyone in the application to write in the local dictionnary")
        .arg("dicoEntry", "a structure containing the information to write in the local dictionnary");
    addOperation("coReadInLocalDico", &CanOpenController::coReadInLocalDico, this, ClientThread)
        .doc("This operation allows anyone in the application to read the value of the local dictionnary entry")
        .arg("dicoEntry","a structure containing the information to read from the local dictionnary. The result value is written in the receivedData param.");
    addOperation("coWriteInRemoteDico", &CanOpenController::coWriteInRemoteDico, this, ClientThread )
        .doc("This operation allows anyone in the application to write in a remote dictionnary")
        .arg("dicoEntry","a structure containing the information to write in a remote dictionnary this is done via sdo. ");
    addOperation("coReadInRemoteDico", &CanOpenController::coReadInRemoteDico, this, ClientThread)
        .doc("This operation allows anyone in the application to read from a remote dictionnary. It returns an int you have to convert depending of the type value")
        .arg("dicoEntry","a structure containing the information to read from the remote dictionnary. The result value is written in the dicoEntry.value entry.")
        .arg("receivedData","the value read from Can, it is the caller responsibility to conver it into the right format");
    addOperation("ooResetSdoBuffers", &CanOpenController::ooResetSdoBuffers, this, OwnThread)
        .doc("This operation allows to reset all the SDO emission lines. It should be use with care as it is a very intrusive behavior.");

    /**
     * NMT management
     */
    addOperation("coMasterSetNmtNodeState", &CanOpenController::coMasterSetNmtNodeState, this, ClientThread)
        .doc("Send an NMT state command to a Node. return : true when the request has been successfully sended. This is a blocking call if timeout != 0.")
        .arg("nodeId","node ID of the node to which you want to request a new NMT state (or 0 for all nodes on Can)")
        .arg("nmtStateCmd","the requested NMT state")
        .arg("timeout","maximal blocking time");
    addOperation("coMasterAskNmtNodeState", &CanOpenController::coMasterAskNmtNodeState, this, ClientThread)
        .doc("Send a node guard request : ask to a node what is its current NMT state. return : true when the request has been successfully sended. This is a blocking call if timeout != 0.")
        .arg("nodeId", "the node ID of the node of which we want to receive the NMT state, Must not be equal to 0")
        .arg("timeout","maximal blocking time");


    /**
     * Debug Operations
     */
    addOperation("ooPrintRegisteredNodes", &CanOpenDispatcher::ooPrintRegisteredNodes, &m_dispatcher, OwnThread)
        .doc("DEBUG purposes : this operation prints in the console the registred nodes.");
}

CanOpenController::~CanOpenController()
{
    StopTimerLoop(&exitTimerLoopCallback);
    TimerCleanup();
}

bool CanOpenController::configureHook()
{
    bool res = ARDTaskContext::configureHook();

    //Initialize all CanFestival related stuff (shared datas, wrappers, timers loop, loading drivers,...)
    res &= initializeCanFestival();
    if( res )
    {
        LOG(Info) << "CanFestival initialization succeed" << endlog();
    }

    //on donne propMasterMaxBootDelay ms aux devices pour booter
    //à l'issue de ce temps on doit être passé en pre-op
    if( res )
    {
        usleep(1000*propMasterMaxBootDelay);
        inControllerNmtState.read(attrCurrentNMTState);
        if( attrCurrentNMTState == Pre_operational )
        {
            LOG(Info) << propBusName << " NMT Master in Pre-Operationnal state" << endlog();
        }
        else
        {
            LOG(Error) << "failed to Configure : took too much time to transit into Pre-Operationnal state" << endlog();
            res = false;
        }
    }

    return res;
}

bool CanOpenController::startHook()
{
    bool res = ARDTaskContext::startHook();

    EnterMutex();
    UNS8 cmdResult = setState(&CanARD_Data, Operational);
    LeaveMutex();

    if( cmdResult != Operational )
    {
        LOG(Error) << "startHook: failed to switch to Operationl state" << endlog();
        res = false;
    }

    return res;
}

/**
 * Overloaded to update the attrCurrentNMTState value
 */
void CanOpenController::updateHook()
{
    ARDTaskContext::updateHook();

    //synchronize local attribute with CanFestival state
    inControllerNmtState.read(attrCurrentNMTState);

    //dispatch bootup frame to the rigth output port
    m_dispatcher.dispatchBootUp(propNodeId, inBootUpReceived);

    //dispatch bootup frame to the rigth output port
    m_dispatcher.dispatchNmtState();
}


void CanOpenController::cleanupHook()
{
    StopTimerLoop(&exitTimerLoopCallback);
    TimerCleanup();

    ARDTaskContext::cleanupHook();
}

bool CanOpenController::initializeCanFestival()
{
    bool res = true;

    // Initializes CANFestival shared datas
    res &= initialiazeCanFestivalDatas();
    if( res == false )
    {
        LOG(Error) << "failed to configure : initialiazeCanFestivalDatas" << endlog();
    }

    // Open the CAN driver (such as socketCan,rtCan, ...)
    if (LoadCanDriver(propCanFestivalDriverName.c_str()) == NULL)
    {
        LOG(Error) << "failed to configure : unable to load library: " << propCanFestivalDriverName  << endlog();
        res &= false;
    }

    // Initializes CANFestival timers
    TimerInit();

    //Initializes CANFestival wrappers
    res &= initialiazeCanFestivalWrappers();
    if( res == false )
    {
        LOG(Error) << "failed to configure : initialiazeCanFestivalDatas" << endlog();
    }

    //Open the propBusName CAN bus
    res &= openCanBus();
    if( res == false )
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
    CanARD_Data.heartbeatError =    heartbeatErrorCallback;
    CanARD_Data.initialisation =    initialisationCallback;
    CanARD_Data.preOperational =    preOperationalCallback;
    CanARD_Data.operational =       operationalCallback;
    CanARD_Data.stopped =           stoppedCallback;
    CanARD_Data.post_sync =         postSyncCallback;
    CanARD_Data.post_TPDO =         postTPDOCallback;
    CanARD_Data.post_emcy =         postEmcy;
    CanARD_Data.post_SlaveBootup=   postSlaveBootup;

    return res;
}

bool CanOpenController::initialiazeCanFestivalWrappers()
{
    bool res = true ;

    if( !initWrapper() )
    {
        LOG(Error) << "failed to configure : initWrapper failed" << endlog();
        res &= false;
    }

    if( !inControllerNmtState.connectTo(&canFestival_outNMTState) )
    {
        LOG(Error) << "failed to configure : inControllerNmtState failed to connect to canFestival_outNMTState" << endlog();
        res &= false;
    }

    if( !inBootUpReceived.connectTo(&canFestival_outBootUpReceived,
            ConnPolicy::buffer(128, ConnPolicy::LOCK_FREE, true, false)) )
    {
        LOG(Error) << "failed to configure : inBootUpReceived failed to connect to canFestival_outBootUpReceived" << endlog();
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
    CAN_PORT canPort = canOpen(&canChannel,&CanARD_Data);
    if( canPort == NULL)
    {
        LOG(Error) << "failed to configure :  canOpen(" << canChannel.busname << "," << canChannel.baudrate << ") failed." << endlog();
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



//TODO à implémenter
bool CanOpenController::coWriteInLocalDico(CanDicoEntry dicoEntry)
{
    bool res = true;

    res = false;
    cerr << "coWriteInRemoteDico not implemented !" << endl;

    return res;
}

//TODO à implémenter
bool CanOpenController::coReadInLocalDico(CanDicoEntry& dicoEntry)
{
    bool res = true;

    res = false;
    cerr << "coWriteInRemoteDico not implemented !" << endl;

    return res;
}

bool CanOpenController::coWriteInRemoteDico(CanDicoEntry dicoEntry)
{
    bool res = true;

    //send SDO to remote node
    EnterMutex();
    int writeResult = writeNetworkDict (
            &CanARD_Data,
            (UNS8) (dicoEntry.nodeId),
            (UNS16) (dicoEntry.index),
            (UNS8) (dicoEntry.subindex),
            dicoEntry.size,
            dicoEntry.dataType,
            &(dicoEntry.value)
            );
    LeaveMutex();

    if( writeResult == 0xFF )
    {
        LOG(Error) << "Failed to send write SDO request : writeResult=" << writeResult <<  endl;
        res &= false;
    }
    else
    {
        //check received SDO
        UNS32 abortCode;
        EnterMutex();
        writeResult = getWriteResultNetworkDict (&CanARD_Data, dicoEntry.nodeId, &abortCode);
        LeaveMutex();
        while ( writeResult == SDO_DOWNLOAD_IN_PROGRESS )
        {
        	EnterMutex();
            writeResult = getWriteResultNetworkDict (&CanARD_Data, dicoEntry.nodeId, &abortCode);
            LeaveMutex();
            usleep(1000);
        }

        if( writeResult == SDO_FINISHED )
        {
            /* Finalise last SDO transfer with this node */
        	EnterMutex();
        	closeSDOtransfer(&CanARD_Data, dicoEntry.nodeId, SDO_CLIENT);
        	LeaveMutex();
        }
        else
        {
            LOG(Error) << "Failed to send SDO : writeResult=" << writeResult << " abortCode="<< (unsigned int) abortCode << endl;
            res = false;
        }
    }

    return res;
}

bool CanOpenController::coReadInRemoteDico(CanDicoEntry dicoEntry, int* receivedData)
{
    bool res = true;
    uint32_t dataSize;

    EnterMutex();
    int readResult = readNetworkDict (
            &CanARD_Data,
            (UNS8) dicoEntry.nodeId,
            (UNS16) dicoEntry.index,
            (UNS8) dicoEntry.subindex,
            (UNS8) dicoEntry.dataType
            );
    LeaveMutex();

    if( readResult == 0xFF )
    {
        LOG(Error) << "Failed to send SDO read request : readResult=" << readResult <<  endl;
        res &= false;
    }
    else
    {
        //check received SDO
        UNS32 abortCode;
        EnterMutex();
        readResult = getReadResultNetworkDict (&CanARD_Data, dicoEntry.nodeId, receivedData, &dataSize, &abortCode);
        LeaveMutex();
        while ( readResult == SDO_UPLOAD_IN_PROGRESS )
        {
        	EnterMutex();
            readResult = getReadResultNetworkDict (&CanARD_Data, dicoEntry.nodeId, receivedData, &dataSize, &abortCode);
            LeaveMutex();
            usleep(1000);
        }

        if( readResult == SDO_FINISHED )
        {
            /* Finalise last SDO transfer with this node */
        	EnterMutex();
            closeSDOtransfer(&CanARD_Data, dicoEntry.nodeId, SDO_CLIENT);
            LeaveMutex();
        }
        else
        {
            LOG(Error) << "Failed to read SDO : readResult=" << readResult << " abortCode="<< (unsigned int) abortCode << endl;
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

enum_nodeState CanOpenController::coMasterSetNmtNodeState(nodeID_t nodeId, enum_DS301_nmtStateRequest nmtStateCmd, int timeout )
{
	int cmdResult;

    //vérification du paramètre de requete NMT
    if( nmtStateCmd == UnknownRequest )
    {
        LOG(Error) << "coMasterSetNmtNodeState failed you can not ask for the UnknownRequest" << endlog();
        goto failed;
    }

    //vérification du paramètre node ID interdiction du broadcast
    if( nodeId == 0x00 )
    {
        LOG(Error) << "coMasterSetNmtNodeState failed you can not ask for a broadcast request with this operation" << endlog();
        goto failed;
    }

    //envoit de la requête NMT sur le CAN
    EnterMutex();
    cmdResult = masterSendNMTstateChange(&CanARD_Data, (UNS8) nodeId, (UNS8) nmtStateCmd);
    LeaveMutex();
    if( cmdResult )
    {
        LOG(Error) << "coMasterSetNmtNodeState failed (0x" << std::hex << nodeId << ";" << nmtStateCmd << ")" << endlog();
        goto failed;
    }

    //vérification de l'état
    return coMasterAskNmtNodeState(nodeId,timeout);

    failed:
        return Unknown_state;
}

enum_nodeState CanOpenController::coMasterAskNmtNodeState(nodeID_t nodeId, int timeout )
{
    int chrono = 0;
    int cmdResult;

    //vérification du paramètre node ID interdiction du broadcast
    if( nodeId == 0x00 )
    {
        LOG(Error) << "coMasterAskNmtNodeState failed you can not ask for a broadcast request with this operation" << endlog();
        goto failed;
    }

    //envoit de la demande d'état
    EnterMutex();
    cmdResult = masterRequestNodeState (&CanARD_Data, (UNS8) nodeId);
    LeaveMutex();

    if( cmdResult )
    {
        LOG(Error) << "coMasterAskNmtNodeState failed (0x" << std::hex << nodeId << ")" << endlog();
        goto failed;
    }

    //si le timeout est définit on attend la réponse
    if( timeout )
    {
        while( CanARD_Data.NMTable[nodeId] == Unknown_state && chrono < timeout )
        {
            LOG(Debug) << "coMasterAskNmtNodeState : Node " << propNodeId << " is waiting for a NMT start frame ..." << endlog();
            chrono++;
            usleep(1000);
        }
        //si le timeout est explosé c'est que ça a foiré
        if( chrono >= timeout )
        {
            LOG(Error)  << "coMasterAskNmtNodeState : timeout has expired, no NMT frame received in time for node 0x"<< std::hex << propNodeId << endlog();
            goto failed;
        }
    }

    m_dispatcher.dispatchNmtState(nodeId);
    return CanARD_Data.NMTable[nodeId];

    failed:
        return Unknown_state;
}
