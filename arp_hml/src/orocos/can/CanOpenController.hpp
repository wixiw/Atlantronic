/*
 * CanOpenController.hpp
 *
 *  Created on: 10 févr. 2011
 *      Author: ard
 */

#ifndef CANOPEN_CONTROLLER_HPP_
#define CANOPEN_CONTROLLER_HPP_

#include "orocos/taskcontexts/HmlTaskContext.hpp"
#include "orocos/can/ard_can_types.hpp"
#include "orocos/can/CanOpenDispatcher.hpp"

namespace arp_hml
{
    /**
     * This component is responsible for managing one canOpen bus. It interfaces with CanFestival
     * via wrappers defined in the wrappers sub-directory.
     *
     * The choice went in favor of CanFestival because it is a high level driver. It interfaces many can drivers
     * underneath.
     */
    class CanOpenController: public HmlTaskContext
    {
    public:
        /**
         * Constructeur
         * @param name : name of the instanciated component
         */
        CanOpenController(const std::string& name);
        ~CanOpenController();

        /**
         * Initialize CanFestival and set the bus into Operationnal Mode
         */
        bool configureHook();

        /**
         *
         */
        bool startHook();

        /**
         * Overloaded to update the attrCurrentNMTState value
         */
        void updateHook();

        /**
         * Stop CanFestival related stuff.
         */
        void cleanupHook();

        /**
         * This operation allows anyone in the application to write in the local dictionnary
         * @param dicoEntry : a structure containing the information to write in the local dictionnary
         * @return : true when the read succeed
         */
        bool coWriteInLocalDico(CanDicoEntry dicoEntry);

        /**
         * This operation allows anyone in the application to read the value of the local dictionnary entry
         * @param dicoEntry : a structure containing the information to read from the local dictionnary.
         * @return : true when the read succeed, the return value is in dicoEntry.value
         */
        bool coReadInLocalDico(CanDicoEntry& dicoEntry);

        /**
         * This operation allows anyone in the application to write in a remote dictionnary
         * @param sdo : a structure containing the information to write in a remote dictionnary
         *  this is done via sdo
         * @return : true when the sdo has been sent and a acklowledge of the distant node has been
         *  received.
         */
        bool coWriteInRemoteDico(CanDicoEntry dicoEntry);

        /**
         * This operation allows anyone in the application to read the value of a remote dictionnary entry
         * @param sdo : a structure containing the information to read from the distant node.
         * @return : true when the sdo has been send, and a response has arrived from the distant node.
         * The result value is written in the dicoEntry.value entry.
         */
        bool coReadInRemoteDico(CanDicoEntry dicoEntry, int* receivedData);

        /**
         * This operation allows to reset all the SDO emission lines.
         * It should be use with care as it is a very intrusive behavior.
         */
        void ooResetSdoBuffers();

        /**
         * Use this operation to send a PDO.
         * If you are under xenomai you must run in primary mode
         * @param pdoNumber : number of the PDO on the CanFestival table (!! it is *NOT* the COBID !!)
         */
        bool coSendPdo(int pdoNumber);

        /**
         * Use this operation to emit an NMT state change request for a node.
         * This operation will do some polling on the NMT state with PDO request
         * So don't use this operation in operationnal ! (only for booting and configuring).
         * It's a blocking function.
         * It will send 2 things :
         * _ the NMT cmd request with OOO#cmd.nodeId
         * _ an NMT state reques 700+nodeId#R
         * and wait for the 700+nodeId#nmtState message to come.
         * @param nodeId : node ID of the slave node to which we send the request
         * @param nmtStateCmd : new NMT state in which we would like the node to be
         * @param timeout : timeout on the polling
         */
        bool coSendNmtCmd(nodeID_t nodeId, enum_DS301_nmtStateRequest nmtStateCmd, double timeout);

        /**
         * define a new period for SYNC object
         * param : periode in s.
         */
        bool ooSetSyncPeriod(double period);

    protected:
        /** This attribute contains the current NMT status of the Controller node */
        e_nodeState attrCurrentNMTState;
        /** Last sync time received **/
        timespec attrSyncTime;
        /** This is for test purposes only, when sending request to the can via the taskBrowser */
        CanDicoEntry attrTestingSdo;

        /** This property contains the name of the can driver library that will be loaded dynamically */
        std::string propCanFestivalDriverName;
        /** This property contains the name of the bus attached to the CanController */
        std::string propBusName;
        /** This propertu contains the baudrate of the attached bus (10K,250K,500K,1000K, ...)*/
        std::string propBaudRate;
        /** This property contains the nodeID of the Controller node on the attached bus (in decimal)*/
        int propNodeId;
        /** This property defines the maximal allowed duration of slaves nodes before considering they are not on the bus (in s)*/
        double propMasterMaxBootDelay;
        /** Delay between 2 SYNC messgaes in s */
        double propSyncPeriod;
        /** Delay max beetween the sync order and the time we consider all PDO to be received*/
        double propPdoMaxAwaitedDelay;

        /**
         * This port is connected to the CanFestival thread to populate attrCurrentNMTState
         */
        InputPort<e_nodeState> inControllerNmtState;
        /**
         * This port is connected to the CanFestival thread to dispatch the boot event to registred Device Components
         */
        InputPort<nodeID_t> inBootUpReceived;
        /**
         * In Sync. Contains the date of the sync object
         */
        InputPort<timespec> inSync;

        /**
         * clock port, each node must listed this port to execute
         * It contains the SYNC CAN message date
         */
        OutputPort<timespec> outClock;

        /** Delay beetween inSync and last cycle inSync in s*/
        OutputPort<double> outPeriod;


        /**
         * The routing stuff is delegated to this class
         * */
        CanOpenDispatcher m_dispatcher;

        /**
         * Call this to initialize all the CanFestival related stuff (shared datas, wrappers, timers loop, loading drivers,...)
         * @return true if initialization succeed;
         */
        bool initializeCanFestival();

        /**
         * Initialiaze datas and callbacks shared with CanFestival
         * @return true if initialization succeed
         */
        bool initialiazeCanFestivalDatas();

        /**
         * Initialiaze wrappers that interfaces CanFestival with Orocos
         * @return true if initialization succeed
         */
        bool initialiazeCanFestivalWrappers();

        /**
         * Open the propBusName bus from the CAN driver.
         */
        bool openCanBus();

        /**
         * Compares an NMT state to a sended NMT command.
         * The NMT state is get from CanARD_Data.NMTable[nodeId]
         * @param nmtCmd : the NMT command sended to the slave node
         * @param nodeId : Id of the node to whoch the command has been sended
         * @return true is the NMT cmd has been processed
         */
        bool isNmtStateChangeDone(enum_DS301_nmtStateRequest nmtCmd, nodeID_t nodeId);

        /**
         * Is derivated to disable the autocheck
         */
        virtual bool checkInputsPorts();

    private:
        /** This is a temporary buffer to pass the propBusName string to the CanFestival process */
        char m_busNameLocalCopy[100];
        /** This is a temporary buffer to pass the propBaudRate string to the CanFestival process */
        char m_baurateLocalCopy[6];
        /** This is the handler on the attached can bus. It is populated by a call to the CanFestival "canOpen" function */
        CAN_PORT m_canPort;

        /** last Sync time to compute period */
        timespec m_lastSyncTime;
    };

}

#endif /* CANOPEN_CONTROLLER_HPP_ */
