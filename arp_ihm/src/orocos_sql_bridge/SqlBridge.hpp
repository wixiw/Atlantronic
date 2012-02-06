#ifndef OROCOS_SQL_BRIDGE_COMPONENT_HPP
#define OROCOS_SQL_BRIDGE_COMPONENT_HPP

#include <taskcontexts/ARDTaskContext.hpp>
#include <mysql/mysql.h>
#include <rtt/base/PortInterface.hpp>

using namespace std;
using namespace RTT;
using namespace RTT::base;
using namespace arp_core;

namespace arp_ihm
{
    class SqlBridge  : public ARDTaskContext
    {
     public:
        SqlBridge(string const& name);
        virtual ~SqlBridge();

        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();
        void cleanupHook();

        bool ooRegisterStringPort(const string comp, const string portName);
        bool ooRegisterDoublePort(const string comp, const string portName);
        bool ooRegisterBoolPort(const string comp, const string portName);
        void ooDisplayPortList();

     protected:
        MYSQL mysql;
        MYSQL* connection;

        string propServerIp;
        string propSqlUser;
        string propSqlPassword;
        string propBddName;

        int attrNbPorts;

        vector<InputPort<string>* >  m_stringPorts;
        vector<InputPort<double>* >  m_doublePorts;
        vector<InputPort<bool>* >    m_boolPorts;

        /**
         * Use this to update a data that is already in orocos_message_to_hmi BDD
         * @param portName : name of the port to Update
         * @param value : string content of the port to update
         * @return true if the BDD update went OK.
         */
        bool sqlUpdate(const string portName, const string value);

        /**
         * Use this to insert a new entry in the  orocos_message_to_hmi BDD
         * Call this once before calling sqlUpdate
         * @param portName : name of the port to Update
         * @return true if the BDD insertion went OK.
         */
        bool sqlInsert(const string portName);

        /**
         * Use this when work is finished to clean the BDD
         * @return true if the SQL request went OK.
         */
        bool sqlClean();
    };
}

#endif
