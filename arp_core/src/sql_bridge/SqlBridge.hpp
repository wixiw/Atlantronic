#ifndef OROCOS_SQL_BRIDGE_COMPONENT_HPP
#define OROCOS_SQL_BRIDGE_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <iostream>
#include <mysql/mysql.h>

namespace arp_core
{
    class SqlBridge  : public RTT::TaskContext
    {
     public:
        SqlBridge(string const& name);

        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();
        void cleanupHook();

     protected:
        MYSQL mysql;
        MYSQL* connection;
    };
}

#endif
