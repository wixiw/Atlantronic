#include "SqlBridge.hpp"
#include <rtt/Component.hpp>

using namespace RTT;
using namespace arp_core;

ORO_CREATE_COMPONENT(arp_core::SqlBridge)

using namespace RTT;
using namespace arp_core;

SqlBridge::SqlBridge(string const& name)
    : TaskContext(name)
{
    std::cout << "SqlBridge constructed !" <<std::endl;
}

bool SqlBridge::configureHook()
{
    mysql_init(&mysql);
    connection = mysql_real_connect(&mysql,"88.191.124.77","ard_user","robotik","ubiquity",0,0,0);
    if (connection == NULL)
    {
        std::cout << mysql_error(&mysql) << std::endl;
        return false;
    }
    else
    {
        return true;
    }


}

bool SqlBridge::startHook()
{
    std::cout << "SqlBridge started !" <<std::endl;
    return true;
}

void SqlBridge::updateHook()
{
    int query_state;
    std::string request = "UPDATE  `ubiquity`.`ros_message_to_hmi` SET  `value` =  '";
    request.append("SICK");
    request.append("' WHERE  `ros_message_to_hmi`.`topic_name` =  '/ArpHmi/health_status'");

    query_state = mysql_query(connection, request.c_str());

    if (query_state !=0)
    {
        std::cout << mysql_error(connection) << std::endl;
    }
}

void SqlBridge::stopHook()
{
    std::cout << "SqlBridge executes stopping !" <<std::endl;
}

void SqlBridge::cleanupHook()
{
    std::cout << "SqlBridge cleaning up !" <<std::endl;
}
