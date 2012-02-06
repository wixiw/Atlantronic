#include "SqlBridge.hpp"
#include <rtt/Component.hpp>
#include <ros/package.h>

using namespace std;
using namespace RTT;
using namespace arp_core;
using namespace arp_ihm;

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(arp_ihm::SqlBridge)

SqlBridge::SqlBridge(string const& name):
    ARDTaskContext(name, ros::package::getPath("arp_ihm")),
    propServerIp("localhost"),
    propSqlUser("ard_user"),
    propSqlPassword("robotik"),
    propBddName("ubiquity"),
    attrNbPorts(0)
{
    addProperty("propServerIp", propServerIp);
    addProperty("propSqlUser", propSqlUser);
    addProperty("propSqlPassword", propSqlPassword);
    addProperty("propBddName", propBddName);

    addAttribute("attrNbPorts", attrNbPorts);

    addOperation("ooRegisterStringPort", &SqlBridge::ooRegisterStringPort, this, OwnThread)
            .doc("Register a port to be save in Mysql")
            .arg("componentName", "The name of the component containing the port to register")
            .arg("portName", "The name of the port in the component componentName");

    addOperation("ooRegisterDoublePort", &SqlBridge::ooRegisterDoublePort, this, OwnThread)
            .doc("Register a port to be save in Mysql")
            .arg("componentName", "The name of the component containing the port to register")
            .arg("portName", "The name of the port in the component componentName");

    addOperation("ooRegisterBoolPort", &SqlBridge::ooRegisterBoolPort, this, OwnThread)
            .doc("Register a port to be save in Mysql")
            .arg("componentName", "The name of the component containing the port to register")
            .arg("portName", "The name of the port in the component componentName");

    addOperation("ooDisplayPortList", &SqlBridge::ooDisplayPortList, this, OwnThread)
            .doc("Display the list of port registered in MySql");
}

SqlBridge::~SqlBridge()
{
    for( vector<InputPort<string>* >::iterator port=m_stringPorts.begin() ; port!=m_stringPorts.end() ; port++ )
    {
        free((*port));
    }

    for( vector<InputPort<double>* >::iterator port=m_doublePorts.begin() ; port!=m_doublePorts.end() ; port++ )
    {
        free((*port));
    }

    for( vector<InputPort<bool>* >::iterator port=m_boolPorts.begin() ; port!=m_boolPorts.end() ; port++ )
    {
        free((*port));
    }
}

bool SqlBridge::configureHook()
{
    mysql_init(&mysql);
    connection = mysql_real_connect(&mysql,
            propServerIp.c_str(),propSqlUser.c_str(),propSqlPassword.c_str(),
            propBddName.c_str(),0,0,0);
    if (connection == NULL)
    {
        LOG(Error) << mysql_error(connection) << endlog();
        LOG(Error) << "when configuring BDD connection" << endlog();
        goto fail;
    }

    if( !sqlClean() )
    {
        goto fail;
    }

    //success
    goto success;

    fail:
        return false;
    success:
        return true;
}

bool SqlBridge::startHook()
{
    return true;
}

void SqlBridge::updateHook()
{
    for( vector<InputPort<string>* >::iterator it=m_stringPorts.begin() ; it!=m_stringPorts.end() ; it++ )
    {
        InputPort<string>* port = (*it);
        string s;
        port->read(s);
        sqlUpdate(port->getName(),s);
    }

    for( vector<InputPort<double>* >::iterator it=m_doublePorts.begin() ; it!=m_doublePorts.end() ; it++ )
    {
        InputPort<double>* port = (*it);
        double d;
        std::ostringstream oss;

        port->read(d);
        oss << d;
        sqlUpdate(port->getName(),oss.str());
    }

    for( vector<InputPort<bool>* >::iterator it=m_boolPorts.begin() ; it!=m_boolPorts.end() ; it++ )
    {
        InputPort<bool>* port = (*it);
        bool b;
        std::ostringstream oss;

        port->read(b);
        oss << b;
        sqlUpdate(port->getName(),oss.str());
    }
}

void SqlBridge::stopHook()
{
}

void SqlBridge::cleanupHook()
{
    sqlClean();
}

bool SqlBridge::sqlUpdate(const string portName, const string value)
{
    bool res = true;
    int query_state;

    std::string request = "UPDATE  `ubiquity`.`orocos_message_to_hmi` SET  `value` =  '";
    request.append(value);
    request.append("' WHERE  `orocos_message_to_hmi`.`port_name` =  '");
    request.append(portName);
    request.append("'");

    query_state = mysql_query(connection, request.c_str());

    if (query_state !=0)
    {
        LOG(Error) << mysql_error(connection) << endlog();
        LOG(Error) << "when using request :   " << request.c_str() << endlog();
        res = false;
    }

    return res;
}

bool SqlBridge::sqlInsert(const string portName)
{
    bool res = true;
    int query_state;

    std::string request = "INSERT INTO `ubiquity`.`orocos_message_to_hmi` (`port_name`, `value`) VALUES ('";
    request.append(portName);
    request.append("', '");
    request.append("default");
    request.append("')");

    query_state = mysql_query(connection, request.c_str());

    if (query_state !=0)
    {
        LOG(Error) << mysql_error(connection) << endlog();
        LOG(Error) << "when using request :   " << request.c_str() << endlog();
        res = false;
    }

    return res;
}


bool SqlBridge::sqlClean()
{
    bool res = true;
    int query_state;

    std::string request = "TRUNCATE TABLE  `orocos_message_to_hmi`";
    query_state = mysql_query(connection, request.c_str());

    if (query_state !=0)
    {
        LOG(Error) << mysql_error(connection) << endlog();
        LOG(Error) << "when using request :   " << request.c_str() << endlog();
        res = false;
    }

    return res;
}

//---------------------------------------------------------------

bool SqlBridge::ooRegisterStringPort(const string comp, const string portName)
{
    string portExtendedName = comp + "." + portName;
    TaskContext* taskContext = getPeer(comp);

    if ( taskContext == NULL )
    {
        LOG(Error)  << "ooRegisterStringPort : failed to find Component " << comp << endlog();
        goto fail;
    }
    else
    {
        base::PortInterface * portToRegister = taskContext->getPort(portName);

        if( portToRegister == NULL )
        {
            LOG(Error)  << "getOperation : failed to find port " << portExtendedName << endlog();
            goto fail;
        }
        else
        {
            //port creation
            InputPort<string>* portCreated = new InputPort<string>(portExtendedName);

            //port connection
            if( !portCreated->connectTo(portToRegister))
            {
                LOG(Error) << "ooRegisterStringPort : failed to connect to " << portExtendedName << endlog();
                goto fail;
            }

            //register into BDD
            if( !sqlInsert(portExtendedName) )
            {
                LOG(Error)  << "ooRegisterStringPort : failed to connect to " << portExtendedName << endlog();
                goto fail;
            }

            //add to the orocos interface (option)
            addPort(portExtendedName, *portCreated);

            //insert in the local list
            m_stringPorts.push_back(portCreated);

            //increment local port counter for supervision
            attrNbPorts++;

            goto success;
        }
    }

    fail :
        return false;
    success:
        return true;
}


bool SqlBridge::ooRegisterDoublePort(const string comp, const string portName)
{
    string portExtendedName = comp + "." + portName;
    TaskContext* taskContext = getPeer(comp);

     if ( taskContext == NULL )
     {
         LOG(Error)  << "ooRegisterDoublePort : failed to find Component " << comp << endlog();
         goto fail;
     }
     else
     {
         base::PortInterface * portToRegister = taskContext->getPort(portName);

         if( portToRegister == NULL )
         {
             LOG(Error)  << "ooRegisterDoublePort : failed to find port " << portExtendedName << endlog();
             goto fail;
         }
         else
         {
             //port creation
             InputPort<double>* portCreated = new InputPort<double>(portExtendedName);

             //port connection
             if( !portCreated->connectTo(portToRegister))
             {
                 LOG(Error) << "ooRegisterDoublePort : failed to connect to " << portExtendedName << endlog();
                 goto fail;
             }

             //register into BDD
             if( !sqlInsert(portExtendedName) )
             {
                 LOG(Error)  << "ooRegisterDoublePort : failed to connect to " << portExtendedName << endlog();
                 goto fail;
             }

             //add to the orocos interface (option)
             addPort(portExtendedName, *portCreated);

             //insert in the local list
             m_doublePorts.push_back(portCreated);

             //increment local port counter for supervision
             attrNbPorts++;

             goto success;
         }
     }

     fail :
         return false;
     success:
         return true;
}

bool SqlBridge::ooRegisterBoolPort(const string comp, const string portName)
{
    string portExtendedName = comp + "." + portName;
    TaskContext* taskContext = getPeer(comp);

     if ( taskContext == NULL )
     {
         LOG(Error)  << "ooRegisterBoolPort : failed to find Component " << comp << endlog();
         goto fail;
     }
     else
     {
         base::PortInterface * portToRegister = taskContext->getPort(portName);

         if( portToRegister == NULL )
         {
             LOG(Error)  << "ooRegisterBoolPort : failed to find port " << portExtendedName << endlog();
             goto fail;
         }
         else
         {
             //port creation
             InputPort<bool>* portCreated = new InputPort<bool>(portExtendedName);

             //port connection
             if( !portCreated->connectTo(portToRegister))
             {
                 LOG(Error) << "ooRegisterBoolPort : failed to connect to " << portExtendedName << endlog();
                 goto fail;
             }

             //register into BDD
             if( !sqlInsert(portExtendedName) )
             {
                 LOG(Error)  << "ooRegisterBoolPort : failed to connect to " << portExtendedName << endlog();
                 goto fail;
             }

             //add to the orocos interface (option)
             addPort(portExtendedName, *portCreated);

             //insert in the local list
             m_boolPorts.push_back(portCreated);

             //increment local port counter for supervision
             attrNbPorts++;

             goto success;
         }
     }

     fail :
         return false;
     success:
         return true;
}

void SqlBridge::ooDisplayPortList()
{
    cout << endl ;
    cout << "-------------------------" << endl;
    cout << "-- string : " << endl;
    cout << endl ;

    for( vector<InputPort<string>* >::iterator it=m_stringPorts.begin() ; it!=m_stringPorts.end() ; it++ )
    {
        base::PortInterface * port = (*it);
        cout << "-- " << port->getName () << endl;
    }

    cout << endl ;
    cout << "-------------------------" << endl;
    cout << "-- double : " << endl;
    cout << endl ;

    for( vector<InputPort<double>* >::iterator it=m_doublePorts.begin() ; it!=m_doublePorts.end() ; it++ )
    {
        base::PortInterface * port = (*it);
        cout << "-- " << port->getName () << endl;
    }

    cout << endl ;
    cout << "-------------------------" << endl;
    cout << "-- bool : " << endl;
    cout << endl ;

    for( vector<InputPort<bool>* >::iterator it=m_boolPorts.begin() ; it!=m_boolPorts.end() ; it++ )
    {
        base::PortInterface * port = (*it);
        cout << "-- " << port->getName () << endl;
    }

    cout << endl ;
    cout << "-------------------------" << endl;
    cout << endl ;
}
