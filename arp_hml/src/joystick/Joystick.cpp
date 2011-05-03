/*
 * Joystick.cpp
 *
 *  Created on: 29/09/10
 *      Author: wla
 */

#include "Joystick.hpp"
#include <iostream>

using namespace arp_hml;
using namespace arp_core;
using namespace RTT;

/** Constructeur de la classe Joystick
*  @param name nom du composant Orocos
*/
Joystick::Joystick(const std::string& name) :
        ARDTaskContext(name),
        propDevName("/dev/input/js1"),
        propMinimalDriverVersion(0x020000),
        attrIsConnected(false),
        attrIsIdentityOk(false),
        m_fd(-666)
{
    //TODO WLA : workaround en attendant de trouver dans quel dossier on est lancé dans ROS
    attrPropertyPath = "/opt/ros/ard/arp_hml/script/orocos/conf";
    attrScriptPath = "/opt/ros/ard/arp_hml/script/orocos/ops";
    attrStateMachinePath = "/opt/ros/ard/arp_hml/script/orocos/osd";

    addProperty("propDevName",propDevName).doc("linux /dev file which represents the joystick");
    addProperty("propMinimalDriverVersion",propMinimalDriverVersion).doc("minimal linux joystick driver");

    addOperation("ooGetJoystickName", &Joystick::getJoystickName, this, OwnThread );
    addOperation("ooIsJoystickConnected", &Joystick::isJoystickConnected, this, OwnThread );

    addAttribute("attrIsConnected" , attrIsConnected);
    addAttribute("attrIsIdentityOk" , attrIsIdentityOk);
    addAttribute("attrFileDescriptor", m_fd);
}

Joystick::~Joystick()
{

}

bool Joystick::configureHook()
{
    bool res = ARDTaskContext::configureHook();

    //vérification de la version du driver linux
    if( checkDriverVersion() )
    {
        LOG(Info) << "Joystick's driver ok."<< endlog();
        res &= true;
    }
    else
    {
        LOG(Error) << "Joystick's driver is outdated !"<< endlog();
        res &= false;
    }
    return res;
}

bool Joystick::startHook()
{
    bool res = ARDTaskContext::startHook();

    takeJoystick();

    return res;
}

void Joystick::updateHook()
{
    ARDTaskContext::updateHook();

    //on vérifie que le joystick est toujours connecté
    if( isJoystickConnected() && checkIdentity() )
    {
        attrIsConnected = true;
        attrIsIdentityOk = true;

        //s'il y a bien un joystick on peut venir lire les informations
        struct js_event js;
        while (read(m_fd, &js, sizeof(struct js_event)) == sizeof(struct js_event))
        {
            switch ( js.type )
            {
                case JS_EVENT_INIT:
                    initEvent(js);
                    break;
                case JS_EVENT_BUTTON:
                    buttonEvent(js);
                    break;
                case JS_EVENT_AXIS:
                    axisEvent(js);
                    break;
                default:
                    break;
            }
        }

        if( errno != EAGAIN )
        {
            cerr << "BITE ! " << errno << endl;
            close(m_fd);
        }

    }
    else
    {
        attrIsConnected = false;
        attrIsIdentityOk = false;
        releaseJoystick();
        error();
    }
}


void Joystick::errorHook()
{
    ARDTaskContext::errorHook();

    if( takeJoystick() )
    {
        if( attrIsConnected == false )
        {
            cout << "Joystick connected" << endl;
            attrIsConnected = true;
            attrIsIdentityOk = true;
        }

        if( checkIdentity() )
        {
            attrIsIdentityOk = true;
            cout << "Joystick identity checked" << endl;
            takeJoystick();
            recover();
        }
        else
        {
            if( attrIsIdentityOk == true )
            {
                cout << endl << "Current Joystick Name unexpected : " << getJoystickName() << endl;
                attrIsIdentityOk = false;
            }
        }
    }
    else
    {
        if( attrIsConnected == true )
        {
            cout << endl << "Joystick unconnected" << endl;
            attrIsIdentityOk = false;
            attrIsConnected = false;
        }
    }
}

void Joystick::stopHook()
{
    if( releaseJoystick()== false )
    {
        LOG(Error) << "Can't release Joystick" << endlog();
    }
}

//tente une connexion au fichier descripteur du joystick
//true si un joystick est détecté
//false si erreur
bool Joystick::takeJoystick()
{
    bool res = false;

    retry: //goto pour l'appel systeme specifique systeme, c'est comme ça que ça se fait

    if( (m_fd = open(propDevName.c_str(), O_RDONLY)) < 0 )
        res = false;
    else
    {
        fcntl(m_fd, F_SETFL, O_NONBLOCK);
        res = true;
    }

    //gestion du eagain important pour le debugger et xenomai
    if( errno == EAGAIN )
        goto retry;

    return res;
}

//tente une deconnexion du joystick
//true si ok
//false si KO
bool Joystick::releaseJoystick()
{
    bool res = false;

    if( close(m_fd) != 0 )
    {
        res = false;
        m_fd = NULL;
    }
    else
        res = true;


    return res;
}


//test si le joystick est connecté
//true si connecté
//false sinon
bool Joystick::isJoystickConnected()
{
    bool res = false;

    FILE* f = fopen(propDevName.c_str(),"r");

    if( f==NULL )
        res = false;
    else
    {
        res = true;
        fclose(f);
    }

    return res;
}

//teste la version du driver linux
//renvoit true si ok
//false sinon
bool Joystick::checkDriverVersion()
{
    int res = false;

    if( JS_VERSION >= propMinimalDriverVersion )
        res = true;
    else
        res = false;

    return res;
}

//recupère le nom du joystick, donné par le hardware
string Joystick::getJoystickName()
{
    char name[128];
    memset(name, 0, 128 );

    if (ioctl(m_fd, JSIOCGNAME(sizeof(name)), name) < 0)
        strncpy(name, "Unknown", sizeof(name));

    string res(name);
    return res;

}

void Joystick::initEvent( struct js_event js )
{
    return;
}
