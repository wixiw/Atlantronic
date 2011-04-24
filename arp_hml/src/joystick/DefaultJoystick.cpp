/*
 * DefaultJoystick.cpp
 *
 *  Created on: 29 sept. 2010
 *      Author: ard
 */

#include "joystick/DefaultJoystick.hpp"
#include <ocl/Component.hpp>

using namespace arp_hml;

ORO_LIST_COMPONENT_TYPE( arp_hml::DefaultJoystick )


/** Constructeur de la classe DefaultJoystick
*  @param name nom du composant Orocos
*/
DefaultJoystick::DefaultJoystick(const std::string& name) :
Joystick(name)
{

}

DefaultJoystick::~DefaultJoystick()
{

}

//verifie que le joystick connecte est bien le bon
bool DefaultJoystick::checkIdentity()
{
    //cout << "Current joystick name is : " << getJoystickName() << endl;
    return true;
}

void DefaultJoystick::buttonEvent( struct js_event js )
{
    printf("Button #%d value:%d\n",js.number, js.value);
    return;
}

void DefaultJoystick::axisEvent( struct js_event js )
{
    printf("Axis #%d value:%d\n",js.number, js.value);
    return;
}

void DefaultJoystick::initEvent( struct js_event js )
{
    printf("Joystick in initial state\n");
    return;
}
