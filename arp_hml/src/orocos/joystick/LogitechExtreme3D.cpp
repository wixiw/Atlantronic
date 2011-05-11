/*
 * LogitechExtreme3D.cpp
 *
 *  Created on: 29 sept. 2010
 *      Author: ard
 */

#include "LogitechExtreme3D.hpp"
#include <ocl/Component.hpp>

using namespace arp_hml;

ORO_LIST_COMPONENT_TYPE( arp_hml::LogitechExtreme3D )


/** Constructeur de la classe LogitechExtreme3D
*  @param name nom du composant Orocos
*/
LogitechExtreme3D::LogitechExtreme3D(const std::string& name) :
Joystick(name)
{
    addPort("outButton1", outButton1).doc("Is true when trigger index bouton 1 is pushed");
    addPort("outButton2", outButton2).doc("Is true when triger thumb bouton 2 is pushed");
    addPort("outButton3", outButton3).doc("Is true when top thumb bouton 3 is pushed");
    addPort("outButton4", outButton4).doc("Is true when top thumb bouton 4 is pushed");
    addPort("outButton5", outButton5).doc("Is true when top thumb bouton 5 is pushed");
    addPort("outButton6", outButton6).doc("Is true when top thumb bouton 6 is pushed");

    addPort("outButton7", outButton7).doc("Is true when base button 7 is pushed");
    addPort("outButton8", outButton8).doc("Is true when base button 8 is pushed");
    addPort("outButton9", outButton9).doc("Is true when base button 9 is pushed");
    addPort("outButton10", outButton10).doc("Is true when base button 10 is pushed");
    addPort("outButton11", outButton11).doc("Is true when base button 11 is pushed");
    addPort("outButton12", outButton12).doc("Is true when base button 12 is pushed");

    addPort("outX",outX).doc("Value between [-1;1] proportionnal to joystick left-rigth X-axis");
    addPort("outY",outY).doc("Value between [-1;1] proportionnal to joystick far-close Y-axis");
    addPort("outZ",outZ).doc("Value between [-1;1] proportionnal to joystick rotation clock-wise Z-axis");
    addPort("outTopX",outTopX).doc("Value between [-1;1] proportionnal to joystick top thumb pad left-rigth X-axis");
    addPort("outTopY",outTopY).doc("Value between [-1;1] proportionnal to joystick top thumb pad far-close Y-axis");
    addPort("outThrottle",outThrottle).doc("Value between [-1;1] proportionnal to joystick far-close throttle axis");

    addPort("outXYDistance",outXYDistance).doc("Distance of joystick from neutral position in [0;1.42]");
    addPort("outXYAngle",outXYAngle).doc("Angle of joystick from X axis, positive to y in rad");
    addPort("outTopXYDistance",outTopXYDistance).doc("Distance of thumb pad from neutral position in [0;1.42]");
    addPort("outTopXYAngle",outTopXYAngle).doc("Angle of thumb pad from X axis, positive to y in rad");


    outButton1.write(false);
    outButton2.write(false);
    outButton3.write(false);
    outButton4.write(false);
    outButton5.write(false);
    outButton6.write(false);
    outButton7.write(false);
    outButton8.write(false);
    outButton9.write(false);
    outButton10.write(false);
    outButton11.write(false);
    outButton12.write(false);
    outX.write(0.0);
    outY.write(0.0);
    outZ.write(0.0);
    outTopX.write(0.0);
    outTopY.write(0.0);
    outThrottle.write(0.0);
    outXYDistance.write(0.0);
    outXYAngle.write(0.0);
    outTopXYDistance.write(0.0);
    outTopXYAngle.write(0.0);
}


//verifie que le joystick connecte est bien le bon
bool LogitechExtreme3D::checkIdentity()
{
    bool res = false;

    if( getJoystickName() != "Logitech Logitech Extreme 3D")
    {
        res = false;
    }
    else
    {
        res = true;
    }

    return res;
}

void LogitechExtreme3D::buttonEvent( struct js_event js )
{
    switch (js.number)
    {
        case 0:
            outButton1.write(js.value);
            cerr << "bite" << endl;
            break;
        case 1:
            outButton2.write(js.value);
            break;
        case 2:
            outButton3.write(js.value);
            break;
        case 3:
            outButton4.write(js.value);
            break;
        case 4:
            outButton5.write(js.value);
            break;
        case 5:
            outButton6.write(js.value);
            break;
        case 6:
            outButton7.write(js.value);
            break;
        case 7:
            outButton8.write(js.value);
            break;
        case 8:
            outButton9.write(js.value);
            break;
        case 9:
            outButton10.write(js.value);
            break;
        case 10:
            outButton11.write(js.value);
            break;
        case 11:
            outButton12.write(js.value);
            break;
        default:
            LOG(Error) << "Unknown button " << (int) js.number << " value " << (int) js.value << endlog();
            break;
    }
    return;
}

void LogitechExtreme3D::axisEvent( struct js_event js )
{
    switch (js.number)
    {
        case 0:
            outX.write( double(js.value)/(2<<14) );
            break;
        case 1:
            outY.write( double(js.value)/(2<<14) );
            break;
        case 2:
            outZ.write( double(js.value)/(2<<14) );
            break;
        case 3:
            outZ.write( double(js.value)/(2<<14) );
            break;
        case 4:
            outTopX.write( double(js.value)/(2<<14) );
            break;
        case 5:
            outTopY.write( double(js.value)/(2<<14) );
            break;

        default:
            LOG(Error) << "Unknown joystick " << (int) js.number <<  " value " << (int) js.value << endlog();
            break;
    }
    return;
}

void LogitechExtreme3D::initEvent( struct js_event js )
{
    LOG(Info) << "Joystick InitEvent" << endlog();

    outButton1.write(false);
    outButton2.write(false);
    outButton3.write(false);
    outButton4.write(false);
    outButton5.write(false);
    outButton6.write(false);
    outButton7.write(false);
    outButton8.write(false);
    outButton9.write(false);
    outButton10.write(false);
    outButton11.write(false);
    outButton12.write(false);
//    outX.write(0.0);
//    outY.write(0.0);
//    outZ.write(0.0);
//    outTopX.write(0.0);
//    outTopY.write(0.0);
//    outThrottle.write(0.0);
    outXYDistance.write(0.0);
    outXYAngle.write(0.0);
    outTopXYDistance.write(0.0);
    outTopXYAngle.write(0.0);

    return;
}


void LogitechExtreme3D::updateHook()
{
    Joystick::updateHook();

    double lastX = outX.getLastWrittenValue();
    double lastY = outY.getLastWrittenValue();
    double lastTopX = outTopX.getLastWrittenValue();
    double lastTopY = outTopY.getLastWrittenValue();

    outXYDistance.write(sqrt(lastX*lastX + lastY*lastY));
    outXYAngle.write(atan2(lastY,lastX));

    outTopXYDistance.write(sqrt(lastTopX*lastTopX + lastTopY*lastTopY));
    outTopXYAngle.write(atan2(lastTopY,lastTopX));
}

