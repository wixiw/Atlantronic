/*
 * GamepadPS1.cpp
 *
 *  Created on: 29 sept. 2010
 *      Author: ard
 */

#include "GamepadPS1.hpp"
#include <rtt/Component.hpp>
#include <linux/input.h>
#include <sys/ioctl.h>

using namespace arp_hml;

//alors qu'il faut mettre autant de ORO_LIST_COMPONENT_TYPE qu'il y a de composants
ORO_LIST_COMPONENT_TYPE( arp_hml::GamepadPS1 )


/** Constructeur de la classe GamepadPS1
*  @param name nom du composant Orocos
*/
GamepadPS1::GamepadPS1(const std::string& name) :
Joystick(name),
propEventName("/dev/input/event0")
{
    addProperty("propEventName",propEventName).doc("linux /dev file which represents the joystick input");

    addPort("inRumble", inRumble).doc("Makes the joypad rumbking when true");

    addPort("outButton1", outButton1).doc("Is true when rigth thumb bouton 1 is pushed");
    addPort("outButton2", outButton2).doc("Is true when rigth thumb bouton 2 is pushed");
    addPort("outButton3", outButton3).doc("Is true when rigth thumb bouton 3 is pushed");
    addPort("outButton4", outButton4).doc("Is true when rigth thumb bouton 4 is pushed");

    addPort("outTrigger5", outTrigger5).doc("Is true when high left trigger 5 is pushed");
    addPort("outTrigger6", outTrigger6).doc("Is true when high rigth trigger 6 is pushed");
    addPort("outTrigger7", outTrigger7).doc("Is true when high left trigger 7 is pushed");
    addPort("outTrigger8", outTrigger8).doc("Is true when high rigth trigger 8 is pushed");

    addPort("outButton9", outButton9).doc("Is true when \"select\" bouton 9 is pushed");
    addPort("outButton10", outButton10).doc("Is true when \"start\" bouton 10 is pushed");

    addPort("outPadX",outPadX).doc("Value between [-1;1] proportionnal to X-axis direcitonnal pad");
    addPort("outPadY",outPadY).doc("Value between [-1;1] proportionnal to Y-axis direcitonnal pad");

    addPort("outPadXYDistance",outPadXYDistance).doc("Distance of joystick from neutral position in [0;1.42]");
    addPort("outPadXYAngle",outPadXYAngle).doc("Angle of joystick from X axis, positive to y in rad");

    addPort("outX1",outX1).doc("");
    addPort("outY1",outY1).doc("");
    addPort("outX2",outX2).doc("");
    addPort("outY2",outY2).doc("");

    outButton1.write(false);
    outButton2.write(false);
    outButton3.write(false);
    outButton4.write(false);
    outTrigger5.write(false);
    outTrigger6.write(false);
    outTrigger7.write(false);
    outTrigger8.write(false);
    outButton9.write(false);
    outButton10.write(false);
    outPadX.write(0.0);
    outPadY.write(0.0);
    outPadXYDistance.write(0.0);
    outPadXYAngle.write(0.0);
    outX1.write(0.0);
    outY1.write(0.0);
    outX2.write(0.0);
    outY2.write(0.0);

    retry: //goto pour l'appel systeme specifique systeme, c'est comme ça que ça se fait

    if( (m_event_fd = open(propEventName.c_str(), O_WRONLY)) < 0 )
        cerr << "fail to opend propEventName" << endl;

    //gestion du eagain important pour le debugger et xenomai
    if( errno == EAGAIN )
        goto retry;

}


//verifie que le joystick connecte est bien le bon
bool GamepadPS1::checkIdentity()
{
    bool res = false;

    if( getJoystickName() == "Logitech Logitech(R) Precision(TM) Gamepad"
    		||
    	getJoystickName() == "Logitech Logitech Cordless RumblePad 2"	)
    {
        res = true;
    }
    else
    {
        res = false;
    }

    return res;
}

void GamepadPS1::buttonEvent( struct js_event js )
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
            outTrigger5.write(js.value);
            break;
        case 5:
            outTrigger6.write(js.value);
            break;
        case 6:
            outTrigger7.write(js.value);
            break;
        case 7:
            outTrigger8.write(js.value);
            break;
        case 8:
            outButton9.write(js.value);
            break;
        case 9:
            outButton10.write(js.value);
            break;
        default:
            LOG(Error) << "Unknown button" << js.number << endlog();
            break;
    }
    return;
}

void GamepadPS1::axisEvent( struct js_event js )
{
    switch (js.number)
    {
        case 4:
            outPadX.write( js.value>0 ? 1: js.value==0 ? 0 : -1);
            break;
        case 5:
            outPadY.write( js.value>0 ? -1: js.value==0 ? 0 : 1 );
            break;
        case 0:
            outX1.write( double(js.value)/(2<<14) );
            break;
        case 1:
            outY1.write( double(js.value)/(2<<14) );
            break;
        case 2:
            outX2.write( double(js.value)/(2<<14) );
            break;
        case 3:
            outY2.write( double(js.value)/(2<<14) );
            break;

        default:
            LOG(Error) << "Unknown joystick" << js.number << endlog();
            break;
    }
    return;
}

void GamepadPS1::initEvent( struct js_event js )
{
    //LOG(Info) << "Joystick InitEvent" << endlog();

    outButton1.write(false);
    outButton2.write(false);
    outButton3.write(false);
    outButton4.write(false);
    outTrigger5.write(false);
    outTrigger6.write(false);
    outTrigger7.write(false);
    outTrigger8.write(false);
    outButton9.write(false);
    outButton10.write(false);
    outPadX.write(0.0);
    outPadY.write(0.0);
    outPadXYDistance.write(0.0);
    outPadXYAngle.write(0.0);
    outX1.write(0.0);
    outY1.write(0.0);
    outX2.write(0.0);
    outY2.write(0.0);

    return;
}


void GamepadPS1::updateHook()
{
    Joystick::updateHook();

    if( isJoystickAlive() )
    {
		double lastX = outPadX.getLastWrittenValue();
		double lastY = outPadY.getLastWrittenValue();

		outPadXYDistance.write(sqrt(lastX*lastX + lastY*lastY));
		outPadXYAngle.write(atan2(lastY,lastX));
    }
    else
    {
    	js_event js;
    	initEvent(js);
    }

    bool rumble = false;
    inRumble.readNewest(rumble);
    if( true )
    {
        ff_envelope env;
        env.attack_length = 0;
        env.attack_level = 0;
        env.fade_length = 0;
        env.fade_level = 0;

        ff_constant_effect effect;
        effect.level = 0xFFFFFFFF;
        effect.envelope = env;
        ioctl(m_event_fd, EVIOCSFF, effect);
    }
}

/**
 * struct ff_effect - defines force feedback effect
 * @type: type of the effect (FF_CONSTANT, FF_PERIODIC, FF_RAMP, FF_SPRING,
 *  FF_FRICTION, FF_DAMPER, FF_RUMBLE, FF_INERTIA, or FF_CUSTOM)
 * @id: an unique id assigned to an effect
 * @direction: direction of the effect
 * @trigger: trigger conditions (struct ff_trigger)
 * @replay: scheduling of the effect (struct ff_replay)
 * @u: effect-specific structure (one of ff_constant_effect, ff_ramp_effect,
 *  ff_periodic_effect, ff_condition_effect, ff_rumble_effect) further
 *  defining effect parameters
 *
 * This structure is sent through ioctl from the application to the driver.
 * To create a new effect application should set its @id to -1; the kernel
 * will return assigned @id which can later be used to update or delete
 * this effect.
 *
 * Direction of the effect is encoded as follows:
 *  0 deg -> 0x0000 (down)
 *  90 deg -> 0x4000 (left)
 *  180 deg -> 0x8000 (up)
 *  270 deg -> 0xC000 (right)
 */
//struct ff_effect {
//    __u16 type;
//    __s16 id;
//    __u16 direction;
//    struct ff_trigger trigger;
//    struct ff_replay replay;
//
//    union {
//        struct ff_constant_effect constant;
//        struct ff_ramp_effect ramp;
//        struct ff_periodic_effect periodic;
//        struct ff_condition_effect condition[2]; /* One for each axis */
//        struct ff_rumble_effect rumble;
//    } u;
//};


bool GamepadPS1::checkInputsPorts()
{
    return true;
}
