/*
 * Logitech3DTeleop.cpp
 *
 *  Created on: 01 may 2011
 *      Author: wla
 */


#include "Logitech3DTeleop.hpp"
#include <ocl/Component.hpp>
#include <susi.h>
#include <ros/package.h>

using namespace arp_ods;
using namespace arp_core;


ORO_LIST_COMPONENT_TYPE( arp_ods::Logitech3DTeleop )

Logitech3DTeleop::Logitech3DTeleop(const std::string& name) :
    ARDTaskContext(name,ros::package::getPath("arp_ods")),
    propLongGain(1),
    propRotGain(1)
{
    addPort("inY",inY).doc("Value between [-1;1] proportionnal to joystick far-close Y-axis");
    addPort("inZ",inZ).doc("Value between [-1;1] proportionnal to joystick rotation clock-wise Z-axis");
    addPort("inDeadMan",inDeadMan);
    addPort("outLeftSpeed",outLeftSpeed).doc("");
    addPort("outRightSpeed",outRightSpeed).doc("");

    addProperty("propLongGain",propLongGain);
    addProperty("propRotGain",propRotGain);
}

Logitech3DTeleop::~Logitech3DTeleop()
{
}

void Logitech3DTeleop::updateHook()
{
	ARDTaskContext::updateHook();

	double longSpeed;
	double rotSpeed;
	inY.readNewest(longSpeed);
	inZ.readNewest(rotSpeed);

	bool deadMan;
	inDeadMan.readNewest(deadMan);
	if( deadMan == true )
	{
		outLeftSpeed.write(propLongGain*longSpeed - propRotGain*rotSpeed);
		outRightSpeed.write(-propLongGain*longSpeed - propRotGain*rotSpeed);
	}
	else
	{
		outLeftSpeed.write(0);
		outRightSpeed.write(0);
	}
}
