/*
 * OmnidirectOrder.cpp
 *
 *  Created on: 10 april 2012
 *      Author: RMO
 */

#include "OmnidirectOrder.hpp"

using namespace arp_math;
using namespace arp_ods;
using namespace orders;

OmnidirectOrder::OmnidirectOrder() :
    MotionOrder()
{
    m_type = OMNIDIRECT;
    DECLIN=0;
    DECROT=0;

}

OmnidirectOrder::OmnidirectOrder(MotionOrder order) :
    MotionOrder(order)
{
}

void OmnidirectOrder::setDefaults(orders::config conf)
{
    MotionOrder::setDefaults(conf);

    //TODO RMO rapatrier la conf
    DECLIN=1.0;//en m/s2
    DECROT=1.0; //en rad/s2
    VMAXLIN=1.0; //en m/s
    VMAXROT=1.0; // en m/s2
}

shared_ptr<MotionOrder> OmnidirectOrder::createOrder( const OrderGoalConstPtr &goal, Pose2D currentPose, orders::config conf  )
{
    shared_ptr<OmnidirectOrder> order(new OmnidirectOrder());

    Pose2D begin;
    Pose2D end;

    begin.x(currentPose.x());
    begin.y(currentPose.y());
    begin.h(currentPose.h());
    order->setBeginPose(begin);

    end.x(goal->x_des);
    end.y(goal->y_des);
    end.h(goal->theta_des);
    order->setEndPose(end);

    order->setReverse(goal->reverse);
    order->setPass(goal->passe);

    order->setDefaults(conf);

    return static_cast<shared_ptr<MotionOrder>  >(order);
}



Twist2D OmnidirectOrder::computeSpeed(arp_math::Pose2D currentPosition, double dt)
{
    //position error
    Pose2D deltaPos_refTable=m_endPose-currentPosition;

    //position error in robot referential
    Rotation2 orient_robot(currentPosition.h());
    currentPosition.getDisplacement2Matrix();
    Pose2D deltaPos_refRobot;
    deltaPos_refRobot.translation( orient_robot.inverse().toRotationMatrix() * deltaPos_refTable.translation());
    deltaPos_refRobot.orientation(0);

    // brutal correction twist
    Twist2D v_correction;
    v_correction.vx(sqrt2(2*DECLIN)*sqrt2(deltaPos_refRobot.x()));
    v_correction.vy(sqrt2(2*DECLIN)*sqrt2(deltaPos_refRobot.y()));
    v_correction.vh(sqrt2(2*DECROT)*sqrt2(deltaPos_refRobot.h()));

    //saturation of twist: limit max linear speed and max rotation speed
    Twist2D v_correction_saturated;
    //satlin=
    //satrot=
    //sat=max(satlin, satrot,1)
    //v_correction_saturated = v_correction * sat

}


