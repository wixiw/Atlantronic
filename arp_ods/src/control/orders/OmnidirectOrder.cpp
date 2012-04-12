/*
 * OmnidirectOrder.cpp
 *
 *  Created on: 10 april 2012
 *      Author: RMO
 */

#include "OmnidirectOrder.hpp"
#include "control/orders/Logger.hpp"
using namespace arp_math;
using namespace arp_ods;
using namespace orders;
using namespace arp_core::log;

OmnidirectOrder::OmnidirectOrder() :
    MotionOrder()
{
    m_type = OMNIDIRECT;
    m_v_correction_old=Twist2D(0,0,0);
    DECLIN=0;
    DECROT=0;
    VMAXLIN=0;
    VMAXROT=0;

}

OmnidirectOrder::OmnidirectOrder(MotionOrder order) :
    MotionOrder(order)
{
}

void OmnidirectOrder::setDefaults(orders::config conf)
{
    MotionOrder::setDefaults(conf);

    //TODO RMO rapatrier la conf qui est dans MODESELECTOR
    DECLIN=1.0;//en m/s2
    DECROT=2.0; //en rad/s2
    VMAXLIN=0.5; //en m/s
    VMAXROT=1; // en rad/s
}

shared_ptr<MotionOrder> OmnidirectOrder::createOrder( const OrderGoalConstPtr &goal, Pose2D currentPose, orders::config conf  )
{
    shared_ptr<OmnidirectOrder> order(new OmnidirectOrder());

    Pose2D begin;
    Pose2D end;
    Pose2D cpoint;

    begin.x(currentPose.x());
    begin.y(currentPose.y());
    begin.h(currentPose.h());
    order->setBeginPose(begin);

    cpoint.x(goal->x_cpoint);
    cpoint.y(goal->y_cpoint);
    cpoint.h(goal->theta_cpoint);
    order->setCpoint(end);

    end.x(goal->x_des);
    end.y(goal->y_des);
    end.h(goal->theta_des);
    order->setEndPose(end);

    order->setReverse(goal->reverse);
    order->setPass(goal->passe);

    order->setDefaults(conf);

    return static_cast<shared_ptr<MotionOrder>  >(order);
}

void OmnidirectOrder::switchRun(arp_math::Pose2D currentPosition)
{
    //dans le cas du mode PASS c'est le radius approche qui fait office de critère
    if (getRemainingDistance(currentPosition) <= getRadiusApproachZone())
    {
        if (getPass())
        {
            Log(INFO) << "switched MODE_RUN --> MODE_PASS";
            m_currentMode = MODE_PASS;
            m_passTime = getTime();
            return;
        }
    }

    double distance_error = getRemainingDistance(currentPosition);
    double angle_error = getRemainingAngle(currentPosition);

    if (distance_error < m_distanceAccuracy && fabs(angle_error) < m_angleAccuracy)
    {
        Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_DONE";
        char string [250];
        sprintf(string,"(%.3fm,%.3fm,%.1fdeg) with e_d=%.1fmm e_cap=%.1fdeg", currentPosition.x(), currentPosition.y(),
                        rad2deg(currentPosition.h()), distance_error * 1000, rad2deg(angle_error));
        Log(INFO) << string << " endPoint=" << m_endPose.toString();
        m_currentMode = MODE_DONE;
        return;
    }
    testTimeout();

}



Twist2D OmnidirectOrder::computeSpeed(arp_math::Pose2D currentPosition, double dt)
{
    Rotation2 orient_robot(currentPosition.h());

    /* NE FONCTIONNE PAS. LE CALCUL EST SUREMENT FAUX
    //position of the control point on the table
    Pose2D current_cpoint_position;
    current_cpoint_position.translation( currentPosition.translation() + orient_robot.toRotationMatrix()*m_cpoint.translation());
    current_cpoint_position.orientation( betweenMinusPiAndPlusPi(currentPosition.h()+m_cpoint.h()) );
     */

    //position error

    //Pose2D deltaPos_refTable=m_endPose-current_cpoint_position;
    Pose2D deltaPos_refTable=m_endPose-currentPosition;

    //position error in robot referential
    Pose2D deltaPos_refRobot;
    deltaPos_refRobot.translation( orient_robot.inverse().toRotationMatrix() * deltaPos_refTable.translation());
    deltaPos_refRobot.orientation( betweenMinusPiAndPlusPi(deltaPos_refTable.h()) );

    // brutal correction twist. with constant acceleration   v = sqrt ( 2 . acc) . sqrt( d )
    Twist2D v_correction;
    double speedcorrection=sqrt2(2*DECLIN)*sqrt2(deltaPos_refRobot.vectNorm());
    v_correction.vx(speedcorrection*std::cos(deltaPos_refRobot.vectAngle()));
    v_correction.vy(speedcorrection*std::sin(deltaPos_refRobot.vectAngle()));
    v_correction.vh(sqrt2(2*DECROT)*sqrt2(deltaPos_refRobot.h()));

    //saturation of twist: limit max linear speed/acc;  and max rotation speed/acc
    Twist2D v_correction_saturated;
    double vmaxlin=std::min(VMAXLIN,m_v_correction_old.speedNorm()+dt*DECLIN);
    double vmaxrot=std::min(VMAXROT,std::fabs(m_v_correction_old.vh())+dt*DECROT);
    //double vmaxlin=std::min(VMAXLIN,100.0);
    //double vmaxrot=std::min(VMAXROT,100.0);
    double satvlin=v_correction.speedNorm()/vmaxlin;
    double satvrot=std::fabs(v_correction.vh())/vmaxrot;
    double sat=std::max(satvlin,std::max( satvrot,1.0));
    v_correction_saturated = v_correction * (1/sat);

    m_v_correction_old=v_correction_saturated;

    return v_correction_saturated;

}


