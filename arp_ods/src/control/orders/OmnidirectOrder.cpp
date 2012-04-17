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


}

OmnidirectOrder::OmnidirectOrder(MotionOrder order) :
    MotionOrder(order)
{
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
    order->setCpoint(cpoint);

    end.x(goal->x_des);
    end.y(goal->y_des);
    end.h(goal->theta_des);
    order->setEndPose(end);

    order->setReverse(goal->reverse);
    order->setPass(goal->passe);

    order->setConf(conf);

    return static_cast<shared_ptr<MotionOrder>  >(order);
}

void OmnidirectOrder::switchRun(arp_math::Pose2D currentPosition)
{
    //dans le cas du mode PASS c'est le radius approche qui fait office de critère
    if (getRemainingDistance(currentPosition) <= m_conf.RADIUS_APPROACH_ZONE)
    {
        if (getPass())
        {
            Log(INFO) << "switched MODE_RUN --> MODE_PASS";
            m_currentMode = MODE_PASS;
            m_passTime = getTime();
            return;
        }
    }

    //position error in table referential
    Pose2D deltaPos_refTable=getPositionError(currentPosition);
    double distance_error = deltaPos_refTable.vectNorm();
    double angle_error = deltaPos_refTable.h();

    // test for DONE
    if (distance_error < m_conf.DISTANCE_ACCURACY && fabs(angle_error) < m_conf.ANGLE_ACCURACY)
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

Pose2D OmnidirectOrder::getPositionError(arp_math::Pose2D currentPosition)
{
    Rotation2 orient_robot(currentPosition.h());

    //position of the control point on the table
    Pose2D current_cpoint_position;
    current_cpoint_position.translation( currentPosition.translation() + orient_robot.toRotationMatrix()*m_cpoint.translation());
    current_cpoint_position.orientation( betweenMinusPiAndPlusPi(currentPosition.h()+m_cpoint.h()) );

    //position error
    Pose2D deltaPos_refTable=m_endPose-current_cpoint_position;

    return deltaPos_refTable;

}


Twist2D OmnidirectOrder::computeSpeed(arp_math::Pose2D currentPosition, double dt)
{
    //Rotation2 orient_robot(currentPosition.h());
    Rotation2 orient_cpoint(currentPosition.h()+m_cpoint.h());

    //position error in table referential
    Pose2D deltaPos_refTable=getPositionError(currentPosition);

    //position error in robot referential
    Pose2D deltaPos_refCpoint;
    deltaPos_refCpoint.translation( orient_cpoint.inverse().toRotationMatrix() * deltaPos_refTable.translation());
    deltaPos_refCpoint.orientation( betweenMinusPiAndPlusPi(deltaPos_refTable.h()) );

    // brutal correction twist. with constant acceleration   v = sqrt ( 2 . acc) . sqrt( d )
    Twist2D v_correction_cpoint;
    double speedcorrection=sqrt2(2*m_conf.LIN_DEC)*sqrt2(deltaPos_refCpoint.vectNorm());
    v_correction_cpoint.vx(speedcorrection*std::cos(deltaPos_refCpoint.vectAngle()));
    v_correction_cpoint.vy(speedcorrection*std::sin(deltaPos_refCpoint.vectAngle()));
    v_correction_cpoint.vh(sqrt2(2*m_conf.ANG_DEC)*sqrt2(deltaPos_refCpoint.h()));

    //transfer of the twist to robot referential
    Twist2D v_correction_ref;
    v_correction_ref=v_correction_cpoint.transport(m_cpoint.inverse());

    //saturation of twist: limit max linear speed/acc;  and max rotation speed/acc
    Twist2D v_correction_saturated;
    double vmaxlin=std::min(m_conf.LIN_VEL_MAX,m_v_correction_old.speedNorm()+dt*m_conf.LIN_DEC);
    double vmaxrot=std::min(m_conf.ANG_VEL_MAX,std::fabs(m_v_correction_old.vh())+dt*m_conf.ANG_DEC);
    double satvlin=v_correction_ref.speedNorm()/vmaxlin;
    double satvrot=std::fabs(v_correction_ref.vh())/vmaxrot;
    double sat=std::max(satvlin,std::max( satvrot,1.0));
    v_correction_saturated = v_correction_ref * (1/sat);

    m_v_correction_old=v_correction_saturated;


    return v_correction_saturated;

}


