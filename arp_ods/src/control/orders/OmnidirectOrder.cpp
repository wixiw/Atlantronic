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
using namespace boost;

OmnidirectOrder::OmnidirectOrder() :
        MotionOrder()
{
    m_type = OMNIDIRECT;
    m_v_correction_old = Twist2D(0, 0, 0);

}

OmnidirectOrder::OmnidirectOrder(MotionOrder order) :
        MotionOrder(order)
{
}

shared_ptr<MotionOrder> OmnidirectOrder::createOrder(const OrderGoalConstPtr &goal, Pose2D currentPose,
        orders::config conf)
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

    order->setPass(goal->passe);

    order->setConf(conf);

    return static_cast<shared_ptr<MotionOrder> >(order);
}

void OmnidirectOrder::switchRun(arp_math::Pose2D currentPosition)
{

//position error in table referential
    Pose2D deltaPos_refTable = getPositionError(currentPosition);
    double distance_error = deltaPos_refTable.vectNorm();

    // test for approach
    if (distance_error <= m_conf.RADIUS_APPROACH_ZONE)
    {
        Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_APPROACH " << "because in approach zone ";
        m_currentMode = MODE_APPROACH;
        m_approachTime = getTime();
        // si j'entre directement en approche, il faut que je calcule un twist initial

        //TODO on peut pas faire plus dégueu, comme j'ai pas dt je le force.
        m_twist_approach = computeRunTwist(currentPosition,0.02);

        m_error_approach=getPositionError_RobotRef(currentPosition);
        m_pose_approach=currentPosition;
        //to initialize the error
        computeApproachTwist(currentPosition,0.02);
        m_normalizedError_old=m_normalizedError+1.0;//+1 to be sure that the first comparison will be ok

        Log(DEBUG) << "m_twist_approach" << m_twist_approach.toString();
        Log(DEBUG) << "m_error_approach" << m_error_approach.toString();
        return;
    }

    testTimeout();


}

void OmnidirectOrder::switchApproach(arp_math::Pose2D currentPosition)
{
    Pose2D deltaPos_refTable = getPositionError(currentPosition);
    double distance_error = deltaPos_refTable.vectNorm();
    double angle_error = deltaPos_refTable.h();

    Pose2D current_cpoint_position = currentPosition * m_cpoint;

    /*if (distance_error > m_conf.RADIUS_APPROACH_ZONE) //&& fabs(angle_error) > m_conf.ANGLE_ACCURACY)
     {
     Log(INFO) << getTypeString() << " switched MODE_APPROACH --> MODE_RUN " << "because went out of final zone";
     m_currentMode = MODE_RUN;
     m_runTime = getTime();
     }*/

    if (m_normalizedError_old<=m_normalizedError)
    {
        Log(INFO) << getTypeString() << " switched MODE_APPROACH --> MODE_DONE "
                << "because error began to increase";
        Log(INFO) << "robot position  " << currentPosition.toString();
        Log(INFO) << "cpoint position " << current_cpoint_position.toString();
        Log(INFO) << "aim position    " << m_endPose.toString();
        Log(INFO) << "distance_error  " << distance_error;
        Log(INFO) << "angle_error     " << angle_error;

        m_currentMode = MODE_DONE;
        return;
    }
    m_normalizedError_old=m_normalizedError;

    testTimeout();

}

Pose2D OmnidirectOrder::getPositionError(arp_math::Pose2D currentPosition)
{
    // La version sans les mains des 4 lignes du dessus :
    Pose2D current_cpoint_position = currentPosition * m_cpoint;

    // Pourquoi ?
    // currentPosition est la pose du robot dans le repère de la table
    // m_cpoint est la pose du point de controle dans le repère du robot
    // pour avoir le point de controle dans le repère de la table, il suffit de composer (opérateur *) les deux poses

    //position error
    return MathFactory::createPose2D(m_endPose.translation() - current_cpoint_position.translation(),
            Rotation2(betweenMinusPiAndPlusPi(m_endPose.angle() - current_cpoint_position.angle())));
}

Pose2D OmnidirectOrder::getPositionError_RobotRef(arp_math::Pose2D currentPosition)
{
    Rotation2 orient_cpoint(currentPosition.h() + m_cpoint.h());
    //position error in table referential
    Pose2D deltaPos_refTable = getPositionError(currentPosition);
    //position error in robot referential
    Pose2D deltaPos_refCpoint;
    deltaPos_refCpoint.translation(orient_cpoint.inverse().toRotationMatrix() * deltaPos_refTable.translation());
    deltaPos_refCpoint.orientation(betweenMinusPiAndPlusPi(deltaPos_refTable.h()));

    outDEBUGPositionError = deltaPos_refCpoint;

    return deltaPos_refCpoint;
}

Twist2D OmnidirectOrder::computeRunTwist(arp_math::Pose2D currentPosition, double dt)
{
    Pose2D deltaPos_refCpoint = getPositionError_RobotRef(currentPosition);

    //new linear deceleration , will allow to turn in theta before going to the point
    // a = 2 * d / t²
    // t² = time for rotation + additional time
    // time for rotation = sqrt( 2 * d_rot / a_rot )
    double time_for_rotation=SUP_TIME_FOR_ROTATION+sqrt( 2 * fabs(deltaPos_refCpoint.h()) / m_conf.ANG_DEC );
    double reduced_LIN_DEC=2*deltaPos_refCpoint.vectNorm()/(time_for_rotation*time_for_rotation);
    double LIN_DEC=min(reduced_LIN_DEC,m_conf.LIN_DEC);

    // brutal correction twist. with constant acceleration   v = sqrt ( 2 . acc) . sqrt( d )
    Twist2D v_correction_cpoint;
    double speedcorrection = sqrt2(2.0 * LIN_DEC)
            * sqrt2(max(deltaPos_refCpoint.vectNorm() - m_v_correction_old.speedNorm() * TIMELAG, 0.0));

    // pour l'avance de phase sur l'angle, je met une zone morte pour l'empecher d'osciller.
    double distance_ang=deltaPos_refCpoint.h();
    double distance_ang_avance=deltaPos_refCpoint.h() - m_v_correction_old.vh()* TIMELAG;
    double distance_ang_avance_sat;
    /*
    if (sign(distance_ang)==sign(distance_ang_avance))
        distance_ang_avance_sat=distance_ang_avance;
    else
        distance_ang_avance_sat=0;
*/
    distance_ang_avance_sat=distance_ang_avance ;

    //correction en angle, la meme que pour la vitesse
    double angcorrection = sqrt2(2.0 * m_conf.ANG_DEC)
            * sqrt2(distance_ang_avance_sat);
    /*if (m_error_old==Pose2D(0,0,0)) // first turn ?
        m_error_old=deltaPos_refCpoint;
    double error_derivative=betweenMinusPiAndPlusPi(deltaPos_refCpoint.h()-m_error_old.h())/dt;
    double angcorrection = sqrt2(2.0 * m_conf.ANG_DEC)* sqrt2(deltaPos_refCpoint.h())+error_derivative*K_D;*/
    v_correction_cpoint.vx(speedcorrection * std::cos(deltaPos_refCpoint.vectAngle()));
    v_correction_cpoint.vy(speedcorrection * std::sin(deltaPos_refCpoint.vectAngle()));
    v_correction_cpoint.vh(angcorrection);

/*
    Log(DEBUG) << "----";
    Log(DEBUG) << "deltaPos_refCpoint.h() " << deltaPos_refCpoint.h();
    Log(DEBUG) << "m_error_old.h() " << m_error_old.h();
    Log(DEBUG) << "error_derivative " << error_derivative;
    Log(DEBUG) << "racine truc " << sqrt2(2.0 * m_conf.ANG_DEC)* sqrt2(deltaPos_refCpoint.h());
    Log(DEBUG) << "angcorrection " << angcorrection;
    Log(DEBUG) << "----";
*/

    m_error_old=deltaPos_refCpoint;

    outDEBUGLinSpeedCorrection = speedcorrection;
    outDEBUGAngSpeedCorrection = angcorrection;
    //transfer of the twist to robot referential
    Twist2D v_correction_ref;
    v_correction_ref = v_correction_cpoint.transport(m_cpoint.inverse());
    return v_correction_ref;
}

Twist2D OmnidirectOrder::computeApproachTwist(arp_math::Pose2D currentPosition, double dt)
{

    Pose2D deltaPos_refCpoint = getPositionError_RobotRef(currentPosition);
    // this is the current % of the initial error when I entered the approach zone
    m_normalizedError=getTotalError(deltaPos_refCpoint)/getTotalError(m_error_approach);

    outDEBUGNormalizedError=m_normalizedError;
    outDEBUGErrorApproachInit=getTotalError(m_error_approach);
    outDEBUGErrorApproachCur=getTotalError(deltaPos_refCpoint);

    Log(DEBUG) << ">>computeApproachTwist  " ;
    Log(DEBUG) << "current error  "<<deltaPos_refCpoint.toString() ;
    Log(DEBUG) << "current error norm  " <<getTotalError(deltaPos_refCpoint);
    Log(DEBUG) << "initial error  " <<m_error_approach.toString();
    Log(DEBUG) << "initial error norm  "<< getTotalError(m_error_approach);
    Log(DEBUG) << "normalize error  " << m_normalizedError;

    Log(DEBUG) << "m_twist_approach"<< m_twist_approach.toString() ;
    Log(DEBUG) << "twist applied"<< (m_twist_approach*sqrt2(m_normalizedError)).toString() ;
    Log(DEBUG) << "<<computeApproachTwist  ";

    // je remet le twist approach dans le bon referentiel car mon robot a tourné depuis!
    //Pose2D repere_rotation(0,0,currentPosition.angle()-m_pose_approach.angle());
    //Twist2D m_twist_approach_turned=m_twist_approach.changeProjection(repere_rotation);

    //m_twist_approach_turned=m_twist_approach;

    return m_twist_approach*sqrt2(m_normalizedError);


}

void OmnidirectOrder::decideSmoothNeeded(arp_math::Pose2D & currentPosition)
{
    //is a smooth localization needed ?
    Pose2D deltaPos_refCpoint = getPositionError_RobotRef(currentPosition);
    if(deltaPos_refCpoint.vectNorm() < DIST_SMOOTH)
        m_smoothLocNeeded = true;

    else
        m_smoothLocNeeded = false;

}
Twist2D OmnidirectOrder::computeSpeed(arp_math::Pose2D currentPosition, double dt)
{

    if (m_currentMode == MODE_DONE || m_currentMode == MODE_ERROR)
    {
        return Twist2D(0, 0, 0);
    }

    decideSmoothNeeded(currentPosition);

    Twist2D v_correction_ref;

    if (m_currentMode == MODE_RUN)
    {
    v_correction_ref = computeRunTwist(currentPosition,dt);
    }
    if (m_currentMode == MODE_APPROACH)
    {
    v_correction_ref = computeApproachTwist(currentPosition,dt);
    }

    //saturation of twist: limit max linear speed/acc;  and max rotation speed/acc
    Twist2D v_correction_saturated = saturateTwist(v_correction_ref, dt);

    m_v_correction_old = v_correction_saturated;

    return v_correction_saturated;

}

Twist2D OmnidirectOrder::saturateTwist(Twist2D twist_input, double dt)
{
    Twist2D twist_output;
    double vmaxlin = std::min(m_conf.LIN_VEL_MAX, m_v_correction_old.speedNorm() + dt * m_conf.LIN_DEC);
    double vmaxrot = std::min(m_conf.ANG_VEL_MAX, std::fabs(m_v_correction_old.vh()) + dt * m_conf.ANG_DEC);

    double satvlin = max(twist_input.speedNorm() / vmaxlin, 1.0);
    double satvrot = max(std::fabs(twist_input.vh()) / vmaxrot, 1.0);
    double saturated_linspeed = twist_input.speedNorm() * (1.0 / satvlin);
    double saturated_angspeed = twist_input.vh() * (1.0 / satvrot);
    twist_output = Twist2D(saturated_linspeed * cos(twist_input.speedAngle()),
            saturated_linspeed * sin(twist_input.speedAngle()), saturated_angspeed);

    outDEBUGSaturation = saturated_angspeed;

    return twist_output;
}

double OmnidirectOrder::getTotalError(Pose2D pose)
{
return sqrt(pose.x()*pose.x()+pose.y()*pose.y()+0.2*0.2*pose.h()*pose.h());
}
