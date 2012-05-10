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
    m_angle_speedcorrection_old = 0.0;

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

//    Log(DEBUG) << ">> switchRun de Omnidirect";

//position error in table referential
    Pose2D deltaPos_refTable = getPositionError(currentPosition);
    double distance_error = deltaPos_refTable.vectNorm();
    double angle_error = deltaPos_refTable.h();

    Pose2D current_cpoint_position = currentPosition * m_cpoint;

//    Log(DEBUG) << "distance_error "<<distance_error;
//    Log(DEBUG) << "angle_error    "<<angle_error;
//    Log(DEBUG) << "m_conf.DISTANCE_ACCURACY "<<m_conf.DISTANCE_ACCURACY;
//    Log(DEBUG) << "m_conf.ANGLE_ACCURACY    "<<m_conf.ANGLE_ACCURACY;
    // test for DONE
    if (distance_error <= m_conf.RADIUS_APPROACH_ZONE)
    {
        Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_APPROACH " << "because in approach zone ";
        m_currentMode = MODE_APPROACH;
        m_approachTime = getTime();
        // si j'entre directement en approche, il faut que je calcule un twist initial
        m_twist_approach = computeRunTwist(currentPosition);
        return;
    }

    testTimeout();

//    Log(DEBUG) << "<< switchRun de Omnidirect";

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

    if (getTime() - m_approachTime > 0.3) //TODO ceci ne marche pas sur une BF cap. il faut verifier que l'erreur "est assez petite" ou "ne reaugmente pas"
    {

        Log(INFO) << getTypeString() << " switched MODE_APPROACH --> MODE_DONE "
                << "because in approach zone enough time";
        Log(INFO) << "robot position  " << currentPosition.toString();
        Log(INFO) << "cpoint position " << current_cpoint_position.toString();
        Log(INFO) << "aim position    " << m_endPose.toString();
        Log(INFO) << "distance_error  " << distance_error;
        Log(INFO) << "angle_error     " << angle_error;

        m_currentMode = MODE_DONE;
        return;
    }

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

Twist2D OmnidirectOrder::computeRunTwist(arp_math::Pose2D currentPosition)
{
    Pose2D deltaPos_refCpoint = getPositionError_RobotRef(currentPosition);

    // brutal correction twist. with constant acceleration   v = sqrt ( 2 . acc) . sqrt( d )
    Twist2D v_correction_cpoint;
    double speedcorrection = sqrt2(2.0 * m_conf.LIN_DEC)
            * sqrt2(max(deltaPos_refCpoint.vectNorm() - m_v_correction_old.speedNorm() * TIMELAG, 0.0));
    double angle_speedcorrection;
    if (m_currentMode == MODE_APPROACH) // suis je proche
    {

        // il faudrait l'initialiser a la creation de l'ordre
        // l'angle de correction de vitesse reste celui au moment ou je me suis approché
        angle_speedcorrection = m_angle_speedcorrection_old;
        // la vitesse de correction est pondérée par l'angle de l'erreur et l'angle d'approche (du coup ca peut devenir négatif ! ennoorme)
        Vector2 vectcorrection(cos(angle_speedcorrection), sin(angle_speedcorrection));
        Vector2 vecterreur(cos(deltaPos_refCpoint.vectAngle()), sin(deltaPos_refCpoint.vectAngle()));
        // un petit produit scalaire me permet de trouver ce coefficient
        double ponderation = vectcorrection.dot(vecterreur);
        speedcorrection = ponderation * speedcorrection;
    }
    else
    {
        // cas nominal: la correction de vitesse c'est l'angle de l'erreur
        angle_speedcorrection = deltaPos_refCpoint.vectAngle();
        //j'enregistre au cas ou je m'approcherais
        m_angle_speedcorrection_old = angle_speedcorrection;
    }
    double angcorrection = sqrt2(2.0 * m_conf.ANG_DEC)
            * sqrt2(deltaPos_refCpoint.h() - m_v_correction_old.vh() * TIMELAG * 0.5);
    v_correction_cpoint.vx(speedcorrection * std::cos(angle_speedcorrection));
    v_correction_cpoint.vy(speedcorrection * std::sin(angle_speedcorrection));
    v_correction_cpoint.vh(angcorrection);
    outDEBUGLinSpeedCorrection = speedcorrection;
    outDEBUGAngSpeedCorrection = angcorrection;
    //transfer of the twist to robot referential
    Twist2D v_correction_ref;
    v_correction_ref = v_correction_cpoint.transport(m_cpoint.inverse());
    return v_correction_ref;
}

Twist2D OmnidirectOrder::computeApproachTwist(arp_math::Pose2D currentPosition)
{

}

Twist2D OmnidirectOrder::computeSpeed(arp_math::Pose2D currentPosition, double dt)
{

    if (m_currentMode == MODE_DONE || m_currentMode == MODE_ERROR)
    {
        return Twist2D(0, 0, 0);
    }

    Twist2D v_correction_ref;

    /*if (m_currentMode == MODE_RUN)
    {*/
    v_correction_ref = computeRunTwist(currentPosition);
    /*}
    if (m_currentMode == MODE_APPROACH)
    {
    v_correction_ref = computeApproachTwist(currentPosition);
    }*/

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
