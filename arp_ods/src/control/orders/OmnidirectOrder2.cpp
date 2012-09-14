/*
 * OmnidirectOrder.cpp
 *
 *  Created on:  14 sept 2013
 *      Author: RMO
 */

#include "OmnidirectOrder2.hpp"
#include "control/orders/Logger.hpp"
using namespace arp_math;
using namespace arp_ods;
using namespace orders;
using namespace arp_core::log;
using namespace boost;

OmnidirectOrder2::OmnidirectOrder2() :
        MotionOrder()
{
    m_type = OMNIDIRECT2;
    m_v_correction_old = Twist2D(0, 0, 0);
    m_twist_init = Twist2D(0, 0, 0);
    m_twist_init_registered = false;

}

OmnidirectOrder2::OmnidirectOrder2(MotionOrder order) :
        MotionOrder(order)
{
}

shared_ptr<MotionOrder> OmnidirectOrder2::createOrder(const OrderGoalConstPtr &goal, Pose2D currentPose,
        orders::config conf)
{
    shared_ptr<OmnidirectOrder2> order(new OmnidirectOrder2());

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
    if (goal->max_speed > 0.0 and goal->max_speed < conf.LIN_VEL_MAX)
        order->m_vmax_order = goal->max_speed;
    else
        order->m_vmax_order = conf.LIN_VEL_MAX;

    //mettre m_timeout a la bonne valeur pour pas perdre trop de temps
    // il s'agit de timeout = timeout max  si v = 0 et timeoutmin si v=lindec
    // TODO et oui j'ecrase la conf comme un gros porcasse
    conf.ORDER_TIMEOUT = TIMEOUTMAX + (TIMEOUTMIN - TIMEOUTMAX) / (conf.LIN_VEL_MAX) * order->m_vmax_order;

    order->setConf(conf);

    Log(INFO) << order->getTypeString() << "from [" << begin.toString() << "] to [" << end.toString()
            << "] control_point=[" << cpoint.toString() << "]";

    return static_cast<shared_ptr<MotionOrder> >(order);
}

void OmnidirectOrder2::switchRun(arp_math::Pose2D currentPosition)
{

    Pose2D deltaPos_refTable = getPositionError(currentPosition);
    double distance_error = deltaPos_refTable.vectNorm();
    double angle_error = deltaPos_refTable.h();

    Pose2D current_cpoint_position = currentPosition * m_cpoint;

    Pose2D deltaPos_refCpoint = getPositionError_RobotRef(currentPosition);

    if (fabs(m_error_approach.translation().dot(deltaPos_refCpoint.translation())) < m_conf.DISTANCE_ACCURACY
            && fabs(deltaPos_refCpoint.h()) < m_conf.ANGLE_ACCURACY)
    {
        Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_DONE " << "because in final zone ";
        m_currentMode = MODE_DONE;
        return;
    }

    testTimeout();

}

Pose2D OmnidirectOrder2::getPositionError(arp_math::Pose2D currentPosition)
{
    Pose2D current_cpoint_position = currentPosition * m_cpoint;

    // Pourquoi ?
    // currentPosition est la pose du robot dans le repère de la table
    // m_cpoint est la pose du point de controle dans le repère du robot
    // pour avoir le point de controle dans le repère de la table, il suffit de composer (opérateur *) les deux poses

    //position error
    return MathFactory::createPose2D(m_endPose.translation() - current_cpoint_position.translation(),
            Rotation2(betweenMinusPiAndPlusPi(m_endPose.angle() - current_cpoint_position.angle())));
}

Pose2D OmnidirectOrder2::getPositionError_RobotRef(arp_math::Pose2D currentPosition)
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

Twist2DNorm OmnidirectOrder2::computeRunTwist(Pose2D currentPosition, ICRSpeed curICRSpeed, double dt)
{

    //TODO a ce stade le theta est entre 0 et 2 PI, il faut gere le remettage entre -PI et PI
    Pose2DNorm currentPositionNorm(currentPosition);



    Vector3 Cerr=-1.0*currentPositionNorm.getTVector();
    double ro=sqrt(2*0.5)*sqrt(Cerr.norm());
    double phi=atan2(Cerr[1],Cerr[0]);
    double delta=0;
    //double delta=atan(Cerr[2]/sqrt(Cerr[0]*Cerr[0]+Cerr[1]*Cerr[1]));

    ICRSpeed corICRSpeed=ICRSpeed(ro,phi,delta);

    Log(DEBUG) << ">>computeRunTwist  ";
    Log(DEBUG) << "currentPosition  "<<currentPosition.toString();
    Log(DEBUG) << "currentPositionNorm  "<<currentPositionNorm.toString();
    Log(DEBUG) << "curICRSpeed  "<<curICRSpeed.toString();
    Log(DEBUG) << "corICRSpeed  "<<corICRSpeed.toString();
    Log(DEBUG) << "<<computeRunTwist  ";

    return corICRSpeed.twistNorm();
}

void OmnidirectOrder2::decideSmoothNeeded(arp_math::Pose2D & currentPosition)
{
    //is a smooth localization needed ?
    Pose2D deltaPos_refCpoint = getPositionError_RobotRef(currentPosition);
    if (deltaPos_refCpoint.vectNorm() < DIST_SMOOTH)
        m_smoothLocNeeded = true;

    else
        m_smoothLocNeeded = false;

    if (m_smoothLocNeeded)
        outDEBUGSmoothLocNeeded = 1.0;
    else
        outDEBUGSmoothLocNeeded = 0.0;

}
Twist2D OmnidirectOrder2::computeSpeed(Pose2D currentPosition, MotorState motorState, UbiquityParams params, double dt)
{

    if (m_currentMode == MODE_DONE || m_currentMode == MODE_ERROR)
    {
        return Twist2D(0, 0, 0);
    }

    decideSmoothNeeded(currentPosition);

    ICRSpeed curICRSpeed;
    TurretState oTS;
    SlippageReport oSR;

    //conversion des tourelles en ICRSpeed
    UbiquityKinematics::motors2ICRSpeed(motorState, oTS, curICRSpeed, oSR, params);

    //Rtarget = repere objectif
    // Rtarget->Rrobot = (RO->Rtarget)⁻1 x (R0->Rrobot)
    Pose2D RTarget_Rrobot=m_endPose.inverse()*currentPosition;

    //compute run twist travaille dans un espace 3D ou l'objectif est en (0,0,0)
    //attention curICRSpeed est dans le repere target
    ICRSpeed corICRSpeed=computeRunTwist(RTarget_Rrobot, curICRSpeed, dt);
    //dans le repere robot
    corICRSpeed.phi(corICRSpeed.phi()-m_endPose.h() - currentPosition.h());

    Twist2D corTwist = corICRSpeed.twist();
    return corTwist;

}

double OmnidirectOrder2::getTotalError(Pose2D pose)
{
    return sqrt(pose.x() * pose.x() + pose.y() * pose.y() + 0.2 * 0.2 * pose.h() * pose.h());
}
