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

    m_oldICRSpeed = ICRSpeed();
    m_predictedAcc = 0;
    m_lastRo=0;

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

    if (getPositionInNormalRef(currentPosition).getTVector().norm() < m_conf.DISTANCE_ACCURACY)
    {
        Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_DONE " << "because in final zone ";
        m_currentMode = MODE_DONE;
        return;
    }

    testTimeout();

}

double OmnidirectOrder2::profileRo(double distance, ICRSpeed curICRSpeed)
{
    PosVelAcc start;
    start.position = 0;
    start.velocity = curICRSpeed.ro();
    //start.acceleration = (curICRSpeed.ro() - m_oldICRSpeed.ro()) / dt;
    //mmm ca affole l'OTG cette acceleration. essayons sur le predit plutot que le realise. oh ! ca marche !
    start.acceleration = m_predictedAcc;
    PosVelAcc end;
    end.position = distance;
    end.velocity = 0;
    end.acceleration = 0;
    PosVelAcc next;
    double ro;

    if (OTG == NULL)
    {
        ro = 0;
        Log(DEBUG) << "*** PB sur calcul RO : pointeur null sur OTG ****";
    }

    Log(DEBUG) << "*** computeNextStepCheap ****";
    Log(DEBUG)
            << "bool OTGres = OTG->computeNextStepCheap(start, end, m_conf.LIN_VEL_MAX, m_conf.LIN_DEC, m_conf.LIN_DEC * 5, next);";
    Log(DEBUG) << "start.position" << start.position;
    Log(DEBUG) << "start.velocity" << start.velocity;
    Log(DEBUG) << "end.position" << end.position;
    Log(DEBUG) << "m_conf.LIN_VEL_MAX" << m_conf.LIN_VEL_MAX;
    Log(DEBUG) << "m_conf.LIN_DEC" << m_conf.LIN_DEC;
    bool OTGres = OTG->computeNextStepCheap(start, end, m_conf.LIN_VEL_MAX, m_conf.LIN_DEC, m_conf.LIN_DEC * 5, next);
    Log(DEBUG) << "--->next.velocity" << next.velocity;

    if (OTGres)
    {
        ro = next.velocity;
    }
    else
    {
        ro = 0;
        Log(DEBUG) << "*** PB sur calcul RO: reflexxes a retourne false ****";
    }

    m_predictedAcc = next.acceleration;

    return ro;
}

double OmnidirectOrder2::profileRoJerking(double distance, ICRSpeed curICRSpeed)
{
    double lin_dec=0.3;
    double v_max=0.5;

    double ro = sqrt2(2.0 * lin_dec)* sqrt2(distance);
    ro=saturate(ro,-v_max,v_max);
    if ((ro-m_lastRo)>lin_dec )
            {
            //acceleration saturation
            ro=m_lastRo+lin_dec;
            }

    m_lastRo=ro;
    Log(DEBUG) << "<< profileRoJerking";
    Log(DEBUG) << "distance" << distance;
    Log(DEBUG) << "ro" << ro;
    Log(DEBUG) << ">> profileRoJerking";
    return ro;

}


ICRSpeed OmnidirectOrder2::computeRunTwist(Pose2DNorm currentPositionNorm, ICRSpeed curICRSpeed, double dt)
{
    Log(DEBUG) << ">>computeRunTwist  ";
    Log(DEBUG) << "currentPositionNorm  " << currentPositionNorm.toString();
    Log(DEBUG) << "curICRSpeed  " << curICRSpeed.toString();

    Vector3 Cerr = -1.0 * currentPositionNorm.getTVector();
    double k_delta = Cerr.dot(curICRSpeed.speedDirection());

    Log(DEBUG) << "k_delta  " << k_delta;


    double ro = profileRoJerking(k_delta, curICRSpeed);
    //double ro = profileRo(Cerr.norm(), curICRSpeed);

    Log(DEBUG) << "m_conf.LIN_DEC" << m_conf.LIN_DEC;
    Log(DEBUG) << "Cerr.norm()   " << Cerr.norm();
    Log(DEBUG) << "ro            " << ro;

    /*
     * theta and phi:
     * there are first the "perfect" theta and phi
     * but in fact we cannot reach any theta and phi instantly
     * so we choose something inbetween
     */
    double vrotmax = PI; //turret speed in red/s
    double s_max = vrotmax * 0.05; //travel max on sphere

    // gestion du parkinson final
    s_max=s_max*getParkinsonLimitationFactor(Cerr.norm());

    Log(DEBUG) << "getParkinsonLimitationFactor  " << getParkinsonLimitationFactor(Cerr.norm());
    Log(DEBUG) << "s_max                         " << s_max;

    double phi_perfect = atan2(Cerr[1], Cerr[0]);
    double delta_perfect = asin(Cerr[2] / Cerr.norm());
    ICR ICR_perfect(phi_perfect, delta_perfect);

    ICR curICR;
    if (k_delta > 0) //    if kdelta<0, I want to go backward. and I don't want to go to the antipod with the ICR !
    {
        curICR = curICRSpeed.getICR();
    }
    else
    {
        curICR = curICRSpeed.getOppositeRep().getICR();
        m_predictedAcc = -m_predictedAcc;
    }

    //limitation of the motion of the ICR
    ICR ICR_possible = curICR.getIntermediate(ICR_perfect, s_max);

    /***** DEBUG ****/
    ICR_possible=ICR_perfect;

    ICRSpeed corICRSpeed ;
    if (k_delta > 0)
        corICRSpeed= ICRSpeed(ro, ICR_possible);
    else
        corICRSpeed= ICRSpeed(-ro, ICR_possible);

    /*
    //put back corICRspeed with ro >0
    if (corICRSpeed.ro() < 0)
        corICRSpeed = corICRSpeed.getOppositeRep();
        */

    /***** DEBUG ****/
    //corICRSpeed = ICRSpeed(ro, phi_perfect, delta_perfect);

    Log(DEBUG) << "corICRSpeed  " << corICRSpeed.toString();
    Log(DEBUG) << "<<computeRunTwist  ";
    Log(DEBUG) << "                   ";

    m_oldICRSpeed = curICRSpeed;

    //for debug
    outDEBUG1 = Cerr.norm();
    outDEBUG2 = ro;
    outDEBUG3 = m_predictedAcc;
    outDEBUG4 = dt;

    return corICRSpeed;
}

void OmnidirectOrder2::decideSmoothNeeded(arp_math::Pose2D & currentPosition)
{

    if (getPositionInNormalRef(currentPosition).vectNorm() < DIST_SMOOTH)
        m_smoothLocNeeded = true;

    else
        m_smoothLocNeeded = false;

    if (m_smoothLocNeeded)
        outDEBUG8 = 1.0;
    else
        outDEBUG8 = 0.0;

}
Twist2D OmnidirectOrder2::computeSpeed(Pose2D currentPosition, MotorState motorState, UbiquityParams params, double dt)
{

    if (m_currentMode == MODE_ERROR)
    {
        return Twist2D(0, 0, 0);
    }

    decideSmoothNeeded(currentPosition);

    ICRSpeed curICRSpeed;
    TurretState oTS;
    SlippageReport oSR;

    //conversion des tourelles en ICRSpeed
    UbiquityKinematics::motors2ICRSpeed(motorState, oTS, curICRSpeed, oSR, params);
    // curICRSpeed est dans le repere robot il faut  le mettre dans le repere target
    curICRSpeed.phi(curICRSpeed.phi() - m_endPose.h() + currentPosition.h());

    //compute run twist travaille dans un espace 3D ou l'objectif est en (0,0,0)
    ICRSpeed corICRSpeed = computeRunTwist(getPositionInNormalRef(currentPosition), curICRSpeed, dt);
    //dans le repere robot
    corICRSpeed.phi(corICRSpeed.phi() + m_endPose.h() - currentPosition.h());

    Twist2D corTwist = corICRSpeed.twist();
    return corTwist;

}

Pose2DNorm OmnidirectOrder2::getPositionInNormalRef(Pose2D currentPosition)
{
    //Rtarget = repere objectif
    // Rtarget->Rrobot = (RO->Rtarget)⁻1 x (R0->Rrobot)
    Pose2D result(m_endPose.inverse() * currentPosition);
    result.h(betweenMinusPiAndPlusPi(result.h()));
    Pose2DNorm resultNorm(result);
    return resultNorm;
}

double OmnidirectOrder2::getParkinsonLimitationFactor(double distance)
{
    double freeze_distance=0.05;
    double noConstraint_distance=1;
    return smoothStep(distance, freeze_distance, 0, noConstraint_distance, 1);
}
