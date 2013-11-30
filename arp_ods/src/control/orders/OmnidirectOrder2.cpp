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
        MotionOrder(),
        m_firstTime(false)
{
    m_type = OMNIDIRECT2;
    m_v_correction_old = Twist2D(0, 0, 0);
    m_twist_init = Twist2D(0, 0, 0);
    m_twist_init_registered = false;

    m_oldICRSpeed = ICRSpeed();
    m_predictedAcc = 0;
    m_lastRo = 0;

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

void OmnidirectOrder2::switchInit(arp_math::Pose2D currentPosition)
{
    m_firstTime = true;
    ModeSelector::switchInit(currentPosition);
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

//TODO last_ro devrait etre passé en parametre
double OmnidirectOrder2::profileRoJerking(double distance, ICRSpeed curICRSpeed)
{
    //TODO passer les parametres en en argument
    double lin_dec = 0.6;
    double v_max = 0.5;
    //TODO passer period en argument
    double period = 0.010;

    double ro = sqrt2(2.0 * lin_dec) * sqrt2(distance);
    double ros = saturate(ro, -v_max, v_max);
    double ross = ros;

    //TODO pas propre
    if (abs(distance) >= 0.050)
    {
        //ross=firstDerivateLimitation(ross, curICRSpeed.ro(), period, -lin_dec*2.0,lin_dec*2.0);
        ross = firstDerivateLimitation(ross, m_oldICRSpeed.ro(), period, -lin_dec * 2.0, lin_dec * 2.0);
    }

    //TODO idealement les valeurs la dessous dependent des accelerations
    if (abs(m_oldICRSpeed.ro()) < 0.05 && abs(distance) < 0.001)
        ross = 0;

    /*
     Log(DEBUG) << "<< profileRoJerking";
     Log(DEBUG) << "m_lastRo " << m_lastRo;
     Log(DEBUG) << "distance " << distance;
     Log(DEBUG) << "ro       " << ro;
     Log(DEBUG) << "ros      " << ros;
     Log(DEBUG) << "ross     " << ross;
     Log(DEBUG) << ">> profileRoJerking";
     */

    return ross;

}

ICRSpeed OmnidirectOrder2::computeRunTwist(Pose2DNorm currentPositionNorm, ICRSpeed curICRSpeed, double dt)
{
    Log(DEBUG) << ">>computeRunTwist  ";
    Log(DEBUG) << "currentPositionNorm  " << currentPositionNorm.toString();
    Log(DEBUG) << "curICRSpeed  " << curICRSpeed.toString();

    ICRSpeed corICRSpeed;

    Vector3 Cerr = -1.0 * currentPositionNorm.getTVector();

    /*
     * theta and phi:
     * there are first the "perfect" theta and phi
     * but in fact we cannot reach any theta and phi instantly
     * so we choose something inbetween
     */
    //double vrotmax = PI; //turret speed in rad/s
    //double s_max = vrotmax * 0.10; //travel max on sphere
    // gestion du parkinson final
    // TODO depend de dt
    double s_max = 0.2 * getParkinsonLimitationFactor(Cerr.norm());

    Log(DEBUG) << "Cerr.norm()                               " << Cerr.norm();
    Log(DEBUG) << "getParkinsonLimitationFactor(Cerr.norm()) " << getParkinsonLimitationFactor(Cerr.norm());
    Log(DEBUG) << "s_max                                     " << s_max;

    double phi_perfect = atan2(Cerr[1], Cerr[0]);
    double delta_perfect = asin(Cerr[2] / Cerr.norm());

    ICR ICR_perfect(phi_perfect, delta_perfect);
    //limitation of the motion of the ICR
    //ICR curICR = curICRSpeed.getICR();
    ICR curICR = m_oldICRSpeed.getICR();

    ICR ICR_possible = curICR.getIntermediate(ICR_perfect, s_max);

    corICRSpeed = ICRSpeed(1.0, ICR_possible);

    Log(DEBUG) << "ICR_perfect        " << ICR_perfect.toString();
    Log(DEBUG) << "curICR             " << curICR.toString();
    Log(DEBUG) << "ICR_possible       " << ICR_possible.toString();

    double k_delta = Cerr.dot(corICRSpeed.speedDirection());
    double ro = profileRoJerking(k_delta, curICRSpeed);

    Log(DEBUG) << "k_delta       " << k_delta;
    Log(DEBUG) << "ro            " << ro;

    corICRSpeed = ICRSpeed(ro, ICR_possible);

    //je remet tout dans leur range nominales
    corICRSpeed = corICRSpeed.getNormalizedRep();

    //m_lastRo = corICRSpeed.ro();

    Log(DEBUG) << "Cerr.norm()           " << Cerr.norm();

    Log(DEBUG) << "corICRSpeed normalise " << corICRSpeed.toString();
    Log(DEBUG) << "<<computeRunTwist     ";
    Log(DEBUG) << "                      ";

    m_oldICRSpeed = corICRSpeed;

    //for debug
    outDEBUG1 = Cerr.norm();
    outDEBUG2 = ro;
    outDEBUG3 = rad2deg(corICRSpeed.phi()) / 100.0;
    outDEBUG4 = rad2deg(corICRSpeed.delta()) / 100.0;

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

    decideSmoothNeeded(currentPosition);

    ICRSpeed curICRSpeed;
    TurretState oTS;
    SlippageReport oSR;

    //conversion des tourelles en ICRSpeed
    UbiquityKinematics::motors2ICRSpeed(motorState, oTS, curICRSpeed, oSR, params);

    if (m_currentMode == MODE_ERROR)
    {
        return Twist2D(0, 0, 0);
    }

    if( m_firstTime )
    {
        m_firstTime = false;
        m_oldICRSpeed=curICRSpeed;
        Log(DEBUG) << "---curICRSpeed initialise " <<curICRSpeed.toString();
    }

    //DEBUG
    Twist2D oTw;
    TurretState otS;
    UbiquityKinematics::motors2Twist(motorState, oTS, oTw, oSR, params);

    if (abs(oTw.vx()) > 0.001 or abs(oTw.vy()) > 0.001 or abs(oTw.vh()) > 0.001)
    {
        Log(DEBUG) << "calcul en passant par twist  ";
    }
    else
    {
        Log(DEBUG) << "calcul en passant par icr et ro=0  ";
    }
    // FIN DEBUG

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
    double freeze_distance = 0.010;
    double noConstraint_distance = 0.1;
    return smoothStep(distance, 0, freeze_distance, 1, noConstraint_distance);
}
