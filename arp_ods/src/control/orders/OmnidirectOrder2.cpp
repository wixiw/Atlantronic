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
    m_lastRo = 0;

}

OmnidirectOrder2::OmnidirectOrder2(MotionOrder order) :
        MotionOrder(order)
{
}

shared_ptr<MotionOrder> OmnidirectOrder2::createOrder(const OrderGoalConstPtr &goal,
        arp_math::UbiquityMotionState currentMotionState, orders::config conf)
{
    shared_ptr<OmnidirectOrder2> order(new OmnidirectOrder2());

    UbiquityMotionState beginMotionState;
    UbiquityMotionState endMotionState;
    Pose2D end;
    Pose2D cpoint;

    order->setBeginMotionState(currentMotionState);

    cpoint.x(goal->x_cpoint);
    cpoint.y(goal->y_cpoint);
    cpoint.h(goal->theta_cpoint);
    order->setCpoint(cpoint);

    end.x(goal->x_des);
    end.y(goal->y_des);
    end.h(goal->theta_des);
    endMotionState.setPosition(end);
    if (goal->passe == false)
        endMotionState.getSpeed().ro(0.0);
    else
        endMotionState.getSpeed().ro(goal->passe_speed);

    order->setEndMotionState(endMotionState);

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

    Log(INFO) << order->getTypeString() << "from [" << ((Pose2D) (beginMotionState.getPosition())).toString()
            << "] to [" << end.toString() << "] control_point=[" << cpoint.toString() << "]";

    return static_cast<shared_ptr<MotionOrder> >(order);
}

void OmnidirectOrder2::switchInit(arp_math::UbiquityMotionState currentMotionState)
{
    m_oldICRSpeed = currentMotionState.getSpeed();
    Log(DEBUG) << "---curICRSpeed initialise " << m_oldICRSpeed.toString();
    ModeSelector::switchInit(currentMotionState);
}

void OmnidirectOrder2::switchRun(arp_math::UbiquityMotionState currentMotionState)
{

    if (getPositionInNormalRef(currentMotionState.getPosition()).getTVector().norm() < m_conf.DISTANCE_ACCURACY)
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
double OmnidirectOrder2::profileRoJerking(double distance, ICRSpeed curICRSpeed, double roPass)
{
    //TODO passer les parametres en en argument
    double lin_dec = 1;
    double v_max = 1;
    //TODO passer period en argument
    double period = 0.010;

    double curRo = m_oldICRSpeed.ro();
    //double curRo=curICRSpeed.ro();

    //the distance i'll do due to my speed. i'll add that into the distance used for deceleration to take into account the delay
    double distanceDelay = curRo * period;

    double limite_lineaire = 0.030;
    double ro;

    Log(DEBUG) << "<< profileRoJerking";
    Log(DEBUG) << "m_lastRo " << m_lastRo;
    Log(DEBUG) << "distance " << distance;

    if (abs(distance) > limite_lineaire)
    {
        ro = sqrt2(2.0 * lin_dec) * sqrt2(distance);
        Log(DEBUG) << "ro  racine       " << ro;
    }
    else
    {
        ro = (sqrt2(2.0 * lin_dec) * sqrt2(limite_lineaire)) / limite_lineaire * distance;
        Log(DEBUG) << "ro  lineaire     " << ro;
    }

    double ros = saturate(ro, -v_max, v_max);
    double ross = ros;

    // limitation de l'acceleration
    double accStep = lin_dec * period;
    if (ros > 0)
    {
        if (curRo < ros)
        {//accel
            ross = min(ros, curRo + accStep);
        }
        else
        {//decel
        }
    }
    else
    {
        if (curRo > ros)
        {//"augmentation du mouvement"
            ross = max(ros, curRo - accStep);
        }
        else
        {//"diminution du mouvement"
        }
    }

    //ross = firstDerivateLimitation(ross, curRo, period, -lin_dec * 2.0, lin_dec * 2.0);

    Log(DEBUG) << "ros      " << ros;
    Log(DEBUG) << "ross     " << ross;
    Log(DEBUG) << ">> profileRoJerking";

    return ross;

}

double OmnidirectOrder2::distanceModifier(double realDistance, double distanceDelay)
{
    if (abs(realDistance) < 0.001)
        return 0;

    if (realDistance > 0)
        return realDistance - distanceDelay;
    else
        return realDistance + distanceDelay;
}

ICRSpeed OmnidirectOrder2::computeRunTwist(Pose2DNorm currentPositionNorm, ICRSpeed curICRSpeed, double dt)
{
    if (curICRSpeed.getICR().sphericalDistance(m_oldICRSpeed.getICR()) > PI / 2)
        curICRSpeed = curICRSpeed.getOppositeRep();

    Log(DEBUG) << ">>computeRunTwist  ";
    Log(DEBUG) << "currentPositionNorm  " << currentPositionNorm.toString();
    Log(DEBUG) << "curICRSpeed  " << curICRSpeed.toString();

    ICRSpeed corICRSpeed;

    /*
     * theta and phi:
     * there are first the "perfect" theta and phi
     * but in fact we cannot reach any theta and phi instantly
     * so we choose something inbetween
     */
    Vector3 Cerr = -1.0 * currentPositionNorm.getTVector();
    double phi_perfect = atan2(Cerr[1], Cerr[0]);
    double delta_perfect = asin(Cerr[2] / Cerr.norm());

    ICR ICR_perfect(phi_perfect, delta_perfect);
    //limitation of the motion of the ICR
    //ICR curICR = curICRSpeed.getICR();
    ICR curICR = m_oldICRSpeed.getICR();

    // gestion du parkinson final
    // TODO depend de dt
    double s_max = 0.1 * getParkinsonLimitationFactor(Cerr.norm());

    if (ICR_perfect.sphericalDistance(curICR) > PI / 2)
        ICR_perfect = ICR_perfect.getAntipodICR();

    ICR ICR_possible = curICR.getIntermediate(ICR_perfect, s_max);

    corICRSpeed = ICRSpeed(1.0, ICR_possible);

    Log(DEBUG) << "Cerr.norm()                               " << Cerr.norm();
    Log(DEBUG) << "getParkinsonLimitationFactor(Cerr.norm()) " << getParkinsonLimitationFactor(Cerr.norm());
    Log(DEBUG) << "s_max                                     " << s_max;
    Log(DEBUG) << "ICR_perfect        " << ICR_perfect.toString();
    Log(DEBUG) << "curICR             " << curICR.toString();
    Log(DEBUG) << "ICR_possible       " << ICR_possible.toString();

    double k_delta = Cerr.dot(corICRSpeed.speedDirection());
    //TODO donner la vitesse de passage
    double ro = profileRoJerking(k_delta, curICRSpeed, 0.0);

    Log(DEBUG) << "k_delta       " << k_delta;
    Log(DEBUG) << "ro            " << ro;

    corICRSpeed = ICRSpeed(ro, ICR_possible);

    //je remet tout dans leur range nominales
    //corICRSpeed = corICRSpeed.getNormalizedRep();

    //m_lastRo = corICRSpeed.ro();

    Log(DEBUG) << "Cerr.norm()           " << Cerr.norm();

    Log(DEBUG) << "corICRSpeed normalise " << corICRSpeed.toString();
    Log(DEBUG) << "<<computeRunTwist     ";
    Log(DEBUG) << "                      ";

    m_oldICRSpeed = corICRSpeed;

    //for debug
    outDEBUG1 = Cerr.norm();
    outDEBUG2 = ro;
    outDEBUG3 = rad2deg(ICR_possible.phi()) / 100.0;
    outDEBUG4 = rad2deg(ICR_possible.delta()) / 100.0;

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
ICRSpeed OmnidirectOrder2::computeSpeed(UbiquityMotionState currentMotionState, UbiquityParams params, double dt)
{
    Pose2D currentPosition = currentMotionState.getPosition();
    decideSmoothNeeded(currentPosition);
    ICRSpeed curICRSpeed = currentMotionState.getSpeed();

    if (m_currentMode == MODE_ERROR)
    {
        return Twist2D(0, 0, 0);
    }

    // curICRSpeed est dans le repere robot il faut  le mettre dans le repere target
    curICRSpeed.phi(curICRSpeed.phi() - m_endMotionState.getPosition().h() + currentPosition.h());

    //compute run twist travaille dans un espace 3D ou l'objectif est en (0,0,0)
    ICRSpeed corICRSpeed = computeRunTwist(getPositionInNormalRef(currentPosition), curICRSpeed, dt);
    //dans le repere robot
    corICRSpeed.phi(corICRSpeed.phi() + m_endMotionState.getPosition().h() - currentPosition.h());

    Twist2D corTwist = corICRSpeed.twist();
    return corTwist;

}

Pose2DNorm OmnidirectOrder2::getPositionInNormalRef(Pose2D currentPosition)
{
    //Rtarget = repere objectif
    // Rtarget->Rrobot = (RO->Rtarget)⁻1 x (R0->Rrobot)
    Pose2D result(m_endMotionState.getPosition().inverse() * currentPosition);
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
