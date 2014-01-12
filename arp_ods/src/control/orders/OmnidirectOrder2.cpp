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

OmnidirectOrder2::OmnidirectOrder2(const OrderGoalConstPtr &goal, arp_math::UbiquityMotionState currentMotionState,
        UbiquityParams params) :
        MotionOrder(goal, currentMotionState, params)

{

    m_type = OMNIDIRECT2;
    m_v_correction_old = Twist2D(0, 0, 0);
    m_twist_init = Twist2D(0, 0, 0);
    m_twist_init_registered = false;

    m_oldICRSpeed = ICRSpeed();
    m_predictedAcc = 0;
    m_lastRo = 0;

    UbiquityMotionState beginMotionState;
    UbiquityMotionState endMotionState;
    Pose2D end;
    Pose2D cpoint;

    setBeginMotionState(currentMotionState);

    cpoint.x(goal->x_cpoint);
    cpoint.y(goal->y_cpoint);
    cpoint.h(goal->theta_cpoint);
    setCpoint(cpoint);

    end.x(goal->x_des);
    end.y(goal->y_des);
    end.h(goal->theta_des);
    endMotionState.setPosition(end);
    setPass(goal->passe);
    if (goal->passe == false)
    {
        endMotionState.getSpeed().ro(0.0);
    }
    else
    {
        endMotionState.getSpeed().ro(goal->passe_speed);
        setPassSpeed(goal->passe_speed);
    }

    setEndMotionState(endMotionState);

    if (goal->max_speed > 0.0 and goal->max_speed < m_params.getMaxRobotSpeed())
        m_vmax_order = goal->max_speed;
    else
        m_vmax_order = m_params.getMaxRobotSpeed();

    // timeout is initialised by a rough distance / speed with a margin
    m_timeout = max(end.distanceTo(currentMotionState.position)/m_vmax_order*2.0 , 5.0);

    Log(DEBUG) << "goal->max_speed " << goal->max_speed;
    Log(DEBUG) << "m_params.getMaxRobotSpeed() " << m_params.getMaxRobotSpeed();
    Log(DEBUG) << "m_vmax_order " << m_vmax_order;
    Log(DEBUG) << "end " << end.toString();
    Log(DEBUG) << "tcurrentMotionState.position " << currentMotionState.position.toString();
    Log(DEBUG) << "end.distanceTo(currentMotionState.position) " << end.distanceTo(currentMotionState.position);

    Log(DEBUG) << "timeout initialise a " << m_timeout;

}


void OmnidirectOrder2::switchInit(arp_math::UbiquityMotionState currentMotionState)
{
    m_oldICRSpeed = currentMotionState.getSpeed();
    Log(DEBUG) << "---curICRSpeed initialise " << m_oldICRSpeed.toString();
    MotionOrder::switchInit(currentMotionState);
}

void OmnidirectOrder2::switchRun(arp_math::UbiquityMotionState currentMotionState)
{

    if (getPositionInNormalRef(currentMotionState.getPosition()).getTVector().norm() < RO_ACCURACY)
    {
        Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_DONE " << "because in final zone ";
        m_currentMode = MODE_DONE;
        return;
    }

    testTimeout();

}

double OmnidirectOrder2::profileRo(double distance, ICRSpeed curICRSpeed)
{
    //TODO attention cette partie na jamais ete retouche depuis la refactorisation decembre 2013. les parametre utilise pour nourrir reflexxes ne sont plus les bons.

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
    Log(DEBUG) << "m_conf.LIN_VEL_MAX" << m_params.getMaxRobotSpeed();
    Log(DEBUG) << "m_conf.LIN_DEC" << m_params.getMaxRobotAccel();
    bool OTGres = OTG->computeNextStepCheap(start, end, m_params.getMaxRobotSpeed(), m_params.getMaxRobotAccel(), m_params.getMaxRobotAccel() * 5, next);
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

double OmnidirectOrder2::profileRoJerking(double distance, ICRSpeed curICRSpeed, double roPass, double dt)
{
    //TODO brancher vitesse plutot que last ro
    double curRo = m_oldICRSpeed.ro();
    //double curRo=curICRSpeed.ro();


    double ro_non_limited;

    Log(DEBUG) << "<< profileRoJerking";
    Log(DEBUG) << "m_lastRo " << m_lastRo;
    Log(DEBUG) << "distance " << distance;

    /*
     * is this a "passage point" ?
     * if yes, use v=sqrt(dist + vpass²)
     */
    if (roPass > 0.0)
    {
        ro_non_limited = sqrt2(sign(curRo) * roPass * roPass + 2.0 * DECELERATION * distance);
        Log(DEBUG) << "curRo=" << curRo << " roPass=" << roPass << "   ro_non_limited avec ropass=" << ro_non_limited;
    }
    else
        /*
         * otherwise, use v=sqrt(dist) when far, and v=k.dist when close
         */
    {
        if (abs(distance) > DISTANCE_LINEAR_ASSERV)
        {
            ro_non_limited = sqrt2(2.0 * DECELERATION) * sqrt2(distance);
            Log(DEBUG) << "ro_non_limited  racine       " << ro_non_limited;
        }
        else
        {
            ro_non_limited = (sqrt2(2.0 * DECELERATION) * sqrt2(DISTANCE_LINEAR_ASSERV)) / DISTANCE_LINEAR_ASSERV * distance;
            Log(DEBUG) << "ro_non_limited  lineaire     " << ro_non_limited;
        }
    }

    double ro_sat_speed = saturate(ro_non_limited, -m_vmax_order, m_vmax_order);
    double ro_sat_acc = ro_sat_speed;

    // limitation de l'acceleration
    double accStep = m_params.getMaxRobotAccel() * dt;
    if (ro_sat_speed > 0)
    {
        if (curRo < ro_sat_speed)
        { //accel
            ro_sat_acc = min(ro_sat_speed, curRo + accStep);
        }
        else
        { //decel
          //ro_sat_acc = max(ro_sat_speed, curRo - accStep);
        }
    }
    else
    {
        if (curRo > ro_sat_speed)
        { //"augmentation du mouvement"
            ro_sat_acc = max(ro_sat_speed, curRo - accStep);
        }
        else
        { //"diminution du mouvement"
          //ross = min(ros, curRo + accStep);
        }
    }

    Log(DEBUG) << "ros      " << ro_sat_speed;
    Log(DEBUG) << "ro_sat_acc     " << ro_sat_acc;
    Log(DEBUG) << ">> profileRoJerking";

    return ro_sat_acc;

}


ICRSpeed OmnidirectOrder2::computeRunTwist(Pose2DNorm currentPositionNorm, ICRSpeed curICRSpeed, double dt)
{
    if (curICRSpeed.getICR().sphericalDistance(m_oldICRSpeed.getICR()) > PI / 2)
        curICRSpeed = curICRSpeed.getOppositeRep();

    Log(DEBUG) << ">>computeRunTwist  ";
    Log(DEBUG) << "dt donne =  " << dt;

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
    double s_max = 20 * dt * getParkinsonLimitationFactor(Cerr.norm());

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
    double ro;
    if (m_pass == false)
        ro = profileRoJerking(k_delta, curICRSpeed, 0.0, dt);
    else
        ro = profileRoJerking(k_delta, curICRSpeed, m_passSpeed, dt);

    Log(DEBUG) << "k_delta       " << k_delta;
    Log(DEBUG) << "ro            " << ro;

    corICRSpeed = ICRSpeed(ro, ICR_possible);

    //je remet tout dans leur range nominales
    //corICRSpeed = corICRSpeed.getNormalizedRep();

    //m_lastRo = corICRSpeed.ro();

    Log(DEBUG) << "Cerr.norm()           " << Cerr.norm();

    Log(DEBUG) << "corICRSpeed           " << corICRSpeed.toString();
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
ICRSpeed OmnidirectOrder2::computeSpeed(UbiquityMotionState currentMotionState, double dt)
{

    Pose2D currentPosition = currentMotionState.getPosition();
    decideSmoothNeeded(currentPosition);
    ICRSpeed curICRSpeed = currentMotionState.getSpeed();

    if (m_currentMode == MODE_ERROR)
    {
        return ICRSpeed(0, curICRSpeed.getICR());
    }

    if (m_currentMode == MODE_DONE and m_pass == true)
    {
        return curICRSpeed;
    }

    if (m_currentMode == MODE_RUN or (m_currentMode == MODE_DONE and m_pass == false))
    {
        // curICRSpeed est dans le repere robot il faut  le mettre dans le repere target
        curICRSpeed.phi(curICRSpeed.phi() - m_endMotionState.getPosition().h() + currentPosition.h());
        //compute run twist travaille dans un espace 3D ou l'objectif est en (0,0,0)
        ICRSpeed corICRSpeed = computeRunTwist(getPositionInNormalRef(currentPosition), curICRSpeed, dt);
        //dans le repere robot
        corICRSpeed.phi(corICRSpeed.phi() + m_endMotionState.getPosition().h() - currentPosition.h());
        return corICRSpeed;
    }

    return ICRSpeed(0, 0, 0);
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
