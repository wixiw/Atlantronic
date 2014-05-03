/*
 * OmnidirectOrder.cpp
 *
 *  Created on:  14 sept 2013
 *      Author: RMO
 */

#include "OmnidirectOrder2.hpp"
#include "ods_logger/Logger.hpp"
using namespace arp_math;
using namespace arp_ods;
using namespace orders;
using namespace arp_core::log;
using namespace boost;

OmnidirectOrder2::OmnidirectOrder2(const OrderGoalConstPtr &goal, arp_math::UbiquityMotionState currentMotionState,
        UbiquityParams params) :
        MotionOrder(goal, currentMotionState, params), m_last_PosVelAcc_computation()

{

    m_type = OMNIDIRECT2;

    m_ICRSpeed_N_1 = ICRSpeed();

    m_orderTime = 0;

    //TODO mettre la valeur par defaut
    m_dt_N_1 = 0.01;

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
        if (goal->passe_speed > 0.0 and goal->passe_speed < m_params.getMaxRobotSpeed())
        {
            endMotionState.getSpeed().ro(goal->passe_speed);
            setPassSpeed(goal->passe_speed);
        }
        else
        {
            endMotionState.getSpeed().ro(m_params.getMaxRobotSpeed());
            setPassSpeed(m_params.getMaxRobotSpeed());
        }
    }

    setEndMotionState(endMotionState);

    if (goal->max_speed > 0.0 and goal->max_speed < m_params.getMaxRobotSpeed())
        m_vmax_order = goal->max_speed;
    else
        m_vmax_order = m_params.getMaxRobotSpeed();

    // timeout is initialised by a rough distance / speed with a margin
    m_timeout = max(end.distanceTo(currentMotionState.position) / m_vmax_order * 2.0, 15.0);

    Log(DEBUG) << "goal->max_speed " << goal->max_speed;
    Log(DEBUG) << "m_params.getMaxRobotSpeed() " << m_params.getMaxRobotSpeed();
    Log(DEBUG) << "m_vmax_order " << m_vmax_order;
    Log(DEBUG) << "end " << end.toString();
    Log(DEBUG) << "tcurrentMotionState.position " << currentMotionState.position.toString();
    Log(DEBUG) << "end.distanceTo(currentMotionState.position) " << end.distanceTo(currentMotionState.position);

    Log(INFO) << getTypeString() << " from [" << ((Pose2D) (getBeginMotionState().getPosition())).toString() << "] to ["
            << ((Pose2D) (getEndMotionState().getPosition())).toString() << "] control_point=[" << cpoint.toString()
            << "]" << "pass = " << m_pass << " @ " << m_passSpeed << " m/s";

}

void OmnidirectOrder2::switchInit(arp_math::UbiquityMotionState currentMotionState)
{
    Pose2D currentPosition = currentMotionState.getPosition();
    ICRSpeed curICRSpeed = currentMotionState.getSpeed();

    //attention curICRspeed doit s'exprimer dans le bon repere
    //TODO mettre ca d'equerre car un calcul identique est fait plus loin !
    curICRSpeed.phi(curICRSpeed.phi() - m_endMotionState.getPosition().h() + currentPosition.h());

    m_ICRSpeed_N_1 = curICRSpeed;
    //TODO mettre la valeur par defaut
    m_dt_N_1 = 0.01;

    m_last_PosVelAcc_computation.position = 0;
    m_last_PosVelAcc_computation.velocity = curICRSpeed.ro();
    m_last_PosVelAcc_computation.acceleration = 0;

    Log(DEBUG) << "---curICRSpeed initialise " << m_ICRSpeed_N_1.toString();
    MotionOrder::switchInit(currentMotionState);
}

void OmnidirectOrder2::switchRun(arp_math::UbiquityMotionState currentMotionState)
{

    Log(DEBUG) << ">>switchRun  ";

    if (getPositionInNormalRef(currentMotionState.getPosition()).getTVector().norm() < RO_ACCURACY and m_pass == false)
    {
        Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_DONE " << "because in final zone for non pass ";
        m_currentMode = MODE_DONE;
        return;
    }
    if (getPositionInNormalRef(currentMotionState.getPosition()).getTVector().norm() < RO_ACCURACY * 4.0
            and m_pass == true)
    {
        Log(INFO) << getTypeString() << " switched MODE_RUN --> MODE_DONE " << "because in final zone for pass ";
        m_currentMode = MODE_DONE;

        return;
    }
    Log(DEBUG) << "<<switchRun  ";

    testTimeout();

}

PosVelAcc OmnidirectOrder2::profileRoNonJerking(PosVelAcc start, PosVelAcc end, double maxSpeed, double dt)
{
    Log(DEBUG) << "     >>profileRoNonJerking";

    Log(DEBUG) << "          m_ICRSpeed_N_1.ro()          " << m_ICRSpeed_N_1.ro();
    Log(DEBUG) << "          dt                        " << dt;

    double maxAcc = m_params.getMaxRobotAccel();
    double maxJerk = m_params.getMaxRobotJerk();

    PosVelAcc next;
    PosVelAcc nextnext;
    double ro_N = 0;
    bool OTGres = false;

    if (OTG == NULL)
    {
        ro_N = 0;
        Log(DEBUG) << "          *** PB sur calcul RO : pointeur null sur OTG ****";
        return ro_N;
    }
    Log(DEBUG) << "          start.position       " << start.position;
    Log(DEBUG) << "          start.velocity       " << start.velocity;
    Log(DEBUG) << "          start.acceleration   " << start.acceleration;
    Log(DEBUG) << "          end.position         " << end.position;
    Log(DEBUG) << "          end.velocity         " << end.velocity;
    Log(DEBUG) << "          end.acceleration     " << end.acceleration;
    Log(DEBUG) << "          maxSpeed             " << maxSpeed;
    Log(DEBUG) << "          maxAcc               " << maxAcc;
    Log(DEBUG) << "          maxJerk              " << maxJerk;

    OTGres = OTG->computeNextStep(start, end, maxSpeed, maxAcc, maxJerk, next);
    //pour tests
    //OTGres = OTG->computeNextStep(next, end, maxSpeed, maxAcc, maxJerk, nextnext);

    Log(DEBUG) << "          CALCUL REFLEXXES   ...";
    Log(DEBUG) << "          next.position         " << next.position;
    Log(DEBUG) << "          next.velocity         " << next.velocity;
    Log(DEBUG) << "          next.acceleration     " << next.acceleration;

    /*
     outDEBUG1 = start.velocity;
     outDEBUG2 = start.acceleration;
     outDEBUG3 = end.position;
     outDEBUG4 = end.velocity;
     outDEBUG5 = maxSpeed;
     outDEBUG6=next.acceleration;
     */

    if (!OTGres)
    {
        next.position = 0;
        next.velocity = 0;
        next.acceleration = 0;
        Log(DEBUG) << "          *** PB sur calcul RO: reflexxes a retourne false ****";

        //TODO ceci est ineaceptable mais fat bien sauver le bateau. Reflexxes renvoie des erreurs de synchronisation en monoaxe.. ???

        next = profileRoJerking(start, end, maxSpeed, dt);
    }

    Log(DEBUG) << "     <<profileRoNonJerking";

    return next;

}

PosVelAcc OmnidirectOrder2::profileRoJerking(PosVelAcc start, PosVelAcc end, double maxSpeed, double dt)
{
    PosVelAcc next;
    double maxAcc = m_params.getMaxRobotAccel();

    /*
     * NB: la vitesse est toujours positive
     * la position peut etre negative
     * la vitesse renvoyee peut etre negative
     */

    double ro_N;

    Log(DEBUG) << "     << profileRoJerking";
    Log(DEBUG) << "          distance " << end.position;

    /** CONSTRUCTION VITESSE **/

    /*
     * is this a "passage point" ?
     * if yes, use v=sqrt(dist + vpass²)
     */
    if (end.velocity > 0.0)
    {
        ro_N = sqrt2(sign(start.velocity) * end.velocity * end.velocity + 2.0 * DECELERATION * end.position);
        Log(DEBUG) << "     start.velocity=" << start.velocity << " roPass=" << end.velocity
                << "   ro_non_limited avec ropass=" << ro_N;
    }
    else
    /*
     * otherwise, use v=sqrt(dist) when far, and v=k.dist when close
     */
    {
        if (abs(end.position) > DISTANCE_LINEAR_ASSERV)
        {
            ro_N = sqrt2(2.0 * DECELERATION) * sqrt2(end.position);
            Log(DEBUG) << "          ro_non_limited  racine       " << ro_N;
        }
        else
        {
            ro_N = (sqrt2(2.0 * DECELERATION) * sqrt2(DISTANCE_LINEAR_ASSERV)) / DISTANCE_LINEAR_ASSERV * end.position;
            Log(DEBUG) << "          ro_non_limited  lineaire     " << ro_N;
        }
    }
    /*
     * fin de la construction du ro juste pointant la fin
     */
    Log(DEBUG) << "          ro_N non limite      " << ro_N;

    /** SATURATION EN VITESSE **/

    ro_N = saturate(ro_N, -maxSpeed, maxSpeed);

    /*
     * fin de la construction du ro limite en vitesse
     */
    Log(DEBUG) << "          ro_N limit speed      " << ro_N;

    /** SATURATION EN ACCELERATION **/

    // limitation de l'acceleration
    double accStep = maxAcc * dt;

    if (ro_N > 0)
    {
        if (start.velocity < ro_N)
        { //accel
            ro_N = min(ro_N, start.velocity + accStep);
        }
        else
        { //decel
          //ro_sat_acc = max(ro_sat_speed, curRo - accStep);
        }
    }
    else
    {
        if (start.velocity > ro_N)
        { //"augmentation du mouvement"
            ro_N = max(ro_N, start.velocity - accStep);
        }
        else
        { //"diminution du mouvement"
          //ross = min(ros, curRo + accStep);
        }
    }

    /*
     * fin de la construction du ro limite en acceleration
     */
    Log(DEBUG) << "          ro_N limit acc      " << ro_N;

    next.position = start.position + ro_N * dt;
    next.velocity = ro_N;
    next.acceleration = (ro_N - start.velocity) / dt;

    Log(DEBUG) << "          start.position       " << start.position;
    Log(DEBUG) << "          start.velocity       " << start.velocity;
    Log(DEBUG) << "          start.acceleration   " << start.acceleration;
    Log(DEBUG) << "          end.position         " << end.position;
    Log(DEBUG) << "          end.velocity         " << end.velocity;
    Log(DEBUG) << "          end.acceleration     " << end.acceleration;
    Log(DEBUG) << "          maxSpeed             " << maxSpeed;
    Log(DEBUG) << "          maxAcc               " << maxAcc;

    Log(DEBUG) << "          CALCUL ROMEXXES   ...";
    Log(DEBUG) << "          next.position         " << next.position;
    Log(DEBUG) << "          next.velocity         " << next.velocity;
    Log(DEBUG) << "          next.acceleration     " << next.acceleration;

    Log(DEBUG) << "     >> profileRoJerking";
    return next;

}

ICRSpeed OmnidirectOrder2::computeRunTwist(Pose2DNorm currentPositionNorm, ICRSpeed curICRSpeed, double dt)
{

    m_orderTime += dt;

    Log(DEBUG) << ">>computeRunTwist  ";
    Log(DEBUG) << "     m_orderTime =  " << m_orderTime;
    Log(DEBUG) << "     dt donne =  " << dt;

    Log(DEBUG) << "     curICRSpeed a la base    " << curICRSpeed.toString();

    //curICRspeed est toujours positif en ro
    curICRSpeed = m_ICRSpeed_N_1.getNormalizedRep();

    //curICRSpeed = ICRSpeed(vraiRo,m_ICRSpeed_N_1.phi(),m_ICRSpeed_N_1.delta());

    /*
     if (curICRSpeed.getICR().sphericalDistance(m_oldICRSpeed.getICR()) > PI )
     curICRSpeed = curICRSpeed.getOppositeRep();
     */

    Log(DEBUG) << "     currentPositionNorm  " << currentPositionNorm.toString();
    Log(DEBUG) << "     curICRSpeed apres modif  " << curICRSpeed.toString();

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
    ICR curICR = curICRSpeed.getICR();
    //ICR curICR = m_oldICRSpeed.getICR();

    // gestion du parkinson final
    double s_max = 5 * dt * getParkinsonLimitationFactor(Cerr.norm());

    //double s_max = 5 * dt ;

    Log(DEBUG) << "     Cerr.norm()                               " << Cerr.norm();
    //Log(DEBUG) << "     getParkinsonLimitationFactor(Cerr.norm()) " << getParkinsonLimitationFactor(Cerr.norm());
    //Log(DEBUG) << "     s_max                                     " << s_max;

    double distanceICRMove = ICR_perfect.sphericalDistance(curICR);

    Log(DEBUG) << "     ICR_perfect   de baz                      " << ICR_perfect.toString();
    //Log(DEBUG) << "     distanceICRMove                           " << distanceICRMove;
    if (distanceICRMove > PI / 2)
    {
        Log(DEBUG) << "     !!!c'etait plus cours de rejoindre l'antipode alors je pars en marche arriere     ";
        ICR_perfect = ICR_perfect.getAntipodICR();
        //Log(DEBUG) << "     ICR_perfect   modif    " << ICR_perfect.toString();
    }
    else
    {
        Log(DEBUG) << "     il etait du bon coté     ";
    }
    distanceICRMove = ICR_perfect.sphericalDistance(curICR);
    Log(DEBUG) << "     distanceICRMove                           " << distanceICRMove;

    ICR ICR_possible = curICR.getIntermediate(ICR_perfect, s_max);

    Log(DEBUG) << "     curICR             " << curICR.toString();
    Log(DEBUG) << "     ICR_possible       " << ICR_possible.toString();

    //corICRSpeed = ICRSpeed(1.0, ICR_possible);

    //double k_delta = Cerr.dot(corICRSpeed.speedDirection());
    double k_delta = Cerr.dot(curICRSpeed.speedDirection());
    //TODO donner la vitesse de passage
    double ro;

    /*
     * if the distance to move the ICR is too big i need to limit my speed
     */
    Log(DEBUG) << "     k_delta       " << k_delta;

    double vmax = getSpeedLimitationFactor(distanceICRMove) * m_params.getMaxRobotSpeed();
    //double vmax = 1.0* m_params.getMaxRobotSpeed();

    Log(DEBUG) << "     speedlimitation factor                        " << getSpeedLimitationFactor(distanceICRMove);
    Log(DEBUG) << "     vmax    " << vmax;

    double passSpeed;
    if (m_pass == false)
        passSpeed = 0.0;
    else
        passSpeed = m_passSpeed;

    /*
     * ici on choisit d'appeler relfexxes ou moulineau (profileRoJerking) en fonction de la distance
     *
     */

    PosVelAcc start;
    start.position = 0;
    start.velocity = m_last_PosVelAcc_computation.velocity;
    // quand reflexxes me sort l'acceleration du tour suivant, il voit deja dans le futur ! je ne peux pas dervier la vitesse passee
    if (m_last_PosVelAcc_computation.velocity >= 0.0)
    {
        start.velocity = m_last_PosVelAcc_computation.velocity;
        start.acceleration = m_last_PosVelAcc_computation.acceleration;
    }
    else
    {
        start.velocity = -m_last_PosVelAcc_computation.velocity;
        start.acceleration = -m_last_PosVelAcc_computation.acceleration;
    }

    PosVelAcc end;
    end.position = k_delta;
//TODO creer un min(1,2,3) et min(1,2,3,4)
    double velocity_allowingAcceleration = sqrt(
            start.velocity * start.velocity
                    + 2 * m_params.getMaxRobotAccel() * 0.5 * abs(end.position - start.position));
    Log(DEBUG) << "     velocity_allowingAcceleration " << velocity_allowingAcceleration;
    end.velocity = min(min(m_vmax_order, min(passSpeed, vmax)), velocity_allowingAcceleration);
    end.acceleration = 0;

//TODO
// ceci devrait aussi utiliser m_vmax_asked
// a rebranche : min (m_vmax_order,m_vmax_asked)
    double maxSpeed = min(m_vmax_order, vmax);
    /*
     if (start.velocity < 0)
     {
     start.position = -start.position;
     start.velocity = -start.velocity;
     start.acceleration = -start.acceleration;
     end.position = -end.position;
     end.velocity = -end.velocity;
     end.acceleration = -end.acceleration;
     }
     */
    PosVelAcc next;

//TODO: attention a cette condition
//if (Cerr.norm() > DISTANCE_LINEAR_ASSERV)
    if (false)
    {
        //end.position = end.position * 0.8;
        end.position = end.position - start.velocity * 0.060;
        next = profileRoNonJerking(start, end, maxSpeed, dt);
    }
    else
    {
        end.position = end.position - start.velocity * 0.030;
        next = profileRoJerking(start, end, maxSpeed, dt);
    }

    m_last_PosVelAcc_computation = next;

    ro = next.velocity;
    Log(DEBUG) << "     ro            " << ro;

    corICRSpeed = ICRSpeed(ro, ICR_possible);

//je remet tout dans leur range nominales
    corICRSpeed = corICRSpeed.getNormalizedRep();

    Log(DEBUG) << "     Cerr.norm()           " << Cerr.norm();

    Log(DEBUG) << "     corICRSpeed           " << corICRSpeed.toString();
    Log(DEBUG) << "<<computeRunTwist     ";
    Log(DEBUG) << "                      ";

    m_ICRSpeed_N_1 = corICRSpeed;
    m_dt_N_1 = dt;

//for debug

    outDEBUG1 = Cerr.norm();
    outDEBUG2 = corICRSpeed.ro();
    outDEBUG3 = rad2deg(corICRSpeed.phi()) / 100.0;
    outDEBUG4 = rad2deg(corICRSpeed.delta()) / 100.0;
//outDEBUG5 = end.velocity;
//outDEBUG6 = start.acceleration;

    outDEBUG7 = m_orderTime;

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

//outDEBUG5 = m_currentMode;

    if (m_currentMode == MODE_DONE and m_pass==true)
    {
        ICRSpeed corICRSpeed;

        corICRSpeed = ICRSpeed(max(m_ICRSpeed_N_1.ro() - m_params.getMaxRobotAccel() * 0.2 * dt, 0.0),
                m_ICRSpeed_N_1.getICR());

        if (m_pass == true or (m_currentMode == MODE_DONE and m_pass == true))
        {
            corICRSpeed = ICRSpeed(max(m_ICRSpeed_N_1.ro() - m_params.getMaxRobotAccel() * 0.2 * dt, 0.0),
                    m_ICRSpeed_N_1.getICR());

        }
        else
        {
            m_ICRSpeed_N_1.ro(max(m_ICRSpeed_N_1.ro() - dt * m_params.getMaxRobotAccel() * 0.5, 0.0));
            corICRSpeed = m_ICRSpeed_N_1;
            //corICRSpeed = ICRSpeed(0.0, m_ICRSpeed_N_1.getICR());

        }

        m_ICRSpeed_N_1 = corICRSpeed;

        outDEBUG2 = corICRSpeed.ro();
        outDEBUG3 = rad2deg(corICRSpeed.phi()) / 100.0;
        outDEBUG4 = rad2deg(corICRSpeed.delta()) / 100.0;

        Log(DEBUG) << "mode DONE mais j'ai quand meme envoye corICRSpeed:" << corICRSpeed;

        return corICRSpeed;

    }

    if ((m_currentMode == MODE_RUN) or (m_currentMode == MODE_DONE and m_pass == false))
    {
        // curICRSpeed est dans le repere robot il faut  le mettre dans le repere target
        curICRSpeed.phi(curICRSpeed.phi() - m_endMotionState.getPosition().h() + currentPosition.h());
        //compute run twist travaille dans un espace 3D ou l'objectif est en (0,0,0)
        ICRSpeed corICRSpeed = computeRunTwist(getPositionInNormalRef(currentPosition), curICRSpeed, dt);
        //dans le repere robot
        corICRSpeed.phi(corICRSpeed.phi() + m_endMotionState.getPosition().h() - currentPosition.h());

        /*
         if (m_currentMode == MODE_DONE and m_pass == false)
         {
         //la fin d'un ordre non passage
         corICRSpeed=ICRSpeed(corICRSpeed.ro(),m_ICRSpeed_N_1.getICR());
         //bon attention c'est un peu n'imp, le m_icrspeedn_1 est exprimé dans un autre repere que corICRspeed. mais comme on l'utilise juste pour faire des copies d'icr ca se voit pas. gaffe !
         m_ICRSpeed_N_1 = corICRSpeed;
         }
         */

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
    /*
     double freeze_distance = 0.010;
     double noConstraint_distance = 0.1;
     return smoothStep(distance, 0.1, freeze_distance, 1, noConstraint_distance);
     */
    if (distance > RO_ACCURACY)
        return 1;
    else
        return 0;
}

double OmnidirectOrder2::getSpeedLimitationFactor(double distanceICRMove)
{
    double nopb_distance = PI / 10;
    double dontmove_distance = PI / 4;
    return smoothStep(distanceICRMove, 1, nopb_distance, 0.01, dontmove_distance);
}
