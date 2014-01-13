/*
 * UbiquityKinematics.cpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#include "UbiquityKinematics.hpp"
#include "models/Logger.hpp"
#include <iostream>
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;
using namespace arp_math;
using namespace arp_model;
using namespace arp_core::log;

UbiquityKinematics::UbiquityKinematics()
{
}

bool UbiquityKinematics::motors2Turrets(const MotorState & iMS, TurretState& oTS, const UbiquityParams & iParams)
{
    if (!iParams.check())
    {
        Log(ERROR) << "UbiquityKinematics::motors2Turrets failed when checking params";
        return false;
    }

    //Steering position
    oTS.steering.left.position = (iMS.steering.left.position - iParams.getLeftTurretZero()) * iParams.getTurretRatio();
    oTS.steering.right.position = (iMS.steering.right.position - iParams.getRightTurretZero())
            * iParams.getTurretRatio();
    oTS.steering.rear.position = (iMS.steering.rear.position - iParams.getRearTurretZero()) * iParams.getTurretRatio();

    //Steering velocity
    oTS.steering.left.velocity = iMS.steering.left.velocity * iParams.getTurretRatio();
    oTS.steering.right.velocity = iMS.steering.right.velocity * iParams.getTurretRatio();
    oTS.steering.rear.velocity = iMS.steering.rear.velocity * iParams.getTurretRatio();

    //Driving velocity
    oTS.driving.left.velocity = iParams.getLeftWheelDiameter() / 2
            * (iMS.driving.left.velocity * iParams.getTractionRatio()
                    - iMS.steering.left.velocity * iParams.getTurretRatio());
    oTS.driving.right.velocity = iParams.getRightWheelDiameter() / 2
            * (iMS.driving.right.velocity * iParams.getTractionRatio()
                    - iMS.steering.right.velocity * iParams.getTurretRatio());
    oTS.driving.rear.velocity = iParams.getRearWheelDiameter() / 2
            * (iMS.driving.rear.velocity * iParams.getTractionRatio()
                    - iMS.steering.rear.velocity * iParams.getTurretRatio());

    //normalisations
    oTS.steering.left.position = betweenMinusPiAndPlusPi(oTS.steering.left.position);
    oTS.steering.right.position = betweenMinusPiAndPlusPi(oTS.steering.right.position);
    oTS.steering.rear.position = betweenMinusPiAndPlusPi(oTS.steering.rear.position);

    return true;
}

bool UbiquityKinematics::turrets2Motors(const TurretState & iTSbrut, const MotorState & iMS, MotorState& oMS,
        const UbiquityParams & iParams)
{
    if (!iParams.check())
    {
        Log(ERROR) << "UbiquityKinematics::turrets2Motors failed when checking params";
        return false;
    }
    //normalisation des entrees pour travailler dans [-PI;PI]*R
    TurretState iTS = iTSbrut;
    iTS.steering.left.position = betweenMinusPiAndPlusPi(iTS.steering.left.position);
    iTS.steering.right.position = betweenMinusPiAndPlusPi(iTS.steering.right.position);
    iTS.steering.rear.position = betweenMinusPiAndPlusPi(iTS.steering.rear.position);

    //optimisation des consignes,gestion du pilotage mumtitour et gestion du zero

    //calcul de la position courante des tourelles à partir de l'état des moteurs
    // currentTurretPos est dans [-Pi;Pi]
    TurretState currentTurretPos;
    motors2Turrets(iMS, currentTurretPos, iParams);

    //calcul des delta de position commandé
    TurretState deltaCmd = iTS - currentTurretPos;
    deltaCmd.driving = iTS.driving; //pas de delta sur la vitesse
    normalizeDirection(deltaCmd);
    iTS.driving = deltaCmd.driving;

    //ajout a la position brute du moteur le delta de commande, et paf ca fait la consigne a envoyer au prochain step
    oMS.steering.left.position = iMS.steering.left.position
            + deltaCmd.steering.left.position / iParams.getTurretRatio();
    oMS.steering.right.position = iMS.steering.right.position
            + deltaCmd.steering.right.position / iParams.getTurretRatio();
    oMS.steering.rear.position = iMS.steering.rear.position
            + deltaCmd.steering.rear.position / iParams.getTurretRatio();

    //mise a jour des vitesses de traction
    oMS.driving.left.velocity = (2 * iTS.driving.left.velocity / iParams.getLeftWheelDiameter()
            + iMS.steering.left.velocity * iParams.getTurretRatio()) / iParams.getTractionRatio();
    oMS.driving.right.velocity = (2 * iTS.driving.right.velocity / iParams.getRightWheelDiameter()
            + iMS.steering.right.velocity * iParams.getTurretRatio()) / iParams.getTractionRatio();
    oMS.driving.rear.velocity = (2 * iTS.driving.rear.velocity / iParams.getRearWheelDiameter()
            + iMS.steering.rear.velocity * iParams.getTurretRatio()) / iParams.getTractionRatio();

    //Log(DEBUG) << "iTS : " << iTS.toString();
    //Log(DEBUG) << "currentTurretPos : " << currentTurretPos.toString();
    //Log(DEBUG) << "deltaCmd : " << deltaCmd.toString();
    //Log(DEBUG) <<  "oMS : " << oMS.toString();
    return true;
}

bool UbiquityKinematics::turrets2ICRspeed(const TurretState & iTS, ICRSpeed& oICRs, SlippageReport& oSR,
        const UbiquityParams & iParams)
{
    //TODO implementer le cas complexe
    return simpleTurrets2ICRspeed(iTS, oICRs, oSR, iParams);
}

bool UbiquityKinematics::simpleTurrets2ICRspeedWithTwist(const TurretState & iTS, ICRSpeed& oICRs,SlippageReport& oSR,
        const UbiquityParams & iParams)
{
    //TODO faire un truc generique et mathematiquement propre a la boris

    //definition of lines perpendicular to turrets
    Vector2 pLeft1(iParams.getLeftTurretPosition().x(), iParams.getLeftTurretPosition().y());
    Vector2 deltaLeft(cos(iTS.steering.left.position + PI / 2), sin(iTS.steering.left.position + PI / 2));
    Vector2 pLeft2 = pLeft1 + deltaLeft;

    Vector2 pRight1(iParams.getRightTurretPosition().x(), iParams.getRightTurretPosition().y());
    Vector2 deltaRight(cos(iTS.steering.right.position + PI / 2), sin(iTS.steering.right.position + PI / 2));
    Vector2 pRight2 = pRight1 + deltaRight;

    Vector2 pRear1(iParams.getRearTurretPosition().x(), iParams.getRearTurretPosition().y());
    Vector2 deltaRear(cos(iTS.steering.rear.position + PI / 2), sin(iTS.steering.rear.position + PI / 2));
    Vector2 pRear2 = pRear1 + deltaRear;

    //speeds
    Vector2 leftSpeed(cos(iTS.steering.left.position) * iTS.driving.left.velocity,
            sin(iTS.steering.left.position) * iTS.driving.left.velocity);
    Vector2 rightSpeed(cos(iTS.steering.right.position) * iTS.driving.right.velocity,
            sin(iTS.steering.right.position) * iTS.driving.right.velocity);
    Vector2 rearSpeed(cos(iTS.steering.rear.position) * iTS.driving.rear.velocity,
            sin(iTS.steering.rear.position) * iTS.driving.rear.velocity);

    // intersection of lines
    Vector2 ICRLeftRight;
    bool parLeftRight;
    bool colLeftRight;
    linesIntersection(pLeft1, pLeft2, pRight1, pRight2, 1e-6, ICRLeftRight, parLeftRight, colLeftRight);

    Vector2 ICRLeftRear;
    bool parLeftRear;
    bool colLeftRear;
    linesIntersection(pLeft1, pLeft2, pRear1, pRear2, 1e-6, ICRLeftRear, parLeftRear, colLeftRear);

    Vector2 ICRRightRear;
    bool parRightRear;
    bool colRightRear;
    linesIntersection(pRight1, pRight2, pRear1, pRear2, 1e-6, ICRRightRear, parRightRear, colRightRear);

    //now we have the 3 intersections. there might be parralel or colinear turrets.

    //////////////////////////SIMPLIFIED VERSION: takes  only 2 turrets to work

    //if we have a speed then use the standard case
    Twist2D oTw;
    simpleTurrets2Twist(iTS, oTw, oSR, iParams);
    if (abs(oTw.vx()) > 0.001 or abs(oTw.vy()) > 0.001 or abs(oTw.vh()) > 0.001)
    {
        oICRs = ICRSpeed(oTw);
    }
    else
    {
        if (colLeftRight) // front turrets are colinear
        {
            if (parLeftRear) // the third is parralel also
            {
                oICRs = ICRSpeed::createIdleFromTranslation(iTS.steering.rear.position + PI / 2);

            }
            else // the third turret is not parralel: we are turning around a point on the front turrets line
            {
                oICRs = ICRSpeed::createIdleFromICRPosition(ICRRightRear);

            }
        }
        else if (parLeftRight) // front turrets are parralel but not colinear, the robot is in translation
        {
            oICRs = ICRSpeed::createIdleFromTranslation(iTS.steering.left.position + PI / 2);
        }

        else // standard case: the front turrets are crossing
        {
            oICRs = ICRSpeed::createIdleFromICRPosition(ICRLeftRight);
        }
    }
    return true;
}

bool UbiquityKinematics::simpleTurrets2ICRspeedWithICR(const TurretState & iTS, arp_math::ICRSpeed& oICRs, SlippageReport& oSR,
        const UbiquityParams & iParams)
{
    bool res = true;

    if (!iParams.check())
    {
        Log(ERROR) << "UbiquityKinematics::twist2Turrets failed when checking params";
        return false;
    }

    //////////////////////////SIMPLIFIED VERSION: takes  only 3 turrets to work :
    // Front turrets are giving the ICR position, the rear one is giving the speed

    //definition of lines perpendicular to turrets
    Vector2 pLeft1(iParams.getLeftTurretPosition().x(), iParams.getLeftTurretPosition().y());
    Vector2 deltaLeft(cos(iTS.steering.left.position + PI / 2), sin(iTS.steering.left.position + PI / 2));
    Vector2 pLeft2 = pLeft1 + deltaLeft;

    Vector2 pRight1(iParams.getRightTurretPosition().x(), iParams.getRightTurretPosition().y());
    Vector2 deltaRight(cos(iTS.steering.right.position + PI / 2), sin(iTS.steering.right.position + PI / 2));
    Vector2 pRight2 = pRight1 + deltaRight;

    //compute intersection
    Vector2 ICRPosition;
    bool parLeftRight;
    bool colLeftRight;
    linesIntersection(pLeft1, pLeft2, pRight1, pRight2, 1e-6, ICRPosition, parLeftRight, colLeftRight);

    //cout << "linesIntersection in : " << pLeft1 << " | " << pLeft2 << " | " << pRight1 << " | " << pRight2 << endl;
    //cout << "linesIntersection out : " << ICRPosition << " | "<< parLeftRight << " | "<< colLeftRight << endl;

    ICR icr;
    double IcrDistanceFromRearTurret;
    double ro;

    //si les 2 tourelles avant sont parralleles
    if(parLeftRight)
    {
        //si les tourelles sont en configuration differentielle et qu'il n'y a pas translation
        if( iTS.steering.left.position == 0.0 && iTS.driving.left.velocity != iTS.driving.right.velocity )
        {
            double E = iParams.getLeftTurretPosition().distanceTo(iParams.getRightTurretPosition());
            IcrDistanceFromRearTurret = E*(iTS.driving.right.velocity + iTS.driving.left.velocity)/(iTS.driving.right.velocity - iTS.driving.left.velocity);
            //cas de la rotation pure
            if(IcrDistanceFromRearTurret==0)
            {
                ro = (iTS.driving.right.velocity - iTS.driving.left.velocity)/(2*E);
                icr = ICR(0,sign(ro));
            }
            //CIR aligné sur les 2 tourelles avant
            else
            {
                ro = iTS.driving.rear.velocity*ICRPosition.norm()/(IcrDistanceFromRearTurret*cos(icr.delta()));
                icr = ICR(0,0);
            }
        }
        //sinon c'est une translation
        else
        {
            icr = ICR(iTS.steering.left.position,0);
            IcrDistanceFromRearTurret = -666;
            ro = iTS.driving.rear.velocity;
        }
    }
    else
    {
        icr = ICR(Vector3(ICRPosition[0],ICRPosition[1],Twist2DNorm::dmax));
        IcrDistanceFromRearTurret = Vector2(iParams.getRearTurretPosition().x()-ICRPosition[0] , iParams.getRearTurretPosition().y()-ICRPosition[1]).norm();
        ro = iTS.driving.rear.velocity*ICRPosition.norm()/(IcrDistanceFromRearTurret*cos(icr.delta()));
    }

    oICRs = ICRSpeed(ro, icr);

    //cout << "ICR " << oICRs << "distance from turret : " << IcrDistanceFromRearTurret << endl;

    //cohérence : calcul inverse à partir de l'ICRSpeed trouvée
    TurretState computedTurretStates;
    res &= ICRSpeed2Turrets(oICRs, computedTurretStates, iParams);

    oSR.coherency1 = arp_math::betweenMinusPiAndPlusPi(computedTurretStates.steering.rear.position - iTS.steering.rear.position);
    oSR.coherency2 = computedTurretStates.driving.left.velocity - iTS.driving.left.velocity;
    oSR.coherency3 = computedTurretStates.driving.right.velocity - iTS.driving.right.velocity;

    Log(DEBUG) << oICRs.toString();
    Log(DEBUG) << oSR.toString();

    return true;
}

bool UbiquityKinematics::simpleTurrets2ICRspeed(const TurretState & iTS, ICRSpeed& oICRs, SlippageReport& oSR,
        const UbiquityParams & iParams)
{
    //return simpleTurrets2ICRspeedWithICR(iTS, oICRs, oSR, iParams);
    return simpleTurrets2ICRspeedWithTwist(iTS, oICRs, oSR, iParams);
}

bool UbiquityKinematics::ICRSpeed2Turrets(const arp_math::ICRSpeed& iICRs, TurretState& oTS, const UbiquityParams & iParams)
{
    if (!iParams.check())
    {
        Log(ERROR) << "UbiquityKinematics::twist2Turrets failed when checking params";
        return false;
    }

    //calcul des Twist de la BR sur chaque tourelle
    ICRSpeed SpeedLeft = iICRs.transport(iParams.getLeftTurretPosition());
    ICRSpeed SpeedRight = iICRs.transport(iParams.getRightTurretPosition());
    ICRSpeed SpeedRear = iICRs.transport(iParams.getRearTurretPosition());

    //recuperation des angles (pas de modulo c'est fait plus bas) = direction perpendiculaire aux directions du CIR en chaque tourelle
    oTS.steering.left.position = SpeedLeft.phi();
    oTS.steering.right.position = SpeedRight.phi();
    oTS.steering.rear.position = SpeedRear.phi();

    //recuperation des vitesses
    oTS.driving.left.velocity = SpeedLeft.getTranslationSpeedNorm();
    oTS.driving.right.velocity = SpeedRight.getTranslationSpeedNorm();
    oTS.driving.rear.velocity = SpeedRear.getTranslationSpeedNorm();

    //normalisation dans -PI/2 PI/2
    normalizeDirection(oTS);

    return true;
}

bool UbiquityKinematics::ICRSpeed2Motors(const arp_math::ICRSpeed& iICRs, const MotorState & iMS, TurretState& oTS,
        MotorState& oMS, const UbiquityParams & iParams)
{
    bool res = true;
    res &= ICRSpeed2Turrets(iICRs, oTS, iParams);
    res &= turrets2Motors(oTS, iMS, oMS, iParams);
    return res;

}

bool UbiquityKinematics::motors2ICRSpeed(const MotorState & iMS, TurretState& oTS, ICRSpeed& oICRs, SlippageReport& oSR,
        const UbiquityParams & iParams)
{
    bool res = true;
    res &= motors2Turrets(iMS, oTS, iParams);
    turrets2ICRspeed(oTS, oICRs, oSR, iParams);
    return res;
}

void UbiquityKinematics::normalizeDirection(UbiquityKinematicState& state)
{
    normalizeDirection(state.steering.left.position, state.driving.left.velocity);
    normalizeDirection(state.steering.right.position, state.driving.right.velocity);
    normalizeDirection(state.steering.rear.position, state.driving.rear.velocity);
}

void UbiquityKinematics::normalizeDirection(double& angle, double& speed)
{
    //modulo 2PI pour commencer
    angle = betweenMinusPiAndPlusPi(angle);

    if (angle <= -M_PI_2 || angle > M_PI_2)
    {
        if (angle < 0)
            angle += M_PI;
        else
            angle -= M_PI;

        speed *= -1;
    }
}

