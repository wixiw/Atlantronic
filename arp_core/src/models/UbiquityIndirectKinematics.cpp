/*
 * UbiquityIndirectKinematics.cpp
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

bool UbiquityKinematics::twist2Turrets(const Twist2D & iTw, TurretState& oTS, const UbiquityParams & iParams)
{
    if (!iParams.check())
    {
        Log(ERROR) << "UbiquityKinematics::twist2Turrets failed when checking params";
        return false;
    }
    //WARNING ! On fait quoi avec le sens ?

    //calcul des Twist de la BR sur chaque tourelle
    Twist2D Tleft = iTw.transport(iParams.getLeftTurretPosition());
    Twist2D Tright = iTw.transport(iParams.getRightTurretPosition());
    Twist2D Trear = iTw.transport(iParams.getRearTurretPosition());

    //cerr << "Tleft=" << Tleft.toString() << " Tright=" << Tright.toString() << " Trear=" << Trear.toString() << endl;

    //recuperation des angles
    oTS.steering.left.position = Tleft.speedAngle();
    oTS.steering.right.position = Tright.speedAngle();
    oTS.steering.rear.position = Trear.speedAngle();

    //recuperation des vitesses
    oTS.driving.left.velocity = Tleft.speedNorm();
    oTS.driving.right.velocity = Tright.speedNorm();
    oTS.driving.rear.velocity = Trear.speedNorm();

    //normalisation dans -PI/2 PI/2
    normalizeDirection(oTS);

    return true;
}


bool UbiquityKinematics::twist2Motors(const arp_math::Twist2D & iTw, const MotorState & iMS, TurretState& oTS,
        MotorState& oMS, const UbiquityParams & iParams)
{
    bool res = true;
    res &= twist2Turrets(iTw, oTS, iParams);
    res &= turrets2Motors(oTS, iMS, oMS, iParams);
    return res;
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
    oTS.driving.left.velocity = SpeedLeft.getTranslationSpeedNorm()*sign(iICRs.ro());
    oTS.driving.right.velocity = SpeedRight.getTranslationSpeedNorm()*sign(iICRs.ro());
    oTS.driving.rear.velocity = SpeedRear.getTranslationSpeedNorm()*sign(iICRs.ro());

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
