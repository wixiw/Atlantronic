/*
 * UbiquityKinematics.cpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#include "UbiquityKinematics.hpp"
#include "models/Logger.hpp"
#include <iostream>
#include <iostream>
#include <Eigen/SVD>

using namespace std;
using namespace arp_math;
using namespace arp_model;
using namespace arp_core::log;

UbiquityKinematics::UbiquityKinematics()
{
    // TODO Auto-generated constructor stub

}

bool UbiquityKinematics::motors2Turrets(const MotorState & iMS,
                                        TurretState& oTS,
                                        const UbiquityParams & iParams)
{
    if( !iParams.check() )
        return false;

    oTS.steering.left.position = iMS.steering.left.position*iParams.getTurretRatio()
            + iParams.getLeftTurretZero();
    oTS.steering.right.position = iMS.steering.right.position*iParams.getTurretRatio()
            + iParams.getRightTurretZero();
    oTS.steering.rear.position = iMS.steering.rear.position*iParams.getTurretRatio()
            + iParams.getRearTurretZero();

    oTS.driving.left.velocity = iParams.getLeftWheelDiameter()/2
            *(iMS.driving.left.velocity*iParams.getTractionRatio() + iMS.steering.left.velocity*iParams.getTurretRatio());
    oTS.driving.right.velocity = iParams.getRightWheelDiameter()/2
            *(iMS.driving.right.velocity*iParams.getTractionRatio() + iMS.steering.right.velocity*iParams.getTurretRatio());
    oTS.driving.rear.velocity = iParams.getRearWheelDiameter()/2
            *(iMS.driving.rear.velocity*iParams.getTractionRatio() + iMS.steering.rear.velocity*iParams.getTurretRatio());

    //normalisations
    normalizeDirection(oTS.steering.left.position, oTS.driving.left.velocity);
    normalizeDirection(oTS.steering.right.position, oTS.driving.right.velocity);
    normalizeDirection(oTS.steering.rear.position, oTS.driving.rear.velocity);

    return true;
}

bool UbiquityKinematics::turrets2Motors(const TurretState & iTS,
                                        const AxesGroup & iSteeringState,
                                        MotorState& oMS,
                                        const UbiquityParams & iParams)
{
    if( !iParams.check() )
        return false;

    //convertion sur les axes moteurs en sortie de réducteur.
    oMS.steering.left.position = iTS.steering.left.position/iParams.getTurretRatio();
    oMS.steering.right.position = iTS.steering.right.position/iParams.getTurretRatio();
    oMS.steering.rear.position = iTS.steering.rear.position/iParams.getTurretRatio();

    oMS.driving.left.velocity = (2*iTS.driving.left.velocity/iParams.getLeftWheelDiameter() - iSteeringState.left.velocity*iParams.getTurretRatio())
            /iParams.getTractionRatio();
    oMS.driving.right.velocity = (2*iTS.driving.right.velocity/iParams.getRightWheelDiameter() - iSteeringState.right.velocity*iParams.getTurretRatio())
            /iParams.getTractionRatio();
    oMS.driving.rear.velocity = (2*iTS.driving.rear.velocity/iParams.getRearWheelDiameter() - iSteeringState.rear.velocity*iParams.getTurretRatio())
            /iParams.getTractionRatio();

    //normalisation pour travailler dans [-PI/2;PI/2]*R
    normalizeDirection(oMS.steering.left.position, oMS.driving.left.velocity);
    normalizeDirection(oMS.steering.right.position, oMS.driving.right.velocity);
    normalizeDirection(oMS.steering.rear.position, oMS.driving.rear.velocity);

    //optimisation des consignes,gestion du pilotage mumtitour et gestion du zero

    //calcul de la position courante des moteurs en supprimant le multi tour et en ajoutant le zero
    // currenPosModuloPi est dans -Pi/Pi
    AxesGroup currentPosModuloPi;
    currentPosModuloPi.left.position = betweenMinusPiAndPlusPi(iSteeringState.left.position - iParams.getLeftTurretZero());
    currentPosModuloPi.right.position = betweenMinusPiAndPlusPi(iSteeringState.right.position - iParams.getRightTurretZero());
    currentPosModuloPi.rear.position = betweenMinusPiAndPlusPi(iSteeringState.rear.position - iParams.getRearTurretZero());

    //calcul des delta de position commandé
    AxesGroup deltaCmd;
    deltaCmd.left.position = oMS.steering.left.position - currentPosModuloPi.left.position;
    deltaCmd.right.position = oMS.steering.right.position - currentPosModuloPi.right.position;
    deltaCmd.rear.position = oMS.steering.rear.position - currentPosModuloPi.rear.position;

    //normalisation des delta entre -PI/2 et PI/2 et changement de signe des vitesses au besoin
    normalizeDirection(deltaCmd.left.position, oMS.driving.left.velocity);
    normalizeDirection(deltaCmd.right.position, oMS.driving.right.velocity);
    normalizeDirection(deltaCmd.rear.position, oMS.driving.rear.velocity);

    //ajout a la position brute du moteur le delta de commande, et paf ca fait la consigne a envoyer au prochain step
    oMS.steering = iSteeringState + deltaCmd;

    return true;
}

bool UbiquityKinematics::turrets2Twist(const TurretState & iTS, Twist2D& oTw, SlippageReport& oSR, const UbiquityParams & iParams)
{
    if( !iParams.check() )
    {
        return false;
    }

    Eigen::Matrix<double, 6, 4> A;
    {
        A(0,0) =  iParams.getLeftTurretPosition().y();
        A(1,0) = -iParams.getLeftTurretPosition().x();
        A(0,1) = -1.;
        A(1,1) =  0.;
        A(0,2) =  0.;
        A(1,2) = -1.;
        A(0,3) = cos(iTS.steering.left.position) * iTS.driving.left.velocity;
        A(1,3) = sin(iTS.steering.left.position) * iTS.driving.left.velocity;
    }
    {
        A(2,0) =  iParams.getRightTurretPosition().y();
        A(3,0) = -iParams.getRightTurretPosition().x();
        A(2,1) = -1.;
        A(3,1) =  0.;
        A(2,2) =  0.;
        A(3,2) = -1.;
        A(2,3) = cos(iTS.steering.right.position) * iTS.driving.right.velocity;
        A(3,3) = sin(iTS.steering.right.position) * iTS.driving.right.velocity;
    }
    {
        A(4,0) =  iParams.getRearTurretPosition().y();
        A(5,0) = -iParams.getRearTurretPosition().x();
        A(4,1) = -1.;
        A(5,1) =  0.;
        A(4,2) =  0.;
        A(5,2) = -1.;
        A(4,3) = cos(iTS.steering.rear.position) * iTS.driving.rear.velocity;
        A(5,3) = sin(iTS.steering.rear.position) * iTS.driving.rear.velocity;
    }

    Eigen::VectorXd s = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).singularValues();
    Eigen::MatrixXd V = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).matrixV();

    oSR.kernelQuality = s(2) / s(3);

    Eigen::VectorXd X = V.col(3);
    X = X / X(3);

    oTw.vh( X(0) );
    oTw.vx( X(1) );
    oTw.vy( X(2) );

    return true;
}

bool UbiquityKinematics::twist2Turrets(const Twist2D & iTw, TurretState& oTS, const UbiquityParams & iParams)
{
    if( !iParams.check() )
    {
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

    //normalisations
    normalizeDirection(oTS.steering.left.position, oTS.driving.left.velocity);
    normalizeDirection(oTS.steering.right.position, oTS.driving.right.velocity);
    normalizeDirection(oTS.steering.rear.position, oTS.driving.rear.velocity);

    return true;
}

bool UbiquityKinematics::motors2Twist(const MotorState & iMS,
                                    arp_math::Twist2D& oTw,
                                    SlippageReport& oSR,
                                    const UbiquityParams & iParams)
{
    bool res = true;
    TurretState ioTS;
    res &= motors2Turrets(iMS, ioTS, iParams);
    res &= turrets2Twist(ioTS, oTw, oSR, iParams);
    return res;
}

bool UbiquityKinematics::twist2Motors(const arp_math::Twist2D & iTw,
                                       const AxesGroup & iSteeringVelocities,
                                       MotorState& oMS,
                                       const UbiquityParams & iParams)
{
    bool res = true;
    TurretState ioTS;
    res &= twist2Turrets(iTw, ioTS, iParams);
    res &= turrets2Motors(ioTS, iSteeringVelocities, oMS, iParams);
    return res;
}


void UbiquityKinematics::normalizeDirection(double& angle, double& speed)
{
    //modulo 2PI pour commencer
    angle = betweenMinusPiAndPlusPi(angle);

    if( angle <= -M_PI_2 || angle > M_PI_2 )
    {
        if( angle < 0 )
            angle += M_PI;
        else
            angle -= M_PI;

        speed *= -1;
    }
}

