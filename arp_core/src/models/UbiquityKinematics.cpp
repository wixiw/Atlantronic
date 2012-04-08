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
    //cout << "----- UbiquityKinematics::motors2Turrets" << endl;

    oTS.steering.left.position = (iMS.steering.left.position - iParams.getLeftTurretZero())
            *iParams.getTurretRatio() ;
    oTS.steering.right.position = (iMS.steering.right.position - iParams.getRightTurretZero())
            *iParams.getTurretRatio();
    oTS.steering.rear.position = (iMS.steering.rear.position - iParams.getRearTurretZero())
            *iParams.getTurretRatio();

    oTS.driving.left.velocity = iParams.getLeftWheelDiameter()/2
            *(iMS.driving.left.velocity*iParams.getTractionRatio() + iMS.steering.left.velocity*iParams.getTurretRatio());

    oTS.driving.right.velocity = iParams.getRightWheelDiameter()/2
            *(iMS.driving.right.velocity*iParams.getTractionRatio() + iMS.steering.right.velocity*iParams.getTurretRatio());


    oTS.driving.rear.velocity = iParams.getRearWheelDiameter()/2
            *(iMS.driving.rear.velocity*iParams.getTractionRatio() + iMS.steering.rear.velocity*iParams.getTurretRatio());
    //cout << "oTS b4 norm : " << oTS.toString() << endl;

    //normalisations
    normalizeDirection(oTS.steering.left.position, oTS.driving.left.velocity);
    normalizeDirection(oTS.steering.right.position, oTS.driving.right.velocity);
    normalizeDirection(oTS.steering.rear.position, oTS.driving.rear.velocity);

    return true;
}

bool UbiquityKinematics::turrets2Motors(const TurretState & iTSbrut,
                                        const MotorState & iMS,
                                        MotorState& oMS,
                                        const UbiquityParams & iParams)
{
    //cout << "----- UbiquityKinematics::turrets2Motors" << endl;
    if( !iParams.check() )
        return false;

    //normalisation des entrees pour travailler dans [-PI/2;PI/2]*R
    TurretState iTS = iTSbrut;
    normalizeDirection(iTS);

    //optimisation des consignes,gestion du pilotage mumtitour et gestion du zero

    //calcul de la position courante des tourelles à partir de l'état des moteurs
    // currentTurretPos est dans -Pi/2 /Pi/2
    TurretState currentTurretPos ;
    motors2Turrets(iMS,currentTurretPos,iParams);

    //calcul des delta de position commandé
    AxesGroup deltaCmd = iTS.steering - currentTurretPos.steering;
    TurretState dummyDeltaCmd;//utilisé pour retourner les vitesses de traction
    dummyDeltaCmd.steering = deltaCmd;
    dummyDeltaCmd.driving = iTS.driving;
    normalizeDirection(dummyDeltaCmd);
    deltaCmd = dummyDeltaCmd.steering;
    iTS.driving = dummyDeltaCmd.driving;

    //cout << "iTS : " << iTS.toString() << endl;
    //cout << "currentTurretPos : " << currentTurretPos.toString() << endl;
    //cout << "deltaCmd : " << deltaCmd.toString() << endl;

    //ajout a la position brute du moteur le delta de commande, et paf ca fait la consigne a envoyer au prochain step
    oMS.steering.left.position = iMS.steering.left.position + deltaCmd.left.position/iParams.getTurretRatio() + iParams.getLeftTurretZero();
    oMS.steering.right.position = iMS.steering.right.position + deltaCmd.right.position/iParams.getTurretRatio() + iParams.getRightTurretZero();
    oMS.steering.rear.position = iMS.steering.rear.position + deltaCmd.rear.position/iParams.getTurretRatio() + iParams.getRearTurretZero();

    //mise a jour des vitesses de traction
    oMS.driving.left.velocity = (2*iTS.driving.left.velocity/iParams.getLeftWheelDiameter() - iMS.steering.left.velocity*iParams.getTurretRatio())
            /iParams.getTractionRatio();
    oMS.driving.right.velocity = (2*iTS.driving.right.velocity/iParams.getRightWheelDiameter() - iMS.steering.right.velocity*iParams.getTurretRatio())
            /iParams.getTractionRatio();
    oMS.driving.rear.velocity = (2*iTS.driving.rear.velocity/iParams.getRearWheelDiameter() - iMS.steering.rear.velocity*iParams.getTurretRatio())
            /iParams.getTractionRatio();

    //cout << "oMS : " << oMS.toString() << endl;
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
    normalizeDirection(oTS);

    return true;
}

bool UbiquityKinematics::motors2Twist(const MotorState & iMS,
                                    TurretState& oTS,
                                    arp_math::Twist2D& oTw,
                                    SlippageReport& oSR,
                                    const UbiquityParams & iParams)
{
    bool res = true;
    res &= motors2Turrets(iMS, oTS, iParams);
    res &= turrets2Twist(oTS, oTw, oSR, iParams);
    return res;
}

bool UbiquityKinematics::twist2Motors(const arp_math::Twist2D & iTw,
                                       const MotorState & iMS,
                                       TurretState& oTS,
                                       MotorState& oMS,
                                       const UbiquityParams & iParams)
{
    bool res = true;
    res &= twist2Turrets(iTw, oTS, iParams);
    res &= turrets2Motors(oTS, iMS, oMS, iParams);
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

    if( angle <= -M_PI_2 || angle > M_PI_2 )
    {
        if( angle < 0 )
            angle += M_PI;
        else
            angle -= M_PI;

        speed *= -1;
    }
}

