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

bool UbiquityKinematics::turrets2Twist(const TurretState & iTS, Twist2D& oTw, SlippageReport& oSR,
        const UbiquityParams & iParams)
{
    if (!iParams.check())
    {
        Log(ERROR) << "UbiquityKinematics::turrets2Twist failed when checking params";
        return false;
    }

    Eigen::Matrix<double, 6, 4> A;
    {
        A(0, 0) = iParams.getLeftTurretPosition().y();
        A(1, 0) = -iParams.getLeftTurretPosition().x();
        A(0, 1) = -1.;
        A(1, 1) = 0.;
        A(0, 2) = 0.;
        A(1, 2) = -1.;
        A(0, 3) = cos(iTS.steering.left.position) * iTS.driving.left.velocity;
        A(1, 3) = sin(iTS.steering.left.position) * iTS.driving.left.velocity;
    }
    {
        A(2, 0) = iParams.getRightTurretPosition().y();
        A(3, 0) = -iParams.getRightTurretPosition().x();
        A(2, 1) = -1.;
        A(3, 1) = 0.;
        A(2, 2) = 0.;
        A(3, 2) = -1.;
        A(2, 3) = cos(iTS.steering.right.position) * iTS.driving.right.velocity;
        A(3, 3) = sin(iTS.steering.right.position) * iTS.driving.right.velocity;
    }
    {
        A(4, 0) = iParams.getRearTurretPosition().y();
        A(5, 0) = -iParams.getRearTurretPosition().x();
        A(4, 1) = -1.;
        A(5, 1) = 0.;
        A(4, 2) = 0.;
        A(5, 2) = -1.;
        A(4, 3) = cos(iTS.steering.rear.position) * iTS.driving.rear.velocity;
        A(5, 3) = sin(iTS.steering.rear.position) * iTS.driving.rear.velocity;
    }

    Eigen::VectorXd s = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).singularValues();
    Eigen::MatrixXd V = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).matrixV();

    oSR.coherency1 = s(2) / s(3);
    oSR.coherency2 = 0;
    oSR.coherency3 = 0;

    Eigen::VectorXd X = V.col(3);
    X = X / X(3);

    oTw.vh(X(0));
    oTw.vx(X(1));
    oTw.vy(X(2));

    //use this if you think that the nominal code has trouble with singularities in matrix
    //simpleTurrets2Twist(iTS,oTw,oSR,iParams);

    return true;
}

void UbiquityKinematics::simpleTurrets2Twist(const TurretState & iTS, Twist2D& oTw, SlippageReport& oSR,
        const UbiquityParams & iParams)
{
//very simple odometry calculation for debug
//uses only 3 measures

    // TEST MOULINEAU
    //matrice "(vxL,vyL,vxR)= A . (vx,vy,thetap)
    Eigen::Matrix<double, 3, 3> Transfert;
    {
        // vxLeft=vx-thetap. yLeft
        Transfert(0, 0) = 1;
        Transfert(0, 1) = 0;
        Transfert(0, 2) = -iParams.getLeftTurretPosition().y();
        //vyLeft=vy+thetap . xLeft
        Transfert(1, 0) = 0;
        Transfert(1, 1) = 1;
        Transfert(1, 2) = iParams.getLeftTurretPosition().x();

        //idem for vxRight
        Transfert(2, 0) = 1;
        Transfert(2, 1) = 0;
        Transfert(2, 2) = -iParams.getRightTurretPosition().y();
    }

//    Log(DEBUG) << ">> turrets2twist";
//    Log(DEBUG) << "iTS" << iTS.toString();
//    Log(DEBUG) << "iTS.steering.left.position " << iTS.steering.left.position;
//    Log(DEBUG) << "iTS.driving.left.velocity  " << iTS.driving.left.velocity;

    //matrice des mesures
    Eigen::Matrix<double, 3, 1> Mesures;
    {
        Mesures(0, 0) = cos(iTS.steering.left.position) * iTS.driving.left.velocity;
        Mesures(1, 0) = sin(iTS.steering.left.position) * iTS.driving.left.velocity;
        Mesures(2, 0) = cos(iTS.steering.right.position) * iTS.driving.right.velocity;
    }

//    Log(DEBUG) << "Transfert: " << Transfert;
//    Log(DEBUG) << "mesures:   " << Mesures;

    Eigen::Matrix<double, 3, 1> Twist;
    Twist = Transfert.inverse() * Mesures;

    oTw.vx(Twist(0, 0));
    oTw.vy(Twist(1, 0));
    oTw.vh(Twist(2, 0));

    oSR.coherency1 = 0;
    oSR.coherency2 = 0;
    oSR.coherency3 = 0;
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

bool UbiquityKinematics::motors2Twist(const MotorState & iMS, TurretState& oTS, arp_math::Twist2D& oTw,
        SlippageReport& oSR, const UbiquityParams & iParams)
{
    bool res = true;
    res &= motors2Turrets(iMS, oTS, iParams);
//    res &= turrets2Twist(oTS, oTw, oSR, iParams);
    simpleTurrets2Twist(oTS, oTw, oSR, iParams);
    return res;
}

bool UbiquityKinematics::twist2Motors(const arp_math::Twist2D & iTw, const MotorState & iMS, TurretState& oTS,
        MotorState& oMS, const UbiquityParams & iParams)
{
    bool res = true;
    res &= twist2Turrets(iTw, oTS, iParams);
    res &= turrets2Motors(oTS, iMS, oMS, iParams);
    return res;
}


