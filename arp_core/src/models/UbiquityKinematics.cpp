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

    oSR.kernelQuality = s(2) / s(3);

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

    Log(DEBUG) << ">> turrets2twist";
    Log(DEBUG) << "iTS" << iTS.toString();
    Log(DEBUG) << "iTS.steering.left.position " << iTS.steering.left.position;
    Log(DEBUG) << "iTS.driving.left.velocity  " << iTS.driving.left.velocity;

    //matrice des mesures
    Eigen::Matrix<double, 3, 1> Mesures;
    {
        Mesures(0, 0) = cos(iTS.steering.left.position) * iTS.driving.left.velocity;
        Mesures(1, 0) = sin(iTS.steering.left.position) * iTS.driving.left.velocity;
        Mesures(2, 0) = cos(iTS.steering.right.position) * iTS.driving.right.velocity;
    }

    Log(DEBUG) << "Transfert: " << Transfert;
    Log(DEBUG) << "mesures:   " << Mesures;

    Eigen::Matrix<double, 3, 1> Twist;
    Twist = Transfert.inverse() * Mesures;

    oTw.vx(Twist(0, 0));
    oTw.vy(Twist(1, 0));
    oTw.vh(Twist(2, 0));

}

bool UbiquityKinematics::simpleTurrets2ICRspeed(const TurretState & iTS, ICRSpeed& oICRs,
        const UbiquityParams & iParams)
{

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

    //SIMPLIFIED VERSION: takes  only 2 turrets to work
    if (colLeftRight) // front turrets are coliear
    {
        if (parLeftRear) // the third is parralel also
        {
            oICRs = ICRSpeed::createFromTranslation(iTS.steering.rear.position, iTS.driving.rear.velocity);

        }
        else // the third turret is not parralel: we are turning around a point on the front turrets line
        {
            oICRs = ICRSpeed::createFromICR(ICRRightRear, pRear1, rearSpeed);

        }
    }
    else if (parLeftRight) // front turrets are parralel but not colinear, the robot is in translation
    {
        oICRs = ICRSpeed::createFromTranslation(iTS.steering.left.position, iTS.driving.left.velocity);
    }

    else // standard case: the front turrets are crossing
    {
        if (fabs(iTS.driving.left.velocity) >= fabs(iTS.driving.right.velocity))
        {
            oICRs = ICRSpeed::createFromICR(ICRLeftRight, pLeft1, leftSpeed);

        }
        else
        {
            oICRs = ICRSpeed::createFromICR(ICRLeftRight, pRight1, rightSpeed);
        }

    }

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

bool UbiquityKinematics::motors2Twist(const MotorState & iMS, TurretState& oTS, arp_math::Twist2D& oTw,
        SlippageReport& oSR, const UbiquityParams & iParams)
{
    bool res = true;
    res &= motors2Turrets(iMS, oTS, iParams);
    res &= turrets2Twist(oTS, oTw, oSR, iParams);
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

