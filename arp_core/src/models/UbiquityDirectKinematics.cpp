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

    //Driving position
    oTS.driving.left.position = iParams.getLeftWheelDiameter() / 2
            * (iMS.driving.left.position * iParams.getTractionRatio()
                    - iMS.steering.left.position * iParams.getTurretRatio());
    oTS.driving.right.position = iParams.getRightWheelDiameter() / 2
            * (iMS.driving.right.position * iParams.getTractionRatio()
                    - iMS.steering.right.position * iParams.getTurretRatio());
    oTS.driving.rear.position = iParams.getRearWheelDiameter() / 2
            * (iMS.driving.rear.position * iParams.getTractionRatio()
                    - iMS.steering.rear.position * iParams.getTurretRatio());

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

bool UbiquityKinematics::motors2Twist(const MotorState & iMS, TurretState& oTS, arp_math::Twist2D& oTw,
        SlippageReport& oSR, const UbiquityParams & iParams)
{
    bool res = true;
    res &= motors2Turrets(iMS, oTS, iParams);
//    res &= turrets2Twist(oTS, oTw, oSR, iParams);
    simpleTurrets2Twist(oTS, oTw, oSR, iParams);
    return res;
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



bool UbiquityKinematics::motors2ICRSpeed(const MotorState & iMS, TurretState& oTS, ICRSpeed& oICRs, SlippageReport& oSR,
        const UbiquityParams & iParams)
{
    bool res = true;
    res &= motors2Turrets(iMS, oTS, iParams);
    turrets2ICRspeed(oTS, oICRs, oSR, iParams);
    return res;
}


bool UbiquityKinematics::turrets2ICRspeed(const TurretState & iTS, ICRSpeed& oICRs, SlippageReport& oSR,
        const UbiquityParams & iParams)
{
    //TODO implementer le cas complexe
    return simpleTurrets2ICRspeed(iTS, oICRs, oSR, iParams);
}

bool UbiquityKinematics::simpleTurrets2ICRspeed(const TurretState & iTS, ICRSpeed& oICRs, SlippageReport& oSR,
        const UbiquityParams & iParams)
{
    //return simpleTurrets2ICRspeedWithICR(iTS, oICRs, oSR, iParams);  // by Willy
    //return turrets2ICRspeedViaTwistOrIntersections(iTS, oICRs, oSR, iParams);   // by Moulineau
    return turrets2ICRspeedViaTwist(iTS, oICRs, oSR, iParams);  // by Boris
}

bool UbiquityKinematics::turrets2ICRspeedViaTwistOrIntersections(const TurretState & iTS, ICRSpeed& oICRs,SlippageReport& oSR,
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
//    std::cout << "compute twist from TurretState : oTw=" << oTw.toString()  << std::endl;
    if (abs(oTw.vx()) > 0.001 or abs(oTw.vy()) > 0.001 or abs(oTw.vh()) > 0.001)
    {
        oICRs = ICRSpeed(oTw);
//        std::cout << "speed is not almost nul => return ICRSpeed(oTw) : " << oICRs.toString() << std::endl;
        return true;
    }

    if (colLeftRight) // front turrets are colinear
    {
//        std::cout << "front turrets are colinear" << std::endl;
        if (parLeftRear) // the third is parralel also
        {
//            std::cout << "the third is parralel also" << std::endl;
            oICRs = ICRSpeed::createIdleFromTranslation(iTS.steering.rear.position + PI / 2);

        }
        else // the third turret is not parralel: we are turning around a point on the front turrets line
        {
//            std::cout << "the third turret is not parralel: we are turning around a point on the front turrets line" << std::endl;
            oICRs = ICRSpeed::createIdleFromICRPosition(ICRRightRear);

        }
    }
    else if (parLeftRight) // front turrets are parralel but not colinear, the robot is in translation
    {
//        std::cout << "front turrets are parralel but not colinear, the robot is in translation" << std::endl;
        oICRs = ICRSpeed::createIdleFromTranslation(iTS.steering.left.position + PI / 2);
    }
    else // standard case: the front turrets are crossing somewhere
    {
//        std::cout << "standard case: the front turrets are crossing somewhere" << std::endl;
        oICRs = ICRSpeed::createIdleFromICRPosition(ICRLeftRight);
    }
    return true;
}

bool UbiquityKinematics::turrets2ICRspeedViaTwist(const TurretState & iTS, arp_math::ICRSpeed& oICRs, SlippageReport& oSR,
                        const UbiquityParams & iParams)
{
    if (!iParams.check())
    {
        Log(ERROR) << "UbiquityKinematics::turrets2ICRspeedViaTwist failed when checking params";
        return false;
    }

//    std::cout << "turrets2ICRspeedViaTwist : iTS : " << iTS.toString() << std::endl;

    // On numérote les 3 tourelles afin d'avoir la tourelle 1 qui a la vitesse la plus grande.
    Vector3 v(d_abs(iTS.driving.left.velocity), d_abs(iTS.driving.right.velocity), d_abs(iTS.driving.rear.velocity));
    std::pair< Eigen::VectorXd, Eigen::VectorXi > sorted = bubbleSortIndices(v);


    if(sorted.first[2] < iParams.getMinDrivingSpeed())  // la vitesse max est faible : on ne bouge pas
    {
//        std::cout << "turrets2ICRspeedViaTwist : on est a l'arrêt ou presque => turrets2ICRspeedViaTwistOrIntersections" << iTS.toString() << std::endl;
        return turrets2ICRspeedViaTwistOrIntersections(iTS, oICRs, oSR, iParams);  // on calcule l'ICRSpeed via les intersections
    }

    double v1, v2, v3;
    double a1, a2, a3;
    double xT1, xT2, xT3;
    double yT1, yT2, yT3;

    v1  = iTS.driving.left.velocity;
    a1  = iTS.steering.left.position;
    xT1 = iParams.getLeftTurretPosition().x();
    yT1 = iParams.getLeftTurretPosition().y();

    v2 = iTS.driving.right.velocity;
    a2 = iTS.steering.right.position;
    xT2 = iParams.getRightTurretPosition().x();
    yT2 = iParams.getRightTurretPosition().y();

    v3 = iTS.driving.rear.velocity;
    a3 = iTS.steering.rear.position;
    xT3 = iParams.getRearTurretPosition().x();
    yT3 = iParams.getRearTurretPosition().y();

    // On calcule 3 candidats oméga, chacun à partir d'un couple de tourelles
    Vector3 w;
    if( findAngularSpeedFromOdometry(iTS, w, iParams) == false )
    {
        return false;
    }

    // on estime omega à partir des 3 candidats
    //double w = w_candidates.mean();d
    std::pair< Eigen::VectorXd, Eigen::VectorXi > w_sortedCanditates = bubbleSortIndices(w);
    //on reserve l'indice de l'omega median
    int iMedianW = w_sortedCanditates.second[1];

    // On calcule v_x et v_y comme étant la moyenne des v_x et v_y provenant de 2 tourelles adjacentes.
    double v_x_a, v_x_b, v_y_a, v_y_b;
    switch( iMedianW )
    {
        //tourelles gauche et droite
        case 0:
            v_x_a = v1 * cos(a1) + yT1 * w[0];
            v_x_b = v2 * cos(a2) + yT2 * w[0];
            v_y_a = v1 * sin(a1) - xT1 * w[0];
            v_y_b = v2 * sin(a2) - xT2 * w[0];
            break;

        //tourelles droite et arrière
        case 1:
            v_x_a = v2 * cos(a2) + yT2 * w[1];
            v_x_b = v3 * cos(a3) + yT3 * w[1];
            v_y_a = v2 * sin(a2) - xT2 * w[1];
            v_y_b = v3 * sin(a3) - xT3 * w[1];
            break;

        //tourelles arrière et gauche
        case 2:
            v_x_a = v3 * cos(a3) + yT3 * w[2];
            v_x_b = v1 * cos(a1) + yT1 * w[2];
            v_y_a = v3 * sin(a3) - xT3 * w[2];
            v_y_b = v1 * sin(a1) - xT1 * w[2];
            break;

        default:
            return false;
            break;
    }

    Twist2D T;
    T.vx( (v_x_a + v_x_b)/2 );
    T.vy( (v_y_a + v_y_b)/2 );
    T.vh( w[iMedianW] );

    // On convertit en ICRSpeed
    oICRs = ICRSpeed(T);

    return true;
}

bool UbiquityKinematics::simpleTurrets2ICRspeedWithICR(const TurretState & iTS, arp_math::ICRSpeed& oICRs, SlippageReport& oSR,
        const UbiquityParams & iParams)
{
    bool res = true;

    if (!iParams.check())
    {
        Log(ERROR) << "UbiquityKinematics::simpleTurrets2ICRspeedWithICR failed when checking params";
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

bool UbiquityKinematics::findAngularSpeedFromOdometry(const TurretState & iTS, Vector3 & oAngularSpeeds, const UbiquityParams & iParams)
{
    double omegaLeftRigth;
    double omegaRigthRear;
    double omegaRearLeft;

    if (!iParams.check())
    {
        Log(ERROR) << "UbiquityKinematics::findAngularSpeedFromOdometry failed when checking params";
        return false;
    }

    Vector2 posRightToLeft =  iParams.getLeftTurretPosition().translation()  - iParams.getRightTurretPosition().translation();
    Vector2 posRearToRight =  iParams.getRightTurretPosition().translation() - iParams.getRearTurretPosition().translation();
    Vector2 posLeftToRear  =  iParams.getRearTurretPosition().translation()  - iParams.getLeftTurretPosition().translation();

    if( posRightToLeft.y() == 0 || posRearToRight.y() == 0 || posLeftToRear.y() == 0 )
    {
        //this is a degenerated configuration that should not appear on our robot.
       //It's highly possible that this won't work on other types of robot.
        Log(ERROR) << "UbiquityKinematics::findAngularSpeedFromOdometry failed when checking non degenerated turret positions";
        Log(ERROR) << "pRL:" << posRightToLeft << " pRR:" << posRightToLeft << " pLR:" << posLeftToRear;
        return false;
    }

    //cinematic equation, transporting the torsor of the second turret of the pair to the first turret, projecting on x as it ensure no null term on Ubiquity
    //note : we need the y coordinate of intra turret vectors as its the result of a z^y cross product.
    omegaLeftRigth = (-iTS.driving.left.velocity*cos(iTS.steering.left.position) + iTS.driving.right.velocity*cos(iTS.steering.right.position))/posRightToLeft.y();
    omegaRigthRear = (-iTS.driving.right.velocity*cos(iTS.steering.right.position) + iTS.driving.rear.velocity*cos(iTS.steering.rear.position))/posRearToRight.y();
    omegaRearLeft  = (-iTS.driving.rear.velocity*cos(iTS.steering.rear.position) + iTS.driving.left.velocity*cos(iTS.steering.left.position))/posLeftToRear.y();

    oAngularSpeeds[0] = omegaLeftRigth;
    oAngularSpeeds[1] = omegaRigthRear;
    oAngularSpeeds[2] = omegaRearLeft;


    return true;
}

void UbiquityKinematics::findICRfromTurretAngles(const TurretState & iTS, ICR & oIcrPosition,
        double firstTurretAngle, Pose2D firstTurretPosition,
        double secondTurretAngle, Pose2D secondTurretPosition)
{
    if( fabs(betweenMinusPi2AndPlusPi2(firstTurretAngle - secondTurretAngle)) <= 1E-9 )
    {
        oIcrPosition.phi(firstTurretAngle);
        oIcrPosition.delta(0);
        //cout << "angle equal (" << firstTurretAngle << ";" << secondTurretAngle << ")" << endl;
        return;
    }

    //as it is a simple matrix we already have inverted it 1/A*det(A) = AInv
    Eigen::Matrix<double,2,2> AInv;
    {
        AInv(0,0) = cos(secondTurretAngle);
        AInv(0,1) = sin(secondTurretAngle);
        AInv(1,0) = cos(firstTurretAngle);
        AInv(1,1) = sin(firstTurretAngle);
    }

    Vector2 B;
    {
        B(0) = secondTurretPosition.x() - firstTurretPosition.x();
        B(1) = secondTurretPosition.y() - firstTurretPosition.y();
    }


    double invDet = 1/sin(secondTurretAngle - firstTurretAngle);
    Vector2 icrDistances = invDet*(AInv*B);

    //TODO si le CIR est sous une tourelle faut prendre l'autre ... ah bon ?
    ICR firstIcr;
    ICR secondIcr;

    firstIcr.phi(firstTurretAngle);
    secondIcr.phi(secondTurretAngle);

    firstIcr.delta( atan2(Pose2DNorm::dmax, icrDistances[0]) );
    secondIcr.delta( atan2(Pose2DNorm::dmax, icrDistances[1]) );

    ICR candidate1 = firstIcr.transport(firstTurretPosition.opposite());
    ICR candidate2 = secondIcr.transport(secondTurretPosition.opposite());

    if(candidate1.delta()*candidate2.delta() < 0 )
    {
        candidate2 = candidate2.getAntipodICR();
    }

    oIcrPosition.phi((candidate1.phi()+candidate2.phi())/2);
    oIcrPosition.delta((candidate1.delta()+candidate2.delta())/2);

    cout << "invDet =\t"<< invDet << endl;
    cout << "B[0] =\t"<< B[0] << endl;
    cout << "B[1] =\t" << B[1] << endl;
    cout << "phi1 =\t"<< rad2deg(firstTurretAngle) << endl;
    cout << "phi2 =\t" << rad2deg(secondTurretAngle) << endl;
    cout << "distance1 =\t" << icrDistances[0] << endl;
    cout << "distance2 =\t" << icrDistances[1] << endl;
    cout << "firstIcr  =\t" << firstIcr.toString() << endl;
    cout << "secondIcr =\t" << secondIcr.toString() << endl;
    cout << "candidate1 =\t" << candidate1.toString() << endl;
    cout << "candidate2 =\t" << candidate2.toString() << endl;
    cout << "oIcrPosition =\t" << oIcrPosition.toString() << endl;
}


