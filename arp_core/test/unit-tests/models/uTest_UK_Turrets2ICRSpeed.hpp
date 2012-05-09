/*
 * uTest_UK_Turrets2ICRSpeed.hpp
 *
 *  Created on: 8 may 2012
 *      Author: romain
 *
 *      Teste les modèles cinématiques du robot
 */

#include <boost/test/floating_point_comparison.hpp>
#include "models/UbiquityKinematics.hpp"
#include "models/UbiquityParams.hpp"

using namespace arp_math;
using namespace arp_model;

void check_twist2turrets2twix2twist(double vx, double vy, double vh)
{
    arp_model::UbiquityParams params;
    arp_math::Twist2D inTwist;
    arp_math::Twist2D outTwist;
    arp_model::TurretState turretCmd;
    ICRSpeed twix;

    bool res;

    inTwist.vx(vx);
    inTwist.vy(vy);
    inTwist.vh(vh);

    res = arp_model::UbiquityKinematics::twist2Turrets(inTwist, turretCmd, params);

    res = arp_model::UbiquityKinematics::simpleTurrets2ICRspeed(turretCmd, twix, params);
    outTwist = twix.twist();

    BOOST_CHECK_EQUAL(res, true);
    BOOST_CHECK_SMALL(outTwist.vx() - inTwist.vx(), 1.e-10);
    BOOST_CHECK_SMALL(outTwist.vy() - inTwist.vy(), 1.e-10);
    BOOST_CHECK_SMALL(outTwist.vh() - inTwist.vh(), 1.e-10);
}

void check_noMotion(double ICRx, double ICRy)
{
    arp_model::UbiquityParams params;
       arp_math::Twist2D outTwist;
       arp_model::TurretState turretCmd;
       ICRSpeed twix;
       bool res;

       Vector2 ICR(ICRx,ICRy);

       //angle turret to CIR
       Vector2 leftVect(Vector2(params.getLeftTurretPosition().x(),params.getLeftTurretPosition().y())-ICR);
       Vector2 rightVect(Vector2(params.getRightTurretPosition().x(),params.getRightTurretPosition().y())-ICR);
       Vector2 rearVect(Vector2(params.getRearTurretPosition().x(),params.getRearTurretPosition().y())-ICR);
       turretCmd.steering.left.position=atan2(leftVect(1),leftVect(0))+PI/2;
       turretCmd.steering.right.position=atan2(rightVect(1),rightVect(0))+PI/2;
       turretCmd.steering.rear.position=atan2(rearVect(1),rearVect(0))+PI/2;

       res = arp_model::UbiquityKinematics::simpleTurrets2ICRspeed(turretCmd, twix, params);
       outTwist=twix.twist();

       BOOST_CHECK_EQUAL( res , true);
       BOOST_CHECK_SMALL( outTwist.vx() , 1.e-10 );
       BOOST_CHECK_SMALL( outTwist.vy() , 1.e-10 );
       BOOST_CHECK_SMALL( outTwist.vh() , 1.e-10 );

       BOOST_CHECK_SMALL(tan(twix.ro())*cos(twix.alpha()+PI/2)-ICRx,1e-6);
       BOOST_CHECK_SMALL(tan(twix.ro())*sin(twix.alpha()+PI/2)-ICRy,1e-6);

}

BOOST_AUTO_TEST_CASE( UK_Turrets2ICRSpeed_Motions )
{
    cout << "1 check_twist2turrets2twix2twist(0.0,0.0,0.0);"<<endl;
    check_twist2turrets2twix2twist(0.0,0.0,0.0);
    cout << "1 check_twist2turrets2twix2twist(0.0,0.0,-2.0);"<<endl;
    check_twist2turrets2twix2twist(0.0,0.0,-2.0);
    cout << "2 check_twist2turrets2twix2twist(0.0,0.0,10.0);"<<endl;
    check_twist2turrets2twix2twist(0.0,0.0,10.0);
    cout << "3 check_twist2turrets2twix2twist(3.0,0.0,0.0);"<<endl;
    check_twist2turrets2twix2twist(3.0,0.0,0.0);
    cout << "4 check_twist2turrets2twix2twist(-1.8,0.0,0.0);"<<endl;
    check_twist2turrets2twix2twist(-1.8,0.0,0.0);
    cout << "5 check_twist2turrets2twix2twist(0.0,1.2,0.0);"<<endl;
    check_twist2turrets2twix2twist(0.0,1.2,0.0);
    cout << "6 check_twist2turrets2twix2twist(0.0,-15.8,0.0);"<<endl;
    check_twist2turrets2twix2twist(0.0,-15.8,0.0);
    cout << "7 check_twist2turrets2twix2twist(0.0,0.254,0.0);"<<endl;
    check_twist2turrets2twix2twist(0.0,-15.8,0.0);
    cout << "8 check_twist2turrets2twix2twist(-4.212,0.7897,1.5456)"<<endl;
    check_twist2turrets2twix2twist(-4.212,0.7897,1.5456);
    cout << "9 check_twist2turrets2twix2twist(-8.541,1.212,-9.121)"<<endl;
    check_twist2turrets2twix2twist(-8.541,1.212,-9.121);
    cout << "10 check_twist2turrets2twix2twist(-0.2654,5.21,0.0123)"<<endl;
    check_twist2turrets2twix2twist(-0.2654,5.21,0.0123);
}

BOOST_AUTO_TEST_CASE( UK_Turrets2ICRSpeed_NoMotion)
{
    arp_model::UbiquityParams params;

    cout << "1 check_noMotion(0.0,0.0);"<<endl;
    check_noMotion(0.0,0.0);
    cout << "2 check_noMotion(params.getLeftTurretPosition().x(),params.getLeftTurretPosition().y());"<<endl;
    check_noMotion(params.getLeftTurretPosition().x(),params.getLeftTurretPosition().y());
    cout << "3 check_noMotion(params.getRightTurretPosition().x(),params.getRightTurretPosition().y());"<<endl;
    check_noMotion(params.getRightTurretPosition().x(),params.getRightTurretPosition().y());
    cout << "4 check_noMotion(params.getRearTurretPosition().x(),params.getRearTurretPosition().y());"<<endl;
    check_noMotion(params.getRearTurretPosition().x(),params.getRearTurretPosition().y());
    cout << "5 check_noMotion(8.2123,-5.12);"<<endl;
    check_noMotion(8.2123,-5.12);


}

