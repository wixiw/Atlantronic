/*
 * uTest_UK_MotorTurret.hpp
 *
 *  Created on: 04 April 2012
 *      Author: willy
 *
 *      Teste les modèles cinématiques du robot
 *      ATTENTION certains calculs peuvent dépendre des valeurs par défaut de params
 *      on suppose notamment que le rayon de la roue est de 66mm, un rapport de traction de 1 et le rapport tourelle de 0.25
 */

#include <boost/test/floating_point_comparison.hpp>
#include "models/UbiquityKinematics.hpp"
#include "models/UbiquityParams.hpp"


BOOST_AUTO_TEST_CASE( UK_MotorTurret_ZeroToZeroTest )
{
    arp_model::UbiquityParams params;
    params.fillWithFakeValues();

    arp_model::MotorState motorsCmd;
    arp_model::TurretState turretCmd;
    arp_model::MotorState steeringMotorVelocities;
    bool res;

    res = arp_model::UbiquityKinematics::motors2Turrets(motorsCmd, turretCmd, params);

    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( turretCmd.driving.left.velocity,    0 );
    BOOST_CHECK_EQUAL( turretCmd.driving.right.velocity,   0 );
    BOOST_CHECK_EQUAL( turretCmd.driving.rear.velocity,    0 );
    BOOST_CHECK_EQUAL( turretCmd.steering.left.position,   0 );
    BOOST_CHECK_EQUAL( turretCmd.steering.right.position,  0 );
    BOOST_CHECK_EQUAL( turretCmd.steering.rear.position,   0 );

    res = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, steeringMotorVelocities, motorsCmd, params);

    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( motorsCmd.driving.left.velocity,     0 );
    BOOST_CHECK_EQUAL( motorsCmd.driving.right.velocity,    0 );
    BOOST_CHECK_EQUAL( motorsCmd.driving.rear.velocity,     0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.left.position,    0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.right.position,   0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.rear.position,    0 );
}

//test les mouvements de translation avec en condition initiale les tourelles en 0
BOOST_AUTO_TEST_CASE( UK_MotorTurret_turret2Motor_translationFromZero )
{
    arp_model::UbiquityParams params;
    params.fillWithFakeValues();

    arp_model::MotorState motorsCmd;
    arp_model::TurretState turretCmd;
    arp_model::MotorState motorsMeasure;
    bool success;

    //marche avant traction
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.6;
    turretCmd.driving.rear.velocity = 0.7;
    turretCmd.steering.left.position = 0;
    turretCmd.steering.right.position = 0;
    turretCmd.steering.rear.position = 0;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.6*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.7*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( motorsCmd.steering.left.position,    0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.right.position,   0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.rear.position,    0 );

    //marche AR traction
    turretCmd.driving.left.velocity = -0.5;
    turretCmd.driving.right.velocity = -0.6;
    turretCmd.driving.rear.velocity = -0.7;
    turretCmd.steering.left.position = 0;
    turretCmd.steering.right.position = 0;
    turretCmd.steering.rear.position = 0;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     -0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    -0.6*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     -0.7*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( motorsCmd.steering.left.position,    0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.right.position,   0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.rear.position,    0 );

    //translation de P1/4 à gauche en AV
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = M_PI/4;
    turretCmd.steering.right.position = M_PI/4;
    turretCmd.steering.rear.position = M_PI/4;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    M_PI, 1E-6 );

    //translation de P1/2 à gauche en AV
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = M_PI/2;
    turretCmd.steering.right.position = M_PI/2;
    turretCmd.steering.rear.position = M_PI/2;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    2*M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   2*M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    2*M_PI, 1E-6 );

    //translation de P1/2 à gauche en AV + epsilon
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = M_PI/2 + 0.01;
    turretCmd.steering.right.position = M_PI/2 + 0.01;
    turretCmd.steering.rear.position = M_PI/2 + 0.01;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     -0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    -0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     -0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    -2*M_PI + 0.01 / params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   -2*M_PI + 0.01 / params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    -2*M_PI + 0.01 / params.getTurretRatio(), 1E-6 );


    //translation de P1/2 à gauche en AV - epsilon
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = M_PI/2 - 0.01;
    turretCmd.steering.right.position = M_PI/2 - 0.01;
    turretCmd.steering.rear.position = M_PI/2 - 0.01;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    2*M_PI - 0.01 / params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   2*M_PI - 0.01 / params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    2*M_PI - 0.01 / params.getTurretRatio(), 1E-6 );


    //translation de -P1/4 à gauche en AV
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = -M_PI/4;
    turretCmd.steering.right.position = -M_PI/4;
    turretCmd.steering.rear.position = -M_PI/4;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    -M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   -M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    -M_PI, 1E-6 );

    //translation de -P1/2 à gauche en AV
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = -M_PI/2;
    turretCmd.steering.right.position = -M_PI/2;
    turretCmd.steering.rear.position = -M_PI/2;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     -0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    -0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     -0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    2*M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   2*M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    2*M_PI, 1E-6 );

}

//test les mouvements de translation avec en condition initiale les tourelles à PI/2
BOOST_AUTO_TEST_CASE( UK_MotorTurret_turret2Motor_translationFromPI2 )
{
    arp_model::UbiquityParams params;
    params.fillWithFakeValues();

    arp_model::MotorState motorsCmd;
    arp_model::TurretState turretCmd;
    arp_model::MotorState motorsMeasure;
    motorsMeasure.driving.left.velocity = 0.0;
    motorsMeasure.driving.right.velocity = 0.0;
    motorsMeasure.driving.rear.velocity = 0.0;
    motorsMeasure.steering.left.position = M_PI_2/params.getTurretRatio();
    motorsMeasure.steering.right.position = M_PI_2/params.getTurretRatio();
    motorsMeasure.steering.rear.position = M_PI_2/params.getTurretRatio();
    bool success;

    //marche avant traction
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.6;
    turretCmd.driving.rear.velocity = 0.7;
    turretCmd.steering.left.position = 0;
    turretCmd.steering.right.position = 0;
    turretCmd.steering.rear.position = 0;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     -0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    -0.6*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     -0.7*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( motorsCmd.steering.left.position,    M_PI/params.getTurretRatio() );
    BOOST_CHECK_EQUAL( motorsCmd.steering.right.position,   M_PI/params.getTurretRatio() );
    BOOST_CHECK_EQUAL( motorsCmd.steering.rear.position,    M_PI/params.getTurretRatio() );

    //marche AR traction
    turretCmd.driving.left.velocity = -0.5;
    turretCmd.driving.right.velocity = -0.6;
    turretCmd.driving.rear.velocity = -0.7;
    turretCmd.steering.left.position = 0;
    turretCmd.steering.right.position = 0;
    turretCmd.steering.rear.position = 0;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.6*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.7*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( motorsCmd.steering.left.position,    M_PI/params.getTurretRatio() );
    BOOST_CHECK_EQUAL( motorsCmd.steering.right.position,   M_PI/params.getTurretRatio() );
    BOOST_CHECK_EQUAL( motorsCmd.steering.rear.position,    M_PI/params.getTurretRatio() );

    //marche AV traction à 0° + epsilon
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.6;
    turretCmd.driving.rear.velocity = 0.7;
    turretCmd.steering.left.position = 0.01;
    turretCmd.steering.right.position = 0.01;
    turretCmd.steering.rear.position = 0.01;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.6*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.7*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    0.01/params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   0.01/params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    0.01/params.getTurretRatio(), 1E-6  );

    //translation de P1/4 à gauche en AV
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = M_PI/4;
    turretCmd.steering.right.position = M_PI/4;
    turretCmd.steering.rear.position = M_PI/4;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    M_PI/4/params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   M_PI/4/params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    M_PI/4/params.getTurretRatio(), 1E-6 );

    //translation de P1/2 à gauche en AV
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = M_PI/2;
    turretCmd.steering.right.position = M_PI/2;
    turretCmd.steering.rear.position = M_PI/2;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    2*M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   2*M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    2*M_PI, 1E-6 );

    //translation de P1/2 à gauche en AV + epsilon
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = M_PI/2 + 0.01;
    turretCmd.steering.right.position = M_PI/2 + 0.01;
    turretCmd.steering.rear.position = M_PI/2 + 0.01;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    2*M_PI + 0.01 / params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   2*M_PI + 0.01 / params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    2*M_PI + 0.01 / params.getTurretRatio(), 1E-6 );


    //translation de P1/2 à gauche en AV - epsilon
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = M_PI/2 - 0.01;
    turretCmd.steering.right.position = M_PI/2 - 0.01;
    turretCmd.steering.rear.position = M_PI/2 - 0.01;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    2*M_PI - 0.01 / params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   2*M_PI - 0.01 / params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    2*M_PI - 0.01 / params.getTurretRatio(), 1E-6 );


    //translation de -P1/4 à gauche en AV
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = -M_PI/4;
    turretCmd.steering.right.position = -M_PI/4;
    turretCmd.steering.rear.position = -M_PI/4;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     -0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    -0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     -0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    3*M_PI/4/params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   3*M_PI/4/params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    3*M_PI/4/params.getTurretRatio(), 1E-6 );

    //translation de -P1/2 à gauche en AV
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = -M_PI/2;
    turretCmd.steering.right.position = -M_PI/2;
    turretCmd.steering.rear.position = -M_PI/2;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     -0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    -0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     -0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    2*M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   2*M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    2*M_PI, 1E-6 );

}


//test les mouvements de translation avec en condition initiale les tourelles à 3PI/4
BOOST_AUTO_TEST_CASE( UK_MotorTurret_turret2Motor_translationFrom3PI4 )
{
    arp_model::UbiquityParams params;
    params.fillWithFakeValues();

    arp_model::MotorState motorsCmd;
    arp_model::TurretState turretCmd;
    arp_model::MotorState motorsMeasure;
    motorsMeasure.driving.left.velocity = 0.0;
    motorsMeasure.driving.right.velocity = 0.0;
    motorsMeasure.driving.rear.velocity = 0.0;
    motorsMeasure.steering.left.position = 3*M_PI_4/params.getTurretRatio();
    motorsMeasure.steering.right.position = 3*M_PI_4/params.getTurretRatio();
    motorsMeasure.steering.rear.position = 3*M_PI_4/params.getTurretRatio();
    bool success;

    //marche avant traction
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.6;
    turretCmd.driving.rear.velocity = 0.7;
    turretCmd.steering.left.position = 0;
    turretCmd.steering.right.position = 0;
    turretCmd.steering.rear.position = 0;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     -0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    -0.6*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     -0.7*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( motorsCmd.steering.left.position,    M_PI/params.getTurretRatio() );
    BOOST_CHECK_EQUAL( motorsCmd.steering.right.position,   M_PI/params.getTurretRatio() );
    BOOST_CHECK_EQUAL( motorsCmd.steering.rear.position,    M_PI/params.getTurretRatio() );

    //marche AR traction
    turretCmd.driving.left.velocity = -0.5;
    turretCmd.driving.right.velocity = -0.6;
    turretCmd.driving.rear.velocity = -0.7;
    turretCmd.steering.left.position = 0;
    turretCmd.steering.right.position = 0;
    turretCmd.steering.rear.position = 0;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.6*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.7*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( motorsCmd.steering.left.position,    M_PI/params.getTurretRatio() );
    BOOST_CHECK_EQUAL( motorsCmd.steering.right.position,   M_PI/params.getTurretRatio() );
    BOOST_CHECK_EQUAL( motorsCmd.steering.rear.position,    M_PI/params.getTurretRatio() );

    //marche AV traction à PI/4° + epsilon
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.6;
    turretCmd.driving.rear.velocity = 0.7;
    turretCmd.steering.left.position = M_PI_4 + 0.01;
    turretCmd.steering.right.position = M_PI_4 + 0.01;
    turretCmd.steering.rear.position = M_PI_4 + 0.01;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.6*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.7*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    (M_PI_4 + 0.01)/params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   (M_PI_4 + 0.01)/params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    (M_PI_4 + 0.01)/params.getTurretRatio(), 1E-6  );

    //translation de P1/4 à gauche en AV
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = M_PI/4;
    turretCmd.steering.right.position = M_PI/4;
    turretCmd.steering.rear.position = M_PI/4;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     -0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    -0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     -0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    (M_PI/2+3*M_PI/4)/params.getTurretRatio(), 1E-6  ); //!! attention pas de modulo pi sur les tourelles !! c'est du modulo 8PI !
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   (M_PI/2+3*M_PI/4)/params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    (M_PI/2+3*M_PI/4)/params.getTurretRatio(), 1E-6 );

    //translation de P1/4 à gauche en AV - epsilon
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = M_PI_4 - 0.01;
    turretCmd.steering.right.position = M_PI_4 - 0.01;
    turretCmd.steering.rear.position = M_PI_4 - 0.01;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     -0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    -0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     -0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    (M_PI/2+3*M_PI/4 - 0.01) / params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   (M_PI/2+3*M_PI/4 - 0.01) / params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    (M_PI/2+3*M_PI/4 - 0.01) / params.getTurretRatio(), 1E-6 );

    //translation de P1/2 à gauche en AV
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = M_PI/2;
    turretCmd.steering.right.position = M_PI/2;
    turretCmd.steering.rear.position = M_PI/2;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    2*M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   2*M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    2*M_PI, 1E-6 );




    //translation de -P1/4 à gauche en AV
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = -M_PI/4;
    turretCmd.steering.right.position = -M_PI/4;
    turretCmd.steering.rear.position = -M_PI/4;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     -0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    -0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     -0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    3*M_PI/4/params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   3*M_PI/4/params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    3*M_PI/4/params.getTurretRatio(), 1E-6 );

    //translation de -P1/2 à gauche en AV
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = -M_PI/2;
    turretCmd.steering.right.position = -M_PI/2;
    turretCmd.steering.rear.position = -M_PI/2;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     -0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    -0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     -0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    M_PI/2/params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   M_PI/2/params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    M_PI/2/params.getTurretRatio(), 1E-6 );
}


/******************************************************
 * MOTOR 2 TURRET
 */


//test les mouvements de translation
BOOST_AUTO_TEST_CASE( UK_MotorTurret_motor2Turret_translation )
{
    arp_model::UbiquityParams params;
    params.fillWithFakeValues();

    arp_model::TurretState turretMeasure;
    arp_model::MotorState motorsMeasure;
    bool success;

    //marche avant traction
    motorsMeasure.driving.left.velocity = 10;
    motorsMeasure.driving.right.velocity = 11;
    motorsMeasure.driving.rear.velocity = 12;
    motorsMeasure.steering.left.position = 0;
    motorsMeasure.steering.right.position = 0;
    motorsMeasure.steering.rear.position = 0;
    success = arp_model::UbiquityKinematics::motors2Turrets( motorsMeasure, turretMeasure, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( turretMeasure.driving.left.velocity,     10.0/1/2*params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.right.velocity,    11.0/1/2*params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.rear.velocity,     12.0/1/2*params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( turretMeasure.steering.left.position,    0 );
    BOOST_CHECK_EQUAL( turretMeasure.steering.right.position,   0 );
    BOOST_CHECK_EQUAL( turretMeasure.steering.rear.position,    0 );

    //marche ar traction
    motorsMeasure.driving.left.velocity = -10;
    motorsMeasure.driving.right.velocity = -11;
    motorsMeasure.driving.rear.velocity = -12;
    motorsMeasure.steering.left.position = 0;
    motorsMeasure.steering.right.position = 0;
    motorsMeasure.steering.rear.position = 0;
    success = arp_model::UbiquityKinematics::motors2Turrets( motorsMeasure, turretMeasure, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( turretMeasure.driving.left.velocity,     -10.0/1/2*params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.right.velocity,    -11.0/1/2*params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.rear.velocity,     -12.0/1/2*params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( turretMeasure.steering.left.position,    0 );
    BOOST_CHECK_EQUAL( turretMeasure.steering.right.position,   0 );
    BOOST_CHECK_EQUAL( turretMeasure.steering.rear.position,    0 );

    //marche avant à PI/4
    motorsMeasure.driving.left.velocity = 10;
    motorsMeasure.driving.right.velocity = 11;
    motorsMeasure.driving.rear.velocity = 12;
    motorsMeasure.steering.left.position = M_PI_4/params.getTurretRatio();
    motorsMeasure.steering.right.position = M_PI_4/params.getTurretRatio();
    motorsMeasure.steering.rear.position = M_PI_4/params.getTurretRatio();
    success = arp_model::UbiquityKinematics::motors2Turrets( motorsMeasure, turretMeasure, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( turretMeasure.driving.left.velocity,     10.0/1/2*params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.right.velocity,    11.0/1/2*params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.rear.velocity,     12.0/1/2*params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( turretMeasure.steering.left.position,    M_PI_4 );
    BOOST_CHECK_EQUAL( turretMeasure.steering.right.position,   M_PI_4 );
    BOOST_CHECK_EQUAL( turretMeasure.steering.rear.position,    M_PI_4 );

    //marche avant à PI/2
    motorsMeasure.driving.left.velocity = 10;
    motorsMeasure.driving.right.velocity = 11;
    motorsMeasure.driving.rear.velocity = 12;
    motorsMeasure.steering.left.position = M_PI_2/params.getTurretRatio();
    motorsMeasure.steering.right.position = M_PI_2/params.getTurretRatio();
    motorsMeasure.steering.rear.position = M_PI_2/params.getTurretRatio();
    success = arp_model::UbiquityKinematics::motors2Turrets( motorsMeasure, turretMeasure, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( turretMeasure.driving.left.velocity,     10.0/1/2*params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.right.velocity,    11.0/1/2*params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.rear.velocity,     12.0/1/2*params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( turretMeasure.steering.left.position,    M_PI_2 );
    BOOST_CHECK_EQUAL( turretMeasure.steering.right.position,   M_PI_2 );
    BOOST_CHECK_EQUAL( turretMeasure.steering.rear.position,    M_PI_2 );

    //marche avant à PI - epsilon
    motorsMeasure.driving.left.velocity = 10;
    motorsMeasure.driving.right.velocity = 11;
    motorsMeasure.driving.rear.velocity = 12;
    motorsMeasure.steering.left.position = (M_PI - 0.01)/params.getTurretRatio();
    motorsMeasure.steering.right.position = (M_PI - 0.01)/params.getTurretRatio();
    motorsMeasure.steering.rear.position = (M_PI - 0.01)/params.getTurretRatio();
    success = arp_model::UbiquityKinematics::motors2Turrets( motorsMeasure, turretMeasure, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( turretMeasure.driving.left.velocity,     10.0/1/2*params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.right.velocity,    11.0/1/2*params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.rear.velocity,     12.0/1/2*params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( turretMeasure.steering.left.position,    (M_PI - 0.01) );
    BOOST_CHECK_EQUAL( turretMeasure.steering.right.position,   (M_PI - 0.01) );
    BOOST_CHECK_EQUAL( turretMeasure.steering.rear.position,    (M_PI - 0.01) );

    //marche ar à PI - epsilon
    motorsMeasure.driving.left.velocity = -10;
    motorsMeasure.driving.right.velocity = -11;
    motorsMeasure.driving.rear.velocity = -12;
    motorsMeasure.steering.left.position = (M_PI - 0.01)/params.getTurretRatio();
    motorsMeasure.steering.right.position = (M_PI - 0.01)/params.getTurretRatio();
    motorsMeasure.steering.rear.position = (M_PI - 0.01)/params.getTurretRatio();
    success = arp_model::UbiquityKinematics::motors2Turrets( motorsMeasure, turretMeasure, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( turretMeasure.driving.left.velocity,     -10.0/1/2*params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.right.velocity,    -11.0/1/2*params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.rear.velocity,     -12.0/1/2*params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( turretMeasure.steering.left.position,    (M_PI - 0.01) );
    BOOST_CHECK_EQUAL( turretMeasure.steering.right.position,   (M_PI - 0.01) );
    BOOST_CHECK_EQUAL( turretMeasure.steering.rear.position,    (M_PI - 0.01) );

    //marche avant à PI + espsilon
    motorsMeasure.driving.left.velocity = 10;
    motorsMeasure.driving.right.velocity = 11;
    motorsMeasure.driving.rear.velocity = 12;
    motorsMeasure.steering.left.position = (M_PI + 0.01)/params.getTurretRatio();
    motorsMeasure.steering.right.position = (M_PI + 0.01)/params.getTurretRatio();
    motorsMeasure.steering.rear.position = (M_PI + 0.01)/params.getTurretRatio();
    success = arp_model::UbiquityKinematics::motors2Turrets( motorsMeasure, turretMeasure, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( turretMeasure.driving.left.velocity,     10.0/1/2*params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.right.velocity,    11.0/1/2*params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.rear.velocity,     12.0/1/2*params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( turretMeasure.steering.left.position,    (-M_PI + 0.01) );
    BOOST_CHECK_EQUAL( turretMeasure.steering.right.position,   (-M_PI + 0.01) );
    BOOST_CHECK_EQUAL( turretMeasure.steering.rear.position,    (-M_PI + 0.01) );

    //marche ar à PI + espsilon
    motorsMeasure.driving.left.velocity = -10;
    motorsMeasure.driving.right.velocity = -11;
    motorsMeasure.driving.rear.velocity = -12;
    motorsMeasure.steering.left.position = (M_PI + 0.01)/params.getTurretRatio();
    motorsMeasure.steering.right.position = (M_PI + 0.01)/params.getTurretRatio();
    motorsMeasure.steering.rear.position = (M_PI + 0.01)/params.getTurretRatio();
    success = arp_model::UbiquityKinematics::motors2Turrets( motorsMeasure, turretMeasure, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( turretMeasure.driving.left.velocity,     -10.0/1/2*params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.right.velocity,    -11.0/1/2*params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.rear.velocity,     -12.0/1/2*params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( turretMeasure.steering.left.position,    (-M_PI + 0.01) );
    BOOST_CHECK_EQUAL( turretMeasure.steering.right.position,   (-M_PI + 0.01) );
    BOOST_CHECK_EQUAL( turretMeasure.steering.rear.position,    (-M_PI + 0.01) );

    //marche avant à PI/2
    motorsMeasure.driving.left.velocity = 10;
    motorsMeasure.driving.right.velocity = 11;
    motorsMeasure.driving.rear.velocity = 12;
    motorsMeasure.steering.left.position = M_PI_2/params.getTurretRatio();
    motorsMeasure.steering.right.position = M_PI_2/params.getTurretRatio();
    motorsMeasure.steering.rear.position = M_PI_2/params.getTurretRatio();
    success = arp_model::UbiquityKinematics::motors2Turrets( motorsMeasure, turretMeasure, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( turretMeasure.driving.left.velocity,     10.0/1/2*params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.right.velocity,    11.0/1/2*params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.rear.velocity,     12.0/1/2*params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( turretMeasure.steering.left.position,    M_PI_2 );
    BOOST_CHECK_EQUAL( turretMeasure.steering.right.position,   M_PI_2 );
    BOOST_CHECK_EQUAL( turretMeasure.steering.rear.position,    M_PI_2 );
}


//test le couplage de vitesse
BOOST_AUTO_TEST_CASE( UK_MotorTurret_velocity_decoupling )
{
    arp_model::UbiquityParams params;
    params.fillWithFakeValues();

    arp_model::TurretState turretMeasure;
    arp_model::MotorState motorsMeasure;
    bool success;

    //marche avant traction
    motorsMeasure.driving.left.velocity = 0;
    motorsMeasure.driving.right.velocity = 0;
    motorsMeasure.driving.rear.velocity = 0;
    motorsMeasure.steering.left.velocity = 1;
    motorsMeasure.steering.right.velocity = 1;
    motorsMeasure.steering.rear.velocity = 1;
    success = arp_model::UbiquityKinematics::motors2Turrets( motorsMeasure, turretMeasure, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( turretMeasure.driving.left.velocity,     params.getLeftWheelDiameter() / 2* params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.right.velocity,    params.getLeftWheelDiameter() / 2* params.getTurretRatio(), 1E-6  );
    BOOST_CHECK_CLOSE( turretMeasure.driving.rear.velocity,     params.getLeftWheelDiameter() / 2* params.getTurretRatio(), 1E-6  );

