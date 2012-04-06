/*
 * uTest_UK_Turrets2Twist.hpp
 *
 *  Created on: 06 April 2012
 *      Author: boris
 *
 *      Teste les modèles cinématiques du robot
 */

#include <boost/test/floating_point_comparison.hpp>
#include "models/UbiquityKinematics.hpp"
#include "models/UbiquityParams.hpp"


//BOOST_AUTO_TEST_CASE( UK_Turrets2Twist_ZeroToZeroTest )
//{
//    arp_model::UbiquityParams params;
//    arp_math::Twist2D twist;
//    arp_model::TurretState zeroTurretCmd;
//    arp_model::SlippageReport splippage;
//    bool res;
//    res = arp_model::UbiquityKinematics::turrets2Twist(zeroTurretCmd, twist, splippage, params);
//
//    BOOST_CHECK_EQUAL( res , true);
//    BOOST_CHECK_EQUAL( twist.vx(), 0. );
//    BOOST_CHECK_EQUAL( twist.vy(), 0. );
//    BOOST_CHECK_EQUAL( twist.vh(), 0. );
//}

BOOST_AUTO_TEST_CASE( UK_Turrets2Twist_Rotation )
{
    arp_model::UbiquityParams params;
    arp_math::Twist2D inTwist;
    arp_math::Twist2D outTwist;
    arp_model::TurretState turretCmd;
    arp_model::SlippageReport splippage;
    bool res;

    inTwist.vx( 0. );
    inTwist.vy( 0. );
    inTwist.vh( 3. );

    res = arp_model::UbiquityKinematics::twist2Turrets(inTwist, turretCmd, params);

    res = arp_model::UbiquityKinematics::turrets2Twist(turretCmd, outTwist, splippage, params);

    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_CLOSE( outTwist.vx(), inTwist.vx(), 1.f );
    BOOST_CHECK_CLOSE( outTwist.vy(), inTwist.vy(), 1.f );
    BOOST_CHECK_CLOSE( outTwist.vh(), inTwist.vh(), 1.f );
}


