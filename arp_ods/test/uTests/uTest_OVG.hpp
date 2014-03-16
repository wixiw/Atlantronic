/*
 * uTest_OVG.cpp
 *
 *  Created on: 16 mars 2014
 *      Author: willy
 */

#include "control/planners/OnlineSpeedGenerator.hpp"
#include <iostream>

using namespace std;
using namespace arp_ods;
using namespace arp_math;


BOOST_AUTO_TEST_CASE( OVG_test )
{
    double vMax = 1.;
    double aMax = 1.;
    double jMax = 1.;
    OnlineSpeedGenerator ovg(0.010);
    PosVelAcc state;
    PosVelAcc reachableState;

    state = PosVelAcc(0., 0., 0.);
    vMax = 1.;
    aMax = 1.;
    jMax = 1.;
    ovg.setDynamicLimitations(vMax, aMax, jMax);
    ovg.computeNextStep(0, state, reachableState);
    BOOST_CHECK(fabs(reachableState.velocity) <= vMax );
    BOOST_CHECK(fabs(reachableState.acceleration) <= aMax );

    state = PosVelAcc(2., 0., 0.);
    vMax = 1.;
    aMax = 1.;
    jMax = 1.;
    ovg.setDynamicLimitations(vMax, aMax, jMax);
    ovg.computeNextStep(0, state, reachableState);
    BOOST_CHECK(fabs(reachableState.velocity) <= vMax );
    BOOST_CHECK(fabs(reachableState.acceleration) <= aMax );

    state = PosVelAcc(0., -2., 0.);
    vMax = 1.;
    aMax = 1.;
    jMax = 1.;
    ovg.setDynamicLimitations(vMax, aMax, jMax);
    ovg.computeNextStep(0, state, reachableState);
    BOOST_CHECK(fabs(reachableState.velocity) <= vMax );
    BOOST_CHECK(fabs(reachableState.acceleration) <= aMax );

    state = PosVelAcc(0., 0., -5.);
    vMax = 1.;
    aMax = 1.;
    jMax = 1.;
    ovg.setDynamicLimitations(vMax, aMax, jMax);
    ovg.computeNextStep(0, state, reachableState);
    BOOST_CHECK(fabs(reachableState.velocity) <= vMax );
    BOOST_CHECK(fabs(reachableState.acceleration) <= aMax );

    state = PosVelAcc(0., 0., -7.);
    vMax = 1.;
    aMax = 2.;
    jMax = 1.;
    ovg.setDynamicLimitations(vMax, aMax, jMax);
    ovg.computeNextStep(0, state, reachableState);
    BOOST_CHECK(fabs(reachableState.velocity) <= vMax );
    BOOST_CHECK(fabs(reachableState.acceleration) <= aMax );

    state = PosVelAcc(-2., -3., -7.);
    vMax = 1.;
    aMax = 2.;
    jMax = 1.;
    ovg.setDynamicLimitations(vMax, aMax, jMax);
    ovg.computeNextStep(0, state, reachableState);
    BOOST_CHECK(fabs(reachableState.velocity) <= vMax );
    BOOST_CHECK(fabs(reachableState.acceleration) <= aMax );


}
