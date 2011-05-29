/*
 * uTest_ModeSelector.hpp
 *
 *  Created on: 24 may 2011
 *      Author: wla
 */

#include "orders/orders.h"
using namespace arp_ods;

BOOST_AUTO_TEST_CASE( ModeSelectorConstructors )
{
    // Test default constructor
    ModeSelector a;
    BOOST_CHECK( a.getMode() == MODE_INIT );
}

BOOST_AUTO_TEST_CASE( ModeSelectorDistances )
{
    ModeSelector ms;
    Pose beginPose;
    beginPose.x = 0.000;
    beginPose.y = 0.000;
    beginPose.theta = 0;
    Pose endPose;
    endPose.x = 1.000;
    endPose.y = 0.500;
    endPose.theta = 1.2;
    Pose currentPose;
    currentPose.x = 0.020;
    currentPose.y = 0.000;
    currentPose.theta = 0.3;

    ms.setBeginPose(beginPose);
    ms.setEndPose(endPose);

    BOOST_CHECK_EQUAL( ms.getCoveredDistance(beginPose) , 0.0 );
    BOOST_CHECK_EQUAL( ms.getRemainingDistance(beginPose) , sqrt(endPose.x*endPose.x+endPose.y*endPose.y) );
    BOOST_CHECK_EQUAL( ms.getRemainingAngle(beginPose) , 1.2 );

    BOOST_CHECK_CLOSE( ms.getCoveredDistance(currentPose), currentPose.x , 0.0001f);
    BOOST_CHECK_CLOSE( ms.getRemainingAngle(currentPose), 0.9f , 0.0001f);

    BOOST_CHECK_EQUAL( ms.getCoveredDistance(endPose) , sqrt(endPose.x*endPose.x+endPose.y*endPose.y) );
    BOOST_CHECK_EQUAL( ms.getRemainingDistance(endPose) , 0.0 );
    BOOST_CHECK_EQUAL( ms.getRemainingAngle(endPose) , 0 );
}

BOOST_AUTO_TEST_CASE( ModeSelectorModesNoPass )
{
    ModeSelector ms;
    Pose beginPose;
    beginPose.x = 0.000;
    beginPose.y = 0.000;
    beginPose.theta = 0;
    Pose endPose;
    endPose.x = 1.000;
    endPose.y = 0.500;
    endPose.theta = 1.2;
    Pose currentPose;
    currentPose.x = 0.000;
    currentPose.y = 0.000;
    currentPose.theta = 0.3;


    ms.setBeginPose(beginPose);
    ms.setEndPose(endPose);
    ms.setRadiusInitZone(0.010);
    ms.setRadiusApproachZone(0.050);
    ms.setAngleAccuracy(0.2);
    ms.setDistanceAccurancy(0.010);
    ms.setPass(false);

    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_INIT );

    currentPose.x = 0.020;
    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_RUN );

    currentPose.x = 1.000;
    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_RUN );

    currentPose.x = 1.000;
    currentPose.y = 0.455;
    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_APPROACH );

    currentPose.x = endPose.x - 0.011;
    currentPose.y = endPose.y;
    currentPose.theta = endPose.theta;
    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_APPROACH );

    currentPose.x = endPose.x - 0.011;
    currentPose.y = endPose.y;
    currentPose.theta = endPose.theta;
    ms.setDistanceAccurancy(0.020);
    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_DONE );
}


BOOST_AUTO_TEST_CASE( ModeSelectorModesPass )
{
    ModeSelector ms;
    Pose beginPose;
    beginPose.x = 0.000;
    beginPose.y = 0.000;
    beginPose.theta = 0;
    Pose endPose;
    endPose.x = 1.000;
    endPose.y = 0.500;
    endPose.theta = 1.2;
    Pose currentPose;
    currentPose.x = 0.000;
    currentPose.y = 0.000;
    currentPose.theta = 0.3;


    ms.setBeginPose(beginPose);
    ms.setEndPose(endPose);
    ms.setRadiusInitZone(0.010);
    ms.setRadiusApproachZone(0.050);
    ms.setAngleAccuracy(0.2);
    ms.setDistanceAccurancy(0.010);
    ms.setPass(true);
    ms.setPassTimeout(0.5);

    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_INIT );

    currentPose.x = 0.020;
    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_RUN );

    currentPose.x = 1.000;
    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_RUN );

    currentPose.x = 1.000;
    currentPose.y = 0.455;
    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_PASS );
    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_PASS );
    sleep(1);
    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_DONE );

}

BOOST_AUTO_TEST_CASE( ModeSelectorModesTimeout )
{
    ModeSelector ms;
    Pose beginPose;
    beginPose.x = 0.000;
    beginPose.y = 0.000;
    beginPose.theta = 0;
    Pose endPose;
    endPose.x = 1.000;
    endPose.y = 0.500;
    endPose.theta = 1.2;
    Pose currentPose;
    currentPose.x = 0.000;
    currentPose.y = 0.000;
    currentPose.theta = 0.3;


    ms.setBeginPose(beginPose);
    ms.setEndPose(endPose);
    ms.setRadiusInitZone(0.010);
    ms.setRadiusApproachZone(0.050);
    ms.setAngleAccuracy(0.2);
    ms.setDistanceAccurancy(0.010);
    ms.setPass(true);
    ms.setPassTimeout(0.5);
    ms.setOrderTimeout(2.0);

    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_INIT );

    sleep(3);
    ms.switchMode(currentPose);
    BOOST_CHECK_EQUAL( ms.getMode() , MODE_ERROR );

}

