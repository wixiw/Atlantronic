/*
 * uTest_MotionOrder.hpp
 *
 *  Created on: 24 may 2011
 *      Author: wla
 */

#include "orders/orders.h"
#include "math/math.hpp"
using namespace arp_ods;
using namespace arp_math;

BOOST_AUTO_TEST_CASE( MotionOrderConstructors )
{
    // Test default constructor
    MotionOrder a;
    BOOST_CHECK( a.getMode() == MODE_INIT );
    BOOST_CHECK( a.getType() == NO_ORDER );
    BOOST_CHECK( a.getTypeString() == "NO_ORDER" );

    BOOST_CHECK( order::defaultOrder != NULL);
    BOOST_CHECK( order::defaultOrder->getMode() == MODE_INIT );
    BOOST_CHECK( order::defaultOrder->getType() == NO_ORDER );
    BOOST_CHECK( order::defaultOrder->getTypeString() == "NO_ORDER" );
}

BOOST_AUTO_TEST_CASE( MotionOrderComputeSpeed )
{
    MotionOrder o;
    Pose currentPosition;
    currentPosition.x = 1.3;
    currentPosition.y = -0.7;
    currentPosition.theta = 0.2;
    Velocity v = o.computeSpeed(currentPosition);
    BOOST_CHECK_EQUAL( v.linear, 0.0 );
    BOOST_CHECK_EQUAL( v.angular, 0.0 );
}

BOOST_AUTO_TEST_CASE( MotionOrderReversePosition )
{
    MotionOrder o;
    o.setReverse(false);
    Pose currentPosition;
    currentPosition.x = 1.3;
    currentPosition.y = -0.7;
    currentPosition.theta = 0.2;
    Pose reversedPosition = o.reversePosition(currentPosition);
    BOOST_CHECK_EQUAL( reversedPosition.x, currentPosition.x );
    BOOST_CHECK_EQUAL( reversedPosition.y, currentPosition.y );
    BOOST_CHECK_EQUAL( reversedPosition.theta, currentPosition.theta );

    o.setReverse(false);
    currentPosition.theta = 10*PI + 0.2;
    reversedPosition = o.reversePosition(currentPosition);
    BOOST_CHECK_EQUAL( reversedPosition.x, currentPosition.x );
    BOOST_CHECK_EQUAL( reversedPosition.y, currentPosition.y );
    BOOST_CHECK_CLOSE( reversedPosition.theta, 0.2, 0.00001f );

    o.setReverse(true);
    currentPosition.theta = 0.2;
    reversedPosition = o.reversePosition(currentPosition);
    BOOST_CHECK_EQUAL( reversedPosition.x, currentPosition.x );
    BOOST_CHECK_EQUAL( reversedPosition.y, currentPosition.y );
    BOOST_CHECK_CLOSE( reversedPosition.theta, 0.2 - PI , 0.00001f );

    o.setReverse(true);
    currentPosition.theta = 10*PI + 0.2;
    reversedPosition = o.reversePosition(currentPosition);
    BOOST_CHECK_EQUAL( reversedPosition.x, currentPosition.x );
    BOOST_CHECK_EQUAL( reversedPosition.y, currentPosition.y );
    BOOST_CHECK_CLOSE( reversedPosition.theta, 0.2 - PI , 0.00001f );

    o.setReverse(true);
    currentPosition.theta = PI - 0.1;
    reversedPosition = o.reversePosition(currentPosition);
    BOOST_CHECK_EQUAL( reversedPosition.x, currentPosition.x );
    BOOST_CHECK_EQUAL( reversedPosition.y, currentPosition.y );
    BOOST_CHECK_CLOSE( reversedPosition.theta, -0.1, 0.00001f );
}
