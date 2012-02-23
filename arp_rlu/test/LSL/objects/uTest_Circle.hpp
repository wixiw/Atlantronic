/*
 * uTest_Circle.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/objects/Circle.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_Circle )

BOOST_AUTO_TEST_CASE( Constructor_default_1 )
{
    lsl::Circle obj;

    BOOST_CHECK_EQUAL( obj.x(), 0.);
    BOOST_CHECK_EQUAL( obj.y(), 0.);
    BOOST_CHECK_EQUAL( obj.r(), 1.);
}

BOOST_AUTO_TEST_CASE( Constructor_double_1 )
{
    lsl::Circle obj(-1., 2.);

    BOOST_CHECK_EQUAL( obj.x(), -1.);
    BOOST_CHECK_EQUAL( obj.y(),  2.);
    BOOST_CHECK_EQUAL( obj.r(),  1.);
}

BOOST_AUTO_TEST_CASE( Constructor_double_2 )
{
    lsl::Circle obj(1., 2., 3.);

    BOOST_CHECK_EQUAL( obj.x(), 1.);
    BOOST_CHECK_EQUAL( obj.y(), 2.);
    BOOST_CHECK_EQUAL( obj.r(), 3.);
}

BOOST_AUTO_TEST_CASE( Constructor_vector_1 )
{
    lsl::Circle obj(arp_math::Vector2(-1.,2.));

    BOOST_CHECK_EQUAL( obj.x(), -1.);
    BOOST_CHECK_EQUAL( obj.y(),  2.);
    BOOST_CHECK_EQUAL( obj.r(),  1.);
}

BOOST_AUTO_TEST_CASE( Constructor_vector_2 )
{
    lsl::Circle obj(arp_math::Vector2(-1.,2.), 3.);

    BOOST_CHECK_EQUAL( obj.x(), -1.);
    BOOST_CHECK_EQUAL( obj.y(),  2.);
    BOOST_CHECK_EQUAL( obj.r(),  3.);
}

BOOST_AUTO_TEST_CASE( Constructor_x )
{
    lsl::Circle obj(arp_math::Vector2(-1.,2.), 3.);

    BOOST_CHECK_EQUAL( obj.x(), -1.);

    obj.x(10.);

    BOOST_CHECK_EQUAL( obj.x(), 10.);
}

BOOST_AUTO_TEST_CASE( Constructor_y )
{
    lsl::Circle obj(arp_math::Vector2(-1.,2.), 3.);

    BOOST_CHECK_EQUAL( obj.y(),  2.);

    obj.y(10.);

    BOOST_CHECK_EQUAL( obj.y(), 10.);
}

BOOST_AUTO_TEST_CASE( Constructor_r )
{
    lsl::Circle obj(arp_math::Vector2(-1.,2.), 3.);

    BOOST_CHECK_EQUAL( obj.r(),  3.);

    obj.r(10.);

    BOOST_CHECK_EQUAL( obj.r(), 10.);
}

BOOST_AUTO_TEST_CASE( Constructor_getPosition )
{
    lsl::Circle obj(arp_math::Vector2(-1.,2.), 3.);

    arp_math::Vector2 pos1 = obj.getPosition();

    BOOST_CHECK_EQUAL( pos1(0),  -1.);
    BOOST_CHECK_EQUAL( pos1(1),  2.);

    obj.setPosition(arp_math::Vector2(10., 20.));
    arp_math::Vector2 pos2 = obj.getPosition();

    BOOST_CHECK_EQUAL( pos2(0),  10.);
    BOOST_CHECK_EQUAL( pos2(1),  20.);
}


BOOST_AUTO_TEST_SUITE_END()
