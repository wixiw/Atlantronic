/*
 * test_ICRSpeed.hpp
 *
 *  Created on: 28 april 2012
 *      Author: romain
 */

#include <math/core>
#include <iostream>
using namespace arp_math;
using namespace std;

BOOST_AUTO_TEST_CASE( ICRSpeed_Default_Constructor )
{
	// Test default constructor
	ICRSpeed a;
	BOOST_CHECK( a.ro() == PI/2.0 );
	BOOST_CHECK( a.alpha() == PI/2.0 );
	BOOST_CHECK( a.q() == 0.0 );
}

BOOST_AUTO_TEST_CASE( ICRSpeed_Accessors )
{
    ICRSpeed a;
	a.ro(1.);
	a.alpha(2.);
	a.q(3.);
	BOOST_CHECK( a.ro() == 1. );
	BOOST_CHECK( a.alpha() == 2. );
	BOOST_CHECK( a.q() == 3. );
}

BOOST_AUTO_TEST_CASE( ICRSpeed_Consistency )
{
    // default ICRSpeed should give zero speed twist
    ICRSpeed ICR_a;
    Twist2D twist_a=ICR_a.twist();

    BOOST_CHECK_CLOSE( twist_a.vx() , 0.0 ,1e-6);
    BOOST_CHECK_CLOSE( twist_a.vy() , 0.0 ,1e-6 );
    BOOST_CHECK_CLOSE( twist_a.vh() , 0.0 ,1e-6);

    //zero speed twist should give an ICRSpeed as the default
    Twist2D twist_b(0,0,0);
    ICRSpeed ICR_b(twist_b);
    BOOST_CHECK_CLOSE( ICR_b.ro() , PI/2.0 ,1e-6);
    BOOST_CHECK_CLOSE( ICR_b.alpha() , PI/2.0 ,1e-6 );
    BOOST_CHECK_CLOSE( ICR_b.q() , 0.0 ,1e-6);

    // going from twist to ICRSpeed back to twist should be ok
    Twist2D twist_from;
    ICRSpeed ICR_to;
    Twist2D twist_back;

    twist_from=Twist2D(1,0,0);
    ICR_to=ICRSpeed(twist_from);
    twist_back=ICR_to.twist();
    BOOST_CHECK_CLOSE( twist_from.vx() , twist_back.vx() ,1e-6);
    BOOST_CHECK_CLOSE( twist_from.vy() , twist_back.vy() ,1e-6);
    BOOST_CHECK_CLOSE( twist_from.vh() , twist_back.vh() ,1e-6);

    twist_from=Twist2D(0,1,0);
    ICR_to=ICRSpeed(twist_from);
    twist_back=ICR_to.twist();
    //c'est bien gentil boost check close mais la tolerance est un pourcentage des arguments et comme j'ai un argument à 0 j'ai un fail alors que ca allait bien
    //http://www.boost.org/doc/libs/1_34_1/libs/test/doc/components/test_tools/floating_point_comparison.html
    BOOST_CHECK_SMALL( twist_from.vx() - twist_back.vx() ,1e-6);
    BOOST_CHECK_CLOSE( twist_from.vy() , twist_back.vy() ,1e-6);
    BOOST_CHECK_CLOSE( twist_from.vh() , twist_back.vh() ,1e-6);

    twist_from=Twist2D(0,0,1);
    ICR_to=ICRSpeed(twist_from);
    twist_back=ICR_to.twist();
    BOOST_CHECK_CLOSE( twist_from.vx() , twist_back.vx() ,1e-6);
    BOOST_CHECK_CLOSE( twist_from.vy() , twist_back.vy() ,1e-6);
    BOOST_CHECK_CLOSE( twist_from.vh() , twist_back.vh() ,1e-6);

    twist_from=Twist2D(1,1,0);
    ICR_to=ICRSpeed(twist_from);
    twist_back=ICR_to.twist();
    BOOST_CHECK_CLOSE( twist_from.vx() , twist_back.vx() ,1e-6);
    BOOST_CHECK_CLOSE( twist_from.vy() , twist_back.vy() ,1e-6);
    BOOST_CHECK_CLOSE( twist_from.vh() , twist_back.vh() ,1e-6);

    twist_from=Twist2D(1.875,2.414,0.654);
    ICR_to=ICRSpeed(twist_from);
    twist_back=ICR_to.twist();
    BOOST_CHECK_CLOSE( twist_from.vx() , twist_back.vx() ,1e-6);
    BOOST_CHECK_CLOSE( twist_from.vy() , twist_back.vy() ,1e-6);
    BOOST_CHECK_CLOSE( twist_from.vh() , twist_back.vh() ,1e-6);
}
