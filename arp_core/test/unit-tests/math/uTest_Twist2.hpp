/*
 * test_Twist2.cpp
 *
 *  Created on: 13 sept. 2010
 *      Author: boris
 */

#include "math/Twist2.hpp"
using namespace ARPMath;

BOOST_AUTO_TEST_CASE( Twist2_Default_Constructor )
{
	// Test default constructor
	Twist2 a;
	BOOST_CHECK( a.VRot() == 0.0 );
    BOOST_CHECK( a.VTrans() == Vector2(0,0) );
}

BOOST_AUTO_TEST_CASE( Twist2_Accessors )
{
	Twist2 a;
	a.VTrans(Vector2(1,2));
	a.VRot(3);
	BOOST_CHECK( a.VTrans() == Vector2(1,2) );
	BOOST_CHECK( a.VRot() == 3 );
}
