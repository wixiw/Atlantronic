/*
 * test_Pose.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

// TODO BMO: test unitaire pour l'opérateur <<

#include "math/Pose.hpp"
using namespace ARPMath;

BOOST_AUTO_TEST_CASE( Pose_Default_Constructor )
{
	// Test default constructor
	Pose a;
	BOOST_CHECK( a.angle() == 0.0 );
	BOOST_CHECK( a.translation() == Vector2(0,0) );
}

BOOST_AUTO_TEST_CASE( Pose_Accessors )
{
	Pose a;
	a.translation(Vector2(1,2));
	Rotation2 r(M_PI/2);
	a.rotation2(r);
	BOOST_CHECK( a.translation() == Vector2(1,2) );
	BOOST_CHECK( a.rotation2().angle() == M_PI/2 );

	a.x(3);
	a.y(4);
	a.angle(M_PI/6);
	BOOST_CHECK( a.translation() == Vector2(3,4) );
	BOOST_CHECK( a.rotation2().angle() == M_PI/6 );
	BOOST_CHECK( a.x() == 3 );
	BOOST_CHECK( a.y() == 4 );
	BOOST_CHECK( a.angle() == M_PI/6 );

	Eigen::Matrix<double,3,3> mat3;
	double cosA = Eigen::ei_cos(a.angle());
	double sinA = Eigen::ei_sin(a.angle());
	mat3 << cosA, -sinA, a.x(),
    		sinA,  cosA, a.y(),
		     0  ,   0  ,  1   ;
	BOOST_CHECK( a.matrix3() == mat3 );

//	Eigen::Matrix<double,4,4> mat4;
//	mat4 << cosA, -sinA, 0 , a.x(),
//    		sinA,  cosA, 0 , a.y(),
//		     0  ,   0  , 1 ,  0   ,
//		     0  ,   0  , 0 ,  1   ;
//	BOOST_CHECK( a.matrix4() == mat4 );
}

BOOST_AUTO_TEST_CASE( Pose_Other_Constructors )
{
	Pose a;
	a.translation(Vector2(1,2));
	a.angle(M_PI/2);

	Pose b(a);
	BOOST_CHECK( b.angle() == M_PI/2 );
	BOOST_CHECK( b.translation() == Vector2(1,2) );

	Pose c(Vector2(3,4), Rotation2(M_PI/6));
	BOOST_CHECK( c.angle() == M_PI/6 );
	BOOST_CHECK( c.translation() == Vector2(3,4) );
}

BOOST_AUTO_TEST_CASE( Pose_Inverse )
{
	Pose a;
	a.translation(Vector2(1,2));
	a.angle(M_PI/4);
	BOOST_CHECK( a.inverse().angle() == -M_PI/4 );
	BOOST_CHECK( a.inverse().translation() == Vector2(-1,-2) );
}

BOOST_AUTO_TEST_CASE( Pose_Operators )
{
	Pose a,c;
	a.translation(Vector2(1,2));
	a.angle(M_PI/4);
	c.translation(Vector2(3,4));
	c.angle(M_PI/6);

	Pose b;
	b = a;
	BOOST_CHECK( b.translation() == Vector2(1,2) );
	BOOST_CHECK( b.angle() == M_PI/4 );
	BOOST_CHECK( b == a  );

//	Pose d = c + a;
//	BOOST_CHECK( d.translation() == (c.translation() + a.translation()) );
//	BOOST_CHECK( d.angle() == (c.angle() + a.angle()) );
//
//	Pose e = c - a;
//	BOOST_CHECK( e.translation() == (c.translation() - a.translation()) );
//	BOOST_CHECK( e.angle() == (c.angle() - a.angle()) );

	// Ecrire ici le test unitaire pour l'operateur <<
}
