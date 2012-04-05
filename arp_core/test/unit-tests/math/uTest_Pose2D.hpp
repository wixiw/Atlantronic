/*
 * test_Pose2D.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

// TODO BMO: test unitaire pour l'opérateur <<

#include "math/Pose2D.hpp"
#include <iostream>
using namespace std;
using namespace arp_math;

BOOST_AUTO_TEST_CASE( Pose2D_Default_Constructor )
{
	// Test default constructor
	Pose2D a;
	BOOST_CHECK( a.angle() == 0.0 );
	BOOST_CHECK( a.translation() == Vector2(0,0) );
}

BOOST_AUTO_TEST_CASE( Pose2D_Accessors )
{
	Pose2D a;
	a.translation(Vector2(1,2));
	Rotation2 r(M_PI/2);
	a.orientation(r);
	BOOST_CHECK( a.translation() == Vector2(1,2) );
	BOOST_CHECK( a.orientation().angle() == M_PI/2 );

	a.x(3);
	a.y(4);
	a.angle(M_PI/6);
	BOOST_CHECK( a.translation() == Vector2(3,4) );
	BOOST_CHECK( a.orientation().angle() == M_PI/6 );
	BOOST_CHECK( a.x() == 3 );
	BOOST_CHECK( a.y() == 4 );
	BOOST_CHECK( a.h() == M_PI/6 );

	Eigen::Matrix<double,3,3> mat3;
	double cosA = std::cos(a.angle());
	double sinA = std::sin(a.angle());
	mat3 << cosA, -sinA, a.x(),
    		sinA,  cosA, a.y(),
		     0  ,   0  ,  1   ;
	BOOST_CHECK( a.getDisplacement2Matrix() == mat3 );

//	Eigen::Matrix<double,4,4> mat4;
//	mat4 << cosA, -sinA, 0 , a.x(),
//    		sinA,  cosA, 0 , a.y(),
//		     0  ,   0  , 1 ,  0   ,
//		     0  ,   0  , 0 ,  1   ;
//	BOOST_CHECK( a.matrix4() == mat4 );
}

BOOST_AUTO_TEST_CASE( Pose2D_Other_Constructors )
{
	Pose2D a;
	a.translation(Vector2(1,2));
	a.h(M_PI/2);

	Pose2D b(a);
	BOOST_CHECK( b.h() == M_PI/2 );
	BOOST_CHECK( b.translation() == Vector2(1,2) );

	Pose2D c(Vector2(3,4), Rotation2(M_PI/6));
	BOOST_CHECK( c.h() == M_PI/6 );
	BOOST_CHECK( c.translation() == Vector2(3,4) );
}

BOOST_AUTO_TEST_CASE( Pose2D_Inverse )
{
	Pose2D a;
	a.translation(Vector2(1,2));
	a.h(M_PI/4);
	BOOST_CHECK( a.inverse().h() == -M_PI/4 );
	BOOST_CHECK( a.inverse().translation() == Vector2(-1,-2) );
}

BOOST_AUTO_TEST_CASE( Pose2D_Operators )
{
	Pose2D a,c;
	a.translation(Vector2(1,2));
	a.h(M_PI/4);
	c.translation(Vector2(3,4));
	c.h(M_PI/6);

	Pose2D b;
	b = a;
	BOOST_CHECK( b.translation() == Vector2(1,2) );
	BOOST_CHECK( b.h() == M_PI/4 );
	BOOST_CHECK( b == a  );

//	Pose2D d = c + a;
//	BOOST_CHECK( d.translation() == (c.translation() + a.translation()) );
//	BOOST_CHECK( d.h() == (c.h() + a.h()) );
//
//	Pose2D e = c - a;
//	BOOST_CHECK( e.translation() == (c.translation() - a.translation()) );
//	BOOST_CHECK( e.h() == (c.h() - a.h()) );

	// Ecrire ici le test unitaire pour l'operateur <<
}

BOOST_AUTO_TEST_CASE( Pose2D_BigAdjoint )
{
    Pose2D zero;
    BigAdjoint2 id = BigAdjoint2::Identity();
    BigAdjoint2 res;

    res = zero.getBigAdjoint();
    BOOST_CHECK_EQUAL( res , id );

    Pose2D quartTour(0,0,M_PI_2);
    Eigen::Matrix<double,3,3> mat3;
    double cosA = std::cos(M_PI_2);
    double sinA = std::sin(M_PI_2);
    mat3 << 1, 0,       0,
            0, cosA,    -sinA,
            0, sinA,   cosA   ;

    res = quartTour.getBigAdjoint();
    for( int i = 0 ; i< 3 ; i++ )
    {
        for( int j = 0 ; j< 3 ; j++ )
        {
            //cerr << "(i,j)=(" << i << "," << j << ")" << endl;
            if( fabs(mat3(i,j)) >= 1E-6 )
            {
                BOOST_CHECK_CLOSE( res(i,j) , mat3(i,j), 1E-6 );
            }
            else
            {
                BOOST_CHECK_SMALL( res(i,j) , 1E-6 );
            }
        }
    }


    Pose2D demitour(0,0,2.16);
    cosA = std::cos(2.16);
    sinA = std::sin(2.16);
    mat3 << 1, 0,       0,
            0, cosA,    -sinA,
            0, sinA,   cosA   ;

    res = demitour.getBigAdjoint();
    for( int i = 0 ; i< 3 ; i++ )
    {
        for( int j = 0 ; j< 3 ; j++ )
        {
            //cerr << "(i,j)=(" << i << "," << j << ")" << endl;
            if( fabs(mat3(i,j)) >= 1E-6 )
            {
                BOOST_CHECK_CLOSE( res(i,j) , mat3(i,j), 1E-6 );
            }
            else
            {
                BOOST_CHECK_SMALL( res(i,j) , 1E-6 );
            }
        }
    }

    Pose2D p(3,2,2.16);
    mat3 << 1, 0,       0,
            2, cosA,    -sinA,
            -3, sinA,   cosA   ;

    res = p.getBigAdjoint();

    //cerr << "res = " << res << endl;
    //cerr << "mat3 = " << mat3 << endl;

    for( int i = 0 ; i< 3 ; i++ )
    {
        for( int j = 0 ; j< 3 ; j++ )
        {
            //cerr << "(i,j)=(" << i << "," << j << ")" << endl;
            if( fabs(mat3(i,j)) >= 1E-6 )
            {
                BOOST_CHECK_CLOSE( res(i,j) , mat3(i,j), 1E-6 );
            }
            else
            {
                BOOST_CHECK_SMALL( res(i,j) , 1E-6 );
            }
        }
    }
}
