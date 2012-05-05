/*
 * test_Pose2D.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

// TODO BMO: test unitaire pour l'opérateur <<

#include "math/Pose2D.hpp"
#include <math/MathFactory.hpp>
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

    Pose2D c = MathFactory::createPose2D(Vector2(3,4), Rotation2(M_PI/6));
    BOOST_CHECK( c.h() == M_PI/6 );
    BOOST_CHECK( c.translation() == Vector2(3,4) );
}

BOOST_AUTO_TEST_CASE( Pose2D_Inverse )
{
    Pose2D a;
    a.translation(Vector2(3,1));
    a.h(M_PI/2);
    Pose2D b = a.inverse();
    BOOST_CHECK( b.h() == -M_PI/2 );
    BOOST_CHECK_SMALL( b.x() - (-1), 1.e-10 );
    BOOST_CHECK_SMALL( b.y() - 3, 1.e-10 );
}

BOOST_AUTO_TEST_CASE( Pose2D_Operators_equal )
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
}

BOOST_AUTO_TEST_CASE( Pose2D_Operators_product_1 )
{
    Pose2D a;
    a.translation(Vector2(1,2));
    a.h(M_PI/4);

    Pose2D b = a * a.inverse();
    BOOST_CHECK_SMALL( b.x() - 0., 1.e-10 );
    BOOST_CHECK_SMALL( b.y() - 0., 1.e-10 );
    BOOST_CHECK_EQUAL( b.angle() , 0. );


    Pose2D c = MathFactory::createPose2D( Vector2(2,-1), Rotation2(-M_PI/2) );
    Pose2D d = MathFactory::createPose2D( Vector2(-2,1), Rotation2(M_PI) );
    Pose2D e = c * d;
    BOOST_CHECK_SMALL( e.x() - 3., 1.e-10 );
    BOOST_CHECK_SMALL( e.y() - 1., 1.e-10 );
    BOOST_CHECK_SMALL( e.angle() - M_PI/2., 1.e-10 );
}

BOOST_AUTO_TEST_CASE( Pose2D_Operators_product_2 )
{
    {
        Pose2D a;
        Vector2 v1(1., -2.);
        Vector2 v2 = a * v1;
        BOOST_CHECK( v1 == v2 );
    }
    {
        Pose2D H_1_0(1, -2, M_PI/2);
        Vector2 v_1(3., 0.5);
        Vector2 v_0 = H_1_0 * v_1;
        BOOST_CHECK_CLOSE( v_0(0), 0.5, 1.f );
        BOOST_CHECK_CLOSE( v_0(1), 1.0, 1.f );

        Vector2 v = H_1_0.inverse() * v_0;
        BOOST_CHECK_CLOSE( v(0), v_1(0), 1.f );
        BOOST_CHECK_CLOSE( v(1), v_1(1), 1.f );
    }
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
    mat3 << cosA,    -sinA, 0,
            sinA,     cosA, 0,
            0 ,       0 , 1;

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
    mat3 << cosA,    -sinA, 0,
            sinA,     cosA, 0,
            0 ,       0 , 1;

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
    mat3 << cosA,    -sinA, 2,
            sinA,     cosA, -3,
            0 ,       0 , 1;

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
