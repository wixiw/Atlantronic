/*
 * test_Twist2.cpp
 *
 *  Created on: 13 sept. 2010
 *      Author: boris
 */

#include <math/core>
#include <iostream>
using namespace arp_math;
using namespace std;

BOOST_AUTO_TEST_CASE( Twist2_Default_Constructor )
{
	// Test default constructor
	Twist2D a;
	BOOST_CHECK( a.vx() == 0.0 );
	BOOST_CHECK( a.vy() == 0.0 );
	BOOST_CHECK( a.vh() == 0.0 );
}

BOOST_AUTO_TEST_CASE( Twist2_Accessors )
{
	Twist2D a;
	a.vx(1.);
	a.vy(2.);
	a.vh(3.);
	BOOST_CHECK( a.vx() == 1. );
	BOOST_CHECK( a.vy() == 2. );
	BOOST_CHECK( a.vh() == 3. );
}

BOOST_AUTO_TEST_CASE( Twist2D_SpeedAngle )
{
    Twist2D a(1,0);
    BOOST_CHECK_EQUAL( a.speedAngle() , 0 );
    Twist2D b(0,1);
    BOOST_CHECK_CLOSE( b.speedAngle() , M_PI_2, 1E-6);
    Twist2D c(0,-1);
    BOOST_CHECK_CLOSE( c.speedAngle() , -M_PI_2, 1E-6);
    Twist2D d(-1,0);
    BOOST_CHECK_CLOSE( d.speedAngle() , M_PI, 1E-6);
    Twist2D e(1,1);
    BOOST_CHECK_CLOSE( e.speedAngle() , M_PI/4, 1E-6);
    Twist2D f(1,-1);
    BOOST_CHECK_CLOSE( f.speedAngle() , -M_PI/4, 1E-6);
    Twist2D g(-1,1);
    BOOST_CHECK_CLOSE( g.speedAngle() , 3*M_PI/4, 1E-6);
    Twist2D h(-1,-1);
    BOOST_CHECK_CLOSE( h.speedAngle() , -3*M_PI/4, 1E-6);
}

BOOST_AUTO_TEST_CASE( Twist2_distance )
{
    Twist2D a(1,0);
    Twist2D b(0,1);
    Twist2D c(0,0,1);
    Twist2D d(2,-1,2);
    Twist2D zero;
    Vector3 coef1(1,0,0);
    Vector3 coef2(0,1,0);
    Vector3 coef3(0,0,1);
    Vector3 coef(1,1,1);
    BOOST_CHECK_EQUAL( a.distanceTo(zero,1,1) , 1 );
    BOOST_CHECK_EQUAL( a.distanceTo(zero,2,2) , 2 );
    BOOST_CHECK_EQUAL( a.distanceTo(zero,0,1) , 0 );

    BOOST_CHECK_EQUAL( b.distanceTo(zero,1,1) , 1 );
    BOOST_CHECK_EQUAL( b.distanceTo(zero,2,2) , 2 );
    BOOST_CHECK_EQUAL( b.distanceTo(zero,0,1) , 0 );

    BOOST_CHECK_EQUAL( c.distanceTo(zero,1,1) , 1 );
    BOOST_CHECK_EQUAL( c.distanceTo(zero,2,2) , 2 );
    BOOST_CHECK_EQUAL( c.distanceTo(zero,0,1) , 1 );

    BOOST_CHECK_EQUAL( d.distanceTo(zero,1,1) , 3 );
}

BOOST_AUTO_TEST_CASE( Twist2_Equal )
{
    Twist2D a(1,2.3,-1.7);
    Twist2D b(1,2.3,-1.7);
    BOOST_CHECK( a == b );
    Twist2D c(1,0,0);
    Twist2D d(0,0,0);
    BOOST_CHECK( ! (c == d) );
    Twist2D e(0,1,0);
    Twist2D f(0,0,0);
    BOOST_CHECK( ! (e == f) );
    Twist2D g(0,0,1);
    Twist2D h(0,0,0);
    BOOST_CHECK( ! (g == h) );
}

BOOST_AUTO_TEST_CASE( Twist2_TransportTranslation )
{
    Twist2D a(1.3,-2.8,0);
    Twist2D res;
    Pose2D x(1,0);
    Pose2D y(0,1);
    Pose2D demitour(0,0,M_PI);
    Pose2D quarttour(0,0,M_PI_2);

    BOOST_CHECK( a.transport(x) == a );
    BOOST_CHECK( a.transport(y) == a );

    res = a.transport(demitour);
    BOOST_CHECK_CLOSE( res.vx() ,  -a.vx() , 1E-6);
    BOOST_CHECK_CLOSE( res.vy() ,  -a.vy() , 1E-6);
    BOOST_CHECK_CLOSE( res.vh() ,  a.vh()  , 1E-6);

    res = a.transport(quarttour);
    BOOST_CHECK_CLOSE( res.vx() ,  a.vy() , 1E-6);
    BOOST_CHECK_CLOSE( res.vy() ,  -a.vx(), 1E-6);
    BOOST_CHECK_CLOSE( res.vh() ,  a.vh() , 1E-6);
}

BOOST_AUTO_TEST_CASE( Twist2_TransportRotation )
{
    Twist2D a(0,0,2.6);
    Twist2D res;
    Pose2D demitour(0,0,M_PI);
    Pose2D quarttour(0,0,M_PI_2);
    Pose2D x(1,0);
    Pose2D y(0,1);

    res = a.transport(demitour);
    BOOST_CHECK_CLOSE( res.vx() ,  0 , 1E-6);
    BOOST_CHECK_CLOSE( res.vy() ,  0 , 1E-6);
    BOOST_CHECK_CLOSE( res.vh() ,  2.6  , 1E-6);

    res = a.transport(quarttour);
    BOOST_CHECK_CLOSE( res.vx() ,  0, 1E-6);
    BOOST_CHECK_CLOSE( res.vy() ,  0, 1E-6);
    BOOST_CHECK_CLOSE( res.vh() ,  2.6, 1E-6);

    res = a.transport(x);
    //cerr << x.toString() << " -> " << x.getBigAdjoint() << endl;
    //cerr << a.toString() << " -> " << a.transport(x).toString() << endl;
    //cerr << a.toString() << " -> " << a.transport(y).toString() << endl;

    BOOST_CHECK_CLOSE( res.vx() ,  0, 1E-6);
    BOOST_CHECK_CLOSE( res.vy() ,  2.6, 1E-6);
    BOOST_CHECK_CLOSE( res.vh() ,  2.6, 1E-6);

    res = a.transport(y);
    BOOST_CHECK_CLOSE( res.vx() ,  -2.6, 1E-6);
    BOOST_CHECK_CLOSE( res.vy() ,  0, 1E-6);
    BOOST_CHECK_CLOSE( res.vh() ,  2.6, 1E-6);
}

BOOST_AUTO_TEST_CASE( Twist2_changeProjection )
{
    Twist2D T_robot_table_p_robot_r_robot(1.,-2.,2.6);
    Twist2D T_robot_table_p_table_r_robot;
    Pose2D H_robot_table;

    T_robot_table_p_table_r_robot = T_robot_table_p_robot_r_robot.changeProjection( H_robot_table.inverse() );
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vx() ,  T_robot_table_p_robot_r_robot.vx(), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vy() ,  T_robot_table_p_robot_r_robot.vy(), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vh() ,  T_robot_table_p_robot_r_robot.vh(), 1E-6);

    H_robot_table.x(1.);
    H_robot_table.y(3.);
    H_robot_table.h(0.);
    T_robot_table_p_table_r_robot = T_robot_table_p_robot_r_robot.changeProjection( H_robot_table.inverse() );
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vx() ,  T_robot_table_p_robot_r_robot.vx(), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vy() ,  T_robot_table_p_robot_r_robot.vy(), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vh() ,  T_robot_table_p_robot_r_robot.vh(), 1E-6);

    H_robot_table.x(rand() / (double)RAND_MAX * 20 - 10.);
    H_robot_table.y(rand() / (double)RAND_MAX * 20 - 10.);
    H_robot_table.y(rand() / (double)RAND_MAX * 20 - 10.);
    T_robot_table_p_table_r_robot = T_robot_table_p_robot_r_robot.changeProjection( H_robot_table.inverse() );
    Twist2D T = T_robot_table_p_table_r_robot.changeProjection( H_robot_table );
    BOOST_CHECK_CLOSE( T.vx() ,  T_robot_table_p_robot_r_robot.vx(), 1E-6);
    BOOST_CHECK_CLOSE( T.vy() ,  T_robot_table_p_robot_r_robot.vy(), 1E-6);
    BOOST_CHECK_CLOSE( T.vh() ,  T_robot_table_p_robot_r_robot.vh(), 1E-6);

    H_robot_table.x(1.);
    H_robot_table.y(3.);
    H_robot_table.h(M_PI/2.);
    T_robot_table_p_table_r_robot = T_robot_table_p_robot_r_robot.changeProjection( H_robot_table.inverse() );
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vx() ,  2., 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vy() ,  1., 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vh() ,  2.6, 1E-6);
}