/*
 * test_EstimatedTwist2.cpp
 *
 *  Created on: 13 sept. 2010
 *      Author: boris
 */

#include <math/core>
#include <iostream>
using namespace arp_math;
using namespace std;

BOOST_AUTO_TEST_CASE( EstimatedTwist2_Default_Constructor )
{
    // Test default constructor
    EstimatedTwist2D a;
    BOOST_CHECK( a.vx() == 0.0 );
    BOOST_CHECK( a.vy() == 0.0 );
    BOOST_CHECK( a.vh() == 0.0 );
}

BOOST_AUTO_TEST_CASE( EstimatedTwist2_Accessors )
{
    Twist2D a;
    a.vx(1.);
    a.vy(2.);
    a.vh(3.);
    BOOST_CHECK( a.vx() == 1. );
    BOOST_CHECK( a.vy() == 2. );
    BOOST_CHECK( a.vh() == 3. );
}


BOOST_AUTO_TEST_CASE( EstimatedTwist2_Transport_trivial )
{
    Eigen::Matrix<double,3,3> cov;
    cov << 0.5, 0., 0.,
           0. , 1., 0.,
           0. , 0., 2.;
    EstimatedTwist2D a = MathFactory::createEstimatedTwist2DFromCartesianRepr(1.3, -2.8, 3., 100., cov);

    EstimatedTwist2D res;

    res = a.transport( Pose2D(0,0,0) );
    BOOST_CHECK_EQUAL( res.vx() ,  a.vx() );
    BOOST_CHECK_EQUAL( res.vy() ,  a.vy() );
    BOOST_CHECK_EQUAL( res.vh() ,  a.vh() );
    BOOST_CHECK_EQUAL( res.date(), a.date() );
    BOOST_CHECK_EQUAL( res.cov(),  a.cov() );
}


BOOST_AUTO_TEST_CASE( EstimatedTwist2_Transport_Rotation_1 )
{
    Eigen::Matrix<double,3,3> cov;
    cov << 5., 0., 0.,
           0., 3., 0.,
           0., 0., 2.;

    Vector3 T(1., 2., 4.);
    Pose2D p(0.,0.,M_PI);

    EstimatedTwist2D a = MathFactory::createEstimatedTwist2DFromCartesianRepr(T, 100., cov);
    Twist2D b = (Twist2D) a;
    Twist2D ground = b.transport( p );

    EstimatedTwist2D res;

    res = a.transport( p );
    BOOST_CHECK_EQUAL( res.vx() ,  ground.vx() );
    BOOST_CHECK_EQUAL( res.vy() ,  ground.vy() );
    BOOST_CHECK_EQUAL( res.vh() ,  ground.vh() );
    BOOST_CHECK_EQUAL( res.date(), a.date() );
    Covariance3 covRes = res.cov();
    BOOST_CHECK_EQUAL( covRes(0,0), cov(0,0) );
    BOOST_CHECK_EQUAL( covRes(1,1), cov(1,1) );
    BOOST_CHECK_EQUAL( covRes(2,2), cov(2,2) );
}

BOOST_AUTO_TEST_CASE( EstimatedTwist2_Transport_Rotation_2 )
{
    Eigen::Matrix<double,3,3> cov;
    cov << 5., 0., 0.,
            0., 3., 0.,
            0., 0., 2.;

    Vector3 T(1., 2., 4.);
    Pose2D p(0.,0.,M_PI/2);

    EstimatedTwist2D a = MathFactory::createEstimatedTwist2DFromCartesianRepr(T, 100., cov);
    Twist2D b = (Twist2D) a;
    Twist2D ground = b.transport( p );

    EstimatedTwist2D res;

    res = a.transport( p );
    BOOST_CHECK_EQUAL( res.vx() ,  ground.vx() );
    BOOST_CHECK_EQUAL( res.vy() ,  ground.vy() );
    BOOST_CHECK_EQUAL( res.vh() ,  ground.vh() );
    BOOST_CHECK_EQUAL( res.date(), a.date() );
    Covariance3 covRes = res.cov();
    BOOST_CHECK_EQUAL( covRes(0,0), cov(1,1) );
    BOOST_CHECK_EQUAL( covRes(1,1), cov(0,0) );
    BOOST_CHECK_EQUAL( covRes(2,2), cov(2,2) );
}


BOOST_AUTO_TEST_CASE( EstimatedTwist2_changeProjection )
{
    srand( time(NULL) );
    double vx = rand() / (double)RAND_MAX * 20 - 10.;
    double vy = rand() / (double)RAND_MAX * 20 - 10.;
    double vh = rand() / (double)RAND_MAX * 20 - 10.;
    long double date = rand() / (double)RAND_MAX * 200.;
//    Covariance3 cov = Vector3(0.1, 0.3, 0.2).asDiagonal();
//    cov(1,0) = 0.05;
//    cov(0,1) = 0.05;
    Covariance3 rmat = Covariance3::Random();
    Covariance3 cov = ( rmat + rmat.transpose() ) * 0.5;
    EstimatedTwist2D T_robot_table_p_robot_r_robot = MathFactory::createEstimatedTwist2DFromCartesianRepr(vx, vy, vh, date, cov);
    EstimatedTwist2D T_robot_table_p_table_r_robot;
    Pose2D H_robot_table;

    T_robot_table_p_table_r_robot = T_robot_table_p_robot_r_robot.changeProjection( H_robot_table.inverse() );
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vx() ,  T_robot_table_p_robot_r_robot.vx(), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vy() ,  T_robot_table_p_robot_r_robot.vy(), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vh() ,  T_robot_table_p_robot_r_robot.vh(), 1E-6);
    BOOST_CHECK( T_robot_table_p_table_r_robot.date() == T_robot_table_p_robot_r_robot.date() );
    BOOST_CHECK( T_robot_table_p_table_r_robot.cov() == T_robot_table_p_robot_r_robot.cov() );

    H_robot_table.x(1.);
    H_robot_table.y(3.);
    H_robot_table.h(0.);
    T_robot_table_p_table_r_robot = T_robot_table_p_robot_r_robot.changeProjection( H_robot_table.inverse() );
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vx() ,  T_robot_table_p_robot_r_robot.vx(), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vy() ,  T_robot_table_p_robot_r_robot.vy(), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vh() ,  T_robot_table_p_robot_r_robot.vh(), 1E-6);
    BOOST_CHECK( T_robot_table_p_table_r_robot.date() == T_robot_table_p_robot_r_robot.date() );
    BOOST_CHECK( T_robot_table_p_table_r_robot.cov() == T_robot_table_p_robot_r_robot.cov() );

    H_robot_table.x(rand() / (double)RAND_MAX * 20 - 10.);
    H_robot_table.y(rand() / (double)RAND_MAX * 20 - 10.);
    H_robot_table.y(rand() / (double)RAND_MAX * 20 - 10.);
    T_robot_table_p_table_r_robot = T_robot_table_p_robot_r_robot.changeProjection( H_robot_table.inverse() );
    EstimatedTwist2D T = T_robot_table_p_table_r_robot.changeProjection( H_robot_table );
    BOOST_CHECK_CLOSE( T.vx() ,  T_robot_table_p_robot_r_robot.vx(), 1E-6);
    BOOST_CHECK_CLOSE( T.vy() ,  T_robot_table_p_robot_r_robot.vy(), 1E-6);
    BOOST_CHECK_CLOSE( T.vh() ,  T_robot_table_p_robot_r_robot.vh(), 1E-6);
    BOOST_CHECK( T.date() == T_robot_table_p_robot_r_robot.date() );
    BOOST_CHECK( T.cov() == T_robot_table_p_robot_r_robot.cov() );

    H_robot_table.x(1.);
    H_robot_table.y(3.);
    H_robot_table.h(M_PI/2.);
    T_robot_table_p_table_r_robot = T_robot_table_p_robot_r_robot.changeProjection( H_robot_table.inverse() );

//    std::cout << "EstimatedTwist2_changeProjection - T_robot_table_p_robot_r_robot : " << T_robot_table_p_robot_r_robot.toString() << std::endl;
//    std::cout << "EstimatedTwist2_changeProjection -       cov:\n" << T_robot_table_p_robot_r_robot.cov() << std::endl;
//    std::cout << "EstimatedTwist2_changeProjection - H_robot_table : " << H_robot_table.toString() << std::endl;
//    std::cout << "EstimatedTwist2_changeProjection - H_robot_table.inverse() : " << H_robot_table.inverse().toString() << std::endl;
//    std::cout << "EstimatedTwist2_changeProjection - T_robot_table_p_table_r_robot : " << T_robot_table_p_table_r_robot.toString() << std::endl;
//    std::cout << "EstimatedTwist2_changeProjection -       cov:\n" << T_robot_table_p_table_r_robot.cov() << std::endl;

    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vx() ,  -vy, 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vy() ,   vx, 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.vh() ,   vh, 1E-6);
    BOOST_CHECK( T_robot_table_p_table_r_robot.date() == T_robot_table_p_robot_r_robot.date() );
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.cov()(0,0) ,  T_robot_table_p_robot_r_robot.cov()(1,1), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.cov()(1,1) ,  T_robot_table_p_robot_r_robot.cov()(0,0), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.cov()(2,2) ,  T_robot_table_p_robot_r_robot.cov()(2,2), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.cov()(0,1) , -T_robot_table_p_robot_r_robot.cov()(1,0), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.cov()(1,0) , -T_robot_table_p_robot_r_robot.cov()(0,1), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.cov()(2,0) , -T_robot_table_p_robot_r_robot.cov()(2,1), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.cov()(2,1) ,  T_robot_table_p_robot_r_robot.cov()(2,0), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.cov()(0,2) , -T_robot_table_p_robot_r_robot.cov()(1,2), 1E-6);
    BOOST_CHECK_CLOSE( T_robot_table_p_table_r_robot.cov()(1,2) ,  T_robot_table_p_robot_r_robot.cov()(0,2), 1E-6);
}

