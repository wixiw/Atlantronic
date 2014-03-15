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

BOOST_AUTO_TEST_CASE( EstimatedPose2_Default_Constructor )
{
    // Test default constructor
    EstimatedPose2D a;
    BOOST_CHECK( a.x() == 0.0 );
    BOOST_CHECK( a.y() == 0.0 );
    BOOST_CHECK( a.h() == 0.0 );
}

BOOST_AUTO_TEST_CASE( EstimatedPose2_Accessors )
{
    EstimatedPose2D a;
    a.x(1.);
    a.y(2.);
    a.h(3.);
    a.date( 100. );
    Covariance3 c = Covariance3::Random();
    c = ( c + c.transpose() ) * 0.5;
    a.cov( c );
    BOOST_CHECK( a.x() == 1. );
    BOOST_CHECK( a.y() == 2. );
    BOOST_CHECK( a.h() == 3. );
    BOOST_CHECK( a.date() == 100. );
    BOOST_CHECK( a.cov() == c );
}



BOOST_AUTO_TEST_CASE( EstimatedPose2_Operator_trivial )
{
    srand( time(NULL) );
    double x = rand() / (double)RAND_MAX * 20 - 10.;
    double y = rand() / (double)RAND_MAX * 20 - 10.;
    double h = rand() / (double)RAND_MAX * 20 - 10.;

    long double date = rand() / (double)RAND_MAX * 20;
    Covariance3 cov = Covariance3::Random();
    cov = ( cov + cov.transpose() ) * 0.5;
    cov(2,0) = 0.;
    cov(2,1) = 0.;
    cov(0,2) = 0.;
    cov(1,2) = 0.;

    EstimatedPose2D a;
    a.x( x );
    a.y( y );
    a.h( h );
    a.date( date );
    a.cov( cov );

//    std::cout << "a : " << a.toString() << std::endl;
//    std::cout << "a.date() : " << a.date() << std::endl;
//    std::cout << "a.cov :\n" << a.cov() << std::endl;

    Pose2D p(0., 0., 0.);

    EstimatedPose2D b = a * p;
    BOOST_CHECK_SMALL( b.x() - x, 1.e-8 );
    BOOST_CHECK_SMALL( b.y() - y, 1.e-8 );
    BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(b.h() - h), 1.e-8 );
    BOOST_CHECK( b.date() == date );
    BOOST_CHECK( b.cov() == cov );

//    std::cout << "b : " << b.toString() << std::endl;
//    std::cout << "b.date() : " << b.date() << std::endl;
//    std::cout << "b.cov :\n" << b.cov() << std::endl;
}

BOOST_AUTO_TEST_CASE( EstimatedPose2_Operator_translation_1 )
{
    srand( time(NULL) );
    double x = 2.;
    double y = 1.;
    double h = M_PI/2.;
    long double date = rand() / (double)RAND_MAX * 20;
    Covariance3 cov = Vector3(0.1, 0.5, 0.2).asDiagonal();

    EstimatedPose2D H_hky_table;
    H_hky_table.x( x );
    H_hky_table.y( y );
    H_hky_table.h( h );
    H_hky_table.date( date );
    H_hky_table.cov( cov );

//    std::cout << "H_hky_table : " << H_hky_table.toString() << std::endl;
//    std::cout << "H_hky_table.date() : " << H_hky_table.date() << std::endl;
//    std::cout << "H_hky_table.cov :\n" << H_hky_table.cov() << std::endl;

    Pose2D H_robot_hky(2., 0., 0.);

    EstimatedPose2D H_robot_table = H_hky_table * H_robot_hky;
    BOOST_CHECK_SMALL( H_robot_table.x() - (2.), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.y() - (3.), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.h() - (M_PI/2), 1.e-8 );
    BOOST_CHECK( H_robot_table.date() == date );
    BOOST_CHECK_SMALL( H_robot_table.cov()(0,0) - (cov(0,0) + 4. * cov(2,2)), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(1,1) - cov(1,1), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(1,0), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(0,1), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(2,0), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(2,1), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(0,2), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(1,2), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(2,2) - cov(2,2), 1.e-8 );

//    std::cout << "H_robot_table : " << H_robot_table.toString() << std::endl;
//    std::cout << "H_robot_table.date() : " << H_robot_table.date() << std::endl;
//    std::cout << "H_robot_table.cov :\n" << H_robot_table.cov() << std::endl;
}

BOOST_AUTO_TEST_CASE( EstimatedPose2_Operator_translation_2 )
{
    srand( time(NULL) );
    double x = 2.;
    double y = 1.;
    double h = M_PI/2.;
    long double date = rand() / (double)RAND_MAX * 20;
    Covariance3 cov = Vector3(0.1, 0.5, 0.2).asDiagonal();

    EstimatedPose2D H_hky_table;
    H_hky_table.x( x );
    H_hky_table.y( y );
    H_hky_table.h( h );
    H_hky_table.date( date );
    H_hky_table.cov( cov );

//    std::cout << "H_hky_table : " << H_hky_table.toString() << std::endl;
//    std::cout << "H_hky_table.date() : " << H_hky_table.date() << std::endl;
//    std::cout << "H_hky_table.cov :\n" << H_hky_table.cov() << std::endl;

    Pose2D H_robot_hky(0., 2., 0.);

    EstimatedPose2D H_robot_table = H_hky_table * H_robot_hky;
    BOOST_CHECK_SMALL( H_robot_table.x() - (0.), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.y() - (1.), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.h() - (M_PI/2), 1.e-8 );
    BOOST_CHECK( H_robot_table.date() == date );
    BOOST_CHECK_SMALL( H_robot_table.cov()(0,0) - cov(0,0), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(1,1) - (cov(1,1) + 4. * cov(2,2)), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(1,0), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(0,1), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(2,0), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(2,1), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(0,2), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(1,2), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(2,2) - cov(2,2), 1.e-8 );

//    std::cout << "H_robot_table : " << H_robot_table.toString() << std::endl;
//    std::cout << "H_robot_table.date() : " << H_robot_table.date() << std::endl;
//    std::cout << "H_robot_table.cov :\n" << H_robot_table.cov() << std::endl;
}

BOOST_AUTO_TEST_CASE( EstimatedPose2_Operator_rotation )
{
    srand( time(NULL) );
    double x = 2.;
    double y = 1.;
    double h = M_PI/2.;
    long double date = rand() / (double)RAND_MAX * 20;
    Covariance3 cov = Vector3(0.1, 0.5, 0.2).asDiagonal();

    EstimatedPose2D H_hky_table;
    H_hky_table.x( x );
    H_hky_table.y( y );
    H_hky_table.h( h );
    H_hky_table.date( date );
    H_hky_table.cov( cov );

//    std::cout << "H_hky_table : " << H_hky_table.toString() << std::endl;
//    std::cout << "H_hky_table.date() : " << H_hky_table.date() << std::endl;
//    std::cout << "H_hky_table.cov :\n" << H_hky_table.cov() << std::endl;

    Pose2D H_robot_hky(0., 0., M_PI);

    EstimatedPose2D H_robot_table = H_hky_table * H_robot_hky;
    BOOST_CHECK_SMALL( H_robot_table.x() - (2.), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.y() - (1.), 1.e-8 );
    BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(H_robot_table.h() - (-M_PI/2)), 1.e-8 );
    BOOST_CHECK( H_robot_table.date() == date );
    BOOST_CHECK_SMALL( H_robot_table.cov()(0,0) - cov(0,0), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(1,1) - cov(1,1), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(1,0), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(0,1), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(2,0), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(2,1), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(0,2), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(1,2), 1.e-8 );
    BOOST_CHECK_SMALL( H_robot_table.cov()(2,2) - cov(2,2), 1.e-8 );

//    std::cout << "H_robot_table : " << H_robot_table.toString() << std::endl;
//    std::cout << "H_robot_table.date() : " << H_robot_table.date() << std::endl;
//    std::cout << "H_robot_table.cov :\n" << H_robot_table.cov() << std::endl;
}

