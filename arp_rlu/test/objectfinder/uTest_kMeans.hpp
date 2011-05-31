/*
 * uTest_kMeans.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

#include "objectfinder/ObjectFinder.hpp"
#include "math/math.hpp"

using namespace Eigen;
using namespace arp_rlu;
using namespace arp_math;
using namespace std;

BOOST_AUTO_TEST_CASE( kMeans_Test_1 )
{
    ObjectFinder objf;
    Scan scan = MatrixXd::Zero(0, 0);

    std::pair<Scan, Scan> p;
    p = objf.kMeans(scan, 0.01, 4);

    BOOST_CHECK_EQUAL( p.first.rows(), 0 );
    BOOST_CHECK_EQUAL( p.first.cols(), 0 );
    BOOST_CHECK_EQUAL( p.second.rows(), 0 );
    BOOST_CHECK_EQUAL( p.second.cols(), 0 );
}


BOOST_AUTO_TEST_CASE( kMeans_Test_2 )
{
    ObjectFinder objf;
    Scan scan = MatrixXd::Zero(3, 5);

    std::pair<Scan, Scan> p;
    p = objf.kMeans(scan, 0.01, 4);

    BOOST_CHECK_EQUAL( p.first.rows(), 0 );
    BOOST_CHECK_EQUAL( p.first.cols(), 0 );
    BOOST_CHECK_EQUAL( p.second.rows(), 0 );
    BOOST_CHECK_EQUAL( p.second.cols(), 0 );
}

BOOST_AUTO_TEST_CASE( kMeans_Test_3 )
{
    ObjectFinder objf;
    Scan scan = MatrixXd::Zero(4, 1);

    std::pair<Scan, Scan> p;
    p = objf.kMeans(scan, 0.01, 4);

    BOOST_CHECK_EQUAL( p.first.rows(), 4 );
    BOOST_CHECK_EQUAL( p.first.cols(), 1 );
    BOOST_CHECK_EQUAL( p.second.rows(), 0 );
    BOOST_CHECK_EQUAL( p.second.cols(), 0 );
}

BOOST_AUTO_TEST_CASE( kMeans_Test_4 )
{
    std::cout << "*************************************" << std::endl;
    ObjectFinder objf;
    Scan scan = MatrixXd::Zero(4, 2);
    scan <<  0.0,  0.0,
             0.0,  0.0,
            -1.4, -0.2,
             1.2,  0.7;

    std::cout << "Scan :" << std::endl;
    std::cout << scan << std::endl;

    std::pair<Scan, Scan> p;
    p = objf.kMeans(scan, 0.01, 4);

    BOOST_CHECK_EQUAL( p.first.rows(), 4 );
    BOOST_CHECK_EQUAL( p.first.cols(), 1 );
    BOOST_CHECK_EQUAL( p.second.rows(), 4 );
    BOOST_CHECK_EQUAL( p.second.cols(), 1 );

    std::cout << "--------------" << std::endl;
    std::cout << "First :" << std::endl;
    std::cout << p.first << std::endl;
    std::cout << "--------------" << std::endl;
    std::cout << "Second :" << std::endl;
    std::cout << p.second << std::endl;
}

BOOST_AUTO_TEST_CASE( kMeans_Test_5 )
{
    std::cout << "*************************************" << std::endl;
    ObjectFinder objf;
    Scan scan = MatrixXd::Zero(4, 5);
    scan <<  0.0,  0.0,  0.0,  0.0,  0.0,
             0.0,  0.0,  0.0,  0.0,  0.0,
            -1.4, -0.2, -1.2, -0.3, -0.31,
             1.2,  0.7,  1.1,  0.6,  0.6;

    std::cout << "Scan :" << std::endl;
    std::cout << scan << std::endl;

    std::pair<Scan, Scan> p;
    p = objf.kMeans(scan, 0.01, 4);

    BOOST_CHECK_EQUAL( p.first.rows(), 4 );
    BOOST_CHECK( p.first.cols() == 2 ||p.first.cols() == 3  );
    BOOST_CHECK_EQUAL( p.second.rows(), 4 );
    BOOST_CHECK( p.second.cols() == 2 ||p.second.cols() == 3  );

    std::cout << "--------------" << std::endl;
    std::cout << "First :" << std::endl;
    std::cout << p.first << std::endl;
    std::cout << "--------------" << std::endl;
    std::cout << "Second :" << std::endl;
    std::cout << p.second << std::endl;
}
