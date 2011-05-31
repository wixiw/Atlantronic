/*
 * uTest_clusterize.cpp
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

BOOST_AUTO_TEST_CASE( clusterize_Test_1 )
{
    ObjectFinder objf;
    Scan scan = MatrixXd::Zero(0, 0);

    std::vector<Scan> v;
    v = objf.clusterize(scan);

    BOOST_CHECK_EQUAL( v.size(), 0 );
}


BOOST_AUTO_TEST_CASE( clusterize_Test_2 )
{
    ObjectFinder objf;
    Scan scan = MatrixXd::Zero(3, 5);

    std::vector<Scan> v;
    v = objf.clusterize(scan);

    BOOST_CHECK_EQUAL( v.size(), 0 );
}

BOOST_AUTO_TEST_CASE( clusterize_Test_3 )
{
    ObjectFinder objf;
    Scan scan = MatrixXd::Zero(4, 1);

    std::vector<Scan> v;
    v = objf.clusterize(scan);

    BOOST_CHECK_EQUAL( v.size(), 0 );
}

BOOST_AUTO_TEST_CASE( clusterize_Test_4 )
{
    std::cout << "*************************************" << std::endl;
    ObjectFinder objf;
    Scan scan = MatrixXd::Zero(4,9);
    scan <<  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,  0.0,   0.0,
             0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,  0.0,   0.0,
            -1.4, -0.2, -0.2, -1.3, -0.3, -0.31, -1.2, -1.2,  -1.3,
             1.2,  0.7,  0.6,  1.1,  0.6,  0.6,   1.1,  1.2,   1.2;

    std::cout << "Scan :" << std::endl;
    std::cout << scan << std::endl;

    std::vector<Scan> v;
    v = objf.clusterize(scan);

    BOOST_CHECK_EQUAL( v.size(), 1 );

    std::cout << "==============" << std::endl;
    for(unsigned int i = 0; i < v.size() ; i++)
    {
    std::cout << "Cluster " << i << std::endl;
    std::cout << v[i] << std::endl;
    std::cout << "--------------" << std::endl;
    }
}

BOOST_AUTO_TEST_CASE( clusterize_Test_5 )
{
    std::cout << "*************************************" << std::endl;
    ObjectFinder objf;
    Scan scan = MatrixXd::Zero(4, 18);
    scan <<  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,  0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,  0.0,   0.0,
             0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,  0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,  0.0,   0.0,
            -1.4, -0.2, -0.2, -1.3, -0.3, -0.31, -1.2, -1.2,  -1.3, -2.4, -1.2, -1.2, -2.3, -1.3, -1.31, -2.2, -2.2,  -2.3,
             1.2,  0.7,  0.6,  1.1,  0.6,  0.6,   1.1,  1.2,   1.2,  2.2, -1.7, -1.6,  2.1, -1.6, -1.6,   2.1,  2.2,   2.2;

    std::cout << "Scan :" << std::endl;
    std::cout << scan << std::endl;

    std::vector<Scan> v;
    v = objf.clusterize(scan);

    BOOST_CHECK_EQUAL( v.size(), 2 );

    std::cout << "==============" << std::endl;
    for(unsigned int i = 0; i < v.size() ; i++)
    {
    std::cout << "Cluster " << i << std::endl;
    std::cout << v[i] << std::endl;
    std::cout << "--------------" << std::endl;
    }
}
