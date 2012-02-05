/*
 * uTest_LaserScan.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/LaserScan.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_LaserScan )

BOOST_AUTO_TEST_CASE( Constructor_Default_1 )
{
    lsl::LaserScan obj;
}

BOOST_AUTO_TEST_CASE( Constructor_Copy_1 )
{
    lsl::LaserScan obj1;
}

BOOST_AUTO_TEST_CASE( getSize_1 )
{
    // test scan vide
    lsl::LaserScan obj;
    unsigned int s = obj.getSize();
    BOOST_CHECK_EQUAL( s, 0 );
}

BOOST_AUTO_TEST_CASE( getSize_2 )
{
    // test scan rempli aléatoirement
    lsl::LaserScan obj;
//    Eigen::MatrixXd data = Eigen::MatrixXd::Random(3, 20);
//    obj.setPolarData(data);

    unsigned int s = obj.getSize();
    BOOST_CHECK_EQUAL( s, 20 );
}

BOOST_AUTO_TEST_CASE( computeCartesianData_1 )
{
    // test scan vide
    lsl::LaserScan obj;
}

BOOST_AUTO_TEST_CASE( computeCartesianData_2 )
{
    // test scan rempli aléatoirement
    lsl::LaserScan obj;
}

BOOST_AUTO_TEST_CASE( setPolarData_1 )
{
    lsl::LaserScan obj;
}

BOOST_AUTO_TEST_CASE( getPolarData_1 )
{
    // test scan vide
    lsl::LaserScan obj;
}

BOOST_AUTO_TEST_CASE( getPolarData_2 )
{
    // test scan rempli aléatoirement
    lsl::LaserScan obj;
}

BOOST_AUTO_TEST_CASE( getCartesianData_1 )
{
    // test scan vide
    lsl::LaserScan obj;
}

BOOST_AUTO_TEST_CASE( getCartesianData_2 )
{
    // test scan rempli aléatoirement
    lsl::LaserScan obj;
}

BOOST_AUTO_TEST_CASE( getTimeData_1 )
{
    // test scan vide
    lsl::LaserScan obj;
}

BOOST_AUTO_TEST_CASE( getTimeData_2 )
{
    // test scan rempli aléatoirement
    lsl::LaserScan obj;
}

BOOST_AUTO_TEST_CASE( areCartesianDataAvailable_1 )
{
    // test cartesian data available
    lsl::LaserScan obj;

    bool val = obj.areCartesianDataAvailable();
    BOOST_CHECK( val );
}

BOOST_AUTO_TEST_CASE( areCartesianDataAvailable_2 )
{
    // test cartesian data unavailable (scan vide)
    lsl::LaserScan obj;

    bool val = obj.areCartesianDataAvailable();
    BOOST_CHECK( !val );
}

BOOST_AUTO_TEST_CASE( areCartesianDataAvailable_3 )
{
    // test cartesian data unavailable (scan random)
    lsl::LaserScan obj;

    bool val = obj.areCartesianDataAvailable();
    BOOST_CHECK( !val );
}

BOOST_AUTO_TEST_CASE( cleanUp_1 )
{
    // test scan vide
    lsl::LaserScan obj;

    bool val = obj.cleanUp();

    BOOST_CHECK( val );
}

BOOST_AUTO_TEST_CASE( cleanUp_2 )
{
    // test cleanUp non nécessaire
    lsl::LaserScan obj;

    bool val = obj.cleanUp();

    BOOST_CHECK( !val );
}

BOOST_AUTO_TEST_CASE( cleanUp_3 )
{
    // test cleanUp nécessaire
    lsl::LaserScan obj;

    bool val = obj.cleanUp();

    BOOST_CHECK( val );
}

BOOST_AUTO_TEST_SUITE_END()
