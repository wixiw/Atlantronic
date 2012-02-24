/*
 * uTest_CartesianSegment.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/filters/CartesianSegment.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_CartesianSegment )

BOOST_AUTO_TEST_CASE( Test_checkConsistency )
{
    lsl::CartesianSegment::Params p;

    BOOST_CHECK( p.checkConsistency() );

    p.kmeansMaxIterations = 3;
    p.kmeansDispThres = 0.1;
    p.minNbPoints = 10;
    p.maxStddev = 0.2;
    BOOST_CHECK( p.checkConsistency() );

    p.kmeansMaxIterations = 0;
    p.kmeansDispThres = 0.1;
    p.minNbPoints = 10;
    p.maxStddev = 0.2;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.kmeansMaxIterations = 3;
    p.kmeansDispThres = -0.1;
    p.minNbPoints = 10;
    p.maxStddev = 0.2;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.kmeansMaxIterations = 3;
    p.kmeansDispThres = 0.1;
    p.minNbPoints = 0;
    p.maxStddev = 0.2;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.kmeansMaxIterations = 3;
    p.kmeansDispThres = 0.1;
    p.minNbPoints = 10;
    p.maxStddev = -0.2;
    BOOST_CHECK( !(p.checkConsistency()) );
}

BOOST_AUTO_TEST_CASE( Test_kMeans_Empty )
{
    lsl::LaserScan scan;

    std::pair<lsl::LaserScan, lsl::LaserScan> p;
    p = lsl::CartesianSegment::kMeans(scan);

    BOOST_CHECK_EQUAL( p.first.getSize(), 0 );
    BOOST_CHECK_EQUAL( p.second.getSize(), 0 );
}

BOOST_AUTO_TEST_CASE( Test_kMeans_OnlyOnePoint )
{
    lsl::LaserScan scan;
    MatrixXd d = MatrixXd::Random(3, 1);
    d <<  0.0,
            1.0,
            -PI/2.;
    scan.setPolarData(d);
    scan.computeCartesianData();

    std::pair<lsl::LaserScan, lsl::LaserScan> p;
    p = lsl::CartesianSegment::kMeans(scan);

    BOOST_CHECK_EQUAL( p.first.getSize(), 1 );
    BOOST_CHECK_EQUAL( p.second.getSize(), 0 );

    MatrixXd pdata = p.first.getPolarData();
    BOOST_CHECK_EQUAL( pdata(0,0), d(0,0) );
    BOOST_CHECK_EQUAL( pdata(1,0), d(1,0) );
    BOOST_CHECK_EQUAL( pdata(2,0), d(2,0) );
}

BOOST_AUTO_TEST_CASE( Test_kMeans_TwoPoints )
{
    lsl::LaserScan scan;
    MatrixXd d = MatrixXd::Zero(3, 2);
    d <<  0.0,    0.1,
            1.0,    1.5,
            -PI/2., 0.0;
    scan.setPolarData(d);
    scan.computeCartesianData();

    std::pair<lsl::LaserScan, lsl::LaserScan> p;
    p = lsl::CartesianSegment::kMeans(scan);

    BOOST_CHECK_EQUAL( p.first.getSize(), 1 );
    BOOST_CHECK_EQUAL( p.second.getSize(), 1 );
}

BOOST_AUTO_TEST_CASE( Test_kMeans_FivePoints )
{
    lsl::LaserScan scan;
    MatrixXd d = MatrixXd::Zero(3, 5);
    d <<  0.0,    0.1,   0.2,   0.3,  0.4,
            1.0,    1.01,  1.0,   0.5,  0.51,
            -1.57, -1.56, -1.55, -0.01, 0.0;
    scan.setPolarData(d);
    scan.computeCartesianData();

    std::pair<lsl::LaserScan, lsl::LaserScan> p;
    p = lsl::CartesianSegment::kMeans(scan);

    BOOST_CHECK( (p.first.getSize() == 3 && p.second.getSize() == 2)
            ||  (p.first.getSize() == 2 && p.second.getSize() == 3) );
}

BOOST_AUTO_TEST_CASE( Test_Apply_Empty )
{
    lsl::LaserScan rawScan;
    std::vector<lsl::LaserScan> objects = lsl::CartesianSegment::apply(rawScan);

    BOOST_CHECK_EQUAL( objects.size(), 1 );
}

BOOST_AUTO_TEST_CASE( Test_Apply_NoCartesian )
{
    lsl::LaserScan rawScan;
    MatrixXd d = MatrixXd::Random(3,4);
    rawScan.setPolarData(d);

    std::vector<lsl::LaserScan> objects = lsl::CartesianSegment::apply(rawScan);

    BOOST_CHECK_EQUAL( objects.size(), 1 );
}

BOOST_AUTO_TEST_CASE( Test_Apply_MinPoints )
{
    lsl::LaserScan rawScan;
    MatrixXd d = MatrixXd::Random(3,4);
    rawScan.setPolarData(d);
    rawScan.computeCartesianData();

    std::vector<lsl::LaserScan> objects = lsl::CartesianSegment::apply(rawScan);

    BOOST_CHECK_EQUAL( objects.size(), 0 );
}

BOOST_AUTO_TEST_CASE( Test_Apply_1 )
{
    lsl::LaserScan rawScan;
    MatrixXd d = MatrixXd::Random(3,9);
    d.row(1) << 1.0,   1.01,  1.0,   0.5,  0.51,  0.99,  0.98,   0.48,  0.52;
    d.row(2) << -0.01, 0.00,  0.01,  1.3,  1.31,  0.02,  0.03,   1.32,  1.33;
    rawScan.setPolarData(d);
    rawScan.computeCartesianData();

    std::vector<lsl::LaserScan> objects = lsl::CartesianSegment::apply(rawScan);

    BOOST_CHECK_EQUAL( objects.size(), 1 );
}

BOOST_AUTO_TEST_CASE( Test_Apply_2 )
{
    lsl::LaserScan rawScan;
    MatrixXd d = MatrixXd::Random(3,18);
    d.row(1) << 1.0,   1.01,  1.0,   0.20,  0.5,  2.00,  0.51,  0.22,  0.99,  0.98,   0.48,  0.52,  0.51,  0.52,  0.20,  0.21,  0.19,  0.18;
    d.row(2) << -0.01, 0.00,  0.01,  2.00,  1.3,  0.00,  1.31,  1.99,  0.02,  0.03,   1.32,  1.33,  1.34,  1.35,  2.01,  2.02,  2.03,  2.04;
    rawScan.setPolarData(d);
    rawScan.computeCartesianData();

    std::vector<lsl::LaserScan> objects = lsl::CartesianSegment::apply(rawScan);

    BOOST_CHECK_EQUAL( objects.size(), 3 );
}



BOOST_AUTO_TEST_SUITE_END()
