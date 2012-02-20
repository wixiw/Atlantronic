/*
 * uTest_MedianFilter.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/filters/MedianFilter.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_MedianFilter )

BOOST_AUTO_TEST_CASE( test_checkConsistency )
{
    lsl::MedianFilter::Params p;
    p.width = 4;
    BOOST_CHECK( p.checkConsistency() );

    p.width = 1;
    BOOST_CHECK( p.checkConsistency() );

    p.width = 0;
    BOOST_CHECK( !(p.checkConsistency()) );
}

BOOST_AUTO_TEST_CASE( N3_w3 )
{
    lsl::LaserScan rawScan;

    MatrixXd raw = MatrixXd::Random(3,3);
    raw.row(1) << 3., -1, 5;
    rawScan.setPolarData(raw);

    lsl::LaserScan filtScan = lsl::MedianFilter::apply(rawScan);

    MatrixXd filt = filtScan.getPolarData();

    BOOST_CHECK_EQUAL( filt.row(0), raw.row(0) );
    BOOST_CHECK_EQUAL( filt.row(2), raw.row(2) );

    BOOST_CHECK_EQUAL( filt(1,0), 3. );
    BOOST_CHECK_EQUAL( filt(1,1), 3. );
    BOOST_CHECK_EQUAL( filt(1,2), 5. );
}

BOOST_AUTO_TEST_CASE( N8_w4 )
{
    lsl::LaserScan rawScan;

    MatrixXd raw = MatrixXd::Random(3,8);
    raw.row(1) << 3., -1, 5, 8., 4., 7., 5., 6.;
    rawScan.setPolarData(raw);

    lsl::MedianFilter::Params p;
    p.width = 4;
    lsl::LaserScan filtScan = lsl::MedianFilter::apply(rawScan, p);

    MatrixXd filt = filtScan.getPolarData();

    BOOST_CHECK_EQUAL( filt.row(0), raw.row(0) );
    BOOST_CHECK_EQUAL( filt.row(2), raw.row(2) );

    BOOST_CHECK_EQUAL( filt(1,0), 3. );
    BOOST_CHECK_EQUAL( filt(1,1), 3. );
    BOOST_CHECK_EQUAL( filt(1,2), 5. );
    BOOST_CHECK_EQUAL( filt(1,3), 5. );
    BOOST_CHECK_EQUAL( filt(1,4), 7. );
    BOOST_CHECK_EQUAL( filt(1,5), 5. );
    BOOST_CHECK_EQUAL( filt(1,6), 6. );
    BOOST_CHECK_EQUAL( filt(1,7), 6. );
}

BOOST_AUTO_TEST_CASE( N8_w5 )
{
    lsl::LaserScan rawScan;

    MatrixXd raw = MatrixXd::Random(3,8);
    raw.row(1) << 3., -1, 5, 8., 4., 7., 5., 6.;
    rawScan.setPolarData(raw);

    lsl::MedianFilter::Params p;
    p.width = 5;
    lsl::LaserScan filtScan = lsl::MedianFilter::apply(rawScan, p);

    MatrixXd filt = filtScan.getPolarData();

    BOOST_CHECK_EQUAL( filt.row(0), raw.row(0) );
    BOOST_CHECK_EQUAL( filt.row(2), raw.row(2) );

    BOOST_CHECK_EQUAL( filt(1,0), 3. );
    BOOST_CHECK_EQUAL( filt(1,1), -1. );
    BOOST_CHECK_EQUAL( filt(1,2), 4. );
    BOOST_CHECK_EQUAL( filt(1,3), 5. );
    BOOST_CHECK_EQUAL( filt(1,4), 5. );
    BOOST_CHECK_EQUAL( filt(1,5), 6. );
    BOOST_CHECK_EQUAL( filt(1,6), 5. );
    BOOST_CHECK_EQUAL( filt(1,7), 6. );
}

BOOST_AUTO_TEST_CASE( N15_w5 )
{
    lsl::LaserScan rawScan;

    MatrixXd raw = MatrixXd::Random(3,15);
    raw.row(1) << 3., -1., 5., 8., 4., 7., 5., 6., 8., 0., 4., 5., 0., 6., 7.;
    rawScan.setPolarData(raw);

    lsl::MedianFilter::Params p;
    p.width = 5;
    lsl::LaserScan filtScan = lsl::MedianFilter::apply(rawScan, p);

    MatrixXd filt = filtScan.getPolarData();

    BOOST_CHECK_EQUAL( filt.row(0), raw.row(0) );
    BOOST_CHECK_EQUAL( filt.row(2), raw.row(2) );

    BOOST_CHECK_EQUAL( filt(1, 0), 3. );
    BOOST_CHECK_EQUAL( filt(1, 1), -1. );
    BOOST_CHECK_EQUAL( filt(1, 2), 4. );  // 3.,-1., 5., 8., 4.
    BOOST_CHECK_EQUAL( filt(1, 3), 5. );  //-1., 5., 8., 4., 7.
    BOOST_CHECK_EQUAL( filt(1, 4), 5. );  // 5., 8., 4., 7., 5.
    BOOST_CHECK_EQUAL( filt(1, 5), 6. );  // 8., 4., 7., 5., 6.
    BOOST_CHECK_EQUAL( filt(1, 6), 6. );  // 4., 7., 5., 6., 8.
    BOOST_CHECK_EQUAL( filt(1, 7), 6. );  // 7., 5., 6., 8., 0.
    BOOST_CHECK_EQUAL( filt(1, 8), 5. );  // 5., 6., 8., 0., 4.
    BOOST_CHECK_EQUAL( filt(1, 9), 5. );  // 6., 8., 0., 4., 5.
    BOOST_CHECK_EQUAL( filt(1,10), 4. );  // 8., 0., 4., 5., 0.
    BOOST_CHECK_EQUAL( filt(1,11), 4. );  // 0., 4., 5., 0., 6.
    BOOST_CHECK_EQUAL( filt(1,12), 5. );  // 4., 5., 0., 6., 7.
    BOOST_CHECK_EQUAL( filt(1,13), 6. );
    BOOST_CHECK_EQUAL( filt(1,14), 7. );
}

BOOST_AUTO_TEST_SUITE_END()
