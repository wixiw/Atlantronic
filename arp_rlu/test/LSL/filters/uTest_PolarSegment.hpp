/*
 * uTest_PolarSegment.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/filters/PolarSegment.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_PolarSegment )

BOOST_AUTO_TEST_CASE( Test_DefaultParams_1 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 10);
    d.row(0) <<   1.00,  1.01,  1.02,  1.03,  1.04,  1.05,  1.06,  1.07,  1.08,  1.09 ;
    d.row(1) <<   0.50,  0.51,  0.52,  0.53,  0.54,  0.55,  0.56,  0.57,  0.58,  0.59 ;
    d.row(2) <<   0.00,  0.01,  0.02,  0.03,  0.04,  0.05,  0.06,  0.07,  0.08,  0.09 ;
    rawScan.setPolarData(d);

    std::vector<lsl::DetectedObject> objects = lsl::PolarSegment::apply(rawScan);

    BOOST_CHECK_EQUAL(objects.size(), 1);

    MatrixXd pdata = objects[0].getScan().getPolarData();
    BOOST_CHECK_EQUAL(pdata.cols(), d.cols());
    for (int i=0; i<pdata.rows(); ++i)
    {
        for (int j=0; j<pdata.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata(i,j), d(i,j));
        }
    }
}

BOOST_AUTO_TEST_CASE( Test_DefaultParams_2 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 10);
    d.row(0) <<   1.00,  1.01,  1.02,  1.03,  1.04,  1.05,  1.06,  1.07,  1.08,  1.09 ;
    d.row(1) <<   0.50,  0.51,  0.52,  0.53,  0.64,  0.61,  0.58,  0.53,  0.50,  0.49 ;
    d.row(2) <<   0.00,  0.01,  0.02,  0.03,  0.04,  0.05,  0.06,  0.07,  0.08,  0.09 ;
    rawScan.setPolarData(d);

    MatrixXd d1 = d.leftCols(4);
    MatrixXd d2 = d.rightCols(6);

    std::vector<lsl::DetectedObject> objects = lsl::PolarSegment::apply(rawScan);

    BOOST_CHECK_EQUAL(objects.size(), 2);

    MatrixXd pdata1 = objects[0].getScan().getPolarData();
    BOOST_CHECK_EQUAL(pdata1.cols(), d1.cols());
    for (int i=0; i<pdata1.rows(); ++i)
    {
        for (int j=0; j<pdata1.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata1(i,j), d1(i,j));
        }
    }

    MatrixXd pdata2 = objects[1].getScan().getPolarData();
    BOOST_CHECK_EQUAL(pdata2.cols(), d2.cols());
    for (int i=0; i<pdata2.rows(); ++i)
    {
        for (int j=0; j<pdata2.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata2(i,j), d2(i,j));
        }
    }
}

BOOST_AUTO_TEST_CASE( Test_DefaultParams_3 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 10);
    d.row(0) <<   1.00,  1.01,  1.02,  1.03,  1.04,  1.05,  1.06,  1.07,  1.08,  1.09 ;
    d.row(1) <<   0.50,  0.51,  0.51,  0.41,  0.42,  0.41,  0.58,  0.53,  0.50,  0.49 ;
    d.row(2) <<   0.00,  0.01,  0.02,  0.03,  0.04,  0.05,  0.06,  0.07,  0.08,  0.09 ;
    rawScan.setPolarData(d);

    MatrixXd d1 = d.leftCols(3);
    MatrixXd d2 = d.block(0, 3, 3, 3);
    MatrixXd d3 = d.rightCols(4);

    std::vector<lsl::DetectedObject> objects = lsl::PolarSegment::apply(rawScan);

    BOOST_CHECK_EQUAL(objects.size(), 3);

    MatrixXd pdata1 = objects[0].getScan().getPolarData();
    BOOST_CHECK_EQUAL(pdata1.cols(), d1.cols());
    for (int i=0; i<pdata1.rows(); ++i)
    {
        for (int j=0; j<pdata1.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata1(i,j), d1(i,j));
        }
    }

    MatrixXd pdata2 = objects[1].getScan().getPolarData();
    BOOST_CHECK_EQUAL(pdata2.cols(), d2.cols());
    for (int i=0; i<pdata2.rows(); ++i)
    {
        for (int j=0; j<pdata2.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata2(i,j), d2(i,j));
        }
    }

    MatrixXd pdata3 = objects[2].getScan().getPolarData();
    BOOST_CHECK_EQUAL(pdata3.cols(), d3.cols());
    for (int i=0; i<pdata3.rows(); ++i)
    {
        for (int j=0; j<pdata3.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata3(i,j), d3(i,j));
        }
    }
}

BOOST_AUTO_TEST_CASE( Test_OtherParams_1 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 10);
    d.row(0) <<   1.00,  1.01,  1.02,  1.03,  1.04,  1.05,  1.06,  1.07,  1.08,  1.09 ;
    d.row(1) <<   0.50,  0.51,  0.52,  0.53,  0.54,  0.73,  0.56,  0.73,  0.78,  0.79 ;
    d.row(2) <<   0.00,  0.01,  0.02,  0.03,  0.04,  0.05,  0.06,  0.07,  0.08,  0.09 ;
    rawScan.setPolarData(d);

    lsl::PolarSegment::Params p;
    p.rangeThres = 0.2;
    std::vector<lsl::DetectedObject> objects = lsl::PolarSegment::apply(rawScan, p);

    BOOST_CHECK_EQUAL(objects.size(), 1);

    MatrixXd pdata = objects[0].getScan().getPolarData();
    BOOST_CHECK_EQUAL(pdata.cols(), d.cols());
    for (int i=0; i<pdata.rows(); ++i)
    {
        for (int j=0; j<pdata.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata(i,j), d(i,j));
        }
    }
}

BOOST_AUTO_TEST_CASE( Test_OtherParams_2 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 10);
    d.row(0) <<   1.00,  1.01,  1.02,  1.03,  1.04,  1.05,  1.06,  1.07,  1.08,  1.09 ;
    d.row(1) <<   0.50,  0.51,  0.52,  0.53,  0.54,  0.75,  0.73,  0.64,  0.78,  0.79 ;
    d.row(2) <<   0.00,  0.01,  0.02,  0.03,  0.04,  0.05,  0.06,  0.07,  0.08,  0.09 ;
    rawScan.setPolarData(d);

    MatrixXd d1 = d.leftCols(5);
    MatrixXd d2 = d.rightCols(5);

    lsl::PolarSegment::Params p;
    p.rangeThres = 0.2;
    std::vector<lsl::DetectedObject> objects = lsl::PolarSegment::apply(rawScan, p);

    MatrixXd pdata1 = objects[0].getScan().getPolarData();
    BOOST_CHECK_EQUAL(pdata1.cols(), d1.cols());
    for (int i=0; i<pdata1.rows(); ++i)
    {
        for (int j=0; j<pdata1.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata1(i,j), d1(i,j));
        }
    }

    MatrixXd pdata2 = objects[1].getScan().getPolarData();
    BOOST_CHECK_EQUAL(pdata2.cols(), d2.cols());
    for (int i=0; i<pdata2.rows(); ++i)
    {
        for (int j=0; j<pdata2.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata2(i,j), d2(i,j));
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
