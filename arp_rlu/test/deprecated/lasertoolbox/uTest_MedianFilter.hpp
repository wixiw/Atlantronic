/*
 * uTest_ComputeSegment.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */


#include "lasertoolbox/MedianFilter.hpp"

using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_CASE( getMedian_Test_1 )
{
    Eigen::VectorXd v(0);
    MedianFilter f;
    double result = f.getMedian(v);
    BOOST_CHECK_EQUAL( result, 0. );
}

BOOST_AUTO_TEST_CASE( getMedian_Test_2 )
{
    Eigen::VectorXd v(1);
    v(0) = 5.;
    MedianFilter f;
    double result = f.getMedian(v);
    BOOST_CHECK_EQUAL( result, 5. );
}

BOOST_AUTO_TEST_CASE( getMedian_Test_3 )
{
    Eigen::VectorXd v(2);
    v(0) = 5.;
    v(1) = -1.;
    MedianFilter f;
    double result = f.getMedian(v);
    BOOST_CHECK_EQUAL( result, -1. );
}

BOOST_AUTO_TEST_CASE( getMedian_Test_4 )
{
    Eigen::VectorXd v(3);
    v(0) = 5.;
    v(1) = -1.;
    v(2) = 3.;
    MedianFilter f;
    double result = f.getMedian(v);
    BOOST_CHECK_EQUAL( result, 3. );
}

BOOST_AUTO_TEST_CASE( getMedian_Test_5 )
{
    Eigen::VectorXd v(3);
    v(0) = 3.;
    v(1) = -1.;
    v(2) = 5.;
    MedianFilter f;
    double result = f.getMedian(v);
    BOOST_CHECK_EQUAL( result, 3. );
}

BOOST_AUTO_TEST_CASE( getMedian_Test_6 )
{
    Eigen::VectorXd v(3);
    v(0) = 3.;
    v(1) = -1.;
    v(2) = 5.;
    MedianFilter f;
    double result = f.getMedian(v);
    BOOST_CHECK_EQUAL( result, 3. );
}

BOOST_AUTO_TEST_CASE( getMedian_Test_7 )
{
    Eigen::VectorXd v(5);
    v(0) = 3.;
    v(1) = -1.;
    v(2) = 8.;
    v(3) = -3.;
    v(4) = 5.;
    MedianFilter f;
    double result = f.getMedian(v);
    BOOST_CHECK_EQUAL( result, 3. );
}



BOOST_AUTO_TEST_CASE( MedianFilter_Test_1 )
{
    MatrixXd raw = MatrixXd::Zero(2,3);
    raw.row(1) << 3., -1, 5;
    MedianFilter f;
    f.setWidth(3);
    f.setScan(raw);
    f.compute();
    Scan filt = f.getResult();
    BOOST_CHECK_EQUAL( filt(1,0), 3. );
    BOOST_CHECK_EQUAL( filt(1,1), 3. );
    BOOST_CHECK_EQUAL( filt(1,2), 5. );
}

BOOST_AUTO_TEST_CASE( MedianFilter_Test_2 )
{
    MatrixXd raw = MatrixXd::Zero(2,8);
    raw.row(1) << 3., -1, 5, 8., 4., 7., 5., 6.;
    std::cout << "raw : " << std::endl;
    std::cout << raw << std::endl;
    MedianFilter f;
    f.setWidth(3);
    f.setScan(raw);
    f.compute();
    Scan filt = f.getResult();
    BOOST_CHECK_EQUAL( filt(1,0), 3. );
    BOOST_CHECK_EQUAL( filt(1,1), 3. );
    BOOST_CHECK_EQUAL( filt(1,2), 5. );
    BOOST_CHECK_EQUAL( filt(1,3), 5. );
    BOOST_CHECK_EQUAL( filt(1,4), 7. );
    BOOST_CHECK_EQUAL( filt(1,5), 5. );
    BOOST_CHECK_EQUAL( filt(1,6), 6. );
    BOOST_CHECK_EQUAL( filt(1,7), 6. );
}

BOOST_AUTO_TEST_CASE( MedianFilter_Test_3 )
{
    MatrixXd raw = MatrixXd::Zero(2,8);
    raw.row(1) << 3., -1, 5, 8., 4., 7., 5., 6.;
    MedianFilter f;
    f.setWidth(4);
    f.setScan(raw);
    f.compute();
    Scan filt = f.getResult();
    BOOST_CHECK_EQUAL( filt(1,0), 3. );
    BOOST_CHECK_EQUAL( filt(1,1), 3. );
    BOOST_CHECK_EQUAL( filt(1,2), 4. );
    BOOST_CHECK_EQUAL( filt(1,3), 5. );
    BOOST_CHECK_EQUAL( filt(1,4), 5. );
    BOOST_CHECK_EQUAL( filt(1,5), 5. );
    BOOST_CHECK_EQUAL( filt(1,6), 5. );
    BOOST_CHECK_EQUAL( filt(1,7), 6. );
}

BOOST_AUTO_TEST_CASE( MedianFilter_Test_4 )
{
    MatrixXd raw = MatrixXd::Zero(2,8);
    raw.row(1) << 3., -1, 5, 8., 4., 7., 5., 6.;
    MedianFilter f;
    f.setWidth(5);
    f.setScan(raw);
    f.compute();
    Scan filt = f.getResult();
    BOOST_CHECK_EQUAL( filt(1,0), 3. );
    BOOST_CHECK_EQUAL( filt(1,1), -1. );
    BOOST_CHECK_EQUAL( filt(1,2), 4. );
    BOOST_CHECK_EQUAL( filt(1,3), 5. );
    BOOST_CHECK_EQUAL( filt(1,4), 5. );
    BOOST_CHECK_EQUAL( filt(1,5), 6. );
    BOOST_CHECK_EQUAL( filt(1,6), 5. );
    BOOST_CHECK_EQUAL( filt(1,7), 6. );
}
