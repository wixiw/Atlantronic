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
    Eigen::MatrixXd pdata = obj.getPolarData();
    Eigen::MatrixXd cdata = obj.getCartesianData();

    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), 0);
    BOOST_CHECK_EQUAL( cdata.rows(), 3);
    BOOST_CHECK_EQUAL( cdata.cols(), 0);
}

BOOST_AUTO_TEST_CASE( Constructor_Copy_1 )
{
    lsl::LaserScan obj1;

    lsl::LaserScan obj2(obj1);

    Eigen::MatrixXd pdata2 = obj2.getPolarData();
    Eigen::MatrixXd cdata2 = obj2.getCartesianData();

    BOOST_CHECK_EQUAL( pdata2.rows(), 3);
    BOOST_CHECK_EQUAL( pdata2.cols(), 0);
    BOOST_CHECK_EQUAL( cdata2.rows(), 3);
    BOOST_CHECK_EQUAL( cdata2.cols(), 0);
}

BOOST_AUTO_TEST_CASE( Constructor_Copy_2 )
{
    lsl::LaserScan obj1;
    unsigned int N = rand() % 1000 + 1 ;
    Eigen::MatrixXd d = Eigen::MatrixXd::Random(3,N);
    obj1.setPolarData(d);

    lsl::LaserScan obj2(obj1);

    Eigen::MatrixXd pdata1 = obj1.getPolarData();
    Eigen::MatrixXd cdata1 = obj1.getCartesianData();

    Eigen::MatrixXd pdata2 = obj2.getPolarData();
    Eigen::MatrixXd cdata2 = obj2.getCartesianData();

    BOOST_CHECK_EQUAL( pdata1.rows(), pdata2.rows());
    BOOST_CHECK_EQUAL( pdata1.cols(), pdata2.cols());

    for (int i=0; i<pdata1.rows(); ++i) {
        for (int j=0; j<pdata1.cols(); ++j) {
            BOOST_CHECK_EQUAL(pdata1(i,j),pdata2(i,j));
        }
    }

    for (int i=0; i<cdata1.rows(); ++i) {
        for (int j=0; j<cdata1.cols(); ++j) {
            BOOST_CHECK_EQUAL(cdata1(i,j),cdata2(i,j));
        }
    }
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
    unsigned int N = rand() % 1000 + 1 ;
    Eigen::MatrixXd d = Eigen::MatrixXd::Random(3,N);
    obj.setPolarData(d);

    unsigned int s = obj.getSize();
    BOOST_CHECK_EQUAL( s, N );
}

BOOST_AUTO_TEST_CASE( computeCartesianData_1 )
{
    // test scan vide
    lsl::LaserScan obj;

    Eigen::VectorXd tt = Eigen::VectorXd::Zero(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(0);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(0);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(0);

    obj.computeCartesianData(tt, xx, yy, hh);

    Eigen::MatrixXd cdata = obj.getCartesianData();

    BOOST_CHECK_EQUAL( cdata.rows(), 3);
    BOOST_CHECK_EQUAL( cdata.cols(), 0);
}

BOOST_AUTO_TEST_CASE( computeCartesianData_2 )
{
    // test scan rempli aléatoirement
    lsl::LaserScan obj;

    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(1) << 0. , M_PI/2., M_PI , -M_PI/2.;
    d.row(2) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    obj.setPolarData(d);

    Eigen::VectorXd tt = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(4);

    obj.computeCartesianData(tt, xx, yy, hh);

    Eigen::MatrixXd cdata = obj.getCartesianData();

    BOOST_CHECK_EQUAL( cdata.rows(), 3 );
    BOOST_CHECK_EQUAL( cdata.cols(), 4 );

    for (int j=0; j<cdata.cols(); ++j) {
        BOOST_CHECK_EQUAL(cdata(0,j),d(0,j));
    }

    BOOST_CHECK_CLOSE( 1.0, cdata(1,0), 1.f); BOOST_CHECK( abs(cdata(2,0)) < 0.00001f);
    BOOST_CHECK( abs(cdata(1,1)) < 0.00001f); BOOST_CHECK_CLOSE( 0.5, cdata(2,1), 1.f);
    BOOST_CHECK_CLOSE(-2.0, cdata(1,2), 1.f); BOOST_CHECK( abs(cdata(2,2)) < 0.00001f);
    BOOST_CHECK( abs(cdata(1,3)) < 0.00001f); BOOST_CHECK_CLOSE(-3.0, cdata(2,3), 1.1f);
}

BOOST_AUTO_TEST_CASE( setPolarData_1 )
{
    lsl::LaserScan obj;
    unsigned int N = rand() % 1000 + 1 ;
    Eigen::MatrixXd d = Eigen::MatrixXd::Random(3,N);
    obj.setPolarData(d);

    Eigen::VectorXd tt = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(N);

    obj.computeCartesianData(tt, xx, yy, hh);

    Eigen::MatrixXd pdata = obj.getPolarData();
    Eigen::MatrixXd cdata = obj.getCartesianData();

    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), N);

    for (int i=0; i<pdata.rows(); ++i) {
        for (int j=0; j<pdata.cols(); ++j) {
            BOOST_CHECK_EQUAL(pdata(i,j),d(i,j));
        }
    }

    BOOST_CHECK_EQUAL( cdata.rows(), 3);
    BOOST_CHECK_EQUAL( cdata.cols(), 0);
}

BOOST_AUTO_TEST_CASE( getPolarData_1 )
{
    // test scan vide
    lsl::LaserScan obj;

    Eigen::MatrixXd pdata = obj.getPolarData();

    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), 0);
}

BOOST_AUTO_TEST_CASE( getPolarData_2 )
{
    // test scan rempli aléatoirement
    lsl::LaserScan obj;
    unsigned int N = rand() % 1000 + 1 ;
    Eigen::MatrixXd d = Eigen::MatrixXd::Random(3,N);
    obj.setPolarData(d);

    Eigen::MatrixXd pdata = obj.getPolarData();

    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), N);

    for (int i=0; i<pdata.rows(); ++i) {
        for (int j=0; j<pdata.cols(); ++j) {
            BOOST_CHECK_EQUAL(pdata(i,j),d(i,j));
        }
    }
}

BOOST_AUTO_TEST_CASE( getCartesianData_1 )
{
    // test scan vide
    lsl::LaserScan obj;

    Eigen::MatrixXd cdata = obj.getCartesianData();

    BOOST_CHECK_EQUAL( cdata.rows(), 3);
    BOOST_CHECK_EQUAL( cdata.cols(), 0);
}

BOOST_AUTO_TEST_CASE( getCartesianData_2 )
{
    // test scan rempli aléatoirement
    lsl::LaserScan obj;
    unsigned int N = rand() % 1000 + 1 ;
    Eigen::MatrixXd d = Eigen::MatrixXd::Random(3,N);
    obj.setPolarData(d);

    Eigen::VectorXd tt = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(N);

    obj.computeCartesianData(tt, xx, yy, hh);

    Eigen::MatrixXd cdata = obj.getCartesianData();

    BOOST_CHECK_EQUAL( cdata.rows(), 3);
    BOOST_CHECK_EQUAL( cdata.cols(), N);

    for (int j=0; j<cdata.cols(); ++j) {
        BOOST_CHECK_EQUAL(cdata(0,j), d(0,j) );
        BOOST_CHECK_EQUAL(cdata(1,j), d(1,j) * std::cos( d(2,j) ) );
        BOOST_CHECK_EQUAL(cdata(2,j), d(1,j) * std::sin( d(2,j) ) );
    }
}

BOOST_AUTO_TEST_CASE( getTimeData_1 )
{
    // test scan vide
    lsl::LaserScan obj;

    Eigen::VectorXd tdata = obj.getTimeData();

    BOOST_CHECK_EQUAL( tdata.size(), 0);
}

BOOST_AUTO_TEST_CASE( getTimeData_2 )
{
    // test scan rempli aléatoirement
    lsl::LaserScan obj;
    unsigned int N = rand() % 1000 + 1 ;
    Eigen::MatrixXd d = Eigen::MatrixXd::Random(3,N);
    obj.setPolarData(d);

    Eigen::VectorXd tdata = obj.getTimeData();

    BOOST_CHECK_EQUAL( tdata.size(), N);

    for (int j=0; j<tdata.cols(); ++j) {
        BOOST_CHECK_EQUAL(tdata(j), d(0,j) );
    }
}

BOOST_AUTO_TEST_CASE( areCartesianDataAvailable_1 )
{
    // test cartesian data available
    lsl::LaserScan obj;
    unsigned int N = rand() % 1000 + 1 ;
    Eigen::MatrixXd d = Eigen::MatrixXd::Random(3,N);
    obj.setPolarData(d);

    Eigen::VectorXd tt = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(N);

    obj.computeCartesianData(tt, xx, yy, hh);

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
    unsigned int N = rand() % 1000 + 1 ;
    Eigen::MatrixXd d = Eigen::MatrixXd::Random(3,N);
    obj.setPolarData(d);

    bool val = obj.areCartesianDataAvailable();
    BOOST_CHECK( !val );
}

BOOST_AUTO_TEST_CASE( cleanUp_1 )
{
    // test scan vide
    lsl::LaserScan obj;

    unsigned int n = obj.cleanUp();

    Eigen::MatrixXd pdata = obj.getPolarData();
    Eigen::MatrixXd cdata = obj.getCartesianData();

    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), 0);
    BOOST_CHECK_EQUAL( cdata.rows(), 3);
    BOOST_CHECK_EQUAL( cdata.cols(), 0);

    BOOST_CHECK_EQUAL( n, 0 );
}

BOOST_AUTO_TEST_CASE( cleanUp_2 )
{
    // test cleanUp non nécessaire
    lsl::LaserScan obj;
    unsigned int N = rand() % 1000 + 1 ;
    Eigen::MatrixXd d = Eigen::MatrixXd::Ones(3,N);
    obj.setPolarData(d);

    unsigned int n = obj.cleanUp();

    Eigen::MatrixXd pdata = obj.getPolarData();

    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), N);

    for (int i=0; i<pdata.rows(); ++i) {
        for (int j=0; j<pdata.cols(); ++j) {
            BOOST_CHECK_EQUAL(pdata(i,j),d(i,j));
        }
    }

    BOOST_CHECK_EQUAL( n, 0 );
}

BOOST_AUTO_TEST_CASE( cleanUp_3 )
{
    // test cleanUp nécessaire
    lsl::LaserScan obj;

    Eigen::MatrixXd d = Eigen::MatrixXd::Random(3,7);
    d.row(1) << 1. , 0., 3., 4., 0.0001, 0., 9.;

    unsigned int n = obj.cleanUp();

    Eigen::MatrixXd pdata = obj.getPolarData();

    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), 5);

    BOOST_CHECK_EQUAL( pdata.col(0), d.col(0) );
    BOOST_CHECK_EQUAL( pdata.col(1), d.col(2) );
    BOOST_CHECK_EQUAL( pdata.col(2), d.col(3) );
    BOOST_CHECK_EQUAL( pdata.col(3), d.col(4) );
    BOOST_CHECK_EQUAL( pdata.col(4), d.col(6) );

    BOOST_CHECK_EQUAL( n, 2 );
}

BOOST_AUTO_TEST_CASE( cleanUp_4 )
{
    // test cleanUp nécessaire
    lsl::LaserScan obj;

    Eigen::MatrixXd d = Eigen::MatrixXd::Random(3,7);
    d.row(1) << 1. , 0., 3., 4., 0.0001, 0., 9.;

    unsigned int n = obj.cleanUp(0.0002);

    Eigen::MatrixXd pdata = obj.getPolarData();

    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), 4);

    BOOST_CHECK_EQUAL( pdata.col(0), d.col(0) );
    BOOST_CHECK_EQUAL( pdata.col(1), d.col(2) );
    BOOST_CHECK_EQUAL( pdata.col(2), d.col(3) );
    BOOST_CHECK_EQUAL( pdata.col(3), d.col(6) );

    BOOST_CHECK_EQUAL( n, 3 );
}

BOOST_AUTO_TEST_SUITE_END()
