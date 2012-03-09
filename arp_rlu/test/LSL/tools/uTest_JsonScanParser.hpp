/*
 * uTest_JsonScanParser.hpp
 *
 *  Created on: 2 mars 2012
 *      Author: boris
 */

#include "LSL/tools/JsonScanParser.hpp"

#include "LSL/Logger.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;
using namespace vjson;
using namespace arp_core::log;

BOOST_AUTO_TEST_SUITE( unittest_JsonScanParser )

BOOST_AUTO_TEST_CASE( test_Scan_Parsing )
{
    arp_rlu::lsl::JsonScanParser parser("../ressource/unittest/BFL/JsonScanParser/scan.json");

    lsl::LaserScan ls;
    bool res = parser.getScan(ls);

    BOOST_CHECK(res);

    BOOST_CHECK_EQUAL( ls.getSize(), 681 );

    Eigen::VectorXd tt = ls.getTimeData();
    BOOST_CHECK_CLOSE( tt[  0], 0.033496093750000004, 1.f );
    BOOST_CHECK_CLOSE( tt[  1], 0.033593749999999999, 1.f );
    BOOST_CHECK_CLOSE( tt[680], 0.099902343750000011, 1.f );

    Eigen::MatrixXd polar = ls.getPolarData();
    BOOST_CHECK_CLOSE( polar(1,  0), 0.0, 1.f );
    BOOST_CHECK_CLOSE( polar(1,  1), 0.0, 1.f );
    BOOST_CHECK_CLOSE( polar(1,680), 0.0, 1.f );
    BOOST_CHECK_CLOSE( polar(2,  0), -2.086213871524472, 1.f );
    BOOST_CHECK_CLOSE( polar(2,  1), -2.0800779483729293, 1.f );
    BOOST_CHECK_CLOSE( polar(2,680), 2.0862138715246088, 1.f );

}

BOOST_AUTO_TEST_CASE( test_Scan_Writing )
{
    arp_rlu::lsl::JsonScanParser parser("../ressource/unittest/BFL/JsonScanParser/scan.json");

    lsl::LaserScan scan;
    bool res = parser.getScan(scan);

    int N = scan.getSize();
    Eigen::VectorXd xx = Eigen::VectorXd::Random(N);
    Eigen::VectorXd yy = Eigen::VectorXd::Random(N);
    Eigen::VectorXd hh = Eigen::VectorXd::Random(N);
    scan.computeCartesianData(scan.getTimeData(), xx, yy, hh);

    BOOST_CHECK( arp_rlu::lsl::export_json( scan, "./scan_out.json" ));

    arp_rlu::lsl::JsonScanParser parser2("./scan_out.json");

    lsl::LaserScan ls;
    res = parser2.getScan(ls);
    BOOST_CHECK(res);

    BOOST_CHECK_EQUAL( ls.getSize(), scan.getSize() );

    Eigen::MatrixXd polarRef = scan.getPolarData();
    Eigen::MatrixXd polar = ls.getPolarData();
    for(int i = 0 ; i < N ; i++)
    {
        BOOST_CHECK( abs(polar(0,i) - polarRef(0,i)) < 0.0001);
        BOOST_CHECK( abs(polar(1,i) - polarRef(1,i)) < 0.0001);
        BOOST_CHECK( abs(polar(2,i) - polarRef(2,i)) < 0.0001);
    }

    BOOST_CHECK( ls.areCartesianDataAvailable() );
    Eigen::MatrixXd cartRef = scan.getCartesianData();
    Eigen::MatrixXd cart = ls.getCartesianData();
    for(int i = 0 ; i < N ; i++)
    {
        BOOST_CHECK_CLOSE( cart(0,i) , cartRef(0,i), 1.f);
        BOOST_CHECK_CLOSE( cart(1,i) , cartRef(1,i), 1.f);
        BOOST_CHECK_CLOSE( cart(2,i) , cartRef(2,i), 1.f);
        BOOST_CHECK_CLOSE( cart(3,i) , cartRef(3,i), 1.f);
        BOOST_CHECK_CLOSE( cart(4,i) , cartRef(4,i), 1.f);
        BOOST_CHECK_CLOSE( cart(5,i) , cartRef(5,i), 1.f);
    }

}

BOOST_AUTO_TEST_SUITE_END()
