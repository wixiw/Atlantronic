/*
 * uTest_JsonScanParser.hpp
 *
 *  Created on: 2 mars 2012
 *      Author: boris
 */

#include "LSL/tools/JsonScanParser.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;
using namespace vjson;

BOOST_AUTO_TEST_SUITE( unittest_JsonScanParser )

BOOST_AUTO_TEST_CASE( test_Scan_Parsing )
{
    arp_rlu::lsl::JsonScanParser parser("./scan.json");

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

BOOST_AUTO_TEST_SUITE_END()
