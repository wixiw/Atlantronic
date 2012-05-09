/*
 * uTest_BeaconDetector.hpp
 *
 *  Created on: 23 Mars 2012
 *      Author: boris
 */

#include "KFL/BeaconDetector.hpp"

#include "LSL/tools/JsonScanParser.hpp"

#include "KFL/Logger.hpp"
#include <tools/vjson/JsonDocument.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace kfl;
using namespace arp_core::log;


BOOST_AUTO_TEST_SUITE( unittest_BeaconDetector )

namespace unittest_BeaconDetector
{

    void doTest(unsigned int xpIndex)
    {
        std::string p = ros::package::getPath("arp_rlu");

        std::stringstream xpSS;
        xpSS << xpIndex;
        std::string xpName = xpSS.str();

        vjson::JsonDocument docTraj;
        std::string trajFileName = p + "/ressource/unittest/KFL/BeaconDetector/traj_" + xpName + ".json";
        BOOST_CHECK( docTraj.parse( trajFileName.c_str() ) );
        unsigned int n = docTraj.getIntegerData( docTraj.getChild( docTraj.root(), "size") );
        Eigen::VectorXd tt = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd xx = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd yy = Eigen::VectorXd::Zero(n);
        Eigen::VectorXd hh = Eigen::VectorXd::Zero(n);
        for(unsigned int i = 0 ; i < n ; i++)
        {
            tt[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "tt") , i ) );
            xx[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "xx") , i ) );
            yy[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "yy") , i ) );
            hh[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "hh") , i ) );
        }

        std::string scanFileName = p + "/ressource/unittest/KFL/BeaconDetector/scan_" + xpName + ".json";
        arp_rlu::lsl::JsonScanParser parser( scanFileName.c_str() );
        lsl::LaserScan ls;
        bool res = parser.getScan(ls);
        BOOST_CHECK(res);
        unsigned int N = ls.getSize();

        BOOST_CHECK_EQUAL(n , N);

        kfl::BeaconDetector obj;

        kfl::BeaconDetector::Params     procParams;
        procParams.mfp.width = 3;
        procParams.pcp.minRange = 0.01;
        procParams.pcp.maxRange = 10.0;
        procParams.pcp.minTheta = -PI;
        procParams.pcp.maxTheta = PI;
        procParams.psp.rangeThres = 0.08;
        procParams.minNbPoints = 4;
        procParams.cip.radius = 0.04;
        procParams.cip.coeffs = std::vector<double>();
        procParams.cip.coeffs.push_back( 1.0);
        procParams.cip.coeffs.push_back( 0.034);
        procParams.tcp.radiusTolerance = 0.03;
        procParams.tcp.distanceTolerance = 0.6;
        procParams.tcp.maxLengthTolerance = 0.05;
        procParams.tcp.medLengthTolerance = 0.05;
        procParams.tcp.minLengthTolerance = 0.05;
        procParams.dcp.radiusTolerance = 0.03;
        procParams.dcp.distanceTolerance = 0.3;
        procParams.dcp.lengthTolerance = 0.05;

        obj.setParams(procParams);

        std::vector<lsl::Circle> referencedBeacons;
        referencedBeacons.push_back(lsl::Circle(-1.5, 1.0, 0.04));
        referencedBeacons.push_back(lsl::Circle(-1.5,-1.0, 0.04));
        referencedBeacons.push_back(lsl::Circle( 1.5, 0.0, 0.04));
        obj.setReferencedBeacons(referencedBeacons);

        obj.process(ls, tt, xx, yy, hh);


        std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > foundBeacons = obj.getFoundBeacons();

        std::vector< lsl::DetectedCircle > vdc;
        for(std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> >::const_iterator it = foundBeacons.begin() ; it != foundBeacons.end() ; ++it)
        {
            vdc.push_back( it->first );
        }

        vjson::JsonDocument docResults;
        std::string resultsFileName = p + "/ressource/unittest/KFL/BeaconDetector/results_" + xpName + ".json";
        BOOST_CHECK( docResults.parse( resultsFileName.c_str() ) );

        unsigned int nbObjects = docResults.getIntegerData( docResults.getChild( docResults.root(), "nbObjects") );

        BOOST_CHECK_EQUAL( nbObjects, vdc.size() );
        Log( DEBUG ) << "unittest_BeaconDetector_" << xpName  << " - " << "vdc.size()=" << vdc.size();

        Log( DEBUG ) << "unittest_BeaconDetector_" << xpName  << " - " << "Beacon found by C++ :";
        for(unsigned int i = 0 ; i < vdc.size() ; i++)
        {
            Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - "
                    << "   dc[" << i << "]"
                    << "  vdc[i].x()=" << vdc[i].x()
                    << "  vdc[i].y()=" << vdc[i].y()
                    << "  vdc[i].t()=" << vdc[i].getApparentCenterTime()
                    << "  vdc[i].scanSize=" << vdc[i].getScan().getSize();
        }

        Log( DEBUG ) << "unittest_BeaconDetector_" << xpName  << " - " << "Compare with python :";
        for(unsigned int i = 0 ; i < vdc.size() ; i++)
        {
            std::stringstream ss;
            ss << "obj_" << i;
            double xCenter = docResults.getFloatData( docResults.getChild( docResults.getChild(docResults.root(), ss.str()), "xCenter") );
            double yCenter = docResults.getFloatData( docResults.getChild( docResults.getChild(docResults.root(), ss.str()), "yCenter") );

            Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - "
                    << "   dc[" << i << "]"
                    << "  xCenter=" << xCenter
                    << "  yCenter=" << yCenter
                    << "  vdc[i].x()=" << vdc[i].x()
                    << "  vdc[i].y()=" << vdc[i].y()
                    << "  vdc[i].t()=" << vdc[i].getApparentCenterTime();

            BOOST_CHECK_CLOSE( vdc[i].r() , 0.04, 1.f);
            BOOST_CHECK( sqrt( (vdc[i].x()-xCenter)*(vdc[i].x()-xCenter) + (vdc[i].y()-yCenter)*(vdc[i].y()-yCenter) ) < 0.025);
        }

        std::vector< std::pair<lsl::Circle, Eigen::Vector2d > > measures;
        Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "measures vector created";
        Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "tt.size()=" << tt.size();
        for(int i = 0 ; i < tt.size() ; i++)
        {
            lsl::Circle target;
            Eigen::Vector2d meas = Eigen::Vector2d::Zero();
            if( obj.getBeacon(tt[i], target, meas) )
            {
                measures.push_back( std::make_pair(target, meas) );
                Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "measure[" << measures.size() << "]:";
                Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "  * xTarget=" << target.x();
                Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "  * yTarget=" << target.y();
                Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "  * range=" << meas(0);
                Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "  * theta=" << meas(1);
            }
        }
        Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "measures done (measures.size()=" << measures.size() << ")";


        vjson::JsonDocument docMeas;
        std::string measFileName = p + "/ressource/unittest/KFL/BeaconDetector/meas_" + xpName + ".json";
        BOOST_CHECK( docMeas.parse( measFileName.c_str() ) );

        unsigned int nbMeas = docMeas.getIntegerData( docMeas.getChild( docMeas.root(), "nbMeas") );
        BOOST_CHECK_EQUAL(measures.size(), nbMeas);

        Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "results comparison";

        for(unsigned int i = 0 ; i < nbMeas ; i++)
        {
            Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "measures[" << i << "]:";

            std::stringstream ss;
            ss << "meas_" << i;
            double trueXBeacon = docMeas.getFloatData( docMeas.getChild( docMeas.getChild(docMeas.root(), ss.str()), "xBeacon") );
            double trueYBeacon = docMeas.getFloatData( docMeas.getChild( docMeas.getChild(docMeas.root(), ss.str()), "yBeacon") );
            double trueRange   = docMeas.getFloatData( docMeas.getChild( docMeas.getChild(docMeas.root(), ss.str()), "range") );
            double trueTheta   = docMeas.getFloatData( docMeas.getChild( docMeas.getChild(docMeas.root(), ss.str()), "theta") );

            lsl::Circle target = measures[i].first;
            Eigen::Vector2d meas = measures[i].second;

            Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "  * xTarget=" << target.x() << "   vs trueXBeacon=" << trueXBeacon;
            Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "  * yTarget=" << target.y() << "   vs trueYBeacon=" << trueYBeacon;
            Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "  * range="   << meas(0)    << "   vs trueRange="   << trueRange;
            Log( DEBUG ) << "unittest_BeaconDetector_" << xpName << " - " << "  * theta="   << meas(1)    << "   vs trueTheta="   << trueTheta;


            BOOST_CHECK_CLOSE( target.x(), trueXBeacon, 1.f);
            BOOST_CHECK_CLOSE( target.y(), trueYBeacon, 1.f);
            BOOST_CHECK( abs(meas(0) - trueRange) < 0.015 );
            BOOST_CHECK( abs(meas(1) - trueTheta) < 0.01);
        }

        kfl::Log( NOTICE ) << "\n";
        kfl::Log( NOTICE ) << "**************** Experience " << xpName << " *******************************";
        kfl::Log( NOTICE ) << obj.getPerformanceReport();
        kfl::Log( NOTICE ) << "************************************************************";
        kfl::Log( NOTICE ) << "\n";
    }
}

BOOST_AUTO_TEST_CASE( test_1 )
{
    unittest_BeaconDetector::doTest(1);
}

BOOST_AUTO_TEST_CASE( test_2 )
{
    unittest_BeaconDetector::doTest(2);
}

BOOST_AUTO_TEST_CASE( test_3 )
{
    unittest_BeaconDetector::doTest(3);
}

BOOST_AUTO_TEST_CASE( test_4 )
{
    unittest_BeaconDetector::doTest(4);
}

BOOST_AUTO_TEST_CASE( test_5 )
{
    unittest_BeaconDetector::doTest(5);
}

BOOST_AUTO_TEST_SUITE_END()
