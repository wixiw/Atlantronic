/*
 * uTest_KFLocalizator.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "KFL/KFLocalizator.hpp"

#include "LSL/tools/JsonScanParser.hpp"
#include "KFL/Logger.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace kfl;
using namespace arp_core::log;

BOOST_AUTO_TEST_SUITE( unittest_KFLocalizator_Dynamic )

namespace unittest_KFLocalizator_Dynamic
{

    void doTest(unsigned int xpIndex)
    {
        std::stringstream xpSS;
        xpSS << xpIndex;
        std::string xpName = xpSS.str();

        kfl::Log( INFO ) << "============================================================================================";
        kfl::Log( INFO ) << "============================================================================================";
        kfl::Log( INFO ) << "=== Experience " << xpIndex;
        kfl::Log( INFO ) << "============================================================================================";
        kfl::Log( INFO ) << "============================================================================================";

        const double transPrecisionAgainstPy = 0.015;
        const double rotPrecisionAgainstPy = deg2rad(1.0);

        const double covPrecisionDiag = 1.e-3;
        const double covPrecisionNonDiag = 1.e-3;

        const double transPrecisionAgainstGroundTruth = 0.015;
        const double rotPrecisionAgainstGroundTruth = deg2rad(1.0);



        //*******************************************
        //********       Construction       *********
        //*******************************************

        kfl::KFLocalizator obj;


        //*******************************************
        //********        Set params        *********
        //*******************************************


        vjson::JsonDocument docParams;
        std::string kflParamsFileName = "../ressource/unittest/KFL/KFLocalizator/dynamic_" + xpName + "/kfl_params.json";
        BOOST_CHECK( docParams.parse( kflParamsFileName.c_str() ) );
        float sigmaInitialPosition  = docParams.getFloatData( docParams.getChild( docParams.root(), "sigmaInitialPosition") );
        float sigmaInitialHeading   = docParams.getFloatData( docParams.getChild( docParams.root(), "sigmaInitialHeading") );
        float sigmaTransOdoVelocity = docParams.getFloatData( docParams.getChild( docParams.root(), "sigmaTransOdoVelocity") );
        float sigmaRotOdoVelocity   = docParams.getFloatData( docParams.getChild( docParams.root(), "sigmaRotOdoVelocity") );
        float sigmaLaserRange       = docParams.getFloatData( docParams.getChild( docParams.root(), "sigmaLaserRange") );
        float sigmaLaserAngle       = docParams.getFloatData( docParams.getChild( docParams.root(), "sigmaLaserAngle") );
        int iekf_Nit                = docParams.getIntegerData( docParams.getChild( docParams.root(), "iekf_Nit") );
        float iekf_xThres           = docParams.getFloatData( docParams.getChild( docParams.root(), "iekf_xThreshold") );
        float iekf_yThres           = docParams.getFloatData( docParams.getChild( docParams.root(), "iekf_yThreshold") );
        float iekf_hThres           = docParams.getFloatData( docParams.getChild( docParams.root(), "iekf_hThreshold") );

        kfl::KFLocalizator::IEKFParams  iekfParams;
        iekfParams.defaultOdoVelTransSigma = sigmaTransOdoVelocity;
        iekfParams.defaultOdoVelRotSigma   = sigmaRotOdoVelocity;
        iekfParams.defaultLaserRangeSigma  = sigmaLaserRange;
        iekfParams.defaultLaserThetaSigma  = sigmaLaserAngle;
        iekfParams.iekfMaxIt               = iekf_Nit;
        iekfParams.iekfInnovationMin       = sqrt( iekf_xThres*iekf_xThres + iekf_yThres*iekf_yThres + iekf_hThres*iekf_hThres );

        kfl::BeaconDetector::Params     procParams;
        procParams.mfp.width = 0;
        procParams.pcp.minRange = 0.01 * Eigen::VectorXd::Ones(1);
        procParams.pcp.maxRange = 10.0 * Eigen::VectorXd::Ones(1);
        procParams.pcp.minTheta = -PI;
        procParams.pcp.maxTheta = PI;
        procParams.psp.rangeThres = 0.08;
        procParams.minNbPoints = 4;
        procParams.cip.radius = 0.04;
        procParams.cip.rangeDelta = 0.034;
        procParams.tcp.radiusTolerance = 0.03;
        procParams.tcp.distanceTolerance = 0.6;
        procParams.tcp.maxLengthTolerance = 0.05;
        procParams.tcp.medLengthTolerance = 0.05;
        procParams.tcp.minLengthTolerance = 0.05;
        procParams.dcp.radiusTolerance = 0.03;
        procParams.dcp.distanceTolerance = 0.3;
        procParams.dcp.lengthTolerance = 0.05;


        kfl::KFLocalizator::Params     kfParams;
        kfParams.bufferSize = 100;
        kfParams.referencedBeacons.push_back( lsl::Circle( 1.5, 0., 0.04 ) );
        kfParams.referencedBeacons.push_back( lsl::Circle(-1.5, 1., 0.04 ) );
        kfParams.referencedBeacons.push_back( lsl::Circle(-1.5,-1., 0.04 ) );
        kfParams.iekfParams = iekfParams;
        kfParams.procParams = procParams;

        obj.setParams(kfParams);

        kfl::Log( INFO ) << "================================";
        kfl::Log( INFO ) << "=== KFLOCALIZATOR PARAMETERS ===";
        kfl::Log( INFO ) << kfParams.getInfo();


        //*******************************************
        //********        Initialize        *********
        //*******************************************

        vjson::JsonDocument docInit;
        std::string initialPositionFileName = "../ressource/unittest/KFL/KFLocalizator/dynamic_" + xpName + "/initial_position.json";
        BOOST_CHECK( docInit.parse(initialPositionFileName.c_str() ) );
        float estimInitialXPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "estimInitialXPosition") );
        float estimInitialYPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "estimInitialYPosition") );
        float estimInitialHPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "estimInitialHPosition") );
        float trueInitialXPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "trueInitialXPosition") );
        float trueInitialYPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "trueInitialYPosition") );
        float trueInitialHPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "trueInitialHPosition") );

        EstimatedPose2D initialPose;
        initialPose.x(estimInitialXPosition);
        initialPose.y(estimInitialYPosition);
        initialPose.h(estimInitialHPosition);
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
        cov.diagonal() = Eigen::Vector3d(sigmaInitialPosition*sigmaInitialPosition,
                sigmaInitialPosition*sigmaInitialPosition,
                sigmaInitialHeading*sigmaInitialHeading );
        initialPose.cov(cov);
        initialPose.date(0.0);

        BOOST_CHECK(obj.initialize(initialPose));


        arp_math::EstimatedPose2D initEstim;
        initEstim = obj.getLastEstimatedPose2D();

//        kfl::Log( DEBUG ) << "position réelle [time=0.0]:";
//        kfl::Log( DEBUG ) << "  sur x (en m): " << trueInitialXPosition;
//        kfl::Log( DEBUG ) << "  sur y (en m): " << trueInitialYPosition;
//        kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( trueInitialHPosition ));
//        kfl::Log( INFO ) << "------------------------------";
//        kfl::Log( INFO ) << "position initiale [time=0.0]:";
//        kfl::Log( INFO ) << "  sur x (en m): " << initEstim.x();
//        kfl::Log( INFO ) << "  sur y (en m): " << initEstim.y();
//        kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( initEstim.h() ));
        kfl::Log( INFO ) << "------------------------------";
        kfl::Log( INFO ) << "erreur initiale [time=0.0]:";
        kfl::Log( INFO ) << "  sur x (en mm): " << (initEstim.x() - trueInitialXPosition) * 1000. << " +/- " << 2000. * sqrt(cov(0,0));
        kfl::Log( INFO ) << "  sur y (en mm): " << (initEstim.y() - trueInitialYPosition) * 1000. << " +/- " << 2000. * sqrt(cov(1,1));
        kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( initEstim.h() - trueInitialHPosition ) ) << 2.0 * rad2deg(sqrt(cov(2,2)));


        //*******************************************
        //********       Load Timeline      *********
        //*******************************************
        vjson::JsonDocument docTimeline;
        std::string timelineFileName = "../ressource/unittest/KFL/KFLocalizator/dynamic_" + xpName + "/time_line.json";
        BOOST_CHECK( docTimeline.parse(timelineFileName.c_str() ) );
        const unsigned int N = docTimeline.getIntegerData( docTimeline.getChild( docTimeline.root(), "nb" ) );
        Eigen::VectorXd times(N);
        Eigen::VectorXd doOdos(N);
        Eigen::VectorXd doLrfs(N);
        for(unsigned int k = 0 ; k < N ; k++)
        {
            times(k) = docTimeline.getFloatData( docTimeline.getChild( docTimeline.getChild( docTimeline.root(), "times"), k ) );
            doOdos(k) = docTimeline.getFloatData( docTimeline.getChild( docTimeline.getChild( docTimeline.root(), "doOdo"), k ) );
            doLrfs(k) = docTimeline.getFloatData( docTimeline.getChild( docTimeline.getChild( docTimeline.root(), "doLrf"), k ) );
        }

        double startTime = arp_math::getTime();
        //*******************************************
        //********          Start           *********
        //*******************************************
        for(unsigned int k = 1 ; k < N ; k++)
        {
            double time = times(k);
            bool doOdo = doOdos(k) > 0.5;
            bool doLrf = doLrfs(k) > 0.5;

            float trueX;
            float trueY;
            float trueH;

            if( doOdo || doLrf )
            {
                kfl::Log( DEBUG ) << "============================================================================================";
                kfl::Log( DEBUG ) << "============================================================================================";
                kfl::Log( DEBUG ) << "Time=" << time;

                vjson::JsonDocument docInit;
                std::stringstream truePosFileName;
                truePosFileName << "../ressource/unittest/KFL/KFLocalizator/dynamic_" << xpName << "/t_" << std::setprecision(6) << time << "_true_position.json";
                BOOST_CHECK( docInit.parse(truePosFileName.str().c_str() ) );
                trueX = docInit.getFloatData( docInit.getChild( docInit.root(), "x") );
                trueY = docInit.getFloatData( docInit.getChild( docInit.root(), "y") );
                trueH = docInit.getFloatData( docInit.getChild( docInit.root(), "h") );

//                kfl::Log( DEBUG ) << "------------------------------";
//                kfl::Log( DEBUG ) << "position réelle :";
//                kfl::Log( DEBUG ) << "  x (en m): " << trueX;
//                kfl::Log( DEBUG ) << "  y (en m): " << trueY;
//                kfl::Log( DEBUG ) << "  h (en deg): " << rad2deg(betweenMinusPiAndPlusPi(trueH));
            }

            if( doOdo )
            {
                kfl::Log( DEBUG ) << "============================================================";
                kfl::Log( DEBUG ) << "Time=" << time << " => ODO";


                vjson::JsonDocument docOdoVel;
                std::stringstream odoVelFileName;
                odoVelFileName << "../ressource/unittest/KFL/KFLocalizator/dynamic_" << xpName << "/t_" << time << "_odo_velocity.json";
                BOOST_CHECK( docOdoVel.parse(odoVelFileName.str().c_str()) );
                BOOST_CHECK_CLOSE( time , docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "t") ), 1.f);

                arp_math::EstimatedTwist2D odoVel;
                odoVel.date( time );
                odoVel.vx( docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vx") ) );
                odoVel.vy( docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vy") ) );
                odoVel.vh( docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vh") ) );
                double sigmaX = docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "sigmaX") );
                double sigmaY = docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "sigmaY") );
                double sigmaH = docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "sigmaH") );
                Eigen::Matrix<double, 3,3> covariance = Eigen::Matrix<double, 3,3>::Identity();
                covariance(0,0) = sigmaX*sigmaX;
                covariance(1,1) = sigmaY*sigmaY;
                covariance(2,2) = sigmaH*sigmaH;
                odoVel.cov( covariance );

//                kfl::Log( DEBUG ) << "------------------------------";
//                kfl::Log( DEBUG ) << "vx=" << odoVel.vx() << " m/s";
//                kfl::Log( DEBUG ) << "vy=" << odoVel.vy() << " m/s";
//                kfl::Log( DEBUG ) << "vh=" << rad2deg(odoVel.vh()) << " deg/s";
//                kfl::Log( DEBUG ) << "vh=" << odoVel.vh() << " rad/s";

                double startOdoTime = arp_math::getTime();
                obj.newOdoVelocity(odoVel);
                double endOdoTime = arp_math::getTime();
                kfl::Log( INFO ) << "Odo Computation Time :" << endOdoTime - startOdoTime;

                arp_math::EstimatedPose2D odoEstim;
                odoEstim = obj.getLastEstimatedPose2D();

                vjson::JsonDocument docOdoEstim;
                std::stringstream odoEstimFileName;
                odoEstimFileName << "../ressource/unittest/KFL/KFLocalizator/dynamic_" << xpName << "/t_" << time << "_odo_estimate.json";
                BOOST_CHECK( docOdoEstim.parse(odoEstimFileName.str().c_str()) );
                BOOST_CHECK_CLOSE( time , docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "t")), 1.f );

                double pyOdoEstimT = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "t"));
                double pyOdoEstimX = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "x"));
                double pyOdoEstimY = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "y"));
                double pyOdoEstimH = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "h"));
                BOOST_CHECK_CLOSE( odoEstim.date() , pyOdoEstimT, 1.f);
                BOOST_CHECK_SMALL( odoEstim.x()    - pyOdoEstimX, transPrecisionAgainstPy);
                BOOST_CHECK_SMALL( odoEstim.y()    - pyOdoEstimY, transPrecisionAgainstPy);
                BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(odoEstim.h()- pyOdoEstimH), rotPrecisionAgainstPy);

                BOOST_CHECK_CLOSE( odoEstim.date() , time, 1.f);
                //            BOOST_CHECK_SMALL( odoEstim.x()    - trueX, transPrecisionAgainstGroundTruth);
                //            BOOST_CHECK_SMALL( odoEstim.y()    - trueY, transPrecisionAgainstGroundTruth);
                //            BOOST_CHECK_SMALL( odoEstim.h()    - trueH, rotPrecisionAgainstGroundTruth);
                for(unsigned int i = 0 ; i < 3 ; i++)
                {
                    std::stringstream rowName;
                    rowName << "row_" << i;
                    for(unsigned int j = 0 ; j < 3 ; j++)
                    {
                        // kfl::Log( NOTICE ) << "check odo covariance (" << i << "," << j << ")";
                        double cov_i_j = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.getChild( docOdoEstim.getChild( docOdoEstim.root(), "covariance"), rowName.str()), j ));
                        if( i == j )
                        {
                            BOOST_CHECK_SMALL( odoEstim.cov()(i,j) - cov_i_j, covPrecisionDiag);
                        }
                        else
                        {
                            BOOST_CHECK_SMALL( odoEstim.cov()(i,j) - cov_i_j, covPrecisionNonDiag);
                        }
                    }
                }

                BOOST_CHECK( abs(odoEstim.x() - trueX) < 3.0 * sqrt(odoEstim.cov()(0,0)) );
                BOOST_CHECK( abs(odoEstim.y() - trueY) < 3.0 * sqrt(odoEstim.cov()(1,1)) );
                BOOST_CHECK( abs(betweenMinusPiAndPlusPi( odoEstim.h() - trueH )) < 3.0 * sqrt(odoEstim.cov()(2,2)) );

//                kfl::Log( INFO ) << "------------------------------";
//                kfl::Log( INFO ) << "position apres les odos [time=" << time << "]:";
//                kfl::Log( INFO ) << "  sur x (en m): " << odoEstim.x();
//                kfl::Log( INFO ) << "  sur y (en m): " << odoEstim.y();
//                kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( odoEstim.h() ) );
                kfl::Log( INFO ) << "------------------------------";
                kfl::Log( INFO ) << "erreur apres les odos [time=" << time << "]:";
                kfl::Log( INFO ) << "  sur x (en mm): " << (odoEstim.x() - trueX) * 1000. << " +/- " << 3000. * sqrt(odoEstim.cov()(0,0));
                kfl::Log( INFO ) << "  sur y (en mm): " << (odoEstim.y() - trueY) * 1000. << " +/- " << 3000. * sqrt(odoEstim.cov()(1,1));
                kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( odoEstim.h() - trueH ) ) << " +/- " << 3.0 * rad2deg(sqrt(odoEstim.cov()(2,2)));
                kfl::Log( INFO ) << "covariance : " << odoEstim.cov().row(0);
                kfl::Log( INFO ) << "             " << odoEstim.cov().row(1);
                kfl::Log( INFO ) << "             " << odoEstim.cov().row(2);
            }


            //            if( k % (int)(scanPeriodInSec/odoPeriodInSec) == 0 )
            if( doLrf )
            {
                kfl::Log( DEBUG ) << "============================================================";
                kfl::Log( DEBUG ) << "Time=" << time << " => LRF";
                kfl::Log( DEBUG ) << "------------------------------";

                arp_rlu::lsl::JsonScanParser scanParser;
                std::stringstream scanFileName;
                scanFileName << "../ressource/unittest/KFL/KFLocalizator/dynamic_" << xpName << "/t_" << time << "_scan.json";
                BOOST_CHECK( scanParser.parse(scanFileName.str().c_str()) );
                lsl::LaserScan scan;
                BOOST_CHECK( scanParser.getScan(scan) );

                double startScanTime = arp_math::getTime();
                obj.newScan(scan);
                double endScanTime = arp_math::getTime();
                kfl::Log( INFO ) << "Scan Computation Time :" << endScanTime - startScanTime;

                kfl::Log( DEBUG ) << "------------------------------";

                arp_math::EstimatedPose2D postScanEstim;
                postScanEstim = obj.getLastEstimatedPose2D();

                vjson::JsonDocument docScanEstim;
                std::stringstream scanEstimFileName;
                scanEstimFileName << "../ressource/unittest/KFL/KFLocalizator/dynamic_" << xpName << "/t_" << time << "_scan_estimate.json";
                BOOST_CHECK( docScanEstim.parse(scanEstimFileName.str().c_str()) );
                BOOST_CHECK_CLOSE( time , docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "t")), 1.f );

                double pyScanEstimT = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "t"));
                double pyScanEstimX = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "x"));
                double pyScanEstimY = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "y"));
                double pyScanEstimH = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "h"));
                BOOST_CHECK_CLOSE( postScanEstim.date() , pyScanEstimT, 1.f);
                BOOST_CHECK_SMALL( postScanEstim.x() - pyScanEstimX, transPrecisionAgainstPy);
                BOOST_CHECK_SMALL( postScanEstim.y() - pyScanEstimY, transPrecisionAgainstPy);
                BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(postScanEstim.h() - pyScanEstimH), rotPrecisionAgainstPy);

                BOOST_CHECK_CLOSE( postScanEstim.date() , time, 1.f);
                BOOST_CHECK_SMALL( postScanEstim.x() - trueX, transPrecisionAgainstGroundTruth);
                BOOST_CHECK_SMALL( postScanEstim.y() - trueY, transPrecisionAgainstGroundTruth);
                BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(postScanEstim.h() - trueH), rotPrecisionAgainstGroundTruth);
                for(unsigned int i = 0 ; i < 3 ; i++)
                {
                    std::stringstream rowName;
                    rowName << "row_" << i;
                    for(unsigned int j = 0 ; j < 3 ; j++)
                    {
                        //                        kfl::Log( NOTICE ) << "check laser covariance (" << i << "," << j << ")";
                        double cov_i_j = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.getChild( docScanEstim.getChild( docScanEstim.root(), "covariance"), rowName.str()), j ));
                        if( i == j )
                        {
                            BOOST_CHECK_SMALL( postScanEstim.cov()(i,j) - cov_i_j, covPrecisionDiag);
                        }
                        else
                        {
                            BOOST_CHECK_SMALL( postScanEstim.cov()(i,j) - cov_i_j, covPrecisionNonDiag);
                        }
                    }
                }

                BOOST_CHECK( abs(postScanEstim.x() - trueX) < 3.0 * sqrt(postScanEstim.cov()(0,0)) );
                BOOST_CHECK( abs(postScanEstim.y() - trueY) < 3.0 * sqrt(postScanEstim.cov()(1,1)) );
                BOOST_CHECK( abs(betweenMinusPiAndPlusPi( postScanEstim.h() - trueH )) < 3.0 * sqrt(postScanEstim.cov()(2,2)) );

//                kfl::Log( INFO ) << "------------------------------";
//                kfl::Log( INFO ) << "position apres le scan [time=" << time << "]:";
//                kfl::Log( INFO ) << "  sur x (en m): " << postScanEstim.x();
//                kfl::Log( INFO ) << "  sur y (en m): " << postScanEstim.y();
//                kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( postScanEstim.h() ) );
                kfl::Log( INFO ) << "------------------------------";
                kfl::Log( INFO ) << "erreur apres le scan [time=" << time << "]:";
                kfl::Log( INFO ) << "  sur x (en mm): " << (postScanEstim.x() - trueX) * 1000. << " +/- " << 3000. * sqrt(postScanEstim.cov()(0,0));
                kfl::Log( INFO ) << "  sur y (en mm): " << (postScanEstim.y() - trueY) * 1000. << " +/- " << 3000. * sqrt(postScanEstim.cov()(1,1));
                kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( postScanEstim.h() - trueH ) )  << " +/- " << 3.0 * rad2deg(sqrt(postScanEstim.cov()(2,2)));
                kfl::Log( INFO ) << "covariance : " << postScanEstim.cov().row(0);
                kfl::Log( INFO ) << "             " << postScanEstim.cov().row(1);
                kfl::Log( INFO ) << "             " << postScanEstim.cov().row(2);
            }
        }

        double endTime = arp_math::getTime();
        kfl::Log( INFO ) << "Computation Time :" << endTime - startTime;
    }

}

BOOST_AUTO_TEST_CASE( test_1 )
{
    unittest_KFLocalizator_Dynamic::doTest(1);
}

BOOST_AUTO_TEST_CASE( test_2 )
{
    unittest_KFLocalizator_Dynamic::doTest(2);
}

BOOST_AUTO_TEST_CASE( test_3 )
{
    unittest_KFLocalizator_Dynamic::doTest(3);
}

BOOST_AUTO_TEST_CASE( test_4 )
{
    unittest_KFLocalizator_Dynamic::doTest(4);
}

BOOST_AUTO_TEST_CASE( test_5 )
{
    unittest_KFLocalizator_Dynamic::doTest(5);
}

BOOST_AUTO_TEST_CASE( test_6 )
{
    unittest_KFLocalizator_Dynamic::doTest(6);
}

BOOST_AUTO_TEST_CASE( test_7 )
{
    unittest_KFLocalizator_Dynamic::doTest(7);
}

BOOST_AUTO_TEST_SUITE_END()
