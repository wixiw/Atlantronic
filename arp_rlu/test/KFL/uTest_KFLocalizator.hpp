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

BOOST_AUTO_TEST_SUITE( unittest_KFLocalizator )

namespace unittest_KFLocalizator
{

    void doTest(unsigned int xpIndex)
    {
        std::stringstream xpSS;
        xpSS << xpIndex;
        std::string xpName = xpSS.str();

        const double transPrecision = 0.025;
        const double rotPrecision = deg2rad(5.0);
        const double covPrecisionDiag = 5.e-3;
        const double covPrecisionNonDiag = 5.e-3;

        const double transFinalPrecision = 0.006;
        const double rotFinalPrecision = deg2rad(0.5);

        //*******************************************
        //********       Construction       *********
        //*******************************************

        kfl::KFLocalizator obj;


        //*******************************************
        //********        Set params        *********
        //*******************************************


        vjson::JsonDocument docParams;
        std::string kflParamsFileName = "../ressource/unittest/KFL/KFLocalizator/static_" + xpName + "/kfl_params.json";
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
        procParams.mfp.width = 3;
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


        //*******************************************
        //********        Initialize        *********
        //*******************************************

        double time = 0.0;

        vjson::JsonDocument docInit;
        std::string initialPositionFileName = "../ressource/unittest/KFL/KFLocalizator/static_" + xpName + "/initial_position.json";
        BOOST_CHECK( docInit.parse(initialPositionFileName.c_str() ) );
        float trueX            = docInit.getFloatData( docInit.getChild( docInit.root(), "trueX") );
        float trueY            = docInit.getFloatData( docInit.getChild( docInit.root(), "trueY") );
        float trueH            = docInit.getFloatData( docInit.getChild( docInit.root(), "trueH") );
        float initialXPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "initialXPosition") );
        float initialYPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "initialYPosition") );
        float initialHPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "initialHPosition") );

        EstimatedPose2D initialPose;
        initialPose.x(initialXPosition);
        initialPose.y(initialYPosition);
        initialPose.h(initialHPosition);
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
        cov.diagonal() = Eigen::Vector3d(sigmaInitialPosition*sigmaInitialPosition,
                sigmaInitialPosition*sigmaInitialPosition,
                sigmaInitialHeading*sigmaInitialHeading );
        initialPose.cov(cov);
        initialPose.date(time);

        BOOST_CHECK(obj.initialize(initialPose));



        //*******************************************
        //********          Start           *********
        //*******************************************

        const unsigned int nbScans = 5;
        const double odoPeriodInSec = 0.01;
        const double scanPeriodInSec = 0.1;

        unsigned int predictionsinceLastUpdate = 0;
        for(unsigned int k = 0 ; k < nbScans ; k++)
        {
            kfl::Log( DEBUG ) << "============================================================================================";
            kfl::Log( DEBUG ) << "============================================================================================";
            kfl::Log( DEBUG ) << " TOUR " << k << "   [time=" << time << " -> " << time + scanPeriodInSec <<"]";

            arp_math::EstimatedPose2D preOdoEstim;
            preOdoEstim = obj.getLastEstimatedPose2D();

            kfl::Log( DEBUG ) << "=======================";
            kfl::Log( DEBUG ) << "erreur statique avant les odos [time=" << time << "]:";
            kfl::Log( DEBUG ) << "  sur x (en mm): " << (preOdoEstim.x() - trueX) * 1000.;
            kfl::Log( DEBUG ) << "  sur y (en mm): " << (preOdoEstim.y() - trueY) * 1000.;
            kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( preOdoEstim.h() - trueH ) );
            kfl::Log( DEBUG ) << "covariance : " << preOdoEstim.cov().row(0);
            kfl::Log( DEBUG ) << "             " << preOdoEstim.cov().row(1);
            kfl::Log( DEBUG ) << "             " << preOdoEstim.cov().row(2);

            for(unsigned int predictionInd = 0 ; predictionInd < scanPeriodInSec/odoPeriodInSec ; predictionInd++)
            {
                time += odoPeriodInSec;

                vjson::JsonDocument docOdoVel;
                std::stringstream odoVelFileName;
                odoVelFileName << "../ressource/unittest/KFL/KFLocalizator/static_" << xpName << "/t_" << time << "_odo.json";
                BOOST_CHECK( docOdoVel.parse(odoVelFileName.str().c_str()) );
                BOOST_CHECK_CLOSE( time , docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "t") ), 1.f);

                arp_math::EstimatedTwist2D odoVel;
                odoVel.date( time );
                odoVel.vx( docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vx") ) );
                odoVel.vy( docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vy") ) );
                odoVel.vh( docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vh") ) );
                Eigen::Matrix3d odoCov = Eigen::Matrix3d::Identity();
                odoCov.diagonal() = Eigen::Vector3d(sigmaTransOdoVelocity, sigmaTransOdoVelocity, sigmaRotOdoVelocity );
                odoVel.cov( odoCov );

                kfl::Log( DEBUG ) << "[time=" << time << "]: newOdoVelocity #" << predictionsinceLastUpdate ;
                obj.newOdoVelocity(odoVel);
                predictionsinceLastUpdate++;

                arp_math::EstimatedPose2D odoEstim;
                odoEstim = obj.getLastEstimatedPose2D();

                vjson::JsonDocument docOdoEstim;
                std::stringstream odoEstimFileName;
                odoEstimFileName << "../ressource/unittest/KFL/KFLocalizator/static_" << xpName << "/t_" << time << "_odo_estimate.json";
                BOOST_CHECK( docOdoEstim.parse(odoEstimFileName.str().c_str()) );
                BOOST_CHECK_CLOSE( time , docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "t")), 1.f );

                double groudTrueT = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "t"));
                double groudTrueX = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "x"));
                double groudTrueY = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "y"));
                double groudTrueH = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "h"));
                BOOST_CHECK_CLOSE( odoEstim.date() , groudTrueT, 1.f);
                BOOST_CHECK_SMALL( odoEstim.x()    - groudTrueX, transPrecision);
                BOOST_CHECK_SMALL( odoEstim.y()    - groudTrueY, transPrecision);
                BOOST_CHECK_SMALL( odoEstim.h()    - groudTrueH, rotPrecision);
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
            }

            arp_math::EstimatedPose2D postOdoEstim;
            postOdoEstim = obj.getLastEstimatedPose2D();

            kfl::Log( DEBUG ) << "=======================";
            kfl::Log( DEBUG ) << "erreur statique apres les odos [time=" << time << "]:";
            kfl::Log( DEBUG ) << "  sur x (en mm): " << (postOdoEstim.x() - trueX) * 1000.;
            kfl::Log( DEBUG ) << "  sur y (en mm): " << (postOdoEstim.y() - trueY) * 1000.;
            kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( postOdoEstim.h() - trueH ) );
            kfl::Log( DEBUG ) << "covariance : " << postOdoEstim.cov().row(0);
            kfl::Log( DEBUG ) << "             " << postOdoEstim.cov().row(1);
            kfl::Log( DEBUG ) << "             " << postOdoEstim.cov().row(2);


            arp_rlu::lsl::JsonScanParser scanParser;
            std::stringstream scanFileName;
            scanFileName << "../ressource/unittest/KFL/KFLocalizator/static_" << xpName << "/t_" << time << "_scan.json";
            BOOST_CHECK( scanParser.parse(scanFileName.str().c_str()) );
            lsl::LaserScan scan;
            BOOST_CHECK( scanParser.getScan(scan) );

            kfl::Log( DEBUG ) << "[time=" << time << "]: newScan";
            obj.newScan(scan);
            predictionsinceLastUpdate = 0;

            arp_math::EstimatedPose2D postScanEstim;
            postScanEstim = obj.getLastEstimatedPose2D();

            vjson::JsonDocument docScanEstim;
            std::stringstream scanEstimFileName;
            scanEstimFileName << "../ressource/unittest/KFL/KFLocalizator/static_" << xpName << "/t_" << time << "_scan_estimate.json";
            BOOST_CHECK( docScanEstim.parse(scanEstimFileName.str().c_str()) );
            BOOST_CHECK_CLOSE( time , docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "t")), 1.f );

            double groundTrueT = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "t"));
            double groundTrueX = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "x"));
            double groundTrueY = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "y"));
            double groundTrueH = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "h"));
            BOOST_CHECK_CLOSE( postScanEstim.date() , groundTrueT, 1.f);
            BOOST_CHECK_SMALL( postScanEstim.x() - groundTrueX, transPrecision);
            BOOST_CHECK_SMALL( postScanEstim.y() - groundTrueY, transPrecision);
            BOOST_CHECK_SMALL( postScanEstim.h() - groundTrueH, rotPrecision);
            for(unsigned int i = 0 ; i < 3 ; i++)
            {
                std::stringstream rowName;
                rowName << "row_" << i;
                for(unsigned int j = 0 ; j < 3 ; j++)
                {
                    //                kfl::Log( NOTICE ) << "check laser covariance (" << i << "," << j << ")";
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

            kfl::Log( DEBUG ) << "=======================";
            kfl::Log( DEBUG ) << "erreur statique apres le scan [time=" << time << "]:";
            kfl::Log( DEBUG ) << "  sur x (en mm): " << (postScanEstim.x() - trueX) * 1000.;
            kfl::Log( DEBUG ) << "  sur y (en mm): " << (postScanEstim.y() - trueY) * 1000.;
            kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( postScanEstim.h() - trueH ) );
            kfl::Log( DEBUG ) << "covariance : " << postScanEstim.cov().row(0);
            kfl::Log( DEBUG ) << "             " << postScanEstim.cov().row(1);
            kfl::Log( DEBUG ) << "             " << postScanEstim.cov().row(2);
        }

        arp_math::EstimatedPose2D lastEstim;
        lastEstim = obj.getLastEstimatedPose2D();

        BOOST_CHECK_SMALL( lastEstim.x() - trueX , transFinalPrecision );
        BOOST_CHECK_SMALL( lastEstim.y() - trueY , transFinalPrecision );
        BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(lastEstim.h() - trueH) , rotFinalPrecision );
    }
}

BOOST_AUTO_TEST_CASE( test_1 )
{
    unittest_KFLocalizator::doTest(1);
}

BOOST_AUTO_TEST_CASE( test_2 )
{
    unittest_KFLocalizator::doTest(2);
}

BOOST_AUTO_TEST_CASE( test_3 )
{
    unittest_KFLocalizator::doTest(3);
}

BOOST_AUTO_TEST_CASE( test_4 )
{
    unittest_KFLocalizator::doTest(4);
}

BOOST_AUTO_TEST_CASE( test_5 )
{
    unittest_KFLocalizator::doTest(5);
}

BOOST_AUTO_TEST_CASE( test_6 )
{
    unittest_KFLocalizator::doTest(6);
}

BOOST_AUTO_TEST_CASE( test_7 )
{
    unittest_KFLocalizator::doTest(7);
}

BOOST_AUTO_TEST_CASE( test_8 )
{
    unittest_KFLocalizator::doTest(8);
}

BOOST_AUTO_TEST_CASE( test_9 )
{
    unittest_KFLocalizator::doTest(9);
}

BOOST_AUTO_TEST_CASE( test_10 )
{
    unittest_KFLocalizator::doTest(10);
}

BOOST_AUTO_TEST_SUITE_END()
