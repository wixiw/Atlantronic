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

BOOST_AUTO_TEST_SUITE( unittest_KFLocalizator_Static )

namespace unittest_KFLocalizator_Static
{

    void doTest(unsigned int xpIndex)
    {
        std::string p = ros::package::getPath("arp_rlu");

        std::stringstream xpSS;
        xpSS << xpIndex;
        std::string xpName = xpSS.str();

        const double transPrecision = 0.025;
        const double rotPrecision = deg2rad(5.0);
        const double covPrecisionDiag = 8.e-3;
        const double covPrecisionNonDiag = 8.e-3;

        const double transFinalPrecision = 0.010;
        const double rotFinalPrecision = deg2rad(0.5);

        //*******************************************
        //********       Construction       *********
        //*******************************************

        kfl::KFLocalizator obj;


        //*******************************************
        //********        Set params        *********
        //*******************************************


        vjson::JsonDocument docParams;
        std::string kflParamsFileName = p + "/ressource/unittest/KFL/KFLocalizator/static_" + xpName + "/kfl_params.json";
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

        arp_math::Pose2D H_hky_robot;
        arp_math::Pose2D H_odo_robot;

        kfl::KFLocalizator::IEKFParams  iekfParams;
        iekfParams.defaultOdoVelTransSigma = sigmaTransOdoVelocity;
        iekfParams.defaultOdoVelRotSigma   = sigmaRotOdoVelocity;
        iekfParams.defaultLaserRangeSigma  = sigmaLaserRange;
        iekfParams.defaultLaserThetaSigma  = sigmaLaserAngle;
        iekfParams.iekfMaxIt               = iekf_Nit;
        iekfParams.iekfInnovationMin       = sqrt( iekf_xThres*iekf_xThres + iekf_yThres*iekf_yThres + iekf_hThres*iekf_hThres );

        kfl::BeaconDetector::Params     procParams;
        procParams.mfp.width = 3;
        procParams.pcp.minRange = 0.01;
        procParams.pcp.maxRange = 10.0;
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
        kfParams.H_hky_robot = H_hky_robot;
        kfParams.H_odo_robot = H_odo_robot;

        obj.setParams(kfParams);


        //*******************************************
        //********        Initialize        *********
        //*******************************************

        double time = 0.0;

        vjson::JsonDocument docInit;
        std::string initialPositionFileName = p + "/ressource/unittest/KFL/KFLocalizator/static_" + xpName + "/initial_position.json";
        BOOST_CHECK( docInit.parse(initialPositionFileName.c_str() ) );
        float trueX            = docInit.getFloatData( docInit.getChild( docInit.root(), "trueX") );
        float trueY            = docInit.getFloatData( docInit.getChild( docInit.root(), "trueY") );
        float trueH            = docInit.getFloatData( docInit.getChild( docInit.root(), "trueH") );
        float initialXPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "initialXPosition") );
        float initialYPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "initialYPosition") );
        float initialHPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "initialHPosition") );

        EstimatedPose2D init_H_hky_table;
        init_H_hky_table.x(initialXPosition);
        init_H_hky_table.y(initialYPosition);
        init_H_hky_table.h(initialHPosition);
        Eigen::Matrix3d cov = Eigen::Vector3d(sigmaInitialPosition*sigmaInitialPosition,
                sigmaInitialPosition*sigmaInitialPosition,
                sigmaInitialHeading*sigmaInitialHeading ).asDiagonal();
        init_H_hky_table.cov(cov);
        init_H_hky_table.date(0.0);

        EstimatedPose2D init_H_robot_table = init_H_hky_table * H_hky_robot.inverse();
        BOOST_CHECK(obj.initialize(init_H_robot_table));

        kfl::Log( DEBUG ) << "============================================================================================";
        kfl::Log( DEBUG ) << "============================================================================================";
        kfl::Log( DEBUG ) << "=== TEST " << xpName;

        arp_math::EstimatedPose2D initEstim_H_robot_table;
        initEstim_H_robot_table = obj.getLastEstimatedPose2D();
        arp_math::EstimatedPose2D initEstim_H_hky_table = initEstim_H_robot_table * H_hky_robot;

        kfl::Log( DEBUG ) << "Position de départ [time=0.0]:   ie initEstim_H_hky_table";
        kfl::Log( DEBUG ) << "  sur x (en m): " << (initEstim_H_hky_table.x());
        kfl::Log( DEBUG ) << "  sur y (en m): " << (initEstim_H_hky_table.y());
        kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( initEstim_H_hky_table.h() ) );
        kfl::Log( DEBUG ) << "covariance : " << initEstim_H_hky_table.cov().row(0);
        kfl::Log( DEBUG ) << "             " << initEstim_H_hky_table.cov().row(1);
        kfl::Log( DEBUG ) << "             " << initEstim_H_hky_table.cov().row(2);


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

            arp_math::EstimatedPose2D preOdoEstim_H_robot_table;
            preOdoEstim_H_robot_table = obj.getLastEstimatedPose2D();
            arp_math::EstimatedPose2D preOdoEstim_H_hky_table = preOdoEstim_H_robot_table * H_hky_robot;

            kfl::Log( DEBUG ) << "=======================";
            kfl::Log( DEBUG ) << "erreur statique avant les odos [time=" << time << "]:";
            kfl::Log( DEBUG ) << "  sur x (en mm): " << (preOdoEstim_H_hky_table.x() - trueX) * 1000.;
            kfl::Log( DEBUG ) << "  sur y (en mm): " << (preOdoEstim_H_hky_table.y() - trueY) * 1000.;
            kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( preOdoEstim_H_hky_table.h() - trueH ) );
            kfl::Log( DEBUG ) << "covariance : " << preOdoEstim_H_hky_table.cov().row(0);
            kfl::Log( DEBUG ) << "             " << preOdoEstim_H_hky_table.cov().row(1);
            kfl::Log( DEBUG ) << "             " << preOdoEstim_H_hky_table.cov().row(2);

            for(unsigned int predictionInd = 0 ; predictionInd < scanPeriodInSec/odoPeriodInSec ; predictionInd++)
            {
                time += odoPeriodInSec;

                vjson::JsonDocument docOdoVel;
                std::stringstream odoVelFileName;
                odoVelFileName << p + "/ressource/unittest/KFL/KFLocalizator/static_" << xpName << "/t_" << time << "_odo.json";
                BOOST_CHECK( docOdoVel.parse(odoVelFileName.str().c_str()) );
                BOOST_CHECK_CLOSE( time , docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "t") ), 1.f);

                double vx = docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vx") );
                double vy = docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vy") );
                double vh = docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vh") );

                arp_math::Vector3 inputMean;
                inputMean << vx, vy, vh;
                kfl::Log( DEBUG ) << "inputMean=" << inputMean.transpose();

                arp_math::Covariance3 inputCov = arp_math::Covariance3::Identity();
                inputCov(0,0) = sigmaTransOdoVelocity*sigmaTransOdoVelocity;
                inputCov(1,1) = sigmaTransOdoVelocity*sigmaTransOdoVelocity;
                inputCov(2,2) = sigmaRotOdoVelocity*sigmaRotOdoVelocity;
                kfl::Log( DEBUG ) << "inputCov=\n" << inputCov;

                EstimatedTwist2D T_hky_table_p_table_r_hky = MathFactory::createEstimatedTwist2DFromCartesianRepr(inputMean, time, inputCov);
                EstimatedTwist2D T_hky_table_p_hky_r_hky = T_hky_table_p_table_r_hky.changeProjection( obj.getLastEstimatedPose2D() * H_hky_robot );
                EstimatedTwist2D T_odo_table_p_odo_r_odo = T_hky_table_p_hky_r_hky.transport( H_hky_robot.inverse() * H_odo_robot );

                kfl::Log( DEBUG ) << "obj.getLastEstimatedPose2D() : " << obj.getLastEstimatedPose2D().toString();
                kfl::Log( DEBUG ) << "T_hky_table_p_table_r_hky : " << T_hky_table_p_table_r_hky.toString();
                kfl::Log( DEBUG ) << "T_hky_table_p_hky_r_hky : " << T_hky_table_p_hky_r_hky.toString();
                kfl::Log( DEBUG ) << "------------------------------";
                kfl::Log( DEBUG ) << "T_odo_table_p_odo_r_odo :";
                kfl::Log( DEBUG ) << "  vx=" << T_odo_table_p_odo_r_odo.vx() << " m/s";
                kfl::Log( DEBUG ) << "  vy=" << T_odo_table_p_odo_r_odo.vy() << " m/s";
                kfl::Log( DEBUG ) << "  vh=" << rad2deg(T_odo_table_p_odo_r_odo.vh()) << " deg/s";

                kfl::Log( DEBUG ) << "[time=" << time << "]: newOdoVelocity #" << predictionsinceLastUpdate ;
                BOOST_CHECK(obj.newOdoVelocity(T_odo_table_p_odo_r_odo));
                predictionsinceLastUpdate++;

                arp_math::EstimatedPose2D odoEstim_H_robot_table;
                odoEstim_H_robot_table = obj.getLastEstimatedPose2D();
                arp_math::EstimatedPose2D odoEstim_H_hky_table = odoEstim_H_robot_table * H_hky_robot;

                vjson::JsonDocument docOdoEstim;
                std::stringstream odoEstimFileName;
                odoEstimFileName << p + "/ressource/unittest/KFL/KFLocalizator/static_" << xpName << "/t_" << time << "_odo_estimate.json";
                BOOST_CHECK( docOdoEstim.parse(odoEstimFileName.str().c_str()) );
                BOOST_CHECK_CLOSE( time , docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "t")), 1.f );

                double groundTrueT = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "t"));
                double groundTrueX = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "x"));
                double groundTrueY = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "y"));
                double groundTrueH = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "h"));
                BOOST_CHECK_CLOSE( odoEstim_H_hky_table.date() , groundTrueT, 1.f);
                BOOST_CHECK_SMALL( odoEstim_H_hky_table.x()    - groundTrueX, transPrecision);
                BOOST_CHECK_SMALL( odoEstim_H_hky_table.y()    - groundTrueY, transPrecision);
                BOOST_CHECK_SMALL( odoEstim_H_hky_table.h()    - groundTrueH, rotPrecision);
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
                            BOOST_CHECK_SMALL( odoEstim_H_hky_table.cov()(i,j) - cov_i_j, covPrecisionDiag);
                        }
                        else
                        {
                            BOOST_CHECK_SMALL( odoEstim_H_hky_table.cov()(i,j) - cov_i_j, covPrecisionNonDiag);
                        }
                    }
                }

                BOOST_CHECK( abs(odoEstim_H_hky_table.x() - trueX) < 3.0 * sqrt(odoEstim_H_hky_table.cov()(0,0)) );
                BOOST_CHECK( abs(odoEstim_H_hky_table.y() - trueY) < 3.0 * sqrt(odoEstim_H_hky_table.cov()(1,1)) );
                BOOST_CHECK( abs(betweenMinusPiAndPlusPi( odoEstim_H_hky_table.h() - trueH )) < 3.0 * sqrt(odoEstim_H_hky_table.cov()(2,2)) );
            }
            arp_math::EstimatedPose2D postOdoEstim_H_robot_table;
            postOdoEstim_H_robot_table = obj.getLastEstimatedPose2D();
            arp_math::EstimatedPose2D postOdoEstim_H_hky_table = postOdoEstim_H_robot_table * H_hky_robot;


            kfl::Log( DEBUG ) << "=======================";
            kfl::Log( DEBUG ) << "erreur statique apres les odos [time=" << time << "]:";
            kfl::Log( DEBUG ) << "  sur x (en mm): " << (postOdoEstim_H_hky_table.x() - trueX) * 1000. << " +/- " << 3000. * sqrt(postOdoEstim_H_hky_table.cov()(0,0));
            kfl::Log( DEBUG ) << "  sur y (en mm): " << (postOdoEstim_H_hky_table.y() - trueY) * 1000. << " +/- " << 3000. * sqrt(postOdoEstim_H_hky_table.cov()(1,1));
            kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( postOdoEstim_H_hky_table.h() - trueH ) ) << 3.0 * rad2deg(sqrt(postOdoEstim_H_hky_table.cov()(2,2)));
            kfl::Log( DEBUG ) << "covariance : " << postOdoEstim_H_hky_table.cov().row(0);
            kfl::Log( DEBUG ) << "             " << postOdoEstim_H_hky_table.cov().row(1);
            kfl::Log( DEBUG ) << "             " << postOdoEstim_H_hky_table.cov().row(2);


            arp_rlu::lsl::JsonScanParser scanParser;
            std::stringstream scanFileName;
            scanFileName << p + "/ressource/unittest/KFL/KFLocalizator/static_" << xpName << "/t_" << time << "_scan.json";
            BOOST_CHECK( scanParser.parse(scanFileName.str().c_str()) );
            lsl::LaserScan scan;
            BOOST_CHECK( scanParser.getScan(scan) );

            kfl::Log( DEBUG ) << "[time=" << time << "]: newScan";
            obj.newScan(scan);
            predictionsinceLastUpdate = 0;

            arp_math::EstimatedPose2D postScanEstim_H_robot_table;
            postScanEstim_H_robot_table = obj.getLastEstimatedPose2D();
            arp_math::EstimatedPose2D postScanEstim_H_hky_table = postScanEstim_H_robot_table * H_hky_robot;

            vjson::JsonDocument docScanEstim;
            std::stringstream scanEstimFileName;
            scanEstimFileName << p + "/ressource/unittest/KFL/KFLocalizator/static_" << xpName << "/t_" << time << "_scan_estimate.json";
            BOOST_CHECK( docScanEstim.parse(scanEstimFileName.str().c_str()) );
            BOOST_CHECK_CLOSE( time , docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "t")), 1.f );

            double groundTrueT = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "t"));
            double groundTrueX = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "x"));
            double groundTrueY = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "y"));
            double groundTrueH = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "h"));
            BOOST_CHECK_CLOSE( postScanEstim_H_hky_table.date() , groundTrueT, 1.f);
            //            BOOST_CHECK_SMALL( postScanEstim.x() - groundTrueX, transPrecision);
            //            BOOST_CHECK_SMALL( postScanEstim.y() - groundTrueY, transPrecision);
            //            BOOST_CHECK_SMALL( postScanEstim.h() - groundTrueH, rotPrecision);
            BOOST_CHECK_SMALL( postScanEstim_H_hky_table.x() - trueX, transPrecision);
            BOOST_CHECK_SMALL( postScanEstim_H_hky_table.y() - trueY, transPrecision);
            BOOST_CHECK_SMALL( postScanEstim_H_hky_table.h() - trueH, rotPrecision);
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
                        BOOST_CHECK_SMALL( postScanEstim_H_hky_table.cov()(i,j) - cov_i_j, covPrecisionDiag);
                    }
                    else
                    {
                        BOOST_CHECK_SMALL( postScanEstim_H_hky_table.cov()(i,j) - cov_i_j, covPrecisionNonDiag);
                    }
                }
            }

            BOOST_CHECK( abs(postScanEstim_H_hky_table.x() - trueX) < 3.0 * sqrt(postScanEstim_H_hky_table.cov()(0,0)) );
            BOOST_CHECK( abs(postScanEstim_H_hky_table.y() - trueY) < 3.0 * sqrt(postScanEstim_H_hky_table.cov()(1,1)) );
            BOOST_CHECK( abs(betweenMinusPiAndPlusPi( postScanEstim_H_hky_table.h() - trueH )) < 3.0 * sqrt(postScanEstim_H_hky_table.cov()(2,2)) );

            kfl::Log( DEBUG ) << "=======================";
            kfl::Log( DEBUG ) << "erreur statique apres le scan [time=" << time << "]:";
            kfl::Log( DEBUG ) << "  sur x (en mm): " << (postScanEstim_H_hky_table.x() - trueX) * 1000. << " +/- " << 3000. * sqrt(postScanEstim_H_hky_table.cov()(0,0));
            kfl::Log( DEBUG ) << "  sur y (en mm): " << (postScanEstim_H_hky_table.y() - trueY) * 1000. << " +/- " << 3000. * sqrt(postScanEstim_H_hky_table.cov()(1,1));
            kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( postScanEstim_H_hky_table.h() - trueH ) ) << " +/- " << 3.0 * rad2deg(sqrt(postScanEstim_H_hky_table.cov()(2,2)));
            kfl::Log( DEBUG ) << "covariance : " << postScanEstim_H_hky_table.cov().row(0);
            kfl::Log( DEBUG ) << "             " << postScanEstim_H_hky_table.cov().row(1);
            kfl::Log( DEBUG ) << "             " << postScanEstim_H_hky_table.cov().row(2);
        }

        arp_math::EstimatedPose2D lastEstim_H_robot_table;
        lastEstim_H_robot_table = obj.getLastEstimatedPose2D();
        arp_math::EstimatedPose2D lastEstim_H_hky_table = lastEstim_H_robot_table * H_hky_robot;

        BOOST_CHECK_SMALL( lastEstim_H_hky_table.x() - trueX , transFinalPrecision );
        BOOST_CHECK_SMALL( lastEstim_H_hky_table.y() - trueY , transFinalPrecision );
        BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(lastEstim_H_hky_table.h() - trueH) , rotFinalPrecision );
    }
}

BOOST_AUTO_TEST_CASE( test_1 )
{
    unittest_KFLocalizator_Static::doTest(1);
}

BOOST_AUTO_TEST_CASE( test_2 )
{
    unittest_KFLocalizator_Static::doTest(2);
}

BOOST_AUTO_TEST_CASE( test_3 )
{
    unittest_KFLocalizator_Static::doTest(3);
}

BOOST_AUTO_TEST_CASE( test_4 )
{
    unittest_KFLocalizator_Static::doTest(4);
}

BOOST_AUTO_TEST_CASE( test_5 )
{
    unittest_KFLocalizator_Static::doTest(5);
}

BOOST_AUTO_TEST_CASE( test_6 )
{
    unittest_KFLocalizator_Static::doTest(6);
}

BOOST_AUTO_TEST_CASE( test_8 )
{
    unittest_KFLocalizator_Static::doTest(8);
}

BOOST_AUTO_TEST_CASE( test_9 )
{
    unittest_KFLocalizator_Static::doTest(9);
}

BOOST_AUTO_TEST_CASE( test_10 )
{
    unittest_KFLocalizator_Static::doTest(10);
}

BOOST_AUTO_TEST_SUITE_END()
