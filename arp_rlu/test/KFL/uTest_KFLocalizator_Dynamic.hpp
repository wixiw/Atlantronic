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
        std::string p = ros::package::getPath("arp_rlu");

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
        std::string kflParamsFileName = p + "/ressource/unittest/KFL/KFLocalizator/dynamic_" + xpName + "/kfl_params.json";
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
        procParams.mfp.width = 0;
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

        kfl::Log( INFO ) << "================================";
        kfl::Log( INFO ) << "=== KFLOCALIZATOR PARAMETERS ===";
        kfl::Log( INFO ) << kfParams.getInfo();


        //*******************************************
        //********        Initialize        *********
        //*******************************************

        vjson::JsonDocument docInit;
        std::string initialPositionFileName = p + "/ressource/unittest/KFL/KFLocalizator/dynamic_" + xpName + "/initial_position.json";
        BOOST_CHECK( docInit.parse(initialPositionFileName.c_str() ) );
        float estimInitialXPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "estimInitialXPosition") );
        float estimInitialYPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "estimInitialYPosition") );
        float estimInitialHPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "estimInitialHPosition") );
        float trueInitialXPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "trueInitialXPosition") );
        float trueInitialYPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "trueInitialYPosition") );
        float trueInitialHPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "trueInitialHPosition") );

        EstimatedPose2D initialPose_H_hky_table;
        initialPose_H_hky_table.x(estimInitialXPosition);
        initialPose_H_hky_table.y(estimInitialYPosition);
        initialPose_H_hky_table.h(estimInitialHPosition);
        Eigen::Matrix3d cov = Eigen::Vector3d(sigmaInitialPosition*sigmaInitialPosition,
                sigmaInitialPosition*sigmaInitialPosition,
                sigmaInitialHeading*sigmaInitialHeading ).asDiagonal();
        initialPose_H_hky_table.cov(cov);
        initialPose_H_hky_table.date(0.0);

        kfl::Log( INFO ) << "estimée initiale (H_hky_table) demandée :";
        kfl::Log( INFO ) << "  sur x (en m): " << initialPose_H_hky_table.x();
        kfl::Log( INFO ) << "  sur y (en m): " << initialPose_H_hky_table.y();
        kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( initialPose_H_hky_table.h() ));
        kfl::Log( INFO ) << "  covariance : " << initialPose_H_hky_table.cov().row(0);
        kfl::Log( INFO ) << "               " << initialPose_H_hky_table.cov().row(1);
        kfl::Log( INFO ) << "               " << initialPose_H_hky_table.cov().row(2);

        EstimatedPose2D initialPose_H_robot_table = initialPose_H_hky_table * H_hky_robot.inverse();

        kfl::Log( INFO ) << "estimée initiale (H_robot_table) demandée :";
        kfl::Log( INFO ) << "  sur x (en m): " << initialPose_H_robot_table.x();
        kfl::Log( INFO ) << "  sur y (en m): " << initialPose_H_robot_table.y();
        kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( initialPose_H_robot_table.h() ));
        kfl::Log( INFO ) << "  covariance : " << initialPose_H_robot_table.cov().row(0);
        kfl::Log( INFO ) << "               " << initialPose_H_robot_table.cov().row(1);
        kfl::Log( INFO ) << "               " << initialPose_H_robot_table.cov().row(2);

        BOOST_CHECK(obj.initialize(initialPose_H_robot_table));


        arp_math::EstimatedPose2D initEstim_H_robot_table;
        initEstim_H_robot_table = obj.getLastEstimatedPose2D();
        arp_math::EstimatedPose2D initEstim_H_hky_table = initEstim_H_robot_table * H_hky_robot;


        kfl::Log( DEBUG ) << "position réelle (H_hky_table) au départ [time=0.0]:";
        kfl::Log( DEBUG ) << "  sur x (en m): " << trueInitialXPosition;
        kfl::Log( DEBUG ) << "  sur y (en m): " << trueInitialYPosition;
        kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( trueInitialHPosition ));
        kfl::Log( INFO ) << "------------------------------";
        kfl::Log( INFO ) << "estimée initiale (H_hky_table) au départ [time=0.0]:";
        kfl::Log( INFO ) << "  sur x (en m): " << initEstim_H_hky_table.x();
        kfl::Log( INFO ) << "  sur y (en m): " << initEstim_H_hky_table.y();
        kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( initEstim_H_hky_table.h() ));
        kfl::Log( INFO ) << "  covariance : " << initEstim_H_hky_table.cov().row(0);
        kfl::Log( INFO ) << "               " << initEstim_H_hky_table.cov().row(1);
        kfl::Log( INFO ) << "               " << initEstim_H_hky_table.cov().row(2);
        kfl::Log( INFO ) << "------------------------------";
        kfl::Log( INFO ) << "erreur initiale (sur H_hky_table) [time=0.0]:";
        kfl::Log( INFO ) << "  sur x (en mm): " << (initEstim_H_hky_table.x() - trueInitialXPosition) * 1000. << " +/- " << 2000. * sqrt(initEstim_H_hky_table.cov()(0,0));
        kfl::Log( INFO ) << "  sur y (en mm): " << (initEstim_H_hky_table.y() - trueInitialYPosition) * 1000. << " +/- " << 2000. * sqrt(initEstim_H_hky_table.cov()(1,1));
        kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( initEstim_H_hky_table.h() - trueInitialHPosition ) ) << 2.0 * rad2deg(sqrt(initEstim_H_hky_table.cov()(2,2)));


        //*******************************************
        //********       Load Timeline      *********
        //*******************************************
        vjson::JsonDocument docTimeline;
        std::string timelineFileName = p + "/ressource/unittest/KFL/KFLocalizator/dynamic_" + xpName + "/time_line.json";
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
                truePosFileName << p + "/ressource/unittest/KFL/KFLocalizator/dynamic_" << xpName << "/t_" << std::setprecision(6) << time << "_true_position.json";
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
                odoVelFileName << p + "/ressource/unittest/KFL/KFLocalizator/dynamic_" << xpName << "/t_" << time << "_odo_velocity.json";
                BOOST_CHECK( docOdoVel.parse(odoVelFileName.str().c_str()) );
                BOOST_CHECK_CLOSE( time , docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "t") ), 1.f);

                double vx = docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vx") );
                double vy = docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vy") );
                double vh = docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vh") );
                double sigmaX = docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "sigmaX") );
                double sigmaY = docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "sigmaY") );
                double sigmaH = docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "sigmaH") );


                arp_math::Vector3 inputMean;
                inputMean << vx, vy, vh;
                kfl::Log( DEBUG ) << "inputMean=" << inputMean.transpose();

                arp_math::Covariance3 inputCov = arp_math::Covariance3::Identity();
                inputCov(0,0) = sigmaX*sigmaX;
                inputCov(1,1) = sigmaY*sigmaY;
                inputCov(2,2) = sigmaH*sigmaH;
                kfl::Log( DEBUG ) << "inputCov=\n" << inputCov;

                EstimatedTwist2D T_hky_table_p_table_r_hky = MathFactory::createEstimatedTwist2DFromCartesianRepr(inputMean, time, inputCov);
                EstimatedTwist2D T_hky_table_p_hky_r_hky = T_hky_table_p_table_r_hky.changeProjection( obj.getLastEstimatedPose2D() * H_hky_robot );
                EstimatedTwist2D T_odo_table_p_odo_r_odo = T_hky_table_p_hky_r_hky.transport( H_hky_robot.inverse() * H_odo_robot );

                kfl::Log( DEBUG ) << "T_hky_table_p_table_r_hky : " << T_hky_table_p_table_r_hky.toString();
                kfl::Log( INFO ) << "   covariance : " << T_hky_table_p_table_r_hky.cov().row(0);
                kfl::Log( INFO ) << "                " << T_hky_table_p_table_r_hky.cov().row(1);
                kfl::Log( INFO ) << "                " << T_hky_table_p_table_r_hky.cov().row(2);
                kfl::Log( DEBUG ) << "T_hky_table_p_hky_r_hky : " << T_hky_table_p_hky_r_hky.toString();
                kfl::Log( INFO ) << "   covariance : " << T_hky_table_p_hky_r_hky.cov().row(0);
                kfl::Log( INFO ) << "                " << T_hky_table_p_hky_r_hky.cov().row(1);
                kfl::Log( INFO ) << "                " << T_hky_table_p_hky_r_hky.cov().row(2);
                kfl::Log( DEBUG ) << "------------------------------";
                kfl::Log( DEBUG ) << "T_odo_table_p_odo_r_odo :";
                kfl::Log( DEBUG ) << "  vx=" << T_odo_table_p_odo_r_odo.vx() << " m/s";
                kfl::Log( DEBUG ) << "  vy=" << T_odo_table_p_odo_r_odo.vy() << " m/s";
                kfl::Log( DEBUG ) << "  vh=" << rad2deg(T_odo_table_p_odo_r_odo.vh()) << " deg/s";
                kfl::Log( INFO ) << "   covariance : " << T_odo_table_p_odo_r_odo.cov().row(0);
                kfl::Log( INFO ) << "                " << T_odo_table_p_odo_r_odo.cov().row(1);
                kfl::Log( INFO ) << "                " << T_odo_table_p_odo_r_odo.cov().row(2);

                double startOdoTime = arp_math::getTime();
                BOOST_CHECK(obj.newOdoVelocity(T_odo_table_p_odo_r_odo));
                double endOdoTime = arp_math::getTime();
                kfl::Log( INFO ) << "Odo Computation Time :" << endOdoTime - startOdoTime;


                arp_math::EstimatedPose2D odoEstim_H_robot_table;
                odoEstim_H_robot_table = obj.getLastEstimatedPose2D();
                arp_math::EstimatedPose2D odoEstim_H_hky_table = odoEstim_H_robot_table * H_hky_robot;

                vjson::JsonDocument docOdoEstim;
                std::stringstream odoEstimFileName;
                odoEstimFileName << p + "/ressource/unittest/KFL/KFLocalizator/dynamic_" << xpName << "/t_" << time << "_odo_estimate.json";
                BOOST_CHECK( docOdoEstim.parse(odoEstimFileName.str().c_str()) );
                BOOST_CHECK_CLOSE( time , docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "t")), 1.f );

                double pyOdoEstimT = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "t"));
                double pyOdoEstimX = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "x"));
                double pyOdoEstimY = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "y"));
                double pyOdoEstimH = docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "h"));
                BOOST_CHECK_CLOSE( odoEstim_H_hky_table.date() , pyOdoEstimT, 1.f);
                BOOST_CHECK_SMALL( odoEstim_H_hky_table.x()    - pyOdoEstimX, transPrecisionAgainstPy);
                BOOST_CHECK_SMALL( odoEstim_H_hky_table.y()    - pyOdoEstimY, transPrecisionAgainstPy);
                BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(odoEstim_H_hky_table.h()- pyOdoEstimH), rotPrecisionAgainstPy);

                BOOST_CHECK_CLOSE( odoEstim_H_hky_table.date() , time, 1.f);
                //            BOOST_CHECK_SMALL( odoEstim_H_hky_table.x()    - trueX, transPrecisionAgainstGroundTruth);
                //            BOOST_CHECK_SMALL( odoEstim_H_hky_table.y()    - trueY, transPrecisionAgainstGroundTruth);
                //            BOOST_CHECK_SMALL( odoEstim_H_hky_table.h()    - trueH, rotPrecisionAgainstGroundTruth);
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

                kfl::Log( INFO ) << "------------------------------";
                kfl::Log( INFO ) << "estimée apres les odos (H_hky_table) [time=" << time << "]:";
                kfl::Log( INFO ) << "  sur x (en m): " << odoEstim_H_hky_table.x();
                kfl::Log( INFO ) << "  sur y (en m): " << odoEstim_H_hky_table.y();
                kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( odoEstim_H_hky_table.h() ) );
                kfl::Log( INFO ) << "------------------------------";
                kfl::Log( INFO ) << "erreur apres les odos  (sur H_hky_table) [time=" << time << "]:";
                kfl::Log( INFO ) << "  sur x (en mm): " << (odoEstim_H_hky_table.x() - trueX) * 1000. << " +/- " << 3000. * sqrt(odoEstim_H_hky_table.cov()(0,0));
                kfl::Log( INFO ) << "  sur y (en mm): " << (odoEstim_H_hky_table.y() - trueY) * 1000. << " +/- " << 3000. * sqrt(odoEstim_H_hky_table.cov()(1,1));
                kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( odoEstim_H_hky_table.h() - trueH ) ) << " +/- " << 3.0 * rad2deg(sqrt(odoEstim_H_hky_table.cov()(2,2)));
                kfl::Log( INFO ) << "covariance : " << odoEstim_H_hky_table.cov().row(0);
                kfl::Log( INFO ) << "             " << odoEstim_H_hky_table.cov().row(1);
                kfl::Log( INFO ) << "             " << odoEstim_H_hky_table.cov().row(2);
            }


            if( doLrf )
            {
                kfl::Log( DEBUG ) << "============================================================";
                kfl::Log( DEBUG ) << "Time=" << time << " => LRF";
                kfl::Log( DEBUG ) << "------------------------------";

                arp_rlu::lsl::JsonScanParser scanParser;
                std::stringstream scanFileName;
                scanFileName << p + "/ressource/unittest/KFL/KFLocalizator/dynamic_" << xpName << "/t_" << time << "_scan.json";
                BOOST_CHECK( scanParser.parse(scanFileName.str().c_str()) );
                lsl::LaserScan scan;
                BOOST_CHECK( scanParser.getScan(scan) );

                double startScanTime = arp_math::getTime();
                obj.newScan(scan);
                double endScanTime = arp_math::getTime();
                kfl::Log( INFO ) << "Scan Computation Time :" << endScanTime - startScanTime;

                kfl::Log( DEBUG ) << "------------------------------";
                arp_math::EstimatedPose2D postScanEstim_H_robot_table;
                postScanEstim_H_robot_table = obj.getLastEstimatedPose2D();
                arp_math::EstimatedPose2D postScanEstim_H_hky_table = postScanEstim_H_robot_table * H_hky_robot;

                vjson::JsonDocument docScanEstim;
                std::stringstream scanEstimFileName;
                scanEstimFileName << p + "/ressource/unittest/KFL/KFLocalizator/dynamic_" << xpName << "/t_" << time << "_scan_estimate.json";
                BOOST_CHECK( docScanEstim.parse(scanEstimFileName.str().c_str()) );
                BOOST_CHECK_CLOSE( time , docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "t")), 1.f );

                double pyScanEstimT = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "t"));
                double pyScanEstimX = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "x"));
                double pyScanEstimY = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "y"));
                double pyScanEstimH = docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "h"));
                BOOST_CHECK_CLOSE( postScanEstim_H_hky_table.date() , pyScanEstimT, 1.f);
                BOOST_CHECK_SMALL( postScanEstim_H_hky_table.x() - pyScanEstimX, transPrecisionAgainstPy);
                BOOST_CHECK_SMALL( postScanEstim_H_hky_table.y() - pyScanEstimY, transPrecisionAgainstPy);
                BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(postScanEstim_H_hky_table.h() - pyScanEstimH), rotPrecisionAgainstPy);

                BOOST_CHECK_CLOSE( postScanEstim_H_hky_table.date() , time, 1.f);
                BOOST_CHECK_SMALL( postScanEstim_H_hky_table.x() - trueX, transPrecisionAgainstGroundTruth);
                BOOST_CHECK_SMALL( postScanEstim_H_hky_table.y() - trueY, transPrecisionAgainstGroundTruth);
                BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(postScanEstim_H_hky_table.h() - trueH), rotPrecisionAgainstGroundTruth);
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
                            BOOST_CHECK_SMALL( postScanEstim_H_hky_table.cov()(i,j) - cov_i_j, covPrecisionDiag);
                        }
                        else
                        {
                            BOOST_CHECK_SMALL( postScanEstim_H_hky_table.cov()(i,j) - cov_i_j, covPrecisionNonDiag);
                        }
                    }
                }

                BOOST_CHECK( abs(postScanEstim_H_hky_table.x() - trueX) < 3.5 * sqrt(postScanEstim_H_hky_table.cov()(0,0)) );
                BOOST_CHECK( abs(postScanEstim_H_hky_table.y() - trueY) < 3.5 * sqrt(postScanEstim_H_hky_table.cov()(1,1)) );
                BOOST_CHECK( abs(betweenMinusPiAndPlusPi( postScanEstim_H_hky_table.h() - trueH )) < 3.5 * sqrt(postScanEstim_H_hky_table.cov()(2,2)) );

                //                kfl::Log( INFO ) << "------------------------------";
                //                kfl::Log( INFO ) << "position apres le scan [time=" << time << "]:";
                //                kfl::Log( INFO ) << "  sur x (en m): " << postScanEstim.x();
                //                kfl::Log( INFO ) << "  sur y (en m): " << postScanEstim.y();
                //                kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( postScanEstim.h() ) );
                kfl::Log( INFO ) << "------------------------------";
                kfl::Log( INFO ) << "erreur apres le scan [time=" << time << "]:";
                kfl::Log( INFO ) << "  sur x (en mm): " << (postScanEstim_H_hky_table.x() - trueX) * 1000. << " +/- " << 3500. * sqrt(postScanEstim_H_hky_table.cov()(0,0));
                kfl::Log( INFO ) << "  sur y (en mm): " << (postScanEstim_H_hky_table.y() - trueY) * 1000. << " +/- " << 3500. * sqrt(postScanEstim_H_hky_table.cov()(1,1));
                kfl::Log( INFO ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( postScanEstim_H_hky_table.h() - trueH ) )  << " +/- " << 3.5 * rad2deg(sqrt(postScanEstim_H_hky_table.cov()(2,2)));
                kfl::Log( INFO ) << "covariance : " << postScanEstim_H_hky_table.cov().row(0);
                kfl::Log( INFO ) << "             " << postScanEstim_H_hky_table.cov().row(1);
                kfl::Log( INFO ) << "             " << postScanEstim_H_hky_table.cov().row(2);
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
