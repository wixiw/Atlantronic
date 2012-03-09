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
using namespace Eigen;
using namespace arp_rlu;
using namespace std;
using namespace kfl;
using namespace arp_core::log;

BOOST_AUTO_TEST_SUITE( unittest_KFLocalizator )

BOOST_AUTO_TEST_CASE( Static_1 )
{
    //*******************************************
    //********       Construction       *********
    //*******************************************

    kfl::KFLocalizator obj;



    //*******************************************
    //********        Set params        *********
    //*******************************************

    kfl::KFLocalizator::InitParams  initParams;
    kfl::KFLocalizator::IEKFParams  iekfParams;
    kfl::BeaconDetector::Params     procParams;

    vjson::JsonDocument docParams;
    BOOST_CHECK( docParams.parse("../ressource/unittest/KFL/KFLocalizator/static_1/kfl_params.json") );
    float sigmaInitialPosition  = docParams.getFloatData( docParams.getChild( docParams.root(), "sigmaInitialPosition") );
    float sigmaInitialHeading   = docParams.getFloatData( docParams.getChild( docParams.root(), "sigmaInitialHeading") );
    float sigmaTransOdoVelocity = docParams.getFloatData( docParams.getChild( docParams.root(), "sigmaTransOdoVelocity") );
    float sigmaRotOdoVelocity   = docParams.getFloatData( docParams.getChild( docParams.root(), "sigmaRotOdoVelocity") );
    float sigmaLaserRange       = docParams.getFloatData( docParams.getChild( docParams.root(), "sigmaLaserRange") );
    float sigmaLaserAngle       = docParams.getFloatData( docParams.getChild( docParams.root(), "sigmaLaserAngle") );
    int iekf_Nit                = docParams.getIntegerData( docParams.getChild( docParams.root(), "iekf_Nit") );
    //    float iekf_xThreshold       = docParams.getFloatData( docParams.getChild( docParams.root(), "iekf_xThreshold") );
    //    float iekf_yThreshold       = docParams.getFloatData( docParams.getChild( docParams.root(), "iekf_yThreshold") );
    //    float iekf_hThreshold       = docParams.getFloatData( docParams.getChild( docParams.root(), "iekf_hThreshold") );
    float maxDistance           = docParams.getFloatData( docParams.getChild( docParams.root(), "beacondetector_maxDistance") );
    float rangeThreshold        = docParams.getFloatData( docParams.getChild( docParams.root(), "beacondetector_rangeThreshold") );


    iekfParams.defaultOdoVelTransSigma = sigmaTransOdoVelocity;
    iekfParams.defaultOdoVelRotSigma   = sigmaRotOdoVelocity;
    iekfParams.defaultLaserRangeSigma  = sigmaLaserRange;
    iekfParams.defaultLaserThetaSigma  = sigmaLaserAngle;
    iekfParams.iekfMaxIt               = iekf_Nit;

    procParams.maxDistance2NearestReferencedBeacon = maxDistance;
    procParams.psp.rangeThres = rangeThreshold;

    obj.setParams(iekfParams);
    obj.setParams(procParams);



    //*******************************************
    //********        Initialize        *********
    //*******************************************

    double time = 0.0;

    vjson::JsonDocument docInit;
    BOOST_CHECK( docInit.parse("../ressource/unittest/KFL/KFLocalizator/static_1/initial_position.json") );
    float trueX            = docInit.getFloatData( docInit.getChild( docInit.root(), "trueX") );
    float trueY            = docInit.getFloatData( docInit.getChild( docInit.root(), "trueY") );
    float trueH            = docInit.getFloatData( docInit.getChild( docInit.root(), "trueH") );
    float initialXPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "initialXPosition") );
    float initialYPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "initialYPosition") );
    float initialHPosition = docInit.getFloatData( docInit.getChild( docInit.root(), "initialHPosition") );

    initParams.initialPose.x(initialXPosition);
    initParams.initialPose.y(initialYPosition);
    initParams.initialPose.h(initialHPosition);
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
    cov.diagonal() = Eigen::Vector3d(sigmaInitialPosition, sigmaInitialPosition, sigmaInitialHeading );
    initParams.initialPose.cov(cov);
    initParams.initialPose.date(time);

    obj.setParams(initParams);
//    BOOST_CHECK(obj.initialize());



    //*******************************************
    //********          Start           *********
    //*******************************************

    const unsigned int nbScans = 5;
    const double odoPeriodInSec = 0.01;
    const double scanPeriodInSec = 0.1;

    time += odoPeriodInSec;
    for(unsigned int k = 0 ; k < nbScans ; k++)
    {
        kfl::Log( DEBUG ) << "==============================================";
        kfl::Log( DEBUG ) << " TOUR " << k << "   [time=" << time << " -> " << time + scanPeriodInSec <<"]";

        arp_math::EstimatedPose2D preOdoEstim;
        //        preOdoEstim = obj.getLastEstimatedPose2D();

        kfl::Log( DEBUG ) << "=======================";
        kfl::Log( DEBUG ) << "erreur statique avant les odos [time=" << time << "]:";
        kfl::Log( DEBUG ) << "  sur x (en mm): " << (preOdoEstim.x() - trueX) * 1000.;
        kfl::Log( DEBUG ) << "  sur y (en mm): " << (preOdoEstim.y() - trueY) * 1000.;
        kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( preOdoEstim.h() - trueH ) );
        kfl::Log( DEBUG ) << "covariance : " << preOdoEstim.cov().row(0);
        kfl::Log( DEBUG ) << "             " << preOdoEstim.cov().row(1);
        kfl::Log( DEBUG ) << "             " << preOdoEstim.cov().row(2);

        for(double t = time ; t < time + scanPeriodInSec ; t += odoPeriodInSec )
        {
            vjson::JsonDocument docOdoVel;
            std::stringstream odoVelFileName;
            odoVelFileName << "../ressource/unittest/KFL/KFLocalizator/static_1/t_" << t << "_odo.json";
            BOOST_CHECK( docOdoVel.parse(odoVelFileName.str().c_str()) );
            BOOST_CHECK_CLOSE( t , docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "t") ), 1.f);

            arp_math::EstimatedTwist2D odoVel;
            odoVel.date( t );
            odoVel.vx( docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vx") ) );
            odoVel.vy( docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vy") ) );
            odoVel.vh( docOdoVel.getFloatData( docOdoVel.getChild( docOdoVel.root(), "vh") ) );
            Eigen::Matrix3d odoCov = Eigen::Matrix3d::Identity();
            odoCov.diagonal() = Eigen::Vector3d(sigmaTransOdoVelocity, sigmaTransOdoVelocity, sigmaRotOdoVelocity );
            odoVel.cov( odoCov );

            //            obj.newOdoVelocity(odoVel);

            arp_math::EstimatedPose2D odoEstim;
            //            odoEstim = obj.getLastEstimatedPose2D();

            vjson::JsonDocument docOdoEstim;
            std::stringstream odoEstimFileName;
            odoEstimFileName << "../ressource/unittest/KFL/KFLocalizator/static_1/t_" << t << "_odo_estimate.json";
            BOOST_CHECK( docOdoEstim.parse(odoEstimFileName.str().c_str()) );
            BOOST_CHECK_CLOSE( t , docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "t")), 1.f );

            //            BOOST_CHECK_CLOSE( odoEstim.date() , docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "t")), 1.f);
            //            BOOST_CHECK_CLOSE( odoEstim.x()    , docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "x")), 1.f);
            //            BOOST_CHECK_CLOSE( odoEstim.y()    , docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "y")), 1.f);
            //            BOOST_CHECK_CLOSE( odoEstim.h()    , docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.root(), "h")), 1.f);
            for(unsigned int i = 0 ; i < 3 ; i++)
            {
                for(unsigned int j = 0 ; j < 3 ; j++)
                {
                    std::stringstream rowName;
                    rowName << "row_" << i;
                    //                    BOOST_CHECK_CLOSE( odoEstim.cov()(i,j), docOdoEstim.getFloatData( docOdoEstim.getChild( docOdoEstim.getChild( docOdoEstim.getChild( docOdoEstim.root(), "covariance"), rowName.str()), j )), 1.f);
                }
            }
        }
        time += scanPeriodInSec;

        arp_math::EstimatedPose2D postOdoEstim;
        //        postOdoEstim = obj.getLastEstimatedPose2D();

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
        scanFileName << "../ressource/unittest/KFL/KFLocalizator/static_1/t_" << time << "_scan.json";
        BOOST_CHECK( scanParser.parse(scanFileName.str().c_str()) );
        lsl::LaserScan scan;
        BOOST_CHECK( scanParser.getScan(scan) );

        //        obj.newScan(scan);

        arp_math::EstimatedPose2D postScanEstim;
        //        postScanEstim = obj.getLastEstimatedPose2D();

        vjson::JsonDocument docScanEstim;
        std::stringstream scanEstimFileName;
        scanEstimFileName << "../ressource/unittest/KFL/KFLocalizator/static_1/t_" << time << "_scan_estimate.json";
        BOOST_CHECK( docScanEstim.parse(scanEstimFileName.str().c_str()) );
        BOOST_CHECK_CLOSE( time , docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "t")), 1.f );

        //            BOOST_CHECK_CLOSE( postScanEstim.date() , docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "t")), 1.f);
        //            BOOST_CHECK_CLOSE( postScanEstim.x()    , docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "x")), 1.f);
        //            BOOST_CHECK_CLOSE( postScanEstim.y()    , docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "y")), 1.f);
        //            BOOST_CHECK_CLOSE( postScanEstim.h()    , docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.root(), "h")), 1.f);
        for(unsigned int i = 0 ; i < 3 ; i++)
        {
            for(unsigned int j = 0 ; j < 3 ; j++)
            {
                std::stringstream rowName;
                rowName << "row_" << i;
                //                    BOOST_CHECK_CLOSE( postScanEstim.cov()(i,j), docScanEstim.getFloatData( docScanEstim.getChild( docScanEstim.getChild( docScanEstim.getChild( docScanEstim.root(), "covariance"), rowName.str()), j )), 1.f);
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
    //        lastEstim = obj.getLastEstimatedPose2D();

    //    BOOST_CHECK_SMALL( lastEstim.x() - trueX , 0.01 );
    //    BOOST_CHECK_SMALL( lastEstim.y() - trueY , 0.01 );
    //    BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(lastEstim.h() - trueH) , 0.01 );
}

BOOST_AUTO_TEST_SUITE_END()
