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

BOOST_AUTO_TEST_SUITE( unittest_KFLocalizator_Trivial )

BOOST_AUTO_TEST_CASE( test_Static )
{
    std::string p = ros::package::getPath("arp_rlu");

    const double transPrecisionAgainstGroundTruth = 0.001;
    const double rotPrecisionAgainstGroundTruth = deg2rad(0.1);
    const double covPrecision = 9.e-3;


    //*******************************************
    //********       Construction       *********
    //*******************************************

    kfl::KFLocalizator obj;

    double sigmaX = 0.001;
    double sigmaY = 0.001;
    double sigmaH = 0.01;

    //*******************************************
    //********        Set params        *********
    //*******************************************


    vjson::JsonDocument docParams;
    std::string kflParamsFileName = p + "/ressource/unittest/KFL/KFLocalizator/static_1/kfl_params.json";
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

    obj.setParams(kfParams);


    //*******************************************
    //********        Initialize        *********
    //*******************************************

    vjson::JsonDocument docInit;
    std::string initialPositionFileName = p + "/ressource/unittest/KFL/KFLocalizator/static_1/initial_position.json";
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
    initialPose.date(0.0);

    BOOST_CHECK(obj.initialize(initialPose));


    arp_math::EstimatedPose2D initEstim;
    initEstim = obj.getLastEstimatedPose2D();

    kfl::Log( DEBUG ) << "erreur statique avant les odos [time=0.0]:";
    kfl::Log( DEBUG ) << "  sur x (en mm): " << (initEstim.x() - trueX) * 1000.;
    kfl::Log( DEBUG ) << "  sur y (en mm): " << (initEstim.y() - trueY) * 1000.;
    kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( initEstim.h() - trueH ) );
    kfl::Log( DEBUG ) << "covariance : " << initEstim.cov().row(0);
    kfl::Log( DEBUG ) << "             " << initEstim.cov().row(1);
    kfl::Log( DEBUG ) << "             " << initEstim.cov().row(2);


    BOOST_CHECK_CLOSE( initEstim.date() , 0.0, 1.f);
    BOOST_CHECK_SMALL( initEstim.x()    - initialXPosition, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim.y()    - initialYPosition, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim.h()    - initialHPosition, rotPrecisionAgainstGroundTruth);
    for(unsigned int i = 0 ; i < 3 ; i++)
    {
        std::stringstream rowName;
        rowName << "row_" << i;
        for(unsigned int j = 0 ; j < 3 ; j++)
        {
            BOOST_CHECK_SMALL( initEstim.cov()(i,j) - initialPose.cov()(i,j), covPrecision);
        }
    }



    //*******************************************
    //********        Loop              *********
    //*******************************************

    const double duration = 0.601;
    const double odoPeriodInSec = 0.01;

    for(double time = odoPeriodInSec ; time < duration ; time = time + odoPeriodInSec)
    {
        kfl::Log( DEBUG ) << "============================================================";
        kfl::Log( DEBUG ) << "Time=" << time << " => ODO";

        arp_math::EstimatedTwist2D odoVel;
        odoVel.date( time );
        odoVel.vx( 0. );
        odoVel.vy( 0. );
        odoVel.vh( 0. );
        odoVel.cov() = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 3,3> covariance = Eigen::Matrix<double, 3,3>::Identity();
        covariance(0,0) = sigmaX*sigmaX;
        covariance(1,1) = sigmaY*sigmaY;
        covariance(2,2) = sigmaH*sigmaH;
        odoVel.cov( covariance );

        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "vx=" << odoVel.vx() << " m/s";
        kfl::Log( DEBUG ) << "vy=" << odoVel.vy() << " m/s";
        kfl::Log( DEBUG ) << "vh=" << deg2rad(odoVel.vh()) << " deg/s";

        BOOST_CHECK(obj.newOdoVelocity(odoVel));

        arp_math::EstimatedPose2D odoEstim;
        odoEstim = obj.getLastEstimatedPose2D();

        BOOST_CHECK_CLOSE( odoEstim.date() , time, 1.f);
        BOOST_CHECK_SMALL( odoEstim.x()    - initialXPosition, transPrecisionAgainstGroundTruth);
        BOOST_CHECK_SMALL( odoEstim.y()    - initialYPosition, transPrecisionAgainstGroundTruth);
        BOOST_CHECK_SMALL( odoEstim.h()    - initialHPosition, rotPrecisionAgainstGroundTruth);

        BOOST_CHECK_SMALL( sqrt(odoEstim.cov()(0,0)) - (sqrt(initialPose.cov()(0,0)) + time * sigmaX ), covPrecision);
        BOOST_CHECK_SMALL( sqrt(odoEstim.cov()(1,1)) - (sqrt(initialPose.cov()(1,1)) + time * sigmaY ), covPrecision);
        BOOST_CHECK_SMALL( sqrt(odoEstim.cov()(2,2)) - (sqrt(initialPose.cov()(2,2)) + time * sigmaH ), covPrecision);

        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "position apres les odos [time=" << time << "]:";
        kfl::Log( DEBUG ) << "  sur x (en m): " << odoEstim.x();
        kfl::Log( DEBUG ) << "  sur y (en m): " << odoEstim.y();
        kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( odoEstim.h() ) );
        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "erreur apres les odos [time=" << time << "]:";
        kfl::Log( DEBUG ) << "  sur x (en mm): " << (odoEstim.x() - trueX) * 1000.;
        kfl::Log( DEBUG ) << "  sur y (en mm): " << (odoEstim.y() - trueY) * 1000.;
        kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( odoEstim.h() - trueH ) );
        kfl::Log( DEBUG ) << "covariance : " << odoEstim.cov().row(0);
        kfl::Log( DEBUG ) << "             " << odoEstim.cov().row(1);
        kfl::Log( DEBUG ) << "             " << odoEstim.cov().row(2);
    }
}

BOOST_AUTO_TEST_CASE( test_Bad_InitTime )
{
    std::string p = ros::package::getPath("arp_rlu");

    const double transPrecisionAgainstGroundTruth = 0.001;
    const double rotPrecisionAgainstGroundTruth = deg2rad(0.1);
    const double covPrecision = 9.e-3;


    //*******************************************
    //********       Construction       *********
    //*******************************************

    kfl::KFLocalizator obj;

    double vx =  1.;
    double vy = -2.;
    double vh =  0.1;

    double sigmaX = 0.001;
    double sigmaY = 0.001;
    double sigmaH = 0.01;

    //*******************************************
    //********        Set params        *********
    //*******************************************


    vjson::JsonDocument docParams;
    std::string kflParamsFileName = p + "/ressource/unittest/KFL/KFLocalizator/static_1/kfl_params.json";
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

    obj.setParams(kfParams);


    //*******************************************
    //********        Initialize        *********
    //*******************************************

    vjson::JsonDocument docInit;
    std::string initialPositionFileName = p + "/ressource/unittest/KFL/KFLocalizator/static_1/initial_position.json";
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
    initialPose.date(0.0);

    BOOST_CHECK(obj.initialize(initialPose));


    arp_math::EstimatedPose2D initEstim;
    initEstim = obj.getLastEstimatedPose2D();

    kfl::Log( DEBUG ) << "erreur statique avant les odos [time=0.0]:";
    kfl::Log( DEBUG ) << "  sur x (en mm): " << (initEstim.x() - trueX) * 1000.;
    kfl::Log( DEBUG ) << "  sur y (en mm): " << (initEstim.y() - trueY) * 1000.;
    kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( initEstim.h() - trueH ) );
    kfl::Log( DEBUG ) << "covariance : " << initEstim.cov().row(0);
    kfl::Log( DEBUG ) << "             " << initEstim.cov().row(1);
    kfl::Log( DEBUG ) << "             " << initEstim.cov().row(2);


    BOOST_CHECK_CLOSE( initEstim.date() , 0.0, 1.f);
    BOOST_CHECK_SMALL( initEstim.x()    - initialXPosition, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim.y()    - initialYPosition, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim.h()    - initialHPosition, rotPrecisionAgainstGroundTruth);
    for(unsigned int i = 0 ; i < 3 ; i++)
    {
        std::stringstream rowName;
        rowName << "row_" << i;
        for(unsigned int j = 0 ; j < 3 ; j++)
        {
            BOOST_CHECK_SMALL( initEstim.cov()(i,j) - initialPose.cov()(i,j), covPrecision);
        }
    }



    //*******************************************
    //********        Loop              *********
    //*******************************************

    const double duration = 0.601;
    const double odoPeriodInSec = 0.01;

    for(double time = 1.5 ; time < duration ; time = time + odoPeriodInSec)
    {
        arp_math::EstimatedTwist2D odoVel;
        odoVel.date( time );
        odoVel.vx( vx );
        odoVel.vy( vy );
        odoVel.vh( vh );
        odoVel.cov() = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 3,3> covariance = Eigen::Matrix<double, 3,3>::Identity();
        covariance(0,0) = sigmaX*sigmaX;
        covariance(1,1) = sigmaY*sigmaY;
        covariance(2,2) = sigmaH*sigmaH;
        odoVel.cov( covariance );

        BOOST_CHECK(!obj.newOdoVelocity(odoVel));
    }
}

BOOST_AUTO_TEST_CASE( test_V_constante )
{
    std::string p = ros::package::getPath("arp_rlu");

    const double transPrecisionAgainstGroundTruth = 0.001;
    const double rotPrecisionAgainstGroundTruth = deg2rad(0.1);
    const double covPrecision = 9.e-3;


    //*******************************************
    //********       Construction       *********
    //*******************************************

    kfl::KFLocalizator obj;

    double vx =  1.;
    double vy = -2.;
    double vh =  0.1;

    double sigmaX = 0.001;
    double sigmaY = 0.001;
    double sigmaH = 0.01;

    //*******************************************
    //********        Set params        *********
    //*******************************************


    vjson::JsonDocument docParams;
    std::string kflParamsFileName = p + "/ressource/unittest/KFL/KFLocalizator/static_1/kfl_params.json";
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

    obj.setParams(kfParams);


    //*******************************************
    //********        Initialize        *********
    //*******************************************

    vjson::JsonDocument docInit;
    std::string initialPositionFileName = p + "/ressource/unittest/KFL/KFLocalizator/static_1/initial_position.json";
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
    initialPose.date(0.0);

    BOOST_CHECK(obj.initialize(initialPose));


    arp_math::EstimatedPose2D initEstim;
    initEstim = obj.getLastEstimatedPose2D();

    kfl::Log( DEBUG ) << "erreur statique avant les odos [time=0.0]:";
    kfl::Log( DEBUG ) << "  sur x (en mm): " << (initEstim.x() - trueX) * 1000.;
    kfl::Log( DEBUG ) << "  sur y (en mm): " << (initEstim.y() - trueY) * 1000.;
    kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( initEstim.h() - trueH ) );
    kfl::Log( DEBUG ) << "covariance : " << initEstim.cov().row(0);
    kfl::Log( DEBUG ) << "             " << initEstim.cov().row(1);
    kfl::Log( DEBUG ) << "             " << initEstim.cov().row(2);


    BOOST_CHECK_CLOSE( initEstim.date() , 0.0, 1.f);
    BOOST_CHECK_SMALL( initEstim.x()    - initialXPosition, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim.y()    - initialYPosition, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim.h()    - initialHPosition, rotPrecisionAgainstGroundTruth);
    for(unsigned int i = 0 ; i < 3 ; i++)
    {
        std::stringstream rowName;
        rowName << "row_" << i;
        for(unsigned int j = 0 ; j < 3 ; j++)
        {
            BOOST_CHECK_SMALL( initEstim.cov()(i,j) - initialPose.cov()(i,j), covPrecision);
        }
    }



    //*******************************************
    //********        Loop              *********
    //*******************************************

    const double duration = 0.601;
    const double odoPeriodInSec = 0.01;

    for(double time = odoPeriodInSec ; time < duration ; time = time + odoPeriodInSec)
    {
        kfl::Log( DEBUG ) << "============================================================";
        kfl::Log( DEBUG ) << "Time=" << time << " => ODO";

        arp_math::EstimatedTwist2D odoVel;
        odoVel.date( time );
        odoVel.vx( vx );
        odoVel.vy( vy );
        odoVel.vh( vh );
        odoVel.cov() = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 3,3> covariance = Eigen::Matrix<double, 3,3>::Identity();
        covariance(0,0) = sigmaX*sigmaX;
        covariance(1,1) = sigmaY*sigmaY;
        covariance(2,2) = sigmaH*sigmaH;
        odoVel.cov( covariance );

        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "vx=" << odoVel.vx() << " m/s";
        kfl::Log( DEBUG ) << "vy=" << odoVel.vy() << " m/s";
        kfl::Log( DEBUG ) << "vh=" << deg2rad(odoVel.vh()) << " deg/s";

        BOOST_CHECK(obj.newOdoVelocity(odoVel));

        arp_math::EstimatedPose2D odoEstim;
        odoEstim = obj.getLastEstimatedPose2D();

        BOOST_CHECK_CLOSE( odoEstim.date() , time, 1.f);
        BOOST_CHECK_SMALL( odoEstim.x() - (initialXPosition + time * vx), transPrecisionAgainstGroundTruth);
        BOOST_CHECK_SMALL( odoEstim.y() - (initialYPosition + time * vy), transPrecisionAgainstGroundTruth);
        BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(odoEstim.h()) - betweenMinusPiAndPlusPi(initialHPosition + time * vh), rotPrecisionAgainstGroundTruth);

        BOOST_CHECK_SMALL( sqrt(odoEstim.cov()(0,0)) - (sqrt(initialPose.cov()(0,0)) + time * sigmaX), covPrecision);
        BOOST_CHECK_SMALL( sqrt(odoEstim.cov()(1,1)) - (sqrt(initialPose.cov()(1,1)) + time * sigmaY), covPrecision);
        BOOST_CHECK_SMALL( sqrt(odoEstim.cov()(2,2)) - (sqrt(initialPose.cov()(2,2)) + time * sigmaH), covPrecision);

        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "position apres les odos [time=" << time << "]:";
        kfl::Log( DEBUG ) << "  sur x (en m): " << odoEstim.x();
        kfl::Log( DEBUG ) << "  sur y (en m): " << odoEstim.y();
        kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( odoEstim.h() ) );
        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "erreur apres les odos [time=" << time << "]:";
        kfl::Log( DEBUG ) << "  sur x (en mm): " << (odoEstim.x() - trueX) * 1000.;
        kfl::Log( DEBUG ) << "  sur y (en mm): " << (odoEstim.y() - trueY) * 1000.;
        kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( odoEstim.h() - trueH ) );
        kfl::Log( DEBUG ) << "covariance : " << odoEstim.cov().row(0);
        kfl::Log( DEBUG ) << "             " << odoEstim.cov().row(1);
        kfl::Log( DEBUG ) << "             " << odoEstim.cov().row(2);
    }
}


BOOST_AUTO_TEST_CASE( test_acc )
{
    std::string p = ros::package::getPath("arp_rlu");

    const double transPrecisionAgainstGroundTruth = 0.001;
    const double rotPrecisionAgainstGroundTruth = deg2rad(0.1);
    const double covPrecision = 9.e-3;


    //*******************************************
    //********       Construction       *********
    //*******************************************

    kfl::KFLocalizator obj;

    double vx =  1.;
    double vy = -2.;
    double vh =  0.1;

    double ax =  0.1; // m/s^2
    double ay =  0.; // m/s^2
    double ah =  0.; // rad/s^2

    double sigmaX = 0.001;
    double sigmaY = 0.001;
    double sigmaH = 0.01;

    //*******************************************
    //********        Set params        *********
    //*******************************************


    vjson::JsonDocument docParams;
    std::string kflParamsFileName = p + "/ressource/unittest/KFL/KFLocalizator/static_1/kfl_params.json";
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

    obj.setParams(kfParams);


    //*******************************************
    //********        Initialize        *********
    //*******************************************

    vjson::JsonDocument docInit;
    std::string initialPositionFileName = p + "/ressource/unittest/KFL/KFLocalizator/static_1/initial_position.json";
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
    initialPose.date(0.0);

    BOOST_CHECK(obj.initialize(initialPose));


    arp_math::EstimatedPose2D initEstim;
    initEstim = obj.getLastEstimatedPose2D();

    kfl::Log( DEBUG ) << "erreur statique avant les odos [time=0.0]:";
    kfl::Log( DEBUG ) << "  sur x (en mm): " << (initEstim.x() - trueX) * 1000.;
    kfl::Log( DEBUG ) << "  sur y (en mm): " << (initEstim.y() - trueY) * 1000.;
    kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( initEstim.h() - trueH ) );
    kfl::Log( DEBUG ) << "covariance : " << initEstim.cov().row(0);
    kfl::Log( DEBUG ) << "             " << initEstim.cov().row(1);
    kfl::Log( DEBUG ) << "             " << initEstim.cov().row(2);


    BOOST_CHECK_CLOSE( initEstim.date() , 0.0, 1.f);
    BOOST_CHECK_SMALL( initEstim.x()    - initialXPosition, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim.y()    - initialYPosition, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim.h()    - initialHPosition, rotPrecisionAgainstGroundTruth);
    for(unsigned int i = 0 ; i < 3 ; i++)
    {
        std::stringstream rowName;
        rowName << "row_" << i;
        for(unsigned int j = 0 ; j < 3 ; j++)
        {
            BOOST_CHECK_SMALL( initEstim.cov()(i,j) - initialPose.cov()(i,j), covPrecision);
        }
    }



    //*******************************************
    //********        Loop              *********
    //*******************************************

    const double duration = 0.101;
    const double odoPeriodInSec = 0.01;

    for(double time = odoPeriodInSec ; time < duration ; time = time + odoPeriodInSec)
    {
        kfl::Log( DEBUG ) << "============================================================";
        kfl::Log( DEBUG ) << "Time=" << time << " => ODO";

        arp_math::EstimatedTwist2D odoVel;
        odoVel.date( time );
        odoVel.vx( vx + time*ax );
        odoVel.vy( vy + time*ay );
        odoVel.vh( vh + time*ah );
        odoVel.cov() = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 3,3> covariance = Eigen::Matrix<double, 3,3>::Identity();
        covariance(0,0) = sigmaX*sigmaX;
        covariance(1,1) = sigmaY*sigmaY;
        covariance(2,2) = sigmaH*sigmaH;
        odoVel.cov( covariance );

        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "vx=" << odoVel.vx() << " m/s";
        kfl::Log( DEBUG ) << "vy=" << odoVel.vy() << " m/s";
        kfl::Log( DEBUG ) << "vh=" << deg2rad(odoVel.vh()) << " deg/s";

        BOOST_CHECK(obj.newOdoVelocity(odoVel));

        arp_math::EstimatedPose2D odoEstim;
        odoEstim = obj.getLastEstimatedPose2D();

        BOOST_CHECK_CLOSE( odoEstim.date() , time, 1.f);
        BOOST_CHECK_SMALL( odoEstim.x() - (initialXPosition + time * (vx + time*ax)), transPrecisionAgainstGroundTruth);
        BOOST_CHECK_SMALL( odoEstim.y() - (initialYPosition + time * (vy + time*ay)), transPrecisionAgainstGroundTruth);
        BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(odoEstim.h()) - betweenMinusPiAndPlusPi(initialHPosition + time * (vh + time*ah)), rotPrecisionAgainstGroundTruth);

        BOOST_CHECK_SMALL( sqrt(odoEstim.cov()(0,0)) - (sqrt(initialPose.cov()(0,0)) + time * sigmaX), covPrecision);
        BOOST_CHECK_SMALL( sqrt(odoEstim.cov()(1,1)) - (sqrt(initialPose.cov()(1,1)) + time * sigmaY), covPrecision);
        BOOST_CHECK_SMALL( sqrt(odoEstim.cov()(2,2)) - (sqrt(initialPose.cov()(2,2)) + time * sigmaH), covPrecision);

        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "position apres les odos [time=" << time << "]:";
        kfl::Log( DEBUG ) << "  sur x (en m): " << odoEstim.x();
        kfl::Log( DEBUG ) << "  sur y (en m): " << odoEstim.y();
        kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( odoEstim.h() ) );
        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "erreur apres les odos [time=" << time << "]:";
        kfl::Log( DEBUG ) << "  sur x (en mm): " << (odoEstim.x() - trueX) * 1000.;
        kfl::Log( DEBUG ) << "  sur y (en mm): " << (odoEstim.y() - trueY) * 1000.;
        kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( odoEstim.h() - trueH ) );
        kfl::Log( DEBUG ) << "covariance : " << odoEstim.cov().row(0);
        kfl::Log( DEBUG ) << "             " << odoEstim.cov().row(1);
        kfl::Log( DEBUG ) << "             " << odoEstim.cov().row(2);
    }
}


BOOST_AUTO_TEST_SUITE_END()
