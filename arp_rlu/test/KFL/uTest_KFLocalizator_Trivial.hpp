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
    //const double covPrecision = 9.e-3;


    //*******************************************
    //********       Construction       *********
    //*******************************************

    kfl::KFLocalizator obj;

    double vx = 0.;
    double vy = 0.;
    double vh = 0.;

    double sigmaX = 0.001;
    double sigmaY = 0.001;
    double sigmaH = 0.01;

    float initialX =  1.3;
    float initialY = -0.8;
    float initialH = -M_PI/2.;

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

    arp_math::Pose2D H_hky_robot(0.1, 0.2, M_PI);
    arp_math::Pose2D H_odo_robot(-0.3, 0.5, M_PI/2.);

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

    EstimatedPose2D init_H_hky_table;
    init_H_hky_table.x(initialX);
    init_H_hky_table.y(initialY);
    init_H_hky_table.h(initialH);
    Eigen::Matrix3d cov = Eigen::Vector3d(sigmaInitialPosition*sigmaInitialPosition,
            sigmaInitialPosition*sigmaInitialPosition,
            sigmaInitialHeading*sigmaInitialHeading ).asDiagonal();
    init_H_hky_table.cov(cov);
    init_H_hky_table.date(0.0);

    EstimatedPose2D init_H_robot_table = init_H_hky_table * H_hky_robot.inverse();

    kfl::Log( DEBUG ) << "init_H_hky_table : " << init_H_hky_table.toString();
    kfl::Log( DEBUG ) << "     covariance  :\n" << init_H_hky_table.cov();
    kfl::Log( DEBUG ) << "init_H_robot_table : " << init_H_robot_table.toString();
    kfl::Log( DEBUG ) << "     covariance  :\n" << init_H_robot_table.cov();
    BOOST_CHECK(obj.initialize( init_H_robot_table ));


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


    BOOST_CHECK_CLOSE( initEstim_H_hky_table.date() , 0.0, 1.f);
    BOOST_CHECK_SMALL( initEstim_H_hky_table.x()    - initialX, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim_H_hky_table.y()    - initialY, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim_H_hky_table.h()    - initialH, rotPrecisionAgainstGroundTruth);
    //    for(unsigned int i = 0 ; i < 3 ; i++)
    //    {
    //        for(unsigned int j = 0 ; j < 3 ; j++)
    //        {
    //            BOOST_CHECK_SMALL( initEstim_H_hky_table.cov()(i,j) - init_H_hky_table.cov()(i,j), covPrecision);
    //        }
    //    }
    // on retire le test car si on applique plusieurs changements de repère sur un EstimatedPose2D, on dégrade sa covariance.



    //*******************************************
    //********        Loop              *********
    //*******************************************

    const double duration = 0.601;
    const double odoPeriodInSec = 0.01;

    for(double time = odoPeriodInSec ; time < duration ; time = time + odoPeriodInSec)
    {
        kfl::Log( DEBUG ) << "============================================================";
        kfl::Log( DEBUG ) << "Time=" << time << " => ODO";
        kfl::Log( DEBUG ) << "------------------------------";

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

        kfl::Log( DEBUG ) << "obj.getLastEstimatedPose2D() : " << obj.getLastEstimatedPose2D().toString();
        kfl::Log( DEBUG ) << "T_hky_table_p_table_r_hky : " << T_hky_table_p_table_r_hky.toString();
        kfl::Log( DEBUG ) << "T_hky_table_p_hky_r_hky : " << T_hky_table_p_hky_r_hky.toString();
        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "T_odo_table_p_odo_r_odo :";
        kfl::Log( DEBUG ) << "  vx=" << T_odo_table_p_odo_r_odo.vx() << " m/s";
        kfl::Log( DEBUG ) << "  vy=" << T_odo_table_p_odo_r_odo.vy() << " m/s";
        kfl::Log( DEBUG ) << "  vh=" << rad2deg(T_odo_table_p_odo_r_odo.vh()) << " deg/s";

        BOOST_CHECK(obj.newOdoVelocity(T_odo_table_p_odo_r_odo));

        arp_math::EstimatedPose2D odoEstim_H_robot_table = obj.getLastEstimatedPose2D();
        arp_math::EstimatedPose2D odoEstim_H_hky_table = odoEstim_H_robot_table * H_hky_robot;

        BOOST_CHECK_CLOSE( odoEstim_H_hky_table.date() , time, 1.f);
        BOOST_CHECK_SMALL( odoEstim_H_hky_table.x() - initialX, transPrecisionAgainstGroundTruth);
        BOOST_CHECK_SMALL( odoEstim_H_hky_table.y() - initialY, transPrecisionAgainstGroundTruth);
        BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(odoEstim_H_hky_table.h() - initialH), rotPrecisionAgainstGroundTruth);

        //        BOOST_CHECK_SMALL( sqrt(odoEstim_H_hky_table.cov()(0,0)) - (sigmaInitialPosition + time * sigmaX), covPrecision);
        //        BOOST_CHECK_SMALL( sqrt(odoEstim_H_hky_table.cov()(1,1)) - (sigmaInitialPosition + time * sigmaY), covPrecision);
        //        BOOST_CHECK_SMALL( sqrt(odoEstim_H_hky_table.cov()(2,2)) - (sigmaInitialHeading + time * sigmaH), covPrecision);
        // on retire le test car si on applique plusieurs changements de repère sur un EstimatedPose2D, on dégrade sa covariance.

        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "position apres les odos [time=" << time << "]:";
        kfl::Log( DEBUG ) << "  sur x (en m): " << odoEstim_H_hky_table.x();
        kfl::Log( DEBUG ) << "  sur y (en m): " << odoEstim_H_hky_table.y();
        kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( odoEstim_H_hky_table.h() ) );
        kfl::Log( DEBUG ) << "covariance : " << odoEstim_H_hky_table.cov().row(0);
        kfl::Log( DEBUG ) << "             " << odoEstim_H_hky_table.cov().row(1);
        kfl::Log( DEBUG ) << "             " << odoEstim_H_hky_table.cov().row(2);
    }
}

BOOST_AUTO_TEST_CASE( test_Bad_InitTime )
{
    std::string p = ros::package::getPath("arp_rlu");

    const double transPrecisionAgainstGroundTruth = 0.001;
    const double rotPrecisionAgainstGroundTruth = deg2rad(0.1);
//    const double covPrecision = 9.e-3;


    //*******************************************
    //********       Construction       *********
    //*******************************************

    kfl::KFLocalizator obj;

    double vx =  1.;
    double vy = -2.;
    double vh =  0.1;

    float initialX =  1.3;
    float initialY = -0.8;
    float initialH = -M_PI/2.;

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

    arp_math::Pose2D H_hky_robot(0.1, 0.2, M_PI);
    arp_math::Pose2D H_odo_robot(-0.3, 0.5, M_PI/2.);

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

    EstimatedPose2D init_H_hky_table;
    init_H_hky_table.x(initialX);
    init_H_hky_table.y(initialY);
    init_H_hky_table.h(initialH);
    Eigen::Matrix3d cov = Eigen::Vector3d(sigmaInitialPosition*sigmaInitialPosition,
            sigmaInitialPosition*sigmaInitialPosition,
            sigmaInitialHeading*sigmaInitialHeading ).asDiagonal();
    init_H_hky_table.cov(cov);
    init_H_hky_table.date(0.0);

    EstimatedPose2D init_H_robot_table = init_H_hky_table * H_hky_robot.inverse();

    kfl::Log( DEBUG ) << "init_H_hky_table : " << init_H_hky_table.toString();
    kfl::Log( DEBUG ) << "     covariance  :\n" << init_H_hky_table.cov();
    kfl::Log( DEBUG ) << "init_H_robot_table : " << init_H_robot_table.toString();
    kfl::Log( DEBUG ) << "     covariance  :\n" << init_H_robot_table.cov();
    BOOST_CHECK(obj.initialize( init_H_robot_table ));


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


    BOOST_CHECK_CLOSE( initEstim_H_hky_table.date() , 0.0, 1.f);
    BOOST_CHECK_SMALL( initEstim_H_hky_table.x()    - initialX, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim_H_hky_table.y()    - initialY, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim_H_hky_table.h()    - initialH, rotPrecisionAgainstGroundTruth);
    //    for(unsigned int i = 0 ; i < 3 ; i++)
    //    {
    //        for(unsigned int j = 0 ; j < 3 ; j++)
    //        {
    //            BOOST_CHECK_SMALL( initEstim_H_hky_table.cov()(i,j) - init_H_hky_table.cov()(i,j), covPrecision);
    //        }
    //    }
    // on retire le test car si on applique plusieurs changements de repère sur un EstimatedPose2D, on dégrade sa covariance.



    //*******************************************
    //********        Loop              *********
    //*******************************************

    const double duration = 0.601;
    const double odoPeriodInSec = 0.01;

    for(double time = 1.5 ; time < duration ; time = time + odoPeriodInSec)
    {
        arp_math::Vector3 inputMean;
        inputMean << vx, vy, vh;

        arp_math::Covariance3 inputCov = arp_math::Covariance3::Identity();
        inputCov(0,0) = sigmaX*sigmaX;
        inputCov(1,1) = sigmaY*sigmaY;
        inputCov(2,2) = sigmaH*sigmaH;

        EstimatedTwist2D T_hky_table_p_table_r_hky = MathFactory::createEstimatedTwist2DFromCartesianRepr(inputMean, time, inputCov);
        EstimatedTwist2D T_hky_table_p_hky_r_hky = T_hky_table_p_table_r_hky.changeProjection( obj.getLastEstimatedPose2D() * H_hky_robot );
        EstimatedTwist2D T_odo_table_p_odo_r_odo = T_hky_table_p_hky_r_hky.transport( H_hky_robot.inverse() * H_odo_robot );

        BOOST_CHECK(!obj.newOdoVelocity(T_odo_table_p_odo_r_odo));
    }
}

BOOST_AUTO_TEST_CASE( test_V_constante )
{
    std::string p = ros::package::getPath("arp_rlu");

    const double transPrecisionAgainstGroundTruth = 1.e-8;
    const double rotPrecisionAgainstGroundTruth = deg2rad(1.e-6);
//    const double covPrecision = 9.e-6;


    //*******************************************
    //********       Construction       *********
    //*******************************************

    kfl::KFLocalizator obj;

    double vx = -2.;
    double vy =  1.;
    double vh =  deg2rad(100.);

    float initialX =  1.3;
    float initialY = -0.8;
    float initialH = -M_PI/2.;

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

    arp_math::Pose2D H_hky_robot(0.1, 0.2, M_PI);
    arp_math::Pose2D H_odo_robot(-0.3, 0.5, M_PI/2.);

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

    EstimatedPose2D init_H_hky_table;
    init_H_hky_table.x(initialX);
    init_H_hky_table.y(initialY);
    init_H_hky_table.h(initialH);
    Eigen::Matrix3d cov = Eigen::Vector3d(sigmaInitialPosition*sigmaInitialPosition,
            sigmaInitialPosition*sigmaInitialPosition,
            sigmaInitialHeading*sigmaInitialHeading ).asDiagonal();
    init_H_hky_table.cov(cov);
    init_H_hky_table.date(0.0);

    EstimatedPose2D init_H_robot_table = init_H_hky_table * H_hky_robot.inverse();

    kfl::Log( DEBUG ) << "init_H_hky_table : " << init_H_hky_table.toString();
    kfl::Log( DEBUG ) << "     covariance  :\n" << init_H_hky_table.cov();
    kfl::Log( DEBUG ) << "init_H_robot_table : " << init_H_robot_table.toString();
    kfl::Log( DEBUG ) << "     covariance  :\n" << init_H_robot_table.cov();
    BOOST_CHECK(obj.initialize( init_H_robot_table ));


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


    BOOST_CHECK_CLOSE( initEstim_H_hky_table.date() , 0.0, 1.f);
    BOOST_CHECK_SMALL( initEstim_H_hky_table.x()    - initialX, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim_H_hky_table.y()    - initialY, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim_H_hky_table.h()    - initialH, rotPrecisionAgainstGroundTruth);
    //    for(unsigned int i = 0 ; i < 3 ; i++)
    //    {
    //        for(unsigned int j = 0 ; j < 3 ; j++)
    //        {
    //            BOOST_CHECK_SMALL( initEstim_H_hky_table.cov()(i,j) - init_H_hky_table.cov()(i,j), covPrecision);
    //        }
    //    }
    // on retire le test car si on applique plusieurs changements de repère sur un EstimatedPose2D, on dégrade sa covariance.



    //*******************************************
    //********        Loop              *********
    //*******************************************

    const double duration = 0.601;
    const double odoPeriodInSec = 0.01;

    for(double time = odoPeriodInSec ; time < duration ; time = time + odoPeriodInSec)
    {
        kfl::Log( DEBUG ) << "============================================================";
        kfl::Log( DEBUG ) << "Time=" << time << " => ODO";
        kfl::Log( DEBUG ) << "------------------------------";

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

        kfl::Log( DEBUG ) << "obj.getLastEstimatedPose2D() : " << obj.getLastEstimatedPose2D().toString();
        kfl::Log( DEBUG ) << "T_hky_table_p_table_r_hky : " << T_hky_table_p_table_r_hky.toString();
        kfl::Log( DEBUG ) << "T_hky_table_p_hky_r_hky : " << T_hky_table_p_hky_r_hky.toString();
        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "T_odo_table_p_odo_r_odo :";
        kfl::Log( DEBUG ) << "  vx=" << T_odo_table_p_odo_r_odo.vx() << " m/s";
        kfl::Log( DEBUG ) << "  vy=" << T_odo_table_p_odo_r_odo.vy() << " m/s";
        kfl::Log( DEBUG ) << "  vh=" << rad2deg(T_odo_table_p_odo_r_odo.vh()) << " deg/s";

        BOOST_CHECK(obj.newOdoVelocity(T_odo_table_p_odo_r_odo));

        arp_math::EstimatedPose2D odoEstim_H_robot_table = obj.getLastEstimatedPose2D();
        arp_math::EstimatedPose2D odoEstim_H_hky_table = odoEstim_H_robot_table * H_hky_robot;

        BOOST_CHECK_CLOSE( odoEstim_H_hky_table.date() , time, 1.f);
        BOOST_CHECK_SMALL( odoEstim_H_hky_table.x() - (initialX + time * vx), transPrecisionAgainstGroundTruth);
        BOOST_CHECK_SMALL( odoEstim_H_hky_table.y() - (initialY + time * vy), transPrecisionAgainstGroundTruth);
        BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(odoEstim_H_hky_table.h() - (initialH + time * vh)), rotPrecisionAgainstGroundTruth);

        //        BOOST_CHECK_SMALL( sqrt(odoEstim_H_hky_table.cov()(0,0)) - (sigmaInitialPosition + time * sigmaX), covPrecision);
        //        BOOST_CHECK_SMALL( sqrt(odoEstim_H_hky_table.cov()(1,1)) - (sigmaInitialPosition + time * sigmaY), covPrecision);
        //        BOOST_CHECK_SMALL( sqrt(odoEstim_H_hky_table.cov()(2,2)) - (sigmaInitialHeading + time * sigmaH), covPrecision);
        // on retire le test car si on applique plusieurs changements de repère sur un EstimatedPose2D, on dégrade sa covariance.

        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "position apres les odos [time=" << time << "]:";
        kfl::Log( DEBUG ) << "  sur x (en m): " << odoEstim_H_hky_table.x();
        kfl::Log( DEBUG ) << "  sur y (en m): " << odoEstim_H_hky_table.y();
        kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( odoEstim_H_hky_table.h() ) );
        kfl::Log( DEBUG ) << "covariance : " << odoEstim_H_hky_table.cov().row(0);
        kfl::Log( DEBUG ) << "             " << odoEstim_H_hky_table.cov().row(1);
        kfl::Log( DEBUG ) << "             " << odoEstim_H_hky_table.cov().row(2);
    }
}


BOOST_AUTO_TEST_CASE( test_acc )
{
    std::string p = ros::package::getPath("arp_rlu");

    const double transPrecisionAgainstGroundTruth = 0.001;
    const double rotPrecisionAgainstGroundTruth = deg2rad(0.1);
//    const double covPrecision = 9.e-3;


    //*******************************************
    //********       Construction       *********
    //*******************************************

    kfl::KFLocalizator obj;

    float initialX =  1.3;
    float initialY = -0.8;
    float initialH = -M_PI/2.;

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

    arp_math::Pose2D H_hky_robot(0.1, 0.2, M_PI);
    arp_math::Pose2D H_odo_robot(-0.3, 0.5, M_PI/2.);

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

    EstimatedPose2D init_H_hky_table;
    init_H_hky_table.x(initialX);
    init_H_hky_table.y(initialY);
    init_H_hky_table.h(initialH);
    Eigen::Matrix3d cov = Eigen::Vector3d(sigmaInitialPosition*sigmaInitialPosition,
            sigmaInitialPosition*sigmaInitialPosition,
            sigmaInitialHeading*sigmaInitialHeading ).asDiagonal();
    init_H_hky_table.cov(cov);
    init_H_hky_table.date(0.0);

    EstimatedPose2D init_H_robot_table = init_H_hky_table * H_hky_robot.inverse();

    kfl::Log( DEBUG ) << "init_H_hky_table : " << init_H_hky_table.toString();
    kfl::Log( DEBUG ) << "     covariance  :\n" << init_H_hky_table.cov();
    kfl::Log( DEBUG ) << "init_H_robot_table : " << init_H_robot_table.toString();
    kfl::Log( DEBUG ) << "     covariance  :\n" << init_H_robot_table.cov();
    BOOST_CHECK(obj.initialize( init_H_robot_table ));


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


    BOOST_CHECK_CLOSE( initEstim_H_hky_table.date() , 0.0, 1.f);
    BOOST_CHECK_SMALL( initEstim_H_hky_table.x()    - initialX, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim_H_hky_table.y()    - initialY, transPrecisionAgainstGroundTruth);
    BOOST_CHECK_SMALL( initEstim_H_hky_table.h()    - initialH, rotPrecisionAgainstGroundTruth);
    //    for(unsigned int i = 0 ; i < 3 ; i++)
    //    {
    //        for(unsigned int j = 0 ; j < 3 ; j++)
    //        {
    //            BOOST_CHECK_SMALL( initEstim_H_hky_table.cov()(i,j) - init_H_hky_table.cov()(i,j), covPrecision);
    //        }
    //    }
    // on retire le test car si on applique plusieurs changements de repère sur un EstimatedPose2D, on dégrade sa covariance.


    //*******************************************
    //********        Loop              *********
    //*******************************************

    const double duration = 0.101;
    const double odoPeriodInSec = 0.01;

    for(double time = odoPeriodInSec ; time < duration ; time = time + odoPeriodInSec)
    {
        kfl::Log( DEBUG ) << "============================================================";
        kfl::Log( DEBUG ) << "Time=" << time << " => ODO";
        kfl::Log( DEBUG ) << "------------------------------";

        arp_math::Vector3 inputMean;
        inputMean << vx + time*ax, vy + time*ay, vh + time*ah;
        kfl::Log( DEBUG ) << "inputMean=" << inputMean.transpose();

        arp_math::Covariance3 inputCov = arp_math::Covariance3::Identity();
        inputCov(0,0) = sigmaX*sigmaX;
        inputCov(1,1) = sigmaY*sigmaY;
        inputCov(2,2) = sigmaH*sigmaH;
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

        BOOST_CHECK(obj.newOdoVelocity(T_odo_table_p_odo_r_odo));

        arp_math::EstimatedPose2D odoEstim_H_robot_table = obj.getLastEstimatedPose2D();
        arp_math::EstimatedPose2D odoEstim_H_hky_table = odoEstim_H_robot_table * H_hky_robot;

        BOOST_CHECK_CLOSE( odoEstim_H_hky_table.date() , time, 1.f);
        BOOST_CHECK_SMALL( odoEstim_H_hky_table.x() - (initialX + time * (vx + time*ax)), transPrecisionAgainstGroundTruth);
        BOOST_CHECK_SMALL( odoEstim_H_hky_table.y() - (initialY + time * (vy + time*ay)), transPrecisionAgainstGroundTruth);
        BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(odoEstim_H_hky_table.h() - (initialH + time * (vh + time*ah))), rotPrecisionAgainstGroundTruth);

        //        BOOST_CHECK_SMALL( sqrt(odoEstim_H_hky_table.cov()(0,0)) - (sigmaInitialPosition + time * sigmaX), covPrecision);
        //        BOOST_CHECK_SMALL( sqrt(odoEstim_H_hky_table.cov()(1,1)) - (sigmaInitialPosition + time * sigmaY), covPrecision);
        //        BOOST_CHECK_SMALL( sqrt(odoEstim_H_hky_table.cov()(2,2)) - (sigmaInitialHeading + time * sigmaH), covPrecision);
        // on retire le test car si on applique plusieurs changements de repère sur un EstimatedPose2D, on dégrade sa covariance.

        kfl::Log( DEBUG ) << "------------------------------";
        kfl::Log( DEBUG ) << "position apres les odos [time=" << time << "]:";
        kfl::Log( DEBUG ) << "  sur x (en m): " << odoEstim_H_hky_table.x();
        kfl::Log( DEBUG ) << "  sur y (en m): " << odoEstim_H_hky_table.y();
        kfl::Log( DEBUG ) << "  en cap (deg) : " << rad2deg( betweenMinusPiAndPlusPi( odoEstim_H_hky_table.h() ) );
        kfl::Log( DEBUG ) << "covariance : " << odoEstim_H_hky_table.cov().row(0);
        kfl::Log( DEBUG ) << "             " << odoEstim_H_hky_table.cov().row(1);
        kfl::Log( DEBUG ) << "             " << odoEstim_H_hky_table.cov().row(2);
    }
}


BOOST_AUTO_TEST_SUITE_END()
