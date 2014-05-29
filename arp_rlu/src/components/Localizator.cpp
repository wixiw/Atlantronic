/*
 * Localizator.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include <iomanip>

#include "Localizator.hpp"
#include <rtt/Component.hpp>
#include "LSL/Logger.hpp"
#include "KFL/Logger.hpp"
#include <ros/ros.h>
#include "ObstacleParams.hpp"

#define stringify( name ) # name

using namespace arp_core::log;
using namespace arp_rlu;
using namespace arp_math;
using namespace arp_time;
using namespace RTT;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Localizator )

Localizator::Localizator(const std::string& name)
: RluTaskContext(name)
, kfloc()
, propMaxReliableBadOdoTransStddev(0.10)
, propMaxReliableBadOdoRotStddev( deg2rad(12.) )
, propMaxReliableLostTransStddev(0.20)
, propMaxReliableLostRotStddev( deg2rad(22.) )
, propLaserRangeSigma(0.01) //, propLaserRangeSigma(0.10)
, propLaserThetaSigma(0.1) //, propLaserThetaSigma(0.10)
, propLaserRangeSigmaSmooth(0.1) //, propLaserRangeSigmaSmooth(1000)
, propLaserThetaSigmaSmooth(1.0) //, propLaserThetaSigmaSmooth(1000)
, propIEKFMaxIt(10)
, propIEKFInnovationMin(0.0122474)
, propCornerBeaconX(1.562)
, propCornerBeaconY(1.062)
, propMiddleBeaconX(1.562)
, propMiddleBeaconY(0.000)
, propBeaconRadius(0.040)
, propTimeReporting(false)
, smoothMode(false)
, predictionOk(false)
, updateOk(false)
, nbSeenBeacons(0)
, currentState(STOPPED)
, currentMode(ODO_ONLY)
, currentQuality(LOST)
, currentVisibility(NONE)
, m_timer(0)
{
    //***WARNING*** Ne pas laisser tourner des logs verbeux sur le robot
    //arp_rlu::lsl::Logger::InitFile("LSL", WARN);
    //arp_rlu::kfl::Logger::InitFile("KFL", WARN);

    createOrocosInterface();

    m_monotonicTimeToRealTime = ros::Time::now().toSec() - getAbsoluteTime();

    propParams.defaultInitCovariance = Vector3(0.020 * 0.020, 0.020 * 0.020, deg2rad(1.) * deg2rad(1.)).asDiagonal();

    propParams.bufferSize = 100;
    propParams.maxTime4OdoPrediction = 0.5;

    propParams.H_hky_robot = Pose2D(-0.051, 0.0003 /*...hum bravo meca ?*/, 0.);
    propParams.H_odo_robot = Pose2D();

    // Red is default config
    propParams.referencedBeacons = std::vector< lsl::Circle >();
    propParams.referencedBeacons.push_back( lsl::Circle(-propMiddleBeaconX, propMiddleBeaconY   , propBeaconRadius ) );
    propParams.referencedBeacons.push_back( lsl::Circle( propMiddleBeaconX,-propCornerBeaconY   , propBeaconRadius ) );
    propParams.referencedBeacons.push_back( lsl::Circle( propMiddleBeaconX, propCornerBeaconY   , propBeaconRadius ) );

    propParams.iekfParams.defaultOdoVelTransSigma = 0.01;
    propParams.iekfParams.defaultOdoVelRotSigma   = 0.01;
    propParams.iekfParams.defaultLaserRangeSigma  = 0.10;
    propParams.iekfParams.defaultLaserThetaSigma  = 0.10;
    propParams.iekfParams.iekfMaxIt               = 100; //10
    propParams.iekfParams.iekfInnovationMin       = 0.001; //0.0122474;

    propParams.procParams.mfp.width = 3;

    propParams.procParams.pcp.minRange = 0.15;
    propParams.procParams.pcp.maxRange = 3.5;
    propParams.procParams.pcp.minTheta = -arp_math::PI;
    propParams.procParams.pcp.maxTheta =  arp_math::PI;

    propParams.procParams.psp.rangeThres = 0.08;

    propParams.procParams.xMin = -1.7;
    propParams.procParams.xMax =  1.7;
    propParams.procParams.yMin = -1.2;
    propParams.procParams.yMax =  1.2;

    propParams.procParams.xMinObstacle = OBSTACLE_X_MIN;
    propParams.procParams.xMaxObstacle = OBSTACLE_X_MAX;
    propParams.procParams.yMinObstacle = OBSTACLE_Y_MIN;
    propParams.procParams.yMaxObstacle = OBSTACLE_Y_MAX;

    propParams.procParams.cip.radius = propBeaconRadius;
    propParams.procParams.cip.coeffs = std::vector<double>();
    propParams.procParams.cip.coeffs.push_back(-0.01743846);
    propParams.procParams.cip.coeffs.push_back( 0.19259734);
    propParams.procParams.cip.coeffs.push_back(-0.83735629);
    propParams.procParams.cip.coeffs.push_back( 1.81203033);
    propParams.procParams.cip.coeffs.push_back(-2.04349845);
    propParams.procParams.cip.coeffs.push_back( 1.17177993);
    propParams.procParams.cip.coeffs.push_back( 0.67248282);
    propParams.procParams.cip.coeffs.push_back( 0.07096937);

    propParams.procParams.tcp.radiusTolerance = 0.03;
    propParams.procParams.tcp.distanceTolerance = 0.6;
    propParams.procParams.tcp.maxLengthTolerance = 0.05;
    propParams.procParams.tcp.medLengthTolerance = 0.05;
    propParams.procParams.tcp.minLengthTolerance = 0.05;

    propParams.procParams.dcp.radiusTolerance = 0.03;
    propParams.procParams.dcp.distanceTolerance = 0.4;
    propParams.procParams.dcp.lengthTolerance = 0.05;

    propParams.procParams.minNbPoints = 3;

    kfloc.setParams(propParams);
}

bool Localizator::configureHook()
{
    if( !RluTaskContext::configureHook() )
        return false;

    return true;
}


void Localizator::updateHook()
{
    if(currentState == STOPPED)
    {
        kfl::Log( Info ) << getInfo();
        return;
    }

    smoothMode = false;
    inSmoothMode.read(smoothMode);
    if(smoothMode && (propParams.iekfParams.defaultLaserRangeSigma !=  propLaserRangeSigmaSmooth||
            propParams.iekfParams.defaultLaserThetaSigma != propLaserThetaSigmaSmooth ) )
    {
        propParams.iekfParams.defaultLaserRangeSigma = propLaserRangeSigmaSmooth;
        propParams.iekfParams.defaultLaserThetaSigma = propLaserThetaSigmaSmooth;
        kfloc.setParams( propParams.iekfParams );
        //LOG(Info) << "Switched to smooth mode (LaserRangeSigma: " << kfloc.getParams().iekfParams.defaultLaserRangeSigma << " - LaserThetaSigma: " << kfloc.getParams().iekfParams.defaultLaserThetaSigma << ")" << endlog();
    }
    if( !smoothMode && (propParams.iekfParams.defaultLaserRangeSigma != propLaserRangeSigma ||
            propParams.iekfParams.defaultLaserThetaSigma != propLaserThetaSigma ) )
    {
        propParams.iekfParams.defaultLaserRangeSigma = propLaserRangeSigma;
        propParams.iekfParams.defaultLaserThetaSigma = propLaserThetaSigma;
        kfloc.setParams( propParams.iekfParams );
        //LOG(Info) << "Switched to normal mode (LaserRangeSigma: " << kfloc.getParams().iekfParams.defaultLaserRangeSigma << " - LaserThetaSigma: " << kfloc.getParams().iekfParams.defaultLaserThetaSigma << ")" << endlog();
    }

    predictionOk = false;

    EstimatedICRSpeed odoSpeed;
    if( RTT::NewData == inOdo.read(odoSpeed) )
    {
        EstimatedTwist2D T_odo_table_p_odo_r_odo = odoSpeed.twist();
        //update du Kalman
        predictionOk = kfloc.newOdoVelocity(T_odo_table_p_odo_r_odo);
    }

    hokuyo_scan rosScan;
    if( RTT::NewData == inScan.read(rosScan) )
    {
        double dateBeg = 0.0; //TODO JB//rosScan.header.stamp.toSec() - m_monotonicTimeToRealTime;
        double HOKUYO_TIME_INCREMENT = 0.0; //TODO JB
        int HOKUYO_MIN_RANGE = 20 ; //TODO JB
        int HOKUYO_MAX_RANGE = 4000 ; //TODO JB
        double HOKUYO_DTHETA = (M_PI / 512.0f);//TODO JB
        double HOKUYO_START_ANGLE = ((- 135 * M_PI / 180.0f) + 44 * HOKUYO_DTHETA);//TODO JB

        lsl::LaserScan lslScan;
        Eigen::MatrixXd polarData(3, HOKUYO_NUM_POINTS);
        for (unsigned int i = 0; i != HOKUYO_NUM_POINTS; i++)
        {
            polarData(0,i) = dateBeg + i * HOKUYO_TIME_INCREMENT;
            polarData(2,i) = HOKUYO_START_ANGLE + i*HOKUYO_TIME_INCREMENT;
            if (rosScan.distance[i] <= HOKUYO_MAX_RANGE && HOKUYO_MIN_RANGE <= rosScan.distance[i])
            {
                polarData(1,i) = rosScan.distance[i]/1000.0;
            }
            else
            {
                polarData(1,i) = 0.;
            }
        }
        lslScan.setPolarData(polarData);

        nbSeenBeacons = kfloc.newScan(lslScan);
        updateOk = (nbSeenBeacons > 1);

        outNbSeenBeacons.write(nbSeenBeacons);
    }

    updateLocalizationStates();
    kfl::Log( INFO ) << getInfo();

    EstimatedPose2D estim_H_robot_table = kfloc.getLastEstimatedPose2D();
    EstimatedTwist2D estim_T_robot_table_p_robot_r_robot = kfloc.getLastEstimatedTwist2D();

    std::vector< arp_math::Vector2 > obstacles = kfloc.getDetectedObstacles();
    outObstacles.write(obstacles);

    ICRSpeed outSpeed(estim_T_robot_table_p_robot_r_robot);

    outPose.write(estim_H_robot_table);
    outICRSpeed.write(outSpeed);
    outLocalizationState.write(currentState);
    outLocalizationMode.write(currentMode);
    outLocalizationQuality.write(currentQuality);
    outLocalizationVisibility.write(currentVisibility);

}

bool Localizator::ooInitialize(double x, double y, double theta)
{

    ArdAbsoluteTime initDate = getAbsoluteTime();
    EstimatedPose2D pose = MathFactory::createEstimatedPose2D(x,y,theta, initDate, propParams.defaultInitCovariance);

    propParams.iekfParams.iekfInnovationMin = propIEKFInnovationMin;
    propParams.iekfParams.iekfMaxIt = propIEKFMaxIt;
    kfloc.setParams( propParams.iekfParams );

    if( kfloc.initialize(pose) )
    {
        EstimatedPose2D estim_H_robot_table = kfloc.getLastEstimatedPose2D();
        EstimatedTwist2D estim_T_robot_table_p_robot_r_robot = kfloc.getLastEstimatedTwist2D();

        outPose.write(estim_H_robot_table);
        outICRSpeed.write(ICRSpeed(estim_T_robot_table_p_robot_r_robot));

        predictionOk = false;
        updateOk = false;
        currentState = RUNNING;
        currentMode = ODO_ONLY;
        currentQuality = GOOD;
        currentVisibility = NONE;

        LOG(Info) << "initialize to " << pose.toString() << " with date : "  << initDate <<  " (sec)" << endlog();
        kfl::Log( INFO ) << getInfo();


        outLocalizationState.write(currentState);
        outLocalizationMode.write(currentMode);
        outLocalizationQuality.write(currentQuality);
        outLocalizationVisibility.write(currentVisibility);

        return true;
    }
    LOG(Info) << "Fail to initialize" << endlog();
    return false;
}


void Localizator::setParams(LocalizatorParams params)
{
    propParams = params;
    kfloc.setParams(propParams);
    LOG(Info) << "New params defined :" << endlog();
    LOG(Info) << kfloc.getParams().getInfo() << endlog();
}


std::string Localizator::printParams()
{
    std::stringstream ss;
    ss << "****************************" << std::endl;
    ss << kfloc.getParams().getInfo();
    ss << " [*] propMaxReliableBadOdoTransStddev : " << propMaxReliableBadOdoTransStddev << " (m)" << std::endl;
    ss << " [*] propMaxReliableBadOdoRotStddev : " << rad2deg(propMaxReliableBadOdoRotStddev) << " (deg)" << std::endl;
    ss << " [*] propMaxReliableLostTransStddev : " << propMaxReliableLostTransStddev << " (m)" << std::endl;
    ss << " [*] propMaxReliableLostRotStddev : " << rad2deg(propMaxReliableLostRotStddev) << " (deg)" << std::endl;
    ss << "****************************" << std::endl;
    ss << "****************************" << std::endl;
    return ss.str();
}

void Localizator::createOrocosInterface()
{
    addProperty("propParams",propParams);


    addPort("inScan",inScan)
    .doc("LaserScan from LRF");

    addPort("inOdo",inOdo)
    .doc("Estimation of T_odo_table_p_odo_r_odo : Twist of odo reference frame relative to table frame, reduced and expressed in odo reference frame.\n It is an EstimatedTwist2D, so it contains Twist, estimation date (in sec) and covariance matrix.");

    addPort("inGyroAngle",inGyroAngle)
    .doc("Angular position from the Gyrometer");

    addPort("inSmoothMode",inSmoothMode)
    .doc("When true, change settings to avoid jumps in estimate");

    addPort("outPose",outPose)
    .doc("Last estimation of H_robot_table.\n It is an EstimatedPose2D, so it contains Pose2D, estimation date (in sec) and covariance matrix.");

    addPort("outICRSpeed",outICRSpeed)
    .doc("Last estimation of T_robot_table_p_robot_r_robot Twist of robot reference frame relative to table frame, reduced and expressed in robot reference frame.\n It is an EstimatedTwist2D, so it contains Twist, estimation date (in sec) and covariance matrix.");

    std::stringstream ssLocState;
    ssLocState << "Localization state :" << std::endl;
    ssLocState << " [*] 0 : STOPPED  if Localization is halt" << std::endl;
    ssLocState << " [*] 1 : RUNNING if Localization is currently running" << std::endl;
    addPort("outLocalizationState",outLocalizationState)
    .doc(ssLocState.str());

    std::stringstream ssLocMode;
    ssLocMode << "Localization mode :" << std::endl;
    ssLocMode << " [*] 0 : ODO_ONLY  if Localization is using odometry only" << std::endl;
    ssLocMode << " [*] 1 : SMOOTH if Localization is using odometry with large gains for Laser" << std::endl;
    ssLocMode << " [*] 2 : FUSION if Localization is using both odometry and laser" << std::endl;
    addPort("outLocalizationMode",outLocalizationMode)
    .doc(ssLocMode.str());

    std::stringstream ssLocQuality;
    ssLocQuality << "Localization quality :" << std::endl;
    ssLocQuality << " [*] 0 : LOST  if Localizator is lost" << std::endl;
    ssLocQuality << " [*] 1 : BAD if Localization has big covariance" << std::endl;
    ssLocQuality << " [*] 2 : GOOD if Localization has small covariance" << std::endl;
    addPort("outLocalizationQuality",outLocalizationQuality)
    .doc(ssLocQuality.str());

    std::stringstream ssLocVisu;
    ssLocVisu << "Localization visibility :" << std::endl;
    ssLocVisu << " [*] 0 : NONE  if information is not available or sensless" << std::endl;
    ssLocVisu << " [*] 1 : VISIBLE if laser update is running correctly" << std::endl;
    ssLocVisu << " [*] 2 : OCCULTED if Localization fails to find a beacon" << std::endl;
    addPort("outLocalizationVisibility",outLocalizationVisibility)
    .doc(ssLocVisu.str());

    addPort("outObstacles",outObstacles)
    .doc("Last detected obstacles");

    addPort("outNbSeenBeacons", outNbSeenBeacons);


    addOperation("ooInitialize",&Localizator::ooInitialize, this, OwnThread)
    .doc("Initialisation de la Localisation")
    .arg("x","m")
    .arg("y","m")
    .arg("theta","rad");

    addOperation("ooHalt",&Localizator::halt, this, OwnThread)
    .doc("halt localization");

    addOperation("ooResume",&Localizator::resume, this, OwnThread)
    .doc("resume localization");

    addOperation("ooSetParams",&Localizator::setParams, this, OwnThread)
    .doc("")
    .arg("params","");

    addOperation("ooPrintParams",&Localizator::printParams, this, OwnThread)
    .doc("");

    addOperation("ooSwitchToRedConfig",&Localizator::ooSwitchToRedConfig, this, OwnThread)
    .doc("Définit les balises pour le départ Red");

    addOperation("ooSwitchToYellowConfig",&Localizator::ooSwitchToYellowConfig, this, OwnThread)
    .doc("Définit les balises pour le départ Yellow");


    addProperty("propMaxReliableBadOdoTransStddev",propMaxReliableBadOdoTransStddev)
    .doc("Threshold on translation for bad odometry detection (unit is meter)");

    addProperty("propMaxReliableBadOdoRotStddev",propMaxReliableBadOdoRotStddev)
    .doc("Threshold on rotation variance for bad odometry detection (unit is radian)");

    addProperty("propMaxReliableLostTransStddev",propMaxReliableLostTransStddev)
    .doc("Threshold on translation variance for lost detection (unit is meter)");

    addProperty("propMaxReliableLostRotStddev",propMaxReliableLostRotStddev)
    .doc("Threshold on rotation variance for lost detection (unit is radian)");


    addProperty("propLaserRangeSigma",propLaserRangeSigma)
    .doc("Laser Range confidence in meter for normal mode");

    addProperty("propLaserThetaSigma",propLaserThetaSigma)
    .doc("Laser theta confidence in rad for normal mode");

    addProperty("propLaserRangeSigmaSmooth",propLaserRangeSigmaSmooth)
    .doc("Laser Range confidence in meter for smooth mode");

    addProperty("propLaserThetaSigmaSmooth",propLaserThetaSigmaSmooth)
    .doc("Laser theta confidence in rad for smooth mode");

    addProperty("propIEKFMaxIt", propIEKFMaxIt);
    addProperty("propIEKFInnovationMin", propIEKFInnovationMin);

}

void Localizator::ooGetPerformanceReport()
{
    if( !isRunning() || !propTimeReporting )
        cout << "Time Stats are disabled. The component must be in running state with propTimereporting=true." << endl;
    else
    {
        cout << m_timer.GetReport() << endl;
        cout << kfloc.getPerformanceReport() << endl;
    }
}

void Localizator::ooSwitchToRedConfig()
{
    propParams.referencedBeacons = std::vector< lsl::Circle >();
    propParams.referencedBeacons.push_back( lsl::Circle(-propMiddleBeaconX, propMiddleBeaconY, propBeaconRadius ) );
    propParams.referencedBeacons.push_back( lsl::Circle( propCornerBeaconX,-propCornerBeaconY, propBeaconRadius ) );
    propParams.referencedBeacons.push_back( lsl::Circle( propCornerBeaconX, propCornerBeaconY, propBeaconRadius ) );
    kfloc.setParams(propParams);

    LOG(Info) << "Switched to Red Beacon configuration" << endlog();
}

void Localizator::ooSwitchToYellowConfig()
{
    propParams.referencedBeacons = std::vector< lsl::Circle >();
    propParams.referencedBeacons.push_back( lsl::Circle( propMiddleBeaconX, propMiddleBeaconY, propBeaconRadius ) );
    propParams.referencedBeacons.push_back( lsl::Circle(-propMiddleBeaconX, propCornerBeaconY, propBeaconRadius ) );
    propParams.referencedBeacons.push_back( lsl::Circle(-propMiddleBeaconX,-propCornerBeaconY, propBeaconRadius ) );
    kfloc.setParams(propParams);

    LOG(Info) << "Switched to Yellow Beacon configuration" << endlog();
}


void Localizator::updateLocalizationStates()
{
    EstimatedPose2D estim_H_robot_table = kfloc.getLastEstimatedPose2D();
    Covariance3 cov = estim_H_robot_table.cov();
    bool reliabilityOdo = true;
    reliabilityOdo = reliabilityOdo && (cov(0,0) < propMaxReliableBadOdoTransStddev * propMaxReliableBadOdoTransStddev);
    reliabilityOdo = reliabilityOdo && (cov(1,1) < propMaxReliableBadOdoTransStddev * propMaxReliableBadOdoTransStddev);
    reliabilityOdo = reliabilityOdo && (cov(2,2) < propMaxReliableBadOdoRotStddev * propMaxReliableBadOdoRotStddev);
    //TODO workaround WLA : desactivation de check de qualite odo:
    reliabilityOdo = true;


    bool reliabilityLost = true;
    reliabilityLost = reliabilityLost && (cov(0,0) < propMaxReliableLostTransStddev * propMaxReliableLostTransStddev);
    reliabilityLost = reliabilityLost && (cov(1,1) < propMaxReliableLostTransStddev * propMaxReliableLostTransStddev);
    reliabilityLost = reliabilityLost && (cov(2,2) < propMaxReliableLostRotStddev * propMaxReliableLostRotStddev);

    // Mode
    if( updateOk == true )
    {
        if( smoothMode )
        {
            currentMode = SMOOTH;
        }
        else
        {
            currentMode = FUSION;
        }
    }
    else
    {
        currentMode = ODO_ONLY;
    }

    // Quality
    if( currentQuality != LOST )
    {
        if( reliabilityOdo )
        {
            currentQuality = GOOD;
        }
        else
        {
            if( reliabilityLost )
            {
                currentQuality = BAD;
            }
            else
            {
                currentQuality = LOST;
            }
        }
    }

    // Visibility
    if( updateOk == true )
    {
        if( nbSeenBeacons == 2 )
        {
            currentVisibility = SEGMENT;
        }
        else
        {
            currentVisibility = TRIANGLE;
        }
    }
    else
    {
        if( kfloc.getTheoricalVisibility() > 1 )
        {
            currentVisibility = OCCULTED;
        }
        else
        {
            currentVisibility = NONE;
        }
    }
}

bool Localizator::halt()
{
    currentState = STOPPED;
    return true;
}

bool Localizator::resume()
{
    currentState = RUNNING;
    return true;
}


std::string Localizator::getInfo()
{
    std::stringstream ss;
    ss << "***************************************************************************************************" << std::endl;
    ss << "Localization";
    ss << " - State: " << LocalizationStateNames[currentState];
    ss << " - Mode: " << LocalizationModeNames[currentMode];
    ss << " - Quality: " << LocalizationQualityNames[currentQuality];
    ss << " - Visibility: " << LocalizationVisibilityNames[currentVisibility];

    if( currentState == RUNNING)
    {
        ss << " - Theoretical Visu: " << kfloc.getTheoricalVisibility() << std::endl;
        EstimatedPose2D estim_H_robot_table = kfloc.getLastEstimatedPose2D();
        ss << "Estimate : " << estim_H_robot_table.toString() << std::endl;
        std::vector< arp_math::Vector2 > obstacles = kfloc.getDetectedObstacles();
        //ss << "Obstacles (N = " << obstacles.size() << "): ";
        for(unsigned int i = 0 ; (i < obstacles.size()) && ( i < 3) ; i++)
        {
            ss << "(" << obstacles[i].transpose() << ") ";
        }
    }
    else
    {
        ss << " - Theoretical Visu: None" << std::endl;
        ss << "Estimate : None" << std::endl;
        //ss << "Obstacles : None";
    }

    return ss.str();
}

