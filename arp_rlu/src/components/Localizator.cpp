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

#define stringify( name ) # name

using namespace arp_core::log;
using namespace arp_rlu;
using namespace arp_math;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Localizator )


const char * LocalizationStateNames[6] = {
        stringify( __STOPED__ ),
        stringify( _ODO_ONLY_ ),
        stringify( __FUSION__ ),
        stringify( _OCCULTED_ ),
        stringify( _BAD__ODO_ ),
        stringify( ___LOST___ )
};

Localizator::Localizator(const std::string& name)
: RluTaskContext(name)
, kfloc()
, propMaxReliableBadOdoTransStddev(0.10)
, propMaxReliableBadOdoRotStddev( deg2rad(12.) )
, propMaxReliableLostTransStddev(0.20)
, propMaxReliableLostRotStddev( deg2rad(22.) )
, propLaserRangeSigma(0.10)
, propLaserThetaSigma(0.05)
, propLaserRangeSigmaSmooth(1000)
, propLaserThetaSigmaSmooth(1000)
, updateNeverTried(true)
, updateTried(false)
, predictionOk(false)
, updateOk(false)
, currentState(___LOST___)
{
    //***WARNING*** Ne pas laisser tourner des logs verbeux sur le robot
    arp_rlu::lsl::Logger::InitFile("LSL", INFO);
    arp_rlu::kfl::Logger::InitFile("KFL", INFO);

    createOrocosInterface();

    m_monotonicTimeToRealTime = ros::Time::now().toSec() - getTime();

    propParams.defaultInitCovariance = Vector3(0.020 * 0.020, 0.020 * 0.020, deg2rad(1.) * deg2rad(1.)).asDiagonal();

    propParams.bufferSize = 100;
    propParams.maxTime4OdoPrediction = 0.5;

    propParams.H_hky_robot = Pose2D(-0.058, 0., M_PI);
    propParams.H_odo_robot = Pose2D();

    // Red is default config
    propParams.referencedBeacons = std::vector< lsl::Circle >();
    propParams.referencedBeacons.push_back( lsl::Circle(-1.560, 0.   , 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle( 1.565,-1.040, 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle( 1.565, 1.040, 0.04 ) );

    propParams.iekfParams.defaultOdoVelTransSigma = 0.01;
    propParams.iekfParams.defaultOdoVelRotSigma   = 0.01;
    propParams.iekfParams.defaultLaserRangeSigma  = 0.10;
    propParams.iekfParams.defaultLaserThetaSigma  = 0.05;
    propParams.iekfParams.iekfMaxIt               = 10;
    propParams.iekfParams.iekfInnovationMin       = 0.0122474;

    propParams.procParams.mfp.width = 3;

    propParams.procParams.pcp.minRange = 0.10;
    propParams.procParams.pcp.maxRange = 3.5;
    propParams.procParams.pcp.minTheta = -arp_math::PI;
    propParams.procParams.pcp.maxTheta =  arp_math::PI;

    propParams.procParams.psp.rangeThres = 0.08;

    propParams.procParams.cip.radius = 0.04;
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
    propParams.procParams.tcp.distanceTolerance = 0.8;
    propParams.procParams.tcp.maxLengthTolerance = 0.05;
    propParams.procParams.tcp.medLengthTolerance = 0.05;
    propParams.procParams.tcp.minLengthTolerance = 0.05;

    propParams.procParams.dcp.radiusTolerance = 0.03;
    propParams.procParams.dcp.distanceTolerance = 0.6;
    propParams.procParams.dcp.lengthTolerance = 0.05;

    propParams.procParams.minNbPoints = 3;

    kfloc.setParams(propParams);
}

bool Localizator::configureHook()
{
    if( !RluTaskContext::configureHook() )
        goto fail;

    if(  !ooInitialize(0,0,0) )
    {
        LOG(Error) << "failed to initialize kfloc" << endlog();
        goto fail;
    }

    //reussit
    goto success;

    fail:
    return false;
    success:
    return true;
}


void Localizator::updateHook()
{
    bool smoothMode = false;
    inSmoothMode.read(smoothMode);
    if(smoothMode && (propParams.iekfParams.defaultLaserRangeSigma !=  propLaserRangeSigmaSmooth||
            propParams.iekfParams.defaultLaserThetaSigma != propLaserThetaSigmaSmooth ) )
    {
        propParams.iekfParams.defaultLaserRangeSigma = propLaserRangeSigmaSmooth;
        propParams.iekfParams.defaultLaserThetaSigma = propLaserThetaSigmaSmooth;
        kfloc.setParams( propParams.iekfParams );
        LOG(Info) << "Switched to smooth mode (LaserRangeSigma: " << kfloc.getParams().iekfParams.defaultLaserRangeSigma << " - LaserThetaSigma: " << kfloc.getParams().iekfParams.defaultLaserThetaSigma << ")" << endlog();
    }
    if( !smoothMode && (propParams.iekfParams.defaultLaserRangeSigma != propLaserRangeSigma ||
            propParams.iekfParams.defaultLaserThetaSigma != propLaserThetaSigma ) )
    {
        propParams.iekfParams.defaultLaserRangeSigma = propLaserRangeSigma;
        propParams.iekfParams.defaultLaserThetaSigma = propLaserThetaSigma;
        kfloc.setParams( propParams.iekfParams );
        LOG(Info) << "Switched to normal mode (LaserRangeSigma: " << kfloc.getParams().iekfParams.defaultLaserRangeSigma << " - LaserThetaSigma: " << kfloc.getParams().iekfParams.defaultLaserThetaSigma << ")" << endlog();
    }



    EstimatedTwist2D T_odo_table_p_odo_r_odo;
    if( RTT::NewData == inOdo.read(T_odo_table_p_odo_r_odo) )
    {
        //update du Kalman
        predictionOk = kfloc.newOdoVelocity(T_odo_table_p_odo_r_odo);
    }

    updateTried = false;
    sensor_msgs::LaserScan rosScan;
    if( RTT::NewData == inScan.read(rosScan) )
    {
        updateNeverTried = false;
        updateTried = true;
        double dateBeg = rosScan.header.stamp.toSec() - m_monotonicTimeToRealTime;

        lsl::LaserScan lslScan;
        Eigen::MatrixXd polarData(3, rosScan.ranges.size());
        for (unsigned int i = 0; i != rosScan.ranges.size(); i++)
        {
            polarData(0,i) = dateBeg + i * rosScan.time_increment;
            polarData(2,i) = rosScan.angle_min + i*rosScan.angle_increment;
            if (rosScan.ranges[i] <= rosScan.range_max && rosScan.range_min <= rosScan.ranges[i])
            {
                polarData(1,i) = rosScan.ranges[i];
            }
            else
            {
                polarData(1,i) = 0.;
            }
        }
        lslScan.setPolarData(polarData);

        updateOk = kfloc.newScan(lslScan);

    }

    updateLocalizationState();


    EstimatedPose2D estim_H_robot_table = kfloc.getLastEstimatedPose2D();
    EstimatedTwist2D estim_T_robot_table_p_robot_r_robot = kfloc.getLastEstimatedTwist2D();


    kfl::Log( Info ) << "***************************************************************************************************";
    kfl::Log( Info ) << "Localization - State: " << LocalizationStateNames[currentState] << " - Visu: " << kfloc.getTheoricalVisibility() << " - Estimate : " << estim_H_robot_table.toString();

    std::vector< arp_math::Vector2 > obstacles = kfloc.getDetectedObstacles();
    outObstacles.write(obstacles);
    std::stringstream ss;
    ss << "  Obstacles : ";
    for(unsigned int i = 0 ; (i < obstacles.size()) && ( i < 3) ; i++)
    {
        ss << "(" << obstacles[i].transpose() << ") ";
    }
    kfl::Log( Info ) << ss.str();

    outPose.write(estim_H_robot_table);
    outTwist.write(estim_T_robot_table_p_robot_r_robot);
    outLocalizationState.write(currentState);

}

bool Localizator::ooInitialize(double x, double y, double theta)
{

    long double initDate = arp_math::getTime();
    EstimatedPose2D pose = MathFactory::createEstimatedPose2D(x,y,theta, initDate, propParams.defaultInitCovariance);


    if( kfloc.initialize(pose) )
    {
        EstimatedPose2D estim_H_robot_table = kfloc.getLastEstimatedPose2D();
        EstimatedTwist2D estim_T_robot_table_p_robot_r_robot = kfloc.getLastEstimatedTwist2D();

        outPose.write(estim_H_robot_table);
        outTwist.write(estim_T_robot_table_p_robot_r_robot);

        updateNeverTried = true;
        predictionOk = false;
        updateOk = false;
        currentState = __STOPED__;

        LOG(Info) << "initialize to " << pose.toString() << " with date : "  << initDate <<  " (sec)" << endlog();
        kfl::Log( Info ) << "***************************************************************************************************";
        kfl::Log( Info ) << "Localization - State: " << LocalizationStateNames[currentState] << " - Visu: " << kfloc.getTheoricalVisibility() << " - Estimate : " << estim_H_robot_table.toString();


        outLocalizationState.write(currentState);

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

    addPort("inSmoothMode",inSmoothMode)
    .doc("When true, change settings to avoid jumps in estimate");

    addPort("outPose",outPose)
    .doc("Last estimation of H_robot_table.\n It is an EstimatedPose2D, so it contains Pose2D, estimation date (in sec) and covariance matrix.");

    addPort("outTwist",outTwist)
    .doc("Last estimation of T_robot_table_p_robot_r_robot Twist of robot reference frame relative to table frame, reduced and expressed in robot reference frame.\n It is an EstimatedTwist2D, so it contains Twist, estimation date (in sec) and covariance matrix.");

    std::stringstream ss;
    ss << "Localization state :" << std::endl;
    ss << " [*] __STOPED__ if Localization is switched off or if robot does not move" << std::endl;
    ss << " [*] _ODO_ONLY_ if Localization is using odometry only (beacons are not visible)" << std::endl;
    ss << " [*] __FUSION__ if Localization is using both odometry and laser (the most accurate state)" << std::endl;
    ss << " [*] _OCCULTED_ if Localization should see beacons but they are occulted and localization accuracy is still good." << std::endl;
    ss << " [*] _BAD__ODO_ if Localization is using odometry only since long time." << std::endl;
    ss << " [*] ___LOST___ if Localization is lost. Only a new initialization can quit this state." << std::endl;
    addPort("outLocalizationState",outLocalizationState)
    .doc(ss.str());

    addPort("outObstacles",outObstacles)
    .doc("Last detected obstacles");


    addOperation("ooInitialize",&Localizator::ooInitialize, this, OwnThread)
    .doc("Initialisation de la Localisation")
    .arg("x","m")
    .arg("y","m")
    .arg("theta","rad");

    addOperation("ooSetParams",&Localizator::setParams, this, OwnThread)
    .doc("")
    .arg("params","");

    addOperation("ooPrintParams",&Localizator::printParams, this, OwnThread)
    .doc("");

    addOperation("ooSwitchToRedConfig",&Localizator::ooSwitchToRedConfig, this, OwnThread)
    .doc("Définit les balises pour le départ Red");
    propParams.referencedBeacons.push_back( lsl::Circle(-1.560, 0.   , 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle( 1.565,-1.040, 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle( 1.565, 1.040, 0.04 ) );

    addOperation("ooSwitchToPurpleConfig",&Localizator::ooSwitchToPurpleConfig, this, OwnThread)
    .doc("Définit les balises pour le départ Purple");


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

}

std::string Localizator::coGetPerformanceReport()
{
    std::stringstream os;
    os << timer.GetReport();
    os << kfloc.getPerformanceReport();
    return os.str();
}


void Localizator::ooSwitchToRedConfig()
{
    propParams.referencedBeacons = std::vector< lsl::Circle >();
    propParams.referencedBeacons.push_back( lsl::Circle(-1.560, 0.   , 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle( 1.565,-1.040, 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle( 1.565, 1.040, 0.04 ) );
    kfloc.setParams(propParams);

    LOG(Info) << "Switched to Red Beacon configuration" << endlog();
}

void Localizator::ooSwitchToPurpleConfig()
{
    propParams.referencedBeacons = std::vector< lsl::Circle >();
    propParams.referencedBeacons.push_back( lsl::Circle( 1.560, 0., 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle(-1.555, 1.040, 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle(-1.555,-1.040, 0.04 ) );
    kfloc.setParams(propParams);

    LOG(Info) << "Switched to Purple Beacon configuration" << endlog();
}


void Localizator::updateLocalizationState()
{
    EstimatedPose2D estim_H_robot_table = kfloc.getLastEstimatedPose2D();
    Covariance3 cov = estim_H_robot_table.cov();
    bool reliabilityOdo = true;
    reliabilityOdo = reliabilityOdo && (cov(0,0) < propMaxReliableBadOdoTransStddev * propMaxReliableBadOdoTransStddev);
    reliabilityOdo = reliabilityOdo && (cov(1,1) < propMaxReliableBadOdoTransStddev * propMaxReliableBadOdoTransStddev);
    reliabilityOdo = reliabilityOdo && (cov(2,2) < propMaxReliableBadOdoRotStddev * propMaxReliableBadOdoRotStddev);

    bool reliabilityLost = true;
    reliabilityLost = reliabilityLost && (cov(0,0) < propMaxReliableLostTransStddev * propMaxReliableLostTransStddev);
    reliabilityLost = reliabilityLost && (cov(1,1) < propMaxReliableLostTransStddev * propMaxReliableLostTransStddev);
    reliabilityLost = reliabilityLost && (cov(2,2) < propMaxReliableLostRotStddev * propMaxReliableLostRotStddev);

    if(currentState < ___LOST___)
    {
        if(predictionOk == false)
        {
            currentState = __STOPED__;
            return;
        }
        else
        {
            if( updateOk == true )
            {
                currentState = __FUSION__;
                return;
            }
            else // update == false
            {
                // I din't tried
                if( updateTried == false )
                {
                    if( updateNeverTried == true )
                    {
                        // Laser should not be connected
                        if( reliabilityOdo )
                        {
                            currentState = _ODO_ONLY_;
                            return;
                        }
                        else
                        {
                            currentState = _BAD__ODO_;
                            return;
                        }
                    }
                    else
                    {
                        // it is odo cycle, without scan info. Keep last state
                        return;
                    }
                }
                else //updateTried == true

                {
                    // I should have seen something
                    if( kfloc.getTheoricalVisibility() > 1 )
                    {
                        if( !reliabilityLost )
                        {
                            currentState = ___LOST___;
                            return;
                        }
                        else
                        {
                            currentState = _OCCULTED_;
                            return;
                        }
                    }
                    else // No visibility
                    {
                        if( reliabilityOdo )
                        {
                            currentState = _ODO_ONLY_;
                            return;
                        }
                        else
                        {
                            currentState = _BAD__ODO_;
                            return;
                        }
                    }
                }
            }
        }
    }
}


