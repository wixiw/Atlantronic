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

using namespace arp_core::log;
using namespace arp_rlu;
using namespace arp_math;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Localizator )

Localizator::Localizator(const std::string& name)
: RluTaskContext(name)
, kfloc()
, propMaxReliableTransStddev(0.1 * 0.1)
, propMaxReliableRotStddev( deg2rad(5.0) * deg2rad(5.0))
{
    //***WARNING*** Ne pas laisser tourner des logs verbeux sur le robot
    arp_rlu::lsl::Logger::InitFile("LSL", INFO);
    arp_rlu::kfl::Logger::InitFile("KFL", INFO);

    createOrocosInterface();

    m_monotonicTimeToRealTime = ros::Time::now().toSec() - getTime();

    propParams.defaultInitCovariance = Vector3(0.01, 0.01, 0.01).asDiagonal();

    propParams.bufferSize = 100;
    propParams.maxTime4OdoPrediction = 0.5;

    propParams.H_hky_robot = Pose2D(-0.058, 0., M_PI);
    propParams.H_odo_robot = Pose2D();

    propParams.referencedBeacons = std::vector< lsl::Circle >();
    propParams.referencedBeacons.push_back( lsl::Circle(-1.550, 0.  , 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle( 1.550,-1.05, 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle( 1.550, 1.05, 0.04 ) );

    propParams.iekfParams.defaultOdoVelTransSigma = 0.01;
    propParams.iekfParams.defaultOdoVelRotSigma   = 0.01;
    propParams.iekfParams.defaultLaserRangeSigma  = 0.006;
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
    EstimatedTwist2D T_odo_table_p_odo_r_odo;
    if( RTT::NewData == inOdo.read(T_odo_table_p_odo_r_odo) )
    {
        //update du Kalman
        kfloc.newOdoVelocity(T_odo_table_p_odo_r_odo);
    }

    sensor_msgs::LaserScan rosScan;
    if( RTT::NewData == inScan.read(rosScan) )
    {
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

        kfloc.newScan(lslScan);
    }


    EstimatedPose2D estim_H_robot_table = kfloc.getLastEstimatedPose2D();
    EstimatedTwist2D estim_T_robot_table_p_robot_r_robot = kfloc.getLastEstimatedTwist2D();

    Covariance3 cov = estim_H_robot_table.cov();
    bool reliability = true;
    reliability = reliability && (cov(0,0) < propMaxReliableTransStddev);
    reliability = reliability && (cov(1,1) < propMaxReliableTransStddev);
    reliability = reliability && (cov(2,2) < propMaxReliableRotStddev);

    outPose.write(estim_H_robot_table);
    outTwist.write(estim_T_robot_table_p_robot_r_robot);
    outReliability.write(reliability);
    outObstacles.write(kfloc.getDetectedObstacles());
}

bool Localizator::ooInitialize(double x, double y, double theta)
{
    outReliability.write(false);

    long double initDate = arp_math::getTime();
    EstimatedPose2D pose = MathFactory::createEstimatedPose2D(x,y,theta, initDate, propParams.defaultInitCovariance);


    if( kfloc.initialize(pose) )
    {
        EstimatedPose2D estim_H_robot_table = kfloc.getLastEstimatedPose2D();
        EstimatedTwist2D estim_T_robot_table_p_robot_r_robot = kfloc.getLastEstimatedTwist2D();

        outPose.write(estim_H_robot_table);
        outTwist.write(estim_T_robot_table_p_robot_r_robot);
        outReliability.write(true);

        LOG(Info) << "initialize to " << pose.toString() << " with date : "  << initDate <<  " (sec)" << endlog();

        return true;
    }
    LOG(Info) << "Fail to initialize" << endlog();
    return false;
}


void Localizator::setParams(LocalizatorParams params)
{
    propParams = params;
    kfloc.setParams(propParams);
    LOG(Info) << "New params defined !" << endlog();
}


std::string Localizator::printParams()
{
    std::stringstream ss;
    ss << "****************************" << std::endl;
    ss << propParams.getInfo();
    ss << " [*] propMaxReliableTransStddev : " << propMaxReliableTransStddev << " (m2)" << std::endl;
    ss << " [*] propMaxReliableRotStddev : " << rad2deg(propMaxReliableRotStddev) << " (deg2)" << std::endl;
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

    addPort("outPose",outPose)
    .doc("Last estimation of H_robot_table.\n It is an EstimatedPose2D, so it contains Pose2D, estimation date (in sec) and covariance matrix.");

    addPort("outTwist",outTwist)
    .doc("Last estimation of T_robot_table_p_robot_r_robot Twist of robot reference frame relative to table frame, reduced and expressed in robot reference frame.\n It is an EstimatedTwist2D, so it contains Twist, estimation date (in sec) and covariance matrix.");

    addPort("outReliability",outReliability)
    .doc("False if Localizator has diverged");

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

    addOperation("ooSwitchToPurpleConfig",&Localizator::ooSwitchToPurpleConfig, this, OwnThread)
    .doc("Définit les balises pour le départ Purple");


    addProperty("propMaxReliableTransStddev",propMaxReliableTransStddev)
    .doc("Threshold on translation for reliability boolean elaboration (warning : unity is square meters)");

    addProperty("propMaxReliableRotStddev",propMaxReliableRotStddev)
    .doc("Threshold on rotation for reliability boolean elaboration (warning : unity is square radians)");

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

