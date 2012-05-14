/*
 * LaserOnlyLocalizatorCpn.cpp
 *
 *  Created on: 9 mai 2012
 *      Author: ard
 */

#include <iomanip>

#include "LaserOnlyLocalizatorCpn.hpp"
#include <rtt/Component.hpp>
#include "LSL/Logger.hpp"
#include "KFL/Logger.hpp"
#include <ros/ros.h>

using namespace arp_core::log;
using namespace arp_rlu;
using namespace arp_math;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_rlu::LaserOnlyLocalizatorCpn )

LaserOnlyLocalizatorCpn::LaserOnlyLocalizatorCpn(const std::string& name)
: RluTaskContext(name)
, loloc()
, lastSuccess(false)
{
    //***WARNING*** Ne pas laisser tourner des logs verbeux sur le robot
    arp_rlu::lsl::Logger::InitFile("LSL", DEBUG);
    arp_rlu::kfl::Logger::InitFile("KFL", DEBUG);

    createOrocosInterface();

    m_monotonicTimeToRealTime = ros::Time::now().toSec() - getTime();


    propParams.mfp.width = 0;

    propParams.pcp.minRange = 0.01;
    propParams.pcp.maxRange = 3.2;
    propParams.pcp.minTheta = -PI;
    propParams.pcp.maxTheta = PI;

    propParams.psp.rangeThres = 0.08;

    propParams.minNbPoints = 3;
    propParams.cartStddevMax = 0.03;

    propParams.xMinAccessible = -1.4;
    propParams.xMaxAccessible =  1.4;
    propParams.yMinAccessible = -0.9;
    propParams.yMaxAccessible =  0.9;

    propParams.cip.radius = 0.04;
    propParams.cip.coeffs = std::vector<double>();
    propParams.cip.coeffs.push_back(-0.01743846);
    propParams.cip.coeffs.push_back( 0.19259734);
    propParams.cip.coeffs.push_back(-0.83735629);
    propParams.cip.coeffs.push_back( 1.81203033);
    propParams.cip.coeffs.push_back(-2.04349845);
    propParams.cip.coeffs.push_back( 1.17177993);
    propParams.cip.coeffs.push_back( 0.67248282);
    propParams.cip.coeffs.push_back( 0.07096937);

    propParams.tcp.radiusTolerance = 0.03;
    propParams.tcp.distanceTolerance = 0.;
    propParams.tcp.maxLengthTolerance = 0.03;
    propParams.tcp.medLengthTolerance = 0.03;
    propParams.tcp.minLengthTolerance = 0.03;

    propParams.dcp.radiusTolerance = 0.03;
    propParams.dcp.distanceTolerance = 0.;
    propParams.dcp.lengthTolerance = 0.03;

    propParams.H_hky_robot = Pose2D(-0.058, 0., M_PI);

    // RED is DEFAULT CONFIG
    propParams.referencedBeacons = std::vector< lsl::Circle >();
    propParams.referencedBeacons.push_back( lsl::Circle(-1.560, 0.   , 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle( 1.565,-1.040, 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle( 1.565, 1.040, 0.04 ) );

    loloc.setParams(propParams);
}

void LaserOnlyLocalizatorCpn::createOrocosInterface()
{
    addProperty("propParams",propParams);


    addPort("inScan",inScan)
    .doc("LaserScan from LRF");


    addOperation("ooDo",&LaserOnlyLocalizatorCpn::ooDo, this, OwnThread)
    .doc("Try Localization using Laser only");

    addOperation("ooGetEstimatedPose",&LaserOnlyLocalizatorCpn::ooGetEstimatedPose, this, OwnThread)
    .doc("Get Estimation (when Localization succeed)");

    addOperation("ooGetRelativeHeadingForConfirmation",&LaserOnlyLocalizatorCpn::ooGetRelativeHeadingForConfirmation, this, OwnThread)
    .doc("Get confirmation angle");

    addOperation("ooSetParams",&LaserOnlyLocalizatorCpn::setParams, this, OwnThread)
    .doc("")
    .arg("params","");

    addOperation("ooPrintParams",&LaserOnlyLocalizatorCpn::printParams, this, OwnThread)
    .doc("");

    addOperation("ooSwitchToRedConfig",&LaserOnlyLocalizatorCpn::ooSwitchToRedConfig, this, OwnThread)
    .doc("Définit les balises pour le départ Red");

    addOperation("ooSwitchToPurpleConfig",&LaserOnlyLocalizatorCpn::ooSwitchToPurpleConfig, this, OwnThread)
    .doc("Définit les balises pour le départ Purple");

}

bool LaserOnlyLocalizatorCpn::configureHook()
{
    if( !RluTaskContext::configureHook() )
        return false;

    return true;
}


void LaserOnlyLocalizatorCpn::updateHook()
{
    RluTaskContext::updateHook();
    return;
}


void LaserOnlyLocalizatorCpn::setParams(LaserOnlyLocalizatorParams params)
{
    propParams = params;
    loloc.setParams(propParams);
    LOG(Info) << "New params defined !" << endlog();
}


std::string LaserOnlyLocalizatorCpn::printParams()
{
    std::stringstream ss;
    ss << "****************************" << std::endl;
    ss << propParams.getInfo();
    ss << "****************************" << std::endl;
    ss << "****************************" << std::endl;
    return ss.str();
}

bool LaserOnlyLocalizatorCpn::ooDo()
{
    sensor_msgs::LaserScan rosScan;
    if( RTT::NewData == inScan.read(rosScan) )
    {
        double dateBeg = rosScan.header.stamp.toSec() - m_monotonicTimeToRealTime;

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
    }
    else
    {
        LOG(Info) << "LaserOnlyLocalizator failed : no Scan available:-(" << endlog();
        return false;
    }

    lastSuccess = loloc.process(lslScan);

    if( lastSuccess )
    {
        LOG(Info) << "LaserOnlyLocalizator succeed" << endlog();
        LOG(Info) << "  => Estimated Pose : " << loloc.getEstimatedPose().toString() << endlog();
    }
    else
    {
        LOG(Info) << "LaserOnlyLocalizator failed" << endlog();
    }
    return lastSuccess;
}

arp_math::EstimatedPose2D LaserOnlyLocalizatorCpn::ooGetEstimatedPose()
{
    if(!lastSuccess)
        LOG(Warning) << "You try to access estimation but last localization failed. Result will be bad." << endlog();

    return loloc.getEstimatedPose();
}

double LaserOnlyLocalizatorCpn::ooGetRelativeHeadingForConfirmation()
{
    return loloc.getRelativeHeadingForConfirmation().angle();
}

std::string LaserOnlyLocalizatorCpn::coGetPerformanceReport()
{
    std::stringstream os;
    os << timer.GetReport();
    os << loloc.getPerformanceReport();
    return os.str();
}


void LaserOnlyLocalizatorCpn::ooSwitchToRedConfig()
{
    propParams.referencedBeacons = std::vector< lsl::Circle >();
    propParams.referencedBeacons.push_back( lsl::Circle(-1.560, 0.   , 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle( 1.565,-1.040, 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle( 1.565, 1.040, 0.04 ) );
    loloc.setParams(propParams);

    LOG(Info) << "Switched to Red Beacon configuration" << endlog();
}

void LaserOnlyLocalizatorCpn::ooSwitchToPurpleConfig()
{
    propParams.referencedBeacons = std::vector< lsl::Circle >();
    propParams.referencedBeacons.push_back( lsl::Circle( 1.560, 0., 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle(-1.555, 1.040, 0.04 ) );
    propParams.referencedBeacons.push_back( lsl::Circle(-1.555,-1.040, 0.04 ) );
    loloc.setParams(propParams);

    LOG(Info) << "Switched to Purple Beacon configuration" << endlog();
}

