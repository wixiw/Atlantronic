/*
 * Odometry4Ubiquity.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "Odometry4Ubiquity.hpp"
#include <rtt/Component.hpp>

using namespace arp_math;
using namespace arp_model;
using namespace arp_rlu;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Odometry4Ubiquity )

Odometry4Ubiquity::Odometry4Ubiquity(const std::string& name):
    RluTaskContext(name),
    propMinKernelQuality(100.0)
{
    createOrocosInterface();
}

void Odometry4Ubiquity::updateHook()
{
    ARDTaskContext::updateHook();

    if( RTT::NewData != inTime.readNewest(attrTime))
    {
        //WLA->BMO : attention, si on nous appelle via une commande c'est possible que ça trigger l'updateHook
        //tu es egalement triggered après le start() il me semble
        LOG( Warning ) << "No new data in inTime port : updateHook should not be externally trigger => return" << endlog();
        return;
    }

    if( RTT::NoData == inParams.readNewest(attrParams))
    {
        LOG( Warning ) << "No data in inParams port => return" << endlog();
        return;
    }

    if( RTT::NewData != inMotorState.readNewest(attrMotorState))
    {
        LOG( Warning ) << "No new data in inMotorState port => return" << endlog();
        return;
    }

    //calcul de l'odometrie (oh oui en une ligne c'est beau)
    Twist2D computedTwist;
    SlippageReport report;
    if( UbiquityKinematics::motors2Twist(attrMotorState, computedTwist, report, attrParams) == false )
    {
        LOG(Error) << "Failed to compute Turrets Cmd" << endlog();
    }

    attrKernelQuality = report.kernelQuality;
    if( attrKernelQuality < propMinKernelQuality )
    {
        LOG(Warning) << "Slippage detected ! (kernelQuality=" << report.kernelQuality << ")" << endlog();
    }

    EstimatedTwist2D measuredTwist = computedTwist;

    // En attendant de propager les erreurs des moteurs, on définit arbitrairement (cf maquette python) des covariances :
    // sigmaXOdo = np.max(  [params.simu_cfg["minSigmaTransOdoVelocity"], params.simu_cfg["percentSigmaTransOdoVelocity"] * np.fabs(vxOdo)])
    // sigmaYOdo = np.max( [params.simu_cfg["minSigmaTransOdoVelocity"], params.simu_cfg["percentSigmaTransOdoVelocity"] * np.fabs(vyOdo)])
    // sigmaHOdo = np.max( [params.simu_cfg["minSigmaRotOdoVelocity"],   params.simu_cfg["percentSigmaRotOdoVelocity"]   * np.fabs(vhOdo)])

    const double percentSigmaTransOdoVelocity = 0.03; // 3 %
    const double percentSigmaRotOdoVelocity = 0.03;   // 3 %
    const double minSigmaTransOdoVelocity = 0.001;    // 1 mm/s
    const double minSigmaRotOdoVelocity = 0.01;       // 0.5 deg/s

    double sigmaXOdo = max( minSigmaTransOdoVelocity, fabs(percentSigmaTransOdoVelocity * computedTwist.vx()) ); // en m/s
    double sigmaYOdo = max( minSigmaTransOdoVelocity, fabs(percentSigmaTransOdoVelocity * computedTwist.vy()) ); // en m/s
    double sigmaHOdo = max( minSigmaRotOdoVelocity, fabs(percentSigmaRotOdoVelocity * computedTwist.vh()) ); // en rad/s

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
    covariance(0,0) = sigmaXOdo * sigmaXOdo;
    covariance(1,1) = sigmaYOdo * sigmaYOdo;
    covariance(2,2) = sigmaHOdo * sigmaHOdo;
    measuredTwist.cov( covariance );

    measuredTwist.date( attrTime );

    outTwist.write(measuredTwist);
}


void Odometry4Ubiquity::createOrocosInterface()
{
    addAttribute("attrMotorState", attrMotorState);
    addAttribute("attrParams", attrParams);
    addAttribute("attrTime", attrTime);
    addAttribute("attrKernelQuality",attrKernelQuality);

    addProperty("propMinKernelQuality",propMinKernelQuality)
        .doc("");

    addEventPort("inTime",inTime)
            .doc("time in second.\n This port is used as trigger.\n This time is used as date of sensors data.");
    addPort("inParams",inParams)
            .doc("UbiquityParams : model parameters");

    addPort("inMotorState",inMotorState)
            .doc("Measures from HML");

    addPort("outTwist",outTwist)
            .doc("T_robot_table_p_robot_r_robot : Twist of robot reference frame relative to table frame, reduced and expressed in robot reference frame.\n It is an EstimatedTwist, so it contains Twist, estimation date (in sec) and covariance matrix.");
}
