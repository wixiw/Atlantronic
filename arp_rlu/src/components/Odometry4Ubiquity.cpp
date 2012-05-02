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
using namespace RTT;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Odometry4Ubiquity )

Odometry4Ubiquity::Odometry4Ubiquity(const std::string& name):
    RluTaskContext(name),
    propMinKernelQuality(100.0),
    propMinVelocity(0.001)
{
    createOrocosInterface();
}

void Odometry4Ubiquity::updateHook()
{
    ARDTaskContext::updateHook();
    Twist2D computedTwist;
    EstimatedTwist2D measuredTwist;
    SlippageReport report;
    if( RTT::NewData != inTime.readNewest(attrTime))
    {
        //WLA->BMO : attention, si on nous appelle via une commande c'est possible que ça trigger l'updateHook
        //tu es egalement triggered après le start() il me semble
        LOG( Error ) << "No new data in inTime port : updateHook should not be externally trigger => return" << endlog();
        return;
    }


    if( RTT::NoData == inParams.readNewest(attrParams))
    {
        LOG( Error ) << "No data in inParams port => return" << endlog();
        return;
    }

    if( RTT::NewData != inMotorState.readNewest(attrMotorState))
    {
        LOG( Error ) << "No new data in inMotorState port => return" << endlog();
        return;
    }

    //calcul de l'odometrie (oh oui en une ligne c'est beau)
    if( UbiquityKinematics::motors2Twist(attrMotorState, attrTurretState, computedTwist, report, attrParams) == false )
    {
        LOG(Error) << "Failed to compute Turrets Cmd" << endlog();
    }

    outSlippageDetected.write(report.kernelQuality < propMinKernelQuality);
    if( report.kernelQuality < propMinKernelQuality )
    {
        //LOG(Info) << "Slippage detected ! (kernelQuality=" << report.kernelQuality << ")" << endlog();
    }

    //Si le twist calculé est trop petit on envoit 0
    if( fabs(computedTwist.speedNorm()) < propMinVelocity )
    {
        measuredTwist = EstimatedTwist2D();
    }
    else
    {
        measuredTwist = computedTwist;
    }



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

    measuredTwist.date( timespec2Double(attrTime) );

    //TODO : seuiller à l'arret pour eviter de derriver quand on bouge pas

    outTwist.write(measuredTwist);
    outKernelQuality.write(report.kernelQuality);
}


void Odometry4Ubiquity::createOrocosInterface()
{
    addAttribute("attrTurretState", attrTurretState);
    addAttribute("attrMotorState", attrMotorState);
    addAttribute("attrParams", attrParams);
    addAttribute("attrTime", attrTime);

    addProperty("propMinVelocity",propMinVelocity)
        .doc("La vitesse minimale considérée pour les moteurs de traction (en dessous de quoi le robot est estimé comme étant à l'arrêt complet)");
    addProperty("propMinKernelQuality",propMinKernelQuality)
        .doc("");

    addPort("inTime",inTime)
            .doc("time in second.\n This port is used as trigger.\n This time is used as date of sensors data.");
    addPort("inParams",inParams)
            .doc("UbiquityParams : model parameters");

    addPort("inMotorState",inMotorState)
            .doc("Measures from HML");

    addPort("outTwist",outTwist)
            .doc("T_robot_table_p_robot_r_robot : Twist of robot reference frame relative to table frame, reduced and expressed in robot reference frame.\n It is an EstimatedTwist, so it contains Twist, estimation date (in sec) and covariance matrix.");
    addPort("outKernelQuality",outKernelQuality)
            .doc("Quality of the Kernel when trying to resolve the constraint equations");
    addPort("outSlippageDetected",outSlippageDetected)
        .doc("The computation has detection a slippage");
}
