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
    propMinVelocityOnTurretDriving(0.01),
    propMinVelocityOnTwist(0.001),
    propPercentSigmaTransOdoVelocity(0.1),
    propPercentSigmaRotOdoVelocity(0.1),
    propMinSigmaTransOdoVelocity(0.001),
    propMinSigmaRotOdoVelocity(0.01)
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
        report.kernelQuality = 0.;
        computedTwist = Twist2D();
    }
    else
    {
        if( fabs(attrTurretState.driving.left.velocity) > propMinVelocityOnTurretDriving ||
            fabs(attrTurretState.driving.right.velocity) > propMinVelocityOnTurretDriving ||
            fabs(attrTurretState.driving.rear.velocity) > propMinVelocityOnTurretDriving)
        {
            //on conserve le calcul
        }
        else
        {
            //on fait comme si on ne bougeait pas
            report.kernelQuality = 1000.;
            computedTwist = arp_math::Twist2D();
        }
    }

    outSlippageDetected.write(report.kernelQuality < propMinKernelQuality);
    if( report.kernelQuality < propMinKernelQuality )
    {
        //LOG(Info) << "Slippage detected ! (kernelQuality=" << report.kernelQuality << ")" << endlog();
    }

    //Si le twist calculé est trop petit on envoit 0
    if( fabs(computedTwist.distanceTo(Twist2D(0,0,0),1.0,0.200)) < propMinVelocityOnTwist )
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


    double sigmaXOdo = max( propMinSigmaTransOdoVelocity, fabs(propPercentSigmaTransOdoVelocity * computedTwist.vx()) ); // en m/s
    double sigmaYOdo = max( propMinSigmaTransOdoVelocity, fabs(propPercentSigmaTransOdoVelocity * computedTwist.vy()) ); // en m/s
    double sigmaHOdo = max( propMinSigmaRotOdoVelocity, fabs(propPercentSigmaRotOdoVelocity * computedTwist.vh()) ); // en rad/s

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
    covariance(0,0) = sigmaXOdo * sigmaXOdo;
    covariance(1,1) = sigmaYOdo * sigmaYOdo;
    covariance(2,2) = sigmaHOdo * sigmaHOdo;
    measuredTwist.cov( covariance );

    measuredTwist.date( timespec2Double(attrTime) );

    outTwist.write(measuredTwist);
    outKernelQuality.write(report.kernelQuality);
}


void Odometry4Ubiquity::createOrocosInterface()
{
    addAttribute("attrTurretState", attrTurretState);
    addAttribute("attrMotorState", attrMotorState);
    addAttribute("attrParams", attrParams);
    addAttribute("attrTime", attrTime);

    addProperty("propMinVelocityOnTurretDriving",propMinVelocityOnTurretDriving)
        .doc("Vitesse driving minimale à avoir sur les 3 tourelles pour effectuer le calcul. Sinon on renvoie un Twist nul");
    addProperty("propMinVelocityOnTwist",propMinVelocityOnTwist)
        .doc("Norme minimale du Twist résultant des calculs d'ocométrie, en mm/s. Sinon on renvoit un Twist nul");
    addProperty("propMinKernelQuality",propMinKernelQuality)
        .doc("");
    addProperty( "propPercentSigmaTransOdoVelocity", propPercentSigmaTransOdoVelocity);
    addProperty( "propPercentSigmaRotOdoVelocity", propPercentSigmaRotOdoVelocity);
    addProperty( "propMinSigmaTransOdoVelocity", propMinSigmaTransOdoVelocity);
    addProperty( "propMinSigmaRotOdoVelocity", propMinSigmaRotOdoVelocity);

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
