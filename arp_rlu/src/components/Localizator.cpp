/*
 * Localizator.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "Localizator.hpp"
#include <rtt/Component.hpp>

using namespace arp_rlu;
using namespace arp_math;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Localizator )

Localizator::Localizator(const std::string& name)
     : RluTaskContext(name)
     , kfloc()
{
    createOrocosInterface();
}


bool Localizator::initialize(EstimatedPose2D pose)
{
    return kfloc.initialize(pose);
}


void Localizator::setParams(LocalizatorParams params)
{
    propParams = params;
    kfloc.setParams(propParams);
}

void Localizator::scanCb(RTT::base::PortInterface* portInterface)
{

}

void Localizator::odoCb(RTT::base::PortInterface* portInterface)
{
    EstimatedTwist2D T_r_t;

    inOdo.readNewest(T_r_t);

    Eigen::Vector3d T;
    T << T_r_t.vx(), T_r_t.vy(), T_r_t.vh();
    Eigen::Vector3d V = kfloc.getLastEstimatedPose2D().getDisplacement2Matrix() * T;
    Eigen::Matrix3d covariance = kfloc.getLastEstimatedPose2D().getDisplacement2Matrix() * T_r_t.cov();
    T_r_t.vx( V(0) );
    T_r_t.vy( V(1) );
    T_r_t.vh( V(2) );
    T_r_t.cov( covariance );

    kfloc.newOdoVelocity(T_r_t);

}

void Localizator::updadeHook()
{
    // !!!!!!!! BIG FAT WARNING : l'updateHook est appele apres chaque callback automatiquement par orocos !!!

    EstimatedPose2D p = kfloc.getLastEstimatedPose2D();
    EstimatedTwist2D T_r_t = kfloc.getLastEstimatedTwist2D();

    Eigen::Vector3d T;
    T << T_r_t.vx(), T_r_t.vy(), T_r_t.vh();
    Eigen::Vector3d V = kfloc.getLastEstimatedPose2D().getDisplacement2Matrix().inverse() * T;
    Eigen::Matrix3d covariance = kfloc.getLastEstimatedPose2D().getDisplacement2Matrix().inverse() * T_r_t.cov();
    T_r_t.vx( V(0) );
    T_r_t.vy( V(1) );
    T_r_t.vh( V(2) );
    T_r_t.cov( covariance );

    outTwist.write(T_r_t);
    outPose.write(p);
}

void Localizator::createOrocosInterface()
{
    addProperty("propParams",propParams);

    addEventPort("inScan",inScan, boost::bind(&Localizator::scanCb,this,_1))
            .doc("LaserScan from LRF");
    addEventPort("inOdo",inOdo , boost::bind(&Localizator::odoCb,this,_1))
            .doc("Estimation from odometry of T_robot_table_p_robot_r_robot : Twist of robot reference frame relative to table frame, reduced and expressed in robot reference frame.\n It is an EstimatedTwist, so it contains Twist, estimation date (in sec) and covariance matrix.");
    addPort("outPose",outPose)
            .doc("Last estimation of H_robot_table.\n Cette estimée est datée et dispose d'une matrice de covariance.");
    addPort("outTwist",outTwist)
            .doc("Last estimation of T_robot_table_p_robot_r_robot.\n Cette estimée est datée et dispose d'une matrice de covariance.");

    addOperation("ooInitialize",&Localizator::initialize, this, OwnThread)
                .doc("Initialisation de la Localisation")
                .arg("pose","EstimatedPose2D : pose initiale, datée, avec sa covariance");

    addOperation("ooSetParams",&Localizator::setParams, this, OwnThread)
                .doc("")
                .arg("params","");
}
