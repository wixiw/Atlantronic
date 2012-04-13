/*
 * Localizator.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "Localizator.hpp"
#include <rtt/Component.hpp>
#include "LSL/Logger.hpp"
#include "KFL/Logger.hpp"

using namespace arp_core::log;
using namespace arp_rlu;
using namespace arp_math;
using namespace RTT;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Localizator )

Localizator::Localizator(const std::string& name)
     : RluTaskContext(name)
     , kfloc()
{
    arp_rlu::lsl::Logger::InitConsole("LSL", ERROR);
    arp_rlu::kfl::Logger::InitFile("KFL", DEBUG);
    createOrocosInterface();
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

void Localizator::scanCb(RTT::base::PortInterface* portInterface)
{
    sensor_msgs::LaserScan rosScan;
    inScan.readNewest(rosScan);

    lsl::LaserScan lslScan;
    Eigen::MatrixXd polarData(3, rosScan.ranges.size());
    //ROS_WARN("min : %f max : %f",scan->range_min, scan->range_max);
    for (unsigned int i = 0; i != rosScan.ranges.size(); i++)
    {
        polarData(0,i) = rosScan.header.stamp.toSec() + i * rosScan.time_increment;
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

void Localizator::odoCb(RTT::base::PortInterface* portInterface)
{
    //TODO faire des test sur les inputs ports ?

    EstimatedTwist2D T_r_t;
    EstimatedTwist2D T_r_t_t; //je ne sais pas le vrai nom mais je rajoute un t pour dire qu'il y a un bricolage pour etre dans le repere de la table
    inOdo.readNewest(T_r_t);

    //changement de repère de T_r_t
    Eigen::Vector3d T;
    T << T_r_t.vx(), T_r_t.vy(), T_r_t.vh();
    Eigen::Matrix<double,3,3> R = Eigen::Matrix<double,3,3>::Identity();
    R.topLeftCorner(2,2) = kfloc.getLastEstimatedPose2D().orientation().toRotationMatrix();
    Eigen::Vector3d V = R * T;
    Eigen::Matrix3d covariance = R * T_r_t.cov() * R.inverse();
    T_r_t_t.vx( V(0) );
    T_r_t_t.vy( V(1) );
    T_r_t_t.vh( V(2) );
    T_r_t_t.cov( covariance );
    T_r_t_t.date(T_r_t.date());

    //update du Kalman
    kfloc.newOdoVelocity(T_r_t_t);

}

void Localizator::updateHook()
{
    // !!!!!!!! BIG FAT WARNING : l'updateHook est appele apres chaque callback automatiquement par orocos !!!

    EstimatedPose2D p = kfloc.getLastEstimatedPose2D();
    EstimatedTwist2D T_r_t_t = kfloc.getLastEstimatedTwist2D(); //idem odoCB
    EstimatedTwist2D T_r_t;


    Eigen::Vector3d T;
    T << T_r_t_t.vx(), T_r_t_t.vy(), T_r_t_t.vh();
    Eigen::Matrix<double,3,3> R = Eigen::Matrix<double,3,3>::Identity();
    R.topLeftCorner(2,2) = kfloc.getLastEstimatedPose2D().orientation().toRotationMatrix();
    Eigen::Vector3d V = R.inverse() * T;
    Eigen::Matrix3d covariance = R.inverse() * T_r_t.cov() * R;
    T_r_t.vx( V(0) );
    T_r_t.vy( V(1) );
    T_r_t.vh( V(2) );
    T_r_t.cov( covariance );
    T_r_t.date(T_r_t_t.date());

    outTwist.write(T_r_t);
    outPose.write(p);
}

bool Localizator::ooInitialize(double x, double y, double theta)
{
    EstimatedPose2D pose(x,y,theta);
    double initDate = arp_math::getTime();
    pose.date( initDate );
    
    LOG(Info) << "initialize to " << pose.toString() << " with date : "  << initDate <<  " (sec)" << endlog();
    
    return kfloc.initialize(pose);
}


void Localizator::setParams(LocalizatorParams params)
{
    propParams = params;
    kfloc.setParams(propParams);
    LOG(Info) << "New params defined !" << endlog();

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

    addOperation("ooInitialize",&Localizator::ooInitialize, this, OwnThread)
                .doc("Initialisation de la Localisation")
                .arg("x","m")
                .arg("y","m")
                .arg("theta","rad");

    addOperation("ooSetParams",&Localizator::setParams, this, OwnThread)
                .doc("")
                .arg("params","");
}
