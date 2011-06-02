/*
 * ReLocalizatorNode.cpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */

#include <ros/ros.h>
#include "ReLocalizatorNode.hpp"

using namespace arp_rlu;

ReLocalizatorNode::ReLocalizatorNode() :
    nh()
{
    estimatePosition_srv = nh.advertiseService("/ReLocalizator/EstimatePosition",
            &ReLocalizatorNode::estimatePositionCallback, this);
    cornerdetection_client_ = nh.serviceClient<arp_rlu::DetectCorner> ("/CornerDetector/DetectCorner");

    if (!ros::param::get("ReLocalizator/base_frame", m_baseFrameName))
    {
        ROS_WARN("did not found base_frame, taking base_link");
        m_baseFrameName = "base_link";
    }

    if (!ros::param::get("ReLocalizator/front_laser_frame", m_frontLaserFrameName))
    {
        ROS_WARN("did not found front_laser_frame, taking top_laser");
        m_frontLaserFrameName = "top_laser";
    }

    try
    {
        ros::Time t = ros::Time(0,0);
        m_tfListener.waitForTransform(m_baseFrameName, m_frontLaserFrameName, t, ros::Duration(2.0));
        m_tfListener.lookupTransform(m_baseFrameName, m_frontLaserFrameName, t, m_baseToFrontLaser);
    } catch (tf::TransformException ex)
    {
        ROS_WARN("ReLocalizatorNode: Could get initial front laser transform (%s)", ex.what());
    }
}

ReLocalizatorNode::~ReLocalizatorNode()
{
}

bool ReLocalizatorNode::estimatePositionCallback(EstimatePosition::Request& req, EstimatePosition::Response& res)
{
    ROS_INFO("***************************");
    ROS_INFO("Estimation asked");
    ROS_INFO("================");

    double frontal_deport = 0.26; //m_baseToFrontLaser.getOrigin().getX();
    double lateral_deport = 0.053; //m_baseToFrontLaser.getOrigin().getY();

    //frontal_deport = 0.0;
    //lateral_deport = 0.0;

    double c = cos(req.previousTheta);
    double s = sin(req.previousTheta);
    rl.previousX = req.previousX + frontal_deport * c - lateral_deport * s;
    rl.previousY = req.previousY + frontal_deport * s + lateral_deport * c;
    rl.previousTheta = req.previousTheta;

    TableCorner target = rl.selectTargetTableCorner();
    ROS_INFO("TableCorner targeted :");
    target.print();

    std::pair<double, double> p = rl.chooseScanWindow(target);
    ROS_INFO("Selected window in Hokuyo ref : from %f to %f", arp_math::rad2deg(p.first), arp_math::rad2deg(p.second));

    arp_rlu::DetectCorner dc_srv;
    dc_srv.request.minAngle = p.first;
    dc_srv.request.maxAngle = p.second;
    if (!cornerdetection_client_.call(dc_srv))
    {
        res.estimatedX = rl.previousX;
        res.estimatedY = rl.previousY;
        res.estimatedTheta = rl.previousTheta;
        res.quality = -1;
        ROS_WARN("EstimatePosition Service : calling CornerDetection service failed");
        return true;
    }

    Corner detected;
    detected.d1 = dc_srv.response.d1;
    detected.alpha1 = dc_srv.response.alpha1;
    detected.length1 = dc_srv.response.length1;
    detected.angleBegin1 = dc_srv.response.angleBegin1;
    detected.angleEnd1 = dc_srv.response.angleEnd1;
    detected.d2 = dc_srv.response.d2;
    detected.alpha2 = dc_srv.response.alpha2;
    detected.length2 = dc_srv.response.length2;
    detected.angleBegin2 = dc_srv.response.angleBegin2;
    detected.angleEnd2 = dc_srv.response.angleEnd2;
    detected.diag = dc_srv.response.diag;
    detected.theta = dc_srv.response.theta;
    detected.cornerAngle = dc_srv.response.cornerAngleInDeg;

    rl.estimatePose(detected, target);

    c = cos(rl.estimatedTheta);
    s = sin(rl.estimatedTheta);
    res.estimatedX = rl.estimatedX - frontal_deport * c + lateral_deport * s;
    res.estimatedY = rl.estimatedY - frontal_deport * s - lateral_deport * c;
    res.estimatedTheta = rl.estimatedTheta;
    res.quality = rl.quality;

    if (rl.quality > 0)
    {
        ROS_INFO("Laser estimatedX: %.3f", rl.estimatedX);
        ROS_INFO("Laser estimatedY: %.3f", rl.estimatedY);
        ROS_INFO("Laser estimatedTheta: %.3f", rl.estimatedTheta);
        ROS_INFO("SUCCESS !!");
    }
    else
    {
        ROS_WARN("FAILED !!");
    }

    return true;
}

void ReLocalizatorNode::go()
{
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ReLocalizator");
    ReLocalizatorNode node;

    node.go();

    return 0;
}
