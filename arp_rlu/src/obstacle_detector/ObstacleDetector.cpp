/*
 * ObstacleDetector.cpp
 *
 *  Created on: 11 mai 2011
 *      Author: wla
 */

#include "ros/param.h"
#include "ros/ros.h"
#include "ObstacleDetector.hpp"

using namespace arp_rlu;
using namespace arp_core;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace geometry_msgs;

ObstacleDetector::ObstacleDetector()
{
    ros::NodeHandle nh = ros::NodeHandle();

    m_scanSubscriber = nh.subscribe("top_scan", 1, &ObstacleDetector::scanCallback, this);
    m_poseSubscriber = nh.subscribe("Localizator/odomRos", 1, &ObstacleDetector::poseCallback, this);
    m_rearObstacleSubscriber
            = nh.subscribe("Protokrot/rear_obstacle", 1, &ObstacleDetector::rearObstacleCallback, this);

    m_rearObstaclePublisher = nh.advertise<Obstacle> ("ObstacleDetector/rear_obstacle", 1);
    m_frontObstaclePublisher = nh.advertise<Obstacle> ("ObstacleDetector/front_obstacle", 1);

    updateRosParams();
    initTransforms();
}

void ObstacleDetector::go()
{
    ros::spin();
}

void ObstacleDetector::scanCallback(LaserScanConstPtr scan)
{
    Obstacle obs;
    obs.detected = 0;
    float d = scan->range_max;
    float angle = 0;
    ros::Time t = ros::Time::now();
    tf::StampedTransform obstacleTf;
    btTransform relativeObstacleTf;
    double front_detection_distance;
    double lateral_detection_distance;

    //ROS_WARN("min : %f max : %f",scan->range_min, scan->range_max);
    for (unsigned int i = 0; i != scan->ranges.size(); i++)
    {
        if (scan->ranges[i] < d && scan->range_min < scan->ranges[i])
        {
            d = scan->ranges[i];
            angle = scan->angle_min + i*scan->angle_increment;
        }

    }

    if (d > scan->range_min)
    {
        //création de la tf associée au point de mesure
        obstacleTf.setOrigin( tf::Vector3(d*cos(angle), d*sin(angle), 0) );
        obstacleTf.setRotation( tf::Quaternion(0, 0, 0) );

        //Changement de repère pour être en relatif dans le repère robot
        relativeObstacleTf = m_baseToFrontLaser * obstacleTf;
        front_detection_distance = relativeObstacleTf.getOrigin().getX();
        lateral_detection_distance = relativeObstacleTf.getOrigin().getY();
        m_tfBroadcaster.sendTransform(tf::StampedTransform(obstacleTf, t , m_frontLaserFrameName, "front_obstacle"));

        //Test si la détection est suffisamment proche
        if ( 0 < front_detection_distance && front_detection_distance < FRONT_DETECTION_THRESHOLD
                &&
             fabs(lateral_detection_distance) < LATERAL_DETECTION_THRESHOLD   )
        {
            //Test si la détection n'est pas trop proche (câble, tour à nous)
            if ( FRONT_BLIND_ZONE < front_detection_distance
                    ||
                LATERAL_BLIND_ZONE < fabs(lateral_detection_distance)  )
            {
                obs.detected = 1;
            }
            //ROS_INFO("tf : (%0.3f,%0.3f)",obstacleTf.getOrigin().getX(), obstacleTf.getOrigin().getY());
            //ROS_INFO("relative tf : (%0.3f,%0.3f)",relativeObstacleTf.getOrigin().getX(), relativeObstacleTf.getOrigin().getY());
            //ROS_INFO("detected : d=%0.3f a=%0.3f",d,angle);
        }
    }

    obs.header.stamp = t;
    m_frontObstaclePublisher.publish(obs);
}

void ObstacleDetector::poseCallback(OdometryConstPtr odom)
{
    m_currentPose = odom->pose.pose;
}

void ObstacleDetector::rearObstacleCallback(BoolConstPtr detected)
{
    Obstacle obstacle;
    ros::Time t = ros::Time::now();
    tf::StampedTransform obstacleTf;

    if( detected->data )
    {
        obstacleTf.setOrigin( tf::Vector3(0.0, 0.0, 0.200) );
        obstacleTf.setRotation( tf::Quaternion(0, 0, 0) );
        m_tfBroadcaster.sendTransform(tf::StampedTransform(obstacleTf, t , m_rearIrFrameName, "rear_obstacle"));
    }

    obstacle.detected = detected->data;
    obstacle.header.stamp = t;
    m_rearObstaclePublisher.publish(obstacle);
}

void ObstacleDetector::updateRosParams()
{
    if (!ros::param::get("ObstacleDetector/FRONT_DETECTION_THRESHOLD", FRONT_DETECTION_THRESHOLD))
    {
        ROS_FATAL("did not found ObstacleDetector/FRONT_DETECTION_THRESHOLD");
        FRONT_DETECTION_THRESHOLD = - 1.000;
    }

    if (!ros::param::get("ObstacleDetector/FRONT_BLIND_ZONE", FRONT_BLIND_ZONE))
    {
        ROS_FATAL("did not found ObstacleDetector/FRONT_BLIND_ZONE");
        FRONT_BLIND_ZONE = - 1.000;
    }

    if (!ros::param::get("ObstacleDetector/LATERAL_DETECTION_THRESHOLD", LATERAL_DETECTION_THRESHOLD))
    {
        ROS_FATAL("did not found ObstacleDetector/LATERAL_DETECTION_THRESHOLD");
        LATERAL_DETECTION_THRESHOLD = - 1.000;
    }

    if (!ros::param::get("ObstacleDetector/LATERAL_BLIND_ZONE", LATERAL_BLIND_ZONE))
    {
        ROS_FATAL("did not found ObstacleDetector/LATERAL_BLIND_ZONE");
        LATERAL_BLIND_ZONE = - 1.000;
    }

    if (!ros::param::get("ObstacleDetector/base_frame", m_baseFrameName))
    {
        ROS_FATAL("did not found ObstacleDetector/base_frame, taking base_link");
        m_baseFrameName = "base_link";
    }

    if (!ros::param::get("ObstacleDetector/rear_ir_switch_frame", m_rearIrFrameName))
    {
        ROS_FATAL("did not found ObstacleDetector/rear_ir_switch_frame, taking rear_ir_switch");
        m_rearIrFrameName = "rear_ir_switch";
    }

    if (!ros::param::get("ObstacleDetector/front_laser_frame", m_frontLaserFrameName))
    {
        ROS_FATAL("did not found ObstacleDetector/front_laser_frame, taking top_laser");
        m_frontLaserFrameName = "top_laser";
    }
}

void ObstacleDetector::initTransforms()
{
    // **** get base to laser tf
    ros::Time t = ros::Time::now();
    try
    {
        m_tfListener.waitForTransform(m_baseFrameName, m_rearIrFrameName, t, ros::Duration(2.0));
        m_tfListener.lookupTransform(m_baseFrameName, m_rearIrFrameName, t, m_baseToIr);
    } catch (tf::TransformException ex)
    {
        ROS_WARN("ObstacleDetector: Could get initial IR transform (%s)", ex.what());
    }

    try
    {
        m_tfListener.waitForTransform(m_baseFrameName, m_frontLaserFrameName, t, ros::Duration(2.0));
        m_tfListener.lookupTransform(m_baseFrameName, m_frontLaserFrameName, t, m_baseToFrontLaser);
    } catch (tf::TransformException ex)
    {
        ROS_WARN("ObstacleDetector: Could get initial front laser transform (%s)", ex.what());
    }
}
