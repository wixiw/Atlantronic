/*
 * ObjectFinderNode.cpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */

#include <ros/ros.h>
#include "objectfinder/ObjectFinderNode.hpp"

using namespace arp_rlu;
using namespace sensor_msgs;
using namespace arp_math;

ObjectFinderNode::ObjectFinderNode() :
    nh(), scan(Eigen::MatrixXd::Zero(1, 1)), reverse_scan(false)
{
    std::string front_scan_topic_name;
    if (nh.getParam("ObjectFinder/scan_topic_name", front_scan_topic_name))
    {
        ROS_INFO("Got param named 'scan_topic_name' : %s", front_scan_topic_name);
    }
    else
    {
        ROS_WARN("Failed to get param 'scan_topic_name'. Take default value (/scan)");
        front_scan_topic_name = "/scan";
    }

    if (nh.getParam("ObjectFinder/reverse_scan", reverse_scan))
    {
        ROS_INFO("Got param named 'reverse_scan' : %i", reverse_scan);
    }
    else
    {
        ROS_WARN("Failed to get param 'reverse_scan'. Take default value (false)");
        reverse_scan = false;
    }

    m_towerPublisher = nh.advertise<PointCloud>("/ObjectFinder/display/tower",1);
    m_robotPublisher = nh.advertise<PointCloud>("/ObjectFinder/display/robot",1);
    m_figurePublisher = nh.advertise<PointCloud>("/ObjectFinder/display/figure",1);
    m_ufoPublisher = nh.advertise<PointCloud>("/ObjectFinder/display/ufo",1);

    scan_sub = nh.subscribe(front_scan_topic_name, 1, &ObjectFinderNode::scanCallback, this);
    findobjects_srv = nh.advertiseService("/ObjectFinder/FindObjects", &ObjectFinderNode::findobjectsCallback, this);

    srand(time(NULL));
}

ObjectFinderNode::~ObjectFinderNode()
{
}

void ObjectFinderNode::go()
{
    ros::spin();
}

void ObjectFinderNode::scanCallback(LaserScanConstPtr s)
{
    unsigned int n = s->ranges.size();
    unsigned int nb = 0;
    for (unsigned int i = 0; i != n; i++)
    {
        if (s->ranges[i] > s->range_min && s->ranges[i] < s->range_max)
            nb++;
    }
    scan = Eigen::MatrixXd(2, nb);
    if (!reverse_scan)
    {
        unsigned int k = 0;
        for (unsigned int i = 0; i != n; i++)
        {
            if (s->ranges[i] > s->range_min && s->ranges[i] < s->range_max)
            {
                scan(0, k) = s->angle_min + i * s->angle_increment;
                scan(1, k) = s->ranges[i];
                k++;
            }
        }
    }
    else
    {
        unsigned int k = 0;
        for (unsigned int i = 0; i != n; i++)
        {
            if (s->ranges[i] > s->range_min && s->ranges[i] < s->range_max)
            {
                scan(0, nb - 1 - k) = betweenMinusPiAndPlusPi(-s->angle_min - i * s->angle_increment);
                scan(1, nb - 1 - k) = s->ranges[i];
                k++;
            }
        }
    }

}

bool ObjectFinderNode::findobjectsCallback(FindObjects::Request& req, FindObjects::Response& res)
{
    if (scan.rows() < 2)
    {
        ROS_WARN("ObjectFinderNode findobjectsCallback : scan is empty");
        return false;
    }

    objf.setPolarScan(scan);
    // TODO BOR : Changement de repère Hokuyo -> Base_Frame
    Scan cartScan = objf.computeCartesianScan(req.xRobot, req.yRobot, req.thetaRobot);
    Scan cropedScan = objf.onTableOnly();

    ROS_INFO("Nb of points in croped scan : %d", cropedScan.cols());
    std::vector<Scan> vect = objf.clusterize();

    ROS_INFO("*****************************");
    ROS_INFO("Nb of detected clusters : %d", vect.size());

    objects.clear();
    for (unsigned int i = 0; i < vect.size(); i++)
    {
        KnownObject obj;
        obj.recognize(vect[i]);
        recordObjectForRviz(obj);
        ROS_INFO("Object %d", i);
        ROS_INFO_STREAM(obj.print());
    }

    publishForRviz();
    res.confidence = -1;
    return true;
}

void ObjectFinderNode::recordObjectForRviz(KnownObject obj)
{
    switch (obj.type) {
        case ROBOT:
            addToPointCloud(obj.scan,m_robotPointCloud);
            break;
        case FIGURE:
            addToPointCloud(obj.scan,m_figurePointCloud);
            break;
        case TOWER:
            addToPointCloud(obj.scan,m_towerPointCloud);
            break;
        case UFO:
            addToPointCloud(obj.scan,m_ufoPointCloud);
            break;
        default:
            break;
    }
}

void ObjectFinderNode::publishForRviz()
{
    ros::Time t = ros::Time::now();
    m_towerPointCloud.header.stamp = t;
    m_robotPointCloud.header.stamp = t;
    m_figurePointCloud.header.stamp = t;
    m_ufoPointCloud.header.stamp = t;

    m_towerPointCloud.header.frame_id = "base_link";
    m_robotPointCloud.header.frame_id = "base_link";
    m_figurePointCloud.header.frame_id = "base_link";
    m_ufoPointCloud.header.frame_id = "base_link";

    m_towerPublisher.publish(m_towerPointCloud);
    m_robotPublisher.publish(m_robotPointCloud);
    m_figurePublisher.publish(m_figurePointCloud);
    m_ufoPublisher.publish(m_ufoPointCloud);

    m_towerPointCloud = sensor_msgs::PointCloud();
    m_robotPointCloud = sensor_msgs::PointCloud();
    m_figurePointCloud = sensor_msgs::PointCloud();
    m_ufoPointCloud = sensor_msgs::PointCloud();
}

void ObjectFinderNode::addToPointCloud(Scan s, sensor_msgs::PointCloud c)
{
    unsigned int n = s.cols();
    for( unsigned int i=0 ; i < n ; i++ )
    {
        geometry_msgs::Point32 pi;
        pi.x = s(1,i);
        pi.y = s(2,i);
        //TODO WLA : mettre la hauteur du hokuyo qui va bien
        pi.z = 0.0;
        c.points.push_back(pi);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ObjectFinderNode");
    ObjectFinderNode node;

    node.go();

    return 0;
}
