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
        ROS_WARN("Failed to get param 'scan_topic_name'. Take default value (/top_scan)");
        front_scan_topic_name = "/top_scan";
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

    m_towerPublisher = nh.advertise<PointCloud>("/ObjectFinder/display/pawn",1);
    m_robotPublisher = nh.advertise<PointCloud>("/ObjectFinder/display/robot",1);
    m_figurePublisher = nh.advertise<PointCloud>("/ObjectFinder/display/figure",1);
    m_ufoPublisher = nh.advertise<PointCloud>("/ObjectFinder/display/ovni",1);

    scan_sub = nh.subscribe(front_scan_topic_name, 1, &ObjectFinderNode::scanCallback, this);
    findobjects_srv = nh.advertiseService("/ObjectFinder/FindObjects", &ObjectFinderNode::findobjectsCallback,
            this);
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
    scan = Eigen::MatrixXd(2, n);
    if (!reverse_scan)
    {
        for (unsigned int i = 0; i != n; i++)
        {
            scan(0, i) = s->angle_min + i * s->angle_increment;
            scan(1, i) = s->ranges[i];
        }
    }
    else
    {
        for (unsigned int i = 0; i != n; i++)
        {
            scan(0, n-1-i) = betweenMinusPiAndPlusPi( - s->angle_min - i * s->angle_increment );
            scan(1, n-1-i) = s->ranges[i];
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
    objf.computeCartesianScan(req.xRobot, req.yRobot, req.thetaRobot);
    objf.onTableOnly();
    std::vector<Scan> vect = objf.clusterize();

    ROS_INFO("*****************************");
    ROS_INFO("Nb of detected clusters : %d", vect.size());

    objects.clear();
    for(unsigned int i = 0; i < vect.size() ; i++)
    {
        KnownObject obj;
        obj.recognize(vect[i]);
        recordObjectForRviz(obj);
        ROS_INFO("Object %d", i);
        ROS_INFO_STREAM( obj.print() );
    }

    publishForRviz();

//    cd.setScan(cropScan(req.minAngle, req.maxAngle));
//
//    Corner c = cd.compute();
//
//    res.d1 = c.d1;
//    res.alpha1 = c.alpha1;
//    res.length1 = c.length1;
//    res.angleBegin1 = c.angleBegin1;
//    res.angleEnd1 = c.angleEnd1;
//    res.d2 = c.d2;
//    res.alpha2 = c.alpha2;
//    res.length2 = c.length2;
//    res.angleBegin2 = c.angleBegin2;
//    res.angleEnd2 = c.angleEnd2;
//    res.diag = c.diag;
//    res.theta = c.theta;
//    res.cornerAngleInDeg = c.cornerAngle * 180. / PI;

    for(unsigned int i = 0; i < vect.size() ; i++)
    {
        ROS_INFO("Cluster %d - size: %d", i, vect[i].cols());
    }

    res.confidence = vect.size();
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
    //TODO WLA : mettre la frame ID
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
