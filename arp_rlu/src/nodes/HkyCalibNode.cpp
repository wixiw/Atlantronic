/*
 * HkyCalibNode.cpp
 *
 *  Created on: 13 avril 2012
 *      Author: Boris
 */

#include <fstream>

#include <ros/ros.h>
#include "HkyCalibNode.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/lexical_cast.hpp>

using namespace arp_rlu;

HkyCalibNode::HkyCalibNode() :
            nh()
{
    m_scanSubscriber = nh.subscribe("scan", 1, &HkyCalibNode::scanCallback, this);

    calib_srv = nh.advertiseService("/HkyCalib/do",
            &HkyCalibNode::serviceCallback, this);
}

HkyCalibNode::~HkyCalibNode()
{
}

void HkyCalibNode::scanCallback(sensor_msgs::LaserScanConstPtr scan)
{
    lastScan = lsl::LaserScan();
    Eigen::MatrixXd polarData(3, scan->ranges.size());

    //ROS_WARN("min : %f max : %f",scan->range_min, scan->range_max);
    for (unsigned int i = 0; i != scan->ranges.size(); i++)
    {
        polarData(0,i) = scan->header.stamp.toSec() + i * scan->time_increment;
        polarData(2,i) = scan->angle_min + i*scan->angle_increment;
        if (scan->ranges[i] <= scan->range_max && scan->range_min <= scan->ranges[i])
        {
            polarData(1,i) = scan->ranges[i];
        }
        else
        {
            polarData(1,i) = 0.;
        }
    }
    lastScan.setPolarData(polarData);
}

bool HkyCalibNode::serviceCallback(HkyCalib::Request& req, HkyCalib::Response& res)
{
    ROS_INFO("*****************************");
    ROS_INFO("Scan size : %d", lastScan.getSize());
    Eigen::VectorXd tt = lastScan.getTimeData();
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(lastScan.getSize());
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(lastScan.getSize());
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(lastScan.getSize());
    beaconDetector.process(lastScan, tt, xx, yy, hh);

    std::vector< lsl::DetectedObject > dObjs = beaconDetector.getDetectedObjects();
    std::vector< lsl::DetectedCircle > dCircles = beaconDetector.getDetectedCircles();
    ROS_INFO("dObjs size : %d", dObjs.size());
    ROS_INFO("dCircles size : %d", dCircles.size());

    double xTarget = req.distance * 0.01;
    int xpIndex = req.xp;

    lsl::DetectedObject bestObj;
    for(unsigned int i = 0 ; i < dObjs.size() ; i++)
    {
//        ROS_INFO("dObjs[%d]", i);
//        ROS_INFO("  * xMean = %f", dObjs[i].getCartesianMean()(0));
//        ROS_INFO("  * yMean = %f", dObjs[i].getCartesianMean()(1));
        if( (dObjs[i].getCartesianMean() - Eigen::Vector2d(xTarget, 0.)).norm() < 0.2)
        {
            bestObj = dObjs[i];
//            ROS_INFO("  => retenu !");
        }
    }
    lsl::DetectedCircle bestCirc;
    for(unsigned int i = 0 ; i < dCircles.size() ; i++)
    {
        if( (dCircles[i].getCartesianMean() - Eigen::Vector2d(xTarget, 0.)).norm() < 0.2)
        {
            bestCirc = dCircles[i];
        }
    }

    boost::property_tree::ptree pt;
    pt.put("distance", xTarget);
    pt.put("xp", xpIndex);

    boost::property_tree::ptree ptObj;
    ptObj.put("size", bestObj.getScan().getSize());
    if(bestObj.getScan().getSize() > 0)
    {
        ptObj.put("minT", bestObj.getScan().getCartesianData().row(0).minCoeff());
        ptObj.put("maxT", bestObj.getScan().getCartesianData().row(0).maxCoeff());
        ptObj.put("minX", bestObj.getScan().getCartesianData().row(3).minCoeff());
        ptObj.put("maxX", bestObj.getScan().getCartesianData().row(3).maxCoeff());
        ptObj.put("minY", bestObj.getScan().getCartesianData().row(4).minCoeff());
        ptObj.put("maxY", bestObj.getScan().getCartesianData().row(4).maxCoeff());
    }
    ptObj.put("xMean", bestObj.getCartesianMean()(0));
    ptObj.put("yMean", bestObj.getCartesianMean()(1));
    ptObj.put("rho", bestObj.getApparentCartesianMeanRange());
    ptObj.put("theta", bestObj.getApparentCartesianMeanTheta());
    ptObj.put("time", bestObj.getApparentCartesianMeanTime());
    pt.add_child("DetectedObject", ptObj);

    boost::property_tree::ptree ptCirc;
    ptCirc.put("size", bestCirc.getScan().getSize());
    ptCirc.put("x", bestCirc.x());
    ptCirc.put("y", bestCirc.y());
    ptCirc.put("r", bestCirc.r());
    ptCirc.put("CenterRange", bestCirc.getApparentCenterRange());
    ptCirc.put("CenterTheta", bestCirc.getApparentCenterTheta());
    pt.add_child("DetectedCircle", ptCirc);

    std::ofstream jsonFile;
    jsonFile.open( ("LSI1_" + boost::lexical_cast<std::string>((int)(100*xTarget)) + "x" + boost::lexical_cast<std::string>(xpIndex) + ".json").c_str());
    boost::property_tree::json_parser::write_json(jsonFile, pt);
    jsonFile.close();

    return true;
}

void HkyCalibNode::go()
{
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "HkyCalibNode");
    HkyCalibNode node;

    node.go();

    return 0;
}
