/*
 * .cpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */

#include <ros/ros.h>
#include "objectfinder/ChessboardReader.hpp"

using namespace arp_rlu;
using namespace sensor_msgs;
using namespace arp_math;

ChessboardReader::ChessboardReader() :
    nh(), scan(Eigen::MatrixXd::Zero(1, 1))
{

    scan_sub = nh.subscribe("/scan", 1, &ChessboardReader::scanCallback, this);
    findroyalfamily_srv = nh.advertiseService("/ChessboardReader/FindRoyalFamily", &ChessboardReader::findRoyalFamilyCallback, this);

    srand(time(NULL));
}

ChessboardReader::~ChessboardReader()
{
}

void ChessboardReader::go()
{
    ros::spin();
}

void ChessboardReader::scanCallback(LaserScanConstPtr s)
{
    unsigned int n = s->ranges.size();
    unsigned int nb = 0;
    for (unsigned int i = 0; i != n; i++)
    {
        if (s->ranges[i] > s->range_min && s->ranges[i] < s->range_max)
            nb++;
    }
    scan = Eigen::MatrixXd(2, nb);
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

bool ChessboardReader::findRoyalFamilyCallback(FindRoyalFamily::Request& req, FindRoyalFamily::Response& res)
{

    int idFirst = rand() % 4;
    int idSecond = idFirst;
    while (idFirst == idSecond)
    {
        idSecond = rand() % 4;
    }

    res.figure1 = idFirst;
    res.figure2 = idSecond;
    res.confidence = 1.0;

//    if (scan.rows() < 2)
//    {
//        ROS_WARN("ChessboardReader findRoyalFamilyCallback : scan is empty");
//        return false;
//    }
//
//    objf.setPolarScan(scan);
//    // TODO BOR : Changement de repère Hokuyo -> Base_Frame
//    Scan cartScan = objf.computeCartesianScan(req.xRobot, req.yRobot, req.thetaRobot);
//    Scan cropedScan = objf.onTableOnly();
//
//    ROS_INFO("Nb of points in croped scan : %d", cropedScan.cols());
//    std::vector<Scan> vect = objf.clusterize();
//
//    ROS_INFO("*****************************");
//    ROS_INFO("Nb of detected clusters : %d", vect.size());
//
//    objects.clear();
//    for (unsigned int i = 0; i < vect.size(); i++)
//    {
//        KnownObject obj;
//        obj.recognize(vect[i]);
//        ROS_INFO("Object %d", i);
//        ROS_INFO_STREAM(obj.print());
//    }
//
//    res.confidence = -1;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ChessboardReader");
    ChessboardReader node;

    node.go();

    return 0;
}
