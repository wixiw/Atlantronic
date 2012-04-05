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

    scan_sub = nh.subscribe("/front_scan", 1, &ChessboardReader::scanCallback, this);
    findroyalfamily_srv = nh.advertiseService("/ChessboardReader/FindRoyalFamily",
            &ChessboardReader::findRoyalFamilyCallback, this);

    cartScan = Eigen::MatrixXd::Zero(0, 0);

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
    scan = Eigen::MatrixXd::Zero(2, nb);
    if (nb < 1)
        return;
    unsigned int k = nb - 1;
    for (unsigned int i = 0; i != n; i++)
    {
        if (s->ranges[i] > s->range_min && s->ranges[i] < s->range_max)
        {
            scan(0, k) = betweenMinusPiAndPlusPi(-s->angle_min - i * s->angle_increment);
            scan(1, k) = s->ranges[i];
            k--;
        }
    }

}

bool ChessboardReader::findRoyalFamilyCallback(FindRoyalFamily::Request& req, FindRoyalFamily::Response& res)
{
    res.figure1 = 0;
    res.figure2 = 2;
    res.confidence1 = -1.0;
    res.confidence2 = -1.0;

    if (scan.rows() < 2)
    {
        scan = Eigen::MatrixXd::Zero(0, 0);
        ROS_WARN("ChessboardReader findRoyalFamilyCallback : scan is empty");
        return true;
    }

    objf.setPolarScan(scan);

    ROS_INFO("xRobot=%f  yRobot=%f  thetaRobot=%f", req.xRobot, req.yRobot, req.thetaRobot);
    ROS_INFO("nbPoints in scan : %d", scan.cols());

    const double frontal_deport = 0.26;
    const double lateral_deport = 0.053;
    double c = cos(req.thetaRobot);
    double s = sin(req.thetaRobot);
    double x = req.xRobot + frontal_deport * c - lateral_deport * s;
    double y = req.yRobot + frontal_deport * s + lateral_deport * c;
    double theta = req.thetaRobot;

    ROS_INFO("xLaser=%f  yLaser=%f  thetaLaser=%f", x, y, theta);

    cartScan = objf.computeCartesianScan(x, y, theta);
    std::pair<bool,bool> success = compute(req.color);

    res.figure1 = result.first;
    res.figure2 = result.second;

    if(result.first > 3 || result.first < 0 || result.second > 3 || result.second < 0 || result.first == result.second)
    {
        res.figure1 = 0;
        res.figure2 = 2;
    }


    if (success.first)
        res.confidence1 = 1.0;
    else
        res.confidence1 = -1.0;

    if (success.second)
        res.confidence2 = 1.0;
    else
        res.confidence2 = -1.0;

    return true;
}

std::pair<bool,bool> ChessboardReader::compute(std::string color)
{
    // default result
    result = std::make_pair(0, 2);

    if (cartScan.cols() == 0)
    {
        ROS_WARN("ChessboardReader compute : cartScan is empty. We choose (0, 2)");
        result = std::make_pair(0, 2);
        return std::make_pair(false,false);
    }

    if (cartScan.rows() != 4)
    {
        ROS_WARN("ChessboardReader compute : cartScan.rows() != 4 => Scan is not in cartesian representation ? We choose (0, 2)");
        result = std::make_pair(0, 2);
        return std::make_pair(false,false);
    }

    double xMin = -1.500;
    double xMax = -1.100;
    if (color.compare("blue") == 0)
    {
        xMin = 1.100;
        xMax = 1.500;
    }
    ROS_INFO("xMin : %f", xMin);
    ROS_INFO("xMax : %f", xMax);

    Eigen::VectorXi nbPoints = Eigen::VectorXi::Zero(5);
    for (unsigned int i = 0; i < cartScan.cols(); i++)
    {
        if (cartScan(2, i) > xMin && cartScan(2, i) < xMax)
        {
            if (cartScan(3, i) > 0.220 && cartScan(3, i) < 0.500)
            {
                nbPoints(0) = nbPoints(0) + 1;
            }
            if (cartScan(3, i) > -0.060 && cartScan(3, i) < 0.220)
            {
                nbPoints(1) = nbPoints(1) + 1;
            }
            if (cartScan(3, i) > -0.340 && cartScan(3, i) < -0.060)
            {
                nbPoints(2) = nbPoints(2) + 1;
            }
            if (cartScan(3, i) > -0.620 && cartScan(3, i) < -0.340)
            {
                nbPoints(3) = nbPoints(3) + 1;
            }
            if (cartScan(3, i) > -0.900 && cartScan(3, i) < -0.620)
            {
                nbPoints(4) = nbPoints(4) + 1;
            }
        }
    }

    ROS_INFO_STREAM("nbPoints :" << nbPoints.transpose() );

    int idFirst;
    int idSecond;

    // get first maximum
    int i_max;
    int nbPoints_max = nbPoints.maxCoeff(&i_max);
    const int nbPointsMin = 5;
    if (nbPoints_max < nbPointsMin)
    {
        ROS_WARN("ChessboardReader compute : Not enough detected points on maximum case (%d). We choose (0,2)", nbPoints_max);
        result = std::make_pair(0, 2);
        return std::make_pair(false,false);
    }
    idFirst = i_max;
    ROS_INFO("ChessboardReader compute : Case %d has maximum points (%d)", i_max, nbPoints_max);

    // get second maximum
    nbPoints(i_max) = -1;
    nbPoints_max = nbPoints.maxCoeff(&i_max);
    if (nbPoints_max < nbPointsMin)
    {
        ROS_WARN("ChessboardReader compute : Not enough detected points on second maximum case (%d)", nbPoints_max);
        if (idFirst == 4)
        {
            ROS_INFO("ChessboardReader compute : Case 4 is unacceptable for position 1. We choose (0,2)");
            result = std::make_pair(0, 2);
            return std::make_pair(false,false);
        }
        if (idFirst != 0)
            idSecond = 0;
        else
            idSecond = 2;
        result = std::make_pair(idFirst, idSecond);
        ROS_INFO("ChessboardReader compute : We choose (%d,%d)", idFirst, idSecond);
        return std::make_pair(true,false);
    }
    idSecond = i_max;
    ROS_INFO("ChessboardReader compute : Case %d has second maximum points (%d)", i_max, nbPoints_max);

    // we avoid the config 4
    if (idFirst == 4)
    {
        if (idSecond != 0)
            idFirst = 0;
        else
            idFirst = 2;
        ROS_INFO("ChessboardReader compute : Case 4 is unacceptable for position 1. We choose (%d,%d)", idFirst, idSecond);
        result = std::make_pair(idFirst, idSecond);
        return std::make_pair(false,true);
    }
    if (idSecond == 4)
    {
        if (idFirst != 0)
            idSecond = 0;
        else
            idSecond = 2;
        ROS_INFO("ChessboardReader compute : Case 4 is unacceptable for position 2. We choose (%d,%d)", idFirst, idSecond);
        result = std::make_pair(idFirst, idSecond);
        return std::make_pair(true,false);
    }

    result = std::make_pair(idFirst, idSecond);
    ROS_INFO("ChessboardReader compute : Everything is all rigth. We choose (%d,%d)", idFirst, idSecond);
    return std::make_pair(true,true);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ChessboardReader");
    ChessboardReader node;

    node.go();

    return 0;
}
