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
    if (scan.rows() < 2)
    {
        scan = Eigen::MatrixXd::Zero(0, 0);
        ROS_WARN("ChessboardReader findRoyalFamilyCallback : scan is empty");
        return false;
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
    bool success = compute(req.color);

    res.figure1 = result.first;
    res.figure2 = result.second;
    if (success)
        res.confidence = 1.0;
    else
        res.confidence = -1.0;

    return true;
}

bool ChessboardReader::compute(std::string color)
{
    // default result
    result = std::make_pair(0, 2);

    if (cartScan.cols() == 0)
    {
        ROS_WARN("ChessboardReader compute : cartScan is empty");
        return false;
    }

    if (cartScan.rows() != 4)
    {
        ROS_WARN("ChessboardReader compute : cartScan.rows() != 4 => Scan is not in cartesian representation ?");
        return false;
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
            if (cartScan(3, i) > 0.310 && cartScan(3, i) < 0.650)
            {
                nbPoints(0) = nbPoints(0) + 1;
            }
            if (cartScan(3, i) > -0.030 && cartScan(3, i) < 0.310)
            {
                nbPoints(1) = nbPoints(1) + 1;
            }
            if (cartScan(3, i) > -0.370 && cartScan(3, i) < -0.030)
            {
                nbPoints(2) = nbPoints(2) + 1;
            }
            if (cartScan(3, i) > -0.710 && cartScan(3, i) < -0.370)
            {
                nbPoints(3) = nbPoints(3) + 1;
            }
            if (cartScan(3, i) > -1.050 && cartScan(3, i) < -0.710)
            {
                nbPoints(4) = nbPoints(4) + 1;
            }
        }
    }

    int idFirst;
    int idSecond;

    int i_max;
    int nbPoints_max = nbPoints.maxCoeff(&i_max);
    const int nbPointsMin = 5;
    if (nbPoints_max < nbPointsMin)
    {
        ROS_WARN("ChessboardReader compute : Not enough detected points on maximum case (%d)", nbPoints_max);
        return false;
    }
    idFirst = i_max;
    ROS_INFO("ChessboardReader compute : Case %d has maximum points (%d)", i_max, nbPoints_max);
    nbPoints(i_max) = -1;
    nbPoints_max = nbPoints.maxCoeff(&i_max);
    if (nbPoints_max < nbPointsMin)
    {
        ROS_WARN("ChessboardReader compute : Not enough detected points on second maximum case (%d)", nbPoints_max);
        if (idFirst == 4)
        {
            ROS_INFO("ChessboardReader compute : Case 4 is unacceptable. We choose (0,2)");
            return false;
        }
        if (idFirst != 0)
            idSecond = 0;
        else
            idSecond = 2;
        result = std::make_pair(idFirst, idSecond);
        return false;
    }
    idSecond = i_max;
    ROS_INFO("ChessboardReader compute : Case %d has second maximum points (%d)", i_max, nbPoints_max);

    if (idFirst == 4)
    {
        if (idSecond != 0)
            idFirst = 0;
        else
            idFirst = 2;
        ROS_INFO("ChessboardReader compute : Case 4 is unacceptable. We choose %d", idFirst);
    }
    if (idSecond == 4)
    {
        if (idFirst != 0)
            idSecond = 0;
        else
            idSecond = 2;
        ROS_INFO("ChessboardReader compute : Case 4 is unacceptable. We choose %d", idSecond);
        result = std::make_pair(idFirst, idSecond);
        return false;
    }

    result = std::make_pair(idFirst, idSecond);
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ChessboardReader");
    ChessboardReader node;

    node.go();

    return 0;
}
