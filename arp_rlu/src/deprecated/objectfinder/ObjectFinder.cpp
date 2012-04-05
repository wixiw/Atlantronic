/*
 * ObjectFinder.cpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */

#include "ObjectFinder.hpp"
#include <iostream>
#include <ros/console.h>

using namespace arp_rlu;
using namespace Eigen;

ObjectFinder::ObjectFinder() :
    mf(3), xMinTable(-1.300), xMaxTable(1.300), yMinTable(-0.850), yMaxTable(1.000), clusterizeMinNbPoints(5),
            clusterizeStddevMax(0.1), kMeanThreshDisplacement(0.01), kMeanMaxIterations(10)
{
    // Pour les tests sur table
    xMaxTable = 1.3;
    xMinTable = 0.;
    yMaxTable = 1.;
    yMinTable = -1.;
}

ObjectFinder::~ObjectFinder()
{
}

void ObjectFinder::setPolarScan(Scan s)
{
    if (s.cols() == 0)
    {
        ROS_WARN("ObjectFinder setPolarScan : Scan is empty");
        polarScan = MatrixXd::Zero(0, 0);
        return;
    }

    if (s.rows() != 2)
    {
        ROS_WARN("ObjectFinder setPolarScan : Scan.rows() != 2 => Scan is not in polar representation ?");
        polarScan = MatrixXd::Zero(0, 0);
        return;
    }

    polarScan.resize(2, s.cols());
    polarScan = s;
}

void ObjectFinder::setCartesianScan(Scan s)
{
    if (s.cols() == 0)
    {
        ROS_WARN("ObjectFinder setCartesianScan : Scan is empty");
        cartesianScan = MatrixXd::Zero(0, 0);
        return;
    }

    if (s.rows() != 4)
    {
        ROS_WARN("ObjectFinder setCartesianScan : Scan.rows() != 4 => Scan is not in cartesian representation ?");
        cartesianScan = MatrixXd::Zero(0, 0);
        return;
    }

    cartesianScan.resize(4, s.cols());
    cartesianScan = s;
}

Scan ObjectFinder::computeCartesianScan(double xOnTable, double yOnTable, double thetaOnTable)
{
    unsigned int n = polarScan.cols();
//    std::cout << "ObjectFinder computeCartesianScan : polarScan" << std::endl << polarScan.transpose() << std::endl;
    if (n == 0)
    {
        ROS_WARN("ObjectFinder computeCartesianScan : Polar Scan is empty");
        return MatrixXd::Zero(0, 0);
    }

    cartesianScan.resize(4,n);
    cartesianScan = MatrixXd::Zero(4, n);
    for (unsigned int i = 0; i < n; i++)
    {
        cartesianScan(0, i) = polarScan(0, i);
        cartesianScan(1, i) = polarScan(1, i);
        cartesianScan(2, i) = xOnTable + polarScan(1, i) * cos(polarScan(0, i) + thetaOnTable);
        cartesianScan(3, i) = yOnTable + polarScan(1, i) * sin(polarScan(0, i) + thetaOnTable);
    }

    return cartesianScan;
}

Scan ObjectFinder::onTableOnly()
{
    unsigned int n = cartesianScan.cols();
    if (n == 0)
    {
        ROS_WARN("ObjectFinder onTableOnly : Scan is empty");
        return MatrixXd::Zero(0, 0);
    }
    if (cartesianScan.rows() != 4)
    {
        ROS_WARN("ObjectFinder onTableOnly : Scan.rows() != 4 => Scan is not in cartesian representation ?");
        return MatrixXd::Zero(0, 0);
    }

    unsigned int nb = 0;
    for (unsigned int i = 0; i < n; i++)
    {
        if (cartesianScan(2, i) > xMaxTable)
            continue;
        if (cartesianScan(2, i) < xMinTable)
            continue;
        if (cartesianScan(3, i) > yMaxTable)
            continue;
        if (cartesianScan(3, i) < yMinTable)
            continue;
        nb++;
    }

    cropedScan = MatrixXd(4, nb);
    nb = 0;
    for (unsigned int i = 0; i < n; i++)
    {
        if (cartesianScan(2, i) > xMaxTable)
            continue;
        if (cartesianScan(2, i) < xMinTable)
            continue;
        if (cartesianScan(3, i) > yMaxTable)
            continue;
        if (cartesianScan(3, i) < yMinTable)
            continue;
        cropedScan(0, nb) = cartesianScan(0, i);
        cropedScan(1, nb) = cartesianScan(1, i);
        cropedScan(2, nb) = cartesianScan(2, i);
        cropedScan(3, nb) = cartesianScan(3, i);
        nb++;
    }
    return cropedScan;
}

std::pair<Scan, Scan> ObjectFinder::kMeans(Scan s)
{
    // On verifie que le scan n'est pas débile
    unsigned int n = s.cols();
    if (n == 0)
    {
        ROS_WARN("ObjectFinder kMeans : Scan is empty");
        return std::make_pair(MatrixXd::Zero(0, 0), MatrixXd::Zero(0, 0));
    }
    if (s.rows() != 4)
    {
        ROS_WARN("ObjectFinder kMeans : Scan.rows() != 4 => Scan is not in cartesian representation ?");
        return std::make_pair(MatrixXd::Zero(0, 0), MatrixXd::Zero(0, 0));
    }
    if (n == 1)
    {
        ROS_WARN("ObjectFinder kMeans : Only one point in scan");
        return std::make_pair(s, MatrixXd::Zero(0, 0));
    }

    // On prend deux graines au hasard
    unsigned int idFirst = rand() % n;
    unsigned int idSecond = idFirst;
    while (idFirst == idSecond)
    {
        idSecond = rand() % n;
    }
    double xFirst = s(2, idFirst);
    double yFirst = s(3, idFirst);
    double xSecond = s(2, idSecond);
    double ySecond = s(3, idSecond);

    unsigned nbIt = 0;
    VectorXi attribution(n);
    while (nbIt < kMeanMaxIterations)
    {
        // On compte les iterations
        nbIt++;

        // on attribue chaque point à la graine la plus proche
        // et dans la même passe on calcule les nouvelles graines
        double newXFirst = 0;
        double newYFirst = 0;
        double newXSecond = 0;
        double newYSecond = 0;
        for (unsigned int i = 0; i < n; i++)
        {
            double distFirst = sqrt((xFirst - s(2, i)) * (xFirst - s(2, i)) + (yFirst - s(3, i)) * (yFirst - s(3, i)));
            double distSecond = sqrt(
                    (xSecond - s(2, i)) * (xSecond - s(2, i)) + (ySecond - s(3, i)) * (ySecond - s(3, i)));
            if (distFirst < distSecond)
            {
                attribution(i) = 0;
                newXFirst += s(2, i);
                newYFirst += s(3, i);
            }
            else
            {
                attribution(i) = 1;
                newXSecond += s(2, i);
                newYSecond += s(3, i);
            }
        }
        newXFirst = newXFirst / (n - attribution.sum());
        newYFirst = newYFirst / (n - attribution.sum());
        newXSecond = newXSecond / attribution.sum();
        newYSecond = newYSecond / attribution.sum();

        double dispFirst = sqrt(
                (xFirst - newXFirst) * (xFirst - newXFirst) + (yFirst - newYFirst) * (yFirst - newYFirst));
        double dispSecond = sqrt(
                (xSecond - newXSecond) * (xSecond - newXSecond) + (ySecond - newYSecond) * (ySecond - newYSecond));
        xFirst = newXFirst;
        yFirst = newYFirst;
        xSecond = newXSecond;
        ySecond = newYSecond;

        if (dispFirst < kMeanThreshDisplacement && dispSecond < kMeanThreshDisplacement)
        {
            ROS_INFO("ObjectFinder kMeans : Break on displacement threshold (%f)", kMeanThreshDisplacement);
            break;
        }
    }
    if (nbIt == kMeanMaxIterations)
    {
        ROS_WARN("ObjectFinder kMeans : Convergence failed => max iteration (%d) reached !", kMeanMaxIterations);
        return std::make_pair(s, MatrixXd::Zero(0, 0));
    }

    ROS_INFO("ObjectFinder kMeans : nbIt=%d", nbIt);
    ROS_INFO("First cluster :  x=%f  y=%f", xFirst, yFirst);
    ROS_INFO("Second cluster :  x=%f  y=%f", xSecond, ySecond);

    // On met en forme les scans
    MatrixXd scanFirst(4, (n - attribution.sum()));
    MatrixXd scanSecond(4, attribution.sum());
    idFirst = 0;
    idSecond = 0;
    for (unsigned int i = 0; i < n; i++)
    {
        if (attribution(i) == 0)
        {
            scanFirst(0, idFirst) = s(0, i);
            scanFirst(1, idFirst) = s(1, i);
            scanFirst(2, idFirst) = s(2, i);
            scanFirst(3, idFirst) = s(3, i);
            idFirst++;
        }
        else
        {
            scanSecond(0, idSecond) = s(0, i);
            scanSecond(1, idSecond) = s(1, i);
            scanSecond(2, idSecond) = s(2, i);
            scanSecond(3, idSecond) = s(3, i);
            idSecond++;
        }
    }
    ROS_INFO("Scans OK");

    return std::make_pair(scanFirst, scanSecond);
}

std::vector<Scan> ObjectFinder::clusterize()
{
    return clusterize(cropedScan);
}

std::vector<Scan> ObjectFinder::clusterize(Scan s)
{
    std::vector<Scan> result;
    result.clear();

    // On verifie que le scan n'est pas débile
    unsigned int n = s.cols();
    if (n == 0)
    {
        //ROS_WARN("ObjectFinder clusterize : Scan is empty");
        return result;
    }
    if (s.rows() != 4)
    {
        ROS_WARN("ObjectFinder clusterize : Scan.rows() != 4 => Scan is not in cartesian representation ?");
        return result;
    }
    if (n < 5)
    {
        ROS_WARN("ObjectFinder clusterize : Less than %d points in scan", clusterizeMinNbPoints);
        return result;
    }

    // on check les stats
    double xMean = s.block(2, 0, 1, n).sum() / n;
    double yMean = s.block(3, 0, 1, n).sum() / n;
    double stddev = 0.;
    for (unsigned int i = 0; i < n; i++)
    {
        stddev += (xMean - s(2, i)) * (xMean - s(2, i)) + (yMean - s(3, i)) * (yMean - s(3, i));
    }
    stddev = sqrt(stddev / n);

    ROS_INFO("Clusterize : %d points with stddev=%f (compared to %f)", n , stddev, clusterizeStddevMax);

    if (stddev < clusterizeStddevMax)
    {
        result.push_back(s);
        return result;
    }

    std::pair<Scan, Scan> p = kMeans(s);

    result = clusterize(p.first);
    std::vector<Scan> right = clusterize(p.second);

    result.insert(result.end(), right.begin(), right.end());

    return result;
}
