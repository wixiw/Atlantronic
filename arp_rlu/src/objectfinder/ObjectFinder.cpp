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
    mf(3), xMinTable(-1.300), xMaxTable(1.300), yMinTable(-0.850), yMaxTable(1.000)
{

}

ObjectFinder::~ObjectFinder()
{
}

void ObjectFinder::setPolarScan(Scan s)
{
    if (s.cols() == 0)
    {
        ROS_WARN("ObjectFinder setPolarScan : Scan is empty");
        scan = MatrixXd::Zero(0, 0);
        return;
    }

    if (s.rows() != 2)
    {
        ROS_WARN("ObjectFinder setPolarScan : Scan.rows() != 2 => Scan is not in polar representation ?");
        scan = MatrixXd::Zero(0, 0);
        return;
    }

    scan = MatrixXd(2, s.cols());
    scan = s;
}

void ObjectFinder::setCartesianScan(Scan s)
{
    if (s.cols() == 0)
    {
        ROS_WARN("ObjectFinder setCartesianScan : Scan is empty");
        scan = MatrixXd::Zero(0, 0);
        return;
    }

    if (s.rows() != 4)
    {
        ROS_WARN("ObjectFinder setCartesianScan : Scan.rows() != 4 => Scan is not in cartesian representation ?");
        scan = MatrixXd::Zero(0, 0);
        return;
    }

    scan = MatrixXd(4, s.cols());
    scan = s;
}

Scan ObjectFinder::computeCartesianScan(double xOnTable, double yOnTable, double thetaOnTable)
{
    unsigned int n = scan.cols();
    if (n == 0)
    {
        ROS_WARN("ObjectFinder computeCartesianScan : Polar Scan is empty");
        return MatrixXd::Zero(0, 0);
    }

    Scan cartScan = MatrixXd(4, n);
    for (unsigned int i = 0; i < n; i++)
    {
        cartScan(0, i) = scan(0, i);
        cartScan(1, i) = scan(1, i);
        cartScan(2, i) = xOnTable + scan(1, i) * cos(scan(0, i) + thetaOnTable);
        cartScan(3, i) = yOnTable + scan(1, i) * sin(scan(0, i) + thetaOnTable);
    }
    scan = MatrixXd(4, n);
    scan = cartScan;
    return scan;
}

Scan ObjectFinder::onTableOnly()
{
    unsigned int n = scan.cols();
    if (n == 0)
    {
        ROS_WARN("ObjectFinder onTableOnly : Scan is empty");
        return MatrixXd::Zero(0, 0);
    }
    if (scan.rows() != 4)
    {
        ROS_WARN("ObjectFinder onTableOnly : Scan.rows() != 4 => Scan is not in cartesian representation ?");
        return MatrixXd::Zero(0, 0);
    }

    unsigned int nb = 0;
    for (unsigned int i = 0; i < n; i++)
    {
        if (scan(2, i) > xMaxTable)
            continue;
        if (scan(2, i) < xMinTable)
            continue;
        if (scan(3, i) > yMaxTable)
            continue;
        if (scan(3, i) < yMinTable)
            continue;
        nb++;
    }

    Scan s = MatrixXd(4, nb);
    nb = 0;
    for (unsigned int i = 0; i < n; i++)
    {
        if (scan(2, i) > xMaxTable)
            continue;
        if (scan(2, i) < xMinTable)
            continue;
        if (scan(3, i) > yMaxTable)
            continue;
        if (scan(3, i) < yMinTable)
            continue;
        s(0, nb) = scan(0, i);
        s(1, nb) = scan(1, i);
        s(2, nb) = scan(2, i);
        s(3, nb) = scan(3, i);
        nb++;
    }
    scan = MatrixXd(4, n);
    scan = s;
    return scan;
}

std::pair<Scan, Scan> ObjectFinder::kMeans(Scan s, double threshDisplacement, unsigned int maxIterations)
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
    srand(time(NULL));
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
    while (nbIt < maxIterations)
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

        if (dispFirst < threshDisplacement && dispSecond < threshDisplacement)
        {
            ROS_INFO("ObjectFinder kMeans : Break on displacement threshold");
            break;
        }
    }
    if (nbIt == maxIterations)
    {
        ROS_WARN("ObjectFinder kMeans : Convergence failed => max iteration (%d) reached !", maxIterations);
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

std::vector<Scan> clusterize(Scan s, unsigned int minNbPoints, double stddev)
{
    std::vector<Scan> result;
    result.clear();

    // On verifie que le scan n'est pas débile
    unsigned int n = s.cols();
    if (n == 0)
    {
        ROS_WARN("ObjectFinder clusterize : Scan is empty");
        return result;
    }
    if (s.rows() != 4)
    {
        ROS_WARN("ObjectFinder clusterize : Scan.rows() != 4 => Scan is not in cartesian representation ?");
        return result;
    }
    if (n < 5)
    {
        ROS_WARN("ObjectFinder clusterize : Less than %d points in scan", minNbPoints);
        return result;
    }

    return result;
}
