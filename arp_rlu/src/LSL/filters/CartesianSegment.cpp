/*
 * CartesianSegment.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include <iostream>

#include "CartesianSegment.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

CartesianSegment::Params::Params()
: kmeansMaxIterations(10)
, kmeansDispThres(0.01)
, minNbPoints(5)
, maxStddev(0.1)
{
}

std::string CartesianSegment::Params::getInfo()
{
    std::stringstream ss;
    ss << "CartesianSegment params :" << std::endl;
    ss << " [*] kmeansMaxIterations : " << kmeansMaxIterations << std::endl;
    ss << " [*] kmeansDispThres     : " << kmeansDispThres << " (m)"<< std::endl;
    ss << " [*] minNbPoints         : " << minNbPoints << std::endl;
    ss << " [*] maxStddev           : " << maxStddev << " (m)"<< std::endl;
    return ss.str();
}

bool CartesianSegment::Params::checkConsistency() const
{
    if(kmeansMaxIterations == 0)
        return false;
    if(kmeansDispThres <= 0.)
        return false;
    if(minNbPoints == 0)
        return false;
    if(maxStddev <= 0.)
        return false;
    return true;
}

std::vector<LaserScan> CartesianSegment::apply(const LaserScan & raw, const Params & params)
{
    if(!params.checkConsistency())
    {
        std::vector<LaserScan> out;
        out.push_back(raw);
        return out;
    }
    if(!raw.areCartesianDataAvailable())
    {
        std::vector<LaserScan> out;
        out.push_back(raw);
        return out;
    }
    if(raw.getSize() < params.minNbPoints)
    {
        std::vector<LaserScan> out;
        return out;
    }

    // on check les stats
    MatrixXd cart = raw.getCartesianData();
    double xMean = arp_math::mean(cart.row(1));
    double yMean = arp_math::mean(cart.row(2));
    double stddev = 0.;
    for (unsigned int i = 0; i < raw.getSize(); i++)
    {
        stddev += (xMean - cart(1, i)) * (xMean - cart(1, i)) + (yMean - cart(2, i)) * (yMean - cart(2, i));
    }
    stddev = sqrt(stddev / raw.getSize());

    if (stddev < params.maxStddev)
    {
        std::vector<LaserScan> out;
        out.push_back(raw);
        return out;
    }

    std::pair<LaserScan, LaserScan> pp = kMeans(raw, params);

    std::vector<LaserScan> out;
    out = CartesianSegment::apply(pp.first, params);
    std::vector<LaserScan> right = CartesianSegment::apply(pp.second, params);

    out.insert(out.end(), right.begin(), right.end());

    return out;
}


std::pair<LaserScan, LaserScan> CartesianSegment::kMeans(const LaserScan & s, const Params & p)
{
    if(!p.checkConsistency())
    {
        return make_pair(s, LaserScan());
    }
    if(!s.areCartesianDataAvailable())
    {
        return make_pair(s, LaserScan());
    }
    if(s.getSize()< 2)
    {
        return make_pair(s, LaserScan());
    }

    // On prend deux graines au hasard
    MatrixXd cart = s.getCartesianData();
    MatrixXd polar = s.getPolarData();
    unsigned int n = s.getSize();
    unsigned int idFirst = rand() % n;
    unsigned int idSecond = idFirst;
    while (idFirst == idSecond)
    {
        idSecond = rand() % n;
    }
    double xFirst = cart(1, idFirst);
    double yFirst = cart(2, idFirst);
    double xSecond = cart(1, idSecond);
    double ySecond = cart(2, idSecond);

    unsigned nbIt = 0;
    VectorXi attribution(n);
    while (nbIt < p.kmeansMaxIterations)
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
            double distFirst = sqrt((xFirst - cart(1, i)) * (xFirst - cart(1, i)) + (yFirst - cart(2, i)) * (yFirst - cart(2, i)));
            double distSecond = sqrt(
                    (xSecond - cart(1, i)) * (xSecond - cart(1, i)) + (ySecond - cart(2, i)) * (ySecond - cart(2, i)));
            if (distFirst < distSecond)
            {
                attribution(i) = 0;
                newXFirst += cart(1, i);
                newYFirst += cart(2, i);
            }
            else
            {
                attribution(i) = 1;
                newXSecond += cart(1, i);
                newYSecond += cart(2, i);
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

        if (dispFirst < p.kmeansDispThres && dispSecond < p.kmeansDispThres)
        {
            break;
        }
    }
    if (nbIt == p.kmeansMaxIterations)
    {
        return std::make_pair(s, LaserScan());
    }

    // On met en forme les scans
    MatrixXd polarFirst(3, (n - attribution.sum()));
    MatrixXd polarSecond(3, attribution.sum());
    MatrixXd cartFirst(6, (n - attribution.sum()));
    MatrixXd cartSecond(6, attribution.sum());
    idFirst = 0;
    idSecond = 0;
    for (unsigned int i = 0; i < n; i++)
    {
        if (attribution(i) == 0)
        {
            polarFirst.col(idFirst) = polar.col(i);
            cartFirst.col(idFirst) = cart.col(i);
            idFirst++;
        }
        else
        {
            polarSecond.col(idSecond) = polar.col(i);
            cartSecond.col(idSecond) = cart.col(i);
            idSecond++;
        }
    }

    LaserScan scanFirst;
    scanFirst.setPolarData(polarFirst);
    scanFirst.computeCartesianData(cart.row(0), cart.row(3), cart.row(4), cart.row(5));
    LaserScan scanSecond;
    scanSecond.setPolarData(polarSecond);
    scanSecond.computeCartesianData(cart.row(0), cart.row(3), cart.row(4), cart.row(5));

    return std::make_pair(scanFirst, scanSecond);
}
