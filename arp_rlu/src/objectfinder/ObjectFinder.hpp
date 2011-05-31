/*
 * ObjectFinder.hpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */

#ifndef _ARP_RLU_OBJECTFINDER_HPP_
#define _ARP_RLU_OBJECTFINDER_HPP_

#include "lasertoolbox/MedianFilter.hpp"

#include <string>
#include <vector>

namespace arp_rlu
{

class ObjectFinder
{
    public:
        ObjectFinder();
        ~ObjectFinder();

        void setPolarScan(Scan s);
        Scan computeCartesianScan(double xOnTable, double yOnTable, double thetaOnTable);
        void setCartesianScan(Scan s);
        Scan onTableOnly();
        std::pair<Scan, Scan> kMeans(Scan s);

        std::vector<Scan> clusterize(Scan);

        double xMinTable;
        double xMaxTable;
        double yMinTable;
        double yMaxTable;

        unsigned int clusterizeMinNbPoints;
        double clusterizeStddevMax;
        double kMeanThreshDisplacement;
        unsigned int kMeanMaxIterations;

    protected:
        MedianFilter mf;
        Scan scan;
};
}

#endif /* _ARP_RLU_OBJECTFINDER_HPP_ */
