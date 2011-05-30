/*
 * ReLocalizator.hpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */

#ifndef _ARP_RLU_RELOCALIZATOR_HPP_
#define _ARP_RLU_RELOCALIZATOR_HPP_

#include "cornerdetector/CornerDetector.hpp"

#include <arp_rlu/DetectCorner.h>
#include <arp_rlu/EstimatePosition.h>
#include <string>
#include <vector>

namespace arp_rlu
{
enum TableCornerType
{
    NONE, NORTH_WEST, NORTH_EAST, SOUTH_EAST, SOUTH_WEST
};

class TableCorner
{

    public:
        TableCorner();
        TableCorner(double x, double y, TableCornerType type);

        double x;
        double y;
        TableCornerType type;

    public:
        bool isVisibleFrom(double x, double y, double minAngle, double maxAngle);

};

class ReLocalizator
{
    public:
        ReLocalizator();
        ~ReLocalizator();

        std::vector<TableCorner> tableCorners;

        double previousX;
        double previousY;
        double previousTheta;

        double estimatedX;
        double estimatedY;
        double estimatedTheta;
        double quality;

        void setTableCorners(std::vector<TableCorner>);
        void printTableCorners();

        TableCorner selectTargetTableCorner();
        std::pair<double, double> chooseScanWindow(TableCorner);
        void estimatePose(arp_rlu::Corner, TableCorner);

};
}

#endif /* _ARP_RLU_RELOCALIZATOR_HPP_ */
