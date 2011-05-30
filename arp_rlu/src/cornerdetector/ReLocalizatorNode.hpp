/*
 * CornerDetectorNode.hpp
 *
 *  Created on: 29 mai 2011
 *      Author: ard
 */

#ifndef _ARP_RLU_RELOCALIZATORNODE_HPP_
#define _ARP_RLU_RELOCALIZATORNODE_HPP_

#include "CornerDetector.hpp"

#include <arp_rlu/DetectCorner.h>
#include <arp_rlu/EstimatePosition.h>
#include <string>
#include <vector>

namespace arp_rlu
{
enum TableCornerType
{
    NONE,
    NORTH_WEST,
    NORTH_EAST,
    SOUTH_EAST,
    SOUTH_WEST
};

class TableCorner
{

    public:
        TableCorner();

        double x;
        double y;
        TableCornerType type;

    public:
        bool isVisibleFrom(double x, double y, double minAngle, double maxAngle);

};

class ReLocalizatorNode
{
    public:
        ReLocalizatorNode();
        ~ReLocalizatorNode();

        void go();

        void setTableCorners(std::vector<TableCorner>);
        void printTableCorners();

    protected:
        /**
         * NodeHandle on associated node
         */
        ros::NodeHandle nh;


        /**
         * Used to provide DetectCorner service
         */
        ros::ServiceServer estimatePosition_srv;

        /**
         * Called when DtectCorner service is called
         * \returns success boolean
         */
        bool estimatePositionCallback(arp_rlu::EstimatePosition::Request& req, arp_rlu::EstimatePosition::Response& res);

        std::vector<TableCorner> tableCorners;

        double previousX;
        double previousY;
        double previousTheta;

        double estimatedX;
        double estimatedY;
        double estimatedTheta;
        double quality;

        TableCorner selectTargetTableCorner();
        std::pair<double, double> chooseScanWindow(TableCorner);
        void estimatePose(arp_rlu::Corner, TableCorner);

};
}

#endif /* _ARP_RLU_RELOCALIZATORNODE_HPP_ */
