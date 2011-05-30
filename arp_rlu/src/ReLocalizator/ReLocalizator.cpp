/*
 * ReLocalizator.cpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */

#include "ReLocalizator.hpp"

using namespace arp_rlu;

TableCorner::TableCorner() :
    x(0.0), y(0.0), type(NONE)
{

}

bool TableCorner::isVisibleFrom(double xLaser, double yLaser, double minAngle, double maxAngle)
{
    bool result = false;

    // D'abord, on regarde si le coin est compatible
    double capSigth = atan2(yLaser - y, xLaser - x);

    // Ensuite on regarde si le coin est correctement visible dans la plage angulaire du Laser


    return result;
}

ReLocalizator::ReLocalizator() :
    estimatedX(0.0), estimatedY(0.0), estimatedTheta(0.0), quality(-1)
{
    std::vector<TableCorner> vtc;
    setTableCorners(vtc);
    printTableCorners();
}

ReLocalizator::~ReLocalizator()
{
}

void ReLocalizator::setTableCorners(std::vector<TableCorner> tc)
{
    tableCorners = tc;
}

void ReLocalizator::printTableCorners()
{
    std::cout << "*****************************" << std::endl;
    std::cout << "Nb of TableCorners :" << tableCorners.size() << std::endl;
    for (unsigned int i = 0; i < tableCorners.size(); i++)
    {
        std::cout << "TableCorner " << i << std::endl;
        std::cout << "  x : " << tableCorners[i].x << std::endl;
        std::cout << "  y : " << tableCorners[i].y << std::endl;

        switch (tableCorners[i].type)
        {
            case NORTH_WEST:
                std::cout << "  type : NORTH_WEST" << std::endl;
                break;
            case NORTH_EAST:
                std::cout << "  type : NORTH_EAST" << std::endl;
                break;
            case SOUTH_EAST:
                std::cout << "  type : SOUTH_EAST" << std::endl;
                break;
            case SOUTH_WEST:
                std::cout << "  type : SOUTH_WEST" << std::endl;
                break;
            default:
                std::cout << "  type : NONE" << std::endl;
                break;
        }

    }
}

TableCorner ReLocalizator::selectTargetTableCorner()
{
    return TableCorner();
}

std::pair<double, double> ReLocalizator::chooseScanWindow(TableCorner target)
{
    return std::make_pair(0.0, 0.0);
}

void ReLocalizator::estimatePose(Corner detected, TableCorner target)
{
    return;
}
