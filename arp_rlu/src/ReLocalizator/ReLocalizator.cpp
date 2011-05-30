/*
 * ReLocalizator.cpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */

#include "ReLocalizator.hpp"
#include <iostream>

using namespace arp_rlu;
using namespace arp_math;

TableCorner::TableCorner() :
    x(0.0), y(0.0), type(NONE)
{

}

TableCorner::TableCorner(double x, double y, TableCornerType type) :
    x(x), y(y), type(type)
{

}

bool TableCorner::isVisibleFrom(double xLaser, double yLaser, double minAngle, double maxAngle)
{
    // D'abord, on regarde si le coin est compatible avec la ligne de vue
    double capSigth = betweenMinusPiAndPlusPi(atan2(yLaser - y, xLaser - x));

    switch (type)
    {
        case NORTH_WEST:
            if (capSigth > PI / 2.)
                return false;
            if (capSigth < 0.)
                return false;
            break;
        case NORTH_EAST:
            if (capSigth > 0.)
                return false;
            if (capSigth < -PI / 2.)
                return false;
            break;
        case SOUTH_EAST:
            if (capSigth > -PI / 2.)
                return false;
            break;
        case SOUTH_WEST:
            if (capSigth < PI / 2.)
                return false;
            break;
        default:
            return false;
    }

    // Ensuite on regarde si le coin est correctement visible dans la plage angulaire du Laser
    capSigth = betweenMinusPiAndPlusPi(capSigth + PI);

    if (betweenMinusPiAndPlusPi(maxAngle - minAngle) > 0)
    {
        if (betweenMinusPiAndPlusPi(capSigth - minAngle) < 0)
        {
            return false;
        }
        if (betweenMinusPiAndPlusPi(maxAngle - capSigth) < 0)
        {
            return false;
        }
    }

    return true;
}

void TableCorner::print()
{
    std::stringstream ss;
    ROS_INFO("  x : %f",this->x);
    ROS_INFO("  y : %f",this->y);

    switch (this->type)
    {
        case NORTH_WEST:
            ROS_INFO("  type : NORTH_WEST");
            break;
        case NORTH_EAST:
            ROS_INFO("  type : NORTH_EAST");
            break;
        case SOUTH_EAST:
            ROS_INFO("  type : SOUTH_EAST");
            break;
        case SOUTH_WEST:
            ROS_INFO("  type : SOUTH_WEST");
            break;
        default:
            ROS_INFO("  type : NONE");
            break;
    }
}

ReLocalizator::ReLocalizator() :
    estimatedX(0.0), estimatedY(0.0), estimatedTheta(0.0), quality(-1)
{
    /*

     _________________________________________________________
     |M        L|----------|I          H|----------|E        D|
     |          |K        J|            |G        F|          |
     |                                                        |
     |                                                        |
     |                                                        |
     |                                                        |
     |                                                        |
     |                                                        |
     |                                                        |
     |                                                        |
     |                                                        |
     |N_______                                         ______C|
     |O                                                      B|
     |                                                        |
     |P______________________________________________________A|

     */

    std::vector<TableCorner> vtc;

    //coin A
    TableCorner A = TableCorner(-1.500,1.050,NORTH_EAST);
    vtc.push_back(A);
    //coin C
    TableCorner C = TableCorner(-1.500, 0.628, NORTH_EAST);
    vtc.push_back(C);
    //coin D
    TableCorner D = TableCorner(-1.500,-1.050,NORTH_WEST);
    vtc.push_back(D);
    //coin M
    TableCorner M = TableCorner(1.500,-1.050,SOUTH_WEST);
    vtc.push_back(M);
    //coin N
    TableCorner N = TableCorner(1.500, 0.628, SOUTH_EAST);
    vtc.push_back(N);
    //coin P
    TableCorner P = TableCorner(1.500,1.050,SOUTH_EAST);
    vtc.push_back(P);

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
    ROS_INFO("*****************************");
    int n = tableCorners.size();
    ROS_INFO("Nb of TableCorners referenced : %d", n);
    for (unsigned int i = 0; i < tableCorners.size(); i++)
    {
        ROS_INFO("TableCorner %d", i);
        tableCorners[i].print();
    }
    ROS_INFO("*****************************");
    ROS_INFO(" ");
}

TableCorner ReLocalizator::selectTargetTableCorner()
{
    // On selectionne d'abord tout les TableCorner qui sont visibles
    std::vector<TableCorner> compatibleCorners;
    for (std::vector<TableCorner>::iterator it = tableCorners.begin(); it != tableCorners.end(); ++it)
    {
        if (it->isVisibleFrom(previousX, previousY, previousTheta, previousTheta + PI / 2.) || it->isVisibleFrom(
                previousX, previousY, previousTheta - PI / 2., previousTheta))
        {
            compatibleCorners.push_back(*it);
        }
    }

    ROS_INFO("Nb of compatible TableCorner : %d", compatibleCorners.size());
    for(unsigned int i = 0; i < compatibleCorners.size(); i++)
    {
        ROS_INFO("Compatible TableCorner %d", i);
        compatibleCorners[i].print();
    }

    if (compatibleCorners.size() == 0)
    {
        ROS_WARN("ReLocalizator selectTargetTableCorner : No compatible TableCorner found");
        return TableCorner();
    }

    // On sélectionne celui qui est le plus près
    double min_distance = 666.;
    TableCorner target;
    for (std::vector<TableCorner>::iterator it = compatibleCorners.begin(); it != compatibleCorners.end(); ++it)
    {
        double distance = sqrt((it->x - previousX) * (it->x - previousX) + (it->y - previousY) * (it->y - previousY));
        if (distance < min_distance)
        {
            min_distance = distance;
            target = *it;
        }
    }

    ROS_INFO("ReLocalizator selectTargetCorner : Selected TableCorner :");
    target.print();
    ROS_INFO(" with min_distance %f",min_distance);

    return target;
}

std::pair<double, double> ReLocalizator::chooseScanWindow(TableCorner target)
{
    if (target.type == NONE)
        return std::make_pair(0.0, 0.0);

    double angleMin;
    double angleMax;
    double xPtMin = target.x;
    double yPtMin = target.y;
    double xPtMax = target.x;
    double yPtMax = target.y;
    const double length_sgmt = 0.25;

    switch (target.type)
    {
        case NORTH_WEST:
            yPtMin = target.y + length_sgmt;
            xPtMax = target.x + length_sgmt;
            break;
        case NORTH_EAST:
            xPtMin = target.x + length_sgmt;
            yPtMax = target.y - length_sgmt;
            break;
        case SOUTH_EAST:
            yPtMin = target.y - length_sgmt;
            xPtMax = target.x - length_sgmt;
            break;
        case SOUTH_WEST:
            xPtMin = target.x - length_sgmt;
            yPtMax = target.y + length_sgmt;
            break;
        default:
            ROS_WARN("ReLocalizator chooseScanWindow : TableCorner type is NONE");
            return std::make_pair(0.0, 0.0);
    }

    angleMin = betweenMinusPiAndPlusPi(atan2(previousY - yPtMin, previousX - xPtMin) + PI);
    angleMax = betweenMinusPiAndPlusPi(atan2(previousY - yPtMax, previousX - xPtMax) + PI);

    return std::make_pair(betweenMinusPiAndPlusPi(angleMin - previousTheta),
            betweenMinusPiAndPlusPi(angleMax - previousTheta));
}

void ReLocalizator::estimatePose(Corner detected, TableCorner target)
{
    if (detected.d1 == 0. || detected.d2 == 0. )
    {
        estimatedX = previousX;
        estimatedY = previousY;
        estimatedTheta = previousTheta;
        quality = -1.;
        ROS_WARN("ReLocalizator estimatePose : Detected Corner is invalid");
        return;
    }

    switch(target.type)
    {
        case NORTH_WEST:
            estimatedX = target.x + detected.d1;
            estimatedY = target.y + detected.d2;
            estimatedTheta = betweenMinusPiAndPlusPi( PI - detected.alpha1);
            quality = 1.;
            return;
        case NORTH_EAST:
            estimatedX = target.x + detected.d2;
            estimatedY = target.y - detected.d1;
            estimatedTheta = betweenMinusPiAndPlusPi( PI - detected.alpha2);
            quality = 1.;
            return;
        case SOUTH_EAST:
            estimatedX = target.x - detected.d1;
            estimatedY = target.y - detected.d2;
            estimatedTheta = betweenMinusPiAndPlusPi( -detected.alpha1);
            quality = 1.;
            return;
        case SOUTH_WEST:
            estimatedX = target.x - detected.d2;
            estimatedY = target.y + detected.d1;
            estimatedTheta = betweenMinusPiAndPlusPi( -detected.alpha2);
            quality = 1.;
            return;
        default:
            estimatedX = previousX;
            estimatedY = previousY;
            estimatedTheta = previousTheta;
            quality = -1.;
            ROS_WARN("ReLocalizator estimatePose : TargetCorner type is NONE");
            return;
    }

}
