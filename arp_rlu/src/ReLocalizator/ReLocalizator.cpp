/*
 * ReLocalizator.cpp
 *
 *  Created on: 30 mai 2011
 *      Author: Boris
 */

#include "ReLocalizator.hpp"

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
    std::cout << "  x : " << this->x << std::endl;
    std::cout << "  y : " << this->y << std::endl;

    switch (this->type)
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
    std::cout << "*****************************" << std::endl;
    std::cout << "Nb of TableCorners referenced :" << tableCorners.size() << std::endl;
    for (unsigned int i = 0; i < tableCorners.size(); i++)
    {
        std::cout << "TableCorner " << i << std::endl;
        tableCorners[i].print();
    }
    std::cout << "*****************************" << std::endl;
    std::cout << " " << std::endl;
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

    std::cout << "Nb of compatible TableCorner : " << compatibleCorners.size() << std::endl;
    for(unsigned int i = 0; i < compatibleCorners.size(); i++)
    {
        std::cout << "Compatible TableCorner " << i << std::endl;
        compatibleCorners[i].print();
    }

    if (compatibleCorners.size() == 0)
    {
        std::cout << "ReLocalizator selectTargetTableCorner : No compatible TableCorner found" << std::endl;
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

    std::cout << "ReLocalizator selectTargetCorner : Selected TableCorner :" << std::endl;
    target.print();
    std::cout << " with min_distance " << min_distance << std::endl;

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
            std::cout << "ReLocalizator chooseScanWindow : TableCorner type is NONE" << std::endl;
            return std::make_pair(0.0, 0.0);
    }

    angleMin = betweenMinusPiAndPlusPi(atan2(previousY - yPtMin, previousX - xPtMin) + PI);
    angleMax = betweenMinusPiAndPlusPi(atan2(previousY - yPtMax, previousX - xPtMax) + PI);

//    std::cout << "chooseScanWindow angleMin: " << angleMin << std::endl;
//    std::cout << "chooseScanWindow angleMax: " << angleMax << std::endl;

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
        std::cout << "ReLocalizator estimatePose : Detected Corner is invalid" << std::endl;
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
            std::cout << "ReLocalizator estimatePose : TargetCorner type is NONE" << std::endl;
            return;
    }

}
