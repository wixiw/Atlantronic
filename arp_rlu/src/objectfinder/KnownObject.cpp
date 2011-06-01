/*
 * KnownObject.cpp
 *
 *  Created on: 31 mai 2011
 *      Author: Boris
 */

#include "objectfinder/KnownObject.hpp"
#include <iostream>
#include <ros/console.h>

using namespace arp_rlu;
using namespace Eigen;

KnownObject::KnownObject() :
    type(NONE), x(0.), y(0.), confidence(-1.), diameter(0.)
{

}

std::string KnownObject::print()
{
    std::stringstream ss;
    switch(type)
    {
        case NONE:
            ss << "  type :       NONE" << std::endl;
            break;
        case UFO:
            ss << "  type :       UFO" << std::endl;
            break;
        case FIGURE:
            ss << "  type :       FIGURE" << std::endl;
            break;
        case TOWER:
            ss << "  type :       TOWER" << std::endl;
            break;
        case ROBOT:
            ss << "  type :       ROBOT" << std::endl;
            break;
    }
    ss << "  x :          " << x << std::endl;
    ss << "  y :          " << y << std::endl;
    ss << "  diameter :   " << diameter << std::endl;
    ss << "  size :       " << scan.cols() << std::endl;
    ss << "  confidence : " << confidence << std::endl;

    return ss.str();
}

void KnownObject::recognize(Scan s)
{
    unsigned int n = s.cols();
    if (n == 0)
    {
        ROS_WARN("KnownObject recognize : Scan is empty");
        scan = MatrixXd::Zero(0, 0);
        return ;
    }
    if (s.rows() != 4)
    {
        ROS_WARN("KnownObject recognize : Scan.rows() != 4 => Scan is not in cartesian representation ?");
        scan = MatrixXd::Zero(0, 0);
        return ;
    }
    if (n == 1)
    {
        ROS_WARN("KnownObject recognize : Only one point in scan");
        scan = s;
        x = s(2,0);
        y = s(3,0);
        diameter = 0.;
        confidence = -1.;
        return;
    }

    scan = s;

    // Compute mean and stddev
    x = s.block(2, 0, 1, n).sum() / n;
    y = s.block(3, 0, 1, n).sum() / n;
    diameter = 0.;
    for (unsigned int i = 0; i < n; i++)
    {
        diameter += (x - s(2, i)) * (x - s(2, i)) + (y - s(3, i)) * (y - s(3, i));
    }
    diameter = 3 * sqrt(diameter / n);

    // Recognise object
    type = UFO;
    if( diameter > diameter_tower_min && diameter < diameter_tower_max)
    {
        type = TOWER;
        confidence = 1.;
        return;
    }
    if( diameter > diameter_figure_min && diameter < diameter_figure_max)
    {
        type = FIGURE;
        confidence = 1.;
        return;
    }
    if( diameter > diameter_robot_min)
    {
        type = ROBOT;
        confidence = 1.;
        return;
    }
}
