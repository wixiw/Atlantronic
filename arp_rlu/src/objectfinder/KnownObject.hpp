/*
 * KnownObject.hpp
 *
 *  Created on: 31 mai 2011
 *      Author: ard
 */

#ifndef _ARP_RLU_KNOWNOBJECT_HPP_
#define _ARP_RLU_KNOWNOBJECT_HPP_

#include <math/math.hpp>
#include "lasertoolbox/common.hpp"

#include <string>
#include <vector>

namespace arp_rlu
{
enum KnownObjectType
{
    NONE, UFO, FIGURE, TOWER, ROBOT
};

class KnownObject
{
    public:
        KnownObject();

        void recognize(Scan s);

        std::string print();

        static const double diameter_tower_min = 0.0;
        static const double diameter_tower_max = 0.1;

        static const double diameter_figure_min = 0.1;
        static const double diameter_figure_max = 0.2;

        static const double diameter_robot_min = 0.2;

        KnownObjectType type;
        Scan scan;
        double x;
        double y;
        double confidence;
        double diameter;  // = 3 sigma
};
}

#endif /* _ARP_RLU_KNOWNOBJECT_HPP_ */
