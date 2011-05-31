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

        KnownObjectType type;
        Scan scan;
        double x;
        double y;
        double confidence;
};
}

#endif /* _ARP_RLU_KNOWNOBJECT_HPP_ */
