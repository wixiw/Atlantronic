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

    protected:
        MedianFilter mf;
};
}

#endif /* _ARP_RLU_OBJECTFINDER_HPP_ */
