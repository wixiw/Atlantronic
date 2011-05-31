/*
 * ObjectFinderNode.hpp
 *
 *  Created on: 29 mai 2011
 *      Author: ard
 */

#ifndef _ARP_RLU_OBJECTFINDERNODE_HPP_
#define _ARP_RLU_OBJECTFINDERNODE_HPP_

#include "ObjectFinder.hpp"

#include <string>
#include <vector>

namespace arp_rlu
{

class ObjectFinderNode
{
    public:
        ObjectFinderNode();
        ~ObjectFinderNode();

        void go();

    protected:
        /**
         * NodeHandle on associated node
         */
        ros::NodeHandle nh;

};
}

#endif /* _ARP_RLU_OBJECTFINDERNODE_HPP_ */
