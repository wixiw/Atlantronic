/*
 * CartesianCrop.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_CARTESIANCROP_HPP_
#define _ARP_RLU_LSL_CARTESIANCROP_HPP_

#include <math/math.hpp>
#include "LSL/LaserScan.hpp"

namespace arp_rlu
{

namespace lsl
{

class CartesianCrop
{
    public:
        class Params
        {
        public:
            Params();
            std::string getInfo();
        };

    public:
        static LaserScan apply(const LaserScan &, const Params & p = Params());


    protected:

};

} // namespace lsl
} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_CARTESIANCROP_HPP_ */