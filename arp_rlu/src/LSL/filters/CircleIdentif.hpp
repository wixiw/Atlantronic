/*
 * CircleIdentif.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_CIRCLEIDENTIF_HPP_
#define _ARP_RLU_LSL_CIRCLEIDENTIF_HPP_

#include <math/core>

#include <vector>

#include <LSL/LaserScan.hpp>
#include <LSL/objects/DetectedObject.hpp>
#include <LSL/objects/DetectedCircle.hpp>

namespace arp_rlu
{

namespace lsl
{

class CircleIdentif
{
    public:
        class Params
        {
        public:
            Params();
            std::string getInfo();
        };

    public:
        static DetectedCircle apply(const DetectedObject &, const Params & p = Params());
        static std::vector<DetectedCircle> apply(const std::vector<DetectedObject> &, const Params & p = Params());

    protected:

};

} // namespace lsl
} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_CIRCLEIDENTIF_HPP_ */
