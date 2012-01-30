/*
 * Circle.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_CIRCLE_HPP_
#define _ARP_RLU_LSL_CIRCLE_HPP_

#include <math/core>

namespace arp_rlu
{

namespace lsl
{

class Circle
{
    public:
        Circle();
        Circle(double _x, double _y, double _r);
        Circle(arp_math::Vector2 _pos, double _r);

        double x() const;
        double y() const;
        double r() const;
        void x(double);
        void y(double);
        void r(double);
        arp_math::Vector2 getPosition() const;
        void setPosition(arp_math::Vector2);


    protected:
        double radius;
        arp_math::Vector2 cartesianPosition;

};

} // namespace lsl
} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_CIRCLE_HPP_ */
