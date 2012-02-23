/*
 * Circle.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include "Circle.hpp"

#include <exceptions/NotImplementedException.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace Eigen;
using namespace lsl;

Circle::Circle()
: cartesianPosition(0., 0.)
, radius(1.)
{
    ;
}

Circle::Circle(double _x, double _y, double _r)
: cartesianPosition(_x, _y)
, radius(_r)
{
    ;
}

Circle::Circle(Vector2 _pos, double _r)
: cartesianPosition(_pos)
, radius(_r)
{
    ;
}

double Circle::x() const
{
    return cartesianPosition[0];
}

double Circle::y() const
{
    return cartesianPosition[1];
}

double Circle::r() const
{
    return radius;
}

void Circle::x(double _x)
{
    cartesianPosition[0] = _x;
}

void Circle::y(double _y)
{
    cartesianPosition[1] = _y;
}

void Circle::r(double _r)
{
    radius = _r;
}

Vector2 Circle::getPosition() const
{
    return cartesianPosition;
}

void Circle::setPosition(Vector2 _pos)
{
    cartesianPosition = _pos;
}

