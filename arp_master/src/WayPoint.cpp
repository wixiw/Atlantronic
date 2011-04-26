#include "WayPoint.hpp"

#define PI 3.14159265

using namespace arp_master;


Point::Point():
  x_(0.0),
  y_(0.0),
  angle_(0.0)
{
  ;
}

Point::Point( Point & pt)
{
  x_ = pt.x();
  y_ = pt.y();
  angle_ = pt.angle();
}

Point::Point( double x, double y, double angle ):
  x_(x),
  y_(y),
  angle_(angle)
{
  ;
}

double & Point::x()
{
  return x_;
}

double & Point::y()
{
  return y_;
}

double & Point::angle()
{
  return angle_;
}


std::ostream & operator<<( std::ostream & os, arp_master::Point & pt)
{
  return os << "x=" << pt.x() << "  y=" << pt.y() << "  angle=" << pt.angle();
}


WayPoint::WayPoint(const WayPoint & wp):
  Point(0.0, 0.0, 0.0),
  color_(COLOR_RED),
  direction_(FORWARD)
{
  this->x_ = wp.get(COLOR_RED, FORWARD).x();
  this->y_ = wp.get(COLOR_RED, FORWARD).y();
  this->angle_ = wp.get(COLOR_RED, FORWARD).angle();
}

WayPoint::WayPoint(Color col, Direction dir, Point pt):
  Point(),
  color_(col),
  direction_(dir)
{
  this->x_ = pt.x();
  this->y_ = pt.y();
  this->angle_ = pt.angle();
}

WayPoint::WayPoint(Color col, Direction dir, double x, double y, double angle):
  Point(x, y, angle),
  color_(col),
  direction_(dir)
{
}

WayPoint::~WayPoint()
{
  ;
}


Point WayPoint::get(const Color col, const Direction dir) const
{
  double x = this->x_;
  double y = this->y_;
  double angle = this->angle_;
  if( color_ != col )
  {
    // Change color
    x = -x;
    angle = fmod(5*PI -angle , 2*PI);
  }
  if( direction_ != dir)
  {
    // Change direction
    angle = fmod( angle + 5*PI, 2*PI);
  }
  Point pt(x, y, angle);
  return pt;
}

