/*
 * WayPoint.hpp
 *
 *  Created on: 15 apr. 2011
 *      Author: boris
 */

#ifndef ARP_MASTER_WAYPOINT_HPP
#define ARP_MASTER_WAYPOINT_HPP

#include <ros/ros.h>
#include <iostream>


namespace arp_master
{

class Point
{
protected:
  /**
  * X axis is centered on the perfect middle of the table
  * X axis is aligned with the longest border of table
  * x_ is in meter
  */
  double x_;

  /**
  * Y axis is centered on the perfect middle of the table
  * Y axis is aligned with the shortest border of table
  * y_ is in meter
  */
  double y_;

  /**
  * angle is in radian, counter-clock wise.
  * angle = 0.0 means heading is aligned with X axis.
  */
  double angle_;

public:
  /**
  * You will have 0.0 on x, y and angle
  */
  Point();

  /**
  * Copy constructor
  */
  Point( Point & pt);

  /**
  * Directly specify your Point
  */
  Point( double x, double y, double angle );

  /**
  * Non-const accessor. Can be used both direction.
  */
  double & x();

  /**
  * Non-const accessor. Can be used both direction.
  */
  double & y();

  /**
  * Non-const accessor. Can be used both direction.
  */
  double & angle();
};


/**
* Stream a formated output for logging.
*/
std::ostream & operator<<( std::ostream & os, Point & pt);

/**
* The both table color.
*/
enum Color
{
    COLOR_RED,
    COLOR_BLUE
};

/**
* Direction, or "Sens" in French.
*/
enum Direction
{
    FORWARD,
    BACKWARD
};


/**
* WayPoint is a usefull Point which is independant of color and direction.
*/
class WayPoint : public Point
{
protected:
/**
* Color convention of stored Point.
*/
Color color_;

/**
* Direction convention of stored Point.
*/
Direction direction_;

public:
  /**
  * Oh my godness ! 
  * There is no default constructor !
  * It is a fully design-driven choise. You HAVE to specify your Color and Direction conventions when you instialize a WayPoint.
  */
  WayPoint(const WayPoint & wp);

  /**
  * Oh my godness ! 
  * There is no default constructor !
  * It is a fully design-driven choise. You HAVE to specify your Color and Direction conventions when you instialize a WayPoint.
  */
  WayPoint(Color col, Direction dir, Point pt);

  /**
  * Oh my godness ! 
  * There is no default constructor !
  * It is a fully design-driven choise. You HAVE to specify your Color and Direction conventions when you instialize a WayPoint.
  */
  WayPoint(Color col, Direction dir, double x, double y, double angle);
  ~WayPoint();
  
  /**
  * Get the version of the Point you want.
  * Choose your Color and Direction. 
  * get() method will compute your specific Point from the stored coordinate.
  */
  Point get(const Color col, const Direction dir) const;

};

}
#endif
