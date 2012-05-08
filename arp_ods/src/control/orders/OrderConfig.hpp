/*
 * OrderConfig.hpp
 *
 *  Created on: 05 mai 2012
 *      Author: wla
 */

#ifndef ORDERCONFIG_HPP_
#define ORDERCONFIG_HPP_

namespace arp_ods{ namespace orders
{

struct config
{
    /** This default parameter defines the APPROACH mode area in m*/
    double RADIUS_APPROACH_ZONE;

    /** This default parameter defines the condition in distance to end motion in m*/
    double DISTANCE_ACCURACY;

    /** This default parameter defines the condition in angle to end motion in rad*/
    double ANGLE_ACCURACY;

    /** Pass default timeout */
    double PASS_TIMEOUT;

    /** Order timeout*/
    double ORDER_TIMEOUT;

    /**
     * final velocity
     */
     double VEL_FINAL;

     /*
      * maximum linear speed
      */
     double LIN_VEL_MAX;
     /*
      * maximum angular speed
      */
     double ANG_VEL_MAX;

     /*
      * linear deceleration (for point approach) in m/s2
      */
     double LIN_DEC;
     /*
      * angular deceleration (for point approach) in rad/s2
      */
     double ANG_DEC;
};

}}

#endif
