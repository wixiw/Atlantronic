/*
 * TinyStrat.hpp
 *
 *  Created on: 15 apr. 2011
 *      Author: boris
 */

#ifndef ARP_MASTER_TINYSTRAT_HPP
#define ARP_MASTER_TINYSTRAT_HPP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arp_master/OrderAction.h>

#include <arp_core/Obstacle.h>
#include <arp_core/StartColor.h>
#include <arp_core/Start.h>

#include <arp_core/Spawn.h>

#include "math/math.hpp"

        
namespace arp_master
{
/** \ingroup arp_master
* \nonstableyet
*
* \class TinyStrat
*
* \brief Only here to expose color, start and obstacle service during teleop
*/
class TinyStrat
{

private:

    /**
     * red start position.
     * in meter, along the longest table axis
     */
  static const double START_POSITION_RED_X = -1.45;

  /**
   * red start position.
   * in meter, along the shortest table axis
   */
  static const double START_POSITION_RED_Y = 0.84;

  /**
   * red start orientation.
   */
  static const double START_POSITION_RED_THETA = 0.0;

public:
  TinyStrat();
  ~TinyStrat();

  /**
   * manage start and spin ROS
   */
  void go();

private:
  /**
   * used to catch obstacle message
   */
  void obstacleCallback(const arp_core::ObstacleConstPtr& c);

  /**
   * used to catch color message
   */
  void colorCallback(const arp_core::StartColorConstPtr& o);

  /**
     * used to catch start message
     */
  void startCallback(const arp_core::StartConstPtr& s);

private:  

  /**
   * obstacle suscriber
   */
  ros::Subscriber obstacle_sub_;


  /**
   * color suscriber
   */
  ros::Subscriber color_sub_;


  /**
   * start suscriber
   */
  ros::Subscriber start_sub_;

  /**
   * used to call spawn service of Localizator
   */
  ros::ServiceClient loc_spawn_;


  /**
   * used to call spawn service of Simulator
   */
  ros::ServiceClient simu_spawn_;

  /**
   * is started ?
   */
  bool start_;

  /**
   * obstacle detected ?
   */
  bool obstacleDetected_;

  /**
   * used to stop robot after 90sec
   */
  ros::WallTime start_time_;

  /**
   * initialized by default in TinyStrat constructor
   */
  ros::NodeHandle nh_;
};

}

#endif
