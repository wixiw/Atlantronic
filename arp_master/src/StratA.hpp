/*
 * StratA.hpp
 *
 *  Created on: 15 apr. 2011
 *      Author: boris
 */

#ifndef ARP_MASTER_STRATA_HPP
#define ARP_MASTER_STRATA_HPP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arp_master/OrderAction.h>

#include <arp_core/Obstacle.h>
#include <arp_core/StartColor.h>
#include <arp_core/Start.h>

#include <arp_master/Spawn.h>

#include "math/math.hpp"
#include "WayPoint.hpp"
#include "CircularTraj.hpp"
     

namespace arp_master
{

typedef actionlib::SimpleActionClient<arp_master::OrderAction> Client;

/**
 * \ingroup arp_master
 * \nonstableyet
 *
 * \class StratA
 *
 * \brief First ARD Strat : just turn around the table
 */
class StratA
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
  StratA();
  ~StratA();

  /**
   * wait for actionlib server (instanciate by MotionControl Node)
   */
  void waitForServer();

  /**
   * here, we set up the trajectory
   */
  void initTraj();

  /**
   * begin !
   * go() will wait the start, then pilot the robot along trajectory and "avoid" obstacle.
   */
  void go();

  /**
   * Shut the obsject down
   */
  void shutDown();

private:
  /**
   * used to received obstacle message
   */
  void obstacleCallback(const arp_core::ObstacleConstPtr& c);

  /**
   * used to receive color message
   */
  void colorCallback(const arp_core::StartColorConstPtr& o);

  /**
   * used to receive start message
   */
  void startCallback(const arp_core::StartConstPtr& s);

  /**
   * called every time an (actionlib) order is successfully achieved
   */
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const OrderResultConstPtr& result);

private:
  /**
   * CircularTraj instanciate in initTraj()
   */
  CircularTraj ct_;

  /**
   * actionlib client
   */
  Client ac_;

  /**
   * obstacle subscriber
   */
  ros::Subscriber obstacle_sub_;

  /**
   * color subscriber
   */
  ros::Subscriber color_sub_;

  /**
   * start subscriber
   */
  ros::Subscriber start_sub_;

  /**
   * used to call spawn (reset) service of Localizator
   */
  ros::ServiceClient loc_spawn_;

  /**
   * used to call spawn service of Simulator
   */
  ros::ServiceClient simu_spawn_;

  /**
   * is match begin ?
   */
  bool start_;

  /**
   * is obstacle detected ?
   * setted to True when obstacle message is received
   * setted to False when obstacle avoidance begin
   */
  bool obstacleDetected_;

  /**
   * is action finished ?
   */
  bool actionFinished_;

  /**
   * used to stop robot after 90sec
   */
  ros::WallTime start_time_;

  /**
   * instanciated by default in constructor
   */
  ros::NodeHandle nh_;

};

}

#endif
