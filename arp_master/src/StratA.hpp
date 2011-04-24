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
#include <arp_master/Obstacle.h>
#include <arp_master/StartColor.h>
#include <arp_master/Start.h>
#include <arp_master/Spawn.h>

#include "math/math.hpp"
#include "WayPoint.hpp"
#include "CircularTraj.hpp"
     

namespace arp_master
{

typedef actionlib::SimpleActionClient<arp_master::OrderAction> Client;


class StratA
{

private:
  static const double START_POSITION_RED_X = -1.45;
  static const double START_POSITION_RED_Y = 0.84;
  static const double START_POSITION_RED_THETA = 0.0;

public:
  StratA();
  ~StratA();

  void waitForServer();
  void initTraj();
  void go();

private:
  void obstacleCallback(const ObstacleConstPtr& c);
  void colorCallback(const StartColorConstPtr& o);
  void startCallback(const StartConstPtr& s);

  void doneCb(const actionlib::SimpleClientGoalState& state,
              const OrderResultConstPtr& result);

private:  
  CircularTraj ct_;
  Client ac_;

  ros::Subscriber obstacle_sub_;
  ros::Subscriber color_sub_;
  ros::Subscriber start_sub_;
  ros::ServiceClient loc_spawn_;
  ros::ServiceClient simu_spawn_;
  bool start_;
  bool obstacleDetected_;
  bool actionFinished_;
  ros::WallTime start_time_;
  ros::NodeHandle nh_;

};

}

#endif
