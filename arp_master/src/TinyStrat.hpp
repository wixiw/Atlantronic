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
//ros messages
#include <arp_core/Obstacle.h>
#include <arp_core/StartColor.h>
#include <arp_core/Start.h>
//ros services
#include <arp_master/Spawn.h>

using namespace arp_core;
        
namespace arp_master
{

class TinyStrat
{
    private:
      static const double START_POSITION_RED_X = -1.45;
      static const double START_POSITION_RED_Y = 0.84;
      static const double START_POSITION_RED_THETA = 0.0;

    public:
      TinyStrat();
      ~TinyStrat();

      void go();

    private:
      void obstacleCallback(const ObstacleConstPtr& c);
      void colorCallback(const StartColorConstPtr& o);
      void startCallback(const StartConstPtr& s);

    private:
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
