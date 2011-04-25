/*
 * Command.hpp
 *
 *  Created on: 15 apr. 2011
 *      Author: boris
 */

#ifndef ARP_MASTER_COMMAND_HPP
#define ARP_MASTER_COMMAND_HPP

#include <ros/ros.h>

#include <arp_core/Velocity.h>
#include <arp_core/DifferentialCommand.h>

#include <sstream>

using namespace arp_core;

namespace arp_master
{

  /**
  * At the moment Command class is pretty simple.
  * It just exists to avoid having code in Node file.
  *
  * When you instantiate a Command, your Command will subscribe to 
  * a Velocity topic named "Command/velocity" and will publish
  * a DifferentialCommand on topic named "Protokrot/differential_command"
  */
  class Command
  {
    public:
      Command();
      ~Command();

    protected:
      /**
      * NodeHandle on associated node
      */
      ros::NodeHandle nh;  

      /**
      * Used to subscribe to "Command/velocity"
      */
      ros::Subscriber velocity_sub;

      /**
      * Used to publish on "Protokrot/differential_command"
      */
      ros::Publisher command_pub;

      /**
      * This callback do stuff every time Velocity message is reveived.
      */
      void velocityCallback(const VelocityConstPtr& v);
  };

}

#endif

