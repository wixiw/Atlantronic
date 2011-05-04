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


namespace arp_master
{

  /**
  * \ingroup arp_master
  *
  * \class Command
  *
  * \brief Use twist to compute differential command
  *
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
      void velocityCallback(const arp_core::VelocityConstPtr& v);

      /**
      * Distance in meters between wheels.
      */
      double base_line;

      /**
      * Diameter in meters.
      */
      double wheel_diameter;

  };

}

#endif

