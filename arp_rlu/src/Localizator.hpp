/*
 * Localizator.hpp
 *
 *  Created on: 15 apr. 2011
 *      Author: boris
 */

#ifndef ARP_MASTER_LOCALIZATOR_HPP
#define ARP_MASTER_LOCALIZATOR_HPP

#include "ros/ros.h"

#include <arp_core/Odo.h>
#include <arp_core/Pose.h>
#include <arp_core/Spawn.h>

#include <sstream>

#include "math/Geometry.hpp"

using namespace arp_core;

namespace arp_rlu
{
  /**
  * \ingroup arp_master
  *
  * \class Localizator
  *
  * \brief Use odo to estimate pose
  *
  * At the moment Localizator class is pretty simple.
  * It just exists to avoid having code in Node file.
  *
  * When you instantiate a Localizator, your Localizator will subscribe to 
  * an Odo topic named "Protokrot/odo" and will publish
  * a Pose on a topic named "Localizator/pose".
  * Localizator provide also a respawnService, pretty usefull for resetting.
  */
  class Localizator
  {
  public:
    Localizator();
    ~Localizator();

  protected:  
    /**
    * NodeHandle on associated node
    */
    ros::NodeHandle nh;
  
    /**
    * Used to subscribe to "Protokrot/odo"
    */
    ros::Subscriber    odo_sub;

    /**
    * Used to publish on "Localizator/pose"
    */
    ros::Publisher     pose_pub;

    /**
    * Used to provide a reset service
    */
    ros::ServiceServer respawn_srv;

    /**
    * Used to provide a init service
    */
    ros::ServiceServer init_srv;

    /**
    * buffer last left odo
    */
    double last_odo_left;

    /**
    * buffer last right odo
    */
    double last_odo_right;

    /**
    * time of last update
    */
    ros::WallTime last_time;

    /**
    * duration since last update (for integration)
    */
    ros::WallDuration duration;

    /**
    * estimated translation
    */
    arp_math::Vector2   trans;

    /**
    * estimated orientation
    */
    arp_math::Rotation2 orient;

    /**
    * these are ros param. see .launch for more info
    */
    double BASE_LINE;
    double WHEEL_DIAMETER;

    /**
    * Called when resetting service is called
    * \returns success boolean
    */
    bool respawnCallback(Spawn::Request& req, Spawn::Response& res);

    /**
    * Called when a new odo message is received
    * \returns succes boolean
    */
    void odoCallback(const arp_core::OdoConstPtr& o);

  };
}

#endif
