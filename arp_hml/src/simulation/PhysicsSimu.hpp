/*
 * PhysicsSimu.hpp
 *
 *  Created on: 15 apr. 2011
 *      Author: boris
 */

#ifndef ARP_MASTER_PHYSICSSIMU_HPP
#define ARP_MASTER_PHYSICSSIMU_HPP


#include <ros/ros.h>

//#include <std_srvs/Empty.h>
#include <arp_core/Spawn.h>
#include <arp_core/Kill.h>
#include <map>

#include "PhysicsSimuRobot.hpp"

using namespace arp_core;

namespace arp_hml
{
/** \ingroup arp_master
  * \nonstableyet
  *
  * \class PhysicsSimu
  *
  * \brief Table Simulation
  *
  * Manage physical and graphical simulation of table and robot
  * Should be divided in two parts
  *
  */

class PhysicsSimu
{

private:

  /**
   * table length in pixel. Should be the width of table image
   */
  static const double table_length_in_meter = 3.045;

  /**
   * table width in pixel. Should be the height of table image
   */
  static const double table_width_in_meter = 2.145;
public:

  /**
   * Default constructor
   * A timer is set width default_dt_ms time step
   * onUpdate is then called every default_dt_ms
   */
  PhysicsSimu();
  ~PhysicsSimu();

  /**
   * used by spawn service
   * \param x desired position in meter along table longest axis
   * \param y desired position in meter along table shortest axis
   * \param angle desired orientation
   */
  void spawnRobot(double x, double y, double angle);

  /**
     * should be called each time step
     * Measure actual time and call SimuRobot update
     */
    void updateRobot();

private:

  /**
   * used by respawn service
   */
  bool respawnCallback(Spawn::Request&, Spawn::Response&);

  /**
   * Simulator Nodehandle
   * Instanciated by PhysicsSimu constructor
   */
  ros::NodeHandle nh_;

  /**
   * ServiceServer used to clear table of all trace
   */
  ros::ServiceServer clear_srv_;

  /**
   * ServiceServer used to respawn robot
   */
  ros::ServiceServer respawn_srv_;

  /**
   * robot
   */
  PhysicsSimuRobotPtr mRobot;

  /**
   * used to compute actual time step
   */
  ros::WallTime last_robot_update_;

};

}

#endif
