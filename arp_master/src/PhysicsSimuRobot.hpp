/*
 * SimuRobot.hpp
 *
 *  Created on: 15 apr. 2011
 *      Author: boris
 */

#ifndef ARP_MASTER_SIMUROBOT_HPP
#define ARP_MASTER_SIMUROBOT_HPP

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include "math/Geometry.hpp"

#include <arp_core/Pose.h>
#include <arp_core/Velocity.h>
#include <arp_core/DifferentialCommand.h>
#include <arp_core/Odo.h>

#include "math/math.hpp"

using namespace arp_math;

namespace arp_master
{
/** \ingroup arp_master
* \nonstableyet
*
* \class SimuRobot
*
* \brief Robot Simulation
*
* Use differential command to simulate robot deplacement on the table.
* Emulate odo data.
*
*/

class SimuRobot
{
public:
    /**
     * Constructor
     * \param nh NodeHandle of Physical simulator
     * \param pos Robot initial translation
     * \param orient Robot initial orientation (in radian)
     * \param one_meter_in_pixel number of pixel corresponding to 1 meter in robot_image
     */
  SimuRobot(const ros::NodeHandle& nh, const Vector2& pos, double orient);

  /**
   * update one time step and plot trace
   * \param dt time step in sec
   * \param canvas_width table width in meter
   * \param canvas_height table height in meter
   */
  void update(double dt, double canvas_width, double canvas_height);




private:

  /**
   * distance between wheels in meter
   */
  double base_line;

  /**
   * wheel diameter in meter
   */
  double wheel_diameter;


  // Callbacks
  /**
   * called every time SimuRobot receives a DifferentialCommand message
   */
  void commandCallback(const arp_core::DifferentialCommandConstPtr& c);

  /**
   * local copy of NodeHandle of Physical simulator
   */
  ros::NodeHandle nh_;

  /**
   * current position (translation)
   */
  Vector2 pos_;

  /**
   * current orientation
   */
  Rotation2 orient_;

  /**
   * last position (translation)
   */
  Vector2 old_pos_;

  /**
   * last orientation
   */
  Rotation2 old_orient_;

  /**
   * desired angular velocity (in rad/sec) for left wheel (received via DifferentialMessage)
   */
  double v_left_;

  /**
   * desired angular velocity (in rad/sec) for right wheel (received via DifferentialMessage)
   */
  double v_right_;

  /**
   * linear velocity (in m/sec) computed from v_left_ and v_right_
   */
  double lin_vel_;

  /**
   * angular velocity (in rad/sec) computed from v_left_ and v_right_
   */
  double ang_vel_;


  /**
   * left odo data (in radian, cumulated since start)
   */
  double odo_left_;

  /**
   * right odo data (in radian, cumulated since start)
   */
  double odo_right_;

  /**
   * Subscriber used to receive DifferentialCommand
   */
  ros::Subscriber differential_command_sub_;

  /**
   * Publisher used to publish Pose
   */
  ros::Publisher pose_pub_;

  /**
   * Publisher used to publish Odo
   */
  ros::Publisher odo_pub_;


  /**
   * time of last command received
   */
  ros::WallTime last_command_time_;


};
typedef boost::shared_ptr<SimuRobot> SimuRobotPtr;

}

#endif
