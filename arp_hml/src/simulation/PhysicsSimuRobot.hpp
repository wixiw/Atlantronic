/*
 * PhysicsSimuRobot.hpp
 *
 *  Created on: 15 apr. 2011
 *      Author: boris
 */

#ifndef ARP_MASTER_SIMUROBOT_HPP
#define ARP_MASTER_SIMUROBOT_HPP

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <std_msgs/Bool.h>

#include <math/Geometry.hpp>
#include <math/math.hpp>

#include <arp_core/Pose.h>
#include <arp_core/Velocity.h>
#include <arp_core/DifferentialCommand.h>
#include <arp_core/Odo.h>
#include <arp_hml/SetMotorPower.h>
#include <arp_hml/ResetHml.h>

using namespace arp_math;

namespace arp_hml
{
    /** \ingroup arp_master
     * \nonstableyet
     *
     * \class PhysicsSimuRobot
     *
     * \brief Robot Simulation
     *
     * Use differential command to simulate robot deplacement on the table.
     * Emulate odo data.
     *
     */

    class PhysicsSimuRobot
    {
    public:
        /**
         * Nécessaire pour éviter ;
         * http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
         */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * Constructor
         * \param nh NodeHandle of Physical simulator
         * \param pos Robot initial translation
         * \param orient Robot initial orientation (in radian)
         * \param one_meter_in_pixel number of pixel corresponding to 1 meter in robot_image
         */
        PhysicsSimuRobot(const ros::NodeHandle& nh, const Vector2& pos,
                double orient);

        /**
         * update one time step and plot trace
         * \param dt time step in sec
         * \param canvas_width table width in meter
         * \param canvas_height table height in meter
         */
        void update(double dt, double canvas_width, double canvas_height);

    private:

        /**
         * these are rosparam. see .launch for more info
         */
        double BASE_LINE;
        double LEFT_WHEEL_DIAMETER;
        double RIGHT_WHEEL_DIAMETER;

        // Callbacks
        /**
         * called every time PhysicsSimuRobot receives a DifferentialCommand message
         */
        void commandCallback(const arp_core::DifferentialCommandConstPtr& c);

        /**
         * called every time PhysicsSimuRobot receives a EmergencyStop message
         */
        void emergencyCallback(const std_msgs::BoolConstPtr& c);

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
         * Subscriber used to receive emergency_stop
         */
        ros::Subscriber emergency_sub_;

        /**
         * Publisher used to publish Pose
         */
        ros::Publisher pose_pub_;

        /**
         * Publisher used to publish Odo
         */
        ros::Publisher odo_pub_;

        /**
         * Publisher used to publish the enable state of motors
         */
        ros::Publisher enable_pub_;

        /**
         * time of last command received
         */
        ros::Time last_command_time_;

        /** SImulate the motor power enable **/
        bool m_powerEnabled;

        /** node handle to store the service advertiser srvSetMotorPower**/
        ros::ServiceServer m_srvSetMotorPower;

        /** node handle to store the service advertiser srvResetHml**/
        ros::ServiceServer m_srvResetHml;

        /**
         * ROS wrapper on the simulated ooSetPowerMotor operation
         */
        bool srvSetMotorPower(SetMotorPower::Request& req,
                SetMotorPower::Response& res);

        /**
         * ROS wrapper on the simulated ooResetHml operation
         */
        bool srvResetHml(ResetHml::Request& req, ResetHml::Response& res);

    };
    typedef boost::shared_ptr<PhysicsSimuRobot> PhysicsSimuRobotPtr;

}

#endif
