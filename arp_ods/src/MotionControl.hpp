/*
 * MotionControl.hpp
 *
 *  Created on: 15 apr. 2011
 *      Author: boris
 */

#ifndef ARP_MASTER_MOTIONCONTROL_HPP
#define ARP_MASTER_MOTIONCONTROL_HPP

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>

#include <arp_ods/OrderAction.h>
#include <arp_core/Velocity.h>
#include <arp_core/Pose.h>
#include <math/Geometry.hpp>
#include <math/math.hpp>

using namespace arp_core;
using namespace nav_msgs;
using namespace geometry_msgs;

namespace arp_ods
{
    /**
     * MotionControl allows you to drive a non-holonome robot to a specific Point (x, y, angle).
     * MotionControl receives its order in actionlib standards.
     */
    class MotionControl
    {
    protected:

        /**
         * NodeHandle on associated node
         */
        ros::NodeHandle nh_;

        /**
         * Actionlib server.
         * Simple version is perfect here.
         */
        actionlib::SimpleActionServer<arp_ods::OrderAction> as_;

        /**
         * Actionlib stuff : action name which will be used to recognozied action server over ROS.
         */
        std::string action_name_;

        /**
         * Actionlib feedback (type define by actionlib script on Order.action file)
         */
        OrderFeedback feedback_;

        /**
         * Actionlib result (type define by actionlib script on Order.action file)
         */
        OrderResult result_;

        /**
         * Used to subscribe to "Localizator/pose"
         */
        ros::Subscriber pose_sub_;
        /**
         * TODO WLA à virer, utile pour tester Laserator
         */
        ros::Subscriber pose_sub_WLA;

        /**
         * Used to publish on "Command/velocity"
         */
        ros::Publisher vel_pub_;

        /** buffered translation (will be use during the main loop)
         * It is copied from m_transCallback at the begin of the loop */
        arp_math::Vector2 m_position;

        /** current translation (direct from the subscribe callback) */
        arp_math::Vector2 m_transCallback;

        /** buffered orientation (will be use during the main loop)
         * It is copied from m_orientCallback at the begin of the loop */
        arp_math::Rotation2 m_cap;

        /** current orientation (direct from the subscribe callback) */
        arp_math::Rotation2 m_orientCallback;

        /**
         *  possibly reversed orientation (to go backward) used by the motion control
         */
        arp_math::Rotation2 orientLocal_;

        /**
         *  possibly reversed objective
         */
        arp_math::Rotation2 orient_desLocal_;

        /**
         * published linear velocity
         */
        double m_linearSpeedCmd;

        /**
         * published angular velocity
         */
        double m_angularSpeedCmd;

        /**
         * time we went through the loop last time
         * used for derivation
         */
        double loop_date;

        /**
         *  keep track of the former loop value of angle_error, to compute derivative
         */
        double old_angle_error;

        /**
         * mode of operation
         */
        int mode;
        static const int MODE_APPROACH = 1;
        static const int MODE_FANTOM = 2;

        /**
         * this are coefficient that are defined by rosparam. see .launch files.
         */
        double DISTANCE_ACCURACY;
        double ANGLE_ACCURACY;
        double ROTATION_GAIN;
        double ROTATION_D_GAIN;
        double TRANSLATION_GAIN;
        double LIN_VEL_MAX;
        double ANG_VEL_MAX;
        double VEL_FINAL;
        double RADIUS_APPROACH_ZONE;
        double RADIUS_FANTOM_ZONE;
        double FANTOM_COEF;

    public:

        /**
         * Specify name of the Actionlib Server
         * This name will be usefull for the Actionlib clienti
         */
        MotionControl(std::string name);
        ~MotionControl(void);

    protected:

        /**
         * Called when a new pose message is received
         */
        void poseCallback(OdometryConstPtr c);

        /**
         * TODO WLA a virer test laserator
         */
        void pose2DCallback(Pose2DConstPtr c);

        /**
         * Called when a new order is received
         */
        void executeCB(const OrderGoalConstPtr &goal);

        /**
         * this is the restriction of linear speed due to angle error. the coefficient is between -1 and 1
         */
        double linearReductionCoef(double error);
    };
}

#endif
