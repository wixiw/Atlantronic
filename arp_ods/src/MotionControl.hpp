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
#include <arp_ods/OrderAction.h>

#include <arp_core/Velocity.h>
#include <arp_core/Pose.h>

#include <math/Geometry.hpp>
#include <math/math.hpp>

using namespace arp_core;

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
         * Used to publish on "Command/velocity"
         */
        ros::Publisher vel_pub_;

        /**
         * current translation (direct from Localizator)
         */
        arp_math::Vector2 trans_;

        /**
         * current orientation (direct from Localizator)
         */
        arp_math::Rotation2 orient_;

        /**
         *  possibly reversed orientation (to go backward) used by the motion control
         */
        arp_math::Rotation2 orientLocal_;

        /**
         * published linear velocity
         */
        double lin_vel_;

        /**
         * published angular velocity
         */
        double ang_vel_;

        /**
         * this are coefficient that are defined by rosparam. see .launch files.
         */
        double DISTANCE_ACCURACY;
        double ANGLE_ACCURACY;
        double ROTATION_GAIN;
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
        void poseCallback(const PoseConstPtr& c);

        /**
         * Called when a new order is received
         */
        void executeCB(const OrderGoalConstPtr &goal);

    };
}

#endif
