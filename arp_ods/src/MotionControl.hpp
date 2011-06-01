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

#include "timer/StatTimer.hpp"
#include "arp_core/TimerReport.h"

#include <arp_ods/OrderAction.h>
#include <arp_ods/SetVMax.h>
#include <arp_core/Velocity.h>
#include <arp_core/Pose.h>
#include <math/Geometry.hpp>
#include <math/math.hpp>

#include "orders/orders.h"

using namespace arp_core;
using namespace nav_msgs;
using namespace geometry_msgs;

namespace arp_ods
{
class ModeSelector;

/**
 * MotionControl allows you to drive a non-holonome robot to a specific Point (x, y, angle).
 * MotionControl receives its order in actionlib standards.
 *
 * An order is stored into a MotionOrder by the OrderSelector. The OrderSelector is responsible of
 * linking orders and interupting gently when requested.
 * The selection of modes into an MotionOrder is delegated to a ModeSelector that you can find
 * within the MotionOrder
 */
class MotionControl
{

    public:

        /**
         * Nécessaire pour éviter ;
         * http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
         */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         */
        MotionControl();

        //the mainloop that should be called by the node
        void execute(void);

    protected:

        /**************************************************************************************
         *  Motion Control data
         **************************************************************************************/

        /** The current order to do. There is always one at a time since
         * newOrderCB is blocking by design of the action lib
         */
        shared_ptr<MotionOrder> m_order;

        /** buffered Pose (will be use during the main loop)
         */
        Odometry m_poseFromCallback;

        /** current Pose
         */
        arp_core::Pose m_currentPose;

        /** last processed Pose
         */
        arp_core::Pose m_lastPose;

        /**
         * computed speed commands
         */
        Velocity m_computedVelocityCmd;

        /** Motion control configuration. They are feeded with rosparam during init */
        order::config m_orderConfig;

        /**
         *  current max speed
         */
        double m_vMax;

        /**
         * Read the inputs
         */
        void getInputs();

        /**
         * Publish the processed datas
         */
        void setOutputs();

        /**
         * Tells if the current order is finished with success. It is finished when the robot is in PASS mode when the order
         * is a "pass" type or When the mode is DONE.
         */
        bool isOrderFinished();

        /**************************************************************************************
         *  ROS specific data
         **************************************************************************************/

        /**
         * Use this mutex to avoid corruption of the order data
         */
        boost::mutex m_orderMutex;

        /**
         * Actionlib server.
         * Simple version is perfect here.
         */
        actionlib::SimpleActionServer<arp_ods::OrderAction> m_actionServer;

        /**
         * Used to subscribe to "Localizator/pose"
         */
        ros::Subscriber pose_sub_;

        /**
         * Used to publish on "Command/velocity"
         */
        ros::Publisher vel_pub_;

        /**
         * used to offer velocity limitation service
         */
        ros::ServiceServer setVMax_srv;

        /**
         * Called when a new pose message is received
         */
        void poseCallback(OdometryConstPtr c);

        /**
         * TODO WLA a virer test laserator
         */
        //void pose2DCallback(Pose2DConstPtr c);

        /**
         * Called when a new order is received
         * This *MUST* be a blocking function since ActionLib wants it.
         * The caller is not blocked because a background thread is waiting
         * the action result to call a callback.
         * In a way this should be done independantly of the periodic job
         * to publish velocity commands. For simplicity, this callback stay here
         * but it is as if it was independant
         */
        void newOrderCB(const OrderGoalConstPtr &goal);

        /**
         * Read ROS parameters from the ROS server
         */
        void updateParams();

        /**
         * Timer to scope performance
         */
        StatTimer timer_;

        /**
         * Used to provide timer report
         */
        ros::ServiceServer timerreport_srv;

        /**
         * Called when timerreport service is called
         * \returns success boolean
         */
        bool timerreportCallback(TimerReport::Request& req, TimerReport::Response& res);

        /**
         * called when setvmax service is called
         */
        bool setVMaxCallback(SetVMax::Request& req, SetVMax::Response& res);

};
}

#endif
