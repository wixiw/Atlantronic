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

#include <math/math.hpp>

#include <sstream>

namespace arp_ods
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
         * linear velocity
         */
        double lin_vel_;

        /**
         * angular velocity
         */
        double ang_vel_;

        /**
         * time we went through the loop last time
         * used for derivation
         */
        double loop_date;

        /**
         * these constants are initialized by ros parameters. see .launch for explananations
         */
        double LIN_ACC_MAX;
        double LIN_DEC_MAX;
        double ANG_ACC_MAX;
        double BASE_LINE;
        double LEFT_WHEEL_DIAMETER;
        double RIGHT_WHEEL_DIAMETER;
    };

}

#endif

