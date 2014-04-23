/*
 * ActuatorGraphicsNode.cpp
 *
 *  Created on: 11 mars 2014
 *      Author: wla
 */
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <iostream>
#include <arp_core/CannonState.h>
#include "frames/CannonFrame.hpp"

arp_hml::CannonFrame frame;

void quit(int sig)
{
    ROS_INFO("Ctrl C catched => Shuting down HokuyoManager");
    frame.shutDown();
    ros::shutdown();
    exit(0);
}

arp_hml::CannonFrame::eFingerPosition findFingerPosition(double pos)
{
    //TODO
    //0 is the shooting position full read
    if( fabs(pos) <= 0.4 )
    {
        return arp_hml::CannonFrame::SHOOTING;
    }
    if( pos >= 0.4 && pos <= M_PI_2 - 0.4 )
    {
        return arp_hml::CannonFrame::ARMED;
    }
    if( pos >= M_PI_2 - 0.4 )
    {
        return arp_hml::CannonFrame::UP;
    }

    return arp_hml::CannonFrame::DOWN;
}

arp_hml::CannonFrame::eStockerPosition findStockerPosition(double pos)
{
    //TODO
    //0 is the IDLE position
    if( fabs(pos) <= 0.4 )
    {
        return arp_hml::CannonFrame::IDLE;
    }
    if( pos >= 0.4 )
    {
        return arp_hml::CannonFrame::LOADING;
    }

    return arp_hml::CannonFrame::UNLOADING;
}


void cannonStateCB(const arp_core::CannonStateConstPtr state)
{
   frame.setNumberOfBallsInCanon(arp_hml::CannonFrame::LEFT,  state->nbBallsInLeftStock);
   frame.setNumberOfBallsInCanon(arp_hml::CannonFrame::RIGHT, state->nbBallsInRightStock);

   frame.setFingerPosition(arp_hml::CannonFrame::LEFT, findFingerPosition(state->leftFingerPosition));
   frame.setFingerPosition(arp_hml::CannonFrame::RIGHT, findFingerPosition(state->rightFingerPosition));
   frame.setStockerPosition(arp_hml::CannonFrame::LEFT, findStockerPosition(state->leftStockerPosition));
   frame.setStockerPosition(arp_hml::CannonFrame::RIGHT, findStockerPosition(state->rightStockerPosition));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ActuatorGraphics");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/Ubiquity/canonState", 1, cannonStateCB);

    signal(SIGINT, quit);

    if( false == frame.init(argc, argv) )
    {
        ROS_ERROR("Failed to init ActuatorGraphicNode");
        return -1;
    }
    ROS_INFO("ActuatorGraphicNode spawned");


    ros::Rate r(10);

    while (ros::ok() && frame.spin())
    {
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}

