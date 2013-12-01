/*
 * RosOdsItf.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "RosOdsItf.hpp"
#include <rtt/Component.hpp>
#include "control/orders/Logger.hpp"

using namespace arp_math;
using namespace arp_ods;
using namespace orders;
using namespace RTT;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_ods::RosOdsItf )

RosOdsItf::RosOdsItf(std::string const name):
        OdsTaskContext(name),
        m_actionServer("MotionControl", boost::bind(&RosOdsItf::newOrderCB, this, _1), false),
        propOrderConfig(),
        m_blockTime(0.0)
{
    createOrocosInterface();
    createRosInterface();

    m_order = orders::defaultOrder;

    //C'est une conf par defaut safe ! Utiliser le fichier de conf dans script/orocos/conf pour modifier
    propOrderConfig.RADIUS_APPROACH_ZONE = 0.020;
    propOrderConfig.ANGLE_ACCURACY = deg2rad(10);
    propOrderConfig.DISTANCE_ACCURACY = 0.007;

    propOrderConfig.LIN_VEL_MAX = 0.3;
    propOrderConfig.ANG_VEL_MAX = 1.0;
    propOrderConfig.VEL_FINAL = 0.100;

    propOrderConfig.ORDER_TIMEOUT = 10.0;
    propOrderConfig.PASS_TIMEOUT = 1.0;

    propOrderConfig.LIN_DEC=1.0;
    propOrderConfig.ANG_DEC=3.0;

    arp_ods::orders::Logger::InitFile("arp_ods", DEBUG);
}

bool RosOdsItf::configureHook()
{
    bool res = OdsTaskContext::configureHook();
    res &= getOperation("MotionControl",      "ooSetOrder",  m_ooSetOrder);
    res &= getOperation("MotionControl",      "ooSetVMax",  m_ooSetVMax);
    //TODO implementer le hook qui va bien pour checker la propOrderConfig
    return res;
}

bool RosOdsItf::startHook()
{
    bool res = OdsTaskContext::startHook();
    m_actionServer.start();
    return res;
}

void RosOdsItf::newOrderCB(const OrderGoalConstPtr &goal)
{
    ros::Rate r(20);
    OrderResult result;
    EstimatedPose2D pose;
    EstimatedICRSpeed speed;
    bool tmp;
    bool finished;

    /*
    //if a goal was already defined it is refused
    if (m_order->getType() != NO_ORDER)
    {
        ROS_ERROR(
                "MotionControl : you are trying to send a new order (%s) since the last one is not processed. It won't be performed, try to interrupt it before",
                goal->move_type.c_str());
        goto abort;
    }
*/

    //TODO a remplacer par une factory plus efficace
    char string[250];

    inPose.readNewest(pose);
    inSpeed.readNewest(speed);
    UbiquityMotionState currentMotionState(pose,speed);

    if (goal->move_type == "OMNIDIRECT")
    {
        m_order = OmnidirectOrder::createOrder(goal, currentMotionState, propOrderConfig);
        sprintf( string, "new Omnidirect goal (%0.3f,%0.3f,%0.3f) pass %d @ %0.3f m/s", goal->x_des, goal->y_des,
                goal->theta_des, goal->passe, goal->passe_speed);
        arp_ods::orders::Log(Info) << string << endlog();
    }
    
    if (goal->move_type == "OMNIDIRECT2")
    {
        m_order = OmnidirectOrder2::createOrder(goal, currentMotionState, propOrderConfig);
        sprintf( string, "new Omnidirect2 goal (%0.3f,%0.3f,%0.3f) pass %d @ %0.3f m/s", goal->x_des, goal->y_des,
                goal->theta_des, goal->passe, goal->passe_speed);
        arp_ods::orders::Log(Info) << string << endlog();
    }
    else if (goal->move_type == "OPENLOOP")
    {
        m_order = OpenloopOrder::createOrder(goal, currentMotionState, propOrderConfig);
        sprintf( string, "new Openloop goal Twist:(%0.3f,%0.3f,%0.3f) time : %0.3f ", goal->x_speed, goal->y_speed,
                goal->theta_speed, goal->openloop_duration);
        arp_ods::orders::Log(Info) << string << endlog();
    }
    else if (goal->move_type == "REPLAY")
    {
        m_order = ReplayOrder::createOrder(goal, currentMotionState, propOrderConfig);
        sprintf( string, "new Replay goal Twist:(%0.3f,%0.3f,%0.3f) time : %0.3f ", goal->x_speed, goal->y_speed,
                goal->theta_speed, goal->openloop_duration);
        arp_ods::orders::Log(Info) << string << endlog();
    }
    else
    {
        arp_ods::orders::Log(Error) << "order " << goal->move_type.c_str() << "is not possible" << endlog();
        goto abort;
    }

    if( m_ooSetOrder(m_order) == false )
    {

        goto abort;
    }


    //TODO WLA mettre ça dans order
    //m_orderSelector.setWorkTimeout(15);

    finished = inCurrentOrderIsFinished.readNewest(tmp) == NewData && tmp;
    while (!finished)
    {
        //An interrupt has been requested
        if (m_actionServer.isPreemptRequested() || !ros::ok())
        {
            arp_ods::orders::Log(Error) << goal->move_type.c_str() << " Preempted" << endlog();
            //TODO faudrait peut être l'envoyer à LittleSexControl
            goto preempted;
        }

        //If order is in error something went wrong
        bool inError;
        if( inCurrentOrderIsInError.readNewest(inError) != NewData )
        {
            inError = false;
        }
        if ( inError )
        {
            arp_ods::orders::Log(Error) << goal->move_type.c_str() << ": not processed due to MODE_ERROR" << endlog();
            goto abort;
        }

        //robot is blocked ?
        bool blocked;
        double time=getTime();
        inRobotBlocked.readNewest(blocked);
        if(blocked and time-m_blockTime>1.0) //1 of occultation, to allow the beginning of the new motion
        {
            arp_ods::orders::Log(Error) << goal->move_type.c_str() << ": not processed due to Robot Blockage detection" << endlog();
            m_blockTime= time;
            goto abort;
        }

        r.sleep();

        if( inCurrentOrderIsFinished.readNewest(finished) != NewData )
        {
            finished = false;
        }
    }
    //if we are here it's because an order has been interrupt and the robot is halted
    goto success;

    abort:
        inPose.readNewest(pose);
        result.x_end = pose.x();
        result.y_end = pose.y();
        result.theta_end = pose.h();
        m_order = orders::defaultOrder;
        m_ooSetOrder(m_order);
        m_actionServer.setAborted(result);
        return;
    success:
        ROS_INFO("%s: Finished", goal->move_type.c_str());
        inPose.readNewest(pose);
        result.x_end = pose.x();
        result.y_end = pose.y();
        result.theta_end = pose.h();
        //je veux laisser vivre mon ordre ! qu'il puisse finir son asservissement sur le point final pendant que ca fait de la bouine dans le master
        //m_order = orders::defaultOrder;
        //m_ooSetOrder(m_order);
        m_actionServer.setSucceeded(result);
        return;
    preempted:
        m_order = orders::defaultOrder;
        m_ooSetOrder(m_order);
        m_actionServer.setPreempted();
        return;
}

bool RosOdsItf::setVMaxCallback(SetVMax::Request& req, SetVMax::Response& res)
{
    return m_ooSetVMax(req.vMax);
}

void RosOdsItf::ooSetNewSpeedConf(double linSpeed, double angSpeed)
{
    if( linSpeed > 0 && linSpeed < 10 )
        propOrderConfig.LIN_VEL_MAX = linSpeed;
    else
        arp_ods::orders::Log(Error) << "You required a new linSpeed for propOrderConfig.LIN_VEL_MAX which is out range (check units ? sign ?)" << endlog();

    if( angSpeed > 0 && angSpeed < 30 )
        propOrderConfig.ANG_VEL_MAX = angSpeed;
    else
        arp_ods::orders::Log(Error) << "You required a new angSpeed for propOrderConfig.ANG_VEL_MAX which is out range (check units ? sign ?)" << endlog();
}

void RosOdsItf::ooSetNewAccConf(double linAcc, double angAcc)
{
    if( linAcc > 0 && linAcc < 15 )
        propOrderConfig.LIN_DEC = linAcc;
    else
        arp_ods::orders::Log(Error) << "You required a new linAcc for propOrderConfig.LIN_DEC which is out range (check units ? sign ?)" << endlog();

    if( angAcc > 0 && angAcc < 50 )
        propOrderConfig.ANG_DEC = angAcc;
    else
        arp_ods::orders::Log(Error) << "You required a new angAcc for propOrderConfig.ANG_DEC which is out range (check units ? sign ?)" << endlog();
}

void RosOdsItf::createOrocosInterface()
{
    addProperty("propOrderConfig",propOrderConfig);

    addPort("inPose",inPose);
    addPort("inSpeed",inSpeed);
    addPort("inCurrentOrderIsFinished",inCurrentOrderIsFinished);
    addPort("inCurrentOrderIsInError",inCurrentOrderIsInError);
    addPort("inRobotBlocked",inRobotBlocked);

    addOperation("ooSetNewSpeedConf", &RosOdsItf::ooSetNewSpeedConf,
            this, OwnThread) .doc("Use this the define new maximal speeds in propOrderConfig.");
    addOperation("ooSetNewAccConf", &RosOdsItf::ooSetNewAccConf,
            this, OwnThread) .doc("Use this the define new maximal accelerations in propOrderConfig.");
}

void RosOdsItf::createRosInterface()
{
    ros::NodeHandle nh;
    m_srvSetVMax = nh.advertiseService("/MotionControl/setVMax", &RosOdsItf::setVMaxCallback, this);
}


