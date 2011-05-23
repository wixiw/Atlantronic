#include "PhysicsSimu.hpp"

#include <ros/package.h>
#include <cstdlib>
#include <ctime>

using namespace arp_core;
using namespace arp_hml;

PhysicsSimu::PhysicsSimu() :
    nh_(ros::NodeHandle("PhysicsSimu"))
{
    respawn_srv_ = nh_.advertiseService("respawn",
            &PhysicsSimu::respawnCallback, this);
    setPosition_srv_ = nh_.advertiseService("setPosition",
            &PhysicsSimu::setPositionCallback, this);

    ROS_INFO("Starting PhysicsSimu with node name %s",
            ros::this_node::getName().c_str());
}

PhysicsSimu::~PhysicsSimu()
{
}

bool PhysicsSimu::respawnCallback(Spawn::Request& req, Spawn::Response& res)
{
    ROS_INFO("Respawing Simu to x=%f, y=%f and theta=%f", req.x, req.y,
            req.theta);
    mRobot.reset(); //"deleting" shared_ptr
    spawnRobot(req.x, req.y, req.theta);
    res.name = "Protokrot";
    this->updateRobot();
    return true;
}

bool PhysicsSimu::setPositionCallback(SetPosition::Request& req,
        SetPosition::Response& res)
{
    ROS_INFO("setPosition Simu to x=%f, y=%f and theta=%f", req.x, req.y,
            req.theta);
    mRobot->setPosition(req.x, req.y, req.theta);
    this->updateRobot();
    return true;
}

void PhysicsSimu::spawnRobot(double x, double y, double angle)
{
    PhysicsSimuRobotPtr t(
            new PhysicsSimuRobot(ros::NodeHandle("Protokrot"), Vector2(x, y),
                    angle));
    mRobot = t;

    ROS_INFO("Spawning robot [%s] at x=[%f], y=[%f], theta=[%f]", "Protokrot",
            x, y, angle);

    return;
}

void PhysicsSimu::updateRobot()
{
    if (last_robot_update_.isZero())
    {
        last_robot_update_ = ros::WallTime::now();
        return;
    }

    ros::WallTime t = ros::WallTime::now();
    double dt = (t - last_robot_update_).toSec();
    //ROS_INFO("Time step : %f",dt);
    last_robot_update_ = t;
    mRobot->update(dt, table_length_in_meter, table_width_in_meter);

}

