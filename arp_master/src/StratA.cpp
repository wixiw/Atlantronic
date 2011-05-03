#include "StratA.hpp"

using namespace arp_math;
using namespace arp_core;
using namespace arp_master;


StratA::StratA():
  ac_("MotionControl", true),
  start_(false),
  obstacleDetected_(false),
  actionFinished_(true),
  nh_()
{
  obstacle_sub_ = nh_.subscribe("obstacle", 1, &StratA::obstacleCallback, this);
  color_sub_ = ros::NodeHandle("Protokrot").subscribe("color", 1, &StratA::colorCallback, this);
  start_sub_ = ros::NodeHandle("Protokrot").subscribe("start", 1, &StratA::startCallback, this);
  loc_spawn_ = nh_.serviceClient<arp_master::Spawn>("Localizator/respawn");
  simu_spawn_ = nh_.serviceClient<arp_master::Spawn>("PhysicsSimu/respawn");
  robot_setpen_ = nh_.serviceClient<arp_master::SetPen>("Protokrot/set_pen");
}

StratA::~StratA()
{
  shutDown();
}

void StratA::waitForServer()
{
  ROS_INFO("Waiting for action server to start.");
  ac_.waitForServer();
  ROS_INFO("Action server started");
}

void StratA::initTraj()
{
  WayPoint wp1(COLOR_RED, FORWARD, 0.0, 0.84, 0.0);
  ct_.push_back(wp1);

  WayPoint wp2(COLOR_RED, FORWARD, 0.8, 0.0, 3*PI/2);
  ct_.push_back(wp2);

  WayPoint wp3(COLOR_RED, FORWARD, 0.0, -0.65, PI);
  ct_.push_back(wp3);

  WayPoint wp4(COLOR_RED, FORWARD, -0.8, 0.0, PI/2);
  ct_.push_back(wp4);

  ct_.setColor(COLOR_RED);
  ct_.setDirection(FORWARD);
  ct_.setIndexPreviousWayPoint(ct_.getNumberOfWayPoints());
  
}

void StratA::go()
{

  ROS_INFO("Waiting for start");
  ROS_INFO("TIPS (open a new terminal) :");
  ROS_INFO("* Choose color with :");
  ROS_INFO("\trostopic pub -1 /Protokrot/color arp_core/StartColor -- \"red\"");
  ROS_INFO("\tor rostopic pub -1 /Protokrot/color arp_core/StartColor -- \"blue\"");
  ROS_INFO("* Start with : rostopic pub -1 /start arp_core/Start -- 1");
  ROS_INFO("* Simule obstacle with : rostopic pub -1 /obstacle arp_core/Obstacle -- 1");

  ros::Rate r(10);

  while(!start_)
  {
    ros::spinOnce();
    r.sleep();
  }

  start_time_ = ros::WallTime::now();

  ROS_INFO("Go !!");
  while( ros::WallTime::now() - start_time_ < ros::WallDuration(90) )
  {
    if( obstacleDetected_ )
    {
      obstacleDetected_ = false;
      ac_.cancelAllGoals();
      ROS_INFO("toggle Direction");
      ct_.toggleDirection();
      actionFinished_ = true;
    }

    if( actionFinished_ )
    {
      arp_master::Point pt;
      pt = ct_.getNextPoint();
      std::stringstream ss;
      //ss << "Next Goal Point is : " << pt;
      //ROS_INFO("%s", ss.str().c_str() );

      // send a goal to the action
      arp_master::OrderGoal goal;
      goal.x_des = pt.x();
      goal.y_des = pt.y();
      goal.theta_des = pt.angle();
      ac_.sendGoal(goal, boost::bind(&StratA::doneCb, this, _1, _2));

      actionFinished_ = false;
    }
    
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Time out !!");
  ac_.cancelAllGoals();

}

void StratA::shutDown()
{
  if(!actionFinished_)
  {
    ac_.cancelAllGoals();
    ac_.stopTrackingGoal();

    Velocity vel;
    vel.linear = 0.0;
    vel.angular = 0.0;
    vel_pub_.publish(vel);
  }
}

void StratA::obstacleCallback(const ObstacleConstPtr& c)
{
  if( c->detected > 0.5 )
  {
    obstacleDetected_ = true;
#define PI 3.14159265
    ROS_INFO("Obstacle in sight !");
  }
}

void StratA::colorCallback(const StartColorConstPtr& o)
{
  if( !start_ )
  {
    if( o->color.compare("red") == 0 )
    {
      if( color_ == COLOR_RED && color_selected_)
      {
        ROS_INFO("Color already red");
        return;
      }
      ct_.setColor(COLOR_RED);
      ROS_INFO("Color is red");        
      arp_master::Spawn srv;
      srv.request.x = START_POSITION_RED_X;
      srv.request.y = START_POSITION_RED_Y;
      srv.request.theta = START_POSITION_RED_THETA;

      if(simu_spawn_.call(srv))
        ROS_INFO("Strat sent respawn call to PhysicsSimulator");
      else
        ROS_INFO("Strat failed to send respawn call to PhysicsSimulator");
      if(loc_spawn_.call(srv))
        ROS_INFO("Strat sent respawn call to Localizator");
      else
        ROS_INFO("Strat failed to send respawn call to Localizator");

      color_ = COLOR_RED;
      color_selected_ = true;
    }
    else if( o->color.compare("blue") == 0 )
    {
      if( color_ == COLOR_BLUE && color_selected_)
      {
        ROS_INFO("Color already blue");
        return;
      }
      ct_.setColor(COLOR_BLUE);
      ROS_INFO("Color is blue");
      arp_master::Spawn srv;
      srv.request.x = -START_POSITION_RED_X;
      srv.request.y = START_POSITION_RED_Y;
      srv.request.theta = fmod(START_POSITION_RED_THETA + PI, 2*PI);
      if(simu_spawn_.call(srv))
        ROS_INFO("Strat sent respawn call to PhysicsSimulator");
      else
        ROS_INFO("Strat failed to send respawn call to PhysicsSimulator");
      if(loc_spawn_.call(srv))
        ROS_INFO("Strat sent respawn call to Localizator");
      else
        ROS_INFO("Strat failed to send respawn call to Localizator");

      color_ = COLOR_BLUE;
      color_selected_ = true;
    }
    else
    {
      ROS_INFO("undefined color msg");
    }
  }
}

void StratA::startCallback(const StartConstPtr& s)
{
  if( !start_ )
  {
    if( s->go > 0 )
    {
      start_ = true;
      ROS_INFO("Start !");

      arp_master::SetPen srv_setpen;
      srv_setpen.request.r = 0xb3;
      srv_setpen.request.g = 0xb8;
      srv_setpen.request.b = 0xff;
      srv_setpen.request.width = 3;
      srv_setpen.request.off = false;
      if(robot_setpen_.call(srv_setpen))
        ROS_INFO("Strat sent set_pen call to Protokrot");
      else
        ROS_INFO("Strat failed to send set_pen off call to Protokrot");
    }

  }
}

void StratA::doneCb(const actionlib::SimpleClientGoalState& state,
            const OrderResultConstPtr& result)
{
  ROS_INFO("Action finished: %s",state.toString().c_str());
  actionFinished_ = true;
}
