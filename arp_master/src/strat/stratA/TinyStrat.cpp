#include "TinyStrat.hpp"


using namespace arp_core;
using namespace arp_math;
        
using namespace arp_master;


TinyStrat::TinyStrat():
  start_(false),
  obstacleDetected_(false),
  nh_()
{
  obstacle_sub_ = nh_.subscribe("obstacle", 1, &TinyStrat::obstacleCallback, this);
  color_sub_ = nh_.subscribe("color", 1, &TinyStrat::colorCallback, this);
  start_sub_ = nh_.subscribe("start", 1, &TinyStrat::startCallback, this);
  loc_spawn_ = nh_.serviceClient<arp_core::Spawn>("Localizator/respawn");
  simu_spawn_ = nh_.serviceClient<arp_core::Spawn>("ARDSimu/respawn");
  loc_setPosition_ = nh_.serviceClient<arp_core::SetPosition>("Localizator/setPosition");
  simu_setPosition_ = nh_.serviceClient<arp_core::SetPosition>("ARDSimu/setPosition");


}

TinyStrat::~TinyStrat()
{
  ;
}

void TinyStrat::go()
{

  ROS_INFO("Waiting for start");
  while(!start_)
  {
    ros::spinOnce();
  }

  start_time_ = ros::WallTime::now();

  ROS_INFO("Go !!");
  while( ros::WallTime::now() - start_time_ < ros::WallDuration(90) )
  {
    if( obstacleDetected_ )
    {
      obstacleDetected_ = false;
    }
    
    ros::spinOnce();
  }

  ROS_INFO("Time out !!");


}

void TinyStrat::obstacleCallback(const ObstacleConstPtr& c)
{
  if( c->detected > 0.5 )
  {
    obstacleDetected_ = true;
    ROS_INFO("Obstacle in sight !");
  }
}

void TinyStrat::colorCallback(const StartColorConstPtr& o)
{
  if( !start_ )
  {
    if( o->color.compare("red") == 0 )
    {
      ROS_INFO("Color is red");        
      arp_core::SetPosition srv;
      srv.request.x = START_POSITION_RED_X;
      srv.request.y = START_POSITION_RED_Y;
      srv.request.theta = START_POSITION_RED_THETA;
      if(loc_setPosition_.call(srv))
        ROS_INFO("Start sent setPosition call to Localizator");
      else
        ROS_INFO("Start failed to send respawn call to Localizator");
      if(simu_spawn_.call(srv))
        ROS_INFO("Start sent setPosition call to Simulator");
      else
        ROS_INFO("Start failed to send setPosition call to Simulator");
    }
    else if( o->color.compare("blue") == 0 )
    {
      ROS_INFO("Color is blue");
      arp_core::SetPosition srv;
      srv.request.x = -START_POSITION_RED_X;
      srv.request.y = START_POSITION_RED_Y;
      srv.request.theta = fmod(START_POSITION_RED_THETA + PI, 2*PI);
      if(loc_setPosition_.call(srv))
        ROS_INFO("Start sent setPosition call to Localizator");
      else
        ROS_INFO("Start failed to send setPosition call to Localizator");
      if(simu_setPosition_.call(srv))
        ROS_INFO("Start sent setPosition call to Simulator");
      else
        ROS_INFO("Start failed to send setPosition call to Simulator");
    }
    else
    {
      ROS_INFO("undefined color msg");
    }
  }
}

void TinyStrat::startCallback(const StartConstPtr& s)
{
  if( !start_ )
  {
    if( s->go > 0 )
    {
      start_ = true;
      ROS_INFO("Start !");
    }
  }
}

