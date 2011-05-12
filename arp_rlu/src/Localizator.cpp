#include "Localizator.hpp"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

using namespace arp_math;
using namespace arp_core;
using namespace arp_rlu;

Localizator::Localizator():
	nh(),
	odo_sub(),
	pose_pub(),
	odom_pub(),
	odom_tf_pub(),
	respawn_srv(),
	init_srv(),
	last_odo_left(0.0),
	last_odo_right(0.0),
	last_time(0.0),//initialized at a non valid valid on purposes
	trans(Vector2(0.0, 0.0)),
	orient(Rotation2(0.0))
{
	odo_sub = nh.subscribe("Protokrot/odo", 1, &Localizator::odoCallback, this);
	pose_pub = nh.advertise<Pose>("Localizator/pose", 1);
	odom_pub = nh.advertise<nav_msgs::Odometry>("Localizator/odomRos", 1);
	respawn_srv = nh.advertiseService("Localizator/respawn", &Localizator::respawnCallback, this);

    // Parameters
    nh.getParam("/Protokrot/BASE_LINE", BASE_LINE);
    nh.getParam("/Protokrot/WHEEL_DIAMETER", WHEEL_DIAMETER);
}

Localizator::~Localizator()
{
}


bool Localizator::respawnCallback(Spawn::Request& req, Spawn::Response& res)
{
	ROS_INFO("Respawing ARDLocalizator to x=%f, y=%f and theta=%f", req.x, req.y, req.theta);
	trans.x() = req.x;
	trans.y() = req.y;
	orient = arp_math::Rotation2(req.theta);
	//set time to be unvalid so the odoCallback will reinitiliaze
	last_time = ros::Time(0.0);
	ros::Time t = ros::Time::now();

	publishTransform(t);
	publishOdomTopic(t,0,0,0);
	publishPoseTopic(t,0,0);

	return true;
}

void Localizator::odoCallback(const OdoConstPtr& o)
{
	double dt;
	double odo_left, odo_right;
	double lin_vel,ang_vel;
	double vx,vy;

	// Récupération des données odo
	ros::Time t = ros::Time::now();
	odo_left  = o->odo_left;
	odo_right = o->odo_right;

	dt = (t - last_time).toSec();

	if( !last_time.isZero() && dt >0)
	{
		

		// Calcul des vitesses odo
		double dleft  = odo_left  - last_odo_left ;
		double dright = odo_right - last_odo_right;
		double dl = 0.25 * ( dright + dleft ) * WHEEL_DIAMETER;
		double dth = 0.50 * ( dright - dleft ) * WHEEL_DIAMETER / BASE_LINE;
		lin_vel = dl/dt;
		ang_vel = dth/dt;

		// Calcul de la position réelle
		orient = normalizeAngle(orient * Rotation2(dth));
		Vector2 delta_trans = dl * Vector2(1.0, 0.0);
		vx = (orient * delta_trans).x()/dt;
		vy = (orient * delta_trans).y()/dt;
		trans += orient * delta_trans;
	}
	else
	{
		ROS_INFO("Initialiaze odometry");
		//Last time is going to be initiliaze under to now
		//Last odo values will have current values
		//position is unchanged
		//speed are considered as null
		lin_vel = 0;
		ang_vel = 0;
		vx = 0;
		vy = 0;
	}

	publishTransform(t);
	publishOdomTopic(t,vx,vy,ang_vel);
	publishPoseTopic(t,lin_vel,ang_vel);

	// Buffer
	last_time = t;
	last_odo_right = odo_right;
	last_odo_left  = odo_left;
}

void Localizator::publishTransform( const ros::Time t)
{
	//first, we'll publish the transform over tf
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(orient.angle());
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = t;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = trans.x();
    odom_trans.transform.translation.y = trans.y();
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //send the transform
    odom_tf_pub.sendTransform(odom_trans);
}

void Localizator::publishOdomTopic( const ros::Time t, const double vx, const double vy, const double vth)
{
	nav_msgs::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = "Localizator/odomRos";
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(orient.angle());

    //set the position
    odom.pose.pose.position.x = trans.x();
    odom.pose.pose.position.y = trans.y();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
}

void Localizator::publishPoseTopic( const ros::Time t, const double vl, const double vth)
{
	// Publication de la position estimée
	Pose pose;
	pose.theta = orient.angle();
	pose.x = trans.x();
	pose.y = trans.y();
	pose.linear_velocity = vl;
	pose.angular_velocity = vth;
	pose_pub.publish(pose);
}
