#include "OldLocalizator.hpp"

using namespace arp_math;
using namespace arp_core;
using namespace arp_rlu;

OldLocalizator::OldLocalizator():
	nh(),
	odo_sub(),
	pose_pub(),
	respawn_srv(),
	init_srv(),
	last_odo_left(0.0),
	last_odo_right(0.0),
	last_time(),
	duration(),
	trans(),
	orient(0.0)
{
	odo_sub = nh.subscribe("Protokrot/odo", 1, &OldLocalizator::odoCallback, this);
	pose_pub = nh.advertise<Pose>("Localizator/pose", 1);
	respawn_srv = nh.advertiseService("Localizator/respawn", &OldLocalizator::respawnCallback, this);


	// Initialize
	trans = Vector2(0.0, 0.0);
	orient = Rotation2(0.0);
	last_time = ros::WallTime::now();
	last_odo_left = 0.0;
	last_odo_right = 0.0;

    // Parameters
    nh.getParam("/Protokrot/BASE_LINE", BASE_LINE);
    nh.getParam("/Protokrot/WHEEL_DIAMETER", WHEEL_DIAMETER);
}

OldLocalizator::~OldLocalizator()
{
}


bool OldLocalizator::respawnCallback(Spawn::Request& req, Spawn::Response& res)
{
	ROS_INFO("Respawing ARDOldLocalizator to x=%f, y=%f and theta=%f", req.x, req.y, req.theta);
	trans.x() = req.x;
	trans.y() = req.y;
	orient = arp_math::Rotation2(req.theta);
	last_time = ros::WallTime::now();
	last_odo_left = 0.0;
	last_odo_right = 0.0;



	return true;
}

void OldLocalizator::odoCallback(const OdoConstPtr& o)
{
	ros::WallTime t = ros::WallTime::now();
	double dt = (t - last_time).toSec();

	// Récupération des données odo
	double odo_left  = o->odo_left;
	double odo_right = o->odo_right;

	// Calcul des vitesses odo
	double v_left  = (odo_left  - last_odo_left ) / dt;
	double v_right = (odo_right - last_odo_right) / dt;
	double lin_vel = 0.25 * ( v_right + v_left ) * WHEEL_DIAMETER;
	double ang_vel = 0.50 * ( v_right - v_left ) * WHEEL_DIAMETER / BASE_LINE;

	// Calcul de la position réelle
	orient = orient * arp_math::Rotation2(ang_vel * dt);
	arp_math::Vector2 delta_trans = lin_vel * dt * arp_math::Vector2(1.0, 0.0);
	trans += orient * delta_trans;

	// Publication de la position estimée
	Pose pose;
	pose.theta = orient.angle();
	pose.x = trans.x();
	pose.y = trans.y();
	pose.linear_velocity = lin_vel;
	pose.angular_velocity = ang_vel;
	pose_pub.publish(pose);

	// Buffer
	last_time = t;
	last_odo_right = odo_right;
	last_odo_left  = odo_left;
}

