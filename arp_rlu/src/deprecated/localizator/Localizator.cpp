#include "Localizator.hpp"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point32.h>

using namespace arp_math;
using namespace arp_core;
using namespace arp_rlu;


Localizator::Localizator() :
    nh(), odo_sub(), pose_pub(), odom_pub(), odom_tf_pub(), respawn_srv(),setPosition_srv(),
            init_srv(), last_odo_left(0.0), last_odo_right(0.0),
            last_time(0.0),//initialized at a non valid valid on purposes
            trans(Vector2(0.0, 0.0)), orient(Rotation2(0.0))
{
    odo_sub = nh.subscribe("Protokrot/odo", 1, &Localizator::odoCallback, this);
    pose_pub = nh.advertise<arp_core::Pose> ("Localizator/pose", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry> ("Localizator/odomRos", 1);
    m_footprintPublisher = nh.advertise<geometry_msgs::Polygon> ("Localizator/footprint",1);
    respawn_srv = nh.advertiseService("Localizator/respawn",
            &Localizator::respawnCallback, this);
    setPosition_srv = nh.advertiseService("Localizator/setPosition",
                &Localizator::setPositionCallback, this);

    timerreport_srv = nh.advertiseService(ros::this_node::getName() + "/timerReport",
            &Localizator::timerreportCallback, this);

    // Parameters
    if (nh.getParam("/Protokrot/RIGHT_ROTATION_FACTOR", RIGHT_ROTATION_FACTOR) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre RIGHT_ROTATION_FACTOR");
    if (nh.getParam("/Protokrot/LEFT_ROTATION_FACTOR", LEFT_ROTATION_FACTOR) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre LEFT_ROTATION_FACTOR");
    if (nh.getParam("/Protokrot/RIGHT_WHEEL_DIAMETER", RIGHT_WHEEL_DIAMETER) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre RIGHT_WHEEL_DIAMETER");
    if (nh.getParam("/Protokrot/LEFT_WHEEL_DIAMETER", LEFT_WHEEL_DIAMETER) == 0)
        ROS_FATAL("pas reussi a recuperer le parametre LEFT_WHEEL_DIAMETER");

    m_monotonicTimeToRealTime = ros::Time::now().toSec() - getTime();

    ROS_INFO("Localizator: delay from HML time set to %f",m_monotonicTimeToRealTime);

    geometry_msgs::Point32 p1; p1.x = -0.005; p1.y = 0.130;
    m_footprint.points.push_back(p1);
    geometry_msgs::Point32 p2; p2.x = 0.065; p2.y = 0.130;
    m_footprint.points.push_back(p2);
    geometry_msgs::Point32 p3; p3.x = 0.280; p3.y = 0.080;
    m_footprint.points.push_back(p3);
    geometry_msgs::Point32 p4; p4.x = 0.280; p4.y = 0.030;
    m_footprint.points.push_back(p4);
    geometry_msgs::Point32 p6; p6.x = 0.090; p6.y = 0.030;
    m_footprint.points.push_back(p6);
    geometry_msgs::Point32 p7; p7.x = 0.070; p7.y = -0.130;
    m_footprint.points.push_back(p7);
    geometry_msgs::Point32 p8; p8.x = 0.130; p8.y = 0.180;
    m_footprint.points.push_back(p8);
    geometry_msgs::Point32 p9; p9.x = 0.065; p9.y = -0.130;
    m_footprint.points.push_back(p9);
    geometry_msgs::Point32 p5; p5.x = -0.005; p5.y = -0.130;
    m_footprint.points.push_back(p5);
}

Localizator::~Localizator()
{
}

bool Localizator::respawnCallback(Spawn::Request& req, Spawn::Response& res)
{
    ROS_INFO("Respawing ARDLocalizator to x=%f, y=%f and theta=%f", req.x,
            req.y, req.theta);
    trans.x() = req.x;
    trans.y() = req.y;
    orient = arp_math::Rotation2(req.theta);
    //set time to be unvalid so the odoCallback will reinitiliaze
    last_time = 0;
    ros::Time t = ros::Time::now();

    publishTransform(t);
    publishOdomTopic(t, 0, 0, 0);
    publishPoseTopic(t, 0, 0);

    return true;
}

bool Localizator::setPositionCallback(SetPosition::Request& req, SetPosition::Response& res)
{
    ROS_INFO("setPosition ARDLocalizator to x=%f, y=%f and theta=%f", req.x,
            req.y, req.theta);
    trans.x() = req.x;
    trans.y() = req.y;
    orient = arp_math::Rotation2(req.theta);
    //set time to be unvalid so the odoCallback will reinitiliaze
    last_time = 0;
    ros::Time t = ros::Time::now();

    publishTransform(t);
    publishOdomTopic(t, 0, 0, 0);
    publishPoseTopic(t, 0, 0);

    return true;
}

void Localizator::odoCallback(const OdoConstPtr& o)
{
    // timer to scope performances
    timer_.Start();

    double dt;
    double odo_left, odo_right;
    double lin_vel, ang_vel;
    double vx, vy;

    // Récupération des données odo
    odo_left = o->odo_left;
    odo_right = o->odo_right;
    double t = o->time + m_monotonicTimeToRealTime;

    dt = t - last_time;

    if (last_time != 0 && dt > 0)
    {

        // Calcul des vitesses odo
        double dleft = odo_left - last_odo_left;
        double dright = odo_right - last_odo_right;
        double dl = (dright*RIGHT_WHEEL_DIAMETER/2.0 + dleft*LEFT_WHEEL_DIAMETER/2.0) / 2.0;
        double dth = (dright*RIGHT_ROTATION_FACTOR - dleft*LEFT_ROTATION_FACTOR) / 2.0;
        lin_vel = dl / dt;
        ang_vel = dth / dt;

        // Calcul de la position réelle
        orient = normalizeAngle(orient * Rotation2(dth));
        Vector2 delta_trans = dl * Vector2(1.0, 0.0);
        vx = (orient * delta_trans).x() / dt;
        vy = (orient * delta_trans).y() / dt;
        trans += orient * delta_trans;
    }
    else
    {
        ROS_INFO("Initialize odometry t=%f last_t=%f dt=%f",t,last_time,dt);
        //Last time is going to be initiliaze under to now
        //Last odo values will have current values
        //position is unchanged
        //speed are considered as null
        lin_vel = 0;
        ang_vel = 0;
        vx = 0;
        vy = 0;
    }

    publishTransform(ros::Time(t));
    publishOdomTopic(ros::Time(t), vx, vy, ang_vel);
    publishPoseTopic(ros::Time(t), lin_vel, ang_vel);
    publishFootprint(trans.x(),trans.y(),orient.angle());

    // Buffer
    if( dt > 0 )
    {
        last_time = t;
    }
    last_odo_right = odo_right;
    last_odo_left = odo_left;

    // timer to scope performances
    timer_.Stop();
}

void Localizator::publishTransform(const ros::Time t)
{
    //first, we'll publish the transform over tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(
            orient.angle());
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

void Localizator::publishOdomTopic(const ros::Time t, const double vx,
        const double vy, const double vth)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = "/Localizator/odomRos";
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(
            orient.angle());

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

void Localizator::publishPoseTopic(const ros::Time t, const double vl,
        const double vth)
{
    // Publication de la position estimée
    arp_core::Pose pose;
    pose.theta = orient.angle();
    pose.x = trans.x();
    pose.y = trans.y();
    pose.linear_velocity = vl;
    pose.angular_velocity = vth;
    pose.date=t.toSec();
    pose_pub.publish(pose);
}

void Localizator::publishFootprint(double x, double y, double cap)
{
    std::vector<geometry_msgs::Point32> v = m_footprint.points;
    for (std::vector<geometry_msgs::Point32>::iterator it = v.begin(); it!=v.end(); ++it)
    {
        (*it).x += x;
        (*it).y += y;
    }

    geometry_msgs::Polygon p;
    p.points = v;
    m_footprintPublisher.publish(p);

}

bool Localizator::timerreportCallback(TimerReport::Request& req, TimerReport::Response& res)
{
    std::stringstream info;
    info << "==============================================" << std::endl;
    info << ros::this_node::getName() << " Performance Report (ms)" << std::endl;
    info << "----------------------------------------------" << std::endl;
    info << "  [*] Number of samples used : " << timer_.GetRawRefreshTime().size() << std::endl;
    info << "  [*] Actual loop period   : mean=" << timer_.GetMeanRefreshTime() * 1000.0;
    info << "  , stddev=" << timer_.GetStdDevRefreshTime() * 1000.0;
    info << "  , min=" << timer_.GetMinRefreshTime() * 1000.0;
    info << "  , max=" << timer_.GetMaxRefreshTime() * 1000.0;
    info << "  , last=" << timer_.GetLastRefreshTime() * 1000.0 << std::endl;
    /*info << "  [*] Raw actual loop periods :  ( ";
     for(std::vector<double>::const_iterator it = timer_.GetRawRefreshTime().begin(); it != timer_.GetRawRefreshTime().end(); ++it)
     info << (*it) * 1000.0 << " ";
     info << " )" << std::endl;*/
    info << "  [*] Loop duration    : mean=" << timer_.GetMeanElapsedTime() * 1000.0;
    info << "  , stddev=" << timer_.GetStdDevElapsedTime() * 1000.0;
    info << "  , min=" << timer_.GetMinElapsedTime() * 1000.0;
    info << "  , max=" << timer_.GetMaxElapsedTime() * 1000.0;
    info << "  , last=" << timer_.GetLastElapsedTime() * 1000.0 << std::endl;
    /*info << "  [*] Raw loop durations :  ( ";
     for(std::vector<double>::const_iterator it = timer_.GetRawElapsedTime().begin(); it != timer_.GetRawElapsedTime().end(); ++it)
     info << (*it) * 1000.0 << " ";
     info << " )" << std::endl; */

    res.report = info.str();
    return true;
}
