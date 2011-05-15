#include "Command.hpp"

using namespace arp_core;

using namespace arp_ods;

using namespace arp_math;

Command::Command() :
	velocity_sub(), command_pub(), lin_vel_(0.0), ang_vel_(0.0), loop_date(0.0) {
	velocity_sub = nh.subscribe("Command/velocity", 1,
			&Command::velocityCallback, this);
	command_pub = nh.advertise<DifferentialCommand> (
			"Protokrot/differential_command", 1);

	if (nh.getParam("/arp_ods/LIN_ACC_MAX", LIN_ACC_MAX) == 0)
		ROS_FATAL("pas reussi a recuperer le parametre LIN_ACC_MAX");
	if (nh.getParam("/arp_ods/LIN_DEC_MAX", LIN_DEC_MAX) == 0)
		ROS_FATAL("pas reussi a recuperer le parametre LIN_DEC_MAX");
	if (nh.getParam("/arp_ods/ANG_ACC_MAX", ANG_ACC_MAX) == 0)
		ROS_FATAL("pas reussi a recuperer le parametre ANG_ACC_MAX");
	if (nh.getParam("/Protokrot/BASE_LINE", BASE_LINE) == 0)
		ROS_FATAL("pas reussi a recuperer le parametre BASE_LINE");
	if (nh.getParam("/Protokrot/WHEEL_DIAMETER", WHEEL_DIAMETER) == 0)
		ROS_FATAL("pas reussi a recuperer le parametre WHEEL_DIAMETER");
}

Command::~Command() {
	;
}

void Command::velocityCallback(const VelocityConstPtr& v) {

	double lin_vel_cons_full = v->linear;
	double ang_vel_cons_full = v->angular;
	double v_right, v_left;
	////////////////////rampage des consignes

	double old_loop_date = loop_date;
	loop_date = ros::Time::now().toSec();
	double delta_date = loop_date - old_loop_date;

	//si la date est "0" c'est que c'est la première callback reçue
	if (old_loop_date != 0 && delta_date > 0) {
		double old_lin_vel = lin_vel_;
		double old_ang_vel = ang_vel_;

		double delta_lin_vel = saturate(
				(lin_vel_cons_full - old_lin_vel) / delta_date, LIN_DEC_MAX,
				LIN_ACC_MAX);
		double delta_ang_vel = saturate(
				(ang_vel_cons_full - old_ang_vel) / delta_date, -ANG_ACC_MAX,
				ANG_ACC_MAX);

		lin_vel_ = old_lin_vel + delta_lin_vel * delta_date;
		ang_vel_ = old_ang_vel + delta_ang_vel * delta_date;

		////////////////creation consigne droite et consigne gauche
		// l'expression usuelle serait:    ( linvel + ang_vel*baseline/2  )  / wheel_radius
		v_right = (2.0 * lin_vel_ + BASE_LINE * ang_vel_) / WHEEL_DIAMETER;
		v_left = (2.0 * lin_vel_ - BASE_LINE * ang_vel_) / WHEEL_DIAMETER;

	}
	//c'est la première fois qu'on commande une vitesse
	else {
		lin_vel_ = 0;
		ang_vel_ = 0;
		v_right = 0;
		v_left = 0;
	}

	DifferentialCommand c;
	c.v_left = v_left;
	c.v_right = v_right;
	command_pub.publish(c);

	//ROS_INFO("lin_vel=%f, ang_vel=%f, v_left=%f, v_right=%f", lin_vel_,
	//        ang_vel_, v_left, v_right);
}

