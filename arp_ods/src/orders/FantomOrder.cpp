/*
 * FantomOrder.cpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#include "FantomOrder.hpp"

using namespace arp_core;
using namespace arp_math;

namespace arp_ods
{

FantomOrder::FantomOrder() :
    MotionOrder()
{
    m_type = FANTOM;

    old_loop_date = 0;
    old_angle_error = 0;

    setFANTOM_COEF(-1);
    setTRANSLATION_GAIN(-1);
    setROTATION_GAIN(-1);
    setROTATION_D_GAIN(-1);
    setVEL_FINAL(-1);

}

FantomOrder::FantomOrder(MotionOrder order) :
    MotionOrder(order)
{
}

void FantomOrder::setDefaults(order::config conf)
{
    MotionOrder::setDefaults(conf);
    setFANTOM_COEF(conf.FANTOM_COEF);
    setTRANSLATION_GAIN(conf.TRANSLATION_GAIN);
    setROTATION_GAIN(conf.ROTATION_GAIN);
    setROTATION_D_GAIN(conf.ROTATION_D_GAIN);
    setVEL_FINAL(conf.VEL_FINAL);
}

shared_ptr<MotionOrder> FantomOrder::createOrder( const OrderGoalConstPtr &goal, Pose currentPose, order::config conf  )
{
    shared_ptr<FantomOrder> order(new FantomOrder());

    Pose begin;
    Pose end;

    begin.x = currentPose.x;
    begin.y = currentPose.y;
    begin.theta = currentPose.theta;
    order->setBeginPose(begin);

    end.x = goal->x_des;
    end.y = goal->y_des;
    end.theta = goal->theta_des;
    order->setEndPose(end);

    order->setReverse(goal->reverse);
    order->setPass(goal->passe);

    order->setDefaults(conf);

    return static_cast<shared_ptr<MotionOrder>  >(order);
}

double FantomOrder::linearReductionCoef(double angle_error)
{
    //le coefficient est 1 si angle est =0, il est 0 si angle=PI/8 au +, avec un smoothstep entre 2
    double result = smoothStep(fabs(angle_error), 1, 0, 0, PI / 8.0);
    //ROS_INFO("smoothStep  error %.3f  =>> abs %.3f ",angle_error,d_abs(angle_error));
    //ROS_INFO("smoothStep abs error %.3f  =>> coef %.3f ",d_abs(angle_error), result);
    return result;
}

double FantomOrder::getRemainingDistance(arp_core::Pose currentPosition)
{
    double result = 0.0;
    Vector2 position(currentPosition.x, currentPosition.y);
    Vector2 trans_des(m_endPose.x, m_endPose.y);
    Rotation2 orient_desLocal_(reversePosition(m_endPose).theta);

    if (m_currentMode == MODE_APPROACH)
    {
        //calcul de la distance à la droite passant par le point desire et perpendiculaire à l'angle desire
        //la distance à la ligne d'arrivée en quelque sorte
        result = (orient_desLocal_.inverse() * (trans_des - position)).x();
    }
    else
    {
        //classical distance
        result = ModeSelector::getRemainingDistance(currentPosition);
    }

    return result;
}

double FantomOrder::getRemainingAngle(arp_core::Pose currentPosition, double distance_error)
{
    double result = 0.0;
    Vector2 trans_des(m_endPose.x, m_endPose.y);
    Vector2 position(currentPosition.x, currentPosition.y);
    Rotation2 orientLocal_(reversePosition(currentPosition).theta);
    Rotation2 orient_desLocal_(reversePosition(m_endPose).theta);
    Vector2 u_x(1.0, 0.0);
    Vector2 phantom_direction_unit;
    Vector2 direction_in_current;
    double angle_error_fant;

    ///////////////////////////////////////////////// ANGLES
    //direction du point fantome

    phantom_direction_unit = orient_desLocal_ * u_x;
    //creation du point fantome
    //il s'agit d'un point qui se situe devant le point final, et suivant l'angle final.
    //c'est lui qu'on va viser en cap
    //ROS_WARN("trans_des (%0.3f,%0.3f)",trans_des.x(),trans_des.y());
    Vector2 phantompoint = trans_des - getFANTOM_COEF() * distance_error * phantom_direction_unit;
    //ROS_WARN("phantompoint (%0.3f,%0.3f)  coef %0.3f  end.thetat %0.3f current.theta %0.3f",phantompoint.x(),phantompoint.y(), getFANTOM_COEF() * distance_error,m_endPose.theta, currentPosition.theta);
    //calcul de l'angle au point fantome
    Vector2 phantom = phantompoint - position;
    //ROS_WARN("phantom (%0.3f,%0.3f)",phantom.x(),phantom.y());
    //direction du point fantome
    direction_in_current = orientLocal_.inverse() * phantom;
    angle_error_fant = atan2(direction_in_current.y(), direction_in_current.x());

    //calcul de l'erreur d'angle par rapport a l'objectif
    double angle_error = ModeSelector::getRemainingAngle(currentPosition);

    //TODO RMO checker que le cas de defaut est bien celui du mode fantome
    if (m_currentMode == MODE_APPROACH || m_currentMode == MODE_PASS )
        result = angle_error;
    else
        result = angle_error_fant;

    //ROS_WARN("angle error (%0.3f) erreur fant (%0.3f) res (%0.3f) m_reverse %d",angle_error,angle_error_fant,result,m_reverse);
    return result;
}

double FantomOrder::getDerivatedAngleError(arp_core::Pose currentPosition, double angle_error)
{
    double d_angle_error;
    double dt = currentPosition.date - old_loop_date;
    if (old_loop_date != 0 && dt > 0)
    {
        d_angle_error = (angle_error - old_angle_error) / dt;
    }
    else
    {
        d_angle_error = 0;
    }
    old_angle_error = angle_error;
    old_loop_date = currentPosition.date;

    return d_angle_error;
}

double FantomOrder::getLinearSpeedCmd(double distance_error, double angle_error)
{
    double v_linear;
    double final_speed;

    //when the mode is configure to pass, there is a minimal speed to keep at the end
    if (getPass())
        final_speed = getVEL_FINAL();
    else
        final_speed = 0;

    v_linear = (getTRANSLATION_GAIN() * sqrt2(distance_error) * linearReductionCoef(angle_error) + final_speed);

    //in reverse mode we have to inverse to sign
    if (m_reverse)
        v_linear *= -1;

    return v_linear;
}

double FantomOrder::getAngularSpeedCmd(double angle_error, double d_angle_error)
{
    return getROTATION_GAIN() * angle_error + getROTATION_D_GAIN() * d_angle_error;
}

Velocity FantomOrder::computeSpeed(arp_core::Pose currentPosition)
{
    Velocity v;

    double distance_error = 0;
    double angle_error = 0;
    double d_angle_error = 0;

    //ROS_WARN("current (%0.3f,%0.3f,%0.3f) target (%0.3f,%0.3f,%0.3f)", currentPosition.x, currentPosition.y, currentPosition.theta, m_endPose.x,m_endPose.y, m_endPose.theta);

    if (m_currentMode == MODE_PASS)
    {
        return m_passSpeed;
    }

    if (m_currentMode == MODE_DONE)
    {
        v.linear = 0;
        v.angular = 0;
        return v;
    }

    //get the distance error for longitudinal control. It handles the change in different modes
    distance_error = getRemainingDistance(currentPosition);
    //get the angle error for orientation control
    angle_error = getRemainingAngle(currentPosition, distance_error);
    // calcul de la derivee de l'angle_error
    d_angle_error = getDerivatedAngleError(currentPosition, angle_error);

    //creation des consignes full patates
    v.linear = getLinearSpeedCmd(distance_error, angle_error);
    v.angular = getAngularSpeedCmd(angle_error, d_angle_error);

    //ROS_INFO("e_d %0.3f e_a %0.3f sens_lin %0.1f", distance_error, angle_error, sens_lin);
    //ROS_INFO("vlf %0.3f vaf %0.3f", lin_vel_cons_full, ang_vel_cons_full);

    //Note  WLA : la saturation est faite dans motion control. A discuter.

    m_lastSpeedCmd = v;

    return v;
}

void FantomOrder::switchInit(arp_core::Pose currentPosition)
{
    // as init is left as soon as it is entered, I allow to put the last init time into m_initTime
    m_initTime = get_time();

    m_beginPose = currentPosition;

    ROS_INFO("switched MODE_INIT --> MODE_RUN automatically");
    m_currentMode = MODE_RUN;
    return;
}

void FantomOrder::switchRun(arp_core::Pose currentPosition)
{
    ModeSelector::switchRun(currentPosition);

    //if a transition to PASS occured, keep the speed
    if (m_currentMode == MODE_PASS)
    {
        m_passSpeed = m_lastSpeedCmd;
    }
}

double FantomOrder::getFANTOM_COEF() const
{
    return FANTOM_COEF;
}

double FantomOrder::getROTATION_D_GAIN() const
{
    return ROTATION_D_GAIN;
}

double FantomOrder::getROTATION_GAIN() const
{
    return ROTATION_GAIN;
}

double FantomOrder::getTRANSLATION_GAIN() const
{
    return TRANSLATION_GAIN;
}

double FantomOrder::getVEL_FINAL() const
{
    return VEL_FINAL;
}

void FantomOrder::setFANTOM_COEF(double FANTOM_COEF)
{
    this->FANTOM_COEF = FANTOM_COEF;
}

void FantomOrder::setROTATION_D_GAIN(double ROTATION_D_GAIN)
{
    this->ROTATION_D_GAIN = ROTATION_D_GAIN;
}

void FantomOrder::setROTATION_GAIN(double ROTATION_GAIN)
{
    this->ROTATION_GAIN = ROTATION_GAIN;
}

void FantomOrder::setTRANSLATION_GAIN(double TRANSLATION_GAIN)
{
    this->TRANSLATION_GAIN = TRANSLATION_GAIN;
}

void FantomOrder::setVEL_FINAL(double VEL_FINAL)
{
    this->VEL_FINAL = VEL_FINAL;
}

}
