/*
 * UbiquityParams.hpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#ifndef UBIQUITYPARAMS_HPP_
#define UBIQUITYPARAMS_HPP_

#include <math/core>

namespace arp_model
{

class UbiquityParams
{
    public:
        UbiquityParams();

        /** this function is used by unit tests to create a default parameters list*/
        void fillWithFakeValues(void);

        //use this to check to validity of params
        bool check() const;

        double getLeftTurretZero() const;
        double getRearTurretZero() const;
        double getRightTurretZero() const;
        arp_math::Pose2D getLeftTurretPosition() const;
        arp_math::Pose2D getRearTurretPosition() const;
        arp_math::Pose2D getRightTurretPosition() const;
        arp_math::Pose2D getChassisCenter() const;
        double getLeftWheelDiameter() const;
        double getRearWheelDiameter() const;
        double getRightWheelDiameter() const;
        double getTractionRatio() const;
        double getTurretRatio() const;
        double getMaxDrivingAcc() const;
        double getMaxDrivingSpeed() const;
        double getMaxDrivingTorque() const;
        double getMaxSteeringAcc() const;
        double getMaxSteeringSpeed() const;
        double getMaxSteeringTorque() const;

        double getMaxSteeringMotorAcc() const;
        double getMaxSteeringMotorSpeed() const;
        double getMaxDrivingMotorAcc() const;
        double getMaxDrivingMotorSpeed() const;

        double getMaxRobotAccel() const;
        double getMaxRobotSpeed() const;

        double& getLeftTurretZeroRef();
        double& getRearTurretZeroRef();
        double& getRightTurretZeroRef();
        arp_math::Pose2D& getLeftTurretPositionRef();
        arp_math::Pose2D& getRearTurretPositionRef();
        arp_math::Pose2D& getRightTurretPositionRef();
        arp_math::Pose2D& getChassisCenterRef();
        double& getLeftWheelDiameterRef();
        double& getRearWheelDiameterRef();
        double& getRightWheelDiameterRef();
        double& getTractionRatioRef();
        double& getTurretRatioRef();
        double& getMaxDrivingAccRef();
        double& getMaxDrivingSpeedRef();
        double& getMaxDrivingTorqueRef();
        double& getMaxSteeringAccRef();
        double& getMaxSteeringSpeedRef();
        double& getMaxSteeringTorqueRef();
        double& getMaxRobotAccelRef();
        double& getMaxRobotSpeedRef();

        void setLeftTurretZero(double leftTurretZero);
        void setRearTurretZero(double rearTurretZero);
        void setRightTurretZero(double rightTurretZero);
        void setLeftWheelDiameter(double leftWheelDiameter);
        void setRearWheelDiameter(double rearWheelDiameter);
        void setRightWheelDiameter(double rightWheelDiameter);

        void setMaxRobotAccel(double maxRobotAccel);
        void setMaxRobotSpeed(double maxRobotSpeed);

    protected:
        /** Position du moteur de direction en rad lorsque le moteur est sur le top tour. ATTENTION : peut être en dehors de -PI/PI*/
        double m_leftTurretZero;
        /** Position du moteur de direction en rad lorsque le moteur est sur le top tour. ATTENTION : peut être en dehors de -PI/PI*/
        double m_rightTurretZero;
        /** Position du moteur de direction en rad lorsque le moteur est sur le top tour. ATTENTION : peut être en dehors de -PI/PI*/
        double m_rearTurretZero;

        /** Position de la tourelle avant gauche sur le chassis en (m,m,rad) */
        arp_math::Pose2D m_leftTurretPosition;
        /** Position de la tourelle avant droite sur le chassis en (m,m,rad) */
        arp_math::Pose2D m_rightTurretPosition;
        /** Position de la tourelle arriere sur le chassis en (m,m,rad) */
        arp_math::Pose2D m_rearTurretPosition;
        /** Position du centre du chassis en (m,m,rad) */
        arp_math::Pose2D m_chassisCenter;

        /** taille de la roue avant gauche en m*/
        double m_leftWheelDiameter;
        /** taille de la roue avant droite en m*/
        double m_rightWheelDiameter;
        /** taille de la roue arriere en m*/
        double m_rearWheelDiameter;

        /** Rapport de direction position tourelle = rappport*position moteur */
        double m_turretRatio;
        /** Rapport de traction vitesse rotation roue = rappport*vitesse moteur */
        double m_tractionRatio;

        /**
         * Note : on ne peut pas exprimer les limitations cinématiques autrement que sur les moteurs pour la direction à cause
         * du couplage avec les vitesses de directions qui nous font dépendre du temps.
         */
        /** Vitesse maximale en m/s qu'une roue peut fournir au sol */
        double m_maxDrivingSpeed;
        /** Acceleration maximale en m/s2 qu'une roue peut fournir au sol */
        double m_maxDrivingAcc;
        /** Couple maximal en Nm qu'une roue peut fournir au sol */
        double m_maxDrivingTorque;

        /**
         * Note : on peut exprimer les limitations cinématiques sur la tourelle parce qu'il n'y a pas de couplage côté direction
         */
        /** Vitesse maximale en rad/s que la tourelle peut avoir par rapport au chassis */
        double m_maxSteeringSpeed;
        /** Acceleration maximale en rad/s2 que la tourelle peut avoir par rapport au chassis */
        double m_maxSteeringAcc;
        /** Couple maximal en Nm que la tourelle peut fournir par rapport au chassis */
        double m_maxSteeringTorque;

        /** Vitesse maximale du robot, exprimee en ro de l'ICRSpeed */
        double m_maxRobotSpeed;
        /** Acceleration maximale du robot, exprime en derivee de ro de l'ICRSpeed */
        double m_maxRobotAccel;

};

} /* namespace arp_core */
#endif /* UBIQUITYPARAMS_HPP_ */
