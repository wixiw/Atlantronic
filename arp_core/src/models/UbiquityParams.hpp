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
        double getMaxDrivingDec() const;
        double getMaxDrivingSpeed() const;
        double getMaxDrivingTorque() const;
        double getMaxSteeringAcc() const;
        double getMaxSteeringDec() const;
        double getMaxSteeringSpeed() const;
        double getMaxSteeringTorque() const;

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
        double& getMaxDrivingDecRef();
        double& getMaxDrivingSpeedRef();
        double& getMaxDrivingTorqueRef();
        double& getMaxSteeringAccRef();
        double& getMaxSteeringDecRef();
        double& getMaxSteeringSpeedRef();
        double& getMaxSteeringTorqueRef();

        void setLeftTurretZero(double leftTurretZero);
        void setRearTurretZero(double rearTurretZero);
        void setRightTurretZero(double rightTurretZero);
        void setLeftWheelDiameter(double leftWheelDiameter);
        void setRearWheelDiameter(double rearWheelDiameter);
        void setRightWheelDiameter(double rightWheelDiameter);


    protected:
        /** Position du zero de la tourelle en rad lorsque le moteur est sur le top tour*/
        double m_leftTurretZero;
        /** Position du zero de la tourelle en rad lorsque le moteur est sur le top tour*/
        double m_rightTurretZero;
        /** Position du zero de la tourelle en rad lorsque le moteur est sur le top tour*/
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
        /** Deceleration maximale en m/s2 qu'une roue peut fournir au sol */
        double m_maxDrivingDec;
        /** Couple maximal en Nm qu'une roue peut fournir au sol */
        double m_maxDrivingTorque;


        /**
         * Note : on peut exprimer les limitations cinématiques sur la tourelle parce qu'il n'y a pas de couplage côté direction
         */
        /** Vitesse maximale en rad/s que la tourelle peut avoir par rapport au chassis */
        double m_maxSteeringSpeed;
        /** Acceleration maximale en rad/s2 que la tourelle peut avoir par rapport au chassis */
        double m_maxSteeringAcc;
        /** Decceleration maximale en rad/s2 que la tourelle peut avoir par rapport au chassis */
        double m_maxSteeringDec;
        /** Couple maximal en Nm que la tourelle peut fournir par rapport au chassis */
        double m_maxSteeringTorque;

};

} /* namespace arp_core */
#endif /* UBIQUITYPARAMS_HPP_ */
