/*
 * UbiquityParams.hpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#ifndef UBIQUITYPARAMS_HPP_
#define UBIQUITYPARAMS_HPP_

#include <math/core>

using namespace arp_math;

namespace arp_core
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
        Pose2D getLeftTurretPosition() const;
        Pose2D getRearTurretPosition() const;
        Pose2D getRightTurretPosition() const;
        Pose2D getChassisCenter() const;
        double getLeftWheelDiameter() const;
        double getRearWheelDiameter() const;
        double getRightWheelDiameter() const;
        double getTractionRatio() const;
        double getTurretRatio() const;
        double getMaxTractionAcc() const;
        double getMaxTractionDec() const;
        double getMaxTractionSpeed() const;
        double getMaxTractionTorque() const;
        double getMaxTurretAcc() const;
        double getMaxTurretDec() const;
        double getMaxTurretSpeed() const;
        double getMaxTurretTorque() const;

        double& getLeftTurretZeroRef();
        double& getRearTurretZeroRef();
        double& getRightTurretZeroRef();
        Pose2D& getLeftTurretPositionRef();
        Pose2D& getRearTurretPositionRef();
        Pose2D& getRightTurretPositionRef();
        Pose2D& getChassisCenterRef();
        double& getLeftWheelDiameterRef();
        double& getRearWheelDiameterRef();
        double& getRightWheelDiameterRef();
        double& getTractionRatioRef();
        double& getTurretRatioRef();
        double& getMaxTractionAccRef();
        double& getMaxTractionDecRef();
        double& getMaxTractionSpeedRef();
        double& getMaxTractionTorqueRef();
        double& getMaxTurretAccRef();
        double& getMaxTurretDecRef();
        double& getMaxTurretSpeedRef();
        double& getMaxTurretTorqueRef();

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
        Pose2D m_leftTurretPosition;
        /** Position de la tourelle avant droite sur le chassis en (m,m,rad) */
        Pose2D m_rightTurretPosition;
        /** Position de la tourelle arriere sur le chassis en (m,m,rad) */
        Pose2D m_rearTurretPosition;
        /** Position du centre du chassis en (m,m,rad) */
        Pose2D m_chassisCenter;

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

        /** Vitesse maximale en m/s qu'une roue peut fournir au sol */
        double m_maxTractionSpeed;
        /** Vitesse maximale en rad/s que la tourelle peut avoir par rapport au chassis */
        double m_maxTurretSpeed;
        /** Acceleration maximale en m/s2 qu'une roue peut fournir au sol */
        double m_maxTractionAcc;
        /** Acceleration maximale en rad/s2 que la tourelle peut avoir par rapport au chassis */
        double m_maxTurretAcc;
        /** Deceleration maximale en m/s2 qu'une roue peut fournir au sol */
        double m_maxTractionDec;
        /** Decceleration maximale en rad/s2 que la tourelle peut avoir par rapport au chassis */
        double m_maxTurretDec;
        /** Couple maximal en Nm qu'une roue peut fournir au sol */
        double m_maxTractionTorque;
        /** Couple maximal en Nm que la tourelle peut fournir par rapport au chassis */
        double m_maxTurretTorque;

};

} /* namespace arp_core */
#endif /* UBIQUITYPARAMS_HPP_ */
