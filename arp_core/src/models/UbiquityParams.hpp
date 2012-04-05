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

        void setLeftTurretZero(double leftTurretZero);
        void setRearTurretZero(double rearTurretZero);
        void setRightTurretZero(double rightTurretZero);
        void setLeftTurretPosition(Pose2D leftTurretPosition);
        void setRearTurretPosition(Pose2D rearTurretPosition);
        void setRightTurretPosition(Pose2D rightTurretPosition);
        void setLeftWheelDiameter(double leftWheelDiameter);
        void setRearWheelDiameter(double rearWheelDiameter);
        void setRightWheelDiameter(double rightWheelDiameter);
        void setTractionRatio(double tractionRatio);
        void setTurretRatio(double turretRatio);
        void setChassisCenter(Pose2D chassisCenter);

    protected:
        /** Position de la tourelle en rad lorsque le moteur est sur le top tour*/
        double m_leftTurretZero;
        /** Position de la tourelle en rad lorsque le moteur est sur le top tour*/
        double m_rightTurretZero;
        /** Position de la tourelle en rad lorsque le moteur est sur le top tour*/
        double m_rearTurretZero;

        /** Position de la tourelle avant gauche sur le chassis */
        Pose2D m_leftTurretPosition;
        /** Position de la tourelle avant droite sur le chassis */
        Pose2D m_rightTurretPosition;
        /** Position de la tourelle arriere sur le chassis */
        Pose2D m_rearTurretPosition;
        /** Position du centre du chassis */
        Pose2D m_chassisCenter;

        /** taille de la roue avant gauche*/
        double m_leftWheelDiameter;
        /** taille de la roue avant droite*/
        double m_rightWheelDiameter;
        /** taille de la roue arriere*/
        double m_rearWheelDiameter;

        /** Rapport de direction position tourelle = rappport*position moteur */
        double m_turretRatio;
        /** Rapport de traction vitesse rotation roue = rappport*vitesse moteur */
        double m_tractionRatio;
};

} /* namespace arp_core */
#endif /* UBIQUITYPARAMS_HPP_ */
