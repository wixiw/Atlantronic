/*
 * UbiquityKinematics.hpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#ifndef UBIQUITYKINEMATICS_HPP_
#define UBIQUITYKINEMATICS_HPP_

#include "UbiquityParams.hpp"

namespace arp_core
{

struct MotorCommands
{
        double leftDrivingMotorSpeed;
        double rightDrivingMotorSpeed;
        double rearDrivingMotorSpeed;
        double leftSteeringMotorPosition;
        double rightSteeringMotorPosition;
        double rearSteeringMotorPosition;

        MotorCommands():
            leftDrivingMotorSpeed(0.0),
            rightDrivingMotorSpeed(0.0),
            rearDrivingMotorSpeed(0.0),
            leftSteeringMotorPosition(0.0),
            rightSteeringMotorPosition(0.0),
            rearSteeringMotorPosition(0.0)
        {}
};

struct TurretCommands
{
        double leftDrivingTurretSpeed;
        double rightDrivingTurretSpeed;
        double rearDrivingTurretSpeed;
        double leftSteeringTurretPosition;
        double rightSteeringTurretPosition;
        double rearSteeringTurretPosition;

        TurretCommands():
            leftDrivingTurretSpeed(0.0),
            rightDrivingTurretSpeed(0.0),
            rearDrivingTurretSpeed(0.0),
            leftSteeringTurretPosition(0.0),
            rightSteeringTurretPosition(0.0),
            rearSteeringTurretPosition(0.0)
        {}
};

struct CouplingSpeeds
{
        double leftSteeringMotorSpeed;
        double rightSteeringMotorSpeed;
        double rearSteeringMotorSpeed;

        CouplingSpeeds():
            leftSteeringMotorSpeed(0.0),
            rightSteeringMotorSpeed(0.0),
            rearSteeringMotorSpeed(0.0)
        {}
};

class UbiquityKinematics
{
    public:
        UbiquityKinematics();

        /**
         * Modèle direct de tourelle
         * Convertit le couple (domega,omega) exprimé sur les axes des motoréducteurs
         * en un couple (vitesse, angle) exprimé sur le centre de la roue de la tourelle
         * @param inputs : vitesse des moteurs de traction exprimee en rad/s en sortie de réducteur
         *              et positions des moteurs de direction exprime en rad en sortie du réducteur
         * @param outputs : vitesse de traction du chassis exprimée en mm/s par rapport au sol sur chaque tourelle et
         *               angles des tourelles par rapport au chassis en rad. Tient compte des 0 tourelle calibrés
         * @param turretSpeeds : vitesse de direction de tourelle pour le couplage
         * @param params : paramètres géométriques du robot
         * @return : true if computation succeed, false otherwise (param inconsistent for instance)
         */
        static bool motors2Turrets(MotorCommands const inputs, TurretCommands& outputs, CouplingSpeeds const turretSpeeds,  UbiquityParams const params);

        /**
         * Modèle inverse de tourelle
         * Convertit le couple (vitesse, angle) exprimé sur le centre de la roue de la tourelle
         * en un couple (domega,omega) exprimé sur les axes des motoréducteurs
         * @param inputs : vitesse de traction du chassis exprimée en mm/s par rapport au sol sur chaque tourelle et
         *               angles des tourelles par rapport au chassis en rad. Tient compte des 0 tourelle calibrés
         * @param outputs : vitesse des moteurs de traction exprimee en rad/s en sortie de réducteur
         *              et positions des moteurs de direction exprime en rad en sortie du réducteur
         * @param turretSpeeds : vitesse de direction de tourelle pour le couplage
         * @param params : paramètres géométriques du robot
         * @return : true if computation succeed, false otherwise (param inconsistent for instance)
         */
        static bool turrets2Motors(TurretCommands const inputs, MotorCommands& outputs, CouplingSpeeds const turretSpeeds,  UbiquityParams const params);

        static bool turrets2Twist(UbiquityParams const params);
        static bool twist2Turrets(UbiquityParams const params);

};

} /* namespace arp_core */
#endif /* UBIQUITYKINEMATICS_HPP_ */
