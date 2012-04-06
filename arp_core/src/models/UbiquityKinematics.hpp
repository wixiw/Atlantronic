/*
 * UbiquityKinematics.hpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#ifndef UBIQUITYKINEMATICS_HPP_
#define UBIQUITYKINEMATICS_HPP_

#include "UbiquityParams.hpp"
#include <math/core>

namespace arp_model
{

struct SteeringMotorPositions
{
        double leftSteeringMotorPosition;
        double rightSteeringMotorPosition;
        double rearSteeringMotorPosition;

        SteeringMotorPositions():
            leftSteeringMotorPosition(0.0),
            rightSteeringMotorPosition(0.0),
            rearSteeringMotorPosition(0.0)
        {}
};

struct DrivingMotorPositions
{
        double leftDrivingMotorPosition;
        double rightDrivingMotorPosition;
        double rearDrivingMotorPosition;

        DrivingMotorPositions():
            leftDrivingMotorPosition(0.0),
            rightDrivingMotorPosition(0.0),
            rearDrivingMotorPosition(0.0)
        {}
};

struct SteeringMotorVelocities
{
        double leftSteeringMotorVelocity;
        double rightSteeringMotorVelocity;
        double rearSteeringMotorVelocity;

        SteeringMotorVelocities():
            leftSteeringMotorVelocity(0.0),
            rightSteeringMotorVelocity(0.0),
            rearSteeringMotorVelocity(0.0)
        {}
};

struct DrivingMotorVelocities
{
        double leftDrivingMotorVelocity;
        double rightDrivingMotorVelocity;
        double rearDrivingMotorVelocity;

        DrivingMotorVelocities():
            leftDrivingMotorVelocity(0.0),
            rightDrivingMotorVelocity(0.0),
            rearDrivingMotorVelocity(0.0)
        {}
};


struct MotorState : public  DrivingMotorPositions,
                            DrivingMotorVelocities,
                            SteeringMotorPositions,
                            SteeringMotorVelocities
{
        MotorState()
        : DrivingMotorPositions()
        , DrivingMotorVelocities()
        , SteeringMotorPositions()
        , SteeringMotorVelocities()
        {}

        std::string toString() const
        {
            std::stringstream ss;
            ss << "Driving : " << "(" << leftDrivingMotorPosition << ","
               << rightDrivingMotorPosition << ","
               << rearDrivingMotorPosition << ") m" << ","
               << "(" << leftDrivingMotorVelocity << ","
               << rightDrivingMotorVelocity << ","
               << rearDrivingMotorVelocity << ") m/s" << " - "
               << "Steering : " << "(" << leftSteeringMotorPosition << ","
               << rightSteeringMotorPosition << ","
               << rearSteeringMotorPosition << ") rad" << ","
               << "(" << leftSteeringMotorVelocity << ","
               << rightSteeringMotorVelocity << ","
               << rearSteeringMotorVelocity << ") rad/s";
            return ss.str();
        }
};



struct SteeringTurretPositions
{
        double leftSteeringTurretPosition;
        double rightSteeringTurretPosition;
        double rearSteeringTurretPosition;

        SteeringTurretPositions():
            leftSteeringTurretPosition(0.0),
            rightSteeringTurretPosition(0.0),
            rearSteeringTurretPosition(0.0)
        {}
};


struct DrivingTurretPositions
{
        double leftDrivingTurretPosition;
        double rightDrivingTurretPosition;
        double rearDrivingTurretPosition;

        DrivingTurretPositions():
            leftDrivingTurretPosition(0.0),
            rightDrivingTurretPosition(0.0),
            rearDrivingTurretPosition(0.0)
        {}
};


struct SteeringTurretVelocities
{
        double leftSteeringTurretVelocity;
        double rightSteeringTurretVelocity;
        double rearSteeringTurretVelocity;

        SteeringTurretVelocities():
            leftSteeringTurretVelocity(0.0),
            rightSteeringTurretVelocity(0.0),
            rearSteeringTurretVelocity(0.0)
        {}
};


struct DrivingTurretVelocities
{
        double leftDrivingTurretVelocity;
        double rightDrivingTurretVelocity;
        double rearDrivingTurretVelocity;

        DrivingTurretVelocities():
            leftDrivingTurretVelocity(0.0),
            rightDrivingTurretVelocity(0.0),
            rearDrivingTurretVelocity(0.0)
        {}
};


struct TurretState : public  DrivingTurretPositions,
                            DrivingTurretVelocities,
                            SteeringTurretPositions,
                            SteeringTurretVelocities
{
        TurretState()
        : DrivingTurretPositions()
        , DrivingTurretVelocities()
        , SteeringTurretPositions()
        , SteeringTurretVelocities()
        {}

        std::string toString() const
        {
            std::stringstream ss;
            ss << "Driving : " << "(" << leftDrivingTurretPosition << ","
               << rightDrivingTurretPosition << ","
               << rearDrivingTurretPosition << ") m" << ","
               << "(" << leftDrivingTurretVelocity << ","
               << rightDrivingTurretVelocity << ","
               << rearDrivingTurretVelocity << ") m/s" << " - "
               << "Steering : " << "(" << leftSteeringTurretPosition << ","
               << rightSteeringTurretPosition << ","
               << rearSteeringTurretPosition << ") rad" << ","
               << "(" << leftSteeringTurretVelocity << ","
               << rightSteeringTurretVelocity << ","
               << rearSteeringTurretVelocity << ") rad/s";
            return ss.str();
        }
};


struct SlippageReport
{
        double kernelQuality;
};

class UbiquityKinematics
{
    public:
        UbiquityKinematics();

        /**
         * Modèle cinématique direct de tourelle
         * Convertit le couple (domega,omega) exprimé sur les axes des motoréducteurs
         * en un couple (vitesse, angle) exprimé sur le centre de la roue de la tourelle
         * @param[in] iMS : Etat des moteurs ie positions et vitesses des moteurs (traction et direction) exprimee en rad et rad/s
         *               en sortie de réducteur
         * @param[out] oTS : Etat des tourelles ie positions des tourelles (traction et direction) exprimés en rad et rad/s.
         *               Tient compte des 0 tourelle calibrés
         * @param[in] iParams : paramètres géométriques du robot
         * @return : true if computation succeed, false otherwise (param inconsistent for instance)
         */
        static bool motors2Turrets(const MotorState & iMS, TurretState& oTS, const UbiquityParams & iParams);

        /**
         * Modèle cinématique inverse de tourelle
         * Convertit le couple (vitesse, angle) exprimé sur le centre de la roue de la tourelle
         * en un couple (domega,omega) exprimé sur les axes des motoréducteurs
         * @param[in] iTS : Etat des tourelles ie positions des tourelles (traction et direction) exprimés en rad et rad/s.
         *               Tient compte des 0 tourelle calibrés
         * @param[in] iSMV : Vitesses courantes des moteurs de direction (pour prendre en compte le couplage)
         * @param[out] oMS : Etat des moteurs ie positions et vitesses des moteurs (traction et direction) exprimee en rad et rad/s
         *               en sortie de réducteur
         * @param[in] iParams : paramètres géométriques du robot
         * @return : true if computation succeed, false otherwise (param inconsistent for instance)
         */
        static bool turrets2Motors(const TurretState & iTS, const SteeringMotorVelocities & iSMV, MotorState& oMS, const UbiquityParams & iParams);

        /**
         * Modèle cinématique direct de la base.
         * Convertit l'état des tourelles en un Twist (Twist du repère de référence du chassis par rapport au sol projeté et réduit dans le
         * repère de référence du chassis).\n
         * En parrallèle, un rapport sur les glissements est publié.
         * @remark : cette fonction correspond à la fonction d'odométrie.
         * @param[in] iTS : Etat des tourelles ie positions des tourelles (traction et direction) exprimés en rad et rad/s.
         *               Tient compte des 0 tourelle calibrés
         * @param[out] oTw : Twist du repère de référence du chassis par rapport au sol projeté et réduit dans le repère de référence du chassis.
         * @param[out] oSR : donne une information sur le taux de glissement du robot.
         * @param[in] iParams : paramètres géométriques du robot
         * @return : true if computation succeed, false otherwise (param inconsistent for instance)
         */
        static bool turrets2Twist(const TurretState & iTS, arp_math::Twist2D& oTw, SlippageReport& oSR, const UbiquityParams & iParams);

        /**
         * Modèle cinématique indirect de la base
         * Convertit un Twist (Twist du repère de référence du chassis par rapport au sol projeté et réduit dans le repère de référence du chassis)
         * en consignes articulaires pour les tourelles.
         * à destination du modèle de tourelle.
         * @param[in] iTw : Twist du repère de référence du chassis par rapport au sol projeté et réduit dans le repère de référence du chassis.
         * @param[out] oTS : Etat des tourelles ie positions des tourelles (traction et direction).
         *               Tient compte des 0 tourelle calibrés
         * @param[in] iParams : paramètres géométriques du robot
         * @return : true if computation succeed, false otherwise (param inconsistent for instance)
         */
        static bool twist2Turrets(const arp_math::Twist2D & iTw, TurretState& oTS, const UbiquityParams & iParams);

        /**
         * Les tourelles permettent de recouvrir l'état de possibles de plusieurs façons lorsqu'elles sont pilotés en marche
         * AV et AR entre -PI et PI. Cette fonction permet de réduire ce recouvrement à un pilotage entre -PI/2 et PI/2 puis marche AV/AR
         * @param[in] angle : angle en rad à normalizer entre -PI/2 et PI/2
         * @param[in] speed : la vitesse dont il faut éventuellement changer le signe
         */
        static void normalizeDirection(double& angle, double& speed);
};

} /* namespace arp_core */
#endif /* UBIQUITYKINEMATICS_HPP_ */
