/*
 * UbiquityKinematics.hpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#ifndef UBIQUITYKINEMATICS_HPP_
#define UBIQUITYKINEMATICS_HPP_

#include "UbiquityStates.hpp"
#include "UbiquityParams.hpp"
#include <math/core>

namespace arp_model
{
class UbiquityKinematics
{
    public:
        UbiquityKinematics();

        /**
         * Modèle cinématique direct de tourelle
         * Convertit le couple (domega,omega) exprimé sur les axes des motoréducteurs
         * en un couple (vitesse, angle) exprimé sur le centre de la roue de la tourelle avec v pouvant être négatif et angle dans ]-PI;PI]
         * Il y a recouvrement des configurations mais sans ça, on perd l'info du sens moteur (il faut bien aligner les v+ sur 0 ou PI sur la tourelle !)
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
         * Le modèle tient compte du pilotage multitour et de l'optimisation à réaliser pour trouver la position de tourelle la plus proche
         * quitte à faire domega *= -1 ou omega += PI
         * Le modèle tient compte des 0 calibrés de tourelle.
         * Il utilise motors2Turrets en interne sur iMS
         * @param[in] iTS : Etat des tourelles ie positions des tourelles (traction et direction) exprimés en rad et rad/s.
         *               Tient compte des 0 tourelle calibrés
         * @param[in] iMS : mesure courante de l'état des moteurs (pour prendre en compte le couplage)
         * @param[out] oMS : Etat des moteurs ie positions et vitesses des moteurs (traction et direction) exprimee en rad et rad/s
         *               en sortie de réducteur
         * @param[in] iParams : paramètres géométriques du robot
         * @return : true if computation succeed, false otherwise (param inconsistent for instance)
         */
        static bool turrets2Motors(const TurretState & iTS,
                                const MotorState & iMS,
                                MotorState& oMS,
                                const UbiquityParams & iParams);

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
        static bool turrets2Twist(const TurretState & iTS,
                                    arp_math::Twist2D& oTw,
                                    SlippageReport& oSR,
                                    const UbiquityParams & iParams);

        /**
         * Modèle cinématique direct de la base, version simpliste qui ne prend que 3 mesures pour faire le calcul des 3 coordonées du twist (avec une matrice inversible)
         * ne sert qu'au debug
         */
        static void simpleTurrets2Twist(const TurretState & iTS, arp_math::Twist2D& oTw, SlippageReport& oSR, const UbiquityParams & iParams);

        /**
         * Modèle cinématique direct de la base.
         * Convertit l'état des tourelles en un ICRSpeed (mouvement du repère de référence du chassis par rapport au sol projeté et réduit dans le
         * repère de référence du chassis).\n
         */
        static bool simpleTurrets2ICRspeed(const TurretState & iTS, arp_math::ICRSpeed& oICRs, const UbiquityParams & iParams);


        /**
         * Modèle cinématique indirect de la base
         * Convertit un Twist (Twist du repère de référence du chassis par rapport au sol projeté et réduit dans le repère de référence du chassis)
         * en consignes articulaires pour les tourelles.
         * Le couple(phi,v) de la position/vitesse de chaque tourelle vérifié en sortie : v pouvant etre négatif et phi dans ]-PI/2;PI/2]
         * (il n'y a pas lieu de repérer le sens de v+ parce qu'à ce niveau on a pas encore décidé quelle "face" de la tourelle on va mettre)
         * Charge au modèle de tourelle d'optimiser les déplacements rotation/traction
         * @param[in] iTw : Twist du repère de référence du chassis par rapport au sol projeté et réduit dans le repère de référence du chassis.
         * @param[out] oTS : Etat des tourelles ie positions des tourelles (traction et direction).
         *               Tient compte des 0 tourelle calibrés
         * @param[in] iParams : paramètres géométriques du robot
         * @return : true if computation succeed, false otherwise (param inconsistent for instance)
         */
        static bool twist2Turrets(const arp_math::Twist2D & iTw, TurretState& oTS, const UbiquityParams & iParams);


        /**
         * Echainement des modèles direct de tourelle et cinématique
         * @param oTS : [out] the intermediate computation of Turret State. Should be only used for debug or user feedback
         */
        static bool motors2Twist(const MotorState & iMS,
                TurretState& oTS,
                arp_math::Twist2D& oTw,
                SlippageReport& oSR,
                const UbiquityParams & iParams);

        /**
         * Enchainement des modèles indirects de tourelle et cinématique
         * @param oTS : [out] the intermediate computation of Turret State. Should be only used for debug or user feedback
         */
        static bool twist2Motors(const arp_math::Twist2D & iTw,
                                    const MotorState & iMS,
                                    TurretState& oTS,
                                    MotorState& oMS,
                                    const UbiquityParams & iParams);

        /**
         * Les tourelles permettent de recouvrir l'état de possibles de plusieurs façons lorsqu'elles sont pilotés en marche
         * AV et AR entre -PI et PI. Cette fonction permet de réduire ce recouvrement à un pilotage entre ]-PI/2;PI/2] puis marche AV/AR
         * @param[in] angle : angle en rad à normalizer entre -PI/2 et PI/2
         * @param[in] speed : la vitesse dont il faut éventuellement changer le signe
         */
        static void normalizeDirection(double& angle, double& speed);

        /**
         * Idem que normalizeDirection(double& angle, double& speed) mais sur un ensemble de 3 tourelles
         */
        static void normalizeDirection(UbiquityKinematicState& state);
};

} /* namespace arp_model */
#endif /* UBIQUITYKINEMATICS_HPP_ */
