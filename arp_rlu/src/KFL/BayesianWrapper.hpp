/*
 * BayesianWrapper.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_BAYESIANWRAPPER_HPP_
#define _ARP_RLU_KFL_BAYESIANWRAPPER_HPP_

#include <math/core>

#include <KFL/KFLVariables.hpp>

namespace arp_rlu
{

namespace kfl
{
/** \ingroup kfl
 *
 * \interface BayesianWrapper
 *
 * \brief Cette classe abstraite regroupe les appels à un framework de filtrage bayésien.
 *
 * L'instanciation de cette classe abstraite permet de rendre la classe KFLocalizator agnostique du framework
 * de filtrage bayésien utilisé.
 */
class BayesianWrapper
{
    public:
        /**
         * Initialise le filtrage bayésien.
         * \param t la date d'origine
         * \param v l'état initial
         * \param c la covariance de l'état initial
         */
        virtual void init(double t, KFLStateVar v, KFLStateCov c) = 0;

        /**
         * Réalise la prédiction : simulation du système
         * \param dt âge de la dernière estimée
         * \param i variable d'entrée correspondant à une vitesse
         */
        virtual void predict(double dt, KFLSysInput i) = 0;

        /**
         * Réalise la confrontation aux mesures
         * \param m la mesure
         * \param c la covariance de la mesure
         * \param t la cible mesurée
         */
        virtual void update(KFLMeasVar m, KFLMeasCov c, KFLMeasTarget t) = 0;

        /**
         * Permet d'obtenir la dernière estimée
         * \return la dernière estimée
         */
        virtual KFLStateVar getEstimate() const = 0;

        /**
         * Permet d'obtenir la covariance de la dernière estimée
         * \return la covariance de la dernière estimée
         */
        virtual KFLStateCov getCovariance() const = 0;
};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_BAYESIANWRAPPER_HPP_ */
