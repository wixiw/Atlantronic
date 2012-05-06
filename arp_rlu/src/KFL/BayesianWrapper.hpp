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

        /** \ingroup kfl
         *
         * \class UpdateParams
         *
         * \brief BayesianWrapper::UpdateParams est une classe abstraite rassemble les paramètres liés à l'update.
         *
         */
        class UpdateParams
        {

            /**
             * Permet de formatter les paramètres en un message lisible.
             */
            virtual std::string getInfo() const = 0;
        };

        /** \ingroup kfl
         *
         * \class FilterParams
         *
         * \brief BayesianWrapper::FilterParams est une classe abstraite rassemble les paramètres du filtrage bayesien.
         *
         */
        class FilterParams
        {

                /**
                 * Permet de formatter les paramètres en un message lisible.
                 */
                virtual std::string getInfo() const = 0;
        };

        /**
         * Initialise le filtrage bayésien.
         * \param v l'état initial
         * \param c la covariance de l'état initial
         * \param p les paramètres à utiliser
         */
        virtual void init(const KFLStateVar & v, const KFLStateCov & c, BayesianWrapper::FilterParams & p) = 0;

        /**
         * Réalise la prédiction : simulation du système
         * \param i variable d'entrée correspondant à une commande
         * \param cov matrice de covariance de la commande
         * \param dt le pas de temps à simuler
         */
        virtual void predict( const KFLInputVar & i, const KFLInputCov & cov, double dt) = 0;

        /**
         * Réalise la confrontation aux mesures
         * \param m la mesure
         * \param t la cible
         */
        virtual void update(const KFLMeasVar & m, const KFLMeasTarget & t, BayesianWrapper::UpdateParams & p) = 0;

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
