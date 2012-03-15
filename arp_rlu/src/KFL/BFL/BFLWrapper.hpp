/*
 * BFLWrapper.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_BFLWRAPPER_HPP_
#define _ARP_RLU_KFL_BFLWRAPPER_HPP_

#include <math/math.hpp>
#include <KFL/BayesianWrapper.hpp>

namespace arp_rlu
{

namespace kfl
{
/** \ingroup kfl
 *
 * \class BFLWrapper
 *
 * \brief Cette classe implémente l'interface BayesianWrapper à l'aide de la BFL.
 */
class BFLWrapper : public BayesianWrapper
{
    public:

    /**
     * Initialise le filtrage bayésien.
     * \param v l'état initial
     * \param c la covariance de l'état initial
     */
    void init(KFLStateVar, KFLStateCov);

    /**
     * Réalise la prédiction : simulation du système
     * \param i variable d'entrée correspondant à une commande
     */
    void predict( KFLSysInput );

    /**
     * Réalise la confrontation aux mesures
     * \param m la mesure
     * \param c la covariance de la mesure
     * \param t la cible mesurée
     */
    void update(KFLMeasVar, KFLMeasCov, KFLMeasTarget);

    /**
     * Permet d'obtenir la dernière estimée
     * \return la dernière estimée
     */
    KFLStateVar getEstimate() const;

    /**
     * Permet d'obtenir la covariance de la dernière estimée
     * \return la covariance de la dernière estimée
     */
    KFLStateCov getCovariance() const;

    protected:
    KFLStateVar var;
    KFLStateCov cov;


};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_BFLWRAPPER_HPP_ */
