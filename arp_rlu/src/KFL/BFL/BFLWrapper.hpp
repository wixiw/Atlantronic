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

#include <KFL/BFL/BFLSysConditionalPdf.hpp>
#include <KFL/BFL/BFLMeasConditionalPdf.hpp>

// BFL includes
#include <filter/iteratedextendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/analyticmeasurementmodel_gaussianuncertainty.h>


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
    /** \ingroup kfl
     *
     * \class PredictParams
     *
     * \brief BFLWrapper::PredictParams est une classe abstraite rassemble les paramètres liés à la prédiction.
     *
     */
    class PredictParams : public BayesianWrapper::PredictParams
    {
        public:
        /** Constructeur par défault.
         *  Il initialise des paramètres classiques non-stupides.
         */
        PredictParams();

        /**
         * Permet de formatter les paramètres en un message lisible.
         */
        virtual std::string getInfo() const;

        /**
         * Covariance de l'estimée vitesse de translation odo selon X en m/s.\n
         * Il s'agit de l'écart type.
         */
        double odoVelXSigma;

        /**
         * La précision estimée de la vitesse de translation odo selon Y en m/s.\n
         * Il s'agit de l'écart type.
         */
        double odoVelYSigma;

        /**
         * La précision estimée de la vitesse de rotation odo en rad/s.\n
         * Il s'agit de l'écart type.
         */
        double odoVelHSigma;
    };

    /** \ingroup kfl
     *
     * \class UpdateParams
     *
     * \brief BFLWrapper::UpdateParams est une classe abstraite rassemble les paramètres liés à l'update.
     *
     */
    class UpdateParams : public BayesianWrapper::UpdateParams
    {
        public:
        /** Constructeur par défault.
         *  Il initialise des paramètres classiques non-stupides.
         */
        UpdateParams();

        /**
         * Permet de formatter les paramètres en un message lisible.
         */
        virtual std::string getInfo() const;

        /**
         * La précision estimée de la mesure de distance du laser en m.\n
         * Il s'agit de l'écart type.
         */
        double laserRangeSigma;

        /**
         * La précision estimée de la mesure d'angle du laser en rad.\n
         * Il s'agit de l'écart type.
         */
        double laserThetaSigma;
    };

    /** \ingroup kfl
     *
     * \class FilterParams
     *
     * \brief BFLWrapper::FilterParams rassemble les paramètres du filtrage bayesien.
     *
     */
    class FilterParams : public BayesianWrapper::FilterParams
    {
        public:
        /** Constructeur par défault.
         *  Il initialise des paramètres classiques non-stupides.
         */
        FilterParams();

        /**
         * Permet de formatter les paramètres en un message lisible.
         */
        virtual std::string getInfo() const;

        /**
         * Le nombre d'itérations maximal de la routine itérative du kalman étendu itératif.\n
         * Au delà de ce nombre d'itérations, même si l'estimée n'a pas convergé, l'algo rend la dernière estimée.
         */
        unsigned int iekfMaxIt;

        /**
         * L'innovation minimale désirée pour le IEKF.\n
         * Il s'agit de la norme au carré de l'innovation minimale.
         */
        double iekfInnovationMin;
    };

    /**
     * Constructeur par défault
     */
    BFLWrapper();

    /**
     * Initialise le filtrage bayésien.
     * \param v l'état initial
     * \param c la covariance de l'état initial
     * \param p les paramètres à utiliser
     * \warning p doit être de type BFLWrapper::InitParams
     */
    void init(const KFLStateVar & v, const KFLStateCov & c, BayesianWrapper::FilterParams & p);

    /**
     * Réalise la prédiction : simulation du système
     * \param i variable d'entrée correspondant à une commande
     * \param dt le pas de temps à simuler
     * \param p paramètres de l'update (contient les écarts types de l'entrée)
     * \warning p doit être de type BFLWrapper::PredictParams
     */
    void predict( const KFLSysInput & t, double dt, BayesianWrapper::PredictParams & p);

    /**
     * Réalise la confrontation aux mesures
     * \param m la mesure
     * \param t la cible
     * \param p paramètres de l'update (contient les écarts types de mesure)
     * \warning p doit être de type BFLWrapper::UpdateParams
     */
    void update(const KFLMeasVar & m, const KFLMeasTarget & t, BayesianWrapper::UpdateParams & p);

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
    BFLSysConditionalPdf * sysPdf;
    BFL::LinearAnalyticSystemModelGaussianUncertainty * sysModel;

    BFLMeasConditionalPdf * measPdf;
    BFL::AnalyticMeasurementModelGaussianUncertainty * measModel;

    BFL::InnovationCheck * innovationCheck;
    BFL::IteratedExtendedKalmanFilter * filter;


    protected:
    void clear();

};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_BFLWRAPPER_HPP_ */
