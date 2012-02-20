/*
 * PolarCrop.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_POLARCROP_HPP_
#define _ARP_RLU_LSL_POLARCROP_HPP_

#include <math/core>

#include "LSL/filters/ParamsInterface.hpp"
#include "LSL/LaserScan.hpp"

namespace arp_rlu
{

namespace lsl
{

/*!
 *  \addtogroup lsl
 *  @{
 */

/** \class PolarCrop
 *
 * \brief PolarCrop est un filtre qui supprime les points qui dépassent de bornes
 * définies en coordonnées polaires.
 *
 */
class PolarCrop
{
    public:
        /** \ingroup lsl
         * \class Params
         *
         * \brief PolarCrop::Params rassemble les paramètres du filtre PolarCrop.
         *
         */
        class Params : ParamsInterface
        {
            public:
            /** Constructeur par défault.
             *  Il initialise des paramètres classiques non-stupides :\n
             *  minRange = 0.1 * Eigen::Ones(1)\n
             *  maxRange = 10.0 * Eigen::Ones(1)\n
             *  minTheta = -pi\n
             *  maxTheta = pi\n
             */
            Params();

            /**
             * Permet de formatter les paramètres en un message lisible.
             */
            std::string getInfo();

            /**
             * Permet de vérifier que les paramètres sont consistants.\n
             * A savoir :\n
             * * minRange et maxRange contiennent des valeurs non-négatives \n
             * * minRange < maxRange pour chaque élément \n
             * * minTheta < maxTheta
             */
            bool checkConsistency();

            /**
             * Range minimal.\n
             * La taille du vecteur peut être de 1 ou de N, avec N la taille du scan à traiter.\n
             * Si le vecteur est de taille 1, le même range minimal est utilisé pour tous les points.\n
             * Si le vecteur est de taille N, chaque élément du vecteur définit le range minimal pour chaque point du scan.
             */
            Eigen::VectorXd minRange;

            /**
             * Range maximal.\n
             * La taille du vecteur peut être de 1 ou de N, avec N la taille du scan à traiter.\n
             * Si le vecteur est de taille 1, le même range maximal est utilisé pour tous les points.\n
             * Si le vecteur est de taille N, chaque élément du vecteur définit le range maximal pour chaque point du scan.
             */
            Eigen::VectorXd maxRange;

            /**
             * Angle minimal.
             */
            double minTheta;

            /**
             * Angle maximal.
             */
            double maxTheta;
        };

    public:
        /** Applique le filtre sur un scan
         * \param ls scan d'origine
         * \param p paramètres du filtre
         * \return LaserScan filtré
         */
        static LaserScan apply(const LaserScan &, const Params & p = Params());


    protected:

};

/*! @} End of Doxygen Groups*/

} // namespace lsl

} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_POLARCROP_HPP_ */
