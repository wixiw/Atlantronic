/*
 * CircleIdentif.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_CIRCLEIDENTIF_HPP_
#define _ARP_RLU_LSL_CIRCLEIDENTIF_HPP_

#include <math/core>

#include <vector>

#include "LSL/filters/ParamsInterface.hpp"
#include <LSL/LaserScan.hpp>
#include <LSL/objects/DetectedObject.hpp>
#include <LSL/objects/DetectedCircle.hpp>

namespace arp_rlu
{

namespace lsl
{

/*!
 *  \addtogroup lsl
 *  @{
 */

/** \class CircleIdentif
 *
 * \brief CircleIdentif est un filtre qui permet de déterminer les paramètres d'un cercle approchant un DetectedObject.
 *
 */
class CircleIdentif
{
    public:
        /** \ingroup lsl
         * \class Params
         *
         * \brief CircleIdentif::Params rassemble les paramètres du filtre CircleIdentif.
         *
         */
        class Params : ParamsInterface
        {
            public:
            /** Constructeur par défault.
             *  Il initialise des paramètres classiques non-stupides :\n
             */
            Params();

            /**
             * Permet de formatter les paramètres en un message lisible.
             */
            std::string getInfo();

            /**
             * Permet de vérifier que les paramètres sont consistants.\n
             * A savoir : \n
             * * radius > 0. \n
             * * rangeDelta < radius
             */
            bool checkConsistency();

            /**
             * Rayon du cercle que l'on cherche.\n
             * Valeur par défault : 0.04m
             */
            double radius;

            /**
             * Offset statistique entre le centre de gravité du scan et le centre du cercle.\n
             * Valeur par défault : 85 pourcents du rayon (donc pour un rayon de 4cm : 0.034m)
             */
            double rangeDelta;
        };

    public:
        /** Applique le filtre sur un DetectedObject
         * \param d un object détecté
         * \param p paramètres du filtre
         * \return Le cercle détecté
         */
        static DetectedCircle apply(const DetectedObject & d, const Params & p = Params());

        /** Applique le filtre sur un vecteur de DetectedObject
         * \param d un object détecté
         * \param p paramètres du filtre
         * \return vecteur de cercles détectés
         * \remarks Cette méthode se contente d'appeler plusieurs fois la méthode apply(const DetectedObject & d, const Params & p = Params())
         */
        static std::vector<DetectedCircle> apply(const std::vector<DetectedObject> &, const Params & p = Params());

    protected:

};

/*! @} End of Doxygen Groups*/

} // namespace lsl

} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_CIRCLEIDENTIF_HPP_ */
