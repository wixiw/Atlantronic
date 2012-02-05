/*
 * PolarSegment.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_POLARSEGMENT_HPP_
#define _ARP_RLU_LSL_POLARSEGMENT_HPP_

#include <math/core>

#include "LSL/LaserScan.hpp"
#include <vector>

namespace arp_rlu
{

namespace lsl
{

/** \ingroup lsl
 * \nonstableyet
 *
 * \class PolarSegment
 *
 * \brief PolarSegment est un filtre qui permet de segmenter un scan en plusieurs éléments consistants.
 *
 * Il parcourt les points dans l'ordre (theta croissant) et détecte les écarts de profondeur.\n
 * Il segmente ainsi les groupes de points qui sont séparés par un changement brusque de profondeur.
 *
 */
class PolarSegment
{
    public:
        /** \ingroup lsl
         * \nonstableyet
         *
         * \class Params
         *
         * \brief PolarSegment::Params rassemble les paramètres du filtre PolarSegment.
         *
         */
        class Params
        {
        public:
            /** Constructeur par défault.
             *  Il initialise des paramètres classiques non-stupides :\n
             *  rangeThres = 0.08
             */
            Params();

            /**
             * Permet de formatter les paramètres en un message lisible.
             */
            std::string getInfo();

            /**
             * Seuil de distance en mètre.
             */
            double rangeThres;
        };

    public:
        /** Applique le filtre sur un scan
         * \param ls scan d'origine
         * \param p paramètres du filtre
         * \return vecteur de LaserScan
         */
        static std::vector<LaserScan> apply(const LaserScan &, const Params & p = Params());


    protected:

};

} // namespace lsl
} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_POLARSEGMENT_HPP_ */
