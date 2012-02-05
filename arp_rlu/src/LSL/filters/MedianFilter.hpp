/*
 * MedianFilter.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_MEDIANFILTER_HPP_
#define _ARP_RLU_LSL_MEDIANFILTER_HPP_

#include <math/core>

#include "LSL/LaserScan.hpp"

namespace arp_rlu
{

namespace lsl
{

/** \ingroup lsl
 * \nonstableyet
 *
 * \class MedianFilter
 *
 * \brief MedianFilter est un filtre médian sur le range (donnée polaire de distance).
 *
 */
class MedianFilter
{
    public:
        /** \ingroup lsl
         * \nonstableyet
         *
         * \class Params
         *
         * \brief MedianFilter::Params rassemble les paramètres du filtre MedianFilter.
         *
         */
        class Params
        {
        public:
            /** Constructeur par défault.
             *  Il initialise des paramètres classiques non-stupides :\n
             *  width = 3.
             */
            Params();

            /**
             * Permet de formatter les paramètres en un message lisible.
             */
            std::string getInfo();

            /**
             * Largeur du filtre (en nombre de points).\n
             * Cette largeur peut être paire ou impaire.
             */
            unsigned int width;
        };

    public:
        /** Applique le filtre sur un scan
         * \param ls scan d'origine
         * \param p paramètres du filtre
         * \return LaserScan filtré
         */
        static LaserScan apply(const LaserScan & ls, const Params & p = Params());

    protected:

};

} // namespace lsl
} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_MEDIANFILTER_HPP_ */
