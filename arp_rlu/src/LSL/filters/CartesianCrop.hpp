/*
 * CartesianCrop.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_CARTESIANCROP_HPP_
#define _ARP_RLU_LSL_CARTESIANCROP_HPP_

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

/** \class CartesianCrop
 *
 * \brief CartesianCrop est un filtre qui supprime les points qui dépassent de bornes
 * définies en cartésien.
 *
 */
class CartesianCrop
{
    public:
        /** \ingroup lsl
         * \class Params
         *
         * \brief CartesianCrop::Params rassemble les paramètres du filtre CartesianCrop.
         *
         */
        class Params : ParamsInterface
        {
        public:
            /** Constructeur par défault.
             *  Il initialise des paramètres classiques non-stupides :\n
             *  minX = -1.5 \n
             *  maxX =  1.5 \n
             *  minY = -1.0 \n
             *  maxY = -1.0 \n
             */
            Params();

            /**
             * Permet de formatter les paramètres en un message lisible.
             */
            std::string getInfo();

            /**
             * Permet de vérifier que les paramètres sont consistants.\n
             * A savoir :\n
             * * minX < maxX \n
             * * minY < maxY
             */
            bool checkConsistency() const;

            double minX;
            double maxX;
            double minY;
            double maxY;
        };

    public:
        /** Applique le filtre sur un scan
         * \param ls scan d'origine
         * \param p paramètres du filtre
         * \return LaserScan filtré
         * \remarks Si les paramètres sont inconsistants, le filtre renvoie le scan initial.
         * \remarks si les données cartésiennes ne sont pas disponible, le filtre renvoie le scan initial.
         */
        static LaserScan apply(const LaserScan &, const Params & p = Params());

    protected:

};

/*! @} End of Doxygen Groups*/

} // namespace lsl

} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_CARTESIANCROP_HPP_ */
