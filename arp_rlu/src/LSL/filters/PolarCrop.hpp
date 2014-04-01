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

#include <time/StatTimer.hpp>

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
             *  \li minRange = 0.1
             *  \li maxRange = 10.0
             *  \li minTheta = -pi
             *  \li maxTheta = pi
             */
            Params();

            /**
             * Permet de formatter les paramètres en un message lisible.
             */
            std::string getInfo() const;

            /**
             * Permet de vérifier que les paramètres sont consistants.\n
             * A savoir :
             * \li minRange < maxRange
             * \li minTheta < maxTheta
             */
            bool checkConsistency() const;

            /**
             * Range minimal.
             */
            double minRange;

            /**
             * Range maximal.
             */
            double maxRange;

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
         * \param[in] ls scan d'origine
         * \param[in] p paramètres du filtre
         * \return LaserScan filtré
         * \remarks Si les paramètres sont inconsistants, le filtre renvoie le scan initial.
         */
        static LaserScan apply(const LaserScan & ls, const Params & p = Params());


};

/*! @} End of Doxygen Groups*/

} // namespace lsl

} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_POLARCROP_HPP_ */
