/*
 * PolarSegment.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_POLARSEGMENT_HPP_
#define _ARP_RLU_LSL_POLARSEGMENT_HPP_

#include <math/core>

#include "LSL/filters/ParamsInterface.hpp"
#include "LSL/LaserScan.hpp"
#include "LSL/objects/DetectedObject.hpp"
#include <vector>

namespace arp_rlu
{

namespace lsl
{

/*!
 *  \addtogroup lsl
 *  @{
 */

/** \class PolarSegment
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
         *
         * \class Params
         *
         * \brief PolarSegment::Params rassemble les paramètres du filtre PolarSegment.
         *
         */
        class Params : ParamsInterface
        {
        public:
            /** Constructeur par défault.
             *  Il initialise des paramètres classiques non-stupides :\n
             *  \li rangeThres = 0.08
             */
            Params();

            /**
             * Permet de formatter les paramètres en un message lisible.
             */
            std::string getInfo() const;

            /**
             * Permet de vérifier que les paramètres sont consistants.\n
             * A savoir :\n
             * \li rangeThres doit être strictement positif
             */
            bool checkConsistency() const;

            /**
             * Seuil de distance en mètre.
             */
            double rangeThres;
        };

    public:
        /** Applique le filtre sur un scan
         * \param[in] ls scan d'origine
         * \param[in] p paramètres du filtre
         * \return vecteur de DetectedObject
         * \remarks Si les paramètres sont inconsistants, le filtre renvoie un vecteur contenant seulement le scan initial.
         */
        static std::vector<DetectedObject> apply(const LaserScan &, const Params & p = Params());


    protected:

};

/*! @} End of Doxygen Groups*/

} // namespace lsl

} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_POLARSEGMENT_HPP_ */
