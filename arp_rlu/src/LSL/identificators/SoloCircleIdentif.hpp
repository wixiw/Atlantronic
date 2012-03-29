/*
 * SoloCircleIdentif.hpp
 *
 *  Created on: 23 Mars 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_SOLOCIRCLEIDENTIF_HPP_
#define _ARP_RLU_LSL_SOLOCIRCLEIDENTIF_HPP_

#include <vector>
#include <math/core>

#include "LSL/filters/ParamsInterface.hpp"
#include "LSL/objects/Circle.hpp"
#include "LSL/objects/DetectedCircle.hpp"


namespace arp_rlu
{

namespace lsl
{

/*!
 *  \addtogroup lsl
 *  @{
 */

/** \class SoloCircleIdentif
 *
 * \brief SoloCircleIdentif permet d'identifier des DetectedCircle parmi des Circle référencés.
 *
 * Un DetectedCircle est apparié à un Circle de référence si leurs rayons semblable (cf tolérance en paramètre) et
 * qu'ils sont proches l'un de l'autre (cf tolérance en paramètre)
 * Dans le cas où plusieurs DetectedCircle sont elligibles pour être apparié au même Circle de référence,
 * seul le plus proche est gardé.
 */
class SoloCircleIdentif
{
    public:
        /** \ingroup lsl
         * \class Params
         *
         * \brief SoloCircleIdentif::Params rassemble les paramètres du filtre SoloCircleIdentif.
         *
         */
        class Params : ParamsInterface
        {
            public:
            /** Constructeur par défault.
             *  Il initialise des paramètres classiques non-stupides :\n
             *  \li radiusTolerance = 0.05
             *  \li distanceTolerance = 0.1
             */
            Params();

            /**
             * Permet de formatter les paramètres en un message lisible.
             */
            std::string getInfo() const;

            /**
             * Permet de vérifier que les paramètres sont consistants.\n
             * \li radiusTolerance >= 0.
             * \li distanceTolerance >= 0.
             */
            bool checkConsistency() const;

            /**
             * Tolerance sur le rayon du DetectedCircle par rapport au Circle de référence pour qu'il soit élligible.
             */
            double radiusTolerance;

            /**
             * Tolerance sur la distance du DetectedCircle par rapport au Circle de référence pour qu'il soit élligible.
             */
            double distanceTolerance;
        };

    public:
        /** Applique le filtre sur un vecteur de DetectedCircle
         * \param[in] vdc vecteur des DetectedCircle à considérer
         * \param[in] vrc vecteur des Circle référence
         * \param[in] p paramètres du filtre
         * \return le vecteur des appariements trouvés
         */
        static std::vector< std::pair<DetectedCircle, Circle > > apply(const std::vector<DetectedCircle> & vdc, const std::vector<Circle> & vrc, const Params & p = Params());

};

/*! @} End of Doxygen Groups*/

} // namespace lsl

} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_SOLOCIRCLEIDENTIF_HPP_ */
