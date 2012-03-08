/*
 * ParamsInterface.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_PARAMSINTERFACE_HPP_
#define _ARP_RLU_LSL_PARAMSINTERFACE_HPP_

#include <math/core>

namespace arp_rlu
{

namespace lsl
{
/*!
 *  \addtogroup lsl
 *  @{
 */

/** \interface ParamsInterface
 *
 * \brief ParamsInterface est l'interface pour les paramètres des filtres de LSL.
 *
 */
class ParamsInterface
{
    public:

        /**
         * Permet de formatter les paramètres en un message lisible.
         */
        virtual std::string getInfo() const = 0;

        /**
         * Permet de vérifier que les paramètres sont consistants.
         */
        virtual bool checkConsistency() const = 0;

};

/*! @} End of Doxygen Groups*/

} // namespace lsl

} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_PARAMSINTERFACE_HPP_ */
