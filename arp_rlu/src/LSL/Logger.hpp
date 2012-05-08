/*
 * Logger.hpp
 *
 *  Created on: 5 mars 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_LOGGER_HPP_
#define _ARP_RLU_LSL_LOGGER_HPP_

#include <logger/DefaultLogger.hpp>

namespace arp_rlu
{

namespace lsl
{

/*!
 *  \addtogroup lsl
 *  @{
 */

/**
 *  \function Log
 *  Permet de logger avec le niveau de priorité spécifié.\n
 *  Exemple : Log( INFO ) << "exemple de message d'information";
 */
arp_core::log::CategoryStream Log(arp_core::log::Priority::Value priority);

/** \class Logger
 *
 * \brief Logger est la classe qui gère les log de LSL. C'est un singleton.
 *
 * Il s'initialise simplement en faisant par exemple :\n
 * arp_rlu::lsl::Logger::InitConsole("CategoryName", DEBUG);\n
 * ou :\n
 * arp_rlu::lsl::Logger::InitFile("FileName", DEBUG);
 *
 */
class Logger: public arp_core::log::DefaultLogger
{
    private:
        static arp_core::log::Logger* m_pInstance;
        static arp_core::log::Category* m_pCat;
};

/*! @} End of Doxygen Groups*/

}
}

#endif //_ARP_RLU_LSL_LOGGER_HPP_
