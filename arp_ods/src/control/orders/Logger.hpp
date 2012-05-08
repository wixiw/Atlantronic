/*
 * Logger.hpp
 *
 *  Created on: 29 February 2012
 *      Author: Boris
 */

#ifndef _ARP_ODS_ORDER_LOGGER_HPP_
#define _ARP_ODS_ORDER_LOGGER_HPP_

#include <logger/DefaultLogger.hpp>

namespace arp_ods
{

namespace orders
{

/*!
 *  \addtogroup kfl
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
 * \brief Logger est la classe qui gère les log des Orders. C'est un singleton.
 *
 * Il s'initialise simplement en faisant par exemple :\n
 * arp_rlu::kfl::Logger::InitConsole("CategoryName", DEBUG);\n
 * ou :\n
 * arp_rlu::kfl::Logger::InitFile("FileName", DEBUG);
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

#endif //_ARP_RLU_KFL_LOGGER_HPP_
